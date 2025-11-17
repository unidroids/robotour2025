# pilot.py
from __future__ import annotations
import threading
import time
import math 
import traceback
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

from drive_client import DriveClient
from fusion_client import FusionClient
from data.nav_fusion_data import NavFusionData

from geo_utils import heading_gnss_to_enu, lla_to_ecef, ecef_to_enu
from near_waypoint import NearWaypoint
from pp_velocity import PPVelocityPlanner

from data_loger import DataLogger

# ---------------------- util -----------------------------

@staticmethod
def _wrap_angle_deg(a: float) -> float:
    """wrap to [-180,180]"""
    a = (a + 180.0) % 360.0 - 180.0
    return a

@staticmethod
def _sign(x: float) -> int:
    return (x > 0) - (x < 0)  # -1, 0, +1      

@dataclass
class PilotState:
    mode: str = "IDLE"                 # IDLE | NAVIGATE | GOAL_REACHED | GOAL_NOT_REACHED
    near_case: str = "N/A"             # TWO_INTERSECTIONS | TANGENT | NO_INTERSECTION | N/A
    #dist_to_goal_m: float = 0.0
    #cross_track_m: float = 0.0         # kolmá vzdálenost k přímce S–E (telemetrie)
    #left_pwm: int = 0
    #right_pwm: int = 0
    #heading_enu_deg: float = 0.0
    last_note: str = ""
    ts_mono: float = 0.0               # monotonic timestamp poslední aktualizace

class Pilot:

    VERSION = "1.2.0"

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.fusion_client: Optional[FusionClient] = None
        self.drive_client: Optional[DriveClient] = None

        self._state_lock = threading.Lock()
        self._state = PilotState()

    # ---------------------- stavové API ----------------------

    def _set_state(self, **updates) -> None:
        with self._state_lock:
            for k, v in updates.items():
                setattr(self._state, k, v)
            self._state.ts_mono = time.monotonic()

    def get_state(self) -> dict:
        with self._state_lock:
            return asdict(self._state)

    # ---------------------- lifecycle ------------------------

    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            if not self._initialized:
                self.drive_client = DriveClient()
                self.fusion_client = FusionClient()
                self._initialized = True

            self.drive_client.connect()
            self.fusion_client.connect()

            self.running = True
            self._set_state(mode="IDLE", near_case="N/A", last_note="SERVICE STARTED")
            print("[SERVICE] STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "OK WAS NOT RUNNING"
            self._stop_event.set()
            if self._thread:
                self._thread.join(timeout=2.0)
            try:
                if self.fusion_client:
                    self.fusion_client.disconnect()
                if self.drive_client:
                    self.drive_client.send_motors_off()
                    self.drive_client.disconnect()
            finally:
                self.drive_client = None
                self.fusion_client = None
                self._initialized = False
                self.running = False
                self._set_state(mode="IDLE", near_case="N/A", last_note="SERVICE STOPPED")
                print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[PILOT SERVICE] Service is not running. Call START first.")

  

    # ---------------------- navigační vlákno -----------------

    def _navigate_thread(self, fusion: FusionClient, drive: DriveClient, start_lat, start_lon, goal_lat, goal_lon, goal_radius,):
        """
        Navigační smyčka:
        - čte GNSS NavFusionData (10 Hz)
        - vybere near point na přímce S–E s L_near
        - vyhodnotí cíl
        - vypočte korekci rychlostí kol
        - odešle rychlosti kol do služby drive
        """
        S_lat, S_lon = float(start_lat), float(start_lon)
        E_lat, E_lon = float(goal_lat), float(goal_lon)
        GOAL_RADIUS = float(goal_radius)

        print(f"[PILOT] Navigation started: from (lat={S_lat}, lon={S_lon}) "
              f"to (lat={E_lat}, lon={E_lon}) within radius {GOAL_RADIUS}m")

        L_NEAR = 20  # lookahead pro near point (m)
        B = 0.58     # rozchod kol (m)
        nearwaypoint = NearWaypoint(S_lat=S_lat, S_lon=S_lon, E_lat=E_lat, E_lon=E_lon, L_near_m=L_NEAR)
        pp_velocity = PPVelocityPlanner(
            a_y_max=0.5,        # m/s^2
            L=L_NEAR,              # m
            b=B,              # m
            max_speed_cm_s=50.0,# cm/s
            min_wheel_speed_cm_s=20.0, # cm/s
            min_turn_radius_m=0.29,  # m
        )

        max_erros = 5
        error_count = 0

        self._set_state(mode="NAVIGATE", near_case="N/A", last_note="Navigation started")
        drive.send_motors_on()
        drive.send_break()
        left_speed, right_speed = 0.0, 0.0
        
        #heading_one_wheeel_comp_deg = math.atan2(0.3, (B/2)) * (180.0 / math.pi)  # small angle approx
        #heading_comp_deg = 0.0
        #smooth_heading_comp_deg = 0.0

        distance_to_goal_m, abs_distance_to_goal_m, heading_to_near_gnss_deg = 0.0, 0.0, 0.0
        heading_error = 0.0
        kappa = 0.0
        drive_mode = "N/A"

        log = DataLogger()

        # print cvs header 
        log.print(
              "ts_mono," # timestamp
              "lat,lon,hAcc," # position
              "raw_heading,smoot_heading,heading_acc,cumulated_angleZ," # heading
              "raw_speed,smooth_speed,speed_acc," # speed
              "last_gyroZ,smooth_gyroZ,gyroZ_acc," # gyroZ
              "gnssFixOK,drUsed," # fix types
              "distance_to_goal_m,abs_distance_to_goal_m,heading_to_near_gnss_deg," # near point
              "heading_error_deg," # heading error
              "left_speed,right_speed,kappa,drive_mode," # drive commands
              "heading_comp_deg,smooth_heading_comp_deg" # heading compensation
        )

        last_loop = time.monotonic()
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                loop_dt_ms = (loop_start - last_loop) * 1000.0
                dt_s = max((loop_start - last_loop), 1e-3)
                last_loop = loop_start

                # 1) Načti Nav Fusion Data
                nav: Optional[NavFusionData] = fusion.read_nav_fusion_data() # blokuj max 1s
                if not nav:
                    drive.send_break()
                    print("[PILOT] No nav data -> sending BREAK")
                    continue
                #print(f"[PILOT] Nav data: lat={nav.lat}, lon={nav.lon}, heading={nav.heading}, speed_m={nav.speed}, gnssFixOK={nav.gnssFixOK}, drUsed={nav.drUsed} ")
                #print(f"[PILOT] Nav data: lat={nav.lat:12.8f}, lon={nav.lon:12.8f}, heading={nav.heading:6.2f}, speed_m={nav.speed*100:6.2f}, gnssFixOK={int(nav.gnssFixOK)}, drUsed={int(nav.drUsed)}")
                
                #continue
                #print("recieved",nav.to_json())
                
                # print telemetry csv line
                log.print(
                    # timestamp "ts_mono,"
                    f"{nav.ts_mono:.3f}," 
                    # position  "lat,lon,hAcc" 
                    f"{nav.lat:.8f},{nav.lon:.8f},{nav.hAcc:.2f}," # position
                    # heading "raw_heading,smoot_heading(odo_angle),heading_acc,cumulated_angleZ," 
                    f"{nav.heading:.2f},{nav.heading:.2f},{nav.headingAcc:.2f},{nav.heading:.2f}," # heading
                    # speed "raw_speed,smooth_speed,speed_acc," 
                    f"{nav.speed:.2f},{nav.speed:.2f},{nav.sAcc:.2f}," # speed
                    # gyroZ "last_gyroZ,smooth_gyroZ(odo_gyro),gyroZ_acc," 
                    f"{nav.gyroZ:.2f},{nav.gyroZ:.2f},{nav.gyroZAcc:.2f}," # gyroZ
                    # fix types "gnssFixOK,drUsed," 
                    f"{int(nav.gnssFixOK)},{int(nav.drUsed)}," # fix types
                    # near point "distance_to_goal_m,abs_distance_to_goal_m,heading_to_near_gnss_deg," 
                    f"{distance_to_goal_m:.2f},{abs_distance_to_goal_m:.2f},{heading_to_near_gnss_deg:.2f},"
                    # heading error "heading_error_deg,"
                    f"{heading_error:.2f},"
                    # drive commands "left_speed,right_speed,kappa,drive_mode," 
                    f"{left_speed:.2f},{right_speed:.2f},{kappa:.4f},{drive_mode},"
                    # heading compensation "heading_comp_deg,smooth_heading_comp_deg," 
                    #f"{heading_comp_deg:.2f},{smooth_heading_comp_deg:.2f}"
                )

                # 2) Zjisti near point
                (distance_to_goal_m, abs_distance_to_goal_m, heading_to_near_gnss_deg) = nearwaypoint.update(R_lat=nav.lat, R_lon=nav.lon)

                # 3) Ověř zda jsi v cíli
                if abs_distance_to_goal_m <= GOAL_RADIUS:
                    print(f"[PILOT] Goal reached -> stop. Distance to goal: {distance_to_goal_m:.2f} m")
                    drive.send_break()
                    self._set_state(mode="GOAL_REACHED", last_note="Goal reached")
                    break
                
                # 4) Ověř zda existje near point
                if not heading_to_near_gnss_deg:
                    print(f"[PILOT] No near point found -> sending BREAK. Distance to goal: {abs_distance_to_goal_m:.2f} m")
                    drive.send_break()
                    self._set_state(mode="NAVIGATE", near_case="NO_INTERSECTION", last_note="No near point found")
                    break

                # 5) Spočti chybu heading robota vůči near point
                heading_error = _wrap_angle_deg(heading_to_near_gnss_deg - nav.heading) 

                # 6) Vypočti nové rychlosti kola
                if (abs(heading_error) > 90):
                    # turn in place
                    s = _sign(heading_error)
                    left_speed, right_speed = 30 * s, -30 * s
                    #heading_comp_deg = -s * 90.0
                    drive_mode = "TURN_IN_PLACE"
                elif abs(heading_error) > 20:
                    # slow turn one wheel stopped
                    if heading_error < 0:
                        left_speed, right_speed = 0, 30    # doleva
                        #heading_comp_deg = +heading_one_wheeel_comp_deg
                    else:
                        left_speed, right_speed = 30, 0    # doprava
                        #heading_comp_deg = -heading_one_wheeel_comp_deg
                    drive_mode = "SLOW_TURN_ONE_WHEEL"
                else:
                    # PP velocity planning
                    left_speed, right_speed, kappa = pp_velocity.calculate(alpha_deg=-heading_error)
                    #heading_comp_deg = math.atan(0.3 * kappa) * (180.0 / math.pi)  # small angle approx 
                    drive_mode = "PP_VELOCITY"

                # smooth heading compensation
                #smooth_heading_comp_deg = 0.8 * smooth_heading_comp_deg + 0.2 * heading_comp_deg
                #print(f"[PILOT] Heading error: {heading_error:.2f} deg, heading_comp: {heading_comp_deg:.2f} deg, smooth_comp: {smooth_heading_comp_deg:.2f} deg")

                # 7) Odešli rychlosti kol do drive služby
                pwm = 70 # pevné PWM pro nyní  (left_speed + right_speed)
                result = drive.send_drive(pwm, left_speed, right_speed)
                #result = drive.send_drive(pwm, 40, -40) # testovací pevná rychlost
                #result = drive.send_drive(pwm, -30, 30) # testovací pevná rychlost
                #result = drive.send_pwm(1,1) # testovací duty cycle
                print("[Pilot]", pwm, left_speed, right_speed, result)
                #print(f"[PILOT] Drive command sent: PWM={pwm}, left_speed={left_speed} cm/s, right_speed={right_speed} cm/s")
                # TODO: check result?

            except Exception as e:
                drive.send_break()
                print(f"[PILOT ERROR] {e}")
                traceback.print_exc()
                time.sleep(1.0)
                error_count += 1
                if error_count >= max_erros:
                    print("[PILOT] Maximum error count reached, stopping navigation.")
                    self._set_state(mode="GOAL_NOT_REACHED", last_note="Max errors reached")
                    break

        drive.send_break()
        time.sleep(0.2)
        drive.send_motors_off()
        print("[PILOT] Navigation ended.")
        

    # ---------------------- API pro řízení -------------------

    def navigate(self, start_lat, start_lon, goal_lat, goal_lon, goal_radius):
        fusion = self.fusion_client
        drive = self.drive_client
        with self._lock:
            self._ensure_running()
            if self._thread and self._thread.is_alive():
                self._stop_event.set()
                self._thread.join()
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._navigate_thread,
                args=(fusion, drive, start_lat, start_lon, goal_lat, goal_lon, goal_radius,),
                daemon=True
            )
            self._thread.start()

if __name__ == "__main__":
    print(f"Sign test {_sign(-30)} {_sign(0)} {_sign(30)}") 
    try:
        pilot = Pilot()
        pilot.start()
        pilot.navigate(
            start_lat=50.0615486,
            start_lon=14.5996717,
            goal_lat=50.0615486,
            goal_lon=14.5996717+0.00002,
            goal_radius=2.0
        )
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        pilot.stop()