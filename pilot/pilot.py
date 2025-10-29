# pilot.py
from __future__ import annotations
import threading
import time
import traceback
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

from drive_client import DriveClient
from gnss_client import GnssClient
from data.nav_fusion_data import NavFusionData

from geo_utils import heading_gnss_to_enu, lla_to_ecef, ecef_to_enu
from near_waypoint import NearWaypoint
from pp_velocity import PPVelocityPlanner

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

        self.gnss_client: Optional[GnssClient] = None
        self.drive_client: Optional[DriveClient] = None
        self._log: Optional[PilotLog] = None

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
                self.gnss_client = GnssClient()
                self._initialized = True

            self.drive_client.start()
            self.gnss_client.start()

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
                if self.gnss_client:
                    self.gnss_client.stop()
                if self.drive_client:
                    self.drive_client.stop()
            finally:
                self.drive_client = None
                self.gnss_client = None
                self._initialized = False
                self.running = False
                self._set_state(mode="IDLE", near_case="N/A", last_note="SERVICE STOPPED")
                print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[PILOT SERVICE] Service is not running. Call START first.")

  

    # ---------------------- navigační vlákno -----------------

    def _navigate_thread(self, gnss: GnssClient, drive: DriveClient, start_lat, start_lon, goal_lat, goal_lon, goal_radius,):
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

        L_NEAR = 1.0  # lookahead pro near point (m)

        nearwaypoint = NearWaypoint(S_lat=S_lat, S_lon=S_lon, E_lat=E_lat, E_lon=E_lon, L_near_m=L_NEAR)
        pp_velocity = PPVelocityPlanner(
            a_y_max=0.5,        # m/s^2
            L=L_NEAR,              # m
            b=0.58,              # m
            max_speed_cm_s=50.0,# cm/s
            min_wheel_speed_cm_s=20.0, # cm/s
            min_turn_radius_m=0.29,  # m
        )

        max_erros = 5
        error_count = 0

        self._set_state(mode="NAVIGATE", near_case="N/A", last_note="Navigation started")

        last_loop = time.monotonic()
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                loop_dt_ms = (loop_start - last_loop) * 1000.0
                dt_s = max((loop_start - last_loop), 1e-3)
                last_loop = loop_start

                # 1) Načti data GNSS
                nav: Optional[NavFusionData] = gnss.read_nav_fusion_data() # blokuj max 1s
                if not nav:
                    drive.send_break()
                    print("[PILOT] No nav data -> sending BREAK")
                    continue
                print(f"[PILOT] Nav data: lat={nav.lat}, lon={nav.lon}, heading={nav.heading}, speed_m={nav.speed}")

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
                heading_error = _wrap_angle_deg(heading_to_near_gnss_deg - nav.heading) # ccw positive

                # 6) Vypočti nové rychlosti kola
                left_speed, right_speed = 0.0, 0.0
                if (abs(heading_error) > 90):
                    # turn in place
                    s = _sign(heading_error)
                    left_speed, right_speed = -30 * s, 30 * s
                elif abs(heading_error) > 30:
                    # slow turn one wheel stopped
                    if heading_error > 0:
                        left_speed, right_speed = 0, 30    # doleva
                    else:
                        left_speed, right_speed = 30, 0    # doprava
                else:
                    # PP velocity planning
                    left_speed, right_speed = pp_velocity.calculate(alpha_deg=heading_error)

                # 7) Odešli rychlosti kol do drive služby
                pwm = 100 # pevné PWM pro nyní  (left_speed + right_speed)
                result = drive.send_drive(pwm, left_speed, right_speed)
                print(f"[PILOT] Drive command sent: PWM={pwm}, left_speed={left_speed} cm/s, right_speed={right_speed} cm/s")
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
        print("[PILOT] Navigation ended.")
        

    # ---------------------- API pro řízení -------------------

    def navigate(self, start_lat, start_lon, goal_lat, goal_lon, goal_radius):
        gnss = self.gnss_client
        drive = self.drive_client
        with self._lock:
            self._ensure_running()
            if self._thread and self._thread.is_alive():
                self._stop_event.set()
                self._thread.join()
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._navigate_thread,
                args=(gnss, drive, start_lat, start_lon, goal_lat, goal_lon, goal_radius,),
                daemon=True
            )
            self._thread.start()

if __name__ == "__main__":
    print(f"Sign test {_sign(-30)} {_sign(0)} {_sign(30)}") 
    pilot = Pilot()
    pilot.start()
    pilot.navigate(
        start_lat=50.0616314,
        start_lon=14.599517,
        goal_lat=50.0615758,
        goal_lon=14.5996074+0.0004,
        goal_radius=5.0
    )
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pilot.stop()