# pilot.py
from __future__ import annotations
import threading
import time
import traceback
import math
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

from drive_client import DriveClient
from gnss_client import GnssClient
from data.nav_fusion_data import NavFusionData

from geo_utils import heading_gnss_to_enu, lla_to_ecef, ecef_to_enu
from near_waypoint import select_near_point
from motion_controller import MotionController2D, ControllerConfig, SpeedMode
from pilot_log import PilotLog  # <-- NOVÉ

def _wrap_deg(angle: float) -> float:
    a = (angle + 180.0) % 360.0 - 180.0
    return a if a != -180.0 else 180.0

@dataclass
class PilotState:
    mode: str = "IDLE"                 # IDLE | NAVIGATE | GOAL_REACHED | GOAL_NOT_REACHED
    near_case: str = "N/A"             # TWO_INTERSECTIONS | TANGENT | NO_INTERSECTION | N/A
    dist_to_goal_m: float = 0.0
    cross_track_m: float = 0.0         # kolmá vzdálenost k přímce S–E (telemetrie)
    left_pwm: int = 0
    right_pwm: int = 0
    heading_enu_deg: float = 0.0
    last_note: str = ""
    ts_mono: float = 0.0               # monotonic timestamp poslední aktualizace

class Pilot:

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.gnss_client: Optional[GnssClient] = None
        self.drive_client: Optional[DriveClient] = None

        # cíle v pořadí, jak přicházejí z klienta: (lon, lat)
        self.target: Optional[Tuple[Tuple[float, float], Tuple[float, float], float]] = None  # (start, goal, radius)

        # stav publikovaný přes STATE
        self._state_lock = threading.Lock()
        self._state = PilotState()

        # logger (živý jen v době běhu navigace)
        self.log: Optional[PilotLog] = None

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
                return "NOT_RUNNING"
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
                self._set_state(mode="IDLE", near_case="N/A", left_pwm=0, right_pwm=0, last_note="SERVICE STOPPED")
                print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[PILOT SERVICE] Service is not running. Call START first.")

    # ---------------------- navigační vlákno -----------------

    def _navigate_thread(self, gnss: GnssClient, drive: DriveClient, start_lat, start_lon, goal_lat, goal_lon, goal_radius,):
        """
        Baseline navigační smyčka:
        - čte GNSS NavFusionData
        - vybere near point na PŘÍMCE S–E s L_near = 1.0 m
        - spočítá PWM z (heading_enu, near v ENU(R)) s limity (v_max, ω_max)
        - loguje průběh (GNSS_IN, COMPUTE, ACT_CMD, EVENT) do jednoho CSV
        """
        # POZOR: start, goal dorazily z klienta jako (lon, lat)!
        # Pro výpočty potřebujeme (lat, lon):
        S_lat, S_lon = float(start_lat), float(start_lon)
        E_lat, E_lon = float(goal_lat), float(goal_lon)

        print(f"[PILOT] Navigation started: from (lat={S_lat}, lon={S_lon}) to (lat={E_lat}, lon={E_lon}) "
              f"within radius {goal_radius}m")

        # --- Konfigurace controlleru ---
        ctrl = MotionController2D(
            ControllerConfig(
                v_max_debug_mps=0.5,   # ladění
                v_max_normal_mps=1.5,  # ostrý
                omega_max_dps=90.0,
                max_pwm=40,
                deadband_pwm=15,
                slow_down_dist_m=5.0,
                k_heading_to_omega=2.0,
                v_scale=0.6,
            ),
            mode=SpeedMode.DEBUG  # přepni na NORMAL po odladění
        )
        L_NEAR = 1.0  # [m] délka "provázku"
        GOAL_RADIUS = float(goal_radius)

        # --- Log init (RUN_META) ---
        self.log = PilotLog(
            start_lat=S_lat, start_lon=S_lon,
            goal_lat=E_lat, goal_lon=E_lon, goal_radius=GOAL_RADIUS,
            ctrl=ctrl,
            lookahead_m=L_NEAR,
        )

        self._set_state(mode="NAVIGATE", near_case="N/A", last_note="Navigation loop started")
        self.log.event("NAVIGATE", "Navigation loop started")

        last_loop = time.monotonic()

        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                nav: Optional[NavFusionData] = gnss.read_nav_fusion_data()
                loop_dt_ms = (loop_start - last_loop) * 1000.0
                last_loop = loop_start

                if not nav:
                    drive.send_pwm(0, 0)
                    self._set_state(left_pwm=0, right_pwm=0, last_note="No nav data")
                    self.log.event("NAVIGATE", "No nav data")
                    time.sleep(0.2)
                    continue

                # --- Near dle specifikace (line S–E ∩ circle R(L_NEAR)) ---
                near = select_near_point(
                    S_lat=S_lat, S_lon=S_lon,
                    E_lat=E_lat, E_lon=E_lon,
                    R_lat=nav.lat, R_lon=nav.lon,
                    L_near_m=L_NEAR,
                )

                # --- Heading (GNSS -> ENU) ---
                heading_gnss = getattr(nav, "heading", None)
                if heading_gnss is None:
                    heading_gnss = getattr(nav, "motHeading", None)
                if heading_gnss is None:
                    heading_gnss = getattr(nav, "vehHeading", None)
                heading_enu = heading_gnss_to_enu(float(heading_gnss or 0.0))

                # --- GNSS_IN log ---
                self.log.nav(
                    state="NAVIGATE",
                    loop_dt_ms=loop_dt_ms,
                    nav=nav,
                    theta_enu=heading_enu
                )

                # --- Vzdálenost k cíli a diagnostika (COMPUTE) ---
                diag = self.log.near_and_errors(
                    state="NAVIGATE",
                    goal_lat=E_lat, goal_lon=E_lon, goal_radius_m=GOAL_RADIUS,
                    R_lat=nav.lat, R_lon=nav.lon,
                    theta_enu=heading_enu,
                    near=near
                )
                dist_to_goal_m = diag["dist_to_goal_m"]

                if near.case == "NO_INTERSECTION":
                    # Eskalace nahoru: zastav a publikuj stav
                    drive.send_pwm(0, 0)
                    self._set_state(
                        mode="GOAL_NOT_REACHED",
                        near_case=near.case,
                        dist_to_goal_m=dist_to_goal_m,
                        cross_track_m=getattr(near, "d_perp_m", 0.0),
                        left_pwm=0, right_pwm=0,
                        heading_enu_deg=heading_enu,
                        last_note="Near selection failed (NO_INTERSECTION)"
                    )
                    self.log.event("GOAL_NOT_REACHED", "Near selection failed (NO_INTERSECTION)")
                    print("[PILOT] Near selection failed (NO_INTERSECTION) -> GOAL_NOT_REACHED")
                    #time.sleep(0.2)
                    #continue
                    break  # ukonči navigaci

                # --- Povolení dopředně/spin (zatím bez FSM: obojí povoleno) ---
                allow_forward = True
                allow_spin = True

                # --- PŘEDPOČET řídicích veličin pro log (stejně jako v controlleru) ---
                # Desired ENU úhel k near
                desired_deg = math.degrees(math.atan2(near.near_y_m or 0.0, near.near_x_m or 0.0))
                err_deg = _wrap_deg(desired_deg - heading_enu)
                # ω
                k_heading = getattr(ctrl.cfg, "k_heading_to_omega", 2.0)
                omega_cmd = k_heading * err_deg
                omega_limit = getattr(ctrl.cfg, "omega_max_dps", 90.0)
                if not allow_spin:
                    omega_cmd = 0.0
                else:
                    omega_cmd = max(-omega_limit, min(omega_limit, omega_cmd))
                # v (plynulé zpomalení)
                slow_down_dist = getattr(ctrl.cfg, "slow_down_dist_m", 5.0)
                v_scale = getattr(ctrl.cfg, "v_scale", 0.6)
                v_limit = ctrl.cfg.v_max_debug_mps if ctrl.mode == SpeedMode.DEBUG else ctrl.cfg.v_max_normal_mps
                if not allow_forward:
                    v_cmd = 0.0
                else:
                    if dist_to_goal_m <= GOAL_RADIUS:
                        v_cmd = 0.0
                    else:
                        if dist_to_goal_m < slow_down_dist:
                            v_cmd = v_scale * (dist_to_goal_m / slow_down_dist) * v_limit
                        else:
                            v_cmd = v_scale * v_limit

                # --- Výpočet PWM ---
                left_pwm, right_pwm, status = ctrl.compute_for_near(
                    heading_enu_deg=heading_enu,
                    near_x_m=near.near_x_m or 0.0,
                    near_y_m=near.near_y_m or 0.0,
                    allow_forward=allow_forward,
                    allow_spin=allow_spin,
                    dist_to_goal_m=dist_to_goal_m,
                    goal_radius_m=GOAL_RADIUS,
                )

                # SATURACE (pouze odhad dle limitů)
                sat_v = int(abs(v_cmd) >= v_limit - 1e-6)
                sat_omega = int(abs(omega_cmd) >= omega_limit - 1e-6)

                # --- Log aktuátorů ---
                self.log.act_cmd(
                    state="NAVIGATE",
                    lookahead_m=L_NEAR,
                    k_heading=k_heading,
                    k_cte=None,
                    v_cmd_mps=v_cmd,
                    omega_cmd_dps=omega_cmd,
                    v_limit_mps=v_limit,
                    omega_limit_dps=omega_limit,
                    left_pwm=left_pwm, right_pwm=right_pwm,
                    sat_v=sat_v, sat_omega=sat_omega,
                    note=status
                )

                # Publikuj stav
                self._set_state(
                    mode="NAVIGATE",
                    near_case=near.case,
                    dist_to_goal_m=dist_to_goal_m,
                    cross_track_m=getattr(near, "d_perp_m", 0.0),
                    left_pwm=left_pwm, right_pwm=right_pwm,
                    heading_enu_deg=heading_enu,
                    last_note=status
                )

                # Debug log
                print(f"[PILOT] near={near.case} cte={getattr(near,'d_perp_m',0.0):.3f}m "
                      f"goal_remain={dist_to_goal_m:.2f}m {status}")

                # Odeslání na kola
                drive.send_pwm(left_pwm, right_pwm)

                # --- Dosažen cíl? ---
                if dist_to_goal_m <= GOAL_RADIUS:
                    print("[PILOT] Goal reached -> stop")
                    drive.send_pwm(0, 0)
                    self._set_state(mode="GOAL_REACHED", last_note="Goal reached")
                    self.log.event("GOAL_REACHED", "Goal reached")
                    break

            except Exception as e:
                drive.send_pwm(0, 0)
                self._set_state(mode="NAVIGATE", left_pwm=0, right_pwm=0, last_note=f"ERROR: {e}")
                print(f"[PILOT ERROR] {e}")
                traceback.print_exc()
                try:
                    if self.log:
                        self.log.event("NAVIGATE", f"ERROR: {e}")
                except Exception:
                    pass
                time.sleep(1.0)

        # konec smyčky
        self._set_state(left_pwm=0, right_pwm=0)
        try:
            if self.log:
                self.log.event("STOPPED", "Navigation ended")
                self.log.close()
        finally:
            self.log = None

    # ---------------------- API pro řízení -------------------

    def navigate(self, start_lat, start_lon, goal_lat, goal_lon, goal_radius):
        """
        Spustí/obnoví navigační vlákno.
        Parametry z klienta přicházejí jako:
          start = (lat, lon), goal = (lat, lon), radius [m]
        """
        gnss = self.gnss_client
        drive = self.drive_client
        with self._lock:
            self._ensure_running()
            # restart případného běžícího navigačního vlákna
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
