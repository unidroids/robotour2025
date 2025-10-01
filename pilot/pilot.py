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

from pilot_log import PilotLog
from pilot_fsm import NavigatorFSM, FsmConfig, NavQuality, NavigatorState

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

    VERSION = "1.1.0"

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.gnss_client: Optional[GnssClient] = None
        self.drive_client: Optional[DriveClient] = None
        self._log: Optional[PilotLog] = None

        # stav publikovaný přes STATE
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
                if self._log:
                    self._log.event("IDLE", "IDLE", "Service stopped")
                    self._log.close()
                    self._log = None
                self._set_state(mode="IDLE", near_case="N/A", left_pwm=0, right_pwm=0, last_note="SERVICE STOPPED")
                print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[PILOT SERVICE] Service is not running. Call START first.")

    # ---------------------- util -----------------------------

    @staticmethod
    def _wrap_angle_deg(a: float) -> float:
        """wrap to [-180,180)"""
        a = (a + 180.0) % 360.0 - 180.0
        return a

    # ---------------------- navigační vlákno -----------------

    def _navigate_thread(self, gnss: GnssClient, drive: DriveClient, start_lat, start_lon, goal_lat, goal_lon, goal_radius,):
        """
        Navigační smyčka:
        - čte GNSS NavFusionData (30 Hz)
        - vybere near point na přímce S–E s L_near
        - rozhoduje PWM přes MotionController
        - loguje do CSV (jeden soubor na běh)
        - používá NavigatorFSM pro řízení režimů (WAIT_GNSS/ACQUIRE_HEADING/...)
        """
        # Parametry (pozor: klient posílá (lat,lon) – zde to respektujeme)
        S_lat, S_lon = float(start_lat), float(start_lon)
        E_lat, E_lon = float(goal_lat), float(goal_lon)
        GOAL_RADIUS = float(goal_radius)

        print(f"[PILOT] Navigation started: from (lat={S_lat}, lon={S_lon}) "
              f"to (lat={E_lat}, lon={E_lon}) within radius {GOAL_RADIUS}m")

        # --- Controller config (mírně zklidněné) ---
        ctrl = MotionController2D(
            ControllerConfig(
                v_max_debug_mps=0.5,
                v_max_normal_mps=1.5,
                omega_max_dps=90.0,
                max_pwm=40,
                deadband_pwm=15,
                slow_down_dist_m=5.0,
                k_heading_to_omega=0.9,   # z 2.0 -> 0.9 (mírnější yaw)
                v_scale=0.6,
            ),
            mode=SpeedMode.DEBUG
        )
        # „provázek“ (lookahead) – pro zklidnění ↑
        L_NEAR = 2.0

        # --- FSM ---
        fsm = NavigatorFSM(FsmConfig(
            hacc_ready_m=1.5,
            acquire_heading_acc_max_deg=40.0,    # tvé „dokud přesnost není < 40°“
            acquire_heading_window_deg=15.0,     # natočit k near do ±15°
            heading_uncertain_deg=20.0,          # při jízdě – pokud >, jdi do SAFE_SPIN
            t_stable_s=0.7,
            t_hold_s=0.3,
            v_max_mps=0.5,
            omega_max_dps=90.0,
            omega_acquire_gain=2.0,
        ))

        # --- Log ---
        self._log = PilotLog(S_lat, S_lon, E_lat, E_lon, GOAL_RADIUS, ctrl.config, str(ctrl.mode), version=self.VERSION)
        state = "NAVIGATE"
        last_loop = time.monotonic()
        self._log.event(state, fsm.state.name, "Navigation loop started")
        self._set_state(mode="NAVIGATE", near_case="N/A", last_note="Navigation loop started")

        prev_fsm_state = fsm.state

        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                nav: Optional[NavFusionData] = gnss.read_nav_fusion_data()
                loop_dt_ms = (loop_start - last_loop) * 1000.0
                dt_s = max((loop_start - last_loop), 1e-3)
                last_loop = loop_start

                if not nav:
                    drive.send_pwm(0, 0)
                    self._log.event(state, fsm.state.name, "No nav data")
                    time.sleep(0.2)
                    continue

                # --- Extract heading ENU ---
                heading_gnss = getattr(nav, "heading", None)
                if heading_gnss is None:
                    heading_gnss = getattr(nav, "motHeading", None)
                if heading_gnss is None:
                    heading_gnss = getattr(nav, "vehHeading", None)
                theta_enu = heading_gnss_to_enu(float(heading_gnss or 0.0))

                # --- vzdálenost k cíli v ENU(R) ---
                Ex, Ey, _ = ecef_to_enu(*lla_to_ecef(E_lat, E_lon), nav.lat, nav.lon, 0.0)
                dist_to_goal_m = math.hypot(Ex, Ey)

                # --- near point ---
                near = select_near_point(
                    S_lat=S_lat, S_lon=S_lon,
                    E_lat=E_lat, E_lon=E_lon,
                    R_lat=nav.lat, R_lon=nav.lon,
                    L_near_m=L_NEAR,
                )

                # Bearing v ENU: 0=E,90=N
                bearing_to_goal_deg = (math.degrees(math.atan2(Ey, Ex)) + 360.0) % 360.0
                heading_error_deg = self._wrap_angle_deg(bearing_to_goal_deg - theta_enu)

                # Bearing na NEAR z (0,0)->(near_x,near_y)
                bearing_to_near_deg = (math.degrees(math.atan2(near.near_y_m or 0.0, near.near_x_m or 0.0)) + 360.0) % 360.0
                heading_err_to_near_deg = self._wrap_angle_deg(bearing_to_near_deg - theta_enu)

                # --- FSM krok ---
                quality = NavQuality(has_fix=bool(nav.gnssFixOK), hacc_m=float(nav.hAcc), heading_acc_deg=float(nav.headingAcc))
                action = fsm.step(
                    dt_s=dt_s,
                    quality=quality,
                    dist_to_goal_m=dist_to_goal_m,
                    goal_radius_m=GOAL_RADIUS,
                    near_case=near.case,
                    heading_err_to_near_deg=heading_err_to_near_deg,
                )

                # Log přechodu stavu FSM
                if action.state != prev_fsm_state:
                    self._log.event(state, action.state.name, f"FSM: {prev_fsm_state.name} -> {action.state.name}")
                    prev_fsm_state = action.state

                # --- GNSS log ---
                self._log.nav(state, action.state.name, loop_dt_ms,
                              lat=nav.lat, lon=nav.lon, alt_m=nav.alt,
                              theta_deg=theta_enu, speed_mps=nav.speed, omega_dps=nav.gyroZ,
                              hAcc_m=nav.hAcc, headingAcc_deg=nav.headingAcc,
                              gnssFixOK=nav.gnssFixOK, drUsed=nav.drUsed)

                # --- COMPUTE log ---
                self._log.compute(state, action.state.name,
                                  goal_lat=E_lat, goal_lon=E_lon, goal_radius_m=GOAL_RADIUS,
                                  dist_to_goal_m=dist_to_goal_m,
                                  bearing_to_goal_deg=bearing_to_goal_deg,
                                  heading_error_deg=heading_error_deg,
                                  near_name=getattr(near,'name',""), near_s=getattr(near,'s',""),
                                  near_case=near.case,
                                  note=f"err_to_near={heading_err_to_near_deg:.1f}")

                if near.case == "NO_INTERSECTION":
                    drive.send_pwm(0, 0)
                    self._set_state(mode="GOAL_NOT_REACHED", near_case=near.case,
                                    dist_to_goal_m=dist_to_goal_m, cross_track_m=near.d_perp_m,
                                    left_pwm=0, right_pwm=0, heading_enu_deg=theta_enu,
                                    last_note="Near selection failed (NO_INTERSECTION)")
                    time.sleep(0.2)
                    continue

                # --- Gating z FSM ---
                allow_forward = action.allow_forward
                allow_spin = action.allow_spin

                # --- Motion controller ---
                left_pwm, right_pwm, status = ctrl.compute_for_near(
                    heading_enu_deg=theta_enu,
                    near_x_m=near.near_x_m or 0.0,
                    near_y_m=near.near_y_m or 0.0,
                    allow_forward=allow_forward,
                    allow_spin=allow_spin,
                    dist_to_goal_m=dist_to_goal_m,
                    goal_radius_m=GOAL_RADIUS,
                )

                # Výstupní info pro log – limity/saturace (pokud controller nastavuje)
                v_cmd = getattr(ctrl, "last_v_cmd", 0.0)
                omega_cmd = getattr(ctrl, "last_omega_cmd", 0.0)
                v_limit = getattr(ctrl, "v_max", 0.0)
                omega_limit = getattr(ctrl, "omega_max", 0.0)
                sat_v = abs(v_cmd) >= (v_limit - 1e-3) if v_limit else 0
                sat_omega = abs(omega_cmd) >= (omega_limit - 1e-3) if omega_limit else 0

                # --- ACT_CMD log ---
                self._log.act_cmd(state, action.state.name,
                                  lookahead_m=L_NEAR,
                                  k_heading=getattr(ctrl, "k_heading_to_omega", ""),
                                  k_cte=getattr(ctrl, "k_cte", ""),
                                  v_cmd_mps=f"{v_cmd:.3f}",
                                  omega_cmd_dps=f"{omega_cmd:.2f}",
                                  v_limit_mps=f"{v_limit:.2f}",
                                  omega_limit_dps=f"{omega_limit:.2f}",
                                  sat_v=sat_v, sat_omega=sat_omega,
                                  left_pwm=left_pwm, right_pwm=right_pwm,
                                  omega_setpoint_dps=f"{action.omega_setpoint_dps:.1f}",
                                  note=status)

                # Publikace stavu + PWM
                self._set_state(mode="NAVIGATE", near_case=near.case, dist_to_goal_m=dist_to_goal_m,
                                cross_track_m=near.d_perp_m, left_pwm=left_pwm, right_pwm=right_pwm,
                                heading_enu_deg=theta_enu, last_note=f"{status} | FSM={action.state.name}")
                print(f"[PILOT] fsm={action.state.name} near={near.case} cte={near.d_perp_m:.3f}m "
                      f"goal_remain={dist_to_goal_m:.2f}m {status}")

                drive.send_pwm(left_pwm, right_pwm)

                # --- Cíl? ---
                if dist_to_goal_m <= GOAL_RADIUS:
                    print("[PILOT] Goal reached -> stop")
                    drive.send_pwm(0, 0)
                    self._log.event("GOAL_REACHED", action.state.name, "Goal reached")
                    self._set_state(mode="GOAL_REACHED", last_note="Goal reached")
                    break

            except Exception as e:
                drive.send_pwm(0, 0)
                if self._log:
                    self._log.event("NAVIGATE", prev_fsm_state.name, f"ERROR: {e}")
                print(f"[PILOT ERROR] {e}")
                traceback.print_exc()
                time.sleep(1.0)

        # Konec smyčky
        self._set_state(left_pwm=0, right_pwm=0)
        if self._log:
            self._log.event("STOPPED", prev_fsm_state.name, "Navigation ended")
            self._log.close()
            self._log = None

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
