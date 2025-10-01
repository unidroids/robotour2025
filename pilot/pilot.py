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
from near_waypoint import select_near_point
from motion_controller import MotionController2D, ControllerConfig, SpeedMode


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

        self._set_state(mode="NAVIGATE", near_case="N/A", last_note="Navigation loop started")

        while not self._stop_event.is_set():
            try:
                nav: Optional[NavFusionData] = gnss.read_nav_fusion_data()
                if not nav:
                    drive.send_pwm(0, 0)
                    self._set_state(left_pwm=0, right_pwm=0, last_note="No nav data")
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

                # --- Vzdálenost k cíli v ENU(R) (pro zpomalování a GOAL_RADIUS) ---
                Ex, Ey, _ = ecef_to_enu(*lla_to_ecef(E_lat, E_lon), nav.lat, nav.lon, 0.0)
                dist_to_goal_m = (Ex**2 + Ey**2) ** 0.5

                if near.case == "NO_INTERSECTION":
                    # Eskalace nahoru: zastav a publikuj stav
                    drive.send_pwm(0, 0)
                    self._set_state(
                        mode="GOAL_NOT_REACHED",
                        near_case=near.case,
                        dist_to_goal_m=dist_to_goal_m,
                        cross_track_m=near.d_perp_m,
                        left_pwm=0, right_pwm=0,
                        heading_enu_deg=heading_enu,
                        last_note="Near selection failed (NO_INTERSECTION)"
                    )
                    print("[PILOT] Near selection failed (NO_INTERSECTION) -> GOAL_NOT_REACHED")
                    time.sleep(0.2)
                    continue

                # --- Povolení dopředně/spin (zatím bez FSM: obojí povoleno) ---
                allow_forward = True
                allow_spin = True

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

                # Publikuj stav
                self._set_state(
                    mode="NAVIGATE",
                    near_case=near.case,
                    dist_to_goal_m=dist_to_goal_m,
                    cross_track_m=near.d_perp_m,
                    left_pwm=left_pwm, right_pwm=right_pwm,
                    heading_enu_deg=heading_enu,
                    last_note=status
                )

                # Debug log
                print(f"[PILOT] near={near.case} cte={near.d_perp_m:.3f}m "
                      f"goal_remain={dist_to_goal_m:.2f}m {status}")

                # Odeslání na kola
                drive.send_pwm(left_pwm, right_pwm)

                # --- Dosažen cíl? ---
                if dist_to_goal_m <= GOAL_RADIUS:
                    print("[PILOT] Goal reached -> stop")
                    drive.send_pwm(0, 0)
                    self._set_state(mode="GOAL_REACHED", last_note="Goal reached")
                    break

            except Exception as e:
                drive.send_pwm(0, 0)
                self._set_state(mode="NAVIGATE", left_pwm=0, right_pwm=0, last_note=f"ERROR: {e}")
                print(f"[PILOT ERROR] {e}")
                traceback.print_exc()
                time.sleep(1.0)

        # konec smyčky
        self._set_state(left_pwm=0, right_pwm=0)

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
