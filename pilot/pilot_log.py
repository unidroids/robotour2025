# pilot_log.py
from __future__ import annotations
import os
import csv
import json
import time
import math
import datetime
from typing import Optional

def _now_iso():
    # Lokální čas, milisekundy, s offsetem (Praha)
    return datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=2))).isoformat(timespec="milliseconds")

def _wrap_deg(angle: float) -> float:
    """Zavinutí do (-180, 180]."""
    a = (angle + 180.0) % 360.0 - 180.0
    return a if a != -180.0 else 180.0

class PilotLog:
    """
    CSV logger pro službu PILOT.
    - 1 soubor na běh: /data/robot/pilot/PILOT_YYYYMMDD_HHMMSS.csv
    - první řádek: RUN_META s JSON parametry (start/goal/radius + controller config)
    - řádky: GNSS_IN, COMPUTE, ACT_CMD, EVENT
    """

    HEADER = [
        "ts_iso","t_mono","typ","state","loop_dt_ms",
        "lat","lon","alt_m","theta_deg","speed_mps","omega_dps","hAcc_m","headingAcc_deg","gnssFixOK","drUsed",
        "goal_lat","goal_lon","goal_radius_m","dist_to_goal_m","bearing_to_goal_deg","heading_error_deg",
        "near_case","near_x_m","near_y_m","cte_m",
        "lookahead_m","k_heading","k_cte","v_cmd_mps","omega_cmd_dps","v_limit_mps","omega_limit_dps","sat_v","sat_omega",
        "left_pwm","right_pwm","note"
    ]

    def __init__(
        self,
        start_lat: float, start_lon: float,
        goal_lat: float, goal_lon: float, goal_radius: float,
        ctrl,  # MotionController2D
        lookahead_m: float,
        log_dir: str = "/data/robot/pilot/"
    ) -> None:
        os.makedirs(log_dir, exist_ok=True)
        ts_name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"PILOT_{ts_name}.csv")
        self.f = open(self.path, "w", encoding="utf-8", newline='', buffering=1)
        self.w = csv.writer(self.f, delimiter=';')
        self.w.writerow(self.HEADER)

        # Ulož meta
        meta = {
            "start_lat": float(start_lat),
            "start_lon": float(start_lon),
            "goal_lat": float(goal_lat),
            "goal_lon": float(goal_lon),
            "goal_radius": float(goal_radius),
            "lookahead_m": float(lookahead_m),
            "pilot_version": getattr(ctrl, "VERSION", None) or getattr(self, "VERSION", "1.0.0"),
            "controller": {
                "mode": getattr(ctrl, "mode", None).__class__.__name__ if getattr(ctrl, "mode", None) else None,
                "v_max_debug_mps": getattr(ctrl, "cfg", None).v_max_debug_mps if getattr(ctrl, "cfg", None) else None,
                "v_max_normal_mps": getattr(ctrl, "cfg", None).v_max_normal_mps if getattr(ctrl, "cfg", None) else None,
                "omega_max_dps": getattr(ctrl, "cfg", None).omega_max_dps if getattr(ctrl, "cfg", None) else None,
                "max_pwm": getattr(ctrl, "cfg", None).max_pwm if getattr(ctrl, "cfg", None) else None,
                "deadband_pwm": getattr(ctrl, "cfg", None).deadband_pwm if getattr(ctrl, "cfg", None) else None,
                "slow_down_dist_m": getattr(ctrl, "cfg", None).slow_down_dist_m if getattr(ctrl, "cfg", None) else None,
                "k_heading_to_omega": getattr(ctrl, "cfg", None).k_heading_to_omega if getattr(ctrl, "cfg", None) else None,
                "v_scale": getattr(ctrl, "cfg", None).v_scale if getattr(ctrl, "cfg", None) else None,
            },
        }
        self._write_row("RUN_META", state="", note=json.dumps(meta))

    def close(self) -> None:
        try:
            self.f.flush()
        finally:
            self.f.close()

    # ------------------- PUBLIC API -------------------

    def event(self, state: str, note: str) -> None:
        self._write_row("EVENT", state=state, note=note)

    def nav(self, state: str, loop_dt_ms: Optional[float], nav, theta_enu: float) -> None:
        """Log vstupu z GNSS fúze (30 Hz)."""
        self._write_row(
            "GNSS_IN",
            state=state,
            loop_dt_ms=f"{loop_dt_ms:.1f}" if loop_dt_ms is not None else "",
            lat=getattr(nav, "lat", ""),
            lon=getattr(nav, "lon", ""),
            alt_m=getattr(nav, "alt", ""),
            theta_deg=theta_enu,
            speed_mps=getattr(nav, "speed", ""),
            omega_dps=getattr(nav, "gyroZ", ""),
            hAcc_m=getattr(nav, "hAcc", ""),
            headingAcc_deg=getattr(nav, "headingAcc", ""),
            gnssFixOK=int(bool(getattr(nav, "gnssFixOK", False))),
            drUsed=int(bool(getattr(nav, "drUsed", False))),
        )

    def near_and_errors(
        self,
        state: str,
        goal_lat: float, goal_lon: float, goal_radius_m: float,
        R_lat: float, R_lon: float,
        theta_enu: float,
        near,  # objekt z select_near_point()
    ) -> dict:
        """
        Spočítá diagnostické hodnoty (dist k cíli, bearing ENU, heading_error) a zaloguje COMPUTE.
        Vrací dict s 'dist_to_goal_m', 'bearing_to_goal_deg', 'heading_error_deg'.
        """
        # ENU vektory: R (referenční = nav.lat, nav.lon) -> E (cíl)
        Ex, Ey, _ = _ecef_to_enu_goal(goal_lat, goal_lon, R_lat, R_lon)
        dist_to_goal_m = math.hypot(Ex, Ey)
        bearing_to_goal_deg = math.degrees(math.atan2(Ey, Ex))  # ENU: 0=E, 90=N
        heading_error_deg = _wrap_deg(bearing_to_goal_deg - theta_enu)

        self._write_row(
            "COMPUTE",
            state=state,
            goal_lat=goal_lat, goal_lon=goal_lon, goal_radius_m=goal_radius_m,
            dist_to_goal_m=f"{dist_to_goal_m:.2f}",
            bearing_to_goal_deg=f"{bearing_to_goal_deg:.2f}",
            heading_error_deg=f"{heading_error_deg:.2f}",
            near_case=getattr(near, "case", ""),
            near_x_m=getattr(near, "near_x_m", ""),
            near_y_m=getattr(near, "near_y_m", ""),
            cte_m=getattr(near, "d_perp_m", ""),
        )
        return {
            "dist_to_goal_m": dist_to_goal_m,
            "bearing_to_goal_deg": bearing_to_goal_deg,
            "heading_error_deg": heading_error_deg,
        }

    def act_cmd(
        self,
        state: str,
        lookahead_m: float,
        k_heading: float,
        k_cte: Optional[float],
        v_cmd_mps: Optional[float],
        omega_cmd_dps: Optional[float],
        v_limit_mps: float,
        omega_limit_dps: float,
        left_pwm: int, right_pwm: int,
        sat_v: Optional[int] = "",
        sat_omega: Optional[int] = "",
        note: str = "",
    ) -> None:
        self._write_row(
            "ACT_CMD",
            state=state,
            lookahead_m=lookahead_m,
            k_heading=k_heading,
            k_cte=(k_cte if k_cte is not None else ""),
            v_cmd_mps=(f"{v_cmd_mps:.3f}" if isinstance(v_cmd_mps, (int, float)) else ""),
            omega_cmd_dps=(f"{omega_cmd_dps:.2f}" if isinstance(omega_cmd_dps, (int, float)) else ""),
            v_limit_mps=f"{v_limit_mps:.2f}",
            omega_limit_dps=f"{omega_limit_dps:.2f}",
            sat_v=sat_v,
            sat_omega=sat_omega,
            left_pwm=left_pwm,
            right_pwm=right_pwm,
            note=note,
        )

    # ------------------- interní ---------------------

    def _write_row(self, typ: str, state: str = "", note: str = "", **cols) -> None:
        t_iso = _now_iso()
        t_mono = f"{time.monotonic():.3f}"
        # Postav mapu všech sloupců s defaulty
        row = {h: "" for h in self.HEADER}
        row.update({
            "ts_iso": t_iso,
            "t_mono": t_mono,
            "typ": typ,
            "state": state,
            "note": note,
        })
        # Doplnění poskytnutých hodnot
        for k, v in cols.items():
            if k in row:
                row[k] = v
        # Zapiš v pořadí hlavičky
        self.w.writerow([row[h] for h in self.HEADER])

# Pomocná funkce pro výpočet ENU (R->E) bez nutnosti importovat pilotovy utilitky:
# Použijeme stejnou logiku jako pilot: lla_to_ecef / ecef_to_enu (průchozí přes funkce v pilotu),
# ale zde jen proxy, aby PilotLog fungoval samostatně, když mu dáme R(E) přes externí wrapper.
def _ecef_to_enu_goal(E_lat: float, E_lon: float, R_lat: float, R_lon: float):
    # Lazy import (kvůli čisté separaci)
    from geo_utils import lla_to_ecef, ecef_to_enu
    Ex, Ey, Ez = ecef_to_enu(*lla_to_ecef(E_lat, E_lon), R_lat, R_lon, 0.0)
    return Ex, Ey, Ez
