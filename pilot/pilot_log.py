# pilot_log.py
from __future__ import annotations
import os, csv, json, time, datetime
from typing import Optional

class PilotLog:
    def __init__(self, start_lat: float, start_lon: float,
                 goal_lat: float, goal_lon: float, goal_radius: float,
                 ctrl_config: object, ctrl_mode: str,
                 version: str = "1.0.0",
                 log_dir: str = "/data/robot/pilot") -> None:
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"PILOT_{ts}.csv")
        self._f = open(self.path, "w", encoding="utf-8", newline='', buffering=1)
        self._w = csv.writer(self._f, delimiter=';')
        self._write_header()
        meta = {
            "start_lat": float(start_lat), "start_lon": float(start_lon),
            "goal_lat": float(goal_lat), "goal_lon": float(goal_lon),
            "goal_radius": float(goal_radius),
            "ctrl": getattr(ctrl_config, "__dict__", str(ctrl_config)),
            "ctrl_mode": str(ctrl_mode),
            "pilot_version": version,
        }
        self._write_row("RUN_META", state="", fsm_state="", note=json.dumps(meta, ensure_ascii=False))

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:
            pass

    def _now_iso(self) -> str:
        return datetime.datetime.now().isoformat(timespec="milliseconds")

    def _write_header(self) -> None:
        self._w.writerow([
            "ts_iso","t_mono","typ","state","fsm_state","loop_dt_ms",
            "lat","lon","alt_m","theta_deg","speed_mps","omega_dps","hAcc_m","headingAcc_deg","gnssFixOK","drUsed",
            "goal_lat","goal_lon","goal_radius_m","dist_to_goal_m","bearing_to_goal_deg","heading_error_deg",
            "near_name","near_s","near_case",
            "lookahead_m","k_heading","k_cte","v_cmd_mps","omega_cmd_dps","v_limit_mps","omega_limit_dps","sat_v","sat_omega",
            "left_pwm","right_pwm","omega_setpoint_dps","note"
        ])

    def _write_row(self, typ: str, **kw) -> None:
        t_iso = self._now_iso()
        t_mono = f"{time.monotonic():.3f}"
        get = lambda k: kw.get(k, "")
        self._w.writerow([
            t_iso, t_mono, typ,
            get("state"), get("fsm_state"), get("loop_dt_ms"),
            get("lat"), get("lon"), get("alt_m"), get("theta_deg"), get("speed_mps"), get("omega_dps"),
            get("hAcc_m"), get("headingAcc_deg"), get("gnssFixOK"), get("drUsed"),
            get("goal_lat"), get("goal_lon"), get("goal_radius_m"), get("dist_to_goal_m"),
            get("bearing_to_goal_deg"), get("heading_error_deg"),
            get("near_name"), get("near_s"), get("near_case"),
            get("lookahead_m"), get("k_heading"), get("k_cte"), get("v_cmd_mps"), get("omega_cmd_dps"),
            get("v_limit_mps"), get("omega_limit_dps"), get("sat_v"), get("sat_omega"),
            get("left_pwm"), get("right_pwm"), get("omega_setpoint_dps"),
            get("note"),
        ])

    def event(self, state: str, fsm_state: str, note: str) -> None:
        self._write_row("EVENT", state=state, fsm_state=fsm_state, note=note)

    def nav(self, state: str, fsm_state: str, loop_dt_ms: float,
            lat: float, lon: float, alt_m: float,
            theta_deg: float, speed_mps: float, omega_dps: float,
            hAcc_m: float, headingAcc_deg: float, gnssFixOK: bool, drUsed: bool) -> None:
        self._write_row("GNSS_IN",
                        state=state, fsm_state=fsm_state, loop_dt_ms=f"{loop_dt_ms:.1f}",
                        lat=lat, lon=lon, alt_m=alt_m,
                        theta_deg=theta_deg, speed_mps=speed_mps, omega_dps=omega_dps,
                        hAcc_m=hAcc_m, headingAcc_deg=headingAcc_deg, gnssFixOK=int(bool(gnssFixOK)),
                        drUsed=int(bool(drUsed)))

    def compute(self, state: str, fsm_state: str,
                goal_lat: float, goal_lon: float, goal_radius_m: float,
                dist_to_goal_m: float, bearing_to_goal_deg: float, heading_error_deg: float,
                near_name: str, near_s, near_case: str, note: str = "") -> None:
        self._write_row("COMPUTE",
                        state=state, fsm_state=fsm_state,
                        goal_lat=goal_lat, goal_lon=goal_lon, goal_radius_m=goal_radius_m,
                        dist_to_goal_m=f"{dist_to_goal_m:.2f}",
                        bearing_to_goal_deg=f"{bearing_to_goal_deg:.2f}",
                        heading_error_deg=f"{heading_error_deg:.2f}",
                        near_name=near_name, near_s=near_s, near_case=near_case,
                        note=note)

    def act_cmd(self, state: str, fsm_state: str,
                lookahead_m: float, k_heading, k_cte,
                v_cmd_mps, omega_cmd_dps, v_limit_mps, omega_limit_dps,
                sat_v, sat_omega, left_pwm: int, right_pwm: int,
                omega_setpoint_dps: float, note: str) -> None:
        self._write_row("ACT_CMD",
                        state=state, fsm_state=fsm_state,
                        lookahead_m=lookahead_m, k_heading=k_heading, k_cte=k_cte,
                        v_cmd_mps=v_cmd_mps, omega_cmd_dps=omega_cmd_dps,
                        v_limit_mps=v_limit_mps, omega_limit_dps=omega_limit_dps,
                        sat_v=int(bool(sat_v)), sat_omega=int(bool(sat_omega)),
                        left_pwm=left_pwm, right_pwm=right_pwm,
                        omega_setpoint_dps=omega_setpoint_dps,
                        note=note)
