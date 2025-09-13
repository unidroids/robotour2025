import threading
import time
import socket
import re
from pathlib import Path
from typing import Optional, Tuple

from services import send_command
from util import log_event, parse_lidar_distance

# Používané služby:
PORT_LIDAR   = 9002
PORT_DRIVE   = 9003
PORT_PILOT   = 9008
PORT_GNSS    = 9006
PORT_PPOINT  = 9007  # PointPerfect

point_running = threading.Event()
_stop_requested = threading.Event()

_client_conn_lock = threading.Lock()
_client_conn: Optional[socket.socket] = None


def _safe_send_to_client(text: str) -> None:
    with _client_conn_lock:
        conn = _client_conn
    if not conn:
        return
    try:
        conn.sendall(text.encode())
    except Exception:
        pass


def _send_and_report(port: int, cmd: str, expect_response: bool = True) -> str:
    resp = send_command(port, cmd, expect_response=expect_response)
    line = f"SERVICE[{port}] {cmd} -> {resp}"
    _safe_send_to_client(line + "\n")
    return resp


def _read_point_ini() -> Tuple[float, float, float]:
    p = Path(__file__).with_name("point.ini")
    txt = p.read_text(encoding="utf-8").strip()
    parts = txt.replace(",", " ").split()
    if len(parts) < 3:
        raise ValueError("point.ini musí být: 'lat lon radius'")
    lat = float(parts[0]); lon = float(parts[1]); radius = float(parts[2])
    return lat, lon, radius


def _hacc_mm_from_gnss_data(s: str) -> Optional[float]:
    """
    Tolerantní parser hAcc:
    - podporuje formáty: "hAcc=0.42 m", "hAcc: 420 mm", '"hAcc":0.42' (m),
      i např. "hAcc 0.42 m".
    Vrací hodnotu v milimetrech.
    """
    if not s:
        return None

    # JSON-like: "hAcc": 0.42  => m (předpoklad)
    m = re.search(r'"?hAcc"?\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)\s*(mm|cm|m)?', s, flags=re.I)
    if not m:
        # fallback: hAcc 0.42 m
        m = re.search(r'hAcc\s+([0-9]+(?:\.[0-9]+)?)\s*(mm|cm|m)?', s, flags=re.I)

    if not m:
        return None

    val = float(m.group(1))
    unit = (m.group(2) or "m").lower()

    if unit == "mm":
        return val
    if unit == "cm":
        return val * 10.0
    # default/meters:
    return val * 1000.0


def _distance_cm() -> Optional[float]:
    resp = _send_and_report(PORT_LIDAR, "DISTANCE")
    idx, dist = parse_lidar_distance(resp)
    if idx is None:
        return None
    if idx == -1:
        return None
    return dist  # očekáváme cm jako v demo workflow


def _gnss_hacc_mm() -> Optional[float]:
    resp = _send_and_report(PORT_GNSS, "DATA")
    return _hacc_mm_from_gnss_data(resp)


def _pilot_status() -> str:
    resp = _send_and_report(PORT_PILOT, "STATUS")
    return resp.strip().upper()


def _emergency_brake_5s() -> None:
    """Pošle CLEAR na PILOT a 5 s drží PWM 0 0 (50 ms interval)."""
    try:
        _send_and_report(PORT_PILOT, "CLEAR")
    except Exception:
        pass

    t_end = time.time() + 5.0
    while time.time() < t_end and not _stop_requested.is_set():
        _send_and_report(PORT_DRIVE, "PWM 0 0", expect_response=False)
        time.sleep(0.05)


def _safe_to_go(hacc_mm: Optional[float], dist_cm: Optional[float]) -> bool:
    return (hacc_mm is not None and hacc_mm < 500.0) and (dist_cm is not None and dist_cm > 50.0)


def _unsafe(hacc_mm: Optional[float], dist_cm: Optional[float]) -> bool:
    # Pozor: pokud některá hodnota není k dispozici, nevyhodnocujeme jako unsafe (vrátí False)
    if hacc_mm is not None and hacc_mm > 800.0:
        return True
    if dist_cm is not None and dist_cm < 40.0:
        return True
    return False


def _point_workflow():
    try:
        log_event("POINT workflow: START")
        _safe_send_to_client("WORKFLOW POINT START\n")

        lat, lon, radius = _read_point_ini()

        # 1) START všech služeb (GNSS, PP, DRIVE, PILOT, LIDAR)
        _send_and_report(PORT_GNSS,   "PING")
        _send_and_report(PORT_PPOINT, "PING")
        _send_and_report(PORT_DRIVE,  "PING")
        _send_and_report(PORT_PILOT,  "PING")
        _send_and_report(PORT_LIDAR,  "PING")

        _send_and_report(PORT_GNSS,   "START")
        _send_and_report(PORT_PPOINT, "START")
        _send_and_report(PORT_DRIVE,  "START")
        _send_and_report(PORT_PILOT,  "START")
        _send_and_report(PORT_LIDAR,  "START")

        # 2) Hlavní smyčka
        last_gnss_ts   = 0.0
        last_status_ts = 0.0

        hacc_mm: Optional[float] = None
        dist_cm: Optional[float] = None
        last_status = ""

        waypoint_sent = False
        brake_phase   = False  # indikace, že jsme byli v nouzovém zastavení

        while not _stop_requested.is_set():
            now = time.time()

            # a) LIDAR – čti průběžně (rychleji)
            dist_cm = _distance_cm()

            # b) GNSS – cca 1 s
            if now - last_gnss_ts >= 1.0:
                hacc_mm = _gnss_hacc_mm()
                last_gnss_ts = now

            # c) PILOT STATUS – cca 1 s
            if now - last_status_ts >= 1.0:
                try:
                    st = _pilot_status()
                    if st != last_status:
                        _safe_send_to_client(f"PILOT STATUS: {st}\n")
                        last_status = st
                    if st == "REACHED":
                        log_event("POINT workflow: REACHED – končím.")
                        break
                except Exception:
                    pass
                last_status_ts = now

            # d) Bezpečnostní logika
            if _unsafe(hacc_mm, dist_cm):
                _safe_send_to_client("STATE: UNSAFE -> CLEAR + PWM 0 0 (5s)\n")
                _emergency_brake_5s()
                brake_phase = True
                waypoint_sent = False  # budeme posílat waypoint znovu až po zklidnění
                continue

            # e) Když je bezpečno a ještě jsme neposlali waypoint (nebo po brzde)
            if _safe_to_go(hacc_mm, dist_cm) and not waypoint_sent:
                cmd = f"WAYPOINT {lat:.7f} {lon:.7f} {radius:.3f}"
                _send_and_report(PORT_PILOT, cmd)
                waypoint_sent = True
                brake_phase = False
                _safe_send_to_client(f"SENT: {cmd}\n")

            time.sleep(0.10)  # rozumná perioda smyčky

        _safe_send_to_client("WORKFLOW POINT END\n")

    except Exception as e:
        log_event(f"POINT WORKFLOW ERROR: {e}")
        _safe_send_to_client(f"WORKFLOW ERROR: {e}\n")

    finally:
        # Při ukončení workflow pošli STOP všem dotčeným službám
        try:
            _send_and_report(PORT_PILOT,  "STOP")
            _send_and_report(PORT_DRIVE,  "STOP")
            _send_and_report(PORT_LIDAR,  "STOP")
            _send_and_report(PORT_PPOINT, "STOP")
            _send_and_report(PORT_GNSS,   "STOP")
        except Exception as e:
            log_event(f"POINT stop cleanup error: {e}")

        point_running.clear()
        _stop_requested.clear()
        log_event("POINT workflow: END")


def start_point_workflow(client_conn: Optional[socket.socket]) -> None:
    """Spustí POINT workflow (pokud už neběží)."""
    if point_running.is_set():
        raise RuntimeError("POINT already running")
    point_running.set()
    _stop_requested.clear()
    with _client_conn_lock:
        global _client_conn
        _client_conn = client_conn
    t = threading.Thread(target=_point_workflow, daemon=True)
    t.start()


def stop_point_workflow() -> None:
    """Požádá o STOP a pošle STOP do všech relevantních služeb (idempotentní)."""
    _stop_requested.set()
    try:
        _send_and_report(PORT_PILOT,  "STOP")
        _send_and_report(PORT_DRIVE,  "STOP")
        _send_and_report(PORT_LIDAR,  "STOP")
        _send_and_report(PORT_PPOINT, "STOP")
        _send_and_report(PORT_GNSS,   "STOP")
    except Exception:
        pass
