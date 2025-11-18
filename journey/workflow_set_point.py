import threading
import time
import socket
import json
import re
from pathlib import Path
from typing import Optional, Tuple

from services import send_command
from util import log_event


# Používané služby:
PORT_GNSS    = 9006
PORT_PPOINT  = 9007  # PointPerfect
PORT_FUSION  = 9009
PORT_HEADING = 9010

# Stav běhu workflow SET-POINT
setpoint_running = threading.Event()

# Interní stop flag pro toto workflow
_stop_requested = threading.Event()

# Připojení klienta (pro průběžné hlášky)
_client_conn_lock = threading.Lock()
_client_conn: Optional[socket.socket] = None


def _safe_send_to_client(text: str) -> None:
    """Bezpečné odeslání textu na aktuální klientské spojení (pokud existuje)."""
    with _client_conn_lock:
        conn = _client_conn
    if not conn:
        return
    try:
        conn.sendall(text.encode())
    except Exception:
        pass


def _send_and_report(port: int, cmd: str, expect_response: bool = True) -> str:
    """
    Pošle cmd na daný port, vrátí odpověď a zároveň ji pošle klientovi
    ve formátu: SERVICE[port] CMD -> RESP
    """
    resp = send_command(port, cmd, expect_response=expect_response)
    line = f"SERVICE[{port}] {cmd} -> {resp}"
    _safe_send_to_client(line + "\n")
    return resp


def _write_point_ini(lat: float, lon: float, radius_m: float = 1.0) -> None:
    """
    Zapíše bod do point.ini ve formátu:
    'lat, lon radius'
    (např. '50.0615544, 14.5998017 1.0')
    Radius == vzdálenost k cíli (1 m).
    """
    p = Path(__file__).with_name("point.ini")
    line = f"{lat:.7f} {lon:.7f} {radius_m:.3f}\n"
    p.write_text(line, encoding="utf-8")


def _hacc_mm_from_str(s: str) -> Optional[float]:
    """
    Z řetězce s JSON payloadem (např. odpověď FUSION/DATA) se pokusí
    vytáhnout hAcc (v mm). Hledá zanořeně v dict/list strukturách.

    Pokud JSON parsování selže, použije fallback regex na 'hAcc: číslo'.
    """
    if not s:
        return None

    try:
        # Najít JSON část mezi { ... }
        start = s.find("{")
        end = s.rfind("}")
        payload = s[start:end + 1] if (start != -1 and end != -1 and end > start) else s
        obj = json.loads(payload)

        def find_hacc(o) -> Optional[float]:
            if isinstance(o, dict):
                if "hAcc" in o and isinstance(o["hAcc"], (int, float)):
                    return float(o["hAcc"])
                for v in o.values():
                    r = find_hacc(v)
                    if r is not None:
                        return r
            elif isinstance(o, list):
                for v in o:
                    r = find_hacc(v)
                    if r is not None:
                        return r
            return None

        val = find_hacc(obj)
        return float(val) if val is not None else None

    except Exception:
        # Fallback: hAcc: číslo
        m = re.search(r'"?hAcc"?\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)', s)
        return float(m.group(1)) if m else None


def _extract_pose_and_hacc_mm_from_fusion() -> Optional[Tuple[float, float, float]]:
    """
    Zavolá na FUSION službu 'DATA' a z odpovědi vytáhne:
      - lat (float)
      - lon (float)
      - hAcc v mm (float)

    Vrací tuple (lat, lon, hAcc_mm) nebo None, pokud něco chybí.
    """
    resp = _send_and_report(PORT_FUSION, "DATA")
    if not resp:
        return None

    hacc_mm = _hacc_mm_from_str(resp)

    #print(hacc_mm)

    try:
        start = resp.find("{")
        end = resp.rfind("}")
        payload = resp[start:end + 1] if (start != -1 and end != -1 and end > start) else resp
        obj = json.loads(payload)
        lat = obj.get("lat")
        lon = obj.get("lon")
    except Exception:
        return None

    if lat is None or lon is None or hacc_mm is None:
        return None

    return float(lat), float(lon), float(hacc_mm)


def _set_point_workflow() -> None:
    """
    Workflow SET-POINT:

    - PING na FUSION, GNSS, POINT, HEADING (pro rychlou validaci).
    - START na GNSS, POINT (PointPerfect), HEADING.
    - Opakovaně čte FUSION/DATA, z toho lat, lon, hAcc (mm).
    - Jakmile hAcc < 500 mm (0.5 m), uloží:
          lat, lon, radius=1.0
      do point.ini.
    - Vždy nakonec pošle STOP na GNSS, POINT, HEADING.
    """
    try:
        log_event("SET-POINT workflow: START")
        _safe_send_to_client("WORKFLOW SET-POINT START\n")

        # --- PING sanity check ---
        _send_and_report(PORT_FUSION, "PING")
        _send_and_report(PORT_GNSS,   "PING")
        _send_and_report(PORT_PPOINT, "PING")
        _send_and_report(PORT_HEADING,"PING")

        # --- START potřebných služeb ---
        _send_and_report(PORT_FUSION, "RESTART")
        _send_and_report(PORT_GNSS,   "START")
        _send_and_report(PORT_PPOINT, "START")
        _send_and_report(PORT_HEADING,"START")

        _safe_send_to_client("WAIT FUSION(hAcc<500mm)...\n")

        timeout_s = 120.0
        poll_s = 1.0
        t0 = time.time()
        saved = False
        threshold_mm = 500.0   # 0.5 m
        radius_m = 1.0         # vzdálenost k cíli, uložená do point.ini

        while not _stop_requested.is_set() and (time.time() - t0) < timeout_s:
            pose = _extract_pose_and_hacc_mm_from_fusion()
            if pose is None:
                time.sleep(poll_s)
                continue

            lat, lon, hacc_mm = pose
            _safe_send_to_client(
                f"FUSION hAcc: {hacc_mm:.0f} mm (lat={lat:.7f}, lon={lon:.7f})\n"
            )

            if hacc_mm < threshold_mm:
                _write_point_ini(lat=lat, lon=lon, radius_m=radius_m)
                msg = f"POINT_SET: {lat:.7f} {lon:.7f} {radius_m:.3f}\n"
                _safe_send_to_client(msg)
                log_event(f"SET-POINT workflow: uložen bod lat:{lat:.7f}, lon:{lon:.7f}, radius={radius_m:.3f} m")
                saved = True
                break

            time.sleep(poll_s)

        if not saved:
            _safe_send_to_client(
                "ERROR: FUSION hAcc >= 500mm po timeoutu, point.ini NEBYL aktualizován.\n"
            )
            log_event("SET-POINT workflow: timeout bez dosažení hAcc < 500 mm")

        _safe_send_to_client("WORKFLOW SET-POINT END\n")

    except Exception as e:
        log_event(f"SET-POINT WORKFLOW ERROR: {e}")
        _safe_send_to_client(f"WORKFLOW SET-POINT ERROR: {e}\n")

    finally:
        # Úklid: STOP jen na služby, které jsme pro toto workflow startovali
        try:
            _send_and_report(PORT_HEADING, "STOP")
            _send_and_report(PORT_PPOINT,  "STOP")
            _send_and_report(PORT_GNSS,    "STOP")
        except Exception as e:
            log_event(f"SET-POINT stop cleanup error: {e}")

        setpoint_running.clear()
        _stop_requested.clear()
        log_event("SET-POINT workflow: END")


def start_setpoint_workflow(client_conn: Optional[socket.socket]) -> None:
    """
    Spustí workflow SET-POINT (pokud už neběží).
    Volá se z Journey serveru při příkazu SET-POINT.
    """
    if setpoint_running.is_set():
        raise RuntimeError("SET-POINT already running")
    setpoint_running.set()
    _stop_requested.clear()
    with _client_conn_lock:
        global _client_conn
        _client_conn = client_conn
    t = threading.Thread(target=_set_point_workflow, daemon=True)
    t.start()


def stop_setpoint_workflow() -> None:
    """
    Požádá o STOP SET-POINT workflow a pošle STOP do relevantních služeb.
    Idempotentní – stejné STOPy jako ve finally.
    """
    _stop_requested.set()
    try:
        _send_and_report(PORT_HEADING, "STOP")
        _send_and_report(PORT_PPOINT,  "STOP")
        _send_and_report(PORT_GNSS,    "STOP")
        _send_and_report(PORT_FUSION, "RESTART")
    except Exception:
        pass
