import threading
import time
import socket
from typing import Optional

from services import send_command
from util import log_event

PORT_LIDAR   = 9002
PORT_DRIVE   = 9003
PORT_PILOT   = 9008
PORT_GNSS    = 9006
PORT_PPOINT  = 9007
# případně CAMERA = 9001 apod. podle budoucí logiky

auto_running = threading.Event()
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


def _auto_workflow():
    try:
        log_event("AUTO workflow: START")
        _safe_send_to_client("WORKFLOW AUTO START\n")

        # Minimální: zvedni základní služby
        for p in (PORT_GNSS, PORT_PPOINT, PORT_DRIVE, PORT_PILOT, PORT_LIDAR):
            _send_and_report(p, "PING")
        for p in (PORT_GNSS, PORT_PPOINT, PORT_DRIVE, PORT_PILOT, PORT_LIDAR):
            _send_and_report(p, "START")

        _safe_send_to_client("AUTO: TODO – sem přijde plná logika.\n")

        # Prozatím jen krátká smyčka čekající na STOP
        t0 = time.time()
        while not _stop_requested.is_set() and (time.time() - t0) < 2.0:
            time.sleep(0.1)

        _safe_send_to_client("WORKFLOW AUTO END\n")

    except Exception as e:
        log_event(f"AUTO WORKFLOW ERROR: {e}")
        _safe_send_to_client(f"WORKFLOW ERROR: {e}\n")
    finally:
        # Vypni služby
        try:
            for p in (PORT_PILOT, PORT_DRIVE, PORT_LIDAR, PORT_PPOINT, PORT_GNSS):
                _send_and_report(p, "STOP")
        except Exception:
            pass

        auto_running.clear()
        _stop_requested.clear()
        log_event("AUTO workflow: END")


def start_auto_workflow(client_conn: Optional[socket.socket]) -> None:
    if auto_running.is_set():
        raise RuntimeError("AUTO already running")
    auto_running.set()
    _stop_requested.clear()
    with _client_conn_lock:
        global _client_conn
        _client_conn = client_conn
    t = threading.Thread(target=_auto_workflow, daemon=True)
    t.start()


def stop_auto_workflow() -> None:
    _stop_requested.set()
    try:
        for p in (PORT_PILOT, PORT_DRIVE, PORT_LIDAR, PORT_PPOINT, PORT_GNSS):
            _send_and_report(p, "STOP")
    except Exception:
        pass
