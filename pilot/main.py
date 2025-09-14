#!/usr/bin/env python3
import socket
import signal
import threading
import sys
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

HOST = "127.0.0.1"
PORT = 9008  # Autopilot (Pilot)

# Importy jsou jednosměrné: main -> client,control (control neimportuje main)
from client import handle_client
from control import stop_controller


def log(msg: str):
    print(msg, flush=True)


@dataclass
class AutopilotContext:
    shutdown_flag: threading.Event = field(default_factory=threading.Event)
    lock: threading.RLock = field(default_factory=threading.RLock)

    # Stav služby
    status: str = "IDLE"       # IDLE | RUNNING | REACHED | ERROR
    status_msg: str = ""       # detail k ERROR/diagnostice

    # Waypoint: (lat, lon, reach_radius_m) nebo None
    waypoint: Optional[Tuple[float, float, float]] = None

    # Poslední známá GNSS pozice (lat, lon)
    last_pose: Optional[Tuple[float, float]] = None

    # Drží instanci kontroleru (třída v control.py)
    controller_thread: Optional[object] = None

    # Vlákna klientů
    client_threads: List[threading.Thread] = field(default_factory=list)


def run_server():
    ctx = AutopilotContext()

    # graceful shutdown
    def _sig_handler(signum, frame):
        ctx.shutdown_flag.set()
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)
    srv.settimeout(1.0)

    log(f"🚀 Pilot naslouchá na {HOST}:{PORT}")

    try:
        while not ctx.shutdown_flag.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            log(f"📱 Klient připojen: {addr}")
            t = threading.Thread(target=handle_client, args=(conn, addr, ctx), daemon=True)
            t.start()
            ctx.client_threads.append(t)
    finally:
        # Bezpečné zastavení robota a úklid
        try:
            stop_controller(ctx)
        except Exception as e:
            log(f"⚠️ stop_controller selhal: {e}")

        try:
            srv.close()
        except Exception:
            pass

        for t in ctx.client_threads:
            t.join(timeout=1.0)

        log("🛑 Pilot ukončen.")


if __name__ == "__main__":
    try:
        run_server()
    except Exception as e:
        print(f"Fatální chyba: {e}", file=sys.stderr)
        sys.exit(1)
