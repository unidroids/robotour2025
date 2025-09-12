# autopilot/main.py
import socket
import signal
import threading
import sys
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

# ── Konfigurace služby ─────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 9008 

# Stavové hodnoty (včetně požadovaného REACHED)
STATUS_IDLE = "IDLE"
STATUS_RUNNING = "RUNNING"
STATUS_REACHED = "REACHED"
STATUS_ERROR = "ERROR"

# handle_client bude dodán v dalším kroku
from client import handle_client  # def handle_client(conn, addr, ctx): ...
# start/stop kontroleru budou v control.py (další krok)
from control import start_controller, stop_controller


# ── Sdílený kontext služby ────────────────────────────────────────────────────
@dataclass
class AutopilotContext:
    shutdown_flag: threading.Event = field(default_factory=threading.Event)
    lock: threading.RLock = field(default_factory=threading.RLock)

    # stav služby
    status: str = STATUS_IDLE
    status_msg: str = ""  # detail k ERROR/diagnostice

    # cílový waypoint: (lat, lon, reach_radius_m) nebo None
    waypoint: Optional[Tuple[float, float, float]] = None

    # poslední známá pozice robota (lat, lon), doplní kontroler přes GNSS službu
    last_pose: Optional[Tuple[float, float]] = None

    # interní kontroler (vlákno) – nastavuje start_controller/stop_controller
    controller_thread: Optional[threading.Thread] = None

    # pro sledování klientských vláken kvůli čistému ukončení
    client_threads: List[threading.Thread] = field(default_factory=list)


# ── Log helper ────────────────────────────────────────────────────────────────
def log(msg: str):
    print(msg, flush=True)


# ── Server ────────────────────────────────────────────────────────────────────
def run_server():
    ctx = AutopilotContext()

    # graceful shutdown přes SIGINT (Ctrl+C) i SIGTERM (systemd)
    def _sig_handler(signum, frame):
        ctx.shutdown_flag.set()
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    # TCP server
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)
    srv.settimeout(1.0)

    log(f"🚀 Autopilot nasloucha na {HOST}:{PORT}")

    try:
        while not ctx.shutdown_flag.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break  # socket již zavřen

            log(f"📱 Klient pripojen: {addr}")
            t = threading.Thread(
                target=handle_client,
                args=(conn, addr, ctx),
                daemon=True,
            )
            t.start()
            ctx.client_threads.append(t)
    finally:
        # Stop kontroleru (bezpečné zastavení robota)
        try:
            stop_controller(ctx)  # pošle 0 PWM a ukončí smyčku (implementujeme v control.py)
        except Exception as e:
            log(f"⚠️ stop_controller selhal: {e}")

        # zavřít server socket
        try:
            srv.close()
        except Exception:
            pass

        # počkat na klienty
        for t in ctx.client_threads:
            t.join(timeout=1.0)

        log("🛑 Autopilot ukoncen.")


if __name__ == "__main__":
    try:
        run_server()
    except Exception as e:
        print(f"Fatální chyba: {e}", file=sys.stderr)
        sys.exit(1)
