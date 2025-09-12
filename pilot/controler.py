# autopilot/control.py
import math
import threading
import time
import socket
from typing import Optional, Tuple

from main import (
    AutopilotContext,
    STATUS_IDLE,
    STATUS_RUNNING,
    STATUS_REACHED,
    STATUS_ERROR,
)

GNSS_PORT = 9006   # GNSS služba
DRIVE_PORT = 9003  # Drive služba (PWM)

# ── Pomocné funkce ────────────────────────────────────────────────
def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = phi2 - phi1
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    brng = math.degrees(math.atan2(x, y))
    return (brng + 360) % 360

def angle_diff(a: float, b: float) -> float:
    """Vrátí rozdíl úhlů (a-b) v rozsahu -180..180."""
    d = (a - b + 180) % 360 - 180
    return d

def send_command(port: int, cmd: str, timeout: float = 1.0) -> Optional[str]:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
            s.sendall((cmd + "\n").encode())
            data = s.recv(4096)
        return data.decode(errors="ignore").strip()
    except Exception as e:
        print(f"⚠️ Nelze poslat '{cmd}' na port {port}: {e}")
        return None

# ── Kontroler ─────────────────────────────────────────────────────
class AutopilotController:
    def __init__(self, ctx: AutopilotContext):
        self.ctx = ctx
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        with self.ctx.lock:
            self.ctx.status = STATUS_RUNNING

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        with self.ctx.lock:
            self.ctx.status = STATUS_IDLE
        # STOP robota
        send_command(DRIVE_PORT, "STOP")

    def _loop(self):
        while not self.stop_event.is_set():
            pose = self._get_gnss_pose()
            if not pose:
                # GNSS nedostupné -> STOP
                with self.ctx.lock:
                    self.ctx.status = STATUS_ERROR
                send_command(DRIVE_PORT, "STOP")
                time.sleep(0.5)
                continue

            lat, lon, heading, speed, hacc = pose
            with self.ctx.lock:
                self.ctx.last_pose = (lat, lon)

                if self.ctx.waypoint:
                    wlat, wlon, radius = self.ctx.waypoint
                    dist = haversine_distance(lat, lon, wlat, wlon)
                    brg = bearing(lat, lon, wlat, wlon)
                    diff = angle_diff(brg, heading)

                    if dist <= radius:
                        self.ctx.status = STATUS_REACHED
                        self.stop()
                        return
                    else:
                        self.ctx.status = STATUS_RUNNING
                        self._send_drive_command(dist, diff)

            time.sleep(0.5)

    def _get_gnss_pose(self) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Vrátí (lat, lon, heading, speed, hAcc) z GNSS služby.
        Očekává formát: lat lon alt heading speed hAcc ts
        """
        resp = send_command(GNSS_PORT, "GET")
        if not resp:
            return None
        try:
            parts = resp.split()
            lat = float(parts[0])
            lon = float(parts[1])
            heading = float(parts[3]) if len(parts) > 3 else 0.0
            speed = float(parts[4]) if len(parts) > 4 else 0.0
            hacc = float(parts[5]) if len(parts) > 5 else 999.0
            return (lat, lon, heading, speed, hacc)
        except Exception as e:
            print(f"⚠️ GNSS parsování selhalo: {resp} ({e})")
            return None

    def _send_drive_command(self, dist: float, angle_err: float, speed: float = 0.0):
        """
        Pošle příkaz do služby drive.
        - dist: vzdálenost k cíli
        - angle_err: odchylka od směru (°), -180..180
        - speed: aktuální rychlost z GNSS (m/s)
        """
        # Pokud stojíme → otáčení na místě
        if speed < 0.1:  
            if angle_err > 10:   # potřebujeme otočit vpravo
                left, right = 20, -20
            elif angle_err < -10:  # otočit vlevo
                left, right = -20, 20
            else:
                left, right = 0, 0
        else:
            # Dopředný pohyb s korekcí
            base_pwm = 80
            turn = max(min(int(angle_err / 5), 20), -20)  # škálování úhlu na ±20

            left = base_pwm - turn
            right = base_pwm + turn

            # Žádné couvání v běžné jízdě
            left = max(left, 0)
            right = max(right, 0)

            # saturace na 100
            left = min(left, 100)
            right = min(right, 100)

        cmd = f"PWM {left} {right}"
        send_command(DRIVE_PORT, cmd)
        print(f"➡️ drive: {cmd} (dist={dist:.1f}m, err={angle_err:.1f}°, speed={speed:.2f}m/s)")
        

# ── API pro main.py ───────────────────────────────────────────────
def start_controller(ctx: AutopilotContext):
    ctl = AutopilotController(ctx)
    ctx.controller_thread = ctl
    ctl.start()

def stop_controller(ctx: AutopilotContext):
    ctl: AutopilotController = ctx.controller_thread  # type: ignore
    if ctl:
        ctl.stop()
