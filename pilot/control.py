#!/usr/bin/env python3
import math
import threading
import time
import socket
import json
from typing import Optional, Tuple, Any

# ── Stavové hodnoty ───────────────────────────────────────────────
STATUS_IDLE = "IDLE"
STATUS_RUNNING = "RUNNING"
STATUS_REACHED = "REACHED"
STATUS_ERROR = "ERROR"

# ── Porty jiných služeb ───────────────────────────────────────────
GNSS_PORT = 9006
DRIVE_PORT = 9003

# ── OSTRÝ REŽIM (DRY_RUN = False) ─────────────────────────────────
DRY_RUN = False  # <— jede se “naostro” (PWM se posílá do DRIVE)

# ── Tuning / limity ───────────────────────────────────────────────
MAX_PWM = 40                 # absolutní strop pro testování
BASE_PWM_FORWARD = 12        # základní dopředná síla (opatrná)
TURN_MAX = 20                # max korekce řízení (±)
TARGET_SPEED = 0.30          # m/s (cílová rychlost)
SPEED_BRAKE = 0.35           # m/s (tvrdá brzda: 0,0)
ROTATE_SPEED_THRESH = 0.03   # m/s (pod tím točíme na místě)
ANGLE_DEAD_BAND = 10         # ° (mrtvé pásmo při točení na místě)
PWM_RAMP_STEP = 6            # max změna PWM / cyklus
NEAR_RADIUS_SLOWDOWN = 0.8   # m: “plížení” blízko cíle (nižší base pwm)

# EMA filtr pro vzdálenost (jen pro debug vyhlazení)
DIST_EMA_ALPHA = 0.3

# ── Utility ───────────────────────────────────────────────────────
def clamp(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v

def ramp(current: int, target: int, step: int) -> int:
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target

R_EARTH = 6371000.0

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = phi2 - phi1
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R_EARTH * c

def equirectangular_distance_bearing(lat1: float, lon1: float, lat2: float, lon2: float):
    """
    Stabilní na malé vzdálenosti:
      dx ~ R * dLon * cos(lat0)
      dy ~ R * dLat
      bearing = atan2(dx, dy) od severu (CW), v °.
    """
    lat0 = math.radians((lat1 + lat2) / 2.0)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    dx = R_EARTH * dlon * math.cos(lat0)
    dy = R_EARTH * dlat
    dist = math.hypot(dx, dy)
    brg = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
    return dist, brg, dx, dy

def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    brng = math.degrees(math.atan2(x, y))
    return (brng + 360) % 360

def angle_diff(target_deg: float, current_deg: float) -> float:
    """(target - current) v rozsahu -180..180."""
    return (target_deg - current_deg + 180) % 360 - 180

def send_command(port: int, cmd: str, timeout: float = 1.0) -> Optional[str]:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
            s.sendall((cmd + "\n").encode())
            data = s.recv(4096)
        return data.decode(errors="ignore").strip()
    except Exception as e:
        print(f"⚠️ Nelze poslat '{cmd}' na port {port}: {e}")
        return None

def normalize_hacc_meters(hacc_value: float) -> float:
    """
    GNSS posílá hAcc často v milimetrech (např. 42 => 0.042 m).
    Pokud číslo vypadá jako mm (>=10), převedeme mm->m.
    """
    return hacc_value / 1000.0 if hacc_value >= 10.0 else hacc_value

# ── Kontroler ─────────────────────────────────────────────────────
class AutopilotController:
    """
    Ostrá verze:
      - GNSS -> lat/lon/heading/speed/hAcc
      - equirectangular + haversine pro cross-check
      - řízení: BASE_PWM_FORWARD + korekce ±TURN_MAX; bez couvání při jízdě
      - točení na místě při malé rychlosti; bezpečný STOP = PWM 0 0
      - guvernér rychlosti (TARGET_SPEED), brzda nad SPEED_BRAKE
      - rampování PWM
      - log: záměr (“INTENT”), dist, bearing, radius
    """
    def __init__(self, ctx: Any):
        self.ctx = ctx
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.prev_left = 0
        self.prev_right = 0
        self.dist_ema: Optional[float] = None
        self._last_heading = None
        self._last_time = None

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
        # Bezpečné zastavení
        send_command(DRIVE_PORT, "PWM 0 0")
        with self.ctx.lock:
            if self.ctx.status not in (STATUS_REACHED, STATUS_ERROR):
                self.ctx.status = STATUS_IDLE

    def _loop(self):
        while not self.stop_event.is_set():
            pose = self._get_gnss_pose()
            if not pose:
                with self.ctx.lock:
                    self.ctx.status = STATUS_ERROR
                send_command(DRIVE_PORT, "PWM 0 0")
                time.sleep(0.5)
                continue

            lat, lon, heading_deg, speed_mps, hacc_m = pose
            with self.ctx.lock:
                self.ctx.last_pose = (lat, lon)
                wp = self.ctx.waypoint

            if wp:
                wlat, wlon, radius = wp

                # 1) výpočet vzdálenosti/bearingu dvěma způsoby
                dist_hav = haversine_distance(lat, lon, wlat, wlon)
                dist_eq, brg_eq, dx, dy = equirectangular_distance_bearing(lat, lon, wlat, wlon)
                brg_hav = bearing(lat, lon, wlat, wlon)

                # 2) EMA
                dist_raw = dist_eq
                if self.dist_ema is None:
                    self.dist_ema = dist_raw
                else:
                    self.dist_ema = (1 - DIST_EMA_ALPHA) * self.dist_ema + DIST_EMA_ALPHA * dist_raw

                # 3) úhlová chyba proti GNSS headingu
                err_eq = angle_diff(brg_eq, heading_deg)
                err_hav = angle_diff(brg_hav, heading_deg)

                # 4) diagnostika změny headingu (nepoužitá pro řízení, jen info)
                now = time.time()
                if self._last_heading is not None and self._last_time is not None:
                    dt = max(now - self._last_time, 1e-3)
                    _ = angle_diff(heading_deg, self._last_heading) / dt  # °/s
                self._last_heading = heading_deg
                self._last_time = now

                # 5) řízení (zohlední zpomalení blízko cíle)
                left, right, rotate_only = self._compute_pwm(dist_raw, err_eq, speed_mps, radius)

                # 6) log (včetně radius)
                print(
                    "INTENT"
                    f" dist_eq={dist_eq:.2f}m dist_hav={dist_hav:.2f}m dist_ema={self.dist_ema:.2f}m"
                    f" brg_eq={brg_eq:.1f}° brg_hav={brg_hav:.1f}°"
                    f" head={heading_deg:.1f}° err_eq={err_eq:.1f}° err_hav={err_hav:.1f}°"
                    f" speed={speed_mps:.2f}m/s hAcc={hacc_m:.3f}m"
                    f" dx={dx:.2f} dy={dy:.2f} radius={radius:.2f}m"
                    f" rotate_only={rotate_only} PWM=({left},{right})"
                )

                # 7) dosažení cíle
                if dist_raw <= radius:
                    with self.ctx.lock:
                        self.ctx.status = STATUS_REACHED
                    send_command(DRIVE_PORT, "PWM 0 0")
                    print("✅ REACHED – cílový poloměr dosažen, zastavuji")
                    self.stop_event.set()
                    return

                # 8) aktace (pokud nejsme v DRY_RUN)
                with self.ctx.lock:
                    self.ctx.status = STATUS_RUNNING
                if not DRY_RUN:
                    self._actuate(left, right)

            else:
                # bez cíle – stát
                send_command(DRIVE_PORT, "PWM 0 0")
                print("INTENT bez waypointu – stát")

            time.sleep(0.5)

    def _get_gnss_pose(self) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Vrátí (lat, lon, heading, speed, hAcc[m]) z GNSS služby.
        Podporuje:
          - JSON: {"lat","lon","heading","speed","hAcc",...} (hAcc očekávám v mm -> převod!)
          - text: lat lon alt heading speed hAcc vAcc numSV fixType time
        """
        resp = send_command(GNSS_PORT, "GET")
        if not resp:
            return None

        # JSON
        if resp.startswith("{") or '"lat"' in resp:
            try:
                j = json.loads(resp)
                hacc_m = normalize_hacc_meters(float(j.get("hAcc", 999.0)))
                return (
                    float(j["lat"]),
                    float(j["lon"]),
                    float(j.get("heading", 0.0)),
                    float(j.get("speed", 0.0)),
                    hacc_m,
                )
            except Exception as e:
                print(f"⚠️ GNSS JSON parse fail: {e} | {resp}")

        # Text
        try:
            parts = resp.split()
            lat = float(parts[0]); lon = float(parts[1])
            heading = float(parts[3]) if len(parts) > 3 else 0.0
            speed = float(parts[4]) if len(parts) > 4 else 0.0
            hacc_raw = float(parts[5]) if len(parts) > 5 else 999.0
            hacc_m = normalize_hacc_meters(hacc_raw)
            return (lat, lon, heading, speed, hacc_m)
        except Exception as e:
            print(f"⚠️ GNSS text parse fail: {e} | {resp}")
            return None

    def _compute_pwm(self, dist: float, angle_err_deg: float, speed: float, radius: float):
        """
        Vrátí (left,right,rotate_only) – ostrý záměr pro DRIVE.
        - točení na místě, pokud rychlost < ROTATE_SPEED_THRESH
        - dopředná jízda bez couvání, korekce ±TURN_MAX
        - zpomalení, když jsme blízko cíle (dist < NEAR_RADIUS_SLOWDOWN)
        - guvernér rychlosti (TARGET_SPEED), brzda při SPEED_BRAKE
        - rampování
        """
        # bezpečnostní brzda
        if speed > SPEED_BRAKE:
            target_left, target_right = 0, 0
            rotate_only = False
        else:
            rotate_only = (speed < ROTATE_SPEED_THRESH)
            if rotate_only:
                if angle_err_deg > ANGLE_DEAD_BAND:
                    target_left, target_right =  TURN_MAX, -TURN_MAX
                elif angle_err_deg < -ANGLE_DEAD_BAND:
                    target_left, target_right = -TURN_MAX,  TURN_MAX
                else:
                    target_left, target_right = 0, 0
            else:
                # blízko cíle -> ještě opatrnější base
                base = BASE_PWM_FORWARD if dist > NEAR_RADIUS_SLOWDOWN else max(8, BASE_PWM_FORWARD - 4)
                turn = clamp(int(angle_err_deg / 5), -TURN_MAX, TURN_MAX)
                target_left  = base - turn
                target_right = base + turn

                # žádné couvání v běžné jízdě
                target_left  = max(target_left, 0)
                target_right = max(target_right, 0)

                # saturace
                target_left  = min(target_left, MAX_PWM)
                target_right = min(target_right, MAX_PWM)

                # guvernér rychlosti – když GNSS rychlost přesahuje target, škáluj dolů
                if speed > TARGET_SPEED:
                    factor = TARGET_SPEED / max(speed, 1e-3)  # <= 1.0
                    target_left  = int(target_left  * factor)
                    target_right = int(target_right * factor)

        # rampování
        left  = ramp(self.prev_left,  int(target_left),  PWM_RAMP_STEP)
        right = ramp(self.prev_right, int(target_right), PWM_RAMP_STEP)
        self.prev_left, self.prev_right = left, right

        return left, right, rotate_only

    def _actuate(self, left: int, right: int):
        cmd = f"PWM {left} {right}"
        send_command(DRIVE_PORT, cmd)
        # stručný log aktuace (INTENT už obsahuje všechny metriky)
        print(f"➡️ DRIVE: {cmd}")

# ── API pro main.py ───────────────────────────────────────────────
def start_controller(ctx: Any):
    ctl = getattr(ctx, "controller_thread", None)
    t = getattr(ctl, "thread", None)
    if isinstance(ctl, AutopilotController) and getattr(t, "is_alive", lambda: False)():
        return
    ctl = AutopilotController(ctx)
    ctx.controller_thread = ctl
    ctl.start()

def stop_controller(ctx: Any):
    ctl = getattr(ctx, "controller_thread", None)
    if isinstance(ctl, AutopilotController):
        ctl.stop()
