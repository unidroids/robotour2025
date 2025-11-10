from __future__ import annotations
import csv
import math
import os
import time
import threading
from collections import deque
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Optional, Deque, Iterable

# --- Optional project dataclasses -------------------------------------------
# If your project provides these (data.nav_pvat_data / data.nav_fusion_data),
# those will be used instead of the local fallbacks below.
try:
    from data.nav_pvat_data import NavPvatData  # type: ignore
    from data.nav_fusion_data import NavFusionData  # type: ignore
except Exception:  # pragma: no cover - used for standalone testing
    @dataclass
    class NavPvatData:  # minimal fallback for tests
        lat: float = 0.0
        lon: float = 0.0
        hAcc: float = 0.0
        accHeading: float = 10.0  # [deg]
        gSpeed: float = 0.0       # [mm/s]
        sAcc: float = 0.0
        carrSoln: int = 0         # 0=none,1=float,2=fixed, 3=map-matching (optional)
        fixType: int = 0          # 0..5 per UBX
        motHeading: float = 0.0   # [deg cw from North]
        heading: Optional[float] = None  # [deg cw], if available: vehicle attitude heading
        vehHeading: Optional[float] = None  # alias if present in your feed

    @dataclass
    class NavFusionData:  # minimal fallback for tests / logging example
        ts_mono: float
        lat: float
        lon: float
        hAcc: float
        heading: float
        headingAcc: float
        speed: float
        sAcc: float
        gyroZ: float
        gyroZAcc: float
        gnssFixOK: int
        drUsed: int
        vehHeading: float
        motHeading: float
        lastGyroZ: float
        gSpeed: float


# --- Helpers ----------------------------------------------------------------

def _wrap_deg_180(x: float) -> float:
    """Wrap angle in degrees into (-180, 180] range."""
    x = (x + 180.0) % 360.0
    if x <= 0:
        x += 360.0
    return x - 180.0


def _wrap_deg_360(x: float) -> float:
    """Wrap angle in degrees into [0, 360)."""
    return x % 360.0


def _cw_to_ccw_deg(heading_cw_deg: float) -> float:
    """Convert navigation heading (0=N, cw positive) to ccw-positive convention."""
    return (360.0 - heading_cw_deg) % 360.0


def _ccw_to_cw_deg(angle_ccw_deg: float) -> float:
    return (360.0 - angle_ccw_deg) % 360.0


def _circ_mean_deg(samples_deg: Iterable[float]) -> float:
    """Circular mean of angles in degrees (returns angle in degrees in (-180, 180])."""
    s = 0.0
    c = 0.0
    n = 0
    for a in samples_deg:
        rad = math.radians(a)
        s += math.sin(rad)
        c += math.cos(rad)
        n += 1
    if n == 0:
        return 0.0
    return math.degrees(math.atan2(s / n, c / n))


# --- Main fusion engine ------------------------------------------------------

class NavFusion:
    """
    Fúzní engine pro směrování (heading) robota.

    Vstupy (10 Hz):
      - IMU: úhlová rychlost (omega) [raw] a relativní úhel robota [raw]
      - ODOM: rychlosti levého/pravého kola [mm/s]
      - GNSS PVAT: obsahuje (motHeading nebo vehHeading/heading), rychlost, kvality

    Geometrie:
      - ARP (anténa) v robotím rámci (x dopředu, y vlevo) v metrech
      - Rozchod kol B v metrech

    Pozn.: IMU vstupy jsou převáděny dle zadaných konstant ve `on_imu_data`.
    """

    # převodní konstanty pro vaše IMU (ponechejte/změňte dle vašich dat)
    _IMU_OMEGA_DIV = 13106.8      # -> deg/s (max 500 dps), ccw+
    _IMU_ANGLE_DIV = 3355340.8    # -> deg (-360..360), ccw+

    def __init__(self, arp_x: float = 0.30, arp_y: float = 0.03, B: float = 0.58,
                 log_root: str = "/data/robot/fusion"):
        # Geometrie (m)
        self.arp_x = float(arp_x)
        self.arp_y = float(arp_y)
        self.B = float(B)

        # Stav
        self._latest: Optional[NavFusionData] = None
        self._latest_lock = threading.Lock()
        self._cond = threading.Condition()

        # IMU
        self._imu_mono = 0
        self._imu_omega = 0.0      # deg/s (ccw+)
        self._imu_angle = 0.0      # deg (ccw, wrapped)
        self._last_gyroZ = 0.0

        # ODO
        self._odo_mono = 0
        self._odo_left_speed = 0.0   # mm/s
        self._odo_right_speed = 0.0  # mm/s

        # GNSS<->IMU bias (globální - relativní) v deg (ccw)
        self._bias_window: Deque[float] = deque(maxlen=30)
        self._angle_bias = 0.0
        self._bias_initialized = False

        # Odvozené
        self._tangent_offset_deg = 0.0  # odchylka pohybové tečny v ARP vůči ose robota (ccw+)

        # Logging
        self._log_root = log_root
        self._log_path, self._csv, self._csv_file = self._init_logger()

    # ---- Alternativní konstruktor z centimetrů -----------------------------
    @classmethod
    def from_cm(cls, arp_x_cm: float, arp_y_cm: float, B_cm: float, **kw) -> "NavFusion":
        return cls(arp_x=arp_x_cm / 100.0, arp_y=arp_y_cm / 100.0, B=B_cm / 100.0, **kw)

    # === Vstup z IMU handleru ===============================================
    def on_imu_data(self, mono_ms: int, omega_raw: float, angle_raw: float) -> None:
        """Přijme IMU data a převede je na deg/s a deg (ccw+)."""
        self._last_gyroZ = self._imu_omega
        self._imu_mono = int(mono_ms)
        self._imu_omega = float(omega_raw) / self._IMU_OMEGA_DIV
        self._imu_angle = _wrap_deg_180(float(angle_raw) / self._IMU_ANGLE_DIV)

    # === Vstup z ODOM handleru ==============================================
    def on_odm_data(self, mono_ms: int, left_speed_mm_s: float, right_speed_mm_s: float) -> None:
        self._odo_mono = int(mono_ms)
        self._odo_left_speed = float(left_speed_mm_s)
        self._odo_right_speed = float(right_speed_mm_s)

    # === Vstup z NAV-PVAT handleru ==========================================
    def on_nav_pvat(self, pvat: NavPvatData) -> None:
        now = time.monotonic()

        # 1) Tečná odchylka v místě antény dle odometrie + geometrie (ccw+)
        tangent_offset_deg = self._compute_tangent_offset_deg(
            self._odo_left_speed, self._odo_right_speed
        )
        self._tangent_offset_deg = tangent_offset_deg

        # 2) Vyber GNSS heading robota (ccw) — preferuj vehHeading/heading, jinak koriguj motHeading
        gnss_robot_ccw = self._select_gnss_robot_heading_ccw(pvat, tangent_offset_deg)

        # 3) Rozdíl GNSS vs IMU (ccw) a okno pro průměr
        imu_ccw = self._imu_angle
        delta_ccw = _wrap_deg_180(gnss_robot_ccw - imu_ccw)
        self._bias_window.append(delta_ccw)

        # 4) Vyhodnocení kvality GNSS a aktualizace biasu
        quality = self._compute_quality(pvat)
        if not self._bias_initialized and len(self._bias_window) >= 5:
            self._angle_bias = _circ_mean_deg(self._bias_window)
            self._bias_initialized = True
        else:
            if quality > 0.7 and len(self._bias_window) >= 1:
                new_bias = _circ_mean_deg(self._bias_window)
                # Exponenciální filtrace: 0.8 stará, 0.2 nová
                blended = _wrap_deg_180(self._angle_bias + 0.2 * _wrap_deg_180(new_bias - self._angle_bias))
                self._angle_bias = blended

        # 5) Finální fúzovaný heading (ccw), převedený na cw (0=N, 90=E)
        fused_ccw = _wrap_deg_360(imu_ccw + self._angle_bias)
        fused_cw = _ccw_to_cw_deg(fused_ccw)

        # 6) Rychlosti
        v_center_mm_s = 0.5 * (self._odo_left_speed + self._odo_right_speed)

        # 7) Publish + log
        fusion_data = NavFusionData(
            ts_mono=now,
            lat=getattr(pvat, "lat", 0.0),
            lon=getattr(pvat, "lon", 0.0),
            hAcc=getattr(pvat, "hAcc", 0.0),
            heading=fused_cw,
            headingAcc=getattr(pvat, "accHeading", 10.0),
            speed=v_center_mm_s,             # [mm/s] rychlost těžiště (pro kompatibilitu)
            sAcc=getattr(pvat, "sAcc", 0.0),
            gyroZ=self._imu_omega,
            gyroZAcc=2.0,                    # Odhad chyby gyroZ [deg/s]
            gnssFixOK=int(getattr(pvat, "carrSoln", 0) in (2, 3)),
            drUsed=int(getattr(pvat, "fixType", 0) in (4, 5)),
            vehHeading=self._imu_angle,
            motHeading=getattr(pvat, "motHeading", 0.0),
            lastGyroZ=self._last_gyroZ,
            gSpeed=getattr(pvat, "gSpeed", 0.0),
        )

        self._publish(fusion_data)
        self._log_row(now, pvat, fused_cw, fused_ccw, imu_ccw, gnss_robot_ccw,
                      delta_ccw, self._angle_bias, quality, tangent_offset_deg,
                      v_center_mm_s)

    # === Odběratelské API ====================================================

    def get_latest(self) -> Optional[NavFusionData]:
        with self._latest_lock:
            return self._latest

    def wait_for_update(self, timeout: Optional[float] = None) -> bool:
        with self._cond:
            return self._cond.wait(timeout=timeout)

    # === Interní =============================================================

    def _publish(self, res: NavFusionData) -> None:
        with self._latest_lock:
            self._latest = res
        with self._cond:
            self._cond.notify_all()

    # --- GNSS quality heuristic (0..1) --------------------------------------
    def _compute_quality(self, pvat: NavPvatData) -> float:
        # headingAcc [deg] -> 1.0 is best (0 deg), ~0 at >=15 deg
        acc = float(getattr(pvat, "accHeading", 15.0))
        q_acc = max(0.0, min(1.0, 1.0 - (acc / 15.0)))

        # speed reliability: motion heading je spolehlivý až od ~0.5 m/s
        gspeed_mm_s = float(getattr(pvat, "gSpeed", 0.0))
        gspeed_m_s = gspeed_mm_s / 1000.0
        q_spd = max(0.0, min(1.0, (gspeed_m_s - 0.1) / 0.4))  # 0 @0.1, 1 @0.5

        carr = int(getattr(pvat, "carrSoln", 0))
        fix = int(getattr(pvat, "fixType", 0))
        q_fix = 1.0 if carr in (2, 3) else (0.7 if carr == 1 else (0.5 if fix >= 3 else 0.0))

        quality = 0.5 * q_acc + 0.3 * q_spd + 0.2 * q_fix
        return max(0.0, min(1.0, quality))

    # --- Tangent offset at antenna ------------------------------------------
    def _compute_tangent_offset_deg(self, v_l_mm_s: float, v_r_mm_s: float) -> float:
        """Angle (ccw+) between robot forward axis and instantaneous velocity vector
        at the antenna point (ARP). Uses differential drive kinematics.
        """
        v_l = v_l_mm_s / 1000.0  # m/s
        v_r = v_r_mm_s / 1000.0  # m/s
        v_c = 0.5 * (v_r + v_l)  # center linear velocity (m/s)
        # angular velocity ccw+, rad/s
        if self.B == 0:
            omega = 0.0
        else:
            omega = (v_r - v_l) / self.B
        # velocity at point r = (x, y)
        vx = v_c - omega * self.arp_y
        vy = omega * self.arp_x
        angle = math.degrees(math.atan2(vy, vx))  # ccw+
        return _wrap_deg_180(angle)

    # --- Select GNSS robot heading in CCW ------------------------------------
    def _select_gnss_robot_heading_ccw(self, pvat: NavPvatData, tangent_offset_ccw: float) -> float:
        # Prefer explicit vehicle attitude heading if available
        # veh_head_cw = None
        # for name in ("vehHeading", "heading"):
        #     if hasattr(pvat, name):
        #         val = getattr(pvat, name)
        #         if val is not None:
        #             veh_head_cw = float(val)
        #             break
        # if veh_head_cw is not None:
        #     return _cw_to_ccw_deg(veh_head_cw)

        # Fall back to motion heading corrected by tangent offset at ARP
        mot_head_cw = float(getattr(pvat, "motHeading", 0.0))
        mot_ccw = _cw_to_ccw_deg(mot_head_cw)
        robot_ccw = _wrap_deg_360(mot_ccw - tangent_offset_ccw)
        return robot_ccw

    # --- Logger --------------------------------------------------------------
    def _init_logger(self):
        day = time.strftime("%Y-%m-%d", time.localtime())
        ts = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        day_dir = os.path.join(self._log_root, day)
        os.makedirs(day_dir, exist_ok=True)
        path = os.path.join(day_dir, f"fusio{ts}.csv")
        f = open(path, "w", newline="")
        writer = csv.writer(f)
        writer.writerow([
            "ts_mono",
            # inputs
            "imu_mono_ms", "imu_omega_deg_s", "imu_angle_ccw_deg",
            "odo_mono_ms", "odo_left_mm_s", "odo_right_mm_s",
            "pvat_motHeading_cw", "pvat_vehHeading_cw", "pvat_headingAcc_deg",
            "pvat_gSpeed_mm_s", "pvat_sAcc", "pvat_carrSoln", "pvat_fixType",
            # derived
            "tangent_offset_ccw_deg", "gnss_robot_ccw_deg", "delta_ccw_deg",
            "bias_ccw_deg", "quality_0_1",
            # outputs
            "fused_heading_cw_deg", "fused_heading_ccw_deg", "center_speed_mm_s",
        ])
        f.flush()
        return path, writer, f

    def _log_row(self, ts_mono: float, pvat: NavPvatData,
                 fused_cw_deg: float, fused_ccw_deg: float,
                 imu_ccw_deg: float, gnss_robot_ccw_deg: float,
                 delta_ccw_deg: float, bias_ccw_deg: float, quality: float,
                 tangent_ccw_deg: float, center_speed_mm_s: float) -> None:
        self._csv.writerow([
            f"{ts_mono:.3f}",
            self._imu_mono, f"{self._imu_omega:.3f}", f"{imu_ccw_deg:.3f}",
            self._odo_mono, f"{self._odo_left_speed:.1f}", f"{self._odo_right_speed:.1f}",
            f"{getattr(pvat, 'motHeading', 0.0):.3f}",
            f"{getattr(pvat, 'vehHeading', getattr(pvat, 'heading', float('nan')))}",
            f"{getattr(pvat, 'accHeading', float('nan'))}",
            f"{getattr(pvat, 'gSpeed', float('nan'))}",
            f"{getattr(pvat, 'sAcc', float('nan'))}",
            f"{getattr(pvat, 'carrSoln', float('nan'))}",
            f"{getattr(pvat, 'fixType', float('nan'))}",
            f"{tangent_ccw_deg:.3f}", f"{gnss_robot_ccw_deg:.3f}", f"{delta_ccw_deg:.3f}",
            f"{bias_ccw_deg:.3f}", f"{quality:.3f}",
            f"{fused_cw_deg:.3f}", f"{fused_ccw_deg:.3f}", f"{center_speed_mm_s:.1f}",
        ])
        self._csv_file.flush()


# === Jednoduchý test (__main__) =============================================
if __name__ == "__main__":
    print("Running standalone demo @10 Hz for ~8 s …")

    # Geometrie: ARP 30 cm vepředu, 3 cm vlevo; rozchod 58 cm
    fusion = NavFusion.from_cm(arp_x_cm=30, arp_y_cm=3, B_cm=58)

    # Simulační stav
    sim_dt = 0.1  # 10 Hz
    sim_t = 0.0
    imu_bias_ccw = -25.0  # IMU má -25 deg ofset vůči severu (nutné odhadnout)
    robot_yaw_ccw = 80.0  # globální směrování robota na začátku (ccw, 0=N)

    def inv_angle_raw(deg: float) -> float:
        return deg * fusion._IMU_ANGLE_DIV

    def inv_omega_raw(deg_s: float) -> float:
        return deg_s * fusion._IMU_OMEGA_DIV

    # Scénář rychlostí kol (mm/s)
    phases = [
        (20, 500.0, 500.0),   # rovně 2 s
        (20, 400.0, 600.0),   # mírná levá 2 s
        (20, 300.0, 700.0),   # ostřejší levá 2 s
        (20, 500.0, 500.0),   # rovně 2 s
    ]

    mono_ms = 0
    for steps, vl, vr in phases:
        for _ in range(steps):
            mono_ms += int(sim_dt * 1000)

            # ODO
            fusion.on_odm_data(mono_ms, vl, vr)
            v_c_m_s = 0.5 * (vl + vr) / 1000.0
            omega_rad_s = (vr - vl) / 1000.0 / fusion.B
            robot_yaw_ccw = _wrap_deg_360(robot_yaw_ccw + math.degrees(omega_rad_s * sim_dt))

            # IMU (má bias vůči globálu)
            imu_yaw_ccw = _wrap_deg_180(robot_yaw_ccw + imu_bias_ccw)
            imu_omega_deg_s = math.degrees(omega_rad_s)
            fusion.on_imu_data(mono_ms, inv_omega_raw(imu_omega_deg_s), inv_angle_raw(imu_yaw_ccw))

            # GNSS (motHeading + šum) -> cw [0=N], korigovatelné tečnou odchylkou
            # Z motHeading (směr pohybu antény): přičti tečnou odchylku
            tangent = fusion._compute_tangent_offset_deg(vl, vr)
            mot_ccw = _wrap_deg_360(robot_yaw_ccw + tangent)
            mot_cw = _ccw_to_cw_deg(mot_ccw)
            noise = (math.sin(sim_t * 0.7) * 1.5)  # ~±1.5° šum

            pvat = NavPvatData(
                lat=0.0, lon=0.0, hAcc=0.02,
                accHeading=2.0,          # 2°
                gSpeed=(vl + vr) * 0.5,  # mm/s
                sAcc=0.02,
                carrSoln=2,
                fixType=5,
                motHeading=mot_cw + noise,
                heading=None,            # žádný vehHeading v této simulaci
                vehHeading=None,
            )

            fusion.on_nav_pvat(pvat)

            sim_t += sim_dt
            time.sleep(sim_dt)

    latest = fusion.get_latest()
    print(f"\nLog file: {fusion._log_path}")
    if latest:
        print(f"Fused heading (cw): {latest.heading:.2f}°  | IMU angle (ccw): {latest.vehHeading:.2f}°")
        print(f"gSpeed: {latest.gSpeed:.1f} mm/s | gyroZ: {latest.gyroZ:.2f} deg/s")
    print("Done.")
