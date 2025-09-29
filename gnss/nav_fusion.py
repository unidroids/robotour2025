from __future__ import annotations
import time
import threading
from collections import deque
from typing import Optional, Callable, Deque, List

from data.esf_raw_data import EsfRawData
from data.nav_pvat_data import NavPvatData
from data.nav_fusion_data import NavFusionData

from gyro_smoother import GyroRateSmoother
from lever_arm_heading import LeverArmHeading


class FusionResult:
    def __init__(
        self,
        t_mono: float,
        iTOW_ms: int,
        heading_deg: float,
        speed_mps: float,
        quality: str,
    ):
        self.t_mono = t_mono
        self.iTOW_ms = iTOW_ms
        self.heading_deg = heading_deg
        self.speed_mps = speed_mps
        self.quality = quality

    def __repr__(self):
        return (
            f"<FusionResult t={self.t_mono:.3f} iTOW={self.iTOW_ms} "
            f"heading={self.heading_deg:.2f}° speed={self.speed_mps:.3f}m/s quality={self.quality}>"
        )

# --- Hlavní engine ----------------------------------------------------------

class NavFusion:
    """
    Fúzní engine:
      - on_esf_raw(EsfRawData): ukládá GyroZ přes smoother
      - on_nav_pvat(NavPvatData): použije vyhlazený GyroZ, spočte heading (lever arm), uloží výsledky
    """

    def __init__(self):
        self._gyro_smoother = GyroRateSmoother()
        self._latest: Optional[NavFusionData] = None
        self._latest_lock = threading.Lock()
        self._cond = threading.Condition()
        self._lever_arm = LeverArmHeading(r_x=0.3, r_y=0.03) # [m]
        self._last_gyroZ_value = 0.0 

    # === Vstup z ESF-RAW handleru ============================================

    def on_esf_raw(self, raw: EsfRawData) -> None:
        self._last_gyroZ_value = raw.gyroZ
        self._gyro_smoother.update(raw.gyroZ)


    # === Vstup z NAV-PVAT handleru ===========================================

    def on_nav_pvat(self, pvat: NavPvatData) -> None:
        now = time.monotonic()
        smoothed_gyroZ = self._gyro_smoother.last

        if pvat.fixType in (4, 5):  # dead reckoning
            heading_deg = pvat.vehHeading
            speed = pvat.gSpeed
        elif self._lever_arm:
            heading_deg, speed = self._lever_arm.theta_from_alpha_speed_deg(
                alpha_deg=pvat.motHeading,
                speed=pvat.gSpeed,
                omega_deg=smoothed_gyroZ
            )
        else:
            heading_deg = pvat.vehHeading
            speed = pvat.gSpeed

        # Zde nastav další pole podle dostupných dat
        fusion_data = NavFusionData(
            ts_mono=now,
            lat=pvat.lat,
            lon=pvat.lon,
            hAcc=pvat.hAcc,
            heading=heading_deg,
            headingAcc=pvat.accHeading,
            speed=speed,
            sAcc=pvat.sAcc,
            gyroZ=smoothed_gyroZ,
            gyroZAcc=2.0,  # odhad chyby gyroZ
            gnssFixOK=int(pvat.carrSoln in (2, 3)),  # fix s GNSS
            drUsed=int(pvat.fixType in (4, 5)),  # dead reckoning
            vehHeading=pvat.vehHeading,
            motHeading=pvat.motHeading,
            lastGyroZ=self._last_gyroZ_value,
            gSpeed=pvat.gSpeed,
        )
        self._publish(fusion_data)

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
