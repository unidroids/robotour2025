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
        self._lever_arm = None # LeverArmHeading(r_x=0.3, r_y=0.03, speed_eps=0.03, omega_eps=0.3) # [m]
        
        self._last_gyroZ_value = 0.0 
        self._gyro_last_sTag:Optional[int] = None
        self._gyroZ_calibration_offset = 0.0
        self._cumulated_angleZ = 0.0
        self._heading_acc = 180.0  # výchozí hodnota přesnosti headingu
        self._smooth_heading = 0.0
        self._smooth_speed = 0.0


    # === Vstup z ESF-RAW handleru ============================================

    def on_esf_raw(self, raw: EsfRawData) -> None:
        calibrated_gyroZ = raw.gyroZ + self._gyroZ_calibration_offset
        self._last_gyroZ_value = calibrated_gyroZ
        self._gyro_smoother.update(calibrated_gyroZ)

        #if self._gyro_last_sTag is not None:
        #    dt_ms = raw.sTtag - self._gyro_last_sTag
        #    if dt_ms < 0:
        #        dt_ms += 2**32  # rollover
        #    self._cumulated_angleZ +=  calibrated_gyroZ * (dt_ms / 1000.0)
        #    #self._smooth_heading = (self._cumulated_angleZ + self._smooth_heading) / 2.0
        #self._gyro_last_sTag = raw.sTtag


    # === Vstup z NAV-PVAT handleru ===========================================

    def on_nav_pvat(self, pvat: NavPvatData) -> None:
        now = time.monotonic()
        smoothed_gyroZ = self._gyro_smoother.last
        #self._heading_acc = min(self._heading_acc, pvat.accHeading)

        # if pvat.fixType in (4, 5):  # dead reckoning
        #     heading_deg = pvat.vehHeading
        #     speed = pvat.gSpeed
        # elif self._lever_arm:
        #     heading_deg, speed = self._lever_arm.theta_from_motHeading_deg(
        #         motHeading_deg=pvat.motHeading,
        #         speed=pvat.gSpeed,
        #         omega_deg_cw=smoothed_gyroZ
        #     )
        # else:
        #     heading_deg = pvat.motHeading
        #     speed = pvat.gSpeed
        self._smooth_heading = (pvat.motHeading + self._smooth_heading * 3.0) / 4.0
        self._smooth_speed = (pvat.gSpeed + self._smooth_speed * 3.0) / 4.0

        heading_deg = self._smooth_heading
        speed = self._smooth_speed

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
