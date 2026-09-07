import time
from dataclasses import dataclass
from data.nav_fusion_data import NavFusionData

@dataclass
class HeadingData:
    heading: float = 0.0
    acc: float = 180.0
    sol: str = "NONE"

class FusionCore:
    def __init__(self):
        self.ready = False
        self._heading_initialized = False
        self._last_gyro_ts = None
        self._last_msg_mono_ts = None

        # GPS state
        self._lat: float = 0.0
        self._lon: float = 0.0
        self._hAcc: float = 0.0
        self._gpsSol: str = "NONE"
        self._have_position: bool = False
        self._bestnav_pos_type: str = "NONE"
        self._uniheading_pos_type: str = "NONE"

        # Headings
        self.gps_heading = HeadingData()
        self.dual_heading = HeadingData()
        self.compass_heading = HeadingData()
        self.default_heading = HeadingData()
        self.fused_heading = HeadingData()

        # Odometry state
        self._speed: float = 0.0
        self._sAcc: float = 0.05

        # Compass state
        self._gyroZ: float = 0.0
        self._gyroZAcc: float = 5.0

        # Fusion state
        self._fusionSol: str = "NONE"

    @property
    def bestnav_pos_type(self) -> str:
        return self._bestnav_pos_type

    @property
    def uniheading_pos_type(self) -> str:
        return self._uniheading_pos_type

    @staticmethod
    def _norm_deg(a: float) -> float:
        """Normalizace do [0, 360)."""
        a = a % 360.0
        if a < 0.0:
            a += 360.0
        return a

    def _update_ready_flag(self) -> None:
        self.ready = self._have_position and self._heading_initialized

    def update_position(self, lat: float, lon: float, hAcc: float, gpsSol: str):
        self._lat = float(lat)
        self._lon = float(lon)
        self._hAcc = float(hAcc)
        self._gpsSol = gpsSol
        self._bestnav_pos_type = gpsSol
        self._have_position = True
        self._last_msg_mono_ts = time.monotonic()
        self._update_ready_flag()
        if not self._heading_initialized:
            self.fused_heading.acc = 180.0
            self.fused_heading.sol = "NONE"
            self._fusionSol = "NONE"

    def update_dual_heading(self, heading: float, headingAcc: float, headingSol: str, length: float = 0.0):
        """Heading from dual-antenna GPS (North East)"""
        self.dual_heading.heading = self._norm_deg(heading)
        self.dual_heading.acc = float(headingAcc)
        self.dual_heading.sol = headingSol
        self._uniheading_pos_type = headingSol
        self._last_msg_mono_ts = time.monotonic()
        
        if self.dual_heading.sol == "NARROW_INT" and self.dual_heading.acc < 1.5:
            self._heading_initialized = True
            self.fused_heading.heading = self.dual_heading.heading
            self.fused_heading.acc = self.dual_heading.acc
            self.fused_heading.sol = self.dual_heading.sol
            self._fusionSol = "UNIHEADING"
            
        self._update_ready_flag()

    def update_gps_heading(self, heading: float, headingAcc: float, headingSol: str):
        """Heading derived from GPS velocity (BESTNAV)"""
        self.gps_heading.heading = self._norm_deg(heading)
        self.gps_heading.acc = float(headingAcc)
        self.gps_heading.sol = headingSol

    def update_odometry(self, speed_left: float, speed_right: float):
        """Odometry speed in mm/s"""
        self._speed = (speed_left + speed_right) / 2.0

    def update_gyro(self, ts: float, wz: float):
        """Compass gyro in deg/s"""
        self._gyroZ = float(wz)
        self._last_msg_mono_ts = time.monotonic()
        if self._last_gyro_ts is not None:
            dt = ts - self._last_gyro_ts
            if 0 < dt < 1.0:
                if self._heading_initialized:
                    if not (self.dual_heading.sol == "NARROW_INT" and self.dual_heading.acc < 1.5):
                        self.fused_heading.heading = self._norm_deg(self.fused_heading.heading - self._gyroZ * dt)
                        self.fused_heading.acc = 4.0
                        self.fused_heading.sol = "GYRO"
                        self._fusionSol = "GYRO"
        
        self._last_gyro_ts = ts
        self._update_ready_flag()

    def update_compass_angle(self, yaw: float):
        """Compass angle in deg"""
        self.compass_heading.heading = self._norm_deg(185-float(yaw))
        self.compass_heading.acc = 4.0
        self.compass_heading.sol = "COMPASS"

    def get_solution(self) -> NavFusionData:
        now = time.monotonic()
        if self._last_msg_mono_ts is not None and (now - self._last_msg_mono_ts) > 1.0:
            self.fused_heading.sol = "NONE"
            self.fused_heading.acc = 180.0
            self._fusionSol = "NONE"

        return NavFusionData(
            ts_mono=now,
            lat=self._lat,
            lon=self._lon,
            hAcc=self._hAcc,
            heading=self.fused_heading.heading,
            headingAcc=self.fused_heading.acc,
            speed=self._speed,
            sAcc=self._sAcc,
            gyroZ=self._gyroZ,
            gyroZAcc=self._gyroZAcc,
            gpsSol=self._gpsSol,
            headingSol=self.fused_heading.sol,
            fusionSol=self._fusionSol,
            bestnav_pos_type=self.bestnav_pos_type,
            uniheading_pos_type=self.uniheading_pos_type,
        )