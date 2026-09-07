# data/nav_fusion_data.py
from __future__ import annotations
from dataclasses import dataclass
import json


@dataclass
class NavFusionData:
    """
    Kompaktní 2D stav pro PILOTA, verze 1.
    """

    # --- čas ---
    ts_mono: float

    # --- poloha/orientace/pohyb ---
    lat: float
    lon: float
    hAcc: float
    heading: float
    headingAcc: float
    speed: float
    sAcc: float
    gyroZ: float
    gyroZAcc: float

    # --- flagy/stavy ---
    gpsSol: str
    headingSol: str
    fusionSol: str
    bestnav_pos_type: str = "NONE"
    uniheading_pos_type: str = "NONE"

    # --- API ---
    def to_json(self) -> str:
        """Vrátí obsah objektu jako JSON string."""
        return json.dumps({
            "ts_mono": self.ts_mono,
            "lat": self.lat,
            "lon": self.lon,
            "hAcc": self.hAcc * 1000,
            "heading": self.heading,
            "headingAcc": self.headingAcc,
            "speed": self.speed,
            "sAcc": self.sAcc,
            "gyroZ": self.gyroZ,
            "gyroZAcc": self.gyroZAcc,
            "gpsSol": self.gpsSol,
            "headingSol": self.headingSol,
            "fusionSol": self.fusionSol,
            "bestnav_pos_type": self.bestnav_pos_type,
            "uniheading_pos_type": self.uniheading_pos_type,
        })


# --- self-test ---
if __name__ == "__main__":
    state = NavFusionData(
        ts_mono=12345.678,
        lat=49.0001234,
        lon=17.0005678,
        hAcc=0.25,
        heading=92.4,
        headingAcc=1.2,
        speed=0.54,
        sAcc=0.05,
        gyroZ=-12.3,
        gyroZAcc=0.8,
        gpsSol="SINGLE",
        headingSol="NONE",
        fusionSol="NONE",
        bestnav_pos_type="SINGLE",
        uniheading_pos_type="NONE",
    )
    print("to_json:", state.to_json())
