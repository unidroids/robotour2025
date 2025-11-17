# data/nav_fusion_data.py
from __future__ import annotations
from dataclasses import dataclass
import struct
from typing import ClassVar
import json


@dataclass
class NavFusionData:
    """
    Kompaktní 2D stav pro PILOTA, verze 2.

    Binární formát (Little-Endian), velikost 63 B:

        B      version     (uint8)   - musí být 2
        d      ts_mono     (float64) - monotonic timestamp [s]
        d      lat         (float64) - WGS84 [deg]
        d      lon         (float64) - WGS84 [deg]
        f      hAcc        (float32) - horizontal accuracy [m]
        f      heading     (float32) - [deg]
        f      headingAcc  (float32) - [deg]
        f      speed       (float32) - [m/s]
        f      sAcc        (float32) - [m/s]
        f      gyroZ       (float32) - [deg/s]
        f      gyroZAcc    (float32) - [deg/s]
        B      gnssFixOK   (uint8)   - 0/1
        B      drUsed      (uint8)   - 0/1
        f      vehHeading  (float32) - [deg] (debug)
        f      motHeading  (float32) - [deg] (debug)
        f      lastGyroZ   (float32) - [deg/s] (debug)
    """

    VERSION: ClassVar[int] = 2

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

    # --- flagy ---
    gnssFixOK: bool
    drUsed: bool

    # --- ladící položky ---
    vehHeading: float
    motHeading: float
    lastGyroZ: float
    gSpeed: float  

    # --- binární formát ---
    _STRUCT_FMT: ClassVar[str] = "<B d d d f f f f f f f B B f f f f"
    _STRUCT: ClassVar[struct.Struct] = struct.Struct(_STRUCT_FMT)

    # --- API ---
    def to_bytes(self) -> bytes:
        """Zabalí objekt do LE binárního streamu (63 B)."""
        return self._STRUCT.pack(
            self.VERSION,
            self.ts_mono,
            self.lat,
            self.lon,
            float(self.hAcc),
            float(self.heading),
            float(self.headingAcc),
            float(self.speed),
            float(self.sAcc),
            float(self.gyroZ),
            float(self.gyroZAcc),
            1 if self.gnssFixOK else 0,
            1 if self.drUsed else 0,
            float(self.vehHeading),
            float(self.motHeading),
            float(self.lastGyroZ),
            float(self.gSpeed),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "NavFusionData":
        """Vytvoří objekt z binárního streamu a ověří verzi."""
        if len(data) != cls._STRUCT.size:
            raise ValueError(f"Invalid data length: {len(data)} (expected {cls._STRUCT.size})")
        unpacked = cls._STRUCT.unpack(data)
        version = unpacked[0]
        if version != cls.VERSION:
            raise ValueError(f"Unsupported version: {version} (expected {cls.VERSION})")

        return cls(
            ts_mono=unpacked[1],
            lat=unpacked[2],
            lon=unpacked[3],
            hAcc=unpacked[4],
            heading=unpacked[5],
            headingAcc=unpacked[6],
            speed=unpacked[7],
            sAcc=unpacked[8],
            gyroZ=unpacked[9],
            gyroZAcc=unpacked[10],
            gnssFixOK=bool(unpacked[11]),
            drUsed=bool(unpacked[12]),
            vehHeading=unpacked[13],
            motHeading=unpacked[14],
            lastGyroZ=unpacked[15],
            gSpeed=unpacked[16],
        )

    @classmethod
    def byte_size(cls) -> int:
        """Vrátí velikost binární reprezentace (63 B)."""
        return cls._STRUCT.size

    def to_json(self) -> str:
        """Vrátí obsah objektu jako JSON string."""
        return json.dumps({
            "ts_mono": self.ts_mono,
            "lat": self.lat,
            "lon": self.lon,
            "hAcc": self.hAcc,
            "heading": self.heading,
            "headingAcc": self.headingAcc,
            "speed": self.speed,
            "sAcc": self.sAcc,
            "gyroZ": self.gyroZ,
            "gyroZAcc": self.gyroZAcc,
            "gnssFixOK": bool(self.gnssFixOK),
            "drUsed": bool(self.drUsed),
            "vehHeading": self.vehHeading,
            "motHeading": self.motHeading,
            "lastGyroZ": self.lastGyroZ,
            "gSpeed": self.speed,  
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
        gnssFixOK=True,
        drUsed=False,
        vehHeading=90.0,
        motHeading=91.0,
        lastGyroZ=-12.0,
        gSpeed=0.54,  
    )
    blob = state.to_bytes()
    print("Byte size:", len(blob), "expected:", NavFusionData.byte_size())
    restored = NavFusionData.from_bytes(blob)
    print("Restored:", restored)
    print("to_json:", restored.to_json())
