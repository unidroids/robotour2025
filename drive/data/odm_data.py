# data/odm_data.py
from __future__ import annotations
from dataclasses import dataclass
import struct
from typing import ClassVar
import json

__all__ = ["OdmData"]


@dataclass
class OdmData:
    """
    Kompaktní data odometrie přenášená zprávou „ODM“.

    Zdrojová textová zpráva má formát:
        ODM%PRIu32,%PRId16,%PRId32,%PRId16,%PRId16
        => ts_mono_ms, gyroZ_adc, accumAngle_adc, leftSpeed, rightSpeed

    Binární formát (Little-Endian), velikost 15 B:

        B   version         (uint8)  - musí být 1
        I   ts_mono_ms      (uint32) - monotonic time [ms]
        h   gyroZ_adc       (int16)
        i   accumAngle_adc  (int32)
        h   leftSpeed       (int16)
        h   rightSpeed      (int16)
    """

    VERSION: ClassVar[int] = 1

    ts_mono_ms: int          # uint32
    gyroZ_adc: int           # int16
    accumAngle_adc: int      # int32
    leftSpeed: int           # int16
    rightSpeed: int          # int16

    # --- binární formát ---
    _STRUCT_FMT: ClassVar[str] = "<B I h i h h"
    _STRUCT: ClassVar[struct.Struct] = struct.Struct(_STRUCT_FMT)

    # --- API ---
    def to_bytes(self) -> bytes:
        """Zabalí objekt do LE binárního streamu (15 B)."""
        return self._STRUCT.pack(
            self.VERSION,
            self.ts_mono_ms & 0xFFFFFFFF,    # uint32
            int(self.gyroZ_adc),
            int(self.accumAngle_adc),
            int(self.leftSpeed),
            int(self.rightSpeed),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "OdmData":
        """Vytvoří objekt z binárního streamu a ověří verzi."""
        if len(data) != cls._STRUCT.size:
            raise ValueError(f"Invalid data length: {len(data)} (expected {cls._STRUCT.size})")
        version, ts_mono_ms, gyroZ_adc, accumAngle_adc, leftSpeed, rightSpeed = cls._STRUCT.unpack(data)
        if version != cls.VERSION:
            raise ValueError(f"Unsupported version: {version} (expected {cls.VERSION})")

        return cls(
            ts_mono_ms=int(ts_mono_ms),
            gyroZ_adc=int(gyroZ_adc),
            accumAngle_adc=int(accumAngle_adc),
            leftSpeed=int(leftSpeed),
            rightSpeed=int(rightSpeed),
        )

    @classmethod
    def byte_size(cls) -> int:
        """Vrátí velikost binární reprezentace (15 B)."""
        return cls._STRUCT.size

    def to_json(self) -> str:
        """Vrátí obsah objektu jako JSON string."""
        return json.dumps({
            "ts_mono_ms": int(self.ts_mono_ms),
            "gyroZ_adc": int(self.gyroZ_adc),
            "accumAngle_adc": int(self.accumAngle_adc),
            "leftSpeed": int(self.leftSpeed),
            "rightSpeed": int(self.rightSpeed),
        })


# --- self-test ---
if __name__ == "__main__":
    sample = OdmData(
        ts_mono_ms=123456789,
        gyroZ_adc=-123,
        accumAngle_adc=4567890,
        leftSpeed=321,
        rightSpeed=-400,
    )
    blob = sample.to_bytes()
    print("Byte size:", len(blob), "expected:", OdmData.byte_size())
    restored = OdmData.from_bytes(blob)
    print("Restored:", restored)
    print("to_json:", restored.to_json())
