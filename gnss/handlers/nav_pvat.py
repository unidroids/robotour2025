import struct
import time
import threading
from typing import Optional, Callable
from data.nav_pvat_data import NavPvatData

def parse_nav_pvat_flags(flags: int):
    return {
        "gnssFixOK":        bool(flags & (1 << 0)),
        "diffSoln":         bool(flags & (1 << 1)),
        "vehRollValid":     bool(flags & (1 << 3)),
        "vehPitchValid":    bool(flags & (1 << 4)),
        "vehHeadingValid":  bool(flags & (1 << 5)),
        "carrSoln":         (flags >> 6) & 0b11,
    }

class NavPvatHandler:
    NAV_PVAT_PAYLOAD_LEN = 116

    # Stejný struct jako v původním kódu
    _S = struct.Struct(
        "<I B B H B B B B B B 2s I i B B B B i i i i I I i i i i I i i i i H H H h H H I I 4s 4s"
    )

    def __init__(self, on_pvat: Optional[Callable[[NavPvatData], None]] = None):
        self._lock = threading.Lock()
        self._last: Optional[NavPvatData] = None
        self.on_pvat = on_pvat

    def handle(self, msg_class: int, msg_id: int, payload: bytes) -> None:
        if len(payload) != self.NAV_PVAT_PAYLOAD_LEN:
            return

        now = time.monotonic()
        unpacked = self._S.unpack_from(payload, 0)
        (
            iTOW, version, valid,
            year, month, day, hour, minute, sec, _reserved0, _reserved1,
            tAcc, nano, fixType, flags, flags2, numSV,
            lon, lat, height, hMSL, hAcc, vAcc,
            velN, velE, velD, gSpeed, sAcc,
            vehRoll, vehPitch, vehHeading, motHeading,
            accRoll, accPitch, accHeading,
            magDec, magAcc,
            errEllipseOrient, errEllipseMajor, errEllipseMinor,
            _reserved2, _reserved3
        ) = unpacked

        # Škálování do SI jednotek
        fi = parse_nav_pvat_flags(flags)
        data = NavPvatData(
            iTOW=iTOW,
            version=version,
            valid=valid,
            year=year, month=month, day=day, hour=hour, minute=minute, sec=sec, tAcc=tAcc, nano=nano,
            fixType=fixType, flags=flags, flags2=flags2, numSV=numSV,
            lon=lon/1e7, lat=lat/1e7, height=height/1000, hMSL=hMSL/1000, hAcc=hAcc/1000, vAcc=vAcc/1000,
            velN=velN/1000, velE=velE/1000, velD=velD/1000, gSpeed=gSpeed/1000, sAcc=sAcc/1000,
            vehRoll=vehRoll/1e5, vehPitch=vehPitch/1e5, vehHeading=vehHeading/1e5, motHeading=motHeading/1e5,
            accRoll=accRoll/100, accPitch=accPitch/100, accHeading=accHeading/100,
            magDec=magDec/100, magAcc=magAcc/100,
            errEllipseOrient=errEllipseOrient, errEllipseMajor=errEllipseMajor, errEllipseMinor=errEllipseMinor,
            carrSoln=fi["carrSoln"], gnssFixOK=fi["gnssFixOK"], diffSoln=fi["diffSoln"],
            vehRollValid=fi["vehRollValid"], vehPitchValid=fi["vehPitchValid"], vehHeadingValid=fi["vehHeadingValid"],
            rx_mono=now
        )
        with self._lock:
            self._last = data
        if self.on_pvat:
            self.on_pvat(data)

    def get_last(self) -> Optional[NavPvatData]:
        with self._lock:
            return self._last

    def get_log(self) -> str:
        d = self.get_last()
        if d is None:
            return "No data"
        return d.get_log()