import struct
import time
import threading
import queue


def parse_nav_pvat_flags(flags):
    return {
        "gnssFixOK":        bool(flags & (1 << 0)),
        "diffSoln":         bool(flags & (1 << 1)),
        "vehRollValid":     bool(flags & (1 << 3)),
        "vehPitchValid":    bool(flags & (1 << 4)),
        "vehHeadingValid":  bool(flags & (1 << 5)),
        "carrSoln":         (flags >> 6) & 0b11,  # 0: none, 1: float, 2: fix
    }


class NavPvatHandler:
    NAV_PVAT_PAYLOAD_LEN = 116
    # Odpovídá přesně oficiální specifikaci, včetně reserved polí!
    NAV_PVAT_STRUCT_FMT = (
        "<"   # Little-endian
        "I"   # iTOW
        "B"   # version
        "B"   # valid
        "H"   # year
        "B"   # month
        "B"   # day
        "B"   # hour
        "B"   # min
        "B"   # sec
        "B"   # reserved0
        "2s"  # reserved1 (2B)
        "I"   # tAcc
        "i"   # nano
        "B"   # fixType
        "B"   # flags
        "B"   # flags2
        "B"   # numSV
        "i"   # lon
        "i"   # lat
        "i"   # height
        "i"   # hMSL
        "I"   # hAcc
        "I"   # vAcc
        "i"   # velN
        "i"   # velE
        "i"   # velD
        "i"   # gSpeed
        "I"   # sAcc
        "i"   # vehRoll
        "i"   # vehPitch
        "i"   # vehHeading
        "i"   # motHeading
        "H"   # accRoll
        "H"   # accPitch
        "H"   # accHeading
        "h"   # magDec
        "H"   # magAcc
        "H"   # errEllipseOrient
        "I"   # errEllipseMajor
        "I"   # errEllipseMinor
        "4s"  # reserved2 (4B)
        "4s"  # reserved3 (4B)
    )

    def __init__(self, bin_stream_fifo=None, fifo_lock=None):
        self.context = None
        self.bin_stream_fifo = bin_stream_fifo
        self._fifo_lock = fifo_lock or threading.Lock()
        self.dropped = 0
        self.count = 0
        self._last_print_sec = None

    def handle(self, msg_class, msg_id, payload):
        if len(payload) != self.NAV_PVAT_PAYLOAD_LEN:
            print("[NAV-PVAT] Wrong payload length:", len(payload))
            return

        self.count += 1

        # Správné rozbalení payloadu dle specifikace
        (
            iTOW, version, valid, year, month, day, hour, minute, sec, reserved0, reserved1,
            tAcc, nano, fixType, flags, flags2, numSV, lon, lat, height, hMSL, hAcc, vAcc,
            velN, velE, velD, gSpeed, sAcc, vehRoll, vehPitch, vehHeading, motHeading,
            accRoll, accPitch, accHeading, magDec, magAcc, errEllipseOrient, errEllipseMajor,
            errEllipseMinor, reserved2, reserved3
        ) = struct.unpack(self.NAV_PVAT_STRUCT_FMT, payload)

        flags_info = parse_nav_pvat_flags(flags)

        ctx = dict(
            iTOW=iTOW, year=year, month=month, day=day, hour=hour, minute=minute, sec=sec,
            fixType=fixType, flags=flags_info, flags2=flags2, numSV=numSV,
            lon=lon/1e7, lat=lat/1e7, height=height/1000, hMSL=hMSL/1000,
            hAcc=hAcc/1000, vAcc=vAcc/1000,
            gSpeed=gSpeed/1000, velN=velN/1000, velE=velE/1000, velD=velD/1000, sAcc=sAcc/1000,
            roll=vehRoll/1e5, pitch=vehPitch/1e5, heading=vehHeading/1e5, motHeading=motHeading/1e5,
            accRoll=accRoll/100, accPitch=accPitch/100, accHeading=accHeading/100,
            magDec=magDec/100, magAcc=magAcc/100,
            timestamp=time.time()
        )
        self.context = ctx

        # Výpis pouze jednou za sekundu (dle GPS času)
        sec_epoch = iTOW // 1000
        if self._last_print_sec is None or sec_epoch != self._last_print_sec:
            self._last_print_sec = sec_epoch

            flags_msg = (
                f"FixOK={flags_info['gnssFixOK']} "
                f"DiffCorr={flags_info['diffSoln']} "
                f"Roll={flags_info['vehRollValid']} "
                f"Pitch={flags_info['vehPitchValid']} "
                f"Heading={flags_info['vehHeadingValid']} "
                f"CarrSoln={flags_info['carrSoln']} ({['none','float','fix','reserved'][flags_info['carrSoln']]})"
            )

            print(
                f"[NAV-PVAT] {year:04}-{month:02}-{day:02} {hour:02}:{minute:02}:{sec:02} "
                f"fix={fixType} SV={numSV} ({flags_msg}) "
                f"lat={lat/1e7:.7f} lon={lon/1e7:.7f} hEll={height/1000:.2f}m hMSL={hMSL/1000:.2f}m "
                f"gSpd={gSpeed/1000:.3f}m/s vN={velN/1000:.3f} vE={velE/1000:.3f} vD={velD/1000:.3f} sAcc={sAcc/1000:.3f}m/s "
                f"roll={vehRoll/1e5:.2f}°({accRoll/100:.2f}) pitch={vehPitch/1e5:.2f}°({accPitch/100:.2f}) hdg={vehHeading/1e5:.2f}°({accHeading/100:.2f}) mot={motHeading/1e5:.2f}° "
                f"hAcc={hAcc/1000:.3f}m vAcc={vAcc/1000:.3f}m "
            )

        # Možnost pushnout binární data do fronty (volitelně uprav)
        if self.bin_stream_fifo is not None:
            with self._fifo_lock:
                if self.bin_stream_fifo.full():
                    try:
                        self.bin_stream_fifo.get_nowait()
                        self.dropped += 1
                    except queue.Empty:
                        pass
                try:
                    # Uprav strukturu podle svých potřeb (například jen základní info)
                    data = struct.pack('<I i i i i B B',
                        iTOW, lon, lat, height, gSpeed, fixType, numSV
                    )
                    self.bin_stream_fifo.put_nowait(data)
                except Exception:
                    pass

    def get_last_context(self):
        return self.context
