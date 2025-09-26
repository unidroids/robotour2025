import struct
import time
from collections import deque
import threading
import queue

# --- Pomocné funkce mimo RT cesty (logování, dekódování) ---------------------

def parse_nav_pvat_flags(flags: int):
    """Lehký parser flagů – používej mimo RT cesty (např. v logovacích metodách)."""
    return {
        "gnssFixOK":        bool(flags & (1 << 0)),
        "diffSoln":         bool(flags & (1 << 1)),
        "vehRollValid":     bool(flags & (1 << 3)),
        "vehPitchValid":    bool(flags & (1 << 4)),
        "vehHeadingValid":  bool(flags & (1 << 5)),
        "carrSoln":         (flags >> 6) & 0b11,  # 0: none, 1: float, 2: fix
    }

def _carr_soln_str(carr_soln: int) -> str:
    return ("none", "float", "fix", "reserved")[carr_soln & 0b11]


class NavPvatHandler:
    """
    Time-critical handler pro UBX-NAV-PVAT (116 B, ~30 Hz).

    Hot-path (handle):
      - žádné printy
      - žádné float převody
      - minimální alokace
      - 1 tuple na zprávu uložený do deque (ring buffer)

    Tuple pořadí (pevné; raw škálování dle UBX):
      (
        iTOW, version, valid,
        year, month, day, hour, minute, sec,
        tAcc, nano, fixType, flags, flags2, numSV,
        lon, lat, height, hMSL,
        hAcc, vAcc,
        velN, velE, velD, gSpeed, sAcc,
        vehRoll, vehPitch, vehHeading, motHeading,
        accRoll, accPitch, accHeading,
        magDec, magAcc,
        errEllipseOrient, errEllipseMajor, errEllipseMinor
      )

    Kde škálování (pro lidský výpis) je:
      lon/lat: 1e-7 deg
      height/hMSL: mm -> m ( /1000 )
      hAcc/vAcc: mm -> m ( /1000 )
      velN/velE/velD/gSpeed: mm/s -> m/s ( /1000 )
      sAcc: mm/s -> m/s ( /1000 )
      vehRoll/vehPitch/vehHeading/motHeading: 1e-5 deg
      accRoll/accPitch/accHeading: 1e-2 deg
      magDec: 1e-2 deg, magAcc: 1e-2 deg
    """

    NAV_PVAT_PAYLOAD_LEN = 116

    # Připravený struct pro rychlost (unpack_from nealokuje nový buffer)
    _S = struct.Struct(
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

    __slots__ = (
        "count", "t0", "context", "_maxlen",
        "bin_stream_fifo", "_fifo_lock", "dropped"
    )

    def __init__(self, bin_stream_fifo: "queue.Queue|None" = None,
                 fifo_lock: "threading.Lock|None" = None,
                 max_context: int = 256):
        # RT metriky
        self.count = 0
        self.t0 = time.monotonic()

        # Ring buffer posledních N zpráv
        self._maxlen = max_context
        self.context = deque(maxlen=max_context)

        # (volitelně) binární stream pro tenký výstup (např. do TCP)
        self.bin_stream_fifo = bin_stream_fifo
        self._fifo_lock = fifo_lock or threading.Lock()
        self.dropped = 0

    # --- HOT PATH -------------------------------------------------------------

    def handle(self, msg_class: int, msg_id: int, payload: bytes) -> None:
        """RT-safe: žádné printy, žádné floaty, minimum alokací."""
        if len(payload) != self.NAV_PVAT_PAYLOAD_LEN:
            return

        self.count += 1

        # Unpack bez mezialokací
        (
            iTOW, version, valid, year, month, day, hour, minute, sec, _reserved0, _reserved1,
            tAcc, nano, fixType, flags, flags2, numSV,
            lon, lat, height, hMSL, hAcc, vAcc,
            velN, velE, velD, gSpeed, sAcc,
            vehRoll, vehPitch, vehHeading, motHeading,
            accRoll, accPitch, accHeading,
            magDec, magAcc,
            errEllipseOrient, errEllipseMajor, errEllipseMinor,
            _reserved2, _reserved3
        ) = self._S.unpack_from(payload, 0)

        # 1 tuple = 1 zpráva
        tup = (
            iTOW, version, valid,
            year, month, day, hour, minute, sec,
            tAcc, nano, fixType, flags, flags2, numSV,
            lon, lat, height, hMSL,
            hAcc, vAcc,
            velN, velE, velD, gSpeed, sAcc,
            vehRoll, vehPitch, vehHeading, motHeading,
            accRoll, accPitch, accHeading,
            magDec, magAcc,
            errEllipseOrient, errEllipseMajor, errEllipseMinor
        )
        self.context.append(tup)

        # (Volitelné) – tenký binární push do FIFO bez floatů
        if self.bin_stream_fifo is not None:
            # Minimal pack: iTOW, lon, lat, height, gSpeed, fixType, numSV
            try:
                with self._fifo_lock:
                    if hasattr(self.bin_stream_fifo, "full") and self.bin_stream_fifo.full():
                        try:
                            self.bin_stream_fifo.get_nowait()
                            self.dropped += 1
                        except queue.Empty:
                            pass
                    data = struct.pack("<I i i i i B B",
                                       iTOW, lon, lat, height, gSpeed, fixType, numSV)
                    self.bin_stream_fifo.put_nowait(data)
            except Exception:
                # RT cesta: nepropagujeme výjimky, dropneme
                pass

    # --- MIMO HOT PATH: metody pro logy/sumarizaci ---------------------------

    def get_stats(self):
        """Vrátí (count, elapsed_s, avg_rate_hz)."""
        now = time.monotonic()
        elapsed = now - self.t0
        rate = (self.count / elapsed) if elapsed > 0 else 0.0
        return self.count, elapsed, rate

    def last_tuple(self):
        """Poslední uložený tuple nebo None."""
        return self.context[-1] if self.context else None

    def snapshot_and_clear(self):
        """Vezme aktuální obsah bufferu jako list a buffer vymaže."""
        items = list(self.context)
        self.context.clear()
        return items

    # --- Lidský výpis (dělej „mimo RT“ – např. 1×/s) -------------------------

    def log_stats(self, print_fn=print):
        count, elapsed, rate = self.get_stats()
        print_fn(f"[NAV-PVAT] Count={count} Elapsed={elapsed:.1f}s AvgRate={rate:.1f} Hz")

    def log_last_tuple(self, print_fn=print):
        t = self.last_tuple()
        if not t:
            print_fn("[NAV-PVAT] (no data)")
            return

        (
            iTOW, version, valid,
            year, month, day, hour, minute, sec,
            tAcc, nano, fixType, flags, flags2, numSV,
            lon, lat, height, hMSL,
            hAcc, vAcc,
            velN, velE, velD, gSpeed, sAcc,
            vehRoll, vehPitch, vehHeading, motHeading,
            accRoll, accPitch, accHeading,
            magDec, magAcc,
            errEllipseOrient, errEllipseMajor, errEllipseMinor
        ) = t

        fi = parse_nav_pvat_flags(flags)

        # Převody pro lidské čtení (mimo RT)
        lon_deg = lon / 1e7
        lat_deg = lat / 1e7
        h_ell_m = height / 1000.0
        h_msl_m = hMSL / 1000.0
        hAcc_m = hAcc / 1000.0
        vAcc_m = vAcc / 1000.0
        vN_ms = velN / 1000.0
        vE_ms = velE / 1000.0
        vD_ms = velD / 1000.0
        g_ms   = gSpeed / 1000.0
        sAcc_ms = sAcc / 1000.0

        roll_deg    = vehRoll / 1e5
        pitch_deg   = vehPitch / 1e5
        hdg_deg     = vehHeading / 1e5
        mot_deg     = motHeading / 1e5

        accRoll_deg    = accRoll / 100.0
        accPitch_deg   = accPitch / 100.0
        accHeading_deg = accHeading / 100.0

        magDec_deg  = magDec / 100.0
        magAcc_deg  = magAcc / 100.0

        flags_msg = (
            f"FixOK={fi['gnssFixOK']} "
            f"DiffCorr={fi['diffSoln']} "
            f"Roll={fi['vehRollValid']} "
            f"Pitch={fi['vehPitchValid']} "
            f"Heading={fi['vehHeadingValid']} "
            f"CarrSoln={fi['carrSoln']} ({_carr_soln_str(fi['carrSoln'])})"
        )

        print_fn(
            f"[NAV-PVAT] {year:04}-{month:02}-{day:02} {hour:02}:{minute:02}:{sec:02} "
            f"fix={fixType} SV={numSV} ({flags_msg}) "
            f"lat={lat_deg:.7f} lon={lon_deg:.7f} hEll={h_ell_m:.2f}m hMSL={h_msl_m:.2f}m "
            f"gSpd={g_ms:.3f}m/s vN={vN_ms:.3f} vE={vE_ms:.3f} vD={vD_ms:.3f} sAcc={sAcc_ms:.3f}m/s "
            f"roll={roll_deg:.2f}°({accRoll_deg:.2f}) pitch={pitch_deg:.2f}°({accPitch_deg:.2f}) "
            f"hdg={hdg_deg:.2f}°({accHeading_deg:.2f}) mot={mot_deg:.2f}° "
            f"hAcc={hAcc_m:.3f}m vAcc={vAcc_m:.3f}m "
            f"iTOW={iTOW} nano={nano} tAcc={tAcc} errEll[ori={errEllipseOrient}, maj={errEllipseMajor}, min={errEllipseMinor}]"
        )

