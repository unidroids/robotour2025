import struct
import time
import threading

class NavPvatHandler:
    def __init__(self, bin_stream_fifo=None, fifo_lock=None):
        self.context = None
        self.bin_stream_fifo = bin_stream_fifo
        self._fifo_lock = fifo_lock or threading.Lock()
        self.dropped = 0
        self.count = 0
        self._last_print_sec = None

    def handle(self, msg_class, msg_id, payload):
        if len(payload) < 116:
            print("[NAV-PVAT] Wrong payload length:", len(payload))
            return

        self.count += 1
        # Dekódování payloadu podle popisu z dokumentace (verze 1.40)
        # (
        #     iTOW, version, valid, year, month, day, hour, minute, sec, reserved0, reserved1,
        #     tAcc, nano, fixType, flags, flags2, numSV, lon, lat, height, hMSL, hAcc, vAcc,
        #     velN, velE, velD, gSpeed, sAcc, vehRoll, vehPitch, vehHeading, motHeading,
        #     accRoll, accPitch, accHeading, magDec, magAcc, errEllipseOrient, errEllipseMajor,
        #     errEllipseMinor, reserved2, reserved3
        # ) = struct.unpack('<IBB HBBBBBB 2s I i B B B B i i i i I I i i i i I I i i i i H H H h H H I I 4s 4s', payload)

        (
            iTOW, version, valid, year, month, day, hour, minute, second, reserved0, reserved1,
            tAcc, nano, fixType, flags, flags2, numSV, lon, lat, height, hMSL, hAcc, vAcc,
            velN, velE, velD, gSpeed, sAcc, vehRoll, vehPitch, vehHeading, motHeading,
            accRoll, accPitch, accHeading, magDec, magAcc, errEllipseOrient,
            errEllipseMajor, errEllipseMinor, reserved2, reserved3
        ) = struct.unpack('<IBBHBBBBBI2sI i B B B i i i i I I i i i i i i i i i i h H h h h h I B B B B', payload)

        # Vytvoření kontextu
        ctx = dict(
            iTOW=iTOW, year=year, month=month, day=day, hour=hour, minute=minute, sec=sec,
            fixType=fixType, flags=flags, numSV=numSV, lon=lon/1e7, lat=lat/1e7, height=height/1000,
            hMSL=hMSL/1000, hAcc=hAcc/1000, vAcc=vAcc/1000,
            gSpeed=gSpeed/1000, velN=velN/1000, velE=velE/1000, velD=velD/1000, sAcc=sAcc/1000,
            roll=vehRoll/1e5, pitch=vehPitch/1e5, heading=vehHeading/1e5,
            motHeading=motHeading/1e5,
            accRoll=accRoll/100, accPitch=accPitch/100, accHeading=accHeading/100,
            magDec=magDec/100, magAcc=magAcc/100,
            timestamp=time.time()
        )
        self.context = ctx

        # Výpis pouze jednou za sekundu (podle GPS času)
        sec = iTOW // 1000
        if self._last_print_sec is None or sec != self._last_print_sec:
            self._last_print_sec = sec
            print(
                f"[NAV-PVAT] {year:04}-{month:02}-{day:02} {hour:02}:{minute:02}:{sec:02} "
                f"fix={fixType} SV={numSV} "
                f"lat={lat/1e7:.7f} lon={lon/1e7:.7f} hEll={height/1000:.2f}m hMSL={hMSL/1000:.2f}m "
                f"hAcc={hAcc/1000:.3f}m vAcc={vAcc/1000:.3f}m "
                f"gSpd={gSpeed/1000:.3f}m/s vN={velN/1000:.3f} vE={velE/1000:.3f} vD={velD/1000:.3f} "
                f"sAcc={sAcc/1000:.3f}m/s roll={vehRoll/1e5:.2f}°({accRoll/100:.2f}) "
                f"pitch={vehPitch/1e5:.2f}°({accPitch/100:.2f}) hdg={vehHeading/1e5:.2f}°({accHeading/100:.2f})"
            )

        # Možnost pushnout binární data do fronty
        if self.bin_stream_fifo is not None:
            with self._fifo_lock:
                if self.bin_stream_fifo.full():
                    try:
                        self.bin_stream_fifo.get_nowait()
                        self.dropped += 1
                    except queue.Empty:
                        pass
                try:
                    # Vyber si co logovat binárně
                    data = struct.pack('<I i i i i I I',
                        iTOW, lon, lat, height, gSpeed, fixType, numSV
                    )
                    self.bin_stream_fifo.put_nowait(data)
                except Exception:
                    pass

    def get_last_context(self):
        return self.context
