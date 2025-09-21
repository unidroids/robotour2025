import struct
import time
import threading

class EsfInsHandler:
    def __init__(self, bin_stream_fifo=None, fifo_lock=None):
        self.context = None
        self.bin_stream_fifo = bin_stream_fifo
        self._fifo_lock = fifo_lock or threading.Lock()
        self.dropped = 0
        self.count = 0

    def handle(self, msg_class, msg_id, payload):
        if len(payload) != 36:
            print("[ESF-INS] Wrong payload length:", len(payload))
            return

        self.count += 1
        # Rozbalení všech valid bitů a hodnot (dle dokumentace)
        bitfield0, = struct.unpack('<I', payload[0:4])
        version = bitfield0 & 0xFF

        xAngRateValid = bool((bitfield0 >> 8) & 0x01)
        yAngRateValid = bool((bitfield0 >> 9) & 0x01)
        zAngRateValid = bool((bitfield0 >> 10) & 0x01)
        xAccelValid = bool((bitfield0 >> 11) & 0x01)
        yAccelValid = bool((bitfield0 >> 12) & 0x01)
        zAccelValid = bool((bitfield0 >> 13) & 0x01)

        # payload[4:8] je reserved0
        iTOW, = struct.unpack('<I', payload[8:12])
        xAngRate, = struct.unpack('<i', payload[12:16])   # 1e-3 deg/s
        yAngRate, = struct.unpack('<i', payload[16:20])
        zAngRate, = struct.unpack('<i', payload[20:24])
        xAccel, = struct.unpack('<i', payload[24:28])     # 1e-2 m/s^2
        yAccel, = struct.unpack('<i', payload[28:32])
        zAccel, = struct.unpack('<i', payload[32:36])

        ctx = dict(
            iTOW=iTOW,
            version=version,
            xAngRate=xAngRate, xAngRateValid=xAngRateValid,
            yAngRate=yAngRate, yAngRateValid=yAngRateValid,
            zAngRate=zAngRate, zAngRateValid=zAngRateValid,
            xAccel=xAccel, xAccelValid=xAccelValid,
            yAccel=yAccel, yAccelValid=yAccelValid,
            zAccel=zAccel, zAccelValid=zAccelValid,
            bitfield0=bitfield0,
            timestamp=time.time()
        )
        self.context = ctx

        # Podrobný log každých 100 zpráv
        if self.count % 100 == 0: # or not (xAngRateValid or yAngRateValid or zAngRateValid):
            print(f"[ESF-INS] {iTOW} v={version} "
                  f"xAngRate={xAngRate/1e3 if xAngRateValid else 'N/A'} ({xAngRateValid}) | "
                  f"yAngRate={yAngRate/1e3 if yAngRateValid else 'N/A'} ({yAngRateValid}) | "
                  f"zAngRate={zAngRate/1e3 if zAngRateValid else 'N/A'} ({zAngRateValid}) || "
                  f"xAccel={xAccel/1e2 if xAccelValid else 'N/A'} ({xAccelValid}) | "
                  f"yAccel={yAccel/1e2 if yAccelValid else 'N/A'} ({yAccelValid}) | "
                  f"zAccel={zAccel/1e2 if zAccelValid else 'N/A'} ({zAccelValid}) || "
                  f"bitfield0=0x{bitfield0:08X}")

        if self.bin_stream_fifo is not None:
            with self._fifo_lock:
                if self.bin_stream_fifo.full():
                    try:
                        self.bin_stream_fifo.get_nowait()
                        self.dropped += 1
                    except Exception:
                        pass
                try:
                    # Příklad: do streamu push jen zAngRate hodnoty (upravit dle potřeby)
                    data = struct.pack('<I ? i', iTOW, zAngRateValid, zAngRate)
                    self.bin_stream_fifo.put_nowait(data)
                except Exception:
                    pass

    def get_last_context(self):
        return self.context
