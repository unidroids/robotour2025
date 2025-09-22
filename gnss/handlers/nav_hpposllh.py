import struct
import time
import threading

class NavHpposllhHandler:
    def __init__(self, bin_stream_fifo=None, fifo_lock=None):
        self.context = None
        self.bin_stream_fifo = bin_stream_fifo
        self._fifo_lock = fifo_lock or threading.Lock()
        self.dropped = 0  # počítadlo zahozených zpráv
        self.count = 0
        self._last_print_sec = None

    def handle(self, msg_class, msg_id, payload):
        if len(payload) != 36:
            print("[HPPOSLLH] Wrong payload length:", len(payload))
            return
        self.count += 1
            
        (version, reserved0_0, reserved0_1, flags, iTOW, lon, lat, height, hMSL,
         lonHp, latHp, heightHp, hMSLHp, hAcc, vAcc) = struct.unpack('<BBB B I i i i i b b b b I I', payload)

        invalidLlh = bool(flags & 0x01)
        ctx = dict(
            iTOW=iTOW, invalidLlh=invalidLlh, lon=lon, lat=lat, height=height,
            hMSL=hMSL, lonHp=lonHp, latHp=latHp, heightHp=heightHp, hMSLHp=hMSLHp,
            hAcc=hAcc, vAcc=vAcc, timestamp=time.time()
        )
        self.context = ctx

        sec = iTOW // 1000
        if self._last_print_sec is None or sec != self._last_print_sec:
            self._last_print_sec = sec

            print(f"[HPPOSLLH] {iTOW} {'INVALID' if invalidLlh else 'OK'}  lon={lon} lat={lat} hAcc={hAcc/10:.1f}mm")

        # -- Binární stream do FIFO, thread-safe, full→get→put --
        if self.bin_stream_fifo is not None:
            with self._fifo_lock:
                if self.bin_stream_fifo.full():
                    try:
                        self.bin_stream_fifo.get_nowait()
                        self.dropped += 1
                    except queue.Empty:
                        pass
                try:
                    data = struct.pack('<I ? i i i i b b b b I I',
                        iTOW, invalidLlh, lon, lat, height, hMSL, lonHp, latHp, heightHp, hMSLHp, hAcc, vAcc)
                    self.bin_stream_fifo.put_nowait(data)
                except queue.Full:
                    pass

    def get_last_context(self):
        return self.context
