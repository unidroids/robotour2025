import struct
import time
import threading

class EsfInsHandler:
    def __init__(self, bin_stream_fifo=None, fifo_lock=None):
        self.context = None
        self.bin_stream_fifo = bin_stream_fifo
        self._fifo_lock = fifo_lock or threading.Lock()
        self.dropped = 0

    def handle(self, msg_class, msg_id, payload):
        if len(payload) != 36:
            print("[ESF-INS] Wrong payload length:", len(payload))
            return

        bitfield0, = struct.unpack('<I', payload[0:4])
        zAngRateValid = bool((bitfield0 >> 10) & 0x01)
        iTOW, = struct.unpack('<I', payload[8:12])
        zAngRate, = struct.unpack('<i', payload[20:24])

        ctx = dict(
            iTOW=iTOW, zAngRateValid=zAngRateValid, zAngRate=zAngRate, timestamp=time.time()
        )
        self.context = ctx

        print(f"[ESF-INS] {iTOW} zAngRateValid={zAngRateValid} zAngRate={zAngRate/1e3:.3f} deg/s")

        if self.bin_stream_fifo is not None:
            with self._fifo_lock:
                if self.bin_stream_fifo.full():
                    try:
                        self.bin_stream_fifo.get_nowait()
                        self.dropped += 1
                    except queue.Empty:
                        pass
                try:
                    data = struct.pack('<I ? i', iTOW, zAngRateValid, zAngRate)
                    self.bin_stream_fifo.put_nowait(data)
                except queue.Full:
                    pass

    def get_last_context(self):
        return self.context
