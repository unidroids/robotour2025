import struct
import time
import threading

class NavVelNedHandler:
    def __init__(self, bin_stream_fifo=None, fifo_lock=None):
        self.context = None
        self.bin_stream_fifo = bin_stream_fifo
        self._fifo_lock = fifo_lock or threading.Lock()
        self.dropped = 0

    def handle(self, msg_class, msg_id, payload):
        if len(payload) != 36:
            print("[VELNED] Wrong payload length:", len(payload))
            return

        (iTOW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc) = struct.unpack('<I i i i I I i I I', payload)
        ctx = dict(
            iTOW=iTOW, speed=speed, gSpeed=gSpeed, heading=heading,
            sAcc=sAcc, cAcc=cAcc, timestamp=time.time()
        )
        self.context = ctx
        #print(f"[VELNED] {iTOW}  speed={speed/100:.2f}m/s gSpeed={gSpeed/100:.2f} heading={heading/1e5:.2f}°")

        if self.bin_stream_fifo is not None:
            with self._fifo_lock:
                if self.bin_stream_fifo.full():
                    try:
                        self.bin_stream_fifo.get_nowait()
                        self.dropped += 1
                    except queue.Empty:
                        pass
                try:
                    data = struct.pack('<I I I i I I', iTOW, speed, gSpeed, heading, sAcc, cAcc)
                    self.bin_stream_fifo.put_nowait(data)
                except queue.Full:
                    pass

    def get_last_context(self):
        return self.context
