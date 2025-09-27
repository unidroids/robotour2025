import time
import threading
from typing import Optional, Callable
from data.esf_raw_data import EsfRawData

# --- GNSS datový typ → index v rámci frame ---
_TYPE_TO_IDX = {
    14: 0,  # gyroX
    13: 1,  # gyroY
    5:  2,  # gyroZ
    16: 3,  # accX
    17: 4,  # accY
    18: 5,  # accZ
    12: 6,  # tempGyro
}
_IDX_TO_NAME = ("gyroX", "gyroY", "gyroZ", "accX", "accY", "accZ", "tempGyro")

class EsfRawHandler:
    """
    Handler pro UBX-ESF-RAW.
    - Vytváří EsfRawData (už s přeškálovanými hodnotami v SI).
    - Uchovává poslední instanci, volá callback (je-li zadán).
    """
    __slots__ = ("count", "t0", "_lock", "_last", "on_data")

    def __init__(self, on_data: Optional[Callable[[EsfRawData], None]] = None):
        self.count = 0
        self.t0 = time.monotonic()
        self._lock = threading.Lock()
        self._last: Optional[EsfRawData] = None
        self.on_data = on_data

    def handle(self, msg_class: int, msg_id: int, payload: bytes) -> None:
        """RT-safe: žádné printy, minimální alokace."""
        self.count += 1
        if len(payload) < 4:
            return

        now = time.monotonic()
        # [4B reserved][N * (4B data + 4B sTtag)]
        N = (len(payload) - 4) // 8
        if N <= 0:
            return

        base = 4
        frame = [None]*7
        last_sttag = 0

        for i in range(N):
            o = base + i * 8
            d = int.from_bytes(payload[o:o+4], "little", signed=False)
            df = d & 0xFFFFFF
            # sign-extend 24-bit
            if df & 0x800000:
                df -= 1 << 24
            dtype = (d >> 24) & 0xFF
            last_sttag = int.from_bytes(payload[o+4:o+8], "little", signed=False)
            idx = _TYPE_TO_IDX.get(dtype)
            if idx is not None:
                # --- Škálování už zde (pro SI výstup) ---
                if dtype in (16, 17, 18):   # acc m/s^2 * 2^-10
                    frame[idx] = df / 1024
                elif dtype in (5, 13, 14):  # gyro deg/s * 2^-12
                    frame[idx] = df / 4096
                elif dtype == 12:           # temp °C * 1e-2
                    frame[idx] = df / 100
                else:
                    frame[idx] = df

        # Pokud něco chybí, doplň na 0/None (není vždy kompletní frame)
        f = lambda v: v if v is not None else 0.0
        esf_raw = EsfRawData(
            gyroX=f(frame[0]),
            gyroY=f(frame[1]),
            gyroZ=f(frame[2]),
            accX=f(frame[3]),
            accY=f(frame[4]),
            accZ=f(frame[5]),
            tempGyro=f(frame[6]),
            sTtag=last_sttag,
            rx_mono=now
        )
        with self._lock:
            self._last = esf_raw
        if self.on_data:
            self.on_data(esf_raw)

    def get_last(self) -> Optional[EsfRawData]:
        with self._lock:
            return self._last

    def get_stats(self):
        now = time.monotonic()
        elapsed = now - self.t0
        rate = (self.count / elapsed) if elapsed > 0 else 0.0
        return self.count, elapsed, rate

    def log_last(self, print_fn=print):
        raw = self.get_last()
        if not raw:
            print_fn("[ESF-RAW] (no data)")
            return
        print_fn(raw.get_log())
