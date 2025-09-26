import time
from collections import deque

# Mapování ESF RAW typů na indexy v tuple (pořadí jsme zvolili: gyroX, gyroY, gyroZ, accX, accY, accZ, tempGyro)
# UBX ESF-RAW typy: 14:xGyro, 13:yGyro, 5:zGyro, 16:accX, 17:accY, 18:accZ, 12:tempGyro
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

def _decode_units(dtype: int, value: int) -> str:
    # rychlá jednotková konverze pro lidské logy (volá se mimo RT cesty)
    if dtype in (16, 17, 18):   # acc m/s^2 * 2^-10
        return f"{value/1024:.4f} m/s²"
    if dtype in (5, 13, 14):    # gyro deg/s * 2^-12
        return f"{value/4096:.4f} deg/s"
    if dtype == 12:             # temp °C * 1e-2
        return f"{value/100:.2f} °C"
    return f"{value} raw"

class EsfRawHandler:
    """
    Optimalizovaný handler pro UBX ESF-RAW (cca 110 Hz).

    - handle(): bez výpisů, minimální alokace
    - ukládá poslední frame (gyro/acc/temp) jako tuple + sTtag
    - get_stats(): vrátí (count, elapsed, rate)
    - last_tuple(): vrátí poslední tuple
    - log_last_tuple(): srozumitelný lidský výpis posledního tuple (volat mimo RT)
    """
    __slots__ = ("count", "t0", "context", "_maxlen", "_last_rate_ts")

    def __init__(self, max_context: int = 1024):
        self.count = 0
        self.t0 = time.monotonic()
        self._last_rate_ts = self.t0
        self._maxlen = max_context
        # context drží tuply: (gyroX, gyroY, gyroZ, accX, accY, accZ, tempGyro, sTtag)
        self.context = deque(maxlen=max_context)

    def handle(self, msg_class: int, msg_id: int, payload: bytes) -> None:
        """RT-safe: žádné printy, žádné floaty, minimum alokací."""
        self.count += 1
        if len(payload) < 4:
            return

        # Každá ESF-RAW zpráva je: [4B reserved][N * (4B data + 4B sTtag)]
        # Z parsovaných položek složíme 1 "frame" (poslední hodnoty daných typů).
        N = (len(payload) - 4) // 8
        if N <= 0:
            return

        base = 4
        frame = [None, None, None, None, None, None, None]
        last_sttag = 0

        # Lokální proměnné pro rychlost
        p = payload
        to_idx = _TYPE_TO_IDX

        for i in range(N):
            o = base + i * 8

            # 4B little endian: [dataField (24b, signed)] + [dataType (8b)]
            d = int.from_bytes(p[o:o+4], "little", signed=False)
            df = d & 0xFFFFFF

            # sign-extend 24-bit
            if df & 0x800000:
                df -= 1 << 24

            dtype = (d >> 24) & 0xFF
            last_sttag = int.from_bytes(p[o+4:o+8], "little", signed=False)

            idx = to_idx.get(dtype)
            if idx is not None:
                frame[idx] = df

        # Ulož 1 tuple za celou zprávu
        self.context.append((*frame, last_sttag))

    # --- „volá se zvenčí“, mimo RT cesty ------------------------------------

    def get_stats(self):
        """Vrátí (count, elapsed_s, avg_rate_hz). Žádný print."""
        now = time.monotonic()
        elapsed = now - self.t0
        rate = (self.count / elapsed) if elapsed > 0 else 0.0
        return self.count, elapsed, rate

    def last_tuple(self):
        """Poslední (gyroX, gyroY, gyroZ, accX, accY, accZ, tempGyro, sTtag) nebo None."""
        return self.context[-1] if self.context else None

    def snapshot_and_clear(self):
        """Vezme aktuální kontext jako list, kontext vymaže. Použij pro dávkové logování."""
        items = list(self.context)
        self.context.clear()
        return items

    def log_last_tuple(self, decode_units: bool = True, print_fn=print):
        """
        Srozumitelný výpis jednoho tuple (pro ladění/logy; volat jen občas).
        decode_units=True provede převod na jednotky, jinak vypíše raw.
        """
        t = self.last_tuple()
        if not t:
            print_fn("[ESF-RAW] (no data)")
            return

        *values, sTtag = t
        parts = []
        for idx, val in enumerate(values):
            name = _IDX_TO_NAME[idx]
            if val is None:
                parts.append(f"{name}=None")
                continue
            if decode_units:
                # pro převod potřebujeme i „dtype“; mapujeme zpět z indexu:
                # idx->dtype (inverzní mapování)
                # 0:14, 1:13, 2:5, 3:16, 4:17, 5:18, 6:12
                dtype = (14, 13, 5, 16, 17, 18, 12)[idx]
                parts.append(f"{name}={val} ({_decode_units(dtype, val)})")
            else:
                parts.append(f"{name}={val}")

        print_fn(f"[ESF-RAW] sTtag={sTtag} | " + " ".join(parts))

    def log_stats(self, print_fn=print):
        """Krátký přehled pro log (volat zvenčí třeba 1× za sekundu)."""
        count, elapsed, rate = self.get_stats()
        print_fn(f"[ESF-RAW] Count={count} Elapsed={elapsed:.1f}s AvgRate={rate:.1f} Hz")
