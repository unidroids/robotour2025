# gnss/nav_fusion.py
from __future__ import annotations
import time
import threading
from dataclasses import dataclass
from collections import deque
from typing import Deque, Optional, Tuple, List

# Později použijeme tvoji LeverArmHeading pro reálný výpočet:
# from .lever_arm_heading import LeverArmHeading

@dataclass
class RawSample:
    t_mono: float       # host monotonic time
    gyro_z_dps: float   # deg/s
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0

@dataclass
class FusionResult:
    t_mono: float
    iTOW_ms: int
    heading_deg: float      # dočasně placeholder (NaN)
    speed_mps: float
    quality: str            # "NO_DATA" | "OK" | "LOW_SPEED" | další do budoucna

class NavFusion:
    """
    Minimální kostra „NavFusion“:
      - ESF-RAW handler volá on_esf_raw(...)
      - NAV-PVAT handler volá on_nav_pvat(...)
      - wait_for_update() blokuje, než dorazí nová data (PVAT je „tick“)
      - get_latest() vrací poslední výsledek

    POZN.: Výpočet headingu zatím záměrně neděláme. Cílem je integrace a signály.
           V dalším kroku napojíme LeverArmHeading a doplníme skutečnou numeriku.
    """
    def __init__(self, raw_window_sec: float = 0.15, max_samples: int = 256):
        self._raw_window = raw_window_sec
        self._raw: Deque[RawSample] = deque(maxlen=max_samples)
        self._raw_lock = threading.Lock()

        self._latest: Optional[FusionResult] = None
        self._latest_lock = threading.Lock()
        self._cond = threading.Condition()

        # future: self._lah = LeverArmHeading(...)

    # === Volají handlery =====================================================

    def on_esf_raw(self, gyro_z_dps: float, ax: float, ay: float, az: float) -> None:
        """Rychlý append do ring bufferu (110 Hz)."""
        s = RawSample(time.monotonic(), gyro_z_dps, ax, ay, az)
        with self._raw_lock:
            self._raw.append(s)

    def on_nav_pvat(self, iTOW_ms: int, gSpeed_mps: float) -> None:
        """
        „Tick“ na 30 Hz – zde později uděláme skutečný výpočet (lever-arm).
        Prozatím jen:
          - vybereme okno posledních RAW vzorků (kvůli budoucímu výpočtu),
          - vytvoříme placeholder FusionResult a notifneme odběratele.
        """
        now = time.monotonic()

        # připravené okno pro budoucí numeriku (aktuálně nevyužijeme)
        cutoff = now - self._raw_window
        with self._raw_lock:
            _window: List[RawSample] = [s for s in self._raw if s.t_mono >= cutoff]

        # placeholder hodnoty
        speed = max(0.0, float(gSpeed_mps))
        quality = "OK" if speed > 0.05 else "LOW_SPEED"

        res = FusionResult(
            t_mono=now,
            iTOW_ms=int(iTOW_ms),
            heading_deg=float('nan'),  # skutečný výpočet doplníme příště
            speed_mps=speed,
            quality=quality,
        )
        self._publish(res)

    # === Odběratelské API ====================================================

    def get_latest(self) -> Optional[FusionResult]:
        with self._latest_lock:
            return self._latest

    def wait_for_update(self, timeout: Optional[float] = None) -> bool:
        """Blokuje do notify_all nebo do timeoutu. True = přišel update, False = timeout."""
        with self._cond:
            return self._cond.wait(timeout=timeout)

    # === Interní =============================================================

    def _publish(self, res: FusionResult) -> None:
        with self._latest_lock:
            self._latest = res
        with self._cond:
            self._cond.notify_all()
