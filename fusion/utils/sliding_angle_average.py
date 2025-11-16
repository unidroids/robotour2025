from collections import deque
from math import radians, sin, cos, atan2, degrees
from typing import Deque, Tuple

class SlidingAngleAverage:
    """
    Klouzavý průměr rozdílu úhlů (ve stupních) přes pevné okno.
    update(a1_deg, a2_deg) bere Δ=a2-a1, převádí na (x,y) a udržuje součty.
    Výstup: (průměrný_úhel_deg, kvalita), kde kvalita ∈ <0,1>.
    """
    __slots__ = ("size", "_q", "_sx", "_sy", "ready")

    def __init__(self, size: int) -> None:
        if size <= 0:
            raise ValueError("size musí být > 0")
        self.size: int = size
        self._q: Deque[Tuple[float, float]] = deque()
        self._sx: float = 0.0
        self._sy: float = 0.0
        self.ready: bool = False

    def __len__(self) -> int:
        return len(self._q)

    def reset(self) -> None:
        self._q.clear()
        self._sx = 0.0
        self._sy = 0.0
        self.ready = False

    def update(self, a1_deg: float, a2_deg: float) -> Tuple[float, float]:
        # Δ úhel → jednotkový vektor
        theta = radians(a2_deg - a1_deg)
        x = cos(theta)
        y = sin(theta)

        # FIFO + součty (O(1))
        if len(self._q) == self.size:
            ox, oy = self._q.popleft()
            self._sx -= ox
            self._sy -= oy

        self._q.append((x, y))
        self._sx += x
        self._sy += y

        n = len(self._q)
        self.ready = (n >= self.size)

        # průměrný vektor → úhel a kvalita
        avg_x = self._sx / n
        avg_y = self._sy / n
        ang_deg = degrees(atan2(avg_y, avg_x))   # standardní pořadí atan2(y, x)
        quality = (avg_x * avg_x + avg_y * avg_y) ** 0.5
        return ang_deg, quality


# -----------------------------
# Jednoduché testy (spustitelné přímo)
# -----------------------------
def _almost(a, b, eps=1e-9):
    return abs(a - b) <= eps

def _run_tests():
    # 1) readiness + konstantní delta
    saa = SlidingAngleAverage(size=4)
    for i in range(3):
        ang, q = saa.update(0, 10)
        assert saa.ready is False
    ang, q = saa.update(0, 10)
    assert saa.ready is True
    assert _almost(ang, 10.0, 1e-9)
    assert q > 0.999999  # téměř jednotková kvalita

    # 2) wrap-around (a2-a1 = 350-10 = 340° ≡ -20°)
    saa = SlidingAngleAverage(size=3)
    for _ in range(3):
        ang, q = saa.update(10, 350)
    assert _almost(ang, -20.0, 1e-9)  # atan2 vrací v (-180, 180], takže -20°
    assert q > 0.999999

    # 3) sliding okno – vytlačení prvku
    # Sekvence Δ: [0°, 0°, 0°] → úhel 0°, kvalita ~1
    saa = SlidingAngleAverage(size=3)
    saa.update(0, 0)
    saa.update(0, 0)
    ang, q = saa.update(0, 0)
    assert _almost(ang, 0.0, 1e-9)
    assert q > 0.999999
    # Přidáme 90°, čímž vytlačíme první 0° → vektory ~ [(1,0),(1,0),(0,1)]
    ang, q = saa.update(0, 90)
    # očekávaný vektorový průměr: (2/3, 1/3) → úhel atan2(1,2) ≈ 26.565°, kvalita ≈ sqrt(5)/3
    from math import atan, sqrt
    expected_ang = degrees(atan(1/2))
    expected_q = sqrt(5)/3
    assert abs(ang - expected_ang) < 1e-9
    assert abs(q - expected_q) < 1e-12

    # 4) reset
    saa.reset()
    assert len(saa) == 0 and saa._sx == 0 and saa._sy == 0 and saa.ready is False

if __name__ == "__main__":
    _run_tests()
    print("OK")
