# gyro_smoother.py
# Plně kauzální vyhlazení úhlové rychlosti z IMU (110 Hz) s důrazem na poslední vzorek.
# Volitelné: adaptivní One-Euro chování (rychlé reakce při prudkých změnách).
# Použití:
#   f = GyroRateSmoother(sample_rate_hz=110.0, min_cutoff_hz=8.0, beta=0.5)
#   for t, wz in stream:   # wz v °/s (nebo rad/s – jednotky misí stejné!)
#       smoothed = f.update(wz, t)   # t v sekundách (float, monotónně rostoucí)
#   # Každých 1/30 s (PVAT) si prostě vezmi f.last

from dataclasses import dataclass
import math
from typing import Optional

def _alpha_from_cutoff(fc_hz: float, dt: float) -> float:
    """
    Převod cutoff frekvence fc na koeficient EMA (One-Euro standard).
    τ = 1/(2πfc); alpha = 1 / (1 + τ/dt)
    """
    if fc_hz <= 0.0:
        return 1.0  # žádné filtrování (plně propustné)
    tau = 1.0 / (2.0 * math.pi * fc_hz)
    return 1.0 / (1.0 + tau / max(dt, 1e-6))

@dataclass
class GyroRateSmoother:
    """
    EMA / One-Euro filtr pro úhlovou rychlost.
    - min_cutoff_hz: základní „pomalu hladící“ cutoff (8–10 Hz pro 110 Hz IMU je dobrý start).
    - beta: jak moc se cutoff zvýší při rychlých změnách (0 = čistá EMA; 0.2–0.7 = adaptivní).
    - deriv_cutoff_hz: vyhlazení odhadu derivace (jerk), aby adaptace nebyla nervózní.
    - clamp_abs_dps: volitelné omezení |ω| kvůli outlierům / saturaci.
    Jednotky: je jedno zda °/s nebo rad/s, jen to nemíchat.
    """
    sample_rate_hz: float = 110.0
    min_cutoff_hz: float = 8.0
    beta: float = 0.5
    deriv_cutoff_hz: float = 10.0
    clamp_abs_dps: Optional[float] = 720.0  # None = bez omezení

    # interní stav
    _x_prev: Optional[float] = None
    _dx_prev: float = 0.0
    _y_prev: Optional[float] = None
    _t_prev: Optional[float] = None

    # poslední publikovaná hodnota (pro snadné čtení v 30 Hz smyčce)
    last: float = 0.0

    def reset(self) -> None:
        self._x_prev = None
        self._dx_prev = 0.0
        self._y_prev = None
        self._t_prev = None
        self.last = 0.0

    def update(self, x: float, t: Optional[float] = None) -> float:
        """
        Přijme nový „raw“ vzorek úhlové rychlosti x a vrátí vyhlazený výstup.
        t – volitelný čas v sekundách (float). Když není, použije 1/sample_rate_hz.
        """
        # Anti-outlier clamp
        if self.clamp_abs_dps is not None:
            a = self.clamp_abs_dps
            if x > a: x = a
            elif x < -a: x = -a

        if self._t_prev is None:
            # První vzorek – jen inicializace
            self._t_prev = 0.0 if t is None else float(t)
            self._x_prev = x
            self._y_prev = x
            self._dx_prev = 0.0
            self.last = x
            return x

        # Δt
        if t is None:
            dt = 1.0 / max(self.sample_rate_hz, 1e-6)
            self._t_prev += dt
        else:
            dt = max(float(t) - self._t_prev, 1e-6)  # brání dělení nulou i časovým šumům
            self._t_prev = float(t)

        # --- One-Euro: nejdřív odhad derivace (jerku) a jeho vyhlazení ---
        # Pro čistou EMA stačí beta=0. Nic dalšího netřeba měnit.
        dx = (x - (self._x_prev if self._x_prev is not None else x)) / dt
        alpha_d = _alpha_from_cutoff(self.deriv_cutoff_hz, dt)
        dx_hat = self._dx_prev + alpha_d * (dx - self._dx_prev)

        # Adaptivní cutoff – zvýšíme při rychlých změnách (|dx_hat|).
        fc = self.min_cutoff_hz + self.beta * abs(dx_hat)

        # Low-pass vstupu s dynamickým alpha
        alpha = _alpha_from_cutoff(fc, dt)
        y = (self._y_prev if self._y_prev is not None else x) + alpha * (x - (self._y_prev if self._y_prev is not None else x))

        # Ulož stav
        self._x_prev = x
        self._y_prev = y
        self._dx_prev = dx_hat
        self.last = y
        return y

    # Volitelné pomocníky pro rozhodování PILOTu:
    def started_rotating(self, threshold: float = 8.0) -> bool:
        """Jednoduché prahování |ω| > threshold (°/s). Pro robustnost řeš hysterézi ve volající logice."""
        return abs(self.last) > threshold

    def too_fast(self, limit: float = 90.0) -> bool:
        """Překročena bezpečná rychlost otáčení (°/s)."""
        return abs(self.last) > limit
