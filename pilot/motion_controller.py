# motion_controller.py
# -----------------------------------------------------------------------------
# Mapování (v, ω) -> (left_pwm, right_pwm) pro diferenciální řízení.
#
# - v_cmd_mps      : požadovaná dopředná rychlost [m/s]
# - omega_cmd_dps  : požadovaná úhlová rychlost [°/s], +CCW (matematická konvence)
#
# Vnitřní kroky:
#  1) Omezíme v_cmd na v_max (podle režimu: ladění 0.5 m/s / ostrý 1.5 m/s).
#  2) Omezíme omega_cmd na ±omega_max (90 °/s).
#  3) Normalizace do <-1..+1>: v_norm = v/v_max, w_norm = ω/ω_max.
#  4) Diferenciální mix: left = v_norm - w_norm, right = v_norm + w_norm.
#     Pokud |left|/|right| překročí 1, reskalujeme oba zachováním poměru.
#  5) Převod <-1..+1> na PWM s deadbandem (např. 20) a max 255.
#
# Volitelně: "komfortní" obálka compute_for_near(), která z chyby headingu spočítá ω
# a ze vzdálenosti k cíli spočítá v (se zpomalováním u cíle). Hodí se jako start.
# -----------------------------------------------------------------------------

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from typing import Tuple, Optional
import math

def _wrap_deg(angle: float) -> float:
    """Zavinutí do (-180, 180]."""
    a = (angle + 180.0) % 360.0 - 180.0
    return a if a != -180.0 else 180.0

class SpeedMode(Enum):
    DEBUG = auto()   # v_max = 0.5 m/s
    NORMAL = auto()  # v_max = 1.5 m/s

@dataclass
class ControllerConfig:
    # Limity rychlostí
    v_max_debug_mps: float = 0.5
    v_max_normal_mps: float = 1.5
    omega_max_dps: float = 90.0

    # PWM mapování
    max_pwm: int = 255
    deadband_pwm: int = 20  # 15–25 dle tvého base; hodnoty < deadband nemají účinek

    # "Komfortní" řízení pro compute_for_near()
    slow_down_dist_m: float = 5.0  # pod touto vzdáleností plynule snižuj v_cmd
    k_heading_to_omega: float = 2.0  # [ (°/s) / ° ] převod chyby natočení -> ω
    v_scale: float = 0.6            # škálování v_cmd vůči v_max (0..1)

class MotionController2D:
    def __init__(self, cfg: Optional[ControllerConfig] = None, mode: SpeedMode = SpeedMode.DEBUG) -> None:
        self.cfg = cfg or ControllerConfig()
        self.mode = mode

    # -------------------------------
    # Režim rychlosti (ladění / ostrý)
    # -------------------------------
    def set_mode(self, mode: SpeedMode) -> None:
        self.mode = mode

    def _v_max(self) -> float:
        return self.cfg.v_max_debug_mps if self.mode == SpeedMode.DEBUG else self.cfg.v_max_normal_mps

    # -------------------------------
    # Jádro: (v, ω) -> (L/R PWM)
    # -------------------------------
    def mix_v_omega_to_pwm(self, v_cmd_mps: float, omega_cmd_dps: float) -> Tuple[int, int]:
        """
        Přímé mapování z požadovaných rychlostí na PWM kol.

        Doctest (základní směrová intuice):
        >>> ctrl = MotionController2D()
        >>> # Dopředně 0.5 m/s (v_max=0.5 v DEBUG) bez zatáčení => L~R~max (po deadband)
        >>> L, R = ctrl.mix_v_omega_to_pwm(0.5, 0.0)
        >>> L > 200 and R > 200
        True
        >>> # Čisté otáčení CCW (ω>0) bez dopředné => L<0, R>0
        >>> L2, R2 = ctrl.mix_v_omega_to_pwm(0.0, 60.0)
        >>> L2 < 0 and R2 > 0
        True
        """
        # 1) Limity
        v_max = max(1e-6, self._v_max())
        w_max = max(1e-6, self.cfg.omega_max_dps)
        v_cmd = max(-v_max, min(v_max, v_cmd_mps))
        w_cmd = max(-w_max, min(w_max, omega_cmd_dps))

        # 2) Normalizace
        v_norm = v_cmd / v_max      # [-1..1]
        w_norm = w_cmd / w_max      # [-1..1]

        # 3) Diferenciální mix
        left = v_norm - w_norm
        right = v_norm + w_norm

        # 4) Rescale při přesahu
        m = max(1.0, abs(left), abs(right))
        left /= m
        right /= m

        # 5) Na PWM s deadbandem
        left_pwm = self._norm_to_pwm(left)
        right_pwm = self._norm_to_pwm(right)
        return left_pwm, right_pwm

    def _norm_to_pwm(self, n: float) -> int:
        n = max(-1.0, min(1.0, n))
        if abs(n) < 1e-6:
            return 0
        span = self.cfg.max_pwm - self.cfg.deadband_pwm
        if n > 0:
            return int(round(self.cfg.deadband_pwm + n * span))
        else:
            return -int(round(self.cfg.deadband_pwm + (-n) * span))

    # -------------------------------
    # Komfortní obálka: řízení k near bodu
    # -------------------------------
    def compute_for_near(
        self,
        heading_enu_deg: float,
        near_x_m: float,
        near_y_m: float,
        allow_forward: bool,
        allow_spin: bool,
        dist_to_goal_m: float,
        goal_radius_m: float,
    ) -> Tuple[int, int, str]:
        """
        Vypočítá PWM tak, aby robot mířil k near (v ENU), s respektem k limitům a povolením FSM.

        - Pokud allow_forward=False -> dopředná složka v=0.
        - Pokud allow_spin=False    -> ω=0 (jen dopředně bez otáčení).

        Strategii lze později vyměnit; tohle je čitelný baseline.

        Doctest (jen logika směru/limitů, ne na fyz. kalibraci):
        >>> ctrl = MotionController2D()
        >>> # Robot míří na východ (0° ENU), near je na sever (0, +1) => chyba +90° => CCW spin
        >>> L, R, st = ctrl.compute_for_near(heading_enu_deg=0.0, near_x_m=0.0, near_y_m=1.0,
        ...                                  allow_forward=False, allow_spin=True,
        ...                                  dist_to_goal_m=10.0, goal_radius_m=2.0)
        >>> L < 0 and R > 0 and "SPIN" in st
        True
        """
        # 1) Desired heading k near
        desired_deg = math.degrees(math.atan2(near_y_m, near_x_m))  # ENU: 0=E, 90=N
        err_deg = _wrap_deg(desired_deg - heading_enu_deg)

        # 2) ω z chyby natočení
        omega_cmd = self.cfg.k_heading_to_omega * err_deg  # [°/s]
        # limity + případné zákazy
        if not allow_spin:
            omega_cmd = 0.0
        else:
            omega_cmd = max(-self.cfg.omega_max_dps, min(self.cfg.omega_max_dps, omega_cmd))

        # 3) v podle vzdálenosti (plynule zpomal u cíle)
        if not allow_forward:
            v_cmd = 0.0
            mode = "SPIN_ONLY"
        else:
            if dist_to_goal_m <= goal_radius_m:
                v_cmd = 0.0
                mode = "GOAL_RADIUS"
            else:
                if dist_to_goal_m < self.cfg.slow_down_dist_m:
                    v_cmd = self.cfg.v_scale * (dist_to_goal_m / self.cfg.slow_down_dist_m) * self._v_max()
                else:
                    v_cmd = self.cfg.v_scale * self._v_max()
                mode = "NAV"

        # 4) Namixuj
        left_pwm, right_pwm = self.mix_v_omega_to_pwm(v_cmd, omega_cmd)
        status = f"{mode}: err={err_deg:.1f}deg v={v_cmd:.2f}m/s w={omega_cmd:.1f}dps -> pwm=({left_pwm},{right_pwm})"
        return left_pwm, right_pwm, status
