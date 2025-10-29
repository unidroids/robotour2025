import math
from dataclasses import dataclass, field
from typing import Tuple, Optional

@dataclass
class PPVelocityPlanner:
    """
    Pure-Pursuit based velocity planner (diff-drive).
    Units:
      - a_y_max: m/s^2 (limit bočního zrychlení)
      - L: m (lookahead)
      - b: m (rozteč kol)
      - max_speed_cm_s: cm/s (limit rychlosti středu, tj. (vL+vR)/2)
      - min_wheel_speed_cm_s: cm/s (volitelné min. rychlost každého kola; default 0 = bez minima)
      - min_turn_radius_m: m (volitelný minimální poloměr zatáčení; default b/2 = obě kola vpřed)

    `calculate(alpha_deg)`:
      - alpha_deg: [-90, +90], CCW > 0, CW < 0
      - vrací (vL_cm_s, vR_cm_s)
    """
    a_y_max: float                 # m/s^2
    L: float                       # m
    b: float                       # m
    max_speed_cm_s: float          # cm/s (center speed limit)
    min_wheel_speed_cm_s: float = 0.0     # cm/s
    min_turn_radius_m: Optional[float] = None  # m (None => b/2)

    # internal cached values
    _alpha_max_deg: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        # basic parameter checks
        if self.a_y_max <= 0:
            raise ValueError("a_y_max must be > 0 m/s^2.")
        if self.L <= 0:
            raise ValueError("Lookahead L must be > 0 m.")
        if self.b <= 0:
            raise ValueError("Track width b must be > 0 m.")
        if self.max_speed_cm_s <= 0:
            raise ValueError("max_speed_cm_s must be > 0 cm/s.")
        if self.min_wheel_speed_cm_s < 0:
            raise ValueError("min_wheel_speed_cm_s must be >= 0 cm/s.")

        # default min_turn_radius = b/2 (obě kola vpřed)
        if self.min_turn_radius_m is None:
            self.min_turn_radius_m = self.b / 2.0
        if self.min_turn_radius_m < self.b / 2.0:
            raise ValueError(
                f"min_turn_radius_m must be >= b/2 ({self.b/2:.3f} m) to keep both wheels forward."
            )

        self._refresh_cache()

    # --------- public API ---------

    def set_params(
        self,
        *,
        a_y_max: Optional[float] = None,
        L: Optional[float] = None,
        b: Optional[float] = None,
        max_speed_cm_s: Optional[float] = None,
        min_wheel_speed_cm_s: Optional[float] = None,
        min_turn_radius_m: Optional[float] = None,
    ) -> None:
        """Update any subset of parameters; validates and updates derived limits."""
        if a_y_max is not None:
            if a_y_max <= 0:
                raise ValueError("a_y_max must be > 0 m/s^2.")
            self.a_y_max = a_y_max
        if L is not None:
            if L <= 0:
                raise ValueError("Lookahead L must be > 0 m.")
            self.L = L
        if b is not None:
            if b <= 0:
                raise ValueError("Track width b must be > 0 m.")
            self.b = b
        if max_speed_cm_s is not None:
            if max_speed_cm_s <= 0:
                raise ValueError("max_speed_cm_s must be > 0 cm/s.")
            self.max_speed_cm_s = max_speed_cm_s
        if min_wheel_speed_cm_s is not None:
            if min_wheel_speed_cm_s < 0:
                raise ValueError("min_wheel_speed_cm_s must be >= 0 cm/s.")
            self.min_wheel_speed_cm_s = min_wheel_speed_cm_s
        if min_turn_radius_m is not None:
            if min_turn_radius_m < self.b / 2.0:
                raise ValueError(
                    f"min_turn_radius_m must be >= b/2 ({self.b/2:.3f} m) to keep both wheels forward."
                )
            self.min_turn_radius_m = min_turn_radius_m

        self._refresh_cache()

    def calculate(self, alpha_deg: float) -> Tuple[float, float]:
        """
        Compute wheel speeds (vL, vR) in cm/s for a given heading error alpha (deg, CCW positive).
        Raises ValueError if requested alpha violates reachable curvature (min radius/forward-only)
        or if the inner wheel would drop below `min_wheel_speed_cm_s`.
        """
        #alpha_deg = -alpha_deg
        if not (-90.0 <= alpha_deg <= 90.0):
            raise ValueError("alpha must be in [-90, +90] degrees.")

        # Check radius constraint for this alpha
        if abs(alpha_deg) > self._alpha_max_deg + 1e-12:
            R_eff = max(self.min_turn_radius_m, self.b / 2.0)
            raise ValueError(
                f"|alpha|={abs(alpha_deg):.2f}° exceeds allowed {self._alpha_max_deg:.2f}° "
                f"for current limits (min radius = {R_eff:.3f} m, lookahead L = {self.L:.3f} m)."
            )

        # curvature from Pure Pursuit: kappa = 2*sin(alpha)/L
        alpha_rad = math.radians(alpha_deg)
        kappa = 2.0 * math.sin(alpha_rad) / self.L  # can be negative (CW)

        # safety: forward-only curvature bound |kappa| <= 2/b (R >= b/2)
        kappa_fwd_max = 2.0 / self.b
        if abs(kappa) > kappa_fwd_max + 1e-12:
            raise ValueError(
                f"Curvature |kappa|={abs(kappa):.3f} 1/m exceeds forward-only limit {kappa_fwd_max:.3f} (R>=b/2)."
            )

        # center speed limit: a_y and max_speed
        v_max_center_m_s = self.max_speed_cm_s / 100.0
        if abs(kappa) > 0:
            v_from_ay = math.sqrt(self.a_y_max / abs(kappa))
            v = min(v_max_center_m_s, v_from_ay)
        else:
            v = v_max_center_m_s  # alpha ~ 0 => no lateral limit

        # wheel speeds (m/s)
        half_b_k = 0.5 * self.b * kappa
        vR = v * (1.0 + half_b_k)
        vL = v * (1.0 - half_b_k)

        # forward-only (both >= 0); tiny negative due to numerics is clipped to 0
        if vL < -1e-9 or vR < -1e-9:
            raise ValueError(
                "Requested alpha would require reversing one wheel (forward-only mode is enforced)."
            )

        # minimum wheel speed check (if any)
        vL_cm = max(0.0, vL * 100.0)
        vR_cm = max(0.0, vR * 100.0)
        vmin = self.min_wheel_speed_cm_s
        if vmin > 0.0:
            inner = "L" if abs(1.0 - half_b_k) < abs(1.0 + half_b_k) else "R"
            inner_speed = vL_cm if inner == "L" else vR_cm
            if inner_speed + 1e-9 < vmin:
                # what center speed would be needed to reach vmin on the inner wheel?
                denom = (1.0 - abs(half_b_k))  # inner factor
                need_center = (vmin / 100.0) / max(denom, 1e-12)
                msg = (
                    f"Inner wheel {inner} would be {inner_speed:.1f} cm/s < min {vmin:.1f} cm/s "
                    f"at alpha={alpha_deg:.2f}°. Needed center speed ≥ {need_center*100:.1f} cm/s, "
                    f"but limited to {v_max_center_m_s*100:.1f} cm/s and a_y-limit."
                )
                raise ValueError(msg)

        return (vL_cm, vR_cm)

    # --------- internals ---------

    def _refresh_cache(self) -> None:
        """Recompute the maximum admissible |alpha| from radius constraints."""
        # Effective minimal radius: respect both user min_turn_radius and forward-only (b/2).
        R_eff = max(self.min_turn_radius_m, self.b / 2.0)

        # With PP (kappa = 2 sin α / L), the smallest radius over |α|<=90° is L/2.
        # Enforce R >= R_eff  =>  |sin α| <= L/(2 R_eff)
        arg = self.L / (2.0 * R_eff)
        arg = max(0.0, min(1.0, arg))  # clamp to [0,1]
        self._alpha_max_deg = math.degrees(math.asin(arg))

if __name__ == "__main__":
    # simple test
    planner = PPVelocityPlanner(
        a_y_max=0.5,
        L=1.0,
        b=0.58,
        max_speed_cm_s=50.0,
        min_wheel_speed_cm_s=20.0,
        min_turn_radius_m=0.29,
    )

    test_alphas = [-90, -45, -20, 0, 20, 45, 90]
    for alpha in test_alphas:
        try:
            vL, vR = planner.calculate(alpha)
            print(f"alpha={alpha:6}° => vL={vL:6.1f} cm/s, vR={vR:6.1f} cm/s")
        except ValueError as e:
            print(f"alpha={alpha:6}° => ERROR: {e}")

    pp = PPVelocityPlanner(
        a_y_max=0.2,   # m/s^2
        L=2.0,         # m
        b=0.58,        # m
        max_speed_cm_s=150.0,     # cm/s
        min_wheel_speed_cm_s=20.0 # cm/s (volitelně)
        # min_turn_radius_m default = b/2
    )

    # výpočet pro několik úhlů (vrací cm/s)
    for a in [5, 15, 30, -20]:
        try:
            vL, vR = pp.calculate(a)
            print(a, "deg -> vL=", round(vL,1), "cm/s,", "vR=", round(vR,1), "cm/s")
        except ValueError as e:
            print(a, "deg ->", e)

    # změna lookaheadu za běhu (validace proběhne uvnitř)
    pp.set_params(L=1.0)            