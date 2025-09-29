# lever_arm_heading.py

import math
from dataclasses import dataclass
from typing import Tuple

def wrap_angle(a: float) -> float:
    """Zalomí úhel do (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a if a > -math.pi else a + 2.0 * math.pi

@dataclass(frozen=True)
class LeverArmHeading:
    """
    Výpočet headingu vozidla (θ) z GNSS rychlosti antény a yaw-rate IMU pro libovolnou
    páčku r = [r_x, r_y] v TĚLESOVÝCH osách (x dopředu, y doleva, z nahoru).

    SIGN KONVENCE (důležité):
    - r_x [m]: kladně DO PŘEDU od středu otáčení (C).
    - r_y [m]: kladně DOLEVA od středu otáčení (C). (Anténa vpravo ⇒ r_y < 0.)
    - α [rad]: směr rychlosti antény v GLOBÁLNÍ rovině, měřený STEJNOU konvencí jako θ
               (např. ENU: 0 = +X/East, kladně proti směru hodinových ručiček).
    - s [m/s]: velikost vektoru rychlosti antény (||v_A||) z GNSS.
    - ω [rad/s]: yaw-rate z IMU. ω > 0 = rotace proti směru hodinových ručiček (CCW) kolem +Z (nahoru).
    - Všechny úhly v radiánech.
    """

    r_x: float  # [m] páčka dopředu (+), dozadu (-)
    r_y: float  # [m] páčka doleva (+), doprava (-)
    speed_eps: float = 1e-6
    omega_eps: float = 1e-6

    def theta_from_motHeading_deg(
        self,
        motHeading_deg: float,   # [°] GNSS heading (0=N, 90=E)
        speed: float,            # [m/s]
        omega_deg: float,        # [°/s] yaw-rate, +CCW
        allow_reverse: bool = False
    ) -> Tuple[float, float]:
        """
        Wrapper: 
        - Převádí GNSS motHeading (azimut, 0=N, 90=E, 180=S, 270=W) na ENU konvenci (0=E, 90=N).
        - Výstup (theta) vždy v rozsahu 0–360° (jako vehHeading).
        - Vrací (theta_deg_0_360, v_center).
        """
        # GNSS motHeading (azimut) na ENU konvenci (0=E, 90=N):
        alpha_deg_enu = (90.0 - motHeading_deg) % 360.0
        alpha_rad = math.radians(alpha_deg_enu)
        omega_rad = math.radians(omega_deg)
        theta_rad, v_center = self.theta_from_alpha_speed(alpha_rad, speed, omega_rad, allow_reverse)
        theta_deg = math.degrees(theta_rad)
        # Výstup pro logy/export:
        theta_deg_0_360 = theta_deg % 360.0
        return theta_deg_0_360, v_center
 

    def theta_from_alpha_speed(
        self,
        alpha: float,      # [rad] kurz (směr) v_A
        speed: float,      # [m/s] velikost v_A (||v_A||)
        omega: float,      # [rad/s] yaw-rate, +CCW
        allow_reverse: bool = False
    ) -> Tuple[float, float]:
        """
        Vrátí (theta, v_center):
        - theta [rad]: heading STŘEDU vozidla (C), zalomený do (-pi, pi]
        - v_center [m/s]: dopředná rychlost středu (kladně = dopředu)

        Vstupy:
        - α (alpha): směr rychlosti antény v globálním rámci (stejná konvence jako θ)
        - speed: velikost rychlosti antény (||v_A||)
        - ω (omega): yaw-rate z IMU (+CCW)
        - allow_reverse: povolit reverzní jízdu (vybere se fyzikálně konzistentní kořen,
                         i když vyjde v<0). Pokud False, algoritmus volí ne-negativní v,
                         případně 0 při nejednoznačnosti.

        Hrany:
        - Pokud speed^2 < (ω r_x)^2 (nekonzistence / šum), kořen se ořízne na 0.
        - Pokud ω je velmi malé a speed je malé, přejde se na limitní režim „jen vektor v_A“
          (θ ≈ α) – čistě translační pohyb bez rotace.
        - Pokud speed malé a |ω| velké (spin), použije se speciální „spin“ formulace.
        """
        rx, ry = self.r_x, self.r_y
        s = max(float(speed), 0.0)
        w = float(omega)
        a = wrap_angle(float(alpha))

        # a) Čistý spin (v ~ 0): rychlost antény je tečná, θ odvodíme přímo z α a r.
        if s < self.speed_eps and abs(w) > self.omega_eps:
            phi = math.atan2(rx, -ry)
            if w < 0.0:
                phi = wrap_angle(phi + math.pi)
            theta = wrap_angle(a - phi)
            v_center = 0.0
            return theta, v_center

        # b) Téměř bez rotace (ω ~ 0): α je dobrá aproximace θ, v ≈ s.
        if abs(w) <= self.omega_eps:
            theta = a
            v_center = s
            if not allow_reverse:
                v_center = max(v_center, 0.0)
            return theta, v_center

        # Obecný případ
        term = s * s - (w * rx) * (w * rx)
        if term < 0.0:
            root = 0.0
        else:
            root = math.sqrt(term)

        v_plus  = w * ry + root
        v_minus = w * ry - root

        if allow_reverse:
            # Vyber kořen s menší korekcí (heuristika)
            phi_plus  = math.atan2(w * rx, v_plus  - w * ry)
            phi_minus = math.atan2(w * rx, v_minus - w * ry)
            err_plus  = abs(wrap_angle(a - (a - phi_plus)))
            err_minus = abs(wrap_angle(a - (a - phi_minus)))
            v = v_plus if err_plus <= err_minus else v_minus
        else:
            if v_plus >= 0.0 and v_minus >= 0.0:
                v = min(v_plus, v_minus)
            elif v_plus >= 0.0:
                v = v_plus
            elif v_minus >= 0.0:
                v = v_minus
            else:
                v = 0.0

        denom = v - w * ry
        phi = math.atan2(w * rx, denom)
        theta = wrap_angle(a - phi)
        return theta, v

    def theta_from_velocity_vector(
        self,
        v_ax: float,
        v_ay: float,
        omega: float,
        allow_reverse: bool = False
    ) -> Tuple[float, float, float]:
        alpha = math.atan2(v_ay, v_ax)
        speed = math.hypot(v_ax, v_ay)
        theta, v_center = self.theta_from_alpha_speed(alpha, speed, omega, allow_reverse=allow_reverse)
        return theta, v_center, wrap_angle(alpha)

# ---------- TESTOVACÍ BLOK -----------
if __name__ == "__main__":
    def deg(a_rad): return math.degrees(a_rad)
    def rad(a_deg): return math.radians(a_deg)

    def print_heading_case(desc, lah, alpha_deg, speed, omega_deg, expect=None):
        alpha = rad(alpha_deg)
        omega = rad(omega_deg)
        theta, v_center = lah.theta_from_alpha_speed(alpha, speed, omega)
        print(f"{desc}:")
        print(f"  input:  alpha={alpha_deg:.2f}°  speed={speed:.3f} m/s  omega={omega_deg:.2f}°/s")
        print(f"  result: theta={deg(theta):.2f}°  v_center={v_center:.3f} m/s")
        if expect:
            theta_exp, v_exp = expect
            print(f"  expect: theta={theta_exp:.2f}°  v_center={v_exp:.3f} m/s")
        print("")

    print("\n==== LeverArmHeading SELF-TEST ====\n")

    lah1 = LeverArmHeading(r_x=0.30, r_y=0.00)
    print_heading_case(
        desc="Dopředu, spin CCW (omega=+90°/s)",
        lah=lah1,
        alpha_deg=90.0,
        speed=0.471,
        omega_deg=90.0,
        expect=(0.0, 0.0)
    )
    print_heading_case(
        desc="Dopředu, spin CW (omega=-90°/s)",
        lah=lah1,
        alpha_deg=-90.0,
        speed=0.471,
        omega_deg=-90.0,
        expect=(0.0, 0.0)
    )
    print_heading_case(
        desc="Dopředu, jízda vpřed (omega=0)",
        lah=lah1,
        alpha_deg=0.0,
        speed=1.0,
        omega_deg=0.0,
        expect=(0.0, 1.0)
    )
    print_heading_case(
        desc="Dopředu, jízda vpřed a zatáčím vlevo (omega=+45°/s)",
        lah=lah1,
        alpha_deg=15.0,
        speed=1.03,
        omega_deg=45.0
    )

    lah2 = LeverArmHeading(r_x=0.0, r_y=-0.30)
    print_heading_case(
        desc="Vpravo, spin CCW (omega=+90°/s)",
        lah=lah2,
        alpha_deg=0.0,
        speed=0.471,
        omega_deg=90.0,
        expect=(0.0, 0.0)
    )
    print_heading_case(
        desc="Vpravo, spin CW (omega=-90°/s)",
        lah=lah2,
        alpha_deg=180.0,
        speed=0.471,
        omega_deg=-90.0,
        expect=(0.0, 0.0)
    )
    print_heading_case(
        desc="Vpravo, jízda vpřed (omega=0)",
        lah=lah2,
        alpha_deg=0.0,
        speed=1.0,
        omega_deg=0.0,
        expect=(0.0, 1.0)
    )
    print_heading_case(
        desc="Vpravo, jízda vpřed a zatáčím vlevo (omega=+45°/s)",
        lah=lah2,
        alpha_deg=5.0,
        speed=1.05,
        omega_deg=45.0
    )

    lah3 = LeverArmHeading(r_x=0.25, r_y=0.20)
    print_heading_case(
        desc="Diagonálně (0.25 vpřed, 0.20 vlevo), spin CCW",
        lah=lah3,
        alpha_deg=128.66,
        speed=0.393,
        omega_deg=90.0
    )
