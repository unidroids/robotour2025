# near_waypoint_class.py
# -----------------------------------------------------------------------------
# Objektová varianta "near waypoint" podle zadání.
# 
# - Pracujeme s PŘÍMKOU procházející S–E (bez ořezu na segment).
# - V rámci __init__ se uloží S, E (včetně ECEF cache). L_near_m je volitelný.
# - Vypočítává se:
#     * signed vzdálenost k cíli (m): (L_seg - t_proj),
#       kde t_proj je projekce z R na směr S→E v ENU(R).
#       Je záporná, pokud průmět leží ZA E (ve směru od S k E dále).
#     * heading k nejbližšímu průsečíku (pokud existuje)
#       → vracíme v GNSS konvenci: 0°=Sever, 90°=Východ.
#     * case: "TWO_INTERSECTIONS" | "TANGENT" | None
#       (None znamená, že průsečík neexistuje NEBO L_near_m nebyl zadán.)
# - Metoda update(lat, lon) přepočte výše uvedené hodnoty.
# - S/E je možné udržovat v ECEF pro drobnou optimalizaci.
# 
# Pozn.: Převody LLA<->ECEF<->ENU jsou v geo_utils.py (dodané uživatelem).
# -----------------------------------------------------------------------------

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Literal
import math

from geo_utils import (
    lla_to_ecef, ecef_to_lla,
    ecef_to_enu, enu_to_ecef,
    heading_enu_to_gnss,
)

NearCase = Literal["TWO_INTERSECTIONS", "TANGENT"]

@dataclass
class NearState:
    # Výstupy výpočtu pro aktuální (R_lat, R_lon)
    distance_to_goal_m: float                    # podepsaná vzdálenost k E (projekce na S→E)
    abs_distance_to_goal_m: float                # skutečná euklidovská vzdálenost R→E
    heading_to_near_gnss_deg: Optional[float]    # 0=N, 90=E; None pokud není průsečík
    case: Optional[NearCase]                     # None pokud není průsečík
    # (volitelně užitečná telemetrie)
    near_lat: Optional[float] = None
    near_lon: Optional[float] = None
    near_x_m: Optional[float] = None             # ENU(R)
    near_y_m: Optional[float] = None             # ENU(R)
    d_perp_m: Optional[float] = None             # kolmice z R na přímku S–E             # kolmice z R na přímku S–E


class NearWaypoint:
    def __init__(
        self,
        S_lat: float, S_lon: float,
        E_lat: float, E_lon: float,
        L_near_m: Optional[float] = 1.0,
        eps_m: float = 2e-3,
    ) -> None:
        self.S_lat = float(S_lat)
        self.S_lon = float(S_lon)
        self.E_lat = float(E_lat)
        self.E_lon = float(E_lon)
        self.L_near_m = float(L_near_m) if L_near_m is not None else None
        self.eps_m = float(eps_m)

        # ECEF cache pro S a E (optimalizace)
        self._S_ecef = lla_to_ecef(self.S_lat, self.S_lon, 0.0)
        self._E_ecef = lla_to_ecef(self.E_lat, self.E_lon, 0.0)

        # Místo pro poslední stav
        self.state: Optional[NearState] = None

    # -------------------------------
    # Vnitřní výpočet pro dané R
    # -------------------------------
    def _compute(self, R_lat: float, R_lon: float) -> NearState:
        # S, E do ENU se vztahem v R
        Sx, Sy, _ = ecef_to_enu(*self._S_ecef, R_lat, R_lon, 0.0)
        Ex, Ey, _ = ecef_to_enu(*self._E_ecef, R_lat, R_lon, 0.0)

        # euklidovská vzdálenost k cíli v ENU
        abs_dist_goal = math.hypot(Ex, Ey)

        # směr přímky S->E
        vx, vy = Ex - Sx, Ey - Sy
        L_seg = math.hypot(vx, vy)
        if L_seg < 1e-12:
            # degenerace: S≈E ⇒ přímka nedef.
            # vzdálenost k cíli = vzdálenost R k E (v ENU ~ k S)
            dist_goal = math.hypot(Ex, Ey)  # ~ vzdálenost k bodu E
            return NearState(
                distance_to_goal_m=dist_goal,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=None,
                case=None,
                near_lat=None, near_lon=None,
                near_x_m=None, near_y_m=None,
                d_perp_m=None,
            )

        # normalizace směru
        vx /= L_seg
        vy /= L_seg

        # pata kolmice Q z R(0,0) na přímku S + t*v
        t_q = (-(Sx * vx + Sy * vy))
        Qx = Sx + t_q * vx
        Qy = Sy + t_q * vy
        d_perp = math.hypot(Qx, Qy)

        # Podepsaná vzdálenost k cíli (E) podél přímky:
        #   t_q je "kolmá projekce" R na směr od S; E je v parametru L_seg.
        distance_to_goal_m = (L_seg - t_q)

        # Pokud L_near není zadán, nepočítáme průsečík (case=None)
        if self.L_near_m is None:
            return NearState(
                distance_to_goal_m=distance_to_goal_m,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=None,
                case=None,
                near_lat=None, near_lon=None,
                near_x_m=None, near_y_m=None,
                d_perp_m=d_perp,
            )

        Lr = self.L_near_m
        eps = self.eps_m

        if d_perp > Lr + eps:
            # žádný průsečík
            return NearState(
                distance_to_goal_m=distance_to_goal_m,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=None,
                case=None,
                near_lat=None, near_lon=None,
                near_x_m=None, near_y_m=None,
                d_perp_m=d_perp,
            )
        elif abs(d_perp - Lr) <= eps:
            # tečna: near = Q
            nx, ny = Qx, Qy
            nx_ecef, ny_ecef, nz_ecef = enu_to_ecef(nx, ny, 0.0, R_lat, R_lon, 0.0)
            nlat, nlon, _ = ecef_to_lla(nx_ecef, ny_ecef, nz_ecef)
            heading_enu = math.degrees(math.atan2(ny, nx)) % 360.0
            heading_gnss = heading_enu_to_gnss(heading_enu)
            return NearState(
                distance_to_goal_m=distance_to_goal_m,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=heading_gnss,
                case="TANGENT",
                near_lat=nlat, near_lon=nlon,
                near_x_m=nx, near_y_m=ny,
                d_perp_m=d_perp,
            )
        else:
            # 2 průsečíky: N1 = Q + delta*v, N2 = Q - delta*v
            delta = math.sqrt(max(0.0, Lr * Lr - d_perp * d_perp))
            n1x, n1y = Qx + delta * vx, Qy + delta * vy
            n2x, n2y = Qx - delta * vx, Qy - delta * vy
            # Vybereme ten "blíže k E" => větší projekce na v (od S)
            t1 = ((n1x - Sx) * vx + (n1y - Sy) * vy)
            t2 = ((n2x - Sx) * vx + (n2y - Sy) * vy)
            if t1 >= t2:
                nx, ny = n1x, n1y
            else:
                nx, ny = n2x, n2y

            nx_ecef, ny_ecef, nz_ecef = enu_to_ecef(nx, ny, 0.0, R_lat, R_lon, 0.0)
            nlat, nlon, _ = ecef_to_lla(nx_ecef, ny_ecef, nz_ecef)
            heading_enu = math.degrees(math.atan2(ny, nx)) % 360.0
            heading_gnss = heading_enu_to_gnss(heading_enu)
            return NearState(
                distance_to_goal_m=distance_to_goal_m,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=heading_gnss,
                case="TWO_INTERSECTIONS",
                near_lat=nlat, near_lon=nlon,
                near_x_m=nx, near_y_m=ny,
                d_perp_m=d_perp,
            )

    # -------------------------------
    # Veřejné API
    # -------------------------------
    def update(self, R_lat: float, R_lon: float) -> tuple[float, Optional[float]]:
        """
        Přepočte a vrátí aktuální stav pro polohu (R_lat, R_lon).
        Hodnoty jsou také dostupné jako self.state.
        """
        s = self._compute(R_lat, R_lon)
        self.state = s
        return (s.distance_to_goal_m, s.heading_to_near_gnss_deg)        


# -------------------------------
# Jednoduché testy / ukázky použití
# -------------------------------
if __name__ == "__main__":
    def show(label: str, st: NearState):
        print(f"\n[{label}]\n"
              f"  distance_to_goal_m      : {st.distance_to_goal_m:.3f}\n"
              f"  abs_distance_to_goal_m: {st.abs_distance_to_goal_m:.3f}\n"
              f"  heading_gnss_deg   : {st.heading_to_near_gnss_deg}\n"
              f"  case               : {st.case}\n"
              f"  near(ENU)          : ({st.near_x_m}, {st.near_y_m})\n"
              f"  near(LL)           : ({st.near_lat}, {st.near_lon})\n"
              f"  d_perp_m           : {st.d_perp_m}")

    # Společná základna
    R = (50.000000, 14.000000)
    L = 1.0  # L_near_m

    # 0) DVA PRŮSEČÍKY: vodorovná přímka skrz R, E napravo
    cls = NearWaypoint(S_lat=R[0], S_lon=R[1],
                       E_lat=R[0], E_lon=R[1]+0.0002,
                       L_near_m=L)
    st = cls.update(R_lat=R[0], R_lon=R[1])
    show("na startu", cls.state)    
    print(st)


    # 1) DVA PRŮSEČÍKY: vodorovná přímka skrz R, E napravo
    cls = NearWaypoint(S_lat=R[0], S_lon=R[1]-0.0002,
                       E_lat=R[0], E_lon=R[1]+0.0002,
                       L_near_m=L)
    st = cls.update(R_lat=R[0], R_lon=R[1])
    show("two intersections", cls.state)
    print(st)

    # 2) TEČNA: přímka posunutá o +1 m v osy ENU-y (Sever)
    cls2 = NearWaypoint(S_lat=R[0] + (1.0/111_132.954), S_lon=R[1]-0.0002,
                        E_lat=R[0] + (1.0/111_132.954), E_lon=R[1]+0.0002,
                        L_near_m=L)
    st2 = cls2.update(R_lat=R[0], R_lon=R[1])
    show("tangent", cls2.state)
    print(st2)

    # 3) ŽÁDNÝ PRŮSEČÍK: přímka 1.2 m od R
    cls3 = NearWaypoint(S_lat=R[0] + (1.2/111_132.954), S_lon=R[1]-0.0002,
                        E_lat=R[0] + (1.2/111_132.954), E_lon=R[1]+0.0002,
                        L_near_m=L)
    st3 = cls3.update(R_lat=R[0], R_lon=R[1])
    show("no intersection", cls3.state)
    print(st3)

    # 4) ZÁPORNÁ vzdálenost k cíli: R leží ZA E (projekce za E ve směru S→E)
    #    Vezmeme přímku východním směrem a přesuneme R ~2 m za E.
    S = (50.0, 14.0)
    E = (50.0, 14.0 + 0.00002)  # cca ~1.3–1.5 m podle zem. šířky
    tester = NearWaypoint(S_lat=S[0], S_lon=S[1], E_lat=E[0], E_lon=E[1], L_near_m=L)
    # Posun R o další +0.00002 stupně východně od E (tj. projekce > L_seg)
    R_past = (50.0 + 0.000004, 14.0 + 0.00002)
    st4 = tester.update(R_lat=R_past[0], R_lon=R_past[1])
    show("negative distance (past goal)", tester.state)
    print(st4)
