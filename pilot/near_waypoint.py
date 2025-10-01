# near_waypoint.py
# -----------------------------------------------------------------------------
# Near-point selektor podle domluvené specifikace:
# - Pracujeme s PŘÍMKOU procházející S–E (bez ořezu na segment).
# - Hledáme průsečík(y) této přímky s kružnicí C(R, L_near):
#     2 průsečíky  -> vybereme bod "blíže k E" (větší projekce na směr S→E)
#     1 průsečík   -> vezmeme tečný bod
#     0 průsečíků  -> vrátíme NO_INTERSECTION (Pilot/FSM => GOAL_NOT_REACHED)
#
# Převody LLA<->ECEF<->ENU jsou přesunuty do geo_utils.py.
# -----------------------------------------------------------------------------

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, Optional, Literal
import math

from geo_utils import (
    lla_to_ecef, ecef_to_lla,
    ecef_to_enu, enu_to_ecef,
)

NearCase = Literal["TWO_INTERSECTIONS", "TANGENT", "NO_INTERSECTION"]

@dataclass
class NearPointResult:
    case: NearCase
    near_lat: Optional[float]
    near_lon: Optional[float]
    near_x_m: Optional[float]  # ENU (origin = R), for debugging/telemetrie
    near_y_m: Optional[float]  # ENU (origin = R)
    d_perp_m: float            # vzdálenost z R na PŘÍMKU(S–E)
    chosen_t_along: Optional[float]  # projekce podél S'->E' (vztaženo k S')

def select_near_point(
    S_lat: float, S_lon: float,
    E_lat: float, E_lon: float,
    R_lat: float, R_lon: float,
    L_near_m: float,
    #eps_m: float = 1e-6,
    eps_m: float = 2e-3,
) -> NearPointResult:
    """
    Najde průsečík(y) PŘÍMKY S–E s kružnicí C(R, L_near) v ENU(R),
    vybere near bod (blíže k E) a převede jej do LLA.

    Doctests (geometrické sanity-checky s tolerancí):

    >>> # 1) Dvě průsečnice: vodorovná přímka přes R, E "vpravo". Near = (+L, 0).
    >>> R = (50.0, 14.0)
    >>> res = select_near_point(S_lat=R[0], S_lon=R[1]-0.0002,
    ...                         E_lat=R[0], E_lon=R[1]+0.0002,
    ...                         R_lat=R[0], R_lon=R[1],
    ...                         L_near_m=1.0)
    >>> res.case
    'TWO_INTERSECTIONS'
    >>> abs((res.near_x_m or 0.0) - 1.0) < 1e-3 and abs((res.near_y_m or 0.0) - 0.0) < 1e-3
    True

    >>> # 2) Tečna: přímka paralelní s x, posunutá o 1 m v +y; near = (0, +1)
    >>> res2 = select_near_point(S_lat=R[0]+(1.0/111_132.954), S_lon=R[1]-0.0002,
    ...                          E_lat=R[0]+(1.0/111_132.954), E_lon=R[1]+0.0002,
    ...                          R_lat=R[0], R_lon=R[1],
    ...                          L_near_m=1.0)
    >>> res2.case
    'TANGENT'
    >>> abs((res2.near_x_m or 0.0) - 0.0) < 2e-3 and abs((res2.near_y_m or 0.0) - 1.0) < 2e-3
    True

    >>> # 3) Bez průsečíku: přímka ve vzdálenosti 1.2 m od R
    >>> res3 = select_near_point(S_lat=R[0]+(1.2/111_132.954), S_lon=R[1]-0.0002,
    ...                          E_lat=R[0]+(1.2/111_132.954), E_lon=R[1]+0.0002,
    ...                          R_lat=R[0], R_lon=R[1],
    ...                          L_near_m=1.0)
    >>> res3.case
    'NO_INTERSECTION'
    """
    # 1) Převod S,E do ENU s referencí v R (robot)
    Sx, Sy, _ = ecef_to_enu(*lla_to_ecef(S_lat, S_lon), R_lat, R_lon, 0.0)
    Ex, Ey, _ = ecef_to_enu(*lla_to_ecef(E_lat, E_lon), R_lat, R_lon, 0.0)

    # 2) Parametrizace přímky S'->E' v ENU (2D)
    vx, vy = Ex - Sx, Ey - Sy
    L_seg = math.hypot(vx, vy)
    if L_seg < 1e-12:
        # Degenerace: S≈E -> přímka špatně definovaná; vracíme NO_INTERSECTION
        return NearPointResult(
            case="NO_INTERSECTION",
            near_lat=None, near_lon=None,
            near_x_m=None, near_y_m=None,
            d_perp_m=math.hypot(Sx, Sy),
            chosen_t_along=None,
        )
    # normovaný směr
    vx /= L_seg
    vy /= L_seg

    # 3) Pata kolmice Q z R=(0,0) na přímku S'+t*v (projekce -S' na v)
    t_q = (-(Sx * vx + Sy * vy))
    Qx = Sx + t_q * vx
    Qy = Sy + t_q * vy
    d_perp = math.hypot(Qx, Qy)

    # 4) Průsečík(y) s kružnicí x^2 + y^2 = L^2
    Lr = float(L_near_m)
    eps = float(eps_m)
    if d_perp > Lr + eps:
        # žádný průsečík
        return NearPointResult(
            case="NO_INTERSECTION",
            near_lat=None, near_lon=None,
            near_x_m=None, near_y_m=None,
            d_perp_m=d_perp,
            chosen_t_along=None,
        )
    elif abs(d_perp - Lr) <= eps:
        # tečna: near = Q
        nx, ny = Qx, Qy
        nx_ecef, ny_ecef, nz_ecef = enu_to_ecef(nx, ny, 0.0, R_lat, R_lon, 0.0)
        nlat, nlon, _ = ecef_to_lla(nx_ecef, ny_ecef, nz_ecef)
        t_along = ((nx - Sx) * vx + (ny - Sy) * vy)
        return NearPointResult(
            case="TANGENT",
            near_lat=nlat, near_lon=nlon,
            near_x_m=nx, near_y_m=ny,
            d_perp_m=d_perp,
            chosen_t_along=t_along,
        )
    else:
        # 2 průsečíky: N1 = Q + delta*v, N2 = Q - delta*v
        delta = math.sqrt(max(0.0, Lr * Lr - d_perp * d_perp))
        n1x, n1y = Qx + delta * vx, Qy + delta * vy
        n2x, n2y = Qx - delta * vx, Qy - delta * vy
        # Vyber "blíže k E" => větší projekce na směr v (od S')
        t1 = ((n1x - Sx) * vx + (n1y - Sy) * vy)
        t2 = ((n2x - Sx) * vx + (n2y - Sy) * vy)
        if t1 >= t2:
            nx, ny, t_along = n1x, n1y, t1
        else:
            nx, ny, t_along = n2x, n2y, t2

        nx_ecef, ny_ecef, nz_ecef = enu_to_ecef(nx, ny, 0.0, R_lat, R_lon, 0.0)
        nlat, nlon, _ = ecef_to_lla(nx_ecef, ny_ecef, nz_ecef)

        return NearPointResult(
            case="TWO_INTERSECTIONS",
            near_lat=nlat, near_lon=nlon,
            near_x_m=nx, near_y_m=ny,
            d_perp_m=d_perp,
            chosen_t_along=t_along,
        )

# -------------------------------
# CLI/doctest runner
# -------------------------------
if __name__ == "__main__":
    import doctest
    n_failed, n_tests = doctest.testmod(optionflags=doctest.ELLIPSIS)
    print(f"[near_waypoint] doctest: {n_tests} tests, {n_failed} failed")
