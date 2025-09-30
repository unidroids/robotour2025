# geo_utils.py
# -----------------------------------------------------------------------------
# Převody souřadnic WGS-84:
# - LLA <-> ECEF
# - ECEF <-> ENU (referenční bod: lat0, lon0, h0)
# + Pomocné převody headingu a yaw-rate mezi konvencemi (GNSS vs. ENU/CCW)
#
# Pozn.: Implementace je bez externích závislostí a s dvojitou přesností.
# Pro vzdálenosti v řádu desítek/stovek metrů je chyba zanedbatelná (mm–cm).
# -----------------------------------------------------------------------------

from __future__ import annotations
from typing import Tuple
import math

__all__ = [
    "deg2rad", "rad2deg",
    "lla_to_ecef", "ecef_to_lla",
    "ecef_to_enu", "enu_to_ecef",
    "heading_gnss_to_enu", "heading_enu_to_gnss",
    "yawrate_cw_to_ccw", "yawrate_ccw_to_cw",
]

# -------------------------------
# WGS-84 constants
# -------------------------------
_WGS84_A = 6378137.0                  # semi-major axis [m]
_WGS84_F = 1.0 / 298.257223563        # flattening
_WGS84_B = _WGS84_A * (1.0 - _WGS84_F)
_WGS84_E2 = (_WGS84_A**2 - _WGS84_B**2) / (_WGS84_A**2)   # first eccentricity^2
_WGS84_EP2 = (_WGS84_A**2 - _WGS84_B**2) / (_WGS84_B**2)  # second eccentricity^2

def deg2rad(d: float) -> float:
    return d * math.pi / 180.0

def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi

# -------------------------------
# LLA <-> ECEF
# -------------------------------
def lla_to_ecef(lat_deg: float, lon_deg: float, h_m: float = 0.0) -> Tuple[float, float, float]:
    lat = deg2rad(lat_deg)
    lon = deg2rad(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1.0 - _WGS84_E2) + h_m) * sin_lat
    return x, y, z

def ecef_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Robustní převod ECEF -> LLA (Bowring-like, bez iterace).

    Doctest (round-trip LLA->ECEF->LLA):
    >>> lat, lon, h = 50.0, 14.0, 300.0
    >>> x, y, z = lla_to_ecef(lat, lon, h)
    >>> lat2, lon2, h2 = ecef_to_lla(x, y, z)
    >>> abs(lat - lat2) < 1e-9 and abs(lon - lon2) < 1e-9 and abs(h - h2) < 1e-6
    True
    """
    r = math.hypot(x, y)
    if r < 1e-12:
        # polar singularity handling
        lat = math.copysign(math.pi / 2.0, z)
        lon = 0.0
        h = abs(z) - _WGS84_B
        return rad2deg(lat), rad2deg(lon), h

    F = 54.0 * _WGS84_B**2 * z * z
    G = r * r + (1.0 - _WGS84_E2) * z * z - _WGS84_E2 * (_WGS84_A**2 - _WGS84_B**2)
    c = (_WGS84_E2**2) * F * r * r / (G**3)
    s = (1.0 + c + math.sqrt(c * c + 2.0 * c)) ** (1.0 / 3.0)
    P = F / (3.0 * (s + 1.0 / s + 1.0) ** 2 * G * G)
    Q = math.sqrt(1.0 + 2.0 * _WGS84_E2 * _WGS84_E2 * P)
    r0 = -(P * _WGS84_E2 * r) / (1.0 + Q) + math.sqrt(
        0.5 * _WGS84_A * _WGS84_A * (1.0 + 1.0 / Q)
        - P * (1.0 - _WGS84_E2) * z * z / (Q * (1.0 + Q))
        - 0.5 * P * r * r
    )
    U = math.sqrt((r - _WGS84_E2 * r0) ** 2 + z * z)
    V = math.sqrt((r - _WGS84_E2 * r0) ** 2 + (1.0 - _WGS84_E2) * z * z)
    z0 = (_WGS84_B**2) * z / (_WGS84_A * V)
    h = U * (1.0 - (_WGS84_B**2) / (_WGS84_A * V))
    lat = math.atan2(z + _WGS84_EP2 * z0, r)
    lon = math.atan2(y, x)
    return rad2deg(lat), rad2deg(lon), h

# -------------------------------
# ECEF <-> ENU (ref: lat0, lon0, h0)
# -------------------------------
def _enu_rotation(lat0_deg: float, lon0_deg: float):
    lat0 = deg2rad(lat0_deg)
    lon0 = deg2rad(lon0_deg)
    sL, cL = math.sin(lat0), math.cos(lat0)
    sO, cO = math.sin(lon0), math.cos(lon0)
    # ECEF->ENU rotation matrix R such that v_enu = R * (v_ecef - ref_ecef)
    R = (
        (-sO,           cO,            0.0),
        (-sL * cO,     -sL * sO,      cL),
        ( cL * cO,      cL * sO,      sL),
    )
    return R

def ecef_to_enu(x: float, y: float, z: float,
                lat0_deg: float, lon0_deg: float, h0_m: float = 0.0) -> Tuple[float, float, float]:
    """
    Doctest (round-trip přes ENU):
    >>> lat0, lon0 = 50.0, 14.0
    >>> x0, y0, z0 = lla_to_ecef(lat0 + 0.001, lon0 + 0.001, 200.0)
    >>> e, n, u = ecef_to_enu(x0, y0, z0, lat0, lon0, 250.0)
    >>> X, Y, Z = enu_to_ecef(e, n, u, lat0, lon0, 250.0)
    >>> abs(X - x0) < 1e-6 and abs(Y - y0) < 1e-6 and abs(Z - z0) < 1e-6
    True
    """
    x0, y0, z0 = lla_to_ecef(lat0_deg, lon0_deg, h0_m)
    dx, dy, dz = x - x0, y - y0, z - z0
    R = _enu_rotation(lat0_deg, lon0_deg)
    e = R[0][0] * dx + R[0][1] * dy + R[0][2] * dz
    n = R[1][0] * dx + R[1][1] * dy + R[1][2] * dz
    u = R[2][0] * dx + R[2][1] * dy + R[2][2] * dz
    return e, n, u

def enu_to_ecef(e: float, n: float, u: float,
                lat0_deg: float, lon0_deg: float, h0_m: float = 0.0) -> Tuple[float, float, float]:
    # inverse of ECEF->ENU: v_ecef = ref + R^T * v_enu
    x0, y0, z0 = lla_to_ecef(lat0_deg, lon0_deg, h0_m)
    R = _enu_rotation(lat0_deg, lon0_deg)
    # R je ortonormální; inverze = transpozice
    dx = R[0][0] * e + R[1][0] * n + R[2][0] * u
    dy = R[0][1] * e + R[1][1] * n + R[2][1] * u
    dz = R[0][2] * e + R[1][2] * n + R[2][2] * u
    return x0 + dx, y0 + dy, z0 + dz

# -------------------------------
# Heading / yaw-rate konverze
# -------------------------------
def heading_gnss_to_enu(heading_gnss_deg: float) -> float:
    """
    GNSS azimut: 0°=Sever, 90°=Východ
    ENU (matematický): 0°=Východ, 90°=Sever
    """
    return (90.0 - heading_gnss_deg) % 360.0

def heading_enu_to_gnss(heading_enu_deg: float) -> float:
    return (90.0 - heading_enu_deg) % 360.0

def yawrate_cw_to_ccw(yawrate_dps: float) -> float:
    """Převrátí znaménko: +CW -> +CCW (matematické kladné)."""
    return -yawrate_dps

def yawrate_ccw_to_cw(yawrate_dps: float) -> float:
    """Převrátí znaménko: +CCW -> +CW (GNSS zvyklost, pokud je potřeba)."""
    return -yawrate_dps

# -------------------------------
# CLI/doctest
# -------------------------------
if __name__ == "__main__":
    import doctest
    fails, total = doctest.testmod(optionflags=doctest.ELLIPSIS)
    print(f"[geo_utils] doctest: {total} tests, {fails} failed")
