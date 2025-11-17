import time
from collections import deque
from math import fmod
from typing import Deque, Optional, Tuple

from utils.sliding_angle_average import SlidingAngleAverage
from data.nav_fusion_data import NavFusionData


class FusionCore:
    def __init__(self, angle_window_size: int = 20):
        self.ready = False

        # --- fúze headingu ---
        self._angle_corr = SlidingAngleAverage(angle_window_size)
        self._corr_angle: float = 0.0     # průměrná korekce (glob - lokál) [deg]
        self._corr_quality: float = 0.0   # kvalita <0,1>

        # vyhlazená korekce (smooth_chyba)
        self._smooth_corr_angle: float | None = None

        # max 2 poslední globální headingy (čas, heading)
        self._global_headings: Deque[Tuple[float, float]] = deque(maxlen=2)
        # poslední lokální heading (čas, heading)
        self._last_local_heading: Optional[Tuple[float, float]] = None

        # --- poslední stav polohy ---
        self._lat: float = 0.0
        self._lon: float = 0.0
        self._hAcc: float = 0.0
        self._have_position: bool = False

        # --- rychlost kol ---
        self._left_speed: float = 0.0   # očekávám [mm/s]
        self._right_speed: float = 0.0  # očekávám [mm/s]
        self._have_speed: bool = False

        # --- gyroZ (omega) ---
        self._last_omega: float = 0.0   # [deg/s]

    # -------------------------
    # Pomocné funkce pro úhly
    # -------------------------
    @staticmethod
    def _norm_deg(a: float) -> float:
        """Normalizace do [0, 360)."""
        a = fmod(a, 360.0)
        if a < 0.0:
            a += 360.0
        return a

    @staticmethod
    def _diff_deg(a_from: float, a_to: float) -> float:
        """
        Nejkratší rozdíl a_to - a_from v intervalu (-180, 180].
        """
        diff = (a_to - a_from + 180.0) % 360.0 - 180.0
        return diff

    def _interp_angle_deg(self, a0: float, a1: float, alpha: float) -> float:
        """
        Interpolace úhlů po nejkratší cestě.
        alpha ∈ [0,1], a0,a1 [deg].
        """
        alpha = max(0.0, min(1.0, alpha))
        diff = self._diff_deg(a0, a1)
        return self._norm_deg(a0 + alpha * diff)

    def _update_ready_flag(self) -> None:
        self.ready = (
            self._have_position
            and self._last_local_heading is not None
            and self._have_speed
            and self._angle_corr.ready
        )

    # -------------------------
    # Update metody
    # -------------------------

    def update_position(self, iTow, lat, lon, height, vAcc, hAcc):
        # výšku a vAcc zatím ignorujeme
        self._lat = float(lat)
        self._lon = float(lon)
        self._hAcc = float(hAcc)
        self._have_position = True
        self._update_ready_flag()

    def update_global_heading(self, iTow, heading, gstddev, lenght):
        """
        Uložíme globální heading i s časem (iTow).
        Max 2 poslední hodnoty držíme v deque.
        gstddev a lenght zatím nepoužíváme.
        """
        self._global_headings.append((float(iTow), float(heading)))

    def update_global_roll(self, iTow, roll, gstddev, lenght):
        # zatím nepoužito
        pass

    def update_local_heading(self, tmark, heading, omega):
        tmark = float(tmark)
        heading = float(heading)
        self._last_omega = float(omega)

        if self._last_local_heading is not None and self._global_headings:
            t_prev, h_prev = self._last_local_heading
            t_curr = tmark

            if t_curr > t_prev:
                # zahodíme globální headingy starší než t_prev
                while self._global_headings and self._global_headings[0][0] < t_prev:
                    self._global_headings.popleft()

                # zpracujeme globální headingy v intervalu <t_prev, t_curr>
                while self._global_headings and self._global_headings[0][0] <= t_curr:
                    t_g, h_g = self._global_headings.popleft()
                    alpha = (t_g - t_prev) / (t_curr - t_prev)
                    h_local_at_tg = self._interp_angle_deg(h_prev, heading, alpha)

                    # SlidingAngleAverage: update(a1, a2) → průměruje (a2 - a1)
                    corr_deg, q = self._angle_corr.update(h_local_at_tg, h_g)
                    self._corr_angle = corr_deg
                    self._corr_quality = q

                    # --- vyhlazení korekce pouze při dostatečné kvalitě ---
                    #print(f"update_heading: correction: {corr_deg}, quality: {q}, ready: {self.ready}")
                    if q > 0.8:
                        if self._smooth_corr_angle is None:
                            # první použitelná hodnota – rovnou ji převezmeme
                            self._smooth_corr_angle = corr_deg
                        else:
                            # exponenciální vyhlazování: 90% staré, 10% nové
                            self._smooth_corr_angle = (
                                0.9 * self._smooth_corr_angle + 0.1 * corr_deg
                            )

        self._last_local_heading = (tmark, heading)
        self._update_ready_flag()

    def update_whell_speed(self, tmark, left_wheel_speed, right_wheel_speed):
        """
        Rychlosti kol – očekávám [mm/s].
        (Pokud máš v [m/s], vyhoď dělení 1000 v get().)
        """
        self._left_speed = float(left_wheel_speed)
        self._right_speed = float(right_wheel_speed)
        self._have_speed = True
        self._update_ready_flag()

    # -------------------------
    # Výstup pro pilota
    # -------------------------

    def get_solution(self) -> NavFusionData:
        """
        Poskládá NavFusionData z posledních známých hodnot.
        ts_mono = čas dotazu (monotonic).
        heading = poslední lokální heading + průměrná korekce na globální.
        speed = průměr z kol, převedený z [mm/s] na [m/s].
        Ostatní přesně podle zadání (hard-coded accuracy, flagy 0).
        """
        ts_mono = time.monotonic()

        lat = self._lat
        lon = self._lon
        hAcc = self._hAcc

        if self._last_local_heading is not None:
            heading_local = self._last_local_heading[1]
        else:
            heading_local = 0.0

        # vybereme, jakou korekci použít
        if self._smooth_corr_angle is not None:
            corr_used = self._smooth_corr_angle
        else:
            corr_used = self._corr_angle

        heading = self._norm_deg(heading_local + corr_used)

        # průměrná rychlost, vstup  [mm/s] -> výstup [m/s]
        avg_speed_mm_s = (self._left_speed + self._right_speed) * 0.5
        speed_m_s = avg_speed_mm_s / 1000.0

        # sAcc = 20 mm/s -> 0.02 m/s
        sAcc_m_s = 20.0 / 1000.0

        return NavFusionData(
            ts_mono=ts_mono,
            lat=lat,
            lon=lon,
            hAcc=hAcc,
            heading=heading,
            headingAcc=2.0,        # zatím konstanta – můžeme navázat na quality později
            speed=speed_m_s,
            sAcc=sAcc_m_s,
            gyroZ=self._last_omega,
            gyroZAcc=1.0,
            gnssFixOK=False,
            drUsed=False,
        )