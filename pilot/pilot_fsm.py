# pilot_fsm.py
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Literal

NearCase = Literal["TWO_INTERSECTIONS", "TANGENT", "NO_INTERSECTION"]

class NavigatorState(Enum):
    WAIT_GNSS = auto()
    ACQUIRE_HEADING = auto()
    NAVIGATE = auto()
    SAFE_SPIN = auto()
    GOAL_REACHED = auto()
    GOAL_NOT_REACHED = auto()

@dataclass
class FsmConfig:
    # Kvalitativní prahy (headingAcc je odhad chyby směru – menší je lepší)
    hacc_ready_m: float = 1.5                   # přesnost polohy pro rozjezd
    acquire_heading_acc_max_deg: float = 40.0   # max headingAcc pro přechod z acquire (tvoje „< 40°“)
    acquire_heading_window_deg: float = 15.0    # okno ±deg na NEAR pro puštění do NAVIGATE
    heading_uncertain_deg: float = 20.0         # při jízdě: když headingAcc >=, jdi do SAFE_SPIN
    # Časové filtry (stabilita)
    t_stable_s: float = 0.7                     # doba udržení „dobrého“ stavu
    t_hold_s: float = 0.3                       # doba držení špatného stavu před SAFE_SPIN
    # Rychlostní limity
    v_max_mps: float = 0.5
    omega_max_dps: float = 90.0
    # Síla otáčení v acquire/safe (pro návrh setpointu ω)
    omega_acquire_gain: float = 2.0             # [deg/s] na [deg] chyby (saturováno max_dps)

@dataclass
class NavQuality:
    has_fix: bool
    hacc_m: float
    heading_acc_deg: float

@dataclass
class FsmAction:
    state: NavigatorState
    allow_forward: bool
    allow_spin: bool
    omega_setpoint_dps: float  # velikost; znaménko řeší controller podle near
    note: str

class NavigatorFSM:
    def __init__(self, cfg: Optional[FsmConfig] = None) -> None:
        self.cfg = cfg or FsmConfig()
        self.state = NavigatorState.WAIT_GNSS
        self._t_good = 0.0
        self._t_bad = 0.0

    def reset(self, state: Optional[NavigatorState] = None) -> None:
        self.state = state or NavigatorState.WAIT_GNSS
        self._t_good = 0.0
        self._t_bad = 0.0

    def step(
        self,
        dt_s: float,
        quality: NavQuality,
        dist_to_goal_m: float,
        goal_radius_m: float,
        near_case: NearCase,
        heading_err_to_near_deg: float,
    ) -> FsmAction:
        """
        FSM krok – rozhoduje na základě headingAcc a chyby vůči near.
        """
        s = self.state
        cfg = self.cfg

        # Cíl
        if dist_to_goal_m <= goal_radius_m:
            self.state = NavigatorState.GOAL_REACHED
            return FsmAction(self.state, False, False, 0.0, "Goal reached")

        # Near selhal
        if near_case == "NO_INTERSECTION":
            self.state = NavigatorState.GOAL_NOT_REACHED
            return FsmAction(self.state, False, False, 0.0, "Near selection failed (NO_INTERSECTION)")

        # Kvalita
        pos_ready = quality.has_fix and (quality.hacc_m <= cfg.hacc_ready_m)
        # „dobrý“ pro jízdu – malá nejistota směru
        head_good_for_nav = (quality.heading_acc_deg <= cfg.heading_uncertain_deg)
        # „hotový acquire“ – přesnost směru OK a jsme natočeni k near
        acquire_ready = (quality.heading_acc_deg <= cfg.acquire_heading_acc_max_deg) and \
                        (abs(heading_err_to_near_deg) <= cfg.acquire_heading_window_deg)

        # WAIT_GNSS: čekáme na polohu; volitelně mírný dither ve směru
        if s == NavigatorState.WAIT_GNSS:
            if pos_ready:
                self.state = NavigatorState.ACQUIRE_HEADING
                return FsmAction(self.state, False, True, cfg.omega_max_dps * 0.5, "Pos OK -> Acquire heading")
            else:
                return FsmAction(self.state, False, True, cfg.omega_max_dps * 0.2, "Waiting GNSS")

        # ACQUIRE_HEADING: točíme do „rozumné“ přesnosti směru a okna k near
        if self.state == NavigatorState.ACQUIRE_HEADING:
            if acquire_ready:
                self._t_good += dt_s
                if self._t_good >= cfg.t_stable_s:
                    self.state = NavigatorState.NAVIGATE
                    self._t_good = 0.0
                    return FsmAction(self.state, True, True, 0.0, "Heading ready -> Navigate")
            else:
                self._t_good = 0.0

            # doporuč ω podle velikosti chyby k near (sign řeší controller)
            omega_mag = min(cfg.omega_max_dps * 0.8, cfg.omega_acquire_gain * abs(heading_err_to_near_deg))
            return FsmAction(self.state, False, True, omega_mag, "Acquiring heading")

        # NAVIGATE: běžná jízda; zhoršení kvality → SAFE_SPIN
        if self.state == NavigatorState.NAVIGATE:
            if not head_good_for_nav:
                self._t_bad += dt_s
                if self._t_bad >= cfg.t_hold_s:
                    self.state = NavigatorState.SAFE_SPIN
                    self._t_bad = 0.0
                    omega_mag = min(cfg.omega_max_dps * 0.6, cfg.omega_acquire_gain * abs(heading_err_to_near_deg))
                    return FsmAction(self.state, False, True, omega_mag, "Heading uncertain -> Safe spin")
            else:
                self._t_bad = 0.0
            return FsmAction(self.state, True, True, 0.0, "Navigate")

        # SAFE_SPIN: stojíme dopředně, otáčíme do zlepšení
        if self.state == NavigatorState.SAFE_SPIN:
            if acquire_ready:
                self._t_good += dt_s
                if self._t_good >= cfg.t_stable_s:
                    self.state = NavigatorState.NAVIGATE
                    self._t_good = 0.0
                    return FsmAction(self.state, True, True, 0.0, "Recovered -> Navigate")
            else:
                self._t_good = 0.0
            omega_mag = min(cfg.omega_max_dps * 0.6, cfg.omega_acquire_gain * abs(heading_err_to_near_deg))
            return FsmAction(self.state, False, True, omega_mag, "Safe spin")

        if self.state == NavigatorState.GOAL_REACHED:
            return FsmAction(self.state, False, False, 0.0, "Goal reached")

        if self.state == NavigatorState.GOAL_NOT_REACHED:
            return FsmAction(self.state, False, False, 0.0, "Goal not reached (escalate)")

        # Fallback
        return FsmAction(self.state, False, False, 0.0, "Idle")
