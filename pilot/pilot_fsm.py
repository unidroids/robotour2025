# pilot_fsm.py
# -----------------------------------------------------------------------------
# Lehký stavový automat pro Pilot:
# - WAIT_GNSS       : čekáme na polohovou kvalitu (hAcc) a fix
# - ACQUIRE_HEADING : stojíme dopředně, točíme řízeně (±omega_max) do zlepšení headingAcc
# - NAVIGATE        : běžná jízda k near bodu
# - SAFE_SPIN       : během jízdy vyletěla headingAcc -> v=0, řízený spin, návrat po zlepšení
# - GOAL_REACHED    : dosažen cíl (radius)
# - GOAL_NOT_REACHED: near selektor vrací NO_INTERSECTION -> eskalace "nahoru"
#
# FSM vrací "akci" v abstraktních hodnotách (povolení v/ω a doporučení ω_setpoint),
# samotný převod na PWM řeší MotionController (mimo tento soubor).
# -----------------------------------------------------------------------------

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
    # Kvalitativní prahy
    hacc_ready_m: float = 1.5          # polohová přesnost pro rozjezd
    heading_ready_deg: float = 12.0    # headingAcc limit pro rozjezd
    heading_uncertain_deg: float = 20.0  # headingAcc limit pro SAFE_SPIN
    # Časové filtry (stabilita)
    t_stable_s: float = 0.7            # doba pod prahem pro přechod do NAVIGATE
    t_hold_s: float = 0.3              # doba nad prahem pro přechod do SAFE_SPIN
    # Rychlostní limity
    v_max_mps: float = 0.5             # ladění (změň na 1.5 pro ostrý režim)
    omega_max_dps: float = 90.0

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
    omega_setpoint_dps: float  # doporučená velikost ω (znaménko určí controller dle near)
    note: str

class NavigatorFSM:
    def __init__(self, cfg: Optional[FsmConfig] = None) -> None:
        self.cfg = cfg or FsmConfig()
        self.state = NavigatorState.WAIT_GNSS
        # vnitřní akumulační časy
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
    ) -> FsmAction:
        """
        Jediný vstupní krok FSM. Vrací doporučenou akci pro Pilot.
        - dt_s: krok času (sekundy)
        - quality: kvalita GNSS/heading
        - dist_to_goal_m: vzdálenost k cíli
        - goal_radius_m: poloměr "dosaženo"
        - near_case: výsledek near selektoru (NO_INTERSECTION => GOAL_NOT_REACHED)

        Doctest: jednoduché přechody (synteticky bez času pro ilustraci)
        >>> fsm = NavigatorFSM(FsmConfig(hacc_ready_m=1.5, heading_ready_deg=12.0, heading_uncertain_deg=20.0))
        >>> q_ok = NavQuality(has_fix=True, hacc_m=1.0, heading_acc_deg=10.0)
        >>> # WAIT_GNSS -> ACQUIRE_HEADING (protože splněna poloha, ale chceme projít přes acquire)
        >>> a = fsm.step(dt_s=0.1, quality=q_ok, dist_to_goal_m=10.0, goal_radius_m=2.0, near_case="TWO_INTERSECTIONS")
        >>> a.state in (NavigatorState.ACQUIRE_HEADING, NavigatorState.NAVIGATE)
        True
        """
        # Stavový logický blok
        s = self.state
        cfg = self.cfg
        note = ""

        # GOAL dosažen kdekoliv
        if dist_to_goal_m <= goal_radius_m:
            self.state = NavigatorState.GOAL_REACHED
            return FsmAction(self.state, False, False, 0.0, "Goal reached")

        # Pokud near selhal, eskalujeme
        if near_case == "NO_INTERSECTION":
            self.state = NavigatorState.GOAL_NOT_REACHED
            return FsmAction(self.state, False, False, 0.0, "Near selection failed (NO_INTERSECTION)")

        # Pomocné flagy kvality
        pos_ready = quality.has_fix and (quality.hacc_m <= cfg.hacc_ready_m)
        head_good = (quality.heading_acc_deg <= cfg.heading_ready_deg)
        head_bad  = (quality.heading_acc_deg >= cfg.heading_uncertain_deg)

        # Stavové přechody
        if s == NavigatorState.WAIT_GNSS:
            if pos_ready:
                self.state = NavigatorState.ACQUIRE_HEADING
                note = "Pos OK -> Acquire heading"
            else:
                return FsmAction(self.state, False, True, cfg.omega_max_dps * 0.2, "Waiting GNSS (optional micro-dither)")

        if self.state == NavigatorState.ACQUIRE_HEADING:
            # akumulace "dobrého" času pod prahem
            if head_good:
                self._t_good += dt_s
                if self._t_good >= cfg.t_stable_s:
                    self.state = NavigatorState.NAVIGATE
                    self._t_good = 0.0
                    note = "Heading stable -> Navigate"
            else:
                self._t_good = 0.0
            # v Acquire povolíme spin, dopředně NE
            return FsmAction(self.state, False, True, cfg.omega_max_dps * 0.8, note or "Acquiring heading")

        if self.state == NavigatorState.NAVIGATE:
            # hlídání zhoršení headingu během jízdy
            if head_bad:
                self._t_bad += dt_s
                if self._t_bad >= cfg.t_hold_s:
                    self.state = NavigatorState.SAFE_SPIN
                    self._t_bad = 0.0
                    return FsmAction(self.state, False, True, cfg.omega_max_dps * 0.6, "Heading uncertain -> Safe spin")
            else:
                self._t_bad = 0.0
            # v navigate povoleno dopředně i spin (controller si rozloží na L/R PWM)
            return FsmAction(self.state, True, True, 0.0, "Navigate")

        if self.state == NavigatorState.SAFE_SPIN:
            if head_good:
                self._t_good += dt_s
                if self._t_good >= cfg.t_stable_s:
                    self.state = NavigatorState.NAVIGATE
                    self._t_good = 0.0
                    return FsmAction(self.state, True, True, 0.0, "Recovered -> Navigate")
            else:
                self._t_good = 0.0
            # v safe spin: dopředně NE, spin ANO
            return FsmAction(self.state, False, True, cfg.omega_max_dps * 0.6, "Safe spin")

        if self.state == NavigatorState.GOAL_REACHED:
            return FsmAction(self.state, False, False, 0.0, "Goal reached")

        if self.state == NavigatorState.GOAL_NOT_REACHED:
            return FsmAction(self.state, False, False, 0.0, "Goal not reached (escalate)")

        # fallback
        return FsmAction(self.state, False, False, 0.0, note or "Idle")

# -------------------------------
# CLI quick check
# -------------------------------
if __name__ == "__main__":
    # jednoduchý ruční běh (není to plný test – doctest je v .step docstringu)
    fsm = NavigatorFSM()
    q = NavQuality(has_fix=False, hacc_m=2.0, heading_acc_deg=30.0)
    a = fsm.step(0.1, q, 10.0, 2.0, "TWO_INTERSECTIONS")
    print("[pilot_fsm] state:", a.state.name, "note:", a.note)
