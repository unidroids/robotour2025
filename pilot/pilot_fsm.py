# pilot_fsm.py
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Literal, Tuple

NearCase = Literal["TWO_INTERSECTIONS", "TANGENT", "NO_INTERSECTION"]

class NavigatorState(Enum):
    WAIT_GNSS = auto()
    ACQUIRE_HEADING = auto()
    NAVIGATE = auto()
    SAFE_SPIN = auto()
    GOAL_REACHED = auto()
    GOAL_NOT_REACHED = auto()

class AcquirePhase(Enum):
    ROTATE = auto()  # točím naslepo až do headingAcc <= threshold
    SEEK   = auto()  # točím stejným směrem, dokud |err_to_near| > okno
    READY  = auto()  # v okně (např. 20..7°), zpomalím rotaci, pak odjezd vpřed

@dataclass
class FsmConfig:
    hacc_ready_m: float = 1.5
    acquire_heading_acc_max_deg: float = 40.0
    acquire_heading_window_deg: float = 15.0
    heading_uncertain_deg: float = 20.0
    t_stable_s: float = 0.7
    t_hold_s: float = 0.3
    v_max_mps: float = 0.5
    omega_max_dps: float = 90.0
    omega_acquire_gain: float = 2.0
    acquire_spin_dir: Literal["RIGHT","LEFT","AUTO"] = "RIGHT"
    acquire_pwm_spin: int = 25
    acquire_pwm_prespin: int = 25
    acquire_pwm_drive: int = 28

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
    omega_setpoint_dps: float
    note: str
    substate: Optional[str] = None
    direct_pwm: Optional[Tuple[int,int]] = None

class NavigatorFSM:
    def __init__(self, cfg: Optional[FsmConfig] = None) -> None:
        self.cfg = cfg or FsmConfig()
        self.state = NavigatorState.WAIT_GNSS
        self._t_good = 0.0
        self._t_bad = 0.0
        self._acq_phase: AcquirePhase = AcquirePhase.ROTATE
        self._spin_sign: int = +1  # +1=RIGHT, -1=LEFT

    def reset(self, state: Optional[NavigatorState] = None) -> None:
        self.state = state or NavigatorState.WAIT_GNSS
        self._t_good = 0.0
        self._t_bad = 0.0
        self._acq_phase = AcquirePhase.ROTATE
        self._spin_sign = +1

    def _decide_spin_sign(self, heading_err_to_near_deg: float) -> int:
        if self.cfg.acquire_spin_dir == "RIGHT":
            return +1
        if self.cfg.acquire_spin_dir == "LEFT":
            return -1
        return +1 if heading_err_to_near_deg >= 0 else -1

    def step(self, dt_s: float, quality: NavQuality,
             dist_to_goal_m: float, goal_radius_m: float,
             near_case: NearCase, heading_err_to_near_deg: float) -> FsmAction:

        if dist_to_goal_m <= goal_radius_m:
            self.state = NavigatorState.GOAL_REACHED
            return FsmAction(self.state, False, False, 0.0, "Goal reached")

        if near_case == "NO_INTERSECTION":
            self.state = NavigatorState.GOAL_NOT_REACHED
            return FsmAction(self.state, False, False, 0.0, "Near selection failed (NO_INTERSECTION)")

        pos_ready = quality.has_fix and (quality.hacc_m <= self.cfg.hacc_ready_m)

        # NAVIGATE → ACQUIRE pokud ztratíme heading na near
        if self.state == NavigatorState.NAVIGATE and abs(heading_err_to_near_deg) > 60.0:
            self.state = NavigatorState.ACQUIRE_HEADING
            self._acq_phase = AcquirePhase.ROTATE
            self._spin_sign = self._decide_spin_sign(heading_err_to_near_deg)
            return FsmAction(self.state, False, True, 0.0, "Re-acquire (|err|>60°)", substate=self._acq_phase.name)

        # WAIT_GNSS
        if self.state == NavigatorState.WAIT_GNSS:
            if pos_ready:
                self.state = NavigatorState.ACQUIRE_HEADING
                self._acq_phase = AcquirePhase.ROTATE
                self._spin_sign = self._decide_spin_sign(heading_err_to_near_deg)
                pwm = self.cfg.acquire_pwm_spin
                return FsmAction(self.state, False, True, self.cfg.omega_max_dps*0.5,
                                 "Pos OK -> Acquire", substate=self._acq_phase.name,
                                 direct_pwm=(+pwm, -pwm) if self._spin_sign>0 else (-pwm, +pwm))
            pwm = max(15, self.cfg.acquire_pwm_spin-10)
            return FsmAction(self.state, False, True, self.cfg.omega_max_dps*0.2,
                             "Waiting GNSS", substate=None,
                             direct_pwm=(+pwm, -pwm))

        # ACQUIRE_HEADING – 3 fáze
        if self.state == NavigatorState.ACQUIRE_HEADING:
            # 1) ROTATE: točím pořád stejným směrem, dokud headingAcc <= 40°
            if self._acq_phase == AcquirePhase.ROTATE:
                if quality.heading_acc_deg <= self.cfg.acquire_heading_acc_max_deg:
                    self._acq_phase = AcquirePhase.SEEK
                pwm = self.cfg.acquire_pwm_spin
                return FsmAction(self.state, False, True, self.cfg.omega_max_dps*0.8,
                                 "Acquire: ROTATE", substate="ROTATE",
                                 direct_pwm=(+pwm, -pwm) if self._spin_sign>0 else (-pwm, +pwm))

            # 2) SEEK: stále točím týmž směrem, dokud |err_to_near| > 20°
            if self._acq_phase == AcquirePhase.SEEK:
                if abs(heading_err_to_near_deg) <= 20.0:
                    self._acq_phase = AcquirePhase.READY
                pwm = self.cfg.acquire_pwm_spin
                return FsmAction(self.state, False, True, self.cfg.omega_max_dps*0.6,
                                 "Acquire: SEEK", substate="SEEK",
                                 direct_pwm=(+pwm, -pwm) if self._spin_sign>0 else (-pwm, +pwm))

            # 3) READY: v okně 20..7°
            if self._acq_phase == AcquirePhase.READY:
                if abs(heading_err_to_near_deg) <= 7.0:
                    self.state = NavigatorState.NAVIGATE
                    pwm = self.cfg.acquire_pwm_drive
                    return FsmAction(self.state, True, True, 0.0,
                                     "Acquire: READY done -> NAVIGATE",
                                     substate="READY",
                                     direct_pwm=(+pwm, +pwm))
                pwm = self.cfg.acquire_pwm_prespin
                if self._spin_sign > 0:
                    dp = (+pwm, 0)
                else:
                    dp = (0, +pwm)
                return FsmAction(self.state, False, True, self.cfg.omega_max_dps*0.3,
                                 "Acquire: READY prespin", substate="READY",
                                 direct_pwm=dp)

        # SAFE_SPIN
        if self.state == NavigatorState.SAFE_SPIN:
            if quality.heading_acc_deg <= self.cfg.heading_uncertain_deg and abs(heading_err_to_near_deg) <= 30.0:
                self._t_good += dt_s
                if self._t_good >= self.cfg.t_stable_s:
                    self.state = NavigatorState.NAVIGATE
                    self._t_good = 0.0
                    return FsmAction(self.state, True, True, 0.0, "Recovered -> NAVIGATE", substate=None)
            else:
                self._t_good = 0.0
            pwm = self.cfg.acquire_pwm_spin
            return FsmAction(self.state, False, True, self.cfg.omega_max_dps*0.6,
                             "SAFE_SPIN", substate=None,
                             direct_pwm=(+pwm, -pwm))

        if self.state == NavigatorState.NAVIGATE:
            return FsmAction(self.state, True, True, 0.0, "Navigate", substate=None)

        if self.state in (NavigatorState.GOAL_REACHED, NavigatorState.GOAL_NOT_REACHED):
            return FsmAction(self.state, False, False, 0.0, self.state.name, substate=None)
