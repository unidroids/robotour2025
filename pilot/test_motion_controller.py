# tests/test_motion_controller.py
import math
import unittest

from motion_controller import MotionController2D, ControllerConfig, SpeedMode

class TestMotionController(unittest.TestCase):
    def setUp(self):
        self.ctrl = MotionController2D(
            ControllerConfig(
                v_max_debug_mps=0.5,
                v_max_normal_mps=1.5,
                omega_max_dps=90.0,
                max_pwm=255,
                deadband_pwm=20,
                slow_down_dist_m=5.0,
                k_heading_to_omega=2.0,
                v_scale=0.6,
            ),
            mode=SpeedMode.DEBUG
        )

    def test_forward_only(self):
        # v=0.5 m/s je v DEBUG přesně v_max => oba PWM ~ maximum po deadbandu
        L, R = self.ctrl.mix_v_omega_to_pwm(0.5, 0.0)
        self.assertGreater(L, 200)
        self.assertGreater(R, 200)
        self.assertAlmostEqual(L, R, delta=2)

    def test_spin_only_ccw(self):
        # čistý CCW spin => levé záporné, pravé kladné
        L, R = self.ctrl.mix_v_omega_to_pwm(0.0, 60.0)
        self.assertLess(L, 0)
        self.assertGreater(R, 0)

    def test_limits_clip(self):
        # výrazně nad limity se musí omezit do rozsahu PWM
        L, R = self.ctrl.mix_v_omega_to_pwm(5.0, 500.0)
        self.assertGreaterEqual(L, -255)
        self.assertLessEqual(L, 255)
        self.assertGreaterEqual(R, -255)
        self.assertLessEqual(R, 255)

    def test_compute_for_near_spin_only(self):
        # Robot míří na východ, near je na sever => chyba +90° => čistý spin CCW
        L, R, st = self.ctrl.compute_for_near(
            heading_enu_deg=0.0,
            near_x_m=0.0, near_y_m=1.0,
            allow_forward=False, allow_spin=True,
            dist_to_goal_m=10.0, goal_radius_m=2.0,
        )
        self.assertIn("SPIN", st)
        self.assertLess(L, 0)
        self.assertGreater(R, 0)

    def test_compute_for_near_nav_forward(self):
        # Robot míří na východ, near je také na východ => dopředně, malé natočení
        L, R, st = self.ctrl.compute_for_near(
            heading_enu_deg=0.0,
            near_x_m=2.0, near_y_m=0.0,
            allow_forward=True, allow_spin=True,
            dist_to_goal_m=10.0, goal_radius_m=2.0,
        )
        self.assertIn("NAV", st)
        self.assertGreater(L, 0)
        self.assertGreater(R, 0)

if __name__ == "__main__":
    unittest.main()
