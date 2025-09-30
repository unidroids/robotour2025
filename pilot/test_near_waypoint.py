# tests/test_near_waypoint.py
import unittest
from near_waypoint import select_near_point

class TestNearWaypoint(unittest.TestCase):
    def test_two_intersections(self):
        R = (50.0, 14.0)
        res = select_near_point(
            S_lat=R[0], S_lon=R[1]-0.0002,
            E_lat=R[0], E_lon=R[1]+0.0002,
            R_lat=R[0], R_lon=R[1],
            L_near_m=1.0
        )
        self.assertEqual(res.case, "TWO_INTERSECTIONS")
        self.assertIsNotNone(res.near_x_m)
        self.assertAlmostEqual(res.near_y_m or 0.0, 0.0, delta=1e-3)
        self.assertAlmostEqual(res.near_x_m or 0.0, 1.0, delta=1e-2)

    def test_tangent(self):
        R = (50.0, 14.0)
        res = select_near_point(
            S_lat=R[0] + (1.0/111_132.954), S_lon=R[1]-0.0002,
            E_lat=R[0] + (1.0/111_132.954), E_lon=R[1]+0.0002,
            R_lat=R[0], R_lon=R[1],
            L_near_m=1.0
        )
        self.assertEqual(res.case, "TANGENT")
        self.assertAlmostEqual(res.near_x_m or 0.0, 0.0, delta=2e-3)
        self.assertAlmostEqual(res.near_y_m or 0.0, 1.0, delta=2e-3)

    def test_no_intersection(self):
        R = (50.0, 14.0)
        res = select_near_point(
            S_lat=R[0] + (1.2/111_132.954), S_lon=R[1]-0.0002,
            E_lat=R[0] + (1.2/111_132.954), E_lon=R[1]+0.0002,
            R_lat=R[0], R_lon=R[1],
            L_near_m=1.0
        )
        self.assertEqual(res.case, "NO_INTERSECTION")
        self.assertIsNone(res.near_lat)
        self.assertIsNone(res.near_lon)

if __name__ == "__main__":
    unittest.main()
