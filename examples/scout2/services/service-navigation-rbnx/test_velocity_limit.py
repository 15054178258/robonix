import math
import unittest

from nav2_wrapper.velocity_limit import bounded_linear_velocity


class VelocityLimitTest(unittest.TestCase):
    def test_planar_speed_below_limit_is_unchanged(self):
        self.assertEqual(
            bounded_linear_velocity(0.2, 0.1, 0.4),
            (0.2, 0.1, False),
        )

    def test_planar_speed_is_scaled_without_changing_direction(self):
        x, y, limited = bounded_linear_velocity(0.6, 0.8, 0.4)
        self.assertTrue(limited)
        self.assertAlmostEqual(math.hypot(x, y), 0.4)
        self.assertAlmostEqual(x / y, 0.6 / 0.8)

    def test_reverse_motion_uses_the_same_metric_limit(self):
        x, y, limited = bounded_linear_velocity(-0.6, 0.0, 0.3)
        self.assertTrue(limited)
        self.assertAlmostEqual(x, -0.3)
        self.assertEqual(y, 0.0)

if __name__ == "__main__":
    unittest.main()
