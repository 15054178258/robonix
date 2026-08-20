import math
import unittest

from nav2_wrapper.rotation_guard_core import GuardLimits, RotationGuard, normalize_uuid_octets


class RotationGuardTest(unittest.TestCase):
    def test_ros_uuid_octets_are_plain_python_ints(self):
        class ByteLike:
            def __init__(self, value):
                self.value = value

            def __int__(self):
                return self.value

        result = normalize_uuid_octets(ByteLike(value) for value in range(16))
        self.assertEqual(result, list(range(16)))
        self.assertTrue(all(type(value) is int for value in result))

    def test_terminal_rotation_without_progress_latches(self):
        guard = RotationGuard(GuardLimits(terminal_no_progress_s=3.0))
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        self.assertEqual(guard.evaluate(0.0, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0)), "")
        guard.observe_odom(0.2)
        reason = guard.evaluate(3.1, 0.0, 0.2, (0, 0, 0.0), (0.1, 0, 1.0))
        self.assertEqual(reason, "terminal yaw made no progress")
        self.assertEqual(guard.evaluate(3.2, 0.0, 0.0, None, None), reason)

    def test_new_action_uuid_is_the_only_reset(self):
        guard = RotationGuard(GuardLimits(terminal_no_progress_s=1.0))
        guard.begin_goal("goal-a")
        guard.evaluate(0.0, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0))
        guard.evaluate(1.1, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0))
        self.assertTrue(guard.latched_reason)
        guard.begin_goal("goal-a")
        self.assertTrue(guard.latched_reason)
        guard.begin_goal("goal-b")
        self.assertFalse(guard.latched_reason)

    def test_global_guard_catches_recovery_spin_away_from_goal(self):
        guard = RotationGuard(GuardLimits(global_spin_limit_rad=1.0))
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        guard.evaluate(0.0, 0.0, 0.2, None, None)
        guard.observe_odom(0.6)
        guard.observe_odom(1.2)
        reason = guard.evaluate(1.0, 0.0, 0.2, None, None)
        self.assertEqual(reason, "continuous stationary rotation limit")

    def test_legitimate_half_turn_is_below_global_limit(self):
        guard = RotationGuard()
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        guard.evaluate(0.0, 0.0, 0.2, None, None)
        for i in range(1, 17):
            guard.observe_odom(i * math.pi / 16.0)
            self.assertEqual(guard.evaluate(i * 0.5, 0.0, 0.2, None, None), "")


if __name__ == "__main__":
    unittest.main()
