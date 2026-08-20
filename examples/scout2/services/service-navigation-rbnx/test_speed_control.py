import math
import unittest

from nav2_wrapper.speed_control import (
    SpeedSettings,
    decide_adjustment,
    decide_explicit,
    speed_settings,
)


class SpeedSettingsTest(unittest.TestCase):
    def test_deployment_must_set_metric_maximum_and_normal_percentage(self):
        for cfg in ({}, {"dynamic_speed": {}}, {"dynamic_speed": []}):
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                speed_settings(cfg)

    def test_deploy_can_select_metric_maximum_and_relative_step(self):
        settings = speed_settings(
            {
                "dynamic_speed": {
                    "max_linear_speed_mps": 0.3,
                    "default_percentage": 75,
                    "step_percentage": 15,
                    "min_percentage": 25,
                    "topic": "/robot/navigation_speed_limit",
                }
            }
        )
        self.assertEqual(
            settings,
            SpeedSettings(
                0.3,
                75.0,
                15.0,
                25.0,
                "/robot/navigation_speed_limit",
            ),
        )

    def test_rejects_invalid_policy(self):
        valid_required = {
            "max_linear_speed_mps": 0.3,
            "default_percentage": 75,
        }
        invalid = (
            {"dynamic_speed": {**valid_required, "unknown": 1}},
            {"dynamic_speed": {**valid_required, "max_linear_speed_mps": 0}},
            {"dynamic_speed": {**valid_required, "default_percentage": 101}},
            {"dynamic_speed": {**valid_required, "min_percentage": 0}},
            {"dynamic_speed": {**valid_required, "step_percentage": math.inf}},
            {"dynamic_speed": {**valid_required, "topic": "speed_limit"}},
        )
        for cfg in invalid:
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                speed_settings(cfg)


class SpeedDecisionTest(unittest.TestCase):
    def setUp(self):
        self.settings = SpeedSettings(
            max_linear_speed_mps=0.3,
            default_percentage=75.0,
            step_percentage=20.0,
            min_percentage=20.0,
        )

    def test_relative_commands_change_percentage_and_metric_limit(self):
        slower = decide_adjustment(75.0, "slower", self.settings)
        faster = decide_adjustment(
            slower.percentage,
            "faster",
            self.settings,
        )
        self.assertEqual(
            (slower.percentage, slower.linear_speed_mps),
            (55.0, 0.165),
        )
        self.assertEqual(
            (faster.percentage, faster.linear_speed_mps),
            (75.0, 0.225),
        )

    def test_relative_commands_are_safely_bounded(self):
        self.assertEqual(
            decide_adjustment(95.0, "faster", self.settings).percentage,
            100.0,
        )
        decision = decide_adjustment(20.0, "slower", self.settings)
        self.assertEqual(decision.percentage, 20.0)
        self.assertFalse(decision.changed)

    def test_explicit_and_normal_commands(self):
        self.assertEqual(
            decide_explicit(75.0, 42.5, self.settings).percentage,
            42.5,
        )
        self.assertEqual(
            decide_adjustment(42.5, "normal", self.settings).percentage,
            75.0,
        )

    def test_rejects_unknown_or_unsafe_values(self):
        for direction in ("", "fastest"):
            with self.subTest(direction=direction), self.assertRaises(ValueError):
                decide_adjustment(75.0, direction, self.settings)
        for value in (19.9, 100.1, math.nan):
            with self.subTest(value=value), self.assertRaises(ValueError):
                decide_explicit(75.0, value, self.settings)


if __name__ == "__main__":
    unittest.main()
