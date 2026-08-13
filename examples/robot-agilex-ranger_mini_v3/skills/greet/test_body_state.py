# SPDX-License-Identifier: Apache-2.0
from types import SimpleNamespace
import unittest

from greet_skill.body_state import snapshot_is_moving, snapshot_motion_state


def metric(component_id: str, moving: float, quality: int = 0):
    return SimpleNamespace(
        component_id=component_id,
        name="moving",
        value=SimpleNamespace(value=moving, quality=quality),
    )


class SnapshotMotionTest(unittest.TestCase):
    def test_reads_selected_chassis_only(self):
        snapshot = SimpleNamespace(metrics=[
            metric("body/chassis/other", 1.0),
            metric("body/chassis/ranger_chassis", 0.0),
        ])
        self.assertFalse(snapshot_is_moving(snapshot, "ranger_chassis"))

    def test_valid_moving_metric_is_true(self):
        snapshot = SimpleNamespace(
            metrics=[metric("body/chassis/ranger_chassis", 1.0)]
        )
        self.assertTrue(snapshot_is_moving(snapshot, "ranger_chassis"))
        self.assertEqual(snapshot_motion_state(snapshot, "ranger_chassis"), "moving")

    def test_valid_zero_metric_is_stationary(self):
        snapshot = SimpleNamespace(
            metrics=[metric("body/chassis/ranger_chassis", 0.0)]
        )
        self.assertEqual(
            snapshot_motion_state(snapshot, "ranger_chassis"), "stationary"
        )

    def test_stale_or_missing_state_pauses(self):
        stale = SimpleNamespace(
            metrics=[metric("body/chassis/ranger_chassis", 1.0, quality=1)]
        )
        self.assertFalse(snapshot_is_moving(stale, "ranger_chassis"))
        self.assertFalse(snapshot_is_moving(SimpleNamespace(metrics=[]), "ranger_chassis"))
        self.assertEqual(
            snapshot_motion_state(stale, "ranger_chassis"), "unavailable"
        )
        self.assertEqual(
            snapshot_motion_state(SimpleNamespace(metrics=[]), "ranger_chassis"),
            "unavailable",
        )


if __name__ == "__main__":
    unittest.main()
