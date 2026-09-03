from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import yaml

from go2_dashboard.initial_pose_store import InitialPoseError, InitialPoseStore


def _pose(x: float = 1.25) -> dict:
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.25
    covariance[35] = 0.068
    return {
        "frame_id": "map",
        "position": {"x": x, "y": -0.5, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "covariance": covariance,
    }


class InitialPoseStoreTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.maps = Path(self.temporary.name)
        map_dir = self.maps / "lab-map"
        map_dir.mkdir()
        (map_dir / "rtabmap.db").write_bytes(b"sqlite-placeholder")
        (map_dir / "generation").write_text("7\n", encoding="utf-8")
        self.store = InitialPoseStore(self.maps)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_save_restore_and_recoverable_reset_are_generation_bound(self) -> None:
        status = self.store.observe_lifecycle("lab-map", "localization", 7)
        self.assertFalse(status["saved"])
        saved = self.store.save_operator_pose(_pose())
        self.assertTrue(saved["saved"])
        sidecar = self.maps / "lab-map.initial-pose.operator.yaml"
        self.assertTrue(sidecar.is_file())
        self.assertEqual(yaml.safe_load(sidecar.read_text())["map_generation"], 7)

        restored = self.store.restore_pose(map_id="lab-map", generation=7)
        self.assertEqual(restored["position"]["x"], 1.25)
        with self.assertRaisesRegex(InitialPoseError, "does not match"):
            self.store.restore_pose(map_id="lab-map", generation=8)

        reset = self.store.reset(confirm_map_id="lab-map", generation=7)
        self.assertFalse(reset["saved"])
        self.assertFalse(sidecar.exists())
        self.assertEqual(
            len(list(self.maps.glob("lab-map.initial-pose.operator.disabled.*.yaml"))),
            1,
        )

    def test_overwrite_archives_the_previous_operator_seed(self) -> None:
        self.store.observe_lifecycle("lab-map", "localization", 7)
        self.store.save_operator_pose(_pose(1.0))
        self.store.save_operator_pose(_pose(2.0))
        self.assertEqual(
            len(list(self.maps.glob("lab-map.initial-pose.operator.2*.yaml"))),
            1,
        )
        current = self.store.restore_pose(map_id="lab-map", generation=7)
        self.assertEqual(current["position"]["x"], 2.0)

    def test_live_and_disk_generation_must_match(self) -> None:
        status = self.store.observe_lifecycle("lab-map", "localization", 8)
        self.assertFalse(status["saved"])
        self.assertIn("generation", status["error"])
        with self.assertRaisesRegex(InitialPoseError, "generation"):
            self.store.save_operator_pose(_pose())

    def test_mapping_mode_cannot_save_or_restore_localization_seed(self) -> None:
        self.store.observe_lifecycle("lab-map", "mapping", 7)
        with self.assertRaisesRegex(InitialPoseError, "localization mode"):
            self.store.save_operator_pose(_pose())
        with self.assertRaisesRegex(InitialPoseError, "localization mode"):
            self.store.restore_pose(map_id="lab-map", generation=7)

    def test_legacy_v1_sidecar_without_covariance_remains_readable(self) -> None:
        document = {
            "schema": "robonix-go2-operator-initial-pose-v1",
            "map_id": "lab-map",
            "map_generation": 7,
            "frame_id": "map",
            "pose": {
                "position": _pose()["position"],
                "orientation": _pose()["orientation"],
            },
            "covariance_hint": {"xy_variance": 0.25, "yaw_variance": 0.068},
            "source": {"kind": "legacy"},
        }
        (self.maps / "lab-map.initial-pose.operator.yaml").write_text(
            yaml.safe_dump(document), encoding="utf-8"
        )
        status = self.store.observe_lifecycle("lab-map", "localization", 7)
        self.assertTrue(status["saved"])
        restored = self.store.restore_pose(map_id="lab-map", generation=7)
        self.assertEqual(len(restored["covariance"]), 36)


if __name__ == "__main__":
    unittest.main()
