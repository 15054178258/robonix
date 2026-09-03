from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    ROOT / "deploy" / "time-sync" / "render_workstation_first_motion_manifest.py"
)
SPEC = importlib.util.spec_from_file_location("first_motion_renderer", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
renderer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = renderer
SPEC.loader.exec_module(renderer)


class FirstMotionManifestRendererTest(unittest.TestCase):
    def setUp(self) -> None:
        self.base = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )

    def test_render_keeps_only_guarded_chassis_and_exact_envelope(self) -> None:
        result = renderer.render(self.base, ROOT)
        self.assertEqual(set(result["system"]), {"atlas", "executor", "soma"})
        self.assertEqual(result["service"], [])
        self.assertEqual(result["skill"], [])
        self.assertEqual(len(result["primitive"]), 1)
        chassis = result["primitive"][0]
        config = chassis["config"]
        self.assertEqual(chassis["path"], str(ROOT / "packages/go2_chassis"))
        self.assertEqual(
            config["state_topic"],
            "/robonix/time_corrected/motion/sportmodestate",
        )
        self.assertEqual(config["state_fallback_topic"], "")
        self.assertEqual(config["twist_in_topic"], "/go2/commissioning/cmd_vel")
        self.assertNotEqual(config["twist_in_topic"], "/cmd_vel")
        self.assertEqual(config["odom_source"], "external_verified")
        self.assertEqual(
            config["external_odom_topic"],
            "/robonix/time_corrected/raw/utlidar/robot_odom",
        )
        self.assertEqual(config["odom_topic"], "/odom")
        self.assertTrue(config["publish_odom_tf"])
        self.assertFalse(config["stationary_pose_hold_enabled"])
        self.assertEqual(config["stationary_hold_dwell_s"], 2.0)
        self.assertEqual(config["stationary_hold_sport_max_linear_mps"], 0.03)
        self.assertEqual(config["stationary_hold_sport_max_yaw_rps"], 0.03)
        self.assertEqual(
            config["stationary_hold_external_twist_max_linear_mps"], 0.03
        )
        self.assertEqual(
            config["stationary_hold_external_twist_max_yaw_rps"], 0.03
        )
        self.assertEqual(
            config["stationary_hold_pose_max_linear_rate_mps"], 0.005
        )
        self.assertEqual(config["stationary_hold_pose_max_yaw_rate_rps"], 0.01)
        self.assertTrue(config["allow_motion"])
        self.assertEqual(config["motion_profile"], renderer.PROFILE)
        self.assertEqual(config["allowed_modes"], [255])
        self.assertNotIn("allowed_state_markers", config)
        self.assertEqual(config["max_linear_x_mps"], 0.05)
        self.assertEqual(config["max_linear_y_mps"], 0.0)
        self.assertEqual(config["max_angular_z_rps"], 0.0)
        self.assertEqual(config["command_timeout_s"], 0.20)
        self.assertEqual(config["state_timeout_s"], 0.20)
        self.assertEqual(config["max_source_stamp_age_s"], 0.20)
        self.assertEqual(config["commissioning_max_duration_s"], 2.0)
        self.assertEqual(config["commissioning_max_distance_m"], 0.10)

    def test_render_forces_stationary_pose_hold_off_and_validator_enforces_it(
        self,
    ) -> None:
        poisoned = yaml.safe_load(yaml.safe_dump(self.base))
        next(
            item
            for item in poisoned["primitive"]
            if item["name"] == "go2_chassis"
        )["config"]["stationary_pose_hold_enabled"] = True
        next(
            item
            for item in poisoned["primitive"]
            if item["name"] == "go2_chassis"
        )["config"]["stationary_hold_pose_max_yaw_rate_rps"] = 99.0

        rendered = renderer.render(poisoned, ROOT)
        config = rendered["primitive"][0]["config"]
        self.assertFalse(config["stationary_pose_hold_enabled"])
        self.assertEqual(config["stationary_hold_pose_max_yaw_rate_rps"], 0.01)

        config["stationary_pose_hold_enabled"] = True
        with self.assertRaisesRegex(
            renderer.ManifestError, "stationary_pose_hold_disabled"
        ):
            renderer.validate_rendered(rendered, ROOT)

        config["stationary_pose_hold_enabled"] = False
        config["stationary_hold_pose_max_yaw_rate_rps"] = 0.02
        with self.assertRaisesRegex(
            renderer.ManifestError, "stationary_hold_pose_yaw"
        ):
            renderer.validate_rendered(rendered, ROOT)

        config["stationary_hold_pose_max_yaw_rate_rps"] = 0.01
        config["max_source_stamp_age_s"] = 0.10
        with self.assertRaisesRegex(
            renderer.ManifestError, "source_stamp_age"
        ):
            renderer.validate_rendered(rendered, ROOT)

    def test_soma_sidecar_pins_urdf_to_this_checkout(self) -> None:
        base_soma = yaml.safe_load(
            (ROOT / "soma.yaml").read_text(encoding="utf-8")
        )
        result = renderer.render_soma(base_soma, ROOT)
        self.assertEqual(
            result["urdf"]["path"],
            str(ROOT / "packages/go2_description/urdf/go2_robonix.urdf"),
        )

    def test_outputs_are_private(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as temporary:
            directory = Path(temporary)
            directory.chmod(0o700)
            manifest = directory / "robonix_manifest.yaml"
            soma = directory / "soma.yaml"
            renderer._atomic_yaml(manifest, renderer.render(self.base, ROOT))
            renderer._atomic_yaml(
                soma,
                renderer.render_soma(
                    yaml.safe_load((ROOT / "soma.yaml").read_text()), ROOT
                ),
            )
            self.assertEqual(manifest.stat().st_mode & 0o777, 0o600)
            self.assertEqual(soma.stat().st_mode & 0o777, 0o600)


if __name__ == "__main__":
    unittest.main()
