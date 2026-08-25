from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    ROOT
    / "deploy"
    / "time-sync"
    / "render_workstation_second_motion_manifest.py"
)
SPEC = importlib.util.spec_from_file_location(
    "second_motion_renderer",
    MODULE_PATH,
)
assert SPEC is not None and SPEC.loader is not None
renderer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = renderer
SPEC.loader.exec_module(renderer)


class SecondMotionManifestRendererTest(unittest.TestCase):
    def setUp(self) -> None:
        self.base = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )

    def test_render_is_private_and_uses_exact_second_envelope(self) -> None:
        result = renderer.render(self.base, ROOT)
        self.assertEqual(
            result["name"],
            "robonix-go2-private-second-motion-corrected",
        )
        self.assertEqual(
            result["env"]["GO2_MOTION_PROFILE"],
            renderer.PROFILE,
        )
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
        self.assertEqual(
            config["twist_in_topic"],
            "/go2/second_motion/cmd_vel",
        )
        self.assertNotEqual(config["twist_in_topic"], "/cmd_vel")
        self.assertEqual(config["odom_source"], "external_verified")
        self.assertEqual(
            config["external_odom_topic"],
            "/robonix/time_corrected/raw/utlidar/robot_odom",
        )
        self.assertEqual(config["external_odom_timeout_s"], 0.20)
        self.assertEqual(config["odom_topic"], "/odom")
        self.assertTrue(config["publish_odom_tf"])
        self.assertFalse(config["stationary_pose_hold_enabled"])
        self.assertTrue(config["allow_motion"])
        self.assertEqual(config["motion_profile"], renderer.PROFILE)
        self.assertEqual(config["allowed_modes"], [255])
        self.assertNotIn("allowed_state_markers", config)
        self.assertEqual(config["max_linear_x_mps"], 0.30)
        self.assertEqual(config["max_linear_y_mps"], 0.0)
        self.assertEqual(config["max_angular_z_rps"], 0.0)
        self.assertEqual(config["max_linear_accel_mps2"], 0.30)
        self.assertEqual(config["max_angular_accel_rps2"], 0.10)
        self.assertEqual(config["command_timeout_s"], 0.20)
        self.assertEqual(config["control_rate_hz"], 20.0)
        self.assertEqual(config["state_timeout_s"], 0.20)
        self.assertEqual(config["max_source_stamp_age_s"], 0.20)
        self.assertEqual(config["max_source_stamp_future_skew_s"], 0.05)
        self.assertEqual(config["commissioning_max_duration_s"], 1.5)
        self.assertEqual(config["commissioning_max_distance_m"], 0.30)

    def test_validator_rejects_route_or_envelope_drift(self) -> None:
        result = renderer.render(self.base, ROOT)
        config = result["primitive"][0]["config"]
        for key, value, failure in (
            ("twist_in_topic", "/cmd_vel", "private_command"),
            ("external_odom_timeout_s", 0.19, "external_odom_timeout"),
            ("stationary_pose_hold_enabled", True, "stationary_pose_hold"),
            ("max_linear_x_mps", 0.299, "vx"),
            ("control_rate_hz", 50.0, "control_rate"),
            ("commissioning_max_duration_s", 1.49, "duration"),
            ("commissioning_max_distance_m", 0.29, "distance"),
        ):
            original = config[key]
            config[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(
                renderer.ManifestError,
                failure,
            ):
                renderer.validate_rendered(result, ROOT)
            config[key] = original

    def test_soma_and_outputs_remain_checkout_local_and_private(self) -> None:
        base_soma = yaml.safe_load(
            (ROOT / "soma.yaml").read_text(encoding="utf-8")
        )
        soma = renderer.render_soma(base_soma, ROOT)
        self.assertEqual(
            soma["urdf"]["path"],
            str(ROOT / "packages/go2_description/urdf/go2_robonix.urdf"),
        )
        with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as temporary:
            directory = Path(temporary)
            directory.chmod(0o700)
            manifest_path = directory / "robonix_manifest.yaml"
            soma_path = directory / "soma.yaml"
            renderer._atomic_yaml(
                manifest_path,
                renderer.render(self.base, ROOT),
            )
            renderer._atomic_yaml(soma_path, soma)
            self.assertEqual(manifest_path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(soma_path.stat().st_mode & 0o777, 0o600)


if __name__ == "__main__":
    unittest.main()
