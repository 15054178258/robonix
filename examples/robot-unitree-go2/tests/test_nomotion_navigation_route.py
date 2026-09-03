from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class NoMotionNavigationRouteTest(unittest.TestCase):
    def test_manifest_routes_navigation_through_gate_owned_value(self) -> None:
        manifest = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            manifest["env"]["ROBONIX_VELOCITY_OUTPUT_TOPIC"],
            "${ROBONIX_VELOCITY_OUTPUT_TOPIC}",
        )
        navigation = next(
            row for row in manifest["service"] if row["name"] == "nav2"
        )
        self.assertEqual(
            navigation["config"]["velocity_output_topic"],
            "${ROBONIX_VELOCITY_OUTPUT_TOPIC}",
        )

    def test_build_and_start_derive_both_routes_after_gate_checks(self) -> None:
        for relative in ("build.sh", "start.sh"):
            with self.subTest(relative=relative):
                source = (ROOT / relative).read_text(encoding="utf-8")
                canonical = "export ROBONIX_VELOCITY_OUTPUT_TOPIC=/cmd_vel"
                no_motion = (
                    "export ROBONIX_VELOCITY_OUTPUT_TOPIC="
                    "/robonix/nomotion/cmd_vel"
                )
                self.assertIn(canonical, source)
                self.assertIn(no_motion, source)
                self.assertGreater(
                    source.index(canonical),
                    source.index("GO2_ALLOWED_MODES"),
                )
                self.assertNotIn("${ROBONIX_VELOCITY_OUTPUT_TOPIC:-", source)

    def test_nav2_composition_is_only_enabled_by_nomotion_renderer(self) -> None:
        manifest = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        navigation = next(
            row for row in manifest["service"] if row["name"] == "nav2"
        )
        self.assertFalse(navigation["config"].get("use_composition", False))
        renderer = (
            ROOT / "deploy" / "time-sync" /
            "render_workstation_nomotion_manifest.py"
        ).read_text(encoding="utf-8")
        self.assertIn('nav_cfg["use_composition"] = True', renderer)

    def test_pinned_navigation_satisfies_velocity_contract(self) -> None:
        result = subprocess.run(
            [
                "bash",
                str(ROOT / "scripts/check_navigation_velocity_contract.sh"),
                str(ROOT / "third_party/service-navigation-rbnx"),
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("PASS", result.stdout)

    def test_legacy_fixed_cmd_vel_cache_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "nav2_wrapper").mkdir()
            (root / "config.spec").write_text(
                "velocity_output_topic:\n", encoding="utf-8"
            )
            (root / "nav2_wrapper/configuration.py").write_text(
                "def resolve_velocity_output_topic(value):\n    return value\n",
                encoding="utf-8",
            )
            (root / "nav2_wrapper/atlas_bridge.py").write_text(
                "resolve_velocity_output_topic(cfg)\n", encoding="utf-8"
            )
            (root / "nav2_wrapper/velocity_guard.py").write_text(
                'create_publisher(Twist, "/cmd_vel", 10)\n', encoding="utf-8"
            )
            result = subprocess.run(
                [
                    "bash",
                    str(ROOT / "scripts/check_navigation_velocity_contract.sh"),
                    str(root),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("resolved velocity publisher", result.stderr)


if __name__ == "__main__":
    unittest.main()
