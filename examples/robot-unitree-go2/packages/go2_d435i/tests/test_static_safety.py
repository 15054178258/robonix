from __future__ import annotations

import ast
from pathlib import Path
import re
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "go2_d435i_provider" / "main.py"
OBSERVER = ROOT / "go2_d435i_provider" / "observer.py"
QUALITY = ROOT / "go2_d435i_provider" / "quality.py"


class StaticSafetyTests(unittest.TestCase):
    def test_manifest_and_runtime_capabilities_are_exact(self) -> None:
        manifest = yaml.safe_load(
            (ROOT / "package_manifest.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            set(manifest),
            {
                "manifestVersion",
                "package",
                "build",
                "start",
                "capabilities",
                "depends",
            },
        )
        self.assertEqual(manifest["manifestVersion"], 1)
        self.assertEqual(manifest["build"], "bash build.sh")
        self.assertEqual(manifest["start"], "bash start.sh")
        self.assertEqual(manifest["depends"], [])
        capabilities = {item["name"] for item in manifest["capabilities"]}
        self.assertEqual(
            capabilities,
            {
                "robonix/primitive/camera/driver",
                "robonix/primitive/camera/rgb",
                "robonix/primitive/camera/depth",
                "robonix/primitive/camera/intrinsics",
            },
        )
        source = MAIN.read_text(encoding="utf-8")
        tree = ast.parse(source)
        data_capabilities = None
        for statement in tree.body:
            if (
                isinstance(statement, ast.Assign)
                and any(
                    isinstance(target, ast.Name)
                    and target.id == "DATA_CAPABILITIES"
                    for target in statement.targets
                )
            ):
                data_capabilities = ast.literal_eval(statement.value)
        self.assertIsNotNone(data_capabilities)
        self.assertEqual(
            {entry[0] for entry in data_capabilities},
            capabilities - {"robonix/primitive/camera/driver"},
        )

    def test_runtime_has_no_process_or_control_surface(self) -> None:
        sources = "\n".join(
            path.read_text(encoding="utf-8") for path in (MAIN, OBSERVER, QUALITY)
        )
        for forbidden in (
            "create_publisher",
            "create_client",
            "create_service",
            "ActionClient",
            "provider.spawn",
            "subprocess",
            "os.system",
            "ros2 launch",
            "realsense2_camera",
            "SportClient",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, sources)

    def test_observer_uses_low_level_subscription_only_handles(self) -> None:
        source = OBSERVER.read_text(encoding="utf-8")
        self.assertIn("_rclpy.Node(", source)
        self.assertIn("_rclpy.Subscription(", source)
        self.assertIn("_rclpy.WaitSet(3, 0, 0, 0, 0, 0", source)
        self.assertNotIn("from rclpy.node import Node", source)
        self.assertNotIn(".create_subscription(", source)
        self.assertIn('"ros_publishers_created": False', source)

    def test_scripts_do_not_install_or_change_networking(self) -> None:
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "build.sh", ROOT / "start.sh")
        )
        for forbidden in (
            r"\bsudo\b",
            r"\bapt(?:-get)?\b",
            r"\bnmcli\b",
            r"\bip\s+(?:addr|route|link)\b",
            r"\bsystemctl\b",
            r"\bdocker\b",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertIsNone(re.search(forbidden, sources))

    def test_build_produces_the_ros_overlay_required_by_start(self) -> None:
        build = (ROOT / "build.sh").read_text(encoding="utf-8")
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        required_setup = (
            '${ROOT}/rbnx-build/codegen/ros2_idl/install/setup.bash'
        )
        self.assertIn(
            'source "${DEPLOY_ROOT}/scripts/build_robonix_ros2_overlay.sh"',
            build,
        )
        self.assertIn(
            'robonix_build_ros2_overlay "${ROBONIX_IDL_ROOT}"',
            build,
        )
        self.assertIn("--packages-select lifecycle", build)
        self.assertIn('[[ ! -r "${ROBONIX_IDL_SETUP}" ]]', build)
        self.assertIn(required_setup, start)

    def test_capability_frontmatter_has_only_description(self) -> None:
        source = (ROOT / "CAPABILITY.md").read_text(encoding="utf-8")
        match = re.match(r"\A---\n(.*?)\n---\n", source, re.DOTALL)
        self.assertIsNotNone(match)
        frontmatter = yaml.safe_load(match.group(1))
        self.assertEqual(set(frontmatter), {"description"})
        for forbidden in ("extrinsics", "snapshot", "depth_snapshot"):
            self.assertNotIn(
                f"- `robonix/primitive/camera/{forbidden}`",
                source,
            )


if __name__ == "__main__":
    unittest.main()
