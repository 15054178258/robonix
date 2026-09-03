from __future__ import annotations

import ast
import argparse
import re
import unittest
from pathlib import Path

from go2_dashboard.main import _loopback_host


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "go2_dashboard"


class ReadOnlySafetyTests(unittest.TestCase):
    def test_dashboard_cli_rejects_non_loopback_binding(self) -> None:
        self.assertEqual(_loopback_host("127.0.0.1"), "127.0.0.1")
        with self.assertRaises(argparse.ArgumentTypeError):
            _loopback_host("0.0.0.0")

    def test_runtime_only_adds_the_explicit_localization_seed_publisher(self) -> None:
        subscription_count = 0
        publisher_count = 0
        publish_count = 0
        forbidden_calls = {
            "ActionClient",
            "send_goal",
            "send_goal_async",
            "create_client",
        }
        forbidden_imports = {"Twist", "TwistStamped", "unitree_sdk2py"}
        for path in RUNTIME.rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.Call):
                    if isinstance(node.func, ast.Attribute):
                        call_name = node.func.attr
                    elif isinstance(node.func, ast.Name):
                        call_name = node.func.id
                    else:
                        call_name = ""
                    self.assertNotIn(call_name, forbidden_calls, str(path))
                    if call_name == "create_subscription":
                        subscription_count += 1
                    if call_name == "create_publisher":
                        publisher_count += 1
                    if call_name == "publish":
                        publish_count += 1
                if isinstance(node, ast.ImportFrom):
                    for imported in node.names:
                        self.assertNotIn(imported.name, forbidden_imports, str(path))
                if isinstance(node, ast.Import):
                    for imported in node.names:
                        self.assertNotIn(imported.name, forbidden_imports, str(path))
        self.assertGreaterEqual(subscription_count, 6)
        self.assertEqual(publisher_count, 1)
        self.assertEqual(publish_count, 1)
        bridge = (RUNTIME / "ros_bridge.py").read_text(encoding="utf-8")
        self.assertIn("config.initial_pose_topic", bridge)
        self.assertIn("PoseWithCovarianceStamped", bridge)
        store = (RUNTIME / "initial_pose_store.py").read_text(encoding="utf-8")
        self.assertIn('"motion_command": False', store)

    def test_runtime_has_no_known_robot_command_routes(self) -> None:
        forbidden_literals = (
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "SportClient",
            "sport_mode_ctrl",
            "go2_sport_client",
            "go2_stand_example",
            "low_level_ctrl",
        )
        runtime_files = [
            *RUNTIME.rglob("*.py"),
            *RUNTIME.rglob("*.html"),
        ]
        runtime_text = "\n".join(
            path.read_text(encoding="utf-8") for path in runtime_files
        )
        for literal in forbidden_literals:
            self.assertNotIn(literal, runtime_text)

    def test_web_page_has_voice_and_localization_seed_buttons_only(self) -> None:
        page = (RUNTIME / "static" / "index.html").read_text(encoding="utf-8").lower()
        self.assertEqual(page.count("<button"), 3)
        self.assertIn('id="voicebutton"', page)
        self.assertIn('id="restoreposebutton"', page)
        self.assertIn('id="resetposebutton"', page)
        self.assertIn('type="button" disabled', page)
        self.assertNotIn("<form", page)
        self.assertNotIn("<input", page)
        self.assertIn("read-only diagnostic / not navigation ready", page)
        self.assertIn("不得作为导航验收证据", page)
        self.assertIn("blocked · no call", page)
        self.assertIn('voice.execution_mode === "preview"', page)
        self.assertIn('await context.resume()', page)
        self.assertIn("未采集到麦克风音频", page)
        self.assertIn("function freshmappose(data)", page)
        self.assertIn("!data?.bridge?.connected", page)
        self.assertIn('topic?.state !== "fresh"', page)
        self.assertIn("freshmappose(latest)", page)
        self.assertIn("const pose = freshmappose(data)", page)
        self.assertIn("位姿已过期", page)
        self.assertIn("位姿错误", page)
        self.assertIn("function mapavailability(data)", page)
        self.assertIn("当前地图快照 · fresh", page)
        self.assertIn("缓存快照 · ros 已断开", page)
        self.assertIn("缓存快照 · 地图错误", page)
        self.assertIn("mapdegraded", page)

    def test_browser_voice_gateway_only_delegates_to_liaison(self) -> None:
        source = (RUNTIME / "voice_gateway.py").read_text(encoding="utf-8")
        self.assertIn("RobonixSystemLiaisonVoiceStub", source)
        self.assertIn("StartVoiceSession", source)
        self.assertIn("capability_calls_observed", source)
        self.assertIn("预览安全策略违规", source)
        for forbidden in (
            "robonix/service/navigation",
            "robonix/service/speech/asr",
            "unitree_sdk2py",
            "rclpy",
            "ActionClient",
        ):
            self.assertNotIn(forbidden, source)

    def test_shell_scripts_do_not_invoke_ros_or_unitree_commands(self) -> None:
        shell_text = "\n".join(
            path.read_text(encoding="utf-8") for path in (ROOT / "scripts").glob("*.sh")
        )
        forbidden_patterns = (
            r"ros2\s+topic\s+pub",
            r"ros2\s+action\s+send_goal",
            r"sport_mode_ctrl",
            r"go2_sport_client",
            r"go2_stand_example",
            r"low_level_ctrl",
        )
        for pattern in forbidden_patterns:
            self.assertIsNone(re.search(pattern, shell_text), pattern)

    def test_manifest_declares_only_driver_and_readonly_status(self) -> None:
        manifest = (ROOT / "package_manifest.yaml").read_text(encoding="utf-8")
        self.assertIn("robonix/service/telemetry/dashboard/driver", manifest)
        self.assertIn("robonix/service/telemetry/dashboard/status", manifest)
        self.assertEqual(manifest.count("  - name:"), 2)

    def test_provider_identity_and_contracts_are_exact(self) -> None:
        provider = (RUNTIME / "service.py").read_text(encoding="utf-8")
        self.assertIn('PROVIDER_ID = "go2_dashboard"', provider)
        self.assertIn(
            'NAMESPACE = "robonix/service/telemetry/dashboard"', provider
        )
        driver = (ROOT / "capabilities" / "driver.v1.toml").read_text(
            encoding="utf-8"
        )
        status = (ROOT / "capabilities" / "status.v1.toml").read_text(
            encoding="utf-8"
        )
        self.assertIn('idl = "lifecycle/srv/Driver.srv"', driver)
        self.assertIn('idl = "go2_dashboard/srv/GetDashboardStatus.srv"', status)

    def test_start_runs_provider_instead_of_dashboard_child(self) -> None:
        start = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertIn("-m go2_dashboard.service", start)
        self.assertNotIn("-m go2_dashboard.main", start)

    def test_build_requests_mcp_codegen(self) -> None:
        build = (ROOT / "scripts" / "build.sh").read_text(encoding="utf-8")
        self.assertRegex(build, r"codegen\s+-p\s+.*--mcp")


if __name__ == "__main__":
    unittest.main()
