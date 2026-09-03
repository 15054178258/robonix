from __future__ import annotations

import ast
import importlib.util
import json
from pathlib import Path
import socket
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "offline_voice_e2e.py"
FIXTURE = ROOT / "tests" / "fixtures" / "offline_voice_e2e.yaml"


def _load_runner():
    spec = importlib.util.spec_from_file_location("go2_offline_voice_e2e", RUNNER)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load offline voice runner")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


offline_demo = _load_runner()


class OfflineVoiceEndToEndTests(unittest.TestCase):
    def test_required_phrase_reaches_read_only_dashboard_success(self) -> None:
        result = offline_demo.run_scenario(FIXTURE)
        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["asr_final"]["text"], "走到前面自动售货机那里")
        self.assertTrue(result["asr_final"]["is_final"])
        self.assertEqual(result["liaison_transcript"], "走到前面自动售货机那里")
        self.assertEqual(
            result["pilot_selection"]["contract_id"],
            "robonix/skill/semantic_navigation/navigate_landmark",
        )
        self.assertEqual(
            result["pilot_selection"]["arguments"], {"name": "自动售货机"}
        )
        self.assertTrue(result["landmark"]["verified"])
        self.assertEqual(result["landmark"]["map_generation"], 17)
        self.assertEqual(result["navigation"]["states"], ["RUNNING", "SUCCEEDED"])
        self.assertEqual(result["navigation"]["semantic_state"], "SUCCEEDED")
        self.assertTrue(result["navigation"]["remote_terminal"])
        self.assertTrue(result["dashboard"]["read_only"])
        self.assertFalse(result["dashboard"]["bridge_connected"])
        self.assertEqual(
            result["dashboard"]["semantic_task"]["status"], "succeeded"
        )
        self.assertTrue(
            result["dashboard"]["semantic_task"]["read_only_effect"]
        )

    def test_fixture_pose_is_verified_and_bound_to_active_generation(self) -> None:
        fixture = yaml.safe_load(FIXTURE.read_text(encoding="utf-8"))
        active = fixture["active_map"]
        document = fixture["landmark_document"]
        landmark = document["landmarks"][0]
        self.assertTrue(landmark["verified"])
        self.assertEqual(document["map_id"], active["map_id"])
        self.assertEqual(document["map_generation"], active["generation"])
        self.assertEqual(active["mode"], "localization")

    def test_contract_selection_comes_from_real_capability_descriptor(self) -> None:
        result = offline_demo.run_scenario(FIXTURE)
        descriptor = (
            ROOT
            / "packages"
            / "semantic_navigation"
            / "capabilities"
            / "navigate_landmark.v1.toml"
        ).read_text(encoding="utf-8")
        self.assertIn(
            f'id      = "{result["pilot_selection"]["contract_id"]}"', descriptor
        )
        idl = (
            ROOT
            / "packages"
            / "semantic_navigation"
            / "capabilities"
            / "lib"
            / "semantic_navigation"
            / "srv"
            / "NavigateLandmark.srv"
        ).read_text(encoding="utf-8")
        self.assertIn("string name", idl)
        soma = (ROOT / "soma.yaml").read_text(encoding="utf-8")
        for contract in result["navigation"]["contracts"]:
            self.assertIn(contract, soma)

    def test_trace_is_deterministic(self) -> None:
        first = offline_demo.run_scenario(FIXTURE)
        second = offline_demo.run_scenario(FIXTURE)
        self.assertEqual(
            json.dumps(first, ensure_ascii=False, sort_keys=True),
            json.dumps(second, ensure_ascii=False, sort_keys=True),
        )

    def test_runner_has_no_ros_network_process_or_motion_surface(self) -> None:
        source = RUNNER.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(RUNNER))
        forbidden_import_roots = {
            "grpc",
            "rclpy",
            "subprocess",
            "unitree_sdk2",
            "unitree_sdk2py",
        }
        forbidden_calls = {
            "ActionClient",
            "create_client",
            "create_node",
            "create_publisher",
            "create_subscription",
            "publish",
            "send_goal",
            "send_goal_async",
            "system",
        }
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for imported in node.names:
                    self.assertNotIn(
                        imported.name.split(".", 1)[0], forbidden_import_roots
                    )
            elif isinstance(node, ast.ImportFrom) and node.module:
                self.assertNotIn(
                    node.module.split(".", 1)[0], forbidden_import_roots
                )
            elif isinstance(node, ast.Call):
                if isinstance(node.func, ast.Attribute):
                    name = node.func.attr
                elif isinstance(node.func, ast.Name):
                    name = node.func.id
                else:
                    name = ""
                self.assertNotIn(name, forbidden_calls)

        forbidden_literals = (
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "SportClient",
            "StandUp",
            "StandDown",
            "RecoveryStand",
        )
        for literal in forbidden_literals:
            self.assertNotIn(literal, source)

        shell = (ROOT / "scripts" / "demo_offline_voice_e2e.sh").read_text(
            encoding="utf-8"
        )
        for command in ("ros2 ", "docker ", "ssh ", "curl ", "nc "):
            self.assertNotIn(command, shell)

    def test_runtime_socket_tripwire_stays_at_zero(self) -> None:
        original = socket.socket
        result = offline_demo.run_scenario(FIXTURE)
        self.assertIs(socket.socket, original)
        self.assertEqual(result["isolation"]["network_socket_attempts"], 0)
        self.assertFalse(result["isolation"]["ros_graph_started"])
        self.assertFalse(result["isolation"]["motion_interfaces_loaded"])
        self.assertEqual(result["isolation"]["publishers_created"], 0)


if __name__ == "__main__":
    unittest.main()
