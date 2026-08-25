from __future__ import annotations

import ast
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "gate2_replay.py"
SPEC = importlib.util.spec_from_file_location("gate2_replay", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
gate2 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate2)


class Gate2ReplayTest(unittest.TestCase):
    def setUp(self) -> None:
        self.raw_scenario = yaml.safe_load(
            (ROOT / "config" / "gate2_replay.example.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.scenario = gate2.validate_scenario(copy.deepcopy(self.raw_scenario))
        self.fixture = json.loads(
            (ROOT / "tests" / "fixtures" / "gate2_evidence.json").read_text(
                encoding="utf-8"
            )
        )

    def test_example_is_valid_and_repository_local(self) -> None:
        self.assertEqual(self.scenario["runtime"]["domain_id"], 211)
        self.assertTrue(
            Path(self.scenario["source"]["bag"]).is_relative_to(ROOT)
        )
        self.assertTrue(
            Path(self.scenario["map"]["database"]).is_relative_to(ROOT)
        )
        self.assertEqual(self.scenario["topics"]["cmd_vel"], "/cmd_vel")

    def test_scenario_rejects_motion_and_generated_replay_topics(self) -> None:
        for topic in ("/cmd_vel", "/lowcmd", "/api/sport/request", "/map"):
            with self.subTest(topic=topic):
                raw = copy.deepcopy(self.raw_scenario)
                raw["source"]["replay_topics"].append(
                    {
                        "topic": topic,
                        "type": "geometry_msgs/msg/Twist",
                        "required": True,
                    }
                )
                with self.assertRaises(gate2.ConfigurationError):
                    gate2.validate_scenario(raw)

    def test_scenario_rejects_paths_outside_repository(self) -> None:
        for key, value in (("bag", "/tmp/capture"), ("bag", "../../outside")):
            with self.subTest(value=value):
                raw = copy.deepcopy(self.raw_scenario)
                raw["source"][key] = value
                with self.assertRaises(gate2.ConfigurationError):
                    gate2.validate_scenario(raw)

    def test_isolated_environment_strips_live_transport_configuration(self) -> None:
        old = os.environ.copy()
        try:
            os.environ["GO2_NETWORK_INTERFACE"] = "enp3s0"
            os.environ["GO2_ALLOW_MOTION"] = "true"
            os.environ["UNITREE_DOMAIN"] = "live"
            with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as temp:
                environment = gate2.isolated_environment(
                    self.scenario, Path(temp)
                )
        finally:
            os.environ.clear()
            os.environ.update(old)
        self.assertNotIn("GO2_NETWORK_INTERFACE", environment)
        self.assertNotIn("UNITREE_DOMAIN", environment)
        self.assertEqual(environment["GO2_ALLOW_MOTION"], "false")
        self.assertEqual(environment["ROS_LOCALHOST_ONLY"], "1")
        self.assertEqual(environment["ROS2CLI_NO_DAEMON"], "1")
        self.assertIn("AllowMulticast>false", environment["CYCLONEDDS_URI"])
        self.assertNotIn("NetworkInterface", environment["CYCLONEDDS_URI"])

    def test_fixture_shape_passes_every_measurement_except_authenticity(self) -> None:
        evaluation = gate2.evaluate_evidence(self.scenario, self.fixture)
        failures = [
            item["name"] for item in evaluation["checks"] if not item["passed"]
        ]
        self.assertEqual(failures, ["real_rosbag_source"])
        self.assertFalse(evaluation["acceptance_pass"])

        measured = copy.deepcopy(self.fixture)
        measured["source"] = {"kind": "rosbag", "sha256": "a" * 64}
        self.assertTrue(
            gate2.evaluate_evidence(self.scenario, measured)["acceptance_pass"]
        )

    def test_fixture_cli_can_never_return_success(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(MODULE_PATH),
                "--scenario",
                str(ROOT / "config" / "gate2_replay.example.yaml"),
                "--fixture-evidence",
                str(ROOT / "tests" / "fixtures" / "gate2_evidence.json"),
            ],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, gate2.SKIP_EXIT, result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["status"], "FIXTURE_ONLY")
        self.assertFalse(payload["acceptance_pass"])

    def test_stop_check_rejects_permanently_zero_fixture(self) -> None:
        evidence = copy.deepcopy(self.fixture)
        for sample in evidence["cmd_vel_sink"]["samples"]:
            sample["x"] = sample["y"] = sample["z"] = 0.0
        evaluation = gate2.evaluate_evidence(self.scenario, evidence)
        stop = next(
            item
            for item in evaluation["checks"]
            if item["name"] == "sensor_loss_stops_cmd_vel"
        )
        self.assertFalse(stop["passed"])

    def test_costmap_check_requires_both_mark_and_clear_windows(self) -> None:
        evidence = copy.deepcopy(self.fixture)
        evidence["observer"]["costmaps"]["local"] = evidence["observer"][
            "costmaps"
        ]["local"][:1]
        evaluation = gate2.evaluate_evidence(self.scenario, evidence)
        local = next(
            item
            for item in evaluation["checks"]
            if item["name"] == "local_costmap_mark_and_clear"
        )
        self.assertFalse(local["passed"])

    def test_runtime_nodes_have_no_unitree_or_twist_publisher_surface(self) -> None:
        for relative in (
            "scripts/gate2_noop_cmd_vel_sink.py",
            "scripts/gate2_observer.py",
        ):
            with self.subTest(relative=relative):
                source = (ROOT / relative).read_text(encoding="utf-8")
                tree = ast.parse(source)
                lowered = source.lower()
                calls = {
                    node.func.attr
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                }
                imported = {
                    alias.name
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Import)
                    for alias in node.names
                } | {
                    str(node.module)
                    for node in ast.walk(tree)
                    if isinstance(node, ast.ImportFrom)
                }
                self.assertNotIn("create_publisher", calls)
                self.assertFalse(
                    any(name.lower().startswith("unitree") for name in imported)
                )
                self.assertNotIn("sportclient", lowered)
                self.assertNotIn("/lowcmd", source)
                self.assertNotIn("/api/sport/request", source)
        sink_source = (
            ROOT / "scripts" / "gate2_noop_cmd_vel_sink.py"
        ).read_text(encoding="utf-8")
        self.assertIn("create_subscription", sink_source)
        sink_tree = ast.parse(sink_source)
        self.assertFalse(
            any(
                isinstance(node, ast.Import)
                and any(alias.name == "socket" for alias in node.names)
                for node in ast.walk(sink_tree)
            )
        )

    def test_runner_does_not_start_unitree_processes(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        for forbidden in (
            "sport_mode_ctrl",
            "go2_sport_client",
            "go2_stand_example",
            "low_level_ctrl",
            "unitree_sdk2",
        ):
            self.assertNotIn(forbidden, source)
        self.assertIn('"ROS_LOCALHOST_ONLY": "1"', source)
        self.assertIn('"GO2_ALLOW_MOTION": "false"', source)


if __name__ == "__main__":
    unittest.main()
