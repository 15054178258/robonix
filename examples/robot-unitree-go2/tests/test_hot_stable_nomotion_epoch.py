#!/usr/bin/env python3
"""Offline contracts for thermal-stable no-motion epoch preparation."""

from __future__ import annotations

import argparse
import ast
import os
from pathlib import Path
from types import SimpleNamespace
import types
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
TIME_SYNC = ROOT / "deploy" / "time-sync"

import sys

sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(TIME_SYNC))

import prepare_hot_stable_nomotion_epoch as epoch  # noqa: E402
import verify_workstation_writer_identity_readonly as verifier  # noqa: E402
from workstation_nomotion_approval import ACK, EXPECTED_RAW_TOPICS  # noqa: E402
from workstation_nomotion_identity_monitor import (  # noqa: E402
    writer_identity_failures,
)


GIDS = {
    stream: bytes([index]) * 24
    for index, stream in enumerate(EXPECTED_RAW_TOPICS, 1)
}


def receipt(identity: Path, phase: str, gids: dict[str, str]) -> dict:
    return {
        "schema": verifier.SCHEMA,
        "phase": phase,
        "mode": "graph-only-read-only",
        "result": "pass",
        "identity_evidence_directory": str(identity),
        "identity_evidence": {"pcap": {"sha256": "a" * 64}},
        "identity_evidence_unchanged": True,
        "expected_writer_gids": gids,
        "graph": {"passed": True},
        "ros_subscriptions_created": False,
        "ros_publishers_created": False,
        "unitree_clients_created": False,
        "motion_ready": False,
    }


class HotStableNomotionEpochTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        (ROOT / "logs" / "go2-readonly").mkdir(parents=True, exist_ok=True)
        (ROOT / "rbnx-build" / "run").mkdir(parents=True, exist_ok=True)
        cls.epoch_source = (
            SCRIPTS / "prepare_hot_stable_nomotion_epoch.py"
        ).read_text(encoding="utf-8")
        cls.verifier_source = (
            SCRIPTS / "verify_workstation_writer_identity_readonly.py"
        ).read_text(encoding="utf-8")

    def _arguments(self, identity: Path, **changes: object) -> argparse.Namespace:
        values: dict[str, object] = {
            "identity_evidence_dir": identity,
            "thermal_stable_ack": epoch.THERMAL_STABLE_ACK,
            "operator_ack": ACK,
            "valid_for_seconds": 900,
            "session_id": "hot-stable-offline-session-01",
            "epoch_dir": None,
            "approval_output": None,
        }
        values.update(changes)
        return argparse.Namespace(**values)

    def test_identity_directory_is_required_and_has_no_implicit_default(self) -> None:
        action = next(
            item
            for item in epoch.build_argument_parser()._actions
            if item.dest == "identity_evidence_dir"
        )
        self.assertTrue(action.required)
        self.assertIsNone(action.default)
        self.assertNotIn("collect_go2_publisher_locators_readonly", self.epoch_source)

    def test_plan_brackets_exact_identity_around_fresh_75_second_capture(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary:
            identity = Path(temporary)
            identity.chmod(0o700)
            plan = epoch.build_plan(self._arguments(identity), now=1_700_000_000)
            commands = plan.commands()

        self.assertEqual(
            [stage for stage, _command in commands],
            [
                "identity-before",
                "time-capture-75s",
                "identity-after",
                "approval-generation",
            ],
        )
        flattened = [list(command) for _stage, command in commands]
        self.assertIn(str(identity), flattened[0])
        self.assertIn(str(identity), flattened[2])
        self.assertIn(str(identity), flattened[3])
        self.assertEqual(flattened[1][-1], "75")
        self.assertIn("900", flattened[3])
        self.assertNotIn(str(identity), flattened[1])

    def test_exact_two_acknowledgements_and_bounded_validity_are_required(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary:
            identity = Path(temporary)
            identity.chmod(0o700)
            with self.assertRaisesRegex(epoch.EpochError, "thermal-stable"):
                epoch.build_plan(
                    self._arguments(identity, thermal_stable_ack="YES")
                )
            with self.assertRaisesRegex(epoch.EpochError, "operator"):
                epoch.build_plan(self._arguments(identity, operator_ack="YES"))

        parser = epoch.build_argument_parser()
        required = [
            "--identity-evidence-dir",
            "/tmp/identity",
            "--thermal-stable-ack",
            epoch.THERMAL_STABLE_ACK,
            "--operator-ack",
            ACK,
        ]
        with self.assertRaises(SystemExit):
            parser.parse_args(required + ["--valid-for-seconds", "299"])
        with self.assertRaises(SystemExit):
            parser.parse_args(required + ["--valid-for-seconds", "3601"])
        self.assertEqual(
            parser.parse_args(required).valid_for_seconds,
            epoch.DEFAULT_VALIDITY_SECONDS,
        )

    def test_bracketing_receipts_must_have_same_writer_session(self) -> None:
        identity = ROOT / "logs" / "explicit-identity"
        gids = {stream: value.hex() for stream, value in GIDS.items()}
        before = receipt(identity, "before-time-capture", gids)
        after = receipt(identity, "after-time-capture", gids)
        self.assertEqual(
            epoch.validate_identity_receipt_pair(before, after, identity), gids
        )

        changed = dict(gids)
        changed["mid360_odom"] = (b"z" * 24).hex()
        with self.assertRaisesRegex(epoch.EpochError, "did not remain identical"):
            epoch.validate_identity_receipt_pair(
                before,
                receipt(identity, "after-time-capture", changed),
                identity,
            )
        failed = receipt(identity, "after-time-capture", gids)
        failed["result"] = "fail"
        with self.assertRaisesRegex(epoch.EpochError, "did not pass"):
            epoch.validate_identity_receipt_pair(before, failed, identity)

    def test_failed_precheck_stops_before_capture_and_approval(self) -> None:
        with tempfile.TemporaryDirectory(
            dir=ROOT / "logs" / "go2-readonly"
        ) as temporary:
            parent = Path(temporary)
            parent.chmod(0o700)
            identity = parent / "identity"
            identity.mkdir(mode=0o700)
            epoch_dir = parent / "epoch"
            approval = ROOT / "rbnx-build" / "run" / (
                f"offline-precheck-failure-{os.getpid()}.json"
            )
            approval.unlink(missing_ok=True)
            approval.with_name(
                approval.name + ".evidence-manifest.json"
            ).unlink(missing_ok=True)
            plan = epoch.build_plan(
                self._arguments(
                    identity,
                    session_id=f"offline-precheck-failure-{os.getpid()}",
                    epoch_dir=epoch_dir,
                    approval_output=approval,
                )
            )
            calls: list[list[str]] = []

            def fail_first(command, _environment, _timeout):
                calls.append(list(command))
                return 23

            with self.assertRaisesRegex(epoch.EpochError, "identity-before"):
                epoch.execute_plan(plan, runner=fail_first)
            self.assertEqual(len(calls), 1)
            self.assertIn("verify_workstation_writer_identity_readonly.sh", calls[0][1])
            self.assertFalse(approval.exists())

    def test_shared_identity_comparator_rejects_count_and_gid_changes(self) -> None:
        expected = {stream: value.hex() for stream, value in GIDS.items()}
        observations = {
            stream: [SimpleNamespace(endpoint_gid=list(value))]
            for stream, value in GIDS.items()
        }
        self.assertEqual(writer_identity_failures(expected, observations), [])
        observations["mid360_cloud"] = []
        self.assertEqual(
            writer_identity_failures(expected, observations),
            ["mid360_cloud:publisher_count:0"],
        )
        observations["mid360_cloud"] = [
            SimpleNamespace(endpoint_gid=list(b"x" * 24))
        ]
        self.assertEqual(
            writer_identity_failures(expected, observations),
            ["mid360_cloud:writer_gid_mismatch"],
        )

    def test_identity_path_must_be_private_real_and_inside_repository_logs(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary:
            identity = Path(temporary)
            identity.chmod(0o700)
            self.assertEqual(verifier._private_identity_directory(identity), identity)
            identity.chmod(0o755)
            with self.assertRaisesRegex(verifier.VerificationError, "accessible"):
                verifier._private_identity_directory(identity)
        with tempfile.TemporaryDirectory() as outside:
            path = Path(outside)
            path.chmod(0o700)
            with self.assertRaisesRegex(verifier.VerificationError, "repository logs"):
                verifier._private_identity_directory(path)

    def test_sources_keep_live_checker_graph_only_and_epoch_no_motion(self) -> None:
        for forbidden in (
            "create_subscription",
            "create_publisher",
            "create_client",
            "create_service",
            "create_action",
            "SportClient",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.verifier_source)
        tree = ast.parse(self.verifier_source)
        imported_modules = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module is not None:
                imported_modules.add(node.module)
        self.assertNotIn("rclpy.node", imported_modules)
        self.assertNotIn("rclpy.executors", imported_modules)
        low_level_nodes = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "_rclpy"
            and node.func.attr == "Node"
        ]
        self.assertEqual(len(low_level_nodes), 1)
        self.assertEqual(len(low_level_nodes[0].args), 6)
        self.assertIs(low_level_nodes[0].args[4].value, False)
        self.assertIs(low_level_nodes[0].args[5].value, False)
        graph_queries = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "_rclpy"
            and node.func.attr == "rclpy_get_publishers_info_by_topic"
        ]
        self.assertEqual(len(graph_queries), 1)
        self.assertEqual(len(graph_queries[0].args), 3)
        self.assertIs(graph_queries[0].args[2].value, False)
        context_init = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "context"
            and node.func.attr == "init"
        )
        initialize_logging = next(
            keyword
            for keyword in context_init.keywords
            if keyword.arg == "initialize_logging"
        )
        self.assertIs(initialize_logging.value.value, False)
        self.assertIn("node_handle.destroy_when_not_in_use()", self.verifier_source)
        self.assertIn("context.try_shutdown()", self.verifier_source)
        self.assertIn("TIME_CAPTURE_SECONDS = 75", self.epoch_source)
        self.assertIn('"GO2_ALLOW_MOTION": "false"', self.epoch_source)
        self.assertIn('"automatic_relock_enabled": False', self.epoch_source)
        self.assertIn('"future_allowance_changed": False', self.epoch_source)

    def test_live_checker_uses_only_low_level_graph_handle_and_cleans_up(self) -> None:
        expected = {stream: value.hex() for stream, value in GIDS.items()}
        by_topic = {
            topic: list(GIDS[stream])
            for stream, topic in EXPECTED_RAW_TOPICS.items()
        }
        calls: dict[str, object] = {
            "context_init": [],
            "context_try_shutdown": 0,
            "node_args": None,
            "node_destroy": 0,
            "queries": [],
        }

        class Handle:
            def __enter__(self):
                return self

            def __exit__(self, _exc_type, _exc, _traceback):
                return False

        class Context:
            def __init__(self):
                self.handle = Handle()
                self._ok = False

            def init(self, *, args, initialize_logging):
                calls["context_init"].append((args, initialize_logging))
                self._ok = True

            def ok(self):
                return self._ok

            def try_shutdown(self):
                calls["context_try_shutdown"] += 1
                self._ok = False

        class NodeHandle(Handle):
            def destroy_when_not_in_use(self):
                calls["node_destroy"] += 1

        class TopicEndpointInfo:
            def __init__(self, **values):
                self.endpoint_gid = values["endpoint_gid"]

        def node_factory(*arguments):
            calls["node_args"] = arguments
            return NodeHandle()

        def publishers_info(_node, topic, no_mangle):
            calls["queries"].append((topic, no_mangle))
            return [{"endpoint_gid": by_topic[topic]}]

        implementation = SimpleNamespace(
            Node=node_factory,
            rclpy_get_publishers_info_by_topic=publishers_info,
        )
        rclpy = types.ModuleType("rclpy")
        rclpy_context = types.ModuleType("rclpy.context")
        rclpy_context.Context = Context
        rclpy_impl = types.ModuleType("rclpy.impl")
        implementation_singleton = types.ModuleType(
            "rclpy.impl.implementation_singleton"
        )
        implementation_singleton.rclpy_implementation = implementation
        topic_endpoint_info = types.ModuleType("rclpy.topic_endpoint_info")
        topic_endpoint_info.TopicEndpointInfo = TopicEndpointInfo

        modules = {
            "rclpy": rclpy,
            "rclpy.context": rclpy_context,
            "rclpy.impl": rclpy_impl,
            "rclpy.impl.implementation_singleton": implementation_singleton,
            "rclpy.topic_endpoint_info": topic_endpoint_info,
        }
        with mock.patch.dict(sys.modules, modules):
            graph = verifier.observe_live_graph(
                expected,
                discovery_seconds=1,
                stable_samples=1,
            )

        self.assertTrue(graph["passed"])
        self.assertEqual(graph["graph_check_count"], 1)
        self.assertEqual(calls["context_init"], [([], False)])
        self.assertEqual(calls["context_try_shutdown"], 1)
        self.assertEqual(calls["node_destroy"], 1)
        node_args = calls["node_args"]
        self.assertIsNotNone(node_args)
        self.assertEqual(node_args[0], "go2_writer_identity_check_readonly")
        self.assertEqual(node_args[1], "")
        self.assertIsNone(node_args[3])
        self.assertIs(node_args[4], False)
        self.assertIs(node_args[5], False)
        self.assertEqual(
            calls["queries"],
            [(topic, False) for topic in EXPECTED_RAW_TOPICS.values()],
        )


if __name__ == "__main__":
    unittest.main()
