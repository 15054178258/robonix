#!/usr/bin/env python3
"""Offline contracts for the staged Nav2 subscription-only readiness audit."""

from __future__ import annotations

import ast
import copy
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

import staged_nav2_readiness as readiness  # noqa: E402


NOW_MONOTONIC_NS = 20_000_000_000
NOW_REALTIME_NS = 1_800_000_000_000_000_000
SESSION_ID = "staged-readiness-session-01"
INTERFACE = "enp108s0"
MAP_ID = "second_floor_corridor"
GENERATION = 7
ALLOWED_MODE = 3
ALLOWED_MARKER = 100


def _fresh(
    *,
    valid: bool = True,
    source: bool = True,
    **values: object,
) -> dict[str, object]:
    result: dict[str, object] = {
        "received_monotonic_ns": NOW_MONOTONIC_NS - 10_000_000,
        "valid": valid,
    }
    if source:
        result["source_stamp_ns"] = NOW_REALTIME_NS - 10_000_000
    result.update(values)
    return result


def _lifecycle_services(node: str) -> dict[str, list[str]]:
    return {
        f"{node}/get_state": ["lifecycle_msgs/srv/GetState"],
        f"{node}/change_state": ["lifecycle_msgs/srv/ChangeState"],
    }


def good_snapshot(phase: str) -> dict[str, object]:
    publishers = {
        readiness.GUARD_INPUT_TOPIC: [readiness.EXPECTED_CONTROLLER],
        readiness.STAGED_OUTPUT_TOPIC: (
            [] if phase == "pre_guard" else [readiness.EXPECTED_GUARD]
        ),
        readiness.CANONICAL_OUTPUT_TOPIC: [],
        readiness.BEHAVIOR_SINK_TOPIC: [],
        readiness.MAP_TOPIC: ["/map_server"],
        readiness.LIFECYCLE_TOPIC: ["/mapping_bridge"],
        readiness.GOAL_STATUS_TOPIC: ["/bt_navigator"],
        readiness.GOAL_FEEDBACK_TOPIC: ["/bt_navigator"],
    }
    subscribers = {
        readiness.GUARD_INPUT_TOPIC: (
            [] if phase == "pre_guard" else [readiness.EXPECTED_GUARD]
        ),
        readiness.STAGED_OUTPUT_TOPIC: [readiness.EXPECTED_CHASSIS],
        readiness.CANONICAL_OUTPUT_TOPIC: [],
        readiness.BEHAVIOR_SINK_TOPIC: [],
        readiness.MAP_TOPIC: [],
        readiness.LIFECYCLE_TOPIC: [],
        readiness.GOAL_STATUS_TOPIC: ["/readiness_observer"],
        readiness.GOAL_FEEDBACK_TOPIC: [],
    }
    return {
        "samples": {
            "map": _fresh(
                source=False,
                width=300,
                height=100,
                resolution=0.05,
            ),
            "map_lifecycle": _fresh(
                source=False,
                map_id=MAP_ID,
                mode="localization",
                generation=GENERATION,
            ),
            "localization": _fresh(
                frame_id="map",
                x_m=1.25,
                y_m=-0.50,
                yaw_rad=0.30,
            ),
            "odom": _fresh(frame_id="odom", child_frame_id="base_link"),
            "scan": _fresh(frame_id="laser_link", valid_range_count=300),
            "state": _fresh(
                mode=ALLOWED_MODE,
                error_code=ALLOWED_MARKER,
                gait_type=1,
            ),
            "tf_edges": {
                "map->odom": _fresh(),
                "odom->base_link": _fresh(),
            },
            "goal_status": _fresh(source=False, statuses=[]),
            "chassis": _fresh(
                source=False,
                state="DISARMED",
                daemon_armed="false",
                motion_configured="true",
                motion_profile=readiness.PROFILE,
                odom_source="external_verified",
            ),
        },
        "graph": {
            "publishers": publishers,
            "subscribers": subscribers,
            "topics": {
                readiness.GOAL_STATUS_TOPIC: [
                    "action_msgs/msg/GoalStatusArray"
                ],
                readiness.GOAL_FEEDBACK_TOPIC: [
                    "nav2_msgs/action/NavigateToPose_FeedbackMessage"
                ],
            },
            "services": {
                "/navigate_to_pose/_action/send_goal": [
                    "nav2_msgs/action/NavigateToPose_SendGoal"
                ],
                "/navigate_to_pose/_action/get_result": [
                    "nav2_msgs/action/NavigateToPose_GetResult"
                ],
                "/navigate_to_pose/_action/cancel_goal": [
                    "action_msgs/srv/CancelGoal"
                ],
            },
            "services_by_node": {
                node: _lifecycle_services(node)
                for node in readiness.REQUIRED_LIFECYCLE_NODES
            },
        },
    }


def evaluate(phase: str, snapshot: dict[str, object] | None = None) -> dict:
    return readiness.evaluate_snapshot(
        snapshot or good_snapshot(phase),
        phase=phase,
        session_id=SESSION_ID,
        network_interface=INTERFACE,
        expected_map_id=MAP_ID,
        expected_generation=GENERATION,
        allowed_mode=ALLOWED_MODE,
        allowed_state_marker=ALLOWED_MARKER,
        expected_start_x=1.25 if phase == "post_guard" else None,
        expected_start_y=-0.50 if phase == "post_guard" else None,
        now_monotonic_ns=NOW_MONOTONIC_NS,
        now_realtime_ns=NOW_REALTIME_NS,
    )


class StagedNav2ReadinessPolicyTest(unittest.TestCase):
    def test_pre_guard_exact_topology_passes(self) -> None:
        receipt = evaluate("pre_guard")
        self.assertTrue(receipt["passed"])
        self.assertEqual(receipt["result"], "pass")
        self.assertTrue(receipt["read_only"])
        self.assertEqual(receipt["session_id"], SESSION_ID)
        self.assertEqual(receipt["network_interface"], INTERFACE)
        self.assertEqual(
            receipt["map"],
            {"id": MAP_ID, "mode": "localization", "generation": GENERATION},
        )
        self.assertEqual(
            {item["id"] for item in receipt["checks"]},
            readiness.expected_check_ids("pre_guard"),
        )

    def test_post_guard_exact_topology_and_disarmed_status_pass(self) -> None:
        receipt = evaluate("post_guard")
        self.assertTrue(receipt["passed"])
        checks = {item["id"]: item for item in receipt["checks"]}
        self.assertTrue(checks["chassis_disarmed"]["passed"])
        self.assertEqual(
            checks["guard_input_subscriber_post_guard"]["observed"],
            (readiness.EXPECTED_GUARD,),
        )
        self.assertEqual(
            checks["staged_output_publisher_post_guard"]["observed"],
            (readiness.EXPECTED_GUARD,),
        )

    def test_every_velocity_edge_is_exact_not_set_based(self) -> None:
        mutations = (
            (
                "publishers",
                readiness.GUARD_INPUT_TOPIC,
                [readiness.EXPECTED_CONTROLLER, readiness.EXPECTED_CONTROLLER],
                "guard_input_publisher",
            ),
            (
                "subscribers",
                readiness.STAGED_OUTPUT_TOPIC,
                [readiness.EXPECTED_CHASSIS, "/unexpected"],
                "staged_output_subscriber",
            ),
            (
                "publishers",
                readiness.CANONICAL_OUTPUT_TOPIC,
                ["/controller_server"],
                "canonical_cmd_vel_publisher",
            ),
            (
                "subscribers",
                readiness.BEHAVIOR_SINK_TOPIC,
                ["/behavior_server"],
                "behavior_sink_subscriber",
            ),
        )
        for direction, topic, endpoints, check_id in mutations:
            with self.subTest(check_id=check_id):
                snapshot = good_snapshot("post_guard")
                snapshot["graph"][direction][topic] = endpoints
                receipt = evaluate("post_guard", snapshot)
                self.assertFalse(receipt["passed"])
                self.assertIn(check_id, receipt["failures"])

    def test_phase_specific_guard_ownership_cannot_be_swapped(self) -> None:
        before = good_snapshot("pre_guard")
        before["graph"]["publishers"][readiness.STAGED_OUTPUT_TOPIC] = [
            readiness.EXPECTED_GUARD
        ]
        self.assertIn(
            "staged_output_publisher_pre_guard",
            evaluate("pre_guard", before)["failures"],
        )

        after = good_snapshot("post_guard")
        after["graph"]["subscribers"][readiness.GUARD_INPUT_TOPIC] = []
        self.assertIn(
            "guard_input_subscriber_post_guard",
            evaluate("post_guard", after)["failures"],
        )

    def test_map_lifecycle_and_unique_publishers_are_required(self) -> None:
        snapshot = good_snapshot("pre_guard")
        snapshot["samples"]["map_lifecycle"]["generation"] += 1
        receipt = evaluate("pre_guard", snapshot)
        self.assertIn("map_lifecycle_exact", receipt["failures"])

        snapshot = good_snapshot("pre_guard")
        snapshot["graph"]["publishers"][readiness.MAP_TOPIC] = []
        receipt = evaluate("pre_guard", snapshot)
        self.assertIn("map_unique_publisher", receipt["failures"])

        snapshot = good_snapshot("pre_guard")
        snapshot["graph"]["publishers"][readiness.LIFECYCLE_TOPIC].append(
            "/duplicate"
        )
        receipt = evaluate("pre_guard", snapshot)
        self.assertIn(
            "map_lifecycle_unique_publisher", receipt["failures"]
        )

    def test_dynamic_tf_and_source_freshness_are_required(self) -> None:
        snapshot = good_snapshot("pre_guard")
        del snapshot["samples"]["tf_edges"]["map->odom"]
        self.assertIn(
            "tf_map_to_odom_valid_and_fresh",
            evaluate("pre_guard", snapshot)["failures"],
        )

        snapshot = good_snapshot("pre_guard")
        snapshot["samples"]["odom"]["source_stamp_ns"] = (
            NOW_REALTIME_NS - readiness.SOURCE_FRESHNESS_NS["odom"] - 1
        )
        self.assertIn(
            "odom_valid_and_fresh",
            evaluate("pre_guard", snapshot)["failures"],
        )

    def test_state_source_freshness_allows_snapshot_graph_inspection(self) -> None:
        limit_ns = readiness.SOURCE_FRESHNESS_NS["state"]
        self.assertEqual(limit_ns, 1_000_000_000)

        snapshot = good_snapshot("pre_guard")
        snapshot["samples"]["state"]["source_stamp_ns"] = (
            NOW_REALTIME_NS - limit_ns
        )
        self.assertTrue(evaluate("pre_guard", snapshot)["passed"])

        snapshot["samples"]["state"]["source_stamp_ns"] -= 1
        self.assertIn(
            "state_valid_and_fresh",
            evaluate("pre_guard", snapshot)["failures"],
        )

    def test_state_mode_and_marker_are_exact_per_permit(self) -> None:
        for field in ("mode", "error_code"):
            with self.subTest(field=field):
                snapshot = good_snapshot("pre_guard")
                snapshot["samples"]["state"][field] += 1
                self.assertIn(
                    "state_valid_and_fresh",
                    evaluate("pre_guard", snapshot)["failures"],
                )

    def test_chassis_contract_requires_all_disarmed_profile_fields(self) -> None:
        mutations = {
            "state": "ARMED",
            "daemon_armed": "true",
            "motion_configured": "false",
            "motion_profile": "workstation-first-motion-corrected-v1",
            "odom_source": "sport_state",
        }
        for field, value in mutations.items():
            with self.subTest(field=field):
                snapshot = good_snapshot("post_guard")
                snapshot["samples"]["chassis"][field] = value
                receipt = evaluate("post_guard", snapshot)
                self.assertIn("chassis_disarmed", receipt["failures"])

    def test_lifecycle_and_action_graphs_are_both_required(self) -> None:
        snapshot = good_snapshot("pre_guard")
        del snapshot["graph"]["services_by_node"]["/planner_server"][
            "/planner_server/get_state"
        ]
        self.assertIn(
            "lifecycle_graph_planner_server",
            evaluate("pre_guard", snapshot)["failures"],
        )

        snapshot = good_snapshot("pre_guard")
        del snapshot["graph"]["services"][
            "/navigate_to_pose/_action/send_goal"
        ]
        self.assertIn(
            "navigate_to_pose_action_graph",
            evaluate("pre_guard", snapshot)["failures"],
        )

    def test_active_goal_status_is_rejected(self) -> None:
        for status in (1, 2, 3):
            with self.subTest(status=status):
                snapshot = good_snapshot("post_guard")
                snapshot["samples"]["goal_status"]["statuses"] = [status]
                self.assertIn(
                    "nav2_goal_status_fresh_and_idle",
                    evaluate("post_guard", snapshot)["failures"],
                )


class StagedNav2ReceiptContractTest(unittest.TestCase):
    def _validate(self, receipt: object, **changes: object) -> dict:
        arguments: dict[str, object] = {
            "phase": "post_guard",
            "session_id": SESSION_ID,
            "network_interface": INTERFACE,
            "map_id": MAP_ID,
            "map_generation": GENERATION,
            "allowed_mode": ALLOWED_MODE,
            "allowed_state_marker": ALLOWED_MARKER,
            "expected_start_x": 1.25,
            "expected_start_y": -0.50,
            "now_realtime_ns": NOW_REALTIME_NS,
        }
        arguments.update(changes)
        return readiness.validate_readiness_receipt(receipt, **arguments)

    def test_passing_bundle_is_strictly_reusable(self) -> None:
        receipt = evaluate("post_guard")
        self.assertIs(self._validate(receipt), receipt)
        self.assertEqual(
            receipt["localization"],
            {
                "x_m": 1.25,
                "y_m": -0.50,
                "yaw_rad": 0.30,
                "source_stamp_ns": NOW_REALTIME_NS - 10_000_000,
                "received_monotonic_ns": NOW_MONOTONIC_NS - 10_000_000,
            },
        )

    def test_bundle_rejects_identity_timestamp_and_safety_mutations(self) -> None:
        mutations = (
            ("schema", "wrong"),
            ("read_only", False),
            ("session_id", "another-session-99"),
            ("network_interface", "wlan9"),
            ("map", {"id": MAP_ID, "mode": "mapping", "generation": GENERATION}),
        )
        for key, value in mutations:
            with self.subTest(key=key):
                receipt = evaluate("post_guard")
                receipt[key] = value
                with self.assertRaises(readiness.ReadinessError):
                    self._validate(receipt)

        receipt = evaluate("post_guard")
        receipt["timestamps"]["evaluated_realtime_ns"] -= 31_000_000_000
        with self.assertRaisesRegex(readiness.ReadinessError, "stale"):
            self._validate(receipt)

        receipt = evaluate("post_guard")
        receipt["safety"]["motion_commands_sent"] = True
        with self.assertRaisesRegex(readiness.ReadinessError, "safety"):
            self._validate(receipt)

    def test_bundle_rejects_missing_or_mutated_localization_witness(self) -> None:
        receipt = evaluate("post_guard")
        del receipt["localization"]["x_m"]
        with self.assertRaisesRegex(readiness.ReadinessError, "localization"):
            self._validate(receipt)

        receipt = evaluate("post_guard")
        receipt["localization"]["x_m"] = float("inf")
        with self.assertRaisesRegex(readiness.ReadinessError, "not finite"):
            self._validate(receipt)

        receipt = evaluate("post_guard")
        receipt["localization"]["source_stamp_ns"] -= 1_000_000_000
        with self.assertRaisesRegex(readiness.ReadinessError, "stale"):
            self._validate(receipt)

        receipt = evaluate("post_guard")
        receipt["localization"]["x_m"] += 0.051
        with self.assertRaisesRegex(readiness.ReadinessError, "path start"):
            self._validate(receipt)

    def test_post_guard_live_localization_must_match_path_start(self) -> None:
        snapshot = good_snapshot("post_guard")
        snapshot["samples"]["localization"]["x_m"] = 1.31
        receipt = evaluate("post_guard", snapshot)
        self.assertFalse(receipt["passed"])
        self.assertIn(
            "localization_matches_expected_start", receipt["failures"]
        )

    def test_bundle_requires_exact_unique_all_pass_check_set(self) -> None:
        receipt = evaluate("post_guard")
        receipt["checks"].pop()
        with self.assertRaisesRegex(readiness.ReadinessError, "check set"):
            self._validate(receipt)

        receipt = evaluate("post_guard")
        receipt["checks"].append(copy.deepcopy(receipt["checks"][0]))
        with self.assertRaisesRegex(readiness.ReadinessError, "duplicate"):
            self._validate(receipt)

        receipt = evaluate("post_guard")
        receipt["checks"][0]["passed"] = False
        with self.assertRaisesRegex(readiness.ReadinessError, "did not pass"):
            self._validate(receipt)

    def test_private_writer_is_0600_and_exclusive(self) -> None:
        (ROOT / "logs").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary:
            parent = Path(temporary)
            parent.chmod(0o700)
            output = parent / "readiness.json"
            self.assertEqual(readiness._output_path(output), output)
            readiness._write_exclusive_private(output, evaluate("pre_guard"))
            self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)
            parsed = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(parsed["schema"], readiness.SCHEMA)
            with self.assertRaises(FileExistsError):
                readiness._write_exclusive_private(output, {})

    def test_run_forwards_all_permit_bound_fields_to_pure_evaluator(self) -> None:
        arguments = SimpleNamespace(
            phase="pre_guard",
            session_id=SESSION_ID,
            network_interface=INTERFACE,
            map_id=MAP_ID,
            map_generation=GENERATION,
            allowed_mode=ALLOWED_MODE,
            allowed_state_marker=ALLOWED_MARKER,
            expected_start_x=None,
            expected_start_y=None,
            duration=5.0,
            output=Path("/unused/readiness.json"),
        )
        output = Path("/private/readiness.json")
        snapshot = good_snapshot("pre_guard")
        snapshot["collection"] = {"elapsed_monotonic_ns": 1}
        with (
            mock.patch.object(readiness, "_output_path", return_value=output),
            mock.patch.object(
                readiness, "collect_live_snapshot", return_value=snapshot
            ),
            mock.patch.object(readiness, "_write_exclusive_private") as writer,
            mock.patch.object(
                readiness.time,
                "monotonic_ns",
                return_value=NOW_MONOTONIC_NS,
            ),
            mock.patch.object(
                readiness.time, "time_ns", return_value=NOW_REALTIME_NS
            ),
        ):
            receipt, observed_output = readiness.run(arguments)
        self.assertTrue(receipt["passed"])
        self.assertEqual(observed_output, output)
        self.assertEqual(receipt["session_id"], SESSION_ID)
        self.assertEqual(receipt["allowed_state"]["mode"], ALLOWED_MODE)
        writer.assert_called_once_with(output, receipt)

    def test_cli_requires_bundle_identity_and_exact_state(self) -> None:
        parser = readiness.build_argument_parser()
        required = [
            "--phase",
            "pre_guard",
            "--session-id",
            SESSION_ID,
            "--network-interface",
            INTERFACE,
            "--map-id",
            MAP_ID,
            "--map-generation",
            str(GENERATION),
            "--allowed-mode",
            str(ALLOWED_MODE),
            "--allowed-state-marker",
            str(ALLOWED_MARKER),
            "--output",
            "/tmp/readiness.json",
        ]
        arguments = parser.parse_args(required)
        self.assertEqual(arguments.session_id, SESSION_ID)
        self.assertEqual(arguments.allowed_state_marker, ALLOWED_MARKER)
        with self.assertRaises(SystemExit):
            parser.parse_args(
                [item for item in required if item != "--allowed-state-marker"]
            )
        too_wide_mode = list(required)
        too_wide_mode[too_wide_mode.index(str(ALLOWED_MODE))] = "255"
        with self.assertRaises(SystemExit):
            parser.parse_args(too_wide_mode)


class StagedNav2MessageAndStaticSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.path = SCRIPTS / "staged_nav2_readiness.py"
        cls.source = cls.path.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    def test_disarmed_status_parser_keeps_required_profile_fields(self) -> None:
        message = SimpleNamespace(
            data=(
                "DISARMED: waiting; motion_configured=true; "
                f"motion_profile={readiness.PROFILE}; "
                "odom_source=external_verified; daemon_armed=false"
            )
        )
        parsed = readiness._chassis_sample(message, NOW_MONOTONIC_NS)
        self.assertEqual(parsed["state"], "DISARMED")
        self.assertEqual(parsed["daemon_armed"], "false")
        self.assertEqual(parsed["motion_configured"], "true")
        self.assertEqual(parsed["motion_profile"], readiness.PROFILE)
        self.assertEqual(parsed["odom_source"], "external_verified")

    def test_pose_parser_records_normalized_finite_xy_yaw(self) -> None:
        pose = SimpleNamespace(
            position=SimpleNamespace(x=2.0, y=-1.0, z=0.0),
            # Deliberately use a non-unit, but normalizable, 90-degree yaw.
            orientation=SimpleNamespace(
                x=0.0, y=0.0, z=0.8, w=0.8
            ),
        )
        message = SimpleNamespace(
            header=SimpleNamespace(
                frame_id="map",
                stamp=SimpleNamespace(
                    sec=NOW_REALTIME_NS // 1_000_000_000,
                    nanosec=NOW_REALTIME_NS % 1_000_000_000,
                ),
            ),
            pose=SimpleNamespace(pose=pose, covariance=[0.0] * 36),
        )
        parsed = readiness._pose_sample(message, NOW_MONOTONIC_NS)
        self.assertTrue(parsed["valid"])
        self.assertEqual(parsed["x_m"], 2.0)
        self.assertEqual(parsed["y_m"], -1.0)
        self.assertAlmostEqual(parsed["yaw_rad"], 3.141592653589793 / 2)

    def test_ros_imports_are_lazy_and_no_high_level_node_is_imported(self) -> None:
        top_level_imports: set[str] = set()
        for node in self.tree.body:
            if isinstance(node, ast.Import):
                top_level_imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                top_level_imports.add(node.module)
        self.assertFalse(
            any(name == "rclpy" or name.startswith("rclpy.") for name in top_level_imports)
        )
        self.assertNotIn("from rclpy.node import Node", self.source)
        self.assertIn("_rclpy.Subscription(", self.source)
        self.assertIn("_rclpy.WaitSet(", self.source)

    def test_source_cannot_construct_motion_or_control_endpoints(self) -> None:
        forbidden = (
            "create_publisher(",
            "create_client(",
            "create_service(",
            "ActionClient(",
            "ActionServer(",
            "SportClient(",
            "subprocess.",
            "/api/sport/request",
            "/lowcmd",
        )
        for token in forbidden:
            with self.subTest(token=token):
                self.assertNotIn(token, self.source)
        self.assertIn(
            "witness_refreshed_by_unique_publisher", self.source
        )


if __name__ == "__main__":
    unittest.main()
