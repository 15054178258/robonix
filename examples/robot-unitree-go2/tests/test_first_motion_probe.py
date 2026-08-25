from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROBE_PATH = ROOT / "scripts" / "first_motion_probe.py"
OWNERSHIP_PATH = ROOT / "scripts" / "check_motion_command_ownership.sh"
SPEC = importlib.util.spec_from_file_location("first_motion_probe", PROBE_PATH)
assert SPEC is not None and SPEC.loader is not None
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


class FirstMotionPolicyTest(unittest.TestCase):
    def environment(self) -> dict[str, str]:
        return {
            "GO2_ALLOW_MOTION": "true",
            "GO2_OPERATOR_PRESENT": "true",
            "GO2_SAFETY_ACK": "I_UNDERSTAND_GO2_CAN_MOVE",
            "GO2_FIRST_MOTION_ACK": "I_APPROVE_GO2_FIRST_10CM_MOTION",
            "GO2_NETWORK_INTERFACE": "enp108s0",
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "2010",
        }

    def test_environment_requires_every_exact_motion_gate(self) -> None:
        modes, markers = probe.validate_environment(self.environment())
        self.assertEqual(modes, frozenset({0}))
        self.assertEqual(markers, frozenset({2010}))
        for key in (
            "GO2_ALLOW_MOTION",
            "GO2_OPERATOR_PRESENT",
            "GO2_SAFETY_ACK",
            "GO2_FIRST_MOTION_ACK",
            "GO2_NETWORK_INTERFACE",
            "GO2_ALLOWED_MODES",
        ):
            with self.subTest(key=key):
                changed = self.environment()
                changed[key] = ""
                with self.assertRaises(probe.CommissioningError):
                    probe.validate_environment(changed)

    def test_empty_marker_list_is_valid_but_guessed_values_are_not(self) -> None:
        environment = self.environment()
        environment["GO2_ALLOWED_STATE_MARKERS"] = ""
        _modes, markers = probe.validate_environment(environment)
        self.assertEqual(markers, frozenset())
        environment["GO2_ALLOWED_STATE_MARKERS"] = "mode-2010"
        with self.assertRaises(probe.CommissioningError):
            probe.validate_environment(environment)

    def test_distance_or_duration_limit_stops_whichever_is_first(self) -> None:
        moving = probe.motion_decision(
            10.0,
            10.0 + probe.PROBE_STOP_DURATION_S - 0.01,
            (0.0, 0.0),
            (probe.PROBE_STOP_DISTANCE_M - 0.001, 0.0),
        )
        self.assertTrue(moving.continue_motion)
        distance_stop = probe.motion_decision(
            10.0, 11.0, (0.0, 0.0), (probe.PROBE_STOP_DISTANCE_M, 0.0)
        )
        self.assertFalse(distance_stop.continue_motion)
        self.assertEqual(distance_stop.reason, "distance_limit")
        duration_stop = probe.motion_decision(
            10.0,
            10.0 + probe.PROBE_STOP_DURATION_S,
            (0.0, 0.0),
            (0.01, 0.0),
        )
        self.assertFalse(duration_stop.continue_motion)
        self.assertEqual(duration_stop.reason, "duration_limit")

    def test_post_stop_requires_continuous_low_speed_yaw_and_drift(self) -> None:
        stationary = probe.post_stop_decision(
            (1.0, 2.0), (1.005, 2.0), 0.01, 0.01
        )
        self.assertTrue(stationary.stationary)
        cases = (
            ((1.0, 2.0), (1.0, 2.0), 0.031, 0.0, "post_stop_linear_speed"),
            ((1.0, 2.0), (1.0, 2.0), 0.0, 0.031, "post_stop_yaw_rate"),
            ((1.0, 2.0), (1.021, 2.0), 0.0, 0.0, "post_stop_drift"),
        )
        for start, current, speed, yaw, reason in cases:
            with self.subTest(reason=reason):
                decision = probe.post_stop_decision(
                    start, current, speed, yaw
                )
                self.assertFalse(decision.stationary)
                self.assertEqual(decision.reason, reason)
        with self.assertRaises(probe.CommissioningError):
            probe.post_stop_decision(
                (0.0, 0.0), (float("nan"), 0.0), 0.0, 0.0
            )

    def test_pass_requires_measured_nonzero_displacement(self) -> None:
        forward = probe.measured_motion(
            (1.0, 2.0), 0.0, (1.03, 2.0), 0.0
        )
        probe.require_measured_displacement(forward)
        rotated = probe.measured_motion(
            (1.0, 2.0), 1.5707963267948966, (1.0, 2.03), 1.57
        )
        probe.require_measured_displacement(rotated)
        with self.assertRaises(probe.CommissioningError):
            probe.require_measured_displacement(
                probe.measured_motion((0.0, 0.0), 0.0, (0.0, 0.0), 0.0)
            )
        with self.assertRaises(probe.CommissioningError):
            probe.require_measured_displacement(
                probe.measured_motion(
                    (0.0, 0.0),
                    0.0,
                    (probe.MIN_MEASURED_DISPLACEMENT_M - 0.001, 0.0),
                    0.0,
                )
            )
        with self.assertRaises(probe.CommissioningError):
            probe.require_measured_displacement(
                probe.MeasuredMotion(float("nan"), 0.03, 0.0, 0.0)
            )
        with self.assertRaises(probe.CommissioningError):
            probe.require_measured_displacement(
                probe.MeasuredMotion(0.05, 0.04, 0.031, 0.0)
            )
        with self.assertRaises(probe.CommissioningError):
            probe.require_measured_displacement(
                probe.MeasuredMotion(0.05, 0.04, 0.0, 0.201)
            )

    def test_quaternion_yaw_requires_normalized_finite_orientation(self) -> None:
        self.assertAlmostEqual(probe.quaternion_yaw(0.0, 0.0, 0.0, 1.0), 0.0)
        self.assertAlmostEqual(
            probe.quaternion_yaw(
                0.0, 0.0, 0.7071067811865476, 0.7071067811865476
            ),
            1.5707963267948966,
        )
        for quaternion in (
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 2.0),
            (0.0, 0.0, float("nan"), 1.0),
        ):
            with self.subTest(quaternion=quaternion):
                with self.assertRaises(probe.CommissioningError):
                    probe.quaternion_yaw(*quaternion)

    def test_missing_stale_or_future_receipt_fails_closed(self) -> None:
        with self.assertRaises(probe.CommissioningError):
            probe.assert_fresh(10.0, None, 0.2, "state")
        with self.assertRaises(probe.CommissioningError):
            probe.assert_fresh(10.0, 9.79, 0.2, "state")
        with self.assertRaises(probe.CommissioningError):
            probe.assert_fresh(10.0, 10.01, 0.2, "state")

    def test_diagnostic_level_accepts_ros_byte_and_integer_forms(self) -> None:
        self.assertEqual(probe.diagnostic_level(0), 0)
        self.assertEqual(probe.diagnostic_level(b"\x01"), 1)
        self.assertEqual(probe.diagnostic_level(bytearray(b"\x02")), 2)
        self.assertEqual(probe.diagnostic_level(memoryview(b"\x03")), 3)
        self.assertEqual(probe.diagnostic_level(255), 255)
        for malformed in (
            True,
            -1,
            256,
            b"",
            b"\x01\x02",
            "1",
            None,
        ):
            with self.subTest(malformed=malformed):
                with self.assertRaises(probe.CommissioningError):
                    probe.diagnostic_level(malformed)

    def test_diagnostic_evidence_is_bounded_and_actionable(self) -> None:
        snapshot = probe.diagnostic_evidence(
            "verified external odometry is invalid or stale",
            2,
            "chassis diagnostic level is ERROR",
            {
                "guard_state": "FAULT",
                "external_odom_status": (
                    "external odometry source timestamp is too old; "
                    "process restart required"
                ),
                "external_odom_age_sec": "0.012000",
                "source_stamp_status": "fresh",
                "commissioning_motion_active": "false",
                "unrelated_verbose_key": "must not be retained",
            },
        )
        self.assertEqual(snapshot["level"], 2)
        self.assertEqual(
            snapshot["message"],
            "verified external odometry is invalid or stale",
        )
        self.assertEqual(
            snapshot["validation_issue"],
            "chassis diagnostic level is ERROR",
        )
        values = snapshot["values"]
        self.assertIsInstance(values, dict)
        self.assertEqual(values["guard_state"], "FAULT")
        self.assertIn(
            "external odometry source timestamp",
            values["external_odom_status"],
        )
        self.assertNotIn("unrelated_verbose_key", values)

    def test_runtime_ownership_uses_the_existing_in_process_gate(self) -> None:
        source = PROBE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("subprocess.run(", source)
        self.assertNotIn("check_motion_command_ownership.sh", source)
        self.assertIn("stable >= GRAPH_STABILITY_SAMPLES", source)
        stable_failure = source.index("if stable < GRAPH_STABILITY_SAMPLES:")
        ownership_pass = source.index(
            'evidence["command_ownership"] = "PASS"'
        )
        self.assertLess(stable_failure, ownership_pass)
        self.assertIn(
            "now_s + RUNTIME_GRAPH_CHECK_PERIOD_S",
            source,
        )
        self.assertIn("issue = node.runtime_issue((\"ARMED\",))", source)

    def test_post_stop_runs_after_any_accepted_arm_and_successful_disarm(
        self,
    ) -> None:
        source = PROBE_PATH.read_text(encoding="utf-8")
        accepted = source.index("arm_accepted = True")
        cleanup_gate = source.index("if arm_accepted and disarm_success:")
        post_stop = source.index(") = node.verify_post_stop()", cleanup_gate)
        final_pass = source.index(
            "if exit_code == 0 and disarm_success and post_stop_success:"
        )
        self.assertLess(accepted, cleanup_gate)
        self.assertLess(cleanup_gate, post_stop)
        self.assertLess(post_stop, final_pass)
        self.assertNotIn(
            "if exit_code == 0 and disarm_success:\n"
            "                try:\n"
            "                    (\n"
            "                        post_stop_success,",
            source,
        )

    def test_limits_and_topics_are_fixed_commissioning_values(self) -> None:
        self.assertEqual(probe.COMMAND_TOPIC, "/go2/commissioning/cmd_vel")
        self.assertEqual(
            probe.STATE_TOPIC,
            "/robonix/time_corrected/motion/sportmodestate",
        )
        self.assertEqual(probe.SPEED_MPS, 0.05)
        self.assertEqual(probe.DISTANCE_LIMIT_M, 0.10)
        self.assertEqual(probe.DURATION_LIMIT_S, 2.0)
        self.assertEqual(probe.PROBE_STOP_DISTANCE_M, 0.09)
        self.assertEqual(probe.PROBE_STOP_DURATION_S, 1.8)
        self.assertEqual(probe.MIN_MEASURED_DISPLACEMENT_M, 0.02)
        self.assertEqual(probe.MAX_MEASURED_LATERAL_M, 0.03)
        self.assertEqual(probe.MAX_MEASURED_YAW_CHANGE_RAD, 0.20)
        self.assertLess(probe.PROBE_STOP_DISTANCE_M, probe.DISTANCE_LIMIT_M)
        self.assertLess(probe.PROBE_STOP_DURATION_S, probe.DURATION_LIMIT_S)
        self.assertEqual(probe.SPORT_REQUEST_TOPIC, "/api/sport/request")
        self.assertEqual(
            probe.SPORT_LEASE_REQUEST_TOPIC, "/api/sport_lease/request"
        )
        self.assertEqual(probe.COMMAND_STALE_S, 0.20)
        self.assertEqual(probe.DDS_GID_SIZE, 24)
        self.assertGreaterEqual(probe.ZERO_PREAMBLE_S, 0.5)
        self.assertGreaterEqual(probe.POST_STOP_OBSERVATION_S, 1.0)
        self.assertEqual(probe.POST_STOP_MAX_LINEAR_MPS, 0.03)
        self.assertEqual(probe.POST_STOP_MAX_YAW_RPS, 0.03)
        self.assertEqual(probe.POST_STOP_MAX_DRIFT_M, 0.02)
        self.assertEqual(probe.RUNTIME_GRAPH_CHECK_PERIOD_S, 0.50)
        source = PROBE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("outside 0..0.05 m/s", source)
        self.assertIn("outside 0..{SPEED_MPS:.2f} m/s", source)
        self.assertIn("f\"{DURATION_LIMIT_S:.1f} s\"", source)
        self.assertIn("f\"{DISTANCE_LIMIT_M:.2f} m\"", source)
        self.assertIn("finally:", source)
        self.assertIn("zero_and_disarm", source)
        self.assertIn("disarm service path raised", source)
        self.assertIn("verify_post_stop", source)
        self.assertIn("post_stop_success", source)
        self.assertIn("require_measured_displacement(measurement)", source)
        self.assertIn("commissioning_motion_active_observed", source)
        self.assertIn(
            "canonical /odom does not have the adapter as sole publisher",
            source,
        )
        self.assertIn(
            'values.get("odom_source") != "external_verified"', source
        )
        self.assertNotIn("SportClient", source)
        self.assertNotIn("unitree_api", source)
        self.assertEqual(source.count("self.create_publisher("), 1)
        self.assertIn("Twist, COMMAND_TOPIC, command_qos", source)
        self.assertIn("sport request writer DDS GID changed", source)
        self.assertIn("len(gid) != DDS_GID_SIZE", source)
        self.assertIn("a positive-lease request writer is present", source)
        self.assertIn('"last_chassis_diagnostic"', source)
        self.assertNotIn("/lowcmd", source)
        self.assertNotIn("ros2 topic pub", source)

    def test_preboot_sport_writer_baseline_binds_one_added_writer(self) -> None:
        baseline = frozenset((b"\x01" * 24, b"\x02" * 24))
        added = b"\x03" * 24
        issue, bound = probe.bind_added_sport_writer(
            baseline, tuple(baseline) + (added,), None
        )
        self.assertEqual(issue, "")
        self.assertEqual(bound, added)
        issue, rebound = probe.bind_added_sport_writer(
            baseline, tuple(baseline) + (added,), bound
        )
        self.assertEqual(issue, "")
        self.assertEqual(rebound, added)

        cases = (
            (tuple(baseline), "did not add exactly one"),
            ((b"\x01" * 24, added), "baseline changed"),
            (
                tuple(baseline) + (added, b"\x04" * 24),
                "did not add exactly one",
            ),
            (tuple(baseline) + (b"\x05",), "GID is invalid"),
        )
        for current, expected in cases:
            with self.subTest(expected=expected):
                issue, _bound = probe.bind_added_sport_writer(
                    baseline, current, bound
                )
                self.assertIn(expected, issue)

    def test_sport_writer_baseline_file_is_fresh_and_exact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "baseline.json"
            gids = ("01" * 24, "02" * 24)
            payload = {
                "schema": probe.SPORT_GRAPH_BASELINE_SCHEMA,
                "captured_unix_ns": time.time_ns(),
                "sport_request_topic": probe.SPORT_REQUEST_TOPIC,
                "sport_request_writer_count": len(gids),
                "sport_request_writer_gids": list(gids),
                "sport_lease_request_topic": probe.SPORT_LEASE_REQUEST_TOPIC,
                "sport_lease_request_writer_count": 0,
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            loaded = probe.load_sport_request_baseline(
                {"GO2_SPORT_REQUEST_BASELINE_FILE": str(path)}
            )
            self.assertEqual(loaded, frozenset(bytes.fromhex(gid) for gid in gids))

            payload["sport_lease_request_writer_count"] = 1
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(probe.CommissioningError):
                probe.load_sport_request_baseline(
                    {"GO2_SPORT_REQUEST_BASELINE_FILE": str(path)}
                )


class MotionOwnershipScriptTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.fake_bin = Path(self.temporary.name) / "bin"
        self.fake_bin.mkdir()
        fake_ros2 = self.fake_bin / "ros2"
        fake_ros2.write_text(
            "#!/usr/bin/env bash\n"
            "topic=\"${@: -1}\"\n"
            "if [[ \"$topic\" == /cmd_vel ]]; then\n"
            "  count=${CANONICAL_COUNT:-0}\n"
            "  if [[ \"$count\" == 0 ]]; then echo \"Unknown topic '/cmd_vel'\" >&2; exit 1; fi\n"
            "  printf 'Type: geometry_msgs/msg/Twist\\nPublisher count: %s\\nSubscription count: 0\\n' \"$count\"\n"
            "  exit 0\n"
            "fi\n"
            "[[ \"$topic\" == /go2/commissioning/cmd_vel ]] || { echo \"Unknown topic '$topic'\" >&2; exit 1; }\n"
            "pub_count=${PUB_COUNT:-1}\n"
            "sub_count=${SUB_COUNT:-1}\n"
            "pub_node=${PUB_NODE:-go2_first_motion_probe}\n"
            "sub_node=${SUB_NODE:-go2_chassis_adapter}\n"
            "printf 'Type: geometry_msgs/msg/Twist\\nPublisher count: %s\\n' \"$pub_count\"\n"
            "if [[ \"$pub_count\" != 0 ]]; then\n"
            "  printf 'Node name: %s\\nNode namespace: /\\nGID: 01.02.03\\nReliability: RELIABLE\\n' \"$pub_node\"\n"
            "fi\n"
            "printf 'Subscription count: %s\\n' \"$sub_count\"\n"
            "if [[ \"$sub_count\" != 0 ]]; then\n"
            "  printf 'Node name: %s\\nNode namespace: /\\nGID: 04.05.06\\nReliability: RELIABLE\\n' \"$sub_node\"\n"
            "fi\n",
            encoding="utf-8",
        )
        fake_ros2.chmod(0o755)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def run_checker(self, **overrides: str) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment.update(
            {
                "PATH": f"{self.fake_bin}:{environment['PATH']}",
                "GO2_MOTION_OWNERSHIP_TIMEOUT_S": "3",
                "GO2_MOTION_OWNERSHIP_STABILITY_SAMPLES": "1",
                "GO2_MOTION_OWNERSHIP_QUERY_TIMEOUT_S": "1",
                **overrides,
            }
        )
        return subprocess.run(
            [str(OWNERSHIP_PATH)],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
            timeout=10,
        )

    def test_exact_dedicated_endpoints_and_empty_canonical_topic_pass(self) -> None:
        result = self.run_checker()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("PASS", result.stdout)

    def test_duplicate_or_wrong_endpoint_fails(self) -> None:
        for values in (
            {"PUB_COUNT": "2"},
            {"SUB_COUNT": "2"},
            {"PUB_NODE": "velocity_guard"},
            {"SUB_NODE": "other_adapter"},
        ):
            with self.subTest(values=values):
                result = self.run_checker(**values)
                self.assertEqual(result.returncode, 5, result.stderr)

    def test_live_canonical_cmd_vel_publisher_fails(self) -> None:
        result = self.run_checker(CANONICAL_COUNT="1")
        self.assertEqual(result.returncode, 5, result.stderr)
        self.assertIn("Nav2 must be motion-isolated", result.stderr)

    def test_ownership_checker_is_strictly_read_only(self) -> None:
        source = OWNERSHIP_PATH.read_text(encoding="utf-8")
        for forbidden in (
            "ros2 topic pub",
            "ros2 service call",
            "ros2 action send_goal",
            "sudo",
            "nmcli con",
        ):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
