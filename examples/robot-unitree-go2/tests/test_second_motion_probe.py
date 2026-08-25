from __future__ import annotations

import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROBE_PATH = ROOT / "scripts" / "second_motion_probe.py"
FIRST_PROBE_PATH = ROOT / "scripts" / "first_motion_probe.py"
OWNERSHIP_PATH = (
    ROOT / "scripts" / "check_second_motion_command_ownership.sh"
)
SPEC = importlib.util.spec_from_file_location("second_motion_probe", PROBE_PATH)
assert SPEC is not None and SPEC.loader is not None
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


class SecondMotionPolicyTest(unittest.TestCase):
    def environment(self) -> dict[str, str]:
        return {
            "GO2_ALLOW_MOTION": "true",
            "GO2_OPERATOR_PRESENT": "true",
            "GO2_SAFETY_ACK": "I_UNDERSTAND_GO2_CAN_MOVE",
            "GO2_SECOND_MOTION_ACK": (
                "I_APPROVE_GO2_SECOND_20CM_MOTION"
            ),
            "GO2_NETWORK_INTERFACE": "wlx500ff54809b8",
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "2010",
        }

    def test_environment_requires_the_independent_exact_gate(self) -> None:
        modes, markers = probe.validate_environment(self.environment())
        self.assertEqual(modes, frozenset({0}))
        self.assertEqual(markers, frozenset({2010}))
        for key in (
            "GO2_ALLOW_MOTION",
            "GO2_OPERATOR_PRESENT",
            "GO2_SAFETY_ACK",
            "GO2_SECOND_MOTION_ACK",
            "GO2_NETWORK_INTERFACE",
            "GO2_ALLOWED_MODES",
        ):
            changed = self.environment()
            changed[key] = ""
            with self.subTest(key=key), self.assertRaises(
                probe.CommissioningError
            ):
                probe.validate_environment(changed)
        changed = self.environment()
        changed.pop("GO2_SECOND_MOTION_ACK")
        changed["GO2_FIRST_MOTION_ACK"] = (
            "I_APPROVE_GO2_FIRST_10CM_MOTION"
        )
        with self.assertRaises(probe.CommissioningError):
            probe.validate_environment(changed)

    def test_soft_stop_and_hard_acceptance_are_distinct(self) -> None:
        moving = probe.motion_decision(
            10.0,
            10.0 + probe.PROBE_STOP_DURATION_S - 0.01,
            (0.0, 0.0),
            (probe.PROBE_STOP_DISTANCE_M - 0.001, 0.0),
        )
        self.assertTrue(moving.continue_motion)
        distance_stop = probe.motion_decision(
            10.0,
            11.0,
            (0.0, 0.0),
            (probe.PROBE_STOP_DISTANCE_M, 0.0),
        )
        self.assertFalse(distance_stop.continue_motion)
        self.assertEqual(distance_stop.reason, "distance_limit")
        duration_stop = probe.motion_decision(
            10.0,
            10.0 + probe.PROBE_STOP_DURATION_S + 1e-6,
            (0.0, 0.0),
            (0.01, 0.0),
        )
        self.assertFalse(duration_stop.continue_motion)
        self.assertEqual(duration_stop.reason, "duration_limit")
        self.assertLess(
            probe.PROBE_STOP_DISTANCE_M,
            probe.DISTANCE_LIMIT_M,
        )
        self.assertLess(
            probe.PROBE_STOP_DURATION_S,
            probe.DURATION_LIMIT_S,
        )

    def test_pass_requires_bounded_mostly_forward_motion(self) -> None:
        probe.require_measured_displacement(
            probe.MeasuredMotion(0.20, 0.19, 0.03, 0.05)
        )
        failures = (
            probe.MeasuredMotion(0.14, 0.14, 0.0, 0.0),
            probe.MeasuredMotion(0.30, 0.20, 0.0, 0.0),
            probe.MeasuredMotion(0.20, 0.18, 0.051, 0.0),
            probe.MeasuredMotion(0.20, 0.18, 0.0, 0.201),
            probe.MeasuredMotion(float("nan"), 0.18, 0.0, 0.0),
        )
        for measurement in failures:
            with self.subTest(measurement=measurement), self.assertRaises(
                probe.CommissioningError
            ):
                probe.require_measured_displacement(measurement)

    def test_second_limits_topics_and_post_stop_are_fixed(self) -> None:
        self.assertEqual(
            probe.COMMAND_TOPIC,
            "/go2/second_motion/cmd_vel",
        )
        self.assertEqual(
            probe.MOTION_PROFILE,
            "workstation-second-motion-corrected-v1",
        )
        self.assertEqual(probe.PROBE_NODE_NAME, "go2_second_motion_probe")
        self.assertEqual(probe.SPEED_MPS, 0.30)
        self.assertEqual(probe.PROBE_STOP_DISTANCE_M, 0.20)
        self.assertEqual(probe.PROBE_STOP_DURATION_S, 1.2)
        self.assertEqual(probe.DISTANCE_LIMIT_M, 0.30)
        self.assertEqual(probe.DURATION_LIMIT_S, 1.5)
        self.assertEqual(probe.MIN_MEASURED_DISPLACEMENT_M, 0.15)
        self.assertEqual(probe.MAX_MEASURED_LATERAL_M, 0.05)
        self.assertEqual(probe.MAX_MEASURED_YAW_CHANGE_RAD, 0.20)
        self.assertGreaterEqual(probe.POST_STOP_OBSERVATION_S, 1.0)
        self.assertGreaterEqual(probe.POST_STOP_MIN_ODOM_SAMPLES, 10)
        self.assertEqual(probe.POST_STOP_MAX_LINEAR_MPS, 0.03)
        self.assertEqual(probe.POST_STOP_MAX_YAW_RPS, 0.03)
        self.assertEqual(probe.POST_STOP_MAX_DRIFT_M, 0.02)
        self.assertEqual(probe.RUNTIME_GRAPH_CHECK_PERIOD_S, 0.50)
        source = PROBE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("check_second_motion_command_ownership.sh", source)
        self.assertNotIn("_SecondMotionSubprocess", source)
        self.assertIn("robonix-go2-second-motion-v1", source)
        self.assertIn("require_measured_displacement", source)
        self.assertNotIn("SportClient", source)
        self.assertNotIn("unitree_api", source)
        self.assertNotIn("/lowcmd", source)
        self.assertNotIn("ros2 topic pub", source)
        first_source = FIRST_PROBE_PATH.read_text(encoding="utf-8")
        self.assertIn("0..{SPEED_MPS:.2f} m/s", first_source)

    def test_sport_state_evidence_is_bounded_and_reports_raw_delta(
        self,
    ) -> None:
        start = probe.sport_state_evidence_snapshot(
            (1.0, -2.0, 0.3),
            (0.1, -0.2, 0.0),
            0.05,
            0,
            1,
        )
        end = probe.sport_state_evidence_snapshot(
            (1.18, -1.97, 0.3),
            (0.0, 0.0, 0.0),
            0.0,
            0,
            1,
        )
        evidence = probe.sport_state_pair_evidence(start, end, "")
        self.assertEqual(evidence["start"], start)
        self.assertEqual(evidence["end"], end)
        self.assertAlmostEqual(
            evidence["raw_position_delta_xy_m"][0],
            0.18,
        )
        self.assertAlmostEqual(
            evidence["raw_position_delta_xy_m"][1],
            0.03,
        )
        self.assertAlmostEqual(
            evidence["raw_position_delta_norm_m"],
            math.hypot(0.18, 0.03),
        )
        self.assertEqual(evidence["validation_issue"], "")

    def test_sport_state_evidence_rejects_nonfinite_or_unbounded_fields(
        self,
    ) -> None:
        invalid = (
            ((float("nan"), 0.0), (0.0, 0.0), 0.0, 0, 0),
            ((1_000_001.0, 0.0), (0.0, 0.0), 0.0, 0, 0),
            ((0.0, 0.0), (101.0, 0.0), 0.0, 0, 0),
            ((0.0, 0.0), (0.0, 0.0), float("inf"), 0, 0),
            ((0.0, 0.0), (0.0, 0.0), 0.0, 256, 0),
            ((0.0, 0.0), (0.0, 0.0), 0.0, 0, -1),
        )
        for fields in invalid:
            with self.subTest(fields=fields), self.assertRaises(
                probe.CommissioningError
            ):
                probe.sport_state_evidence_snapshot(*fields)

    def test_loading_second_profile_does_not_mutate_first_module(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "independent_first_motion_probe",
            FIRST_PROBE_PATH,
        )
        assert spec is not None and spec.loader is not None
        first = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = first
        spec.loader.exec_module(first)
        self.assertEqual(
            first.COMMAND_TOPIC,
            "/go2/commissioning/cmd_vel",
        )
        self.assertEqual(first.PROBE_STOP_DISTANCE_M, 0.09)
        self.assertEqual(first.PROBE_STOP_DURATION_S, 1.8)
        self.assertEqual(first.DISTANCE_LIMIT_M, 0.10)
        self.assertEqual(first.DURATION_LIMIT_S, 2.0)


class SecondMotionOwnershipScriptTest(unittest.TestCase):
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
            "  if [[ \"$count\" == 0 ]]; then\n"
            "    echo \"Unknown topic '/cmd_vel'\" >&2; exit 1\n"
            "  fi\n"
            "  printf 'Type: geometry_msgs/msg/Twist\\nPublisher count: %s\\nSubscription count: 0\\n' \"$count\"\n"
            "  exit 0\n"
            "fi\n"
            "[[ \"$topic\" == /go2/second_motion/cmd_vel ]] || exit 1\n"
            "pub_count=${PUB_COUNT:-1}\n"
            "sub_count=${SUB_COUNT:-1}\n"
            "pub_node=${PUB_NODE:-go2_second_motion_probe}\n"
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
                "GO2_SECOND_MOTION_OWNERSHIP_TIMEOUT_S": "3",
                "GO2_SECOND_MOTION_OWNERSHIP_STABILITY_SAMPLES": "1",
                "GO2_SECOND_MOTION_OWNERSHIP_QUERY_TIMEOUT_S": "1",
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

    def test_exact_endpoints_and_empty_canonical_topic_pass(self) -> None:
        result = self.run_checker()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("PASS", result.stdout)

    def test_duplicate_wrong_or_canonical_owner_fails(self) -> None:
        for values in (
            {"PUB_COUNT": "2"},
            {"SUB_COUNT": "2"},
            {"PUB_NODE": "go2_first_motion_probe"},
            {"SUB_NODE": "other_adapter"},
            {"CANONICAL_COUNT": "1"},
        ):
            result = self.run_checker(**values)
            with self.subTest(values=values):
                self.assertEqual(result.returncode, 5, result.stderr)

    def test_checker_is_strictly_read_only(self) -> None:
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
