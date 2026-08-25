from __future__ import annotations

import ast
import importlib.util
import math
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts" / "observe_canonical_odom_stationary_readonly.sh"


def load_module():
    path = ROOT / "scripts" / "observe_canonical_odom_stationary_readonly.py"
    spec = importlib.util.spec_from_file_location(
        "canonical_odom_stationary_observer", path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    import sys

    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


observer = load_module()


def odometry(
    *,
    source_ns: int,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    yaw_degrees: float = 0.0,
    frame_id: str = "odom",
    child_frame_id: str = "base_link",
):
    yaw = math.radians(yaw_degrees)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            frame_id=frame_id,
            stamp=types.SimpleNamespace(
                sec=source_ns // 1_000_000_000,
                nanosec=source_ns % 1_000_000_000,
            ),
        ),
        child_frame_id=child_frame_id,
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=y, z=z),
                orientation=types.SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            ),
            covariance=[0.0] * 36,
        ),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
            covariance=[0.0] * 36,
        ),
    )


def observe_sample(tracker, message, receipt_ns: int, receipt_realtime_ns=None):
    source_ns = (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )
    if receipt_realtime_ns is None:
        receipt_realtime_ns = source_ns + 10_000_000
    return tracker.observe(message, receipt_ns, receipt_realtime_ns)


class CanonicalOdomStationaryMathTest(unittest.TestCase):
    def test_unwrap_drift_excursion_fit_translation_rate_and_gaps(self) -> None:
        tracker = observer.CanonicalOdomTracker(gap_threshold_seconds=0.5)
        base_receipt_ns = 20_000_000_000
        base_source_ns = 10_000_000_000
        for index, (yaw, x, y) in enumerate(
            ((179.0, 0.0, 0.0), (-179.0, 0.003, 0.0), (-177.0, 0.004, 0.003))
        ):
            observe_sample(
                tracker,
                odometry(
                    source_ns=base_source_ns + index * 1_000_000_000,
                    yaw_degrees=yaw,
                    x=x,
                    y=y,
                ),
                base_receipt_ns + index * 1_000_000_000,
            )

        summary = tracker.summary(base_receipt_ns + 2_100_000_000)
        self.assertTrue(summary["analyzed"])
        self.assertEqual(summary["received_messages"], 3)
        self.assertEqual(summary["valid_messages"], 3)
        self.assertEqual(summary["invalid_messages"], 0)
        self.assertAlmostEqual(summary["receipt_sample_rate_hz"], 1.0)
        self.assertAlmostEqual(summary["valid_source_sample_rate_hz"], 1.0)
        self.assertEqual(summary["receipt_gaps"]["observed_intervals"], 2)
        self.assertAlmostEqual(summary["receipt_gaps"]["maximum_seconds"], 1.0)
        self.assertEqual(summary["receipt_gaps"]["intervals_above_threshold"], 2)
        self.assertAlmostEqual(summary["translation_end_to_end_m"], 0.005)
        self.assertAlmostEqual(summary["planar_translation_end_to_end_m"], 0.005)
        self.assertAlmostEqual(summary["yaw_unwrap_end_to_end_deg"], 4.0)
        self.assertAlmostEqual(summary["yaw_unwrap_excursion_deg"], 4.0)
        self.assertAlmostEqual(summary["yaw_slope_deg_per_minute"], 120.0)
        self.assertEqual(summary["yaw_slope_time_basis"], "source_stamp")
        self.assertAlmostEqual(summary["reported_planar_speed_max_mps"], 0.0)
        self.assertAlmostEqual(
            summary["reported_abs_yaw_rate_max_rad_per_second"], 0.0
        )
        self.assertAlmostEqual(summary["last_receipt_age_seconds"], 0.1)

    def test_frame_stamp_finite_and_quaternion_fail_closed(self) -> None:
        tracker = observer.CanonicalOdomTracker(gap_threshold_seconds=0.2)
        observe_sample(
            tracker,
            odometry(source_ns=1_000_000_000),
            1_000_000_000,
        )
        observe_sample(
            tracker,
            odometry(source_ns=2_000_000_000, frame_id="map"),
            2_000_000_000,
        )
        observe_sample(
            tracker,
            odometry(source_ns=2_000_000_000, child_frame_id="base"),
            3_000_000_000,
        )
        observe_sample(
            tracker,
            odometry(source_ns=3_000_000_000, x=float("nan")),
            4_000_000_000,
        )
        invalid_quaternion = odometry(source_ns=4_000_000_000)
        invalid_quaternion.pose.pose.orientation.w = 0.0
        observe_sample(tracker, invalid_quaternion, 5_000_000_000)

        invalid_twist = odometry(source_ns=5_000_000_000)
        invalid_twist.twist.twist.angular.z = float("inf")
        observe_sample(tracker, invalid_twist, 6_000_000_000)

        summary = tracker.summary()
        self.assertEqual(summary["received_messages"], 6)
        self.assertEqual(summary["valid_messages"], 1)
        self.assertEqual(summary["invalid_messages"], 5)
        self.assertEqual(summary["invalid_reasons"]["frame_id_mismatch"], 1)
        self.assertEqual(summary["invalid_reasons"]["child_frame_id_mismatch"], 1)
        self.assertEqual(
            summary["invalid_reasons"]["non_monotonic_source_stamp"], 1
        )
        self.assertEqual(summary["invalid_reasons"]["non_finite_pose"], 1)
        self.assertEqual(summary["invalid_reasons"]["invalid_quaternion_norm"], 1)
        self.assertEqual(summary["invalid_reasons"]["non_finite_twist"], 1)
        self.assertFalse(summary["analyzed"])

    def test_zero_stamp_and_regressing_receipt_are_rejected(self) -> None:
        tracker = observer.CanonicalOdomTracker(gap_threshold_seconds=0.2)
        observe_sample(tracker, odometry(source_ns=0), 10)
        observe_sample(tracker, odometry(source_ns=1_000_000_000), 9)
        summary = tracker.summary()
        self.assertEqual(summary["invalid_messages"], 2)
        self.assertEqual(summary["invalid_reasons"]["invalid_stamp"], 1)
        self.assertEqual(
            summary["invalid_reasons"]["non_monotonic_receipt_time"], 1
        )

    def test_sample_and_gap_retention_is_bounded_and_fails_closed(self) -> None:
        tracker = observer.CanonicalOdomTracker(gap_threshold_seconds=0.2)
        with mock.patch.object(observer, "MAX_RETAINED_SAMPLES", 2):
            for index in range(3):
                observe_sample(
                    tracker,
                    odometry(source_ns=(index + 1) * 1_000_000_000),
                    (index + 1) * 1_000_000_000,
                )
        summary = tracker.summary()
        self.assertEqual(summary["retained_samples"], 2)
        self.assertEqual(summary["receipt_gaps"]["observed_intervals"], 1)
        self.assertEqual(summary["source_stamp_gaps"]["observed_intervals"], 1)
        self.assertEqual(summary["invalid_reasons"]["sample_retention_limit"], 1)

    def test_source_age_future_and_covariance_are_rejected(self) -> None:
        tracker = observer.CanonicalOdomTracker(gap_threshold_seconds=0.2)
        stale = odometry(source_ns=1_000_000_000)
        observe_sample(tracker, stale, 1_000, 1_210_000_001)
        future = odometry(source_ns=2_000_000_000)
        observe_sample(tracker, future, 2_000, 1_949_999_999)
        bad_covariance = odometry(source_ns=3_000_000_000)
        bad_covariance.pose.covariance[5] = float("nan")
        observe_sample(tracker, bad_covariance, 3_000, 3_010_000_000)
        summary = tracker.summary()
        self.assertEqual(summary["invalid_reasons"]["stale_source_stamp"], 1)
        self.assertEqual(summary["invalid_reasons"]["future_source_stamp"], 1)
        self.assertEqual(summary["invalid_reasons"]["non_finite_covariance"], 1)
        self.assertEqual(summary["valid_messages"], 0)

    def test_invalid_payload_does_not_hide_valid_sample_gap(self) -> None:
        tracker = observer.CanonicalOdomTracker(gap_threshold_seconds=10.0)
        observe_sample(
            tracker,
            odometry(source_ns=1_000_000_000),
            1_000_000_000,
        )
        observe_sample(
            tracker,
            odometry(source_ns=2_000_000_000, frame_id="map"),
            2_000_000_000,
        )
        observe_sample(
            tracker,
            odometry(source_ns=3_000_000_000),
            3_000_000_000,
        )
        summary = tracker.summary()
        self.assertEqual(summary["valid_messages"], 2)
        self.assertEqual(summary["source_stamp_gaps"]["observed_intervals"], 1)
        self.assertAlmostEqual(summary["source_stamp_gaps"]["maximum_seconds"], 2.0)
        self.assertAlmostEqual(summary["receipt_gaps"]["maximum_seconds"], 2.0)


class CanonicalOdomStationaryContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        (ROOT / "logs").mkdir(parents=True, exist_ok=True)

    def arguments(self, duration_seconds: int = 1):
        return observer.build_argument_parser().parse_args(
            [
                "--output-dir",
                str(ROOT / "logs" / "example"),
                "--duration-seconds",
                str(duration_seconds),
            ]
        )

    def test_cli_requires_explicit_bounded_duration_and_output(self) -> None:
        parser = observer.build_argument_parser()
        arguments = parser.parse_args(
            [
                "--output-dir",
                str(ROOT / "logs" / "example"),
                "--duration-seconds",
                "600",
            ]
        )
        self.assertEqual(arguments.duration_seconds, 600)
        self.assertEqual(arguments.gap_threshold_seconds, 0.2)
        self.assertEqual(arguments.max_source_age_seconds, 0.2)
        self.assertEqual(arguments.max_future_skew_seconds, 0.05)
        with self.assertRaises(SystemExit):
            parser.parse_args(["--output-dir", str(ROOT / "logs" / "example")])
        with self.assertRaises(SystemExit):
            parser.parse_args(
                [
                    "--output-dir",
                    str(ROOT / "logs" / "example"),
                    "--duration-seconds",
                    "0",
                ]
            )
        with self.assertRaises(SystemExit):
            parser.parse_args(
                [
                    "--output-dir",
                    str(ROOT / "logs" / "example"),
                    "--duration-seconds",
                    "3601",
                ]
            )
        for duration, gap in ((1, "0.5"), (2, "1.0"), (1, "10.0")):
            with self.subTest(duration=duration, gap=gap), self.assertRaises(
                SystemExit
            ):
                parser.parse_args(
                    [
                        "--output-dir",
                        str(ROOT / "logs" / "example"),
                        "--duration-seconds",
                        str(duration),
                        "--gap-threshold-seconds",
                        gap,
                    ]
                )

    def test_run_defensively_rejects_invalid_window_before_output(self) -> None:
        arguments = self.arguments(duration_seconds=1)
        arguments.gap_threshold_seconds = 0.5
        with mock.patch.object(observer, "_prepare_output") as prepare_output:
            with self.assertRaisesRegex(ValueError, "strictly less"):
                observer.run_observer(arguments)
        prepare_output.assert_not_called()

    def test_output_is_confined_to_logs_and_never_overwritten(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "must be below"):
            observer._prepare_output(Path("/tmp/canonical-odom-evidence"))
        with self.assertRaisesRegex(RuntimeError, "new child"):
            observer._prepare_output(ROOT / "logs")
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            evidence = observer._prepare_output(Path(directory) / "evidence")
            observer._write_json(evidence / "summary.json", {"ok": True})
            self.assertEqual(
                (evidence / "summary.json").stat().st_mode & 0o777,
                0o600,
            )
            with self.assertRaisesRegex(RuntimeError, "refusing to reuse"):
                observer._prepare_output(evidence)

    def test_result_rejects_short_coverage_and_accepts_complete_fresh_window(
        self,
    ) -> None:
        arguments = self.arguments()
        complete = observer.CanonicalOdomTracker(gap_threshold_seconds=0.2)
        base_receipt_ns = 1_000_000_000
        base_source_ns = 10_000_000_000
        for index in range(6):
            source_ns = base_source_ns + index * 200_000_000
            observe_sample(
                complete,
                odometry(source_ns=source_ns),
                base_receipt_ns + index * 200_000_000,
                source_ns + 10_000_000,
            )
        complete_result = observer._build_result(
            arguments=arguments,
            tracker=complete,
            ros_log_dir=ROOT / "logs" / "example" / "ros-logs",
            started_realtime_ns=1,
            started_monotonic_ns=base_receipt_ns,
            finished_realtime_ns=2,
            finished_monotonic_ns=base_receipt_ns + 1_050_000_000,
            exit_reason="duration_elapsed",
            runtime_error=None,
            cleanup_errors=[],
            subscriptions_created=1,
            observation_started_monotonic_ns=base_receipt_ns,
        )
        self.assertTrue(complete_result["observer_valid"])
        self.assertTrue(complete_result["stationary_observed"])
        self.assertEqual(complete_result["ros_subscriptions_created"], 1)
        self.assertFalse(complete_result["ros_publishers_created"])
        self.assertFalse(complete_result["services_or_action_clients_created"])

        incomplete = observer.CanonicalOdomTracker(gap_threshold_seconds=0.2)
        for index in range(2):
            source_ns = base_source_ns + index * 100_000_000
            observe_sample(
                incomplete,
                odometry(source_ns=source_ns),
                base_receipt_ns + index * 100_000_000,
                source_ns + 10_000_000,
            )
        incomplete_result = observer._build_result(
            arguments=arguments,
            tracker=incomplete,
            ros_log_dir=ROOT / "logs" / "example" / "ros-logs",
            started_realtime_ns=1,
            started_monotonic_ns=base_receipt_ns,
            finished_realtime_ns=2,
            finished_monotonic_ns=base_receipt_ns + 1_000_000_000,
            exit_reason="duration_elapsed",
            runtime_error=None,
            cleanup_errors=[],
            subscriptions_created=1,
            observation_started_monotonic_ns=base_receipt_ns,
        )
        self.assertFalse(incomplete_result["observer_valid"])
        self.assertFalse(
            incomplete_result["checks"]["requested_observation_window_covered"]
        )
        self.assertFalse(
            incomplete_result["checks"]["last_receipt_fresh_at_finish"]
        )

    def test_source_ast_has_one_subscription_and_no_command_surface(self) -> None:
        source_path = (
            ROOT / "scripts" / "observe_canonical_odom_stationary_readonly.py"
        )
        source = source_path.read_text(encoding="utf-8")
        tree = ast.parse(source)
        attribute_calls = [
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        ]
        self.assertEqual(attribute_calls.count("create_subscription"), 0)
        for forbidden in (
            "create_publisher",
            "create_client",
            "create_service",
            "create_timer",
            "publish",
            "call",
            "call_async",
            "send_goal",
            "send_goal_async",
        ):
            self.assertNotIn(forbidden, attribute_calls)
        imported_modules = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module is not None:
                imported_modules.add(node.module)
        self.assertNotIn("subprocess", imported_modules)
        self.assertFalse(
            any("unitree" in module_name.lower() for module_name in imported_modules)
        )
        self.assertNotIn("rclpy.node", imported_modules)
        self.assertNotIn("/cmd_vel", source)
        self.assertNotIn("/api/sport/request", source)
        self.assertNotIn("/lowcmd", source)
        self.assertIn(
            "while context.ok() and time.perf_counter_ns() < deadline_ns:",
            source,
        )
        low_level_subscriptions = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "_rclpy"
            and node.func.attr == "Subscription"
        ]
        self.assertEqual(len(low_level_subscriptions), 1)
        self.assertIsInstance(low_level_subscriptions[0].args[2], ast.Name)
        self.assertEqual(low_level_subscriptions[0].args[2].id, "TOPIC")
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
        context_init = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "init"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "context"
        )
        initialize_logging = next(
            keyword
            for keyword in context_init.keywords
            if keyword.arg == "initialize_logging"
        )
        self.assertIs(initialize_logging.value.value, False)

    def test_wrapper_bootstraps_ros_and_overlay_under_outer_timeout(self) -> None:
        source = WRAPPER.read_text(encoding="utf-8")
        self.assertIn("timeout --signal=TERM --kill-after=5s", source)
        self.assertIn('source_setup_or_exit "ROS 2" "${ros_setup}"', source)
        self.assertIn(
            'source_setup_or_exit "Unitree ROS 2 overlay" "${unitree_setup}"',
            source,
        )
        self.assertLess(
            source.index('source_setup_or_exit "ROS 2"'),
            source.index('source_setup_or_exit "Unitree ROS 2 overlay"'),
        )
        self.assertLess(
            source.index('source_setup_or_exit "Unitree ROS 2 overlay"'),
            source.index("timeout --signal=TERM"),
        )
        for forbidden in (
            "ros2 ",
            "/cmd_vel",
            "/api/sport/request",
            "/lowcmd",
            "SportClient",
        ):
            self.assertNotIn(forbidden, source)

    def test_wrapper_sources_mock_setups_and_passes_strict_arguments(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary_name:
            temporary = Path(temporary_name)
            fake_bin = temporary / "bin"
            fake_bin.mkdir()
            ros_setup = temporary / "ros-setup.bash"
            overlay_setup = temporary / "overlay-setup.bash"
            marker = temporary / "timeout-arguments"
            ros_setup.write_text("export ROS_SETUP_READY=1\n", encoding="utf-8")
            overlay_setup.write_text(
                ': "${ROS_SETUP_READY:?ROS setup must be sourced first}"\n'
                "export OVERLAY_SETUP_READY=1\n",
                encoding="utf-8",
            )
            timeout_stub = fake_bin / "timeout"
            timeout_stub.write_text(
                "#!/usr/bin/env bash\n"
                "set -euo pipefail\n"
                ': "${ROS_SETUP_READY:?}"\n'
                ': "${OVERLAY_SETUP_READY:?}"\n'
                'printf "%s\\n" "$@" > "${TIMEOUT_ARGUMENTS_MARKER}"\n',
                encoding="utf-8",
            )
            timeout_stub.chmod(timeout_stub.stat().st_mode | stat.S_IXUSR)
            environment = os.environ.copy()
            environment.update(
                {
                    "PATH": f"{fake_bin}:{environment['PATH']}",
                    "ROS_SETUP_FILE": str(ros_setup),
                    "UNITREE_ROS2_SETUP": str(overlay_setup),
                    "TIMEOUT_ARGUMENTS_MARKER": str(marker),
                }
            )
            output = temporary / "evidence"
            result = subprocess.run(
                ["bash", str(WRAPPER), str(output), "1", "0.2"],
                cwd=ROOT,
                env=environment,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10,
                check=False,
            )
            self.assertEqual(result.returncode, 0, (result.stdout, result.stderr))
            arguments = marker.read_text(encoding="utf-8").splitlines()
            self.assertEqual(arguments[:3], ["--signal=TERM", "--kill-after=5s", "16s"])
            self.assertIn(str(ROOT / "scripts" / "observe_canonical_odom_stationary_readonly.py"), arguments)
            self.assertEqual(arguments[-6:], ["--output-dir", str(output), "--duration-seconds", "1", "--gap-threshold-seconds", "0.2"])

    def test_wrapper_rejects_invalid_window_before_sourcing_ros(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary_name:
            output = Path(temporary_name) / "evidence"
            environment = os.environ.copy()
            environment.update(
                {
                    "ROS_SETUP_FILE": str(Path(temporary_name) / "missing-ros"),
                    "UNITREE_ROS2_SETUP": str(
                        Path(temporary_name) / "missing-overlay"
                    ),
                }
            )
            result = subprocess.run(
                ["bash", str(WRAPPER), str(output), "1", "0.5"],
                cwd=ROOT,
                env=environment,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10,
                check=False,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("2 * 间隙必须严格小于持续秒数", result.stderr)
            self.assertNotIn("缺少 ROS 2 环境脚本", result.stderr)


if __name__ == "__main__":
    unittest.main()
