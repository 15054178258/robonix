from __future__ import annotations

import ast
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC_DIR = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC_DIR))

from receipt_gap_core import DEFAULT_THRESHOLDS_NS, ReceiptGapTracker


def load_observer():
    path = ROOT / "scripts" / "observe_odom_receipt_gaps_readonly.py"
    spec = importlib.util.spec_from_file_location("odom_gap_observer", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


observer = load_observer()


class ReceiptGapTrackerTest(unittest.TestCase):
    def test_exact_boundaries_are_not_greater_than_events(self) -> None:
        tracker = ReceiptGapTracker(retained_gap_limit=100)
        self.assertIsNone(tracker.observe(1_000_000_000))
        at_100 = tracker.observe(1_100_000_000)
        over_100 = tracker.observe(1_200_000_001)
        at_150 = tracker.observe(1_350_000_001)
        over_200 = tracker.observe(1_550_000_002)

        assert at_100 is not None
        assert over_100 is not None
        assert at_150 is not None
        assert over_200 is not None
        self.assertEqual(at_100.exceeded_thresholds_ns, ())
        self.assertEqual(over_100.exceeded_thresholds_ns, (100_000_000,))
        self.assertEqual(at_150.exceeded_thresholds_ns, (100_000_000,))
        self.assertEqual(over_200.exceeded_thresholds_ns, DEFAULT_THRESHOLDS_NS)
        self.assertEqual(
            tracker.summary()["threshold_exceedance_counts"],
            [
                {"threshold_ns": 100_000_000, "count": 3},
                {"threshold_ns": 150_000_000, "count": 1},
                {"threshold_ns": 200_000_000, "count": 1},
            ],
        )

    def test_statistics_are_exact_while_percentile_window_is_bounded(self) -> None:
        tracker = ReceiptGapTracker(retained_gap_limit=100)
        now = 10_000_000_000
        tracker.observe(now)
        for gap in range(1, 1_001):
            now += gap
            tracker.observe(now)
        summary = tracker.summary()
        self.assertEqual(summary["received_messages"], 1_001)
        self.assertEqual(summary["observed_intervals"], 1_000)
        self.assertEqual(summary["minimum_gap_ns"], 1)
        self.assertEqual(summary["maximum_gap_ns"], 1_000)
        self.assertEqual(summary["mean_gap_ns"], 500.5)
        self.assertEqual(summary["retained_gap_count"], 100)
        self.assertEqual(len(tracker._retained_gaps_ns), 100)

    def test_invalid_or_regressing_monotonic_receipt_is_rejected(self) -> None:
        tracker = ReceiptGapTracker(retained_gap_limit=100)
        with self.assertRaisesRegex(ValueError, "positive integer"):
            tracker.observe(0)
        tracker.observe(100)
        with self.assertRaisesRegex(ValueError, "regressed"):
            tracker.observe(99)


class ReceiptGapObserverContractTest(unittest.TestCase):
    def test_cli_is_bounded_and_defaults_to_raw_odom(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            arguments = observer.build_argument_parser().parse_args(
                ["--output-dir", directory]
            )
        self.assertEqual(arguments.topic, "/utlidar/robot_odom")
        self.assertEqual(arguments.duration_seconds, 900)
        self.assertEqual(arguments.max_events, 100_000)
        self.assertFalse(arguments.include_sport_primary)
        self.assertFalse(arguments.include_cloud)
        self.assertEqual(arguments.cloud_qos_reliability, "best_effort")
        enabled = observer.build_argument_parser().parse_args(
            [
                "--output-dir",
                directory,
                "--include-sport-primary",
                "--include-cloud",
                "--cloud-qos-reliability",
                "reliable",
            ]
        )
        self.assertTrue(enabled.include_sport_primary)
        self.assertTrue(enabled.include_cloud)
        self.assertEqual(enabled.cloud_qos_reliability, "reliable")
        with self.assertRaises(SystemExit):
            observer.build_argument_parser().parse_args(
                ["--output-dir", "/tmp/evidence", "--duration-seconds", "0"]
            )
        with self.assertRaises(SystemExit):
            observer.build_argument_parser().parse_args(
                [
                    "--output-dir",
                    "/tmp/evidence",
                    "--cloud-qos-reliability",
                    "sometimes",
                ]
            )

    def test_output_is_confined_to_repository_logs_and_never_overwritten(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "must be below"):
            observer._prepare_output(Path("/tmp/odom-gap-evidence"))
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            path = observer._prepare_output(Path(directory) / "evidence")
            (path / "summary.json").write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "refusing to overwrite"):
                observer._prepare_output(path)

    def test_source_ast_is_subscription_only_and_callbacks_are_payload_free(
        self,
    ) -> None:
        source = (
            ROOT / "scripts" / "observe_odom_receipt_gaps_readonly.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(source)
        attribute_calls = [
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        ]
        self.assertEqual(attribute_calls.count("create_subscription"), 3)
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
        callback = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "observe_receipt"
        )
        self.assertEqual([argument.arg for argument in callback.args.args], ["_message"])
        self.assertFalse(
            any(
                isinstance(node, ast.Name)
                and node.id == "_message"
                and isinstance(node.ctx, ast.Load)
                for node in ast.walk(callback)
            )
        )
        self.assertTrue(
            any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "time"
                and node.func.attr == "perf_counter_ns"
                for node in ast.walk(callback)
            )
        )
        self.assertNotIn("time.monotonic_ns()", source)
        self.assertIn(
            "while rclpy.ok() and time.perf_counter_ns() < deadline_ns:",
            source,
        )
        wrapper = (
            ROOT / "scripts" / "observe_odom_receipt_gaps_readonly.sh"
        ).read_text(encoding="utf-8")
        self.assertNotIn("self._subscriptions =", source)
        self.assertIn("self._readonly_subscriptions =", source)
        self.assertNotIn("ActionClient(", source)
        self.assertNotIn("/cmd_vel", source)
        self.assertNotIn("/api/sport/request", source)
        self.assertNotIn("/lowcmd", source)
        self.assertIn("timeout --signal=TERM --kill-after=15s", wrapper)
        self.assertIn('ROS_LOG_DIR="$(realpath -m --', wrapper)
        self.assertIn('"${PROJECT_ROOT}"/*)', wrapper)
        self.assertIn("GO2_ODOM_GAP_INCLUDE_SPORT_PRIMARY", wrapper)
        self.assertIn("sport_primary_args=(--include-sport-primary)", wrapper)
        self.assertIn("GO2_ODOM_GAP_INCLUDE_CLOUD", wrapper)
        self.assertIn("cloud_args=(", wrapper)
        self.assertIn("--include-cloud", wrapper)
        self.assertIn("GO2_ODOM_GAP_CLOUD_QOS_RELIABILITY", wrapper)
        self.assertIn('--cloud-qos-reliability "${CLOUD_QOS_RELIABILITY}"', wrapper)

    def test_wrapper_rejects_invalid_cloud_switch_before_ros_start(self) -> None:
        wrapper = ROOT / "scripts" / "observe_odom_receipt_gaps_readonly.sh"
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            result = subprocess.run(
                ["bash", str(wrapper), str(Path(directory) / "evidence"), "1"],
                cwd=ROOT,
                env={
                    **observer.os.environ,
                    "GO2_ODOM_GAP_INCLUDE_CLOUD": "2",
                },
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
        self.assertEqual(result.returncode, 2)
        self.assertIn("GO2_ODOM_GAP_INCLUDE_CLOUD", result.stderr)
        self.assertNotIn("missing ROS 2 receipt-gap dependency", result.stderr)

    def test_wrapper_rejects_invalid_cloud_qos_before_ros_start(self) -> None:
        wrapper = ROOT / "scripts" / "observe_odom_receipt_gaps_readonly.sh"
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            result = subprocess.run(
                ["bash", str(wrapper), str(Path(directory) / "evidence"), "1"],
                cwd=ROOT,
                env={
                    **observer.os.environ,
                    "GO2_ODOM_GAP_INCLUDE_CLOUD": "1",
                    "GO2_ODOM_GAP_CLOUD_QOS_RELIABILITY": "sometimes",
                },
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
        self.assertEqual(result.returncode, 2)
        self.assertIn("GO2_ODOM_GAP_CLOUD_QOS_RELIABILITY", result.stderr)
        self.assertNotIn("missing ROS 2 receipt-gap dependency", result.stderr)

    def test_wrapper_rejects_unused_reliable_cloud_qos(self) -> None:
        wrapper = ROOT / "scripts" / "observe_odom_receipt_gaps_readonly.sh"
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            result = subprocess.run(
                ["bash", str(wrapper), str(Path(directory) / "evidence"), "1"],
                cwd=ROOT,
                env={
                    **observer.os.environ,
                    "GO2_ODOM_GAP_INCLUDE_CLOUD": "0",
                    "GO2_ODOM_GAP_CLOUD_QOS_RELIABILITY": "reliable",
                },
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
        self.assertEqual(result.returncode, 2)
        self.assertIn("GO2_ODOM_GAP_INCLUDE_CLOUD=1", result.stderr)
        self.assertNotIn("missing ROS 2 receipt-gap dependency", result.stderr)

    def test_wrapper_creates_output_when_ros_log_dir_is_preexisting_sibling(
        self,
    ) -> None:
        wrapper = ROOT / "scripts" / "observe_odom_receipt_gaps_readonly.sh"
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            root = Path(directory)
            output = root / "evidence"
            ros_logs = root / "caller-ros-logs"
            ros_logs.mkdir(mode=0o755)
            fake_bin = root / "fake-bin"
            fake_bin.mkdir()
            fake_timeout = fake_bin / "timeout"
            fake_timeout.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
            fake_timeout.chmod(0o700)

            result = subprocess.run(
                ["bash", str(wrapper), str(output), "1"],
                cwd=ROOT,
                env={
                    **observer.os.environ,
                    "GO2_ODOM_GAP_INCLUDE_CLOUD": "0",
                    "GO2_ODOM_GAP_INCLUDE_SPORT_PRIMARY": "0",
                    "PATH": (
                        f"{fake_bin}:"
                        f"{observer.os.environ.get('PATH', '')}"
                    ),
                    "ROS_LOG_DIR": str(ros_logs),
                },
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(output.is_dir())
            self.assertEqual(output.stat().st_mode & 0o777, 0o700)
            self.assertEqual(ros_logs.stat().st_mode & 0o777, 0o700)

    def test_ros_log_output_is_also_confined_to_the_package(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            output = Path(directory) / "evidence"
            output.mkdir()
            previous = observer.os.environ.get("ROS_LOG_DIR")
            try:
                observer.os.environ["ROS_LOG_DIR"] = "/tmp/forbidden-ros-log"
                with self.assertRaisesRegex(RuntimeError, "must remain below"):
                    observer._prepare_ros_log_dir(output)
                observer.os.environ.pop("ROS_LOG_DIR", None)
                ros_logs = observer._prepare_ros_log_dir(output)
                self.assertEqual(ros_logs, output / "ros-logs")
                self.assertEqual(ros_logs.stat().st_mode & 0o777, 0o700)
            finally:
                if previous is None:
                    observer.os.environ.pop("ROS_LOG_DIR", None)
                else:
                    observer.os.environ["ROS_LOG_DIR"] = previous

    def _run_with_fake_ros(
        self,
        *,
        include_sport_primary: bool,
        include_cloud: bool,
        emit_messages: bool,
        cloud_qos_reliability: str = "best_effort",
    ):
        state = {
            "context_shutdown": False,
            "destroyed": False,
            "init_options": None,
            "shutdown_uninstall_handlers": None,
            "subscriptions": [],
            "samples_emitted": False,
        }

        class SignalHandlerOptions:
            NO = object()

        class ExternalShutdownException(RuntimeError):
            pass

        def rclpy_sigterm_handler(_signal_number, _frame) -> None:
            state["context_shutdown"] = True

        def init(*, args=None, signal_handler_options=None) -> None:
            self.assertIsNone(args)
            state["init_options"] = signal_handler_options
            if signal_handler_options is not SignalHandlerOptions.NO:
                signal.signal(signal.SIGTERM, rclpy_sigterm_handler)

        def ok() -> bool:
            return not state["context_shutdown"]

        def spin_once(_node, *, timeout_sec) -> None:
            self.assertGreaterEqual(timeout_sec, 0)
            if emit_messages and not state["samples_emitted"]:
                for _ in range(2):
                    for subscription in state["subscriptions"]:
                        subscription["callback"](UnreadableMessage())
                state["samples_emitted"] = True
            handler = signal.getsignal(signal.SIGTERM)
            assert callable(handler)
            handler(signal.SIGTERM, None)
            if state["context_shutdown"]:
                raise ExternalShutdownException("context was shut down")

        def shutdown(*, uninstall_handlers=None) -> None:
            state["shutdown_uninstall_handlers"] = uninstall_handlers
            if state["context_shutdown"]:
                raise RuntimeError("context already shut down")
            state["context_shutdown"] = True

        class Node:
            def __init__(self, _name) -> None:
                pass

            def create_subscription(
                self, message_type, topic, callback, qos
            ):
                state["subscriptions"].append(
                    {
                        "message_type": message_type,
                        "topic": topic,
                        "callback": callback,
                        "qos": qos,
                    }
                )
                return object()

            def destroy_node(self) -> None:
                state["destroyed"] = True

        class QoSProfile:
            def __init__(self, **kwargs) -> None:
                self.settings = kwargs

        class Odometry:
            pass

        class SportModeState:
            pass

        class PointCloud2:
            pass

        class UnreadableMessage:
            def __getattribute__(self, name):
                raise AssertionError(f"callback read payload attribute {name}")

        keep_last = object()
        best_effort = object()
        reliable = object()
        volatile = object()
        state["expected_qos"] = {
            "history": keep_last,
            "depth": 1,
            "reliability": best_effort,
            "durability": volatile,
        }
        state["expected_reliable_qos"] = {
            **state["expected_qos"],
            "reliability": reliable,
        }
        fake_rclpy = types.ModuleType("rclpy")
        fake_rclpy.init = init
        fake_rclpy.ok = ok
        fake_rclpy.spin_once = spin_once
        fake_rclpy.shutdown = shutdown
        fake_nav_msgs = types.ModuleType("nav_msgs")
        fake_nav_msgs_msg = types.ModuleType("nav_msgs.msg")
        fake_nav_msgs_msg.Odometry = Odometry
        fake_nav_msgs.msg = fake_nav_msgs_msg
        fake_rclpy_node = types.ModuleType("rclpy.node")
        fake_rclpy_node.Node = Node
        fake_rclpy_qos = types.ModuleType("rclpy.qos")
        fake_rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=volatile)
        fake_rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=keep_last)
        fake_rclpy_qos.QoSProfile = QoSProfile
        fake_rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(
            BEST_EFFORT=best_effort,
            RELIABLE=reliable,
        )
        fake_rclpy_signals = types.ModuleType("rclpy.signals")
        fake_rclpy_signals.SignalHandlerOptions = SignalHandlerOptions
        fake_unitree_go = types.ModuleType("unitree_go")
        fake_unitree_go_msg = types.ModuleType("unitree_go.msg")
        fake_unitree_go_msg.SportModeState = SportModeState
        fake_unitree_go.msg = fake_unitree_go_msg
        fake_sensor_msgs = types.ModuleType("sensor_msgs")
        fake_sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        fake_sensor_msgs_msg.PointCloud2 = PointCloud2
        fake_sensor_msgs.msg = fake_sensor_msgs_msg
        fake_modules = {
            "rclpy": fake_rclpy,
            "rclpy.node": fake_rclpy_node,
            "rclpy.qos": fake_rclpy_qos,
            "rclpy.signals": fake_rclpy_signals,
            "nav_msgs": fake_nav_msgs,
            "nav_msgs.msg": fake_nav_msgs_msg,
            "unitree_go": fake_unitree_go,
            "unitree_go.msg": fake_unitree_go_msg,
            "sensor_msgs": fake_sensor_msgs,
            "sensor_msgs.msg": fake_sensor_msgs_msg,
        }

        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            output_dir = Path(directory) / "evidence"
            arguments = observer.build_argument_parser().parse_args(
                [
                    "--output-dir",
                    str(output_dir),
                    "--duration-seconds",
                    "1",
                ]
                + (["--include-sport-primary"] if include_sport_primary else [])
                + (["--include-cloud"] if include_cloud else [])
                + ["--cloud-qos-reliability", cloud_qos_reliability]
            )
            with (
                mock.patch.dict(sys.modules, fake_modules),
                mock.patch.dict(observer.os.environ, {}, clear=False),
                mock.patch.object(observer, "DEFAULT_THRESHOLDS_NS", (1,)),
            ):
                observer.os.environ.pop("ROS_LOG_DIR", None)
                status = observer.run_observer(arguments)

            summary = json.loads(
                (output_dir / "summary.json").read_text(encoding="utf-8")
            )
            metadata = json.loads(
                (output_dir / "metadata.json").read_text(encoding="utf-8")
            )
            events = [
                json.loads(line)
                for line in (output_dir / "events.jsonl")
                .read_text(encoding="utf-8")
                .splitlines()
            ]

        return status, summary, metadata, events, state, SignalHandlerOptions

    def test_sigterm_writes_clean_summary_and_returns_shell_signal_status(self) -> None:
        status, summary, metadata, events, state, options = self._run_with_fake_ros(
            include_sport_primary=False,
            include_cloud=False,
            emit_messages=False,
        )

        self.assertEqual(status, 128 + signal.SIGTERM)
        self.assertEqual(summary["exit_reason"], "signal_sigterm")
        self.assertIsNone(summary["runtime_error"])
        self.assertEqual(summary["cleanup_errors"], [])
        self.assertIs(state["init_options"], options.NO)
        self.assertIs(state["shutdown_uninstall_handlers"], False)
        self.assertTrue(state["destroyed"])
        self.assertEqual(metadata["ros_subscriptions_created"], 1)
        self.assertEqual(metadata["stream_order"], ["mid360_odom"])
        self.assertEqual(
            metadata["subscriptions"],
            [
                {
                    "stream": "mid360_odom",
                    "topic": "/utlidar/robot_odom",
                    "message_type": "nav_msgs/msg/Odometry",
                }
            ],
        )
        self.assertEqual(
            [item["topic"] for item in state["subscriptions"]],
            ["/utlidar/robot_odom"],
        )
        self.assertEqual(events, [])
        self.assertEqual(
            summary["receipt_gaps"],
            summary["streams"]["mid360_odom"]["receipt_gaps"],
        )

    def test_opt_in_sport_stream_is_independent_and_payload_free(self) -> None:
        status, summary, metadata, events, state, _options = self._run_with_fake_ros(
            include_sport_primary=True,
            include_cloud=False,
            emit_messages=True,
        )

        self.assertEqual(status, 128 + signal.SIGTERM)
        self.assertIsNone(summary["runtime_error"])
        self.assertEqual(summary["cleanup_errors"], [])
        self.assertEqual(metadata["ros_subscriptions_created"], 2)
        self.assertEqual(
            metadata["stream_order"], ["mid360_odom", "sport_primary"]
        )
        self.assertEqual(
            [item["topic"] for item in state["subscriptions"]],
            ["/utlidar/robot_odom", "/sportmodestate"],
        )
        self.assertEqual(
            [item["message_type"].__name__ for item in state["subscriptions"]],
            ["Odometry", "SportModeState"],
        )
        self.assertEqual(
            {event["stream"] for event in events},
            {"mid360_odom", "sport_primary"},
        )
        self.assertEqual(
            {event["topic"] for event in events},
            {"/utlidar/robot_odom", "/sportmodestate"},
        )
        self.assertEqual(summary["total_threshold_event_intervals"], 2)
        self.assertEqual(summary["total_event_records_written"], 2)
        for stream_name in ("mid360_odom", "sport_primary"):
            stream = summary["streams"][stream_name]
            self.assertEqual(stream["receipt_gaps"]["received_messages"], 2)
            self.assertEqual(stream["receipt_gaps"]["observed_intervals"], 1)
            self.assertEqual(stream["threshold_event_intervals"], 1)
            self.assertEqual(stream["event_records_written"], 1)

    def test_opt_in_cloud_stream_is_payload_free_and_qos_compatible(self) -> None:
        status, summary, metadata, events, state, _options = self._run_with_fake_ros(
            include_sport_primary=False,
            include_cloud=True,
            emit_messages=True,
        )

        self.assertEqual(status, 128 + signal.SIGTERM)
        self.assertIsNone(summary["runtime_error"])
        self.assertEqual(summary["cleanup_errors"], [])
        self.assertTrue(metadata["cloud_included"])
        self.assertFalse(metadata["sport_primary_included"])
        self.assertEqual(metadata["cloud_qos_reliability"], "best_effort")
        self.assertEqual(metadata["ros_subscriptions_created"], 2)
        self.assertEqual(
            metadata["stream_order"], ["mid360_odom", "mid360_cloud"]
        )
        self.assertEqual(
            metadata["subscriptions"],
            [
                {
                    "stream": "mid360_odom",
                    "topic": "/utlidar/robot_odom",
                    "message_type": "nav_msgs/msg/Odometry",
                },
                {
                    "stream": "mid360_cloud",
                    "topic": "/utlidar/cloud",
                    "message_type": "sensor_msgs/msg/PointCloud2",
                },
            ],
        )
        self.assertEqual(
            [item["message_type"].__name__ for item in state["subscriptions"]],
            ["Odometry", "PointCloud2"],
        )
        self.assertEqual(
            [item["topic"] for item in state["subscriptions"]],
            ["/utlidar/robot_odom", "/utlidar/cloud"],
        )
        for subscription in state["subscriptions"]:
            self.assertEqual(subscription["qos"].settings, state["expected_qos"])
        self.assertEqual(
            {event["stream"] for event in events},
            {"mid360_odom", "mid360_cloud"},
        )
        self.assertEqual(
            {event["topic"] for event in events},
            {"/utlidar/robot_odom", "/utlidar/cloud"},
        )
        self.assertIn("mid360_cloud", summary["streams"])
        self.assertEqual(
            summary["streams"]["mid360_cloud"]["message_type"],
            "sensor_msgs/msg/PointCloud2",
        )
        # Existing v1 readers continue to see the original odometry aliases.
        self.assertEqual(
            summary["receipt_gaps"],
            summary["streams"]["mid360_odom"]["receipt_gaps"],
        )

    def test_reliable_cloud_qos_does_not_change_other_streams(self) -> None:
        status, summary, metadata, events, state, _options = self._run_with_fake_ros(
            include_sport_primary=True,
            include_cloud=True,
            emit_messages=True,
            cloud_qos_reliability="reliable",
        )

        self.assertEqual(status, 128 + signal.SIGTERM)
        self.assertIsNone(summary["runtime_error"])
        self.assertEqual(summary["cleanup_errors"], [])
        self.assertEqual(metadata["cloud_qos_reliability"], "reliable")
        self.assertEqual(metadata["ros_subscriptions_created"], 3)
        self.assertEqual(
            [item["topic"] for item in state["subscriptions"]],
            ["/utlidar/robot_odom", "/sportmodestate", "/utlidar/cloud"],
        )
        self.assertEqual(
            [item["qos"].settings for item in state["subscriptions"]],
            [
                state["expected_qos"],
                state["expected_qos"],
                state["expected_reliable_qos"],
            ],
        )
        self.assertEqual(
            {event["stream"] for event in events},
            {"mid360_odom", "sport_primary", "mid360_cloud"},
        )


if __name__ == "__main__":
    unittest.main()
