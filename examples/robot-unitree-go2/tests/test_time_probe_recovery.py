from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
TIME_DIR = ROOT / "deploy" / "time-sync"

import sys

sys.path.insert(0, str(TIME_DIR))

import go2_time_core


def load_script(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


probe = load_script("time_probe_signal", "probe_go2_time_readonly.py")
recovery = load_script("time_probe_recovery", "recover_go2_time_summary.py")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_fake_low_level_ros_modules(root: Path) -> None:
    files = {
        "rclpy/__init__.py": (
            "def init(*args, **kwargs):\n"
            "    raise AssertionError('high-level rclpy.init used')\n"
            "def spin_once(*args, **kwargs):\n"
            "    raise AssertionError('high-level rclpy.spin_once used')\n"
        ),
        "rclpy/context.py": (
            "class _Handle:\n"
            "    def __enter__(self):\n        return self\n"
            "    def __exit__(self, *args):\n        return False\n"
            "class Context:\n"
            "    def __init__(self):\n"
            "        self.handle = _Handle()\n"
            "        self._ok = False\n"
            "    def init(self, args=None, initialize_logging=True):\n"
            "        if initialize_logging:\n"
            "            raise AssertionError('logging must remain disabled')\n"
            "        self._ok = True\n"
            "    def ok(self):\n        return self._ok\n"
            "    def try_shutdown(self):\n        self._ok = False\n"
            "    def destroy(self):\n        pass\n"
        ),
        "rclpy/node.py": (
            "raise AssertionError('high-level rclpy.node must not be imported')\n"
        ),
        "rclpy/impl/__init__.py": "",
        "rclpy/impl/implementation_singleton.py": (
            "import os\n"
            "from pathlib import Path\n"
            "from types import SimpleNamespace\n"
            "import time\n"
            "_next_pointer = 1\n"
            "class Node:\n"
            "    def __init__(self, name, namespace, context, cli_args, "
            "use_global_arguments, enable_rosout):\n"
            "        if use_global_arguments or enable_rosout:\n"
            "            raise AssertionError('unsafe low-level node options')\n"
            "    def __enter__(self):\n        return self\n"
            "    def __exit__(self, *args):\n        return False\n"
            "    def destroy_when_not_in_use(self):\n        pass\n"
            "class Subscription:\n"
            "    def __init__(self, node, message_type, topic, qos):\n"
            "        global _next_pointer\n"
            "        self.pointer = _next_pointer\n"
            "        _next_pointer += 1\n"
            "        self.message_type = message_type\n"
            "        self.taken = False\n"
            "    def take_message(self, message_type, raw):\n"
            "        if raw or message_type is not self.message_type:\n"
            "            raise AssertionError('unexpected take_message request')\n"
            "        if self.taken:\n            return None\n"
            "        self.taken = True\n"
            "        return (message_type(), None)\n"
            "    def destroy_when_not_in_use(self):\n        pass\n"
            "class WaitSet:\n"
            "    def __init__(self, subscriptions, guards, timers, clients, "
            "services, events, context):\n"
            "        if (subscriptions, guards, timers, clients, services, events) "
            "!= (5, 0, 0, 0, 0, 0):\n"
            "            raise AssertionError('unexpected wait-set capacity')\n"
            "        self.subscriptions = []\n"
            "    def clear_entities(self):\n        self.subscriptions = []\n"
            "    def add_subscription(self, subscription):\n"
            "        self.subscriptions.append(subscription)\n"
            "        if len(self.subscriptions) == 5:\n"
            "            ready = os.environ.get('FAKE_ROS_READY')\n"
            "            if ready:\n                Path(ready).touch()\n"
            "    def wait(self, timeout_ns):\n"
            "        emit = os.environ.get('FAKE_ROS_EMIT_MESSAGES') == '1'\n"
            "        if not emit or all(item.taken for item in self.subscriptions):\n"
            "            time.sleep(min(timeout_ns / 1_000_000_000, 0.05))\n"
            "    def get_ready_entities(self, kind):\n"
            "        if kind != 'subscription':\n"
            "            raise AssertionError('unexpected entity kind')\n"
            "        if os.environ.get('FAKE_ROS_EMIT_MESSAGES') != '1':\n"
            "            return []\n"
            "        return [item.pointer for item in self.subscriptions if not item.taken]\n"
            "    def destroy_when_not_in_use(self):\n        pass\n"
            "rclpy_implementation = SimpleNamespace(\n"
            "    Node=Node, Subscription=Subscription, WaitSet=WaitSet)\n"
        ),
        "rclpy/qos.py": (
            "class DurabilityPolicy:\n    VOLATILE = 1\n"
            "class HistoryPolicy:\n    KEEP_LAST = 1\n"
            "class ReliabilityPolicy:\n    BEST_EFFORT = 1\n"
            "class QoSProfile:\n"
            "    def __init__(self, **kwargs):\n        pass\n"
            "    def get_c_qos_profile(self):\n        return object()\n"
        ),
        "rclpy/type_support.py": (
            "def check_is_valid_msg_type(message_type):\n"
            "    if not isinstance(message_type, type):\n"
            "        raise AssertionError('invalid fake message type')\n"
        ),
        "sensor_msgs/__init__.py": "",
        "sensor_msgs/msg/__init__.py": (
            "class _Stamp:\n"
            "    sec = 100\n"
            "    nanosec = 1\n"
            "class _Header:\n"
            "    def __init__(self):\n        self.stamp = _Stamp()\n"
            "class Imu:\n"
            "    def __init__(self):\n        self.header = _Header()\n"
            "class PointCloud2:\n"
            "    def __init__(self):\n        self.header = _Header()\n"
        ),
        "nav_msgs/__init__.py": "",
        "nav_msgs/msg/__init__.py": (
            "class _Stamp:\n"
            "    sec = 100\n"
            "    nanosec = 1\n"
            "class _Header:\n"
            "    def __init__(self):\n        self.stamp = _Stamp()\n"
            "class Odometry:\n"
            "    def __init__(self):\n        self.header = _Header()\n"
        ),
        "unitree_go/__init__.py": "",
        "unitree_go/msg/__init__.py": (
            "class _Stamp:\n"
            "    sec = 100\n"
            "    nanosec = 1\n"
            "class SportModeState:\n"
            "    def __init__(self):\n        self.stamp = _Stamp()\n"
        ),
    }
    for relative, source in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source, encoding="utf-8")


class TimeProbeRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        (ROOT / "logs").mkdir(parents=True, exist_ok=True)

    def make_evidence(self, directory: Path) -> tuple[Path, Path]:
        metadata = {
            "schema_version": 1,
            "mode": "read-only-no-adjust",
            "clock_adjustment_requested": False,
            "ros_publishers_created": False,
            "unitree_clients_created": False,
            "started_realtime_ns": 100_000_000_000,
            "started_monotonic_ns": 5_000_000_000,
            "duration_seconds": 60,
            "max_samples": 1000,
            "retained_offsets_per_stream": 100,
            "topics": {"sport_primary": "/sportmodestate"},
        }
        metadata_path = directory / "metadata.json"
        metadata_path.write_text(json.dumps(metadata) + "\n", encoding="utf-8")
        tracker = go2_time_core.StreamTracker("sport_primary", "/sportmodestate")
        records = []
        for index in range(3):
            records.append(
                tracker.observe(
                    100 + index,
                    0,
                    100_500_000_000 + index * 1_000_000_000,
                    5_500_000_000 + index * 1_000_000_000,
                    200 + index,
                ).as_dict()
            )
        samples_path = directory / "samples.jsonl"
        samples_path.write_text(
            "".join(json.dumps(record, sort_keys=True) + "\n" for record in records),
            encoding="utf-8",
        )
        return metadata_path, samples_path

    def test_recovery_validates_raw_records_without_modifying_them(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary:
            directory = Path(temporary)
            metadata_path, samples_path = self.make_evidence(directory)
            before = (digest(metadata_path), digest(samples_path))
            output, summary = recovery.recover_summary(
                directory, "summary.recovered.json"
            )
            self.assertEqual(before, (digest(metadata_path), digest(samples_path)))
            self.assertEqual(summary["total_samples"], 3)
            self.assertEqual(
                summary["exit_reason"], "incomplete_before_duration_or_sample_cap"
            )
            self.assertEqual(summary["termination_cause"], "unknown")
            self.assertFalse(summary["duration_boundary_observed"])
            self.assertFalse(summary["sample_cap_observed"])
            self.assertFalse(summary["safe_for_clock_discipline"])
            self.assertEqual(summary["raw_observation_validation_errors"], 0)
            self.assertEqual(output.stat().st_mode & 0o777, 0o600)
            with self.assertRaises(FileExistsError):
                recovery.recover_summary(directory, "summary.recovered.json")

    def test_recovery_rejects_edited_observation(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary:
            directory = Path(temporary)
            _metadata_path, samples_path = self.make_evidence(directory)
            records = [
                json.loads(line)
                for line in samples_path.read_text(encoding="utf-8").splitlines()
            ]
            records[1]["source_delta_ns"] = 7
            samples_path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "does not match reconstruction"):
                recovery.recover_summary(directory, "summary.recovered.json")
            self.assertFalse((directory / "summary.recovered.json").exists())

    def test_recovery_rejects_paths_outside_repository_logs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with self.assertRaisesRegex(ValueError, "must resolve below"):
                recovery.recover_summary(directory, "summary.recovered.json")
            with tempfile.TemporaryDirectory(dir=ROOT / "logs") as logs_temporary:
                symlink = Path(logs_temporary) / "outside-link"
                symlink.symlink_to(directory, target_is_directory=True)
                with self.assertRaisesRegex(ValueError, "must resolve below"):
                    recovery._validated_evidence_directory(symlink)

    def test_recovery_output_cannot_impersonate_canonical_evidence(self) -> None:
        for invalid in (
            "summary.json",
            "metadata.json",
            "samples.jsonl",
            "../summary.recovered.json",
            "summary.recovered.json.tmp",
            "summary.recovered-.json",
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(ValueError, "independent"):
                    recovery._validated_output_name(invalid)
        for valid in (
            "summary.recovered.json",
            "summary.recovered-canonical.json",
            "summary.recovered.audit-2.json",
        ):
            self.assertEqual(recovery._validated_output_name(valid), valid)

    def test_sigterm_is_deferred_to_auditable_exit_reason(self) -> None:
        request = probe.TerminationRequest()
        request.handler(signal.SIGTERM, None)
        request.handler(signal.SIGINT, None)
        self.assertTrue(request.requested)
        self.assertEqual(request.exit_reason, "signal_sigterm")
        self.assertEqual(request.exit_status, 143)

    def test_offline_fake_ros_sigterm_still_writes_summary(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temporary_path = Path(temporary)
            fake_modules = temporary_path / "fake-modules"
            output = temporary_path / "evidence"
            ready = temporary_path / "fake-ros-ready"
            write_fake_low_level_ros_modules(fake_modules)
            environment = os.environ.copy()
            environment["PYTHONPATH"] = str(fake_modules)
            environment["FAKE_ROS_READY"] = str(ready)
            environment["FAKE_ROS_EMIT_MESSAGES"] = "0"
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "probe_go2_time_readonly.py"),
                    "--output-dir",
                    str(output),
                    "--duration-seconds",
                    "30",
                    "--max-samples",
                    "100",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
            )
            deadline = time.monotonic() + 5
            while not ready.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(ready.exists(), "fake ROS probe did not become ready")
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 143, (stdout, stderr))
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["exit_reason"], "signal_sigterm")
            self.assertEqual(summary["total_samples"], 0)
            self.assertEqual(summary["cleanup_errors"], [])
            self.assertFalse(summary["safe_for_clock_discipline"])
            self.assertEqual((output / "summary.json").stat().st_mode & 0o777, 0o600)

    def test_offline_fake_low_level_waitset_records_all_five_streams(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temporary_path = Path(temporary)
            fake_modules = temporary_path / "fake-modules"
            output = temporary_path / "evidence"
            write_fake_low_level_ros_modules(fake_modules)
            environment = os.environ.copy()
            environment["PYTHONPATH"] = str(fake_modules)
            environment["FAKE_ROS_EMIT_MESSAGES"] = "1"
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "probe_go2_time_readonly.py"),
                    "--output-dir",
                    str(output),
                    "--duration-seconds",
                    "30",
                    "--max-samples",
                    "5",
                ],
                capture_output=True,
                text=True,
                env=environment,
                timeout=5,
                check=False,
            )
            self.assertEqual(result.returncode, 0, (result.stdout, result.stderr))
            metadata = json.loads(
                (output / "metadata.json").read_text(encoding="utf-8")
            )
            summary = json.loads(
                (output / "summary.json").read_text(encoding="utf-8")
            )
            samples = [
                json.loads(line)
                for line in (output / "samples.jsonl")
                .read_text(encoding="utf-8")
                .splitlines()
            ]
            self.assertFalse(metadata["ros_publishers_created"])
            self.assertEqual(summary["exit_reason"], "max_samples_reached")
            self.assertEqual(summary["total_samples"], 5)
            self.assertEqual(summary["cleanup_errors"], [])
            self.assertEqual(len(samples), 5)
            self.assertEqual(
                {sample["stream"] for sample in samples},
                set(probe.DEFAULT_TOPICS),
            )
            self.assertEqual(
                {stream["stream"]: stream["received"] for stream in summary["streams"]},
                {stream: 1 for stream in probe.DEFAULT_TOPICS},
            )

    def test_wrapper_requests_graceful_term_and_preserves_status(self) -> None:
        source = (ROOT / "scripts" / "probe_go2_time_readonly.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("timeout --signal=TERM --kill-after=15s --preserve-status", source)
        self.assertIn('[[ -s "${OUTPUT_DIR}/summary.json" ]]', source)
        self.assertNotIn("timeout --signal=INT --kill-after=5s", source)

    def test_four_qualification_streams_plus_fallback_are_configured(self) -> None:
        arguments = probe.build_argument_parser().parse_args(
            ["--output-dir", str(ROOT / "logs" / "unused-time-probe-test")]
        )
        self.assertEqual(arguments.primary_topic, "/sportmodestate")
        self.assertEqual(arguments.fallback_topic, "/lf/sportmodestate")
        self.assertEqual(arguments.cloud_topic, "/utlidar/cloud")
        self.assertEqual(arguments.imu_topic, "/utlidar/imu")
        self.assertEqual(arguments.odom_topic, "/utlidar/robot_odom")
        self.assertEqual(
            probe.QUALIFICATION_STREAMS,
            ("sport_primary", "mid360_imu", "mid360_cloud", "mid360_odom"),
        )
        self.assertEqual(probe.WITNESS_STREAMS, ("sport_fallback",))

        wrapper = (ROOT / "scripts" / "probe_go2_time_readonly.sh").read_text(
            encoding="utf-8"
        )
        locator = (
            ROOT / "scripts" / "collect_go2_publisher_locators_readonly.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("--odom-topic", wrapper)
        self.assertIn("GO2_TIME_ODOM_TOPIC", wrapper)
        self.assertIn("/utlidar/robot_odom", wrapper)
        self.assertIn("/utlidar/robot_odom", locator)
        self.assertIn("mid360_odom", locator)

    def test_sigterm_handler_restoration_covers_entire_probe(self) -> None:
        source = (ROOT / "scripts" / "probe_go2_time_readonly.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("return execute_probe()", source)
        self.assertIn(
            "signal.signal(signal.SIGTERM, previous_sigterm_handler)", source
        )
        self.assertIn("This outer finally covers normal return", source)

    def test_recovery_source_has_no_ros_network_or_publish_path(self) -> None:
        source = (ROOT / "scripts" / "recover_go2_time_summary.py").read_text(
            encoding="utf-8"
        )
        for forbidden in (
            "import rclpy",
            "create_publisher",
            "create_subscription",
            "SportClient",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "subprocess",
            "socket",
        ):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
