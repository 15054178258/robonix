#!/usr/bin/env python3
"""Offline safety tests for the bounded SportModeState capture helpers."""

from __future__ import annotations

import ast
import importlib.util
import json
from pathlib import Path
import stat
import sys
import tempfile
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "scripts" / "capture_sport_state_readonly.py"
WRAPPER = ROOT / "scripts" / "capture_sport_state_readonly.sh"
EXPECTED_TOPICS = ("/sportmodestate", "/lf/sportmodestate")


def load_capture_module():
    spec = importlib.util.spec_from_file_location(
        "capture_sport_state_readonly_under_test", CAPTURE
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {CAPTURE}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeClock:
    def __init__(self) -> None:
        self._monotonic_values = iter((10.0, 10.1, 11.0))

    @staticmethod
    def time_ns() -> int:
        return 5_000_000_000

    @staticmethod
    def monotonic_ns() -> int:
        return 2_000_000_000

    def monotonic(self) -> float:
        return next(self._monotonic_values)


class Stamp:
    def __init__(self, sec: int, nanosec: int) -> None:
        self.sec = sec
        self.nanosec = nanosec


class SportState:
    def __init__(
        self,
        *,
        sec: int,
        nanosec: int,
        error_code: int,
        mode: int,
        gait_type: int,
        velocity: tuple[float, ...],
        yaw_speed: float,
    ) -> None:
        self.stamp = Stamp(sec, nanosec)
        self.error_code = error_code
        self.mode = mode
        self.gait_type = gait_type
        self.velocity = velocity
        self.yaw_speed = yaw_speed


def fake_ros_modules(*, take_error: bool = False):
    state = {
        "initialized": False,
        "shutdown": False,
        "context_destroyed": False,
        "context_init": None,
        "message_types_validated": [],
        "node_arguments": None,
        "node_destroyed": False,
        "wait_set_arguments": None,
        "wait_set_clear_calls": 0,
        "wait_set_timeouts_ns": [],
        "wait_set_destroyed": False,
        "subscriptions": [],
    }

    messages = {
        "/sportmodestate": SportState(
            sec=123,
            nanosec=456,
            error_code=2010,
            mode=0,
            gait_type=0,
            velocity=(0.1, -0.2, 0.3),
            yaw_speed=-0.4,
        ),
        "/lf/sportmodestate": SportState(
            sec=124,
            nanosec=789,
            error_code=100,
            mode=1,
            gait_type=2,
            velocity=(0.0, 0.0, 0.0),
            yaw_speed=0.0,
        ),
    }

    class FakeHandle:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> bool:
            del exc_type, exc_value, traceback
            return False

    class Context:
        def __init__(self) -> None:
            self.handle = FakeHandle()
            self._ok = False

        def init(self, *, args, initialize_logging) -> None:
            state["context_init"] = {
                "args": args,
                "initialize_logging": initialize_logging,
            }
            if args != []:
                raise AssertionError("capture must isolate itself from ambient ROS args")
            if initialize_logging is not False:
                raise AssertionError("capture must not initialize ROS logging")
            self._ok = True
            state["initialized"] = True

        def ok(self) -> bool:
            return self._ok

        def try_shutdown(self) -> None:
            self._ok = False
            state["shutdown"] = True

        def destroy(self) -> None:
            state["context_destroyed"] = True

    class FakeNode:
        def __init__(
            self,
            name,
            namespace,
            context_handle,
            cli_args,
            use_global_arguments,
            enable_rosout,
        ) -> None:
            state["node_arguments"] = (
                name,
                namespace,
                context_handle,
                cli_args,
                use_global_arguments,
                enable_rosout,
            )
            if use_global_arguments is not False or enable_rosout is not False:
                raise AssertionError("low-level node safety flags must remain disabled")

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> bool:
            del exc_type, exc_value, traceback
            return False

        def destroy_when_not_in_use(self) -> None:
            state["node_destroyed"] = True

    class FakeSubscription:
        _next_pointer = 101

        def __init__(self, node, message_type, topic, c_qos_profile) -> None:
            del node
            if message_type is not SportState:
                raise AssertionError("unexpected message type")
            if topic not in EXPECTED_TOPICS:
                raise AssertionError(f"unexpected subscription: {topic}")
            if c_qos_profile != {
                "history": "keep-last",
                "depth": 32,
                "reliability": "best-effort",
                "durability": "volatile",
            }:
                raise AssertionError(f"unexpected QoS: {c_qos_profile!r}")
            self.topic = topic
            self.pointer = self._next_pointer
            type(self)._next_pointer += 1
            self.destroyed = False
            self.taken = False
            state["subscriptions"].append(self)

        def take_message(self, message_type, raw):
            if message_type is not SportState or raw is not False:
                raise AssertionError("unexpected low-level take_message arguments")
            if take_error:
                raise RuntimeError("synthetic take failure")
            if self.taken:
                return None
            self.taken = True
            return messages[self.topic], {"source": "fake"}

        def destroy_when_not_in_use(self) -> None:
            self.destroyed = True

    class FakeWaitSet:
        def __init__(
            self,
            subscriptions,
            guard_conditions,
            timers,
            clients,
            services,
            events,
            context_handle,
        ) -> None:
            state["wait_set_arguments"] = (
                subscriptions,
                guard_conditions,
                timers,
                clients,
                services,
                events,
                context_handle,
            )
            self.subscriptions = []

        def clear_entities(self) -> None:
            state["wait_set_clear_calls"] += 1
            self.subscriptions.clear()

        def add_subscription(self, subscription) -> None:
            self.subscriptions.append(subscription)

        def wait(self, timeout_ns) -> None:
            state["wait_set_timeouts_ns"].append(timeout_ns)
            if not 0 <= timeout_ns <= 100_000_000:
                raise AssertionError("wait-set timeout must remain tightly bounded")

        def get_ready_entities(self, entity_type):
            if entity_type != "subscription":
                raise AssertionError(f"unexpected entity type: {entity_type}")
            return [subscription.pointer for subscription in self.subscriptions]

        def destroy_when_not_in_use(self) -> None:
            state["wait_set_destroyed"] = True

    class QoSProfile:
        def __init__(self, **values) -> None:
            self.values = values

        def get_c_qos_profile(self):
            return self.values

    rclpy = types.ModuleType("rclpy")
    context = types.ModuleType("rclpy.context")
    implementation = types.ModuleType("rclpy.impl")
    implementation_singleton = types.ModuleType(
        "rclpy.impl.implementation_singleton"
    )
    qos = types.ModuleType("rclpy.qos")
    type_support = types.ModuleType("rclpy.type_support")
    unitree_go = types.ModuleType("unitree_go")
    unitree_msg = types.ModuleType("unitree_go.msg")

    def check_is_valid_msg_type(message_type) -> None:
        if message_type is not SportState:
            raise AssertionError("unexpected type-support validation")
        state["message_types_validated"].append(message_type)

    context.Context = Context
    implementation_singleton.rclpy_implementation = types.SimpleNamespace(
        Node=FakeNode,
        Subscription=FakeSubscription,
        WaitSet=FakeWaitSet,
    )
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep-last")
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best-effort")
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")
    qos.QoSProfile = QoSProfile
    type_support.check_is_valid_msg_type = check_is_valid_msg_type
    unitree_msg.SportModeState = SportState
    rclpy.context = context
    rclpy.impl = implementation
    rclpy.qos = qos
    rclpy.type_support = type_support
    implementation.implementation_singleton = implementation_singleton
    unitree_go.msg = unitree_msg

    modules = {
        "rclpy": rclpy,
        "rclpy.context": context,
        "rclpy.impl": implementation,
        "rclpy.impl.implementation_singleton": implementation_singleton,
        "rclpy.qos": qos,
        "rclpy.type_support": type_support,
        "unitree_go": unitree_go,
        "unitree_go.msg": unitree_msg,
    }
    return modules, state


class SportStateReadonlyCaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.capture_source = CAPTURE.read_text(encoding="utf-8")
        cls.wrapper_source = WRAPPER.read_text(encoding="utf-8")
        cls.capture_tree = ast.parse(cls.capture_source, filename=str(CAPTURE))

    def test_topic_allowlist_is_exact_and_has_no_other_topic_literals(self) -> None:
        module = load_capture_module()
        self.assertEqual(module.ALLOWED_TOPICS, EXPECTED_TOPICS)
        topic_literals = {
            node.value
            for node in ast.walk(self.capture_tree)
            if isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and node.value.startswith("/")
        }
        self.assertEqual(topic_literals, set(EXPECTED_TOPICS))

    def test_python_helper_uses_only_low_level_subscription_handles(self) -> None:
        called_attributes = {
            node.func.attr
            for node in ast.walk(self.capture_tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        self.assertTrue(
            {
                "create_subscription",
                "create_publisher",
                "create_client",
                "create_service",
                "create_timer",
                "publish",
                "call",
                "call_async",
                "send_goal",
                "send_goal_async",
            }.isdisjoint(called_attributes)
        )
        imported_modules = {
            node.module
            for node in ast.walk(self.capture_tree)
            if isinstance(node, ast.ImportFrom) and node.module is not None
        }
        self.assertNotIn("rclpy.node", imported_modules)
        low_level_nodes = [
            node
            for node in ast.walk(self.capture_tree)
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
        low_level_subscriptions = [
            node
            for node in ast.walk(self.capture_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "_rclpy"
            and node.func.attr == "Subscription"
        ]
        self.assertEqual(len(low_level_subscriptions), 1)
        low_level_publishers = [
            node
            for node in ast.walk(self.capture_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "_rclpy"
            and node.func.attr == "Publisher"
        ]
        self.assertEqual(low_level_publishers, [])
        wait_sets = [
            node
            for node in ast.walk(self.capture_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "_rclpy"
            and node.func.attr == "WaitSet"
        ]
        self.assertEqual(len(wait_sets), 1)
        self.assertEqual(
            [argument.value for argument in wait_sets[0].args[:6]],
            [2] + [0] * 5,
        )
        context_init = next(
            node
            for node in ast.walk(self.capture_tree)
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

    def test_wrapper_has_outer_timeout_and_no_motion_or_ros_cli_commands(self) -> None:
        compact = " ".join(
            line.strip().removesuffix("\\").strip()
            for line in self.wrapper_source.splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        )
        self.assertIn(
            'timeout --signal=INT --kill-after=3s "$((duration + 5))s" '
            'python3 "${root}/scripts/capture_sport_state_readonly.py"',
            compact,
        )
        self.assertNotIn("ros2 ", compact)
        self.assertIn('readonly interface="${GO2_NETWORK_INTERFACE:-}"', compact)
        self.assertIn("ip link show dev", compact)
        self.assertIn("export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", compact)
        self.assertIn("export CYCLONEDDS_URI=", compact)
        for forbidden in (
            "/cmd_vel",
            "/api/sport/request",
            "/lowcmd",
            "sport_mode_ctrl",
            "go2_sport_client",
            "go2_stand_example",
            "low_level_ctrl",
            "SportClient",
            "ServiceSwitch",
        ):
            self.assertNotIn(forbidden, self.wrapper_source)

    def test_duration_bounds_are_enforced_before_ros_imports(self) -> None:
        module = load_capture_module()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "summary.json"
            for duration in (0, 121, -1):
                with self.subTest(duration=duration), mock.patch.object(
                    sys,
                    "argv",
                    [str(CAPTURE), "--duration", str(duration), "--output", str(output)],
                ):
                    with self.assertRaisesRegex(SystemExit, "1..120"):
                        module.main()

    def test_default_capture_window_remains_30_seconds(self) -> None:
        module = load_capture_module()
        with mock.patch.object(
            sys,
            "argv",
            [str(CAPTURE), "--output", "unused-summary.json"],
        ):
            self.assertEqual(module._parser().parse_args().duration, 30)

    def test_existing_output_is_not_overwritten(self) -> None:
        module = load_capture_module()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "summary.json"
            output.write_text("keep-me\n", encoding="utf-8")
            with mock.patch.object(
                sys,
                "argv",
                [str(CAPTURE), "--duration", "1", "--output", str(output)],
            ):
                with self.assertRaisesRegex(SystemExit, "refusing to overwrite"):
                    module.main()
            self.assertEqual(output.read_text(encoding="utf-8"), "keep-me\n")

    def test_summary_schema_with_fake_subscriber_only_ros(self) -> None:
        module = load_capture_module()
        modules, ros_state = fake_ros_modules()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "nested" / "summary.json"
            module.time = FakeClock()
            with mock.patch.dict(sys.modules, modules), mock.patch.object(
                sys,
                "argv",
                [str(CAPTURE), "--duration", "1", "--output", str(output)],
            ):
                self.assertEqual(module.main(), 0)

            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema_version"], 1)
            self.assertEqual(payload["mode"], "read-only-subscriber-only")
            self.assertEqual(payload["duration_limit_s"], 1)
            self.assertFalse(payload["publishers_created"])
            self.assertFalse(payload["unitree_clients_created"])
            self.assertEqual(len(payload["streams"]), 2)
            streams = {stream["topic"]: stream for stream in payload["streams"]}
            self.assertEqual(tuple(streams), EXPECTED_TOPICS)
            self.assertEqual(streams["/sportmodestate"]["received"], 1)
            self.assertEqual(
                streams["/sportmodestate"]["first_source_stamp_ns"],
                123_000_000_456,
            )
            self.assertEqual(
                streams["/sportmodestate"]["states"],
                [
                    {
                        "error_code": 2010,
                        "mode": 0,
                        "gait_type": 0,
                        "samples": 1,
                    }
                ],
            )
            self.assertEqual(
                streams["/sportmodestate"]["max_abs_linear_velocity"], 0.3
            )
            self.assertEqual(streams["/sportmodestate"]["max_abs_yaw_speed"], 0.4)
            self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)

        self.assertTrue(ros_state["initialized"])
        self.assertTrue(ros_state["shutdown"])
        self.assertTrue(ros_state["context_destroyed"])
        self.assertTrue(ros_state["node_destroyed"])
        self.assertTrue(ros_state["wait_set_destroyed"])
        self.assertGreaterEqual(ros_state["wait_set_clear_calls"], 2)
        self.assertEqual(
            ros_state["context_init"],
            {"args": [], "initialize_logging": False},
        )
        self.assertEqual(ros_state["message_types_validated"], [SportState])
        self.assertEqual(
            ros_state["node_arguments"][0:2],
            (
                "go2_sport_state_readonly_observer",
                "",
            ),
        )
        self.assertIsNone(ros_state["node_arguments"][3])
        self.assertEqual(ros_state["node_arguments"][4:6], (False, False))
        self.assertEqual(ros_state["wait_set_arguments"][:6], (2, 0, 0, 0, 0, 0))
        self.assertTrue(ros_state["wait_set_timeouts_ns"])
        self.assertTrue(
            all(
                0 <= timeout_ns <= 100_000_000
                for timeout_ns in ros_state["wait_set_timeouts_ns"]
            )
        )
        self.assertEqual(
            [subscription.topic for subscription in ros_state["subscriptions"]],
            list(EXPECTED_TOPICS),
        )
        self.assertTrue(
            all(subscription.destroyed for subscription in ros_state["subscriptions"])
        )

    def test_low_level_entities_are_cleaned_up_after_take_failure(self) -> None:
        module = load_capture_module()
        modules, ros_state = fake_ros_modules(take_error=True)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "summary.json"
            module.time = FakeClock()
            with mock.patch.dict(sys.modules, modules), mock.patch.object(
                sys,
                "argv",
                [str(CAPTURE), "--duration", "1", "--output", str(output)],
            ):
                with self.assertRaisesRegex(RuntimeError, "synthetic take failure"):
                    module.main()
            self.assertFalse(output.exists())

        self.assertTrue(ros_state["shutdown"])
        self.assertTrue(ros_state["context_destroyed"])
        self.assertTrue(ros_state["node_destroyed"])
        self.assertTrue(ros_state["wait_set_destroyed"])
        self.assertTrue(
            all(subscription.destroyed for subscription in ros_state["subscriptions"])
        )


if __name__ == "__main__":
    unittest.main()
