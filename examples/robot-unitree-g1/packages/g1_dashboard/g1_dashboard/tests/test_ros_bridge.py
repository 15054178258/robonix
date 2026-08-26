from __future__ import annotations

import ast
import os
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from go2_dashboard.ros_bridge import (
    RosConfig,
    _pose_map_observation,
    _require_exact_frame_id,
    _retain_telemetry_subscriptions,
    _source_age_seconds,
    _uint8,
    depth_to_rgb_bytes,
)


RUNTIME = Path(__file__).resolve().parents[1] / "go2_dashboard" / "ros_bridge.py"


class _ExecutorVisibleNode:
    """Small model of the rclpy Node subscription registry contract."""

    def __init__(self) -> None:
        self._subscriptions = ["rclpy:preexisting"]

    @property
    def subscriptions(self):
        yield from self._subscriptions

    def create_subscription(self, topic: str) -> str:
        handle = f"telemetry:{topic}"
        self._subscriptions.append(handle)
        return handle


def _pose_message(*, frame_id: str = "map", sec: int = 10, nanosec: int = 0):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame_id,
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.25, y=-2.5, z=0.3),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )


class RosBridgeScalarTests(unittest.TestCase):
    def test_uint8_accepts_integer_and_single_byte_ros_representations(self) -> None:
        self.assertEqual(_uint8(0, "level"), 0)
        self.assertEqual(_uint8(255, "level"), 255)
        self.assertEqual(_uint8(b"\x00", "level"), 0)
        self.assertEqual(_uint8(bytearray(b"\x02"), "level"), 2)
        self.assertEqual(_uint8(memoryview(b"\xff"), "level"), 255)

    def test_uint8_rejects_ambiguous_or_out_of_range_values(self) -> None:
        for value in (-1, 256, True, 1.0, "0", b"", b"\x00\x01", None):
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    _uint8(value, "level")

    def test_aligned_depth_preview_colorizes_zero_near_and_far(self) -> None:
        raw = b"\x00\x00\xfa\x00\x88\x13"
        rgb = depth_to_rgb_bytes(raw, 3, 1, 6, "16UC1", False)
        self.assertEqual(rgb[0:3], bytes((0, 0, 0)))
        self.assertEqual(rgb[3:6], bytes((0, 0, 255)))
        self.assertEqual(rgb[6:9], bytes((255, 0, 0)))

    def test_aligned_depth_preview_rejects_unsafe_layouts(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported"):
            depth_to_rgb_bytes(b"\x00\x00", 1, 1, 2, "32FC1", False)
        with self.assertRaisesRegex(ValueError, "shorter"):
            depth_to_rgb_bytes(b"\x00", 1, 1, 2, "16UC1", False)


class RosBridgeSubscriptionTests(unittest.TestCase):
    def test_retaining_telemetry_keeps_executor_registry_visible(self) -> None:
        node = _ExecutorVisibleNode()
        telemetry = [
            node.create_subscription("/camera"),
            node.create_subscription("/map"),
        ]

        retained = _retain_telemetry_subscriptions(node, telemetry)

        self.assertEqual(retained, tuple(telemetry))
        self.assertEqual(node._telemetry_subscriptions, tuple(telemetry))
        self.assertEqual(
            list(node.subscriptions),
            ["rclpy:preexisting", *telemetry],
        )

    def test_runtime_never_assigns_rclpy_private_subscription_registry(self) -> None:
        tree = ast.parse(RUNTIME.read_text(encoding="utf-8"), filename=str(RUNTIME))
        forbidden = []
        for node in ast.walk(tree):
            if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
                targets = (
                    node.targets if isinstance(node, ast.Assign) else [node.target]
                )
                for target in targets:
                    if (
                        isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id == "self"
                        and target.attr == "_subscriptions"
                    ):
                        forbidden.append(target.lineno)
        self.assertEqual(forbidden, [])

    def test_runtime_uses_depth_one_pose_subscription_and_no_tf_listener(self) -> None:
        source = RUNTIME.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(RUNTIME))
        for forbidden in (
            "tf2_ros",
            "TransformListener",
            "lookup_transform",
            "_tf_buffer",
            "_tf_listener",
            "_tf_timer",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)
        self.assertIn(
            "receipt_time_ns=self.get_clock().now().nanoseconds",
            source,
        )
        self.assertIn("source_age_s=source_age_s", source)

        pose_qos = next(
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "pose_qos"
                for target in node.targets
            )
        )
        self.assertIsInstance(pose_qos, ast.Call)
        keywords = {
            keyword.arg: ast.unparse(keyword.value)
            for keyword in pose_qos.keywords
        }
        self.assertEqual(keywords["history"], "HistoryPolicy.KEEP_LAST")
        self.assertEqual(keywords["depth"], "1")
        self.assertEqual(keywords["reliability"], "ReliabilityPolicy.RELIABLE")
        self.assertEqual(keywords["durability"], "DurabilityPolicy.VOLATILE")

    def test_map_callback_applies_the_exact_configured_frame_guard(self) -> None:
        compact_source = " ".join(RUNTIME.read_text(encoding="utf-8").split())
        self.assertIn(
            'message.header.frame_id, config.map_frame, "map"',
            compact_source,
        )


class RosBridgeConfigTests(unittest.TestCase):
    def test_pose_topic_default_environment_and_topic_spec(self) -> None:
        with patch.dict(os.environ, {}, clear=True):
            defaults = RosConfig.from_environment()
        self.assertEqual(defaults.pose_topic, "/robonix/map/pose")
        self.assertEqual(
            defaults.topic_specs()["pose_map"].topic,
            "/robonix/map/pose",
        )

        with patch.dict(
            os.environ,
            {"GO2_DASHBOARD_POSE_TOPIC": "/lab/map_pose"},
            clear=True,
        ):
            configured = RosConfig.from_environment()
        self.assertEqual(configured.pose_topic, "/lab/map_pose")
        self.assertEqual(
            configured.topic_specs()["pose_map"].topic,
            "/lab/map_pose",
        )

    def test_pose_topic_environment_must_be_absolute(self) -> None:
        with patch.dict(
            os.environ,
            {"GO2_DASHBOARD_POSE_TOPIC": "relative/pose"},
            clear=True,
        ):
            with self.assertRaisesRegex(ValueError, "absolute ROS topic"):
                RosConfig.from_environment()

    def test_d435i_topics_and_pose_store_environment_are_explicit(self) -> None:
        with patch.dict(
            os.environ,
            {
                "GO2_DASHBOARD_D435I_COLOR_TOPIC": "/lab/d435/color",
                "GO2_DASHBOARD_D435I_DEPTH_TOPIC": "/lab/d435/depth",
                "GO2_DASHBOARD_D435I_CAMERA_INFO_TOPIC": "/lab/d435/info",
                "GO2_DASHBOARD_INITIAL_POSE_MAPS_DIR": "/tmp/maps",
                "GO2_DASHBOARD_INITIAL_POSE_AUTO_RESTORE": "1",
            },
            clear=True,
        ):
            config = RosConfig.from_environment()
        self.assertEqual(config.d435i_color_topic, "/lab/d435/color")
        self.assertEqual(config.d435i_depth_topic, "/lab/d435/depth")
        self.assertEqual(config.d435i_camera_info_topic, "/lab/d435/info")
        self.assertEqual(config.initial_pose_maps_dir, "/tmp/maps")
        self.assertTrue(config.initial_pose_auto_restore)
        self.assertEqual(
            config.topic_specs()["d435i_depth"].topic, "/lab/d435/depth"
        )


class RosBridgePoseTests(unittest.TestCase):
    def test_map_frame_validator_requires_an_exact_match(self) -> None:
        self.assertEqual(_require_exact_frame_id("map", "map", "map"), "map")
        for frame_id in ("", "/map", "odom", "map "):
            with self.subTest(frame_id=frame_id):
                with self.assertRaisesRegex(
                    ValueError, "map frame_id must exactly match 'map'"
                ):
                    _require_exact_frame_id(frame_id, "map", "map")

    def test_pose_observation_validates_frame_and_calculates_source_age(self) -> None:
        message = _pose_message(sec=10, nanosec=250_000_000)
        payload, source_age_s = _pose_map_observation(
            message,
            expected_map_frame="map",
            base_frame="base_link",
            receipt_time_ns=12_500_000_000,
        )
        self.assertEqual(source_age_s, 2.25)
        self.assertEqual(payload["parent_frame"], "map")
        self.assertEqual(payload["child_frame"], "base_link")
        self.assertEqual(payload["stamp"]["unix_s"], 10.25)
        self.assertEqual(payload["position"], {"x": 1.25, "y": -2.5, "z": 0.3})
        self.assertEqual(payload["yaw"], 0.0)

    def test_pose_observation_rejects_non_map_frame(self) -> None:
        with self.assertRaisesRegex(ValueError, "exactly match 'map'"):
            _pose_map_observation(
                _pose_message(frame_id="odom"),
                expected_map_frame="map",
                base_frame="base_link",
                receipt_time_ns=10_000_000_000,
            )

    def test_source_age_supports_future_stamp_and_rejects_invalid_nanoseconds(self) -> None:
        self.assertEqual(
            _source_age_seconds(
                9_000_000_000,
                SimpleNamespace(sec=10, nanosec=0),
            ),
            -1.0,
        )
        for nanosec in (-1, 1_000_000_000):
            with self.subTest(nanosec=nanosec):
                with self.assertRaisesRegex(ValueError, "outside ROS time range"):
                    _source_age_seconds(
                        10_000_000_000,
                        SimpleNamespace(sec=10, nanosec=nanosec),
                    )


if __name__ == "__main__":
    unittest.main()
