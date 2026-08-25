from __future__ import annotations

import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import yaml

from go2_description_provider.runtime import (
    StaticTransformSentinel,
    fixed_joint_transforms,
    require_pinned_urdf,
    static_transform_qos_profile,
    validate_urdf,
    write_robot_state_publisher_params,
)


ROOT = Path(__file__).resolve().parents[1]
PINNED = ROOT / "urdf" / "go2_robonix.urdf"


class DescriptionRuntimeTests(unittest.TestCase):
    def test_pinned_model_has_required_single_root(self) -> None:
        root, links, joints = validate_urdf(PINNED.read_text(encoding="utf-8"))
        self.assertEqual(root, "base_link")
        self.assertGreater(links, 20)
        self.assertGreater(joints, 20)

    def test_unilidar_imu_frame_uses_vendor_extrinsic(self) -> None:
        model = ET.fromstring(PINNED.read_text(encoding="utf-8"))
        self.assertIsNotNone(model.find("./link[@name='utlidar_lidar']"))
        self.assertIsNotNone(model.find("./link[@name='utlidar_imu']"))

        joint = model.find("./joint[@name='utlidar_lidar_to_utlidar_imu']")
        self.assertIsNotNone(joint)
        self.assertEqual(joint.attrib["type"], "fixed")
        self.assertEqual(joint.find("parent").attrib["link"], "utlidar_lidar")
        self.assertEqual(joint.find("child").attrib["link"], "utlidar_imu")
        self.assertEqual(
            joint.find("origin").attrib["xyz"],
            "-0.007698 -0.014655 0.00667",
        )
        self.assertEqual(joint.find("origin").attrib["rpy"], "0 0 0")

    def test_unilidar_alias_matches_live_vendor_base_geometry(self) -> None:
        model = ET.fromstring(PINNED.read_text(encoding="utf-8"))
        radar = model.find("./joint[@name='radar_joint']/origin")
        alias = model.find("./joint[@name='radar_to_utlidar_lidar']/origin")
        self.assertIsNotNone(radar)
        self.assertIsNotNone(alias)
        self.assertEqual(radar.attrib["xyz"], "0.28945 0 -0.046825")
        self.assertEqual(radar.attrib["rpy"], "0 2.8782 0")
        self.assertEqual(
            alias.attrib["xyz"], "-0.005152655 0 -0.047108077"
        )
        self.assertEqual(
            alias.attrib["rpy"],
            "-0.001508482 -0.000981520 -2.146758680",
        )

        def vector(text: str) -> tuple[float, float, float]:
            values = tuple(float(value) for value in text.split())
            self.assertEqual(len(values), 3)
            return values

        def rotation(
            rpy: tuple[float, float, float],
        ) -> tuple[tuple[float, float, float], ...]:
            roll, pitch, yaw = rpy
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            return (
                (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                (-sp, cp * sr, cp * cr),
            )

        def multiply(
            left: tuple[tuple[float, float, float], ...],
            right: tuple[tuple[float, float, float], ...],
        ) -> tuple[tuple[float, float, float], ...]:
            return tuple(
                tuple(
                    sum(left[row][inner] * right[inner][column] for inner in range(3))
                    for column in range(3)
                )
                for row in range(3)
            )

        def rotate(
            matrix: tuple[tuple[float, float, float], ...],
            value: tuple[float, float, float],
        ) -> tuple[float, float, float]:
            return tuple(
                sum(matrix[row][column] * value[column] for column in range(3))
                for row in range(3)
            )

        radar_rotation = rotation(vector(radar.attrib["rpy"]))
        alias_rotation = rotation(vector(alias.attrib["rpy"]))
        composite_rotation = multiply(radar_rotation, alias_rotation)
        alias_translation = rotate(
            radar_rotation, vector(alias.attrib["xyz"])
        )
        composite_translation = tuple(
            parent + child
            for parent, child in zip(
                vector(radar.attrib["xyz"]), alias_translation
            )
        )
        expected_rotation = (
            (0.526113940854, -0.810135791020, 0.258619646098),
            (-0.838668148121, -0.544642761197, 0.000001586545),
            (0.140854032834, -0.216896894365, -0.965979233032),
        )
        expected_translation = (0.282160001, 0.0, 0.0)
        for actual, expected in zip(
            (value for row in composite_rotation for value in row),
            (value for row in expected_rotation for value in row),
        ):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(
            composite_translation, expected_translation
        ):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_fixed_joint_transform_contract_covers_base_and_sensors(self) -> None:
        pairs = fixed_joint_transforms(PINNED.read_text(encoding="utf-8"))
        self.assertIn(("base_link", "base"), pairs)
        self.assertIn(("base", "imu"), pairs)
        self.assertIn(("radar", "utlidar_lidar"), pairs)
        self.assertIn(("utlidar_lidar", "utlidar_imu"), pairs)

    def test_static_transform_sentinel_requires_complete_expected_tree(self) -> None:
        sentinel = StaticTransformSentinel(
            {("base_link", "base"), ("base", "imu")}
        )

        def message(*pairs: tuple[str, str]):
            return SimpleNamespace(
                transforms=[
                    SimpleNamespace(
                        header=SimpleNamespace(frame_id=parent),
                        child_frame_id=child,
                    )
                    for parent, child in pairs
                ]
            )

        sentinel.observe(message(("map", "odom"), ("/base_link", "base")))
        self.assertFalse(sentinel.ready)
        self.assertEqual(sentinel.missing, frozenset({("base", "imu")}))
        sentinel.observe(message(("base", "/imu")))
        self.assertTrue(sentinel.ready)
        self.assertEqual(sentinel.missing, frozenset())

    def test_static_transform_qos_is_reliable_and_transient_local(self) -> None:
        class FakeQoSProfile:
            def __init__(self, *, reliability, durability, history, depth) -> None:
                self.reliability = reliability
                self.durability = durability
                self.history = history
                self.depth = depth

        reliability = SimpleNamespace(RELIABLE=object())
        durability = SimpleNamespace(TRANSIENT_LOCAL=object())
        history = SimpleNamespace(KEEP_LAST=object())
        qos_module = ModuleType("rclpy.qos")
        qos_module.QoSProfile = FakeQoSProfile
        qos_module.ReliabilityPolicy = reliability
        qos_module.DurabilityPolicy = durability
        qos_module.HistoryPolicy = history
        rclpy_module = ModuleType("rclpy")
        rclpy_module.qos = qos_module

        with mock.patch.dict(
            sys.modules,
            {"rclpy": rclpy_module, "rclpy.qos": qos_module},
        ):
            qos = static_transform_qos_profile()

        self.assertIs(qos.reliability, reliability.RELIABLE)
        self.assertIs(qos.durability, durability.TRANSIENT_LOCAL)
        self.assertIs(qos.history, history.KEEP_LAST)
        self.assertEqual(qos.depth, 1)

    def test_exact_pinned_model_is_required(self) -> None:
        text = PINNED.read_text(encoding="utf-8")
        digest = require_pinned_urdf(text, PINNED)
        self.assertEqual(len(digest), 64)
        with self.assertRaisesRegex(ValueError, "does not match"):
            require_pinned_urdf(
                text.replace(
                    '<robot name="go2_description">',
                    '<robot name="go2_changed">',
                    1,
                ),
                PINNED,
            )

    def test_external_entities_and_duplicate_links_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "DTD"):
            validate_urdf('<!DOCTYPE robot><robot name="x"><link name="base_link"/></robot>')
        with self.assertRaisesRegex(ValueError, "duplicate"):
            validate_urdf('<robot name="x"><link name="base_link"/><link name="base_link"/></robot>')

    def test_parameter_file_preserves_pinned_urdf(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "params.yaml"
            text = PINNED.read_text(encoding="utf-8")
            write_robot_state_publisher_params(target, text)
            data = yaml.safe_load(target.read_text(encoding="utf-8"))
            self.assertEqual(data["/**"]["ros__parameters"]["robot_description"], text)

    def test_manifest_exposes_only_description_driver(self) -> None:
        manifest = yaml.safe_load((ROOT / "package_manifest.yaml").read_text(encoding="utf-8"))
        self.assertEqual(
            manifest["capabilities"],
            [{"name": "robonix/primitive/robot_description/driver"}],
        )
        source = (ROOT / "go2_description_provider" / "main.py").read_text(encoding="utf-8")
        for forbidden in ("create_publisher", "SportClient", "/cmd_vel", "/lowcmd"):
            self.assertNotIn(forbidden, source)
        self.assertNotIn('description.wait_for_topic(\n            "/tf_static"', source)


if __name__ == "__main__":
    unittest.main()
