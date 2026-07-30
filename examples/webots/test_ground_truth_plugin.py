#!/usr/bin/env python3
"""Unit tests for the Webots-only ground-truth publisher."""

from __future__ import annotations

import importlib.util
import json
import math
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PLUGIN_PATH = (
    ROOT
    / "sim"
    / "ros_ws"
    / "src"
    / "eaios_webots"
    / "eaios_webots"
    / "ground_truth_plugin.py"
)
SPEC = importlib.util.spec_from_file_location("ground_truth_plugin", PLUGIN_PATH)
assert SPEC and SPEC.loader
PLUGIN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLUGIN)


class GroundTruthPluginTest(unittest.TestCase):
    def test_parse_object_control_command(self) -> None:
        command = PLUGIN.parse_object_control_command(
            json.dumps(
                {
                    "request_id": "dropout-1",
                    "target_name": "potted tree(2)",
                    "translation": [2.17, -5.78, -5.0],
                }
            )
        )
        self.assertEqual(command.request_id, "dropout-1")
        self.assertEqual(command.target_name, "potted tree(2)")
        self.assertEqual(command.translation, (2.17, -5.78, -5.0))

    def test_parse_object_control_command_rejects_invalid_values(self) -> None:
        invalid = (
            "{}",
            '{"request_id":"x","target_name":"tree","translation":[1,2]}',
            '{"request_id":"x","target_name":"tree","translation":[1,2,NaN]}',
        )
        for payload in invalid:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    PLUGIN.parse_object_control_command(payload)

    def test_find_root_node_by_name(self) -> None:
        class FakeField:
            def __init__(self, value):
                self.value = value

            def getCount(self):
                return len(self.value)

            def getMFNode(self, index):
                return self.value[index]

            def getSFString(self):
                return self.value

        class FakeNode:
            def __init__(self, name=None, children=None):
                self.fields = {}
                if name is not None:
                    self.fields["name"] = FakeField(name)
                if children is not None:
                    self.fields["children"] = FakeField(children)

            def getField(self, name):
                return self.fields.get(name)

        target = FakeNode(name="potted tree(2)")
        root = FakeNode(children=[FakeNode(name="desk"), target])

        class FakeSupervisor:
            def getRoot(self):
                return root

        self.assertIs(
            PLUGIN.find_root_node_by_name(
                FakeSupervisor(),
                "potted tree(2)",
            ),
            target,
        )
        self.assertIsNone(
            PLUGIN.find_root_node_by_name(FakeSupervisor(), "missing")
        )

    def test_split_sim_time_normalizes_rounding(self) -> None:
        self.assertEqual(PLUGIN.split_sim_time(12.25), (12, 250_000_000))
        self.assertEqual(PLUGIN.split_sim_time(1.9999999996), (2, 0))

    def test_split_sim_time_rejects_invalid_values(self) -> None:
        for value in (-1.0, math.inf, math.nan):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "finite and non-negative"):
                    PLUGIN.split_sim_time(value)

    def test_identity_matrix(self) -> None:
        self.assertEqual(
            PLUGIN.quaternion_from_rotation_matrix(
                (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
            ),
            (0.0, 0.0, 0.0, 1.0),
        )

    def test_planar_quarter_turn(self) -> None:
        quaternion = PLUGIN.quaternion_from_rotation_matrix(
            (0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        )
        self.assertAlmostEqual(quaternion[0], 0.0)
        self.assertAlmostEqual(quaternion[1], 0.0)
        self.assertAlmostEqual(quaternion[2], math.sqrt(0.5))
        self.assertAlmostEqual(quaternion[3], math.sqrt(0.5))

    def test_rejects_wrong_matrix_size(self) -> None:
        with self.assertRaisesRegex(ValueError, "exactly nine"):
            PLUGIN.quaternion_from_rotation_matrix((1.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
