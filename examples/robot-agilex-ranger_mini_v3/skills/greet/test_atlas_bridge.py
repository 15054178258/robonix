# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from greet_skill import atlas_bridge


class ResolveInputsTest(unittest.TestCase):
    def test_camera_is_resolved_with_configured_provider(self):
        calls = []

        def find_unique_capability(**kwargs):
            calls.append(kwargs)
            return SimpleNamespace(provider_id=kwargs["provider_id"] or "speech")

        channel = SimpleNamespace(endpoint="resolved-endpoint", close=lambda: None)
        with (
            patch.object(
                atlas_bridge.ATLAS,
                "find_unique_capability",
                side_effect=find_unique_capability,
            ),
            patch.object(atlas_bridge.greet_skill, "connect_capability", return_value=channel),
        ):
            resolved = atlas_bridge.resolve_inputs(
                camera_provider_id="realsense_camera",
                soma_provider_id="soma",
                deadline_s=0,
            )

        self.assertEqual(resolved["camera"], "resolved-endpoint")
        camera = next(call for call in calls if call["transport"] == "ros2")
        speech = next(call for call in calls if call["transport"] == "mcp")
        self.assertEqual(camera["provider_id"], "realsense_camera")
        self.assertEqual(speech["provider_id"], "")

    def test_missing_camera_provider_is_rejected_before_atlas_query(self):
        with patch.object(atlas_bridge.ATLAS, "find_unique_capability") as find:
            with self.assertRaisesRegex(RuntimeError, "camera_provider_id"):
                atlas_bridge.resolve_inputs(
                    camera_provider_id="", soma_provider_id="soma", deadline_s=0
                )
            find.assert_not_called()

    def test_atlas_error_is_preserved_in_activation_error(self):
        with patch.object(
            atlas_bridge.ATLAS,
            "find_unique_capability",
            side_effect=RuntimeError("provider contract not found"),
        ):
            with self.assertRaisesRegex(RuntimeError, "provider contract not found"):
                atlas_bridge.resolve_inputs(
                    camera_provider_id="realsense_camera",
                    soma_provider_id="soma",
                    deadline_s=0,
                )


if __name__ == "__main__":
    unittest.main()
