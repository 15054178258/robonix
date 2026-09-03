from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semantic_navigation.core import (
    DEFAULT_ARRIVAL_RADIUS_M,
    LandmarkError,
    LandmarkStore,
    normalize_text,
)


def document(
    *, verified: bool = True, map_id: str = "lab_go2", map_generation: int = 7
) -> dict:
    return {
        "schema_version": 2,
        "map_id": map_id,
        "map_generation": map_generation,
        "frame_id": "map",
        "landmarks": [
            {
                "id": "vending_machine_front",
                "name": "自动售货机",
                "aliases": ["售货机", "自动贩卖机"],
                "verified": verified,
                "pose": {"x": 1.25, "y": -0.75, "yaw": 7.0},
            }
        ],
    }


class LandmarkStoreTest(unittest.TestCase):
    def test_required_acceptance_phrase_resolves(self) -> None:
        store = LandmarkStore.from_mapping(document())
        landmark = store.resolve(
            "走到前面自动售货机那里",
            expected_map_id="lab_go2",
            expected_generation=7,
        )
        self.assertEqual(landmark.id, "vending_machine_front")
        self.assertEqual(landmark.name, "自动售货机")
        self.assertTrue(-math.pi <= landmark.yaw <= math.pi)
        self.assertEqual(landmark.kind, "navigation")
        self.assertEqual(landmark.arrival_radius, DEFAULT_ARRIVAL_RADIUS_M)

    def test_asr_spacing_and_punctuation_resolve(self) -> None:
        store = LandmarkStore.from_mapping(document())
        landmark = store.resolve(
            "请走到，自动 售货机！那里。",
            expected_map_id="lab_go2",
            expected_generation=7,
        )
        self.assertEqual(landmark.name, "自动售货机")

    def test_unverified_pose_fails_closed(self) -> None:
        store = LandmarkStore.from_mapping(document(verified=False))
        with self.assertRaisesRegex(LandmarkError, "verified"):
            store.resolve(
                "自动售货机", expected_map_id="lab_go2", expected_generation=7
            )

    def test_quoted_verified_value_is_rejected(self) -> None:
        raw = document()
        raw["landmarks"][0]["verified"] = "false"
        with self.assertRaisesRegex(LandmarkError, "YAML boolean"):
            LandmarkStore.from_mapping(raw)

    def test_wrong_active_map_fails_closed(self) -> None:
        store = LandmarkStore.from_mapping(document(map_id="floor_1"))
        with self.assertRaisesRegex(LandmarkError, "does not match"):
            store.resolve(
                "自动售货机", expected_map_id="floor_2", expected_generation=7
            )

    def test_wrong_map_generation_fails_closed(self) -> None:
        store = LandmarkStore.from_mapping(document(map_generation=12))
        with self.assertRaisesRegex(LandmarkError, "generation"):
            store.resolve(
                "自动售货机", expected_map_id="lab_go2", expected_generation=13
            )

    def test_generation_must_be_real_uint64(self) -> None:
        for invalid in (True, "7", -1, float("nan")):
            raw = document()
            raw["map_generation"] = invalid
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(LandmarkError, "uint64|within"):
                    LandmarkStore.from_mapping(raw)

    def test_unknown_name_fails_closed(self) -> None:
        store = LandmarkStore.from_mapping(document())
        with self.assertRaisesRegex(LandmarkError, "unknown"):
            store.resolve(
                "实验室大门", expected_map_id="lab_go2", expected_generation=7
            )

    def test_two_distinct_landmarks_in_one_utterance_are_rejected(self) -> None:
        raw = document()
        raw["landmarks"].append(
            {
                "id": "lab_door",
                "name": "实验室大门",
                "aliases": ["大门"],
                "verified": True,
                "pose": {"x": 0.0, "y": 1.0, "yaw": 0.0},
            }
        )
        store = LandmarkStore.from_mapping(raw)
        with self.assertRaisesRegex(LandmarkError, "ambiguous"):
            store.resolve(
                "先去实验室大门再去自动售货机",
                expected_map_id="lab_go2",
                expected_generation=7,
            )

    def test_multiple_chinese_destinations_and_aliases_resolve(self) -> None:
        raw = document()
        raw["landmarks"][0]["arrival_radius"] = 0.4
        raw["landmarks"].append(
            {
                "id": "meeting_room_east",
                "name": "东侧会议室",
                "aliases": ["东会议室", "一号会议室"],
                "kind": "navigation",
                "verified": True,
                "arrival_radius": 0.6,
                "pose": {"x": 4.0, "y": 1.5, "yaw": -1.57},
                "region": {
                    "points": [[3.0, 0.5], [5.0, 0.5], [5.0, 2.5], [3.0, 2.5]]
                },
            }
        )
        store = LandmarkStore.from_mapping(raw)
        landmark = store.resolve(
            "请带我去一号 会议室。",
            expected_map_id="lab_go2",
            expected_generation=7,
        )
        self.assertEqual(landmark.id, "meeting_room_east")
        self.assertEqual(landmark.arrival_radius, 0.6)
        self.assertEqual(len(landmark.region or ()), 4)

    def test_non_navigation_initial_marker_is_not_dispatched(self) -> None:
        raw = document()
        raw["landmarks"].append(
            {
                "id": "initial_reference",
                "name": "机器人初始位置",
                "aliases": ["起点", "初始点"],
                "kind": "marker",
                "verified": True,
                "pose": {"x": 0.2, "y": -0.1, "yaw": 0.0},
            }
        )
        store = LandmarkStore.from_mapping(raw)
        with self.assertRaisesRegex(LandmarkError, "non-navigation marker"):
            store.resolve(
                "回到起点",
                expected_map_id="lab_go2",
                expected_generation=7,
            )
        landmark = store.resolve(
            "从起点去售货机",
            expected_map_id="lab_go2",
            expected_generation=7,
        )
        self.assertEqual(landmark.id, "vending_machine_front")

    def test_region_only_marker_is_supported(self) -> None:
        raw = document()
        raw["landmarks"].append(
            {
                "id": "waiting_area",
                "name": "等候区",
                "aliases": [],
                "kind": "marker",
                "verified": False,
                "region": {"points": [[0, 0], [2, 0], [2, 1], [0, 1]]},
            }
        )
        store = LandmarkStore.from_mapping(raw)
        marker = next(item for item in store.landmarks if item.id == "waiting_area")
        self.assertFalse(marker.navigable)
        self.assertIsNone(marker.x)
        self.assertEqual(len(marker.region or ()), 4)

    def test_invalid_arrival_radius_and_region_fail_closed(self) -> None:
        for invalid in (True, 0.0, 10.1, float("nan")):
            raw = document()
            raw["landmarks"][0]["arrival_radius"] = invalid
            with self.subTest(arrival_radius=invalid):
                with self.assertRaisesRegex(LandmarkError, "arrival_radius"):
                    LandmarkStore.from_mapping(raw)
        raw = document()
        raw["landmarks"][0]["region"] = {
            "points": [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]
        }
        with self.assertRaisesRegex(LandmarkError, "zero area"):
            LandmarkStore.from_mapping(raw)

    def test_marker_cannot_carry_navigation_arrival_radius(self) -> None:
        raw = document()
        raw["landmarks"][0].update(
            {"kind": "marker", "arrival_radius": 0.5}
        )
        with self.assertRaisesRegex(LandmarkError, "cannot set arrival_radius"):
            LandmarkStore.from_mapping(raw)

    def test_non_finite_pose_is_rejected(self) -> None:
        raw = document()
        raw["landmarks"][0]["pose"]["x"] = float("nan")
        with self.assertRaisesRegex(LandmarkError, "finite"):
            LandmarkStore.from_mapping(raw)

    def test_normalization_does_not_translate_or_guess(self) -> None:
        self.assertEqual(normalize_text("自动 售货机！？"), "自动售货机")


if __name__ == "__main__":
    unittest.main()
