# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for Scene label evidence and class-safe association."""

import asyncio
from types import SimpleNamespace

import numpy as np
import pytest

from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _adaptive_association_distance_limit,
    _disjoint_periodic_merge_mask,
    _label_evidence,
    _one_to_one_association_mask,
    _resolved_classes,
)
from scene_service.state.object_registry import BBox3D, ObjectRegistry, Pose3D


def _evidence(class_ids, confidences, *, current="chair", label_aliases=None):
    return _label_evidence(
        {"class_id": class_ids, "conf": confidences, "class_name": current},
        ["chair", "table"],
        current_label=current,
        history_size=20,
        min_switch_observations=3,
        min_winner_share=0.65,
        switch_margin=0.20,
        label_aliases=label_aliases,
    )


def test_one_to_one_association_reserves_each_existing_track_once() -> None:
    selected = _one_to_one_association_mask(
        np.asarray([[1.30], [1.20]], dtype=np.float64),
        threshold=0.85,
    )
    assert selected.tolist() == [[True], [False]]


def test_one_to_one_association_finds_global_maximum_with_unmatched_option() -> None:
    selected = _one_to_one_association_mask(
        np.asarray(
            [
                [1.20, 1.19],
                [1.18, 0.10],
                [0.20, 0.30],
            ],
            dtype=np.float64,
        ),
        threshold=0.85,
    )
    assert selected.tolist() == [
        [False, True],
        [True, False],
        [False, False],
    ]


def test_periodic_merge_selection_breaks_transitive_candidate_chain() -> None:
    selected = _disjoint_periodic_merge_mask(
        np.asarray(
            [
                [0.0, 0.90, 0.0],
                [0.90, 0.0, 0.80],
                [0.0, 0.80, 0.0],
            ],
            dtype=np.float64,
        ),
        spatial_threshold=0.50,
        visual_scores=np.asarray(
            [
                [1.0, 0.95, 0.20],
                [0.95, 1.0, 0.93],
                [0.20, 0.93, 1.0],
            ],
            dtype=np.float64,
        ),
        visual_threshold=0.65,
    )
    assert selected.tolist() == [
        [False, True, False],
        [True, False, False],
        [False, False, False],
    ]


def test_adaptive_association_distance_scales_with_measured_geometry() -> None:
    repeated_fixture_limit = _adaptive_association_distance_limit(
        (1.0, 0.05),
        (1.5, 0.05),
        minimum_m=0.45,
        maximum_m=1.5,
        extent_scale=0.80,
    )
    partial_large_object_limit = _adaptive_association_distance_limit(
        (0.5, 0.4),
        (1.5, 0.8),
        minimum_m=0.45,
        maximum_m=1.5,
        extent_scale=0.80,
    )
    assert repeated_fixture_limit < 1.0
    assert partial_large_object_limit > 0.70
    assert _adaptive_association_distance_limit(
        (4.0, 3.0),
        (4.0, 3.0),
        minimum_m=0.45,
        maximum_m=1.5,
        extent_scale=0.80,
    ) == pytest.approx(1.5)


def test_adaptive_association_distance_fails_closed_for_invalid_extent() -> None:
    assert _adaptive_association_distance_limit(
        (float("nan"), 1.0),
        (1.0, 1.0),
        minimum_m=0.45,
        maximum_m=1.5,
        extent_scale=0.80,
    ) == pytest.approx(0.45)


def test_label_stays_provisional_until_repeated_evidence() -> None:
    first = _evidence([0], [0.90])
    assert first["label"] == "chair"
    assert first["provisional"] is True
    assert first["evidence_count"] == 1

    stable = _evidence([0, 0, 0], [0.80, 0.90, 0.85])
    assert stable["label"] == "chair"
    assert stable["provisional"] is False
    assert stable["confidence"] == pytest.approx(1.0)


def test_label_switch_requires_support_share_and_margin() -> None:
    weak_challenger = _evidence(
        [0, 0, 0, 1, 1],
        [0.90, 0.90, 0.90, 0.95, 0.95],
    )
    assert weak_challenger["label"] == "chair"

    strong_challenger = _evidence(
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        [0.90, 0.90, 0.90] + [0.95] * 7,
    )
    assert strong_challenger["label"] == "table"
    assert strong_challenger["provisional"] is False
    assert strong_challenger["candidates"][0]["label"] == "table"


def test_label_aliases_aggregate_into_one_canonical_vote() -> None:
    evidence = _label_evidence(
        {
            "class_id": [0, 1, 0, 1],
            "conf": [0.8, 0.9, 0.85, 0.95],
            "class_name": "plant",
        },
        ["plant", "potted plant"],
        current_label="plant",
        history_size=20,
        min_switch_observations=3,
        min_winner_share=0.65,
        switch_margin=0.20,
        label_aliases={"plant": "potted plant"},
    )
    assert evidence["label"] == "potted plant"
    assert evidence["confidence"] == pytest.approx(1.0)
    assert evidence["provisional"] is False
    assert evidence["candidates"][0]["observations"] == 4


def test_configured_vocabulary_precedes_legacy_env(monkeypatch) -> None:
    monkeypatch.setenv("SCENE_OPEN_VOCAB_CLASSES", "legacy,override")
    assert _resolved_classes() == ["legacy", "override"]
    assert _resolved_classes(
        ["Chair", "chair", "Desk"],
        ["Mug", "desk"],
    ) == ["chair", "desk", "mug"]
    configured_addition = _resolved_classes(None, ["Mug"])
    assert "legacy" not in configured_addition
    assert "mug" in configured_addition


def test_default_vocabulary_covers_common_cross_environment_objects(
    monkeypatch,
) -> None:
    monkeypatch.delenv("SCENE_OPEN_VOCAB_CLASSES", raising=False)
    classes = set(_resolved_classes())
    assert {
        "monitor",
        "sink",
        "refrigerator",
        "washing machine",
        "workbench",
        "pallet",
        "safety helmet",
    } <= classes


def _detector(**kwargs) -> ConceptGraphsDetector:
    async def _noop(_):
        return None

    kwargs.setdefault("allow_cross_class_merge", False)
    classes = kwargs.pop("classes", ["chair", "table", "sofa", "couch"])
    return ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: None,
        depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None,
        on_detections=_noop,
        registry=ObjectRegistry(),
        world_frame_fn=lambda: "map",
        classes=classes,
        **kwargs,
    )


def test_cross_class_association_is_opt_in() -> None:
    strict = _detector()
    assert strict._association_compatible("chair", "chair")
    assert not strict._association_compatible("chair", "table")

    grouped = _detector(confusable_class_groups=[["sofa", "couch"]])
    assert grouped._association_compatible("sofa", "couch")
    assert not grouped._association_compatible("chair", "table")

    legacy = _detector(allow_cross_class_merge=True)
    assert legacy._association_compatible("chair", "table")

    aliased = _detector(
        classes=["plant", "potted plant"],
        label_aliases={"plant": "potted plant"},
    )
    assert aliased._association_compatible("plant", "potted plant")


def test_detection_admission_knobs_are_explicit() -> None:
    detector = _detector(
        confidence_threshold=0.22,
        max_detections=50,
    )
    assert detector._conf_thresh == pytest.approx(0.22)
    assert detector._max_dets == 50


def test_surface_snap_scope_is_explicit_and_vocabulary_checked() -> None:
    detector = _detector(
        classes=["window", "picture frame", "chair"],
        surface_snap_labels=["window", "picture frame"],
        surface_snap_max_distance_m=0.42,
        surface_snap_min_support_cells=17,
    )
    assert detector._surface_snap_labels == {"window", "picture frame"}
    assert detector._surface_snap_max_distance_m == pytest.approx(0.42)
    assert detector._surface_snap_min_support_cells == 17
    assert detector.quality_metrics()["surface_snap"]["labels"] == [
        "picture frame",
        "window",
    ]

    with pytest.raises(ValueError, match="outside the resolved vocabulary"):
        _detector(
            classes=["chair"],
            surface_snap_labels=["window"],
        )


def test_same_class_geometry_knobs_are_explicit() -> None:
    detector = _detector(
        same_class_centroid_max_m=0.11,
        same_class_min_voxel_coverage=0.61,
        same_class_max_extent_ratio=1.42,
        same_class_disjoint_min_unique_frames=3,
        same_class_disjoint_max_frame_gap=2,
        same_class_disjoint_max_center_major_extent_ratio=0.17,
        same_class_disjoint_min_visual_similarity=0.91,
        same_class_merge_interval_ticks=3,
    )
    assert detector.cfg["same_class_centroid_max_m"] == pytest.approx(0.11)
    assert detector.cfg["same_class_min_voxel_coverage"] == pytest.approx(0.61)
    assert detector.cfg["same_class_max_extent_ratio"] == pytest.approx(1.42)
    assert detector.cfg["same_class_disjoint_min_unique_frames"] == 3
    assert detector.cfg["same_class_disjoint_max_frame_gap"] == 2
    assert detector.cfg[
        "same_class_disjoint_max_center_major_extent_ratio"
    ] == pytest.approx(0.17)
    assert detector.cfg[
        "same_class_disjoint_min_visual_similarity"
    ] == pytest.approx(0.91)
    assert detector.cfg["same_class_merge_interval_ticks"] == 3


def test_clip_rerank_is_scoped_and_margin_gated() -> None:
    detector = _detector(
        classes=["window", "picture frame", "chair"],
        clip_rerank_groups=[["window", "picture frame"]],
        clip_rerank_min_score=0.20,
        clip_rerank_min_margin=0.10,
    )
    detector._clip_rerank_text_features = {
        "window": np.asarray([1.0, 0.0], dtype=np.float32),
        "picture frame": np.asarray([0.0, 1.0], dtype=np.float32),
    }

    label, scores = detector._clip_rerank_label(
        "picture frame",
        np.asarray([0.95, 0.15], dtype=np.float32),
    )
    assert label == "window"
    assert scores["window"] > scores["picture frame"]
    assert detector._quality_counters["clip_rerank_attempts"] == 1
    assert detector._quality_counters["clip_rerank_switches"] == 1
    assert detector.quality_metrics()["clip_rerank_recent"][-1]["switched"]

    label, _ = detector._clip_rerank_label(
        "picture frame",
        np.asarray([0.72, 0.69], dtype=np.float32),
    )
    assert label == "picture frame"
    label, scores = detector._clip_rerank_label(
        "chair",
        np.asarray([1.0, 0.0], dtype=np.float32),
    )
    assert label == "chair"
    assert scores == {}


def test_clip_rerank_zero_margin_still_requires_strict_winner() -> None:
    detector = _detector(
        classes=["window", "picture frame"],
        clip_rerank_groups=[["window", "picture frame"]],
        clip_rerank_min_score=0.20,
        clip_rerank_min_margin=0.0,
    )
    detector._clip_rerank_text_features = {
        "window": np.asarray([1.0, 0.0], dtype=np.float32),
        "picture frame": np.asarray([0.0, 1.0], dtype=np.float32),
    }
    label, scores = detector._clip_rerank_label(
        "picture frame",
        np.asarray([1.0, 1.0], dtype=np.float32),
    )
    assert scores["window"] == pytest.approx(scores["picture frame"])
    assert label == "picture frame"
    assert detector._quality_counters["clip_rerank_switches"] == 0
    quality = detector.quality_metrics()
    assert quality["clip_rerank_group_count"] == 1
    assert quality["clip_rerank_ready_label_count"] == 2
    assert quality["clip_rerank_min_score"] == pytest.approx(0.20)
    assert quality["clip_rerank_min_margin"] == pytest.approx(0.0)


def test_clip_rerank_group_margin_overrides_global_threshold() -> None:
    detector = _detector(
        classes=["window", "picture frame", "monitor", "television"],
        clip_rerank_groups=[
            ["window", "picture frame"],
            ["monitor", "television"],
        ],
        clip_rerank_min_score=0.20,
        clip_rerank_min_margin=0.10,
        clip_rerank_min_margin_by_label={
            "window": 0.01,
            "picture frame": 0.01,
        },
    )
    detector._clip_rerank_text_features = {
        "window": np.asarray([1.0, 0.0], dtype=np.float32),
        "picture frame": np.asarray([0.0, 1.0], dtype=np.float32),
        "monitor": np.asarray([1.0, 0.0], dtype=np.float32),
        "television": np.asarray([0.0, 1.0], dtype=np.float32),
    }
    feature = np.asarray([0.73, 0.68], dtype=np.float32)

    label, _ = detector._clip_rerank_label("picture frame", feature)
    assert label == "window"
    assert detector.quality_metrics()["clip_rerank_recent"][-1][
        "min_margin"
    ] == pytest.approx(0.01)

    label, _ = detector._clip_rerank_label("television", feature)
    assert label == "television"
    quality = detector.quality_metrics()
    assert quality["clip_rerank_recent"][-1]["min_margin"] == pytest.approx(
        0.10
    )
    assert quality["clip_rerank_min_margin_by_label"] == {
        "window": pytest.approx(0.01),
        "picture frame": pytest.approx(0.01),
    }


def test_clip_rerank_route_is_source_specific() -> None:
    detector = _detector(
        classes=["chair", "monitor"],
        clip_rerank_routes={"chair": ["chair", "monitor"]},
        clip_rerank_min_score=0.20,
        clip_rerank_min_margin=0.02,
    )
    detector._clip_rerank_text_features = {
        "chair": np.asarray([0.0, 1.0], dtype=np.float32),
        "monitor": np.asarray([1.0, 0.0], dtype=np.float32),
    }
    feature = np.asarray([0.75, 0.65], dtype=np.float32)

    label, scores = detector._clip_rerank_label("chair", feature)
    assert label == "monitor"
    assert set(scores) == {"chair", "monitor"}

    label, scores = detector._clip_rerank_label("monitor", feature)
    assert label == "monitor"
    assert scores == {}
    quality = detector.quality_metrics()
    assert quality["clip_rerank_group_count"] == 0
    assert quality["clip_rerank_route_count"] == 1


def test_persistent_clip_rerank_uses_stable_multi_view_feature() -> None:
    detector = _detector(
        classes=["cup", "can"],
        label_min_switch_observations=2,
        clip_rerank_routes={"cup": ["cup", "can"]},
        clip_rerank_min_score=0.20,
        clip_rerank_min_margin=0.02,
    )
    detector._clip_rerank_text_features = {
        "cup": np.asarray([0.0, 1.0], dtype=np.float32),
        "can": np.asarray([1.0, 0.0], dtype=np.float32),
    }
    points = np.asarray(
        [
            [-0.1, -0.1, 0.0],
            [0.1, -0.1, 0.0],
            [-0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0],
            [-0.1, -0.1, 0.2],
            [0.1, -0.1, 0.2],
            [-0.1, 0.1, 0.2],
            [0.1, 0.1, 0.2],
        ],
        dtype=np.float64,
    )
    detector._map_objects = [
        {
            "id": "u1",
            "class_name": "cup",
            "class_id": [0, 0],
            "conf": [0.9, 0.8],
            "clip_ft": np.asarray([0.8, 0.6], dtype=np.float32),
            "pcd": SimpleNamespace(
                points=points,
                colors=np.zeros_like(points),
            ),
            "bbox": object(),
        }
    ]
    original_history = list(detector._map_objects[0]["class_id"])

    detector._stabilize_map_labels()
    detector._stabilize_map_labels()

    obj = detector._map_objects[0]
    assert obj["class_name"] == "cup"
    assert obj["resolved_class_name"] == "can"
    assert obj["label_source"] == "model_clip"
    assert obj["label_provisional"] is False
    assert obj["label_evidence_count"] == 2
    assert obj["class_id"] == original_history
    assert detector._quality_counters["clip_rerank_attempts"] == 0
    assert detector.quality_metrics()["clip_rerank_persistent_objects"] == 1
    # Normal /objects3d output is the confirmed registry view; raw candidates
    # remain available only through the explicit debug snapshot.
    detector._uuid_to_oid = {"u1": "scene.object.can_001"}
    assert detector.export_3d_snapshot()["objects"][0]["cls"] == "can"


def test_persistent_clip_rerank_waits_for_stable_detector_evidence() -> None:
    detector = _detector(
        classes=["cup", "can"],
        label_min_switch_observations=2,
        clip_rerank_routes={"cup": ["cup", "can"]},
        clip_rerank_min_score=0.20,
        clip_rerank_min_margin=0.02,
    )
    detector._clip_rerank_text_features = {
        "cup": np.asarray([0.0, 1.0], dtype=np.float32),
        "can": np.asarray([1.0, 0.0], dtype=np.float32),
    }
    detector._map_objects = [
        {
            "id": "u1",
            "class_name": "cup",
            "class_id": [0],
            "conf": [0.9],
            "clip_ft": np.asarray([0.8, 0.6], dtype=np.float32),
        }
    ]

    detector._stabilize_map_labels()

    obj = detector._map_objects[0]
    assert obj["class_name"] == "cup"
    assert obj["label_source"] == "model"
    assert obj["label_provisional"] is True


def test_persistent_geometry_can_disambiguate_a_provisional_label() -> None:
    detector = _detector(
        classes=["monitor", "television"],
        label_min_switch_observations=2,
        clip_rerank_groups=[["monitor", "television"]],
        clip_rerank_min_score=0.20,
        clip_rerank_min_margin=0.0,
        clip_rerank_geometry_bonus=0.06,
        clip_rerank_geometry_constraints={
            "monitor": {
                "source_labels": ["monitor", "television"],
                "max_horizontal_extent_m": 0.68,
            },
        },
    )
    detector._clip_rerank_text_features = {
        "monitor": np.asarray([1.0, 0.0], dtype=np.float32),
        "television": np.asarray([0.0, 1.0], dtype=np.float32),
    }
    feature = np.asarray([0.69, 0.72], dtype=np.float32)
    points = np.asarray(
        [
            [-0.20, -0.10, 0.0],
            [0.20, -0.10, 0.0],
            [-0.20, 0.10, 0.0],
            [0.20, 0.10, 0.0],
            [-0.20, -0.10, 0.30],
            [0.20, -0.10, 0.30],
            [-0.20, 0.10, 0.30],
            [0.20, 0.10, 0.30],
        ],
        dtype=np.float64,
    )
    detector._map_objects = [
        {
            "id": "u1",
            "class_name": "television",
            "class_id": [1],
            "conf": [0.9],
            "clip_ft": feature,
            "pcd": SimpleNamespace(points=points),
        }
    ]

    detection_label, _ = detector._clip_rerank_label(
        "television",
        feature,
    )
    assert detection_label == "television"

    detector._stabilize_map_labels()

    obj = detector._map_objects[0]
    assert obj["class_name"] == "television"
    assert obj["resolved_class_name"] == "monitor"
    assert obj["label_source"] == "model_clip"
    assert obj["label_provisional"] is True
    assert detector.quality_metrics()["clip_rerank_persistent_geometry"] == {
        "score_bonus": pytest.approx(0.06),
        "labels": {
            "monitor": {
                "source_labels": ["monitor", "television"],
                "max_horizontal_extent_m": pytest.approx(0.68),
            },
        },
    }


def test_persistent_geometry_does_not_force_an_out_of_range_candidate() -> None:
    detector = _detector(
        classes=["monitor", "television"],
        label_min_switch_observations=2,
        clip_rerank_groups=[["monitor", "television"]],
        clip_rerank_min_score=0.20,
        clip_rerank_min_margin=0.0,
        clip_rerank_geometry_bonus=0.06,
        clip_rerank_geometry_constraints={
            "monitor": {
                "source_labels": ["monitor", "television"],
                "max_horizontal_extent_m": 0.68,
            },
        },
    )
    detector._clip_rerank_text_features = {
        "monitor": np.asarray([1.0, 0.0], dtype=np.float32),
        "television": np.asarray([0.0, 1.0], dtype=np.float32),
    }
    points = np.asarray(
        [
            [-0.50, -0.15, 0.0],
            [0.50, -0.15, 0.0],
            [-0.50, 0.15, 0.0],
            [0.50, 0.15, 0.0],
            [-0.50, -0.15, 0.60],
            [0.50, -0.15, 0.60],
            [-0.50, 0.15, 0.60],
            [0.50, 0.15, 0.60],
        ],
        dtype=np.float64,
    )
    detector._map_objects = [
        {
            "id": "u1",
            "class_name": "television",
            "class_id": [1],
            "conf": [0.9],
            "clip_ft": np.asarray([0.69, 0.72], dtype=np.float32),
            "pcd": SimpleNamespace(points=points),
        }
    ]

    detector._stabilize_map_labels()

    obj = detector._map_objects[0]
    assert obj["class_name"] == "television"
    assert obj["resolved_class_name"] == "television"
    assert obj["label_source"] == "model"
    assert obj["label_provisional"] is True


def test_persistent_geometry_source_scope_excludes_an_unrelated_route() -> None:
    detector = _detector(
        classes=["chair", "monitor", "television"],
        clip_rerank_groups=[["monitor", "television"]],
        clip_rerank_routes={"chair": ["chair", "monitor"]},
        clip_rerank_geometry_bonus=0.06,
        clip_rerank_geometry_constraints={
            "monitor": {
                "source_labels": ["monitor", "television"],
                "max_horizontal_extent_m": 0.68,
            },
        },
    )

    assert detector._clip_rerank_geometry_adjustments(
        "television",
        {"horizontal_extent_m": 0.40},
    ) == {"monitor": pytest.approx(0.06)}
    assert detector._clip_rerank_geometry_adjustments(
        "chair",
        {"horizontal_extent_m": 0.40},
    ) == {}


def test_clip_rerank_rejects_unknown_or_overlapping_groups() -> None:
    with pytest.raises(ValueError, match="outside the resolved vocabulary"):
        _detector(
            classes=["window", "picture frame"],
            clip_rerank_groups=[["window", "painting"]],
        )
    with pytest.raises(ValueError, match="only one group"):
        _detector(
            classes=["window", "picture frame", "monitor"],
            clip_rerank_groups=[
                ["window", "picture frame"],
                ["window", "monitor"],
            ],
        )
    with pytest.raises(
        ValueError,
        match="outside configured groups or route sources",
    ):
        _detector(
            classes=["window", "picture frame", "chair"],
            clip_rerank_groups=[["window", "picture frame"]],
            clip_rerank_min_margin_by_label={"chair": 0.01},
        )
    with pytest.raises(ValueError, match="contain its source"):
        _detector(
            classes=["chair", "monitor", "television"],
            clip_rerank_routes={"chair": ["monitor", "television"]},
        )
    with pytest.raises(ValueError, match="must not also belong"):
        _detector(
            classes=["chair", "monitor", "television"],
            clip_rerank_groups=[["monitor", "television"]],
            clip_rerank_routes={"monitor": ["monitor", "chair"]},
        )
    with pytest.raises(ValueError, match="outside configured groups or routes"):
        _detector(
            classes=["monitor", "television", "chair"],
            clip_rerank_groups=[["monitor", "television"]],
            clip_rerank_geometry_constraints={
                "chair": {"max_horizontal_extent_m": 0.68},
            },
        )
    with pytest.raises(ValueError, match="minimum must not exceed maximum"):
        _detector(
            classes=["monitor", "television"],
            clip_rerank_groups=[["monitor", "television"]],
            clip_rerank_geometry_constraints={
                "monitor": {
                    "min_horizontal_extent_m": 0.80,
                    "max_horizontal_extent_m": 0.68,
                },
            },
        )
    with pytest.raises(ValueError, match="cannot reach candidate"):
        _detector(
            classes=["monitor", "television", "chair"],
            clip_rerank_groups=[["monitor", "television"]],
            clip_rerank_geometry_constraints={
                "monitor": {
                    "source_labels": ["chair"],
                    "max_horizontal_extent_m": 0.68,
                },
            },
        )


def test_3d_snapshot_clip_feature_is_debug_only_and_normalized() -> None:
    detector = ConceptGraphsDetector.__new__(ConceptGraphsDetector)
    points = np.asarray(
        [
            [-0.2, -0.1, 0.0],
            [0.2, -0.1, 0.0],
            [-0.2, 0.1, 0.0],
            [0.2, 0.1, 0.0],
            [-0.2, -0.1, 0.4],
            [0.2, -0.1, 0.4],
            [-0.2, 0.1, 0.4],
            [0.2, 0.1, 0.4],
        ],
        dtype=np.float64,
    )
    detector._map_objects = [
        {
            "id": "map-object-1",
            "class_name": "chair",
            "pcd": SimpleNamespace(
                points=points,
                colors=np.zeros_like(points),
            ),
            "bbox": object(),
            "clip_ft": np.asarray([3.0, 4.0], dtype=np.float32),
            "class_id": [0, 0, 0],
            "conf": [0.8, 0.9, 0.85],
            "image_idx": [11, 12, 13],
            "mask_idx": [1, 2, 3],
            "xyxy": [
                [0.0, 0.0, 10.0, 10.0],
                [1.0, 1.0, 11.0, 11.0],
                [2.0, 2.0, 12.0, 12.0],
            ],
            # Simulate the short unlocked cleanup window: derived fields have
            # been dropped/staled, while class/conf histories remain the
            # authoritative evidence.
            "label_evidence_count": 0,
            "label_candidates": [],
            "label_provisional": True,
        }
    ]
    detector._uuid_to_oid = {"map-object-1": "scene.object.chair_001"}
    detector._missing_uuids = set()
    detector._operator_geometry_oids = set()
    detector._bbox_low_percentile = 0.0
    detector._bbox_high_percentile = 100.0
    detector._classes = ["chair", "table"]
    detector._label_history_size = 20
    detector._label_min_switch_observations = 3
    detector._label_min_winner_share = 0.65
    detector._label_switch_margin = 0.20
    detector._label_aliases = {}

    normal = detector.export_3d_snapshot()
    assert "clip_feature" not in normal["objects"][0]
    assert "registry_id" not in normal["objects"][0]
    assert normal["objects"][0]["label_evidence_count"] == 3
    assert normal["objects"][0]["label_provisional"] is False
    assert normal["objects"][0]["label_candidates"][0]["label"] == "chair"

    debug = detector.export_3d_snapshot(include_clip_feature=True)
    assert debug["debug_clip_features"] is True
    assert debug["objects"][0]["registry_id"] == "scene.object.chair_001"
    assert debug["objects"][0]["clip_feature"] == pytest.approx([0.6, 0.8])
    assert debug["objects"][0]["image_indices"] == [11, 12, 13]
    assert debug["objects"][0]["observation_history"][0] == {
        "frame": 11,
        "bbox_xyxy": [0.0, 0.0, 10.0, 10.0],
        "mask_index": 1,
        "label": "chair",
        "confidence": 0.8,
    }
    assert debug["objects"][0]["class_history"] == [
        {"frame": 11, "label": "chair", "confidence": 0.8},
        {"frame": 12, "label": "chair", "confidence": 0.9},
        {"frame": 13, "label": "chair", "confidence": 0.85},
    ]


def _cloud(points):
    return {"pcd": SimpleNamespace(points=np.asarray(points, dtype=float))}


def _map_object(object_id, label, points, *, operator_label=""):
    obj = {
        "id": object_id,
        "class_name": label,
        "class_id": [0],
        "conf": [0.9],
        "pcd": SimpleNamespace(points=np.asarray(points, dtype=float)),
    }
    if operator_label:
        obj["operator_label"] = operator_label
    return obj


def _install_geometry_merge_stub(detector):
    class MapObjectList(list):
        pass

    merged_pairs = []

    def merge_obj2_into_obj1(*, obj1, obj2, **_kwargs):
        merged_pairs.append((obj1["id"], obj2["id"]))
        obj1["pcd"] = SimpleNamespace(
            points=np.vstack((obj1["pcd"].points, obj2["pcd"].points))
        )
        obj1["class_id"] = list(obj1.get("class_id", ())) + list(
            obj2.get("class_id", ())
        )
        obj1["conf"] = list(obj1.get("conf", ())) + list(
            obj2.get("conf", ())
        )
        return obj1

    detector._cg = {
        "MapObjectList": MapObjectList,
        "merge_obj2_into_obj1": merge_obj2_into_obj1,
    }
    return merged_pairs


def test_exact_duplicate_geometry_allows_near_identical_clouds_only() -> None:
    detector = _detector(
        exact_duplicate_centroid_max_m=0.08,
        exact_duplicate_min_voxel_coverage=0.85,
        exact_duplicate_max_extent_ratio=1.25,
    )
    xs, ys, zs = np.meshgrid(
        np.linspace(-0.30, 0.30, 13),
        np.linspace(-0.15, 0.15, 7),
        np.linspace(0.0, 0.60, 13),
        indexing="ij",
    )
    cabinet = np.column_stack((xs.ravel(), ys.ravel(), zs.ravel()))
    # Sub-voxel sensor noise keeps the physical voxel support identical.
    shelf = cabinet + np.array([0.003, -0.002, 0.001])
    duplicate = detector._exact_duplicate_geometry_matrix(
        [_cloud(cabinet)],
        [_cloud(shelf)],
    )
    assert duplicate.tolist() == [[True]]

    # A small object on the cabinet shares space but not the full support or
    # robust extents. It must remain a distinct ConceptGraphs object.
    plant = cabinet[
        (np.abs(cabinet[:, 0]) <= 0.10)
        & (np.abs(cabinet[:, 1]) <= 0.05)
        & (cabinet[:, 2] >= 0.30)
    ]
    contained = detector._exact_duplicate_geometry_matrix(
        [_cloud(cabinet)],
        [_cloud(plant)],
    )
    assert contained.tolist() == [[False]]


def test_default_duplicate_geometry_accepts_partial_multiview_support() -> None:
    detector = _detector()
    xs, ys, zs = np.meshgrid(
        np.linspace(-0.30, 0.30, 25),
        np.linspace(-0.15, 0.15, 7),
        np.linspace(0.0, 0.60, 13),
        indexing="ij",
    )
    fixture = np.column_stack((xs.ravel(), ys.ravel(), zs.ravel()))
    # Two partial views cover opposite sides of one object. Their robust
    # centers differ by about 12 cm and their bidirectional one-voxel support
    # is materially below the former 0.85 gate, while extents remain
    # equivalent. This is representative of the five-world Webots failures.
    left_view = fixture[fixture[:, 0] <= 0.175]
    right_view = (
        fixture[fixture[:, 0] >= -0.175]
        + np.array([0.008, -0.004, 0.003])
    )
    duplicate, coverage = detector._exact_duplicate_geometry_matrices(
        [_cloud(left_view)],
        [_cloud(right_view)],
    )
    assert 0.40 <= coverage[0, 0] < 0.85
    assert duplicate.tolist() == [[True]]


def test_exact_duplicate_geometry_rejects_same_shape_at_new_position() -> None:
    detector = _detector()
    points = np.array(
        [
            [x, y, z]
            for x in (-0.2, 0.0, 0.2)
            for y in (-0.1, 0.0, 0.1)
            for z in (0.0, 0.3, 0.6)
        ],
        dtype=float,
    )
    matrix = detector._exact_duplicate_geometry_matrix(
        [_cloud(points)],
        [_cloud(points + np.array([0.5, 0.0, 0.0]))],
    )
    assert matrix.tolist() == [[False]]


def test_merge_gate_diagnostics_explain_cross_class_rejection() -> None:
    detector = _detector()
    points = np.array(
        [
            [x, y, z]
            for x in (-0.1, 0.0, 0.1)
            for y in (-0.1, 0.0, 0.1)
            for z in (0.0, 0.2, 0.4)
        ],
        dtype=float,
    )
    left = _cloud(points)
    left.update(
        {
            "id": "left-uuid",
            "class_name": "chair",
            "clip_ft": np.array([1.0, 0.0]),
            "image_idx": [10, 11],
            "xyxy": [[0.0, 0.0, 4.0, 4.0], [1.0, 1.0, 9.0, 9.0]],
        }
    )
    right = _cloud(points + np.array([0.25, 0.0, 0.0]))
    right.update(
        {
            "id": "right-uuid",
            "class_name": "table",
            "clip_ft": np.array([1.0, 0.0]),
            "image_idx": [11, 12],
            "xyxy": [[1.0, 1.0, 9.0, 9.0], [2.0, 2.0, 8.0, 8.0]],
        }
    )

    exact, coverage = detector._exact_duplicate_geometry_matrices(
        [left, right]
    )
    detector._uuid_to_oid.update(
        {
            "left-uuid": "scene.object.chair_001",
            "right-uuid": "scene.object.table_001",
        }
    )
    coobserved_duplicates = detector._record_merge_gate_diagnostics(
        [left, right],
        overlap_matrix=np.array([[0.0, 0.1], [0.1, 0.0]]),
        exact_duplicates=exact,
        tolerant_coverage=coverage,
    )

    diagnostics = detector.quality_metrics()["merge_gate_diagnostics"]
    assert diagnostics["nearby_pairs"] == 1
    assert diagnostics["canonical_eligible_pairs"] == 0
    assert diagnostics["coobserved_pairs"] == 1
    assert diagnostics["disjoint_frame_history_pairs"] == 0
    assert diagnostics["coobserved_high_2d_overlap_pairs"] == 1
    assert diagnostics["coobserved_duplicate_pairs"] == 0
    assert not coobserved_duplicates.any()
    pair = diagnostics["candidate_pairs"][0]
    assert pair["left"] == "chair"
    assert pair["right"] == "table"
    assert pair["left_uuid"] == "left-uuid"
    assert pair["right_uuid"] == "right-uuid"
    assert pair["left_registry_id"] == "scene.object.chair_001"
    assert pair["right_registry_id"] == "scene.object.table_001"
    assert "duplicate_centroid" in pair["rejected_by"]
    assert pair["tolerant_voxel_coverage_evaluated"] is False
    assert "duplicate_coverage" not in pair["rejected_by"]
    assert "spatial_overlap" in pair["rejected_by"]
    assert "visual_similarity" not in pair["rejected_by"]
    assert pair["coobserved"] is True
    assert pair["coobserved_duplicate"] is False
    assert pair["shared_frame_count"] == 1
    assert pair["max_shared_frame_2d_iou"] == pytest.approx(1.0)
    assert pair["median_shared_frame_2d_iou"] == pytest.approx(1.0)
    assert pair["shared_frame_2d_iou_evidence"] == [
        {"frame": 11, "iou": 1.0}
    ]


def test_repeated_coobservation_admits_cross_label_duplicate_evidence() -> None:
    detector = _detector(
        coobserved_duplicate_min_shared_frames=3,
        coobserved_duplicate_min_median_iou=0.85,
        coobserved_duplicate_max_extent_ratio=2.0,
        coobserved_duplicate_min_visual_similarity=0.90,
    )
    points = np.asarray(
        [
            [x, y, z]
            for x in (-0.1, 0.0, 0.1)
            for y in (-0.1, 0.0, 0.1)
            for z in (0.0, 0.1, 0.2)
        ],
        dtype=float,
    )
    left = _cloud(points)
    left.update(
        {
            "id": "bowl-track",
            "class_name": "bowl",
            "clip_ft": np.array([1.0, 0.0]),
            "image_idx": [10, 11, 12],
            "xyxy": [[10.0, 10.0, 30.0, 30.0]] * 3,
        }
    )
    right = _cloud(points + np.array([0.09, 0.0, 0.0]))
    right.update(
        {
            "id": "cereal-track",
            "class_name": "cereal box",
            "clip_ft": np.array([0.95, 0.05]),
            "image_idx": [10, 11, 12],
            "xyxy": [[10.2, 10.1, 30.1, 30.2]] * 3,
        }
    )
    detector._uuid_to_oid.update(
        {
            "bowl-track": "scene.object.bowl_001",
            "cereal-track": "scene.object.cereal_box_003",
        }
    )

    matrix = detector._record_merge_gate_diagnostics(
        [left, right],
        overlap_matrix=np.array([[0.0, 0.69], [0.69, 0.0]]),
        exact_duplicates=np.zeros((2, 2), dtype=bool),
        tolerant_coverage=np.zeros((2, 2), dtype=float),
    )

    assert matrix.tolist() == [[False, True], [True, False]]
    diagnostics = detector.quality_metrics()["merge_gate_diagnostics"]
    assert diagnostics["coobserved_duplicate_pairs"] == 1
    pair = diagnostics["candidate_pairs"][0]
    assert pair["coobserved_duplicate"] is True
    assert pair["canonical_eligible"] is True
    assert pair["shared_frame_count"] == 3
    assert pair["median_shared_frame_2d_iou"] > 0.95


def test_same_class_cleanup_merges_partial_views_with_geometry_evidence() -> None:
    detector = _detector()
    merged_pairs = _install_geometry_merge_stub(detector)
    xs, ys, zs = np.meshgrid(
        np.linspace(-0.30, 0.30, 25),
        np.linspace(-0.15, 0.15, 7),
        np.linspace(0.0, 0.60, 13),
        indexing="ij",
    )
    fixture = np.column_stack((xs.ravel(), ys.ravel(), zs.ravel()))
    left_view = fixture[fixture[:, 0] <= 0.175]
    right_view = (
        fixture[fixture[:, 0] >= -0.175]
        + np.array([0.008, -0.004, 0.003])
    )
    objects = [
        _map_object("chair-left", "chair", left_view),
        _map_object("chair-right", "chair", right_view),
    ]

    result = detector._same_class_proximity_collapse(objects)

    assert len(result) == 1
    assert merged_pairs == [("chair-left", "chair-right")]


def test_same_class_cleanup_rejects_nearby_or_contained_geometry() -> None:
    detector = _detector()
    merged_pairs = _install_geometry_merge_stub(detector)
    xs, ys, zs = np.meshgrid(
        np.linspace(-0.30, 0.30, 13),
        np.linspace(-0.15, 0.15, 7),
        np.linspace(0.0, 0.60, 13),
        indexing="ij",
    )
    fixture = np.column_stack((xs.ravel(), ys.ravel(), zs.ravel()))
    contained = fixture[
        (np.abs(fixture[:, 0]) <= 0.10)
        & (np.abs(fixture[:, 1]) <= 0.05)
        & (fixture[:, 2] >= 0.30)
    ]
    compact = np.array(
        [
            [x, y, z]
            for x in (-0.04, 0.0, 0.04)
            for y in (-0.04, 0.0, 0.04)
            for z in (0.0, 0.08, 0.16)
        ],
        dtype=float,
    )
    nearby = compact + np.array([0.12, 0.0, 0.0])
    objects = [
        _map_object("large-chair", "chair", fixture),
        _map_object("contained-chair", "chair", contained),
        _map_object("near-chair-a", "chair", compact),
        _map_object("near-chair-b", "chair", nearby),
    ]

    result = detector._same_class_proximity_collapse(objects)

    assert len(result) == 4
    assert merged_pairs == []


def test_same_class_cleanup_merges_adjacent_frame_scaled_fragments() -> None:
    detector = _detector(
        same_class_centroid_max_m=0.15,
        same_class_max_extent_ratio=1.75,
        same_class_disjoint_min_unique_frames=2,
        same_class_disjoint_max_frame_gap=1,
        same_class_disjoint_max_center_major_extent_ratio=0.20,
        same_class_disjoint_min_visual_similarity=0.85,
    )
    merged_pairs = _install_geometry_merge_stub(detector)
    xs, ys, zs = np.meshgrid(
        np.linspace(-0.40, 0.40, 17),
        np.linspace(-0.01, 0.01, 3),
        np.linspace(0.0, 0.30, 7),
        indexing="ij",
    )
    left_points = np.column_stack((xs.ravel(), ys.ravel(), zs.ravel()))
    right_points = left_points + np.array([0.0, 0.10, 0.0])
    left = _map_object("left-monitor-view", "monitor", left_points)
    left.update(
        {
            "image_idx": [10, 11],
            "clip_ft": np.array([1.0, 0.0]),
        }
    )
    right = _map_object("right-monitor-view", "monitor", right_points)
    right.update(
        {
            "image_idx": [12, 13],
            "clip_ft": np.array([0.98, 0.02]),
        }
    )
    strict = detector._exact_duplicate_geometry_matrix(
        [left],
        [right],
        centroid_max_m=detector.cfg["same_class_centroid_max_m"],
        min_voxel_coverage=detector.cfg[
            "same_class_min_voxel_coverage"
        ],
        max_extent_ratio=detector.cfg[
            "same_class_max_extent_ratio"
        ],
    )
    assert strict.tolist() == [[False]]
    evidence = detector._same_class_disjoint_identity_evidence(left, right)
    assert evidence is not None
    assert evidence["minimum_frame_gap"] == 1
    assert evidence["center_distance_m"] == pytest.approx(0.10)
    assert evidence["center_major_extent_ratio"] < 0.20

    result = detector._same_class_proximity_collapse([left, right])

    assert len(result) == 1
    assert merged_pairs == [("left-monitor-view", "right-monitor-view")]
    quality = detector.quality_metrics()
    assert quality["same_class_disjoint_candidate_pairs"] == 1
    assert quality["same_class_disjoint_merged_pairs"] == 1
    recent = quality["association"]["same_class_disjoint"][
        "recent_merges"
    ]
    assert recent[-1]["left_uuid"] == "left-monitor-view"
    assert recent[-1]["right_uuid"] == "right-monitor-view"


def test_same_class_disjoint_fallback_rejects_small_or_coobserved_objects() -> None:
    detector = _detector()
    compact = np.array(
        [
            [x, y, z]
            for x in (-0.05, 0.0, 0.05)
            for y in (-0.01, 0.0, 0.01)
            for z in (0.0, 0.05, 0.10)
        ],
        dtype=float,
    )
    small_left = _map_object("small-left", "can", compact)
    small_left.update(
        {
            "image_idx": [10, 11],
            "clip_ft": np.array([1.0, 0.0]),
        }
    )
    small_right = _map_object(
        "small-right",
        "can",
        compact + np.array([0.10, 0.0, 0.0]),
    )
    small_right.update(
        {
            "image_idx": [12, 13],
            "clip_ft": np.array([1.0, 0.0]),
        }
    )
    assert (
        detector._same_class_disjoint_identity_evidence(
            small_left,
            small_right,
        )
        is None
    )

    large_right = _map_object(
        "large-right",
        "monitor",
        np.tile(compact, (8, 1)),
    )
    large_right.update(
        {
            "image_idx": [11, 12],
            "clip_ft": np.array([1.0, 0.0]),
        }
    )
    large_left = _map_object(
        "large-left",
        "monitor",
        np.tile(compact, (8, 1)),
    )
    large_left.update(
        {
            "image_idx": [10, 11],
            "clip_ft": np.array([1.0, 0.0]),
        }
    )
    assert (
        detector._same_class_disjoint_identity_evidence(
            large_left,
            large_right,
        )
        is None
    )


def test_same_class_cleanup_runs_on_every_completed_tick() -> None:
    detector = _detector()
    merged_pairs = _install_geometry_merge_stub(detector)
    detector._tick_idx = 1
    detector.cfg["denoise_interval_ticks"] = 97
    detector.cfg["merge_overlap_interval_ticks"] = 97
    detector.cfg["cross_class_merge_interval_ticks"] = 97
    assert detector.cfg["same_class_merge_interval_ticks"] == 1
    points = np.array(
        [
            [x, y, z]
            for x in (-0.10, -0.05, 0.0, 0.05, 0.10)
            for y in (-0.05, 0.0, 0.05)
            for z in (0.0, 0.05, 0.10)
        ],
        dtype=float,
    )
    detector._map_objects = [
        _map_object("last-frame-a", "chair", points),
        _map_object(
            "last-frame-b",
            "chair",
            points + np.array([0.004, -0.003, 0.002]),
        ),
    ]
    detector._stabilize_map_labels = lambda: None
    projected = []
    detector._project_to_registry = lambda **kwargs: projected.append(kwargs)

    detector._maybe_periodic_cleanup()

    assert len(detector._map_objects) == 1
    assert merged_pairs == [("last-frame-a", "last-frame-b")]
    assert projected == [
        {
            "observed_uuids": set(),
            "visible_miss_uuids": set(),
        }
    ]


def test_periodic_merge_fuses_exact_duplicate_with_robonix_metadata() -> None:
    detector = _detector()
    detector._tick_idx = 1
    detector.cfg["denoise_interval_ticks"] = 97
    detector.cfg["merge_overlap_interval_ticks"] = 1
    detector.cfg["same_class_merge_interval_ticks"] = 97
    detector.cfg["cross_class_merge_interval_ticks"] = 97
    cabinet = {
        "id": "cg-1",
        "class_name": "chair",
        "clip_ft": np.array([1.0, 0.0]),
        "class_id": [0, 0, 0],
        "conf": [0.9, 0.8, 0.85],
        "label_confidence": 0.9,
        "label_provisional": False,
        "label_source": "model",
        "label_evidence_count": 3,
        "label_candidates": [{"label": "chair", "share": 1.0}],
    }
    shelf = {
        "id": "cg-2",
        "class_name": "table",
        "clip_ft": np.array([1.0, 0.0]),
        "class_id": [1, 1, 1],
        "conf": [0.7, 0.8, 0.75],
        "label_confidence": 0.8,
        "label_provisional": False,
        "label_source": "model",
        "label_evidence_count": 3,
        "label_candidates": [{"label": "table", "share": 1.0}],
    }
    detector._map_objects = [cabinet, shelf]
    detector._voxel_pcd_overlap_matrix = lambda *args, **kwargs: np.array(
        [[0.0, 0.10], [0.10, 0.0]],
        dtype=float,
    )
    detector._exact_duplicate_geometry_matrices = lambda *args, **kwargs: (
        np.array([[False, True], [True, False]], dtype=bool),
        np.array([[0.0, 0.45], [0.45, 0.0]], dtype=float),
    )
    detector._project_to_registry = lambda **_kwargs: None
    called = False

    def merge_overlap_objects(**kwargs):
        nonlocal called
        called = True
        objects = kwargs["objects"]
        # The geometry gate makes a tolerant cross-view pair visible to the
        # canonical ConceptGraphs visual merge even though its exact-voxel
        # overlap (0.10) is below the ordinary 0.50 spatial threshold.
        assert kwargs["overlap_matrix"][0, 1] > detector.cfg[
            "merge_overlap_thresh"
        ]
        for current in objects:
            for key in (
                "label_confidence",
                "label_provisional",
                "label_source",
                "label_evidence_count",
                "label_candidates",
            ):
                assert key not in current
        # Model the canonical ConceptGraphs merge: observation histories are
        # extended and only one physical map object remains.
        objects[0]["class_id"].extend(objects[1]["class_id"])
        objects[0]["conf"].extend(objects[1]["conf"])
        return [objects[0]], [0, None]

    detector._cg = {"merge_overlap_objects": merge_overlap_objects}
    detector._maybe_periodic_cleanup()
    assert called is True
    assert len(detector._map_objects) == 1
    merged = detector._map_objects[0]
    assert merged["label_source"] == "model"
    # Conflicting 3-vs-3 histories describe one physical object with an
    # unresolved label, rather than two coincident "certain" objects.
    assert merged["label_provisional"] is True
    assert merged["label_evidence_count"] == 6
    assert merged["label_candidates"][0]["label"] in {"chair", "table"}
    assert len(merged["label_candidates"]) == 2


def test_periodic_merge_exposes_repeated_coobserved_duplicate_to_conceptgraphs() -> None:
    detector = _detector()
    detector._tick_idx = 1
    detector.cfg["denoise_interval_ticks"] = 97
    detector.cfg["merge_overlap_interval_ticks"] = 1
    detector.cfg["same_class_merge_interval_ticks"] = 97
    detector.cfg["cross_class_merge_interval_ticks"] = 97
    left = {
        "id": "cg-left",
        "class_name": "bowl",
        "clip_ft": np.array([1.0, 0.0]),
        "class_id": [0, 0, 0],
        "conf": [0.9, 0.8, 0.85],
    }
    right = {
        "id": "cg-right",
        "class_name": "cereal box",
        "clip_ft": np.array([1.0, 0.0]),
        "class_id": [1, 1, 1],
        "conf": [0.7, 0.8, 0.75],
    }
    detector._map_objects = [left, right]
    detector._voxel_pcd_overlap_matrix = lambda *args, **kwargs: np.array(
        [[0.0, 0.10], [0.10, 0.0]],
        dtype=float,
    )
    detector._exact_duplicate_geometry_matrices = lambda *args, **kwargs: (
        np.zeros((2, 2), dtype=bool),
        np.zeros((2, 2), dtype=float),
    )
    detector._record_merge_gate_diagnostics = lambda *args, **kwargs: np.array(
        [[False, True], [True, False]],
        dtype=bool,
    )
    detector._project_to_registry = lambda **_kwargs: None

    def merge_overlap_objects(**kwargs):
        assert kwargs["overlap_matrix"][0, 1] > detector.cfg[
            "merge_overlap_thresh"
        ]
        objects = kwargs["objects"]
        objects[0]["class_id"].extend(objects[1]["class_id"])
        objects[0]["conf"].extend(objects[1]["conf"])
        return [objects[0]], [0, None]

    detector._cg = {"merge_overlap_objects": merge_overlap_objects}
    detector._maybe_periodic_cleanup()

    assert len(detector._map_objects) == 1
    assert detector._map_objects[0]["label_evidence_count"] == 6


def test_periodic_cleanup_exposes_only_disjoint_pairs_to_conceptgraphs() -> None:
    detector = _detector()
    detector._tick_idx = 1
    detector.cfg["denoise_interval_ticks"] = 97
    detector.cfg["merge_overlap_interval_ticks"] = 1
    detector.cfg["same_class_merge_interval_ticks"] = 97
    detector.cfg["cross_class_merge_interval_ticks"] = 97
    detector._map_objects = [
        {
            "id": f"cg-{index}",
            "class_name": "chair",
            "clip_ft": np.array([1.0, 0.0]),
            "class_id": [0],
            "conf": [0.9],
        }
        for index in range(3)
    ]
    detector._voxel_pcd_overlap_matrix = lambda *args, **kwargs: np.asarray(
        [
            [0.0, 0.90, 0.0],
            [0.90, 0.0, 0.80],
            [0.0, 0.80, 0.0],
        ],
        dtype=float,
    )
    detector._exact_duplicate_geometry_matrices = lambda *args, **kwargs: (
        np.zeros((3, 3), dtype=bool),
        np.zeros((3, 3), dtype=float),
    )
    detector._record_merge_gate_diagnostics = lambda *args, **kwargs: (
        np.zeros((3, 3), dtype=bool)
    )
    detector._project_to_registry = lambda **_kwargs: None

    def merge_overlap_objects(**kwargs):
        matrix = kwargs["overlap_matrix"]
        assert matrix[0, 1] > detector.cfg["merge_overlap_thresh"]
        assert matrix[1, 2] == 0.0
        objects = kwargs["objects"]
        objects[0]["class_id"].extend(objects[1]["class_id"])
        objects[0]["conf"].extend(objects[1]["conf"])
        return [objects[0], objects[2]], [0, None, 1]

    detector._cg = {"merge_overlap_objects": merge_overlap_objects}
    detector._maybe_periodic_cleanup()

    assert len(detector._map_objects) == 2
    metrics = detector.quality_metrics()
    assert metrics["periodic_merge_candidate_pairs"] == 2
    assert metrics["periodic_merge_selected_pairs"] == 1
    assert metrics["periodic_merge_deferred_pairs"] == 1


def test_coobserved_merge_event_requires_survivor_to_contain_both_histories() -> None:
    detector = _detector()
    detector._tick_idx = 17
    admission = {
        "left_index": 0,
        "right_index": 1,
        "left": "bowl",
        "right": "cereal box",
        "left_uuid": "cg-left",
        "right_uuid": "cg-right",
        "left_registry_id": "scene.object.bowl_001",
        "right_registry_id": "scene.object.cereal_box_003",
        "shared_frame_count": 3,
        "median_shared_frame_2d_iou": 0.97,
        "center_distance_m": 0.09,
        "voxel_overlap": 0.69,
        "clip_cosine": 0.92,
        "left_image_idx": [10, 11, 12],
        "right_image_idx": [10, 11, 12],
        "left_num_detections": 3,
        "right_num_detections": 3,
    }
    detector._record_coobserved_merge_outcomes(
        [admission],
        merged_objects=[
            {
                "id": "cg-right",
                "image_idx": [10, 11, 12, 10, 11, 12],
                "num_detections": 6,
            }
        ],
        index_updates=[None, 0],
        pre_count=2,
    )
    event = detector._coobserved_merge_events[-1]
    assert event["confirmed"] is True
    assert event["outcome"] == "confirmed_pair_merge"
    assert event["removed_uuid"] == "cg-left"
    assert event["survivor_uuid"] == "cg-right"
    assert event["history_union_present"] is True
    assert event["history_union_exact"] is True
    assert event["detection_sum_present"] is True
    assert event["detection_sum_exact"] is True

    # A deleted left index is not enough: without the duplicated frame
    # multiset and summed detection count in the right survivor, the left
    # object may have been merged into an unrelated third object.
    detector._record_coobserved_merge_outcomes(
        [admission],
        merged_objects=[
            {
                "id": "cg-right",
                "image_idx": [10, 11, 12],
                "num_detections": 3,
            }
        ],
        index_updates=[None, 0],
        pre_count=2,
    )
    event = detector._coobserved_merge_events[-1]
    assert event["confirmed"] is False
    assert event["outcome"] == "removed_into_other_or_incomplete_evidence"
    assert event["history_union_present"] is False
    assert event["history_union_exact"] is False
    assert event["detection_sum_present"] is False
    assert event["detection_sum_exact"] is False

    # A strict superset is direct evidence that the pair was folded together
    # with at least one unadmitted third track in the same cleanup call.
    detector._record_coobserved_merge_outcomes(
        [admission],
        merged_objects=[
            {
                "id": "cg-right",
                "image_idx": [
                    10,
                    11,
                    12,
                    10,
                    11,
                    12,
                    *([99] * 149),
                ],
                "num_detections": 155,
            }
        ],
        index_updates=[None, 0],
        pre_count=3,
    )
    event = detector._coobserved_merge_events[-1]
    assert event["confirmed"] is False
    assert event["outcome"] == "transitive_survivor"
    assert event["history_union_present"] is True
    assert event["history_union_exact"] is False
    assert event["detection_sum_present"] is True
    assert event["detection_sum_exact"] is False


def test_label_aliases_must_reference_resolved_vocabulary() -> None:
    with pytest.raises(ValueError, match="outside the resolved vocabulary"):
        _detector(
            classes=["plant", "potted plant"],
            label_aliases={"plant": "tree"},
        )


def test_label_alias_chains_are_rejected() -> None:
    with pytest.raises(ValueError, match="another alias"):
        _detector(
            classes=["plant", "potted plant", "foliage"],
            label_aliases={
                "plant": "potted plant",
                "potted plant": "foliage",
            },
        )


def test_confusable_groups_must_reference_resolved_vocabulary() -> None:
    with pytest.raises(ValueError, match="outside the resolved vocabulary"):
        _detector(confusable_class_groups=[["sofa", "loveseat"]])


def test_snapshot_exposes_stable_label_evidence() -> None:
    detector = _detector()
    snapshot = {
        "uuid": "u1",
        "cls": "chair",
        "x": 1.0,
        "y": 2.0,
        "z": 0.5,
        "yaw": 0.0,
        "size_x": 0.5,
        "size_y": 0.5,
        "size_z": 1.0,
        "confidence": 0.9,
        "label_confidence": 0.8,
        "label_provisional": False,
        "label_evidence_count": 4,
        "label_candidates": [{"label": "chair", "share": 0.8}],
    }
    asyncio.run(
        detector._apply_snapshot(
            [snapshot],
            observed_uuids={"u1"},
            frame_seq=4,
        )
    )
    obj = next(iter(detector._registry.all_objects()))
    assert obj.cls == "chair"
    assert obj.attributes["label_confidence"] == pytest.approx(0.8)
    assert obj.attributes["label_provisional"] is False
    assert obj.attributes["label_evidence_count"] == 4
    assert obj.attributes["navigation_grade"] is True


def test_operator_label_override_survives_new_model_evidence() -> None:
    detector = _detector()
    detector._map_objects = [
        {
            "id": "u1",
            "class_name": "chair",
            "class_id": [0],
            "conf": [0.9],
        }
    ]
    detector._uuid_to_oid["u1"] = "scene.object.chair_001"
    asyncio.run(
        detector.update_object_label(
            "scene.object.chair_001",
            "table",
        )
    )
    detector._map_objects[0]["class_id"].extend([0, 0, 0, 0])
    detector._map_objects[0]["conf"].extend([0.99] * 4)
    detector._stabilize_map_labels()
    obj = detector._map_objects[0]
    assert obj["class_name"] == "table"
    assert obj["label_source"] == "operator"
    assert obj["label_provisional"] is False
    asyncio.run(
        detector.clear_object_label_override("scene.object.chair_001")
    )
    assert obj["class_name"] == "chair"
    assert obj["label_source"] == "model"


def test_operator_geometry_survives_new_detector_projection() -> None:
    detector = _detector()
    snapshot = {
        "uuid": "u1",
        "cls": "chair",
        "x": 1.0,
        "y": 2.0,
        "z": 0.5,
        "yaw": 0.0,
        "size_x": 0.5,
        "size_y": 0.5,
        "size_z": 1.0,
        "confidence": 0.9,
        "label_confidence": 0.9,
        "label_provisional": False,
        "label_evidence_count": 4,
        "label_candidates": [],
    }

    async def exercise():
        await detector._apply_snapshot(
            [snapshot],
            observed_uuids={"u1"},
            frame_seq=1,
        )
        obj = next(iter(detector._registry.all_objects()))
        await detector.update_object_geometry_override(obj.object_id)
        async with detector._registry.lock():
            detector._registry.update_object_geometry(
                obj.object_id,
                Pose3D(3.0, 4.0, 0.6, 0.5, "map"),
                BBox3D(0.8, 0.4, 1.2, 0.5, "map"),
            )
        changed = dict(snapshot, x=8.0, y=9.0, size_x=2.0)
        await detector._apply_snapshot(
            [changed],
            observed_uuids={"u1"},
            frame_seq=2,
        )
        return obj

    obj = asyncio.run(exercise())
    assert (obj.pose.x, obj.pose.y, obj.pose.z) == (3.0, 4.0, 0.6)
    assert obj.bbox.size_x == pytest.approx(0.8)
    assert obj.attributes["geometry_source"] == "operator_bbox"
    assert obj.attributes["navigation_grade"] is False
