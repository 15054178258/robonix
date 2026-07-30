# SPDX-License-Identifier: MulanPSL-2.0
"""Observation-aware Scene object lifecycle regression tests."""

import asyncio
import threading
from types import SimpleNamespace

import numpy as np

from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _observed_map_object_uuids,
    _visible_missing_uuids,
)
from scene_service.state import ObjectRegistry


def _snapshot(uuid: str = "cg-1") -> dict:
    return {
        "uuid": uuid,
        "cls": "cup",
        "x": 1.0,
        "y": 2.0,
        "z": 0.8,
        "yaw": 0.0,
        "size_x": 0.1,
        "size_y": 0.1,
        "size_z": 0.2,
        "confidence": 0.9,
    }


def _detector(registry: ObjectRegistry) -> ConceptGraphsDetector:
    detector = ConceptGraphsDetector.__new__(ConceptGraphsDetector)
    detector._registry = registry
    detector._uuid_to_oid = {}
    detector._expired_uuids = set()
    detector._missing_uuids = set()
    detector._world_frame_fn = lambda: "map"
    detector._period_s = 0.6
    detector._object_ttl_s = 30.0
    detector._confirmation_min_unique_frames = 2
    detector._confirmation_singleton_min_mean_confidence = 0.65
    detector._visible_miss_threshold = 3
    detector._bbox_low_percentile = 5.0
    detector._bbox_high_percentile = 95.0
    detector._inference_lock = threading.Lock()
    detector._cg = None
    detector._map_objects = []
    detector._asyncio_loop = None
    detector._asyncio_thread_id = None
    detector._quality_counters = {"registry_projection_failures": 0}
    detector.cfg = {
        "max_merge_dist_m": 1.5,
        "identity_rebind_max_distance_m": 0.45,
    }
    return detector


def test_historical_projection_does_not_refresh_positive_observation() -> None:
    async def scenario() -> None:
        registry = ObjectRegistry(grace_period_s=0.1)
        detector = _detector(registry)

        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        obj = next(iter(registry._objects.values()))
        oid = obj.object_id
        assert obj.last_seen == 100.0
        assert obj.observation_count == 1

        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=2,
            observed_at=101.0,
        )
        obj = registry._objects[oid]
        assert obj.last_seen == 100.0
        assert obj.observation_count == 1
        assert not obj.missing

        # Observation-aware objects do not become missing merely because the
        # sensor/model stopped producing healthy frames.
        async with registry.lock():
            assert registry.mark_stale(200.0) == 0
        assert not registry._objects[oid].missing

    asyncio.run(scenario())


def test_worker_projection_waits_for_registry_reconciliation() -> None:
    """A completed worker tick must expose one coherent object snapshot."""

    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        oid = next(iter(registry._objects))

        original_apply_snapshot = detector._apply_snapshot

        async def delayed_apply_snapshot(*args, **kwargs) -> None:
            # Make the old fire-and-forget implementation deterministically
            # return before the registry could evict the removed CG object.
            await asyncio.sleep(0.05)
            await original_apply_snapshot(*args, **kwargs)

        detector._apply_snapshot = delayed_apply_snapshot
        detector._map_objects = []
        detector._asyncio_loop = asyncio.get_running_loop()
        detector._asyncio_thread_id = threading.get_ident()

        await asyncio.to_thread(
            detector._project_to_registry,
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=2,
            observed_at=101.0,
        )

        assert registry._objects[oid].missing
        assert registry._objects[oid].attributes.get("cg_uuid") is None
        assert detector._uuid_to_oid == {}
        assert detector._quality_counters["registry_projection_failures"] == 0

    asyncio.run(scenario())


def test_new_object_requires_distinct_frame_confirmation() -> None:
    """Same-frame hypotheses stay debuggable but cannot pollute memory."""

    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        points = np.asarray(
            [
                [0.90, 1.90, 0.70],
                [1.10, 1.90, 0.70],
                [1.10, 2.10, 0.70],
                [0.90, 2.10, 0.70],
                [0.90, 1.90, 0.90],
                [1.10, 1.90, 0.90],
                [1.10, 2.10, 0.90],
                [0.90, 2.10, 0.90],
            ],
            dtype=np.float64,
        )
        candidate = {
            "id": "cg-candidate",
            "class_name": "chair",
            "resolved_class_name": "chair",
            "pcd": SimpleNamespace(
                points=points,
                colors=np.zeros_like(points),
            ),
            "bbox": object(),
            "conf": [0.45],
            "image_idx": [7],
            "num_detections": 1,
            "label_provisional": True,
        }
        detector._map_objects = [candidate]
        detector._asyncio_loop = asyncio.get_running_loop()
        detector._asyncio_thread_id = threading.get_ident()

        await asyncio.to_thread(
            detector._project_to_registry,
            observed_uuids={"cg-candidate"},
            visible_miss_uuids=set(),
            frame_seq=7,
            observed_at=100.0,
        )

        assert registry._objects == {}
        assert detector.export_3d_snapshot()["objects"] == []
        debug = detector.export_3d_snapshot(include_clip_feature=True)
        assert len(debug["objects"]) == 1
        assert debug["objects"][0]["published_to_registry"] is False
        assert debug["objects"][0]["confirmation_ready"] is False
        assert (
            debug["objects"][0]["confirmation_confidence_fast_path"]
            is False
        )
        assert debug["objects"][0]["confirmation_unique_frames"] == 1
        assert debug["objects"][0]["registry_id"] is None
        quality = detector.quality_metrics()
        assert quality["unconfirmed_candidate_objects"] == 1
        assert quality["confirmed_candidate_objects"] == 0

        candidate["conf"].append(0.55)
        candidate["image_idx"].append(8)
        candidate["num_detections"] = 2
        await asyncio.to_thread(
            detector._project_to_registry,
            observed_uuids={"cg-candidate"},
            visible_miss_uuids=set(),
            frame_seq=8,
            observed_at=101.0,
        )

        assert len(registry._objects) == 1
        published = detector.export_3d_snapshot()["objects"]
        assert len(published) == 1
        assert published[0]["cls"] == "chair"
        debug = detector.export_3d_snapshot(include_clip_feature=True)
        assert debug["objects"][0]["published_to_registry"] is True
        assert debug["objects"][0]["confirmation_ready"] is True
        assert (
            debug["objects"][0]["confirmation_confidence_fast_path"]
            is False
        )
        assert debug["objects"][0]["confirmation_unique_frames"] == 2
        assert debug["objects"][0]["registry_id"] in registry._objects

    asyncio.run(scenario())


def test_high_confidence_singleton_has_configurable_fast_path() -> None:
    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        points = np.asarray(
            [
                [0.90, 1.90, 0.70],
                [1.10, 1.90, 0.70],
                [1.10, 2.10, 0.70],
                [0.90, 2.10, 0.70],
                [0.90, 1.90, 0.90],
                [1.10, 1.90, 0.90],
                [1.10, 2.10, 0.90],
                [0.90, 2.10, 0.90],
            ],
            dtype=np.float64,
        )
        candidate = {
            "id": "cg-confident",
            "class_name": "monitor",
            "resolved_class_name": "monitor",
            "pcd": SimpleNamespace(
                points=points,
                colors=np.zeros_like(points),
            ),
            "bbox": object(),
            "conf": [0.70],
            "image_idx": [7],
            "num_detections": 1,
        }
        detector._map_objects = [candidate]
        detector._asyncio_loop = asyncio.get_running_loop()
        detector._asyncio_thread_id = threading.get_ident()

        await asyncio.to_thread(
            detector._project_to_registry,
            observed_uuids={"cg-confident"},
            visible_miss_uuids=set(),
            frame_seq=7,
            observed_at=100.0,
        )

        assert len(registry._objects) == 1
        debug = detector.export_3d_snapshot(include_clip_feature=True)
        assert debug["objects"][0]["confirmation_unique_frames"] == 1
        assert debug["objects"][0]["confirmation_ready"] is True
        assert (
            debug["objects"][0]["confirmation_confidence_fast_path"]
            is True
        )
        assert debug["objects"][0]["confirmation_mean_confidence"] == 0.70

    asyncio.run(scenario())


def test_unconfirmed_candidate_cannot_claim_nearby_missing_identity() -> None:
    """A fresh uuid cannot bypass confirmation by stealing a nearby old id."""

    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        await detector._apply_snapshot(
            [_snapshot("old")],
            observed_uuids={"old"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        oid = next(iter(registry._objects))
        await detector._apply_snapshot(
            [],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=2,
            observed_at=101.0,
        )
        assert registry._objects[oid].missing

        far = {
            **_snapshot("far"),
            "x": 2.30,
            "confirmation_ready": False,
        }
        await detector._apply_snapshot(
            [far],
            observed_uuids={"far"},
            visible_miss_uuids=set(),
            frame_seq=3,
            observed_at=102.0,
        )
        assert list(registry._objects) == [oid]
        assert registry._objects[oid].missing
        assert detector._uuid_to_oid == {}

        close = {
            **_snapshot("close"),
            "x": 1.20,
            "confirmation_ready": False,
        }
        await detector._apply_snapshot(
            [close],
            observed_uuids={"close"},
            visible_miss_uuids=set(),
            frame_seq=4,
            observed_at=103.0,
        )
        assert list(registry._objects) == [oid]
        assert registry._objects[oid].missing
        assert detector._uuid_to_oid == {}
        assert (
            "identity_rebind_last_kind"
            not in registry._objects[oid].attributes
        )

        confirmed_close = {
            **close,
            "confirmation_ready": True,
        }
        await detector._apply_snapshot(
            [confirmed_close],
            observed_uuids={"close"},
            visible_miss_uuids=set(),
            frame_seq=5,
            observed_at=104.0,
        )
        assert list(registry._objects) == [oid]
        assert not registry._objects[oid].missing
        assert detector._uuid_to_oid == {"close": oid}
        assert (
            registry._objects[oid].attributes[
                "identity_rebind_last_kind"
            ]
            == "cross_tick"
        )
        assert abs(
            registry._objects[oid].attributes[
                "identity_rebind_last_distance_m"
            ]
            - 0.20
        ) < 1e-9

    asyncio.run(scenario())


def test_already_bound_uuid_continues_when_confirmation_history_shortens() -> None:
    """Confirmation gates new identity claims, not an existing UUID binding."""

    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        confirmed = {
            **_snapshot("stable"),
            "confirmation_ready": True,
        }
        await detector._apply_snapshot(
            [confirmed],
            observed_uuids={"stable"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        oid = next(iter(registry._objects))

        shortened_history = {
            **confirmed,
            "x": 1.10,
            "confirmation_ready": False,
        }
        await detector._apply_snapshot(
            [shortened_history],
            observed_uuids={"stable"},
            visible_miss_uuids=set(),
            frame_seq=2,
            observed_at=101.0,
        )

        assert list(registry._objects) == [oid]
        obj = registry._objects[oid]
        assert obj.pose.x == 1.10
        assert obj.observation_count == 2
        assert not obj.missing
        assert detector._uuid_to_oid == {"stable": oid}
        assert "identity_rebind_last_kind" not in obj.attributes

    asyncio.run(scenario())


def test_confirmed_historical_candidate_repairs_registry_binding() -> None:
    """Cleanup UUID churn cannot strand multi-frame CG evidence unpublished."""

    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        historical = {
            **_snapshot("confirmed-history"),
            "confirmation_ready": True,
            "cg_unique_image_idx_count": 6,
            "latest_observed_frame": 42,
        }

        # A cleanup projection may run after the last contributing sensor
        # frame. It repairs the binding but preserves the historical evidence
        # time/count instead of claiming a current observation.
        await detector._apply_snapshot(
            [historical],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=52,
            observed_at=200.0,
        )
        oid = next(iter(registry._objects))
        obj = registry._objects[oid]
        assert detector._uuid_to_oid == {"confirmed-history": oid}
        assert obj.observation_count == 6
        assert obj.attributes["last_observed_frame"] == 42
        assert abs(obj.last_seen - 194.0) < 1e-9
        assert not obj.missing

    asyncio.run(scenario())


def test_cleanup_projection_does_not_double_count_current_frame() -> None:
    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        confirmed = {
            **_snapshot("current-confirmed"),
            "confirmation_ready": True,
            "cg_unique_image_idx_count": 6,
            "latest_observed_frame": 42,
        }
        await detector._apply_snapshot(
            [confirmed],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=42,
            observed_at=194.0,
        )
        oid = next(iter(registry._objects))

        # The final projection of the same sensor frame must not double-count
        # evidence already reconstructed by the cleanup projection.
        await detector._apply_snapshot(
            [confirmed],
            observed_uuids={"current-confirmed"},
            visible_miss_uuids=set(),
            frame_seq=42,
            observed_at=194.0,
        )
        assert registry._objects[oid].observation_count == 6
        assert registry._objects[oid].last_seen == 194.0

    asyncio.run(scenario())


def test_confirmed_rebind_restores_cg_orphan_not_visible_absence() -> None:
    """Internal UUID churn and depth-proven removal have distinct semantics."""

    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        await detector._apply_snapshot(
            [_snapshot("old")],
            observed_uuids={"old"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        oid = next(iter(registry._objects))

        await detector._apply_snapshot(
            [],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=2,
            observed_at=101.0,
        )
        assert registry._objects[oid].missing
        assert (
            registry._objects[oid].attributes["missing_reason"]
            == "cg_orphan"
        )

        confirmed_history = {
            **_snapshot("replacement"),
            "x": 1.1,
            "confirmation_ready": True,
            "cg_unique_image_idx_count": 2,
            "latest_observed_frame": 1,
        }
        await detector._apply_snapshot(
            [confirmed_history],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=3,
            observed_at=102.0,
        )
        obj = registry._objects[oid]
        assert not obj.missing
        assert "missing_reason" not in obj.attributes
        assert detector._uuid_to_oid == {"replacement": oid}

        detector._visible_miss_threshold = 1
        await detector._apply_snapshot(
            [confirmed_history],
            observed_uuids=set(),
            visible_miss_uuids={"replacement"},
            frame_seq=4,
            observed_at=103.0,
        )
        assert obj.missing
        assert obj.attributes["missing_reason"] == "visible_absence"

        # Historical projection is not a positive observation and must not
        # undo depth-verified absence.
        await detector._apply_snapshot(
            [confirmed_history],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=5,
            observed_at=104.0,
        )
        assert obj.missing
        assert obj.attributes["missing_reason"] == "visible_absence"

    asyncio.run(scenario())


def test_only_repeated_healthy_visible_misses_mark_missing() -> None:
    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        oid = next(iter(registry._objects))

        for frame_seq in (2, 3):
            await detector._apply_snapshot(
                [_snapshot()],
                observed_uuids=set(),
                visible_miss_uuids={"cg-1"},
                frame_seq=frame_seq,
                observed_at=100.0 + frame_seq,
            )
            assert not registry._objects[oid].missing

        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids=set(),
            visible_miss_uuids={"cg-1"},
            frame_seq=4,
            observed_at=104.0,
        )
        obj = registry._objects[oid]
        assert obj.missing
        assert detector._missing_uuids == {"cg-1"}
        assert obj.attributes["consecutive_visible_misses"] == 3
        assert obj.last_seen == 100.0

        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=5,
            observed_at=105.0,
        )
        obj = registry._objects[oid]
        assert not obj.missing
        assert detector._missing_uuids == set()
        assert obj.object_id == oid
        assert obj.observation_count == 2
        assert obj.last_seen == 105.0
        assert obj.attributes["consecutive_visible_misses"] == 0

    asyncio.run(scenario())


def test_current_frame_membership_is_distinct_from_historical_map_membership() -> None:
    objects = [
        {"id": "old", "image_idx": [1, 2]},
        {"id": "matched", "image_idx": [1, 7]},
        {"id": "new", "image_idx": [7]},
        {"id": "", "image_idx": [7]},
    ]
    assert _observed_map_object_uuids(objects, frame_seq=7) == {"matched", "new"}


def test_depth_visibility_requires_clear_line_of_sight() -> None:
    obj = {
        "id": "cg-1",
        "pcd": SimpleNamespace(
            points=np.asarray(
                [
                    [-0.04, -0.04, 2.0],
                    [0.04, -0.04, 2.0],
                    [-0.04, 0.04, 2.0],
                    [0.04, 0.04, 2.0],
                ],
                dtype=np.float32,
            )
        ),
    }
    intrinsics = SimpleNamespace(fx=100.0, fy=100.0, cx=2.0, cy=2.0)
    camera_to_world = np.eye(4, dtype=np.float32)

    removed_depth = np.full((5, 5), 4.0, dtype=np.float32)
    diagnostics = {}
    assert _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=removed_depth,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.2,
        diagnostics=diagnostics,
    ) == {"cg-1"}
    assert diagnostics["cg-1"]["status"] == "clear_absence"
    assert diagnostics["cg-1"]["valid_samples"] == 4
    assert diagnostics["cg-1"]["clear_samples"] == 4
    assert diagnostics["cg-1"]["clear_fraction"] == 1.0
    assert diagnostics["cg-1"]["depth_delta_median_m"] == 2.0

    occluded_depth = np.full((5, 5), 1.0, dtype=np.float32)
    diagnostics = {}
    assert not _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=occluded_depth,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.2,
        diagnostics=diagnostics,
    )
    assert diagnostics["cg-1"]["status"] == "insufficient_clear_support"
    assert diagnostics["cg-1"]["occluded_samples"] == 4

    present_depth = np.full((5, 5), 2.05, dtype=np.float32)
    assert not _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=present_depth,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.2,
    )


def test_depth_visibility_requires_distributed_clear_support() -> None:
    obj = {
        "id": "cg-1",
        "pcd": SimpleNamespace(
            points=np.asarray(
                [
                    [-0.16, 0.0, 2.0],
                    [-0.08, 0.0, 2.0],
                    [0.0, 0.0, 2.0],
                    [0.08, 0.0, 2.0],
                    [0.16, 0.0, 2.0],
                ],
                dtype=np.float32,
            )
        ),
    }
    intrinsics = SimpleNamespace(fx=100.0, fy=100.0, cx=10.0, cy=10.0)
    camera_to_world = np.eye(4, dtype=np.float32)

    # One background/depth-hole region is not enough to delete a track whose
    # remaining footprint is still present at the expected range.
    one_clear_patch = np.full((21, 21), 2.0, dtype=np.float32)
    one_clear_patch[9:12, 9:12] = 4.0
    assert not _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=one_clear_patch,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.2,
        min_clear_samples=3,
        min_clear_fraction=0.60,
    )

    # Three spatially separated clear regions out of five satisfy both the
    # absolute and fractional gates.
    distributed_clear = np.full((21, 21), 1.0, dtype=np.float32)
    for u in (2, 10, 18):
        distributed_clear[9:12, max(0, u - 1):min(21, u + 2)] = 4.0
    assert _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=distributed_clear,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.2,
        min_clear_samples=3,
        min_clear_fraction=0.60,
    ) == {"cg-1"}


def test_depth_visibility_accepts_measured_fifteen_centimetre_background_gap() -> None:
    """Keep the runtime threshold below the gap measured by Webots v85."""
    obj = {
        "id": "cg-1",
        "pcd": SimpleNamespace(
            points=np.asarray(
                [
                    [-0.04, -0.04, 2.0],
                    [0.04, -0.04, 2.0],
                    [-0.04, 0.04, 2.0],
                    [0.04, 0.04, 2.0],
                ],
                dtype=np.float32,
            )
        ),
    }
    intrinsics = SimpleNamespace(fx=100.0, fy=100.0, cx=2.0, cy=2.0)
    camera_to_world = np.eye(4, dtype=np.float32)
    newly_exposed_background = np.full((5, 5), 2.15, dtype=np.float32)

    assert _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=newly_exposed_background,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.10,
    ) == {"cg-1"}
    assert not _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=newly_exposed_background,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.20,
    )


def test_epoch_reset_drops_map_objects_and_uuid_bindings() -> None:
    async def scenario() -> None:
        detector = _detector(ObjectRegistry())
        detector._map_objects = [{"id": "cg-1"}]
        detector._uuid_to_oid["cg-1"] = "scene.object.cup_001"
        detector._tick_idx = 9
        await detector.reset_derived_state()
        assert detector._map_objects == []
        assert detector._uuid_to_oid == {}
        assert detector._tick_idx == 0

    asyncio.run(scenario())


def test_ttl_prunes_registry_binding_and_historical_map_object() -> None:
    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        detector._visible_miss_threshold = 1
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids=set(),
            visible_miss_uuids={"cg-1"},
            frame_seq=2,
            observed_at=101.0,
        )
        detector._map_objects = [{"id": "cg-1"}]
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=3,
            observed_at=131.0,
        )
        assert registry._objects == {}
        assert detector._uuid_to_oid == {}
        assert detector._expired_uuids == {"cg-1"}
        detector._purge_expired_map_objects_locked()
        assert detector._map_objects == []

    asyncio.run(scenario())
