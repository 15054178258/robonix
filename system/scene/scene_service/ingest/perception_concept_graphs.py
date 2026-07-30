# SPDX-License-Identifier: MulanPSL-2.0
"""ConceptGraphs perception and persistent object-map integration.

Each tick runs the following pipeline:

* YOLO-World detects open-vocabulary objects.
* MobileSAM creates bounding-box-prompted masks.
* OpenCLIP produces one visual feature per detection.
* Depth, masks, camera intrinsics, and pose produce map-frame point clouds.
* Spatial and visual similarities associate detections with stored objects.
* Matched detections update objects; unmatched detections create new objects.

This replaces the earlier class-and-distance-only association path, which
could not use visual features and frequently over-segmented objects.

The persistent `MapObjectList` is the source of truth. Every tick we
project it back into scene's `ObjectRegistry` so the existing web UI /
MCP API keep working without changes.

Periodic cleanup runs `denoise_objects` + `merge_overlap_objects` from
concept-graphs to GC duplicates that escape the per-tick merge.
"""
from __future__ import annotations

import asyncio
import concurrent.futures
import copy
import logging
import math
import os
import threading
import time
import uuid
from collections import Counter
from typing import Any, Awaitable, Callable, Optional

# numpy is unconditionally imported because (a) the detector loop
# uses it on every tick and (b) the per-method lazy-import convention
# elsewhere in this file kept biting us — every helper that touched
# `np.*` had to remember the import or the whole tick silently bailed
# with NameError. One module-level import is cheap and removes the
# class of bug entirely.
import numpy as np

log = logging.getLogger("scene.ingest.cg")


from .perception_vlm import _CamIntrinsics, _canon_class
from ..state.data_assoc import Detection
from ..state.object_registry import BBox3D, ObjectRegistry, Pose3D, SceneObject


# ── Open-vocab class list ────────────────────────────────────────────────
_DEFAULT_OPEN_VOCAB = [
    # Furniture and room fixtures.
    "chair", "table", "desk", "couch", "sofa", "bed", "nightstand",
    "bookshelf", "shelf", "cabinet", "drawer", "dresser", "wardrobe",
    "counter", "sink", "bathtub", "toilet", "radiator", "whiteboard",
    "door", "doorway", "window",
    # Computing, media, and office equipment.
    "monitor", "laptop", "keyboard", "mouse", "computer tower",
    "monitor stand", "headphones", "webcam", "router", "power strip",
    "television", "telephone", "printer", "projector",
    # Kitchen, dining, and food storage.
    "cup", "mug", "glass", "wineglass", "water bottle", "bottle",
    "thermos", "paper cup", "bowl", "plate", "tray", "carafe", "jar",
    "can", "cereal box", "refrigerator", "oven", "stove", "microwave",
    "dishwasher", "washing machine", "toaster", "kettle", "pan", "pot",
    "lid", "knife", "fork", "spoon", "cutting board",
    # Personal items and storage.
    "backpack", "handbag", "book", "notebook", "pen", "pencil",
    "phone", "tablet",
    "box", "cardboard box", "basket", "trash bin",
    # Workshop and light-industrial objects.
    "tool", "screwdriver", "wrench", "hammer", "drill", "tape",
    "workbench", "pallet", "crate", "pipe", "valve", "control panel",
    "safety helmet", "machine",
    # Decor and other common indoor objects.
    "plant", "potted plant", "lamp", "clock", "picture frame",
    "toy", "snack", "fruit", "apple", "orange", "banana",
    "fire extinguisher", "person",
]

_BG_CLASSES = frozenset({"floor", "wall", "ceiling", "carpet"})

# Classes we never want to insert (unstable / outdoor / not useful for indoor robot).
_IGNORED_CLASSES = frozenset({"person"})


def _resolved_classes(
    configured: Optional[list[str]] = None,
    additional: Optional[list[str]] = None,
) -> list[str]:
    """Resolve the open-vocabulary prompt list with stable de-duplication.

    Normal deployments use Driver config. ``SCENE_OPEN_VOCAB_CLASSES`` remains
    a legacy full override only when no configured base list is supplied.
    """
    raw = (
        os.environ.get("SCENE_OPEN_VOCAB_CLASSES", "").strip()
        if configured is None and additional is None
        else ""
    )
    if raw:
        candidates = [s.strip() for s in raw.split(",") if s.strip()]
    else:
        candidates = list(configured or _DEFAULT_OPEN_VOCAB)
        candidates.extend(additional or ())
    out: list[str] = []
    seen: set[str] = set()
    for candidate in candidates:
        label = str(candidate or "").strip().lower()
        if label and label not in seen:
            seen.add(label)
            out.append(label)
    return out


def _label_evidence(
    obj: Any,
    classes: list[str],
    *,
    current_label: str,
    history_size: int,
    min_switch_observations: int,
    min_winner_share: float,
    switch_margin: float,
    label_aliases: Optional[dict[str, str]] = None,
) -> dict[str, Any]:
    """Aggregate recent confidence-weighted class evidence for one map object."""
    aliases = label_aliases or {}

    def canonical(label: Any) -> str:
        normalized = str(label or "").strip().lower()
        return aliases.get(normalized, normalized)

    class_ids = list(obj.get("class_id", ()) or ())
    confidences = list(obj.get("conf", ()) or ())
    limit = max(1, int(history_size))
    class_ids = class_ids[-limit:]
    confidences = confidences[-limit:]
    scores: dict[str, float] = {}
    counts: dict[str, int] = {}
    for index, raw_class_id in enumerate(class_ids):
        try:
            class_id = int(raw_class_id)
        except (TypeError, ValueError):
            continue
        if not 0 <= class_id < len(classes):
            continue
        label = canonical(classes[class_id])
        if not label:
            continue
        try:
            confidence = float(confidences[index])
        except (IndexError, TypeError, ValueError):
            confidence = 1.0
        weight = max(0.01, min(1.0, confidence))
        scores[label] = scores.get(label, 0.0) + weight
        counts[label] = counts.get(label, 0) + 1

    fallback = canonical(current_label or obj.get("class_name") or "object")
    if not scores:
        return {
            "label": fallback or "object",
            "confidence": 0.0,
            "provisional": True,
            "evidence_count": 0,
            "candidates": [],
        }
    ranked = sorted(scores, key=lambda label: (-scores[label], label))
    winner = ranked[0]
    total = max(1e-9, sum(scores.values()))
    winner_share = scores[winner] / total
    current = fallback if fallback in scores else ""
    selected = current or winner
    if winner != selected:
        selected_score = scores.get(selected, 0.0)
        enough_support = counts[winner] >= max(1, int(min_switch_observations))
        enough_share = winner_share >= max(0.0, min(1.0, float(min_winner_share)))
        enough_margin = (
            scores[winner] - selected_score
            >= max(0.0, float(switch_margin)) * total
        )
        if enough_support and enough_share and enough_margin:
            selected = winner
    candidates = [
        {
            "label": label,
            "score": round(scores[label], 6),
            "share": round(scores[label] / total, 6),
            "observations": counts[label],
        }
        for label in ranked[:5]
    ]
    return {
        "label": selected,
        "confidence": scores.get(selected, 0.0) / total,
        "provisional": (
            counts.get(selected, 0) < max(1, int(min_switch_observations))
            or scores.get(selected, 0.0) / total
            < max(0.0, min(1.0, float(min_winner_share)))
        ),
        "evidence_count": sum(counts.values()),
        "candidates": candidates,
    }


def _bounded_observation_history(
    obj: Any,
    classes: list[str],
    *,
    limit: int = 64,
) -> list[dict[str, Any]]:
    """Return aligned, JSON-safe per-frame detector evidence for diagnostics."""
    frames = list(obj.get("image_idx", ()) or ())
    boxes = list(obj.get("xyxy", ()) or ())
    masks = list(obj.get("mask_idx", ()) or ())
    class_ids = list(obj.get("class_id", ()) or ())
    confidences = list(obj.get("conf", ()) or ())
    start = max(0, len(frames) - max(1, int(limit)))
    history: list[dict[str, Any]] = []
    for index in range(start, len(frames)):
        try:
            frame = int(frames[index])
            box = [float(value) for value in boxes[index]]
        except (IndexError, TypeError, ValueError):
            continue
        if len(box) != 4 or not all(math.isfinite(value) for value in box):
            continue
        record: dict[str, Any] = {
            "frame": frame,
            "bbox_xyxy": [round(value, 3) for value in box],
        }
        try:
            record["mask_index"] = int(masks[index])
        except (IndexError, TypeError, ValueError):
            record["mask_index"] = None
        try:
            class_id = int(class_ids[index])
            if 0 <= class_id < len(classes):
                record["label"] = str(classes[class_id])
        except (IndexError, TypeError, ValueError):
            pass
        try:
            confidence = float(confidences[index])
            if math.isfinite(confidence):
                record["confidence"] = round(
                    max(0.0, min(1.0, confidence)),
                    6,
                )
        except (IndexError, TypeError, ValueError):
            pass
        history.append(record)
    return history


def _observed_map_object_uuids(
    map_objects: Any,
    *,
    frame_seq: int,
) -> set[str]:
    """Return map-object UUIDs that contain evidence from ``frame_seq``.

    ConceptGraphs keeps every contributing frame in ``image_idx`` when it
    merges a detection into a historical object. This is the positive
    observation boundary: membership in the persistent ``MapObjectList`` is
    not evidence that an object was seen in the current frame.
    """
    observed: set[str] = set()
    for obj in map_objects or ():
        try:
            indices = obj.get("image_idx", ())
            if frame_seq not in indices:
                continue
            uuid_value = str(obj.get("id", "") or "")
            if uuid_value:
                observed.add(uuid_value)
        except (TypeError, ValueError):
            continue
    return observed


def _unique_observation_frame_count(obj: Any) -> int:
    """Count distinct valid sensor frames contributing to one map object.

    ConceptGraphs may retain more than one detector hypothesis from the same
    RGB frame.  Those hypotheses are useful label/geometry evidence, but they
    are not independent temporal confirmation that a persistent object exists.
    """
    unique_frames: set[int] = set()
    for value in obj.get("image_idx", ()) or ():
        try:
            unique_frames.add(int(value))
        except (TypeError, ValueError):
            continue
    return len(unique_frames)


def _object_confirmation_status(
    obj: Any,
    *,
    min_unique_frames: int,
    singleton_min_mean_confidence: float,
) -> tuple[int, float, bool, bool]:
    """Return temporal/confidence evidence for persistent publication."""
    unique_frames = _unique_observation_frame_count(obj)
    confidences: list[float] = []
    for value in obj.get("conf", ()) or ():
        try:
            confidence = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(confidence):
            confidences.append(max(0.0, min(1.0, confidence)))
    mean_confidence = (
        sum(confidences) / len(confidences)
        if confidences
        else 0.0
    )
    confidence_threshold = max(
        0.0,
        min(1.0, float(singleton_min_mean_confidence)),
    )
    confidence_fast_path = bool(
        confidence_threshold > 0.0
        and mean_confidence >= confidence_threshold
    )
    confirmation_ready = bool(
        unique_frames >= max(1, int(min_unique_frames))
        or confidence_fast_path
    )
    return (
        unique_frames,
        mean_confidence,
        confidence_fast_path,
        confirmation_ready,
    )


def _visible_missing_uuids(
    map_objects: Any,
    *,
    observed_uuids: set[str],
    depth_m: Any,
    intrinsics: Any,
    camera_to_world: Any,
    depth_margin_m: float,
    min_clear_samples: int = 3,
    min_clear_fraction: float = 0.60,
    max_projected_samples: int = 25,
    diagnostics: Optional[dict[str, dict[str, Any]]] = None,
) -> set[str]:
    """Return unmatched objects whose old location is visibly empty.

    A spatially distributed set of stored object-cloud samples is projected
    into the current depth image. A miss counts only when enough samples show
    the measured surface materially *behind* the stored object. A closer
    surface means occlusion; a similar depth means the object may still be
    present but the detector missed it; invalid/out-of-FOV depth is unknown.
    Those cases must not delete state.

    Sampling multiple projected regions fixes the old center-pixel blind spot:
    the object center can land on a depth hole or thin occluder even when the
    rest of the former object footprint is clearly empty. Requiring both an
    absolute sample count and a clear fraction prevents one background pixel
    from deleting an object that is mostly occluded or still present.
    """
    try:
        depth = np.asarray(depth_m, dtype=np.float32)
        transform = np.asarray(camera_to_world, dtype=np.float64)
        if depth.ndim != 2 or transform.shape != (4, 4):
            return set()
        world_to_camera = np.linalg.inv(transform)
    except (TypeError, ValueError, np.linalg.LinAlgError):
        return set()

    height, width = depth.shape
    margin = max(0.0, float(depth_margin_m))
    minimum = max(1, int(min_clear_samples))
    clear_fraction = max(0.0, min(1.0, float(min_clear_fraction)))
    sample_limit = max(minimum, int(max_projected_samples))
    visible_missing: set[str] = set()

    def record(uuid_value: str, status: str, **values: Any) -> None:
        if diagnostics is not None and uuid_value:
            diagnostics[uuid_value] = {
                "status": status,
                **values,
            }

    for obj in map_objects or ():
        uuid_value = ""
        try:
            uuid_value = str(obj.get("id", "") or "")
            if not uuid_value:
                continue
            if uuid_value in observed_uuids:
                record(uuid_value, "observed")
                continue
            points = np.asarray(obj["pcd"].points, dtype=np.float64)
            points = points[np.all(np.isfinite(points), axis=1)]
            if points.shape[0] < 1:
                record(uuid_value, "no_finite_points")
                continue
            homogeneous = np.column_stack(
                (points, np.ones(points.shape[0], dtype=np.float64))
            )
            camera_points = (world_to_camera @ homogeneous.T).T[:, :3]
            expected_depths = camera_points[:, 2]
            projectable = np.isfinite(expected_depths) & (
                expected_depths > 0.0
            )
            if not np.any(projectable):
                record(uuid_value, "behind_camera")
                continue
            camera_points = camera_points[projectable]
            expected_depths = expected_depths[projectable]
            us = np.rint(
                float(intrinsics.fx)
                * camera_points[:, 0]
                / expected_depths
                + float(intrinsics.cx)
            ).astype(np.int64)
            vs = np.rint(
                float(intrinsics.fy)
                * camera_points[:, 1]
                / expected_depths
                + float(intrinsics.cy)
            ).astype(np.int64)
            in_view = (
                (us >= 0)
                & (us < width)
                & (vs >= 0)
                & (vs < height)
            )
            if not np.any(in_view):
                record(
                    uuid_value,
                    "out_of_view",
                    projectable_points=int(camera_points.shape[0]),
                )
                continue
            us, vs, expected_depths = (
                us[in_view],
                vs[in_view],
                expected_depths[in_view],
            )

            # Keep the front-most stored surface at each projected pixel, then
            # select evenly across image-space order. This is deterministic,
            # bounded, and avoids overweighting dense patches of the fused
            # cloud while retaining the object's spatial footprint.
            projected: dict[tuple[int, int], float] = {}
            for u, v, expected_depth in zip(
                us.tolist(),
                vs.tolist(),
                expected_depths.tolist(),
            ):
                key = (int(u), int(v))
                projected[key] = min(
                    float(expected_depth),
                    projected.get(key, math.inf),
                )
            samples = sorted(
                (
                    (u, v, expected_depth)
                    for (u, v), expected_depth in projected.items()
                ),
                key=lambda item: (item[1], item[0]),
            )
            if len(samples) > sample_limit:
                indices = np.linspace(
                    0,
                    len(samples) - 1,
                    sample_limit,
                    dtype=np.int64,
                )
                samples = [samples[int(index)] for index in indices]

            clear = 0
            occluded = 0
            similar_depth = 0
            valid_sample_count = 0
            depth_deltas: list[float] = []
            for u, v, expected_depth in samples:
                u0, u1 = max(0, u - 1), min(width, u + 2)
                v0, v1 = max(0, v - 1), min(height, v + 2)
                patch = depth[v0:v1, u0:u1]
                valid = patch[np.isfinite(patch) & (patch > 0.0)]
                if valid.size == 0:
                    continue
                valid_sample_count += 1
                measured_depth = float(np.median(valid))
                delta = measured_depth - expected_depth
                depth_deltas.append(delta)
                if delta > margin:
                    clear += 1
                elif delta < -margin:
                    occluded += 1
                else:
                    similar_depth += 1
            clear_ratio = (
                clear / valid_sample_count
                if valid_sample_count
                else 0.0
            )
            is_visibly_missing = (
                valid_sample_count >= minimum
                and clear >= minimum
                and clear_ratio >= clear_fraction
            )
            record(
                uuid_value,
                (
                    "clear_absence"
                    if is_visibly_missing
                    else "insufficient_clear_support"
                ),
                source_points=int(points.shape[0]),
                unique_projected_pixels=int(len(projected)),
                sampled_pixels=int(len(samples)),
                valid_samples=int(valid_sample_count),
                clear_samples=int(clear),
                occluded_samples=int(occluded),
                similar_depth_samples=int(similar_depth),
                clear_fraction=round(float(clear_ratio), 6),
                depth_delta_median_m=(
                    round(float(np.median(depth_deltas)), 6)
                    if depth_deltas
                    else None
                ),
                depth_delta_max_m=(
                    round(float(np.max(depth_deltas)), 6)
                    if depth_deltas
                    else None
                ),
                required_clear_samples=int(minimum),
                required_clear_fraction=float(clear_fraction),
                depth_margin_m=float(margin),
            )
            if is_visibly_missing:
                visible_missing.add(uuid_value)
        except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
            record(uuid_value, "diagnostic_error", error=type(exc).__name__)
            continue
    return visible_missing


def _erode_binary_mask(mask: np.ndarray, radius_px: int) -> np.ndarray:
    """Square-kernel binary erosion without an OpenCV/SciPy dependency."""
    radius = max(0, int(radius_px))
    source = np.asarray(mask, dtype=bool)
    if radius == 0 or source.ndim != 2:
        return source.copy()
    height, width = source.shape
    padded = np.pad(source, radius, mode="constant", constant_values=False)
    eroded = np.ones_like(source)
    window = 2 * radius + 1
    for dy in range(window):
        for dx in range(window):
            eroded &= padded[dy:dy + height, dx:dx + width]
    return eroded


def _refine_masks_with_depth(
    masks: Any,
    depth_m: Any,
    *,
    erosion_px: int,
    min_depth_m: float,
    max_depth_m: float,
    mad_scale: float,
    min_band_m: float,
    min_points: int,
    min_retained_ratio: float = 0.25,
) -> np.ndarray:
    """Suppress mask-edge background and depth spikes before backprojection.

    Each SAM mask is conservatively eroded, range-gated, then clipped around
    its median depth using a robust MAD band. If either refinement would erase
    a small valid object, the previous valid stage is retained. This improves
    geometry without converting a quality filter into a small-object recall
    regression.
    """
    mask_array = np.asarray(masks, dtype=bool)
    depth = np.asarray(depth_m, dtype=np.float32)
    if mask_array.ndim != 3 or depth.ndim != 2:
        return mask_array.copy()
    if mask_array.shape[1:] != depth.shape:
        return mask_array.copy()

    minimum = max(1, int(min_points))
    near = max(0.0, float(min_depth_m))
    far = max(near, float(max_depth_m))
    finite_range = np.isfinite(depth) & (depth >= near) & (depth <= far)
    out = np.zeros_like(mask_array)
    for index, original in enumerate(mask_array):
        ranged = original & finite_range
        if int(ranged.sum()) < minimum:
            continue

        eroded = _erode_binary_mask(ranged, erosion_px)
        required_after_erode = max(
            minimum,
            int(round(float(ranged.sum()) * max(0.0, min(1.0, min_retained_ratio)))),
        )
        working = eroded if int(eroded.sum()) >= required_after_erode else ranged

        values = depth[working]
        median = float(np.median(values))
        mad = float(np.median(np.abs(values - median)))
        robust_sigma = 1.4826 * mad
        band = max(float(min_band_m), float(mad_scale) * robust_sigma)
        clipped = working & (np.abs(depth - median) <= band)
        required_after_clip = max(
            minimum,
            int(round(float(working.sum()) * max(0.0, min(1.0, min_retained_ratio)))),
        )
        out[index] = clipped if int(clipped.sum()) >= required_after_clip else working
    return out


def _run_frame_pointcloud_filter(
    entry: Any,
    *,
    process_pcd: Any,
    get_bounding_box: Any,
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    spatial_sim_type: str,
    min_points_threshold: int,
    run_dbscan: bool,
) -> tuple[Any, dict[str, Any]]:
    """Apply ConceptGraphs' canonical PCD filter once before association.

    The ali-dev ``detections_to_obj_pcd_and_bbox`` signature accepts all of
    the filtering arguments but does not consume them.  Passing
    ``run_dbscan=True`` to that function therefore only *looks* as if frame
    filtering is enabled.  This adapter deliberately asks the conversion
    function for the unfiltered cloud, then invokes ConceptGraphs'
    ``process_pcd`` here so current and future upstream implementations have
    one unambiguous filtering owner.

    Returning a diagnostic keeps the runtime claim auditable without
    replacing ConceptGraphs' clustering algorithm.
    """
    if entry is None:
        return None, {
            "attempted": False,
            "status": "missing_entry",
            "input_points": 0,
            "output_points": 0,
        }
    try:
        point_cloud = entry["pcd"]
        input_points = int(len(point_cloud.points))
    except (AttributeError, KeyError, TypeError, ValueError):
        return None, {
            "attempted": False,
            "status": "invalid_entry",
            "input_points": 0,
            "output_points": 0,
        }
    if not run_dbscan:
        return entry, {
            "attempted": False,
            "status": "disabled",
            "input_points": input_points,
            "output_points": input_points,
        }

    filtered = process_pcd(
        point_cloud,
        downsample_voxel_size=float(downsample_voxel_size),
        dbscan_remove_noise=bool(dbscan_remove_noise),
        dbscan_eps=float(dbscan_eps),
        dbscan_min_points=int(dbscan_min_points),
        run_dbscan=True,
    )
    output_points = int(len(filtered.points))
    diagnostic = {
        "attempted": True,
        "status": "filtered",
        "input_points": input_points,
        "output_points": output_points,
    }
    minimum = max(1, int(min_points_threshold))
    if output_points < minimum:
        diagnostic["status"] = "insufficient_points"
        return None, diagnostic

    updated = dict(entry)
    updated["pcd"] = filtered
    updated["bbox"] = get_bounding_box(str(spatial_sim_type), filtered)
    return updated, diagnostic


def _occupancy_contains_points(
    points_world: Any,
    occupancy_grid: Any,
    *,
    expected_frame: str,
    margin_m: float = 0.25,
    max_outside_fraction: float = 0.20,
) -> bool:
    """Check a world-frame point cloud against a possibly rotated map grid."""
    try:
        points = np.asarray(points_world, dtype=np.float64)
        points = points[np.all(np.isfinite(points), axis=1)]
        info = occupancy_grid.info
        resolution = float(info.resolution)
        width = int(info.width)
        height = int(info.height)
        frame = str(
            getattr(getattr(occupancy_grid, "header", None), "frame_id", "") or ""
        ).strip().lstrip("/")
        if (
            points.ndim != 2
            or points.shape[0] == 0
            or points.shape[1] < 2
            or resolution <= 0.0
            or width <= 0
            or height <= 0
            or not frame
            or frame != str(expected_frame or "").strip().lstrip("/")
        ):
            return False
        origin = info.origin
        q = origin.orientation
        yaw = math.atan2(
            2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
            1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
        )
        dx = points[:, 0] - float(origin.position.x)
        dy = points[:, 1] - float(origin.position.y)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        map_width_m = width * resolution
        map_height_m = height * resolution
        margin = max(0.0, float(margin_m))
        inside = (
            (local_x >= -margin)
            & (local_x <= map_width_m + margin)
            & (local_y >= -margin)
            & (local_y <= map_height_m + margin)
        )
        outside_fraction = 1.0 - float(np.mean(inside))
        center_x = float(np.median(local_x))
        center_y = float(np.median(local_y))
        center_inside = (
            -margin <= center_x <= map_width_m + margin
            and -margin <= center_y <= map_height_m + margin
        )
        return center_inside and outside_fraction <= max(
            0.0,
            min(1.0, float(max_outside_fraction)),
        )
    except (AttributeError, TypeError, ValueError):
        return False


def _planar_surface_snap_translation(
    points_world: Any,
    occupancy_grid: Any,
    *,
    expected_frame: str,
    max_distance_m: float = 0.60,
    tangent_padding_m: float = 0.25,
    min_shift_m: float = 0.05,
    min_support_cells: int = 30,
    min_dominant_share: float = 0.55,
    min_tangent_coverage: float = 0.50,
    occupancy_threshold: int = 50,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Estimate a conservative XY translation onto a supported map surface.

    Transparent or reflective wall fixtures can be classified correctly in
    RGB while their depth pixels land on geometry behind the fixture.  The
    resulting cloud is often a compact vertical plane parallel to the real
    wall, but displaced along its normal.  This helper finds a dominant line
    of occupied grid cells parallel to the cloud's long XY axis and returns
    only the normal translation.  It never changes yaw, extent, or Z.

    The caller owns the semantic policy deciding which labels may use this
    correction.  Support, dominance, span, frame, and distance gates make the
    geometry operation independent of any particular class or simulator.
    """

    zero = np.zeros(3, dtype=np.float64)

    def rejected(status: str, **details: Any) -> tuple[np.ndarray, dict[str, Any]]:
        return zero.copy(), {"status": status, "applied": False, **details}

    try:
        points = np.asarray(points_world, dtype=np.float64)
        points = points[
            np.all(np.isfinite(points[:, :3]), axis=1)
        ] if points.ndim == 2 and points.shape[1] >= 3 else np.empty((0, 3))
        if points.shape[0] < 4:
            return rejected("insufficient_points")
        bbox = _robust_yaw_bbox(points)
        if bbox is None:
            return rejected("invalid_bbox")
        center, extent, tangent_yaw = bbox
        long_extent = float(extent[0])
        if long_extent <= 0.0:
            return rejected("degenerate_bbox")

        info = occupancy_grid.info
        resolution = float(info.resolution)
        width = int(info.width)
        height = int(info.height)
        frame = str(
            getattr(getattr(occupancy_grid, "header", None), "frame_id", "") or ""
        ).strip().lstrip("/")
        expected = str(expected_frame or "").strip().lstrip("/")
        values = np.asarray(
            list(getattr(occupancy_grid, "data", ()) or ()),
            dtype=np.int16,
        )
        if (
            resolution <= 0.0
            or width <= 0
            or height <= 0
            or values.size != width * height
            or not frame
            or frame != expected
        ):
            return rejected("invalid_occupancy")
        occupied_rows, occupied_columns = np.nonzero(
            values.reshape(height, width) >= int(occupancy_threshold)
        )
        if occupied_rows.size == 0:
            return rejected("no_occupied_cells")

        origin = info.origin
        orientation = origin.orientation
        grid_yaw = math.atan2(
            2.0
            * (
                float(orientation.w) * float(orientation.z)
                + float(orientation.x) * float(orientation.y)
            ),
            1.0
            - 2.0
            * (
                float(orientation.y) ** 2
                + float(orientation.z) ** 2
            ),
        )
        local_x = (occupied_columns.astype(np.float64) + 0.5) * resolution
        local_y = (occupied_rows.astype(np.float64) + 0.5) * resolution
        grid_cosine, grid_sine = math.cos(grid_yaw), math.sin(grid_yaw)
        occupied_xy = np.column_stack(
            (
                float(origin.position.x)
                + grid_cosine * local_x
                - grid_sine * local_y,
                float(origin.position.y)
                + grid_sine * local_x
                + grid_cosine * local_y,
            )
        )

        tangent = np.asarray(
            [math.cos(tangent_yaw), math.sin(tangent_yaw)],
            dtype=np.float64,
        )
        normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float64)
        delta_xy = occupied_xy - center[:2]
        along = delta_xy @ tangent
        across = delta_xy @ normal
        max_distance = max(0.0, float(max_distance_m))
        padding = max(0.0, float(tangent_padding_m))
        candidates = (
            (np.abs(along) <= long_extent * 0.5 + padding)
            & (np.abs(across) <= max_distance)
        )
        candidate_count = int(np.count_nonzero(candidates))
        if candidate_count < max(1, int(min_support_cells)):
            return rejected(
                "insufficient_support",
                candidate_cells=candidate_count,
            )

        candidate_across = across[candidates]
        candidate_along = along[candidates]
        bin_width = max(0.05, resolution)
        bins = np.rint(candidate_across / bin_width).astype(np.int64)
        unique_bins, counts = np.unique(bins, return_counts=True)
        # Prefer the densest parallel surface; a deterministic nearest-offset
        # tie-break avoids jumping to a farther, equally supported obstacle.
        best_index = max(
            range(len(unique_bins)),
            key=lambda index: (
                int(counts[index]),
                -abs(float(unique_bins[index]) * bin_width),
            ),
        )
        mode_center = float(unique_bins[best_index]) * bin_width
        cluster = np.abs(candidate_across - mode_center) <= max(
            0.075,
            1.5 * resolution,
        )
        support_count = int(np.count_nonzero(cluster))
        dominant_share = support_count / candidate_count
        tangent_span = (
            float(np.ptp(candidate_along[cluster]))
            if support_count >= 2
            else 0.0
        )
        tangent_coverage = tangent_span / max(long_extent, resolution)
        if support_count < max(1, int(min_support_cells)):
            return rejected(
                "insufficient_dominant_support",
                candidate_cells=candidate_count,
                support_cells=support_count,
                dominant_share=dominant_share,
                tangent_coverage=tangent_coverage,
            )
        if dominant_share < max(0.0, min(1.0, float(min_dominant_share))):
            return rejected(
                "ambiguous_support",
                candidate_cells=candidate_count,
                support_cells=support_count,
                dominant_share=dominant_share,
                tangent_coverage=tangent_coverage,
            )
        if tangent_coverage < max(
            0.0,
            min(1.0, float(min_tangent_coverage)),
        ):
            return rejected(
                "short_support",
                candidate_cells=candidate_count,
                support_cells=support_count,
                dominant_share=dominant_share,
                tangent_coverage=tangent_coverage,
            )

        signed_shift = float(np.median(candidate_across[cluster]))
        shift_m = abs(signed_shift)
        if shift_m < max(0.0, float(min_shift_m)):
            return rejected(
                "already_supported",
                shift_m=shift_m,
                candidate_cells=candidate_count,
                support_cells=support_count,
                dominant_share=dominant_share,
                tangent_coverage=tangent_coverage,
            )
        if shift_m > max_distance:
            return rejected(
                "support_too_far",
                shift_m=shift_m,
                candidate_cells=candidate_count,
                support_cells=support_count,
            )
        translation = np.asarray(
            [normal[0] * signed_shift, normal[1] * signed_shift, 0.0],
            dtype=np.float64,
        )
        return translation, {
            "status": "applied",
            "applied": True,
            "translation_m": translation.tolist(),
            "shift_m": shift_m,
            "candidate_cells": candidate_count,
            "support_cells": support_count,
            "dominant_share": dominant_share,
            "tangent_coverage": tangent_coverage,
            "tangent_yaw_rad": float(tangent_yaw),
        }
    except (AttributeError, TypeError, ValueError):
        return rejected("invalid_input")


def _adaptive_association_distance_limit(
    detection_extent_xy: Any,
    object_extent_xy: Any,
    *,
    minimum_m: float,
    maximum_m: float,
    extent_scale: float,
) -> float:
    """Return a generic instance-scale centroid gate in metres.

    A single room-wide distance cap lets adjacent repeated instances (windows,
    chairs, cabinets, and similar objects) chain into one persistent track.
    Conversely, a small fixed cap fragments large furniture across viewpoints.
    Use the geometric mean of the two measured horizontal bbox diagonals so
    the gate follows physical evidence from both sides without depending on a
    semantic class name.
    """

    def horizontal_diagonal(raw: Any) -> float:
        try:
            values = np.asarray(raw, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError):
            return 0.0
        if values.size < 2 or not np.all(np.isfinite(values[:2])):
            return 0.0
        return float(np.hypot(max(0.0, values[0]), max(0.0, values[1])))

    lower = max(0.0, float(minimum_m))
    upper = max(lower, float(maximum_m))
    scale = max(0.0, float(extent_scale))
    detection_span = horizontal_diagonal(detection_extent_xy)
    object_span = horizontal_diagonal(object_extent_xy)
    if detection_span <= 0.0 or object_span <= 0.0:
        return lower
    measured = scale * math.sqrt(detection_span * object_span)
    return min(upper, max(lower, measured))


def _one_to_one_association_mask(
    similarities: Any,
    *,
    threshold: float,
) -> np.ndarray:
    """Select a maximum-weight one-to-one detection-to-track assignment.

    ConceptGraphs merges each detection row independently. Without an
    additional assignment constraint, two distinct detections in the same
    camera frame can both update one persistent object and irreversibly blend
    their point clouds. Dummy columns represent leaving a detection unmatched;
    ConceptGraphs then creates a new track through its normal code path.
    """

    scores = np.asarray(similarities, dtype=np.float64)
    if scores.ndim != 2:
        raise ValueError("association similarities must be a matrix")
    rows, columns = scores.shape
    selected = np.zeros((rows, columns), dtype=bool)
    if rows == 0 or columns == 0:
        return selected
    cutoff = float(threshold)
    valid = np.isfinite(scores) & (scores >= cutoff)
    if not np.any(valid):
        return selected

    # scipy is already a Scene runtime dependency. Rectangular assignment lets
    # every detection choose either one unique existing object or its private
    # unmatched dummy column.
    from scipy.optimize import linear_sum_assignment

    floor = min(-1.0e9, cutoff - 1.0e6)
    utility = np.full((rows, columns + rows), floor, dtype=np.float64)
    utility[:, :columns] = np.where(valid, scores, floor)
    unmatched_utility = cutoff - max(1.0e-9, abs(cutoff) * 1.0e-9)
    utility[np.arange(rows), columns + np.arange(rows)] = unmatched_utility
    assigned_rows, assigned_columns = linear_sum_assignment(
        utility,
        maximize=True,
    )
    for row, column in zip(assigned_rows, assigned_columns):
        if column < columns and valid[row, column]:
            selected[row, column] = True
    return selected


def _disjoint_periodic_merge_mask(
    spatial_scores: Any,
    *,
    spatial_threshold: float,
    visual_scores: Any | None = None,
    visual_threshold: float = -1.0,
) -> np.ndarray:
    """Select conservative, endpoint-disjoint periodic merge pairs.

    ConceptGraphs' cleanup merger accepts a full symmetric overlap graph and
    may collapse a connected component transitively in one call.  Pairwise
    Robonix admission cannot justify that: A-B and B-C passing does not prove
    that A and C are the same physical instance.  Keep the upstream merger,
    but expose only a deterministic set of non-overlapping edges per cleanup
    tick.  Remaining candidates are reconsidered after geometry and evidence
    have been recomputed on a later tick.
    """

    spatial = np.asarray(spatial_scores, dtype=np.float64)
    if spatial.ndim != 2 or spatial.shape[0] != spatial.shape[1]:
        raise ValueError("periodic merge spatial scores must be square")
    count = spatial.shape[0]
    selected = np.zeros((count, count), dtype=bool)
    if count < 2:
        return selected

    visual: np.ndarray | None = None
    if visual_scores is not None:
        visual = np.asarray(visual_scores, dtype=np.float64)
        if visual.shape != spatial.shape:
            raise ValueError(
                "periodic merge visual scores must match spatial scores"
            )

    candidates: list[tuple[float, float, int, int]] = []
    for left in range(count):
        for right in range(left + 1, count):
            spatial_score = float(
                max(spatial[left, right], spatial[right, left])
            )
            if (
                not math.isfinite(spatial_score)
                or spatial_score <= float(spatial_threshold)
            ):
                continue
            visual_score = 0.0
            if visual is not None:
                visual_score = float(
                    max(visual[left, right], visual[right, left])
                )
                if (
                    not math.isfinite(visual_score)
                    or visual_score <= float(visual_threshold)
                ):
                    continue
            candidates.append(
                (spatial_score, visual_score, left, right)
            )

    # Prefer the strongest physical evidence, then visual evidence. Stable
    # indices make equal-score runs reproducible.
    candidates.sort(
        key=lambda item: (-item[0], -item[1], item[2], item[3])
    )
    used: set[int] = set()
    for _spatial, _visual, left, right in candidates:
        if left in used or right in used:
            continue
        selected[left, right] = True
        selected[right, left] = True
        used.update((left, right))
    return selected


def _pairwise_object_clip_cosines(objects: Any) -> np.ndarray:
    """Return a symmetric CLIP-cosine matrix for persistent map objects."""

    count = len(objects)
    result = np.full((count, count), np.nan, dtype=np.float64)

    def feature(obj: Any) -> np.ndarray | None:
        try:
            raw = obj["clip_ft"]
            try:
                raw = raw.detach().float().cpu()
            except AttributeError:
                pass
            values = np.asarray(raw, dtype=np.float64).reshape(-1)
        except (KeyError, TypeError, ValueError):
            return None
        if values.size == 0 or not np.all(np.isfinite(values)):
            return None
        norm = float(np.linalg.norm(values))
        return None if norm <= 1e-12 else values / norm

    features = [feature(obj) for obj in objects]
    for left, left_feature in enumerate(features):
        if left_feature is None:
            continue
        result[left, left] = 1.0
        for right in range(left + 1, count):
            right_feature = features[right]
            if (
                right_feature is None
                or left_feature.shape != right_feature.shape
            ):
                continue
            score = float(np.dot(left_feature, right_feature))
            result[left, right] = score
            result[right, left] = score
    return result


def _axis_yaw(yaw: float) -> float:
    """Canonicalize a direction-symmetric axis to ``[-pi/2, pi/2)``."""
    wrapped = (float(yaw) + math.pi / 2.0) % math.pi - math.pi / 2.0
    return 0.0 if abs(wrapped) < 1e-12 else wrapped


def _convex_hull_xy(xy: np.ndarray) -> np.ndarray:
    """Return the deterministic counter-clockwise 2D convex hull."""
    ordered = np.unique(np.asarray(xy, dtype=np.float64)[:, :2], axis=0)
    if ordered.shape[0] <= 2:
        return ordered

    def cross(origin: np.ndarray, left: np.ndarray, right: np.ndarray) -> float:
        return float(
            (left[0] - origin[0]) * (right[1] - origin[1])
            - (left[1] - origin[1]) * (right[0] - origin[0])
        )

    lower: list[np.ndarray] = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[np.ndarray] = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)


def _xy_quantile_rectangle(
    xy: np.ndarray,
    *,
    yaw: float,
    low: float,
    high: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return world center and XY extent for one candidate orientation."""
    anchor = np.median(xy, axis=0)
    cos_yaw, sin_yaw = float(np.cos(yaw)), float(np.sin(yaw))
    world_to_local = np.array(
        [[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]],
        dtype=np.float64,
    )
    local_xy = (xy - anchor) @ world_to_local.T
    lo_xy = np.percentile(local_xy, low, axis=0)
    hi_xy = np.percentile(local_xy, high, axis=0)
    center_xy = anchor + ((lo_xy + hi_xy) * 0.5) @ world_to_local
    return center_xy, np.maximum(hi_xy - lo_xy, 0.0)


def _minimum_area_xy_rectangle(
    xy: np.ndarray,
    *,
    low: float,
    high: float,
    isotropic_ratio: float = 1.05,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Fit a deterministic robust minimum-area rectangle to an XY cloud.

    Hull-edge directions come from a lightly trimmed core so isolated depth
    spikes cannot invent the orientation. Every candidate is then scored with
    the requested quantiles of the complete finite cloud.
    """
    trim_low = max(0.0, min(low, 2.0))
    trim_high = min(100.0, max(high, 98.0))
    core_low = np.percentile(xy, trim_low, axis=0)
    core_high = np.percentile(xy, trim_high, axis=0)
    core = xy[np.all((xy >= core_low) & (xy <= core_high), axis=1)]
    if core.shape[0] < 3:
        core = xy
    hull = _convex_hull_xy(core)

    candidate_yaws = {0.0}
    if hull.shape[0] >= 2:
        edges = np.roll(hull, -1, axis=0) - hull
        for edge in edges:
            if float(np.linalg.norm(edge)) <= 1e-12:
                continue
            yaw = math.atan2(float(edge[1]), float(edge[0])) % (
                math.pi / 2.0
            )
            candidate_yaws.add(round(yaw, 12))

    candidates = []
    for yaw in sorted(candidate_yaws):
        center_xy, extent_xy = _xy_quantile_rectangle(
            xy,
            yaw=yaw,
            low=low,
            high=high,
        )
        canonical_yaw = _axis_yaw(yaw)
        if extent_xy[1] > extent_xy[0]:
            canonical_yaw = _axis_yaw(yaw + math.pi / 2.0)
            center_xy, extent_xy = _xy_quantile_rectangle(
                xy,
                yaw=canonical_yaw,
                low=low,
                high=high,
            )
        area = float(extent_xy[0] * extent_xy[1])
        candidates.append(
            (
                area,
                abs(canonical_yaw),
                canonical_yaw,
                center_xy,
                extent_xy,
            )
        )
    _, _, yaw, center_xy, extent_xy = min(
        candidates,
        key=lambda item: item[:3],
    )

    shortest = float(np.min(extent_xy))
    longest = float(np.max(extent_xy))
    if longest <= 1e-12 or (
        shortest > 1e-12
        and longest / shortest <= max(1.0, float(isotropic_ratio))
    ):
        # A square/circular footprint has no meaningful principal direction.
        # World-axis alignment prevents tiny sampling or sensor changes from
        # turning that ambiguity into visible yaw flips.
        yaw = 0.0
        center_xy, extent_xy = _xy_quantile_rectangle(
            xy,
            yaw=yaw,
            low=low,
            high=high,
        )
    return center_xy, extent_xy, yaw


def _robust_yaw_bbox(
    points_world: Any,
    *,
    low_percentile: float = 5.0,
    high_percentile: float = 95.0,
) -> tuple[np.ndarray, np.ndarray, float] | None:
    """Return a robust yaw-only 3D box as ``(center, extent, yaw)``."""
    points = np.asarray(points_world, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 3:
        return None
    points = points[np.all(np.isfinite(points[:, :3]), axis=1), :3]
    if points.shape[0] < 4:
        return None
    low = max(0.0, min(100.0, float(low_percentile)))
    high = max(low, min(100.0, float(high_percentile)))
    if high <= low:
        return None
    center_xy, extent_xy, yaw = _minimum_area_xy_rectangle(
        points[:, :2],
        low=low,
        high=high,
    )
    lo_z, hi_z = np.percentile(points[:, 2], [low, high])
    center = np.asarray(
        [float(center_xy[0]), float(center_xy[1]), float((lo_z + hi_z) * 0.5)]
    )
    extent = np.asarray(
        [
            float(extent_xy[0]),
            float(extent_xy[1]),
            float(hi_z - lo_z),
        ]
    )
    if not np.all(np.isfinite(center)) or not np.all(np.isfinite(extent)):
        return None
    return center, np.maximum(extent, 0.0), yaw


_DEFAULT_YOLO_WORLD_WEIGHTS = "/opt/models/yolov8l-world.pt"
_DEFAULT_MOBILE_SAM_WEIGHTS = "/opt/models/mobile_sam.pt"
_DEFAULT_CLIP_MODEL = "ViT-B-32"
_DEFAULT_CLIP_PRETRAINED = "/opt/models/open_clip_pytorch_model.bin"


# ── Concept-graphs config knobs ──────────────────────────────────────────
# These are the v0 defaults. They map to concept-graphs's own
# realtime_mapping config; documented per-knob so they're tunable
# without grepping ali-dev's hydra cfg.
_CFG_DEFAULTS = {
    # 3D pcd downsampling (Open3D voxel size, metres). 0.025 = 2.5cm
    # voxels — concept-graphs default for indoor tabletop.
    "downsample_voxel_size": 0.025,
    # DBSCAN params for pcd denoise (drop sparse outlier points).
    "dbscan_remove_noise": True,
    "dbscan_eps": 0.10,                 # 10 cm cluster radius
    "dbscan_min_points": 10,
    # Spatial similarity type. ali-dev exposes:
    #   'iou' / 'giou'                  — axis-aligned bbox IoU
    #   'iou_accurate' / 'giou_accurate' — oriented-bbox IoU
    #   'overlap'                       — voxel-grid pcd intersection
    # 'overlap' is concept-graphs's CANONICAL choice — captures dense
    # pcd similarity so cabinet+shelf at the same position merge even
    # when their AABBs differ. Concept-graphs's reference impl routes
    # through pytorch3d.ops.box3d_overlap, which raises ValueError on
    # coplanar bbox vertices (thin objects on floor) and the Open3D
    # qhull fallback segfaults on degenerate input. We don't use that
    # path: see `_voxel_pcd_overlap_matrix` below — pure numpy
    # voxel-set fraction-overlap, no pytorch3d, no qhull. The
    # `spatial_sim_type` value below is still threaded into
    # concept-graphs internals (merge_detections_to_objects,
    # denoise_objects, merge_overlap_objects); they use it only to
    # decide which similarity to recompute when caller didn't provide
    # one — we always provide our overlap matrix, so 'iou' here is a
    # safe sentinel that avoids touching the broken pytorch3d path.
    "spatial_sim_type": "iou",
    # `match_method`: how to combine spatial + visual sims.
    "match_method": "sim_sum",
    # phys_bias > 0 → trust spatial more than visual; <0 → opposite.
    # 0.0 means equal weight.
    "phys_bias": 0.0,
    # A detection is matched to an existing object only when
    # aggregated similarity >= this. Below it, the detection becomes
    # We're back on `iou` spatial sim (axis-aligned bbox IoU) since
    # the `overlap` path segfaulted via Open3D's qhull. With:
    #   spatial AABB-IoU: 0..1 (often 0 for partial views), visual
    #   cos-sim: -1..1 (typically 0.4-0.85 for same instance).
    #   sim_sum with phys_bias=0 sums them. At threshold=1.10 the
    #   same physical object seen from a new viewpoint (e.g. across
    #   the room) usually has IoU≈0 and visual≈0.6 → 0.60 < 1.10 →
    #   spawned as a NEW object. That's the "ghosting" (duplicate) the
    #   user kept catching.
    #   Lowering to 0.55 was needed when spatial sim was AABB IoU
    #   (which is ≈0 for partial-view bboxes). With voxel pcd-overlap
    #   replacing IoU (`_voxel_pcd_overlap_torch`), spatial sim alone
    #   reliably reaches 0.4–0.7 for "same physical object seen from
    #   another angle" — combined with visual sim that's 1.0–1.3 for
    #   true matches and 0.3–0.7 for spurious pairs, threshold 0.85
    #   is the sweet spot. Lower → over-merging, higher → ghosting again.
    "merge_threshold": 0.85,
    # Hard centroid-distance gate on the per-tick merge. With
    # merge_threshold low, visual similarity alone can match; this
    # caps the physical separation so two "potted_plant" detections
    # at opposite corners of the room never merge into one mega-
    # bbox (the 9 × 5 m blob the user kept seeing). 1.5 m fits "same
    # piece of furniture seen from a couple of angles" but rejects
    # "two of the same kind across the room".
    "max_merge_dist_m": 1.5,
    # Every existing track can consume at most one detection from a camera
    # frame. This prevents simultaneously visible repeated instances from
    # being irreversibly fused before ConceptGraphs' normal merge step.
    "one_to_one_association": True,
    # Tighten the room-wide cap to the measured horizontal scale of each pair.
    # The global max remains an absolute bound for large furniture.
    "adaptive_merge_distance": True,
    "adaptive_merge_min_dist_m": 0.45,
    "adaptive_merge_extent_scale": 0.80,
    # Re-identification is stricter than ordinary multi-view association.
    # A UUID that vanished on a prior tick may reclaim an old registry ID only
    # when its robust center remains within this metric distance. Reusing the
    # 1.5 m furniture-scale merge cap let unrelated shelf fragments steal
    # stale identities across a room.
    "identity_rebind_max_distance_m": 0.45,
    # Minimum 3D points after backprojection to keep a detection.
    # Filters thin/degenerate masks.
    "min_points_threshold": 50,
    # Per-object pcd cap (downsample points if exceeded). Keeps
    # memory bounded as the same object is observed many times.
    "obj_pcd_max_points": 5000,
    # Periodic cleanup interval (in ticks at ~1.6 Hz). 10 ticks = 6 s
    # cadence. Aggressive enough that cross-class duplicates (e.g.
    # cabinet + shelf labelled on the same physical object in the
    # same frame) get merged before they pollute the registry — the
    # per-tick `merge_detections_to_objects` only matches against the
    # existing map, so two BRAND-NEW detections at the same position
    # both get appended; only the periodic merge_overlap pass can
    # collapse them.
    "denoise_interval_ticks": 10,
    "merge_overlap_interval_ticks": 10,
    # Periodic-cleanup thresholds (passed to merge_overlap_objects /
    # denoise_objects in concept-graphs.utils). obj_min_points is the cull
    # gate inside filter_objects: thin/small objects (keyboard, lamp) backproject
    # to sparse clouds, so 50 culled them every cleanup tick → uuid churn →
    # observation_count reset. 20 lets them survive; override via
    # SCENE_CG_OBJ_MIN_POINTS (env table below).
    "obj_min_points": 20,
    "obj_min_detections": 1,
    # merge_overlap_thresh: spatial pcd-overlap ratio above which the
    # pair is a candidate (0..1). 0.5 = "half of A's volume is inside
    # B" — more aggressive than the canonical 0.7 because at this
    # scale and this voxel size, true duplicates often only share half
    # their voxels (noise + sampling differences).
    "merge_overlap_thresh": 0.5,
    # merge_visual_sim_thresh: required CLIP cosine similarity for the
    # pair to be considered "the same object" once spatial passes.
    # 0.65 catches cabinet↔shelf, monitor↔monitor_stand, etc. that
    # share visual features but get different YOLO-World labels.
    "merge_visual_sim_thresh": 0.65,
    "merge_text_sim_thresh": 0.0,        # we don't compute text feats
    # ── Class-agnostic geometric collapse ─────────────────────────────
    # When YOLO-World flickers between e.g. picture_frame / shelf /
    # cabinet for the same physical wall fixture, the same-class gate
    # below keeps them as 3 separate records. CLIP visual sim is also
    # too low because the cropped masks differ. Two safety nets:
    #
    #   (a) **Per-tick centroid proximity bypass.** When a new det's
    #       centroid is within `cross_class_centroid_max_m` of an
    #       existing object's centroid, we drop the same-class gate
    #       for that pair. Visual+spatial sim still has to clear
    #       merge_threshold.
    #   (b) **Periodic class-agnostic AABB-IoU collapse.** Runs on the
    #       cleanup tick AFTER the canonical merge_overlap_objects
    #       pass. Pairs whose AABB IoU >= cross_class_iou_thresh OR
    #       whose pcd-overlap ratio >= cross_class_overlap_thresh get
    #       force-merged regardless of class/visual sim. This is the
    #       only thing strong enough to undo a 3-way YOLO label split
    #       on a wall fixture.
    "cross_class_centroid_max_m": 0.5,
    "cross_class_iou_thresh": 0.30,
    "cross_class_overlap_thresh": 0.50,
    "cross_class_merge_interval_ticks": 10,
    # ── Same-class geometry-verified merge (dedup) ──────────────────────
    # Same-label tracks may still split across partial views when their CLIP
    # crops diverge. Proximity alone is unsafe: separate chairs and small
    # objects on furniture are routinely closer than 0.4 m. Reuse the robust
    # physical-identity test below and keep ConceptGraphs' own
    # merge_obj2_into_obj1 as the only actual merger.
    "same_class_centroid_max_m": 0.15,
    "same_class_min_voxel_coverage": 0.50,
    "same_class_max_extent_ratio": 1.75,
    # A same-class track may legitimately have zero voxel overlap when the
    # detector alternates between two partial views on adjacent frames.  This
    # fallback remains identity-safe by requiring multi-frame evidence on both
    # sides, disjoint histories separated by at most one perception frame,
    # close robust centers relative to object scale, compatible extents, and
    # strong accumulated CLIP similarity.
    "same_class_disjoint_min_unique_frames": 2,
    "same_class_disjoint_max_frame_gap": 1,
    "same_class_disjoint_max_center_major_extent_ratio": 0.20,
    "same_class_disjoint_min_visual_similarity": 0.85,
    # Run on every completed perception tick so a duplicate created on the
    # final frame cannot survive merely because a ten-tick timer did not fire.
    "same_class_merge_interval_ticks": 1,
}


def _parse_merge_class_groups(raw: str) -> dict[str, str]:
    """Parse SCENE_CG_MERGE_CLASS_GROUPS into a {class -> bucket-key} map.

    Groups are separated by ``;``, members within a group by ``,`` — e.g.
    ``"chair,table,desk;sofa,couch"`` maps chair/table/desk to one bucket and
    sofa/couch to another. The bucket key is the sorted member list joined by
    ``|`` (order-independent). Empty / malformed input yields an empty map, so
    the feature is off by default and never silently relabels objects."""
    out: dict[str, str] = {}
    for group in raw.split(";"):
        members = [m.strip().lower() for m in group.split(",") if m.strip()]
        if len(members) < 2:
            continue
        key = "|".join(sorted(set(members)))
        for m in members:
            out[m] = key
    return out


def _merge_class_groups_from_lists(groups: list[list[str]]) -> dict[str, str]:
    """Build the same bucket map from validated Driver configuration."""
    out: dict[str, str] = {}
    for group in groups:
        members = sorted(
            {
                str(member or "").strip().lower()
                for member in group
                if str(member or "").strip()
            }
        )
        if len(members) < 2:
            continue
        key = "|".join(members)
        for member in members:
            out[member] = key
    return out


# ── Model loading ────────────────────────────────────────────────────────
def _try_load_models(
    yolo_path,
    sam_path,
    clip_model_name,
    clip_pretrained,
    classes,
):
    """Load YOLO-World, MobileSAM, and open_clip in one shot. Returns
    `(yolo, sam, clip_model, clip_preprocess, clip_tokenizer, device)`
    or `(None, ...)` if any piece is unavailable."""
    yw = yolo_path or os.environ.get("SCENE_YOLO_WORLD_WEIGHTS",
                                     _DEFAULT_YOLO_WORLD_WEIGHTS)
    mp = sam_path or os.environ.get("SCENE_MOBILE_SAM_WEIGHTS",
                                    _DEFAULT_MOBILE_SAM_WEIGHTS)
    cm = clip_model_name or os.environ.get("SCENE_CLIP_MODEL", _DEFAULT_CLIP_MODEL)
    cp = clip_pretrained or os.environ.get("SCENE_CLIP_PRETRAINED",
                                           _DEFAULT_CLIP_PRETRAINED)
    if not os.path.isfile(yw):
        log.warning("YOLO-World weights not found at %s — perception disabled", yw)
        return (None,) * 6
    if not os.path.isfile(mp):
        log.warning("MobileSAM weights not found at %s — perception disabled", mp)
        return (None,) * 6
    try:
        import torch
        from ultralytics import YOLO, SAM
        import open_clip
    except Exception as e:  # noqa: BLE001
        log.warning("ultralytics / open_clip / torch not importable (%s) — perception disabled", e)
        return (None,) * 6

    # Default to CUDA when available, but allow forcing CPU via env.
    # The Blackwell sm_120 + Open3D + concept-graphs combination has
    # been segfaulting (SIGSEGV exit 139) under CUDA on this host;
    # SCENE_CG_FORCE_CPU=1 sidesteps it at the cost of ~3-5x slower
    # YOLO/SAM inference (still real-time at the 0.6s tick).
    if os.environ.get("SCENE_CG_FORCE_CPU", "").strip() in ("1", "true", "yes"):
        device = "cpu"
        log.warning("[scene-cg] SCENE_CG_FORCE_CPU set — running on CPU")
    else:
        device = "cuda" if torch.cuda.is_available() else "cpu"
    if device == "cpu":
        log.warning(
            "[scene-cg] CUDA not available — concept-graphs runs on CPU; "
            "expect ~5x slower ticks. Detector will still merge correctly."
        )

    try:
        yolo = YOLO(yw)
        yolo.set_classes(classes)
        log.info("[scene-cg] YOLO-World loaded: %d open-vocab classes", len(classes))
    except Exception as e:  # noqa: BLE001
        log.warning("YOLO-World load failed: %s", e)
        return (None,) * 6
    try:
        sam = SAM(mp)
        log.info("[scene-cg] MobileSAM loaded")
    except Exception as e:  # noqa: BLE001
        log.warning("MobileSAM load failed: %s", e)
        return (None,) * 6
    try:
        clip_model, _, clip_preprocess = open_clip.create_model_and_transforms(
            cm, pretrained=cp,
        )
        clip_model = clip_model.to(device).eval()
        clip_tokenizer = open_clip.get_tokenizer(cm)
        log.info("[scene-cg] open_clip %s/%s loaded on %s", cm, cp, device)
    except Exception as e:  # noqa: BLE001
        log.warning("open_clip load failed: %s", e)
        return (None,) * 6

    return yolo, sam, clip_model, clip_preprocess, clip_tokenizer, device


def _import_cg():
    """Import concept-graphs's slam modules. Deferred so that scene
    can boot even if /opt/concept-graphs isn't installed (the load_models
    path will already have failed)."""
    from conceptgraph.slam.slam_classes import MapObjectList, DetectionList
    from conceptgraph.slam.utils import (
        detections_to_obj_pcd_and_bbox,
        get_bounding_box,
        process_pcd,
    )
    from conceptgraph.slam.mapping import (
        compute_spatial_similarities,
        compute_visual_similarities,
        aggregate_similarities,
        merge_detections_to_objects,
    )
    from conceptgraph.utils.model_utils import compute_clip_features_batched
    from conceptgraph.slam.utils import (
        denoise_objects,
        filter_objects,
        merge_overlap_objects,
        compute_overlap_matrix_general,
    )
    return {
        "MapObjectList": MapObjectList,
        "DetectionList": DetectionList,
        "detections_to_obj_pcd_and_bbox": detections_to_obj_pcd_and_bbox,
        "get_bounding_box": get_bounding_box,
        "process_pcd": process_pcd,
        "compute_spatial_similarities": compute_spatial_similarities,
        "compute_visual_similarities": compute_visual_similarities,
        "aggregate_similarities": aggregate_similarities,
        "merge_detections_to_objects": merge_detections_to_objects,
        "compute_clip_features_batched": compute_clip_features_batched,
        "denoise_objects": denoise_objects,
        "filter_objects": filter_objects,
        "merge_overlap_objects": merge_overlap_objects,
        "compute_overlap_matrix_general": compute_overlap_matrix_general,
    }


# ── Detector class ───────────────────────────────────────────────────────
class ConceptGraphsDetector:
    """Per-frame detector that runs the canonical concept-graphs merge
    pipeline. Owns a persistent `MapObjectList` and projects it into
    scene's `ObjectRegistry` once per tick so the web UI/MCP API stay
    consistent.

    `on_detections` is kept for backwards compat but no longer used —
    we write to the registry directly via `_project_to_registry`. The
    caller still passes a registry via this side channel.
    """

    def __init__(
        self,
        *,
        rgb_fetcher_msg: Callable[[], Optional[Any]],
        depth_fetcher_msg: Callable[[], Optional[Any]],
        camera_info_fetcher: Callable[[], Optional[_CamIntrinsics]],
        on_detections: Callable[[list[Detection]], Awaitable[None]],
        registry: ObjectRegistry,
        # Returns the current world frame id from the active localizer.
        # An empty result means spatial projection is not ready.
        world_frame_fn: Optional[Callable[[], str]] = None,
        robot_base_frame_fn: Optional[Callable[[], str]] = None,
        period_s: float = 0.6,
        pose_max_age_s: float = 2.0,
        visible_miss_threshold: int = 3,
        visibility_depth_margin_m: float = 0.10,
        visibility_min_clear_samples: int = 3,
        visibility_min_clear_fraction: float = 0.60,
        visibility_max_projected_samples: int = 25,
        object_ttl_s: float = 30.0,
        confirmation_min_unique_frames: int = 2,
        confirmation_singleton_min_mean_confidence: float = 0.0,
        mask_erosion_px: int = 1,
        min_depth_m: float = 0.15,
        max_depth_m: float = 6.0,
        depth_mad_scale: float = 3.5,
        depth_min_band_m: float = 0.12,
        frame_dbscan: bool = True,
        require_occupancy_bounds: bool = True,
        map_bounds_margin_m: float = 0.25,
        map_max_outside_fraction: float = 0.20,
        bbox_low_percentile: float = 5.0,
        bbox_high_percentile: float = 95.0,
        max_bbox_extent_m: float = 3.0,
        rebase_map_corrections: bool = False,
        surface_snap_labels: Optional[list[str]] = None,
        surface_snap_max_distance_m: float = 0.60,
        surface_snap_tangent_padding_m: float = 0.25,
        surface_snap_min_shift_m: float = 0.05,
        surface_snap_min_support_cells: int = 30,
        surface_snap_min_dominant_share: float = 0.55,
        surface_snap_min_tangent_coverage: float = 0.50,
        surface_snap_occupancy_threshold: int = 50,
        classes: Optional[list[str]] = None,
        additional_classes: Optional[list[str]] = None,
        label_history_size: int = 20,
        label_min_switch_observations: int = 2,
        label_min_winner_share: float = 0.55,
        label_switch_margin: float = 0.15,
        label_aliases: Optional[dict[str, str]] = None,
        clip_rerank_groups: Optional[list[list[str]]] = None,
        clip_rerank_routes: Optional[dict[str, list[str]]] = None,
        clip_rerank_prompts: Optional[dict[str, list[str]]] = None,
        clip_rerank_min_score: float = 0.20,
        clip_rerank_min_margin: float = 0.04,
        clip_rerank_min_margin_by_label: Optional[dict[str, float]] = None,
        clip_rerank_geometry_bonus: float = 0.0,
        clip_rerank_geometry_constraints: Optional[
            dict[str, dict[str, Any]]
        ] = None,
        allow_cross_class_merge: Optional[bool] = None,
        confusable_class_groups: Optional[list[list[str]]] = None,
        exact_duplicate_centroid_max_m: float = 0.15,
        exact_duplicate_min_voxel_coverage: float = 0.40,
        exact_duplicate_max_extent_ratio: float = 1.50,
        coobserved_duplicate_min_shared_frames: int = 3,
        coobserved_duplicate_min_median_iou: float = 0.85,
        coobserved_duplicate_max_extent_ratio: float = 2.0,
        coobserved_duplicate_min_visual_similarity: float = 0.90,
        same_class_centroid_max_m: float = 0.15,
        same_class_min_voxel_coverage: float = 0.50,
        same_class_max_extent_ratio: float = 1.75,
        same_class_disjoint_min_unique_frames: int = 2,
        same_class_disjoint_max_frame_gap: int = 1,
        same_class_disjoint_max_center_major_extent_ratio: float = 0.20,
        same_class_disjoint_min_visual_similarity: float = 0.85,
        same_class_merge_interval_ticks: int = 1,
        identity_rebind_max_distance_m: float = 0.45,
        confidence_threshold: float = 0.30,
        max_detections: int = 30,
        yolo_weights_path: Optional[str] = None,
        sam_weights_path: Optional[str] = None,
        clip_model_name: Optional[str] = None,
        clip_pretrained: Optional[str] = None,
        cfg_overrides: Optional[dict] = None,
        # `hub` exposes the Atlas-selected ROS2 contract slots used to
        # compose the camera-to-world transform.
        hub: Any = None,
        camera_frame: str = "",
        # Body frame asserted for the compatibility pose + extrinsics chain.
        # It must match odometry.child_frame_id when odometry is selected.
        # Soma's live footprint base frame is preferred when available.
        base_frame: Optional[str] = None,
    ) -> None:
        self._rgb_msg = rgb_fetcher_msg
        self._depth_msg = depth_fetcher_msg
        self._cam_info = camera_info_fetcher
        self._on_dets = on_detections
        self._registry = registry
        self._period_s = period_s
        self._conf_thresh = confidence_threshold
        self._max_dets = max_detections
        self._yolo_weights = yolo_weights_path
        self._sam_weights = sam_weights_path
        self._clip_model_name = clip_model_name
        self._clip_pretrained = clip_pretrained
        self._hub = hub
        self._camera_frame = camera_frame
        self._base_frame = (base_frame or "").strip().lstrip("/")
        self._world_frame_fn = world_frame_fn or (lambda: "")
        self._robot_base_frame_fn = robot_base_frame_fn or (lambda: "")
        self._pose_max_age_s = max(0.0, float(pose_max_age_s))
        self._visible_miss_threshold = max(1, int(visible_miss_threshold))
        self._visibility_depth_margin_m = max(
            0.0,
            float(visibility_depth_margin_m),
        )
        self._visibility_min_clear_samples = max(
            1,
            int(visibility_min_clear_samples),
        )
        self._visibility_min_clear_fraction = max(
            0.0,
            min(1.0, float(visibility_min_clear_fraction)),
        )
        self._visibility_max_projected_samples = max(
            self._visibility_min_clear_samples,
            int(visibility_max_projected_samples),
        )
        self._object_ttl_s = max(0.0, float(object_ttl_s))
        self._confirmation_min_unique_frames = max(
            1,
            int(confirmation_min_unique_frames),
        )
        self._confirmation_singleton_min_mean_confidence = max(
            0.0,
            min(
                1.0,
                float(confirmation_singleton_min_mean_confidence),
            ),
        )
        self._mask_erosion_px = max(0, int(mask_erosion_px))
        self._min_depth_m = max(0.0, float(min_depth_m))
        self._max_depth_m = max(self._min_depth_m, float(max_depth_m))
        self._depth_mad_scale = max(0.0, float(depth_mad_scale))
        self._depth_min_band_m = max(0.0, float(depth_min_band_m))
        self._frame_dbscan = bool(frame_dbscan)
        self._require_occupancy_bounds = bool(require_occupancy_bounds)
        self._map_bounds_margin_m = max(0.0, float(map_bounds_margin_m))
        self._map_max_outside_fraction = max(
            0.0,
            min(1.0, float(map_max_outside_fraction)),
        )
        self._bbox_low_percentile = max(
            0.0,
            min(100.0, float(bbox_low_percentile)),
        )
        self._bbox_high_percentile = max(
            self._bbox_low_percentile,
            min(100.0, float(bbox_high_percentile)),
        )
        self._max_bbox_extent_m = max(0.05, float(max_bbox_extent_m))
        self._rebase_map_corrections = bool(rebase_map_corrections)
        self._surface_snap_labels = {
            str(label).strip().lower()
            for label in (surface_snap_labels or ())
            if str(label).strip()
        }
        self._surface_snap_max_distance_m = max(
            0.0,
            float(surface_snap_max_distance_m),
        )
        self._surface_snap_tangent_padding_m = max(
            0.0,
            float(surface_snap_tangent_padding_m),
        )
        self._surface_snap_min_shift_m = max(
            0.0,
            float(surface_snap_min_shift_m),
        )
        self._surface_snap_min_support_cells = max(
            1,
            int(surface_snap_min_support_cells),
        )
        self._surface_snap_min_dominant_share = max(
            0.0,
            min(1.0, float(surface_snap_min_dominant_share)),
        )
        self._surface_snap_min_tangent_coverage = max(
            0.0,
            min(1.0, float(surface_snap_min_tangent_coverage)),
        )
        self._surface_snap_occupancy_threshold = max(
            1,
            min(100, int(surface_snap_occupancy_threshold)),
        )
        self._label_history_size = max(1, int(label_history_size))
        self._label_min_switch_observations = max(
            1,
            int(label_min_switch_observations),
        )
        self._label_min_winner_share = max(
            0.0,
            min(1.0, float(label_min_winner_share)),
        )
        self._label_switch_margin = max(
            0.0,
            min(1.0, float(label_switch_margin)),
        )
        self._exact_duplicate_centroid_max_m = max(
            0.0,
            float(exact_duplicate_centroid_max_m),
        )
        self._exact_duplicate_min_voxel_coverage = max(
            0.0,
            min(1.0, float(exact_duplicate_min_voxel_coverage)),
        )
        self._exact_duplicate_max_extent_ratio = max(
            1.0,
            float(exact_duplicate_max_extent_ratio),
        )
        self._coobserved_duplicate_min_shared_frames = max(
            1,
            int(coobserved_duplicate_min_shared_frames),
        )
        self._coobserved_duplicate_min_median_iou = max(
            0.0,
            min(1.0, float(coobserved_duplicate_min_median_iou)),
        )
        self._coobserved_duplicate_max_extent_ratio = max(
            1.0,
            float(coobserved_duplicate_max_extent_ratio),
        )
        self._coobserved_duplicate_min_visual_similarity = max(
            -1.0,
            min(
                1.0,
                float(coobserved_duplicate_min_visual_similarity),
            ),
        )

        self._yolo: Any = None
        self._sam: Any = None
        self._clip_model: Any = None
        self._clip_preprocess: Any = None
        self._clip_tokenizer: Any = None
        self._device: str = "cpu"
        self._cg: Optional[dict] = None
        self._map_objects: Any = None
        self._task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()
        self._inference_lock = threading.Lock()
        self._classes = _resolved_classes(classes, additional_classes)
        if not self._classes:
            raise ValueError("open-vocabulary class list must not be empty")
        unknown_surface_snap_labels = (
            self._surface_snap_labels - set(self._classes)
        )
        if unknown_surface_snap_labels:
            raise ValueError(
                "surface snap labels reference labels outside the resolved "
                "vocabulary: "
                + ", ".join(sorted(unknown_surface_snap_labels))
            )
        self._class_index = {
            label: index for index, label in enumerate(self._classes)
        }
        self._label_aliases = {
            str(alias).strip().lower(): str(canonical).strip().lower()
            for alias, canonical in (label_aliases or {}).items()
            if str(alias).strip()
            and str(canonical).strip()
            and str(alias).strip().lower() != str(canonical).strip().lower()
        }
        unknown_alias_labels = (
            set(self._label_aliases) | set(self._label_aliases.values())
        ) - set(self._classes)
        if unknown_alias_labels:
            raise ValueError(
                "label aliases reference labels outside the resolved vocabulary: "
                + ", ".join(sorted(unknown_alias_labels))
            )
        chained_aliases = set(self._label_aliases.values()) & set(self._label_aliases)
        if chained_aliases:
            raise ValueError(
                "label alias targets must be canonical, not another alias: "
                + ", ".join(sorted(chained_aliases))
            )
        self._clip_rerank_groups = [
            tuple(
                dict.fromkeys(
                    str(label).strip().lower()
                    for label in group
                    if str(label).strip()
                )
            )
            for group in (clip_rerank_groups or ())
        ]
        if any(len(group) < 2 for group in self._clip_rerank_groups):
            raise ValueError(
                "each CLIP rerank group must contain at least two distinct "
                "non-empty labels"
            )
        flattened_clip_labels = [
            label
            for group in self._clip_rerank_groups
            for label in group
        ]
        unknown_clip_labels = set(flattened_clip_labels) - set(self._classes)
        if unknown_clip_labels:
            raise ValueError(
                "CLIP rerank groups reference labels outside the resolved "
                "vocabulary: "
                + ", ".join(sorted(unknown_clip_labels))
            )
        repeated_clip_labels = {
            label
            for label in flattened_clip_labels
            if flattened_clip_labels.count(label) > 1
        }
        if repeated_clip_labels:
            raise ValueError(
                "CLIP rerank labels may appear in only one group: "
                + ", ".join(sorted(repeated_clip_labels))
            )
        self._clip_rerank_group_by_label = {
            label: group
            for group in self._clip_rerank_groups
            for label in group
        }
        self._clip_rerank_routes = {
            str(source).strip().lower(): tuple(
                dict.fromkeys(
                    str(label).strip().lower()
                    for label in labels
                    if str(label).strip()
                )
            )
            for source, labels in (clip_rerank_routes or {}).items()
            if str(source).strip()
        }
        invalid_clip_routes = {
            source
            for source, labels in self._clip_rerank_routes.items()
            if len(labels) < 2 or source not in labels
        }
        if invalid_clip_routes:
            raise ValueError(
                "each CLIP rerank route must contain its source and at least "
                "one alternative label: "
                + ", ".join(sorted(invalid_clip_routes))
            )
        ambiguous_clip_sources = (
            set(self._clip_rerank_routes)
            & set(self._clip_rerank_group_by_label)
        )
        if ambiguous_clip_sources:
            raise ValueError(
                "CLIP rerank route sources must not also belong to symmetric "
                "groups: " + ", ".join(sorted(ambiguous_clip_sources))
            )
        configured_clip_labels = set(flattened_clip_labels)
        configured_clip_labels.update(
            label
            for labels in self._clip_rerank_routes.values()
            for label in labels
        )
        unknown_route_labels = configured_clip_labels - set(self._classes)
        if unknown_route_labels:
            raise ValueError(
                "CLIP rerank scopes reference labels outside the resolved "
                "vocabulary: " + ", ".join(sorted(unknown_route_labels))
            )
        self._clip_rerank_candidates_by_label = {
            **self._clip_rerank_group_by_label,
            **self._clip_rerank_routes,
        }
        self._clip_rerank_prompts = {
            str(label).strip().lower(): tuple(
                str(prompt).strip()
                for prompt in prompts
                if str(prompt).strip()
            )
            for label, prompts in (clip_rerank_prompts or {}).items()
        }
        unknown_prompt_labels = (
            set(self._clip_rerank_prompts)
            - configured_clip_labels
        )
        if unknown_prompt_labels:
            raise ValueError(
                "CLIP rerank prompts reference labels outside the configured "
                "groups or routes: "
                + ", ".join(sorted(unknown_prompt_labels))
            )
        empty_prompt_labels = {
            label
            for label, prompts in self._clip_rerank_prompts.items()
            if not prompts
        }
        if empty_prompt_labels:
            raise ValueError(
                "CLIP rerank prompt lists must not be empty: "
                + ", ".join(sorted(empty_prompt_labels))
            )
        self._clip_rerank_min_score = max(
            -1.0, min(1.0, float(clip_rerank_min_score))
        )
        self._clip_rerank_min_margin = max(
            0.0, min(2.0, float(clip_rerank_min_margin))
        )
        self._clip_rerank_min_margin_by_label = {
            str(label).strip().lower(): float(margin)
            for label, margin in (
                clip_rerank_min_margin_by_label or {}
            ).items()
            if str(label).strip()
        }
        unknown_margin_labels = (
            set(self._clip_rerank_min_margin_by_label)
            - set(self._clip_rerank_candidates_by_label)
        )
        if unknown_margin_labels:
            raise ValueError(
                "CLIP rerank margin overrides reference labels outside "
                "configured groups or route sources: "
                + ", ".join(sorted(unknown_margin_labels))
            )
        invalid_margins = {
            label
            for label, margin in self._clip_rerank_min_margin_by_label.items()
            if not math.isfinite(margin) or not 0.0 <= margin <= 2.0
        }
        if invalid_margins:
            raise ValueError(
                "CLIP rerank margin overrides must be finite values in "
                "[0, 2]: " + ", ".join(sorted(invalid_margins))
            )
        self._clip_rerank_geometry_bonus = float(
            clip_rerank_geometry_bonus
        )
        if not math.isfinite(self._clip_rerank_geometry_bonus) or not (
            0.0 <= self._clip_rerank_geometry_bonus <= 2.0
        ):
            raise ValueError(
                "CLIP rerank persistent geometry score bonus must be finite "
                "and in [0, 2]"
            )
        self._clip_rerank_geometry_constraints = {
            str(label).strip().lower(): {
                str(key): (
                    [
                        str(source).strip().lower()
                        for source in value
                        if str(source).strip()
                    ]
                    if str(key) == "source_labels"
                    else float(value)
                )
                for key, value in constraints.items()
            }
            for label, constraints in (
                clip_rerank_geometry_constraints or {}
            ).items()
            if str(label).strip()
        }
        unknown_geometry_labels = (
            set(self._clip_rerank_geometry_constraints)
            - configured_clip_labels
        )
        if unknown_geometry_labels:
            raise ValueError(
                "CLIP rerank persistent geometry constraints reference "
                "labels outside configured groups or routes: "
                + ", ".join(sorted(unknown_geometry_labels))
            )
        allowed_geometry_keys = {
            "source_labels",
            "min_horizontal_extent_m",
            "max_horizontal_extent_m",
        }
        for label, constraints in self._clip_rerank_geometry_constraints.items():
            unknown_keys = set(constraints) - allowed_geometry_keys
            if unknown_keys:
                raise ValueError(
                    "CLIP rerank persistent geometry constraints contain "
                    "unknown keys: " + ", ".join(sorted(unknown_keys))
                )
            source_labels = constraints.get("source_labels")
            if source_labels is not None and (
                not isinstance(source_labels, list) or not source_labels
            ):
                raise ValueError(
                    "CLIP rerank persistent geometry source_labels must be "
                    "a non-empty list"
                )
            if source_labels is not None:
                unreachable_sources = sorted(
                    source
                    for source in source_labels
                    if label not in self._clip_rerank_candidates_by_label.get(
                        source,
                        (),
                    )
                )
                if unreachable_sources:
                    raise ValueError(
                        "CLIP rerank persistent geometry source labels cannot "
                        f"reach candidate {label!r}: "
                        + ", ".join(unreachable_sources)
                    )
            metric_constraints = {
                key: value
                for key, value in constraints.items()
                if key != "source_labels"
            }
            if not metric_constraints or any(
                not math.isfinite(value) or value <= 0.0
                for value in metric_constraints.values()
            ):
                raise ValueError(
                    "CLIP rerank persistent geometry extents must be finite "
                    f"positive metres for label {label!r}"
                )
            minimum = constraints.get("min_horizontal_extent_m")
            maximum = constraints.get("max_horizontal_extent_m")
            if (
                minimum is not None
                and maximum is not None
                and minimum > maximum
            ):
                raise ValueError(
                    "CLIP rerank persistent geometry minimum must not exceed "
                    f"maximum for label {label!r}"
                )
        self._clip_rerank_text_features: dict[str, np.ndarray] = {}
        self._clip_rerank_recent: list[dict[str, Any]] = []
        self._merge_gate_diagnostics: dict[str, Any] = {}
        self._coobserved_merge_admissions: list[dict[str, Any]] = []
        self._coobserved_merge_events: list[dict[str, Any]] = []
        self._same_class_disjoint_recent: list[dict[str, Any]] = []
        self._tick_idx = 0
        # Set in start() so worker-thread `_project_to_registry` can
        # schedule the actual mutation on the asyncio loop (the registry
        # uses asyncio.Lock, not a sync lock).
        self._asyncio_loop: Optional[asyncio.AbstractEventLoop] = None
        self._asyncio_thread_id: Optional[int] = None

        self.cfg = dict(_CFG_DEFAULTS)
        self.cfg.update(
            {
                "same_class_centroid_max_m": max(
                    0.0,
                    float(same_class_centroid_max_m),
                ),
                "same_class_min_voxel_coverage": max(
                    0.0,
                    min(1.0, float(same_class_min_voxel_coverage)),
                ),
                "same_class_max_extent_ratio": max(
                    1.0,
                    float(same_class_max_extent_ratio),
                ),
                "same_class_disjoint_min_unique_frames": max(
                    1,
                    int(same_class_disjoint_min_unique_frames),
                ),
                "same_class_disjoint_max_frame_gap": max(
                    0,
                    int(same_class_disjoint_max_frame_gap),
                ),
                "same_class_disjoint_max_center_major_extent_ratio": max(
                    0.0,
                    float(
                        same_class_disjoint_max_center_major_extent_ratio
                    ),
                ),
                "same_class_disjoint_min_visual_similarity": max(
                    -1.0,
                    min(
                        1.0,
                        float(
                            same_class_disjoint_min_visual_similarity
                        ),
                    ),
                ),
                "same_class_merge_interval_ticks": max(
                    1,
                    int(same_class_merge_interval_ticks),
                ),
                "identity_rebind_max_distance_m": max(
                    0.0,
                    float(identity_rebind_max_distance_m),
                ),
            }
        )
        if cfg_overrides:
            self.cfg.update(cfg_overrides)
        # Allow env overrides for the most-tuned knobs. The merge/identity gates
        # below are exposed so the desk chair/table split and object dedup can be
        # tuned live on a running robot without a rebuild.
        for env, key, cast in (
            ("SCENE_CG_MERGE_THRESHOLD", "merge_threshold", float),
            ("SCENE_CG_VOXEL_SIZE", "downsample_voxel_size", float),
            ("SCENE_CG_MIN_POINTS", "min_points_threshold", int),
            ("SCENE_CG_OBJ_MIN_POINTS", "obj_min_points", int),
            ("SCENE_CG_OBJ_MAX_POINTS", "obj_pcd_max_points", int),
            ("SCENE_CG_MAX_MERGE_DIST_M", "max_merge_dist_m", float),
            (
                "SCENE_CG_ADAPTIVE_MERGE_MIN_DIST_M",
                "adaptive_merge_min_dist_m",
                float,
            ),
            (
                "SCENE_CG_ADAPTIVE_MERGE_EXTENT_SCALE",
                "adaptive_merge_extent_scale",
                float,
            ),
            ("SCENE_CG_CROSS_CLASS_CENTROID_MAX_M", "cross_class_centroid_max_m", float),
            ("SCENE_CG_CROSS_CLASS_IOU_THRESH", "cross_class_iou_thresh", float),
            ("SCENE_CG_CROSS_CLASS_OVERLAP_THRESH", "cross_class_overlap_thresh", float),
            ("SCENE_CG_MERGE_OVERLAP_THRESH", "merge_overlap_thresh", float),
            ("SCENE_CG_MERGE_VISUAL_SIM_THRESH", "merge_visual_sim_thresh", float),
            (
                "SCENE_CG_SAME_CLASS_CENTROID_MAX_M",
                "same_class_centroid_max_m",
                float,
            ),
            (
                "SCENE_CG_SAME_CLASS_MIN_VOXEL_COVERAGE",
                "same_class_min_voxel_coverage",
                float,
            ),
            (
                "SCENE_CG_SAME_CLASS_MAX_EXTENT_RATIO",
                "same_class_max_extent_ratio",
                float,
            ),
            (
                "SCENE_CG_SAME_CLASS_DISJOINT_MIN_UNIQUE_FRAMES",
                "same_class_disjoint_min_unique_frames",
                int,
            ),
            (
                "SCENE_CG_SAME_CLASS_DISJOINT_MAX_FRAME_GAP",
                "same_class_disjoint_max_frame_gap",
                int,
            ),
            (
                "SCENE_CG_SAME_CLASS_DISJOINT_MAX_CENTER_MAJOR_EXTENT_RATIO",
                "same_class_disjoint_max_center_major_extent_ratio",
                float,
            ),
            (
                "SCENE_CG_SAME_CLASS_DISJOINT_MIN_VISUAL_SIMILARITY",
                "same_class_disjoint_min_visual_similarity",
                float,
            ),
            (
                "SCENE_CG_SAME_CLASS_MERGE_INTERVAL_TICKS",
                "same_class_merge_interval_ticks",
                int,
            ),
            (
                "SCENE_CG_IDENTITY_REBIND_MAX_DISTANCE_M",
                "identity_rebind_max_distance_m",
                float,
            ),
            # Backwards-compatible name: it now adjusts only the robust-center
            # gate and can no longer bypass support/extent verification.
            (
                "SCENE_CG_SAME_CLASS_MERGE_DIST_M",
                "same_class_centroid_max_m",
                float,
            ),
        ):
            v = os.environ.get(env, "").strip()
            if v:
                try:
                    self.cfg[key] = cast(v)
                except ValueError:
                    pass
        for env, key in (
            ("SCENE_CG_ONE_TO_ONE_ASSOCIATION", "one_to_one_association"),
            ("SCENE_CG_ADAPTIVE_MERGE_DISTANCE", "adaptive_merge_distance"),
        ):
            value = os.environ.get(env, "").strip().lower()
            if value in {"1", "true", "yes", "on"}:
                self.cfg[key] = True
            elif value in {"0", "false", "no", "off"}:
                self.cfg[key] = False
        self.cfg["max_merge_dist_m"] = max(
            0.0,
            float(self.cfg["max_merge_dist_m"]),
        )
        self.cfg["adaptive_merge_min_dist_m"] = min(
            self.cfg["max_merge_dist_m"],
            max(0.0, float(self.cfg["adaptive_merge_min_dist_m"])),
        )
        self.cfg["adaptive_merge_extent_scale"] = max(
            0.0,
            float(self.cfg["adaptive_merge_extent_scale"]),
        )
        self.cfg["same_class_centroid_max_m"] = max(
            0.0,
            float(self.cfg["same_class_centroid_max_m"]),
        )
        self.cfg["same_class_min_voxel_coverage"] = max(
            0.0,
            min(1.0, float(self.cfg["same_class_min_voxel_coverage"])),
        )
        self.cfg["same_class_max_extent_ratio"] = max(
            1.0,
            float(self.cfg["same_class_max_extent_ratio"]),
        )
        self.cfg["same_class_disjoint_min_unique_frames"] = max(
            1,
            int(self.cfg["same_class_disjoint_min_unique_frames"]),
        )
        self.cfg["same_class_disjoint_max_frame_gap"] = max(
            0,
            int(self.cfg["same_class_disjoint_max_frame_gap"]),
        )
        self.cfg[
            "same_class_disjoint_max_center_major_extent_ratio"
        ] = max(
            0.0,
            float(
                self.cfg[
                    "same_class_disjoint_max_center_major_extent_ratio"
                ]
            ),
        )
        self.cfg["same_class_disjoint_min_visual_similarity"] = max(
            -1.0,
            min(
                1.0,
                float(
                    self.cfg[
                        "same_class_disjoint_min_visual_similarity"
                    ]
                ),
            ),
        )
        self.cfg["same_class_merge_interval_ticks"] = max(
            1,
            int(self.cfg["same_class_merge_interval_ticks"]),
        )
        self.cfg["identity_rebind_max_distance_m"] = max(
            0.0,
            min(
                float(self.cfg["max_merge_dist_m"]),
                float(self.cfg["identity_rebind_max_distance_m"]),
            ),
        )

        # uuid → registry object_id binding (see _apply_snapshot). Initialised
        # here (not lazily) so _apply_snapshot is safe to call before the first
        # _project_to_registry tick — including from unit tests.
        self._uuid_to_oid: dict[str, str] = {}
        self._operator_labels: dict[str, str] = {}
        self._operator_geometry_oids: set[str] = set()
        self._expired_uuids: set[str] = set()
        # UUIDs backed by historical ConceptGraphs geometry but currently
        # confirmed absent by repeated, unobstructed RGB-D observations.
        # Keep the binding for stable re-identification, but suppress these
        # objects from the 3D live view until they are observed again.
        self._missing_uuids: set[str] = set()
        self._quality_counters: dict[str, int] = {
            "healthy_frames": 0,
            "masks_input": 0,
            "masks_retained": 0,
            "depth_rejected_masks": 0,
            "frame_dbscan_attempts": 0,
            "frame_dbscan_filtered_detections": 0,
            "frame_dbscan_rejected_detections": 0,
            "frame_dbscan_input_points": 0,
            "frame_dbscan_output_points": 0,
            "map_bounds_rejected_detections": 0,
            "oversized_bbox_rejected_detections": 0,
            "accepted_frame_detections": 0,
            "clip_rerank_attempts": 0,
            "clip_rerank_switches": 0,
            "surface_snap_attempts": 0,
            "surface_snap_applied": 0,
            "surface_snap_apply_failures": 0,
            "map_correction_rebases": 0,
            "map_correction_rebase_failures": 0,
            "adaptive_distance_rejected_pairs": 0,
            "one_to_one_rejected_pairs": 0,
            "one_to_one_unmatched_detections": 0,
            "periodic_merge_candidate_pairs": 0,
            "periodic_merge_selected_pairs": 0,
            "periodic_merge_deferred_pairs": 0,
            "same_class_disjoint_candidate_pairs": 0,
            "same_class_disjoint_merged_pairs": 0,
            "registry_projection_failures": 0,
        }
        self._last_world_from_odom = None
        self._transform_source_counts: dict[str, int] = {}
        self._last_transform_source = ""
        self._last_camera_to_world_pose: dict[str, float] = {}
        self._last_rgb_depth_skew_s: Optional[float] = None
        self._max_rgb_depth_skew_s = 0.0
        self._surface_snap_recent: list[dict[str, Any]] = []
        self._map_correction_total_translation_m = 0.0
        self._map_correction_max_translation_m = 0.0
        self._map_correction_max_yaw_rad = 0.0
        # How long a soft-evicted (missing) registry record is kept so a
        # re-detection can re-bind it before it is hard-pruned. Decouples
        # observation_count from concept-graphs uuid churn across ticks.
        legacy_ttl = os.environ.get("SCENE_OBJECT_TTL_SEC", "").strip()
        if legacy_ttl:
            try:
                self._object_ttl_s = max(0.0, float(legacy_ttl))
            except ValueError:
                pass
        # Optional confusable-class reconciliation (off by default): maps a
        # group of classes (e.g. chair/table/desk) to one bucket so YOLO-World
        # label flicker across the group merges instead of splitting into
        # separate records. Applied only inside the same-class merge gate and
        # the same-class proximity collapse — both still distance-gated.
        if confusable_class_groups is None:
            self._merge_class_group = _parse_merge_class_groups(
                os.environ.get("SCENE_CG_MERGE_CLASS_GROUPS", "")
            )
        else:
            self._merge_class_group = _merge_class_groups_from_lists(
                confusable_class_groups
            )
            unknown_group_labels = (
                set(self._merge_class_group) - set(self._classes)
            )
            if unknown_group_labels:
                raise ValueError(
                    "confusable class groups reference labels outside the "
                    "resolved vocabulary: "
                    + ", ".join(sorted(unknown_group_labels))
                )
        if allow_cross_class_merge is None:
            allow_cross_class_merge = (
                os.environ.get("SCENE_CG_ALLOW_CROSS_CLASS_MERGE", "")
                .strip()
                .lower()
                in {"1", "true", "yes"}
            )
        self._allow_cross_class_merge = bool(allow_cross_class_merge)

    async def start(self) -> None:
        if self._task is not None:
            return
        loop = asyncio.get_running_loop()
        self._asyncio_loop = loop
        self._asyncio_thread_id = threading.get_ident()
        (
            self._yolo, self._sam,
            self._clip_model, self._clip_preprocess, self._clip_tokenizer,
            self._device,
        ) = await loop.run_in_executor(
            None, _try_load_models,
            self._yolo_weights, self._sam_weights,
            self._clip_model_name, self._clip_pretrained,
            self._classes,
        )
        if self._yolo is None:
            log.warning("ConceptGraphsDetector skipped — models unavailable")
            return
        self._prepare_clip_rerank_features()
        try:
            self._cg = _import_cg()
            self._map_objects = self._cg["MapObjectList"]()
        except Exception as e:  # noqa: BLE001
            log.warning("concept-graphs slam modules unavailable (%s) — perception disabled", e)
            return

        self._stop.clear()
        self._task = asyncio.create_task(self._loop(), name="scene-cg-detector")
        log.info(
            "ConceptGraphsDetector started (period=%.1fs, classes=%d, device=%s, "
            "voxel=%s, merge_threshold=%s)",
            self._period_s, len(self._classes), self._device,
            self.cfg["downsample_voxel_size"], self.cfg["merge_threshold"],
        )

    def _prepare_clip_rerank_features(self) -> None:
        """Build one normalized CLIP text prototype per configured label."""
        self._clip_rerank_text_features = {}
        if (
            not self._clip_rerank_candidates_by_label
            or self._clip_model is None
            or self._clip_tokenizer is None
        ):
            return
        try:
            import torch

            labels = list(
                dict.fromkeys(
                    label
                    for candidates in (
                        self._clip_rerank_candidates_by_label.values()
                    )
                    for label in candidates
                )
            )
            prompts_by_label = {
                label: self._clip_rerank_prompts.get(
                    label,
                    (f"a photo of a {label}",),
                )
                for label in labels
            }
            prompts = [
                prompt
                for label in labels
                for prompt in prompts_by_label[label]
            ]
            with torch.no_grad():
                tokens = self._clip_tokenizer(prompts).to(self._device)
                features = self._clip_model.encode_text(tokens).float()
                features = features / features.norm(dim=-1, keepdim=True)
            offset = 0
            for label in labels:
                count = len(prompts_by_label[label])
                prototype = features[offset : offset + count].mean(dim=0)
                prototype = prototype / prototype.norm()
                self._clip_rerank_text_features[label] = (
                    prototype.detach().cpu().numpy().astype(np.float32)
                )
                offset += count
            log.info(
                "[scene-cg] CLIP label rerank ready: %d group(s), "
                "%d route(s), %d label(s)",
                len(self._clip_rerank_groups),
                len(self._clip_rerank_routes),
                len(self._clip_rerank_text_features),
            )
        except Exception as exc:  # noqa: BLE001
            self._clip_rerank_text_features = {}
            log.warning("[scene-cg] CLIP label rerank disabled: %s", exc)

    def _clip_rerank_geometry_adjustments(
        self,
        current_label: str,
        measurements: Optional[dict[str, float]],
    ) -> dict[str, float]:
        """Return positive, configuration-scoped persistent geometry evidence.

        Geometry never removes score from a candidate. A bonus is available
        only when every configured SI-unit constraint for that candidate is
        satisfied by a finite persistent-object measurement.
        """
        current = str(current_label or "").strip().lower()
        candidates = getattr(
            self,
            "_clip_rerank_candidates_by_label",
            {},
        ).get(current) or ()
        bonus = float(getattr(self, "_clip_rerank_geometry_bonus", 0.0))
        if bonus <= 0.0 or not candidates or not measurements:
            return {}
        horizontal = measurements.get("horizontal_extent_m")
        if horizontal is None or not math.isfinite(float(horizontal)):
            return {}
        horizontal = float(horizontal)
        adjustments: dict[str, float] = {}
        for label in candidates:
            constraints = getattr(
                self,
                "_clip_rerank_geometry_constraints",
                {},
            ).get(label)
            if not constraints:
                continue
            source_labels = constraints.get("source_labels")
            if source_labels is not None and current not in source_labels:
                continue
            minimum = constraints.get("min_horizontal_extent_m")
            maximum = constraints.get("max_horizontal_extent_m")
            if minimum is not None and horizontal < minimum:
                continue
            if maximum is not None and horizontal > maximum:
                continue
            adjustments[label] = bonus
        return adjustments

    def _persistent_label_geometry_measurements(
        self,
        obj: Any,
        current_label: str,
    ) -> dict[str, float]:
        """Measure only geometry requested by the current rerank scope.

        The horizontal extent reuses the same robust yaw-only 5–95 percentile
        box as registry projection. This keeps the label evidence independent
        of map-grid-aligned AABBs and avoids extra work for unrelated classes.
        """
        current = str(current_label or "").strip().lower()
        candidates = getattr(
            self,
            "_clip_rerank_candidates_by_label",
            {},
        ).get(current) or ()
        geometry_constraints = getattr(
            self,
            "_clip_rerank_geometry_constraints",
            {},
        )
        if not any(
            label in geometry_constraints
            for label in candidates
        ):
            return {}
        try:
            points = np.asarray(obj["pcd"].points, dtype=np.float64)
        except (AttributeError, KeyError, TypeError, ValueError):
            return {}
        if points.ndim != 2 or points.shape[1:] != (3,):
            return {}
        points = points[np.all(np.isfinite(points), axis=1)]
        if points.shape[0] < 4:
            return {}
        bbox_result = _robust_yaw_bbox(
            points,
            low_percentile=getattr(self, "_bbox_low_percentile", 5.0),
            high_percentile=getattr(self, "_bbox_high_percentile", 95.0),
        )
        if bbox_result is None:
            return {}
        horizontal = float(np.max(bbox_result[1][:2]))
        if not math.isfinite(horizontal) or horizontal <= 0.0:
            return {}
        return {"horizontal_extent_m": horizontal}

    def _clip_rerank_label(
        self,
        current_label: str,
        image_feature: Any,
        *,
        record_quality: bool = True,
        stage: str = "detection",
        geometry_measurements: Optional[dict[str, float]] = None,
    ) -> tuple[str, dict[str, float]]:
        """Conservatively rerank within the current label's configured scope."""
        current = str(current_label or "").strip().lower()
        candidates = self._clip_rerank_candidates_by_label.get(current)
        if not candidates or not self._clip_rerank_text_features:
            return current, {}
        try:
            raw = image_feature
            if hasattr(raw, "detach"):
                raw = raw.detach()
            if hasattr(raw, "float"):
                raw = raw.float()
            if hasattr(raw, "cpu"):
                raw = raw.cpu()
            if hasattr(raw, "numpy"):
                raw = raw.numpy()
            feature = np.asarray(raw, dtype=np.float32).reshape(-1)
            norm = float(np.linalg.norm(feature))
            if not math.isfinite(norm) or norm <= 1e-9:
                return current, {}
            feature = feature / norm
            visual_scores = {
                label: float(
                    np.dot(feature, self._clip_rerank_text_features[label])
                )
                for label in candidates
                if label in self._clip_rerank_text_features
            }
            if current not in visual_scores or len(visual_scores) < 2:
                return current, visual_scores
            geometry_adjustments = self._clip_rerank_geometry_adjustments(
                current,
                geometry_measurements,
            )
            scores = {
                label: score + geometry_adjustments.get(label, 0.0)
                for label, score in visual_scores.items()
            }
            winner = max(scores, key=scores.get)
            min_margin = self._clip_rerank_min_margin_by_label.get(
                current,
                self._clip_rerank_min_margin,
            )
            switched = (
                winner != current
                and scores[winner] >= self._clip_rerank_min_score
                and scores[winner] > scores[current]
                and scores[winner] - scores[current]
                >= min_margin
            )
            if record_quality:
                self._quality_counters["clip_rerank_attempts"] += 1
                self._clip_rerank_recent.append(
                    {
                        "stage": str(stage or "detection"),
                        "candidate": current,
                        "selected": winner if switched else current,
                        "winner": winner,
                        "winner_margin": round(
                            scores[winner] - scores[current],
                            6,
                        ),
                        "min_margin": min_margin,
                        "scores": {
                            label: round(score, 6)
                            for label, score in scores.items()
                        },
                        "visual_scores": {
                            label: round(score, 6)
                            for label, score in visual_scores.items()
                        },
                        "geometry_measurements": {
                            key: round(float(value), 6)
                            for key, value in (
                                geometry_measurements or {}
                            ).items()
                        },
                        "geometry_adjustments": {
                            label: round(value, 6)
                            for label, value in geometry_adjustments.items()
                        },
                        "switched": switched,
                    }
                )
                del self._clip_rerank_recent[:-32]
                if switched:
                    self._quality_counters["clip_rerank_switches"] += 1
            if switched:
                return winner, scores
            return current, scores
        except Exception as exc:  # noqa: BLE001
            log.debug("[scene-cg] CLIP label rerank failed: %s", exc)
            return current, {}

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            await self._task
            self._task = None

    async def reset_derived_state(self) -> None:
        """Atomically drop detector-owned state after a map epoch change."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._reset_derived_state_locked)

    def _reset_derived_state_locked(self) -> None:
        with self._inference_lock:
            if self._cg is not None:
                self._map_objects = self._cg["MapObjectList"]()
            elif self._map_objects is not None:
                try:
                    self._map_objects = type(self._map_objects)()
                except TypeError:
                    self._map_objects = []
            self._uuid_to_oid.clear()
            getattr(self, "_operator_labels", {}).clear()
            getattr(self, "_operator_geometry_oids", set()).clear()
            self._expired_uuids.clear()
            self._missing_uuids.clear()
            for key in getattr(self, "_quality_counters", {}):
                self._quality_counters[key] = 0
            getattr(self, "_clip_rerank_recent", []).clear()
            getattr(self, "_surface_snap_recent", []).clear()
            self._merge_gate_diagnostics = {}
            self._coobserved_merge_admissions = []
            getattr(self, "_coobserved_merge_events", []).clear()
            getattr(self, "_same_class_disjoint_recent", []).clear()
            self._last_world_from_odom = None
            self._transform_source_counts = {}
            self._last_transform_source = ""
            self._last_camera_to_world_pose = {}
            self._last_rgb_depth_skew_s = None
            self._max_rgb_depth_skew_s = 0.0
            self._map_correction_total_translation_m = 0.0
            self._map_correction_max_translation_m = 0.0
            self._map_correction_max_yaw_rad = 0.0
            self._tick_idx = 0
            self._last_logged_total = -1
            self._spatial_not_ready_logged = False

    async def update_object_label(self, object_id: str, label: str) -> bool:
        """Install a sticky operator label in the detector-owned track."""
        loop = asyncio.get_running_loop()
        return bool(
            await loop.run_in_executor(
                None,
                self._update_object_label_locked,
                object_id,
                label,
            )
        )

    def _update_object_label_locked(self, object_id: str, label: str) -> bool:
        with self._inference_lock:
            self._operator_labels[object_id] = label
            updated = False
            for obj in self._map_objects or ():
                uuid_value = str(obj.get("id", "") or "")
                if self._uuid_to_oid.get(uuid_value) != object_id:
                    continue
                if not obj.get("operator_label"):
                    obj["operator_label_previous"] = str(
                        obj.get("class_name", "object") or "object"
                    )
                obj["operator_label"] = label
                obj["class_name"] = label
                updated = True
            return updated

    async def clear_object_label_override(self, object_id: str) -> bool:
        """Remove a runtime operator override, used to roll back failed writes."""
        loop = asyncio.get_running_loop()
        return bool(
            await loop.run_in_executor(
                None,
                self._clear_object_label_override_locked,
                object_id,
            )
        )

    def _clear_object_label_override_locked(self, object_id: str) -> bool:
        with self._inference_lock:
            existed = self._operator_labels.pop(object_id, None) is not None
            for obj in self._map_objects or ():
                uuid_value = str(obj.get("id", "") or "")
                if self._uuid_to_oid.get(uuid_value) != object_id:
                    continue
                obj.pop("operator_label", None)
                previous = str(
                    obj.pop("operator_label_previous", "") or ""
                ).strip()
                if previous:
                    obj["class_name"] = previous
                existed = True
            self._stabilize_map_labels()
            return existed

    async def update_object_geometry_override(self, object_id: str) -> bool:
        """Keep detector projection from overwriting an operator-owned bbox."""
        loop = asyncio.get_running_loop()
        return bool(
            await loop.run_in_executor(
                None,
                self._update_object_geometry_override_locked,
                object_id,
            )
        )

    def _update_object_geometry_override_locked(self, object_id: str) -> bool:
        with self._inference_lock:
            self._operator_geometry_oids.add(object_id)
            return True

    async def clear_object_geometry_override(self, object_id: str) -> bool:
        loop = asyncio.get_running_loop()
        return bool(
            await loop.run_in_executor(
                None,
                self._clear_object_geometry_override_locked,
                object_id,
            )
        )

    def _clear_object_geometry_override_locked(self, object_id: str) -> bool:
        with self._inference_lock:
            existed = object_id in self._operator_geometry_oids
            self._operator_geometry_oids.discard(object_id)
            return existed

    async def delete_object(self, object_id: str) -> bool:
        """Remove one registry-bound object from all detector-owned state."""
        loop = asyncio.get_running_loop()
        return bool(
            await loop.run_in_executor(
                None,
                self._delete_object_locked,
                object_id,
            )
        )

    def _delete_object_locked(self, object_id: str) -> bool:
        with self._inference_lock:
            self._operator_labels.pop(object_id, None)
            self._operator_geometry_oids.discard(object_id)
            uuids = {
                uuid_value
                for uuid_value, oid in self._uuid_to_oid.items()
                if oid == object_id
            }
            if not uuids or self._map_objects is None:
                return False
            if self._cg is not None:
                retained = self._cg["MapObjectList"]()
            else:
                retained = []
            for obj in self._map_objects:
                if str(obj.get("id", "") or "") not in uuids:
                    retained.append(obj)
            self._map_objects = retained
            for uuid_value in uuids:
                self._uuid_to_oid.pop(uuid_value, None)
                self._missing_uuids.discard(uuid_value)
                self._expired_uuids.discard(uuid_value)
            return True

    # ── Text embedding (shared CLIP) ─────────────────────────────────
    def embed_text(self, texts: list[str]) -> Optional[list[list[float]]]:
        """Encode `texts` with the already-loaded open_clip text encoder,
        L2-normalized to match the per-object image features. Returns one
        512-d vector per input, or None when CLIP isn't loaded (models
        unavailable / detector not started) so the caller can fall back.

        Reuses the model loaded in `start()` — no second model instance.
        Runs synchronously under torch.no_grad on `self._device`."""
        if self._clip_model is None or self._clip_tokenizer is None or not texts:
            return None
        try:
            import torch

            with self._inference_lock, torch.no_grad():
                tokens = self._clip_tokenizer(texts).to(self._device)
                feats = self._clip_model.encode_text(tokens)
                feats = feats / feats.norm(dim=-1, keepdim=True)
                return feats.cpu().tolist()
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-cg] embed_text failed: %s", e)
            return None

    def quality_metrics(self) -> dict[str, Any]:
        """Return cumulative admission counters for operator/CI reporting."""
        provisional_count = 0
        persistent_clip_count = 0
        unconfirmed_candidate_count = 0
        confirmed_candidate_count = 0
        if self._map_objects is not None:
            provisional_count = sum(
                bool(obj.get("label_provisional", True))
                for obj in self._map_objects
            )
            persistent_clip_count = sum(
                str(obj.get("label_source", "") or "") == "model_clip"
                for obj in self._map_objects
            )
            confirmation_min = int(
                getattr(self, "_confirmation_min_unique_frames", 1)
            )
            singleton_confidence = float(
                getattr(
                    self,
                    "_confirmation_singleton_min_mean_confidence",
                    0.0,
                )
            )
            unconfirmed_candidate_count = sum(
                not _object_confirmation_status(
                    obj,
                    min_unique_frames=confirmation_min,
                    singleton_min_mean_confidence=singleton_confidence,
                )[3]
                for obj in self._map_objects
            )
            confirmed_candidate_count = (
                len(self._map_objects) - unconfirmed_candidate_count
            )
        return {
            **dict(getattr(self, "_quality_counters", {})),
            "require_occupancy_bounds": bool(
                getattr(self, "_require_occupancy_bounds", False)
            ),
            "frame_dbscan": bool(getattr(self, "_frame_dbscan", False)),
            "depth_range_m": [
                float(getattr(self, "_min_depth_m", 0.0)),
                float(getattr(self, "_max_depth_m", 0.0)),
            ],
            "confidence_threshold": float(
                getattr(self, "_conf_thresh", 0.0)
            ),
            "max_detections": int(getattr(self, "_max_dets", 0)),
            "max_bbox_extent_m": float(
                getattr(self, "_max_bbox_extent_m", 0.0)
            ),
            "rebase_map_corrections": bool(
                getattr(self, "_rebase_map_corrections", False)
            ),
            "transform_source_counts": dict(
                getattr(self, "_transform_source_counts", {})
            ),
            "last_transform_source": str(
                getattr(self, "_last_transform_source", "")
            ),
            "last_camera_to_world_pose": dict(
                getattr(self, "_last_camera_to_world_pose", {})
            ),
            "last_rgb_depth_skew_s": getattr(
                self, "_last_rgb_depth_skew_s", None
            ),
            "max_rgb_depth_skew_s": float(
                getattr(self, "_max_rgb_depth_skew_s", 0.0)
            ),
            "map_correction_total_translation_m": float(
                getattr(self, "_map_correction_total_translation_m", 0.0)
            ),
            "map_correction_max_translation_m": float(
                getattr(self, "_map_correction_max_translation_m", 0.0)
            ),
            "map_correction_max_yaw_rad": float(
                getattr(self, "_map_correction_max_yaw_rad", 0.0)
            ),
            "map_bounds_margin_m": float(
                getattr(self, "_map_bounds_margin_m", 0.0)
            ),
            "label_provisional_objects": provisional_count,
            "confirmation_min_unique_frames": int(
                getattr(self, "_confirmation_min_unique_frames", 1)
            ),
            "confirmation_singleton_min_mean_confidence": float(
                getattr(
                    self,
                    "_confirmation_singleton_min_mean_confidence",
                    0.0,
                )
            ),
            "unconfirmed_candidate_objects": unconfirmed_candidate_count,
            "confirmed_candidate_objects": confirmed_candidate_count,
            "clip_rerank_persistent_objects": persistent_clip_count,
            "label_history_size": int(
                getattr(self, "_label_history_size", 0)
            ),
            "allow_cross_class_merge": bool(
                getattr(self, "_allow_cross_class_merge", False)
            ),
            "association": {
                "one_to_one": bool(
                    getattr(self, "cfg", {}).get(
                        "one_to_one_association",
                        False,
                    )
                ),
                "adaptive_distance": bool(
                    getattr(self, "cfg", {}).get(
                        "adaptive_merge_distance",
                        False,
                    )
                ),
                "identity_rebind_max_distance_m": float(
                    getattr(self, "cfg", {}).get(
                        "identity_rebind_max_distance_m",
                        0.0,
                    )
                ),
                "global_max_distance_m": float(
                    getattr(self, "cfg", {}).get("max_merge_dist_m", 0.0)
                ),
                "adaptive_min_distance_m": float(
                    getattr(self, "cfg", {}).get(
                        "adaptive_merge_min_dist_m",
                        0.0,
                    )
                ),
                "adaptive_extent_scale": float(
                    getattr(self, "cfg", {}).get(
                        "adaptive_merge_extent_scale",
                        0.0,
                    )
                ),
                "same_class_disjoint": {
                    "min_unique_frames": int(
                        getattr(self, "cfg", {}).get(
                            "same_class_disjoint_min_unique_frames",
                            0,
                        )
                    ),
                    "max_frame_gap": int(
                        getattr(self, "cfg", {}).get(
                            "same_class_disjoint_max_frame_gap",
                            0,
                        )
                    ),
                    "max_center_major_extent_ratio": float(
                        getattr(self, "cfg", {}).get(
                            "same_class_disjoint_max_center_major_extent_ratio",
                            0.0,
                        )
                    ),
                    "min_visual_similarity": float(
                        getattr(self, "cfg", {}).get(
                            "same_class_disjoint_min_visual_similarity",
                            0.0,
                        )
                    ),
                    "recent_merges": copy.deepcopy(
                        getattr(
                            self,
                            "_same_class_disjoint_recent",
                            (),
                        )
                    ),
                },
            },
            "clip_rerank_group_count": len(
                getattr(self, "_clip_rerank_groups", ())
            ),
            "clip_rerank_route_count": len(
                getattr(self, "_clip_rerank_routes", {})
            ),
            "clip_rerank_ready_label_count": len(
                getattr(self, "_clip_rerank_text_features", {})
            ),
            "clip_rerank_min_score": float(
                getattr(self, "_clip_rerank_min_score", 0.0)
            ),
            "clip_rerank_min_margin": float(
                getattr(self, "_clip_rerank_min_margin", 0.0)
            ),
            "clip_rerank_min_margin_by_label": copy.deepcopy(
                getattr(self, "_clip_rerank_min_margin_by_label", {})
            ),
            "clip_rerank_persistent_geometry": {
                "score_bonus": float(
                    getattr(self, "_clip_rerank_geometry_bonus", 0.0)
                ),
                "labels": copy.deepcopy(
                    getattr(
                        self,
                        "_clip_rerank_geometry_constraints",
                        {},
                    )
                ),
            },
            "clip_rerank_recent": copy.deepcopy(
                getattr(self, "_clip_rerank_recent", ())
            ),
            "surface_snap": {
                "labels": sorted(
                    getattr(self, "_surface_snap_labels", ())
                ),
                "max_distance_m": float(
                    getattr(self, "_surface_snap_max_distance_m", 0.0)
                ),
                "tangent_padding_m": float(
                    getattr(self, "_surface_snap_tangent_padding_m", 0.0)
                ),
                "min_shift_m": float(
                    getattr(self, "_surface_snap_min_shift_m", 0.0)
                ),
                "min_support_cells": int(
                    getattr(self, "_surface_snap_min_support_cells", 0)
                ),
                "min_dominant_share": float(
                    getattr(
                        self,
                        "_surface_snap_min_dominant_share",
                        0.0,
                    )
                ),
                "min_tangent_coverage": float(
                    getattr(
                        self,
                        "_surface_snap_min_tangent_coverage",
                        0.0,
                    )
                ),
                "occupancy_threshold": int(
                    getattr(
                        self,
                        "_surface_snap_occupancy_threshold",
                        0,
                    )
                ),
                "recent": copy.deepcopy(
                    getattr(self, "_surface_snap_recent", ())
                ),
            },
            "merge_gate_diagnostics": {
                **copy.deepcopy(
                    getattr(self, "_merge_gate_diagnostics", {})
                ),
                "recent_coobserved_merge_events": copy.deepcopy(
                    getattr(self, "_coobserved_merge_events", ())
                ),
            },
        }

    # ── External viz access ──────────────────────────────────────────
    # Web UI's `/3d` page calls into here every frame. Held under the
    # detector's tick lock so we never serialize a half-mutated object.
    # Each object is downsampled to at most `max_points_per_obj` for
    # JSON-friendly transport (Float32Array as base64 would be denser
    # but the JSON form keeps the client trivial — plain `fetch`).
    def export_3d_snapshot(
        self,
        max_points_per_obj: int = 256,
        *,
        include_clip_feature: bool = False,
    ) -> dict:
        """Export visualization geometry and optional bounded debug features.

        CLIP features are deliberately opt-in: the normal 3D UI should not pay
        the payload cost, while a benchmark can explicitly request the
        normalized persistent-map feature used for label analysis.
        """
        if self._map_objects is None:
            return {"objects": [], "stamp_unix": time.time()}
        try:
            import numpy as np
        except Exception:
            return {"objects": [], "stamp_unix": time.time()}
        out = []
        # We don't acquire the inference lock here — that runs on a
        # worker thread and can block for ~100ms during YOLO/SAM. The
        # MapObjectList is appended/replaced atomically per-tick; a
        # stale read between ticks is fine for visualization.
        try:
            map_objs = list(self._map_objects)
        except Exception:
            return {"objects": [], "stamp_unix": time.time()}
        # Filter to only objects that have made it into the registry —
        # `_uuid_to_oid` is the cache `_apply_snapshot` populates, so
        # this is the same set the 2D web UI / MCP API see. Without
        # this filter the 3D view shows every transient MapObjectList
        # entry (including ones that just got created and not yet
        # projected, or ones that periodic cleanup is about to evict),
        # which never matches the 2D view's count.
        live_uuids = getattr(self, "_uuid_to_oid", None)
        missing_uuids = getattr(self, "_missing_uuids", set())
        operator_geometry_oids = getattr(
            self,
            "_operator_geometry_oids",
            set(),
        )
        for obj_idx, obj in enumerate(map_objs):
            object_uuid = str(obj.get("id", f"obj_{obj_idx}"))
            published_oid = (
                live_uuids.get(object_uuid)
                if live_uuids is not None and object_uuid
                else None
            )
            published_to_registry = (
                live_uuids is None
                or bool(
                    published_oid
                    and object_uuid not in missing_uuids
                )
            )
            # Normal product/UI snapshots expose confirmed persistent objects.
            # The explicit debug endpoint retains every raw ConceptGraphs
            # candidate so evaluation can inspect what confirmation withheld.
            if not include_clip_feature and not published_to_registry:
                continue
            if live_uuids is not None:
                if published_oid in operator_geometry_oids:
                    # The web layer emits the operator bbox from the registry.
                    # Keeping the old RGB-D cloud here would visually contradict
                    # the explicit correction and falsely imply measured support.
                    continue
            try:
                pcd = obj.get("pcd")
                bbox = obj.get("bbox")
                if pcd is None or bbox is None:
                    continue
                pts = np.asarray(pcd.points)
                cols = np.asarray(pcd.colors) if hasattr(pcd, "colors") else None
                if pts.size == 0:
                    continue
                # Drop NaN/Inf rows — depth back-projection produces
                # Inf values when depth==0 (sky/uncovered pixels) and
                # NaN when the camera matrix is singular. JSON refuses
                # them ("Out of range float values are not JSON
                # compliant"). Strip before sampling.
                finite_mask = np.all(np.isfinite(pts), axis=1)
                if not finite_mask.all():
                    pts = pts[finite_mask]
                    if cols is not None and cols.size:
                        cols = cols[finite_mask]
                if pts.shape[0] == 0:
                    continue
                # Geometry must use the complete finite cloud. The random
                # sample below is transport-only and must not make the box
                # change between otherwise identical snapshot requests.
                bbox_result = _robust_yaw_bbox(
                    pts,
                    low_percentile=getattr(self, "_bbox_low_percentile", 5.0),
                    high_percentile=getattr(self, "_bbox_high_percentile", 95.0),
                )
                if bbox_result is None:
                    continue
                if pts.shape[0] > max_points_per_obj:
                    idx = np.random.choice(
                        pts.shape[0],
                        size=max_points_per_obj,
                        replace=False,
                    )
                    pts = pts[idx]
                    if cols is not None and cols.size:
                        cols = cols[idx]
                center, extent, yaw = bbox_result
                cos_y = float(np.cos(yaw))
                sin_y = float(np.sin(yaw))
                rot_xy = np.array([[cos_y, sin_y], [-sin_y, cos_y]])
                hx, hy, hz = (float(v) * 0.5 for v in extent)
                local_corners = np.array([
                    [-hx, -hy, -hz], [hx, -hy, -hz],
                    [-hx, hy, -hz], [-hx, -hy, hz],
                    [hx, hy, hz], [-hx, hy, hz],
                    [hx, -hy, hz], [hx, hy, -hz],
                ])
                corners = np.zeros_like(local_corners)
                world_xy = local_corners[:, :2] @ rot_xy
                corners[:, 0] = world_xy[:, 0] + center[0]
                corners[:, 1] = world_xy[:, 1] + center[1]
                corners[:, 2] = local_corners[:, 2] + center[2]
                bbox_corners_arr = corners
                if not np.all(np.isfinite(bbox_corners_arr)):
                    continue
                bbox_corners = bbox_corners_arr.tolist()  # 8x3
                inst_color = obj.get("inst_color")
                if inst_color is None:
                    inst_color = [0.5, 0.5, 0.5]
                else:
                    inst_color = [float(v) for v in inst_color]
                # Periodic cleanup temporarily removes Robonix-derived label
                # fields before calling ConceptGraphs' canonical merger.  The
                # live 3D/debug endpoint intentionally does not take the
                # inference lock, so a read in that short window used to emit
                # stale ``label_evidence_count=0`` metadata even though the
                # registry snapshot immediately afterwards was correct.
                #
                # Recompute the bounded label view from the authoritative
                # class/confidence histories for this snapshot.  This is
                # read-only, inexpensive, and keeps the debug endpoint
                # trustworthy without blocking RGB-D inference.
                if hasattr(self, "_classes"):
                    label_metadata = (
                        self._resolved_object_label_metadata(obj)
                    )
                else:
                    # Lightweight ``__new__`` detector fixtures and external
                    # embedders may not install the label resolver.
                    label_metadata = {
                        "label": obj.get("class_name", "object"),
                        "confidence": float(
                            obj.get("label_confidence", 0.0) or 0.0
                        ),
                        "provisional": bool(
                            obj.get("label_provisional", True)
                        ),
                        "evidence_count": int(
                            obj.get("label_evidence_count", 0) or 0
                        ),
                        "candidates": list(
                            obj.get("label_candidates", ()) or ()
                        ),
                        "source": str(
                            obj.get("label_source", "model") or "model"
                        ),
                    }
                record = {
                    "id": object_uuid,
                    # Match the ObjectRegistry/MCP-facing spelling used by the
                    # 2D state endpoint (for example ``desk`` -> ``table`` and
                    # spaces -> underscores) while retaining the raw ranked
                    # detector labels in ``label_candidates``.
                    "cls": _canon_class(label_metadata["label"]),
                    "label_confidence": float(
                        label_metadata["confidence"]
                    ),
                    "label_provisional": bool(
                        label_metadata["provisional"]
                    ),
                    "label_evidence_count": int(
                        label_metadata["evidence_count"]
                    ),
                    "label_candidates": list(
                        label_metadata["candidates"]
                    ),
                    "label_source": str(
                        label_metadata["source"]
                    ),
                    "num_detections": int(obj.get("num_detections", 1)),
                    "n_points": int(obj.get("n_points", pts.shape[0])),
                    "conf_mean": (float(np.mean(obj["conf"])) if obj.get("conf") else 0.0),
                    # Center + size for HUD; clients can also derive from corners.
                    "center": center.tolist(),
                    "bbox_corners": bbox_corners,
                    "inst_color": inst_color,
                    "points": pts.tolist(),
                    "point_colors": cols.tolist() if (cols is not None and cols.size) else None,
                }
                if include_clip_feature:
                    confirmation_min = int(
                        getattr(
                            self,
                            "_confirmation_min_unique_frames",
                            1,
                        )
                    )
                    singleton_confidence = float(
                        getattr(
                            self,
                            "_confirmation_singleton_min_mean_confidence",
                            0.0,
                        )
                    )
                    (
                        unique_frame_count,
                        confirmation_mean_confidence,
                        confirmation_confidence_fast_path,
                        confirmation_ready,
                    ) = _object_confirmation_status(
                        obj,
                        min_unique_frames=confirmation_min,
                        singleton_min_mean_confidence=singleton_confidence,
                    )
                    record["published_to_registry"] = bool(
                        published_to_registry
                    )
                    record["confirmation_ready"] = confirmation_ready
                    record["confirmation_unique_frames"] = (
                        unique_frame_count
                    )
                    record["confirmation_min_unique_frames"] = (
                        confirmation_min
                    )
                    record["confirmation_mean_confidence"] = round(
                        confirmation_mean_confidence,
                        6,
                    )
                    record["confirmation_confidence_fast_path"] = (
                        confirmation_confidence_fast_path
                    )
                    record[
                        "confirmation_singleton_min_mean_confidence"
                    ] = singleton_confidence
                    record["visibility_debug"] = copy.deepcopy(
                        getattr(self, "_visibility_diagnostics", {}).get(
                            object_uuid,
                            {},
                        )
                    )
                    feature = obj.get("clip_ft")
                    if hasattr(feature, "detach"):
                        feature = feature.detach().float().cpu().numpy()
                    if feature is not None:
                        feature = np.asarray(feature, dtype=np.float32).reshape(-1)
                        if (
                            0 < feature.size <= 4096
                            and np.all(np.isfinite(feature))
                        ):
                            norm = float(np.linalg.norm(feature))
                            if math.isfinite(norm) and norm > 1e-9:
                                record["clip_feature"] = (
                                    feature / norm
                                ).astype(float).tolist()
                    if live_uuids is not None:
                        record["registry_id"] = published_oid
                    # Bounded identity diagnostics for offline merge analysis.
                    # Shared frame ids prove that two tracks were observed
                    # simultaneously and therefore must not be collapsed merely
                    # because their 3D boxes overlap (for example, a cup on a
                    # table). Disjoint histories do not prove sameness, but make
                    # a cross-label duplicate hypothesis testable in the next
                    # benchmark capture.
                    raw_image_indices = list(
                        obj.get("image_idx", ()) or ()
                    )
                    image_indices = []
                    for raw_index in raw_image_indices[-256:]:
                        try:
                            image_indices.append(int(raw_index))
                        except (TypeError, ValueError):
                            continue
                    record["image_indices"] = sorted(set(image_indices))
                    observation_history = []
                    if hasattr(self, "_classes"):
                        observation_history = _bounded_observation_history(
                            obj,
                            self._classes,
                            limit=64,
                        )
                    record["observation_history"] = observation_history
                    record["class_history"] = [
                        {
                            "frame": item["frame"],
                            "label": item.get("label"),
                            "confidence": item.get("confidence"),
                        }
                        for item in observation_history
                        if item.get("label")
                    ]
                out.append(record)
            except Exception:  # noqa: BLE001
                continue
        snapshot = {"objects": out, "stamp_unix": time.time()}
        if include_clip_feature:
            snapshot["debug_clip_features"] = True
        return snapshot

    # ── frame bundle (for the scene-graph image relation pass) ────────
    def latest_frame_bundle(self):
        """Return ``(rgb_bgr, K, T_cam_map)`` for projecting map-frame points
        into the current camera image, or None when any piece is unavailable.

        Consumed by the scene-graph builder's image-grounded relation pass.
        Reads the latest RGB frame, intrinsics, and camera→map transform
        WITHOUT holding ``_inference_lock`` — same rationale as
        ``export_3d_snapshot``: the lock is held by the worker tick for ~100 ms
        of YOLO/SAM and the asyncio-loop caller must not block on it.  The
        transform is nevertheless resolved at this RGB frame's acquisition
        timestamp; using the newest robot pose for an older frame corrupts
        spatial relations while the robot is moving."""
        rgb_msg = self._rgb_msg()
        if rgb_msg is None:
            return None
        rgb = _image_msg_to_bgr(rgb_msg)
        if rgb is None:
            return None
        K = self._cam_info()
        if K is None or K.fx <= 0 or K.fy <= 0:
            return None
        try:
            T = self._build_camera_to_map_transform(
                stamp=self._message_stamp(rgb_msg),
            )
        except Exception as e:  # noqa: BLE001
            log.debug("[scene-cg] frame bundle: transform unavailable: %s", e)
            return None
        if T is None:
            return None
        return rgb, K, T

    # ── tick loop ─────────────────────────────────────────────────────
    async def _loop(self) -> None:
        loop = asyncio.get_running_loop()
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                await loop.run_in_executor(None, self._tick_once)
            except Exception as e:  # noqa: BLE001
                log.exception("cg tick error: %s", e)
            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0.0, self._period_s - elapsed))

    def _tick_once(self) -> None:
        with self._inference_lock:
            self._tick_locked()

    def _association_group(self, label: str) -> str:
        normalized = str(label or "").strip().lower()
        normalized = self._label_aliases.get(normalized, normalized)
        return self._merge_class_group.get(normalized, normalized)

    def _association_compatible(self, left: str, right: str) -> bool:
        """Return whether two labels may share one physical-object track."""
        if self._allow_cross_class_merge:
            return True
        left_group = self._association_group(left)
        right_group = self._association_group(right)
        return bool(left_group and left_group == right_group)

    def _exact_duplicate_geometry_matrices(
        self,
        objects_a,
        objects_b=None,
        *,
        centroid_max_m: Optional[float] = None,
        min_voxel_coverage: Optional[float] = None,
        max_extent_ratio: Optional[float] = None,
    ):
        """Identify only near-identical physical geometry across labels.

        This is deliberately stricter than ConceptGraphs association. It is
        not a replacement clustering algorithm: the result only prevents the
        class-safety mask from hiding a pair from ConceptGraphs' existing
        overlap + CLIP merge pass. Both clouds must cover almost the same
        voxel set, have almost the same robust extents, and share a centroid.
        A small object resting on a larger object therefore cannot pass merely
        because its cloud is contained by the larger one.
        """
        import numpy as np

        symmetric = objects_b is None
        right_objects = objects_a if symmetric else objects_b
        if right_objects is None:
            right_objects = ()

        voxel_size = max(
            1e-6,
            float(self.cfg.get("downsample_voxel_size", 0.025)),
        )
        centroid_limit = (
            self._exact_duplicate_centroid_max_m
            if centroid_max_m is None
            else max(0.0, float(centroid_max_m))
        )
        coverage_limit = (
            self._exact_duplicate_min_voxel_coverage
            if min_voxel_coverage is None
            else max(0.0, min(1.0, float(min_voxel_coverage)))
        )
        extent_ratio_limit = (
            self._exact_duplicate_max_extent_ratio
            if max_extent_ratio is None
            else max(1.0, float(max_extent_ratio))
        )

        def stats(obj):
            try:
                points = np.asarray(obj["pcd"].points, dtype=np.float64)
            except (KeyError, TypeError, ValueError):
                points = np.empty((0, 3), dtype=np.float64)
            if points.ndim != 2 or points.shape[1] != 3:
                points = np.empty((0, 3), dtype=np.float64)
            points = points[np.all(np.isfinite(points), axis=1)]
            if points.shape[0] < 4:
                return None
            voxels = frozenset(
                map(
                    tuple,
                    np.floor(points / voxel_size).astype(np.int64).tolist(),
                )
            )
            if not voxels:
                return None
            low = np.percentile(points, 5.0, axis=0)
            high = np.percentile(points, 95.0, axis=0)
            extent = np.maximum(high - low, voxel_size)
            center = (low + high) * 0.5
            return voxels, center, extent

        left_stats = [stats(obj) for obj in objects_a]
        right_stats = (
            left_stats
            if symmetric
            else [stats(obj) for obj in right_objects]
        )
        voxel_neighbours = tuple(
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
        )

        def covered_fraction(source, target):
            if not source or not target:
                return 0.0
            covered = 0
            for x, y, z in source:
                if any(
                    (x + dx, y + dy, z + dz) in target
                    for dx, dy, dz in voxel_neighbours
                ):
                    covered += 1
            return covered / len(source)

        result = np.zeros(
            (len(left_stats), len(right_stats)),
            dtype=bool,
        )
        coverage_result = np.zeros(
            (len(left_stats), len(right_stats)),
            dtype=np.float32,
        )
        for i, left in enumerate(left_stats):
            if left is None:
                continue
            start = i + 1 if symmetric else 0
            left_voxels, left_center, left_extent = left
            for j in range(start, len(right_stats)):
                right = right_stats[j]
                if right is None:
                    continue
                right_voxels, right_center, right_extent = right
                if (
                    float(np.linalg.norm(left_center - right_center))
                    > centroid_limit
                ):
                    continue
                extent_ratio = np.maximum(
                    left_extent / right_extent,
                    right_extent / left_extent,
                )
                if np.any(
                    extent_ratio > extent_ratio_limit
                ):
                    continue
                coverage = min(
                    covered_fraction(left_voxels, right_voxels),
                    covered_fraction(right_voxels, left_voxels),
                )
                coverage_result[i, j] = coverage
                if symmetric:
                    coverage_result[j, i] = coverage
                if coverage < coverage_limit:
                    continue
                result[i, j] = True
                if symmetric:
                    result[j, i] = True
        return result, coverage_result

    def _exact_duplicate_geometry_matrix(
        self,
        objects_a,
        objects_b=None,
        **thresholds,
    ):
        """Return the admissibility mask from
        :meth:`_exact_duplicate_geometry_matrices`.

        The split helper keeps the original boolean API for tests and callers,
        while the association path also consumes the measured tolerant voxel
        coverage as its ConceptGraphs spatial score. Without that score, a
        cross-view pair may pass the physical-identity gate but still fail the
        upstream merge because exact voxel intersection is near zero.
        """
        result, _coverage = self._exact_duplicate_geometry_matrices(
            objects_a,
            objects_b,
            **thresholds,
        )
        return result

    def _record_merge_gate_diagnostics(
        self,
        objects,
        *,
        overlap_matrix: np.ndarray,
        exact_duplicates: np.ndarray,
        tolerant_coverage: np.ndarray,
    ) -> np.ndarray:
        """Snapshot why nearby ConceptGraphs tracks do or do not merge.

        This is observation only: it never changes the matrices handed to
        ConceptGraphs. Details are bounded so `/api/state` remains small.
        """
        rows: list[dict[str, Any]] = []
        counts = {
            "nearby_pairs": 0,
            "class_compatible_pairs": 0,
            "exact_duplicate_pairs": 0,
            "overlap_gate_pairs": 0,
            "visual_gate_pairs": 0,
            "canonical_eligible_pairs": 0,
            "cross_class_geometry_rejected_pairs": 0,
            "coobserved_pairs": 0,
            "disjoint_frame_history_pairs": 0,
            "coobserved_high_2d_overlap_pairs": 0,
            "coobserved_duplicate_pairs": 0,
        }
        coobserved_duplicates = np.zeros(
            (len(objects), len(objects)),
            dtype=bool,
        )
        max_distance = float(self.cfg["max_merge_dist_m"])
        overlap_threshold = float(self.cfg["merge_overlap_thresh"])
        visual_threshold = float(self.cfg["merge_visual_sim_thresh"])
        centroid_threshold = float(self._exact_duplicate_centroid_max_m)
        coverage_threshold = float(self._exact_duplicate_min_voxel_coverage)
        extent_threshold = float(self._exact_duplicate_max_extent_ratio)
        coobserved_min_frames = int(
            self._coobserved_duplicate_min_shared_frames
        )
        coobserved_min_iou = float(
            self._coobserved_duplicate_min_median_iou
        )
        coobserved_extent_threshold = float(
            self._coobserved_duplicate_max_extent_ratio
        )
        coobserved_visual_threshold = float(
            self._coobserved_duplicate_min_visual_similarity
        )

        def geometry(obj):
            try:
                points = np.asarray(obj["pcd"].points, dtype=np.float64)
            except (KeyError, TypeError, ValueError):
                return None
            points = points[
                np.all(np.isfinite(points), axis=1)
            ] if points.ndim == 2 and points.shape[1:] == (3,) else np.empty(
                (0, 3), dtype=np.float64
            )
            if points.shape[0] < 4:
                return None
            low = np.percentile(points, 5.0, axis=0)
            high = np.percentile(points, 95.0, axis=0)
            return (
                (low + high) * 0.5,
                np.maximum(
                    high - low,
                    float(self.cfg["downsample_voxel_size"]),
                ),
            )

        def clip_cosine(left, right) -> Optional[float]:
            try:
                a = np.asarray(left["clip_ft"].detach().cpu(), dtype=float)
                b = np.asarray(right["clip_ft"].detach().cpu(), dtype=float)
            except AttributeError:
                try:
                    a = np.asarray(left["clip_ft"], dtype=float)
                    b = np.asarray(right["clip_ft"], dtype=float)
                except (KeyError, TypeError, ValueError):
                    return None
            a = a.reshape(-1)
            b = b.reshape(-1)
            denom = float(np.linalg.norm(a) * np.linalg.norm(b))
            return None if denom <= 1e-12 else float(np.dot(a, b) / denom)

        def bbox_iou(left_box, right_box) -> float:
            lx0, ly0, lx1, ly1 = left_box
            rx0, ry0, rx1, ry1 = right_box
            intersection = max(0.0, min(lx1, rx1) - max(lx0, rx0)) * max(
                0.0,
                min(ly1, ry1) - max(ly0, ry0),
            )
            left_area = max(0.0, lx1 - lx0) * max(0.0, ly1 - ly0)
            right_area = max(0.0, rx1 - rx0) * max(0.0, ry1 - ry0)
            union = left_area + right_area - intersection
            return intersection / union if union > 1e-9 else 0.0

        def shared_frame_box_ious(
            left,
            right,
        ) -> list[tuple[int, float]]:
            left_by_frame: dict[int, list[list[float]]] = {}
            right_by_frame: dict[int, list[list[float]]] = {}
            for item in _bounded_observation_history(
                left,
                self._classes,
                limit=256,
            ):
                left_by_frame.setdefault(item["frame"], []).append(
                    item["bbox_xyxy"]
                )
            for item in _bounded_observation_history(
                right,
                self._classes,
                limit=256,
            ):
                right_by_frame.setdefault(item["frame"], []).append(
                    item["bbox_xyxy"]
                )
            result: list[tuple[int, float]] = []
            for frame in sorted(left_by_frame.keys() & right_by_frame.keys()):
                result.append(
                    (
                        frame,
                        max(
                            bbox_iou(left_box, right_box)
                            for left_box in left_by_frame[frame]
                            for right_box in right_by_frame[frame]
                        ),
                    )
                )
            return result

        stats = [geometry(obj) for obj in objects]
        for i, left in enumerate(objects):
            if stats[i] is None:
                continue
            for j in range(i + 1, len(objects)):
                if stats[j] is None:
                    continue
                right = objects[j]
                center_distance = float(
                    np.linalg.norm(stats[i][0] - stats[j][0])
                )
                if center_distance > max_distance:
                    continue
                counts["nearby_pairs"] += 1
                extent_ratio = float(
                    np.max(
                        np.maximum(
                            stats[i][1] / stats[j][1],
                            stats[j][1] / stats[i][1],
                        )
                    )
                )
                coverage = float(tolerant_coverage[i, j])
                coverage_evaluated = (
                    center_distance <= centroid_threshold
                    and extent_ratio <= extent_threshold
                )
                overlap = float(overlap_matrix[i, j])
                visual = clip_cosine(left, right)
                compatible = self._association_compatible(
                    left.get("class_name", ""),
                    right.get("class_name", ""),
                )
                exact = bool(exact_duplicates[i, j])
                left_frames = {
                    int(value)
                    for value in list(left.get("image_idx", ()) or ())
                    if isinstance(value, (int, np.integer))
                }
                right_frames = {
                    int(value)
                    for value in list(right.get("image_idx", ()) or ())
                    if isinstance(value, (int, np.integer))
                }
                shared_frames = left_frames & right_frames
                shared_2d_evidence = shared_frame_box_ious(left, right)
                shared_2d_ious = [
                    value for _frame, value in shared_2d_evidence
                ]
                max_shared_2d_iou = (
                    max(shared_2d_ious) if shared_2d_ious else 0.0
                )
                median_shared_2d_iou = (
                    float(np.median(shared_2d_ious))
                    if shared_2d_ious
                    else 0.0
                )
                overlap_pass = overlap > overlap_threshold
                visual_pass = visual is not None and visual > visual_threshold
                coobserved_duplicate = bool(
                    not compatible
                    and len(shared_frames) >= coobserved_min_frames
                    and median_shared_2d_iou >= coobserved_min_iou
                    and center_distance <= centroid_threshold
                    and extent_ratio <= coobserved_extent_threshold
                    and overlap_pass
                    and visual is not None
                    and visual >= coobserved_visual_threshold
                )
                if coobserved_duplicate:
                    coobserved_duplicates[i, j] = True
                    coobserved_duplicates[j, i] = True
                class_pass = compatible or exact or coobserved_duplicate
                eligible = (
                    class_pass
                    and (overlap_pass or exact or coobserved_duplicate)
                    and visual_pass
                )
                counts["class_compatible_pairs"] += int(compatible)
                counts["exact_duplicate_pairs"] += int(exact)
                counts["overlap_gate_pairs"] += int(overlap_pass)
                counts["visual_gate_pairs"] += int(visual_pass)
                counts["canonical_eligible_pairs"] += int(eligible)
                counts["cross_class_geometry_rejected_pairs"] += int(
                    not compatible and not exact
                )
                counts["coobserved_pairs"] += int(bool(shared_frames))
                counts["disjoint_frame_history_pairs"] += int(
                    bool(left_frames)
                    and bool(right_frames)
                    and not shared_frames
                )
                counts["coobserved_high_2d_overlap_pairs"] += int(
                    max_shared_2d_iou >= 0.70
                )
                counts["coobserved_duplicate_pairs"] += int(
                    coobserved_duplicate
                )
                reasons = []
                if not compatible and not exact:
                    if center_distance > centroid_threshold:
                        reasons.append("duplicate_centroid")
                    if extent_ratio > extent_threshold:
                        reasons.append("duplicate_extent")
                    if (
                        coverage_evaluated
                        and coverage < coverage_threshold
                    ):
                        reasons.append("duplicate_coverage")
                if not overlap_pass and not exact:
                    reasons.append("spatial_overlap")
                if not visual_pass:
                    reasons.append("visual_similarity")
                rows.append(
                    {
                        "left_index": i,
                        "right_index": j,
                        "left": str(left.get("class_name", "")),
                        "right": str(right.get("class_name", "")),
                        "left_uuid": str(left.get("id", "") or ""),
                        "right_uuid": str(right.get("id", "") or ""),
                        "left_registry_id": self._uuid_to_oid.get(
                            str(left.get("id", "") or "")
                        ),
                        "right_registry_id": self._uuid_to_oid.get(
                            str(right.get("id", "") or "")
                        ),
                        "center_distance_m": round(center_distance, 4),
                        "max_extent_ratio": round(extent_ratio, 4),
                        "tolerant_voxel_coverage": round(coverage, 4),
                        "tolerant_voxel_coverage_evaluated": bool(
                            coverage_evaluated
                        ),
                        "voxel_overlap": round(overlap, 4),
                        "clip_cosine": (
                            None if visual is None else round(visual, 4)
                        ),
                        "class_compatible": bool(compatible),
                        "left_frame_count": len(left_frames),
                        "right_frame_count": len(right_frames),
                        "shared_frame_count": len(shared_frames),
                        "coobserved": bool(shared_frames),
                        "max_shared_frame_2d_iou": round(
                            max_shared_2d_iou,
                            4,
                        ),
                        "median_shared_frame_2d_iou": round(
                            median_shared_2d_iou,
                            4,
                        ),
                        "shared_frame_2d_iou_evidence": [
                            {
                                "frame": frame,
                                "iou": round(value, 4),
                            }
                            for frame, value in shared_2d_evidence[-8:]
                        ],
                        "exact_duplicate": exact,
                        "coobserved_duplicate": coobserved_duplicate,
                        "canonical_eligible": bool(eligible),
                        "rejected_by": reasons,
                    }
                )
        rows.sort(
            key=lambda row: (
                row["canonical_eligible"],
                max(row["voxel_overlap"], row["tolerant_voxel_coverage"]),
                -row["center_distance_m"],
            ),
            reverse=True,
        )
        self._coobserved_merge_admissions = [
            copy.deepcopy(row)
            for row in rows
            if row["coobserved_duplicate"]
        ][:32]
        self._merge_gate_diagnostics = {
            **counts,
            "thresholds": {
                "max_merge_distance_m": max_distance,
                "duplicate_centroid_m": centroid_threshold,
                "duplicate_coverage": coverage_threshold,
                "duplicate_extent_ratio": extent_threshold,
                "coobserved_min_shared_frames": coobserved_min_frames,
                "coobserved_min_median_2d_iou": coobserved_min_iou,
                "coobserved_max_extent_ratio": (
                    coobserved_extent_threshold
                ),
                "coobserved_min_visual_similarity": (
                    coobserved_visual_threshold
                ),
                "overlap": overlap_threshold,
                "visual_similarity": visual_threshold,
            },
            "candidate_pairs": rows[:32],
        }
        return coobserved_duplicates

    def _snapshot_coobserved_merge_admissions(
        self,
        objects,
        *,
        selected_pairs: np.ndarray | None = None,
    ) -> list[dict[str, Any]]:
        """Freeze admitted-pair evidence before ConceptGraphs mutates objects."""
        snapshots: list[dict[str, Any]] = []
        for row in getattr(self, "_coobserved_merge_admissions", ()):
            i = int(row.get("left_index", -1))
            j = int(row.get("right_index", -1))
            if not (0 <= i < len(objects) and 0 <= j < len(objects)):
                continue
            if (
                selected_pairs is not None
                and not bool(selected_pairs[i, j])
            ):
                continue
            left = objects[i]
            right = objects[j]
            snapshot = copy.deepcopy(row)
            snapshot.update(
                {
                    "left_image_idx": [
                        int(value)
                        for value in list(left.get("image_idx", ()) or ())
                        if isinstance(value, (int, np.integer))
                    ],
                    "right_image_idx": [
                        int(value)
                        for value in list(right.get("image_idx", ()) or ())
                        if isinstance(value, (int, np.integer))
                    ],
                    "left_num_detections": int(
                        left.get(
                            "num_detections",
                            len(left.get("class_id", ()) or ()),
                        )
                    ),
                    "right_num_detections": int(
                        right.get(
                            "num_detections",
                            len(right.get("class_id", ()) or ()),
                        )
                    ),
                }
            )
            snapshots.append(snapshot)
        return snapshots

    def _record_coobserved_merge_outcomes(
        self,
        admissions: list[dict[str, Any]],
        *,
        merged_objects,
        index_updates,
        pre_count: int,
    ) -> None:
        """Record whether each admitted pair was actually fused upstream.

        A disappearing index alone is ambiguous: it could have merged into a
        third object. Confirmation therefore requires the surviving UUID to
        contain *exactly* the multiset union of both pre-merge ``image_idx``
        histories and exactly their summed ``num_detections``. A strict
        superset proves that the canonical call performed an unadmitted
        transitive merge and must never be reported as a confirmed pair.
        """
        if not admissions:
            return
        post_by_uuid = {
            str(obj.get("id", "") or ""): obj for obj in merged_objects
        }
        updates = list(index_updates or ())
        events: list[dict[str, Any]] = []
        for admission in admissions:
            left_index = int(admission["left_index"])
            right_index = int(admission["right_index"])
            left_uuid = str(admission.get("left_uuid", "") or "")
            right_uuid = str(admission.get("right_uuid", "") or "")
            left_removed = (
                updates[left_index] is None
                if left_index < len(updates)
                else left_uuid not in post_by_uuid
            )
            right_removed = (
                updates[right_index] is None
                if right_index < len(updates)
                else right_uuid not in post_by_uuid
            )
            survivor_uuid = ""
            removed_uuid = ""
            if left_removed and not right_removed:
                removed_uuid, survivor_uuid = left_uuid, right_uuid
            elif right_removed and not left_removed:
                removed_uuid, survivor_uuid = right_uuid, left_uuid
            survivor = post_by_uuid.get(survivor_uuid)
            expected_history = Counter(
                admission.get("left_image_idx", ())
            ) + Counter(admission.get("right_image_idx", ()))
            actual_history = Counter(
                list(survivor.get("image_idx", ()) or ())
                if survivor is not None
                else ()
            )
            history_union_present = bool(survivor is not None) and all(
                actual_history[frame] >= count
                for frame, count in expected_history.items()
            )
            history_union_exact = bool(
                survivor is not None
                and actual_history == expected_history
            )
            expected_detections = int(
                admission.get("left_num_detections", 0)
            ) + int(admission.get("right_num_detections", 0))
            actual_detections = (
                int(survivor.get("num_detections", 0))
                if survivor is not None
                else 0
            )
            detection_sum_present = (
                survivor is not None
                and actual_detections >= expected_detections
            )
            detection_sum_exact = bool(
                survivor is not None
                and actual_detections == expected_detections
            )
            confirmed = bool(
                survivor_uuid
                and history_union_exact
                and detection_sum_exact
            )
            if confirmed:
                outcome = "confirmed_pair_merge"
            elif (
                survivor_uuid
                and history_union_present
                and detection_sum_present
            ):
                outcome = "transitive_survivor"
            elif left_removed and right_removed:
                outcome = "both_removed_or_transitive"
            elif left_removed or right_removed:
                outcome = "removed_into_other_or_incomplete_evidence"
            else:
                outcome = "admitted_not_merged"
            events.append(
                {
                    "tick": int(self._tick_idx),
                    "left": admission.get("left"),
                    "right": admission.get("right"),
                    "left_uuid": left_uuid,
                    "right_uuid": right_uuid,
                    "left_registry_id": admission.get("left_registry_id"),
                    "right_registry_id": admission.get(
                        "right_registry_id"
                    ),
                    "shared_frame_count": admission.get(
                        "shared_frame_count"
                    ),
                    "median_shared_frame_2d_iou": admission.get(
                        "median_shared_frame_2d_iou"
                    ),
                    "center_distance_m": admission.get(
                        "center_distance_m"
                    ),
                    "voxel_overlap": admission.get("voxel_overlap"),
                    "clip_cosine": admission.get("clip_cosine"),
                    "removed_uuid": removed_uuid or None,
                    "survivor_uuid": survivor_uuid or None,
                    "history_union_present": history_union_present,
                    "history_union_exact": history_union_exact,
                    "expected_num_detections": expected_detections,
                    "actual_num_detections": actual_detections,
                    "detection_sum_present": detection_sum_present,
                    "detection_sum_exact": detection_sum_exact,
                    "confirmed": confirmed,
                    "outcome": outcome,
                    "object_count_before": int(pre_count),
                    "object_count_after": len(merged_objects),
                }
            )
        self._coobserved_merge_events.extend(events)
        del self._coobserved_merge_events[:-32]
        log.info(
            "[scene-cg] coobserved merge evidence: %s",
            [
                (
                    event["left_registry_id"],
                    event["right_registry_id"],
                    event["outcome"],
                )
                for event in events
            ],
        )

    def _resolved_object_label_metadata(
        self,
        obj: Any,
    ) -> dict[str, Any]:
        """Resolve one persistent object's label without mutating it.

        Confidence-weighted detector history remains the primary evidence.
        Once that history is stable, the fused multi-view CLIP feature may
        rerank only within an explicitly configured group or source-specific
        route. ConceptGraphs geometry and association are not changed.
        """
        uuid_value = str(obj.get("id", "") or "")
        object_id = getattr(self, "_uuid_to_oid", {}).get(uuid_value, "")
        override = (
            getattr(self, "_operator_labels", {}).get(object_id)
            or str(obj.get("operator_label", "") or "").strip().lower()
        )
        if override:
            evidence_count = len(list(obj.get("class_id", ()) or ()))
            return {
                "label": override,
                "association_label": override,
                "confidence": 1.0,
                "provisional": False,
                "source": "operator",
                "evidence_count": evidence_count,
                "candidates": [
                    {
                        "label": override,
                        "score": 1.0,
                        "share": 1.0,
                        "observations": evidence_count,
                    }
                ],
            }

        evidence = _label_evidence(
            obj,
            self._classes,
            current_label=str(obj.get("class_name", "") or ""),
            history_size=self._label_history_size,
            min_switch_observations=self._label_min_switch_observations,
            min_winner_share=self._label_min_winner_share,
            switch_margin=self._label_switch_margin,
            label_aliases=self._label_aliases,
        )
        evidence["association_label"] = evidence["label"]
        evidence["source"] = "model"
        geometry_measurements = self._persistent_label_geometry_measurements(
            obj,
            evidence["label"],
        )
        geometry_adjustments = self._clip_rerank_geometry_adjustments(
            evidence["label"],
            geometry_measurements,
        )
        if (
            (evidence["provisional"] and not geometry_adjustments)
            or not getattr(self, "_clip_rerank_candidates_by_label", {})
            or not getattr(self, "_clip_rerank_text_features", {})
        ):
            return evidence

        reranked, _scores = self._clip_rerank_label(
            evidence["label"],
            obj.get("clip_ft"),
            record_quality=False,
            stage="persistent",
            geometry_measurements=geometry_measurements,
        )
        if reranked != evidence["label"]:
            evidence["label"] = reranked
            evidence["source"] = "model_clip"
        return evidence

    def _stabilize_map_labels(self) -> None:
        """Resolve each persistent object's label from recent observations."""
        for obj in self._map_objects or ():
            evidence = self._resolved_object_label_metadata(obj)
            if evidence["source"] == "operator":
                obj["operator_label"] = evidence["label"]
            # Keep the detector-history label as ConceptGraphs' association
            # class. The CLIP-refined public label is derived metadata only and
            # must not make a future detector observation split into a new
            # physical track.
            obj["class_name"] = evidence.get(
                "association_label",
                evidence["label"],
            )
            obj["resolved_class_name"] = evidence["label"]
            obj["label_confidence"] = evidence["confidence"]
            obj["label_provisional"] = evidence["provisional"]
            obj["label_evidence_count"] = evidence["evidence_count"]
            obj["label_candidates"] = evidence["candidates"]
            obj["label_source"] = evidence["source"]

    @staticmethod
    def _drop_derived_label_metadata(objects) -> None:
        """Remove Robonix-only fields before an upstream CG merge.

        ConceptGraphs deliberately raises on keys its merge routine does not
        understand. These values are all derived from ``class_id``/``conf``
        histories, which the upstream merge does combine, so they must be
        recomputed afterwards rather than treated as point-cloud attributes.
        """
        derived_keys = (
            "resolved_class_name",
            "label_confidence",
            "label_provisional",
            "label_source",
            "label_evidence_count",
            "label_candidates",
        )
        for obj in objects or ():
            for key in derived_keys:
                obj.pop(key, None)

    def _tick_locked(self) -> None:
        self._purge_expired_map_objects_locked()
        rgb_msg = self._rgb_msg()
        depth_msg = self._depth_msg()
        if rgb_msg is None or depth_msg is None:
            # One-line diagnostic so "no ticks at all" stops looking
            # like a silent crash. Throttle to once every 25 polls (~15s
            # at the default 0.6s tick) so it doesn't drown the log.
            self._tick_idx += 1
            if self._tick_idx % 25 == 1:
                log.info("[scene-cg] waiting for frames: rgb=%s depth=%s",
                         "ok" if rgb_msg is not None else "none",
                         "ok" if depth_msg is not None else "none")
            return
        K = self._cam_info()
        if K is None or K.fx <= 0 or K.fy <= 0:
            self._tick_idx += 1
            if self._tick_idx % 25 == 1:
                log.info("[scene-cg] waiting for camera intrinsics")
            return
        try:
            import numpy as np
            import torch
        except Exception:
            return
        rgb = _image_msg_to_bgr(rgb_msg)
        depth = _depth_msg_to_metres(depth_msg)
        if rgb is None or depth is None:
            return
        rgb_stamp_s = self._message_stamp_seconds(rgb_msg)
        depth_stamp_s = self._message_stamp_seconds(depth_msg)
        if rgb_stamp_s is not None and depth_stamp_s is not None:
            skew_s = abs(rgb_stamp_s - depth_stamp_s)
            self._last_rgb_depth_skew_s = skew_s
            self._max_rgb_depth_skew_s = max(
                float(getattr(self, "_max_rgb_depth_skew_s", 0.0)),
                skew_s,
            )
        world_frame = str(self._world_frame_fn() or "").strip()
        trans_pose = self._build_camera_to_map_transform(
            expected_world_frame=world_frame,
            stamp=self._message_stamp(depth_msg),
        )
        if trans_pose is None or not world_frame:
            if not getattr(self, "_spatial_not_ready_logged", False):
                log.warning(
                    "camera pose/extrinsics unavailable; withholding spatial "
                    "detections until contracts or header-derived TF are ready"
                )
                self._spatial_not_ready_logged = True
            return
        self._spatial_not_ready_logged = False
        occupancy_grid = self._current_occupancy_grid(world_frame)
        if self._require_occupancy_bounds and occupancy_grid is None:
            if not getattr(self, "_occupancy_not_ready_logged", False):
                log.warning(
                    "occupancy grid unavailable or frame-mismatched; "
                    "withholding navigation-grade object detections"
                )
                self._occupancy_not_ready_logged = True
            return
        self._occupancy_not_ready_logged = False
        cam_K_mat = np.array(
            [
                [K.fx, 0, K.cx],
                [0, K.fy, K.cy],
                [0, 0, 1.0],
            ],
            dtype=np.float32,
        )
        # YOLO-World predict expects RGB in standard channel order.
        # Internal Ultralytics handles BGR-as-input fine but CLIP later
        # needs RGB. Convert once here.
        rgb_for_clip = rgb[:, :, ::-1].copy()  # BGR → RGB

        # ── YOLO-World detect ────────────────────────────────────────
        try:
            # device must be explicit: ultralytics auto-selects CUDA when
            # torch sees a GPU, bypassing SCENE_CG_FORCE_CPU — and crashing
            # on hosts where YOLO-under-CUDA is unstable (the reason that
            # flag exists).
            yolo_results = self._yolo.predict(
                rgb, conf=self._conf_thresh, verbose=False,
                device=self._device,
            )
        except Exception as e:  # noqa: BLE001
            log.warning("yolo-world predict failed: %s", e)
            return
        if not yolo_results:
            return
        r0 = yolo_results[0]
        boxes = getattr(r0, "boxes", None)
        if boxes is None or len(boxes) == 0:
            self._finish_healthy_frame(
                depth=depth,
                intrinsics=K,
                camera_to_world=trans_pose,
            )
            return

        try:
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confs = boxes.conf.detach().cpu().numpy()
            cls_idx = boxes.cls.detach().cpu().numpy().astype(int)
        except Exception:  # noqa: BLE001
            return
        if xyxy.shape[0] == 0:
            self._finish_healthy_frame(
                depth=depth,
                intrinsics=K,
                camera_to_world=trans_pose,
            )
            return

        # Cap the count so SAM doesn't get a 200-bbox dump.
        if xyxy.shape[0] > self._max_dets:
            top = np.argsort(confs)[-self._max_dets:]
            xyxy = xyxy[top]
            confs = confs[top]
            cls_idx = cls_idx[top]

        names = getattr(r0, "names", None) or {i: c for i, c in enumerate(self._classes)}
        # Per-tick diagnostic: how many bboxes did YOLO-World return,
        # what classes? Throttled to once every 25 ticks (~15s @ 0.6s
        # period). Without this it's impossible to tell whether
        # "1 dets" downstream means YOLO sees one thing vs the rest
        # got dropped by class/floor/SAM/pcd filters.
        if self._tick_idx % 25 == 0:
            cls_names = [str(names.get(int(c), f"c{int(c)}")) for c in cls_idx]
            log.info("[scene-cg] yolo: %d boxes, top conf=%.2f, classes=%s",
                     int(xyxy.shape[0]),
                     float(confs.max()) if confs.size else 0.0,
                     cls_names[:8])

        # Filter ignored classes BEFORE running SAM (saves work).
        keep = np.array([
            str(names.get(int(cidx), f"class_{cidx}")).lower() not in _IGNORED_CLASSES
            for cidx in cls_idx
        ], dtype=bool)
        if not keep.any():
            self._finish_healthy_frame(
                depth=depth,
                intrinsics=K,
                camera_to_world=trans_pose,
            )
            return
        xyxy = xyxy[keep]
        confs = confs[keep]
        cls_idx = cls_idx[keep]

        # ── MobileSAM mask each bbox ─────────────────────────────────
        try:
            sam_results = self._sam.predict(
                rgb, bboxes=xyxy.tolist(), verbose=False,
            )
        except Exception as e:  # noqa: BLE001
            log.warning("mobile-sam predict failed: %s — skipping tick", e)
            return
        if not sam_results or getattr(sam_results[0], "masks", None) is None:
            log.debug("mobile-sam returned no masks")
            return
        masks_t = sam_results[0].masks.data
        masks = masks_t.detach().cpu().numpy().astype(bool)
        if masks.shape[0] != xyxy.shape[0]:
            # SAM dropped some boxes (e.g. degenerate). Realign.
            n = min(masks.shape[0], xyxy.shape[0])
            masks = masks[:n]
            xyxy = xyxy[:n]
            confs = confs[:n]
            cls_idx = cls_idx[:n]
        if masks.shape[0] == 0:
            return
        self._quality_counters["masks_input"] += int(masks.shape[0])

        # Resize masks to depth resolution if needed (MobileSAM masks
        # come back at the input resolution, which matches RGB; depth
        # may differ; equal-sized streams make this a no-op).
        h_d, w_d = depth.shape[:2]
        h_m, w_m = masks.shape[1], masks.shape[2]
        if (h_m, w_m) != (h_d, w_d):
            try:
                import cv2
                resized = np.zeros((masks.shape[0], h_d, w_d), dtype=bool)
                for i in range(masks.shape[0]):
                    resized[i] = cv2.resize(
                        masks[i].astype(np.uint8), (w_d, h_d),
                        interpolation=cv2.INTER_NEAREST,
                    ).astype(bool)
                masks = resized
            except Exception as e:  # noqa: BLE001
                log.warning("mask resize failed: %s — skipping tick", e)
                return
        masks = _refine_masks_with_depth(
            masks,
            depth,
            erosion_px=self._mask_erosion_px,
            min_depth_m=self._min_depth_m,
            max_depth_m=self._max_depth_m,
            mad_scale=self._depth_mad_scale,
            min_band_m=self._depth_min_band_m,
            min_points=int(self.cfg["min_points_threshold"]),
        )
        retained_masks = int(np.count_nonzero(np.any(masks, axis=(1, 2))))
        self._quality_counters["masks_retained"] += retained_masks
        self._quality_counters["depth_rejected_masks"] += (
            int(masks.shape[0]) - retained_masks
        )
        if not masks.any():
            self._finish_healthy_frame(
                depth=depth,
                intrinsics=K,
                camera_to_world=trans_pose,
            )
            return

        # ── CLIP per-detection feature ──────────────────────────────
        try:
            import supervision as sv
            sv_dets = sv.Detections(
                xyxy=xyxy.astype(np.float32),
                class_id=cls_idx.astype(int),
                confidence=confs.astype(np.float32),
            )
            sv_dets.mask = masks
            _, image_feats, _ = self._cg["compute_clip_features_batched"](
                rgb_for_clip, sv_dets,
                self._clip_model, self._clip_preprocess, self._clip_tokenizer,
                self._classes, self._device,
            )
        except Exception as e:  # noqa: BLE001
            # First failure: dump full traceback so we know which list
            # in concept-graphs's batched CLIP path was overrun (cls
            # idx vs class list mismatch is the usual cause). Subsequent
            # failures stay terse.
            if not getattr(self, "_clip_fail_logged", False):
                import traceback
                log.warning(
                    "CLIP feature compute failed (first occurrence — full trace; "
                    "n_dets=%d, n_classes=%d, max_cls_idx=%s):\n%s",
                    int(xyxy.shape[0]) if hasattr(xyxy, "shape") else -1,
                    len(self._classes) if hasattr(self, "_classes") else -1,
                    int(max(cls_idx)) if len(cls_idx) else "n/a",
                    traceback.format_exc(),
                )
                self._clip_fail_logged = True
            else:
                log.warning("CLIP feature compute failed: %s — skipping tick", e)
            return

        # ── Per-detection PCD + bbox in map frame ────────────────────
        try:
            obj_pcds_and_bboxes = self._cg["detections_to_obj_pcd_and_bbox"](
                depth_array=depth.astype(np.float32),
                masks=masks,
                cam_K=cam_K_mat,
                image_rgb=rgb_for_clip,
                trans_pose=trans_pose,
                min_points_threshold=self.cfg["min_points_threshold"],
                spatial_sim_type=self.cfg["spatial_sim_type"],
                obj_pcd_max_points=self.cfg["obj_pcd_max_points"],
                downsample_voxel_size=self.cfg["downsample_voxel_size"],
                dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                dbscan_eps=self.cfg["dbscan_eps"],
                dbscan_min_points=self.cfg["dbscan_min_points"],
                # The ali-dev converter currently ignores its DBSCAN
                # arguments. Keep conversion unfiltered and make the
                # canonical process_pcd call below the single owner.
                run_dbscan=False,
                device=self._device,
            )
        except Exception as e:  # noqa: BLE001
            log.warning("detections_to_obj_pcd_and_bbox failed: %s — skipping tick", e)
            return

        for index, entry in enumerate(obj_pcds_and_bboxes):
            if entry is None:
                continue
            try:
                filtered, diagnostic = _run_frame_pointcloud_filter(
                    entry,
                    process_pcd=self._cg["process_pcd"],
                    get_bounding_box=self._cg["get_bounding_box"],
                    downsample_voxel_size=self.cfg[
                        "downsample_voxel_size"
                    ],
                    dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                    dbscan_eps=self.cfg["dbscan_eps"],
                    dbscan_min_points=self.cfg["dbscan_min_points"],
                    spatial_sim_type=self.cfg["spatial_sim_type"],
                    min_points_threshold=self.cfg[
                        "min_points_threshold"
                    ],
                    run_dbscan=self._frame_dbscan,
                )
            except Exception as error:  # noqa: BLE001
                self._quality_counters[
                    "frame_dbscan_rejected_detections"
                ] += 1
                obj_pcds_and_bboxes[index] = None
                log.debug(
                    "[scene-cg] frame point-cloud filter rejected "
                    "detection %d: %s",
                    index,
                    error,
                )
                continue
            if diagnostic["attempted"]:
                self._quality_counters["frame_dbscan_attempts"] += 1
                self._quality_counters["frame_dbscan_input_points"] += int(
                    diagnostic["input_points"]
                )
                self._quality_counters["frame_dbscan_output_points"] += int(
                    diagnostic["output_points"]
                )
                if filtered is None:
                    self._quality_counters[
                        "frame_dbscan_rejected_detections"
                    ] += 1
                elif (
                    diagnostic["output_points"]
                    < diagnostic["input_points"]
                ):
                    self._quality_counters[
                        "frame_dbscan_filtered_detections"
                    ] += 1
            obj_pcds_and_bboxes[index] = filtered

        # ── Build DetectionList ──────────────────────────────────────
        DetectionList = self._cg["DetectionList"]
        det_list = DetectionList()
        image_idx = self._tick_idx
        for i in range(xyxy.shape[0]):
            entry = obj_pcds_and_bboxes[i]
            if entry is None:
                continue
            cls_id_i = int(cls_idx[i])
            cls_name = str(names.get(cls_id_i, f"class_{cls_id_i}")).lower()
            reranked_name, rerank_scores = self._clip_rerank_label(
                cls_name,
                image_feats[i],
            )
            if rerank_scores:
                log.debug(
                    "[scene-cg] CLIP rerank candidate=%s selected=%s scores=%s",
                    cls_name,
                    reranked_name,
                    {
                        label: round(score, 4)
                        for label, score in rerank_scores.items()
                    },
                )
            if reranked_name != cls_name:
                cls_name = reranked_name
                cls_id_i = self._class_index[reranked_name]
            # Drop background classes (floor/wall/ceiling/carpet) — these
            # exist in `_resolved_classes` so YOLO-World *can* label
            # them when it gets confused on a flat surface, but they're
            # not "objects" in the scene-graph sense. The previous code
            # set an `is_background` flag on the detection record but
            # never read it, so floor/wall ended up in the registry.
            if cls_name in _BG_CLASSES:
                continue
            # Floor-only filter — when YOLO's mask leaks past an
            # object's footprint or there's a thin object on the floor
            # the depth-backprojected pcd ends up almost entirely
            # below z=0.10 m. Such pcds get labelled "desk" / "shelf"
            # / "trash bin" but the points are obviously the floor.
            # Drop them before they reach the merge pipeline.
            try:
                pts_chk = np.asarray(entry["pcd"].points)
                if (
                    occupancy_grid is not None
                    and not _occupancy_contains_points(
                        pts_chk,
                        occupancy_grid,
                        expected_frame=world_frame,
                        margin_m=self._map_bounds_margin_m,
                        max_outside_fraction=self._map_max_outside_fraction,
                    )
                ):
                    log.debug(
                        "[scene-cg] drop %s detection outside occupancy bounds",
                        cls_name,
                    )
                    self._quality_counters[
                        "map_bounds_rejected_detections"
                    ] += 1
                    continue
                frame_bbox = _robust_yaw_bbox(
                    pts_chk,
                    low_percentile=self._bbox_low_percentile,
                    high_percentile=self._bbox_high_percentile,
                )
                if cls_name in self._surface_snap_labels:
                    self._quality_counters["surface_snap_attempts"] += 1
                    translation, snap_diagnostic = (
                        _planar_surface_snap_translation(
                            pts_chk,
                            occupancy_grid,
                            expected_frame=world_frame,
                            max_distance_m=(
                                self._surface_snap_max_distance_m
                            ),
                            tangent_padding_m=(
                                self._surface_snap_tangent_padding_m
                            ),
                            min_shift_m=self._surface_snap_min_shift_m,
                            min_support_cells=(
                                self._surface_snap_min_support_cells
                            ),
                            min_dominant_share=(
                                self._surface_snap_min_dominant_share
                            ),
                            min_tangent_coverage=(
                                self._surface_snap_min_tangent_coverage
                            ),
                            occupancy_threshold=(
                                self._surface_snap_occupancy_threshold
                            ),
                        )
                    )
                    snap_diagnostic = {
                        **snap_diagnostic,
                        "label": cls_name,
                        "tick": int(self._tick_idx),
                        "detection_index": int(i),
                    }
                    if snap_diagnostic.get("applied"):
                        try:
                            delta = np.eye(4, dtype=np.float64)
                            delta[:3, 3] = translation
                            entry["pcd"].transform(delta)
                            bbox = entry.get("bbox")
                            try:
                                if bbox is None or not hasattr(
                                    bbox,
                                    "transform",
                                ):
                                    raise AttributeError(
                                        "bbox has no transform"
                                    )
                                bbox.transform(delta)
                            except Exception:  # noqa: BLE001
                                get_bbox = getattr(
                                    entry["pcd"],
                                    "get_axis_aligned_bounding_box",
                                    None,
                                )
                                if get_bbox is None:
                                    raise
                                entry["bbox"] = get_bbox()
                            pts_chk = np.asarray(entry["pcd"].points)
                            frame_bbox = _robust_yaw_bbox(
                                pts_chk,
                                low_percentile=self._bbox_low_percentile,
                                high_percentile=self._bbox_high_percentile,
                            )
                            self._quality_counters[
                                "surface_snap_applied"
                            ] += 1
                        except Exception as error:  # noqa: BLE001
                            self._quality_counters[
                                "surface_snap_apply_failures"
                            ] += 1
                            snap_diagnostic.update(
                                {
                                    "status": "apply_failed",
                                    "applied": False,
                                    "error": type(error).__name__,
                                }
                            )
                            log.warning(
                                "[scene-cg] failed to apply %s surface "
                                "snap: %s",
                                cls_name,
                                error,
                            )
                    self._surface_snap_recent.append(snap_diagnostic)
                    del self._surface_snap_recent[:-32]
                if (
                    frame_bbox is None
                    or float(np.max(frame_bbox[1])) > self._max_bbox_extent_m
                ):
                    log.debug(
                        "[scene-cg] drop %s detection with invalid/oversized bbox",
                        cls_name,
                    )
                    self._quality_counters[
                        "oversized_bbox_rejected_detections"
                    ] += 1
                    continue
                if pts_chk.shape[0] >= 4:
                    z_max = float(pts_chk[:, 2].max())
                    z_p90 = float(np.percentile(pts_chk[:, 2], 90))
                    # Class-specific minimum height: a desk top sits
                    # ≥ 0.55 m above the floor, a chair seat ≥ 0.30 m,
                    # a cup on a table ≥ 0.50 m. If none of those hold
                    # it's almost certainly floor noise.
                    floor_classes = {
                        "table", "desk", "couch", "sofa", "shelf",
                        "bookshelf", "cabinet", "drawer", "monitor",
                        "monitor stand", "computer tower",
                    }
                    if cls_name in floor_classes and z_max < 0.30:
                        log.debug(
                            "[scene-cg] drop %s det at z_max=%.2f (floor noise)",
                            cls_name, z_max,
                        )
                        continue
                    if z_p90 < 0.05:
                        # Anything whose 90th percentile is on the
                        # floor cell, drop regardless of class.
                        continue
            except Exception:
                pass
            d = {
                "id": uuid.uuid4(),
                "image_idx": [image_idx],
                "mask_idx": [i],
                "color_path": [""],
                "class_name": cls_name,
                "class_id": [cls_id_i],
                "captions": [""],
                "num_detections": 1,
                "mask": [masks[i]],
                "xyxy": [xyxy[i].tolist()],
                "conf": [float(confs[i])],
                "n_points": len(entry["pcd"].points),
                "contain_number": [None],
                "inst_color": np.random.rand(3),
                "is_background": cls_name in _BG_CLASSES,
                "pcd": entry["pcd"],
                "bbox": entry["bbox"],
                "clip_ft": torch.from_numpy(image_feats[i]).float(),
                "num_obj_in_class": 0,
                "curr_obj_num": 0,
                "new_counter": 0,
            }
            det_list.append(d)
        self._quality_counters["accepted_frame_detections"] += len(det_list)

        if len(det_list) == 0:
            self._finish_healthy_frame(
                depth=depth,
                intrinsics=K,
                camera_to_world=trans_pose,
            )
            return

        # ── Concept-graphs merge ─────────────────────────────────────
        if len(self._map_objects) == 0:
            for d in det_list:
                self._map_objects.append(d)
            log.info("[scene-cg] init map with %d objects", len(self._map_objects))
        else:
            try:
                # Voxel pcd-overlap (our impl) instead of concept-graphs's
                # compute_spatial_similarities('overlap', ...) which crashes
                # on coplanar bbox vertices via pytorch3d.box3d_overlap. AABB-IoU
                # was the previous fallback but was too weak for the cross-view
                # merge case. See `_voxel_pcd_overlap_matrix` for the rationale.
                #
                # Visual sim comes back on whatever device CLIP runs on
                # (cuda when available); pin our spatial_sim to that same
                # device before they're added together by aggregate_similarities.
                visual_sim = self._cg["compute_visual_similarities"](
                    det_list, self._map_objects,
                )
                spatial_sim = self._voxel_pcd_overlap_torch(det_list, self._map_objects)
                spatial_sim = spatial_sim.to(visual_sim.device)
                agg_sim = self._cg["aggregate_similarities"](
                    self.cfg["match_method"], self.cfg["phys_bias"],
                    spatial_sim, visual_sim,
                )

                # ── Hard centroid-distance + same-class gates ───────
                # Two reasons to short-circuit a "high agg_sim" pair:
                #
                # (1) **Distance gate.** With merge_threshold low,
                #     visual sim alone can carry a match — but two
                #     physically distant potted_plants ALSO have visual
                #     sim ≈ 0.6. Result: one bloated bbox spanning the
                #     room (the 9 × 5 m blob).
                #
                # (2) **Same-class gate.** A potted plant sitting on
                #     top of a cabinet has near-zero centroid distance
                #     and reasonable visual sim → without this gate
                #     the two get merged into a single record (the
                #     "potted_plant and cabinet can no longer be told apart" complaint).
                #     CLIP's discrimination breaks down on co-located
                #     objects; the YOLO-World class label is more
                #     reliable. Only merge when the YOLO classes match
                #     OR when one of the two is the unconfigured
                #     fallback class.
                try:
                    import torch
                    det_centroids = []
                    det_extents = []
                    det_classes = []
                    for d in det_list:
                        pp = np.asarray(d["pcd"].points)
                        geometry = _robust_yaw_bbox(
                            pp,
                            low_percentile=self._bbox_low_percentile,
                            high_percentile=self._bbox_high_percentile,
                        )
                        if geometry is None:
                            det_centroids.append(
                                np.full(3, np.nan, dtype=np.float64)
                            )
                            det_extents.append(
                                np.full(3, np.nan, dtype=np.float64)
                            )
                        else:
                            det_centroids.append(geometry[0])
                            det_extents.append(geometry[1])
                        det_classes.append(str(d.get("class_name", "")).lower())
                    obj_centroids = []
                    obj_extents = []
                    obj_classes = []
                    for o in self._map_objects:
                        pp = np.asarray(o["pcd"].points)
                        geometry = _robust_yaw_bbox(
                            pp,
                            low_percentile=self._bbox_low_percentile,
                            high_percentile=self._bbox_high_percentile,
                        )
                        if geometry is None:
                            obj_centroids.append(
                                np.full(3, np.nan, dtype=np.float64)
                            )
                            obj_extents.append(
                                np.full(3, np.nan, dtype=np.float64)
                            )
                        else:
                            obj_centroids.append(geometry[0])
                            obj_extents.append(geometry[1])
                        obj_classes.append(str(o.get("class_name", "")).lower())
                    if det_centroids and obj_centroids:
                        dC = np.asarray(det_centroids)             # (M, 3)
                        oC = np.asarray(obj_centroids)             # (N, 3)
                        diffs = dC[:, None, :] - oC[None, :, :]    # (M, N, 3)
                        dist = np.linalg.norm(diffs, axis=2)       # (M, N)
                        max_d = float(self.cfg["max_merge_dist_m"])
                        distance_limits = np.full_like(
                            dist,
                            max_d,
                            dtype=np.float64,
                        )
                        if bool(self.cfg["adaptive_merge_distance"]):
                            for i, detection_extent in enumerate(det_extents):
                                for j, object_extent in enumerate(obj_extents):
                                    distance_limits[i, j] = (
                                        _adaptive_association_distance_limit(
                                            detection_extent[:2],
                                            object_extent[:2],
                                            minimum_m=self.cfg[
                                                "adaptive_merge_min_dist_m"
                                            ],
                                            maximum_m=max_d,
                                            extent_scale=self.cfg[
                                                "adaptive_merge_extent_scale"
                                            ],
                                        )
                                    )
                        invalid_geometry = (
                            ~np.all(np.isfinite(dC), axis=1)[:, None]
                            | ~np.all(np.isfinite(oC), axis=1)[None, :]
                        )
                        dist_mask_np = invalid_geometry | (
                            ~np.isfinite(dist)
                        ) | (dist > distance_limits)
                        if bool(self.cfg["adaptive_merge_distance"]):
                            extra_rejections = (
                                ~invalid_geometry
                                & np.isfinite(dist)
                                & (dist <= max_d)
                                & (dist > distance_limits)
                            )
                            self._quality_counters[
                                "adaptive_distance_rejected_pairs"
                            ] += int(np.count_nonzero(extra_rejections))

                        # Cross-class association is unsafe by default:
                        # co-location is common for distinct objects (cup on
                        # table, plant on cabinet). Only exact classes or a
                        # deployment-reviewed confusable group may share a
                        # track. The legacy class-agnostic behaviour requires
                        # the explicit allow_cross_class_merge escape hatch.
                        # A narrowly-scoped exception exists for near-identical
                        # voxel clouds: this repairs detector label competition
                        # (the same geometry emitted as cabinet + shelf) without
                        # treating mere co-location as identity.
                        (
                            exact_duplicates,
                            tolerant_duplicate_coverage,
                        ) = self._exact_duplicate_geometry_matrices(
                            det_list,
                            self._map_objects,
                        )
                        # The ordinary overlap score uses exact voxel
                        # intersection. Cross-view RGB-D clouds commonly move
                        # by one 2.5 cm voxel even when their robust center,
                        # extent, and bidirectional support prove they are the
                        # same physical surface. Feed that measured tolerant
                        # coverage to ConceptGraphs only for pairs that passed
                        # all three strict duplicate-geometry gates.
                        tolerant_spatial = torch.from_numpy(
                            np.where(
                                exact_duplicates,
                                tolerant_duplicate_coverage,
                                0.0,
                            )
                        ).to(
                            device=spatial_sim.device,
                            dtype=spatial_sim.dtype,
                        )
                        spatial_sim = torch.maximum(
                            spatial_sim,
                            tolerant_spatial,
                        )
                        agg_sim = self._cg["aggregate_similarities"](
                            self.cfg["match_method"],
                            self.cfg["phys_bias"],
                            spatial_sim,
                            visual_sim,
                        )
                        cls_mismatch = np.zeros(
                            (len(det_classes), len(obj_classes)),
                            dtype=bool,
                        )
                        for i, dc in enumerate(det_classes):
                            for j, oc in enumerate(obj_classes):
                                if (
                                    not self._association_compatible(dc, oc)
                                    and not exact_duplicates[i, j]
                                ):
                                    cls_mismatch[i, j] = True
                        # Apply every hard gate only after the final aggregate
                        # recomputation above. Applying the distance mask to an
                        # earlier matrix silently erased it whenever tolerant
                        # duplicate support caused this recompute.
                        hard_reject = torch.from_numpy(
                            dist_mask_np | cls_mismatch
                        ).to(agg_sim.device)
                        agg_sim[hard_reject] = float("-inf")
                except Exception as _e:  # noqa: BLE001
                    # A malformed cloud or device mismatch must not silently
                    # restore unrestricted association. Leave detections
                    # unmatched so ConceptGraphs creates independent tracks;
                    # periodic evidence-based cleanup may reconcile them later.
                    log.warning(
                        "merge-gate failed closed; detections remain "
                        "unmatched: %s",
                        _e,
                    )
                    agg_sim[:] = float("-inf")

                # Below threshold → unmatched → new object.
                merge_threshold = float(self.cfg["merge_threshold"])
                agg_sim[agg_sim < merge_threshold] = float("-inf")
                if bool(self.cfg["one_to_one_association"]):
                    try:
                        before_assignment = (
                            agg_sim.detach().float().cpu().numpy()
                        )
                        assignment = _one_to_one_association_mask(
                            before_assignment,
                            threshold=merge_threshold,
                        )
                        finite_before = np.isfinite(before_assignment)
                        rejected = finite_before & ~assignment
                        self._quality_counters[
                            "one_to_one_rejected_pairs"
                        ] += int(np.count_nonzero(rejected))
                        self._quality_counters[
                            "one_to_one_unmatched_detections"
                        ] += int(
                            np.count_nonzero(
                                np.any(finite_before, axis=1)
                                & ~np.any(assignment, axis=1)
                            )
                        )
                        assignment_mask = torch.from_numpy(
                            assignment
                        ).to(agg_sim.device)
                        agg_sim[~assignment_mask] = float("-inf")
                    except Exception as error:  # noqa: BLE001
                        log.warning(
                            "one-to-one association failed closed; "
                            "detections remain unmatched: %s",
                            error,
                        )
                        agg_sim[:] = float("-inf")
                pre_n = len(self._map_objects)
                self._map_objects = self._cg["merge_detections_to_objects"](
                    downsample_voxel_size=self.cfg["downsample_voxel_size"],
                    dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                    dbscan_eps=self.cfg["dbscan_eps"],
                    dbscan_min_points=self.cfg["dbscan_min_points"],
                    spatial_sim_type=self.cfg["spatial_sim_type"],
                    device=self._device,
                    match_method=self.cfg["match_method"],
                    phys_bias=self.cfg["phys_bias"],
                    detection_list=det_list,
                    objects=self._map_objects,
                    agg_sim=agg_sim,
                )
                added = len(self._map_objects) - pre_n
                # Per-frame log is too noisy. Only log when (a) something
                # changed (new/total delta) or (b) every Nth tick as a
                # heartbeat. Otherwise INFO once for the first tick is
                # enough so the dev sees ingest is alive.
                self._tick_seq = getattr(self, "_tick_seq", 0) + 1
                prev_total = getattr(self, "_last_logged_total", -1)
                changed = added != 0 or len(self._map_objects) != prev_total
                if changed or self._tick_seq == 1 or self._tick_seq % 100 == 0:
                    log.info(
                        "[scene-cg] tick %d: %d dets, %d new, %d total objects",
                        self._tick_seq, len(det_list), added, len(self._map_objects),
                    )
                    self._last_logged_total = len(self._map_objects)
            except Exception as e:  # noqa: BLE001
                # First few failures: dump the full traceback so we can
                # tell whether it was compute_spatial_similarities,
                # compute_visual_similarities, aggregate, or the merge
                # itself. Subsequent failures stay terse.
                if not getattr(self, "_merge_fail_logged", False):
                    import traceback
                    log.warning(
                        "concept-graphs merge pipeline failed (first occurrence — full trace):\n%s",
                        traceback.format_exc(),
                    )
                    self._merge_fail_logged = True
                else:
                    log.warning("concept-graphs merge pipeline failed: %s", e)
                return

        self._finish_healthy_frame(
            depth=depth,
            intrinsics=K,
            camera_to_world=trans_pose,
        )

    def _finish_healthy_frame(
        self,
        *,
        depth: Any,
        intrinsics: Any,
        camera_to_world: Any,
    ) -> None:
        """Commit one healthy frame's positive and negative evidence."""
        self._quality_counters["healthy_frames"] += 1
        frame_seq = self._tick_idx
        self._tick_idx += 1
        self._stabilize_map_labels()
        self._maybe_periodic_cleanup()
        self._stabilize_map_labels()
        observed_uuids = _observed_map_object_uuids(
            self._map_objects,
            frame_seq=frame_seq,
        )
        visibility_diagnostics: dict[str, dict[str, Any]] = {}
        visible_miss_uuids = _visible_missing_uuids(
            self._map_objects,
            observed_uuids=observed_uuids,
            depth_m=depth,
            intrinsics=intrinsics,
            camera_to_world=camera_to_world,
            depth_margin_m=self._visibility_depth_margin_m,
            min_clear_samples=self._visibility_min_clear_samples,
            min_clear_fraction=self._visibility_min_clear_fraction,
            max_projected_samples=self._visibility_max_projected_samples,
            diagnostics=visibility_diagnostics,
        )
        self._visibility_diagnostics = visibility_diagnostics
        self._project_to_registry(
            observed_uuids=observed_uuids,
            visible_miss_uuids=visible_miss_uuids,
            frame_seq=frame_seq,
            observed_at=time.time(),
        )

    def _purge_expired_map_objects_locked(self) -> None:
        """Remove TTL-expired UUIDs from the persistent ConceptGraphs map."""
        expired = set(getattr(self, "_expired_uuids", set()))
        if not expired or self._map_objects is None:
            return
        self._expired_uuids.difference_update(expired)
        if self._cg is not None:
            retained = self._cg["MapObjectList"]()
        else:
            retained = []
        for obj in self._map_objects:
            if str(obj.get("id", "") or "") not in expired:
                retained.append(obj)
        self._map_objects = retained

    # ── Voxel pcd-overlap (replaces concept-graphs `overlap` mode) ──
    @staticmethod
    def _voxel_pcd_overlap_matrix(
        objects_a, objects_b=None, *, voxel_size: float = 0.025,
    ):
        """Pairwise voxel-set fraction-overlap.

        For each pair (i, j) returns
            overlap(i, j) = |A_i ∩ B_j| / min(|A_i|, |B_j|)
        where A_i / B_j are the voxel-grid hashes of the i-th / j-th
        point clouds at `voxel_size` resolution. Pure numpy + Python
        set ops — no pytorch3d, no qhull. Robust to degenerate
        (coplanar / sparse) point clouds, which crash concept-graphs's
        canonical `compute_overlap_matrix_general` via box3d_overlap.

        Returns an (M, N) float32 numpy array, where M = len(objects_a)
        and N = len(objects_b) (or M when `objects_b is None`, with a
        symmetric matrix and zero diagonal).
        """
        # Lazy numpy import — matches the per-method pattern the rest of
        # this file uses (every other method that calls np.* imports it
        # locally). Without this, `np` is unbound here at run time.
        import numpy as np

        def voxel_set(pcd) -> frozenset:
            pts = np.asarray(pcd.points)
            if pts.size == 0:
                return frozenset()
            ix = np.floor(pts / voxel_size).astype(np.int64)
            return frozenset(map(tuple, ix.tolist()))

        a_sets = [voxel_set(o["pcd"]) for o in objects_a]
        symmetric = objects_b is None
        b_sets = a_sets if symmetric else [voxel_set(o["pcd"]) for o in objects_b]

        m, n = len(a_sets), len(b_sets)
        out = np.zeros((m, n), dtype=np.float32)
        for i, a in enumerate(a_sets):
            if not a:
                continue
            j_start = i + 1 if symmetric else 0
            for j in range(j_start, n):
                b = b_sets[j]
                if not b:
                    continue
                inter = len(a & b)
                if inter == 0:
                    continue
                v = inter / min(len(a), len(b))
                out[i, j] = v
                if symmetric:
                    out[j, i] = v
        return out

    def _voxel_pcd_overlap_torch(self, det_list, map_objects):
        """Wrap `_voxel_pcd_overlap_matrix` for the per-tick merge —
        returns an (M, N) torch tensor on `self._device` so it can be
        passed directly to `aggregate_similarities` alongside the CLIP
        visual similarity matrix."""
        import torch
        arr = self._voxel_pcd_overlap_matrix(
            det_list, map_objects, voxel_size=self.cfg["downsample_voxel_size"],
        )
        return torch.from_numpy(arr).to(self._device)

    # ── Periodic cleanup ─────────────────────────────────────────────
    def _maybe_periodic_cleanup(self) -> None:
        if self._map_objects is None or len(self._map_objects) == 0:
            return
        ran_any = False
        if self._tick_idx % self.cfg["denoise_interval_ticks"] == 0:
            try:
                pre = len(self._map_objects)
                self._map_objects = self._cg["denoise_objects"](
                    downsample_voxel_size=self.cfg["downsample_voxel_size"],
                    dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                    dbscan_eps=self.cfg["dbscan_eps"],
                    dbscan_min_points=self.cfg["dbscan_min_points"],
                    spatial_sim_type=self.cfg["spatial_sim_type"],
                    device=self._device,
                    objects=self._map_objects,
                )
                # filter_objects drops objects with too-few points / detections
                self._map_objects = self._cg["filter_objects"](
                    obj_min_points=self.cfg["obj_min_points"],
                    obj_min_detections=self.cfg["obj_min_detections"],
                    objects=self._map_objects,
                )
                if len(self._map_objects) != pre:
                    log.info("[scene-cg] cleanup denoise+filter: %d → %d", pre, len(self._map_objects))
                ran_any = True
            except Exception as e:  # noqa: BLE001
                log.warning("denoise/filter failed: %s", e)
        if self._tick_idx % self.cfg["merge_overlap_interval_ticks"] == 0:
            try:
                pre = len(self._map_objects)
                # Canonical concept-graphs cleanup pass. Computes the
                # full N×N pcd-overlap matrix (uses pytorch3d under
                # the hood for oriented-bbox IoU prefilter), then
                # merges every pair where overlap > merge_overlap_thresh
                # AND CLIP cosine > merge_visual_sim_thresh. This is
                # what catches cabinet+shelf at the same physical
                # position labelled separately by YOLO-World — per-tick
                # `merge_detections_to_objects` only matches new dets
                # against the existing map, so two BRAND NEW dets in
                # one tick both get appended; this pass collapses them.
                # Same swap as the per-tick merge: our voxel pcd-overlap
                # impl avoids concept-graphs's pytorch3d crash path. The
                # `merge_overlap_objects` callee accepts the matrix as a
                # plain numpy array of shape (N, N).
                overlap_mat = self._voxel_pcd_overlap_matrix(
                    self._map_objects,
                    objects_b=None,
                    voxel_size=self.cfg["downsample_voxel_size"],
                )
                (
                    exact_duplicates,
                    tolerant_duplicate_coverage,
                ) = self._exact_duplicate_geometry_matrices(
                    self._map_objects,
                )
                coobserved_duplicates = self._record_merge_gate_diagnostics(
                    self._map_objects,
                    overlap_matrix=overlap_mat,
                    exact_duplicates=exact_duplicates,
                    tolerant_coverage=tolerant_duplicate_coverage,
                )
                # `merge_overlap_objects` applies its own strict `>` spatial
                # threshold after our class-safety mask. A pair that passed
                # robust center, bidirectional tolerant support, and extent
                # equivalence is already a geometry-verified duplicate; make
                # it visible to the canonical ConceptGraphs visual gate even
                # when exact voxel intersection falls just below the ordinary
                # overlap threshold.
                duplicate_floor = float(
                    self.cfg["merge_overlap_thresh"]
                ) + 1e-6
                overlap_mat[exact_duplicates] = np.maximum(
                    overlap_mat[exact_duplicates],
                    np.maximum(
                        tolerant_duplicate_coverage[exact_duplicates],
                        duplicate_floor,
                    ),
                )
                # A repeated same-frame 2D overlap is independent evidence
                # that two conflicting detector labels describe one image
                # region. The evidence gate is deliberately stricter than the
                # canonical merge thresholds and remains class-agnostic. Once
                # admitted, ConceptGraphs still performs its normal visual and
                # geometric fusion.
                overlap_mat[coobserved_duplicates] = np.maximum(
                    overlap_mat[coobserved_duplicates],
                    duplicate_floor,
                )
                # ConceptGraphs's canonical pass is class-agnostic. Mask
                # incompatible labels before handing it the overlap matrix so
                # a nearby tabletop object cannot be absorbed into furniture.
                # Near-identical voxel clouds remain eligible because they are
                # duplicate physical hypotheses, not merely nearby objects.
                for i, left in enumerate(self._map_objects):
                    for j in range(i + 1, len(self._map_objects)):
                        right = self._map_objects[j]
                        if left.get("operator_label") or right.get(
                            "operator_label"
                        ):
                            overlap_mat[i, j] = 0.0
                            overlap_mat[j, i] = 0.0
                            continue
                        if not self._allow_cross_class_merge:
                            if (
                                not self._association_compatible(
                                    left.get("class_name", ""),
                                    right.get("class_name", ""),
                                )
                                and not exact_duplicates[i, j]
                                and not coobserved_duplicates[i, j]
                            ):
                                overlap_mat[i, j] = 0.0
                                overlap_mat[j, i] = 0.0
                # The upstream cleanup merger treats the matrix as a graph
                # and can collapse A-B-C transitively in one call even when
                # only A-B and B-C were admitted. Pairwise evidence does not
                # prove A and C identity. Expose only endpoint-disjoint pairs
                # and let the next cleanup tick reconsider the remainder
                # against the newly fused geometry and observation history.
                visual_mat = _pairwise_object_clip_cosines(
                    self._map_objects
                )
                selected_pairs = _disjoint_periodic_merge_mask(
                    overlap_mat,
                    spatial_threshold=float(
                        self.cfg["merge_overlap_thresh"]
                    ),
                    visual_scores=visual_mat,
                    visual_threshold=float(
                        self.cfg["merge_visual_sim_thresh"]
                    ),
                )
                candidate_pairs = int(
                    np.count_nonzero(
                        np.triu(
                            (overlap_mat > float(
                                self.cfg["merge_overlap_thresh"]
                            ))
                            & np.isfinite(visual_mat)
                            & (visual_mat > float(
                                self.cfg["merge_visual_sim_thresh"]
                            )),
                            k=1,
                        )
                    )
                )
                selected_pair_count = int(
                    np.count_nonzero(np.triu(selected_pairs, k=1))
                )
                deferred_pair_count = max(
                    0,
                    candidate_pairs - selected_pair_count,
                )
                self._quality_counters[
                    "periodic_merge_candidate_pairs"
                ] += candidate_pairs
                self._quality_counters[
                    "periodic_merge_selected_pairs"
                ] += selected_pair_count
                self._quality_counters[
                    "periodic_merge_deferred_pairs"
                ] += deferred_pair_count
                self._merge_gate_diagnostics.update(
                    {
                        "periodic_candidate_pairs": candidate_pairs,
                        "periodic_selected_pairs": selected_pair_count,
                        "periodic_deferred_pairs": deferred_pair_count,
                    }
                )
                overlap_mat = np.where(
                    selected_pairs,
                    overlap_mat,
                    0.0,
                )
                coobserved_admissions = (
                    self._snapshot_coobserved_merge_admissions(
                        self._map_objects,
                        selected_pairs=selected_pairs,
                    )
                )
                # concept-graphs `merge_overlap_objects` returns
                # `(MapObjectList, index_updates)` — assigning the
                # tuple directly into self._map_objects silently broke
                # every subsequent tick (compute_spatial_similarities
                # then called .get_stacked_values_torch on a tuple).
                # Unpack and discard index_updates (we don't carry
                # MapEdgeMapping anyway).
                self._drop_derived_label_metadata(self._map_objects)
                try:
                    merged_objs, index_updates = self._cg[
                        "merge_overlap_objects"
                    ](
                        merge_overlap_thresh=self.cfg[
                            "merge_overlap_thresh"
                        ],
                        merge_visual_sim_thresh=self.cfg[
                            "merge_visual_sim_thresh"
                        ],
                        merge_text_sim_thresh=self.cfg[
                            "merge_text_sim_thresh"
                        ],
                        objects=self._map_objects,
                        overlap_matrix=overlap_mat,
                        downsample_voxel_size=self.cfg[
                            "downsample_voxel_size"
                        ],
                        dbscan_remove_noise=self.cfg[
                            "dbscan_remove_noise"
                        ],
                        dbscan_eps=self.cfg["dbscan_eps"],
                        dbscan_min_points=self.cfg["dbscan_min_points"],
                        spatial_sim_type=self.cfg["spatial_sim_type"],
                        device=self._device,
                    )
                    self._map_objects = merged_objs
                    self._record_coobserved_merge_outcomes(
                        coobserved_admissions,
                        merged_objects=merged_objs,
                        index_updates=index_updates,
                        pre_count=pre,
                    )
                finally:
                    self._stabilize_map_labels()
                if len(self._map_objects) != pre:
                    log.info("[scene-cg] cleanup merge_overlap: %d → %d", pre, len(self._map_objects))
                ran_any = True
            except Exception as e:  # noqa: BLE001
                log.warning("merge_overlap_objects failed: %s", e)
        if (
            self._allow_cross_class_merge
            and self._tick_idx % self.cfg["cross_class_merge_interval_ticks"] == 0
        ):
            try:
                pre = len(self._map_objects)
                try:
                    self._map_objects = self._cross_class_geometric_collapse(
                        self._map_objects
                    )
                finally:
                    self._stabilize_map_labels()
                if len(self._map_objects) != pre:
                    log.info("[scene-cg] cleanup cross-class geom: %d → %d", pre, len(self._map_objects))
                    ran_any = True
            except Exception as e:  # noqa: BLE001
                log.warning("cross-class geometric collapse failed: %s", e)
        if self._tick_idx % self.cfg["same_class_merge_interval_ticks"] == 0:
            try:
                pre = len(self._map_objects)
                try:
                    self._map_objects = self._same_class_proximity_collapse(
                        self._map_objects
                    )
                finally:
                    self._stabilize_map_labels()
                if len(self._map_objects) != pre:
                    log.info(
                        "[scene-cg] cleanup same-class geometry: %d → %d",
                        pre,
                        len(self._map_objects),
                    )
                    ran_any = True
            except Exception as e:  # noqa: BLE001
                log.warning("same-class geometry collapse failed: %s", e)
        if ran_any:
            # Cleanup rewrites persistent geometry but is not a sensor
            # observation.  Passing no evidence sets activates the legacy
            # compatibility path in _apply_snapshot, which treats every map
            # object as observed and therefore inflates observation_count,
            # refreshes last_seen, and clears visibility-miss streaks.  Keep
            # the immediate geometry/binding reconciliation while making the
            # absence of positive or negative frame evidence explicit.
            self._project_to_registry(
                observed_uuids=set(),
                visible_miss_uuids=set(),
            )

    def _cross_class_geometric_collapse(self, objects):
        """Class-and-visual-agnostic merge pass. Folds together objects
        whose AABBs overlap heavily (IoU or one-inside-other ratio),
        regardless of class label or CLIP feature distance.

        Defends against the YOLO-World label flicker case where the
        same physical wall fixture gets registered three times as
        picture_frame + shelf + cabinet — `merge_overlap_objects`
        skips them because their CLIP features differ enough to fail
        merge_visual_sim_thresh, even though their point clouds are
        on top of each other in 3D.
        """
        # Same per-method lazy numpy import as the rest of this file —
        # without it, `np.*` references below raise NameError and the
        # whole class-agnostic merge pass silently bails every cleanup
        # tick (visible in scene log as "cross-class geometric
        # collapse failed: name 'np' is not defined").
        import numpy as np

        n = len(objects)
        if n < 2:
            return objects
        merge_obj2_into_obj1 = self._cg.get("merge_obj2_into_obj1")
        if merge_obj2_into_obj1 is None:
            try:
                from conceptgraph.slam.utils import merge_obj2_into_obj1 as _m
                merge_obj2_into_obj1 = _m
            except Exception:
                return objects
        self._drop_derived_label_metadata(objects)

        bboxes: list[tuple["np.ndarray", "np.ndarray"]] = []
        for o in objects:
            pts = np.asarray(o["pcd"].points)
            pts = pts[np.all(np.isfinite(pts), axis=1)] if pts.size else pts
            if pts.size == 0:
                bboxes.append((np.zeros(3), np.zeros(3)))
            else:
                bboxes.append((pts.min(axis=0), pts.max(axis=0)))

        iou_thr = float(self.cfg["cross_class_iou_thresh"])
        ratio_thr = float(self.cfg["cross_class_overlap_thresh"])
        keep = [True] * n
        for i in range(n):
            if not keep[i]:
                continue
            for j in range(i + 1, n):
                if not keep[j]:
                    continue
                if objects[i].get("operator_label") or objects[j].get(
                    "operator_label"
                ):
                    continue
                a_min, a_max = bboxes[i]
                b_min, b_max = bboxes[j]
                inter = np.maximum(np.minimum(a_max, b_max) - np.maximum(a_min, b_min), 0.0)
                inter_vol = float(np.prod(inter))
                a_vol = float(np.prod(np.maximum(a_max - a_min, 0.0)))
                b_vol = float(np.prod(np.maximum(b_max - b_min, 0.0)))
                union = a_vol + b_vol - inter_vol
                if union <= 1e-9 or a_vol <= 1e-9 or b_vol <= 1e-9:
                    continue
                iou = inter_vol / union
                ratio = max(inter_vol / a_vol, inter_vol / b_vol)
                if iou < iou_thr and ratio < ratio_thr:
                    continue
                try:
                    objects[i] = merge_obj2_into_obj1(
                        obj1=objects[i], obj2=objects[j],
                        downsample_voxel_size=self.cfg["downsample_voxel_size"],
                        dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                        dbscan_eps=self.cfg["dbscan_eps"],
                        dbscan_min_points=self.cfg["dbscan_min_points"],
                        spatial_sim_type=self.cfg["spatial_sim_type"],
                        device=self._device,
                        run_dbscan=False,
                    )
                except Exception as e:  # noqa: BLE001
                    log.debug("cross-class merge i=%d j=%d failed: %s", i, j, e)
                    continue
                keep[j] = False
                pts = np.asarray(objects[i]["pcd"].points)
                pts = pts[np.all(np.isfinite(pts), axis=1)] if pts.size else pts
                if pts.size:
                    bboxes[i] = (pts.min(axis=0), pts.max(axis=0))

        MapObjectList = self._cg["MapObjectList"]
        out = MapObjectList()
        for k, alive in enumerate(keep):
            if alive:
                out.append(objects[k])
        return out

    def _same_class_disjoint_identity_evidence(
        self,
        left: Any,
        right: Any,
    ) -> Optional[dict[str, Any]]:
        """Admit adjacent-frame fragments with no shared voxel support.

        This is a narrow fallback for alternating partial views of one object.
        It is never a proximity-only merge: both tracks need multiple unique
        frames, their histories must be disjoint but temporally adjacent, the
        robust 3D centers must be close in metres *and* relative to object
        scale, all robust extents must be compatible, and the accumulated CLIP
        features must agree.  The caller still uses ConceptGraphs' canonical
        merger after this evidence gate passes.
        """
        import numpy as np

        minimum_frames = int(
            self.cfg["same_class_disjoint_min_unique_frames"]
        )
        maximum_gap = int(
            self.cfg["same_class_disjoint_max_frame_gap"]
        )
        maximum_center_ratio = float(
            self.cfg[
                "same_class_disjoint_max_center_major_extent_ratio"
            ]
        )
        minimum_visual = float(
            self.cfg["same_class_disjoint_min_visual_similarity"]
        )
        if maximum_center_ratio <= 0.0:
            return None

        def unique_frames(obj: Any) -> tuple[int, ...]:
            return tuple(
                sorted(
                    {
                        int(value)
                        for value in list(obj.get("image_idx", ()) or ())
                        if isinstance(value, (int, np.integer))
                    }
                )
            )

        left_frames = unique_frames(left)
        right_frames = unique_frames(right)
        if (
            len(left_frames) < minimum_frames
            or len(right_frames) < minimum_frames
            or set(left_frames) & set(right_frames)
        ):
            return None
        frame_gap = min(
            abs(left_frame - right_frame)
            for left_frame in left_frames
            for right_frame in right_frames
        )
        if frame_gap > maximum_gap:
            return None

        def geometry(obj: Any):
            try:
                points = np.asarray(obj["pcd"].points, dtype=np.float64)
            except (AttributeError, KeyError, TypeError, ValueError):
                return None
            if points.ndim != 2 or points.shape[1:] != (3,):
                return None
            points = points[np.all(np.isfinite(points), axis=1)]
            if points.shape[0] < 4:
                return None
            low = np.percentile(points, 5.0, axis=0)
            high = np.percentile(points, 95.0, axis=0)
            extent = np.maximum(
                high - low,
                float(self.cfg["downsample_voxel_size"]),
            )
            return (low + high) * 0.5, extent

        left_geometry = geometry(left)
        right_geometry = geometry(right)
        if left_geometry is None or right_geometry is None:
            return None
        left_center, left_extent = left_geometry
        right_center, right_extent = right_geometry
        center_distance = float(
            np.linalg.norm(left_center - right_center)
        )
        if center_distance > float(
            self.cfg["same_class_centroid_max_m"]
        ):
            return None
        extent_ratio = float(
            np.max(
                np.maximum(
                    left_extent / right_extent,
                    right_extent / left_extent,
                )
            )
        )
        if extent_ratio > float(
            self.cfg["same_class_max_extent_ratio"]
        ):
            return None
        reference_major_extent = float(
            min(np.max(left_extent), np.max(right_extent))
        )
        if reference_major_extent <= 0.0:
            return None
        center_major_ratio = center_distance / reference_major_extent
        if center_major_ratio > maximum_center_ratio:
            return None

        visual_matrix = _pairwise_object_clip_cosines([left, right])
        visual_similarity = float(visual_matrix[0, 1])
        if (
            not math.isfinite(visual_similarity)
            or visual_similarity < minimum_visual
        ):
            return None
        return {
            "left_uuid": str(left.get("id", "") or ""),
            "right_uuid": str(right.get("id", "") or ""),
            "left_frame_count": len(left_frames),
            "right_frame_count": len(right_frames),
            "minimum_frame_gap": frame_gap,
            "center_distance_m": round(center_distance, 6),
            "reference_major_extent_m": round(
                reference_major_extent,
                6,
            ),
            "center_major_extent_ratio": round(
                center_major_ratio,
                6,
            ),
            "max_extent_ratio": round(extent_ratio, 6),
            "visual_similarity": round(visual_similarity, 6),
        }

    def _same_class_proximity_collapse(self, objects):
        """Merge same-class tracks only after physical-identity verification.

        Same-label partial views can evade ConceptGraphs' visual gate, but
        distance alone is not identity evidence: adjacent chairs and a cup on
        a table are often closer than the legacy 0.4 m threshold. A candidate
        now has to pass robust-center distance, bidirectional tolerant voxel
        coverage, and per-axis robust extent ratio. The accepted pair is still
        merged by ConceptGraphs' canonical ``merge_obj2_into_obj1``.

        Operator-labelled objects are never touched. Optional reviewed
        confusable-class groups share the same geometry gates.
        """
        centroid_max_m = float(
            self.cfg.get("same_class_centroid_max_m", 0.0)
        )
        if centroid_max_m <= 0.0:
            return objects
        n = len(objects)
        if n < 2:
            return objects
        merge_obj2_into_obj1 = self._cg.get("merge_obj2_into_obj1")
        if merge_obj2_into_obj1 is None:
            try:
                from conceptgraph.slam.utils import merge_obj2_into_obj1 as _m
                merge_obj2_into_obj1 = _m
            except Exception:
                return objects
        self._drop_derived_label_metadata(objects)

        def _grp(o):
            c = str(o.get("class_name", "")).lower()
            return self._merge_class_group.get(c, c)

        groups = [_grp(o) for o in objects]
        keep = [True] * n
        for i in range(n):
            if not keep[i]:
                continue
            for j in range(i + 1, n):
                if not keep[j]:
                    continue
                if objects[i].get("operator_label") or objects[j].get(
                    "operator_label"
                ):
                    continue
                if not groups[i] or groups[i] != groups[j]:
                    continue
                geometry = self._exact_duplicate_geometry_matrix(
                    [objects[i]],
                    [objects[j]],
                    centroid_max_m=centroid_max_m,
                    min_voxel_coverage=float(
                        self.cfg["same_class_min_voxel_coverage"]
                    ),
                    max_extent_ratio=float(
                        self.cfg["same_class_max_extent_ratio"]
                    ),
                )
                disjoint_evidence = None
                if not bool(geometry[0, 0]):
                    disjoint_evidence = (
                        self._same_class_disjoint_identity_evidence(
                            objects[i],
                            objects[j],
                        )
                    )
                if not bool(geometry[0, 0]) and disjoint_evidence is None:
                    continue
                if disjoint_evidence is not None:
                    self._quality_counters[
                        "same_class_disjoint_candidate_pairs"
                    ] += 1
                try:
                    objects[i] = merge_obj2_into_obj1(
                        obj1=objects[i], obj2=objects[j],
                        downsample_voxel_size=self.cfg["downsample_voxel_size"],
                        dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                        dbscan_eps=self.cfg["dbscan_eps"],
                        dbscan_min_points=self.cfg["dbscan_min_points"],
                        spatial_sim_type=self.cfg["spatial_sim_type"],
                        device=self._device,
                        run_dbscan=False,
                    )
                except Exception as e:  # noqa: BLE001
                    log.debug("same-class merge i=%d j=%d failed: %s", i, j, e)
                    continue
                if disjoint_evidence is not None:
                    self._quality_counters[
                        "same_class_disjoint_merged_pairs"
                    ] += 1
                    self._same_class_disjoint_recent.append(
                        {
                            **disjoint_evidence,
                            "association_group": groups[i],
                            "tick": int(self._tick_idx),
                        }
                    )
                    del self._same_class_disjoint_recent[:-32]
                keep[j] = False
                groups[i] = _grp(objects[i])

        MapObjectList = self._cg["MapObjectList"]
        out = MapObjectList()
        for k, alive in enumerate(keep):
            if alive:
                out.append(objects[k])
        return out

    # ── Cross-class merge (UNUSED) ────────────────────────────────────
    # Kept as a fallback for environments where pytorch3d cannot be
    # installed (the canonical path uses
    # `compute_overlap_matrix_general` which calls into pytorch3d's
    # 3D IoU). No callsite hits this today since pytorch3d ships in
    # the docker image; can be deleted once the ali-dev path is
    # confirmed stable across deployments.
    def _merge_overlap_no_p3d(self, objects):
        try:
            import numpy as np
            import torch
            import torch.nn.functional as F
        except Exception:
            return objects
        n = len(objects)
        if n < 2:
            return objects

        # Stack AABB extents (8 corners → min/max).
        bboxes = []
        for o in objects:
            bb = o["bbox"]
            try:
                pts = np.asarray(bb.get_box_points())
            except Exception:
                pts = np.asarray(o["pcd"].points)
            if pts.size == 0:
                bboxes.append((np.zeros(3), np.zeros(3)))
                continue
            mn = pts.min(axis=0); mx = pts.max(axis=0)
            bboxes.append((mn, mx))

        # CLIP feats stacked.
        clip_fts = []
        for o in objects:
            ft = o["clip_ft"]
            if isinstance(ft, np.ndarray):
                ft = torch.from_numpy(ft).float()
            clip_fts.append(F.normalize(ft.float().flatten(), dim=0))
        clip_mat = torch.stack(clip_fts, dim=0)  # (N, D)

        from conceptgraph.slam.utils import merge_obj2_into_obj1

        keep = [True] * n
        merged_into = list(range(n))  # union-find lite
        for i in range(n):
            if not keep[i]:
                continue
            for j in range(i + 1, n):
                if not keep[j]:
                    continue
                # AABB IoU
                a_min, a_max = bboxes[i]
                b_min, b_max = bboxes[j]
                inter_min = np.maximum(a_min, b_min)
                inter_max = np.minimum(a_max, b_max)
                inter_size = np.maximum(inter_max - inter_min, 0.0)
                inter_vol = float(np.prod(inter_size))
                a_vol = float(np.prod(a_max - a_min))
                b_vol = float(np.prod(b_max - b_min))
                union = a_vol + b_vol - inter_vol
                iou = inter_vol / union if union > 1e-9 else 0.0
                # Visual sim (cosine).
                vis = float(F.cosine_similarity(
                    clip_mat[i].unsqueeze(0), clip_mat[j].unsqueeze(0)
                ).item())
                # Spatial overlap proxy: max of (inter/a_vol, inter/b_vol).
                # Same intent as upstream's pcd-based ratio: "how much
                # of one volume is inside the other".
                ratio = 0.0
                if a_vol > 1e-9 and b_vol > 1e-9:
                    ratio = max(inter_vol / a_vol, inter_vol / b_vol)
                merge_thr = self.cfg["merge_overlap_thresh"]
                vis_thr = self.cfg["merge_visual_sim_thresh"]
                if (ratio >= merge_thr and vis >= vis_thr) or (iou > 0.3 and vis >= vis_thr):
                    # j is folded into i.
                    try:
                        objects[i] = merge_obj2_into_obj1(
                            obj1=objects[i], obj2=objects[j],
                            downsample_voxel_size=self.cfg["downsample_voxel_size"],
                            dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                            dbscan_eps=self.cfg["dbscan_eps"],
                            dbscan_min_points=self.cfg["dbscan_min_points"],
                            spatial_sim_type=self.cfg["spatial_sim_type"],
                            device=self._device,
                            run_dbscan=False,
                        )
                    except Exception as e:  # noqa: BLE001
                        log.debug("merge_obj2_into_obj1 failed (i=%d,j=%d): %s", i, j, e)
                        continue
                    keep[j] = False
                    # Update bbox/clip cache for i.
                    pts = np.asarray(objects[i]["pcd"].points)
                    if pts.size:
                        bboxes[i] = (pts.min(axis=0), pts.max(axis=0))
                    ft = objects[i]["clip_ft"]
                    if isinstance(ft, np.ndarray):
                        ft = torch.from_numpy(ft).float()
                    clip_mat[i] = F.normalize(ft.float().flatten(), dim=0)

        # Build new MapObjectList preserving order.
        MapObjectList = self._cg["MapObjectList"]
        out = MapObjectList()
        for k, alive in enumerate(keep):
            if alive:
                out.append(objects[k])
        return out

    # ── Camera-to-world transform ───────────────────────────────────
    def _selected_camera_frame(self) -> str:
        """Return the configured or live RGB optical frame without guessing."""
        configured = str(getattr(self, "_camera_frame", "") or "").strip().lstrip("/")
        if configured:
            return configured
        rgb_msg = self._rgb_msg()
        return str(
            getattr(getattr(rgb_msg, "header", None), "frame_id", "") or ""
        ).strip().lstrip("/")

    def _selected_base_frame(self) -> str:
        """Return Soma's live base frame, checked against explicit config."""
        live_fn = getattr(self, "_robot_base_frame_fn", None)
        live = str(live_fn() if live_fn is not None else "").strip().lstrip("/")
        configured = str(getattr(self, "_base_frame", "") or "").strip().lstrip("/")
        if live and configured and live != configured:
            signature = (live, configured)
            if getattr(self, "_invalid_configured_base_signature", None) != signature:
                log.warning(
                    "configured base_frame %r does not match Soma footprint frame %r",
                    configured,
                    live,
                )
                self._invalid_configured_base_signature = signature
            return ""
        return live or configured

    def _selected_odom_frame(self) -> str:
        """Return the live odometry parent frame without inventing a name."""
        if self._hub is None or not self._hub.has("odom"):
            return ""
        try:
            message, stamp_unix, count = self._hub.latest("odom")
        except Exception:  # noqa: BLE001
            return ""
        if message is None or stamp_unix <= 0.0 or count <= 0:
            return ""
        return str(
            getattr(getattr(message, "header", None), "frame_id", "") or ""
        ).strip().lstrip("/")

    def _current_occupancy_grid(self, expected_world_frame: str):
        """Return the current map grid only when its frame matches projection."""
        if self._hub is None or not self._hub.has("occupancy_grid"):
            return None
        try:
            msg, stamp_unix, count = self._hub.latest("occupancy_grid")
        except Exception:  # noqa: BLE001
            return None
        if msg is None or stamp_unix <= 0.0 or count <= 0:
            return None
        frame = str(
            getattr(getattr(msg, "header", None), "frame_id", "") or ""
        ).strip().lstrip("/")
        expected = str(expected_world_frame or "").strip().lstrip("/")
        if not frame or frame != expected:
            return None
        info = getattr(msg, "info", None)
        if (
            info is None
            or int(getattr(info, "width", 0) or 0) <= 0
            or int(getattr(info, "height", 0) or 0) <= 0
            or float(getattr(info, "resolution", 0.0) or 0.0) <= 0.0
        ):
            return None
        return msg

    @staticmethod
    def _message_stamp(message: Any):
        """Return a non-zero ROS header timestamp, otherwise ``None``."""
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            return None
        if (
            int(getattr(stamp, "sec", 0) or 0) == 0
            and int(getattr(stamp, "nanosec", 0) or 0) == 0
        ):
            return None
        return stamp

    @classmethod
    def _message_stamp_seconds(cls, message: Any) -> Optional[float]:
        """Return a message's ROS acquisition time in seconds."""
        stamp = cls._message_stamp(message)
        if stamp is None:
            return None
        return (
            float(getattr(stamp, "sec", 0) or 0)
            + float(getattr(stamp, "nanosec", 0) or 0) / 1_000_000_000.0
        )

    def _record_transform_result(self, source: str, transform: Any) -> None:
        """Expose which authoritative transform path admitted the frame."""
        try:
            matrix = np.asarray(transform, dtype=np.float64)
            yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
            pose = {
                "x_m": float(matrix[0, 3]),
                "y_m": float(matrix[1, 3]),
                "z_m": float(matrix[2, 3]),
                "yaw_rad": float(yaw),
            }
        except (TypeError, ValueError, IndexError):
            pose = {}
        counts = getattr(self, "_transform_source_counts", None)
        if counts is None:
            counts = {}
            self._transform_source_counts = counts
        counts[source] = int(counts.get(source, 0)) + 1
        self._last_transform_source = source
        self._last_camera_to_world_pose = pose

    def _adopt_world_from_odom_correction(self, transform: Any) -> bool:
        """Optionally rebase clouds for a non-standard moving world frame.

        In the normal REP-105 ``map→odom→base_link`` chain, ``map`` is the
        fixed global frame and a changing ``map←odom`` transform corrects live
        odometry. Historical points already expressed in ``map`` must remain
        fixed, so this compatibility path is disabled by default. It exists
        only for providers that explicitly redefine the world-frame basis
        without changing its frame id.

        ConceptGraphs association and merging remain unchanged; when enabled,
        this method only changes the coordinate basis of accumulated clouds.
        """
        try:
            current = np.asarray(transform, dtype=np.float64)
            if current.shape != (4, 4) or not np.all(np.isfinite(current)):
                return False
        except (TypeError, ValueError):
            return False

        previous = getattr(self, "_last_world_from_odom", None)
        if previous is None:
            self._last_world_from_odom = current.copy()
            return True
        try:
            delta = current @ np.linalg.inv(
                np.asarray(previous, dtype=np.float64)
            )
        except (ValueError, np.linalg.LinAlgError):
            return False
        translation_m = float(np.linalg.norm(delta[:3, 3]))
        yaw_rad = abs(
            math.atan2(float(delta[1, 0]), float(delta[0, 0]))
        )

        # Accumulate sub-millimetre/numerical changes against the same
        # baseline. Once their combined delta is measurable, apply it once.
        if translation_m < 0.002 and yaw_rad < 0.001:
            return True
        map_objects = getattr(self, "_map_objects", None)
        if not map_objects:
            self._last_world_from_odom = current.copy()
            return True
        if not bool(getattr(self, "_rebase_map_corrections", False)):
            self._last_world_from_odom = current.copy()
            return True
        if any(
            not hasattr(obj.get("pcd"), "transform")
            for obj in map_objects
        ):
            self._quality_counters["map_correction_rebase_failures"] += 1
            return False

        try:
            for obj in map_objects:
                pcd = obj["pcd"]
                pcd.transform(delta)
                bbox = obj.get("bbox")
                try:
                    if bbox is None or not hasattr(bbox, "transform"):
                        raise AttributeError("bbox has no transform")
                    bbox.transform(delta)
                except Exception:  # noqa: BLE001
                    # Default ConceptGraphs spatial mode uses an AABB. Some
                    # Open3D AABB versions cannot apply a rotated transform;
                    # rebuild it from the already transformed cloud.
                    get_bbox = getattr(
                        pcd, "get_axis_aligned_bounding_box", None
                    )
                    if get_bbox is None:
                        raise
                    obj["bbox"] = get_bbox()
        except Exception as error:  # noqa: BLE001
            self._quality_counters["map_correction_rebase_failures"] += 1
            log.warning(
                "[scene-cg] failed to rebase map correction: %s",
                error,
            )
            return False

        self._last_world_from_odom = current.copy()
        self._quality_counters["map_correction_rebases"] += 1
        self._map_correction_total_translation_m = (
            float(
                getattr(
                    self,
                    "_map_correction_total_translation_m",
                    0.0,
                )
            )
            + translation_m
        )
        self._map_correction_max_translation_m = max(
            float(
                getattr(
                    self,
                    "_map_correction_max_translation_m",
                    0.0,
                )
            ),
            translation_m,
        )
        self._map_correction_max_yaw_rad = max(
            float(getattr(self, "_map_correction_max_yaw_rad", 0.0)),
            yaw_rad,
        )
        log.info(
            "[scene-cg] rebased %d objects for map<-odom correction "
            "delta=%.3f m/%.3f rad",
            len(map_objects),
            translation_m,
            yaw_rad,
        )
        return True

    def _build_camera_to_map_transform(
        self,
        *,
        expected_world_frame: str = "",
        stamp: Any = None,
    ):
        """Return a validated camera-to-world transform.

        dev-next keeps its TF-first experiment, but both TF endpoints must come
        from the active camera and localizer. If TF is unavailable, the
        Atlas-routed pose and camera-extrinsics contracts are composed only
        when their world/body/camera frame chain is complete and current.
        """
        import numpy as np

        world_frame = str(
            expected_world_frame or self._world_frame_fn() or ""
        ).strip().lstrip("/")
        camera_frame = self._selected_camera_frame()
        if not world_frame or not camera_frame:
            return None

        if self._hub is not None:
            # RTAB-Map intentionally publishes its global map→odom correction
            # behind the newest sensor stamp. A full-chain exact lookup can
            # therefore remain in "future extrapolation" forever even though
            # high-rate odometry brackets the RGB-D observation. Preserve the
            # observation-time camera pose by splitting the TF chain:
            #
            #   T(map←camera,t) =
            #       T(map←odom,latest) @ T(odom←camera,t)
            #
            # Only the slowly changing global correction is latest; robot
            # motion is always evaluated at the image/depth timestamp.
            odom_frame = self._selected_odom_frame() if stamp is not None else ""
            if odom_frame and odom_frame != world_frame:
                camera_to_odom = self._hub.lookup_transform_4x4(
                    camera_frame,
                    odom_frame,
                    stamp=stamp,
                )
                odom_to_world = self._hub.lookup_transform_4x4(
                    odom_frame,
                    world_frame,
                )
                if camera_to_odom is not None and odom_to_world is not None:
                    if not self._adopt_world_from_odom_correction(
                        odom_to_world
                    ):
                        return None
                    result = (odom_to_world @ camera_to_odom).astype(
                        np.float32
                    )
                    self._record_transform_result(
                        "stamped_odom_plus_current_map_correction",
                        result,
                    )
                    return result
                # Do not fall back to an older full-chain map transform after
                # an odom endpoint has been established: mixing historical and
                # current map corrections is exactly what stretches persistent
                # object clouds.
                return None

            transform = self._hub.lookup_transform_4x4(
                camera_frame,
                world_frame,
                stamp=stamp,
            )
            if transform is not None:
                result = transform.astype(np.float32)
                self._record_transform_result(
                    "stamped_full_tf" if stamp is not None else "latest_full_tf",
                    result,
                )
                return result

        pose_transform = self._slot_pose_transform(
            expected_world_frame=world_frame,
        )
        if pose_transform is None:
            return None
        pose_matrix, body_frame = pose_transform
        extrinsics_matrix = self._slot_extrinsics_4x4(body_frame)
        if extrinsics_matrix is None:
            return None
        result = (pose_matrix @ extrinsics_matrix).astype(np.float32)
        self._record_transform_result("contract_pose_plus_extrinsics", result)
        return result

    def _slot_pose_transform(self, *, expected_world_frame: str = ""):
        """Return the body pose matrix and its validated frame endpoint."""
        if self._hub is None:
            return None
        expected_world = str(
            expected_world_frame or self._world_frame_fn() or ""
        ).strip().lstrip("/")
        if not expected_world:
            return None
        expected_base = self._selected_base_frame()
        for kind in ("pose", "odom"):
            if not self._hub.has(kind):
                continue
            msg, stamp_unix, _count = self._hub.latest(kind)
            if msg is None or stamp_unix <= 0:
                continue
            max_age_s = float(getattr(self, "_pose_max_age_s", 0.0) or 0.0)
            if max_age_s > 0.0 and time.time() - stamp_unix > max_age_s:
                continue
            world_frame = str(
                getattr(getattr(msg, "header", None), "frame_id", "") or ""
            ).strip().lstrip("/")
            if world_frame != expected_world:
                continue
            body_frame = self._body_frame_for_pose_source(
                kind,
                msg,
                expected_base=expected_base,
            )
            if body_frame is None:
                continue
            pose = (
                msg.pose.pose
                if hasattr(msg, "pose") and hasattr(msg.pose, "pose")
                else msg.pose
            )
            quaternion = pose.orientation
            return (
                _quat_xyz_to_matrix(
                    float(quaternion.x),
                    float(quaternion.y),
                    float(quaternion.z),
                    float(quaternion.w),
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ),
                body_frame,
            )
        return None

    def _body_frame_for_pose_source(
        self,
        kind: str,
        msg,
        *,
        expected_base: str = "",
    ) -> str | None:
        """Resolve a body endpoint without a robot-model default."""
        expected_base = str(expected_base or "").strip().lstrip("/")
        if kind != "odom":
            return expected_base or None
        odom_child = str(getattr(msg, "child_frame_id", "") or "").strip().lstrip("/")
        if not odom_child:
            return None
        if expected_base and odom_child != expected_base:
            signature = (odom_child, expected_base)
            if getattr(self, "_invalid_odom_body_signature", None) != signature:
                log.warning(
                    "ignoring odometry with child frame %r; active base frame is %r",
                    odom_child,
                    expected_base,
                )
                self._invalid_odom_body_signature = signature
            return None
        return expected_base or odom_child

    def _slot_extrinsics_4x4(self, expected_parent: str | None = None):
        """Return the camera mount only for the selected frame endpoints."""
        expected_parent = str(
            expected_parent or self._selected_base_frame() or ""
        ).strip().lstrip("/")
        expected_child = self._selected_camera_frame()
        if (
            self._hub is None
            or not self._hub.has("camera_extrinsics")
            or not expected_parent
            or not expected_child
        ):
            return None
        msg, stamp_unix, _count = self._hub.latest("camera_extrinsics")
        if msg is None or stamp_unix <= 0:
            return None
        parent_frame = str(
            getattr(getattr(msg, "header", None), "frame_id", "") or ""
        ).strip().lstrip("/")
        child_frame = str(getattr(msg, "child_frame_id", "") or "").strip().lstrip("/")
        if parent_frame != expected_parent or child_frame != expected_child:
            signature = (parent_frame, child_frame, expected_parent, expected_child)
            if getattr(self, "_invalid_extrinsics_signature", None) != signature:
                log.warning(
                    "ignoring camera extrinsics with frames %r ← %r; expected "
                    "%r ← %r",
                    parent_frame,
                    child_frame,
                    expected_parent,
                    expected_child,
                )
                self._invalid_extrinsics_signature = signature
            return None
        translation = msg.transform.translation
        quaternion = msg.transform.rotation
        return _quat_xyz_to_matrix(
            float(quaternion.x),
            float(quaternion.y),
            float(quaternion.z),
            float(quaternion.w),
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )

    # ── Project MapObjectList → ObjectRegistry ──────────────────────
    def _project_to_registry(
        self,
        *,
        observed_uuids: Optional[set[str]] = None,
        visible_miss_uuids: Optional[set[str]] = None,
        frame_seq: int = 0,
        observed_at: Optional[float] = None,
    ) -> None:
        """Project persistent geometry without replaying positive sightings.

        ``observed_uuids`` contains only objects matched or created by the
        current healthy frame. ``visible_miss_uuids`` contains unmatched
        objects whose old location had clear depth evidence of absence.
        Historical map membership alone updates neither ``last_seen`` nor the
        observation count.

        Runs from the worker thread; the registry uses an asyncio.Lock,
        so the actual mutation is scheduled onto the asyncio loop via
        ``run_coroutine_threadsafe``. The worker waits for that mutation to
        finish before returning: otherwise a cleanup merge can already be
        visible in ``/api/objects3d`` while its removed registry record remains
        live in ``/api/state`` until a later loop turn, manufacturing a
        duplicate object for users and evaluators."""
        if self._map_objects is None or self._asyncio_loop is None:
            return
        try:
            import numpy as np
        except Exception:
            return
        # Cache that maps each MapObjectList entry's stable uuid →
        # registry object_id. Without this, every tick we'd doom +
        # re-insert every object, which (a) churns the user-visible
        # `<cls>_NNN` suffix into the thousands, (b) resets
        # observation_count, (c) breaks consumers that hold an oid
        # across ticks. Initialised once in __init__/start.
        if not hasattr(self, "_uuid_to_oid"):
            self._uuid_to_oid = {}
        snapshots = []
        for obj in self._map_objects:
            try:
                pcd = obj["pcd"]
                pts = np.asarray(pcd.points)
                if pts.size == 0:
                    continue
                # Yaw-only bbox via a robust minimum-area rectangle on the XY
                # footprint. We used to call Open3D's oriented box (qhull can
                # segfault on near-coplanar clouds), then used PCA (partial
                # surfaces often biased its axis toward the camera/map grid).
                # The pure-numpy hull-edge fit is deterministic and keeps the
                # world-upright yaw-only contract.
                # Drop NaN/Inf rows BEFORE computing the bbox — depth
                # back-projection produces them when depth is 0
                # (sky/holes), and a single Inf in a coordinate
                # poisons mean()/max()/min() into NaN/Inf, which then
                # breaks JSON serialisation downstream
                # ("Out of range float values are not JSON compliant").
                finite_pts = pts[np.all(np.isfinite(pts), axis=1)]
                if finite_pts.shape[0] < 4:
                    continue
                pts = finite_pts
                bbox_result = _robust_yaw_bbox(
                    pts,
                    low_percentile=self._bbox_low_percentile,
                    high_percentile=self._bbox_high_percentile,
                )
                if bbox_result is None:
                    continue
                obb_center, obb_extent, obb_yaw = bbox_result

                cls = _canon_class(
                    obj.get(
                        "resolved_class_name",
                        obj.get("class_name", "object"),
                    )
                )
                conf_list = obj.get("conf", [])
                conf = float(np.mean(conf_list)) if conf_list else 0.5
                image_indices = list(obj.get("image_idx", ()) or ())
                unique_image_indices = {
                    int(value)
                    for value in image_indices
                    if isinstance(value, (int, np.integer))
                }
                confirmation_min = int(
                    getattr(
                        self,
                        "_confirmation_min_unique_frames",
                        1,
                    )
                )
                singleton_confidence = float(
                    getattr(
                        self,
                        "_confirmation_singleton_min_mean_confidence",
                        0.0,
                    )
                )
                (
                    _confirmation_unique_frames,
                    confirmation_mean_confidence,
                    confirmation_confidence_fast_path,
                    confirmation_ready,
                ) = _object_confirmation_status(
                    obj,
                    min_unique_frames=confirmation_min,
                    singleton_min_mean_confidence=singleton_confidence,
                )
                snapshots.append({
                    # The MapObjectList entry's stable uuid — this is
                    # the key the registry uses to decide insert vs.
                    # update vs. evict, so identifiers stop churning.
                    "uuid": str(obj.get("id", "")),
                    "visibility_debug": copy.deepcopy(
                        getattr(self, "_visibility_diagnostics", {}).get(
                            str(obj.get("id", "") or ""),
                            {},
                        )
                    ),
                    "cls": cls,
                    "x": float(obb_center[0]),
                    "y": float(obb_center[1]),
                    "z": float(obb_center[2]),
                    "yaw": float(obb_yaw),
                    "size_x": float(max(0.05, obb_extent[0])),
                    "size_y": float(max(0.05, obb_extent[1])),
                    "size_z": float(max(0.05, obb_extent[2])),
                    "confidence": max(0.0, min(1.0, conf)),
                    "label_confidence": max(
                        0.0,
                        min(1.0, float(obj.get("label_confidence", 0.0) or 0.0)),
                    ),
                    "label_provisional": bool(
                        obj.get("label_provisional", True)
                    ),
                    "label_evidence_count": int(
                        obj.get("label_evidence_count", 0) or 0
                    ),
                    "label_candidates": list(
                        obj.get("label_candidates", ()) or ()
                    ),
                    "label_source": str(
                        obj.get("label_source", "model") or "model"
                    ),
                    "operator_label": str(
                        obj.get("operator_label", "") or ""
                    ),
                    "geometry_point_count": int(pts.shape[0]),
                    "geometry_view_count": int(obj.get("num_detections", 1) or 1),
                    "cg_num_detections": int(
                        obj.get("num_detections", 1) or 1
                    ),
                    "cg_image_idx_count": len(image_indices),
                    "cg_unique_image_idx_count": len(unique_image_indices),
                    "latest_observed_frame": (
                        max(unique_image_indices)
                        if unique_image_indices
                        else -1
                    ),
                    "confirmation_ready": confirmation_ready,
                    "confirmation_min_unique_frames": confirmation_min,
                    "confirmation_mean_confidence": (
                        confirmation_mean_confidence
                    ),
                    "confirmation_confidence_fast_path": (
                        confirmation_confidence_fast_path
                    ),
                    "confirmation_singleton_min_mean_confidence": (
                        singleton_confidence
                    ),
                })
            except Exception:  # noqa: BLE001
                continue

        # Schedule the registry mutation on the asyncio loop so it serializes
        # with the snapshot reader (same asyncio.Lock), then wait for the
        # reconciliation to become visible before this worker tick completes.
        # The production detector loop always invokes this method through
        # run_in_executor; blocking the event-loop thread here would deadlock.
        if threading.get_ident() == getattr(self, "_asyncio_thread_id", None):
            self._quality_counters["registry_projection_failures"] += 1
            raise RuntimeError(
                "_project_to_registry must run outside the asyncio loop thread"
            )
        future = None
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._apply_snapshot(
                    snapshots,
                    observed_uuids=observed_uuids,
                    visible_miss_uuids=visible_miss_uuids,
                    frame_seq=frame_seq,
                    observed_at=observed_at,
                ),
                self._asyncio_loop,
            )
            future.result(timeout=5.0)
        except concurrent.futures.TimeoutError as e:
            if future is not None:
                future.cancel()
            self._quality_counters["registry_projection_failures"] += 1
            raise RuntimeError(
                "registry projection did not finish within 5 seconds"
            ) from e
        except Exception as e:  # noqa: BLE001
            self._quality_counters["registry_projection_failures"] += 1
            raise RuntimeError("registry projection failed") from e

    async def _apply_snapshot(
        self,
        snapshots: list[dict],
        *,
        observed_uuids: Optional[set[str]] = None,
        visible_miss_uuids: Optional[set[str]] = None,
        frame_seq: int = 0,
        observed_at: Optional[float] = None,
    ) -> None:
        """Reconcile concept-graphs MapObjectList state into the
        registry **incrementally**. Robot self-record and non-
        perception objects are preserved; for our own objects we:

          - REUSE the existing registry record when the
            MapObjectList uuid is already mapped (always refresh fused
            pose / bbox / confidence; bump observation_count only for
            current-frame evidence)
          - RE-BIND a warm-restored object of the same class within
            the merge-distance gate to a fresh detection before minting
            a new id, so a remembered object keeps its stable id on
            re-observation instead of churning a new suffix
          - ADOPT an orphaned live record (its cg_uuid vacated this tick
            by merge-dedup or filter-cull) of the same class within the
            merge gate, so the object keeps its id + accumulated count
            across a uuid swap instead of resetting to obs=1
          - INSERT a new record only when nothing above matches
          - EVICT registry records whose uuid is no longer present in
            the latest MapObjectList (concept-graphs merged or
            filtered them out) — warm-restored records that have not yet
            been re-bound are exempt (they were never in this process's
            MapObjectList); if never re-seen, mark_stale keeps them
            missing rather than deleting them

        Order matters: bind/adopt/insert runs FIRST, eviction AFTER, so
        the new winning uuid of a merged/regenerated object can adopt the
        record the old uuid is about to vacate (the orphan must still
        exist when the snapshot loop runs). Records touched this tick are
        spared from eviction via `adopted_oids`; observation_count is
        registry-owned (one per tick seen), so it survives a uuid swap.

        The previous "drop everything, re-insert everything" approach
        churned object_ids on every tick (the visible `<cls>_NNN`
        suffix climbed into the thousands), reset observation_count,
        and broke any consumer that held an oid across ticks.
        """
        now = time.time() if observed_at is None else float(observed_at)
        # Compatibility for direct callers predating FrameObservation: an
        # omitted set means every supplied snapshot is a positive observation.
        # Production always passes an explicit set, including an empty set.
        if observed_uuids is None:
            observed_uuids = {
                str(s.get("uuid", "") or "")
                for s in snapshots
                if str(s.get("uuid", "") or "")
            }
        if visible_miss_uuids is None:
            visible_miss_uuids = set()
        if not hasattr(self, "_expired_uuids"):
            self._expired_uuids = set()
        if not hasattr(self, "_missing_uuids"):
            self._missing_uuids = set()
        if not hasattr(self, "_operator_labels"):
            self._operator_labels = {}
        if not hasattr(self, "_operator_geometry_oids"):
            self._operator_geometry_oids = set()
        live_uuids = {s["uuid"] for s in snapshots if s.get("uuid")}
        async with self._registry.lock():
            wf = self._world_frame_fn()
            if not wf:
                log.warning(
                    "withholding ConceptGraphs snapshot because the world frame "
                    "is unknown"
                )
                return
            # oids touched this tick (bound / adopted / inserted). Used to (a)
            # stop two detections claiming the same orphan and (b) spare a
            # touched record from the eviction sweep below.
            adopted_oids: set[str] = set()
            # Bind / adopt / insert FIRST; eviction runs after. The new winning
            # uuid of a merged or regenerated object must be able to adopt the
            # record the old uuid is about to vacate, so the orphan has to still
            # exist while this loop runs.
            for s in snapshots:
                u = s.get("uuid", "")
                is_observed = bool(u and u in observed_uuids)
                pose = Pose3D(
                    x=s["x"], y=s["y"], z=s["z"],
                    yaw=s.get("yaw", 0.0), frame_id=wf,
                )
                bbox = BBox3D(
                    size_x=s["size_x"], size_y=s["size_y"], size_z=s["size_z"],
                    yaw=s.get("yaw", 0.0), frame_id=wf,
                )
                oid = self._uuid_to_oid.get(u) if u else None
                existing = self._registry._objects.get(oid) if oid else None
                rebind_kind = ""
                identity_claim_ready = bool(
                    s.get("confirmation_ready", True)
                )
                if existing is None and identity_claim_ready:
                    # No uuid binding yet (first sighting of this uuid). Before
                    # minting a new id, try to re-adopt a stable record so the
                    # object keeps its id (and accumulated count) across a uuid
                    # swap: first a warm-restored object of the same class, then
                    # a live record whose cg_uuid vacated this tick (merge-dedup
                    # / filter-cull). A fresh uuid must first satisfy the same
                    # temporal/confidence confirmation gate as a new insert:
                    # otherwise a one-frame fragment can steal a nearby
                    # published identity and bypass admission entirely. An
                    # already-bound uuid still updates normally above because
                    # it is continuity evidence for the same confirmed track.
                    #
                    # `is_observed` is deliberately not required here. A
                    # periodic ConceptGraphs cleanup can change the surviving
                    # UUID after the sensor frame that supplied confirmation.
                    # Requiring current-frame membership strands a confirmed
                    # multi-frame object in CG with no Registry binding. The
                    # historical evidence time/count below is preserved, so
                    # repairing that binding never fabricates a new positive
                    # observation.
                    # Both rebind paths also use the same class +
                    # merge-distance gate, so re-binding is no looser than
                    # same-tick merging.
                    existing = self._rebind_restored(s["cls"], pose)
                    if existing is not None:
                        rebind_kind = "restored"
                    # Cross-tick re-bind: a record soft-evicted (`missing`) by a
                    # previous cull / uuid-churn tick is reclaimed by this
                    # detection, so the object keeps its id + accumulated
                    # observation_count instead of minting a fresh id at obs=1.
                    # Same class + merge-distance gate as restored-rebind. This
                    # is the cross-tick complement to _adopt_orphan (which only
                    # rescues a uuid swap within the SAME tick).
                    if existing is None:
                        existing = self._registry.find_rebindable(
                            s["cls"], pose,
                            float(
                                self.cfg.get(
                                    "identity_rebind_max_distance_m",
                                    0.45,
                                )
                            ),
                            only_missing=True,
                            exclude_oids=adopted_oids,
                        )
                        if existing is not None:
                            rebind_kind = "cross_tick"
                    # Orphan-adoption needs a uuid to rebind to; a uuid-less
                    # detection can't bind durably, so it must not consume an
                    # orphan another (uuid-bearing) detection could re-adopt
                    # this same tick.
                    if existing is None and u:
                        existing = self._adopt_orphan(
                            s["cls"], pose, live_uuids, adopted_oids,
                        )
                        if existing is not None:
                            rebind_kind = "same_tick_orphan"
                    if existing is not None:
                        if rebind_kind:
                            rebind_distance = math.sqrt(
                                (existing.pose.x - pose.x) ** 2
                                + (existing.pose.y - pose.y) ** 2
                                + (existing.pose.z - pose.z) ** 2
                            )
                            existing.attributes["identity_rebind_count"] = (
                                int(
                                    existing.attributes.get(
                                        "identity_rebind_count", 0
                                    )
                                    or 0
                                )
                                + 1
                            )
                            existing.attributes["identity_rebind_last_kind"] = (
                                rebind_kind
                            )
                            existing.attributes[
                                "identity_rebind_last_distance_m"
                            ] = rebind_distance
                            existing.attributes[
                                "identity_rebind_max_distance_m"
                            ] = max(
                                float(
                                    existing.attributes.get(
                                        "identity_rebind_max_distance_m", 0.0
                                    )
                                    or 0.0
                                ),
                                rebind_distance,
                            )
                        existing.attributes.pop("restored", None)
                        if (
                            existing.missing
                            and existing.attributes.get("missing_reason")
                            == "cg_orphan"
                        ):
                            # Internal CG uuid/filter churn is not negative
                            # scene evidence. A newly confirmed replacement
                            # track restores the live identity even when the
                            # cleanup survivor is projected after its last
                            # sensor frame. Depth-confirmed absence uses a
                            # different reason and remains missing until an
                            # actual positive observation below.
                            existing.missing = False
                            existing.attributes.pop("missing_reason", None)
                        # Rebind to the new uuid only when we have one. A
                        # uuid-less detection (CG object missing `id`) can't
                        # establish a durable binding, so leave the adopted
                        # record's old cg_uuid mapping intact rather than
                        # popping it and orphaning the record (it is spared
                        # this tick via adopted_oids regardless).
                        if u:
                            old_uuid = existing.attributes.get("cg_uuid")
                            if old_uuid and old_uuid != u:
                                self._uuid_to_oid.pop(old_uuid, None)
                            existing.attributes["cg_uuid"] = u
                            self._uuid_to_oid[u] = existing.object_id
                if existing is not None:
                    operator_label = str(
                        existing.attributes.get("operator_label", "") or ""
                    ).strip()
                    if operator_label:
                        s["cls"] = operator_label
                        s["label_confidence"] = 1.0
                        s["label_provisional"] = False
                        s["label_source"] = "operator"
                        s["operator_label"] = operator_label
                        self._operator_labels[existing.object_id] = operator_label
                    # Geometry is the persistent fused MapObject snapshot and
                    # may be projected every tick. Positive-observation state
                    # changes only when this UUID received current-frame
                    # evidence.
                    operator_geometry = bool(
                        existing.attributes.get("operator_geometry")
                        or existing.object_id in self._operator_geometry_oids
                    )
                    if operator_geometry:
                        self._operator_geometry_oids.add(existing.object_id)
                    else:
                        existing.pose = pose
                        existing.bbox = bbox
                    existing.cls = s["cls"]
                    existing.confidence = max(0.0, min(1.0, s["confidence"]))
                    existing.attributes["observation_lifecycle"] = "visibility"
                    if not operator_geometry:
                        existing.attributes["geometry_source"] = "rgbd_multi_view"
                        existing.attributes["bbox_method"] = (
                            "yaw_min_area_quantile"
                        )
                        existing.attributes["geometry_point_count"] = int(
                            s.get("geometry_point_count", 0)
                        )
                        existing.attributes["geometry_view_count"] = int(
                            s.get("geometry_view_count", 1)
                        )
                        existing.attributes["cg_num_detections"] = int(
                            s.get("cg_num_detections", 1)
                        )
                        existing.attributes["cg_image_idx_count"] = int(
                            s.get("cg_image_idx_count", 0)
                        )
                        existing.attributes[
                            "cg_unique_image_idx_count"
                        ] = int(s.get("cg_unique_image_idx_count", 0))
                        existing.attributes["geometry_navigation_grade"] = bool(
                            getattr(self, "_require_occupancy_bounds", False)
                        )
                    existing.attributes["label_confidence"] = float(
                        s.get("label_confidence", 0.0)
                    )
                    existing.attributes["label_provisional"] = bool(
                        s.get("label_provisional", True)
                    )
                    existing.attributes["label_evidence_count"] = int(
                        s.get("label_evidence_count", 0)
                    )
                    existing.attributes["label_candidates"] = list(
                        s.get("label_candidates", ()) or ()
                    )
                    existing.attributes["label_source"] = str(
                        s.get("label_source", "model") or "model"
                    )
                    existing.attributes["visibility_debug"] = copy.deepcopy(
                        s.get("visibility_debug", {}) or {}
                    )
                    if s.get("operator_label"):
                        existing.attributes["operator_label"] = str(
                            s["operator_label"]
                        )
                    existing.attributes["navigation_grade"] = bool(
                        existing.attributes.get("geometry_navigation_grade", False)
                        and not s.get("label_provisional", True)
                    )
                    if is_observed:
                        seeded_frame = existing.attributes.pop(
                            "registry_projection_seeded_frame",
                            None,
                        )
                        existing.last_seen = now
                        existing.missing = False
                        existing.attributes.pop("missing_reason", None)
                        self._missing_uuids.discard(u)
                        if seeded_frame != frame_seq:
                            existing.observation_count += 1
                        existing.attributes["last_observed_frame"] = frame_seq
                        existing.attributes["last_observed_unix"] = now
                        existing.attributes["consecutive_visible_misses"] = 0
                    adopted_oids.add(existing.object_id)
                elif identity_claim_ready:
                    latest_observed_frame = int(
                        s.get("latest_observed_frame", -1)
                    )
                    if (
                        latest_observed_frame >= 0
                        and frame_seq >= latest_observed_frame
                    ):
                        evidence_time = now - (
                            (frame_seq - latest_observed_frame)
                            * max(0.0, float(self._period_s))
                        )
                    else:
                        evidence_time = now
                    obj = self._registry.insert_object(
                        cls=s["cls"],
                        pose=pose,
                        bbox=bbox,
                        confidence=s["confidence"],
                        now=evidence_time,
                        source="concept_graphs",
                    )
                    # Reconstruct the confirmed historical evidence without
                    # counting this Registry projection as a sensor
                    # observation. A later projection of the same frame is
                    # deduplicated by `last_observed_frame` above.
                    obj.observation_count = max(
                        1,
                        int(s.get("cg_unique_image_idx_count", 1) or 1),
                    )
                    if u:
                        obj.attributes["cg_uuid"] = u
                        self._uuid_to_oid[u] = obj.object_id
                    obj.attributes["observation_lifecycle"] = "visibility"
                    obj.attributes["geometry_source"] = "rgbd_multi_view"
                    obj.attributes["bbox_method"] = "yaw_min_area_quantile"
                    obj.attributes["geometry_point_count"] = int(
                        s.get("geometry_point_count", 0)
                    )
                    obj.attributes["geometry_view_count"] = int(
                        s.get("geometry_view_count", 1)
                    )
                    obj.attributes["cg_num_detections"] = int(
                        s.get("cg_num_detections", 1)
                    )
                    obj.attributes["cg_image_idx_count"] = int(
                        s.get("cg_image_idx_count", 0)
                    )
                    obj.attributes["cg_unique_image_idx_count"] = int(
                        s.get("cg_unique_image_idx_count", 0)
                    )
                    obj.attributes["geometry_navigation_grade"] = bool(
                        getattr(self, "_require_occupancy_bounds", False)
                    )
                    obj.attributes["label_confidence"] = float(
                        s.get("label_confidence", 0.0)
                    )
                    obj.attributes["label_provisional"] = bool(
                        s.get("label_provisional", True)
                    )
                    obj.attributes["label_evidence_count"] = int(
                        s.get("label_evidence_count", 0)
                    )
                    obj.attributes["label_candidates"] = list(
                        s.get("label_candidates", ()) or ()
                    )
                    obj.attributes["label_source"] = str(
                        s.get("label_source", "model") or "model"
                    )
                    obj.attributes["visibility_debug"] = copy.deepcopy(
                        s.get("visibility_debug", {}) or {}
                    )
                    if s.get("operator_label"):
                        obj.attributes["operator_label"] = str(
                            s["operator_label"]
                        )
                        self._operator_labels[obj.object_id] = str(
                            s["operator_label"]
                        )
                    obj.attributes["navigation_grade"] = bool(
                        obj.attributes.get("geometry_navigation_grade", False)
                        and not s.get("label_provisional", True)
                    )
                    obj.attributes["last_observed_frame"] = (
                        latest_observed_frame
                    )
                    obj.attributes["last_observed_unix"] = evidence_time
                    if not is_observed:
                        obj.attributes[
                            "registry_projection_seeded_frame"
                        ] = latest_observed_frame
                    obj.attributes["consecutive_visible_misses"] = 0
                    self._missing_uuids.discard(u)
                    adopted_oids.add(obj.object_id)

            # Negative evidence is also frame-scoped. A missing vote is
            # accepted only when the current RGB-D/model pass was healthy and
            # depth showed clear space behind the object's old location.
            for u in visible_miss_uuids:
                oid = self._uuid_to_oid.get(u)
                obj = self._registry._objects.get(oid) if oid else None
                if obj is None or obj.attributes.get("is_robot"):
                    continue
                misses = int(obj.attributes.get("consecutive_visible_misses", 0)) + 1
                obj.attributes["consecutive_visible_misses"] = misses
                obj.attributes["last_visible_miss_frame"] = frame_seq
                if misses >= self._visible_miss_threshold:
                    obj.missing = True
                    obj.attributes["missing_reason"] = "visible_absence"
                    self._missing_uuids.add(u)

            # Evict registry records whose source uuid is gone. Runs AFTER
            # bind/adopt so a record adopted above (cg_uuid rebound to a live
            # uuid, and in adopted_oids) is spared automatically; only
            # genuinely-vanished records remain doomed. We only touch our own
            # records — robot self / surfaces / other-source objects untouched.
            doomed = []
            for oid, obj in list(self._registry._objects.items()):
                if oid in adopted_oids:
                    continue
                if obj.attributes.get("is_robot"):
                    continue
                src = obj.attributes.get("source")
                if src not in ("concept_graphs", None):
                    continue
                # Warm-restored objects were never in this process's
                # MapObjectList, so the uuid-membership rule below can't apply.
                # The loop above re-binds them by class+pose; if never re-seen,
                # mark_stale keeps them missing instead of deleting them here.
                if obj.attributes.get("restored"):
                    continue
                cg_uuid = obj.attributes.get("cg_uuid")
                if cg_uuid is None or cg_uuid not in live_uuids:
                    doomed.append(obj)
                    if cg_uuid is not None:
                        self._uuid_to_oid.pop(cg_uuid, None)
            # Soft-evict (mark `missing`, release the uuid) rather than delete,
            # so a re-detection within SCENE_OBJECT_TTL_SEC can re-bind the same
            # id + observation_count via find_rebindable — fixing the cull→
            # re-detect obs reset. A record concept-graphs deduped lingers
            # `missing` until the TTL prune; a future detection at that spot
            # re-binds the nearest *live* record (the dedup survivor), not the
            # stale one, so soft eviction does not resurrect duplicates.
            for obj in doomed:
                self._registry.soft_evict(obj)
            uuid_by_oid = {
                oid: uuid_value
                for uuid_value, oid in self._uuid_to_oid.items()
            }
            pruned_oids = self._registry.prune_expired(
                now,
                self._object_ttl_s,
            )
            for oid in pruned_oids:
                uuid_value = uuid_by_oid.get(oid)
                if uuid_value:
                    self._uuid_to_oid.pop(uuid_value, None)
                    self._expired_uuids.add(uuid_value)
                    self._missing_uuids.discard(uuid_value)

    def _rebind_restored(self, cls: str, pose: Pose3D) -> Optional[SceneObject]:
        """Nearest warm-restored, not-yet-rebound object of class `cls` whose
        centroid is within the merge-distance gate of `pose`, or None.

        Lets a live detection re-adopt a remembered object's stable id instead
        of spawning a duplicate. Uses the dedicated, stricter
        `identity_rebind_max_distance_m` gate rather than the furniture-scale
        multi-view association radius. Caller must hold the registry lock; the caller
        clears the matched object's `restored` flag so each restored object
        binds at most one detection per tick."""
        max_d = float(
            self.cfg.get("identity_rebind_max_distance_m", 0.45)
        )
        best: Optional[SceneObject] = None
        best_d = max_d
        for obj in self._registry._objects.values():
            if not obj.attributes.get("restored") or obj.cls != cls:
                continue
            d = math.sqrt(
                (obj.pose.x - pose.x) ** 2
                + (obj.pose.y - pose.y) ** 2
                + (obj.pose.z - pose.z) ** 2
            )
            if d <= best_d:
                best_d = d
                best = obj
        return best

    def _adopt_orphan(
        self,
        cls: str,
        pose: Pose3D,
        live_uuids: set[str],
        adopted_oids: set[str],
    ) -> Optional[SceneObject]:
        """Nearest same-class concept-graphs record whose cg_uuid vacated this
        tick (merge-dedup or filter-cull), within the merge-distance gate and
        not already adopted, or None.

        Lets the new uuid of a merged/regenerated object inherit the stable id
        (and accumulated observation_count) of the record the old uuid is about
        to vacate, instead of churning a fresh id at obs=1. Considers only
        records whose cg_uuid is set but absent from this tick's `live_uuids`
        (the eviction sweep removed earlier-tick orphans, so a surviving orphan
        was vacated this tick). The same dedicated identity-rebind distance
        gate applies. Caller must hold the registry lock and
        rebinds the matched record's cg_uuid + records its oid in adopted_oids."""
        max_d = float(
            self.cfg.get("identity_rebind_max_distance_m", 0.45)
        )
        best: Optional[SceneObject] = None
        best_d = max_d
        for obj in self._registry._objects.values():
            if obj.object_id in adopted_oids or obj.cls != cls:
                continue
            if obj.attributes.get("is_robot") or obj.attributes.get("restored"):
                continue
            if obj.attributes.get("source") not in ("concept_graphs", None):
                continue
            cg_uuid = obj.attributes.get("cg_uuid")
            if cg_uuid is None or cg_uuid in live_uuids:
                continue
            d = math.sqrt(
                (obj.pose.x - pose.x) ** 2
                + (obj.pose.y - pose.y) ** 2
                + (obj.pose.z - pose.z) ** 2
            )
            if d <= best_d:
                best_d = d
                best = obj
        return best


# ── Geometry helpers ──────────────────────────────────────────────────────
def _quat_xyz_to_matrix(qx: float, qy: float, qz: float, qw: float,
                        tx: float, ty: float, tz: float):
    """Quaternion + translation → 4×4 homogeneous transform.

    Used by the camera-to-world compatibility path when TF is unavailable
    and Scene composes the pose and validated extrinsics contract slots.
    """
    import numpy as np
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return None
    s = 2.0 / n
    wx, wy, wz = s * qw * qx, s * qw * qy, s * qw * qz
    xx, xy, xz = s * qx * qx, s * qx * qy, s * qx * qz
    yy, yz, zz = s * qy * qy, s * qy * qz, s * qz * qz
    R = np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


# ── Image / depth helpers ────────────────────────────────────────────────
def _image_msg_to_bgr(msg: Any) -> Optional[Any]:
    """sensor_msgs/Image (rgb8 / bgr8 / rgba8 / bgra8 / mono8 / jpeg) → numpy BGR.

    Some cameras emit bgra8 (4-channel); leaving that out of the supported
    set silently dropped every frame. Both rgba8 and bgra8 paths here.
    """
    try:
        import numpy as np
    except Exception:
        return None
    encoding = (getattr(msg, "encoding", "") or "").lower()
    data = bytes(msg.data) if not isinstance(msg.data, bytes) else msg.data
    h = int(getattr(msg, "height", 0))
    w = int(getattr(msg, "width", 0))
    if "jpeg" in encoding:
        try:
            import cv2  # type: ignore
            arr = np.frombuffer(data, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            return None
    if h == 0 or w == 0:
        return None
    if encoding in ("rgb8", "bgr8"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        if encoding == "rgb8":
            arr = arr[:, :, ::-1]
        return arr.copy()
    if encoding in ("rgba8", "bgra8"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 4)
        if encoding == "rgba8":
            arr = arr[:, :, [2, 1, 0]]
        else:  # bgra8
            arr = arr[:, :, :3]
        return arr.copy()
    if encoding in ("mono8", "8uc1"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
        return np.stack([arr, arr, arr], axis=2)
    return None


def _depth_msg_to_metres(msg: Any) -> Optional[Any]:
    """sensor_msgs/Image (16UC1 mm or 32FC1 m) → numpy float32 metres."""
    try:
        import numpy as np
    except Exception:
        return None
    encoding = (getattr(msg, "encoding", "") or "").lower()
    h = int(getattr(msg, "height", 0))
    w = int(getattr(msg, "width", 0))
    if h == 0 or w == 0:
        return None
    data = bytes(msg.data) if not isinstance(msg.data, bytes) else msg.data
    if encoding in ("32fc1", "32fc"):
        return np.frombuffer(data, dtype=np.float32).reshape(h, w).copy()
    if encoding in ("16uc1", "mono16", "16uc"):
        mm = np.frombuffer(data, dtype=np.uint16).reshape(h, w).astype(np.float32)
        return mm / 1000.0
    return None
