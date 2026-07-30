# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for Scene RGB-D geometry admission and robust boxes."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _occupancy_contains_points,
    _planar_surface_snap_translation,
    _refine_masks_with_depth,
    _robust_yaw_bbox,
    _run_frame_pointcloud_filter,
)


def test_depth_refinement_removes_mask_edge_background() -> None:
    masks = np.ones((1, 7, 7), dtype=bool)
    depth = np.full((7, 7), 5.0, dtype=np.float32)
    depth[1:6, 1:6] = 2.0
    refined = _refine_masks_with_depth(
        masks,
        depth,
        erosion_px=1,
        min_depth_m=0.15,
        max_depth_m=6.0,
        mad_scale=3.5,
        min_band_m=0.12,
        min_points=4,
    )
    assert refined.shape == masks.shape
    assert int(refined.sum()) == 25
    assert np.all(refined[0, 1:6, 1:6])
    assert not np.any(refined[0, 0])
    assert not np.any(refined[0, -1])


def test_depth_refinement_preserves_small_valid_mask_when_erosion_erases_it() -> None:
    masks = np.zeros((1, 6, 6), dtype=bool)
    masks[0, 2:4, 2:4] = True
    depth = np.full((6, 6), 2.0, dtype=np.float32)
    refined = _refine_masks_with_depth(
        masks,
        depth,
        erosion_px=1,
        min_depth_m=0.15,
        max_depth_m=6.0,
        mad_scale=3.5,
        min_band_m=0.12,
        min_points=4,
    )
    assert np.array_equal(refined, masks)


def test_frame_pointcloud_filter_invokes_canonical_conceptgraphs_processing() -> None:
    raw = SimpleNamespace(points=np.zeros((20, 3), dtype=np.float64))
    filtered = SimpleNamespace(points=np.zeros((12, 3), dtype=np.float64))
    calls = {}

    def process(point_cloud, **kwargs):
        calls["point_cloud"] = point_cloud
        calls["kwargs"] = kwargs
        return filtered

    def bbox(spatial_sim_type, point_cloud):
        calls["bbox"] = (spatial_sim_type, point_cloud)
        return "filtered-bbox"

    result, diagnostic = _run_frame_pointcloud_filter(
        {"pcd": raw, "bbox": "raw-bbox"},
        process_pcd=process,
        get_bounding_box=bbox,
        downsample_voxel_size=0.025,
        dbscan_remove_noise=True,
        dbscan_eps=0.10,
        dbscan_min_points=10,
        spatial_sim_type="overlap",
        min_points_threshold=8,
        run_dbscan=True,
    )

    assert calls["point_cloud"] is raw
    assert calls["kwargs"] == {
        "downsample_voxel_size": 0.025,
        "dbscan_remove_noise": True,
        "dbscan_eps": 0.10,
        "dbscan_min_points": 10,
        "run_dbscan": True,
    }
    assert calls["bbox"] == ("overlap", filtered)
    assert result == {"pcd": filtered, "bbox": "filtered-bbox"}
    assert diagnostic == {
        "attempted": True,
        "status": "filtered",
        "input_points": 20,
        "output_points": 12,
    }


def test_frame_pointcloud_filter_rejects_too_small_filtered_cloud() -> None:
    raw = SimpleNamespace(points=np.zeros((20, 3), dtype=np.float64))
    filtered = SimpleNamespace(points=np.zeros((4, 3), dtype=np.float64))

    result, diagnostic = _run_frame_pointcloud_filter(
        {"pcd": raw, "bbox": "raw-bbox"},
        process_pcd=lambda *_args, **_kwargs: filtered,
        get_bounding_box=lambda *_args: pytest.fail(
            "bbox must not run for an undersized cloud"
        ),
        downsample_voxel_size=0.025,
        dbscan_remove_noise=True,
        dbscan_eps=0.10,
        dbscan_min_points=10,
        spatial_sim_type="overlap",
        min_points_threshold=8,
        run_dbscan=True,
    )

    assert result is None
    assert diagnostic["status"] == "insufficient_points"
    assert diagnostic["input_points"] == 20
    assert diagnostic["output_points"] == 4


def _grid(
    *,
    yaw: float = 0.0,
    data=None,
    resolution: float = 0.5,
    width: int = 20,
    height: int = 10,
    origin_x: float = -2.0,
    origin_y: float = -1.0,
):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            resolution=resolution,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            ),
        ),
        data=list(data or ()),
    )


def test_occupancy_bounds_reject_far_ghost_and_frame_mismatch() -> None:
    inside = np.asarray([[0.0, 0.0, 0.5], [0.1, -0.1, 0.6]])
    far = np.asarray([[100.0, 100.0, 0.5], [101.0, 100.0, 0.6]])
    assert _occupancy_contains_points(inside, _grid(), expected_frame="map")
    assert not _occupancy_contains_points(far, _grid(), expected_frame="map")
    assert not _occupancy_contains_points(inside, _grid(), expected_frame="odom")


def test_occupancy_bounds_support_rotated_map_origin() -> None:
    # Local grid coordinate (2, 1) rotated +90 degrees and translated by
    # origin (-2, -1) becomes world (-3, 1).
    inside_rotated = np.asarray([[-3.0, 1.0, 0.5], [-3.1, 1.1, 0.6]])
    assert _occupancy_contains_points(
        inside_rotated,
        _grid(yaw=math.pi / 2.0),
        expected_frame="map",
    )


def test_planar_surface_snap_uses_dominant_parallel_occupancy_support() -> None:
    resolution = 0.05
    width = height = 80
    values = [0] * (width * height)
    # With origin (-2, -1), row 20 is centred at y=0.025. The point cloud is
    # a parallel vertical plane at y=0.30, so the expected correction is about
    # -0.275 m along the surface normal.
    for column in range(18, 62):
        values[20 * width + column] = 100
    local_x, local_z = np.meshgrid(
        np.linspace(-0.9, 0.9, 25),
        np.linspace(0.5, 1.7, 9),
    )
    points = np.column_stack(
        (
            local_x.ravel(),
            np.full(local_x.size, 0.30),
            local_z.ravel(),
        )
    )
    translation, diagnostics = _planar_surface_snap_translation(
        points,
        _grid(
            data=values,
            resolution=resolution,
            width=width,
            height=height,
        ),
        expected_frame="map",
        min_support_cells=20,
    )
    assert diagnostics["status"] == "applied"
    assert diagnostics["support_cells"] >= 20
    assert diagnostics["dominant_share"] > 0.9
    assert translation == pytest.approx([0.0, -0.275, 0.0], abs=0.03)


def test_planar_surface_snap_rejects_sparse_map_support() -> None:
    width = 20
    values = [0] * (width * 10)
    for column in range(5):
        values[4 * width + column] = 100
    points = np.column_stack(
        (
            np.linspace(-0.5, 0.5, 20),
            np.full(20, 0.4),
            np.full(20, 1.0),
        )
    )
    translation, diagnostics = _planar_surface_snap_translation(
        points,
        _grid(data=values),
        expected_frame="map",
        min_support_cells=10,
    )
    assert diagnostics["status"] == "insufficient_support"
    assert translation == pytest.approx([0.0, 0.0, 0.0])


def test_planar_surface_snap_respects_rotated_occupancy_origin() -> None:
    resolution = 0.05
    width = height = 80
    values = [0] * (width * height)
    for column in range(18, 62):
        values[20 * width + column] = 100
    local_y, local_z = np.meshgrid(
        np.linspace(0.9, 2.7, 25),
        np.linspace(0.5, 1.7, 9),
    )
    points = np.column_stack(
        (
            np.full(local_y.size, -0.75),
            local_y.ravel(),
            local_z.ravel(),
        )
    )
    translation, diagnostics = _planar_surface_snap_translation(
        points,
        _grid(
            yaw=math.pi / 2.0,
            data=values,
            resolution=resolution,
            width=width,
            height=height,
            origin_x=0.0,
            origin_y=0.0,
        ),
        expected_frame="map",
        min_support_cells=20,
    )
    assert diagnostics["status"] == "applied"
    assert translation == pytest.approx([-0.275, 0.0, 0.0], abs=0.03)


def test_robust_bbox_resists_single_depth_spike() -> None:
    rng = np.random.default_rng(7)
    points = np.column_stack(
        (
            rng.uniform(0.5, 1.5, 500),
            rng.uniform(1.75, 2.25, 500),
            rng.uniform(0.2, 0.8, 500),
        )
    )
    points = np.vstack((points, np.asarray([[30.0, -40.0, 20.0]])))
    result = _robust_yaw_bbox(points)
    assert result is not None
    center, extent, yaw = result
    assert np.linalg.norm(center - np.asarray([1.0, 2.0, 0.5])) < 0.15
    assert sorted(extent[:2]) == pytest.approx([0.45, 0.90], abs=0.12)
    assert extent[2] == pytest.approx(0.54, abs=0.08)
    assert math.isfinite(yaw)


def _rotated_box_points(
    *,
    yaw: float,
    extent_x: float,
    extent_y: float,
) -> np.ndarray:
    local_x, local_y = np.meshgrid(
        np.linspace(-extent_x / 2.0, extent_x / 2.0, 25),
        np.linspace(-extent_y / 2.0, extent_y / 2.0, 13),
    )
    local = np.column_stack(
        (
            local_x.ravel(),
            local_y.ravel(),
            np.linspace(0.2, 0.8, local_x.size),
        )
    )
    rotation = np.asarray(
        [
            [math.cos(yaw), math.sin(yaw)],
            [-math.sin(yaw), math.cos(yaw)],
        ]
    )
    local[:, :2] = local[:, :2] @ rotation + np.asarray([2.0, -1.0])
    return local


def test_robust_bbox_uses_canonical_minimum_area_rectangle() -> None:
    expected_yaw = 0.63
    points = _rotated_box_points(
        yaw=expected_yaw,
        extent_x=4.0,
        extent_y=1.0,
    )
    result = _robust_yaw_bbox(
        points,
        low_percentile=0.0,
        high_percentile=100.0,
    )
    assert result is not None
    center, extent, yaw = result
    assert center == pytest.approx([2.0, -1.0, 0.5], abs=1e-10)
    assert extent == pytest.approx([4.0, 1.0, 0.6], abs=1e-10)
    assert yaw == pytest.approx(expected_yaw, abs=1e-10)


def test_robust_bbox_is_permutation_invariant_and_ignores_nonfinite_rows() -> None:
    points = _rotated_box_points(yaw=-0.41, extent_x=2.0, extent_y=0.7)
    contaminated = np.vstack(
        (
            points,
            np.asarray(
                [
                    [np.nan, 0.0, 0.0],
                    [0.0, np.inf, 0.0],
                    [0.0, 0.0, -np.inf],
                ]
            ),
        )
    )
    expected = _robust_yaw_bbox(points)
    actual = _robust_yaw_bbox(
        contaminated[np.random.default_rng(11).permutation(len(contaminated))]
    )
    assert expected is not None
    assert actual is not None
    assert actual[0] == pytest.approx(expected[0], abs=1e-12)
    assert actual[1] == pytest.approx(expected[1], abs=1e-12)
    assert actual[2] == pytest.approx(expected[2], abs=1e-12)


def test_robust_bbox_world_aligns_near_isotropic_footprint() -> None:
    points = _rotated_box_points(
        yaw=0.73,
        extent_x=1.0,
        extent_y=0.98,
    )
    result = _robust_yaw_bbox(
        points,
        low_percentile=0.0,
        high_percentile=100.0,
    )
    assert result is not None
    assert result[2] == 0.0


def test_export_snapshot_boxes_full_cloud_before_display_sampling(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    full_cloud = _rotated_box_points(yaw=0.37, extent_x=4.0, extent_y=1.0)
    tiny_sample = np.asarray(
        [
            [1.99, -1.01, 0.49],
            [2.01, -1.01, 0.49],
            [1.99, -0.99, 0.51],
            [2.01, -0.99, 0.51],
        ]
    )
    points = np.vstack((tiny_sample, full_cloud))
    detector = ConceptGraphsDetector.__new__(ConceptGraphsDetector)
    detector._map_objects = [
        {
            "id": "stable-object",
            "pcd": SimpleNamespace(
                points=points,
                colors=np.zeros_like(points),
            ),
            "bbox": object(),
            "n_points": len(points),
        }
    ]
    detector._uuid_to_oid = None
    detector._missing_uuids = set()
    detector._operator_geometry_oids = set()
    detector._bbox_low_percentile = 0.0
    detector._bbox_high_percentile = 100.0
    monkeypatch.setattr(
        np.random,
        "choice",
        lambda count, *, size, replace: np.arange(size),
    )

    snapshot = detector.export_3d_snapshot(max_points_per_obj=4)

    assert len(snapshot["objects"]) == 1
    exported = snapshot["objects"][0]
    assert len(exported["points"]) == 4
    corners = np.asarray(exported["bbox_corners"])
    assert np.linalg.norm(corners[1] - corners[0]) == pytest.approx(4.0)
    assert np.linalg.norm(corners[2] - corners[0]) == pytest.approx(1.0)
