#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Verify that Nav2 sees high Webots obstacles without polluting SLAM."""

from __future__ import annotations

import argparse
import base64
import json
import math
import zlib
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

from scene_quality_ground_truth import load_semantic_inventory


def _grid_values(
    points_xy: np.ndarray,
    *,
    data: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> np.ndarray:
    columns = np.floor((points_xy[:, 0] - origin_x) / resolution).astype(int)
    rows = np.floor((points_xy[:, 1] - origin_y) / resolution).astype(int)
    valid = (
        (columns >= 0)
        & (rows >= 0)
        & (columns < data.shape[1])
        & (rows < data.shape[0])
    )
    result = np.full(len(points_xy), -1, dtype=int)
    result[valid] = data[rows[valid], columns[valid]]
    return result


def _sample_obb(
    center: tuple[float, float],
    size: tuple[float, float],
    yaw: float,
    spacing: float,
) -> np.ndarray:
    xs = np.arange(-size[0] * 0.5, size[0] * 0.5 + spacing * 0.5, spacing)
    ys = np.arange(-size[1] * 0.5, size[1] * 0.5 + spacing * 0.5, spacing)
    local_x, local_y = np.meshgrid(xs, ys)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.column_stack(
        (
            center[0] + cosine * local_x.ravel() - sine * local_y.ravel(),
            center[1] + sine * local_x.ravel() + cosine * local_y.ravel(),
        )
    )


def _point_to_obb_distance(
    points_xy: np.ndarray,
    *,
    center: tuple[float, float],
    size: tuple[float, float],
    yaw: float,
) -> np.ndarray:
    dx = points_xy[:, 0] - center[0]
    dy = points_xy[:, 1] - center[1]
    cosine = math.cos(-yaw)
    sine = math.sin(-yaw)
    local_x = cosine * dx - sine * dy
    local_y = sine * dx + cosine * dy
    outside_x = np.maximum(np.abs(local_x) - size[0] * 0.5, 0.0)
    outside_y = np.maximum(np.abs(local_y) - size[1] * 0.5, 0.0)
    return np.hypot(outside_x, outside_y)


def _load_saved_map(image_path: Path, yaml_path: Path) -> tuple[np.ndarray, dict]:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image = np.asarray(Image.open(image_path).convert("L"))
    # Convert image coordinates (top-left origin) into grid coordinates
    # (bottom-left origin) while retaining ROS occupancy-like values.
    return np.flipud(image), metadata


def _load_costmap(path: Path) -> tuple[np.ndarray, dict]:
    message = yaml.safe_load(path.read_text(encoding="utf-8").split("---", 1)[0])
    info = message["info"]
    data = np.asarray(message["data"], dtype=int).reshape(
        int(info["height"]),
        int(info["width"]),
    )
    return data, info


def _load_costmap_snapshot(
    path: Path,
    layer_name: str,
) -> tuple[np.ndarray, dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "robonix.webots.costmaps.v1":
        raise ValueError(f"{path} has unsupported schema {payload.get('schema')!r}")
    layer = (payload.get("layers") or {}).get(layer_name)
    if not isinstance(layer, dict):
        raise ValueError(f"{path} has no costmap layer {layer_name!r}")
    if layer.get("encoding") != "ros-occupancy-int8-zlib-base64":
        raise ValueError(f"{path} has unsupported encoding {layer.get('encoding')!r}")
    width = int(layer["width"])
    height = int(layer["height"])
    raw = zlib.decompress(base64.b64decode(layer["data"]))
    if len(raw) != width * height:
        raise ValueError(
            f"{path} layer {layer_name!r} has {len(raw)} cells, "
            f"expected {width * height}"
        )
    data = np.frombuffer(raw, dtype=np.uint8).astype(int)
    data[data == 255] = -1
    return data.reshape(height, width), {
        "width": width,
        "height": height,
        "resolution": float(layer["resolution"]),
        "origin": {
            "position": {
                "x": float(layer["origin_x"]),
                "y": float(layer["origin_y"]),
            }
        },
    }


def _load_path(path: Path) -> np.ndarray:
    message = yaml.safe_load(path.read_text(encoding="utf-8").split("---", 1)[0])
    return np.asarray(
        [
            [
                pose["pose"]["position"]["x"],
                pose["pose"]["position"]["y"],
            ]
            for pose in message["poses"]
        ],
        dtype=float,
    )


def _load_probe_trajectory(paths: list[Path]) -> np.ndarray:
    points: list[list[float]] = []
    for path in paths:
        payload = json.loads(path.read_text(encoding="utf-8"))
        samples = payload.get("ground_truth_samples") or ()
        points.extend(
            [float(sample["pose"][0]), float(sample["pose"][1])]
            for sample in samples
        )
    if not points:
        raise ValueError("trajectory probe JSON contains no ground-truth samples")
    return np.asarray(points, dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--benchmark", type=Path, required=True)
    parser.add_argument("--world-id", required=True)
    parser.add_argument("--map-image", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    costmap = parser.add_mutually_exclusive_group(required=True)
    costmap.add_argument("--costmap-yaml", type=Path)
    costmap.add_argument("--costmap-json", type=Path)
    parser.add_argument("--costmap-layer", default="global")
    trajectory = parser.add_mutually_exclusive_group(required=True)
    trajectory.add_argument("--path-yaml", type=Path)
    trajectory.add_argument(
        "--trajectory-json",
        action="append",
        type=Path,
        help="odom/ground-truth probe JSON; repeat to combine consecutive probes",
    )
    parser.add_argument("--robot-inscribed-radius-m", type=float, required=True)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--overlay-out", type=Path)
    args = parser.parse_args()

    truths, _ = load_semantic_inventory(
        args.benchmark,
        world_id=args.world_id,
        repository_root=args.repository_root,
    )
    tables = [truth for truth in truths if truth.label == "table"]
    saved_map, map_info = _load_saved_map(args.map_image, args.map_yaml)
    if args.costmap_json is not None:
        costmap, costmap_info = _load_costmap_snapshot(
            args.costmap_json,
            args.costmap_layer,
        )
    else:
        costmap, costmap_info = _load_costmap(args.costmap_yaml)
    if args.trajectory_json:
        trajectory = _load_probe_trajectory(args.trajectory_json)
    else:
        trajectory = _load_path(args.path_yaml)
    map_resolution = float(map_info["resolution"])
    map_origin_x, map_origin_y, _ = (float(v) for v in map_info["origin"])
    cost_origin = costmap_info["origin"]["position"]
    cost_resolution = float(costmap_info["resolution"])

    metrics = []
    for truth in tables:
        points = _sample_obb(
            (truth.center_m[0], truth.center_m[1]),
            (truth.size_m[0], truth.size_m[1]),
            truth.yaw_rad,
            spacing=min(map_resolution, cost_resolution),
        )
        map_values = _grid_values(
            points,
            data=saved_map,
            resolution=map_resolution,
            origin_x=map_origin_x,
            origin_y=map_origin_y,
        )
        cost_values = _grid_values(
            points,
            data=costmap,
            resolution=cost_resolution,
            origin_x=float(cost_origin["x"]),
            origin_y=float(cost_origin["y"]),
        )
        path_distance = _point_to_obb_distance(
            trajectory,
            center=(truth.center_m[0], truth.center_m[1]),
            size=(truth.size_m[0], truth.size_m[1]),
            yaw=truth.yaw_rad,
        )
        known_cost = cost_values >= 0
        known_map = map_values >= 0
        metrics.append(
            {
                "identity": truth.identity,
                "center_m": list(truth.center_m[:2]),
                "size_m": list(truth.size_m[:2]),
                "slam_occupied_fraction": float(
                    np.mean(map_values[known_map] < 65)
                )
                if np.any(known_map)
                else None,
                "nav_lethal_fraction": float(
                    np.mean(cost_values[known_cost] >= 99)
                )
                if np.any(known_cost)
                else None,
                "nav_nonfree_fraction": float(
                    np.mean(cost_values[known_cost] > 0)
                )
                if np.any(known_cost)
                else None,
                "nav_max_cost": int(np.max(cost_values[known_cost]))
                if np.any(known_cost)
                else None,
                "path_min_object_clearance_m": float(np.min(path_distance)),
                "path_min_body_clearance_m": float(
                    np.min(path_distance) - args.robot_inscribed_radius_m
                ),
            }
        )

    visible = [
        metric
        for metric in metrics
        if metric["nav_lethal_fraction"] is not None
        and metric["nav_lethal_fraction"] > 0.0
    ]
    payload = {
        "table_count": len(metrics),
        "tables_marked_lethal": len(visible),
        "trajectory_points": int(len(trajectory)),
        "robot_inscribed_radius_m": args.robot_inscribed_radius_m,
        "minimum_body_clearance_m": min(
            metric["path_min_body_clearance_m"] for metric in metrics
        ),
        "tables": metrics,
    }
    if args.overlay_out:
        background = Image.fromarray(np.flipud(saved_map)).convert("RGB").resize(
            (saved_map.shape[1] * 3, saved_map.shape[0] * 3),
            Image.Resampling.NEAREST,
        )
        draw = ImageDraw.Draw(background)

        def pixel(point: tuple[float, float]) -> tuple[float, float]:
            return (
                (point[0] - map_origin_x) / map_resolution * 3.0,
                (
                    saved_map.shape[0]
                    - (point[1] - map_origin_y) / map_resolution
                )
                * 3.0,
            )

        draw.line(
            [pixel((float(point[0]), float(point[1]))) for point in trajectory],
            fill=(20, 110, 255),
            width=2,
        )
        for truth in tables:
            half_x = truth.size_m[0] * 0.5
            half_y = truth.size_m[1] * 0.5
            cosine = math.cos(truth.yaw_rad)
            sine = math.sin(truth.yaw_rad)
            corners = []
            for local_x, local_y in (
                (-half_x, -half_y),
                (half_x, -half_y),
                (half_x, half_y),
                (-half_x, half_y),
            ):
                corners.append(
                    pixel(
                        (
                            truth.center_m[0]
                            + cosine * local_x
                            - sine * local_y,
                            truth.center_m[1]
                            + sine * local_x
                            + cosine * local_y,
                        )
                    )
                )
            corners.append(corners[0])
            draw.line(corners, fill=(240, 40, 40), width=3)
        args.overlay_out.parent.mkdir(parents=True, exist_ok=True)
        background.save(args.overlay_out)
    output = json.dumps(payload, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
