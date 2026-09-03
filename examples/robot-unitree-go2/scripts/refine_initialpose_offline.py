#!/usr/bin/env python3
"""Refine one manually selected map pose from saved map and LaserScan snapshots.

This is deliberately an offline tool.  It imports no ROS client library,
creates no node, and cannot publish ``/initialpose`` or any motion command.
The input is a JSON snapshot captured separately while the robot is stationary:

.. code-block:: json

  {
    "map_yaml": "rbnx-build/data/maps/example/occupancy.yaml",
    "initial_pose": {"x": -0.60, "y": 1.75, "yaw": 1.57},
    "laser_in_base": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "scans": [{
      "angle_min": -3.14159,
      "angle_increment": 0.01745,
      "range_min": 0.10,
      "range_max": 6.0,
      "ranges": [1.0, null, 1.2]
    }]
  }

Every finite return selected before the search remains in the denominator for
every candidate.  Endpoints in unknown cells or outside the map receive fixed
penalties instead of disappearing from the score.  A correction is recommended
only when its local peak is interior, materially better than the manual pose,
and separated from a geometrically distinct alternative.
"""

from __future__ import annotations

import argparse
from array import array
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "robonix-go2-initialpose-refinement-offline-v1"
ALGORITHM = "fixed-beam-endpoint-distance-local-search-v1"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin: Pose2D
    occupancy: tuple[int, ...]
    occupied_threshold: int = 65


@dataclass(frozen=True)
class BeamSet:
    """Laser endpoints expressed in the robot base frame."""

    endpoint_x: tuple[float, ...]
    endpoint_y: tuple[float, ...]
    source_scan_count: int

    @property
    def count(self) -> int:
        return len(self.endpoint_x)


@dataclass(frozen=True)
class SearchConfig:
    search_xy_m: float = 0.40
    search_yaw_rad: float = math.radians(12.0)
    xy_step_m: float = 0.05
    yaw_step_rad: float = math.radians(2.0)
    refinement_levels: int = 2
    hit_sigma_m: float = 0.10
    hit_distance_m: float = 0.12
    unknown_penalty: float = -0.75
    out_of_bounds_penalty: float = -1.00
    footprint_radius_m: float = 0.20
    min_beams: int = 30
    min_score: float = 0.40
    min_known_fraction: float = 0.65
    min_hit_fraction: float = 0.50
    min_improvement: float = 0.02
    unique_translation_m: float = 0.10
    unique_yaw_rad: float = math.radians(4.0)
    min_unique_margin: float = 0.02


@dataclass(frozen=True)
class PoseScore:
    pose: Pose2D
    score: float
    beam_count: int
    known_count: int
    unknown_count: int
    out_of_bounds_count: int
    hit_count: int
    occupied_endpoint_count: int
    footprint_clear: bool

    @property
    def known_fraction(self) -> float:
        return self.known_count / self.beam_count

    @property
    def unknown_fraction(self) -> float:
        return self.unknown_count / self.beam_count

    @property
    def out_of_bounds_fraction(self) -> float:
        return self.out_of_bounds_count / self.beam_count

    @property
    def hit_fraction(self) -> float:
        return self.hit_count / self.beam_count


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be a finite number") from error
    if not math.isfinite(parsed):
        raise ValueError(f"{label} must be a finite number")
    return parsed


def _positive(value: str) -> float:
    parsed = _finite(value, "value")
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def _nonnegative_integer(value: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be an integer") from error
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be nonnegative")
    return parsed


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_distance(first: float, second: float) -> float:
    return abs(normalize_angle(first - second))


def _pose(raw: object, label: str) -> Pose2D:
    if not isinstance(raw, dict):
        raise ValueError(f"{label} must be an object")
    return Pose2D(
        _finite(raw.get("x"), f"{label}.x"),
        _finite(raw.get("y"), f"{label}.y"),
        normalize_angle(_finite(raw.get("yaw"), f"{label}.yaw")),
    )


def _safe_input(raw: str | Path, *, relative_to: Path | None = None) -> Path:
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        local = (relative_to / candidate) if relative_to is not None else None
        candidate = local if local is not None and local.exists() else ROOT / candidate
    if candidate.is_symlink():
        raise ValueError(f"input must not be a symlink: {candidate}")
    resolved = candidate.resolve(strict=True)
    if not resolved.is_relative_to(ROOT):
        raise ValueError(f"input must remain below {ROOT}: {resolved}")
    if not resolved.is_file():
        raise ValueError(f"input is not a regular file: {resolved}")
    return resolved


def resolve_output(raw: str | Path) -> Path:
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = ROOT / candidate
    if candidate.exists() and candidate.is_symlink():
        raise ValueError("output must not be a symlink")
    output = candidate.resolve()
    if not output.is_relative_to(ROOT):
        raise ValueError(f"output must remain below {ROOT}")
    return output


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _pgm_token(content: bytes, start: int) -> tuple[bytes, int]:
    index = start
    while index < len(content):
        if content[index] == ord("#"):
            newline = content.find(b"\n", index)
            if newline < 0:
                raise ValueError("unterminated PGM comment")
            index = newline + 1
        elif chr(content[index]).isspace():
            index += 1
        else:
            break
    end = index
    while end < len(content) and not chr(content[end]).isspace() and content[end] != ord("#"):
        end += 1
    if end == index:
        raise ValueError("missing PGM header token")
    return content[index:end], end


def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    content = path.read_bytes()
    magic, index = _pgm_token(content, 0)
    if magic != b"P5":
        raise ValueError("only raw P5 occupancy PGM files are supported")
    width_raw, index = _pgm_token(content, index)
    height_raw, index = _pgm_token(content, index)
    maximum_raw, index = _pgm_token(content, index)
    try:
        width = int(width_raw)
        height = int(height_raw)
        maximum = int(maximum_raw)
    except ValueError as error:
        raise ValueError("invalid PGM dimensions or maximum") from error
    if width <= 0 or height <= 0 or not 1 <= maximum <= 255:
        raise ValueError("unsupported PGM dimensions or maximum")
    if index >= len(content) or not chr(content[index]).isspace():
        raise ValueError("PGM header is not separated from image data")
    if content[index:index + 2] == b"\r\n":
        index += 2
    else:
        index += 1
    pixels = content[index:]
    if len(pixels) != width * height:
        raise ValueError(
            f"PGM payload has {len(pixels)} bytes, expected {width * height}"
        )
    return width, height, maximum, pixels


def load_map_yaml(path: Path) -> tuple[GridMap, Path]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot parse map YAML: {error}") from error
    if not isinstance(document, dict):
        raise ValueError("map YAML must contain an object")
    image_name = document.get("image")
    if not isinstance(image_name, str) or not image_name.strip():
        raise ValueError("map YAML image is missing")
    image_path = _safe_input(image_name, relative_to=path.parent)
    width, height, maximum, pixels = read_pgm(image_path)
    resolution = _finite(document.get("resolution"), "map resolution")
    if resolution <= 0.0:
        raise ValueError("map resolution must be positive")
    origin_raw = document.get("origin")
    if not isinstance(origin_raw, list) or len(origin_raw) != 3:
        raise ValueError("map origin must be [x, y, yaw]")
    origin = Pose2D(
        _finite(origin_raw[0], "map origin x"),
        _finite(origin_raw[1], "map origin y"),
        normalize_angle(_finite(origin_raw[2], "map origin yaw")),
    )
    negate = int(document.get("negate", 0))
    if negate not in (0, 1):
        raise ValueError("map negate must be 0 or 1")
    occupied_threshold = _finite(
        document.get("occupied_thresh", 0.65), "occupied threshold"
    )
    free_threshold = _finite(document.get("free_thresh", 0.25), "free threshold")
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise ValueError("map occupancy thresholds are invalid")
    # ROS map_saver writes unknown trinary cells as the canonical gray value
    # 205.  Some RTabMap exports retain a free_thresh above 50/255, which would
    # otherwise misclassify every canonical unknown pixel as free on reload.
    # Preserve the serialized trinary meaning before applying probabilities.
    unknown_gray = round(205 * maximum / 255)

    occupancy: list[int] = [0] * (width * height)
    for map_y in range(height):
        image_y = height - 1 - map_y
        image_offset = image_y * width
        map_offset = map_y * width
        for cell_x in range(width):
            pixel = pixels[image_offset + cell_x]
            if pixel == unknown_gray:
                occupancy[map_offset + cell_x] = -1
                continue
            value = pixel / maximum
            probability = value if negate else 1.0 - value
            if probability > occupied_threshold:
                occupancy[map_offset + cell_x] = 100
            elif probability < free_threshold:
                occupancy[map_offset + cell_x] = 0
            else:
                occupancy[map_offset + cell_x] = -1
    return (
        GridMap(width, height, resolution, origin, tuple(occupancy)),
        image_path,
    )


def load_inline_map(raw: object) -> GridMap:
    if not isinstance(raw, dict):
        raise ValueError("inline map must be an object")
    width = int(raw.get("width", 0))
    height = int(raw.get("height", 0))
    resolution = _finite(raw.get("resolution"), "map resolution")
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("inline map dimensions are invalid")
    origin_raw = raw.get("origin")
    if isinstance(origin_raw, list) and len(origin_raw) == 3:
        origin = Pose2D(
            _finite(origin_raw[0], "map origin x"),
            _finite(origin_raw[1], "map origin y"),
            normalize_angle(_finite(origin_raw[2], "map origin yaw")),
        )
    else:
        origin = _pose(origin_raw, "map.origin")
    data = raw.get("data")
    if not isinstance(data, list) or len(data) != width * height:
        raise ValueError("inline map data length does not match width * height")
    occupancy: list[int] = []
    for index, value in enumerate(data):
        if isinstance(value, bool):
            raise ValueError(f"inline map data[{index}] is invalid")
        parsed = int(value)
        if parsed < -1 or parsed > 100:
            raise ValueError(f"inline map data[{index}] is outside [-1, 100]")
        occupancy.append(parsed)
    threshold = int(raw.get("occupied_threshold", 65))
    if not 1 <= threshold <= 100:
        raise ValueError("inline map occupied_threshold is invalid")
    return GridMap(width, height, resolution, origin, tuple(occupancy), threshold)


def beams_from_snapshot(document: dict[str, object]) -> BeamSet:
    scans = document.get("scans")
    if not isinstance(scans, list) or not scans:
        raise ValueError("snapshot scans must be a nonempty list")
    laser_raw = document.get("laser_in_base", {"x": 0.0, "y": 0.0, "yaw": 0.0})
    laser = _pose(laser_raw, "laser_in_base")
    laser_cosine = math.cos(laser.yaw)
    laser_sine = math.sin(laser.yaw)
    endpoint_x: list[float] = []
    endpoint_y: list[float] = []
    for scan_index, scan_raw in enumerate(scans):
        if not isinstance(scan_raw, dict):
            raise ValueError(f"scans[{scan_index}] must be an object")
        angle_min = _finite(scan_raw.get("angle_min"), f"scans[{scan_index}].angle_min")
        increment = _finite(
            scan_raw.get("angle_increment"),
            f"scans[{scan_index}].angle_increment",
        )
        if abs(increment) < 1e-12:
            raise ValueError(f"scans[{scan_index}].angle_increment must be nonzero")
        range_min = _finite(scan_raw.get("range_min"), f"scans[{scan_index}].range_min")
        range_max = _finite(scan_raw.get("range_max"), f"scans[{scan_index}].range_max")
        if range_min < 0.0 or range_max <= range_min:
            raise ValueError(f"scans[{scan_index}] range limits are invalid")
        ranges = scan_raw.get("ranges")
        if not isinstance(ranges, list):
            raise ValueError(f"scans[{scan_index}].ranges must be a list")
        for beam_index, raw_range in enumerate(ranges):
            if raw_range is None or isinstance(raw_range, bool):
                continue
            try:
                distance = float(raw_range)
            except (TypeError, ValueError):
                continue
            if (
                not math.isfinite(distance)
                or distance < range_min
                or distance >= range_max - 1e-6
            ):
                continue
            angle = angle_min + beam_index * increment
            laser_x = distance * math.cos(angle)
            laser_y = distance * math.sin(angle)
            endpoint_x.append(
                laser.x + laser_cosine * laser_x - laser_sine * laser_y
            )
            endpoint_y.append(
                laser.y + laser_sine * laser_x + laser_cosine * laser_y
            )
    if not endpoint_x:
        raise ValueError("snapshot contains no finite obstacle returns")
    return BeamSet(tuple(endpoint_x), tuple(endpoint_y), len(scans))


def _edt_1d(values: list[float]) -> list[float]:
    """Exact squared Euclidean distance transform for one finite-valued line."""

    count = len(values)
    locations = [0] * count
    boundaries = [0.0] * (count + 1)
    output = [0.0] * count
    active = 0
    locations[0] = 0
    boundaries[0] = -math.inf
    boundaries[1] = math.inf
    for coordinate in range(1, count):
        while True:
            previous = locations[active]
            intersection = (
                (values[coordinate] + coordinate * coordinate)
                - (values[previous] + previous * previous)
            ) / (2.0 * (coordinate - previous))
            if intersection > boundaries[active]:
                break
            active -= 1
        active += 1
        locations[active] = coordinate
        boundaries[active] = intersection
        boundaries[active + 1] = math.inf
    active = 0
    for coordinate in range(count):
        while boundaries[active + 1] < coordinate:
            active += 1
        delta = coordinate - locations[active]
        output[coordinate] = delta * delta + values[locations[active]]
    return output


def occupied_distance_field(grid: GridMap) -> array:
    if not any(value >= grid.occupied_threshold for value in grid.occupancy):
        raise ValueError("map contains no occupied cells")
    far = float((grid.width + grid.height + 1) ** 2 * 4)
    first_pass = array("d", [0.0]) * (grid.width * grid.height)
    for cell_y in range(grid.height):
        offset = cell_y * grid.width
        values = [
            0.0
            if grid.occupancy[offset + cell_x] >= grid.occupied_threshold
            else far
            for cell_x in range(grid.width)
        ]
        transformed = _edt_1d(values)
        first_pass[offset:offset + grid.width] = array("d", transformed)
    result = array("d", [0.0]) * (grid.width * grid.height)
    for cell_x in range(grid.width):
        values = [
            first_pass[cell_y * grid.width + cell_x]
            for cell_y in range(grid.height)
        ]
        transformed = _edt_1d(values)
        for cell_y, squared_cells in enumerate(transformed):
            result[cell_y * grid.width + cell_x] = (
                math.sqrt(squared_cells) * grid.resolution
            )
    return result


def world_to_cell(grid: GridMap, world_x: float, world_y: float) -> tuple[int, int]:
    delta_x = world_x - grid.origin.x
    delta_y = world_y - grid.origin.y
    cosine = math.cos(grid.origin.yaw)
    sine = math.sin(grid.origin.yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    return (
        math.floor(local_x / grid.resolution),
        math.floor(local_y / grid.resolution),
    )


def _cell_value(grid: GridMap, world_x: float, world_y: float) -> int | None:
    cell_x, cell_y = world_to_cell(grid, world_x, world_y)
    if not (0 <= cell_x < grid.width and 0 <= cell_y < grid.height):
        return None
    return grid.occupancy[cell_y * grid.width + cell_x]


def footprint_is_clear(grid: GridMap, pose: Pose2D, radius: float) -> bool:
    sample_offsets = [(0.0, 0.0)]
    if radius > 0.0:
        for ring_radius in (radius * 0.5, radius):
            sample_offsets.extend(
                (
                    ring_radius * math.cos(index * math.tau / 16.0),
                    ring_radius * math.sin(index * math.tau / 16.0),
                )
                for index in range(16)
            )
    for offset_x, offset_y in sample_offsets:
        value = _cell_value(grid, pose.x + offset_x, pose.y + offset_y)
        if value is None or value < 0 or value >= grid.occupied_threshold:
            return False
    return True


def score_pose(
    grid: GridMap,
    distance_field: array,
    beams: BeamSet,
    pose: Pose2D,
    config: SearchConfig,
) -> PoseScore:
    if beams.count <= 0:
        raise ValueError("beam denominator must be positive")
    if len(beams.endpoint_y) != beams.count:
        raise ValueError("beam endpoint coordinate lengths differ")
    if len(distance_field) != grid.width * grid.height:
        raise ValueError("distance field size does not match map")
    clear = footprint_is_clear(grid, pose, config.footprint_radius_m)
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    total = 0.0
    known = 0
    unknown = 0
    out_of_bounds = 0
    hit = 0
    occupied = 0
    for local_x, local_y in zip(beams.endpoint_x, beams.endpoint_y):
        world_x = pose.x + cosine * local_x - sine * local_y
        world_y = pose.y + sine * local_x + cosine * local_y
        cell_x, cell_y = world_to_cell(grid, world_x, world_y)
        if not (0 <= cell_x < grid.width and 0 <= cell_y < grid.height):
            out_of_bounds += 1
            total += config.out_of_bounds_penalty
            continue
        index = cell_y * grid.width + cell_x
        value = grid.occupancy[index]
        if value < 0:
            unknown += 1
            total += config.unknown_penalty
            continue
        known += 1
        distance = distance_field[index]
        likelihood = math.exp(
            -0.5 * (distance / config.hit_sigma_m) ** 2
        )
        total += likelihood
        if distance <= config.hit_distance_m:
            hit += 1
        if value >= grid.occupied_threshold:
            occupied += 1
    score = total / beams.count
    if not clear:
        # Keep evidence strict JSON even when the manual pose footprint is
        # invalid; selection below separately excludes unclear footprints.
        score = min(config.unknown_penalty, config.out_of_bounds_penalty) - 1.0
    return PoseScore(
        pose,
        score,
        beams.count,
        known,
        unknown,
        out_of_bounds,
        hit,
        occupied,
        clear,
    )


def _axis_offsets(radius: float, step: float) -> list[float]:
    count = max(1, math.floor(radius / step + 1e-9))
    values = {round(index * step, 12) for index in range(-count, count + 1)}
    values.update({round(-radius, 12), 0.0, round(radius, 12)})
    return sorted(values)


def _candidate_key(pose: Pose2D) -> tuple[int, int, int]:
    return (
        round(pose.x * 1_000_000_000),
        round(pose.y * 1_000_000_000),
        round(normalize_angle(pose.yaw) * 1_000_000_000),
    )


def _rank_key(candidate: PoseScore, initial: Pose2D) -> tuple[float, float, float]:
    translation = math.hypot(
        candidate.pose.x - initial.x, candidate.pose.y - initial.y
    )
    yaw_delta = angular_distance(candidate.pose.yaw, initial.yaw)
    return (candidate.score, -translation, -yaw_delta)


def refine_pose(
    grid: GridMap,
    beams: BeamSet,
    initial: Pose2D,
    config: SearchConfig,
) -> dict[str, object]:
    if config.hit_sigma_m <= 0.0 or config.hit_distance_m <= 0.0:
        raise ValueError("hit distances must be positive")
    if config.search_xy_m <= 0.0 or config.search_yaw_rad <= 0.0:
        raise ValueError("search extents must be positive")
    if config.xy_step_m <= 0.0 or config.yaw_step_rad <= 0.0:
        raise ValueError("search steps must be positive")
    if beams.count < 1:
        raise ValueError("beam denominator must be positive")

    distance_field = occupied_distance_field(grid)
    evaluated: dict[tuple[int, int, int], PoseScore] = {}

    def evaluate(pose: Pose2D) -> PoseScore:
        normalized = Pose2D(pose.x, pose.y, normalize_angle(pose.yaw))
        key = _candidate_key(normalized)
        if key not in evaluated:
            evaluated[key] = score_pose(
                grid, distance_field, beams, normalized, config
            )
        return evaluated[key]

    current = evaluate(initial)
    x_offsets = _axis_offsets(config.search_xy_m, config.xy_step_m)
    y_offsets = _axis_offsets(config.search_xy_m, config.xy_step_m)
    yaw_offsets = _axis_offsets(config.search_yaw_rad, config.yaw_step_rad)
    for offset_x in x_offsets:
        for offset_y in y_offsets:
            for offset_yaw in yaw_offsets:
                evaluate(
                    Pose2D(
                        initial.x + offset_x,
                        initial.y + offset_y,
                        initial.yaw + offset_yaw,
                    )
                )
    valid = [candidate for candidate in evaluated.values() if candidate.footprint_clear]
    if not valid:
        raise ValueError("no search candidate has a clear known-space footprint")
    best = max(valid, key=lambda item: _rank_key(item, initial))

    xy_step = config.xy_step_m
    yaw_step = config.yaw_step_rad
    for _ in range(config.refinement_levels):
        xy_step *= 0.5
        yaw_step *= 0.5
        center = best.pose
        for offset_x in (-xy_step, 0.0, xy_step):
            for offset_y in (-xy_step, 0.0, xy_step):
                for offset_yaw in (-yaw_step, 0.0, yaw_step):
                    proposal = Pose2D(
                        center.x + offset_x,
                        center.y + offset_y,
                        center.yaw + offset_yaw,
                    )
                    if abs(proposal.x - initial.x) > config.search_xy_m + 1e-9:
                        continue
                    if abs(proposal.y - initial.y) > config.search_xy_m + 1e-9:
                        continue
                    if angular_distance(proposal.yaw, initial.yaw) > config.search_yaw_rad + 1e-9:
                        continue
                    evaluate(proposal)
        valid = [
            candidate for candidate in evaluated.values()
            if candidate.footprint_clear
        ]
        best = max(valid, key=lambda item: _rank_key(item, initial))

    alternatives = [
        candidate
        for candidate in valid
        if candidate is not best
        and (
            math.hypot(
                candidate.pose.x - best.pose.x,
                candidate.pose.y - best.pose.y,
            )
            >= config.unique_translation_m
            or angular_distance(candidate.pose.yaw, best.pose.yaw)
            >= config.unique_yaw_rad
        )
    ]
    alternative = (
        max(alternatives, key=lambda item: _rank_key(item, initial))
        if alternatives
        else None
    )
    unique_margin = (
        best.score - alternative.score if alternative is not None else -math.inf
    )
    delta_x = best.pose.x - initial.x
    delta_y = best.pose.y - initial.y
    delta_yaw = normalize_angle(best.pose.yaw - initial.yaw)
    translation = math.hypot(delta_x, delta_y)
    boundary = (
        abs(delta_x) >= config.search_xy_m - xy_step * 0.51
        or abs(delta_y) >= config.search_xy_m - xy_step * 0.51
        or abs(delta_yaw) >= config.search_yaw_rad - yaw_step * 0.51
    )
    improvement = best.score - current.score
    checks = {
        "fixed_beam_denominator": (
            current.beam_count == best.beam_count == beams.count
        ),
        "minimum_beam_count": beams.count >= config.min_beams,
        "best_footprint_clear": best.footprint_clear,
        "best_not_on_search_boundary": not boundary,
        "best_score": best.score >= config.min_score,
        "best_known_fraction": best.known_fraction >= config.min_known_fraction,
        "best_hit_fraction": best.hit_fraction >= config.min_hit_fraction,
        "material_improvement": improvement >= config.min_improvement,
        "distinct_alternative_evaluated": alternative is not None,
        "unique_peak_margin": (
            alternative is not None and unique_margin >= config.min_unique_margin
        ),
    }
    hard_quality = all(
        checks[name]
        for name in (
            "fixed_beam_denominator",
            "minimum_beam_count",
            "best_footprint_clear",
            "best_not_on_search_boundary",
            "best_score",
            "best_known_fraction",
            "best_hit_fraction",
            "distinct_alternative_evaluated",
            "unique_peak_margin",
        )
    )
    accepted = hard_quality and checks["material_improvement"]
    no_change_translation = xy_step * 1.01
    no_change_yaw = yaw_step * 1.01
    initial_quality = (
        current.footprint_clear
        and current.score >= config.min_score
        and current.known_fraction >= config.min_known_fraction
        and current.hit_fraction >= config.min_hit_fraction
    )
    if accepted:
        decision = "accept_refinement"
        reason = "interior unique local peak materially improves the manual pose"
    elif (
        initial_quality
        and translation <= no_change_translation
        and abs(delta_yaw) <= no_change_yaw
    ):
        decision = "keep_initial"
        reason = "manual pose already matches the best supported local cell"
    elif boundary:
        decision = "reject_boundary"
        reason = "best candidate reaches the requested search boundary"
    elif alternative is None or unique_margin < config.min_unique_margin:
        decision = "reject_ambiguous"
        reason = "a geometrically distinct candidate has a similar score"
    elif improvement < config.min_improvement:
        decision = "reject_insufficient_gain"
        reason = "candidate correction does not materially improve the fixed-beam score"
    else:
        decision = "reject_quality"
        reason = "candidate correction fails one or more map-match quality checks"

    return {
        "decision": decision,
        "reason": reason,
        "apply_recommended": accepted,
        "beam_denominator": beams.count,
        "candidate_count": len(evaluated),
        "initial": pose_score_dict(current),
        "best": pose_score_dict(best),
        "distinct_alternative": (
            pose_score_dict(alternative) if alternative is not None else None
        ),
        "delta": {
            "x_m": delta_x,
            "y_m": delta_y,
            "translation_m": translation,
            "yaw_rad": delta_yaw,
            "yaw_deg": math.degrees(delta_yaw),
        },
        "score_improvement": improvement,
        "unique_margin": unique_margin if alternative is not None else None,
        "search_boundary_hit": boundary,
        "checks": checks,
        "final_step": {
            "xy_m": xy_step,
            "yaw_rad": yaw_step,
            "yaw_deg": math.degrees(yaw_step),
        },
    }


def pose_score_dict(value: PoseScore) -> dict[str, object]:
    return {
        "pose": {"x": value.pose.x, "y": value.pose.y, "yaw": value.pose.yaw},
        "score": value.score,
        "beam_count": value.beam_count,
        "known_count": value.known_count,
        "unknown_count": value.unknown_count,
        "out_of_bounds_count": value.out_of_bounds_count,
        "hit_count": value.hit_count,
        "occupied_endpoint_count": value.occupied_endpoint_count,
        "known_fraction": value.known_fraction,
        "unknown_fraction": value.unknown_fraction,
        "out_of_bounds_fraction": value.out_of_bounds_fraction,
        "hit_fraction": value.hit_fraction,
        "footprint_clear": value.footprint_clear,
    }


def _config_dict(config: SearchConfig) -> dict[str, object]:
    return {
        "search_xy_m": config.search_xy_m,
        "search_yaw_rad": config.search_yaw_rad,
        "xy_step_m": config.xy_step_m,
        "yaw_step_rad": config.yaw_step_rad,
        "refinement_levels": config.refinement_levels,
        "hit_sigma_m": config.hit_sigma_m,
        "hit_distance_m": config.hit_distance_m,
        "unknown_penalty": config.unknown_penalty,
        "out_of_bounds_penalty": config.out_of_bounds_penalty,
        "footprint_radius_m": config.footprint_radius_m,
        "min_beams": config.min_beams,
        "min_score": config.min_score,
        "min_known_fraction": config.min_known_fraction,
        "min_hit_fraction": config.min_hit_fraction,
        "min_improvement": config.min_improvement,
        "unique_translation_m": config.unique_translation_m,
        "unique_yaw_rad": config.unique_yaw_rad,
        "min_unique_margin": config.min_unique_margin,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--search-xy-m", type=_positive, default=0.40)
    parser.add_argument("--search-yaw-deg", type=_positive, default=12.0)
    parser.add_argument("--xy-step-m", type=_positive, default=0.05)
    parser.add_argument("--yaw-step-deg", type=_positive, default=2.0)
    parser.add_argument(
        "--refinement-levels", type=_nonnegative_integer, default=2
    )
    parser.add_argument("--hit-sigma-m", type=_positive, default=0.10)
    parser.add_argument("--hit-distance-m", type=_positive, default=0.12)
    parser.add_argument("--footprint-radius-m", type=_positive, default=0.20)
    parser.add_argument("--min-beams", type=int, default=30)
    parser.add_argument("--min-score", type=float, default=0.40)
    parser.add_argument("--min-known-fraction", type=float, default=0.65)
    parser.add_argument("--min-hit-fraction", type=float, default=0.50)
    parser.add_argument("--min-improvement", type=float, default=0.02)
    parser.add_argument("--unique-translation-m", type=_positive, default=0.10)
    parser.add_argument("--unique-yaw-deg", type=_positive, default=4.0)
    parser.add_argument("--min-unique-margin", type=float, default=0.02)
    return parser.parse_args()


def load_snapshot(path: Path) -> tuple[
    dict[str, object], GridMap, Path | None, Path | None, Pose2D, BeamSet
]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot parse snapshot JSON: {error}") from error
    if not isinstance(document, dict):
        raise ValueError("snapshot must contain an object")
    map_yaml_path: Path | None = None
    image_path: Path | None = None
    if "map_yaml" in document:
        raw_map_yaml = document["map_yaml"]
        if not isinstance(raw_map_yaml, str):
            raise ValueError("snapshot map_yaml must be a string")
        map_yaml_path = _safe_input(raw_map_yaml, relative_to=path.parent)
        grid, image_path = load_map_yaml(map_yaml_path)
    elif "map" in document:
        grid = load_inline_map(document["map"])
    else:
        raise ValueError("snapshot requires map_yaml or inline map")
    initial = _pose(document.get("initial_pose"), "initial_pose")
    beams = beams_from_snapshot(document)
    return document, grid, map_yaml_path, image_path, initial, beams


def write_result(output: Path, payload: dict[str, object]) -> None:
    output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=output.parent,
        prefix=f".{output.name}.",
        delete=False,
    ) as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    temporary.replace(output)


def main() -> int:
    args = parse_args()
    try:
        snapshot_path = _safe_input(args.snapshot)
        output = resolve_output(args.output)
        _, grid, map_yaml_path, image_path, initial, beams = load_snapshot(
            snapshot_path
        )
        config = SearchConfig(
            search_xy_m=args.search_xy_m,
            search_yaw_rad=math.radians(args.search_yaw_deg),
            xy_step_m=args.xy_step_m,
            yaw_step_rad=math.radians(args.yaw_step_deg),
            refinement_levels=args.refinement_levels,
            hit_sigma_m=args.hit_sigma_m,
            hit_distance_m=args.hit_distance_m,
            footprint_radius_m=args.footprint_radius_m,
            min_beams=args.min_beams,
            min_score=args.min_score,
            min_known_fraction=args.min_known_fraction,
            min_hit_fraction=args.min_hit_fraction,
            min_improvement=args.min_improvement,
            unique_translation_m=args.unique_translation_m,
            unique_yaw_rad=math.radians(args.unique_yaw_deg),
            min_unique_margin=args.min_unique_margin,
        )
        result = refine_pose(grid, beams, initial, config)
        payload: dict[str, object] = {
            "schema": SCHEMA,
            "algorithm": ALGORITHM,
            "offline_only": True,
            "publishes_ros_topics": False,
            "snapshot": {
                "path": str(snapshot_path.relative_to(ROOT)),
                "sha256": sha256_file(snapshot_path),
            },
            "map": {
                "width": grid.width,
                "height": grid.height,
                "resolution_m": grid.resolution,
                "origin": {
                    "x": grid.origin.x,
                    "y": grid.origin.y,
                    "yaw": grid.origin.yaw,
                },
                "map_yaml": (
                    {
                        "path": str(map_yaml_path.relative_to(ROOT)),
                        "sha256": sha256_file(map_yaml_path),
                    }
                    if map_yaml_path is not None
                    else None
                ),
                "image": (
                    {
                        "path": str(image_path.relative_to(ROOT)),
                        "sha256": sha256_file(image_path),
                    }
                    if image_path is not None
                    else None
                ),
            },
            "scan_count": beams.source_scan_count,
            "config": _config_dict(config),
            "result": result,
            "operator_review_required_before_apply": True,
        }
        write_result(output, payload)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"refinement failed: {error}")
        return 1
    print(
        f"{result['decision']}: {result['reason']}; "
        f"score {result['initial']['score']:.4f} -> "
        f"{result['best']['score']:.4f}; output={output}"
    )
    return 0 if result["apply_recommended"] or result["decision"] == "keep_initial" else 2


if __name__ == "__main__":
    raise SystemExit(main())
