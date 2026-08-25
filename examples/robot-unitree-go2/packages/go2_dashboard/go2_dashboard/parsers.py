"""Pure telemetry parsers.

This module deliberately has no ROS or web-framework imports.  Keeping the
byte-level conversions here makes them testable on a development machine that
does not have ROS installed.
"""

from __future__ import annotations

import math
import struct
from typing import Any, Iterable, Mapping, Sequence


_POINT_FIELD_FORMATS = {
    1: "b",   # INT8
    2: "B",   # UINT8
    3: "h",   # INT16
    4: "H",   # UINT16
    5: "i",   # INT32
    6: "I",   # UINT32
    7: "f",   # FLOAT32
    8: "d",   # FLOAT64
}


def _field_value(field: Any, name: str) -> Any:
    if isinstance(field, Mapping):
        return field[name]
    return getattr(field, name)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw represented by a quaternion."""

    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    x_n, y_n, z_n, w_n = (value / norm for value in values)
    return math.atan2(
        2.0 * (w_n * z_n + x_n * y_n),
        1.0 - 2.0 * (y_n * y_n + z_n * z_n),
    )


def scan_to_points(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_points: int = 1080,
) -> list[list[float]]:
    """Convert a planar range scan to a bounded top-down point preview."""

    values = list(ranges)
    if max_points < 1:
        raise ValueError("max_points must be positive")
    if not all(
        math.isfinite(float(value))
        for value in (angle_min, angle_increment, range_min, range_max)
    ):
        raise ValueError("scan metadata contains a non-finite value")
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError("invalid scan range bounds")

    stride = max(1, (len(values) + max_points - 1) // max_points)
    points: list[list[float]] = []
    for index in range(0, len(values), stride):
        distance = float(values[index])
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        points.append(
            [
                round(distance * math.cos(angle), 4),
                round(distance * math.sin(angle), 4),
                0.0,
            ]
        )
    return points[:max_points]


def cloud_to_points(
    data: bytes | bytearray | memoryview,
    fields: Sequence[Any],
    point_step: int,
    width: int,
    height: int,
    is_bigendian: bool,
    max_points: int = 1200,
    row_step: int | None = None,
) -> list[list[float]]:
    """Decode x/y/z fields from a PointCloud2-compatible byte buffer.

    Only the x, y, and z scalar fields are read.  Other fields and row padding
    are ignored, which is sufficient for a bounded browser preview while
    avoiding a heavy numerical dependency.
    """

    if point_step < 1 or width < 0 or height < 0 or max_points < 1:
        raise ValueError("invalid point-cloud dimensions")
    field_by_name = {
        str(_field_value(field, "name")): field
        for field in fields
        if str(_field_value(field, "name")) in {"x", "y", "z"}
    }
    if set(field_by_name) != {"x", "y", "z"}:
        raise ValueError("point cloud does not contain x, y, and z fields")

    endian = ">" if is_bigendian else "<"
    decoders: dict[str, tuple[struct.Struct, int]] = {}
    for name in ("x", "y", "z"):
        field = field_by_name[name]
        datatype = int(_field_value(field, "datatype"))
        offset = int(_field_value(field, "offset"))
        fmt = _POINT_FIELD_FORMATS.get(datatype)
        if fmt is None:
            raise ValueError(f"unsupported {name} datatype: {datatype}")
        decoder = struct.Struct(endian + fmt)
        if offset < 0 or offset + decoder.size > point_step:
            raise ValueError(f"invalid {name} field offset")
        decoders[name] = (decoder, offset)

    view = memoryview(data)
    packed_row_size = int(width) * int(point_step)
    effective_row_step = packed_row_size if row_step is None else int(row_step)
    if effective_row_step < packed_row_size:
        raise ValueError("point-cloud row step is smaller than a packed row")
    declared_points = int(width) * int(height)
    total = declared_points
    stride = max(1, (total + max_points - 1) // max_points)
    points: list[list[float]] = []
    for index in range(0, total, stride):
        row = index // int(width) if width else 0
        column = index % int(width) if width else 0
        base = row * effective_row_step + column * int(point_step)
        if base + int(point_step) > len(view):
            continue
        xyz = []
        for name in ("x", "y", "z"):
            decoder, offset = decoders[name]
            xyz.append(float(decoder.unpack_from(view, base + offset)[0]))
        if not all(math.isfinite(value) for value in xyz):
            continue
        points.append([round(value, 4) for value in xyz])
    return points[:max_points]


def occupancy_to_luma(
    data: Sequence[int], width: int, height: int
) -> bytes:
    """Convert OccupancyGrid cells into top-left-origin grayscale pixels."""

    if width < 1 or height < 1 or len(data) < width * height:
        raise ValueError("occupancy grid data is smaller than its dimensions")
    pixels = bytearray(width * height)
    for source_y in range(height):
        target_y = height - source_y - 1
        for x in range(width):
            value = int(data[source_y * width + x])
            if value < 0:
                shade = 150
            else:
                value = min(100, max(0, value))
                shade = int(round(250 - value * 2.3))
            pixels[target_y * width + x] = shade
    return bytes(pixels)


def image_to_rgb_bytes(
    data: bytes | bytearray | memoryview,
    width: int,
    height: int,
    step: int,
    encoding: str,
) -> bytes:
    """Normalize common ROS image encodings to tightly packed RGB bytes."""

    encoding_key = encoding.strip().lower()
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }
    channels = channels_by_encoding.get(encoding_key)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {encoding}")
    if width < 1 or height < 1 or step < width * channels:
        raise ValueError("invalid image dimensions or row step")
    view = memoryview(data)
    if len(view) < step * height:
        raise ValueError("image data is smaller than its dimensions")

    output = bytearray(width * height * 3)
    target = 0
    packed_source_row = width * channels
    packed_target_row = width * 3
    for row_index in range(height):
        row = view[
            row_index * step : row_index * step + packed_source_row
        ]
        if encoding_key == "rgb8":
            output[target : target + packed_target_row] = row
            target += packed_target_row
            continue
        row_end = target + packed_target_row
        if encoding_key == "mono8":
            output[target:row_end:3] = row
            output[target + 1 : row_end : 3] = row
            output[target + 2 : row_end : 3] = row
        elif encoding_key in {"bgr8", "bgra8"}:
            output[target:row_end:3] = row[2:packed_source_row:channels]
            output[target + 1 : row_end : 3] = row[1:packed_source_row:channels]
            output[target + 2 : row_end : 3] = row[0:packed_source_row:channels]
        else:  # rgba8
            output[target:row_end:3] = row[0:packed_source_row:channels]
            output[target + 1 : row_end : 3] = row[1:packed_source_row:channels]
            output[target + 2 : row_end : 3] = row[2:packed_source_row:channels]
        target = row_end
    return bytes(output)


def goal_status_label(status: int) -> str:
    """Translate action_msgs/GoalStatus constants without importing ROS."""

    return {
        0: "unknown",
        1: "accepted",
        2: "executing",
        3: "canceling",
        4: "succeeded",
        5: "canceled",
        6: "aborted",
    }.get(int(status), "unknown")
