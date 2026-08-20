"""Pure range-neighborhood filter used by the ROS scan filter node."""

from __future__ import annotations

import math


def filter_ranges(
    ranges: list[float],
    range_min: float,
    range_max: float,
    window_bins: int = 2,
    max_range_delta_m: float = 0.20,
    min_neighbors: int = 2,
) -> tuple[list[float], list[int]]:
    valid = [
        math.isfinite(value) and range_min <= value <= range_max
        for value in ranges
    ]
    output = list(ranges)
    removed = []
    size = len(ranges)
    for index, value in enumerate(ranges):
        if not valid[index]:
            continue
        neighbors = 0
        for offset in range(-window_bins, window_bins + 1):
            other_index = index + offset
            if offset == 0 or other_index < 0 or other_index >= size:
                continue
            if valid[other_index] and abs(ranges[other_index] - value) <= max_range_delta_m:
                neighbors += 1
        if neighbors < min_neighbors:
            output[index] = math.inf
            removed.append(index)
    return output, removed
