"""ROS-independent helpers for enforcing a planar navigation speed ceiling."""

from __future__ import annotations

import math


def bounded_linear_velocity(
    x: float,
    y: float,
    max_linear_speed_mps: float,
) -> tuple[float, float, bool]:
    """Scale planar velocity to the configured cap while preserving direction."""
    speed = math.hypot(x, y)
    if speed <= max_linear_speed_mps or speed == 0.0:
        return float(x), float(y), False
    scale = max_linear_speed_mps / speed
    return float(x) * scale, float(y) * scale, True
