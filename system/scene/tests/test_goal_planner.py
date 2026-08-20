# SPDX-License-Identifier: MulanPSL-2.0
import math
from types import SimpleNamespace

import numpy as np

from scene_service.goal_planner import object_goal, room_goal, room_yaw_candidates
from scene_service.robot_geometry import RobotFootprint


def _grid(width=30, height=30, resolution=0.1, fill=0):
    """Build the OccupancyGrid subset consumed by the pure planner."""
    data = np.full((height, width), fill, dtype=np.int8)
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.5, y=-1.5),
            ),
        ),
        data=data.tobytes(),
    )


def _footprint(half_x, half_y):
    """Return a rectangular Soma footprint fixture."""
    return RobotFootprint(
        points=(
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        ),
        base_frame="fixture_base",
        inscribed_radius_m=min(half_x, half_y),
        circumscribed_radius_m=(half_x**2 + half_y**2) ** 0.5,
    )


def test_room_goal_changes_with_soma_footprint():
    """The same room must accept a small robot and reject an oversized one."""
    grid = _grid()
    room = [(-0.3, -0.5), (0.3, -0.5), (0.3, 0.5), (-0.3, 0.5)]
    headings = room_yaw_candidates(room)
    assert room_goal(
        grid, room, _footprint(0.1, 0.1), yaw_candidates=headings
    ) is not None
    assert room_goal(
        grid, room, _footprint(0.6, 0.6), yaw_candidates=headings
    ) is None


def test_room_goal_rotates_asymmetric_go2_footprint_to_fit_corridor():
    """An elongated Go2-like base must not be rejected at a fixed yaw of zero."""
    grid = _grid()
    room = [(-0.18, -0.5), (0.18, -0.5), (0.18, 0.5), (-0.18, 0.5)]
    result = room_goal(
        grid,
        room,
        _footprint(0.4, 0.1),
        yaw_candidates=room_yaw_candidates(room),
    )
    assert result is not None
    _, _, yaw = result
    assert abs(abs(yaw) - math.pi / 2.0) < 1e-9


def test_object_goal_uses_complete_polygon():
    """A returned object approach pose keeps the real footprint in bounds."""
    grid = _grid(width=12, height=12)
    result = object_goal(
        grid,
        target_x=0.0,
        target_y=0.0,
        preferred_approach_yaw=0.0,
        minimum_standoff_m=0.4,
        footprint=_footprint(0.2, 0.1),
    )
    assert result is not None
    x, y, yaw = result
    assert (x * x + y * y) ** 0.5 >= 0.4
    assert np.isfinite([x, y, yaw]).all()


def test_room_goal_accepts_ros_signed_int8_sequence():
    """ROS OccupancyGrid commonly exposes unknown cells as integer -1 values."""
    grid = _grid(fill=-1)
    grid.data = [-1] * (grid.info.width * grid.info.height)
    room = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
    assert room_goal(
        grid,
        room,
        _footprint(0.1, 0.1),
        yaw_candidates=room_yaw_candidates(room),
    ) is None


def test_object_goal_rejects_unknown_cells():
    """An object approach pose must not be selected from unexplored map space."""
    grid = _grid(fill=-1)
    result = object_goal(
        grid,
        target_x=0.0,
        target_y=0.0,
        preferred_approach_yaw=None,
        minimum_standoff_m=0.4,
        footprint=_footprint(0.1, 0.1),
    )
    assert result is None


def test_object_goal_minimum_safety_margin():
    """When minimum_standoff_m is very small, the result should still be at
    least 0.3m away from the target (the default safety margin)."""
    grid = _grid(width=30, height=30)
    result = object_goal(
        grid,
        target_x=0.0,
        target_y=0.0,
        preferred_approach_yaw=0.0,
        minimum_standoff_m=0.0,  # Very small standoff
        footprint=_footprint(0.1, 0.1),
    )
    assert result is not None
    x, y, yaw = result
    distance = (x * x + y * y) ** 0.5
    # Should be at least 0.3m away due to the safety margin
    assert distance >= 0.3, f"Distance {distance:.3f}m is less than 0.3m safety margin"
    assert np.isfinite([x, y, yaw]).all()


def test_object_goal_not_at_target():
    """The returned pose should never be exactly at the target coordinates."""
    grid = _grid(width=30, height=30)
    target_x, target_y = 0.5, 0.3
    result = object_goal(
        grid,
        target_x=target_x,
        target_y=target_y,
        preferred_approach_yaw=None,
        minimum_standoff_m=0.0,
        footprint=_footprint(0.15, 0.15),
    )
    assert result is not None
    x, y, yaw = result
    # Ensure the result is not at the target position
    distance_to_target = ((x - target_x) ** 2 + (y - target_y) ** 2) ** 0.5
    assert distance_to_target > 0.01, f"Result is too close to target: {distance_to_target:.3f}m"


def test_object_goal_desired_yaw():
    """When desired_yaw is specified, the returned yaw should match it."""
    grid = _grid(width=30, height=30)
    desired_yaw = math.pi / 2  # 90 degrees
    result = object_goal(
        grid,
        target_x=0.0,
        target_y=0.0,
        preferred_approach_yaw=0.0,
        minimum_standoff_m=0.4,
        footprint=_footprint(0.2, 0.1),
        desired_yaw=desired_yaw,
    )
    assert result is not None
    x, y, yaw = result
    # The yaw should match the desired_yaw
    assert abs(yaw - desired_yaw) < 1e-9, f"Expected yaw {desired_yaw}, got {yaw}"
    # Position should still be at safe distance
    distance = (x * x + y * y) ** 0.5
    assert distance >= 0.4, f"Distance {distance:.3f}m is less than 0.4m"


def test_object_goal_desired_yaw_none():
    """When desired_yaw is None, the robot should face the object."""
    grid = _grid(width=30, height=30)
    target_x, target_y = 0.5, 0.3
    result = object_goal(
        grid,
        target_x=target_x,
        target_y=target_y,
        preferred_approach_yaw=None,
        minimum_standoff_m=0.4,
        footprint=_footprint(0.2, 0.1),
        desired_yaw=None,
    )
    assert result is not None
    x, y, yaw = result
    # Calculate expected yaw (facing the object)
    expected_yaw = math.atan2(target_y - y, target_x - x)
    assert abs(yaw - expected_yaw) < 1e-9, f"Expected yaw {expected_yaw}, got {yaw}"
