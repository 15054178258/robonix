"""Pure state machine for the Ranger final velocity safety guard."""

from __future__ import annotations

import math
from dataclasses import dataclass


def shortest_angle(source: float, target: float) -> float:
    return math.atan2(math.sin(target - source), math.cos(target - source))


def normalize_uuid_octets(raw_uuid) -> list[int]:
    """Return the plain Python integers required by ROS UUID messages."""
    values = [int(value) for value in raw_uuid]
    if len(values) != 16 or any(value < 0 or value > 255 for value in values):
        raise ValueError("goal UUID must contain exactly 16 byte values")
    return values


@dataclass
class GuardLimits:
    terminal_xy_m: float = 0.45
    rotating_linear_mps: float = 0.05
    rotating_angular_rps: float = 0.05
    terminal_timeout_s: float = 15.0
    terminal_rotation_margin_rad: float = 0.50
    terminal_no_progress_s: float = 3.0
    terminal_min_progress_rad: float = 0.04
    global_spin_timeout_s: float = 25.0
    global_spin_limit_rad: float = 2.0 * math.pi + 0.50


class RotationGuard:
    """Latch a navigation UUID after unsafe stationary rotation.

    Rotation is integrated from odometry, not commanded velocity. Terminal
    limits use the current global-plan endpoint, while the global limits catch
    recovery/controller loops that never enter the terminal window.
    """

    def __init__(self, limits: GuardLimits | None = None):
        self.limits = limits or GuardLimits()
        self.goal_uuid = ""
        self.latched_reason = ""
        self.spin_started_at: float | None = None
        self.spin_rotation = 0.0
        self.terminal_started_at: float | None = None
        self.terminal_rotation = 0.0
        self.terminal_allowed_rotation = 0.0
        self.best_yaw_error = math.inf
        self.last_progress_at: float | None = None
        self.last_odom_yaw: float | None = None
        self.rotating = False

    def begin_goal(self, goal_uuid: str) -> None:
        if goal_uuid == self.goal_uuid:
            return
        self.goal_uuid = goal_uuid
        self.latched_reason = ""
        self.spin_started_at = None
        self.spin_rotation = 0.0
        self.terminal_started_at = None
        self.terminal_rotation = 0.0
        self.terminal_allowed_rotation = 0.0
        self.best_yaw_error = math.inf
        self.last_progress_at = None
        self.last_odom_yaw = None
        self.rotating = False

    def clear_if_idle(self) -> None:
        self.goal_uuid = ""
        self.latched_reason = ""
        self.rotating = False

    def observe_odom(self, yaw: float) -> None:
        if self.last_odom_yaw is not None and self.rotating:
            delta = abs(shortest_angle(self.last_odom_yaw, yaw))
            self.spin_rotation += delta
            if self.terminal_started_at is not None:
                self.terminal_rotation += delta
        self.last_odom_yaw = yaw

    def evaluate(
        self,
        now: float,
        linear_speed: float,
        angular_speed: float,
        robot_pose: tuple[float, float, float] | None,
        goal_pose: tuple[float, float, float] | None,
    ) -> str:
        if self.latched_reason:
            return self.latched_reason
        if not self.goal_uuid:
            self.rotating = False
            return ""

        self.rotating = (
            abs(linear_speed) <= self.limits.rotating_linear_mps
            and abs(angular_speed) >= self.limits.rotating_angular_rps
        )
        if not self.rotating:
            self.spin_started_at = None
            self.spin_rotation = 0.0
            return ""

        if self.spin_started_at is None:
            self.spin_started_at = now
            self.spin_rotation = 0.0
        if now - self.spin_started_at > self.limits.global_spin_timeout_s:
            return self._trip("continuous stationary rotation timeout")
        if self.spin_rotation > self.limits.global_spin_limit_rad:
            return self._trip("continuous stationary rotation limit")

        if robot_pose is None or goal_pose is None:
            return ""
        distance = math.hypot(robot_pose[0] - goal_pose[0], robot_pose[1] - goal_pose[1])
        if distance > self.limits.terminal_xy_m:
            return ""

        yaw_error = abs(shortest_angle(robot_pose[2], goal_pose[2]))
        if self.terminal_started_at is None:
            self.terminal_started_at = now
            self.terminal_rotation = 0.0
            self.terminal_allowed_rotation = max(
                0.50, yaw_error + self.limits.terminal_rotation_margin_rad
            )
            self.best_yaw_error = yaw_error
            self.last_progress_at = now
        if now - self.terminal_started_at > self.limits.terminal_timeout_s:
            return self._trip("terminal rotation timeout")
        if self.terminal_rotation > self.terminal_allowed_rotation:
            return self._trip("terminal cumulative rotation limit")
        if yaw_error + self.limits.terminal_min_progress_rad < self.best_yaw_error:
            self.best_yaw_error = yaw_error
            self.last_progress_at = now
        elif (
            self.last_progress_at is not None
            and now - self.last_progress_at > self.limits.terminal_no_progress_s
        ):
            return self._trip("terminal yaw made no progress")
        return ""

    def _trip(self, reason: str) -> str:
        self.latched_reason = reason
        return reason
