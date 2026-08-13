"""Dependency-free planar transform helpers for map-to-odom bridging."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TimedPose2:
    stamp_ns: int
    pose: Pose2


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def compose(a: Pose2, b: Pose2) -> Pose2:
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    return Pose2(
        a.x + c * b.x - s * b.y,
        a.y + s * b.x + c * b.y,
        wrap_angle(a.yaw + b.yaw),
    )


def inverse(value: Pose2) -> Pose2:
    c = math.cos(value.yaw)
    s = math.sin(value.yaw)
    return Pose2(
        -c * value.x - s * value.y,
        s * value.x - c * value.y,
        wrap_angle(-value.yaw),
    )


def interpolate(a: TimedPose2, b: TimedPose2, stamp_ns: int) -> Pose2:
    if b.stamp_ns <= a.stamp_ns:
        return a.pose
    ratio = (stamp_ns - a.stamp_ns) / (b.stamp_ns - a.stamp_ns)
    ratio = min(1.0, max(0.0, ratio))
    dyaw = wrap_angle(b.pose.yaw - a.pose.yaw)
    return Pose2(
        a.pose.x + ratio * (b.pose.x - a.pose.x),
        a.pose.y + ratio * (b.pose.y - a.pose.y),
        wrap_angle(a.pose.yaw + ratio * dyaw),
    )
