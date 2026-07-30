#!/usr/bin/env python3
"""Measure Webots odometry against simulator ground truth on a fixed motion probe.

This evaluation-only node publishes ``/cmd_vel`` and records:

* ``/webots/ground_truth/odom``: supervisor truth, never used by runtime code;
* ``/odom``: the fused navigation odometry;
* ``/wheel_odom``: the raw diff-drive controller odometry.

Each odometry trajectory is rigidly aligned to the first ground-truth sample.
The resulting errors therefore measure relative motion, not arbitrary frame
origins.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Phase:
    name: str
    duration_s: float
    linear_x: float = 0.0
    angular_z: float = 0.0


PROBE = (
    Phase("settle_initial", 2.0),
    Phase("rotate_left", 8.0, angular_z=0.25),
    Phase("settle_after_left", 2.0),
    Phase("rotate_right", 8.0, angular_z=-0.25),
    Phase("settle_after_right", 2.0),
    Phase("forward", 3.0, linear_x=0.15),
    Phase("settle_after_forward", 2.0),
    Phase("reverse", 3.0, linear_x=-0.15),
    Phase("settle_final", 2.0),
)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * quantile
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def relative_pose(
    origin: tuple[float, float, float],
    pose: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = pose[0] - origin[0]
    dy = pose[1] - origin[1]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        wrap_angle(pose[2] - origin[2]),
    )


def compose_pose(
    origin: tuple[float, float, float],
    relative: tuple[float, float, float],
) -> tuple[float, float, float]:
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    return (
        origin[0] + cosine * relative[0] - sine * relative[1],
        origin[1] + sine * relative[0] + cosine * relative[1],
        wrap_angle(origin[2] + relative[2]),
    )


def nearest_sample(
    samples: list[dict[str, Any]],
    stamps: list[float],
    stamp: float,
) -> dict[str, Any] | None:
    if not samples:
        return None
    index = bisect.bisect_left(stamps, stamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda sample: abs(sample["stamp"] - stamp))


def summarize_errors(errors: list[dict[str, Any]]) -> dict[str, Any]:
    position = [row["position_error_m"] for row in errors]
    yaw = [row["yaw_error_deg"] for row in errors]
    time_delta = [row["time_delta_s"] for row in errors]
    return {
        "samples": len(errors),
        "position_error_m": {
            "median": statistics.median(position) if position else math.nan,
            "p95": percentile(position, 0.95),
            "max": max(position, default=math.nan),
        },
        "yaw_error_deg": {
            "median": statistics.median(yaw) if yaw else math.nan,
            "p95": percentile(yaw, 0.95),
            "max": max(yaw, default=math.nan),
        },
        "timestamp_delta_ms": {
            "median": (
                statistics.median(time_delta) * 1000.0
                if time_delta
                else math.nan
            ),
            "p95": percentile(time_delta, 0.95) * 1000.0,
            "max": max(time_delta, default=math.nan) * 1000.0,
        },
    }


def compare_trajectory(
    ground_truth: list[dict[str, Any]],
    measured: list[dict[str, Any]],
    phases: tuple[Phase, ...] = PROBE,
    max_time_delta_s: float = 0.06,
) -> dict[str, Any]:
    ground_truth = sorted(ground_truth, key=lambda row: row["stamp"])
    measured = sorted(measured, key=lambda row: row["stamp"])
    truth_stamps = [row["stamp"] for row in ground_truth]
    if not ground_truth or not measured:
        raise RuntimeError("ground truth and measured trajectories are required")

    measured_origin = measured[0]["pose"]
    truth_origin_sample = nearest_sample(
        ground_truth,
        truth_stamps,
        measured[0]["stamp"],
    )
    if truth_origin_sample is None:
        raise RuntimeError("cannot align trajectory without ground truth")
    truth_origin = truth_origin_sample["pose"]

    errors: list[dict[str, Any]] = []
    for sample in measured:
        truth = nearest_sample(ground_truth, truth_stamps, sample["stamp"])
        if truth is None:
            continue
        time_delta = abs(truth["stamp"] - sample["stamp"])
        if time_delta > max_time_delta_s:
            continue
        predicted = compose_pose(
            truth_origin,
            relative_pose(measured_origin, sample["pose"]),
        )
        position_error = math.hypot(
            predicted[0] - truth["pose"][0],
            predicted[1] - truth["pose"][1],
        )
        yaw_error = abs(
            math.degrees(wrap_angle(predicted[2] - truth["pose"][2]))
        )
        errors.append(
            {
                "stamp": sample["stamp"],
                "phase": sample["phase"],
                "time_delta_s": time_delta,
                "position_error_m": position_error,
                "yaw_error_deg": yaw_error,
                "predicted_pose": predicted,
                "truth_pose": truth["pose"],
            }
        )

    phases = {
        phase.name: summarize_errors(
            [row for row in errors if row["phase"] == phase.name]
        )
        for phase in phases
    }
    return {
        "alignment": {
            "measured_origin": measured_origin,
            "truth_origin": truth_origin,
        },
        "overall": summarize_errors(errors),
        "phases": phases,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/scene177-odom-ground-truth.json"),
    )
    parser.add_argument(
        "--passive-duration-s",
        type=float,
        default=0.0,
        help="Record without publishing motion commands for this duration.",
    )
    parser.add_argument(
        "--record-scans",
        action="store_true",
        help="Include full normalized LaserScan samples in the result.",
    )
    args = parser.parse_args()
    if args.passive_duration_s < 0.0:
        parser.error("--passive-duration-s must be non-negative")
    phases = (
        (Phase("passive_explore", args.passive_duration_s),)
        if args.passive_duration_s > 0.0
        else PROBE
    )

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan

    rclpy.init(args=None)
    node = rclpy.create_node("scene177_odom_ground_truth_probe")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    records: dict[str, list[dict[str, Any]]] = {
        "ground_truth": [],
        "fused_odom": [],
        "wheel_odom": [],
        "scan": [],
    }
    current_phase = "startup"

    def callback(source: str):
        def record(message: Odometry) -> None:
            orientation = message.pose.pose.orientation
            records[source].append(
                {
                    "stamp": (
                        float(message.header.stamp.sec)
                        + float(message.header.stamp.nanosec) / 1_000_000_000.0
                    ),
                    "phase": current_phase,
                    "pose": (
                        float(message.pose.pose.position.x),
                        float(message.pose.pose.position.y),
                        yaw_from_quaternion(
                            orientation.x,
                            orientation.y,
                            orientation.z,
                            orientation.w,
                        ),
                    ),
                }
            )

        return record

    reliable = QoSProfile(depth=50)
    reliable.reliability = ReliabilityPolicy.RELIABLE
    subscriptions = [
        node.create_subscription(
            Odometry,
            "/webots/ground_truth/odom",
            callback("ground_truth"),
            reliable,
        ),
        node.create_subscription(
            Odometry,
            "/odom",
            callback("fused_odom"),
            reliable,
        ),
        node.create_subscription(
            Odometry,
            "/wheel_odom",
            callback("wheel_odom"),
            reliable,
        ),
    ]

    def record_scan(message: LaserScan) -> None:
        records["scan"].append(
            {
                "stamp": (
                    float(message.header.stamp.sec)
                    + float(message.header.stamp.nanosec) / 1_000_000_000.0
                ),
                "phase": current_phase,
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment),
                "range_max": float(message.range_max),
                "ranges": [float(value) for value in message.ranges],
            }
        )

    if args.record_scans:
        subscriptions.append(
            node.create_subscription(
                LaserScan,
                "/scanner_normalized",
                record_scan,
                reliable,
            )
        )
    del subscriptions

    def publish(linear_x: float, angular_z: float) -> None:
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        publisher.publish(message)

    try:
        for phase in phases:
            current_phase = phase.name
            started = time.monotonic()
            while time.monotonic() - started < phase.duration_s:
                if args.passive_duration_s <= 0.0:
                    publish(phase.linear_x, phase.angular_z)
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.03)
        if args.passive_duration_s <= 0.0:
            current_phase = "stopped"
            for _ in range(20):
                publish(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.03)
    finally:
        if args.passive_duration_s <= 0.0:
            publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

    required_records = ("ground_truth", "fused_odom", "wheel_odom")
    if args.record_scans:
        required_records += ("scan",)
    for name in required_records:
        samples = records[name]
        if len(samples) < 20:
            raise RuntimeError(f"{name} produced only {len(samples)} samples")

    result = {
        "probe": [phase.__dict__ for phase in phases],
        "sample_counts": {
            name: len(samples) for name, samples in records.items()
        },
        "fused_odom": compare_trajectory(
            records["ground_truth"],
            records["fused_odom"],
            phases,
        ),
        "wheel_odom": compare_trajectory(
            records["ground_truth"],
            records["wheel_odom"],
            phases,
        ),
        "ground_truth_samples": records["ground_truth"],
        "scan_samples": records["scan"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, allow_nan=True),
        encoding="utf-8",
    )
    print(json.dumps({key: result[key]["overall"] for key in ("fused_odom", "wheel_odom")}, indent=2))
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
