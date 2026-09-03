#!/usr/bin/env python3
"""Bounded, subscription-only stationary observer for canonical ``/odom``.

The process creates exactly one ROS subscription.  It never creates a
publisher, service, action client, timer, Unitree SDK object, or subprocess.
It records aggregate stationary-odometry evidence only and does not authorize
motion.  The observation deadline is based on the host monotonic clock.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import time
from typing import Any, Sequence


ROOT = Path(__file__).resolve().parents[1]
LOG_ROOT = ROOT / "logs"
TOPIC = "/odom"
MESSAGE_TYPE = "nav_msgs/msg/Odometry"
EXPECTED_FRAME_ID = "odom"
EXPECTED_CHILD_FRAME_ID = "base_link"
MIN_DURATION_SECONDS = 1
MAX_DURATION_SECONDS = 3_600
MAX_RETAINED_SAMPLES = 1_000_000
MAX_RECORDED_ISSUES = 32
DEFAULT_MAX_SOURCE_AGE_SECONDS = 0.20
DEFAULT_MAX_FUTURE_SKEW_SECONDS = 0.05
DEFAULT_MAX_STATIONARY_LINEAR_SPEED_MPS = 0.03
DEFAULT_MAX_STATIONARY_YAW_RATE_RAD_PER_SECOND = 0.03


def _bounded_int(value: str, minimum: int, maximum: int, label: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{label} must be an integer") from error
    if not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{label} must be in {minimum}..{maximum}"
        )
    return parsed


def _bounded_float(
    value: str, minimum: float, maximum: float, label: str
) -> float:
    try:
        parsed = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{label} must be numeric") from error
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{label} must be finite and in {minimum}..{maximum}"
        )
    return parsed


def _validate_observation_window(arguments: argparse.Namespace) -> None:
    if (
        2.0 * arguments.gap_threshold_seconds
        >= arguments.duration_seconds
    ):
        raise ValueError(
            "2 * gap-threshold-seconds must be strictly less than "
            "duration-seconds"
        )


class _ObserverArgumentParser(argparse.ArgumentParser):
    def parse_args(self, args=None, namespace=None):
        arguments = super().parse_args(args=args, namespace=namespace)
        try:
            _validate_observation_window(arguments)
        except ValueError as error:
            self.error(str(error))
        return arguments


def build_argument_parser() -> argparse.ArgumentParser:
    parser = _ObserverArgumentParser(
        description=(
            "READ-ONLY bounded stationary observer for canonical /odom; "
            "creates one subscription and no command-capable ROS endpoint"
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="new evidence directory below this package's logs directory",
    )
    parser.add_argument(
        "--duration-seconds",
        type=lambda value: _bounded_int(
            value,
            MIN_DURATION_SECONDS,
            MAX_DURATION_SECONDS,
            "duration-seconds",
        ),
        required=True,
    )
    parser.add_argument(
        "--gap-threshold-seconds",
        type=lambda value: _bounded_float(
            value, 0.001, 10.0, "gap-threshold-seconds"
        ),
        default=0.2,
        help="maximum accepted receipt/source gap and end freshness",
    )
    parser.add_argument(
        "--max-source-age-seconds",
        type=lambda value: _bounded_float(
            value, 0.001, 5.0, "max-source-age-seconds"
        ),
        default=DEFAULT_MAX_SOURCE_AGE_SECONDS,
    )
    parser.add_argument(
        "--max-future-skew-seconds",
        type=lambda value: _bounded_float(
            value, 0.001, 1.0, "max-future-skew-seconds"
        ),
        default=DEFAULT_MAX_FUTURE_SKEW_SECONDS,
    )
    parser.add_argument(
        "--max-stationary-linear-speed-mps",
        type=lambda value: _bounded_float(
            value, 0.001, 0.20, "max-stationary-linear-speed-mps"
        ),
        default=DEFAULT_MAX_STATIONARY_LINEAR_SPEED_MPS,
    )
    parser.add_argument(
        "--max-stationary-yaw-rate-rad-per-second",
        type=lambda value: _bounded_float(
            value,
            0.001,
            0.50,
            "max-stationary-yaw-rate-rad-per-second",
        ),
        default=DEFAULT_MAX_STATIONARY_YAW_RATE_RAD_PER_SECOND,
    )
    return parser


def _prepare_output(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    logs_root = LOG_ROOT.resolve()
    try:
        resolved.relative_to(logs_root)
    except ValueError as error:
        raise RuntimeError(f"output directory must be below {logs_root}") from error
    if resolved == logs_root:
        raise RuntimeError("output directory must be a new child below the logs root")
    if resolved.exists():
        raise RuntimeError(f"refusing to reuse existing evidence directory: {resolved}")
    resolved.mkdir(mode=0o700, parents=True, exist_ok=False)
    os.chmod(resolved, 0o700)
    return resolved


def _prepare_ros_log_dir(output_dir: Path) -> Path:
    ros_log_dir = (output_dir / "ros-logs").resolve()
    try:
        ros_log_dir.relative_to(output_dir.resolve())
    except ValueError as error:
        raise RuntimeError(
            f"ROS log directory must remain below {output_dir.resolve()}"
        ) from error
    ros_log_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(ros_log_dir, 0o700)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)
    return ros_log_dir


def _write_json(path: Path, payload: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(
            payload,
            stream,
            allow_nan=False,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    fraction = position - lower
    return float(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction)


def _gap_summary(gaps_seconds: Sequence[float], threshold_seconds: float) -> dict[str, Any]:
    if not gaps_seconds:
        return {
            "observed_intervals": 0,
            "minimum_seconds": None,
            "mean_seconds": None,
            "p50_seconds": None,
            "p95_seconds": None,
            "p99_seconds": None,
            "maximum_seconds": None,
            "threshold_seconds": threshold_seconds,
            "intervals_above_threshold": 0,
        }
    return {
        "observed_intervals": len(gaps_seconds),
        "minimum_seconds": min(gaps_seconds),
        "mean_seconds": sum(gaps_seconds) / len(gaps_seconds),
        "p50_seconds": _percentile(gaps_seconds, 0.50),
        "p95_seconds": _percentile(gaps_seconds, 0.95),
        "p99_seconds": _percentile(gaps_seconds, 0.99),
        "maximum_seconds": max(gaps_seconds),
        "threshold_seconds": threshold_seconds,
        "intervals_above_threshold": sum(
            gap > threshold_seconds for gap in gaps_seconds
        ),
    }


def _linear_slope(times_seconds: Sequence[float], values: Sequence[float]) -> float:
    if len(times_seconds) != len(values) or len(values) < 2:
        return 0.0
    mean_time = sum(times_seconds) / len(times_seconds)
    mean_value = sum(values) / len(values)
    denominator = sum((value - mean_time) ** 2 for value in times_seconds)
    if denominator == 0.0:
        return 0.0
    numerator = sum(
        (time_value - mean_time) * (sample_value - mean_value)
        for time_value, sample_value in zip(times_seconds, values)
    )
    return numerator / denominator


def _unwrap_yaw(samples: Sequence["OdomSample"]) -> list[float]:
    if not samples:
        return []
    unwrapped = [samples[0].yaw_rad]
    previous_wrapped = samples[0].yaw_rad
    for sample in samples[1:]:
        delta = math.atan2(
            math.sin(sample.yaw_rad - previous_wrapped),
            math.cos(sample.yaw_rad - previous_wrapped),
        )
        unwrapped.append(unwrapped[-1] + delta)
        previous_wrapped = sample.yaw_rad
    return unwrapped


@dataclass(frozen=True)
class OdomSample:
    receipt_ns: int
    receipt_realtime_ns: int
    source_stamp_ns: int
    source_age_seconds: float
    x: float
    y: float
    z: float
    yaw_rad: float
    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float


class CanonicalOdomTracker:
    """Validate canonical odometry and retain a bounded stationary series."""

    INVALID_REASONS = (
        "malformed_message",
        "non_monotonic_receipt_time",
        "invalid_stamp",
        "non_monotonic_source_stamp",
        "stale_source_stamp",
        "future_source_stamp",
        "frame_id_mismatch",
        "child_frame_id_mismatch",
        "non_finite_pose",
        "non_finite_twist",
        "non_finite_covariance",
        "invalid_quaternion_norm",
        "sample_retention_limit",
    )

    def __init__(
        self,
        gap_threshold_seconds: float,
        max_source_age_seconds: float = DEFAULT_MAX_SOURCE_AGE_SECONDS,
        max_future_skew_seconds: float = DEFAULT_MAX_FUTURE_SKEW_SECONDS,
    ) -> None:
        self.gap_threshold_seconds = gap_threshold_seconds
        self.max_source_age_seconds = max_source_age_seconds
        self.max_future_skew_seconds = max_future_skew_seconds
        self.received_messages = 0
        self.valid_messages = 0
        self.invalid_messages = 0
        self.invalid_reasons = {reason: 0 for reason in self.INVALID_REASONS}
        self.issue_examples: list[dict[str, Any]] = []
        self.samples: list[OdomSample] = []
        self.receipt_gaps_seconds: list[float] = []
        self.source_gaps_seconds: list[float] = []
        self.first_receipt_ns: int | None = None
        self.last_receipt_ns: int | None = None
        self.last_source_stamp_ns: int | None = None
        self.last_valid_receipt_ns: int | None = None
        self.last_valid_source_stamp_ns: int | None = None

    def _mark_invalid(
        self, reasons: Sequence[str], receipt_ns: int, source_stamp_ns: int | None
    ) -> None:
        self.invalid_messages += 1
        for reason in reasons:
            self.invalid_reasons[reason] += 1
        if len(self.issue_examples) < MAX_RECORDED_ISSUES:
            self.issue_examples.append(
                {
                    "message_index": self.received_messages,
                    "receipt_monotonic_ns": receipt_ns,
                    "source_stamp_ns": source_stamp_ns,
                    "reasons": list(reasons),
                }
            )

    def observe(
        self,
        message: Any,
        receipt_ns: int,
        receipt_realtime_ns: int | None = None,
    ) -> OdomSample | None:
        self.received_messages += 1
        reasons: list[str] = []
        source_stamp_ns: int | None = None
        if receipt_realtime_ns is None:
            receipt_realtime_ns = time.time_ns()
        retention_available = self.received_messages <= MAX_RETAINED_SAMPLES
        if not retention_available:
            reasons.append("sample_retention_limit")

        if self.first_receipt_ns is None:
            self.first_receipt_ns = receipt_ns
        if self.last_receipt_ns is not None:
            if receipt_ns <= self.last_receipt_ns:
                reasons.append("non_monotonic_receipt_time")
        if self.last_receipt_ns is None or receipt_ns > self.last_receipt_ns:
            self.last_receipt_ns = receipt_ns

        try:
            frame_id = str(message.header.frame_id)
            child_frame_id = str(message.child_frame_id)
            stamp_sec = int(message.header.stamp.sec)
            stamp_nanosec = int(message.header.stamp.nanosec)
            pose = message.pose.pose
            twist = message.twist.twist
            values = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
            twist_values = (
                float(twist.linear.x),
                float(twist.linear.y),
                float(twist.linear.z),
                float(twist.angular.x),
                float(twist.angular.y),
                float(twist.angular.z),
            )
            pose_covariance = tuple(float(value) for value in message.pose.covariance)
            twist_covariance = tuple(float(value) for value in message.twist.covariance)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._mark_invalid(["malformed_message", *reasons], receipt_ns, None)
            return None

        if len(pose_covariance) != 36 or len(twist_covariance) != 36:
            reasons.append("malformed_message")

        if stamp_sec < 0 or not 0 <= stamp_nanosec < 1_000_000_000:
            reasons.append("invalid_stamp")
        else:
            source_stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
            if source_stamp_ns <= 0:
                reasons.append("invalid_stamp")
            elif (
                self.last_source_stamp_ns is not None
                and source_stamp_ns <= self.last_source_stamp_ns
            ):
                reasons.append("non_monotonic_source_stamp")
            else:
                self.last_source_stamp_ns = source_stamp_ns

            if source_stamp_ns > 0:
                source_age_seconds = (
                    receipt_realtime_ns - source_stamp_ns
                ) / 1_000_000_000.0
                if source_age_seconds > self.max_source_age_seconds:
                    reasons.append("stale_source_stamp")
                elif source_age_seconds < -self.max_future_skew_seconds:
                    reasons.append("future_source_stamp")
            else:
                source_age_seconds = math.nan

        if frame_id != EXPECTED_FRAME_ID:
            reasons.append("frame_id_mismatch")
        if child_frame_id != EXPECTED_CHILD_FRAME_ID:
            reasons.append("child_frame_id_mismatch")
        if not all(math.isfinite(value) for value in values):
            reasons.append("non_finite_pose")
        if not all(math.isfinite(value) for value in twist_values):
            reasons.append("non_finite_twist")
        if not all(
            math.isfinite(value)
            for value in (*pose_covariance, *twist_covariance)
        ):
            reasons.append("non_finite_covariance")

        qx, qy, qz, qw = values[3:]
        quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(quaternion_norm) or not 0.98 <= quaternion_norm <= 1.02:
            reasons.append("invalid_quaternion_norm")
        if reasons:
            self._mark_invalid(reasons, receipt_ns, source_stamp_ns)
            return None

        assert source_stamp_ns is not None
        assert math.isfinite(source_age_seconds)
        qx /= quaternion_norm
        qy /= quaternion_norm
        qz /= quaternion_norm
        qw /= quaternion_norm
        yaw_rad = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        sample = OdomSample(
            receipt_ns=receipt_ns,
            receipt_realtime_ns=receipt_realtime_ns,
            source_stamp_ns=source_stamp_ns,
            source_age_seconds=source_age_seconds,
            x=values[0],
            y=values[1],
            z=values[2],
            yaw_rad=yaw_rad,
            linear_x=twist_values[0],
            linear_y=twist_values[1],
            linear_z=twist_values[2],
            angular_x=twist_values[3],
            angular_y=twist_values[4],
            angular_z=twist_values[5],
        )
        if self.last_valid_receipt_ns is not None:
            self.receipt_gaps_seconds.append(
                (receipt_ns - self.last_valid_receipt_ns) / 1_000_000_000.0
            )
        if self.last_valid_source_stamp_ns is not None:
            self.source_gaps_seconds.append(
                (source_stamp_ns - self.last_valid_source_stamp_ns)
                / 1_000_000_000.0
            )
        self.last_valid_receipt_ns = receipt_ns
        self.last_valid_source_stamp_ns = source_stamp_ns
        self.samples.append(sample)
        self.valid_messages += 1
        return sample

    def summary(self, finished_receipt_ns: int | None = None) -> dict[str, Any]:
        receipt_span_seconds = (
            0.0
            if len(self.samples) < 2
            else max(
                0.0,
                (self.samples[-1].receipt_ns - self.samples[0].receipt_ns) / 1e9,
            )
        )
        source_span_seconds = (
            0.0
            if len(self.samples) < 2
            else (
                self.samples[-1].source_stamp_ns
                - self.samples[0].source_stamp_ns
            )
            / 1e9
        )
        base = {
            "received_messages": self.received_messages,
            "valid_messages": self.valid_messages,
            "invalid_messages": self.invalid_messages,
            "invalid_reasons": dict(self.invalid_reasons),
            "issue_examples": list(self.issue_examples),
            "issue_examples_truncated": max(
                0, self.invalid_messages - len(self.issue_examples)
            ),
            "retained_samples": len(self.samples),
            "receipt_span_seconds": receipt_span_seconds,
            "source_stamp_span_seconds": source_span_seconds,
            "receipt_sample_rate_hz": (
                (self.valid_messages - 1) / receipt_span_seconds
                if self.valid_messages > 1 and receipt_span_seconds > 0.0
                else 0.0
            ),
            "valid_source_sample_rate_hz": (
                (self.valid_messages - 1) / source_span_seconds
                if self.valid_messages > 1 and source_span_seconds > 0.0
                else 0.0
            ),
            "receipt_gaps": _gap_summary(
                self.receipt_gaps_seconds, self.gap_threshold_seconds
            ),
            "source_stamp_gaps": _gap_summary(
                self.source_gaps_seconds, self.gap_threshold_seconds
            ),
            "last_receipt_age_seconds": (
                None
                if finished_receipt_ns is None or not self.samples
                else max(
                    0.0,
                    (finished_receipt_ns - self.samples[-1].receipt_ns) / 1e9,
                )
            ),
            "source_age_seconds": {
                "minimum": (
                    min(sample.source_age_seconds for sample in self.samples)
                    if self.samples
                    else None
                ),
                "mean": (
                    sum(sample.source_age_seconds for sample in self.samples)
                    / len(self.samples)
                    if self.samples
                    else None
                ),
                "maximum": (
                    max(sample.source_age_seconds for sample in self.samples)
                    if self.samples
                    else None
                ),
                "maximum_accepted_seconds": self.max_source_age_seconds,
                "maximum_accepted_future_skew_seconds": self.max_future_skew_seconds,
            },
            "analyzed": False,
        }
        if len(self.samples) < 2:
            return base

        samples = self.samples
        unwrapped_yaw = _unwrap_yaw(samples)
        first = samples[0]
        last = samples[-1]
        source_times = [
            (sample.source_stamp_ns - first.source_stamp_ns) / 1e9
            for sample in samples
        ]
        receipt_times = [
            (sample.receipt_ns - first.receipt_ns) / 1e9 for sample in samples
        ]
        source_yaw_slope = _linear_slope(source_times, unwrapped_yaw)
        receipt_yaw_slope = _linear_slope(receipt_times, unwrapped_yaw)

        def translation_from_start(sample: OdomSample) -> float:
            return math.hypot(
                sample.x - first.x,
                sample.y - first.y,
                sample.z - first.z,
            )

        def planar_translation_from_start(sample: OdomSample) -> float:
            return math.hypot(sample.x - first.x, sample.y - first.y)

        return {
            **base,
            "analyzed": True,
            "translation_end_to_end_m": translation_from_start(last),
            "translation_excursion_from_start_m": max(
                translation_from_start(sample) for sample in samples
            ),
            "planar_translation_end_to_end_m": planar_translation_from_start(last),
            "planar_translation_excursion_from_start_m": max(
                planar_translation_from_start(sample) for sample in samples
            ),
            "yaw_unwrap_end_to_end_rad": unwrapped_yaw[-1] - unwrapped_yaw[0],
            "yaw_unwrap_end_to_end_deg": math.degrees(
                unwrapped_yaw[-1] - unwrapped_yaw[0]
            ),
            "yaw_unwrap_excursion_rad": max(unwrapped_yaw) - min(unwrapped_yaw),
            "yaw_unwrap_excursion_deg": math.degrees(
                max(unwrapped_yaw) - min(unwrapped_yaw)
            ),
            "yaw_slope_rad_per_second": source_yaw_slope,
            "yaw_slope_deg_per_minute": math.degrees(source_yaw_slope) * 60.0,
            "yaw_slope_time_basis": "source_stamp",
            "yaw_slope_deg_per_minute_receipt_time": (
                math.degrees(receipt_yaw_slope) * 60.0
            ),
            "reported_planar_speed_max_mps": max(
                math.hypot(sample.linear_x, sample.linear_y) for sample in samples
            ),
            "reported_linear_speed_3d_max_mps": max(
                math.hypot(sample.linear_x, sample.linear_y, sample.linear_z)
                for sample in samples
            ),
            "reported_abs_yaw_rate_max_rad_per_second": max(
                abs(sample.angular_z) for sample in samples
            ),
            "reported_angular_speed_3d_max_rad_per_second": max(
                math.hypot(sample.angular_x, sample.angular_y, sample.angular_z)
                for sample in samples
            ),
            "start": {
                "receipt_monotonic_ns": first.receipt_ns,
                "receipt_realtime_ns": first.receipt_realtime_ns,
                "source_stamp_ns": first.source_stamp_ns,
                "source_age_seconds": first.source_age_seconds,
                "position": {"x": first.x, "y": first.y, "z": first.z},
                "yaw_wrapped_rad": first.yaw_rad,
                "yaw_unwrapped_rad": unwrapped_yaw[0],
            },
            "end": {
                "receipt_monotonic_ns": last.receipt_ns,
                "receipt_realtime_ns": last.receipt_realtime_ns,
                "source_stamp_ns": last.source_stamp_ns,
                "source_age_seconds": last.source_age_seconds,
                "position": {"x": last.x, "y": last.y, "z": last.z},
                "yaw_wrapped_rad": last.yaw_rad,
                "yaw_unwrapped_rad": unwrapped_yaw[-1],
            },
        }


class TerminationRequest:
    def __init__(self) -> None:
        self.signal_number: int | None = None

    def handler(self, signal_number: int, _frame: Any) -> None:
        if self.signal_number is None:
            self.signal_number = int(signal_number)

    @property
    def requested(self) -> bool:
        return self.signal_number is not None

    @property
    def reason(self) -> str:
        if self.signal_number == signal.SIGINT:
            return "signal_sigint"
        if self.signal_number == signal.SIGTERM:
            return "signal_sigterm"
        return f"signal_{self.signal_number}"


def _iso_utc(realtime_ns: int) -> str:
    return datetime.fromtimestamp(realtime_ns / 1e9, timezone.utc).isoformat()


def _build_result(
    *,
    arguments: argparse.Namespace,
    tracker: CanonicalOdomTracker,
    ros_log_dir: Path,
    started_realtime_ns: int,
    started_monotonic_ns: int,
    finished_realtime_ns: int,
    finished_monotonic_ns: int,
    exit_reason: str,
    runtime_error: str | None,
    cleanup_errors: Sequence[str],
    subscriptions_created: int,
    observation_started_monotonic_ns: int | None,
    observation_finished_monotonic_ns: int | None = None,
) -> dict[str, Any]:
    evidence_finished_monotonic_ns = (
        observation_finished_monotonic_ns
        if observation_finished_monotonic_ns is not None
        else finished_monotonic_ns
    )
    metrics = tracker.summary(evidence_finished_monotonic_ns)
    receipt_gap_max = metrics["receipt_gaps"]["maximum_seconds"]
    source_gap_max = metrics["source_stamp_gaps"]["maximum_seconds"]
    last_receipt_age = metrics["last_receipt_age_seconds"]
    minimum_receipt_span = max(
        0.0,
        arguments.duration_seconds - 2.0 * arguments.gap_threshold_seconds,
    )
    checks = {
        "bounded_duration_elapsed": exit_reason == "duration_elapsed",
        "at_least_two_valid_samples": tracker.valid_messages >= 2,
        "all_received_messages_valid": (
            tracker.received_messages > 0 and tracker.invalid_messages == 0
        ),
        "exactly_one_subscription_created": subscriptions_created == 1,
        "requested_observation_window_covered": (
            metrics["receipt_span_seconds"] >= minimum_receipt_span
        ),
        "receipt_gap_within_threshold": (
            receipt_gap_max is not None
            and receipt_gap_max <= arguments.gap_threshold_seconds
        ),
        "source_stamp_gap_within_threshold": (
            source_gap_max is not None
            and source_gap_max <= arguments.gap_threshold_seconds
        ),
        "last_receipt_fresh_at_finish": (
            last_receipt_age is not None
            and last_receipt_age <= arguments.gap_threshold_seconds
        ),
        "frame_ids_valid": (
            tracker.invalid_reasons["frame_id_mismatch"] == 0
            and tracker.invalid_reasons["child_frame_id_mismatch"] == 0
        ),
        "source_stamps_valid_and_strictly_increasing": (
            tracker.invalid_reasons["invalid_stamp"] == 0
            and tracker.invalid_reasons["non_monotonic_source_stamp"] == 0
            and tracker.invalid_reasons["stale_source_stamp"] == 0
            and tracker.invalid_reasons["future_source_stamp"] == 0
        ),
        "poses_finite": tracker.invalid_reasons["non_finite_pose"] == 0,
        "twists_finite": tracker.invalid_reasons["non_finite_twist"] == 0,
        "covariances_finite": (
            tracker.invalid_reasons["non_finite_covariance"] == 0
        ),
        "quaternions_valid": (
            tracker.invalid_reasons["invalid_quaternion_norm"] == 0
        ),
        "receipt_clock_strictly_increasing": (
            tracker.invalid_reasons["non_monotonic_receipt_time"] == 0
        ),
        "runtime_error_absent": runtime_error is None,
        "cleanup_errors_absent": not cleanup_errors,
    }
    observer_valid = all(checks.values())
    stationary_checks = {
        "observer_valid": observer_valid,
        "reported_planar_speed_within_threshold": (
            metrics.get("reported_planar_speed_max_mps", math.inf)
            <= arguments.max_stationary_linear_speed_mps
        ),
        "reported_yaw_rate_within_threshold": (
            metrics.get("reported_abs_yaw_rate_max_rad_per_second", math.inf)
            <= arguments.max_stationary_yaw_rate_rad_per_second
        ),
    }
    stationary_observed = all(stationary_checks.values())
    return {
        "schema": "robonix-go2-canonical-odom-stationary-readonly-v1",
        "schema_version": 1,
        "mode": "read-only-subscription-only",
        "hostname": socket.gethostname(),
        "pid": os.getpid(),
        "topic": TOPIC,
        "message_type": MESSAGE_TYPE,
        "expected_frame_id": EXPECTED_FRAME_ID,
        "expected_child_frame_id": EXPECTED_CHILD_FRAME_ID,
        "requested_duration_seconds": arguments.duration_seconds,
        "gap_threshold_seconds": arguments.gap_threshold_seconds,
        "max_source_age_seconds": arguments.max_source_age_seconds,
        "max_future_skew_seconds": arguments.max_future_skew_seconds,
        "max_stationary_linear_speed_mps": (
            arguments.max_stationary_linear_speed_mps
        ),
        "max_stationary_yaw_rate_rad_per_second": (
            arguments.max_stationary_yaw_rate_rad_per_second
        ),
        "internal_monotonic_deadline_enforced": True,
        "outer_process_timeout_enforced_by_script": False,
        "qos": {
            "history": "keep_last",
            "depth": 10,
            "reliability": "best_effort",
            "durability": "volatile",
        },
        "ros_log_dir": str(ros_log_dir),
        "ros_subscriptions_created": subscriptions_created,
        "ros_publishers_created": False,
        "services_or_action_clients_created": False,
        "unitree_clients_created": False,
        "motion_authorized": False,
        "started_realtime_ns": started_realtime_ns,
        "started_utc": _iso_utc(started_realtime_ns),
        "finished_realtime_ns": finished_realtime_ns,
        "finished_utc": _iso_utc(finished_realtime_ns),
        "observed_wall_duration_seconds": (
            evidence_finished_monotonic_ns
            - (
                observation_started_monotonic_ns
                if observation_started_monotonic_ns is not None
                else started_monotonic_ns
            )
        )
        / 1e9,
        "observation_started_monotonic_ns": observation_started_monotonic_ns,
        "observation_finished_monotonic_ns": observation_finished_monotonic_ns,
        "exit_reason": exit_reason,
        "runtime_error": runtime_error,
        "cleanup_errors": list(cleanup_errors),
        "checks": checks,
        "observer_valid": observer_valid,
        "stationary_checks": stationary_checks,
        "stationary_observed": stationary_observed,
        "metrics": metrics,
        "interpretation": (
            "Canonical odometry transport and reported-velocity evidence only. "
            "stationary_observed uses reported twist thresholds and still "
            "requires operator confirmation that the chassis did not move. "
            "Drift values never authorize motion."
        ),
    }


def run_observer(arguments: argparse.Namespace) -> int:
    _validate_observation_window(arguments)
    os.umask(0o077)
    output_dir = _prepare_output(arguments.output_dir)
    ros_log_dir = _prepare_ros_log_dir(output_dir)
    tracker = CanonicalOdomTracker(
        arguments.gap_threshold_seconds,
        arguments.max_source_age_seconds,
        arguments.max_future_skew_seconds,
    )
    started_realtime_ns = time.time_ns()
    started_monotonic_ns = time.perf_counter_ns()
    runtime_error: str | None = None
    cleanup_errors: list[str] = []
    subscriptions_created = 0
    observation_started_monotonic_ns: int | None = None
    observation_finished_monotonic_ns: int | None = None
    exit_reason = "initialization_failed"
    stop_request = TerminationRequest()
    previous_sigint_handler = signal.signal(signal.SIGINT, stop_request.handler)
    previous_sigterm_handler = signal.signal(signal.SIGTERM, stop_request.handler)
    ros_import_failed = False

    try:
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.impl.implementation_singleton import (
            rclpy_implementation as _rclpy,
        )
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from rclpy.type_support import check_is_valid_msg_type
    except ImportError as error:
        ros_import_failed = True
        runtime_error = f"missing ROS 2 dependency: {error}"
    context = None
    context_initialized = False
    node_handle = None
    subscription_handle = None
    wait_set = None
    if not ros_import_failed:
        try:
            context = Context()
            context.init(args=[], initialize_logging=False)
            context_initialized = True
            check_is_valid_msg_type(Odometry)
            # Humble's high-level Node always creates a /parameter_events
            # publisher and normally creates parameter services.  Use the
            # installed low-level handles deliberately so this process owns
            # one subscription and no other ROS communication endpoint.
            with context.handle:
                node_handle = _rclpy.Node(
                    "go2_canonical_odom_stationary_observer_readonly",
                    "",
                    context.handle,
                    None,
                    False,
                    False,
                )
                wait_set = _rclpy.WaitSet(1, 0, 0, 0, 0, 0, context.handle)
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            with node_handle:
                subscription_handle = _rclpy.Subscription(
                    node_handle,
                    Odometry,
                    TOPIC,
                    qos.get_c_qos_profile(),
                )
            subscriptions_created = 1
            observation_started_monotonic_ns = time.perf_counter_ns()
            deadline_ns = observation_started_monotonic_ns + (
                arguments.duration_seconds * 1_000_000_000
            )
            while context.ok() and time.perf_counter_ns() < deadline_ns:
                if stop_request.requested:
                    break
                remaining_ns = max(0, deadline_ns - time.perf_counter_ns())
                wait_set.clear_entities()
                wait_set.add_subscription(subscription_handle)
                wait_set.wait(min(100_000_000, remaining_ns))
                ready = wait_set.get_ready_entities("subscription")
                if subscription_handle.pointer not in ready:
                    continue
                message_info = subscription_handle.take_message(Odometry, False)
                if message_info is None:
                    continue
                receipt_realtime_ns = time.time_ns()
                receipt_monotonic_ns = time.perf_counter_ns()
                tracker.observe(
                    message_info[0],
                    receipt_monotonic_ns,
                    receipt_realtime_ns,
                )
            if stop_request.requested:
                exit_reason = stop_request.reason
            elif time.perf_counter_ns() >= deadline_ns:
                exit_reason = "duration_elapsed"
            else:
                exit_reason = "rclpy_shutdown"
        except KeyboardInterrupt:
            exit_reason = "signal_sigint"
        except Exception as error:  # pragma: no cover - ROS runtime only
            runtime_error = f"{type(error).__name__}: {error}"
            exit_reason = "runtime_error"
        finally:
            if observation_started_monotonic_ns is not None:
                # End freshness belongs to the bounded observation window.
                # ROS entity/context destruction may take hundreds of
                # milliseconds and must not be misreported as sensor outage.
                observation_finished_monotonic_ns = time.perf_counter_ns()
            if wait_set is not None:
                try:
                    wait_set.clear_entities()
                    wait_set.destroy_when_not_in_use()
                except Exception as error:  # pragma: no cover - ROS runtime only
                    cleanup_errors.append(
                        f"wait_set: {type(error).__name__}: {error}"
                    )
            if subscription_handle is not None:
                try:
                    subscription_handle.destroy_when_not_in_use()
                except Exception as error:  # pragma: no cover - ROS runtime only
                    cleanup_errors.append(
                        f"subscription: {type(error).__name__}: {error}"
                    )
            if node_handle is not None:
                try:
                    node_handle.destroy_when_not_in_use()
                except Exception as error:  # pragma: no cover - ROS runtime only
                    cleanup_errors.append(
                        f"node: {type(error).__name__}: {error}"
                    )
            if context is not None and context_initialized:
                try:
                    context.try_shutdown()
                    context.destroy()
                except Exception as error:  # pragma: no cover - ROS runtime only
                    cleanup_errors.append(
                        f"context: {type(error).__name__}: {error}"
                    )

    try:
        finished_realtime_ns = time.time_ns()
        finished_monotonic_ns = time.perf_counter_ns()
        result = _build_result(
            arguments=arguments,
            tracker=tracker,
            ros_log_dir=ros_log_dir,
            started_realtime_ns=started_realtime_ns,
            started_monotonic_ns=started_monotonic_ns,
            finished_realtime_ns=finished_realtime_ns,
            finished_monotonic_ns=finished_monotonic_ns,
            exit_reason=exit_reason,
            runtime_error=runtime_error,
            cleanup_errors=cleanup_errors,
            subscriptions_created=subscriptions_created,
            observation_started_monotonic_ns=observation_started_monotonic_ns,
            observation_finished_monotonic_ns=observation_finished_monotonic_ns,
        )
        _write_json(output_dir / "summary.json", result)
        if runtime_error is not None:
            print(runtime_error, file=sys.stderr)
        else:
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    finally:
        signal.signal(signal.SIGINT, previous_sigint_handler)
        signal.signal(signal.SIGTERM, previous_sigterm_handler)

    if ros_import_failed:
        return 127
    return 0 if result["observer_valid"] else 2


def main() -> int:
    arguments = build_argument_parser().parse_args()
    try:
        return run_observer(arguments)
    except (OSError, RuntimeError) as error:
        print(f"canonical odom observer failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
