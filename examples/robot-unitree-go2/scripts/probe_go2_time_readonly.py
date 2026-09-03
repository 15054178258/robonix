#!/usr/bin/env python3
"""Bounded, no-adjust timestamp probe for the physical Go2 data plane.

The process creates exactly five low-level ROS subscriptions.  It deliberately
avoids the high-level rclpy Node because that class also creates parameter ROS
endpoints.  It neither publishes ROS data nor opens a Unitree client.  Source
headers are compared with clocks sampled at the receive boundary and are
written unchanged to private evidence files.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time
from typing import Any, Callable


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC_DIR = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC_DIR))

from go2_time_core import (  # noqa: E402
    Observation,
    StreamTracker,
    pairwise_median_offsets,
    read_clock_pair,
)


DEFAULT_TOPICS = {
    "sport_primary": "/sportmodestate",
    "sport_fallback": "/lf/sportmodestate",
    "mid360_cloud": "/utlidar/cloud",
    "mid360_imu": "/utlidar/imu",
    "mid360_odom": "/utlidar/robot_odom",
}
QUALIFICATION_STREAMS = (
    "sport_primary",
    "mid360_imu",
    "mid360_cloud",
    "mid360_odom",
)
WITNESS_STREAMS = ("sport_fallback",)


class TerminationRequest:
    """Record process-stop signals without terminating before evidence closes."""

    def __init__(self) -> None:
        self.signal_number: int | None = None

    def handler(self, signal_number: int, _frame: Any) -> None:
        # Signal handlers should do the minimum possible work.  The bounded
        # spin loop observes this flag, closes the JSONL stream, and writes the
        # summary from ordinary Python control flow.
        if self.signal_number is None:
            self.signal_number = int(signal_number)

    @property
    def requested(self) -> bool:
        return self.signal_number is not None

    @property
    def exit_reason(self) -> str | None:
        if self.signal_number == signal.SIGTERM:
            return "signal_sigterm"
        if self.signal_number == signal.SIGINT:
            return "signal_sigint"
        if self.signal_number is not None:
            return f"signal_{self.signal_number}"
        return None

    @property
    def exit_status(self) -> int:
        return 128 + self.signal_number if self.signal_number is not None else 0


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


def _topic(value: str) -> str:
    if not value.startswith("/") or value == "/" or "//" in value:
        raise argparse.ArgumentTypeError("topic names must be absolute and non-empty")
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_/")
    if any(character not in allowed for character in value):
        raise argparse.ArgumentTypeError("topic contains an unsupported character")
    return value


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="READ-ONLY Go2 source timestamp probe; never adjusts a clock"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--duration-seconds",
        type=lambda value: _bounded_int(value, 1, 86_400, "duration-seconds"),
        default=75,
    )
    parser.add_argument(
        "--max-samples",
        type=lambda value: _bounded_int(value, 1, 10_000_000, "max-samples"),
        default=200_000,
    )
    parser.add_argument(
        "--retained-offsets-per-stream",
        type=lambda value: _bounded_int(
            value, 100, 500_000, "retained-offsets-per-stream"
        ),
        default=200_000,
    )
    parser.add_argument("--primary-topic", type=_topic, default=DEFAULT_TOPICS["sport_primary"])
    parser.add_argument("--fallback-topic", type=_topic, default=DEFAULT_TOPICS["sport_fallback"])
    parser.add_argument("--cloud-topic", type=_topic, default=DEFAULT_TOPICS["mid360_cloud"])
    parser.add_argument("--imu-topic", type=_topic, default=DEFAULT_TOPICS["mid360_imu"])
    parser.add_argument("--odom-topic", type=_topic, default=DEFAULT_TOPICS["mid360_odom"])
    return parser


def _prepare_output(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    resolved.mkdir(mode=0o700, parents=True, exist_ok=True)
    if not resolved.is_dir():
        raise RuntimeError(f"output path is not a directory: {resolved}")
    for name in ("samples.jsonl", "summary.json", "metadata.json"):
        candidate = resolved / name
        if candidate.exists():
            raise RuntimeError(f"refusing to overwrite existing evidence: {candidate}")
    return resolved


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def _stamp_from_sport(message: Any) -> tuple[int, int]:
    return int(message.stamp.sec), int(message.stamp.nanosec)


def _stamp_from_header(message: Any) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


def run_probe(arguments: argparse.Namespace) -> int:
    # ROS imports are deliberately delayed.  Pure timestamp tests therefore do
    # not initialize DDS and can run on a machine with no ROS installation.
    try:
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
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        from unitree_go.msg import SportModeState
    except ImportError as error:
        print(f"missing ROS 2 or Unitree message dependency: {error}", file=sys.stderr)
        return 127

    output_dir = _prepare_output(arguments.output_dir)
    sample_path = output_dir / "samples.jsonl"
    os.umask(0o077)
    topics = {
        "sport_primary": arguments.primary_topic,
        "sport_fallback": arguments.fallback_topic,
        "mid360_cloud": arguments.cloud_topic,
        "mid360_imu": arguments.imu_topic,
        "mid360_odom": arguments.odom_topic,
    }
    trackers = {
        name: StreamTracker(
            name,
            topic,
            retained_offset_limit=arguments.retained_offsets_per_stream,
        )
        for name, topic in topics.items()
    }

    started_realtime_ns, started_monotonic_ns, started_clock_span_ns = read_clock_pair()
    metadata = {
        "schema_version": 1,
        "mode": "read-only-no-adjust",
        "clock_adjustment_requested": False,
        "ros_publishers_created": False,
        "unitree_clients_created": False,
        "hostname": socket.gethostname(),
        "pid": os.getpid(),
        "started_realtime_ns": started_realtime_ns,
        "started_monotonic_ns": started_monotonic_ns,
        "started_clock_read_span_ns": started_clock_span_ns,
        "duration_seconds": arguments.duration_seconds,
        "max_samples": arguments.max_samples,
        "retained_offsets_per_stream": arguments.retained_offsets_per_stream,
        "topics": topics,
        "stream_order": list(topics),
        "qualification_streams": list(QUALIFICATION_STREAMS),
        "witness_streams": list(WITNESS_STREAMS),
        "interpretation": (
            "source_minus_receipt is measurement evidence only; it is never "
            "used to replace a ROS header timestamp"
        ),
    }
    _write_json(output_dir / "metadata.json", metadata)

    total_samples = 0
    sample_stream = sample_path.open("x", encoding="utf-8", buffering=1)
    os.chmod(sample_path, 0o600)

    topic_specs: tuple[
        tuple[str, str, Any, Callable[[Any], tuple[int, int]]], ...
    ] = (
        (
            "sport_primary",
            topics["sport_primary"],
            SportModeState,
            _stamp_from_sport,
        ),
        (
            "sport_fallback",
            topics["sport_fallback"],
            SportModeState,
            _stamp_from_sport,
        ),
        (
            "mid360_cloud",
            topics["mid360_cloud"],
            PointCloud2,
            _stamp_from_header,
        ),
        ("mid360_imu", topics["mid360_imu"], Imu, _stamp_from_header),
        ("mid360_odom", topics["mid360_odom"], Odometry, _stamp_from_header),
    )

    stop_request = TerminationRequest()
    previous_sigterm_handler = signal.signal(signal.SIGTERM, stop_request.handler)

    def execute_probe() -> int:
        nonlocal total_samples
        context = None
        context_initialized = False
        node_handle = None
        wait_set = None
        subscriptions: list[Any] = []
        subscription_specs: dict[
            int, tuple[str, Any, Callable[[Any], tuple[int, int]]]
        ] = {}
        # Duration is measured from immediately before ROS initialization to
        # the monotonic deadline.  Summary elapsed time keeps the earlier
        # pre-metadata clock sample and is therefore intentionally a little
        # longer; both definitions are explicit evidence, not wall-clock
        # guesses.
        deadline_ns = time.monotonic_ns() + arguments.duration_seconds * 1_000_000_000
        exit_reason = "unknown"
        cleanup_errors: list[str] = []
        try:
            context = Context()
            context.init(args=[], initialize_logging=False)
            context_initialized = True
            for _stream_name, _topic_name, message_type, _stamp_getter in topic_specs:
                check_is_valid_msg_type(message_type)

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            # The high-level Node unconditionally owns parameter endpoints on
            # Humble.  Construct only one low-level node, five subscriptions,
            # and one wait set; disable rosout and global arguments.
            with context.handle:
                node_handle = _rclpy.Node(
                    "go2_time_probe_readonly",
                    "",
                    context.handle,
                    None,
                    False,
                    False,
                )
                wait_set = _rclpy.WaitSet(5, 0, 0, 0, 0, 0, context.handle)
            with node_handle:
                for stream_name, topic_name, message_type, stamp_getter in topic_specs:
                    subscription = _rclpy.Subscription(
                        node_handle,
                        message_type,
                        topic_name,
                        qos.get_c_qos_profile(),
                    )
                    subscriptions.append(subscription)
                    subscription_specs[subscription.pointer] = (
                        stream_name,
                        message_type,
                        stamp_getter,
                    )

            while context.ok() and time.monotonic_ns() < deadline_ns:
                if stop_request.requested:
                    break
                if total_samples >= arguments.max_samples:
                    break
                remaining_ns = max(0, deadline_ns - time.monotonic_ns())
                wait_set.clear_entities()
                for subscription in subscriptions:
                    wait_set.add_subscription(subscription)
                wait_set.wait(min(100_000_000, remaining_ns))
                ready = set(wait_set.get_ready_entities("subscription"))
                for subscription in subscriptions:
                    if stop_request.requested:
                        break
                    if total_samples >= arguments.max_samples:
                        break
                    if subscription.pointer not in ready:
                        continue
                    stream_name, message_type, stamp_getter = subscription_specs[
                        subscription.pointer
                    ]
                    message_info = subscription.take_message(message_type, False)
                    if message_info is None:
                        continue
                    receipt_realtime, receipt_monotonic, clock_span = read_clock_pair()
                    seconds, nanoseconds = stamp_getter(message_info[0])
                    observation: Observation = trackers[stream_name].observe(
                        seconds,
                        nanoseconds,
                        receipt_realtime,
                        receipt_monotonic,
                        clock_span,
                    )
                    sample_stream.write(
                        json.dumps(
                            observation.as_dict(),
                            ensure_ascii=False,
                            sort_keys=True,
                            separators=(",", ":"),
                        )
                        + "\n"
                    )
                    total_samples += 1
            if stop_request.requested:
                exit_reason = stop_request.exit_reason or "signal"
            elif total_samples >= arguments.max_samples:
                exit_reason = "max_samples_reached"
            elif time.monotonic_ns() >= deadline_ns:
                exit_reason = "duration_elapsed"
            elif not context.ok():
                exit_reason = "rclpy_shutdown"
        except KeyboardInterrupt:
            stop_request.handler(signal.SIGINT, None)
            exit_reason = stop_request.exit_reason or "signal_sigint"
        finally:
            if wait_set is not None:
                try:
                    wait_set.clear_entities()
                    wait_set.destroy_when_not_in_use()
                except Exception as error:  # pragma: no cover - ROS runtime only
                    cleanup_errors.append(
                        f"wait_set: {type(error).__name__}: {error}"
                    )
            for subscription in reversed(subscriptions):
                try:
                    subscription.destroy_when_not_in_use()
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
                sample_stream.flush()
                os.fsync(sample_stream.fileno())
            except Exception as error:  # pragma: no cover - filesystem failure only
                cleanup_errors.append(
                    f"sample_flush: {type(error).__name__}: {error}"
                )
            finally:
                sample_stream.close()

        # A SIGTERM can arrive during the final spin or cleanup.  Give the
        # signal reason precedence so an externally bounded run is never
        # mislabeled as a normal duration completion.
        if stop_request.requested:
            exit_reason = stop_request.exit_reason or "signal"

        finished_realtime_ns, finished_monotonic_ns, finished_clock_span_ns = (
            read_clock_pair()
        )
        summaries = [tracker.summary() for tracker in trackers.values()]
        unsafe_counts = sum(
            tracker.zero
            + tracker.malformed
            + tracker.duplicates
            + tracker.regressions
            for tracker in trackers.values()
        )
        summary = {
            "schema_version": 1,
            "mode": "read-only-no-adjust",
            "exit_reason": exit_reason,
            "total_samples": total_samples,
            "started_realtime_ns": started_realtime_ns,
            "finished_realtime_ns": finished_realtime_ns,
            "elapsed_monotonic_ns": finished_monotonic_ns - started_monotonic_ns,
            "configured_duration_ns": arguments.duration_seconds * 1_000_000_000,
            "finished_clock_read_span_ns": finished_clock_span_ns,
            "streams": summaries,
            "pairwise_median_offset_comparisons": pairwise_median_offsets(
                list(trackers.values())
            ),
            "timestamp_anomaly_count": unsafe_counts,
            "cleanup_errors": cleanup_errors,
            "safe_for_clock_discipline": False,
            "note": (
                "This probe never authorizes clock discipline. Publisher locator, "
                "cold-boot, long-duration drift, and operator review remain mandatory."
            ),
        }
        _write_json(output_dir / "summary.json", summary)
        print(f"READ-ONLY time evidence saved to {output_dir}")
        if stop_request.requested:
            return stop_request.exit_status
        return 1 if cleanup_errors else 0

    try:
        return execute_probe()
    finally:
        # This outer finally covers normal return, SIGTERM/SIGINT completion,
        # ROS exceptions, summary failures, and cleanup failures.
        signal.signal(signal.SIGTERM, previous_sigterm_handler)


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    return run_probe(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
