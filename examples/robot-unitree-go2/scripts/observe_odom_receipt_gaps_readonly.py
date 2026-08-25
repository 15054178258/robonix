#!/usr/bin/env python3
"""Bounded, subscription-only receipt-gap observer for raw Go2 state streams.

Each callback intentionally does not inspect or copy its message payload.  It
only records a ``perf_counter_ns`` receipt time, keeping this independent witness
far lighter than the five-stream source timestamp evidence probe.  Odometry is
the compatible default; primary SportModeState and raw PointCloud2 observation
are independent opt-ins.
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

from receipt_gap_core import (  # noqa: E402
    DEFAULT_THRESHOLDS_NS,
    GapObservation,
    ReceiptGapTracker,
)


DEFAULT_TOPIC = "/utlidar/robot_odom"
DEFAULT_SPORT_PRIMARY_TOPIC = "/sportmodestate"
DEFAULT_CLOUD_TOPIC = "/utlidar/cloud"
DEFAULT_CLOUD_QOS_RELIABILITY = "best_effort"
CLOUD_QOS_RELIABILITY_CHOICES = ("best_effort", "reliable")
ODOM_STREAM = "mid360_odom"
SPORT_PRIMARY_STREAM = "sport_primary"
CLOUD_STREAM = "mid360_cloud"


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
        if self.signal_number == signal.SIGTERM:
            return "signal_sigterm"
        if self.signal_number == signal.SIGINT:
            return "signal_sigint"
        return f"signal_{self.signal_number}"

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
        description=(
            "READ-ONLY raw receipt-gap observer; creates one Odometry "
            "subscription, optionally SportModeState and PointCloud2 "
            "subscriptions, and no publisher"
        )
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--duration-seconds",
        type=lambda value: _bounded_int(value, 1, 86_400, "duration-seconds"),
        default=900,
    )
    parser.add_argument(
        "--max-events",
        type=lambda value: _bounded_int(value, 1, 1_000_000, "max-events"),
        default=100_000,
        help="maximum threshold events written; exact counters continue after cap",
    )
    parser.add_argument(
        "--retained-gaps",
        type=lambda value: _bounded_int(value, 100, 1_000_000, "retained-gaps"),
        default=200_000,
        help="bounded rolling gap window used for percentile statistics",
    )
    parser.add_argument("--topic", type=_topic, default=DEFAULT_TOPIC)
    parser.add_argument(
        "--include-sport-primary",
        action="store_true",
        help=(
            "also subscribe read-only to /sportmodestate; disabled by default"
        ),
    )
    parser.add_argument(
        "--include-cloud",
        action="store_true",
        help="also subscribe read-only to /utlidar/cloud; disabled by default",
    )
    parser.add_argument(
        "--cloud-qos-reliability",
        choices=CLOUD_QOS_RELIABILITY_CHOICES,
        default=DEFAULT_CLOUD_QOS_RELIABILITY,
        help=(
            "QoS reliability for the optional /utlidar/cloud subscription "
            "only; defaults to best_effort"
        ),
    )
    return parser


def _prepare_output(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    logs_root = (ROOT / "logs").resolve()
    try:
        resolved.relative_to(logs_root)
    except ValueError as error:
        raise RuntimeError(f"output directory must be below {logs_root}") from error
    resolved.mkdir(mode=0o700, parents=True, exist_ok=True)
    if not resolved.is_dir():
        raise RuntimeError(f"output path is not a directory: {resolved}")
    for name in ("events.jsonl", "summary.json", "metadata.json"):
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


def _prepare_ros_log_dir(output_dir: Path) -> Path:
    configured = os.environ.get("ROS_LOG_DIR")
    resolved = (
        Path(configured).expanduser().resolve()
        if configured
        else (output_dir / "ros-logs").resolve()
    )
    workspace_root = ROOT.resolve()
    try:
        resolved.relative_to(workspace_root)
    except ValueError as error:
        raise RuntimeError(
            f"ROS_LOG_DIR must remain below workspace package {workspace_root}"
        ) from error
    resolved.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(resolved, 0o700)
    os.environ["ROS_LOG_DIR"] = str(resolved)
    return resolved


def run_observer(arguments: argparse.Namespace) -> int:
    output_dir = _prepare_output(arguments.output_dir)
    ros_log_dir = _prepare_ros_log_dir(output_dir)
    # Delayed ROS imports keep all offline tests DDS-free.
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from rclpy.signals import SignalHandlerOptions

        if arguments.include_sport_primary:
            from unitree_go.msg import SportModeState
        else:
            SportModeState = None
        if arguments.include_cloud:
            from sensor_msgs.msg import PointCloud2
        else:
            PointCloud2 = None
    except ImportError as error:
        print(f"missing ROS 2 receipt-gap dependency: {error}", file=sys.stderr)
        return 127

    os.umask(0o077)
    started_realtime_ns = time.time_ns()
    started_monotonic_ns = time.perf_counter_ns()
    streams = {
        ODOM_STREAM: {
            "topic": arguments.topic,
            "message_type": "nav_msgs/msg/Odometry",
        }
    }
    if arguments.include_sport_primary:
        streams[SPORT_PRIMARY_STREAM] = {
            "topic": DEFAULT_SPORT_PRIMARY_TOPIC,
            "message_type": "unitree_go/msg/SportModeState",
        }
    if arguments.include_cloud:
        streams[CLOUD_STREAM] = {
            "topic": DEFAULT_CLOUD_TOPIC,
            "message_type": "sensor_msgs/msg/PointCloud2",
        }
    trackers = {
        stream_name: ReceiptGapTracker(
            thresholds_ns=DEFAULT_THRESHOLDS_NS,
            retained_gap_limit=arguments.retained_gaps,
        )
        for stream_name in streams
    }
    metadata = {
        "schema_version": 1,
        "mode": "read-only-raw-receipt-gap-observer",
        "hostname": socket.gethostname(),
        "pid": os.getpid(),
        "topic": arguments.topic,
        "message_type": "nav_msgs/msg/Odometry",
        "stream_order": list(streams),
        "streams": streams,
        "subscriptions": [
            {"stream": stream_name, **stream_definition}
            for stream_name, stream_definition in streams.items()
        ],
        "sport_primary_included": arguments.include_sport_primary,
        "cloud_included": arguments.include_cloud,
        "cloud_qos_reliability": arguments.cloud_qos_reliability,
        "qos": {
            "history": "keep_last",
            "depth": 1,
            "reliability": "best_effort",
            "durability": "volatile",
        },
        "duration_seconds": arguments.duration_seconds,
        "max_event_records": arguments.max_events,
        "retained_gap_limit": arguments.retained_gaps,
        "ros_log_dir": str(ros_log_dir),
        "thresholds_ns": list(DEFAULT_THRESHOLDS_NS),
        "threshold_comparison": "strictly_greater_than",
        "started_realtime_ns": started_realtime_ns,
        "started_monotonic_ns": started_monotonic_ns,
        "ros_subscriptions_created": len(streams),
        "ros_publishers_created": False,
        "services_or_action_clients_created": False,
        "unitree_clients_created": False,
        "payload_copied_or_republished": False,
        "interpretation": (
            "Independent-process raw-topic receipt witnesses. They do not "
            "authorize motion or distinguish DDS loss from host-wide scheduling "
            "delay without comparison to the timestamp-discipline fault evidence."
        ),
    }
    _write_json(output_dir / "metadata.json", metadata)

    events_path = output_dir / "events.jsonl"
    events_stream = events_path.open("x", encoding="utf-8", buffering=1)
    os.chmod(events_path, 0o600)
    event_records_written = 0
    threshold_event_intervals = {stream_name: 0 for stream_name in streams}
    event_records_by_stream = {stream_name: 0 for stream_name in streams}

    class ReadOnlyOdomGapObserver(Node):
        def __init__(self) -> None:
            super().__init__("go2_odom_receipt_gap_observer_readonly")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            cloud_qos = None
            if arguments.include_cloud:
                cloud_reliability = (
                    ReliabilityPolicy.RELIABLE
                    if arguments.cloud_qos_reliability == "reliable"
                    else ReliabilityPolicy.BEST_EFFORT
                )
                cloud_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=cloud_reliability,
                    durability=DurabilityPolicy.VOLATILE,
                )

            def callback_for(stream_name: str) -> Callable[[Any], None]:
                def observe_receipt(_message: Any) -> None:
                    nonlocal event_records_written
                    observation: GapObservation | None = trackers[
                        stream_name
                    ].observe(time.perf_counter_ns())
                    if (
                        observation is None
                        or not observation.exceeded_thresholds_ns
                    ):
                        return
                    threshold_event_intervals[stream_name] += 1
                    if event_records_written >= arguments.max_events:
                        return
                    event = observation.as_event(
                        streams[stream_name]["topic"], time.time_ns()
                    )
                    event["stream"] = stream_name
                    events_stream.write(
                        json.dumps(
                            event,
                            ensure_ascii=False,
                            sort_keys=True,
                            separators=(",", ":"),
                        )
                        + "\n"
                    )
                    event_records_written += 1
                    event_records_by_stream[stream_name] += 1

                return observe_receipt

            # Retain at most three subscriptions explicitly.  There is
            # intentionally no publisher, service, action client, timer, or
            # Unitree SDK object.  Every callback ignores its payload.
            # Do not shadow rclpy.node.Node._subscriptions.  Humble owns that
            # private list and destroy_node() removes each endpoint from it.
            # Appending an endpoint to the same list a second time makes clean
            # shutdown fail with ``list.remove(x): x not in list``.
            self._readonly_subscriptions = [
                self.create_subscription(
                    Odometry,
                    arguments.topic,
                    callback_for(ODOM_STREAM),
                    qos,
                )
            ]
            if arguments.include_sport_primary:
                self._readonly_subscriptions.append(
                    self.create_subscription(
                        SportModeState,
                        DEFAULT_SPORT_PRIMARY_TOPIC,
                        callback_for(SPORT_PRIMARY_STREAM),
                        qos,
                    )
                )
            if arguments.include_cloud:
                self._readonly_subscriptions.append(
                    self.create_subscription(
                        PointCloud2,
                        DEFAULT_CLOUD_TOPIC,
                        callback_for(CLOUD_STREAM),
                        cloud_qos,
                    )
                )

    stop_request = TerminationRequest()
    previous_sigterm_handler = signal.signal(signal.SIGTERM, stop_request.handler)
    deadline_ns = (
        time.perf_counter_ns() + arguments.duration_seconds * 1_000_000_000
    )
    node = None
    initialized = False
    exit_reason = "unknown"
    runtime_error: str | None = None
    cleanup_errors: list[str] = []
    try:
        try:
            # The outer timeout sends SIGTERM and expects this observer to
            # finish its evidence summary before returning 128 + SIGTERM.
            # rclpy's default-context init otherwise replaces our handler,
            # shuts the context down first, and turns normal termination into
            # ExternalShutdownException plus a duplicate-shutdown error.
            rclpy.init(
                args=None,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            initialized = True
            node = ReadOnlyOdomGapObserver()
            while rclpy.ok() and time.perf_counter_ns() < deadline_ns:
                if stop_request.requested:
                    break
                remaining_ns = max(0, deadline_ns - time.perf_counter_ns())
                rclpy.spin_once(node, timeout_sec=min(0.1, remaining_ns / 1e9))
            if stop_request.requested:
                exit_reason = stop_request.reason
            elif time.perf_counter_ns() >= deadline_ns:
                exit_reason = "duration_elapsed"
            elif not rclpy.ok():
                exit_reason = "rclpy_shutdown"
        except KeyboardInterrupt:
            stop_request.handler(signal.SIGINT, None)
            exit_reason = stop_request.reason
        except Exception as error:  # pragma: no cover - ROS runtime only
            runtime_error = f"{type(error).__name__}: {error}"
            exit_reason = "runtime_error"
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception as error:  # pragma: no cover - ROS runtime only
                    cleanup_errors.append(
                        f"destroy_node: {type(error).__name__}: {error}"
                    )
            if initialized:
                try:
                    # No rclpy signal handlers were installed above; leave the
                    # process-owned handler in place until the outer finally
                    # restores the handler that preceded this observer.
                    rclpy.shutdown(uninstall_handlers=False)
                except Exception as error:  # pragma: no cover - ROS runtime only
                    cleanup_errors.append(
                        f"rclpy_shutdown: {type(error).__name__}: {error}"
                    )
            try:
                events_stream.flush()
                os.fsync(events_stream.fileno())
            except Exception as error:  # pragma: no cover - filesystem failure only
                cleanup_errors.append(
                    f"event_flush: {type(error).__name__}: {error}"
                )
            finally:
                events_stream.close()

        if stop_request.requested:
            exit_reason = stop_request.reason
        finished_realtime_ns = time.time_ns()
        finished_monotonic_ns = time.perf_counter_ns()
        stream_summaries = {
            stream_name: {
                **stream_definition,
                "startup_to_first_receipt_ns": (
                    None
                    if trackers[stream_name].first_receipt_monotonic_ns is None
                    else trackers[stream_name].first_receipt_monotonic_ns
                    - started_monotonic_ns
                ),
                "last_receipt_to_finish_ns": (
                    None
                    if trackers[stream_name].last_receipt_monotonic_ns is None
                    else finished_monotonic_ns
                    - trackers[stream_name].last_receipt_monotonic_ns
                ),
                "threshold_event_intervals": threshold_event_intervals[
                    stream_name
                ],
                "event_records_written": event_records_by_stream[stream_name],
                "event_records_omitted": threshold_event_intervals[stream_name]
                - event_records_by_stream[stream_name],
                "receipt_gaps": trackers[stream_name].summary(),
            }
            for stream_name, stream_definition in streams.items()
        }
        odom_summary = stream_summaries[ODOM_STREAM]
        summary = {
            "schema_version": 1,
            "mode": "read-only-raw-receipt-gap-observer",
            "topic": arguments.topic,
            "exit_reason": exit_reason,
            "started_realtime_ns": started_realtime_ns,
            "finished_realtime_ns": finished_realtime_ns,
            "elapsed_monotonic_ns": finished_monotonic_ns
            - started_monotonic_ns,
            "configured_duration_ns": arguments.duration_seconds
            * 1_000_000_000,
            # Preserve the original odometry-only fields as aliases so
            # existing evidence readers remain compatible.
            "startup_to_first_receipt_ns": odom_summary[
                "startup_to_first_receipt_ns"
            ],
            "last_receipt_to_finish_ns": odom_summary[
                "last_receipt_to_finish_ns"
            ],
            "threshold_event_intervals": odom_summary[
                "threshold_event_intervals"
            ],
            "event_records_written": odom_summary["event_records_written"],
            "event_records_omitted": odom_summary["event_records_omitted"],
            "receipt_gaps": odom_summary["receipt_gaps"],
            "stream_order": list(streams),
            "streams": stream_summaries,
            "total_threshold_event_intervals": sum(
                threshold_event_intervals.values()
            ),
            "total_event_records_written": event_records_written,
            "total_event_records_omitted": sum(
                threshold_event_intervals.values()
            )
            - event_records_written,
            "runtime_error": runtime_error,
            "cleanup_errors": cleanup_errors,
            "safe_for_motion": False,
            "note": (
                "This observer is diagnostic evidence only. A successful run "
                "does not authorize physical motion."
            ),
        }
        _write_json(output_dir / "summary.json", summary)
        print(f"READ-ONLY raw receipt-gap evidence saved to {output_dir}")
        if stop_request.requested:
            return stop_request.exit_status
        return 1 if runtime_error or cleanup_errors else 0
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm_handler)


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    return run_observer(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
