#!/usr/bin/env python3
"""Bounded, subscription-only SportModeState summary for service experiments.

ROS 2 Humble's high-level Node creates a parameter-events publisher even when
an application only asks for subscriptions.  This helper therefore owns only
low-level node, subscription, and wait-set handles.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import time
from typing import Any


ALLOWED_TOPICS = ("/sportmodestate", "/lf/sportmodestate")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=30)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def _stamp_ns(message: Any) -> int:
    return int(message.stamp.sec) * 1_000_000_000 + int(message.stamp.nanosec)


def _observe(
    topic: str,
    message: Any,
    metrics: dict[str, dict[str, Any]],
    counts: dict[str, Counter[tuple[int, int, int]]],
) -> None:
    item = metrics[topic]
    source_ns = _stamp_ns(message)
    previous_ns = item["last_source_stamp_ns"]
    item["received"] += 1
    if item["first_source_stamp_ns"] is None:
        item["first_source_stamp_ns"] = source_ns
    if previous_ns is not None and source_ns < previous_ns:
        item["source_regressions"] += 1
    item["last_source_stamp_ns"] = source_ns
    velocity = [float(value) for value in message.velocity]
    item["max_abs_linear_velocity"] = max(
        float(item["max_abs_linear_velocity"]),
        *(abs(value) for value in velocity),
    )
    item["max_abs_yaw_speed"] = max(
        float(item["max_abs_yaw_speed"]), abs(float(message.yaw_speed))
    )
    counts[topic][
        (int(message.error_code), int(message.mode), int(message.gait_type))
    ] += 1


def main() -> int:
    arguments = _parser().parse_args()
    if not 1 <= arguments.duration <= 120:
        raise SystemExit("--duration must be in 1..120 seconds")
    output = arguments.output.expanduser().resolve()
    if output.exists():
        raise SystemExit(f"refusing to overwrite: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

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
    from unitree_go.msg import SportModeState

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=32,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    counts: dict[str, Counter[tuple[int, int, int]]] = {
        topic: Counter() for topic in ALLOWED_TOPICS
    }
    metrics: dict[str, dict[str, Any]] = {
        topic: {
            "received": 0,
            "first_source_stamp_ns": None,
            "last_source_stamp_ns": None,
            "source_regressions": 0,
            "max_abs_linear_velocity": 0.0,
            "max_abs_yaw_speed": 0.0,
        }
        for topic in ALLOWED_TOPICS
    }

    started_realtime_ns = time.time_ns()
    started_monotonic_ns = time.monotonic_ns()
    context = None
    context_initialized = False
    node_handle = None
    wait_set = None
    subscriptions: list[Any] = []
    subscription_topics: dict[int, str] = {}
    try:
        context = Context()
        context.init(args=[], initialize_logging=False)
        context_initialized = True
        check_is_valid_msg_type(SportModeState)
        # The final two False values disable global ROS arguments and rosout.
        # Parameter services/events are a high-level Node feature and are not
        # constructed by these low-level handles.
        with context.handle:
            node_handle = _rclpy.Node(
                "go2_sport_state_readonly_observer",
                "",
                context.handle,
                None,
                False,
                False,
            )
            wait_set = _rclpy.WaitSet(2, 0, 0, 0, 0, 0, context.handle)
        with node_handle:
            for topic in ALLOWED_TOPICS:
                subscription = _rclpy.Subscription(
                    node_handle,
                    SportModeState,
                    topic,
                    qos.get_c_qos_profile(),
                )
                subscriptions.append(subscription)
                subscription_topics[subscription.pointer] = topic

        deadline = time.monotonic() + arguments.duration
        while context.ok():
            remaining_seconds = deadline - time.monotonic()
            if remaining_seconds <= 0:
                break
            wait_set.clear_entities()
            for subscription in subscriptions:
                wait_set.add_subscription(subscription)
            wait_set.wait(
                min(100_000_000, int(remaining_seconds * 1_000_000_000))
            )
            ready = set(wait_set.get_ready_entities("subscription"))
            for subscription in subscriptions:
                if subscription.pointer not in ready:
                    continue
                message_info = subscription.take_message(SportModeState, False)
                if message_info is None:
                    continue
                _observe(
                    subscription_topics[subscription.pointer],
                    message_info[0],
                    metrics,
                    counts,
                )
    finally:
        if wait_set is not None:
            try:
                wait_set.clear_entities()
            except Exception:
                pass
            try:
                wait_set.destroy_when_not_in_use()
            except Exception:
                pass
        for subscription in reversed(subscriptions):
            try:
                subscription.destroy_when_not_in_use()
            except Exception:
                pass
        if node_handle is not None:
            try:
                node_handle.destroy_when_not_in_use()
            except Exception:
                pass
        if context is not None and context_initialized:
            try:
                context.try_shutdown()
            except Exception:
                pass
            try:
                context.destroy()
            except Exception:
                pass
    finished_monotonic_ns = time.monotonic_ns()

    streams = []
    for topic in ALLOWED_TOPICS:
        item = dict(metrics[topic])
        item["topic"] = topic
        item["states"] = [
            {
                "error_code": error_code,
                "mode": mode,
                "gait_type": gait_type,
                "samples": samples,
            }
            for (error_code, mode, gait_type), samples in sorted(counts[topic].items())
        ]
        streams.append(item)
    payload = {
        "schema_version": 1,
        "mode": "read-only-subscriber-only",
        "duration_limit_s": arguments.duration,
        "started_realtime_ns": started_realtime_ns,
        "elapsed_monotonic_ns": finished_monotonic_ns - started_monotonic_ns,
        "publishers_created": False,
        "unitree_clients_created": False,
        "streams": streams,
    }
    output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    output.chmod(0o600)
    print(output)
    return 0 if any(stream["received"] for stream in streams) else 3


if __name__ == "__main__":
    raise SystemExit(main())
