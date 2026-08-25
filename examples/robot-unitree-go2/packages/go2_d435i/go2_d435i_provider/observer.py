"""Bounded, subscription-only D435i quality observation.

ROS 2 Humble's high-level Node unconditionally creates a parameter-events
publisher. This module therefore constructs only the low-level node,
subscription, and wait-set handles required to receive the three external
camera streams.
"""

from __future__ import annotations

import time
from typing import Any, Mapping

from .quality import NANOSECONDS_PER_SECOND, QualityResult, QualityTracker


def _failed(detail: str) -> QualityResult:
    return QualityResult(
        False,
        detail,
        {
            "ros_publishers_created": False,
            "source_mode": "external",
            "problems": [detail],
        },
    )


def observe_external_d435i(config: Mapping[str, Any]) -> QualityResult:
    """Observe the configured topics without creating any ROS publisher."""

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
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as error:
        return _failed(f"D435i quality observer dependency unavailable: {error}")

    tracker = QualityTracker(config)
    context = None
    node_handle = None
    wait_set = None
    subscriptions: list[Any] = []
    quality_started_ns: int | None = None
    finished_ns = time.perf_counter_ns()
    runtime_error: str | None = None

    topic_specs = (
        ("rgb", str(config["rgb_topic"]), Image),
        ("depth", str(config["depth_topic"]), Image),
        ("camera_info", str(config["camera_info_topic"]), CameraInfo),
    )
    subscription_roles: dict[int, tuple[str, Any]] = {}

    try:
        context = Context()
        context.init(args=[], initialize_logging=False)
        for _role, _topic, message_type in topic_specs:
            check_is_valid_msg_type(message_type)

        # Best-effort/volatile subscriptions can receive from either
        # best-effort or reliable live sensor publishers. Atlas registration
        # below remains responsible for advertising each reviewed endpoint's
        # intended consumer QoS.
        observation_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=8,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        with context.handle:
            node_handle = _rclpy.Node(
                "go2_d435i_external_quality_observer",
                "",
                context.handle,
                None,
                False,
                False,
            )
            wait_set = _rclpy.WaitSet(3, 0, 0, 0, 0, 0, context.handle)
        with node_handle:
            for role, topic, message_type in topic_specs:
                subscription = _rclpy.Subscription(
                    node_handle,
                    message_type,
                    topic,
                    observation_qos.get_c_qos_profile(),
                )
                subscriptions.append(subscription)
                subscription_roles[subscription.pointer] = (role, message_type)

        observation_started_ns = time.perf_counter_ns()
        sentinel_deadline_ns = observation_started_ns + int(
            float(config["sentinel_timeout_s"]) * NANOSECONDS_PER_SECOND
        )
        quality_window_ns = int(
            float(config["quality_window_s"]) * NANOSECONDS_PER_SECOND
        )

        while context.ok():
            now_ns = time.perf_counter_ns()
            desired_finish_ns = (
                quality_started_ns + quality_window_ns
                if quality_started_ns is not None
                else sentinel_deadline_ns
            )
            deadline_ns = min(sentinel_deadline_ns, desired_finish_ns)
            if now_ns >= deadline_ns:
                finished_ns = now_ns
                break

            wait_set.clear_entities()
            for subscription in subscriptions:
                wait_set.add_subscription(subscription)
            wait_set.wait(min(100_000_000, deadline_ns - now_ns))
            ready = set(wait_set.get_ready_entities("subscription"))
            for subscription in subscriptions:
                if subscription.pointer not in ready:
                    continue
                role, message_type = subscription_roles[subscription.pointer]
                message_info = subscription.take_message(message_type, False)
                if message_info is None:
                    continue
                tracker.observe(
                    role,
                    message_info[0],
                    receipt_realtime_ns=time.time_ns(),
                    receipt_monotonic_ns=time.perf_counter_ns(),
                )
            if tracker.all_streams_seen and quality_started_ns is None:
                quality_started_ns = time.perf_counter_ns()
        else:
            finished_ns = time.perf_counter_ns()
            runtime_error = "ROS context shut down during D435i observation"
    except Exception as error:  # pragma: no cover - exercised on ROS hardware
        finished_ns = time.perf_counter_ns()
        runtime_error = f"{type(error).__name__}: {error}"
    finally:
        if wait_set is not None:
            try:
                wait_set.clear_entities()
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
        if context is not None:
            try:
                if context.ok():
                    context.shutdown()
            except Exception:
                pass

    if runtime_error is not None:
        return _failed(f"D435i quality observer failed: {runtime_error}")
    quality_duration_ns = (
        max(0, finished_ns - quality_started_ns)
        if quality_started_ns is not None
        else 0
    )
    return tracker.finalize(
        quality_duration_ns=quality_duration_ns,
        finished_monotonic_ns=finished_ns,
    )
