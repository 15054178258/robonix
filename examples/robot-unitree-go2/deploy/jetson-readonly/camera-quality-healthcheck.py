#!/usr/bin/env python3
"""Read-only Docker health gate for the camera diagnostic stream."""

from __future__ import annotations

import os
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main() -> int:
    timeout_s = float(os.environ.get("GO2_CAMERA_HEALTH_TIMEOUT_S", "4.0"))
    if not 0.5 <= timeout_s <= 6.0:
        print("invalid camera health timeout", file=sys.stderr)
        return 2

    context = Context()
    node = None
    passed = False
    detail = "no go2_sensors/camera diagnostic received"

    def observe(message: DiagnosticArray) -> None:
        nonlocal passed, detail
        for status in message.status:
            if status.name != "go2_sensors/camera":
                continue
            values: dict[str, str] = {}
            for entry in status.values:
                if entry.key in values:
                    detail = "duplicate camera diagnostic key"
                    return
                values[entry.key] = entry.value
            ready = values.get("quality_ready") == "true"
            healthy = values.get("healthy") == "true"
            passed = (
                ready
                and healthy
                and status.level == DiagnosticStatus.OK
                and status.message == "camera quality gate passed"
            )
            detail = (
                f"level={status.level} quality_ready={ready} "
                f"healthy={healthy} detail={status.message}"
            )

    try:
        rclpy.init(args=[], context=context)
        node = Node(f"go2_camera_container_health_{os.getpid()}", context=context)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        subscription = node.create_subscription(
            DiagnosticArray, "/go2/sensors/status", observe, qos
        )
        deadline = time.monotonic() + timeout_s
        while not passed and time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.25, max(0.0, deadline - time.monotonic()))
            )
        del subscription
    except Exception as error:
        print(f"camera health observer failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 3
    finally:
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()

    if not passed:
        print(f"camera quality health gate failed: {detail}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
