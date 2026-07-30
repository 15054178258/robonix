#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Publish one evaluation-only Webots object move and wait for its ack."""

from __future__ import annotations

import argparse
import json
import time
import uuid


def send_object_control(
    *,
    target_name: str,
    translation: list[float] | tuple[float, float, float],
    timeout_s: float = 10.0,
    topic: str = "/webots/evaluation/object_control",
    ack_topic: str = "/webots/evaluation/object_control/ack",
) -> dict:
    """Publish one object translation and return its matching ack."""
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("webots_evaluation_object_control")
    publisher = node.create_publisher(String, topic, 10)
    request_id = uuid.uuid4().hex
    response: dict | None = None

    def on_ack(message: String) -> None:
        nonlocal response
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get("request_id") == request_id:
            response = payload

    subscription = node.create_subscription(
        String,
        ack_topic,
        on_ack,
        10,
    )
    message = String()
    message.data = json.dumps(
        {
            "request_id": request_id,
            "target_name": target_name,
            "translation": list(translation),
        },
        sort_keys=True,
    )
    deadline = time.monotonic() + timeout_s
    next_publish = 0.0
    try:
        while response is None and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                publisher.publish(message)
                next_publish = now + 0.5
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if response is None:
        raise RuntimeError(
            f"no object-control ack arrived within {timeout_s:.1f}s"
        )
    return response


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-name", required=True)
    parser.add_argument(
        "--translation",
        required=True,
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--topic",
        default="/webots/evaluation/object_control",
    )
    parser.add_argument(
        "--ack-topic",
        default="/webots/evaluation/object_control/ack",
    )
    args = parser.parse_args()
    response = send_object_control(
        target_name=args.target_name,
        translation=args.translation,
        timeout_s=args.timeout_s,
        topic=args.topic,
        ack_topic=args.ack_topic,
    )
    print(json.dumps(response, indent=2, sort_keys=True))
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
