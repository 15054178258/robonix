#!/usr/bin/env python3
"""Explicit, movement-gated Ranger Mini terminal navigation acceptance."""

from __future__ import annotations

import argparse
import json
import math
import signal
import time
from dataclasses import asdict, dataclass
from pathlib import Path


def angle_error(target: float, actual: float) -> float:
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


@dataclass
class GoalSpec:
    name: str
    x: float
    y: float
    yaw: float


@dataclass
class TerminalMetrics:
    goal: GoalSpec
    xy_enter: float = 0.30
    entered: bool = False
    entered_at: float | None = None
    initial_yaw_error: float | None = None
    previous_yaw: float | None = None
    accumulated_rotation: float = 0.0
    max_xy_error: float = 0.0
    max_cmd_linear: float = 0.0
    max_cmd_angular: float = 0.0
    final_xy_error: float | None = None
    final_yaw_error: float | None = None
    final_linear_speed: float = 0.0
    final_angular_speed: float = 0.0

    def update_pose(self, x: float, y: float, yaw: float, now: float) -> None:
        xy_error = math.hypot(self.goal.x - x, self.goal.y - y)
        yaw_error = abs(angle_error(self.goal.yaw, yaw))
        if not self.entered and xy_error <= self.xy_enter:
            self.entered = True
            self.entered_at = now
            self.initial_yaw_error = yaw_error
            self.previous_yaw = yaw
        if self.entered:
            self.max_xy_error = max(self.max_xy_error, xy_error)
            if self.previous_yaw is not None:
                self.accumulated_rotation += abs(angle_error(yaw, self.previous_yaw))
            self.previous_yaw = yaw
        self.final_xy_error = xy_error
        self.final_yaw_error = yaw_error

    def update_cmd(self, linear: float, angular: float) -> None:
        if self.entered:
            self.max_cmd_linear = max(self.max_cmd_linear, abs(linear))
            self.max_cmd_angular = max(self.max_cmd_angular, abs(angular))

    def update_odom(self, linear: float, angular: float) -> None:
        self.final_linear_speed = abs(linear)
        self.final_angular_speed = abs(angular)

    def evaluate(self, succeeded: bool, finished_at: float) -> list[str]:
        failures: list[str] = []
        if not succeeded:
            failures.append("NavigateToPose did not succeed")
        if not self.entered or self.entered_at is None:
            failures.append("never entered the 0.30 m terminal window")
            return failures
        terminal_s = finished_at - self.entered_at
        if terminal_s > 15.0:
            failures.append(f"terminal duration {terminal_s:.2f}s > 15.0s")
        if self.max_xy_error > 0.45:
            failures.append(f"terminal XY drift {self.max_xy_error:.3f}m > 0.45m")
        if self.max_cmd_linear > 0.05:
            failures.append(f"terminal cmd linear {self.max_cmd_linear:.3f}m/s > 0.05")
        if self.max_cmd_angular > 0.31:
            failures.append(f"terminal cmd angular {self.max_cmd_angular:.3f}rad/s > 0.31")
        rotation_limit = (self.initial_yaw_error or 0.0) + 0.50
        if self.accumulated_rotation > rotation_limit:
            failures.append(
                f"terminal rotation {self.accumulated_rotation:.3f}rad > {rotation_limit:.3f}rad"
            )
        if self.final_xy_error is None or self.final_xy_error > 0.30:
            failures.append(f"final XY error {self.final_xy_error!r} > 0.30m")
        if self.final_yaw_error is None or self.final_yaw_error > 0.12:
            failures.append(f"final yaw error {self.final_yaw_error!r} > 0.12rad")
        if self.final_linear_speed > 0.05 or self.final_angular_speed > 0.05:
            failures.append(
                "robot not stopped: "
                f"linear={self.final_linear_speed:.3f}, angular={self.final_angular_speed:.3f}"
            )
        return failures


def load_plan(path: Path) -> tuple[list[GoalSpec], int]:
    import yaml

    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict) or not isinstance(raw.get("goals"), list):
        raise ValueError("plan must contain a goals list")
    goals = [
        GoalSpec(
            name=str(item["name"]),
            x=float(item["x"]),
            y=float(item["y"]),
            yaw=float(item["yaw"]),
        )
        for item in raw["goals"]
    ]
    if len(goals) < 5:
        raise ValueError("Ranger acceptance requires at least five goals")
    repetitions = int(raw.get("repetitions", 1))
    if repetitions < 1:
        raise ValueError("repetitions must be positive")
    return goals, repetitions


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("plan", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--i-understand-robot-will-move", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--goal-timeout-s", type=float, default=180.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    goals, repetitions = load_plan(args.plan)
    print(json.dumps({"goals": [asdict(g) for g in goals], "repetitions": repetitions}, indent=2))
    if not args.execute:
        print("DRY RUN: no navigation goal was sent")
        return 0
    if not args.i_understand_robot_will_move:
        raise SystemExit("--execute requires --i-understand-robot-will-move")

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    class Runner(Node):
        def __init__(self) -> None:
            super().__init__("ranger_navigation_acceptance")
            self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.metrics: TerminalMetrics | None = None
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, 20)

        def on_cmd(self, msg: Twist) -> None:
            if self.metrics:
                self.metrics.update_cmd(msg.linear.x, msg.angular.z)

        def on_odom(self, msg: Odometry) -> None:
            if self.metrics:
                self.metrics.update_odom(msg.twist.twist.linear.x, msg.twist.twist.angular.z)

        def sample_pose(self) -> None:
            if not self.metrics:
                return
            try:
                tf = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            except Exception:
                return
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
            self.metrics.update_pose(
                tf.transform.translation.x,
                tf.transform.translation.y,
                yaw,
                time.monotonic(),
            )

    rclpy.init()
    node = Runner()
    stop = False
    active_handle = None
    cancel_sent = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    reports: list[dict] = []
    try:
        if not node.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/navigate_to_pose action server unavailable")
        for repetition in range(repetitions):
            for goal in goals:
                if stop:
                    break
                metrics = TerminalMetrics(goal)
                node.metrics = metrics
                msg = NavigateToPose.Goal()
                msg.pose = PoseStamped()
                msg.pose.header.frame_id = "map"
                msg.pose.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.position.x = goal.x
                msg.pose.pose.position.y = goal.y
                msg.pose.pose.orientation.z = math.sin(goal.yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(goal.yaw / 2.0)
                started = time.monotonic()
                send_future = node.client.send_goal_async(msg)
                while rclpy.ok() and not send_future.done():
                    rclpy.spin_once(node, timeout_sec=0.05)
                    node.sample_pose()
                active_handle = send_future.result()
                if not active_handle.accepted:
                    raise RuntimeError(f"goal {goal.name!r} was rejected")
                if stop:
                    cancel_future = active_handle.cancel_goal_async()
                    cancel_sent = True
                    deadline = time.monotonic() + 2.0
                    while not cancel_future.done() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    break
                result_future = active_handle.get_result_async()
                timed_out = False
                cancel_requested_at = None
                while rclpy.ok() and not result_future.done():
                    rclpy.spin_once(node, timeout_sec=0.05)
                    node.sample_pose()
                    timed_out = time.monotonic() - started > args.goal_timeout_s
                    if (stop or timed_out) and not cancel_sent:
                        active_handle.cancel_goal_async()
                        cancel_sent = True
                        cancel_requested_at = time.monotonic()
                    if (
                        cancel_requested_at is not None
                        and time.monotonic() - cancel_requested_at > 5.0
                    ):
                        raise RuntimeError(
                            f"goal {goal.name!r} did not stop within 5s of cancel"
                        )
                finished = time.monotonic()
                result = result_future.result()
                succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.05)
                    node.sample_pose()
                failures = metrics.evaluate(succeeded, finished)
                if timed_out:
                    failures.insert(0, f"goal exceeded {args.goal_timeout_s:.1f}s timeout")
                report = {
                    "goal": goal.name,
                    "repetition": repetition + 1,
                    "status": int(result.status),
                    "duration_s": finished - started,
                    "metrics": asdict(metrics),
                    "failures": failures,
                    "passed": not failures,
                }
                reports.append(report)
                print(json.dumps(report, ensure_ascii=False))
                active_handle = None
                cancel_sent = False
                if failures and not args.continue_on_failure:
                    stop = True
                    break
            if stop:
                break
    finally:
        if active_handle is not None and not cancel_sent:
            cancel_future = active_handle.cancel_goal_async()
            deadline = time.monotonic() + 2.0
            while not cancel_future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()

    summary = {"passed": bool(reports) and all(r["passed"] for r in reports), "runs": reports}
    if args.output:
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["passed"] and len(reports) == len(goals) * repetitions else 1


if __name__ == "__main__":
    raise SystemExit(main())
