# SPDX-License-Identifier: MulanPSL-2.0
"""move_rbnx — simple movement skill (atlas bridge).

A user-invocable skill: accepts direction + distance/angle and executes
the movement by publishing directly to /cmd_vel.

Directions: forward, backward, left (strafe), right (strafe), rotate_cw,
rotate_ccw, stop.

Async mode: returns run_id immediately, caller polls status/cancel.
Duplicate detection: skips re-execution of same command within 15 seconds.

改进: 使用 odom 闭环控制，精确控制距离和角度。
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
import uuid
from collections import deque

from robonix_api import Skill, Ok, Err

log = logging.getLogger("move_skill")

move_skill = Skill(id="move", namespace="robonix/skill/move")

# ROS2 publisher for cmd_vel
_cmd_vel_pub = None
_ros_node = None
_ros_thread = None

# Odom 闭环控制
_latest_odom = None  # 最新的 odom 消息
_odom_sub = None  # odom 订阅者

# Task tracking
_tasks: dict[str, dict] = {}
_task_lock = threading.Lock()

# Duplicate detection — track all recent commands, not just the last one
_recent_commands: deque[tuple[tuple, float]] = deque(maxlen=20)
_DUPLICATE_WINDOW: float = 15.0  # seconds

# Default speeds
LINEAR_SPEED = 0.2   # m/s
ANGULAR_SPEED = 0.5  # rad/s
STRAFE_SPEED = 0.2   # m/s

# 闭环控制参数
DISTANCE_TOLERANCE = 0.02  # 距离容差 (米)
ANGLE_TOLERANCE = 0.03  # 角度容差 (弧度，约 1.7 度)
NO_PROGRESS_TIMEOUT = 5.0  # 无进展超时 (秒)
MOVE_LOOP_RATE = 0.02  # 控制循环周期 (秒)


# ── 闭环控制辅助函数 ─────────────────────────────────────────────────────────
def _odom_callback(msg):
    """存储最新的 odom 消息，用于闭环控制。"""
    global _latest_odom
    _latest_odom = msg


def _distance_between(x1, y1, x2, y2):
    """计算两个 2D 点之间的欧几里得距离。"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def _yaw_from_quaternion(q):
    """从四元数提取偏航角 (弧度)。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(a):
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


def _angle_diff(target, current):
    """计算从 current 到 target 的最短有符号角度差。"""
    return _normalize_angle(target - current)


def _wait_for_odom(timeout=5.0):
    """等待 odom 数据。返回 True 如果收到数据，False 如果超时。"""
    deadline = time.time() + timeout
    while _latest_odom is None and time.time() < deadline:
        time.sleep(0.05)
    return _latest_odom is not None


def _get_ros_node():
    """Get or create a ROS2 node."""
    global _ros_node, _cmd_vel_pub, _odom_sub
    if _ros_node is None:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        if not rclpy.ok():
            rclpy.init()

        _ros_node = Node("move_skill")
        _cmd_vel_pub = _ros_node.create_publisher(
            Twist, "/cmd_vel", qos_profile=10
        )

        # 订阅 odom 用于闭环控制
        odom_topic = os.getenv("MOVE_ODOM_TOPIC", "/odom")
        _odom_sub = _ros_node.create_subscription(
            Odometry, odom_topic, _odom_callback, qos_profile=10
        )
        log.info("ROS2 node created, publishing to /cmd_vel, subscribing to %s", odom_topic)
    return _ros_node


def _spin_ros():
    """Spin the ROS2 node in a background thread."""
    import rclpy
    node = _get_ros_node()
    rclpy.spin(node)


def _publish_twist(linear_x: float = 0.0, linear_y: float = 0.0,
                   angular_z: float = 0.0, duration: float = 0.0):
    """Publish a Twist message for a specified duration."""
    from geometry_msgs.msg import Twist

    node = _get_ros_node()
    if _cmd_vel_pub is None:
        raise RuntimeError("cmd_vel publisher not initialized")

    msg = Twist()
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = angular_z

    if duration <= 0:
        # Single publish
        _cmd_vel_pub.publish(msg)
        return

    # Publish for duration, then stop
    rate = node.create_rate(10)  # 10 Hz
    end_time = time.time() + duration
    while time.time() < end_time:
        _cmd_vel_pub.publish(msg)
        rate.sleep()

    # Stop
    _cmd_vel_pub.publish(Twist())


def _is_duplicate(direction: str, value: float, speed: float) -> bool:
    """Check if this command was seen recently (within _DUPLICATE_WINDOW).

    Tracks all recent commands in a deque, not just the last one, so
    duplicate detection survives interleaved different commands and
    pilot retry bursts spaced >1 s apart.
    """
    now = time.time()
    cmd = (direction, value, speed)

    with _task_lock:
        # Expire old entries
        while _recent_commands and now - _recent_commands[0][1] > _DUPLICATE_WINDOW:
            _recent_commands.popleft()

        # Check if this exact command was seen recently
        for recent_cmd, _ts in _recent_commands:
            if recent_cmd == cmd:
                log.info("duplicate command detected, skipping: %s", cmd)
                return True

        _recent_commands.append((cmd, now))
        return False


def _execute_move(task_id: str, direction: str, value: float, speed: float):
    """Execute a move command in a background thread.使用 odom 闭环控制。"""
    try:
        # Check for duplicate
        if _is_duplicate(direction, value, speed):
            with _task_lock:
                _tasks[task_id]["state"] = "SUCCEEDED"
                _tasks[task_id]["detail"] = "duplicate command, skipped"
            return

        with _task_lock:
            _tasks[task_id]["state"] = "RUNNING"

        if direction == "forward":
            s = speed if speed > 0 else LINEAR_SPEED
            _execute_drive_forward(task_id, value, s)
            result = f"moved forward {value}m at {s}m/s"

        elif direction == "backward":
            s = speed if speed > 0 else LINEAR_SPEED
            _execute_drive_forward(task_id, -abs(value), s)
            result = f"moved backward {value}m at {s}m/s"

        elif direction == "left":
            s = speed if speed > 0 else STRAFE_SPEED
            # 横移目前没有 odom 反馈，使用开环控制
            duration = value / s if s > 0 else 1.0
            _publish_twist(linear_y=s, duration=duration)
            result = f"strafed left {value}m at {s}m/s"

        elif direction == "right":
            s = speed if speed > 0 else STRAFE_SPEED
            # 横移目前没有 odom 反馈，使用开环控制
            duration = value / s if s > 0 else 1.0
            _publish_twist(linear_y=-s, duration=duration)
            result = f"strafed right {value}m at {s}m/s"

        elif direction == "rotate_cw":
            s = speed if speed > 0 else ANGULAR_SPEED
            _execute_rotate(task_id, -abs(value), s)
            result = f"rotated CW {value}deg at {s}rad/s"

        elif direction == "rotate_ccw":
            s = speed if speed > 0 else ANGULAR_SPEED
            _execute_rotate(task_id, abs(value), s)
            result = f"rotated CCW {value}deg at {s}rad/s"

        elif direction == "stop":
            _publish_twist()
            result = "stopped"

        else:
            raise ValueError(f"unknown direction: {direction}")

        with _task_lock:
            # 只有在没有被取消的情况下才更新状态
            if _tasks[task_id]["state"] == "RUNNING":
                _tasks[task_id]["state"] = "SUCCEEDED"
                _tasks[task_id]["detail"] = result
        log.info("task %s completed: %s", task_id, result)

    except Exception as e:
        with _task_lock:
            _tasks[task_id]["state"] = "FAILED"
            _tasks[task_id]["detail"] = str(e)
        log.error("task %s failed: %s", task_id, e)


def _execute_drive_forward(task_id: str, distance_m: float, speed_mps: float):
    """闭环控制：向前/向后行驶指定距离。"""
    from geometry_msgs.msg import Twist

    # 等待 odom 数据
    if not _wait_for_odom():
        log.warning("no odom data received, falling back to open loop")
        duration = abs(distance_m) / speed_mps
        _publish_twist(linear_x=speed_mps if distance_m > 0 else -speed_mps, duration=duration)
        return

    start_x = _latest_odom.pose.pose.position.x
    start_y = _latest_odom.pose.pose.position.y
    target_dist = abs(distance_m)
    linear_x = speed_mps if distance_m > 0 else -speed_mps

    log.info("Drive: start=(%.3f,%.3f), target=%.3f m", start_x, start_y, target_dist)

    last_progress_time = time.time()
    last_travelled = 0.0

    node = _get_ros_node()
    rate = node.create_rate(1.0 / MOVE_LOOP_RATE)

    while True:
        # 检查任务是否被取消
        with _task_lock:
            if _tasks[task_id]["state"] != "RUNNING":
                break

        cur_x = _latest_odom.pose.pose.position.x
        cur_y = _latest_odom.pose.pose.position.y
        travelled = _distance_between(start_x, start_y, cur_x, cur_y)

        # 到达目标距离
        if travelled >= target_dist - DISTANCE_TOLERANCE:
            break

        # 检查进度
        if travelled > last_travelled + 0.001:
            last_progress_time = time.time()
            last_travelled = travelled
        elif time.time() - last_progress_time > NO_PROGRESS_TIMEOUT:
            log.warning("no progress detected - possible obstacle")
            with _task_lock:
                _tasks[task_id]["state"] = "FAILED"
                _tasks[task_id]["detail"] = "no progress detected - possible obstacle"
            break

        # 发布速度命令
        tw = Twist()
        tw.linear.x = linear_x
        _cmd_vel_pub.publish(tw)
        rate.sleep()

    # 停止
    _publish_twist()


def _execute_rotate(task_id: str, angle_deg: float, ang_speed_rps: float):
    """闭环控制：原地旋转指定角度。"""
    from geometry_msgs.msg import Twist

    # 等待 odom 数据
    if not _wait_for_odom():
        log.warning("no odom data received, falling back to open loop")
        rad = math.radians(angle_deg)
        duration = abs(rad) / ang_speed_rps
        _publish_twist(angular_z=ang_speed_rps if angle_deg > 0 else -ang_speed_rps, duration=duration)
        return

    start_yaw = _yaw_from_quaternion(_latest_odom.pose.pose.orientation)
    target_yaw = start_yaw + math.radians(angle_deg)
    angular_speed = ang_speed_rps if angle_deg > 0 else -ang_speed_rps

    log.info("Turn: start_yaw=%.1f deg, target=%.1f deg", math.degrees(start_yaw), angle_deg)

    last_progress_time = time.time()
    last_yaw_diff = abs(_angle_diff(target_yaw, start_yaw))

    node = _get_ros_node()
    rate = node.create_rate(1.0 / MOVE_LOOP_RATE)

    while True:
        # 检查任务是否被取消
        with _task_lock:
            if _tasks[task_id]["state"] != "RUNNING":
                break

        cur_yaw = _yaw_from_quaternion(_latest_odom.pose.pose.orientation)
        remaining = abs(_angle_diff(target_yaw, cur_yaw))

        # 到达目标角度
        if remaining < ANGLE_TOLERANCE:
            break

        # 检查进度
        if remaining < last_yaw_diff - 0.01:
            last_progress_time = time.time()
            last_yaw_diff = remaining
        elif time.time() - last_progress_time > NO_PROGRESS_TIMEOUT:
            log.warning("no progress detected during turn")
            with _task_lock:
                _tasks[task_id]["state"] = "FAILED"
                _tasks[task_id]["detail"] = "no progress detected during turn"
            break

        # 发布速度命令
        tw = Twist()
        tw.angular.z = angular_speed
        _cmd_vel_pub.publish(tw)
        rate.sleep()

    # 停止
    _publish_twist()


# ── MCP tools ────────────────────────────────────────────────────────────────
from move_mcp import (  # noqa: E402
    Move_Request, Move_Response,
    GetMoveStatus_Request, GetMoveStatus_Response,
    CancelMove_Request, CancelMove_Response,
)


@move_skill.mcp("robonix/skill/move/move")
def move(req: Move_Request) -> Move_Response:
    """Execute a movement command with odom feedback (async).

    Directions:
      - forward: move forward (value = distance in metres)
      - backward: move backward (value = distance in metres)
      - rotate_ccw: turn left (value = angle in degrees)
      - rotate_cw: turn right (value = angle in degrees)
      - stop: stop immediately (value ignored)

    Examples:
      {"direction": "forward", "value": 1.0}
      {"direction": "rotate_ccw", "value": 90}
      {"direction": "backward", "value": 0.5, "speed": 0.1}

    Returns run_id; use status() to poll progress, cancel() to abort.
    """
    task_id = str(uuid.uuid4())[:8]

    with _task_lock:
        _tasks[task_id] = {
            "state": "PENDING",
            "detail": f"moving {req.direction}",
            "direction": req.direction,
            "value": req.value,
            "speed": req.speed,
        }

    # Start move in background thread
    thread = threading.Thread(
        target=_execute_move,
        args=(task_id, req.direction, req.value, req.speed),
        daemon=True,
    )
    thread.start()

    return Move_Response(
        accepted=True,
        run_id=task_id,
        message=f"move started: {req.direction} {req.value}",
    )


@move_skill.mcp("robonix/skill/move/move/status")
def status(req: GetMoveStatus_Request) -> GetMoveStatus_Response:
    """Poll progress of a running move task.

    Args:
      - run_id: task ID from move() response (empty = most recent task)

    Returns:
      - known: whether the task exists
      - state: PENDING, RUNNING, SUCCEEDED, FAILED, or CANCELED
      - detail: human-readable status message
    """
    run_id = req.run_id

    with _task_lock:
        if not run_id:
            # Get most recent task
            if not _tasks:
                return GetMoveStatus_Response(known=False, state="PENDING", detail="no tasks")
            run_id = max(_tasks.keys(), key=lambda k: _tasks[k].get("created", 0))

        task = _tasks.get(run_id)
        if task is None:
            return GetMoveStatus_Response(known=False, state="PENDING", detail="no such task")

        return GetMoveStatus_Response(
            known=True,
            state=task["state"],
            detail=task["detail"],
        )


@move_skill.mcp("robonix/skill/move/move/cancel")
def cancel(req: CancelMove_Request) -> CancelMove_Response:
    """Abort an active move command. Idempotent.

    Args:
      - run_id: task ID to cancel (empty = cancel most recent running task)

    Returns:
      - ok: whether cancellation succeeded
      - message: human-readable result
    """
    run_id = req.run_id

    with _task_lock:
        if not run_id:
            # Cancel most recent running task
            for tid, task in _tasks.items():
                if task["state"] == "RUNNING":
                    task["state"] = "CANCELED"
                    task["detail"] = "canceled by user"
                    # Stop the robot
                    _publish_twist()
                    return CancelMove_Response(ok=True, message=f"canceled task {tid}")
            return CancelMove_Response(ok=False, message="no running tasks")

        task = _tasks.get(run_id)
        if task is None:
            return CancelMove_Response(ok=False, message="no such task")

        if task["state"] in ("SUCCEEDED", "FAILED", "CANCELED"):
            return CancelMove_Response(ok=False, message=f"task already {task['state']}")

        task["state"] = "CANCELED"
        task["detail"] = "canceled by user"
        _publish_twist()  # Stop the robot
        return CancelMove_Response(ok=True, message=f"canceled task {run_id}")


# ── lifecycle ────────────────────────────────────────────────────────────────
@move_skill.on_init
def init(cfg):
    """CMD_INIT: light. Just log."""
    log.info("CMD_INIT ok")
    return Ok()


@move_skill.on_activate
def activate():
    """CMD_ACTIVATE: initialize ROS2 and start spinning."""
    global _ros_thread
    if _ros_thread is not None:
        log.info("CMD_ACTIVATE — already running, no-op")
        return Ok()

    try:
        _get_ros_node()
        _ros_thread = threading.Thread(target=_spin_ros, daemon=True)
        _ros_thread.start()
        log.info("CMD_ACTIVATE ok — ROS2 spinning")
        return Ok()
    except Exception as e:
        return Err(f"move skill activate failed: {e}")


@move_skill.on_deactivate
def deactivate():
    """CMD_DEACTIVATE: stop ROS2."""
    global _ros_node, _ros_thread, _cmd_vel_pub

    # Send stop command before shutting down
    if _cmd_vel_pub is not None:
        try:
            from geometry_msgs.msg import Twist
            _cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    if _ros_node is not None:
        try:
            _ros_node.destroy_node()
        except Exception:
            pass
        _ros_node = None
        _cmd_vel_pub = None

    _ros_thread = None
    log.info("CMD_DEACTIVATE ok")
    return Ok()


def main() -> int:
    import rclpy
    move_skill.run()
    if _ros_node is not None:
        _ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
