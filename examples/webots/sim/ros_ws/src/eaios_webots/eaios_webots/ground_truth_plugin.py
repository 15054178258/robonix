"""Publish simulator-only Tiago ground truth for Webots evaluation.

The topic is deliberately outside the Robonix capability graph and must never
be consumed by Mapping, Navigation, Scene, or robot runtime code.
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from typing import Any, NamedTuple


class ObjectControlCommand(NamedTuple):
    """One evaluation-only Webots object translation request."""

    request_id: str
    target_name: str
    translation: tuple[float, float, float]


def parse_object_control_command(payload: str) -> ObjectControlCommand:
    """Validate a JSON object-control request without touching Webots."""

    try:
        value = json.loads(str(payload))
    except json.JSONDecodeError as exc:
        raise ValueError("object-control payload must be valid JSON") from exc
    if not isinstance(value, dict):
        raise ValueError("object-control payload must be a JSON object")
    request_id = str(value.get("request_id") or "").strip()
    target_name = str(value.get("target_name") or "").strip()
    translation = value.get("translation")
    if not request_id or len(request_id) > 128:
        raise ValueError("request_id must contain 1 to 128 characters")
    if not target_name or len(target_name) > 128:
        raise ValueError("target_name must contain 1 to 128 characters")
    if (
        not isinstance(translation, list)
        or len(translation) != 3
        or isinstance(translation, (str, bytes))
    ):
        raise ValueError("translation must be a three-element JSON array")
    try:
        coordinates = tuple(float(item) for item in translation)
    except (TypeError, ValueError) as exc:
        raise ValueError("translation values must be numeric") from exc
    if not all(math.isfinite(item) for item in coordinates):
        raise ValueError("translation values must be finite")
    return ObjectControlCommand(
        request_id=request_id,
        target_name=target_name,
        translation=coordinates,
    )


def find_root_node_by_name(supervisor: Any, target_name: str) -> Any | None:
    """Return a named root object from the active Webots world."""

    root = supervisor.getRoot()
    children = root.getField("children") if root is not None else None
    if children is None:
        return None
    for index in range(int(children.getCount())):
        node = children.getMFNode(index)
        if node is None:
            continue
        name_field = node.getField("name")
        if name_field is not None and name_field.getSFString() == target_name:
            return node
    return None


def split_sim_time(sim_time: float) -> tuple[int, int]:
    """Split Webots simulation time into normalized ROS seconds/nanoseconds."""

    if not math.isfinite(sim_time) or sim_time < 0.0:
        raise ValueError("simulation time must be finite and non-negative")
    seconds = int(sim_time)
    nanoseconds = int(round((sim_time - seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    return seconds, nanoseconds


def quaternion_from_rotation_matrix(
    matrix: Sequence[float],
) -> tuple[float, float, float, float]:
    """Return an ``(x, y, z, w)`` quaternion from a row-major 3x3 matrix."""

    if len(matrix) != 9:
        raise ValueError("rotation matrix must contain exactly nine values")
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = (
        float(value) for value in matrix
    )
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("rotation matrix produced a zero quaternion")
    return (x / norm, y / norm, z / norm, w / norm)


class GroundTruthPlugin:
    """Publish robot truth and expose an isolated evaluation control topic."""

    def init(self, webots_node, properties) -> None:
        from nav_msgs.msg import Odometry
        import rclpy

        self._rclpy = rclpy
        self._owns_rclpy_context = not rclpy.ok()
        if self._owns_rclpy_context:
            rclpy.init(args=None)
        self._ros_node = rclpy.create_node("webots_ground_truth")
        self._message_type = Odometry
        self._robot = webots_node.robot
        self._robot_node = self._robot.getSelf()
        if self._robot_node is None:
            raise RuntimeError(
                "ground-truth publisher requires a supervisor robot with getSelf()"
            )
        self._topic = properties.get("topicName", "/webots/ground_truth/odom")
        self._frame_id = properties.get("frameName", "webots_world")
        self._child_frame_id = properties.get("childFrameName", "base_link")
        self._publisher = self._ros_node.create_publisher(
            Odometry,
            self._topic,
            10,
        )
        from std_msgs.msg import String

        self._string_type = String
        self._object_control_topic = properties.get(
            "objectControlTopic",
            "/webots/evaluation/object_control",
        )
        self._object_control_ack_topic = properties.get(
            "objectControlAckTopic",
            "/webots/evaluation/object_control/ack",
        )
        self._pending_object_controls: list[ObjectControlCommand] = []
        self._object_control_subscription = self._ros_node.create_subscription(
            String,
            self._object_control_topic,
            self._on_object_control,
            10,
        )
        self._object_control_ack = self._ros_node.create_publisher(
            String,
            self._object_control_ack_topic,
            10,
        )
        self._ros_node.get_logger().info(
            f"Webots evaluation ground truth publishing on {self._topic}"
        )
        self._ros_node.get_logger().info(
            "Webots evaluation object control on "
            f"{self._object_control_topic}; ack "
            f"{self._object_control_ack_topic}"
        )

    def _publish_control_ack(self, payload: dict[str, Any]) -> None:
        message = self._string_type()
        message.data = json.dumps(payload, sort_keys=True)
        self._object_control_ack.publish(message)

    def _on_object_control(self, message) -> None:
        try:
            command = parse_object_control_command(message.data)
        except ValueError as exc:
            self._publish_control_ack(
                {
                    "ok": False,
                    "request_id": "",
                    "error": str(exc),
                }
            )
            return
        self._pending_object_controls.append(command)

    def _apply_pending_object_controls(self) -> None:
        while self._pending_object_controls:
            command = self._pending_object_controls.pop(0)
            node = find_root_node_by_name(self._robot, command.target_name)
            if node is None:
                self._publish_control_ack(
                    {
                        "ok": False,
                        "request_id": command.request_id,
                        "target_name": command.target_name,
                        "error": "named root object was not found",
                    }
                )
                continue
            translation_field = node.getField("translation")
            if translation_field is None:
                self._publish_control_ack(
                    {
                        "ok": False,
                        "request_id": command.request_id,
                        "target_name": command.target_name,
                        "error": "named object has no translation field",
                    }
                )
                continue
            previous = [
                float(item)
                for item in translation_field.getSFVec3f()
            ]
            translation_field.setSFVec3f(list(command.translation))
            node.resetPhysics()
            self._publish_control_ack(
                {
                    "ok": True,
                    "request_id": command.request_id,
                    "target_name": command.target_name,
                    "previous_translation": previous,
                    "translation": list(command.translation),
                }
            )

    def step(self) -> None:
        self._rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        self._apply_pending_object_controls()
        position = self._robot_node.getPosition()
        rotation = self._robot_node.getOrientation()
        qx, qy, qz, qw = quaternion_from_rotation_matrix(rotation)
        message = self._message_type()
        seconds, nanoseconds = split_sim_time(float(self._robot.getTime()))
        message.header.stamp.sec = seconds
        message.header.stamp.nanosec = nanoseconds
        message.header.frame_id = self._frame_id
        message.child_frame_id = self._child_frame_id
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        self._publisher.publish(message)

    def stop(self) -> None:
        if getattr(self, "_ros_node", None) is not None:
            self._ros_node.destroy_node()
            self._ros_node = None
        if (
            getattr(self, "_owns_rclpy_context", False)
            and getattr(self, "_rclpy", None) is not None
            and self._rclpy.ok()
        ):
            self._rclpy.shutdown()
