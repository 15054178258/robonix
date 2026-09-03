#!/usr/bin/env python3
"""Consume one exact stage-1 goal permit and send one NavigateToPose goal.

ROS imports and endpoint construction happen only after the private
``goal_dispatch`` permit has been validated and atomically renamed.  The
dispatcher never arms the chassis and never publishes velocity; the separate
staged motion guard remains the only path between Nav2 and the chassis.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import sys
import time
from typing import Mapping


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.runtime_config import (  # noqa: E402
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    STAGED_NAV2_COMMAND_TOPIC,
    STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
    STAGED_NAV2_PROFILE,
    STAGED_NAV2_STATE_TIMEOUT_S,
    normalize_config,
)
from go2_chassis.staged_nav2_permit import (  # noqa: E402
    ConsumedGoalPermit,
    GOAL_EVIDENCE_SHA256_ENV,
    GUARD_ACK_ENV,
    STAGED_NAV2_ACK,
    STAGE_ENV,
    PermitError,
    consume_staged_nav2_goal_permit,
)
from go2_chassis.staged_nav2_result import (  # noqa: E402
    ACTION_RESULT_SCHEMA,
    NAV_ACTION,
    ResultError,
    result_paths,
    write_private_json,
)


NODE_NAME = "go2_staged_nav2_goal_dispatch"
ACTION_NAME = "/navigate_to_pose"
CLAIM_TOPIC = "/robonix/staged_nav2/goal_claim"
CLAIM_SCHEMA = "robonix-go2-staged-nav2-goal-claim-v1"
GOAL_UUID_RE = re.compile(r"^[0-9a-f]{32}$")


def _runtime_from_environment(environ: Mapping[str, str]):
    exact = {
        "GO2_ALLOW_MOTION": "true",
        "GO2_MOTION_PROFILE": STAGED_NAV2_PROFILE,
        STAGE_ENV: "stage1",
        GUARD_ACK_ENV: STAGED_NAV2_ACK,
    }
    for name, expected in exact.items():
        if environ.get(name) != expected:
            raise PermitError(f"{name} must exactly equal {expected!r}")
    network_interface = environ.get("GO2_NETWORK_INTERFACE", "").strip()
    ipc_socket = environ.get("GO2_SDK_SOCKET", "").strip()
    if not network_interface or not ipc_socket:
        raise PermitError(
            "GO2_NETWORK_INTERFACE and GO2_SDK_SOCKET are required"
        )
    return normalize_config(
        {
            "allow_motion": True,
            "motion_profile": STAGED_NAV2_PROFILE,
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": network_interface,
            "ipc_socket": ipc_socket,
            "state_topic": (
                "/robonix/time_corrected/motion/sportmodestate"
            ),
            "state_fallback_topic": "",
            "twist_in_topic": STAGED_NAV2_COMMAND_TOPIC,
            "odom_source": "external_verified",
            "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
            "external_odom_timeout_s": STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
            "odom_topic": "/odom",
            "publish_odom_tf": True,
            "max_linear_x_mps": 0.30,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.40,
            "max_linear_accel_mps2": 0.30,
            "max_angular_accel_rps2": 0.80,
            "command_timeout_s": 0.20,
            "state_timeout_s": STAGED_NAV2_STATE_TIMEOUT_S,
            "max_source_stamp_age_s": 0.20,
            "max_source_stamp_future_skew_s": 0.05,
            "commissioning_max_duration_s": 0.0,
            "commissioning_max_distance_m": 0.0,
        },
        environ,
        ROOT / "packages" / "go2_chassis",
    )


def build_goal_claim(
    permit: ConsumedGoalPermit,
    goal_uuid: str,
    environ: Mapping[str, str],
) -> dict[str, object]:
    if GOAL_UUID_RE.fullmatch(goal_uuid) is None:
        raise PermitError("NavigateToPose goal UUID must be 16-byte lowercase hex")
    evidence_sha256 = environ.get(GOAL_EVIDENCE_SHA256_ENV, "").strip()
    if re.fullmatch(r"[0-9a-f]{64}", evidence_sha256) is None:
        raise PermitError("goal evidence claim must be lowercase SHA-256")
    return {
        "schema": CLAIM_SCHEMA,
        "session_id": permit.session_id,
        "pair_id": permit.pair_id,
        "source": permit.goal_source,
        "target_id": permit.target_id,
        "map_id": permit.map_id,
        "generation": permit.map_generation,
        "pose": {
            "x": permit.goal_x,
            "y": permit.goal_y,
            "yaw": permit.goal_yaw,
        },
        "goal_evidence_sha256": evidence_sha256,
        "goal_uuid": goal_uuid,
    }


def _run_ros(
    permit: ConsumedGoalPermit,
    environ: Mapping[str, str],
    action_result_path: Path,
) -> int:
    # Deliberately imported only after one-time permit consumption.
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class GoalDispatchNode(Node):
        def __init__(self) -> None:
            super().__init__(NODE_NAME)
            claim_qos = QoSProfile(depth=1)
            claim_qos.reliability = ReliabilityPolicy.RELIABLE
            claim_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.claim_publisher = self.create_publisher(
                String, CLAIM_TOPIC, claim_qos
            )
            self.action_client = ActionClient(
                self, NavigateToPose, ACTION_NAME
            )

    rclpy.init(args=None)
    node = GoalDispatchNode()
    goal_handle = None
    result: dict[str, object] = {
        "schema": ACTION_RESULT_SCHEMA,
        "status": "FAIL",
        "session_id": permit.session_id,
        "pair_id": permit.pair_id,
        "map_id": permit.map_id,
        "map_generation": permit.map_generation,
        "target_id": permit.target_id,
        "goal_uuid": "",
        "action": NAV_ACTION,
        "goal_accepted": False,
        "action_status_code": -1,
        "action_status_name": "not_started",
        "cancel_requested": False,
        "cancel_confirmed": False,
        "started_unix_ns": time.time_ns(),
        "finished_unix_ns": 0,
        "error": "",
    }
    try:
        if not node.action_client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error("NavigateToPose action server is unavailable")
            result["action_status_name"] = "server_unavailable"
            result["error"] = "NavigateToPose action server is unavailable"
            return 2
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = permit.goal_x
        goal.pose.pose.position.y = permit.goal_y
        goal.pose.pose.position.z = 0.0
        half_yaw = 0.5 * permit.goal_yaw
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)

        send_future = node.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            node.get_logger().error("NavigateToPose goal request timed out")
            result["action_status_name"] = "request_timeout"
            result["error"] = "NavigateToPose goal request timed out"
            return 3
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            node.get_logger().error("NavigateToPose goal was rejected")
            result["action_status_name"] = "rejected"
            result["error"] = "NavigateToPose goal was rejected"
            return 4

        goal_uuid = bytes(goal_handle.goal_id.uuid).hex()
        result["goal_uuid"] = goal_uuid
        result["goal_accepted"] = True
        claim = String()
        claim.data = json.dumps(
            build_goal_claim(permit, goal_uuid, environ),
            sort_keys=True,
            separators=(",", ":"),
        )
        # Exactly one transient-local claim is published. Re-publishing would
        # be interpreted by the guard as a second goal and fail closed.
        node.claim_publisher.publish(claim)

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            # Standard Nav2 owns goal duration.  Do not impose an additional
            # dispatcher deadline: planning, recovery and controller pauses
            # may legitimately exceed an arbitrary local wall-clock limit.
            rclpy.spin_once(node, timeout_sec=0.05)
        if not result_future.done() or result_future.result() is None:
            result["action_status_name"] = "result_absent"
            result["error"] = "NavigateToPose result was absent"
            return 6
        status = int(result_future.result().status)
        result["action_status_code"] = status
        status_names = {
            int(GoalStatus.STATUS_UNKNOWN): "unknown",
            int(GoalStatus.STATUS_ACCEPTED): "accepted",
            int(GoalStatus.STATUS_EXECUTING): "executing",
            int(GoalStatus.STATUS_CANCELING): "canceling",
            int(GoalStatus.STATUS_SUCCEEDED): "succeeded",
            int(GoalStatus.STATUS_CANCELED): "canceled",
            int(GoalStatus.STATUS_ABORTED): "aborted",
        }
        result["action_status_name"] = status_names.get(status, "invalid")
        if status != int(GoalStatus.STATUS_SUCCEEDED):
            node.get_logger().error(
                f"NavigateToPose ended with status={status}"
            )
            result["error"] = f"NavigateToPose ended with status={status}"
            return 7
        result["status"] = "PASS"
        result["error"] = ""
        node.get_logger().info("one staged NavigateToPose goal succeeded")
        return 0
    except Exception as error:
        result["action_status_name"] = "dispatcher_exception"
        result["error"] = f"{type(error).__name__}: {error}"
        node.get_logger().error(f"staged goal dispatcher exception: {error}")
        return 8
    finally:
        result["finished_unix_ns"] = time.time_ns()
        write_private_json(action_result_path, result)
        if goal_handle is not None:
            # The guard owns the normal cancel decision. This only covers
            # process shutdown while the goal is still active.
            pass
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    environ = dict(os.environ)
    try:
        action_result_path, _ = result_paths(environ)
        runtime = _runtime_from_environment(environ)
        if environ.get("GO2_STAGED_NAV2_RUNTIME_ACK") == STAGED_NAV2_ACK:
            consumed = ConsumedGoalPermit(
                path=Path("/runtime-ack"),
                permit_id="runtime-ack",
                pair_id=environ["GO2_STAGED_NAV2_PAIR_ID"],
                session_id=environ["GO2_STAGED_NAV2_SESSION_ID"],
                map_id=environ["GO2_STAGED_NAV2_MAP_ID"],
                map_generation=int(
                    environ["GO2_STAGED_NAV2_MAP_GENERATION"], 10
                ),
                goal_source=environ["GO2_STAGED_NAV2_GOAL_SOURCE"],
                target_id=environ["GO2_STAGED_NAV2_TARGET_ID"],
                goal_x=float(environ["GO2_STAGED_NAV2_EXPECTED_GOAL_X"]),
                goal_y=float(environ["GO2_STAGED_NAV2_EXPECTED_GOAL_Y"]),
                goal_yaw=float(
                    environ["GO2_STAGED_NAV2_EXPECTED_GOAL_YAW"]
                ),
            )
        else:
            consumed = consume_staged_nav2_goal_permit(
                runtime,
                environ,
                ROOT,
            )
    except (PermitError, ResultError) as error:
        print(f"staged goal dispatch rejected: {error}", file=sys.stderr)
        return 1
    print(f"authorized staged goal target={consumed.target_id}")
    return _run_ros(consumed, environ, action_result_path)


if __name__ == "__main__":
    raise SystemExit(main())
