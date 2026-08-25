#!/usr/bin/env python3
"""Explicit arm/disarm tool for the persistent Go2 voice/Nav2 profile.

Arming is fail-closed: it requires the current map generation, fresh
localization/odometry/scan, an idle NavigateToPose action, exact velocity-topic
ownership and healthy disarmed chassis diagnostics.  It publishes only a
bounded zero-velocity preparation stream, never a motion command or goal.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any, Sequence


ARM_ACK = "I_CONFIRM_CURRENT_SITE_CLEAR_AND_REMOTE_STOP_READY"
PROFILE = "workstation-staged-nav2-corrected-v1"
CLASSIC_MOTION_STATE_MARKERS = frozenset({100, 2010})
ARM_SERVICE = "/go2_chassis/arm"
COMMAND_TOPIC = "/go2/staged_nav2/cmd_vel"
GOAL_STATUS_TOPIC = "/navigate_to_pose/_action/status"
ACTIVE_GOAL_STATES = {1, 2, 3}
SNAPSHOT_TIMEOUT_S = 15.0
INITIAL_GOAL_STATUS_OBSERVATION_S = 6.0
ZERO_PREPARATION_S = 1.0


class ArmError(RuntimeError):
    pass


def _uint(value: str, label: str, maximum: int) -> int:
    if not value.isdecimal():
        raise ArmError(f"{label} must be an unsigned decimal integer")
    result = int(value, 10)
    if result > maximum:
        raise ArmError(f"{label} is outside range")
    return result


def _endpoint_names(entries: Sequence[Any]) -> set[str]:
    names = set()
    for entry in entries:
        namespace = str(entry.node_namespace or "/").rstrip("/")
        name = str(entry.node_name).lstrip("/")
        names.add(f"{namespace}/{name}" if namespace else f"/{name}")
    return names


def infer_initial_idle_goal_state(
    samples: dict[str, Any], status_publishers: Sequence[Any], *, now: float
) -> None:
    """Handle Nav2's empty initial status without masking a live goal.

    rcl_action does not publish an empty GoalStatusArray before the first goal.
    The caller has already observed for the full startup window; an accepted or
    executing goal would publish status during that window.  Therefore the
    only safe no-sample case is one exact bt_navigator status publisher.
    """

    if "goals" in samples:
        return
    publishers = _endpoint_names(status_publishers)
    if publishers != {"/bt_navigator"}:
        raise ArmError(
            "no goal status sample and unexpected status publishers: "
            f"{sorted(publishers)}"
        )
    samples["goals"] = (now, [])


def validate_snapshot(
    samples: dict[str, Any],
    *,
    map_id: str,
    generation: int,
    allowed_mode: int,
    allowed_marker: int,
    now: float,
) -> None:
    if allowed_marker not in CLASSIC_MOTION_STATE_MARKERS:
        raise ArmError("allowed marker is outside the reviewed Classic set")
    required = {"lifecycle", "pose", "odom", "scan", "goals", "diagnostics"}
    missing = sorted(required - set(samples))
    if missing:
        raise ArmError("missing live samples: " + ", ".join(missing))
    lifecycle = samples["lifecycle"][1]
    if lifecycle != (map_id, "localization", generation):
        raise ArmError("live MapLifecycle does not match the requested localization map")
    for key, maximum_age in (("pose", 0.75), ("odom", 0.75), ("scan", 0.75), ("diagnostics", 1.5)):
        if now - float(samples[key][0]) > maximum_age:
            raise ArmError(f"{key} sample is stale")
    if samples["pose"][1] != "map":
        raise ArmError("localization pose frame must be map")
    if samples["odom"][1] != ("odom", "base_link"):
        raise ArmError("odometry frame chain must be odom -> base_link")
    if samples["scan"][1] != "base_link":
        raise ArmError("navigation scan frame must be base_link")
    if samples["goals"][1]:
        raise ArmError("an accepted, executing or canceling navigation goal already exists")
    diagnostics = samples["diagnostics"][1]
    expected = {
        "guard_state": "DISARMED",
        "allow_motion": "true",
        "motion_profile": PROFILE,
        "external_odom_valid": "true",
        "external_odom_fault_latched": "false",
        "daemon_armed": "false",
        "state_valid": "true",
        "opaque_state_marker_change_latched": "false",
        "passive_state_marker_transitions_enabled": "false",
        "motion_state_marker_transitions_enabled": "true",
        "sport_mode": str(allowed_mode),
    }
    drift = {
        key: (diagnostics.get(key), value)
        for key, value in expected.items()
        if diagnostics.get(key) != value
    }
    if drift:
        raise ArmError(f"chassis diagnostic gate failed: {drift}")
    try:
        current_marker = int(str(diagnostics.get("sport_error_code")), 10)
    except (TypeError, ValueError) as error:
        raise ArmError("chassis diagnostic marker is malformed") from error
    if current_marker not in CLASSIC_MOTION_STATE_MARKERS:
        raise ArmError("chassis diagnostic marker is outside the reviewed Classic set")


def run(*, arm: bool, map_id: str, generation: int, allowed_mode: int, allowed_marker: int) -> None:
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from map.msg import MapLifecycle
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import SetBool

    rclpy.init()
    node = Node("go2_persistent_nav2_arm_gate")
    samples: dict[str, Any] = {}
    latched = QoSProfile(depth=1)
    latched.reliability = ReliabilityPolicy.RELIABLE
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

    def observed(key: str, payload: Any) -> None:
        samples[key] = (time.monotonic(), payload)

    subscriptions = [
        node.create_subscription(
            MapLifecycle,
            "/robonix/map/lifecycle",
            lambda message: observed(
                "lifecycle",
                (
                    str(message.map_id),
                    str(message.mode).strip().casefold(),
                    int(message.generation),
                ),
            ),
            latched,
        ),
        node.create_subscription(
            PoseWithCovarianceStamped,
            "/robonix/map/pose",
            lambda message: observed("pose", str(message.header.frame_id)),
            10,
        ),
        node.create_subscription(
            Odometry,
            "/odom",
            lambda message: observed(
                "odom", (str(message.header.frame_id), str(message.child_frame_id))
            ),
            20,
        ),
        node.create_subscription(
            LaserScan,
            "/scanner/scan",
            lambda message: observed("scan", str(message.header.frame_id)),
            20,
        ),
        node.create_subscription(
            GoalStatusArray,
            GOAL_STATUS_TOPIC,
            lambda message: observed(
                "goals",
                [
                    bytes(item.goal_info.goal_id.uuid).hex()
                    for item in message.status_list
                    if int(item.status) in ACTIVE_GOAL_STATES
                ],
            ),
            latched,
        ),
    ]

    def diagnostics(message: DiagnosticArray) -> None:
        for status in message.status:
            if str(status.name) == "go2_chassis_adapter":
                observed(
                    "diagnostics",
                    {str(item.key): str(item.value) for item in status.values},
                )

    subscriptions.append(
        node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, 10)
    )
    client = node.create_client(SetBool, ARM_SERVICE)
    zero_publisher = None
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            raise ArmError("chassis arm service is unavailable")
        if not arm:
            request = SetBool.Request()
            request.data = False
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            response = future.result()
            if response is None or not response.success:
                raise ArmError(
                    "disarm was rejected: " + str(getattr(response, "message", "no response"))
                )
            print("Go2 chassis disarmed")
            return

        snapshot_started = time.monotonic()
        deadline = snapshot_started + SNAPSHOT_TIMEOUT_S
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            required_without_goals = {
                "lifecycle",
                "pose",
                "odom",
                "scan",
                "diagnostics",
            }
            now = time.monotonic()
            streams_fresh = required_without_goals <= set(samples) and all(
                now - float(samples[key][0]) <= maximum_age
                for key, maximum_age in (
                    ("pose", 0.75),
                    ("odom", 0.75),
                    ("scan", 0.75),
                    ("diagnostics", 1.5),
                )
            )
            if streams_fresh and "goals" in samples:
                break
            if (
                streams_fresh
                and now - snapshot_started >= INITIAL_GOAL_STATUS_OBSERVATION_S
            ):
                infer_initial_idle_goal_state(
                    samples,
                    node.get_publishers_info_by_topic(GOAL_STATUS_TOPIC),
                    now=now,
                )
                break

        infer_initial_idle_goal_state(
            samples,
            node.get_publishers_info_by_topic(GOAL_STATUS_TOPIC),
            now=time.monotonic(),
        )

        publishers = _endpoint_names(node.get_publishers_info_by_topic(COMMAND_TOPIC))
        subscribers = _endpoint_names(node.get_subscriptions_info_by_topic(COMMAND_TOPIC))
        if publishers != {"/robonix_velocity_guard"}:
            raise ArmError(f"unexpected command publishers: {sorted(publishers)}")
        if subscribers != {"/go2_chassis_adapter"}:
            raise ArmError(f"unexpected command subscribers: {sorted(subscribers)}")
        validate_snapshot(
            samples,
            map_id=map_id,
            generation=generation,
            allowed_mode=allowed_mode,
            allowed_marker=allowed_marker,
            now=time.monotonic(),
        )

        zero_publisher = node.create_publisher(Twist, COMMAND_TOPIC, 10)

        def publish_zero() -> None:
            assert zero_publisher is not None
            zero_publisher.publish(Twist())

        preparation_deadline = time.monotonic() + ZERO_PREPARATION_S
        next_zero = 0.0
        while time.monotonic() < preparation_deadline:
            now = time.monotonic()
            if now >= next_zero:
                publish_zero()
                next_zero = now + 0.04
            rclpy.spin_once(node, timeout_sec=0.02)

        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        arm_deadline = time.monotonic() + 4.0
        while time.monotonic() < arm_deadline:
            now = time.monotonic()
            if now >= next_zero:
                publish_zero()
                next_zero = now + 0.04
            rclpy.spin_once(node, timeout_sec=0.02)
            current = samples.get("diagnostics", (0.0, {}))[1]
            if (
                future.done()
                and future.result() is not None
                and future.result().success
                and current.get("guard_state") == "ARMED"
                and current.get("daemon_armed") == "true"
            ):
                node.destroy_publisher(zero_publisher)
                zero_publisher = None
                time.sleep(0.05)
                publishers = _endpoint_names(
                    node.get_publishers_info_by_topic(COMMAND_TOPIC)
                )
                if publishers != {"/robonix_velocity_guard"}:
                    raise ArmError(
                        f"command ownership did not return to Navigation: {sorted(publishers)}"
                    )
                print("Go2 chassis armed for persistent voice/Nav2 goals")
                return
        response = future.result() if future.done() else None
        raise ArmError(
            "arm did not reach guarded armed state: "
            + str(getattr(response, "message", "timeout"))
        )
    except Exception:
        if arm and client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        raise
    finally:
        if zero_publisher is not None:
            node.destroy_publisher(zero_publisher)
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--arm", action="store_true")
    action.add_argument("--disarm", action="store_true")
    parser.add_argument("--map-id", default=os.environ.get("GO2_PERSISTENT_NAV2_MAP_ID", ""))
    parser.add_argument(
        "--generation",
        default=os.environ.get("GO2_PERSISTENT_NAV2_MAP_GENERATION", ""),
    )
    parser.add_argument(
        "--allowed-mode",
        default=os.environ.get("GO2_PERSISTENT_NAV2_ALLOWED_MODE", ""),
    )
    parser.add_argument(
        "--allowed-state-marker",
        default=os.environ.get("GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER", ""),
    )
    args = parser.parse_args(argv)
    try:
        map_id = ""
        generation = allowed_mode = allowed_marker = 0
        if args.arm:
            map_id = str(args.map_id).strip()
            if not map_id or len(map_id) > 160 or any(
                character
                not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-"
                for character in map_id
            ):
                raise ArmError("map-id must use the safe map-id grammar")
            generation = _uint(
                str(args.generation), "generation", (1 << 64) - 1
            )
            allowed_mode = _uint(str(args.allowed_mode), "allowed-mode", 254)
            allowed_marker = _uint(
                str(args.allowed_state_marker),
                "allowed-state-marker",
                (1 << 32) - 1,
            )
            if os.environ.get("GO2_PERSISTENT_NAV2_ARM_ACK") != ARM_ACK:
                raise ArmError(
                    f"GO2_PERSISTENT_NAV2_ARM_ACK must exactly equal {ARM_ACK}"
                )
        run(
            arm=bool(args.arm),
            map_id=map_id,
            generation=generation,
            allowed_mode=allowed_mode,
            allowed_marker=allowed_marker,
        )
    except (ArmError, ImportError, OSError, ValueError) as error:
        print(f"persistent Nav2 arm gate rejected: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
