"""Robonix lifecycle wrapper for the standard ROS robot_state_publisher."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import time

import grpc
import robonix_contracts_pb2_grpc as contracts_grpc
import soma_pb2
from robonix_api import Err, Ok, Primitive

from .runtime import (
    fixed_joint_transforms,
    require_pinned_urdf,
    validate_urdf,
    wait_for_static_transforms,
    write_robot_state_publisher_params,
)


ROOT = Path(__file__).resolve().parents[1]
PINNED_URDF = ROOT / "urdf" / "go2_robonix.urdf"
PARAMETERS = ROOT / "rbnx-build" / "runtime" / "robot_description.yaml"

description = Primitive(
    id="robot_description", namespace="robonix/primitive/robot_description"
)
_child = None


def _fetch_soma_urdf() -> tuple[str, str]:
    endpoint = os.environ.get("ROBONIX_SOMA_ENDPOINT", "").strip()
    if not endpoint:
        raise RuntimeError("ROBONIX_SOMA_ENDPOINT was not supplied by Soma")
    channel = grpc.insecure_channel(endpoint)
    try:
        grpc.channel_ready_future(channel).result(timeout=10.0)
        stub = contracts_grpc.RobonixSystemSomaGetUrdfStub(channel)
        response = stub.GetUrdf(
            soma_pb2.GetUrdf_Request(robot_id=""), timeout=10.0
        )
    finally:
        channel.close()
    if not response.robot_id or not response.urdf_xml:
        raise RuntimeError("Soma returned an empty robot id or URDF")
    return response.robot_id, response.urdf_xml


def _stop_child() -> None:
    global _child
    child = _child
    _child = None
    if child is None or child.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(child.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        child.wait(timeout=5.0)
    except Exception:
        try:
            os.killpg(os.getpgid(child.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


@description.on_init
def initialize(config: dict):
    global _child
    if config:
        return Err("robot_description does not accept deployment overrides")
    if _child is not None and _child.poll() is None:
        return Ok()
    try:
        robot_id, urdf_xml = _fetch_soma_urdf()
        digest = require_pinned_urdf(urdf_xml, PINNED_URDF)
        root_link, links, joints = validate_urdf(urdf_xml)
        expected_static = fixed_joint_transforms(urdf_xml)
        write_robot_state_publisher_params(PARAMETERS, urdf_xml)
        _child = description.spawn(
            [
                "ros2",
                "run",
                "robot_state_publisher",
                "robot_state_publisher",
                "--ros-args",
                "--params-file",
                str(PARAMETERS),
            ],
            log="robot-state-publisher.log",
            cwd=ROOT,
        )
        time.sleep(1.0)
        if _child.poll() is not None:
            raise RuntimeError(
                f"robot_state_publisher exited during startup with code {_child.returncode}"
            )
        tf_sentinel = wait_for_static_transforms(expected_static, 10.0)
        if not tf_sentinel.ready:
            missing = ", ".join(
                f"{parent}->{child}"
                for parent, child in sorted(tf_sentinel.missing)[:5]
            )
            raise RuntimeError(
                "robot_state_publisher did not latch the complete /tf_static tree "
                f"(expected={len(expected_static)}, "
                f"observed={len(tf_sentinel.observed)}, missing={missing})"
            )
        print(
            f"[go2_description] robot={robot_id} root={root_link} "
            f"links={links} joints={joints} sha256={digest}"
        )
    except Exception as error:
        _stop_child()
        return Err(f"Go2 description initialization failed: {error}")
    return Ok()


@description.on_shutdown
def shutdown():
    _stop_child()
    return Ok()


if __name__ == "__main__":
    description.run()
