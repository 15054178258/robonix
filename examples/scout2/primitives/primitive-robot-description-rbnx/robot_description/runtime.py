from __future__ import annotations

import os
import signal
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


IMAGE = "robonix/primitive-robot-description:humble"


def inspect_urdf(urdf_xml: str) -> tuple[str, int, int]:
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(f"URDF root must be <robot>, got <{root.tag}>")
    links = root.findall("link")
    joints = root.findall("joint")
    if not links:
        raise ValueError("URDF has no links")
    child_links = {
        child.attrib["link"]
        for joint in joints
        if (child := joint.find("child")) is not None and "link" in child.attrib
    }
    roots = [link.attrib.get("name", "") for link in links if link.attrib.get("name") not in child_links]
    if len(roots) != 1:
        raise ValueError(f"URDF must have one root link, found {roots}")
    return roots[0], len(links), len(joints)


def write_params(path: Path, urdf_xml: str) -> None:
    body = "\n".join(f"      {line}" for line in urdf_xml.splitlines())
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    robot_description: |\n"
        f"{body}\n",
        encoding="utf-8",
    )


def command_for(mode: str, params_path: Path, container_name: str) -> list[str]:
    ros_args = [
        "ros2",
        "run",
        "robot_state_publisher",
        "robot_state_publisher",
        "--ros-args",
        "--params-file",
    ]
    if mode == "native":
        return [*ros_args, str(params_path)]
    if mode != "docker":
        raise ValueError(f"ROBONIX_ROBOT_DESCRIPTION_MODE must be native or docker, got {mode!r}")

    command = [
        "docker",
        "run",
        "--rm",
        "--name",
        container_name,
        "--network",
        "host",
        "-v",
        f"{params_path}:/run/robonix/robot_description.yaml:ro",
    ]
    for name in (
        "ROS_DOMAIN_ID",
        "RMW_IMPLEMENTATION",
        "ROBONIX_ZENOH_ROUTER",
        "ROBONIX_ZENOH_MODE",
        "ROBONIX_ZENOH_LISTEN",
    ):
        value = os.environ.get(name)
        if value:
            command.extend(["-e", f"{name}={value}"])
    return [*command, IMAGE, *ros_args, "/run/robonix/robot_description.yaml"]


def stop_process(proc: subprocess.Popen[bytes] | None, container_name: str) -> None:
    if proc is not None and proc.poll() is None:
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
    if shutil.which("docker"):
        subprocess.run(
            ["docker", "rm", "-f", container_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
