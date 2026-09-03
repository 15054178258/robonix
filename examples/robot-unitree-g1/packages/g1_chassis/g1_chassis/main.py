#!/usr/bin/env python3
"""g1_chassis — Unitree G1 chassis provider (SDK daemon + ROS2 adapter).

Starts the G1 SDK daemon and ROS2 adapter node, then registers with atlas
as a primitive offering chassis/driver, twist_in and odom capabilities.
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import time
from pathlib import Path

from robonix_api import Deferred, Err, Ok, Primitive

logging.basicConfig(
    level=os.environ.get("G1_CHASSIS_LOG_LEVEL", "INFO"),
    format="[g1_chassis] %(message)s",
)
log = logging.getLogger("g1_chassis")

g1_chassis = Primitive(
    id="g1_chassis", namespace="robonix/primitive/chassis"
)

_package_root = Path(
    os.environ.get(
        "RBNX_PACKAGE_ROOT", str(Path(__file__).resolve().parents[1])
    )
).resolve()

_adapter_binary = (
    _package_root
    / "rbnx-build"
    / "ros"
    / "install"
    / "lib"
    / "g1_chassis_adapter"
    / "g1_chassis_adapter_node"
)
_daemon_binary = (
    _package_root
    / "rbnx-build"
    / "sdk"
    / "install"
    / "bin"
    / "g1_loco_daemon"
)

_processes: list[subprocess.Popen] = []


def _is_executable(path: Path) -> bool:
    return path.is_file() and os.access(path, os.X_OK)


def _stop_processes() -> None:
    """Stop adapter first, then daemon (reverse order)."""
    ordered = list(reversed(_processes))
    for process in ordered:
        if process.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
    for process in ordered:
        if process.poll() is not None:
            continue
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    _processes.clear()


def _daemon_argv(socket_path: str) -> list[str]:
    return [
        str(_daemon_binary),
        "--socket", socket_path,
        "--max-vx", "0.5",
        "--max-vy", "0.3",
        "--max-wz", "2.0",
    ]


def _daemon_env(socket_path: str) -> dict[str, str]:
    env = os.environ.copy()
    env["G1_IPC_SOCKET"] = socket_path
    sdk_lib = str(_package_root / "rbnx-build" / "sdk" / "install" / "lib")
    env["LD_LIBRARY_PATH"] = sdk_lib + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


@g1_chassis.on_init
def initialize(config):
    """Start daemon + adapter, wait for odometry, then declare topics."""
    global _processes

    _stop_processes()

    socket_path = os.environ.get(
        "G1_IPC_SOCKET",
        str(Path.home() / ".robonix" / "g1_chassis.sock"),
    )
    os.makedirs(os.path.dirname(socket_path), exist_ok=True)

    if not _is_executable(_adapter_binary):
        return Err(f"adapter not built: {_adapter_binary}")
    if not _is_executable(_daemon_binary):
        return Err(f"daemon not built: {_daemon_binary}")

    # Start SDK daemon (non-motion mode — adapter handles safety)
    daemon_env = _daemon_env(socket_path)

    _processes.append(
        g1_chassis.spawn(
            _daemon_argv(socket_path),
            env=daemon_env,
            log="sdk-daemon.log",
            cwd=str(_package_root),
        )
    )

    # Wait for daemon to bind the socket (up to 3s)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if os.path.exists(socket_path):
            break
        time.sleep(0.1)

    if not os.path.exists(socket_path):
        _stop_processes()
        return Err("g1_loco_daemon did not create socket within 3s")

    if _processes[0].poll() is not None:
        code = _processes[0].returncode
        _stop_processes()
        return Err(f"g1_loco_daemon exited with code {code}")

    # Start ROS2 adapter (daemon env already has G1_IPC_SOCKET)
    _processes.append(
        g1_chassis.spawn(
            [str(_adapter_binary)],
            env=daemon_env,
            log="adapter.log",
            cwd=str(_package_root),
        )
    )

    # Wait for adapter ROS topics to appear (up to 10s)
    try:
        odom_available = g1_chassis.wait_for_topic(
            "/odom",
            "nav_msgs/msg/Odometry",
            10.0,
        )
    except Exception as error:
        _stop_processes()
        return Err(f"wait_for_topic failed: {error}")

    if not odom_available:
        code = _processes[-1].poll()
        _stop_processes()
        if code is not None:
            return Err(f"adapter exited during startup with code {code}")
        return Deferred("/odom topic /odom not available after adapter start")

    # Check adapter didn't crash
    if _processes[-1].poll() is not None:
        code = _processes[-1].returncode
        _stop_processes()
        return Err(f"adapter exited with code {code}")

    # Declare ROS2 topic capabilities with atlas.
    # The *driver capability is auto-declared by _do_bootstrap() with
    # Transport.GRPC — no manual declaration needed here.
    try:
        g1_chassis.declare_ros2_topic(
            "robonix/primitive/chassis/twist_in",
            "/cmd_vel",
            qos="reliable",
        )
        g1_chassis.declare_ros2_topic(
            "robonix/primitive/chassis/odom",
            "/odom",
            qos="reliable",
        )
    except Exception as error:
        _stop_processes()
        return Err(f"failed to declare chassis contracts: {error}")

    log.info(
        "G1 chassis initialized "
        "(daemon pid=%d, adapter pid=%d)",
        _processes[0].pid,
        _processes[-1].pid,
    )
    return Ok()


@g1_chassis.on_shutdown
def shutdown():
    """Stop adapter and daemon."""
    _stop_processes()
    return Ok()


if __name__ == "__main__":
    g1_chassis.run()
