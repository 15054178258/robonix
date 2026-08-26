#!/usr/bin/env python3
"""Register the guarded G1 ROS topics with Robonix.

This provider deliberately exposes no posture or direct movement RPC. Nav2 is
the single intended owner of the continuous ``/cmd_vel`` stream, while the C++
daemon independently enforces the physical motion gates.
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess

from robonix_api import Deferred, Err, Ok, Primitive

from .runtime_config import (
    ConfigError,
    RuntimeConfig,
    normalize_config,
    prepare_private_directory,
)

g1_chassis = Primitive(
    id="g1_chassis", namespace="robonix/primitive/chassis"
)
_package_root = Path(
    os.environ.get(
        "RBNX_PACKAGE_ROOT", str(Path(__file__).resolve().parents[1])
    )
).resolve()
_params_file = _package_root / "config" / "adapter.yaml"
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
_odom_endpoint = None
_runtime: RuntimeConfig | None = None
_processes: list[subprocess.Popen] = []


def _is_executable(path: Path) -> bool:
    return path.is_file() and os.access(path, os.X_OK)


def _stop_processes() -> None:
    """Stop adapter first, then daemon; both are owned process groups."""
    global _processes
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
    _processes = []


def _spawn_runtime(runtime: RuntimeConfig) -> None:
    prepare_private_directory(runtime.ipc_socket.parent)
    environment = runtime.process_env()

    if runtime.starts_sdk_daemon:
        _processes.append(
            g1_chassis.spawn(
                runtime.daemon_argv(_daemon_binary),
                env=runtime.sdk_daemon_env(_daemon_binary),
                log="sdk-daemon.log",
                cwd=_package_root,
            )
        )
    _processes.append(
        g1_chassis.spawn(
            runtime.adapter_argv(_adapter_binary, _params_file),
            env=environment,
            log="adapter.log",
            cwd=_package_root,
        )
    )


@g1_chassis.on_init
def initialize(config):
    """Validate Driver config, spawn guarded processes, then declare topics."""
    global _odom_endpoint, _runtime

    _stop_processes()
    _runtime = None
    try:
        runtime = normalize_config(config, os.environ, _package_root)
    except ConfigError as error:
        return Err(f"unsafe G1 chassis config: {error}")

    if not _is_executable(_adapter_binary):
        return Err(f"G1 adapter is not built: {_adapter_binary}")
    if not _params_file.is_file():
        return Err(f"G1 adapter parameter file is missing: {_params_file}")
    if runtime.starts_sdk_daemon and not _is_executable(_daemon_binary):
        return Err(f"G1 SDK daemon is not built: {_daemon_binary}")

    if runtime.starts_sdk_daemon and runtime.allow_motion:
        # Require explicit motion permit directory on the deploy root
        deploy_root = _package_root.parent.parent
        permit_file = deploy_root / ".motion_permit"
        if not permit_file.is_file():
            return Err(
                "G1 motion is enabled but no motion permit file exists; "
                "create a non-empty .motion_permit in the deploy root"
            )

    try:
        _spawn_runtime(runtime)
    except (OSError, ValueError) as error:
        _stop_processes()
        return Err(f"failed to start guarded G1 runtime: {error}")

    try:
        odom_available = g1_chassis.wait_for_topic(
            runtime.odom_topic,
            "nav_msgs/msg/Odometry",
            runtime.startup_timeout_s,
        )
    except Exception as error:  # ROS graph/type setup failure
        _stop_processes()
        return Err(f"failed while waiting for guarded odometry: {error}")
    if not odom_available:
        adapter_code = _processes[-1].poll() if _processes else None
        _stop_processes()
        if adapter_code is not None:
            return Err(f"G1 adapter exited during startup with code {adapter_code}")
        return Deferred(
            f"guarded odometry topic {runtime.odom_topic} is not available"
        )
    if runtime.starts_sdk_daemon and _processes[0].poll() is not None:
        daemon_code = _processes[0].returncode
        _stop_processes()
        return Err(f"G1 SDK daemon exited during startup with code {daemon_code}")

    try:
        _odom_endpoint = g1_chassis.create_subscription(
            "robonix/primitive/chassis/odom",
            topic=runtime.odom_topic,
            msg_type="nav_msgs/msg/Odometry",
            callback=lambda _message: None,
            qos="best_effort",
            declare=False,
        )
        g1_chassis.declare_ros2_topic(
            "robonix/primitive/chassis/twist_in",
            runtime.twist_in_topic,
            qos="reliable",
        )
        g1_chassis.declare_ros2_topic(
            "robonix/primitive/chassis/odom",
            runtime.odom_topic,
            qos="reliable",
        )
    except Exception as error:
        _stop_processes()
        return Err(f"failed to declare G1 chassis contracts: {error}")
    _runtime = runtime
    return Ok()


@g1_chassis.on_shutdown
def shutdown():
    """Stop adapter before daemon; the spawn registry is a second cleanup."""
    global _runtime
    _stop_processes()
    _runtime = None
    return Ok()


if __name__ == "__main__":
    g1_chassis.run()
