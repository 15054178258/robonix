#!/usr/bin/env python3
"""Register the guarded Go2 ROS topics with Robonix.

This provider deliberately exposes no posture or direct movement RPC. Nav2 is
the single intended owner of the continuous ``/cmd_vel`` stream, while the C++
adapter independently enforces the physical motion gates.
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess

from robonix_api import Deferred, Err, Ok, Primitive

from .first_motion_permit import PermitError, consume_first_motion_permit
from .runtime_config import (
    ConfigError,
    FIRST_MOTION_PROFILE,
    RuntimeConfig,
    SECOND_MOTION_PROFILE,
    STAGED_NAV2_PROFILE,
    normalize_config,
    prepare_private_directory,
)
from .second_motion_permit import (
    PermitError as SecondMotionPermitError,
    consume_second_motion_permit,
)
from .staged_nav2_permit import (
    PermitError as StagedNav2PermitError,
    STAGED_NAV2_ACK,
    consume_staged_nav2_permit,
)


go2_chassis = Primitive(
    id="go2_chassis", namespace="robonix/primitive/chassis"
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
    / "go2_chassis_adapter"
    / "go2_chassis_adapter_node"
)
_daemon_binary = (
    _package_root
    / "rbnx-build"
    / "sdk"
    / "install"
    / "bin"
    / "go2_sport_daemon"
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
            go2_chassis.spawn(
                runtime.daemon_argv(_daemon_binary),
                env=runtime.sdk_daemon_env(_daemon_binary),
                log="sdk-daemon.log",
                cwd=_package_root,
            )
        )
    _processes.append(
        go2_chassis.spawn(
            runtime.adapter_argv(_adapter_binary, _params_file),
            env=environment,
            log="adapter.log",
            cwd=_package_root,
        )
    )


@go2_chassis.on_init
def initialize(config):
    """Validate Driver config, spawn guarded processes, then declare topics."""
    global _odom_endpoint, _runtime

    _stop_processes()
    _runtime = None
    try:
        runtime = normalize_config(config, os.environ, _package_root)
    except ConfigError as error:
        return Err(f"unsafe Go2 chassis config: {error}")

    if not _is_executable(_adapter_binary):
        return Err(f"Go2 adapter is not built: {_adapter_binary}")
    if not _params_file.is_file():
        return Err(f"Go2 adapter parameter file is missing: {_params_file}")
    if runtime.starts_sdk_daemon and not _is_executable(_daemon_binary):
        return Err(f"Go2 SDK daemon is not built: {_daemon_binary}")

    if runtime.starts_sdk_daemon:
        if runtime.motion_profile == FIRST_MOTION_PROFILE:
            try:
                consumed = consume_first_motion_permit(
                    runtime, os.environ, _package_root.parents[1]
                )
            except (OSError, PermitError) as error:
                return Err(f"first-motion permit rejected: {error}")
            print("consumed one-time first-motion permit " + consumed.name)
        elif runtime.motion_profile == SECOND_MOTION_PROFILE:
            try:
                consumed = consume_second_motion_permit(
                    runtime, os.environ, _package_root.parents[1]
                )
            except (OSError, SecondMotionPermitError) as error:
                return Err(f"second-motion permit rejected: {error}")
            print("consumed one-time second-motion permit " + consumed.name)
        elif runtime.motion_profile == STAGED_NAV2_PROFILE:
            if os.environ.get("GO2_STAGED_NAV2_RUNTIME_ACK") == STAGED_NAV2_ACK:
                # Normal Robonix/Nav2 operation uses the already explicit
                # runtime gate plus the adapter/daemon watchdogs.  Historical
                # evidence bundles remain available in the legacy staged
                # commissioning path, but are not a per-start requirement.
                print("accepted staged-nav2 runtime acknowledgement")
            else:
                try:
                    consumed = consume_staged_nav2_permit(
                        runtime, os.environ, _package_root.parents[1]
                    )
                except (OSError, StagedNav2PermitError) as error:
                    return Err(f"staged-nav2 permit rejected: {error}")
                print("consumed one-time staged-nav2 permit " + consumed.name)
        else:
            return Err("motion profile has no permit consumer")

    try:
        _spawn_runtime(runtime)
    except (OSError, ValueError) as error:
        _stop_processes()
        return Err(f"failed to start guarded Go2 runtime: {error}")

    try:
        odom_available = go2_chassis.wait_for_topic(
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
            return Err(f"Go2 adapter exited during startup with code {adapter_code}")
        return Deferred(
            f"guarded odometry topic {runtime.odom_topic} is not available"
        )
    if runtime.starts_sdk_daemon and _processes[0].poll() is not None:
        daemon_code = _processes[0].returncode
        _stop_processes()
        return Err(f"Go2 SDK daemon exited during startup with code {daemon_code}")

    try:
        _odom_endpoint = go2_chassis.create_subscription(
            "robonix/primitive/chassis/odom",
            topic=runtime.odom_topic,
            msg_type="nav_msgs/msg/Odometry",
            callback=lambda _message: None,
            qos="best_effort",
            declare=False,
        )
        go2_chassis.declare_ros2_topic(
            "robonix/primitive/chassis/twist_in",
            runtime.twist_in_topic,
            qos="reliable",
        )
        go2_chassis.declare_ros2_topic(
            "robonix/primitive/chassis/odom",
            runtime.odom_topic,
            qos="reliable",
        )
    except Exception as error:
        _stop_processes()
        return Err(f"failed to declare Go2 chassis contracts: {error}")
    _runtime = runtime
    return Ok()


@go2_chassis.on_shutdown
def shutdown():
    """Stop adapter before daemon; the spawn registry is a second cleanup."""
    global _runtime
    _stop_processes()
    _runtime = None
    return Ok()


if __name__ == "__main__":
    go2_chassis.run()
