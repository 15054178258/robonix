"""Robonix provider owning telemetry and optional Liaison voice UI child."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from robonix_api import Err, Ok, Service

import go2_dashboard_pb2
from go2_dashboard_mcp import (
    GetDashboardStatus_Request,
    GetDashboardStatus_Response,
)

from .provider_runtime import DashboardProcess, status_json


PROVIDER_ID = "go2_dashboard"
NAMESPACE = "robonix/service/telemetry/dashboard"
STATUS_CONTRACT = f"{NAMESPACE}/status"

service = Service(id=PROVIDER_ID, namespace=NAMESPACE)
dashboard = DashboardProcess()


@service.on_init
def initialize(config: dict):
    """Validate deployment config and start the owned read-only web child."""

    try:
        dashboard.configure(config)
        dashboard.start()
    except Exception as error:
        return Err(str(error))
    return Ok()


@service.on_activate
def activate():
    """Ensure the configured child is still online after activation."""

    try:
        dashboard.ensure_running()
    except Exception as error:
        return Err(str(error))
    return Ok()


@service.on_deactivate
def deactivate():
    """Release the only hot resource owned by this service."""

    dashboard.stop()
    return Ok()


@service.on_shutdown
def shutdown():
    """Terminate only the child process created by this provider."""

    dashboard.stop()
    return Ok()


def _status_fields() -> dict:
    snapshot = dashboard.status()
    return {
        "ok": bool(snapshot["ok"]),
        "url": str(snapshot["url"]),
        "detail": str(snapshot["detail"]),
        "status_json": status_json(snapshot),
    }


@service.mcp(STATUS_CONTRACT)
def get_dashboard_status(
    request: GetDashboardStatus_Request,
) -> GetDashboardStatus_Response:
    """Read dashboard process, URL, ROS connection, and telemetry health."""

    _ = request
    return GetDashboardStatus_Response(**_status_fields())


@service.grpc(
    STATUS_CONTRACT,
    description="Read dashboard process, URL, ROS connection, and telemetry health.",
)
def get_dashboard_status_rpc(request, context):
    """Read the same status over the generated unary gRPC contract."""

    _ = (request, context)
    return go2_dashboard_pb2.GetDashboardStatus_Response(**_status_fields())


def main() -> int:
    pid_file = Path(
        os.environ.get(
            "GO2_DASHBOARD_PROVIDER_PID_FILE", "rbnx-build/run/provider.pid"
        )
    ).resolve()
    pid_file.parent.mkdir(parents=True, exist_ok=True)
    pid_file.write_text(f"{os.getpid()}\n", encoding="ascii")
    try:
        service.run()
        return 0
    finally:
        try:
            if pid_file.read_text(encoding="ascii").strip() == str(os.getpid()):
                pid_file.unlink()
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    sys.exit(main())
