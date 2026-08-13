from __future__ import annotations

import logging
import os
import subprocess
import time
from pathlib import Path

import grpc
import robonix_contracts_pb2_grpc as contracts_grpc
import soma_pb2
from robonix_api import Err, Ok, Primitive

from .runtime import command_for, inspect_urdf, stop_process, write_params


logging.basicConfig(level=logging.INFO, format="[robot_description] %(message)s")
log = logging.getLogger("robot_description")

PROVIDER_ID = os.environ.get("RBNX_INSTANCE_NAME", "robot_description")
CONTAINER_NAME = f"robonix_{PROVIDER_ID}"
cap = Primitive(id=PROVIDER_ID, namespace="robonix/primitive/robot_description")
_process: subprocess.Popen[bytes] | None = None


def _fetch_urdf() -> tuple[str, str]:
    endpoint = os.environ.get("ROBONIX_SOMA_ENDPOINT", "").strip()
    if not endpoint:
        raise RuntimeError("ROBONIX_SOMA_ENDPOINT was not supplied by Soma")
    channel = grpc.insecure_channel(endpoint)
    grpc.channel_ready_future(channel).result(timeout=10)
    try:
        stub = contracts_grpc.RobonixSystemSomaGetUrdfStub(channel)
        response = stub.GetUrdf(soma_pb2.GetUrdf_Request(robot_id=""), timeout=10)
    finally:
        channel.close()
    if not response.urdf_xml.strip():
        raise RuntimeError("Soma returned an empty URDF")
    return response.robot_id, response.urdf_xml


@cap.on_init
def init(_cfg: dict):
    global _process
    if _process is not None and _process.poll() is None:
        return Ok()
    try:
        robot_id, urdf_xml = _fetch_urdf()
        root_link, links, joints = inspect_urdf(urdf_xml)
        params_path = Path(os.environ.get("RBNX_PACKAGE_ROOT", Path.cwd())) / "rbnx-build" / "runtime" / "robot_description.yaml"
        write_params(params_path, urdf_xml)
        mode = os.environ.get("ROBONIX_ROBOT_DESCRIPTION_MODE", "docker").strip().lower()
        command = command_for(mode, params_path, CONTAINER_NAME)
        log.info(
            "Soma robot=%s root=%s links=%d joints=%d mode=%s rmw=%s",
            robot_id,
            root_link,
            links,
            joints,
            mode,
            os.environ.get("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
        )
        _process = subprocess.Popen(command)
        time.sleep(1.0)
        if _process.poll() is not None:
            return Err(f"robot_state_publisher exited with rc={_process.returncode}")
        return Ok()
    except Exception as exc:  # noqa: BLE001
        stop_process(_process, CONTAINER_NAME)
        _process = None
        log.exception("initialization failed")
        return Err(str(exc))


@cap.on_shutdown
def shutdown():
    global _process
    stop_process(_process, CONTAINER_NAME)
    _process = None
    return Ok()


if __name__ == "__main__":
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    cap.run()

