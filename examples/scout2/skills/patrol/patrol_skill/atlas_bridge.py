# SPDX-License-Identifier: MulanPSL-2.0
"""patrol_rbnx atlas bridge — Capability + contract-typed MCP tools.

A user-invocable skill: accepts a list of waypoints (x, y, yaw) and
navigates to each one using nav2, capturing a photo at each stop.
Waypoints are automatically sorted by nearest-neighbor from the robot's
current position to minimise travel distance.

Tools are typed against the codegen Request/Response dataclasses for the
patrol/srv/* contracts (Patrol, GetPatrolStatus, CancelPatrol).
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
import uuid
from pathlib import Path

from robonix_api import ATLAS, Skill, Ok, Err

logging.basicConfig(level=logging.INFO,
                    format="[patrol] %(levelname)s %(message)s")
log = logging.getLogger("patrol_rbnx")

patrol_skill = Skill(id="patrol", namespace="robonix/skill/patrol")

# ── Atlas-resolved inputs ────────────────────────────────────────────────────
REQUIRED_INPUTS = {
    "nav_navigate": ("robonix/service/navigation/navigate", "mcp"),
    "nav_status":   ("robonix/service/navigation/navigate/status", "mcp"),
    "nav_cancel":   ("robonix/service/navigation/navigate/cancel", "mcp"),
    "camera_snap":  ("robonix/primitive/camera/snapshot", "mcp"),
}


def resolve_inputs(deadline_s: float = 60.0) -> dict[str, str]:
    resolved: dict[str, str] = {}
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        for key, (cid, transport) in REQUIRED_INPUTS.items():
            if key in resolved:
                continue
            try:
                cap_view = ATLAS.find_unique_capability(
                    contract_id=cid, transport=transport,
                )
                ch = patrol_skill.connect_capability(cap_view, cid, transport)
            except Exception:  # noqa: BLE001
                continue
            ep = ch.endpoint
            ch.close()
            if ep:
                resolved[key] = ep
                log.info("resolved %s [%s] → %s", cid, transport, ep)
        if len(resolved) == len(REQUIRED_INPUTS):
            return resolved
        time.sleep(2.0)
    missing = [k for k in REQUIRED_INPUTS if k not in resolved]
    raise RuntimeError(
        f"patrol skill cannot find dependencies on atlas: missing "
        f"{[REQUIRED_INPUTS[k][0] for k in missing]}. The skill needs "
        f"navigation service (navigate with status/cancel) and camera "
        f"primitive (snapshot) before it can start."
    )


# ── Task state ───────────────────────────────────────────────────────────────
_tasks: dict[str, dict] = {}
_task_lock = threading.Lock()

# ── ROS node + tf2 for robot pose ───────────────────────────────────────────
_ros_node = None
_ros_thread = None
_tf_buffer = None
_tf_listener = None
_stop_evt = threading.Event()


def _get_ros_node():
    """Get or create a ROS2 node with tf2 listener."""
    global _ros_node, _tf_buffer, _tf_listener
    if _ros_node is not None:
        return _ros_node
    import rclpy
    from rclpy.node import Node
    if not rclpy.ok():
        rclpy.init()
    _ros_node = Node("patrol_skill")
    from tf2_ros import Buffer, TransformListener
    _tf_buffer = Buffer()
    _tf_listener = TransformListener(_tf_buffer, _ros_node)
    log.info("ROS2 node + tf2 listener created")
    return _ros_node


def _spin_ros():
    """Spin the ROS2 node in a background thread."""
    import rclpy
    node = _get_ros_node()
    while not _stop_evt.is_set():
        try:
            rclpy.spin_once(node, timeout_sec=0.2)
        except Exception:
            time.sleep(0.1)


def _get_robot_pose() -> tuple[float, float] | None:
    """Look up map→base_link and return (x, y). Returns None if unavailable."""
    if _tf_buffer is None:
        return None
    try:
        from rclpy.time import Time
        from rclpy.duration import Duration
        tr = _tf_buffer.lookup_transform(
            "map", "base_link", Time(),
            timeout=Duration(seconds=2.0))
        x = float(tr.transform.translation.x)
        y = float(tr.transform.translation.y)
        return (x, y)
    except Exception:
        return None


def _nearest_neighbor_sort(waypoints: list[dict]) -> list[dict]:
    """Reorder waypoints by nearest-neighbor from robot's current position.

    Greedy: always pick the closest unvisited waypoint next. This
    minimises total travel distance for typical patrol patterns where
    the robot starts near some waypoints and the rest are scattered.
    """
    pose = _get_robot_pose()
    if pose is None:
        log.info("no robot pose available, keeping original waypoint order")
        return waypoints

    cx, cy = pose
    remaining = list(range(len(waypoints)))
    ordered: list[dict] = []

    while remaining:
        best_idx = min(remaining,
                       key=lambda i: math.hypot(waypoints[i]["x"] - cx,
                                                waypoints[i]["y"] - cy))
        ordered.append(waypoints[best_idx])
        cx, cy = waypoints[best_idx]["x"], waypoints[best_idx]["y"]
        remaining.remove(best_idx)

    log.info("nearest-neighbor sort: robot at (%.2f, %.2f), order: %s",
             pose[0], pose[1],
             [w["name"] for w in ordered])
    return ordered


# ── MCP client for nav/camera RPCs ───────────────────────────────────────────
_mcp_clients: dict[str, object] = {}
_mcp_endpoints: dict[str, str] = {}


def _ensure_mcp_client(name: str):
    if name in _mcp_clients:
        return
    from fastmcp import Client
    url = _mcp_endpoints[name]
    _mcp_clients[name] = Client(url)


async def _mcp_call(endpoint_name: str, tool: str, args: dict) -> dict:
    _ensure_mcp_client(endpoint_name)
    client = _mcp_clients[endpoint_name]
    async with client as c:
        result = await c.call_tool(tool, args)
        if not result.content:
            return {}
        import json
        txt = result.content[0].text
        try:
            return json.loads(txt)
        except Exception:
            return {"raw": txt}


def _mcp_call_sync(endpoint_name: str, tool: str, args: dict) -> dict:
    import asyncio
    try:
        return asyncio.run(_mcp_call(endpoint_name, tool, args))
    except Exception as e:  # noqa: BLE001
        log.warning("mcp call %s/%s failed: %s", endpoint_name, tool, e)
        return {}


# ── Nav helpers ──────────────────────────────────────────────────────────────
def _nav_navigate_blocking(x: float, y: float, yaw: float | None,
                           timeout_s: float, cancel_check) -> tuple[bool, str]:
    """Send a nav goal and poll until terminal or timeout."""
    if yaw is not None:
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    else:
        qz, qw = 0.0, 1.0
    goal_pose = {
        "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
        },
    }
    resp = _mcp_call_sync("nav", "navigate", {"goal": goal_pose})
    if not resp.get("accepted", False):
        return False, f"goal rejected: {resp.get('detail', '')}"
    run_id = resp.get("run_id", "")

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if cancel_check():
            _mcp_call_sync("nav", "cancel", {"run_id": run_id})
            return False, "canceled"
        sresp = _mcp_call_sync("nav", "status", {"run_id": run_id})
        if sresp:
            state = str(sresp.get("state", "")).upper()
            if state in ("SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"):
                return state == "SUCCEEDED", f"nav terminal: {state}"
        time.sleep(1.0)
    _mcp_call_sync("nav", "cancel", {"run_id": run_id})
    return False, "nav timeout"


# ── Camera helpers ───────────────────────────────────────────────────────────
def _capture_photo(save_path: str) -> bool:
    """Capture a photo using the camera snapshot MCP tool and save to disk."""
    try:
        resp = _mcp_call_sync("camera", "snapshot", {})
        if not resp or "data" not in resp:
            log.warning("snapshot returned no data")
            return False
        import base64
        img_bytes = base64.b64decode(resp["data"])
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        with open(save_path, "wb") as f:
            f.write(img_bytes)
        log.info("photo saved: %s (%d bytes)", save_path, len(img_bytes))
        return True
    except Exception as e:  # noqa: BLE001
        log.error("photo capture failed: %s", e)
        return False


# ── Task execution ───────────────────────────────────────────────────────────
def _execute_patrol(task_id: str, waypoints: list[dict], save_dir: str,
                    settle_time_s: float):
    """Execute patrol in a background thread.

    Waypoints are sorted by nearest-neighbor from the robot's current
    position before execution begins.
    """
    try:
        # Sort waypoints: nearest first from current robot position
        waypoints = _nearest_neighbor_sort(waypoints)
        with _task_lock:
            _tasks[task_id]["state"] = "RUNNING"
            _tasks[task_id]["detail"] = (
                f"patrol with {len(waypoints)} waypoints (nearest-first)"
            )

        total = len(waypoints)
        for i, wp in enumerate(waypoints):
            # Check cancellation
            with _task_lock:
                if _tasks[task_id]["state"] != "RUNNING":
                    return

            wp_name = wp.get("name", f"waypoint_{i}")
            x, y = wp["x"], wp["y"]
            yaw = wp.get("yaw")
            log.info("[%s] navigating to waypoint %d/%d: %s (%.2f, %.2f)",
                     task_id, i + 1, total, wp_name, x, y)

            with _task_lock:
                _tasks[task_id]["current_waypoint"] = i
                _tasks[task_id]["detail"] = f"navigating to {wp_name}"

            # Navigate to waypoint
            def cancel_check():
                with _task_lock:
                    return _tasks[task_id]["state"] != "RUNNING"

            ok, msg = _nav_navigate_blocking(x, y, yaw, timeout_s=120.0,
                                             cancel_check=cancel_check)
            if not ok:
                log.warning("[%s] nav to %s failed: %s", task_id, wp_name, msg)
                with _task_lock:
                    _tasks[task_id]["detail"] = f"nav failed at {wp_name}: {msg}"
                continue  # try next waypoint

            # Settle time — let the robot stabilize
            if settle_time_s > 0:
                log.info("[%s] settling for %.1fs at %s", task_id, settle_time_s, wp_name)
                with _task_lock:
                    _tasks[task_id]["detail"] = f"settling at {wp_name}"
                time.sleep(settle_time_s)

            # Capture photo
            with _task_lock:
                if _tasks[task_id]["state"] != "RUNNING":
                    return

            photo_name = f"{i:03d}_{wp_name}.jpg"
            photo_path = os.path.join(save_dir, photo_name)
            log.info("[%s] capturing photo at %s", task_id, wp_name)
            with _task_lock:
                _tasks[task_id]["detail"] = f"capturing photo at {wp_name}"

            if _capture_photo(photo_path):
                with _task_lock:
                    _tasks[task_id]["completed_waypoints"] = i + 1
                    _tasks[task_id]["current_photo_path"] = photo_path
                    _tasks[task_id]["photos"].append(photo_path)
            else:
                log.warning("[%s] photo capture failed at %s", task_id, wp_name)
                with _task_lock:
                    _tasks[task_id]["completed_waypoints"] = i + 1
                    _tasks[task_id]["detail"] = f"photo failed at {wp_name}"

        # All waypoints visited
        with _task_lock:
            if _tasks[task_id]["state"] == "RUNNING":
                _tasks[task_id]["state"] = "SUCCEEDED"
                completed = _tasks[task_id]["completed_waypoints"]
                _tasks[task_id]["detail"] = (
                    f"patrol completed: {completed}/{total} waypoints visited"
                )
        log.info("[%s] patrol completed", task_id)

    except Exception as e:
        with _task_lock:
            _tasks[task_id]["state"] = "FAILED"
            _tasks[task_id]["detail"] = str(e)
        log.error("[%s] patrol failed: %s", task_id, e)


# ── MCP tools (typed against codegen Request/Response) ───────────────────────
from patrol_mcp import (  # noqa: E402
    Patrol_Request, Patrol_Response,
    GetPatrolStatus_Request, GetPatrolStatus_Response,
    CancelPatrol_Request, CancelPatrol_Response,
)


@patrol_skill.mcp("robonix/skill/patrol/patrol")
def patrol(req: Patrol_Request) -> Patrol_Response:
    """Start a multi-waypoint patrol with photo capture.

    Navigate to each waypoint using nav2, take a photo at each stop,
    and save images to disk.

    Args:
      - waypoint_x: list of x coordinates (map frame, metres)
      - waypoint_y: list of y coordinates (map frame, metres)
      - waypoint_yaw: list of yaw angles (radians, 0 = no constraint)
      - waypoint_names: optional human-readable names
      - save_dir: directory to save photos
      - settle_time_s: wait time after arriving before photo capture

    Returns run_id; use status() to poll progress, cancel() to abort.
    """
    # Validate inputs
    n = len(req.waypoint_x)
    if n == 0:
        return Patrol_Response(accepted=False, run_id="",
                               message="no waypoints provided")
    if len(req.waypoint_y) != n:
        return Patrol_Response(accepted=False, run_id="",
                               message="waypoint_x and waypoint_y length mismatch")

    # Build waypoint list
    waypoints = []
    for i in range(n):
        wp = {
            "x": float(req.waypoint_x[i]),
            "y": float(req.waypoint_y[i]),
            "yaw": float(req.waypoint_yaw[i]) if i < len(req.waypoint_yaw) and req.waypoint_yaw[i] != 0 else None,
            "name": req.waypoint_names[i] if i < len(req.waypoint_names) else f"wp{i}",
        }
        waypoints.append(wp)

    # Resolve save directory
    save_dir = req.save_dir
    if not os.path.isabs(save_dir):
        save_dir = os.path.join(
            os.environ.get("PATROL_DATA_DIR",
                           os.path.join(os.path.dirname(__file__), "..", "data")),
            save_dir,
        )
    os.makedirs(save_dir, exist_ok=True)

    task_id = "pat-" + uuid.uuid4().hex[:8]
    settle_time = float(req.settle_time_s) if req.settle_time_s > 0 else 2.0

    with _task_lock:
        _tasks[task_id] = {
            "state": "PENDING",
            "detail": f"patrol with {n} waypoints",
            "total_waypoints": n,
            "completed_waypoints": 0,
            "current_waypoint": -1,
            "current_photo_path": "",
            "photos": [],
            "created": time.time(),
        }

    thread = threading.Thread(
        target=_execute_patrol,
        args=(task_id, waypoints, save_dir, settle_time),
        daemon=True,
    )
    thread.start()

    return Patrol_Response(
        accepted=True,
        run_id=task_id,
        message=f"patrol started: {n} waypoints, saving to {save_dir}",
    )


@patrol_skill.mcp("robonix/skill/patrol/patrol/status")
def status(req: GetPatrolStatus_Request) -> GetPatrolStatus_Response:
    """Poll progress of a running patrol task.

    Args:
      - run_id: task ID from patrol() response (empty = most recent task)

    Returns:
      - known: whether the task exists
      - state: PENDING, RUNNING, SUCCEEDED, FAILED, or CANCELED
      - total_waypoints: total waypoints in this patrol
      - completed_waypoints: waypoints visited so far
      - current_waypoint: index of current target
      - current_photo_path: path to most recent photo
      - detail: human-readable status
    """
    run_id = req.run_id

    with _task_lock:
        if not run_id:
            if not _tasks:
                return GetPatrolStatus_Response(
                    known=False, state="PENDING", total_waypoints=0,
                    completed_waypoints=0, current_waypoint=-1,
                    current_photo_path="", detail="no tasks",
                )
            run_id = max(_tasks.keys(), key=lambda k: _tasks[k].get("created", 0))

        task = _tasks.get(run_id)
        if task is None:
            return GetPatrolStatus_Response(
                known=False, state="PENDING", total_waypoints=0,
                completed_waypoints=0, current_waypoint=-1,
                current_photo_path="", detail="no such task",
            )

        return GetPatrolStatus_Response(
            known=True,
            state=task["state"],
            total_waypoints=task["total_waypoints"],
            completed_waypoints=task["completed_waypoints"],
            current_waypoint=task["current_waypoint"],
            current_photo_path=task["current_photo_path"],
            detail=task["detail"],
        )


@patrol_skill.mcp("robonix/skill/patrol/patrol/cancel")
def cancel(req: CancelPatrol_Request) -> CancelPatrol_Response:
    """Abort an active patrol task. Idempotent.

    Args:
      - run_id: task ID to cancel (empty = cancel most recent running task)

    Returns:
      - ok: whether cancellation succeeded
      - message: human-readable result
    """
    run_id = req.run_id

    with _task_lock:
        if not run_id:
            for tid, task in _tasks.items():
                if task["state"] == "RUNNING":
                    task["state"] = "CANCELED"
                    task["detail"] = "canceled by user"
                    return CancelPatrol_Response(ok=True,
                                                 message=f"canceled task {tid}")
            return CancelPatrol_Response(ok=False, message="no running tasks")

        task = _tasks.get(run_id)
        if task is None:
            return CancelPatrol_Response(ok=False, message="no such task")

        if task["state"] in ("SUCCEEDED", "FAILED", "CANCELED"):
            return CancelPatrol_Response(
                ok=False, message=f"task already {task['state']}")

        task["state"] = "CANCELED"
        task["detail"] = "canceled by user"
        return CancelPatrol_Response(ok=True, message=f"canceled task {run_id}")


# ── lifecycle ────────────────────────────────────────────────────────────────
@patrol_skill.on_init
def init(cfg):
    """CMD_INIT: light. Just log."""
    log.info("CMD_INIT ok")
    return Ok()


@patrol_skill.on_activate
def activate():
    """CMD_ACTIVATE: resolve upstream contracts via atlas and start ROS."""
    global _mcp_endpoints, _ros_thread, _stop_evt
    if _mcp_endpoints:
        log.info("CMD_ACTIVATE — already resolved, no-op")
        return Ok()

    try:
        inputs = resolve_inputs()
        _mcp_endpoints = {
            "nav": inputs["nav_navigate"],
            "camera": inputs["camera_snap"],
        }
        # Start ROS spin thread for tf2 pose lookups
        _stop_evt.clear()
        _ros_thread = threading.Thread(target=_spin_ros, daemon=True)
        _ros_thread.start()
        log.info("CMD_ACTIVATE ok — dependencies resolved, ROS spinning")
        return Ok()
    except Exception as e:
        return Err(f"patrol skill activate failed: {e}")


@patrol_skill.on_deactivate
def deactivate():
    """CMD_DEACTIVATE: stop ROS and clear MCP clients."""
    global _mcp_clients, _mcp_endpoints, _ros_node, _ros_thread
    global _tf_buffer, _tf_listener
    _stop_evt.set()
    if _ros_thread:
        _ros_thread.join(timeout=3.0)
        _ros_thread = None
    if _ros_node is not None:
        try:
            _ros_node.destroy_node()
        except Exception:
            pass
        _ros_node = None
        _tf_buffer = None
        _tf_listener = None
    _mcp_clients.clear()
    _mcp_endpoints.clear()
    log.info("CMD_DEACTIVATE ok")
    return Ok()


def main() -> int:
    patrol_skill.run()
    # Cleanup ROS on exit
    if _ros_node is not None:
        _ros_node.destroy_node()
    try:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
