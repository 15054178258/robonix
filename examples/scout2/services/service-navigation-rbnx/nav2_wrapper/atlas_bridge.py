#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""nav2_wrapper_rbnx — atlas bridge (driver-init lifecycle).

Wraps system-installed nav2_bringup. Owns service/navigation/*.

Spawn order:
  1. start.sh launches THIS process — no nav2 spawn yet.
  2. main() starts the Driver server plus Navigate/Status/Cancel gRPC
     servicers and MCP tools, then registers with atlas.
  3. rbnx boot calls Driver(CMD_INIT, config_json).
  4. Init handler: pick params_file from config, spawn `ros2 launch
     nav2_bringup navigation_launch.py …`, wait for the navigate_to_pose
     action server to come up, declare navigate/status/cancel on atlas.

NavigateToPose action client uses the existing /odom + /map + /tf the
rest of the stack provides. Goals are tracked in an internal dict so
status() / cancel() work even after the goal has terminated.

Config (passed via Driver(CMD_INIT, config_json)):
    params_file      required; absolute or relative to the robot manifest
    bt_xml_file      optional; absolute or relative to the robot manifest
    use_sim_time     default false
    action_wait_s    default 45.0       — nav2 lifecycle takes a while
"""
from __future__ import annotations

import logging
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import grpc

from nav2_wrapper.diagnostics import classify_nav2_line, format_result_detail
from nav2_wrapper.configuration import (
    resolve_bt_xml_file,
    resolve_params_file,
    resolve_velocity_output_topic,
    scan_projection_config,
    validate_absolute_ros_topic,
)
from nav2_wrapper.speed_control import (
    SpeedDecision,
    SpeedSettings,
    decide_adjustment,
    decide_explicit,
    speed_settings,
)

logging.basicConfig(level=os.environ.get("NAV2_LOG_LEVEL", "INFO").upper(),
                    format="[nav2_wrapper] %(message)s")
log = logging.getLogger("nav2_wrapper")


def _pump_output(stream, tag: str) -> None:
    """Forward a child process's merged stdout/stderr into scribe via the
    package logger — one unified log stream, no side-car *.log file."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[%s] %s", tag, line)
            if tag == "nav2":
                diagnostic = classify_nav2_line(line)
                if diagnostic:
                    with _state_lock:
                        _nav_diagnostics.append(diagnostic)


def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        codegen = d / "rbnx-build" / "codegen"
        pg = codegen / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            sys.path.insert(0, str(pg))
            mt = codegen / "robonix_mcp_types"
            if mt.is_dir():
                sys.path.insert(0, str(mt))
            return
        d = d.parent


_ensure_proto_gen()

import navigation_pb2  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # noqa: E402
import soma_pb2  # noqa: E402
from navigation_mcp import (  # noqa: E402
    Navigate_Request as McpNavigateRequest,
    Navigate_Response as McpNavigateResponse,
    GetNavigationStatus_Request as McpStatusRequest,
    GetNavigationStatus_Response as McpStatusResponse,
    CancelNavigation_Request as McpCancelRequest,
    CancelNavigation_Response as McpCancelResponse,
    AdjustNavigationSpeed_Request as McpAdjustSpeedRequest,
    AdjustNavigationSpeed_Response as McpAdjustSpeedResponse,
    SetNavigationSpeedLimit_Request as McpSetSpeedLimitRequest,
    SetNavigationSpeedLimit_Response as McpSetSpeedLimitResponse,
    GetNavigationSpeedLimit_Request as McpGetSpeedLimitRequest,
    GetNavigationSpeedLimit_Response as McpGetSpeedLimitResponse,
)

# Current Robonix provider API (same one mapping_rbnx uses). The Service
# class owns atlas registration, the Driver(CMD_INIT/SHUTDOWN) lifecycle
# server, and heartbeat — so this package no longer talks to the raw
# AtlasStub (its old RegisterCapability RPC no longer exists).
from robonix_api import Service, Ok, Err, Deferred, ATLAS  # noqa: E402

CAP_ID = os.environ.get("ROBONIX_CAPABILITY_ID", "nav2")
NAMESPACE = "robonix/service/navigation"

# The provider. on_init (below) does the nav2 bring-up; nav.run() serves
# the Driver lifecycle + registers + heartbeats.
nav = Service(id=CAP_ID, namespace=NAMESPACE)


# ── shared state ─────────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_cap_id: str = CAP_ID
_pkg_root: Path = Path(__file__).resolve().parent.parent
_nav2_proc: subprocess.Popen | None = None
_velocity_guard_proc: subprocess.Popen | None = None
_scan_projector_proc: subprocess.Popen | None = None
_scan_deskew_proc: subprocess.Popen | None = None
_scan_filter_proc: subprocess.Popen | None = None
_initialized = False

# ROS2 client state (initialized inside Driver.Init after nav2 is alive)
_ros_node = None
_nav_action_client = None
_nav_action_ready = False
_NavigateToPose = None
_PoseStamped = None
_GoalStatus = None
_SpeedLimit = None
_nav_queue: "queue.Queue[tuple[str, dict]]" = queue.Queue()
_goal_states: dict[str, dict] = {}
_goal_handles: dict[str, object] = {}
_nav_diagnostics: deque[str] = deque(maxlen=12)
_last_run_id = ""
_speed_publisher = None
_guard_speed_publisher = None
_speed_lock = threading.Lock()
_speed_settings: SpeedSettings | None = None
_speed_percentage = 100.0
_session_speed_percentage = 100.0
_speed_scope = "session"
_speed_scope_run_id = ""
_GUARD_SPEED_LIMIT_TOPIC = "/robonix/navigation/speed_limit_guard"
_speed_subscriber_connected = False
# Whether nav2 (and therefore the TF tree it consumes) runs on /clock sim
# time. The wrapper's own rclpy node must match: it stamps goal poses with
# node.get_clock().now(), and if that clock is wall time while map->odom TF
# is published on sim time, every goal lookup hits a ~decades extrapolation
# error and the planner aborts. Set from cfg in init().
_USE_SIM_TIME = False

# Costmap state for goal reachability checking
_costmap_sub = None
_costmap_data = None
_costmap_lock = threading.Lock()
_costmap_resolution = 0.05
_costmap_origin_x = 0.0
_costmap_origin_y = 0.0
_costmap_width = 0
_costmap_height = 0
_LETHAL_OBSTACLE = 253
_INSPECTED_OBSTACLE = 252

# Object avoidance state - queries scene service for known objects
_OBJECT_AVOIDANCE_ENABLED = True
_OBJECT_SAFETY_MARGIN_M = 0.5  # 50cm safety margin around objects
_SCENE_OBJECTS_CACHE = {}  # {object_id: (x, y, radius)}
_SCENE_OBJECTS_LOCK = threading.Lock()
_SCENE_OBJECTS_CACHE_TIME = 0.0
_SCENE_OBJECTS_CACHE_TTL = 2.0  # Cache objects for 2 seconds


def _import_ros2() -> None:
    global _NavigateToPose, _PoseStamped, _GoalStatus, _SpeedLimit
    from geometry_msgs.msg import PoseStamped as RosPoseStamped  # type: ignore
    from nav2_msgs.action import NavigateToPose  # type: ignore
    from nav2_msgs.msg import SpeedLimit  # type: ignore
    try:
        from action_msgs.msg import GoalStatus  # type: ignore
        _GoalStatus = GoalStatus
    except ImportError:
        _GoalStatus = None
    _PoseStamped = RosPoseStamped
    _NavigateToPose = NavigateToPose
    _SpeedLimit = SpeedLimit


# ── costmap subscription for goal reachability checking ──────────────────────
def _costmap_callback(msg):
    """Update local costmap data from global_costmap topic."""
    global _costmap_data, _costmap_resolution, _costmap_origin_x, _costmap_origin_y
    global _costmap_width, _costmap_height
    with _costmap_lock:
        _costmap_resolution = msg.info.resolution
        _costmap_origin_x = msg.info.origin.position.x
        _costmap_origin_y = msg.info.origin.position.y
        _costmap_width = msg.info.width
        _costmap_height = msg.info.height
        _costmap_data = bytes(msg.data)


def _setup_costmap_subscriber(node) -> None:
    """Subscribe to global_costmap topic for goal reachability checking."""
    global _costmap_sub
    from nav_msgs.msg import OccupancyGrid  # type: ignore
    _costmap_sub = node.create_subscription(
        OccupancyGrid,
        '/global_costmap/costmap',
        _costmap_callback,
        10
    )
    log.info("subscribed to /global_costmap/costmap for goal reachability checking")


def _world_to_costmap(wx: float, wy: float) -> tuple[int, int]:
    """Convert world coordinates to costmap grid coordinates."""
    mx = int((wx - _costmap_origin_x) / _costmap_resolution)
    my = int((wy - _costmap_origin_y) / _costmap_resolution)
    return mx, my


def _costmap_to_world(mx: int, my: int) -> tuple[float, float]:
    """Convert costmap grid coordinates to world coordinates."""
    wx = mx * _costmap_resolution + _costmap_origin_x
    wy = my * _costmap_resolution + _costmap_origin_y
    return wx, wy


def _get_cost_at(wx: float, wy: float) -> int:
    """Get costmap cost at world coordinates. Returns -1 if out of bounds."""
    with _costmap_lock:
        if _costmap_data is None:
            return -1
        mx, my = _world_to_costmap(wx, wy)
        if mx < 0 or mx >= _costmap_width or my < 0 or my >= _costmap_height:
            return -1
        index = my * _costmap_width + mx
        if index < 0 or index >= len(_costmap_data):
            return -1
        return _costmap_data[index]


def _is_reachable(cost: int) -> bool:
    """Check if a cost value indicates a reachable location."""
    # FREE_SPACE (0), INSCRIBED_INFLATED_OBSTACLE (1-252) are reachable
    # LETHAL_OBSTACLE (253), NO_INFORMATION (255) are not reachable
    return 0 <= cost < _LETHAL_OBSTACLE


def _find_nearest_reachable(wx: float, wy: float, max_search_radius: float = 2.0) -> tuple[float, float]:
    """Find nearest reachable point within max_search_radius meters.
    
    Uses spiral search pattern to find the closest reachable cell.
    Returns the original point if already reachable or no reachable point found.
    """
    # Check if original point is reachable
    cost = _get_cost_at(wx, wy)
    if cost >= 0 and _is_reachable(cost):
        return wx, wy
    
    # Spiral search for nearest reachable point
    resolution = _costmap_resolution
    max_cells = int(max_search_radius / resolution)
    
    for radius in range(1, max_cells + 1):
        # Check cells at current radius
        best_dist = float('inf')
        best_point = None
        
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                # Only check cells on the perimeter of current radius
                if abs(dx) != radius and abs(dy) != radius:
                    continue
                
                check_x = wx + dx * resolution
                check_y = wy + dy * resolution
                check_cost = _get_cost_at(check_x, check_y)
                
                if check_cost >= 0 and _is_reachable(check_cost):
                    dist = math.sqrt(dx * dx + dy * dy) * resolution
                    if dist < best_dist:
                        best_dist = dist
                        best_point = (check_x, check_y)
        
        if best_point is not None:
            log.info(
                "adjusted unreachable goal (%.2f, %.2f) to reachable point (%.2f, %.2f), distance: %.2f m",
                wx, wy, best_point[0], best_point[1], best_dist
            )
            return best_point
    
    # No reachable point found, return original
    log.warning("no reachable point found within %.1f m of (%.2f, %.2f)", max_search_radius, wx, wy)
    return wx, wy


def _adjust_goal_if_needed(x: float, y: float) -> tuple[float, float]:
    """Adjust goal coordinates if they are on an obstacle.
    
    Returns adjusted coordinates that are guaranteed to be reachable.
    """
    if _costmap_data is None:
        log.warning("costmap not available, skipping goal adjustment")
        return x, y
    
    cost = _get_cost_at(x, y)
    if cost < 0:
        log.warning("goal (%.2f, %.2f) outside costmap bounds", x, y)
        return x, y
    
    if _is_reachable(cost):
        log.info("goal (%.2f, %.2f) is reachable (cost=%d)", x, y, cost)
        return x, y
    
    log.warning(
        "goal (%.2f, %.2f) is on obstacle (cost=%d), searching for nearest reachable point",
        x, y, cost
    )
    return _find_nearest_reachable(x, y)


# ── Object avoidance ─────────────────────────────────────────────────────────
def _query_scene_objects() -> dict:
    """Query scene service for known objects and cache them."""
    global _SCENE_OBJECTS_CACHE, _SCENE_OBJECTS_CACHE_TIME
    
    now = time.time()
    with _SCENE_OBJECTS_LOCK:
        if now - _SCENE_OBJECTS_CACHE_TIME < _SCENE_OBJECTS_CACHE_TTL:
            return dict(_SCENE_OBJECTS_CACHE)
    
    try:
        # Query scene service via atlas
        contract_id = "robonix/system/scene/list_objects"
        records = ATLAS.find_capability(contract_id=contract_id, transport="grpc")
        if not records:
            return {}
        
        connection = nav.connect_capability(
            records[0], contract_id=contract_id, transport="grpc"
        )
        endpoint = (connection.endpoint or "").strip()
        connection.close()
        if not endpoint:
            return {}
        
        # Import scene types
        import grpc
        from scene_mcp import ListObjects_Request, ListObjects_Response
        
        channel = grpc.insecure_channel(endpoint)
        try:
            grpc.channel_ready_future(channel).result(timeout=2.0)
            # Use MCP-style gRPC call
            from robonix_contracts_pb2_grpc import RobonixSystemSceneListObjectsStub
            stub = RobonixSystemSceneListObjectsStub(channel)
            
            # Create request
            request = ListObjects_Request()
            response = stub.ListObjects(request, timeout=2.0)
            
            objects = {}
            for obj in response.objects:
                if obj.label.lower() != "robot":
                    # Calculate object radius from bbox if available
                    radius = 0.3  # Default 30cm radius
                    if hasattr(obj, 'bbox') and obj.bbox:
                        radius = max(
                            math.hypot(obj.bbox.size_x * 0.5, obj.bbox.size_y * 0.5),
                            0.1  # Minimum 10cm
                        )
                    objects[obj.id] = (float(obj.x), float(obj.y), radius)
            
            with _SCENE_OBJECTS_LOCK:
                _SCENE_OBJECTS_CACHE = objects
                _SCENE_OBJECTS_CACHE_TIME = now
            
            return objects
        finally:
            channel.close()
    except Exception as e:
        log.debug("Failed to query scene objects: %s", e)
        return {}


def _check_too_close_to_objects(x: float, y: float, margin: float = None) -> tuple[bool, str]:
    """Check if a point is too close to any known object.
    
    Returns:
        (is_too_close, object_id_or_empty)
    """
    if not _OBJECT_AVOIDANCE_ENABLED:
        return False, ""
    
    if margin is None:
        margin = _OBJECT_SAFETY_MARGIN_M
    
    objects = _query_scene_objects()
    for obj_id, (obj_x, obj_y, obj_radius) in objects.items():
        distance = math.sqrt((x - obj_x) ** 2 + (y - obj_y) ** 2)
        min_distance = obj_radius + margin
        if distance < min_distance:
            return True, obj_id
    return False, ""


def _adjust_goal_for_objects(x: float, y: float, margin: float = None) -> tuple[float, float]:
    """Adjust goal coordinates to avoid being too close to objects.
    
    If the goal is too close to an object, find the nearest point that
    is at least (object_radius + margin) away from the object center.
    """
    if not _OBJECT_AVOIDANCE_ENABLED:
        return x, y
    
    if margin is None:
        margin = _OBJECT_SAFETY_MARGIN_M
    
    objects = _query_scene_objects()
    for obj_id, (obj_x, obj_y, obj_radius) in objects.items():
        distance = math.sqrt((x - obj_x) ** 2 + (y - obj_y) ** 2)
        min_distance = obj_radius + margin
        if distance < min_distance:
            # Goal is too close to this object, adjust it
            if distance < 0.01:  # Goal is at object center
                # Move to a point on the safety circle
                angle = math.atan2(y - obj_y, x - obj_x)
                new_x = obj_x + min_distance * math.cos(angle)
                new_y = obj_y + min_distance * math.sin(angle)
            else:
                # Move along the line from object to goal
                scale = min_distance / distance
                new_x = obj_x + (x - obj_x) * scale
                new_y = obj_y + (y - obj_y) * scale
            
            log.info(
                "adjusted goal to avoid object %s: (%.2f, %.2f) -> (%.2f, %.2f), "
                "object at (%.2f, %.2f), distance was %.2f, min_distance %.2f",
                obj_id, x, y, new_x, new_y, obj_x, obj_y, distance, min_distance,
            )
            return new_x, new_y
    
    return x, y


# ── atlas-driven dependency discovery ────────────────────────────────────────
# nav2 needs a few upstream data streams. We DO NOT hardcode which package
# provides them — we ask atlas for each contract and remap the topic into
# nav2 at launch time. This keeps the wrapper coupled to contracts only;
# whoever publishes them on this deploy is irrelevant.
#
# (config_key, contract_id, default_remap_target) — config_key is the
# string we look up in `cfg["topic_remap"]` so an operator can override
# any individual binding without disabling discovery.
_REQUIRED_DEPS: tuple[tuple[str, str, str], ...] = (
    # robonix/service/map/occupancy_grid → nav2 expects /map for the
    # global costmap's StaticLayer.
    ("map",   "robonix/service/map/occupancy_grid",  "/map"),
    # Consume the deployment's canonical odometry provider directly. Mapping
    # also consumes this stream, but must not re-declare the same ROS topic as
    # a second capability owner. Ranger currently pins this to ranger_chassis;
    # a future Mid360 LIO package can replace it through provider_ids.odom.
    ("odom",  "robonix/primitive/chassis/odom",       "/odom"),
)

# Optional deps: if present on atlas, we wire them; if absent, nav2 still
# launches and just won't have that observation source. Useful when the
# deploy has e.g. a 3D lidar but nav2's costmap is configured around 2D
# scan — the operator may legitimately not provide one.
_OPTIONAL_DEPS: tuple[tuple[str, str, str], ...] = (
    # 2D scan for ObstacleLayer (some configs); 3D lidar for VoxelLayer.
    ("scan",        "robonix/primitive/lidar/lidar",   "/scan"),
    ("scan_cloud",  "robonix/primitive/lidar/lidar3d", "/scanner/cloud"),
)


def _resolve_dep(contract_id: str, provider_id: str = "") -> str | None:
    """Ask atlas which ROS2 topic backs `contract_id`; return it or None.

    Uses the same ATLAS.find_capability + connect_capability path mapping
    uses — we want the resolved topic string, and connecting also records
    nav2 as a consumer of that contract. The Channel is closed immediately."""
    recs = ATLAS.find_capability(
        contract_id=contract_id, transport="ros2", provider_id=provider_id
    )
    if not recs:
        return None
    rec = recs[0]
    try:
        ch = nav.connect_capability(rec, contract_id=contract_id, transport="ros2")
    except Exception as e:  # noqa: BLE001
        log.warning("connect %s/%s failed: %s", rec.provider_id, contract_id, e)
        return None
    endpoint = (ch.endpoint or "").strip()
    ch.close()
    return endpoint or None


def _build_remap_args(cfg: dict) -> tuple[list[str], list[str]]:
    """Return (remap_args, missing_required).
    remap_args is a list of `from:=to` strings ready to pass to ros2 launch.
    missing_required is a list of contract_ids that should have been there
    but weren't — caller decides whether to defer / degrade / fail."""
    overrides = dict(cfg.get("topic_remap", {}) or {})
    providers = dict(cfg.get("provider_ids", {}) or {})
    remap_args: list[str] = []
    missing: list[str] = []

    for key, contract_id, default_target in _REQUIRED_DEPS:
        if key in overrides:
            ep = str(overrides[key])
        else:
            ep = _resolve_dep(contract_id, str(providers.get(key) or "")) or ""
        if not ep:
            missing.append(contract_id)
            continue
        # ros2 launch syntax: pass remaps via the ros-args mechanism. The
        # nav2_bringup composable nodes pick them up via DeclareLaunchArgument.
        # Cleanest path: rewrite a temp params file with the resolved topic
        # name (the params YAML is where most nav2 nodes look for it).
        remap_args.append(f"{key}:={ep}")
        log.info("resolved %s → %s = %s", contract_id, default_target, ep)

    for key, contract_id, default_target in _OPTIONAL_DEPS:
        if key in overrides:
            ep = str(overrides[key])
        else:
            ep = _resolve_dep(contract_id, str(providers.get(key) or "")) or ""
        if ep:
            remap_args.append(f"{key}:={ep}")
            log.info("resolved (optional) %s → %s = %s", contract_id, default_target, ep)
        else:
            log.info("optional dep %s not on atlas — skipping", contract_id)

    return remap_args, missing


def _binding_value(bindings: list[str], key: str) -> str:
    """Return the resolved Atlas endpoint for a named dependency binding."""
    prefix = f"{key}:="
    for binding in bindings:
        if binding.startswith(prefix):
            return binding[len(prefix):]
    return ""


def _uses_projected_scan(cfg: dict) -> bool:
    """Whether the deployment explicitly enables PointCloud2 projection."""
    return bool(scan_projection_config(cfg)["enabled"])


def _kill_scan_projector() -> None:
    global _scan_projector_proc, _scan_deskew_proc, _scan_filter_proc
    procs = (_scan_projector_proc, _scan_deskew_proc, _scan_filter_proc)
    _scan_projector_proc = None
    _scan_deskew_proc = None
    _scan_filter_proc = None
    for proc in procs:
        if proc is None:
            continue
        try:
            # ros2 run is a Python parent that spawns the actual ROS binary.
            # The parent may exit while its child still owns this session's
            # process group, so target the known PGID directly.
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        if proc.poll() is None:
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def _prepare_scan(cfg: dict, bindings: list[str]) -> list[str]:
    """Provide a LaserScan from lidar3d when the deployment requests it."""
    global _scan_projector_proc, _scan_deskew_proc, _scan_filter_proc
    if not _uses_projected_scan(cfg) or _binding_value(bindings, "scan"):
        return bindings

    projection = scan_projection_config(cfg)
    cloud_topic = _binding_value(bindings, "scan_cloud")
    if not cloud_topic:
        raise RuntimeError(
            "scan_projection requires robonix/primitive/lidar/lidar3d"
        )
    if _scan_projector_proc is not None and _scan_projector_proc.poll() is None:
        return [*bindings, "scan:=/scanner/scan"]

    _, footprint_radius, soma_base_frame = _soma_footprint_info()
    target_frame = str(projection["target_frame"] or soma_base_frame)
    range_min = footprint_radius + float(projection["self_filter_margin_m"])
    projector_cloud_topic = cloud_topic
    if bool(projection["deskewing"]):
        projector_cloud_topic = f"{cloud_topic.rstrip('/')}/deskewed"
        deskew_args = [
            "ros2", "run", "rtabmap_util", "lidar_deskewing",
            "--ros-args",
            "-r", "__node:=robonix_nav_lidar_deskewing",
            "-r", f"input_cloud:={cloud_topic}",
            "-p", f"fixed_frame_id:={projection['deskew_fixed_frame']}",
            "-p", f"wait_for_transform:={projection['deskew_wait_for_transform_s']}",
            "-p", "slerp:=true",
        ]
        try:
            _scan_deskew_proc = subprocess.Popen(
                deskew_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except FileNotFoundError as exc:
            raise RuntimeError(
                "scan_deskewing requires ros-humble-rtabmap-util"
            ) from exc
        threading.Thread(
            target=_pump_output,
            args=(_scan_deskew_proc.stdout, "lidar_deskewing"),
            daemon=True,
        ).start()
        time.sleep(0.25)
        if _scan_deskew_proc.poll() is not None:
            _kill_scan_projector()
            raise RuntimeError("lidar_deskewing exited during startup")

    args = [
        "ros2", "run", "pointcloud_to_laserscan", "pointcloud_to_laserscan_node",
        "--ros-args",
        "-r", "__node:=robonix_pointcloud_to_laserscan",
        "-r", f"cloud_in:={projector_cloud_topic}",
        "-r", "scan:=/scanner/scan_raw",
        "-p", f"target_frame:={target_frame}",
        "-p", f"transform_tolerance:={projection['transform_tolerance_s']}",
        "-p", f"min_height:={projection['min_height_m']}",
        "-p", f"max_height:={projection['max_height_m']}",
        "-p", f"range_min:={range_min:.3f}",
        "-p", f"range_max:={projection['range_max_m']}",
        "-p", "use_inf:=true",
    ]
    try:
        _scan_projector_proc = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except FileNotFoundError as exc:
        _kill_scan_projector()
        raise RuntimeError(
            "scan_projection requires "
            "ros-humble-pointcloud-to-laserscan"
        ) from exc
    threading.Thread(
        target=_pump_output, args=(_scan_projector_proc.stdout, "pointcloud_to_laserscan"),
        daemon=True,
    ).start()
    time.sleep(0.25)
    if _scan_projector_proc.poll() is not None:
        _kill_scan_projector()
        raise RuntimeError("pointcloud_to_laserscan exited during startup")
    _scan_filter_proc = subprocess.Popen(
        [sys.executable, "-m", "nav2_wrapper.scan_filter"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output, args=(_scan_filter_proc.stdout, "scan_filter"), daemon=True
    ).start()
    time.sleep(0.25)
    if _scan_filter_proc.poll() is not None:
        _kill_scan_projector()
        raise RuntimeError("scan speckle filter exited during startup")
    log.info(
        "projecting %s to /scanner/scan_raw then filtered /scanner/scan "
        "for Nav2 (target=%s, deskew=%s, self-filter range_min=%.3fm)",
        projector_cloud_topic,
        target_frame,
        bool(projection["deskewing"]),
        range_min,
    )
    return [*bindings, "scan:=/scanner/scan"]


# ── nav2 subprocess management ───────────────────────────────────────────────
_footprint_cache: tuple[str, float, str] | None = None


def _soma_footprint_info() -> tuple[str, float, str]:
    """Resolve the footprint polygon and circumscribed radius through Soma."""
    global _footprint_cache
    if _footprint_cache is not None:
        return _footprint_cache
    contract_id = "robonix/system/soma/footprint"
    records = ATLAS.find_capability(contract_id=contract_id, transport="grpc")
    if not records:
        raise RuntimeError(f"required capability unavailable: {contract_id}")

    connection = nav.connect_capability(
        records[0], contract_id=contract_id, transport="grpc"
    )
    endpoint = (connection.endpoint or "").strip()
    connection.close()
    if not endpoint:
        raise RuntimeError(f"{contract_id} resolved to an empty endpoint")

    channel = grpc.insecure_channel(endpoint)
    try:
        grpc.channel_ready_future(channel).result(timeout=10)
        response = contracts_grpc.RobonixSystemSomaFootprintStub(channel).GetFootprint(
            soma_pb2.GetFootprint_Request(), timeout=10
        )
    finally:
        channel.close()

    if not response.base_frame:
        raise ValueError("Soma footprint response has no base_frame")
    if len(response.points) < 3:
        raise ValueError("Soma footprint requires at least three polygon points")
    points = []
    radius = 0.0
    for point in response.points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            raise ValueError("Soma footprint contains non-finite point coordinates")
        points.append(f"[{point.x:.6f}, {point.y:.6f}]")
        radius = max(radius, math.hypot(point.x, point.y))
    value = "[ " + ", ".join(points) + " ]"
    log.info(
        "resolved Soma footprint base_frame=%s vertices=%d inscribed=%.3fm",
        response.base_frame,
        len(response.points),
        response.inscribed_radius_m,
    )
    _footprint_cache = (value, radius, response.base_frame)
    return _footprint_cache


def _soma_footprint() -> str:
    return _soma_footprint_info()[0]


def _materialize_params(cfg: dict, bindings: list[str]) -> tuple[str, list[str]]:
    """Fill Atlas topic and Soma body tokens in target-specific profiles."""
    source = resolve_params_file(cfg)
    text = source.read_text(encoding="utf-8")
    if "__ROBONIX_" not in text:
        return str(source), bindings

    resolved = {}
    for item in bindings:
        key, sep, value = item.partition(":=")
        if sep:
            resolved[key] = value

    bt_xml = resolve_bt_xml_file(cfg)
    replacements = {
        "__ROBONIX_MAP_TOPIC__": resolved.get("map", ""),
        "__ROBONIX_ODOM_TOPIC__": resolved.get("odom", ""),
        "__ROBONIX_SCAN_TOPIC__": resolved.get("scan", ""),
        "__ROBONIX_SCAN_CLOUD_TOPIC__": resolved.get("scan_cloud", ""),
        "__ROBONIX_BT_XML__": str(bt_xml) if bt_xml else "",
    }
    if "__ROBONIX_FOOTPRINT__" in text:
        replacements["__ROBONIX_FOOTPRINT__"] = _soma_footprint()

    for token, value in replacements.items():
        if token in text:
            if not value:
                raise RuntimeError(f"cannot materialize {source.name}: {token} is unresolved")
            text = text.replace(token, value)

    unresolved = sorted(set(re.findall(r"__ROBONIX_[A-Z_]+__", text)))
    if unresolved:
        raise RuntimeError(
            f"cannot materialize {source.name}: unresolved tokens {unresolved}"
        )

    runtime_dir = _pkg_root / "rbnx-build" / "runtime"
    runtime_dir.mkdir(parents=True, exist_ok=True)
    target = runtime_dir / f"nav2_params_{_cap_id}.yaml"
    target.write_text(text, encoding="utf-8")
    log.info("materialized nav2 params %s -> %s", source, target)
    # Target-specific profiles consume Atlas bindings inside the generated
    # params file. They must not be passed as undeclared launch arguments.
    return str(target), []


def _materialize_guarded_launch() -> str:
    """Patch the distro launch so every Nav2 velocity crosses our final guard."""
    from ament_index_python.packages import get_package_share_directory  # type: ignore

    source = Path(get_package_share_directory("nav2_bringup")) / "launch" / "navigation_launch.py"
    text = source.read_text(encoding="utf-8")
    old_behavior = "remappings=remappings)"
    behavior_marker = "package='nav2_behaviors'"
    search_from = 0
    for _ in range(2):
        behavior_start = text.index(behavior_marker, search_from)
        behavior_end = text.index("package='nav2_bt_navigator'", behavior_start)
        behavior = text[behavior_start:behavior_end]
        if behavior.count(old_behavior) != 1:
            raise RuntimeError("unsupported nav2 behavior_server launch layout")
        behavior = behavior.replace(
            old_behavior,
            "remappings=remappings + [('cmd_vel', 'cmd_vel_guard_input')])",
        )
        text = text[:behavior_start] + behavior + text[behavior_end:]
        search_from = behavior_end
    old_smoother = "[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])"
    new_smoother = "[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_guard_input')])"
    if text.count(old_smoother) != 2:
        raise RuntimeError("unsupported nav2 velocity_smoother launch layout")
    text = text.replace(old_smoother, new_smoother)
    target = _pkg_root / "rbnx-build" / "runtime" / "guarded_navigation_launch.py"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    return str(target)


def _spawn_velocity_guard(cfg: dict) -> None:
    global _velocity_guard_proc
    output_topic = resolve_velocity_output_topic(cfg)
    settings = _speed_settings
    if settings is None:
        raise RuntimeError("dynamic speed settings are not initialized")
    env = os.environ.copy()
    env.update({
        # Always pass a validated, fully-qualified topic explicitly.  This
        # prevents the child from inheriting an empty/relative value and lets
        # deployments route guarded output to a non-motion sink until their
        # physical motion gate is deliberately enabled.
        "ROBONIX_VELOCITY_OUTPUT_TOPIC": output_topic,
        "ROBONIX_NAV_MAX_LINEAR_SPEED_MPS": str(
            settings.max_linear_speed_mps
        ),
        "ROBONIX_NAV_DEFAULT_SPEED_PERCENTAGE": str(
            settings.default_percentage
        ),
        "ROBONIX_NAV_GUARD_SPEED_LIMIT_TOPIC": _GUARD_SPEED_LIMIT_TOPIC,
        "ROBONIX_NAV_TRACE_DIR": str(
            cfg.get("trajectory_log_dir", _pkg_root / "rbnx-build" / "data" / "trajectories")
        ),
        "ROBONIX_GUARD_TERMINAL_XY_M": str(cfg.get("guard_terminal_xy_m", 0.45)),
        "ROBONIX_GUARD_TERMINAL_TIMEOUT_S": str(cfg.get("guard_terminal_timeout_s", 15.0)),
        "ROBONIX_GUARD_NO_PROGRESS_S": str(cfg.get("guard_no_progress_s", 3.0)),
        "ROBONIX_GUARD_GLOBAL_TIMEOUT_S": str(cfg.get("guard_global_spin_timeout_s", 25.0)),
        "ROBONIX_GUARD_GLOBAL_ROTATION_RAD": str(cfg.get("guard_global_spin_limit_rad", 6.783185307)),
    })
    _velocity_guard_proc = subprocess.Popen(
        [sys.executable, "-m", "nav2_wrapper.velocity_guard"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=env,
    )
    threading.Thread(
        target=_pump_output, args=(_velocity_guard_proc.stdout, "velocity_guard"), daemon=True
    ).start()


def _spawn_nav2(cfg: dict, remap_args: list[str]) -> None:
    global _nav2_proc
    params_file, launch_remaps = _materialize_params(cfg, remap_args)
    _spawn_velocity_guard(cfg)
    launch_file = _materialize_guarded_launch()
    use_sim_time = "true" if cfg.get("use_sim_time", False) else "false"
    args = [
        "ros2", "launch", launch_file,
        f"use_sim_time:={use_sim_time}",
        f"params_file:={params_file}",
    ]
    # Topic remaps from atlas resolution arrive as launch-arg-shaped
    # `<key>:=<resolved-topic>` pairs. The launch file translates them
    # into ros2 remap ops via `<set_remap>` blocks; for keys the launch
    # doesn't know about we still pass them — no-op if unused. (Future:
    # rewrite the params YAML with substitutions for nodes that read
    # topic names from params rather than via remap.)
    args.extend(launch_remaps)
    log.info("spawning nav2 (params=%s, remaps=%s)", params_file, launch_remaps)
    _nav2_proc = subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(target=_pump_output, args=(_nav2_proc.stdout, "nav2"),
                     daemon=True).start()


def _kill_nav2() -> None:
    global _velocity_guard_proc
    _kill_scan_projector()
    p = _nav2_proc
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            p.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    guard = _velocity_guard_proc
    _velocity_guard_proc = None
    if guard is not None and guard.poll() is None:
        try:
            os.killpg(os.getpgid(guard.pid), signal.SIGTERM)
            guard.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(guard.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


# ── ROS2 wiring (started after nav2 is alive) ────────────────────────────────
def _publish_speed_limit(percentage: float | None = None) -> None:
    """Publish one limit to Nav2 and the independent final velocity guard."""
    publisher = _speed_publisher
    guard_publisher = _guard_speed_publisher
    node = _ros_node
    if (
        publisher is None
        or guard_publisher is None
        or node is None
        or _SpeedLimit is None
    ):
        raise RuntimeError("navigation speed publisher is not initialized")
    if percentage is None:
        with _speed_lock:
            percentage = _speed_percentage
    message = _SpeedLimit()
    message.header.stamp = node.get_clock().now().to_msg()
    message.percentage = True
    message.speed_limit = float(percentage)
    publisher.publish(message)
    guard_publisher.publish(message)


def _speed_channels_available() -> bool:
    """Require both Nav2's controller and the final guard to receive limits."""
    return bool(
        _speed_publisher is not None
        and _guard_speed_publisher is not None
        and _speed_publisher.get_subscription_count() > 0
        and _guard_speed_publisher.get_subscription_count() > 0
    )


def _refresh_speed_subscriber() -> None:
    """Reapply once when Controller Server reconnects, never on every tick."""
    global _speed_subscriber_connected
    if _speed_publisher is None:
        return
    connected = _speed_publisher.get_subscription_count() > 0
    if not connected:
        _speed_subscriber_connected = False
        return
    if _speed_subscriber_connected:
        return
    try:
        _publish_speed_limit()
        _speed_subscriber_connected = True
    except Exception as error:  # noqa: BLE001
        log.warning("speed-limit reconnect refresh failed: %s", error)


def _start_ros2_thread() -> None:
    """Spin a rclpy node + ActionClient. Re-entrant: only acts once."""
    def _run():
        global _ros_node, _nav_action_client, _nav_action_ready
        global _speed_publisher, _guard_speed_publisher
        import rclpy  # type: ignore
        from rclpy.executors import MultiThreadedExecutor  # type: ignore
        from rclpy.action import ActionClient  # type: ignore
        from rclpy.parameter import Parameter  # type: ignore

        rclpy.init(args=None)
        # use_sim_time must match nav2 / the TF tree (see _USE_SIM_TIME) so
        # node.get_clock() — which timestamps goal poses — is on the same
        # clock domain as the map->odom transform.
        node = rclpy.create_node(
            "nav2_wrapper_atlas_bridge",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, _USE_SIM_TIME)
            ],
        )
        _ros_node = node
        _import_ros2()
        settings = _speed_settings
        if settings is None:
            raise RuntimeError("dynamic speed settings are not initialized")
        _nav_action_client = ActionClient(node, _NavigateToPose, "navigate_to_pose")
        _speed_publisher = node.create_publisher(
            _SpeedLimit, settings.topic, 10
        )
        _guard_speed_publisher = node.create_publisher(
            _SpeedLimit,
            _GUARD_SPEED_LIMIT_TOPIC,
            10,
        )
        # Setup costmap subscriber for goal reachability checking
        _setup_costmap_subscriber(node)
        node.create_timer(1.0, _refresh_speed_subscriber)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        log.info(
            "rclpy node up; waiting on navigate_to_pose and speed-limit subscriber"
        )
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            # Drain goals queued by Navigate gRPC handler.
            while True:
                try:
                    gid, payload = _nav_queue.get_nowait()
                except queue.Empty:
                    break
                _dispatch_goal(node, gid, payload)
    threading.Thread(target=_run, daemon=True).start()


def _wait_for_action(timeout_s: float) -> bool:
    """Block until `navigate_to_pose` is ready (post-Init nav2 lifecycle bring-up)."""
    global _nav_action_ready
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _nav_action_client is not None and _nav_action_client.wait_for_server(timeout_sec=0.5):
            _nav_action_ready = True
            return True
        time.sleep(0.5)
    return False


def _wait_for_speed_subscriber(timeout_s: float) -> bool:
    """Wait until Controller Server is subscribed, then apply the default."""
    global _speed_subscriber_connected
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _speed_channels_available():
            _publish_speed_limit()
            _speed_subscriber_connected = True
            return True
        time.sleep(0.1)
    return False


def _make_pose(node, frame_id: str, x: float, y: float, yaw: float):
    g = _PoseStamped()
    g.header.frame_id = frame_id
    # Leave the goal stamp at 0 (the message default). A zero stamp tells
    # tf2 "use the LATEST available transform" instead of requiring the
    # frame_id->costmap transform at one exact instant. This is the
    # environment-agnostic fix for the "goal aborts with Extrapolation
    # Error" failure:
    #   - webots/sim: node clock and the map->odom TF can sit in different
    #     clock domains; a "now()" stamp lands decades away from the TF.
    #   - real robot: use_sim_time is false, but sensor/SLAM TF still lags
    #     wall-clock "now" by tens of ms, so a "now()" stamp can fall past
    #     the newest TF and extrapolate into the future.
    # Stamping 0 sidesteps both — the planner transforms against whatever
    # TF is currently buffered, which is exactly what "go to this pose"
    # means. (node is kept use_sim_time-consistent for action timing, but
    # the goal transform no longer depends on clock alignment at all.)
    g.pose.position.x = float(x)
    g.pose.position.y = float(y)
    g.pose.position.z = 0.0
    g.pose.orientation.z = math.sin(yaw / 2.0)
    g.pose.orientation.w = math.cos(yaw / 2.0)
    return g


def _goal_status_name(status: int) -> str:
    if _GoalStatus is None:
        return str(int(status))
    g = _GoalStatus
    m = {
        int(g.STATUS_UNKNOWN):  "UNKNOWN",
        int(g.STATUS_ACCEPTED): "ACCEPTED",
        int(g.STATUS_EXECUTING): "EXECUTING",
        int(g.STATUS_CANCELING): "CANCELING",
        int(g.STATUS_SUCCEEDED): "SUCCEEDED",
        int(g.STATUS_CANCELED): "CANCELED",
        int(g.STATUS_ABORTED):  "ABORTED",
    }
    return m.get(int(status), str(int(status)))


def _canonical_state(nav2_state: str) -> str:
    """Map Nav2 action result/status names to executor async state names."""
    return {
        "UNKNOWN": "RUNNING",
        "ACCEPTED": "RUNNING",
        "EXECUTING": "RUNNING",
        "CANCELING": "RUNNING",
        "SUCCEEDED": "SUCCEEDED",
        "CANCELED": "CANCELED",
        "ABORTED": "FAILED",
    }.get(nav2_state, "FAILED")


def _resolve_run_id(run_id: str) -> str:
    """Use the explicit run id, or fall back to the most recent navigation run."""
    return run_id or _last_run_id


def _active_speed_run_id(run_id: str) -> str:
    """Resolve and validate the goal that owns a non-persistent speed change."""
    with _state_lock:
        resolved = _resolve_run_id(run_id)
        state = _goal_states.get(resolved)
        if state is None:
            raise ValueError("unknown run_id")
        if state.get("state") not in {"PENDING", "RUNNING"}:
            raise ValueError(
                f"run is already {state.get('state', 'terminal').lower()}"
            )
    return resolved


def _restore_speed_after_run(run_id: str) -> None:
    """Restore the session limit when its goal-scoped override terminates."""
    global _speed_percentage, _speed_scope, _speed_scope_run_id
    global _speed_subscriber_connected
    with _speed_lock:
        if _speed_scope != "goal" or _speed_scope_run_id != run_id:
            return
        _speed_percentage = _session_speed_percentage
        _speed_scope = "session"
        _speed_scope_run_id = ""
        try:
            _publish_speed_limit(_speed_percentage)
        except Exception as error:  # noqa: BLE001
            _speed_subscriber_connected = False
            log.warning(
                "speed reset after navigation run %s will retry: %s",
                run_id,
                error,
            )
    log.info(
        "restored session navigation speed after run %s: %.1f%%",
        run_id,
        _speed_percentage,
    )


def _goal_response_cb(fut, gid: str):
    try:
        gh = fut.result()
    except Exception as e:  # noqa: BLE001
        with _state_lock:
            previous = _goal_states.get(gid, {})
            canceled = bool(previous.get("cancel_requested"))
            _goal_states[gid] = {
                "state": "CANCELED" if canceled else "FAILED",
                "detail": "canceled before acceptance" if canceled else str(e),
            }
        _restore_speed_after_run(gid)
        return
    if not gh.accepted:
        with _state_lock:
            previous = _goal_states.get(gid, {})
            canceled = bool(previous.get("cancel_requested"))
            _goal_states[gid] = {
                "state": "CANCELED" if canceled else "FAILED",
                "detail": "canceled before acceptance" if canceled else "goal rejected",
            }
        _restore_speed_after_run(gid)
        return
    with _state_lock:
        previous = _goal_states.get(gid, {})
        cancel_requested = bool(previous.get("cancel_requested"))
        _goal_handles[gid] = gh
        _goal_states[gid] = {
            "state": "RUNNING",
            "detail": (
                "goal accepted; forwarding queued cancel"
                if cancel_requested
                else "goal accepted"
            ),
            "cancel_requested": cancel_requested,
        }
    res_fut = gh.get_result_async()
    res_fut.add_done_callback(lambda f: _result_cb(f, gid))
    if cancel_requested:
        _issue_cancel(gh, gid)


def _cancel_response_cb(fut, gid: str) -> None:
    try:
        response = fut.result()
        accepted = bool(getattr(response, "goals_canceling", []))
        detail = (
            "cancel accepted by Nav2"
            if accepted
            else "cancel rejected by Nav2; goal remains active"
        )
    except Exception as error:  # noqa: BLE001
        accepted = False
        detail = f"cancel response failed; goal may remain active: {error}"
    with _state_lock:
        state = _goal_states.get(gid)
        if state is not None and state.get("state") == "RUNNING":
            state["detail"] = detail
            state["cancel_requested"] = accepted


def _issue_cancel(goal_handle, gid: str) -> tuple[bool, str]:
    try:
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(lambda f, g=gid: _cancel_response_cb(f, g))
    except Exception as error:  # noqa: BLE001
        with _state_lock:
            state = _goal_states.get(gid)
            if state is not None:
                state["detail"] = f"cancel request failed; goal may remain active: {error}"
                state["cancel_requested"] = False
        return False, f"cancel failed: {error}"
    return True, "cancel requested"


def _feedback_cb(message, gid: str) -> None:
    """Keep progress context because Humble NavigateToPose has an empty result."""
    feedback = message.feedback
    pose = feedback.current_pose.pose.position
    summary = {
        "distance_remaining": float(feedback.distance_remaining),
        "recoveries": int(feedback.number_of_recoveries),
        "x": float(pose.x),
        "y": float(pose.y),
    }
    with _state_lock:
        state = _goal_states.get(gid)
        if state is not None:
            state["feedback"] = summary


def _result_cb(fut, gid: str):
    try:
        res = fut.result()
        st_name = _goal_status_name(getattr(res, "status", -1))
        state = _canonical_state(st_name)
        with _state_lock:
            previous = _goal_states.get(gid, {})
            diagnostics = list(_nav_diagnostics) if state == "FAILED" else []
            _goal_states[gid] = {
                "state": state,
                "detail": format_result_detail(
                    st_name, previous.get("feedback"), diagnostics
                ),
            }
            _goal_handles.pop(gid, None)
    except Exception as e:  # noqa: BLE001
        with _state_lock:
            _goal_states[gid] = {"state": "FAILED", "detail": str(e)}
            _goal_handles.pop(gid, None)
    _restore_speed_after_run(gid)


def _dispatch_goal(node, gid: str, payload: dict):
    with _state_lock:
        current = _goal_states.get(gid)
        if current is None:
            return
        if current.get("state") == "CANCELED" and current.get("cancel_requested"):
            return
        # Claim the queue entry while holding the same lock used by cancel.
        # A cancel racing after this point is latched and forwarded as soon as
        # Nav2 returns the goal handle; it must not look terminal prematurely.
        current["state"] = "RUNNING"
        current["detail"] = "dispatching goal"
    pose = _make_pose(node, payload["frame_id"], payload["x"], payload["y"], payload["yaw"])
    goal_msg = _NavigateToPose.Goal()
    goal_msg.pose = pose
    if _nav_action_client is None or not _nav_action_ready:
        with _state_lock:
            _goal_states[gid] = {"state": "FAILED",
                                 "detail": "nav action server not ready"}
        _restore_speed_after_run(gid)
        return
    with _state_lock:
        _nav_diagnostics.clear()
    send_future = _nav_action_client.send_goal_async(
        goal_msg, feedback_callback=lambda msg, g=gid: _feedback_cb(msg, g)
    )
    send_future.add_done_callback(lambda f, g=gid: _goal_response_cb(f, g))
    with _state_lock:
        previous = _goal_states.get(gid, {})
        cancel_requested = bool(previous.get("cancel_requested"))
        _goal_states[gid] = {
            "state": "RUNNING",
            "detail": (
                "goal sent; cancel queued until acceptance"
                if cancel_requested
                else "goal sent"
            ),
            "cancel_requested": cancel_requested,
        }


# ── lifecycle (Driver CMD_INIT / CMD_SHUTDOWN via robonix_api.Service) ────────
@nav.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. rbnx delivers the deploy config here via
    Driver(CMD_INIT, config_json) — the only sanctioned config path (never
    disk / env). Brings up nav2:

      1. Resolve upstream deps (map, odom, optional scan/cloud) from atlas
         by contract — never hardcoded topic names. Missing a REQUIRED dep
         → Deferred (rbnx retries once the upstream provider registers),
         so we never spawn a half-wired nav2.
      2. Spawn nav2_bringup with the resolved remaps + the params profile.
      3. Bring up the rclpy node + navigate_to_pose ActionClient and wait
         for nav2's lifecycle to advertise the action server.

    The navigate/status/cancel gRPC and MCP interfaces are hosted + declared by
    run() (see attach_grpc_servicer below); each guards on `_ros_node`, so
    a call landing before nav2 is ready returns a clean 'not initialized'."""
    global _initialized, _speed_settings, _speed_percentage
    global _session_speed_percentage, _speed_scope, _speed_scope_run_id
    global _speed_subscriber_connected
    with _state_lock:
        if _initialized:
            return Ok()

    # Reject an ambiguous velocity destination before Atlas discovery or any
    # scan/Nav2 child is started.  _spawn_velocity_guard resolves it again and
    # explicitly passes the validated value into the child environment.
    try:
        resolve_velocity_output_topic(cfg)
    except (TypeError, ValueError) as error:
        return Err(f"invalid velocity_output_topic: {error}")
    try:
        settings = speed_settings(cfg)
        validate_absolute_ros_topic(settings.topic, "dynamic_speed.topic")
    except (TypeError, ValueError) as error:
        return Err(f"invalid dynamic_speed config: {error}")

    with _speed_lock:
        _speed_settings = settings
        _speed_percentage = settings.default_percentage
        _session_speed_percentage = settings.default_percentage
        _speed_scope = "session"
        _speed_scope_run_id = ""
        _speed_subscriber_connected = False

    action_wait = float(cfg.get("action_wait_s", 45.0))

    remap_args, missing = _build_remap_args(cfg)
    if missing:
        return Deferred(
            f"missing required atlas contracts: {missing} "
            f"(awaiting upstream provider)"
        )

    global _USE_SIM_TIME
    _USE_SIM_TIME = bool(cfg.get("use_sim_time", False))

    try:
        _spawn_nav2(cfg, _prepare_scan(cfg, remap_args))
    except Exception as e:  # noqa: BLE001
        _kill_nav2()
        return Err(f"spawn nav2 failed: {e}")

    _start_ros2_thread()
    if not _wait_for_action(action_wait):
        # A failed Driver.Init must not orphan controller, scan, or guard
        # process groups after rbnx reports the package as failed.
        _kill_nav2()
        return Err(
            f"navigate_to_pose action server did not come up within {action_wait:.1f}s"
        )
    speed_wait = min(action_wait, 10.0)
    try:
        speed_ready = _wait_for_speed_subscriber(speed_wait)
    except Exception as error:  # noqa: BLE001
        _kill_nav2()
        return Err(f"failed to apply initial navigation speed: {error}")
    if not speed_ready:
        _kill_nav2()
        return Err(
            "Nav2 controller or final velocity guard did not subscribe to "
            f"its speed-limit channel within {speed_wait:.1f}s "
            f"(Nav2 topic: {_speed_settings.topic})"
        )

    with _state_lock:
        _initialized = True
    log.info(
        "init complete: nav2 alive, navigation and speed capabilities serving "
        "(max_linear=%.3fm/s, speed=%.1f%%)",
        settings.max_linear_speed_mps,
        _speed_percentage,
    )
    return Ok()


@nav.on_shutdown
def shutdown():
    """INACTIVE/ACTIVE → TERMINATED. Tear the nav2 subprocess down so it
    doesn't outlive the wrapper. Best-effort; never fails shutdown."""
    _kill_nav2()
    return Ok()


# ── navigation RPC/MCP shared implementation ─────────────────────────────────
def _quat_to_yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def _navigate_impl(goal) -> dict:
    """Queue a Nav2 goal and return the contract-level response fields."""
    global _last_run_id
    if _ros_node is None:
        return {"accepted": False, "run_id": "", "detail": "ROS2 not initialized"}
    run_id = str(uuid.uuid4())
    frame_id = goal.header.frame_id or "map"
    yaw = _quat_to_yaw(goal.pose.orientation.z, goal.pose.orientation.w)
    
    # Log incoming goal for debugging
    log.info(
        "navigate: received goal (%.3f, %.3f, %.3f) frame=%s",
        float(goal.pose.position.x), float(goal.pose.position.y), yaw, frame_id,
    )
    
    # Adjust goal if it's on an obstacle
    x, y = _adjust_goal_if_needed(float(goal.pose.position.x), float(goal.pose.position.y))
    
    # Adjust goal to avoid being too close to known objects
    x, y = _adjust_goal_for_objects(x, y)
    
    # Log adjusted goal
    orig_x, orig_y = float(goal.pose.position.x), float(goal.pose.position.y)
    if x != orig_x or y != orig_y:
        log.info(
            "navigate: adjusted goal from (%.3f, %.3f) to (%.3f, %.3f)",
            orig_x, orig_y, x, y,
        )
    
    with _state_lock:
        _last_run_id = run_id
        _goal_states[run_id] = {"state": "PENDING", "detail": "queued"}
    _nav_queue.put((run_id, {
        "frame_id": frame_id,
        "x": x,
        "y": y,
        "yaw": float(yaw),
    }))
    return {"accepted": True, "run_id": run_id, "detail": "queued"}


def _status_impl(run_id: str) -> dict:
    """Return state/detail for an explicit run id, or the most recent run."""
    with _state_lock:
        resolved = _resolve_run_id(run_id)
        st = _goal_states.get(resolved)
    if st is None:
        return {"known": False, "state": "FAILED", "detail": "unknown run_id"}
    return {
        "known": True,
        "state": st.get("state", "FAILED"),
        "detail": st.get("detail", ""),
    }


def _cancel_impl(run_id: str) -> dict:
    """Cancel an explicit run id, or the most recent active navigation run."""
    restore_speed = False
    with _state_lock:
        resolved = _resolve_run_id(run_id)
        state = _goal_states.get(resolved)
        if state is None:
            return {"accepted": False, "detail": "unknown run_id"}
        if state.get("state") in {"SUCCEEDED", "FAILED", "CANCELED"}:
            return {
                "accepted": False,
                "detail": f"run is already {state.get('state', 'terminal').lower()}",
            }
        gh = _goal_handles.get(resolved)
        if gh is None:
            state["cancel_requested"] = True
            if state.get("state") == "PENDING":
                state["state"] = "CANCELED"
                state["detail"] = "canceled before dispatch"
                restore_speed = True
            else:
                state["detail"] = "cancel queued until goal acceptance"
            detail = state["detail"]
            gh = None
        else:
            state["cancel_requested"] = True
            state["detail"] = "submitting cancel to Nav2"
            detail = state["detail"]
    if restore_speed:
        _restore_speed_after_run(resolved)
        return {"accepted": True, "detail": detail}
    if gh is None:
        return {"accepted": True, "detail": detail}
    accepted, detail = _issue_cancel(gh, resolved)
    return {"accepted": accepted, "detail": detail}


def _speed_mutation_failure(detail: str) -> dict:
    """Return the current speed state alongside a rejected mutation."""
    status = _get_speed_limit_impl()
    return {
        "accepted": False,
        "changed": False,
        "effective_percentage": status["effective_percentage"],
        "effective_linear_speed_mps": status[
            "effective_linear_speed_mps"
        ],
        "scope": status["scope"],
        "run_id": status["run_id"],
        "detail": detail,
    }


def _apply_speed_decision(
    decision: SpeedDecision,
    run_id: str,
    persist: bool,
) -> dict:
    """Publish one decision and commit either session or goal ownership."""
    global _speed_percentage, _session_speed_percentage
    global _speed_scope, _speed_scope_run_id
    _publish_speed_limit(decision.percentage)
    _speed_percentage = decision.percentage
    if persist:
        _session_speed_percentage = decision.percentage
        _speed_scope = "session"
        _speed_scope_run_id = ""
    else:
        _speed_scope = "goal"
        _speed_scope_run_id = run_id
    scope = _speed_scope
    scoped_run_id = _speed_scope_run_id
    log.info(
        "dynamic navigation speed scope=%s run_id=%s effective=%.1f%% "
        "(linear=%.3fm/s)",
        scope,
        scoped_run_id or "-",
        decision.percentage,
        decision.linear_speed_mps,
    )
    return {
        "accepted": True,
        "changed": decision.changed,
        "effective_percentage": decision.percentage,
        "effective_linear_speed_mps": decision.linear_speed_mps,
        "scope": scope,
        "run_id": scoped_run_id,
        "detail": decision.detail,
    }


def _adjust_speed_impl(direction: str, run_id: str, persist: bool) -> dict:
    """Apply faster, slower, or normal to a goal or provider session."""
    publisher = _speed_publisher
    if not _initialized or publisher is None:
        return _speed_mutation_failure("navigation is not initialized")
    if not _speed_channels_available():
        return _speed_mutation_failure(
            "Nav2 or final-guard speed-limit subscriber is unavailable"
        )
    try:
        with _speed_lock:
            settings = _speed_settings
            if settings is None:
                raise RuntimeError("dynamic speed settings are unavailable")
            resolved = "" if persist else _active_speed_run_id(run_id)
            decision = decide_adjustment(
                _speed_percentage,
                direction,
                settings,
            )
            return _apply_speed_decision(decision, resolved, persist)
    except Exception as error:  # noqa: BLE001
        return _speed_mutation_failure(str(error))


def _set_speed_limit_impl(
    percentage: float,
    run_id: str,
    persist: bool,
) -> dict:
    """Set an explicit percentage for a goal or provider session."""
    publisher = _speed_publisher
    if not _initialized or publisher is None:
        return _speed_mutation_failure("navigation is not initialized")
    if not _speed_channels_available():
        return _speed_mutation_failure(
            "Nav2 or final-guard speed-limit subscriber is unavailable"
        )
    try:
        with _speed_lock:
            settings = _speed_settings
            if settings is None:
                raise RuntimeError("dynamic speed settings are unavailable")
            resolved = "" if persist else _active_speed_run_id(run_id)
            decision = decide_explicit(
                _speed_percentage,
                percentage,
                settings,
            )
            return _apply_speed_decision(decision, resolved, persist)
    except Exception as error:  # noqa: BLE001
        return _speed_mutation_failure(str(error))


def _get_speed_limit_impl() -> dict:
    """Read configuration and effective state without mutating Navigation."""
    with _speed_lock:
        settings = _speed_settings
        if settings is None:
            return {
                "available": False,
                "max_linear_speed_mps": 0.0,
                "default_percentage": 0.0,
                "min_percentage": 0.0,
                "step_percentage": 0.0,
                "effective_percentage": 0.0,
                "effective_linear_speed_mps": 0.0,
                "scope": "",
                "run_id": "",
                "detail": "dynamic speed settings are unavailable",
            }
        effective_linear_speed = (
            settings.max_linear_speed_mps * _speed_percentage / 100.0
        )
        available = bool(_initialized and _speed_channels_available())
        return {
            "available": available,
            "max_linear_speed_mps": settings.max_linear_speed_mps,
            "default_percentage": settings.default_percentage,
            "min_percentage": settings.min_percentage,
            "step_percentage": settings.step_percentage,
            "effective_percentage": _speed_percentage,
            "effective_linear_speed_mps": effective_linear_speed,
            "scope": _speed_scope,
            "run_id": _speed_scope_run_id,
            "detail": (
                "navigation speed limit is available"
                if available
                else "Nav2 or final-guard speed-limit subscriber is unavailable"
            ),
        }


# ── gRPC servicers ───────────────────────────────────────────────────────────
class _NavigateServicer(contracts_grpc.RobonixServiceNavigationNavigateServicer):
    def Navigate(self, request, context):
        out = _navigate_impl(request.goal)
        return navigation_pb2.Navigate_Response(**out)


class _StatusServicer(contracts_grpc.RobonixServiceNavigationNavigateStatusServicer):
    def GetNavigationStatus(self, request, context):
        out = _status_impl(request.run_id)
        return navigation_pb2.GetNavigationStatus_Response(**out)


class _CancelServicer(contracts_grpc.RobonixServiceNavigationNavigateCancelServicer):
    def CancelNavigation(self, request, context):
        out = _cancel_impl(request.run_id)
        return navigation_pb2.CancelNavigation_Response(**out)


class _AdjustSpeedServicer(
    contracts_grpc.RobonixServiceNavigationAdjustSpeedServicer
):
    def AdjustNavigationSpeed(self, request, context):
        out = _adjust_speed_impl(
            request.direction,
            request.run_id,
            request.persist,
        )
        return navigation_pb2.AdjustNavigationSpeed_Response(**out)


class _SetSpeedLimitServicer(
    contracts_grpc.RobonixServiceNavigationSetSpeedLimitServicer
):
    def SetNavigationSpeedLimit(self, request, context):
        out = _set_speed_limit_impl(
            request.percentage,
            request.run_id,
            request.persist,
        )
        return navigation_pb2.SetNavigationSpeedLimit_Response(**out)


class _GetSpeedLimitServicer(
    contracts_grpc.RobonixServiceNavigationGetSpeedLimitServicer
):
    def GetNavigationSpeedLimit(self, request, context):
        return navigation_pb2.GetNavigationSpeedLimit_Response(
            **_get_speed_limit_impl()
        )


# ── MCP tools ────────────────────────────────────────────────────────────────
@nav.mcp("robonix/service/navigation/navigate")
def navigate(req: McpNavigateRequest) -> McpNavigateResponse:
    """Start a Nav2 NavigateToPose run. Returns a run_id for status/cancel."""
    out = _navigate_impl(req.goal)
    if not out["accepted"]:
        raise RuntimeError(out["detail"])
    return McpNavigateResponse(**out)


@nav.mcp("robonix/service/navigation/navigate/status")
def status(req: McpStatusRequest) -> McpStatusResponse:
    """Poll a navigation run. Empty run_id means the most recent run."""
    out = _status_impl(req.run_id)
    if not out["known"]:
        raise RuntimeError(out["detail"])
    return McpStatusResponse(**out)


@nav.mcp("robonix/service/navigation/navigate/cancel")
def cancel(req: McpCancelRequest) -> McpCancelResponse:
    """Cancel a navigation run. Empty run_id means the most recent active run."""
    out = _cancel_impl(req.run_id)
    if not out["accepted"]:
        raise RuntimeError(out["detail"])
    return McpCancelResponse(**out)


@nav.mcp("robonix/service/navigation/adjust_speed")
def adjust_speed(req: McpAdjustSpeedRequest) -> McpAdjustSpeedResponse:
    """Make Navigation faster, slower, or normal without replacing its goal.

    direction must be faster, slower, or normal. Empty run_id selects the most
    recent active run. persist=false restores the session limit when that run
    ends; persist=true applies across navigation runs and ignores run_id.
    """
    out = _adjust_speed_impl(req.direction, req.run_id, req.persist)
    if not out["accepted"]:
        raise RuntimeError(out["detail"])
    return McpAdjustSpeedResponse(**out)


@nav.mcp("robonix/service/navigation/set_speed_limit")
def set_speed_limit(
    req: McpSetSpeedLimitRequest,
) -> McpSetSpeedLimitResponse:
    """Set a percentage of the configured maximum planar navigation speed.

    Empty run_id selects the most recent active run. persist=false restores the
    session limit when that run ends; persist=true applies across navigation
    runs and ignores run_id.
    """
    out = _set_speed_limit_impl(req.percentage, req.run_id, req.persist)
    if not out["accepted"]:
        raise RuntimeError(out["detail"])
    return McpSetSpeedLimitResponse(**out)


@nav.mcp("robonix/service/navigation/get_speed_limit")
def get_speed_limit(
    req: McpGetSpeedLimitRequest,
) -> McpGetSpeedLimitResponse:
    """Read maximum, normal, minimum, step, effective limit, and scope."""
    out = _get_speed_limit_impl()
    if not out["available"]:
        raise RuntimeError(out["detail"])
    return McpGetSpeedLimitResponse(**out)


# ── attach the navigate/status/cancel gRPC servicers ─────────────────────────
# run() hosts these on the same auto-allocated port as the Driver lifecycle
# and atlas-declares each by contract. They're live from bootstrap; each one
# guards on `_ros_node`, so a call before CMD_INIT finishes returns a clean
# 'not initialized' rather than crashing.
nav.attach_grpc_servicer("robonix/service/navigation/navigate", _NavigateServicer())
nav.attach_grpc_servicer("robonix/service/navigation/navigate/status", _StatusServicer())
nav.attach_grpc_servicer("robonix/service/navigation/navigate/cancel", _CancelServicer())
nav.attach_grpc_servicer(
    "robonix/service/navigation/adjust_speed", _AdjustSpeedServicer()
)
nav.attach_grpc_servicer(
    "robonix/service/navigation/set_speed_limit", _SetSpeedLimitServicer()
)
nav.attach_grpc_servicer(
    "robonix/service/navigation/get_speed_limit", _GetSpeedLimitServicer()
)


def main() -> int:
    """Blocking. Service.run() registers nav2 with atlas, serves the Driver
    lifecycle + navigate/status/cancel gRPC/MCP, heartbeats, and dispatches
    CMD_INIT/CMD_SHUTDOWN to the on_init / on_shutdown callbacks above."""
    nav.run()
    return 0


if __name__ == "__main__":
    main()
