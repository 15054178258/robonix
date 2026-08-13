# SPDX-License-Identifier: MulanPSL-2.0
"""Runtime map-management operations for the mapping service.

Backs three RPC+MCP capabilities (declared in atlas_bridge):
  - save_map      snapshot the live SLAM map to disk under a map_id
  - load_map      switch rtabmap onto a saved map (localization / mapping)
  - pose_estimate seed a pose so rtabmap's localization re-converges

These talk to the *running* rtabmap (launched as a separate process by
start_engine.sh in the same ROS graph) over DDS — this module spins its own
lightweight rclpy node, independent of the SLAM launch.

load_map strategy (per design): try rtabmap's runtime services FIRST
(`/rtabmap/load_database` + `/set_mode_localization|mapping`); only if those
are unavailable does the caller fall back to a process restart. Each impl
returns a plain dict whose keys match the contract's response fields, so the
gRPC servicer and the MCP handler in atlas_bridge can share one code path.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import glob
import sqlite3
import threading
import time
from typing import Optional

import logging

from mapping_rbnx import lifecycle

log = logging.getLogger("mapping_rbnx.map_ops")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
PKG_HOST_DIR = os.environ.get("ROBONIX_PKG_HOST_DIR", "/mapping")
RUNTIME_DB_DIR = os.environ.get("MAPPING_RUNTIME_DB_DIR", "/tmp/robonix-mapping-runtime")

# rtabmap node name prefix; the launch runs the slam node as `/rtabmap/rtabmap`
# so its services live under `/rtabmap/...`.
RTABMAP_NS = os.environ.get("MAPPING_RTABMAP_NS", "/rtabmap")
# Where rtabmap subscribes for an externally-seeded pose. The launch remaps
# the standard `/initialpose` into rtabmap; keep them in sync.
INITIALPOSE_TOPIC = os.environ.get("MAPPING_INITIALPOSE_TOPIC", "/initialpose")
# Live map-frame pose (PoseWithCovarianceStamped), published by the tf_to_pose
# adapter on the bound `robonix/service/map/pose` contract. get_pose reads it.
POSE_TOPIC = os.environ.get("MAPPING_POSE_TOPIC", "/robonix/map/pose")


def _sanitize_map_id(map_id: str) -> str:
    import re
    return re.sub(r"[^A-Za-z0-9._-]", "_", (map_id or "").strip()) or "default"


def _sqlite_quick_check(db_path: str) -> tuple[bool, str]:
    """Return whether db_path is a readable SQLite database.

    RTAB-Map stores maps in SQLite. Loading a partially-copied live DB can
    crash rtabmap, so validate before exposing or loading a saved map.
    """
    if not os.path.isfile(db_path):
        return False, "missing rtabmap.db"
    try:
        con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=30.0)
        try:
            row = con.execute("PRAGMA quick_check").fetchone()
            msg = str(row[0]) if row else "no quick_check result"
            return msg.lower() == "ok", msg
        finally:
            con.close()
    except Exception as e:  # noqa: BLE001
        return False, str(e)


def _sqlite_backup(src: str, dst: str) -> tuple[bool, str]:
    """Consistently snapshot a live SQLite DB using sqlite3's backup API.

    A plain file copy of ~/.ros/rtabmap.db races RTAB-Map's writer and can
    produce "database disk image is malformed" on load. SQLite backup takes a
    transactionally-consistent snapshot while the source remains live.
    """
    if not os.path.isfile(src):
        return False, f"live database not found: {src}"
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    tmp = dst + ".tmp"
    try:
        if os.path.exists(tmp):
            os.remove(tmp)
        src_con = sqlite3.connect(f"file:{src}?mode=ro", uri=True, timeout=60.0)
        dst_con = sqlite3.connect(tmp, timeout=60.0)
        try:
            src_con.backup(dst_con, pages=1024, sleep=0.05)
        finally:
            dst_con.close()
            src_con.close()
        ok, detail = _sqlite_quick_check(tmp)
        if not ok:
            try:
                os.remove(tmp)
            except OSError:
                pass
            return False, f"backup integrity check failed: {detail}"
        os.replace(tmp, dst)
        return True, "sqlite backup ok"
    except Exception as e:  # noqa: BLE001
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except OSError:
            pass
        return False, str(e)


# ── rclpy node (lazy, shared, own spin thread) ────────────────────────────────
_node = None
_node_lock = threading.Lock()


def _get_node():
    """Create (once) an rclpy node + background executor so map ops can call
    rtabmap services / publish poses. Returns None if rclpy is unavailable
    (e.g. ROS not sourced) — callers degrade to a clear error."""
    global _node
    with _node_lock:
        if _node is not None:
            return _node
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node("mapping_map_ops")
            ex = SingleThreadedExecutor()
            ex.add_node(node)
            t = threading.Thread(target=ex.spin, daemon=True)
            t.start()
            node._robonix_executor = ex  # keep refs alive
            node._robonix_spin = t
            _node = node
            log.info("map_ops rclpy node up (ns=%s)", RTABMAP_NS)
            return _node
        except Exception as e:  # noqa: BLE001
            log.warning("map_ops: rclpy node unavailable: %s", e)
            return None


def _yaw_to_quat(theta: float):
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


def _call_service(node, srv_type, name: str, request, timeout_s: float = 5.0):
    """Blocking service call from the spinning node. Returns (ok, result_or_err)."""
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False, f"service {name} unavailable"
    fut = cli.call_async(request)
    deadline = time.time() + timeout_s
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not fut.done():
        return False, f"service {name} timed out"
    return True, fut.result()


# ── pose_estimate ─────────────────────────────────────────────────────────────
def pose_estimate_impl(x: float, y: float, theta: float,
                       cov_xy: float = 0.0, cov_theta: float = 0.0) -> dict:
    """Publish a PoseWithCovarianceStamped to INITIALPOSE_TOPIC so rtabmap
    re-localizes from the given guess. Returns {ok, detail}."""
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    try:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(float(theta))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Diagonal covariance: [x, y, z, roll, pitch, yaw]. Default to the
        # conventional rviz "2D Pose Estimate" values when caller passes 0.
        var_xy = (cov_xy or 0.25) ** 2 if cov_xy else 0.25
        var_yaw = (cov_theta or 0.07) ** 2 if cov_theta else 0.068
        cov = [0.0] * 36
        cov[0] = var_xy
        cov[7] = var_xy
        cov[35] = var_yaw
        msg.pose.covariance = cov
        # Latch one publish; a fresh publisher each call keeps this stateless.
        pub = node.create_publisher(PoseWithCovarianceStamped, INITIALPOSE_TOPIC, 1)
        # DDS needs a beat to match the subscriber before the sample is kept.
        time.sleep(0.3)
        pub.publish(msg)
        time.sleep(0.2)
        node.destroy_publisher(pub)
        return {"ok": True, "detail": f"seeded pose ({x:.2f}, {y:.2f}, {theta:.2f}) on {INITIALPOSE_TOPIC}"}
    except Exception as e:  # noqa: BLE001
        log.exception("pose_estimate failed")
        return {"ok": False, "detail": str(e)}


# ── list_maps ─────────────────────────────────────────────────────────────────
def list_maps_impl() -> dict:
    """List saved maps under MAPS_DIR.

    Returns {ok, detail, maps_json}. The JSON payload mirrors the mapping Web
    UI library rows but is exposed through the standard map capability surface
    so consumers such as scene never depend on debug HTTP or shared volumes.
    """
    maps = []
    try:
        if not os.path.isdir(MAPS_DIR):
            return {"ok": True, "detail": "", "maps_json": "[]"}
        for name in sorted(os.listdir(MAPS_DIR)):
            d = os.path.join(MAPS_DIR, name)
            if not os.path.isdir(d):
                continue
            db = os.path.join(d, "rtabmap.db")
            meta = {}
            mp = os.path.join(d, "meta.yaml")
            if os.path.isfile(mp):
                try:
                    with open(mp, "r", encoding="utf-8") as fh:
                        for line in fh:
                            if ":" in line:
                                k, v = line.split(":", 1)
                                meta[k.strip()] = v.strip()
                except Exception:  # noqa: BLE001
                    pass
            has_artifact = os.path.isfile(db)
            artifact_ok, artifact_detail = _sqlite_quick_check(db) if has_artifact else (False, "missing spatial artifact")
            preview = os.path.join(d, "occupancy.png")
            maps.append({
                "map_id": name,
                "has_spatial_artifact": has_artifact,
                "spatial_ok": bool(artifact_ok),
                "artifact_detail": artifact_detail,
                "has_preview": os.path.isfile(preview),
                "artifact_path": db if has_artifact else "",
                "preview_path": preview if os.path.isfile(preview) else "",
                "artifact_size": os.path.getsize(db) if has_artifact else 0,
                "updated": int(os.path.getmtime(db)) if has_artifact else 0,
                "meta": meta,
            })
        return {"ok": True, "detail": "", "maps_json": json.dumps(maps, ensure_ascii=False)}
    except Exception as e:  # noqa: BLE001
        log.exception("list_maps failed")
        return {"ok": False, "detail": str(e), "maps_json": "[]"}


# ── load_map ──────────────────────────────────────────────────────────────────
def _runtime_db_copy(saved_db: str, map_id: str) -> str:
    """Copy an immutable saved DB to a runtime DB used by RTAB-Map.

    Loading RTAB-Map directly on /mapping/maps/<map_id>/rtabmap.db makes the
    supposedly saved artifact mutable again. Use a runtime copy instead; the
    saved map remains a read-only artifact for Robonix semantics. The runtime
    copy must be loaded with LoadDatabase.clear=false; clear=true deletes an
    existing target DB before opening it in RTAB-Map.
    """
    os.makedirs(RUNTIME_DB_DIR, exist_ok=True)
    safe_id = _sanitize_map_id(map_id)
    for old in glob.glob(os.path.join(RUNTIME_DB_DIR, f"{safe_id}-*.db")):
        try:
            os.remove(old)
        except OSError:
            pass
    runtime_db = os.path.join(RUNTIME_DB_DIR, f"{safe_id}-{int(time.time() * 1000)}.db")
    shutil.copy2(saved_db, runtime_db)
    db_ok, db_detail = _sqlite_quick_check(runtime_db)
    if not db_ok:
        raise RuntimeError(f"runtime db copy failed integrity check: {db_detail}")
    return runtime_db


def _publish_full_map(node, timeout_s: float = 30.0) -> tuple[bool, str]:
    """Ask RTAB-Map to republish the global optimized map after load."""
    try:
        from rtabmap_msgs.srv import PublishMap
    except Exception as e:  # noqa: BLE001
        return False, f"rtabmap_msgs/PublishMap unavailable: {e}"
    req = PublishMap.Request()
    req.global_map = True
    req.optimized = True
    req.graph_only = False
    ok, res = _call_service(node, PublishMap, f"{RTABMAP_NS}/publish_map", req, timeout_s=timeout_s)
    if not ok:
        return False, str(res)
    return True, "published optimized global map"


def _load_database(node, runtime_db: str, timeout_s: float) -> tuple[bool, str]:
    """Load one runtime database, isolated for ordering tests."""
    try:
        from rtabmap_msgs.srv import LoadDatabase
    except Exception as e:  # noqa: BLE001
        return False, f"rtabmap_msgs/LoadDatabase unavailable: {e}"
    req = LoadDatabase.Request()
    req.database_path = runtime_db
    req.clear = False
    ok, res = _call_service(
        node, LoadDatabase, f"{RTABMAP_NS}/load_database", req, timeout_s=timeout_s
    )
    return (ok, "load_database completed" if ok else str(res))


def _occupancy_sample_ready(msg) -> tuple[bool, str]:
    """Return whether an occupancy sample is non-empty and structurally valid.

    Map identity comes from the successful database load plus the lifecycle
    epoch. RTAB-Map may optimize cell boundaries when it republishes a loaded
    database, so occupancy pixels are readiness data, not an identity hash.
    """
    info = msg.info
    width = int(info.width)
    height = int(info.height)
    resolution = float(info.resolution)
    cells = width * height
    data = msg.data
    known = sum(1 for value in data if int(value) >= 0)
    summary = (
        f"{width}x{height}@{resolution:.6f} "
        f"origin=({info.origin.position.x:.3f},{info.origin.position.y:.3f}) "
        f"cells={len(data)} known={known}"
    )
    ready = (
        width > 0
        and height > 0
        and resolution > 0.0
        and len(data) == cells
        and known > 0
    )
    return ready, summary


def _begin_target_map_wait(node) -> dict:
    """Wait for the latched occupancy published after a successful map load."""
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)

    ready = threading.Event()
    observed = {"summary": "no occupancy sample received"}

    def on_map(msg):
        try:
            valid, summary = _occupancy_sample_ready(msg)
            observed["summary"] = summary
            if valid:
                ready.set()
        except Exception as exc:  # noqa: BLE001
            observed["summary"] = f"invalid occupancy sample: {exc}"

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    topic = os.environ.get("MAPPING_OCCUPANCY_TOPIC", "/map")
    sub = node.create_subscription(OccupancyGrid, topic, on_map, qos)
    return {
        "event": ready,
        "observed": observed,
        "subscription": sub,
        "topic": topic,
    }


def _finish_target_map_wait(node, barrier: dict, timeout_s: float) -> tuple[bool, str]:
    try:
        matched = barrier["event"].wait(timeout=max(0.0, timeout_s))
    finally:
        node.destroy_subscription(barrier["subscription"])
    if not matched:
        return False, (
            f"fresh non-empty occupancy was not observed on {barrier['topic']} "
            f"within {timeout_s:.1f}s; observed={barrier['observed']['summary']}"
        )
    return True, (
        f"verified fresh occupancy on {barrier['topic']} "
        f"({barrier['observed']['summary']})"
    )


def load_map_impl(map_id: str, mode: str = "localization",
                  has_initial_pose: bool = False,
                  x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> dict:
    """Load an immutable saved map through a runtime DB copy.

    Saved spatial maps are artifacts and must not be modified after save.
    Therefore this always loads a copied database and switches RTAB-Map to
    localization mode, then forces a full optimized map publish so /map reflects
    the saved artifact instead of the previous live mapping session.
    """
    map_id = _sanitize_map_id(map_id)
    requested_mode = (mode or "localization").strip().lower()
    if requested_mode not in ("localization", "mapping"):
        return {"ok": False, "detail": f"mode={requested_mode!r} invalid (localization|mapping)"}
    mode = "localization"
    db_path = os.path.join(MAPS_DIR, map_id, "rtabmap.db")
    if not os.path.isfile(db_path):
        return {"ok": False, "detail": f"no saved map at {db_path}"}
    db_ok, db_detail = _sqlite_quick_check(db_path)
    if not db_ok:
        return {"ok": False, "detail": f"saved map database is invalid: {db_detail}"}

    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}

    try:
        started = time.monotonic()
        log.info("load_map[%s] stage=prepare source=%s requested_mode=%s", map_id,
                 db_path, requested_mode)
        runtime_db = _runtime_db_copy(db_path, map_id)

        # RTAB-Map only restores the saved 2D occupancy grid when the database
        # is opened in localization mode. Loading first while still in mapping
        # mode omits that grid; a second load then appears to fix the UI/RViz.
        log.info("load_map[%s] stage=switch_mode target=localization", map_id)
        ok2, info2 = _set_mode(node, mode)
        if not ok2:
            return {"ok": False, "detail": f"failed to switch localization before load: {info2}"}
        set_current_mode(mode)

        load_timeout_s = float(os.environ.get("MAPPING_LOAD_DATABASE_TIMEOUT_S", "180"))
        log.info("load_map[%s] stage=load_database runtime_db=%s timeout=%.1fs",
                 map_id, runtime_db, load_timeout_s)
        ok, load_detail = _load_database(node, runtime_db, load_timeout_s)
        if not ok:
            return {"ok": False,
                    "detail": f"load_database failed: {load_detail} after {load_timeout_s:.0f}s"}

        # The database load is the map-identity authority. Publish that
        # identity before waiting for occupancy so consumers can bind the
        # following grid to the correct map epoch without pixel comparison.
        lifecycle.set_state(map_id, mode, bump=(mode == "mapping"))

        publish_timeout_s = float(os.environ.get("MAPPING_PUBLISH_MAP_TIMEOUT_S", "45"))
        log.info("load_map[%s] stage=publish_map timeout=%.1fs", map_id, publish_timeout_s)
        pub_ok, pub_detail = _publish_full_map(node, timeout_s=publish_timeout_s)
        if not pub_ok:
            return {"ok": False, "detail": f"loaded {map_id}, but full map publish failed: {pub_detail}"}
        # /map is transient-local. Subscribing after publish receives the
        # latest grid and avoids accepting the previous map's latched sample.
        barrier = _begin_target_map_wait(node)
        verify_timeout_s = float(os.environ.get("MAPPING_VERIFY_MAP_TIMEOUT_S", "30"))
        verified, verify_detail = _finish_target_map_wait(node, barrier, verify_timeout_s)
        if not verified:
            log.error("load_map[%s] stage=verify failed: %s", map_id, verify_detail)
            return {"ok": False, "detail": f"loaded {map_id}, but {verify_detail}"}
        elapsed = time.monotonic() - started
        log.info("load_map[%s] stage=complete elapsed=%.3fs %s", map_id, elapsed,
                 verify_detail)

        seeded = ""
        if has_initial_pose:
            ps = pose_estimate_impl(x, y, theta)
            seeded = f"; {ps['detail']}"
        note = "" if requested_mode == "localization" else f"; requested {requested_mode} coerced to localization"
        return {"ok": True,
                "runtime_db_path": runtime_db,
                "detail": f"loaded immutable map {map_id} via runtime copy; {pub_detail}; "
                          f"{verify_detail}; elapsed={elapsed:.1f}s{seeded}{note}"}
    except Exception as e:  # noqa: BLE001
        log.exception("load_map failed")
        return {"ok": False, "detail": str(e)}


# ── mode tracking (get_mode) ──────────────────────────────────────────────────
# Single source of truth for "which SLAM mode is in effect right now", updated
# by init (startup map_mode), switch_mode and load_map — so get_mode reflects
# the real runtime mode regardless of how it changed (config, MCP, or webui).
_current_mode: str = ""


def set_current_mode(mode: str) -> None:
    """Record the SLAM mode now in effect. Called by atlas_bridge.init with the
    startup map_mode and by switch_mode_impl / load_map_impl on success."""
    global _current_mode
    if mode:
        _current_mode = mode.strip().lower()


def get_mode_impl() -> dict:
    """Return the SLAM mode currently in effect (read-only). Returns
    {ok, mode, detail}; mode is "" with ok=False before init has run."""
    if not _current_mode:
        return {"ok": False, "mode": "", "detail": "mode not initialized yet"}
    return {"ok": True, "mode": _current_mode, "detail": ""}


def get_pose_impl(timeout_s: float = 2.0) -> dict:
    """Read the robot's current pose in the MAP frame from the live pose topic
    (PoseWithCovarianceStamped on POSE_TOPIC). Returns
    {ok, x, y, theta (yaw rad), frame_id, detail}. ok=False with a hint if no
    pose arrives within timeout_s (mapping not localized / not publishing)."""
    node = _get_node()
    if node is None:
        return {"ok": False, "x": 0.0, "y": 0.0, "theta": 0.0, "frame_id": "",
                "detail": "rclpy node unavailable (ROS not running?)"}
    try:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                               DurabilityPolicy, HistoryPolicy)
        got = threading.Event()
        holder: dict = {}

        def _cb(msg):
            holder["msg"] = msg
            got.set()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        sub = node.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, _cb, qos)
        try:
            got.wait(timeout=timeout_s)
        finally:
            node.destroy_subscription(sub)
        if "msg" not in holder:
            return {"ok": False, "x": 0.0, "y": 0.0, "theta": 0.0, "frame_id": "",
                    "detail": f"no pose on {POSE_TOPIC} within {timeout_s:.1f}s "
                              "(is mapping localized / publishing?)"}
        msg = holder["msg"]
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {"ok": True, "x": float(p.position.x), "y": float(p.position.y),
                "theta": float(yaw), "frame_id": msg.header.frame_id or "map",
                "detail": ""}
    except Exception as e:  # noqa: BLE001
        log.exception("get_pose failed")
        return {"ok": False, "x": 0.0, "y": 0.0, "theta": 0.0, "frame_id": "",
                "detail": str(e)}


# ── switch_mode ───────────────────────────────────────────────────────────────
def _set_mode(node, mode: str) -> tuple[bool, str]:
    """Call rtabmap's set_mode_localization|set_mode_mapping (std_srvs/Empty)."""
    from std_srvs.srv import Empty
    srv = "set_mode_localization" if mode == "localization" else "set_mode_mapping"
    mode_timeout_s = float(os.environ.get("MAPPING_SET_MODE_TIMEOUT_S", "60"))
    ok, res = _call_service(node, Empty, f"{RTABMAP_NS}/{srv}", Empty.Request(), timeout_s=mode_timeout_s)
    return (ok, srv if ok else f"{res} after {mode_timeout_s:.0f}s")


def switch_mode_impl(mode: str) -> dict:
    """Flip the running rtabmap between mapping and localization on the CURRENT
    map — no map load, no restart. Returns {ok, detail}."""
    mode = (mode or "").strip().lower()
    if mode not in ("localization", "mapping"):
        return {"ok": False, "detail": f"mode={mode!r} invalid (localization|mapping)"}
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    try:
        ok, info = _set_mode(node, mode)
        if not ok:
            return {"ok": False, "detail": f"{info} — rtabmap may lack the mode service "
                                           "(fall back to restart with config map_mode)"}
        set_current_mode(mode)
        # Mode flip only — the live frame does not move, so no generation bump.
        lifecycle.set_mode(mode)
        return {"ok": True, "detail": f"switched to {mode} mode"}
    except Exception as e:  # noqa: BLE001
        log.exception("switch_mode failed")
        return {"ok": False, "detail": str(e)}


# ── reset_map ─────────────────────────────────────────────────────────────────
def reset_map_impl() -> dict:
    """Wipe the running rtabmap's map (working memory + live database) and
    restart SLAM from scratch — for when mapping has diverged and you want a
    clean rebuild without a full redeploy. Calls rtabmap's `/rtabmap/reset`
    (std_srvs/Empty).

    Caveat: rtabmap restarts with the robot's CURRENT pose as the new origin,
    so the rebuilt map's frame will NOT align with the pre-reset one (origin
    drift). Saved maps on disk are untouched. Returns {ok, detail}.
    """
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    try:
        from std_srvs.srv import Empty
        ok, res = _call_service(node, Empty, f"{RTABMAP_NS}/reset", Empty.Request(), timeout_s=10.0)
        if not ok:
            return {"ok": False, "detail": f"{res} — rtabmap /reset unavailable "
                                           "(fall back to restart with config)"}
        mode_ok, mode_detail = _set_mode(node, "mapping")
        if not mode_ok:
            return {"ok": False, "detail": f"map reset, but failed to switch back to mapping mode: {mode_detail}"}
        set_current_mode("mapping")
        # Same map_id, new origin: bump the frame epoch so consumers know
        # their stored map-frame coordinates just went stale. Reset resumes
        # in mapping mode — broadcast that too.
        lifecycle.mark_reset(mode="mapping")
        return {"ok": True, "detail": "map cleared — rebuilding from current pose "
                                      "(origin reset; new frame won't match the old map); switched to mapping mode"}
    except Exception as e:  # noqa: BLE001
        log.exception("reset_map failed")
        return {"ok": False, "detail": str(e)}




def _set_rtabmap_paused(node, paused: bool, timeout_s: float = 10.0) -> tuple[bool, str]:
    """Pause/resume RTAB-Map processing around live database snapshots.

    RTAB-Map writes statistics and node data while mapping. A concurrent SQLite
    backup can otherwise make the RTAB-Map process abort with
    "database is locked". Treat pause failure as a hard save failure; an
    unchecked live snapshot is worse than refusing to save.
    """
    try:
        from std_srvs.srv import Empty
    except Exception as e:  # noqa: BLE001
        return False, f"std_srvs/Empty unavailable: {e}"
    service = f"{RTABMAP_NS}/{'pause' if paused else 'resume'}"
    ok, res = _call_service(node, Empty, service, Empty.Request(), timeout_s=timeout_s)
    if not ok:
        return False, str(res)
    return True, f"rtabmap {'paused' if paused else 'resumed'}"

def _flush_rtabmap_database(node, live_db: str, timeout_s: float = 180.0) -> tuple[bool, str, str]:
    """Ask RTAB-Map to serialize memory without switching databases.

    Do not use LoadDatabase(live_db, clear=false) as a save shortcut: that
    callback closes the current database, clears runtime state, and reloads the
    requested DB. In long Webots mapping sessions this can drop the live rtabmap
    node and leave only wrapper/viz processes alive. RTAB-Map provides a
    dedicated /backup service that saves memory, writes the 2D map cache, copies
    live_db to live_db + ".back", and reinitializes the same database.
    """
    try:
        from std_srvs.srv import Empty
    except Exception as e:  # noqa: BLE001
        return False, f"std_srvs/Empty unavailable: {e}", ""
    started_at = time.time()
    ok, res = _call_service(node, Empty, f"{RTABMAP_NS}/backup", Empty.Request(), timeout_s=timeout_s)
    if not ok:
        return False, str(res), ""
    back = f"{live_db}.back"
    if not os.path.isfile(back):
        return False, f"rtabmap backup completed but did not produce {back}", ""
    # RTAB-Map's backup service serializes working memory and writes a stable
    # sibling copy. Refuse stale copies so save_map cannot publish an old graph
    # with a fresh occupancy preview.
    if os.path.getmtime(back) + 1.0 < started_at:
        return False, f"rtabmap backup file is stale: {back}", ""
    ok2, detail2 = _sqlite_quick_check(back)
    if not ok2:
        return False, f"rtabmap backup integrity check failed: {detail2}", ""
    return True, "rtabmap backup completed; source=rtabmap backup artifact", back


# ── save_map ──────────────────────────────────────────────────────────────────
def _atomic_publish_map_dir(staging_dir: str, map_dir: str) -> None:
    """Publish a completed staged map directory without leaving half-saves.

    Directory replacement cannot be a single POSIX rename over a non-empty
    existing directory, so keep the old map beside it until the staged directory
    is in place. If publishing fails, restore the old map when possible.
    """
    previous_dir = f"{map_dir}.previous-{os.getpid()}-{int(time.time() * 1000)}"
    if os.path.exists(previous_dir):
        shutil.rmtree(previous_dir, ignore_errors=True)
    moved_previous = False
    try:
        if os.path.exists(map_dir):
            os.replace(map_dir, previous_dir)
            moved_previous = True
        os.replace(staging_dir, map_dir)
        if moved_previous:
            shutil.rmtree(previous_dir, ignore_errors=True)
    except Exception:
        if moved_previous and not os.path.exists(map_dir) and os.path.exists(previous_dir):
            os.replace(previous_dir, map_dir)
        raise


def _run_preview_snapshot(map_dir: str) -> bool:
    """Write occupancy preview artifacts for the map library UI."""
    import subprocess
    candidates = [
        os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "scripts", "save_map.py")),
        os.path.join(PKG_HOST_DIR, "scripts", "save_map.py"),
        "/mapping/scripts/save_map.py",
    ]
    script = next((c for c in candidates if c and os.path.isfile(c)), "")
    if not script:
        log.warning("save_map.py not found in candidates %s", candidates)
        return False
    try:
        proc = subprocess.run(
            ["python3", script, "--out-dir", map_dir, "--timeout", "12"],
            check=False, timeout=40, text=True, capture_output=True,
        )
        occupancy_ok = os.path.isfile(os.path.join(map_dir, "occupancy.png"))
        if proc.returncode != 0 and not occupancy_ok:
            log.warning("save_map.py failed for %s rc=%s stdout=%s stderr=%s",
                        map_dir, proc.returncode, proc.stdout[-1000:], proc.stderr[-1000:])
            return False
        if proc.returncode != 0:
            log.info("save_map.py wrote occupancy preview for %s but optional artifacts were incomplete: %s",
                     map_dir, proc.stdout[-1000:])
        else:
            log.info("save_map.py wrote preview for %s: %s", map_dir, proc.stdout[-1000:])
        return occupancy_ok
    except Exception as e:  # noqa: BLE001
        log.warning("save_map.py failed for %s: %s", map_dir, e)
        return False


def save_map_impl(map_id: str, note: str = "",
                  active_db: Optional[str] = None) -> dict:
    """Snapshot the current SLAM map under {MAPS_DIR}/<map_id>/.

    A spatial map is immutable once published. User annotations and scene
    objects are saved by scene under the same map_id; this function owns only
    the RTAB-Map spatial artifact.
    """
    map_id = _sanitize_map_id(map_id)
    map_dir = os.path.join(MAPS_DIR, map_id)
    db_path = os.path.join(map_dir, "rtabmap.db")
    staging_dir = ""
    try:
        os.makedirs(MAPS_DIR, exist_ok=True)

        if os.path.isfile(db_path):
            db_ok, db_detail = _sqlite_quick_check(db_path)
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": db_path,
                "detail": f"spatial map {map_id!r} already exists and is immutable; update scene annotations/objects separately ({db_detail})",
            }

        live_db = active_db if (active_db and os.path.isfile(active_db)) else None
        if live_db is None:
            for cand in (
                os.environ.get("RTABMAP_DATABASE_PATH", ""),
                os.path.expanduser("~/.ros/rtabmap.db"),
            ):
                if cand and os.path.isfile(cand):
                    live_db = cand
                    break
        if not live_db:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": "no live rtabmap database found to snapshot",
            }

        node = _get_node()
        if node is None:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": "rclpy node unavailable; cannot ask RTAB-Map to save",
            }

        flush_ok, flush_detail, snapshot_src = _flush_rtabmap_database(
            node,
            live_db,
            float(os.environ.get("MAPPING_SAVE_BACKUP_TIMEOUT_S",
                                 os.environ.get("MAPPING_SAVE_FLUSH_TIMEOUT_S", "180"))),
        )
        if not flush_ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"rtabmap save/flush failed: {flush_detail}",
            }

        staging_dir = os.path.join(
            MAPS_DIR,
            f".{map_id}.staging-{os.getpid()}-{int(time.time() * 1000)}",
        )
        if os.path.exists(staging_dir):
            shutil.rmtree(staging_dir, ignore_errors=True)
        os.makedirs(staging_dir, exist_ok=False)
        staged_db = os.path.join(staging_dir, "rtabmap.db")

        ok, detail = _sqlite_backup(snapshot_src, staged_db)
        if not ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"failed to snapshot flushed RTAB-Map database: {detail}",
            }
        flush_detail = f"{flush_detail}; sqlite_backup={detail}"

        db_ok, db_detail = _sqlite_quick_check(staged_db)
        if not db_ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"staged database failed integrity check: {db_detail}",
            }

        pub_ok, pub_detail = _publish_full_map(
            node, timeout_s=float(os.environ.get("MAPPING_PUBLISH_MAP_TIMEOUT_S", "45"))
        )
        if not pub_ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"saved DB snapshot but RTAB-Map did not publish a complete map preview: {pub_detail}",
            }
        flush_detail = f"{flush_detail}; {pub_detail}"

        preview_ok = _run_preview_snapshot(staging_dir)
        meta_path = os.path.join(staging_dir, "meta.yaml")
        if os.path.isfile(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as fh:
                    lines = fh.readlines()
                with open(meta_path, "w", encoding="utf-8") as fh:
                    wrote = False
                    for line in lines:
                        if line.startswith("map_id:"):
                            fh.write(f"map_id: {map_id}\n")
                            wrote = True
                        else:
                            fh.write(line)
                    if not wrote:
                        fh.write(f"map_id: {map_id}\n")
            except Exception as e:  # noqa: BLE001
                log.warning("failed to normalize metadata map_id for %s: %s", staging_dir, e)
        if not preview_ok or not os.path.isfile(os.path.join(staging_dir, "occupancy.png")):
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": "map preview/occupancy snapshot was not produced; refusing to publish incomplete spatial artifact",
            }

        # Re-check after preview generation so the published directory is known
        # loadable at the exact point it becomes visible to list/load calls.
        db_ok, db_detail = _sqlite_quick_check(staged_db)
        if not db_ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"staged database failed final integrity check: {db_detail}",
            }

        _atomic_publish_map_dir(staging_dir, map_dir)
        staging_dir = ""
        return {
            "ok": True,
            "map_id": map_id,
            "artifact_path": db_path,
            "detail": f"saved spatial map {map_id}; {flush_detail}",
        }
    except Exception as e:  # noqa: BLE001
        log.exception("save_map failed for %s", map_id)
        return {"ok": False, "map_id": map_id, "artifact_path": "", "detail": str(e)}
    finally:
        if staging_dir and os.path.exists(staging_dir):
            shutil.rmtree(staging_dir, ignore_errors=True)


# ── delete_map ────────────────────────────────────────────────────────────────
def delete_map_impl(map_id: str) -> dict:
    """Remove a saved map's directory ({MAPS_DIR}/<map_id>/) and all its
    artifacts (db + preview). Refuses an empty id or a missing map. Does not
    touch the live SLAM session — only on-disk storage. Returns {ok, detail}."""
    map_id = _sanitize_map_id(map_id)
    map_dir = os.path.join(MAPS_DIR, map_id)
    try:
        if not os.path.isdir(map_dir):
            return {"ok": False, "map_id": map_id, "detail": f"no saved map {map_id!r}"}
        import shutil
        shutil.rmtree(map_dir)
        return {"ok": True, "map_id": map_id, "detail": f"deleted {map_id}"}
    except Exception as e:  # noqa: BLE001
        log.exception("delete_map failed for %s", map_id)
        return {"ok": False, "map_id": map_id, "detail": str(e)}
