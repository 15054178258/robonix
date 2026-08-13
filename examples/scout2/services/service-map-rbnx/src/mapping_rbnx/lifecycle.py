# SPDX-License-Identifier: MulanPSL-2.0
"""Map identity / lifecycle broadcast — robonix/service/map/lifecycle.

Publishes {map_id, mode, generation} (map/msg/MapLifecycle) latched
(RELIABLE + TRANSIENT_LOCAL, depth 1) so late-joining consumers — the
scene service keys its semantic object store by map_id — learn the
current map identity at startup without polling, and re-publishes on
every lifecycle transition (init / load / reset / mode switch).

`generation` is the map-frame epoch. Consumers holding map-frame
coordinates must treat a change in EITHER (map_id, generation) as
"previously stored coordinates are no longer anchored": reset_map keeps
map_id but moves the map origin, which only generation makes visible.

Bump rules (who calls what is wired in atlas_bridge.init + map_ops):
  - init/load in mapping mode      → bump   (origin re-derived per session)
  - init/load in localization mode → NO bump (stable frame — the whole point)
  - reset_map                      → bump   (origin moves to current pose)
  - switch_mode                    → NO bump (live frame unchanged at switch)
  - save_map                       → no state change (re-key semantics of
                                     "save live map under a new id" is a P3
                                     design question; do not guess here)

For a NAMED map the counter persists in {MAPS_DIR}/<map_id>/generation so
a localization round-trip across restarts keeps the same generation. An
ephemeral session (no map_id) publishes map_id="" — consumers treat that
as "no named identity" and fall back to their own binding config — with a
process-local generation.

The publisher needs the `map` interface package (rbnx-build/codegen/
ros2_idl overlay, built by scripts/build.sh, sourced by the entrypoint).
When the overlay is missing the broadcast is disabled with a loud log —
mapping itself must keep working, consumers just fall back to static
binding.
"""
from __future__ import annotations

import logging
import os
import threading

log = logging.getLogger("mapping_rbnx.lifecycle")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
LIFECYCLE_TOPIC = os.environ.get("MAPPING_LIFECYCLE_TOPIC", "/robonix/map/lifecycle")

_lock = threading.Lock()
_state: dict = {"map_id": "", "mode": "", "generation": 0}
_pub = None            # lazy rclpy publisher (created on first publish)
_msg_cls = None        # map.msg.MapLifecycle, imported lazily
_disabled = False      # set when the msg type is unavailable (permanent)
# Retry bookkeeping for transient publish failures (node not up yet at
# CMD_INIT, rcl hiccup): _state is always correct, so re-running _publish
# later resends the latest identity. Bounded so a permanently broken RMW
# doesn't retry forever.
_retry_pending = False
_retry_attempts = 0
_RETRY_PERIOD_S = 5.0
_RETRY_MAX = 24  # ~2 minutes


def _gen_path(map_id: str) -> str:
    return os.path.join(MAPS_DIR, map_id, "generation")


def _load_gen(map_id: str) -> int:
    """Read the persisted generation for a named map; 0 when absent (fresh
    map — first bump makes it 1). An EXISTING but unparseable file is not
    the same thing: it means the counter was lost (crash mid-write, manual
    edit), so warn — silently restarting at 0 would reuse generation values
    consumers have already seen."""
    path = _gen_path(map_id)
    try:
        with open(path) as f:
            return max(0, int(f.read().strip()))
    except FileNotFoundError:
        return 0
    except (OSError, ValueError) as e:
        log.warning("generation file %s unreadable (%s) — treating as 0; "
                    "epoch numbers may repeat for this map", path, e)
        return 0


def _store_gen(map_id: str, gen: int) -> None:
    """Persist the generation next to the map's rtabmap.db, atomically
    (tmp + rename) so a crash mid-write can't leave a corrupt/empty file.
    Best-effort: on failure the in-memory counter stays authoritative for
    this session (set_state reconciles with max()), but cross-restart
    monotonicity is lost."""
    path = _gen_path(map_id)
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write(str(gen))
        os.replace(tmp, path)
    except OSError as e:
        log.warning("could not persist generation for %s: %s", map_id, e)


def set_state(map_id: str, mode: str, *, bump: bool) -> None:
    """Bind the broadcast to (map_id, mode) and publish. For a named map
    the generation is loaded from disk and, when `bump` is set, incremented
    and persisted. Ephemeral (empty map_id) keeps a process-local counter.
    When re-binding to the map already in effect, the in-memory counter is
    reconciled with max(disk, memory) so a failed persist (or a hand-reset
    file) can never move the broadcast generation backwards in-session —
    the .msg promises monotonicity."""
    with _lock:
        if map_id:
            gen = _load_gen(map_id)
            if _state["map_id"] == map_id:
                gen = max(gen, _state["generation"])
            if bump:
                gen += 1
                _store_gen(map_id, gen)
        else:
            gen = _state["generation"] + 1 if bump else _state["generation"]
        _state.update({"map_id": map_id, "mode": (mode or "").strip().lower(),
                       "generation": gen})
    _publish()


def set_mode(mode: str) -> None:
    """Update the broadcast mode without touching identity/generation
    (switch_mode: the live frame does not move at the switch)."""
    with _lock:
        _state["mode"] = (mode or "").strip().lower()
    _publish()


def mark_reset(mode: str | None = None) -> None:
    """reset_map: same map_id, new origin → bump generation (+ persist for
    a named map) and publish. rtabmap's reset always resumes mapping, so
    callers pass the post-reset mode to keep the broadcast truthful."""
    with _lock:
        gen = _state["generation"] + 1
        _state["generation"] = gen
        if _state["map_id"]:
            _store_gen(_state["map_id"], gen)
        if mode is not None:
            _state["mode"] = (mode or "").strip().lower()
    _publish()


def current() -> dict:
    with _lock:
        return dict(_state)


def _get_publisher():
    """Create (once) the latched publisher on map_ops' shared rclpy node.
    Returns None — and disables the broadcast with one loud log — when the
    `map` interface package is unavailable, or None WITHOUT disabling when
    the rclpy node isn't up yet (transient; _publish schedules a retry).
    Holds _lock while mutating the module singletons: map ops arrive
    concurrently from the gRPC servicers, MCP handlers and the webui, and
    two racing first-publishes must not create two publishers."""
    global _pub, _msg_cls, _disabled
    with _lock:
        if _disabled:
            return None
        if _pub is not None:
            return _pub
        try:
            from map.msg import MapLifecycle  # ros2_idl overlay (see module doc)
        except ImportError as e:
            _disabled = True
            log.error("map/msg/MapLifecycle unavailable (%s) — lifecycle broadcast "
                      "DISABLED for this session. Consumers (scene) will fall "
                      "back to static map binding. Build the ros2_idl overlay "
                      "(scripts/build.sh) and check the entrypoint sources its "
                      "install/setup.bash.", e)
            return None
        from mapping_rbnx import map_ops  # lazy: map_ops imports this module
        node = map_ops._get_node()
        if node is None:
            log.warning("lifecycle publisher not up yet: rclpy node unavailable "
                        "— identity broadcast absent until a retry succeeds")
            return None
        from rclpy.qos import (QoSProfile, ReliabilityPolicy,  # type: ignore
                               DurabilityPolicy, HistoryPolicy)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        _msg_cls = MapLifecycle
        _pub = node.create_publisher(MapLifecycle, LIFECYCLE_TOPIC, qos)
        log.info("lifecycle publisher up on %s (latched)", LIFECYCLE_TOPIC)
        return _pub


def _schedule_retry() -> None:
    """Arm one bounded timer that re-runs _publish. _state is always the
    source of truth, so a later publish resends the latest identity —
    needed when CMD_INIT lands before the ROS graph is ready and no further
    lifecycle transition would otherwise re-trigger the broadcast."""
    global _retry_pending, _retry_attempts
    with _lock:
        if _retry_pending or _disabled or _retry_attempts >= _RETRY_MAX:
            if _retry_attempts >= _RETRY_MAX:
                log.error("lifecycle broadcast still failing after %d retries — "
                          "giving up; consumers fall back to static binding",
                          _RETRY_MAX)
            return
        _retry_pending = True
        _retry_attempts += 1

    def _fire():
        global _retry_pending
        with _lock:
            _retry_pending = False
        _publish()

    t = threading.Timer(_RETRY_PERIOD_S, _fire)
    t.daemon = True
    t.start()


def _publish() -> None:
    """Publish the current state. Latched QoS keeps the last sample for
    late joiners, so publishing once per transition is sufficient.

    NEVER raises: this is called from the success paths of map ops (and
    from CMD_INIT) — a broadcast hiccup must neither fail the op that
    already happened nor be reported as its failure. On transient failure
    the state is kept and a bounded retry is armed."""
    global _retry_attempts
    try:
        pub = _get_publisher()
        if pub is None:
            if not _disabled:
                _schedule_retry()
            return
        with _lock:
            msg = _msg_cls()
            msg.map_id = _state["map_id"]
            msg.mode = _state["mode"]
            msg.generation = int(_state["generation"])
            snapshot = dict(_state)
        pub.publish(msg)
        with _lock:
            _retry_attempts = 0
        log.info("lifecycle: map_id=%r mode=%s generation=%d",
                 snapshot["map_id"], snapshot["mode"], snapshot["generation"])
    except Exception as e:  # noqa: BLE001
        log.error("lifecycle broadcast publish failed (%s) — state kept, "
                  "retrying; the latched sample on the wire is stale until "
                  "a publish succeeds", e)
        _schedule_retry()
