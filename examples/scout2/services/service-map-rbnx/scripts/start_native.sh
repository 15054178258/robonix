#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx native (no-docker) launcher. Mirrors docker/entrypoint.sh
# but runs directly on the host ROS 2 install (rtabmap_* from apt).
# Picked by scripts/start.sh when ROBONIX_MAPPING_FORCE=native (or
# ROBONIX_MAPPING_PLATFORM matches the native whitelist — jetson_orin).
#
# Same sibling-process flow as the container path:
#   1. atlas_bridge   — registers cap, resolves sensors via atlas,
#                       writes /tmp/<algo>_resolved.yaml, declares outputs.
#   2. SLAM engine    — start_engine.sh → ros2 launch rtabmap_2d.launch.py
#                       (MAPPING_LAUNCH_DIR points at the package's launch/).
#
# SIGTERM tears down both children.
set -eo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ── ROS 2 (host) — includes rtabmap_slam / _odom / _viz from apt ───────
if [[ -z "${ROS_DISTRO:-}" || -z "${AMENT_PREFIX_PATH:-}" ]] || ! command -v ros2 >/dev/null 2>&1; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        set +u; source /opt/ros/humble/setup.bash; set -u
    else
        echo "[mapping-native] ERR: ROS 2 not sourced and /opt/ros/humble missing" >&2
        echo "[mapping-native]      set ROBONIX_MAPPING_FORCE=docker to use the container path" >&2
        exit 2
    fi
fi
if [[ -f "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash" ]]; then
    set +u; source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"; set -u
fi
# Fail loud if rtabmap isn't actually installed natively.
if ! ros2 pkg prefix rtabmap_slam >/dev/null 2>&1; then
    echo "[mapping-native] ERR: rtabmap_slam not found on the host ROS install." >&2
    echo "[mapping-native]      sudo apt install ros-humble-rtabmap-ros" >&2
    echo "[mapping-native]      (or ROBONIX_MAPPING_FORCE=docker)" >&2
    exit 2
fi
if ! ros2 pkg prefix imu_filter_madgwick >/dev/null 2>&1; then
    echo "[mapping-native] ERR: imu_filter_madgwick not found on the host ROS install." >&2
    echo "[mapping-native]      sudo apt install ros-humble-imu-filter-madgwick" >&2
    exit 2
fi

# ── PYTHONPATH: pkg src + codegen stubs + robonix-api ──────────────────
CODEGEN="$PKG/rbnx-build/codegen"
if [[ ! -d "$CODEGEN/proto_gen" ]]; then
    echo "[mapping-native] ERR: $CODEGEN/proto_gen missing — run \`rbnx codegen -p $PKG\` first" >&2
    exit 2
fi
export PYTHONPATH="$PKG/src:$CODEGEN/proto_gen:$CODEGEN/robonix_mcp_types:${PYTHONPATH:-}"
if command -v rbnx >/dev/null 2>&1; then
    if API="$(rbnx path robonix-api 2>/dev/null)" && [[ -d "$API" ]]; then
        export PYTHONPATH="$API:$PYTHONPATH"
    fi
fi

mkdir -p "$PKG/rbnx-build/data"

# ── Env (mirror the docker -e block) ───────────────────────────────────
export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-mapping}"
export ROBONIX_PKG_HOST_DIR="$PKG"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export MAPPING_RESOLVED_DIR="${MAPPING_RESOLVED_DIR:-/tmp}"
# start_engine.sh reads the launch from here (container used /mapping/launch).
export MAPPING_LAUNCH_DIR="$PKG/launch"
export MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"
# Persistent map store. Container default is /mapping/maps (bind-mounted);
# natively there is no /mapping, so anchor it under the package dir so
# saved maps survive restarts. Override with MAPPING_MAPS_DIR.
export MAPPING_MAPS_DIR="${MAPPING_MAPS_DIR:-$PKG/maps}"
# Runtime databases can grow by gigabytes during a full-floor scan. Keep them
# on the package filesystem (the Jetson SSD), never rootfs /tmp, and remove
# prior ephemeral sessions before starting the single supported map engine.
export MAPPING_RUNTIME_DB_DIR="${MAPPING_RUNTIME_DB_DIR:-$PKG/rbnx-build/runtime}"
mkdir -p "$MAPPING_RUNTIME_DB_DIR"
find "$MAPPING_RUNTIME_DB_DIR" -maxdepth 1 -type f \
    \( -name '*.db' -o -name '*.db-wal' -o -name '*.db-shm' \) -delete

PYBIN="${MAPPING_NATIVE_PYTHON:-python3}"

ATLAS_PID=
ENGINE_PID=
cleanup() {
    [ -n "$ENGINE_PID" ] && kill -TERM "$ENGINE_PID" 2>/dev/null || true
    [ -n "$ATLAS_PID" ]  && kill -TERM "$ATLAS_PID"  2>/dev/null || true
    pkill -TERM -P $$ 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Clear stale gate files from a prior aborted boot (rbnx 2026-05-23 patch).
# Without this, start_native.sh bypasses the bridge-write gate, runs engine
# on a stale resolved.yaml from a previous run, fails fast, and trap kills
# the bridge BEFORE rbnx delivers CMD_INIT — Cancelling all calls error.
rm -f /tmp/mapping_algo /tmp/*_resolved.yaml

# ── 1. atlas_bridge (the cap) ──────────────────────────────────────────
"$PYBIN" -u -m mapping_rbnx.atlas_bridge 2>&1 | sed 's/^/[bridge] /' &
ATLAS_PID=$!

# Gate on atlas_bridge writing /tmp/mapping_algo + /tmp/<algo>_resolved.yaml.
for _ in $(seq 1 60); do
    [ -f /tmp/mapping_algo ] && break
    sleep 0.5
done
ALGO="$(cat /tmp/mapping_algo 2>/dev/null || echo rtabmap)"
export MAPPING_ALGO="$ALGO"
for _ in $(seq 1 60); do
    [ -f "/tmp/${ALGO}_resolved.yaml" ] && break
    sleep 0.5
done

# ── 2. SLAM engine ─────────────────────────────────────────────────────
bash "$PKG/scripts/start_engine.sh" 2>&1 | sed 's/^/[engine] /' &
ENGINE_PID=$!

wait "$ENGINE_PID"
