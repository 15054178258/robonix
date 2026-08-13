#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx container entrypoint.
#
# Two processes run in this container:
#   1. atlas_bridge               — Python: registers cap with atlas,
#                                   queries sensor primitives, writes
#                                   resolved.yaml, declares own outputs.
#   2. SLAM engine                — rtabmap (default) / dlio / fastlio2
#                                   (selected by config.algo via
#                                   /mapping/scripts/start_engine.sh).
# Both are siblings; we trap on EXIT so SIGTERM tears the lot down.

set -eo pipefail

source /opt/ros/humble/setup.bash
if [[ -f /mapping/rbnx-build/codegen/ros2_idl/install/setup.bash ]]; then
    source /mapping/rbnx-build/codegen/ros2_idl/install/setup.bash
fi

# Robonix ros2_idl overlay (map interface package — the bridge's
# lifecycle broadcast publishes map/msg/MapLifecycle). Built by
# scripts/build.sh onto the bind-mounted rbnx-build/; when absent the
# bridge logs loudly and runs with the broadcast disabled.
ROS2_IDL_SETUP=/mapping/rbnx-build/codegen/ros2_idl/install/setup.bash
if [ -f "$ROS2_IDL_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$ROS2_IDL_SETUP"
fi

configure_zenoh_session() {
    if [ "${RMW_IMPLEMENTATION:-}" != "rmw_zenoh_cpp" ] || [ -z "${ROBONIX_ZENOH_ROUTER:-}" ]; then
        return 0
    fi
    local src="/opt/ros/${ROS_DISTRO:-humble}/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"
    local dst="/tmp/robonix_zenoh_session.json5"
    if [ ! -f "$src" ]; then
        echo "[entrypoint] missing Zenoh session config: $src" >&2
        return 1
    fi
    local mode="${ROBONIX_ZENOH_MODE:-client}"
    sed \
        -e "s#mode: \"peer\"#mode: \"${mode}\"#" \
        -e "s#\"tcp/localhost:7447\"#\"${ROBONIX_ZENOH_ROUTER}\"#g" \
        "$src" > "$dst"
    if [ -n "${ROBONIX_ZENOH_LISTEN:-}" ]; then
        sed -i "s#\"tcp/localhost:0\"#\"${ROBONIX_ZENOH_LISTEN}\"#g" "$dst"
    fi
    export ZENOH_SESSION_CONFIG_URI="$dst"
    export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"
    echo "[entrypoint] rmw_zenoh_cpp mode=${mode} router=${ROBONIX_ZENOH_ROUTER} listen=${ROBONIX_ZENOH_LISTEN:-<default>}"
}

configure_zenoh_session

cd /mapping

# Docker start generates these modules with this image and bind-masks the
# host-generated proto_gen. Emit a focused error before Python's descriptor
# compatibility traceback obscures the deployment problem.
PROTO_GEN=/mapping/rbnx-build/codegen/proto_gen
if [ ! -f "$PROTO_GEN/atlas_pb2.py" ] \
    || [ ! -f "$PROTO_GEN/map_pb2.py" ] \
    || [ ! -f "$PROTO_GEN/robonix_contracts_pb2_grpc.py" ]; then
    echo "[entrypoint] missing runtime-compatible protobuf stubs in $PROTO_GEN; rebuild and restart mapping" >&2
    exit 1
fi

# Codegen output lives under <pkg>/rbnx-build/codegen/ per v0.1 (matches
# `rbnx codegen` default + `robonix_api.codegen.ensure_proto_gen` walks).
# atlas_bridge needs the generated atlas_pb2 etc., so prepend the
# proto_gen + robonix_mcp_types subdirs.
export PYTHONPATH="/mapping/src:/mapping/rbnx-build/codegen/proto_gen:/mapping/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if [ -d /robonix-api ]; then
    export PYTHONPATH="/robonix-api:${PYTHONPATH}"
fi

mkdir -p /mapping/rbnx-build/data

ZENOHD_PID=
BRIDGE_PID=

ENGINE_PID=
ATLAS_PID=

cleanup() {
    [ -n "$ENGINE_PID" ] && kill -TERM "$ENGINE_PID" 2>/dev/null || true
    [ -n "$ATLAS_PID" ]  && kill -TERM "$ATLAS_PID"  2>/dev/null || true
    kill -TERM "$BRIDGE_PID" 2>/dev/null || true
    kill -TERM "$ZENOHD_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1

# ── 3. atlas_bridge ─────────────────────────────────────────────────────────
python3 -m mapping_rbnx.atlas_bridge 2>&1 | sed 's/^/[bridge] /' &
ATLAS_PID=$!

# Wait for atlas_bridge to choose an algo + write resolved config.
# atlas_bridge writes /tmp/mapping_algo (one-line algo name) once
# config is parsed AND /tmp/<algo>_resolved.yaml when atlas-discovered
# topics are available. Both are gating signals for start_engine.sh.
for _ in $(seq 1 30); do
    [ -f /tmp/mapping_algo ] && break
    sleep 0.5
done
ALGO="$(cat /tmp/mapping_algo 2>/dev/null || echo rtabmap)"
export MAPPING_ALGO="$ALGO"
RESOLVED="/tmp/${ALGO}_resolved.yaml"
for _ in $(seq 1 30); do
    [ -f "$RESOLVED" ] && break
    sleep 0.5
done

# ── 4. SLAM engine (selected by algo) ───────────────────────────────────────
bash /mapping/scripts/start_engine.sh 2>&1 | sed 's/^/[engine] /' &
ENGINE_PID=$!

wait "$ENGINE_PID"
