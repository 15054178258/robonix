#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# explore_rbnx container entrypoint.
#
# Single long-running process: explore_skill.atlas_bridge runs an
# rclpy node, registers 3 MCP capabilities with atlas, and serves
# them via FastMCP HTTP. The skill's task lifecycle (driving the
# robot through frontier-based exploration) is internal to that
# process; map / nav are atlas-resolved.
set -eo pipefail

source /opt/ros/humble/setup.bash
if [[ -f /explore/rbnx-build/codegen/ros2_idl/install/setup.bash ]]; then
    source /explore/rbnx-build/codegen/ros2_idl/install/setup.bash
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

cd /explore

# start.sh generates these stubs with this image and bind-masks host proto_gen.
# Fail before importing the skill so protobuf compatibility errors are explicit.
PROTO_GEN=/explore/rbnx-build/codegen/proto_gen
if [ ! -f "$PROTO_GEN/atlas_pb2.py" ] \
    || [ ! -f "$PROTO_GEN/explore_pb2.py" ] \
    || [ ! -f "$PROTO_GEN/robonix_contracts_pb2_grpc.py" ]; then
    echo "[entrypoint] missing runtime-compatible protobuf stubs in $PROTO_GEN; rebuild and restart explore" >&2
    exit 1
fi

export PYTHONPATH="/explore:/explore/rbnx-build/codegen/proto_gen:/explore/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if [ -d /robonix-api ]; then
    export PYTHONPATH="/robonix-api:${PYTHONPATH}"
fi

mkdir -p /explore/rbnx-build/data

# Run the skill as PID 1 so its stdout/stderr flow straight to the container
# log and SIGTERM reaches it directly. Do NOT pipe through `sed` for a prefix:
# `sed` block-buffers in a pipe, so the skill's log lines never flushed to the
# container's stdout and `rbnx logs -t com.robonix.skill.explore` showed
# nothing. rbnx already tags each component's output, so no prefix is needed.
exec python3 -u -m explore_skill.atlas_bridge
