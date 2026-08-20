#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# nav2_wrapper start phase. Two execution shapes (same pattern as
# mapping_rbnx):
#
#   1. docker  (default) — `docker run` against `robonix-nav2`, which
#       bundles ROS2 Humble + nav2_bringup. Works on any host with docker
#       even when the host has NO ROS2 (x86 dev boxes, the webots CI box).
#   2. native  — scripts/start_native.sh: the atlas bridge as a host
#       process against a host-installed ROS2 + Nav2. Preferred on a robot
#       whose host already runs ROS2 (avoids the container hop).
#
# Selection (operator-set env in the shell running rbnx boot/start; the
# cap config arrives via Driver(CMD_INIT) so it can't be read here):
#   ROBONIX_NAV2_FORCE=native|docker     # explicit hard pin
#   ROBONIX_NAV2_PLATFORM=<platform>     # match NATIVE_PLATFORMS
#   default → docker
set -eo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

NATIVE_PLATFORMS=("jetson_orin")
is_native_platform() {
    local p="$1"
    for w in "${NATIVE_PLATFORMS[@]}"; do
        [[ "$p" == "$w" ]] && return 0
    done
    return 1
}

MODE=""
case "${ROBONIX_NAV2_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") ;;
    *) echo "[nav2/start] ROBONIX_NAV2_FORCE=${ROBONIX_NAV2_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_NAV2_PLATFORM:-}"; then MODE=native; else MODE=docker; fi
fi
echo "[nav2/start] mode=${MODE} (FORCE=${ROBONIX_NAV2_FORCE:-} PLATFORM=${ROBONIX_NAV2_PLATFORM:-})"

if [[ "$MODE" == "native" ]]; then
    exec bash "${PKG}/scripts/start_native.sh"
fi

# ── Docker path ─────────────────────────────────────────────────────────
set -u
CT="${ROBONIX_NAV2_CONTAINER:-robonix_nav2}"
IMG="${ROBONIX_NAV2_IMAGE:-robonix-nav2}"
RUNTIME_PROTO_TMP=""

cleanup() {
    docker stop "$CT" >/dev/null 2>&1 || true
    if [[ -n "$RUNTIME_PROTO_TMP" ]]; then
        rm -rf -- "$RUNTIME_PROTO_TMP"
    fi
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

docker rm -f "$CT" >/dev/null 2>&1 || true

# `rbnx codegen` uses the host's active Python grpc_tools installation, while
# this deployment imports generated modules with the Docker image's protobuf
# runtime. Those versions are allowed to differ, and an old host generator can
# therefore produce descriptors that protobuf 4+ refuses to import. Generate a
# Docker-only copy with the exact grpc_tools/protobuf stack that will execute
# it, validate every generated module offline, then atomically expose it to the
# runtime container. Keep the host output intact for native deployments.
prepare_runtime_proto_gen() {
    local proto_staging="$PKG/rbnx-build/proto-staging"
    local runtime_proto
    local runtime_proto_gen="$PKG/rbnx-build/codegen/nav2_proto_gen"

    runtime_proto="$(rbnx path runtime-proto)" || {
        echo "[nav2/start] cannot resolve Robonix runtime proto directory" >&2
        return 1
    }
    [[ -d "$runtime_proto" && -f "$runtime_proto/atlas.proto" ]] || {
        echo "[nav2/start] missing runtime atlas.proto: $runtime_proto" >&2
        return 1
    }
    [[ -d "$proto_staging" ]] \
        && find "$proto_staging" -maxdepth 1 -type f -name '*.proto' -print -quit \
            | grep -q . || {
        echo "[nav2/start] missing staged package protos; run rbnx build first" >&2
        return 1
    }

    mkdir -p "$PKG/rbnx-build/codegen"
    RUNTIME_PROTO_TMP="$(mktemp -d "${runtime_proto_gen}.tmp.XXXXXX")"

    docker run --rm \
        --network none \
        --entrypoint sh \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp \
        -v "$runtime_proto:/runtime-proto:ro" \
        -v "$proto_staging:/proto-staging:ro" \
        -v "$RUNTIME_PROTO_TMP:/proto-gen" \
        "$IMG" -ec '
            python3 -m grpc_tools.protoc \
                -I/runtime-proto \
                -I/proto-staging \
                --python_out=/proto-gen \
                --grpc_python_out=/proto-gen \
                /runtime-proto/*.proto \
                /proto-staging/*.proto
            PYTHONPATH=/proto-gen python3 -c '\''import importlib, pathlib; p = pathlib.Path("/proto-gen"); modules = sorted({f.stem for f in p.glob("*_pb2.py")} | {f.stem for f in p.glob("*_pb2_grpc.py")}); assert modules; [importlib.import_module(name) for name in modules]'\''
        '

    rm -rf -- "$runtime_proto_gen"
    mv -- "$RUNTIME_PROTO_TMP" "$runtime_proto_gen"
    RUNTIME_PROTO_TMP=""
    echo "[nav2/start] runtime-compatible protobuf stubs ready"
}

prepare_runtime_proto_gen
mkdir -p rbnx-build/data

declare -a ZENOH_ARGS=()
if [[ -n "${ROBONIX_ZENOH_ROUTER:-}" ]]; then
    ZENOH_ARGS=(-e "ROBONIX_ZENOH_ROUTER=${ROBONIX_ZENOH_ROUTER}")
fi
if [[ -n "${ROBONIX_ZENOH_MODE:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_MODE=${ROBONIX_ZENOH_MODE}")
fi
if [[ -n "${ROBONIX_ZENOH_LISTEN:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_LISTEN=${ROBONIX_ZENOH_LISTEN}")
fi

# Do not synthesize an empty value: absence preserves the compatible /cmd_vel
# default, while an explicitly exported empty value is forwarded so the bridge
# can reject it instead of silently falling back to a motion topic.
declare -a VELOCITY_OUTPUT_ARGS=()
if [[ "${ROBONIX_VELOCITY_OUTPUT_TOPIC+x}" == "x" ]]; then
    VELOCITY_OUTPUT_ARGS=(
        -e "ROBONIX_VELOCITY_OUTPUT_TOPIC=${ROBONIX_VELOCITY_OUTPUT_TOPIC-}"
    )
fi

# A deploy-owned params_file/BT is resolved relative to the robot manifest,
# not this package checkout. Preserve rbnx's manifest directory at the same
# absolute path inside Docker so Docker and native execution agree.
declare -a DEPLOY_ARGS=()
if [[ -n "${RBNX_INVOCATION_CWD:-}" ]]; then
    if [[ ! -d "$RBNX_INVOCATION_CWD" ]]; then
        echo "[nav2/start] RBNX_INVOCATION_CWD is not a directory: $RBNX_INVOCATION_CWD" >&2
        exit 2
    fi
    DEPLOY_DIR="$(cd "$RBNX_INVOCATION_CWD" && pwd -P)"
    DEPLOY_ARGS=(
        -e "RBNX_INVOCATION_CWD=$DEPLOY_DIR"
        -v "$DEPLOY_DIR:$DEPLOY_DIR:ro"
    )
fi

# config arrives via Driver(CMD_INIT) over gRPC; the container's bridge
# binds NAV2_DRIVER_PORT and registers with atlas at ROBONIX_ATLAS.
exec docker run --rm \
    --memory 2g \
    --memory-swap 2g \
    --name "$CT" \
    --network host \
    --ipc=host \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}" \
    -e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-nav2}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    -e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
    "${ZENOH_ARGS[@]}" \
    "${VELOCITY_OUTPUT_ARGS[@]}" \
    -e NAV2_DRIVER_PORT="${NAV2_DRIVER_PORT:-50235}" \
    -e NAV2_LOG_LEVEL="${NAV2_LOG_LEVEL:-info}" \
    "${DEPLOY_ARGS[@]}" \
    -v "$(pwd)":/nav2 \
    -v "$PKG/rbnx-build/codegen/nav2_proto_gen:/nav2/rbnx-build/codegen/proto_gen:ro" \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    -v "$(pwd)/docker/no_shm_profile.xml":/etc/fastrtps_no_shm.xml:ro \
    "$IMG"
