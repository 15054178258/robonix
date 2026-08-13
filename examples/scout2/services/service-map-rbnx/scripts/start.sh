#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx start phase. Two execution shapes (same pattern as
# system/scene/scripts/start.sh):
#
#   1. native  (jetson_orin, or any host with ROS 2 Humble + rtabmap_*
#       installed natively) — runs scripts/start_native.sh: atlas_bridge
#       + rtabmap launch as host processes, no docker. Preferred on the
#       car (avoids the nvidia-container-runtime + DDS-namespace hops).
#   2. docker  (default fallback) — docker run against `robonix-mapping`.
#
# Selection (operator-set env in the shell that runs rbnx boot/start;
# the cap config arrives via Driver(CMD_INIT) so it can't be read here):
#   ROBONIX_MAPPING_FORCE=native|docker     # explicit hard pin
#   ROBONIX_MAPPING_PLATFORM=<platform>     # match NATIVE_PLATFORMS
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
case "${ROBONIX_MAPPING_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") ;;
    *) echo "[mapping/start] ROBONIX_MAPPING_FORCE=${ROBONIX_MAPPING_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_MAPPING_PLATFORM:-}"; then MODE=native; else MODE=docker; fi
fi
echo "[mapping/start] mode=${MODE} (FORCE=${ROBONIX_MAPPING_FORCE:-} PLATFORM=${ROBONIX_MAPPING_PLATFORM:-})"

if [[ "$MODE" == "native" ]]; then
    exec bash "${PKG}/scripts/start_native.sh"
fi

# ── Docker path (original behaviour) ───────────────────────────────────
set -u

CT="${ROBONIX_MAPPING_CONTAINER:-robonix_mapping}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"
XHOST_AUTHORIZED=false
XHOST_DISPLAY=""
RUNTIME_PROTO_TMP=""

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [[ "$XHOST_AUTHORIZED" == true ]]; then
        DISPLAY="$XHOST_DISPLAY" xhost -local:docker >/dev/null 2>&1 || true
        XHOST_AUTHORIZED=false
    fi
    docker stop "$CT" >/dev/null 2>&1 || true
    if [[ -n "$RUNTIME_PROTO_TMP" && -d "$RUNTIME_PROTO_TMP" ]]; then
        rm -rf -- "$RUNTIME_PROTO_TMP"
    fi
    return "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

docker rm -f "$CT" >/dev/null 2>&1 || true

# Host codegen is retained for native deployments. Docker imports protobuf
# modules generated and validated with the exact image that executes them.
prepare_runtime_proto_gen() {
    local proto_staging="$PKG/rbnx-build/proto-staging"
    local runtime_proto
    local runtime_proto_gen="$PKG/rbnx-build/codegen/mapping_proto_gen"

    runtime_proto="$(rbnx path runtime-proto)" || {
        echo "[mapping/start] cannot resolve Robonix runtime proto directory" >&2
        return 1
    }
    [[ -d "$runtime_proto" && -f "$runtime_proto/atlas.proto" ]] || {
        echo "[mapping/start] missing runtime atlas.proto: $runtime_proto" >&2
        return 1
    }
    [[ -d "$proto_staging" ]] \
        && find "$proto_staging" -maxdepth 1 -type f -name '*.proto' -print -quit \
            | grep -q . || {
        echo "[mapping/start] missing staged package protos; run rbnx build first" >&2
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
    mv "$RUNTIME_PROTO_TMP" "$runtime_proto_gen"
    RUNTIME_PROTO_TMP=""
    echo "[mapping/start] runtime-compatible protobuf stubs ready"
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

declare -a EXTRA_MOUNTS=()
# NOTE: RBNX_CONFIG_FILE intentionally NOT mounted — config arrives via
# Driver(CMD_INIT, config_json) over gRPC, never a file.
# A deploy-owned params_file is resolved relative to the robot manifest, not
# this package checkout. rbnx deploy exports that directory; preserve the same
# absolute path inside Docker so both relative paths and deploy-local absolute
# paths resolve exactly as they do in native mode.
declare -a DEPLOY_ARGS=()
if [[ -n "${RBNX_INVOCATION_CWD:-}" ]]; then
    if [[ ! -d "$RBNX_INVOCATION_CWD" ]]; then
        echo "[mapping/start] RBNX_INVOCATION_CWD is not a directory: $RBNX_INVOCATION_CWD" >&2
        exit 2
    fi
    DEPLOY_DIR="$(cd "$RBNX_INVOCATION_CWD" && pwd -P)"
    DEPLOY_ARGS=(
        -e "RBNX_INVOCATION_CWD=$DEPLOY_DIR"
        -v "$DEPLOY_DIR:$DEPLOY_DIR:ro"
    )
fi
# If set, keep saved maps in an explicit runtime directory instead of the
# provider cache checkout. CI uses this to keep runs isolated.
if [[ -n "${MAPPING_MAPS_DIR:-}" ]]; then
    mkdir -p "$MAPPING_MAPS_DIR"
    EXTRA_MOUNTS+=(-v "${MAPPING_MAPS_DIR}:${MAPPING_MAPS_DIR}")
fi

# X11 is an explicit opt-in. In particular, a missing DISPLAY must not cause
# an unattended/headless deployment to discover a desktop and widen its X
# server ACL. Any ACL entry we add is revoked by cleanup after docker exits.
MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"
VIZ_ENABLED=false
case "$MAPPING_ENABLE_VIZ" in
    1|true|TRUE|yes|YES|on|ON) VIZ_ENABLED=true ;;
esac
declare -a X11_ARGS=()
if [[ "$VIZ_ENABLED" == true ]]; then
    if [[ -z "${DISPLAY:-}" ]] && command -v xset &>/dev/null; then
        for d in :0 :1 :10; do
            if DISPLAY="$d" xset q &>/dev/null; then
                export DISPLAY="$d"
                break
            fi
        done
    fi
    if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
        if xhost +local:docker >/dev/null 2>&1; then
            XHOST_AUTHORIZED=true
            XHOST_DISPLAY="$DISPLAY"
            X11_ARGS=(-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
        else
            echo "[mapping/start] xhost authorization failed; visualization disabled" >&2
        fi
    fi
fi

# Bash 3.2 treats an empty array expansion as an unset variable under `set -u`.
# All scalar values below already have explicit defaults.
set +u
docker run --rm \
    --name "$CT" \
    --network host \
    --ipc=host \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}" \
    -e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-mapping}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    -e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
    "${ZENOH_ARGS[@]}" \
    -e MAPPING_GRPC_PORT="${MAPPING_GRPC_PORT:-50120}" \
    -e MAPPING_ENABLE_VIZ="$MAPPING_ENABLE_VIZ" \
    -e MAPPING_WEBUI_HOST="${MAPPING_WEBUI_HOST:-127.0.0.1}" \
    -e MAPPING_MAPS_DIR="${MAPPING_MAPS_DIR:-/mapping/maps}" \
    "${DEPLOY_ARGS[@]}" \
    "${X11_ARGS[@]}" \
    -v "$(pwd)":/mapping \
    -v "$PKG/rbnx-build/codegen/mapping_proto_gen:/mapping/rbnx-build/codegen/proto_gen:ro" \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
