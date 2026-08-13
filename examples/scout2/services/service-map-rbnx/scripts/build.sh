#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx build phase — runs `rbnx codegen`, then builds for the
# selected deployment target.
#
# Target is chosen by the per-target package manifest's `build:` line
# (see package_manifest*.yaml). Add a target by adding a case branch
# below plus its Dockerfile / native step — nothing else changes.
#   x86-docker     x86_64 + docker, ROS2 in image (docker/Dockerfile)   [default]
#   jetson-docker  arm64 Jetson + docker, L4T base (docker/Dockerfile.jetson)
#   jetson-native  arm64 Jetson + host ROS2 — no docker; ensure
#                  ros-humble-rtabmap-ros is apt-installed on the host.
#
# RBNX_BUILD_CLEAN=1     nuke rbnx-build/ and rebuild without docker cache.
# RBNX_BUILD_VARIANT=fastlio2_full  (x86-docker only) heavy FASTLIO2 image.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# shellcheck disable=SC1091
source "$PKG/scripts/docker_base_image.sh"
cd "$PKG"

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"
VARIANT="${RBNX_BUILD_VARIANT:-light}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"
TARGET="${RBNX_BUILD_TARGET:-x86-docker}"
ROS_BASE_IMAGE="${ROBONIX_MAPPING_ROS_BASE_IMAGE:-robonix-ros:humble-ros-base}"
UPSTREAM_ROS_BASE_IMAGE="ros:humble-ros-base"
JETSON_ROS_BASE_IMAGE="${ROBONIX_MAPPING_JETSON_ROS_BASE_IMAGE:-dustynv/ros:humble-ros-base-l4t-r36.4.0}"
PYBIN="${MAPPING_NATIVE_PYTHON:-python3}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen (proto stubs for atlas + IDL types + MCP types) — every target ─
# --mcp is REQUIRED: atlas_bridge.py imports `map_mcp` (SaveMap/LoadMap/
# PoseEstimate/SwitchMode request/response dataclasses for the MCP tools).
# Without it codegen emits only proto stubs, `map_mcp` is missing, and the
# bridge dies at import with ModuleNotFoundError → the service never registers.
if command -v rbnx >/dev/null 2>&1; then
    # --ros2 also emits rbnx-build/codegen/ros2_idl (canonical ROS 2
    # interface overlay). The bridge publishes map/msg/MapLifecycle on
    # robonix/service/map/lifecycle from that overlay, so every target
    # needs the sources. Codegen itself needs no host ROS; colcon stays
    # target-specific: jetson-native builds on the host (below), docker
    # targets build inside the image after it exists (further below).
    FLAGS=(--mcp --ros2)
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[build] rbnx codegen ${FLAGS[*]}"
    RBNX_CODEGEN_PYTHON="$PYBIN" \
        rbnx codegen -p "$PKG" "${FLAGS[@]}"

    CODEGEN_PYTHONPATH="$PKG/$BUILD/codegen/proto_gen:$PKG/$BUILD/codegen/robonix_mcp_types"
    PYTHONPATH="$CODEGEN_PYTHONPATH:${PYTHONPATH:-}" "$PYBIN" - <<'PY'
import atlas_pb2_grpc
import map_mcp
import robonix_contracts_pb2_grpc

print("[build] generated Mapping gRPC and MCP imports OK")
PY

    if [[ "$TARGET" == "jetson-native" ]]; then
        set +u
        source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
        set -u
        ROS2_IDL="$PKG/rbnx-build/codegen/ros2_idl"
        bash "$PKG/scripts/build_ros2_overlay.sh" "$ROS2_IDL"
    fi
else
    echo "[build] WARNING: rbnx not in PATH — skipping proto codegen"
    echo "[build]   install robonix-cli + run \`rbnx setup\` once from the robonix source root"
fi

echo "[build] target=$TARGET"

# ── 2. Per-target build ─────────────────────────────────────────────────────
case "$TARGET" in
    x86-docker|jetson-docker)
        if ! command -v docker >/dev/null 2>&1; then
            echo "[build] error: target $TARGET needs docker on PATH" >&2
            exit 1
        fi
        DOCKER_BUILD_FLAGS=(--network=host --pull=false)
        [[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)
        if [[ "$TARGET" == "jetson-docker" ]]; then
            DF=docker/Dockerfile.jetson
            DOCKER_BUILD_FLAGS+=(--build-arg "JETSON_ROS_BASE_IMAGE=${JETSON_ROS_BASE_IMAGE}")
        else
            robonix_ensure_local_base_image "$ROS_BASE_IMAGE" "$UPSTREAM_ROS_BASE_IMAGE"
            DOCKER_BUILD_FLAGS+=(--build-arg "ROS_BASE_IMAGE=${ROS_BASE_IMAGE}")
            case "$VARIANT" in
                light)         DF=docker/Dockerfile ;;
                fastlio2_full) DF=docker/Dockerfile.fastlio2_full ;;
                *) echo "[build] unknown RBNX_BUILD_VARIANT: $VARIANT (light|fastlio2_full)" >&2; exit 2 ;;
            esac
        fi
        if [[ "$CLEAN" != "1" ]] && docker image inspect "$IMG" >/dev/null 2>&1; then
            echo "[build] image $IMG present; rebuilding incrementally"
        fi
        echo "[build] docker build -f $DF -t $IMG"
        docker build "${DOCKER_BUILD_FLAGS[@]}" -f "$DF" -t "$IMG" docker/

        # colcon-build the ros2_idl overlay (map interface package for the
        # lifecycle broadcast) inside the image we just built, so the
        # install tree lands on the host bind mount at the SAME path the
        # runtime container sees (/mapping/rbnx-build/...). Skipped when
        # codegen was skipped (no rbnx on PATH) — the bridge then runs
        # with the lifecycle broadcast disabled.
        IDL="$PKG/rbnx-build/codegen/ros2_idl"
        if [[ -d "$IDL/src/map" ]]; then
            echo "[build] build ros2_idl map interface package in $IMG"
            # --user: build/ install/ land on the HOST bind mount — as root
            # they would survive a later non-root cleanup (RBNX_BUILD_CLEAN,
            # rbnx clean --cache). HOME=/tmp gives colcon a writable home
            # for its metadata as that uid.
            docker run --rm --entrypoint bash --user "$(id -u):$(id -g)" -e HOME=/tmp \
                -v "$PKG":/mapping "$IMG" -lc \
                "source /opt/ros/\${ROS_DISTRO:-humble}/setup.bash && \
                 bash /mapping/scripts/build_ros2_overlay.sh \
                   /mapping/rbnx-build/codegen/ros2_idl"
        else
            echo "[build] WARNING: ros2_idl/src/map missing — lifecycle broadcast will be disabled"
        fi
        ;;

    jetson-native)
        # No docker: rtabmap runs as a host process (start_native.sh). The
        # only build-time requirement is that the host has ROS2 Humble +
        # rtabmap. We don't apt-install for the operator (needs sudo and a
        # specific ROS apt setup) — we verify and tell them what's missing.
        echo "[build] native target — verifying host ROS2 + rtabmap"
        missing=0
        if ! command -v ros2 >/dev/null 2>&1; then
            echo "[build] ERROR: ros2 not on PATH — source /opt/ros/humble/setup.bash" >&2
            missing=1
        fi
        if ! ros2 pkg prefix rtabmap_slam >/dev/null 2>&1; then
            echo "[build] ERROR: rtabmap not installed. On the Jetson host run:" >&2
            echo "[build]   sudo apt install ros-humble-rtabmap-ros" >&2
            missing=1
        fi
        if ! ros2 pkg prefix imu_filter_madgwick >/dev/null 2>&1; then
            echo "[build] ERROR: imu_filter_madgwick not installed. On the Jetson host run:" >&2
            echo "[build]   sudo apt install ros-humble-imu-filter-madgwick" >&2
            missing=1
        fi
        [[ "$missing" == "1" ]] && exit 1
        echo "[build] host rtabmap OK ($(ros2 pkg prefix rtabmap_slam))"
        ;;

    *)
        echo "[build] unknown RBNX_BUILD_TARGET: $TARGET" >&2
        echo "[build]   supported: x86-docker | jetson-docker | jetson-native" >&2
        exit 2
        ;;
esac

echo "[build] done (target=$TARGET)."
