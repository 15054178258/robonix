#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
WORKSPACE_ROOT="$(cd "$DEPLOY_DIR/../.." 2>/dev/null && pwd)"
WORKSPACE_RBNX_DIR="$WORKSPACE_ROOT/.tools/rbnx/bin"
WORKSPACE_RBNX_PYTHON_DIR="$WORKSPACE_ROOT/.tools/rbnx-python/bin"
WORKSPACE_UV_DIR="$WORKSPACE_ROOT/.tools/uv"
WORKSPACE_RUST_BIN="$WORKSPACE_ROOT/.tools/rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin"
WORKSPACE_ROBONIX_SOURCE_ROOT="$WORKSPACE_ROOT/upstream/robonix-go2-build"
WORKSPACE_CARGO_HOME="$WORKSPACE_ROOT/.tools/cargo"
WORKSPACE_CARGO_TARGET_DIR="$WORKSPACE_ROOT/.tools/cargo-target/robonix"
WORKSPACE_RUSTUP_HOME="$WORKSPACE_ROOT/.tools/rustup"
export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"
if [[ -x "$WORKSPACE_RBNX_DIR/rbnx" ]]; then
  export PATH="$WORKSPACE_RBNX_DIR:$PATH"
fi
if [[ -x "$WORKSPACE_RBNX_PYTHON_DIR/python3" ]]; then
  export PATH="$WORKSPACE_RBNX_PYTHON_DIR:$PATH"
fi
if [[ -x "$WORKSPACE_UV_DIR/uv" ]]; then
  export PATH="$WORKSPACE_UV_DIR:$PATH"
fi
if [[ -x "$WORKSPACE_RUST_BIN/cargo" ]]; then
  export PATH="$WORKSPACE_RUST_BIN:$PATH"
fi
[[ -f "$ROBONIX_HOME/config.yaml" ]] || {
  echo "missing workspace-local rbnx config: $ROBONIX_HOME/config.yaml" >&2
  echo "run rbnx setup for the audited Robonix source tree with ROBONIX_HOME set to this directory" >&2
  exit 1
}
export UV_CACHE_DIR="${UV_CACHE_DIR:-$DEPLOY_DIR/.cache/uv}"
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$DEPLOY_DIR/.cache/pip}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-$DEPLOY_DIR/.cache/xdg}"
export MODELSCOPE_CACHE="${MODELSCOPE_CACHE:-$DEPLOY_DIR/.cache/modelscope}"
export MODELSCOPE_CREDENTIALS_PATH="${MODELSCOPE_CREDENTIALS_PATH:-$DEPLOY_DIR/.cache/modelscope/credentials}"
export HF_HOME="${HF_HOME:-$DEPLOY_DIR/.cache/huggingface}"
export TORCH_HOME="${TORCH_HOME:-$DEPLOY_DIR/.cache/torch}"
export GO2_FUNASR_MODEL_PATH="${GO2_FUNASR_MODEL_PATH:-$DEPLOY_DIR/.cache/modelscope/models/iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online}"

# Scene ignores ambient proxy variables unless this explicit switch is set.
# Honor an operator-provided proxy for this invocation without storing its URL
# or changing host network configuration.
if [[ -z "${RBNX_BUILD_PROXY+x}" ]]; then
  if [[ -n "${HTTPS_PROXY:-${https_proxy:-}}" || -n "${HTTP_PROXY:-${http_proxy:-}}" ]]; then
    export RBNX_BUILD_PROXY=1
  else
    export RBNX_BUILD_PROXY=0
  fi
fi

if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

# Reassert workspace ownership after loading local configuration. Neither an
# inherited shell nor .env may redirect source/build state into ~/.cargo, a
# system directory, or a different Robonix checkout.
export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"
export ROBONIX_SOURCE_ROOT="$WORKSPACE_ROBONIX_SOURCE_ROOT"
export CARGO_HOME="$WORKSPACE_CARGO_HOME"
export CARGO_TARGET_DIR="$WORKSPACE_CARGO_TARGET_DIR"
export RUSTUP_HOME="$WORKSPACE_RUSTUP_HOME"
export ROBONIX_BUILD_PROFILE="${ROBONIX_BUILD_PROFILE:-debug}"
case "$ROBONIX_BUILD_PROFILE" in
  debug)
    ROBONIX_CARGO_PROFILE_ARGS=()
    ;;
  release)
    ROBONIX_CARGO_PROFILE_ARGS=(--release)
    ;;
  *)
    echo "ROBONIX_BUILD_PROFILE must be debug or release" >&2
    exit 1
    ;;
esac
export ROBONIX_SYSTEM_BIN_DIR="$CARGO_TARGET_DIR/$ROBONIX_BUILD_PROFILE"
export PATH="$ROBONIX_SYSTEM_BIN_DIR:$PATH"

# Keep generated packages on the same loopback-only control-plane contract as
# runtime. The upstream compatibility gate separately requires Mapping to
# honor MAPPING_ENABLE_VIZ=false before it may inspect X11 or invoke xhost.
export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1
export ROBONIX_ADVERTISE_HOST=127.0.0.1
export SPEECH_BIND_ADDR=127.0.0.1
export MAPPING_ENABLE_VIZ=false
export MAPPING_WEBUI_HOST=127.0.0.1
export SCENE_WEB_HOST=127.0.0.1
unset DISPLAY

export GO2_MAP_ID="${GO2_MAP_ID:-lab_go2}"
export GO2_MAP_MODE="${GO2_MAP_MODE:-localization}"
export SEMANTIC_LANDMARKS_FILE="${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}"
export SPEECH_BACKEND="${SPEECH_BACKEND:-local}"
export GO2_DASHBOARD_PORT="${GO2_DASHBOARD_PORT:-8092}"
export GO2_RUNTIME_PLACEMENT="${GO2_RUNTIME_PLACEMENT:-workstation-local}"
export GO2_ALLOW_MOTION="${GO2_ALLOW_MOTION:-false}"
export GO2_OPERATOR_PRESENT="${GO2_OPERATOR_PRESENT:-false}"
export GO2_SAFETY_ACK="${GO2_SAFETY_ACK:-}"
export GO2_ALLOWED_MODES="${GO2_ALLOWED_MODES:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export UNITREE_SDK2_DIR="${UNITREE_SDK2_DIR:-$DEPLOY_DIR/third_party/unitree_sdk2}"
export UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-$DEPLOY_DIR/rbnx-build/unitree_ros2/install/setup.bash}"

# Resolve the manifest's sole Navigation velocity output from the complete
# motion gate. An inherited/.env ROBONIX_VELOCITY_OUTPUT_TOPIC is ignored.
# This keeps generated package configuration fail-closed as well as runtime.
case "${GO2_ALLOW_MOTION,,}" in
  1|true|yes|on)
    [[ "${GO2_OPERATOR_PRESENT,,}" =~ ^(1|true|yes|on)$ ]] || {
      echo "motion build requested but GO2_OPERATOR_PRESENT is not true" >&2
      exit 1
    }
    [[ "$GO2_SAFETY_ACK" == "I_UNDERSTAND_GO2_CAN_MOVE" ]] || {
      echo "motion build requested but GO2_SAFETY_ACK does not match the required phrase" >&2
      exit 1
    }
    [[ -n "$GO2_ALLOWED_MODES" ]] || {
      echo "motion build requested but GO2_ALLOWED_MODES has not been audited" >&2
      exit 1
    }
    export GO2_ALLOW_MOTION=true
    export ROBONIX_VELOCITY_OUTPUT_TOPIC=/cmd_vel
    ;;
  *)
    export GO2_ALLOW_MOTION=false
    export ROBONIX_VELOCITY_OUTPUT_TOPIC=/robonix/nomotion/cmd_vel
    ;;
esac

case "$GO2_RUNTIME_PLACEMENT" in
  workstation-local)
    export GO2_SENSOR_SOURCE_MODE=local
    ;;
  workstation-full-nx-sensors|workstation-ui-nx-full)
    export GO2_SENSOR_SOURCE_MODE=external
    ;;
  *)
    echo "unsupported GO2_RUNTIME_PLACEMENT: $GO2_RUNTIME_PLACEMENT" >&2
    exit 1
    ;;
esac

[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232" >&2
  exit 1
}

for command in bzip2 cmake colcon curl docker g++ git python3 rbnx tar uv; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing dependency: $command" >&2
    exit 1
  }
done
python3 -c 'import grpc_tools.protoc' >/dev/null 2>&1 || {
  echo "missing Python build module: grpc_tools.protoc in $(command -v python3)" >&2
  echo "provide a workspace-local Python environment containing grpcio-tools" >&2
  exit 1
}

python3 "$DEPLOY_DIR/scripts/validate_robonix_home.py" \
  "$ROBONIX_HOME/config.yaml" \
  "$ROBONIX_SOURCE_ROOT"
[[ -f "$ROBONIX_SOURCE_ROOT/Cargo.toml" ]] || {
  echo "missing audited Robonix Cargo workspace: $ROBONIX_SOURCE_ROOT/Cargo.toml" >&2
  exit 1
}

# rbnx is separate from both its contract generator and the five Rust system
# processes it starts by name. Build every Rust system named by this manifest
# from the exact audited source tree into the one workspace target directory.
# Vitals is included automatically if a future manifest enables system.vitals.
command -v cargo >/dev/null 2>&1 || {
  echo "missing dependency: cargo (required to build Robonix system binaries)" >&2
  exit 1
}
ROBONIX_SYSTEM_PACKAGE_LIST="$(
  python3 "$DEPLOY_DIR/scripts/robonix_system_artifacts.py" list \
    --manifest "$MANIFEST"
)"
mapfile -t ROBONIX_SYSTEM_PACKAGES <<< "$ROBONIX_SYSTEM_PACKAGE_LIST"
(( ${#ROBONIX_SYSTEM_PACKAGES[@]} > 0 )) || {
  echo "manifest contains no Rust Robonix system binaries" >&2
  exit 1
}
ROBONIX_CARGO_PACKAGE_ARGS=(-p robonix-codegen)
for package in "${ROBONIX_SYSTEM_PACKAGES[@]}"; do
  ROBONIX_CARGO_PACKAGE_ARGS+=(-p "$package")
done
cargo build --locked \
  --manifest-path "$ROBONIX_SOURCE_ROOT/Cargo.toml" \
  "${ROBONIX_CARGO_PROFILE_ARGS[@]}" \
  "${ROBONIX_CARGO_PACKAGE_ARGS[@]}"
export ROBONIX_CODEGEN_BIN="$ROBONIX_SYSTEM_BIN_DIR/robonix-codegen"
[[ -x "$ROBONIX_CODEGEN_BIN" ]] || {
  echo "robonix-codegen build produced no executable: $ROBONIX_CODEGEN_BIN" >&2
  exit 1
}
python3 "$DEPLOY_DIR/scripts/robonix_system_artifacts.py" verify \
  --manifest "$MANIFEST" \
  --bin-dir "$ROBONIX_SYSTEM_BIN_DIR" \
  --require-path >/dev/null

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "missing /opt/ros/humble/setup.bash; ROS 2 Humble is required" >&2
  exit 1
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

ros2 pkg prefix rosidl_generator_dds_idl >/dev/null 2>&1 || {
  echo "missing ROS package: ros-humble-rosidl-generator-dds-idl" >&2
  exit 1
}

[[ -f "$UNITREE_SDK2_DIR/CMakeLists.txt" ]] || {
  echo "missing official unitree_sdk2 submodule; run: git submodule update --init --recursive" >&2
  exit 1
}
[[ -f "$DEPLOY_DIR/third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/package.xml" ]] || {
  echo "missing official unitree_ros2 submodule; run: git submodule update --init --recursive" >&2
  exit 1
}

"$DEPLOY_DIR/scripts/verify_submodule_pins.sh"

"$DEPLOY_DIR/scripts/validate_offline.sh"

colcon \
  --log-base "$DEPLOY_DIR/rbnx-build/unitree_ros2/log" \
  build \
  --base-paths "$DEPLOY_DIR/third_party/unitree_ros2/cyclonedds_ws/src/unitree" \
  --build-base "$DEPLOY_DIR/rbnx-build/unitree_ros2/build" \
  --install-base "$DEPLOY_DIR/rbnx-build/unitree_ros2/install" \
  --packages-select unitree_api unitree_go \
  --merge-install \
  --cmake-args -DBUILD_TESTING=OFF

set +u
# shellcheck disable=SC1090
source "$UNITREE_ROS2_SETUP"
set -u

bash "$DEPLOY_DIR/packages/go2_motion_state_relay/scripts/build_ros.sh"

rbnx build -f "$MANIFEST" "$@"
bash "$DEPLOY_DIR/scripts/verify_upstream_compatibility.sh"
