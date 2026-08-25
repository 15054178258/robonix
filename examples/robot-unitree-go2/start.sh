#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
WORKSPACE_ROOT="$(cd "$DEPLOY_DIR/../.." 2>/dev/null && pwd)"
WORKSPACE_TOOLS_BIN="$WORKSPACE_ROOT/.tools/rbnx/bin"
WORKSPACE_RBNX_PYTHON_DIR="$WORKSPACE_ROOT/.tools/rbnx-python/bin"
WORKSPACE_ROBONIX_SOURCE_ROOT="$WORKSPACE_ROOT/upstream/robonix-go2-build"
WORKSPACE_CARGO_HOME="$WORKSPACE_ROOT/.tools/cargo"
WORKSPACE_CARGO_TARGET_DIR="$WORKSPACE_ROOT/.tools/cargo-target/robonix"
WORKSPACE_RUSTUP_HOME="$WORKSPACE_ROOT/.tools/rustup"
INHERITED_FORCE_NOMOTION_PROFILE="${GO2_FORCE_NOMOTION_PROFILE:-}"
INHERITED_NOMOTION_NETWORK_INTERFACE="${GO2_NOMOTION_NETWORK_INTERFACE:-}"
INHERITED_NOMOTION_NAV2_CONTAINER="${ROBONIX_NAV2_CONTAINER:-}"
if [[ -x "$WORKSPACE_TOOLS_BIN/rbnx" ]]; then
  export PATH="$WORKSPACE_TOOLS_BIN:$PATH"
fi
if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi
unset NOMOTION_PRIVATE_WIFI_SSID
readonly NOMOTION_PRIVATE_WIFI_SSID=Robonix-Go2

# The timestamp-corrected workstation wrapper owns these internal markers.
# Capture them before .env is read, then use them only for the exact no-motion
# profile.  This lets the validated wrapper carry its already-checked Wi-Fi
# interface through the second .env load without creating a generic network
# override that a motion-capable launch could inherit.
case "$INHERITED_FORCE_NOMOTION_PROFILE" in
  "")
    unset GO2_FORCE_NOMOTION_PROFILE
    unset GO2_NOMOTION_NETWORK_INTERFACE
    ;;
  workstation-full-nomotion-corrected-v1)
    [[ -n "$INHERITED_NOMOTION_NETWORK_INTERFACE" ]] || {
      echo "corrected no-motion wrapper did not provide its validated network interface" >&2
      exit 1
    }
    export GO2_FORCE_NOMOTION_PROFILE=workstation-full-nomotion-corrected-v1
    export GO2_NETWORK_INTERFACE="$INHERITED_NOMOTION_NETWORK_INTERFACE"
    unset GO2_NOMOTION_NETWORK_INTERFACE
    export GO2_RUNTIME_PLACEMENT=workstation-local
    export GO2_ALLOW_MOTION=false
    export GO2_OPERATOR_PRESENT=false
    export GO2_SAFETY_ACK=""
    export GO2_ALLOWED_MODES=""
    [[ "$INHERITED_NOMOTION_NAV2_CONTAINER" =~ \
      ^robonix_nav2_nomotion_[0-9a-f]{32}$ ]] || {
        echo "corrected no-motion wrapper did not provide its private Nav2 container name" >&2
        exit 1
      }
    export ROBONIX_NAV2_CONTAINER="$INHERITED_NOMOTION_NAV2_CONTAINER"
    # Keep all wrapper checks, the provider start hook and the exact cleanup
    # hook on the same local Docker daemon even if .env contains a remote
    # endpoint intended for another workflow.
    unset DOCKER_HOST DOCKER_TLS_VERIFY DOCKER_CERT_PATH
    export DOCKER_CONTEXT=default
    ;;
  *)
    echo "unsupported GO2_FORCE_NOMOTION_PROFILE" >&2
    exit 1
    ;;
esac

# Python primitives must use the exact workspace environment prepared for the
# audited Robonix source.  Do this after .env so local configuration cannot
# redirect them to an ambient interpreter that merely happens to contain grpc.
[[ -x "$WORKSPACE_RBNX_PYTHON_DIR/python3" ]] || {
  echo "missing workspace-local Robonix Python: $WORKSPACE_RBNX_PYTHON_DIR/python3" >&2
  echo "run the documented workspace setup and ./build.sh before startup" >&2
  exit 1
}
export PATH="$WORKSPACE_RBNX_PYTHON_DIR:$PATH"
[[ "$(command -v python3)" == "$WORKSPACE_RBNX_PYTHON_DIR/python3" ]] || {
  echo "workspace-local Robonix Python did not win PATH resolution" >&2
  exit 1
}

# Local configuration cannot redirect rbnx to ~/.robonix or another checkout.
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"
export ROBONIX_SOURCE_ROOT="$WORKSPACE_ROBONIX_SOURCE_ROOT"
export CARGO_HOME="$WORKSPACE_CARGO_HOME"
export CARGO_TARGET_DIR="$WORKSPACE_CARGO_TARGET_DIR"
export RUSTUP_HOME="$WORKSPACE_RUSTUP_HOME"
export ROBONIX_BUILD_PROFILE="${ROBONIX_BUILD_PROFILE:-debug}"
case "$ROBONIX_BUILD_PROFILE" in
  debug|release)
    ;;
  *)
    echo "ROBONIX_BUILD_PROFILE must be debug or release" >&2
    exit 1
    ;;
esac
export ROBONIX_SYSTEM_BIN_DIR="$CARGO_TARGET_DIR/$ROBONIX_BUILD_PROFILE"
export PATH="$ROBONIX_SYSTEM_BIN_DIR:$PATH"
command -v python3 >/dev/null 2>&1 || {
  echo "python3 is required to validate the workspace-local Robonix config" >&2
  exit 1
}
python3 -c 'import grpc' >/dev/null 2>&1 || {
  echo "missing Python runtime module: grpc in $(command -v python3)" >&2
  echo "run ./build.sh to prepare the workspace-local Robonix Python environment" >&2
  exit 1
}
python3 "$DEPLOY_DIR/scripts/validate_robonix_home.py" \
  "$ROBONIX_HOME/config.yaml" \
  "$ROBONIX_SOURCE_ROOT"
[[ -f "$ROBONIX_SOURCE_ROOT/Cargo.toml" ]] || {
  echo "missing audited Robonix Cargo workspace: $ROBONIX_SOURCE_ROOT/Cargo.toml" >&2
  exit 1
}
[[ -f "$MANIFEST" ]] || {
  echo "Robonix manifest does not exist: $MANIFEST" >&2
  exit 1
}
MANIFEST="$(readlink -f -- "$MANIFEST")"
RBNX_STATE_PATH="$(dirname -- "$MANIFEST")/rbnx-boot/state.json"

# ModelScope must never fall back to ~/.cache or ~/.modelscope. The online
# FunASR model is an audited local artifact; runtime is not allowed to download
# a replacement or continue with a partial directory.
export MODELSCOPE_CACHE="$DEPLOY_DIR/.cache/modelscope"
export MODELSCOPE_CREDENTIALS_PATH="$DEPLOY_DIR/.cache/modelscope/credentials"
export GO2_FUNASR_MODEL_PATH="${GO2_FUNASR_MODEL_PATH:-$MODELSCOPE_CACHE/models/iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online}"
[[ "$GO2_FUNASR_MODEL_PATH" == /* ]] || {
  echo "GO2_FUNASR_MODEL_PATH must be an absolute path" >&2
  exit 1
}
for required_model_file in model.pt config.yaml; do
  [[ -s "$GO2_FUNASR_MODEL_PATH/$required_model_file" ]] || {
    echo "missing or empty audited FunASR artifact: $GO2_FUNASR_MODEL_PATH/$required_model_file" >&2
    echo "runtime download fallback is disabled; populate the workspace model cache first" >&2
    exit 1
  }
done

# The deployment owns every non-DDS bind address. Local configuration cannot
# widen the control, capability, audio, or UI surface onto a LAN interface.
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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export UNITREE_SDK2_DIR="${UNITREE_SDK2_DIR:-$DEPLOY_DIR/third_party/unitree_sdk2}"
export UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-$DEPLOY_DIR/rbnx-build/unitree_ros2/install/setup.bash}"

[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232" >&2
  exit 1
}

case "$GO2_RUNTIME_PLACEMENT" in
  workstation-local)
    EXPECTED_SENSOR_SOURCE_MODE=local
    if [[ "${GO2_FORCE_NOMOTION_PROFILE:-}" == \
      workstation-full-nomotion-corrected-v1 ]]; then
      OWNERSHIP_PREFLIGHT=workstation-full-nomotion-corrected
    else
      OWNERSHIP_PREFLIGHT=workstation-local
    fi
    START_UI_CLIENT_ONLY=false
    echo "Runtime placement: workstation owns chassis, sensors, description and UI"
    ;;
  workstation-full-nx-sensors)
    EXPECTED_SENSOR_SOURCE_MODE=external
    OWNERSHIP_PREFLIGHT=workstation-full-nx-sensors
    START_UI_CLIENT_ONLY=false
    echo "================================================================"
    echo " NX SENSOR OWNER - WORKSTATION SENSOR PUBLISHERS DISABLED"
    echo " Robonix will only wait for and register the standardized topics."
    echo "================================================================"
    ;;
  workstation-ui-nx-full)
    EXPECTED_SENSOR_SOURCE_MODE=external
    OWNERSHIP_PREFLIGHT=workstation-ui-nx-full
    START_UI_CLIENT_ONLY=true
    export GO2_ALLOW_MOTION=false
    export GO2_DASHBOARD_BROWSER_VOICE_ENABLED=0
    echo "================================================================"
    echo " NX FULL READ-ONLY OWNER - WORKSTATION UI/CLIENT ONLY"
    echo " No local chassis, sensor, description, Mapping or Nav2 provider starts."
    echo "================================================================"
    ;;
  *)
    echo "unsupported GO2_RUNTIME_PLACEMENT: $GO2_RUNTIME_PLACEMENT" >&2
    exit 1
    ;;
esac
if [[ -n "${GO2_SENSOR_SOURCE_MODE:-}" \
  && "$GO2_SENSOR_SOURCE_MODE" != "$EXPECTED_SENSOR_SOURCE_MODE" ]]; then
  echo "GO2_SENSOR_SOURCE_MODE conflicts with GO2_RUNTIME_PLACEMENT" >&2
  exit 1
fi
export GO2_SENSOR_SOURCE_MODE="$EXPECTED_SENSOR_SOURCE_MODE"

# UI-client-only deliberately does not boot Robonix system processes. Every
# full placement must resolve each manifest-selected Rust system executable to
# the exact debug/release directory produced by build.sh before any ROS or
# network preflight is attempted.
if [[ "$START_UI_CLIENT_ONLY" == false ]]; then
  python3 "$DEPLOY_DIR/scripts/robonix_system_artifacts.py" verify \
    --manifest "$MANIFEST" \
    --bin-dir "$ROBONIX_SYSTEM_BIN_DIR" \
    --require-path >/dev/null
  echo "Robonix system artifact gate passed: $ROBONIX_SYSTEM_BIN_DIR"
fi

required_commands=(flock ip nmcli timeout)
if [[ "$START_UI_CLIENT_ONLY" == false ]]; then
  required_commands+=(docker git rbnx)
fi
for command in "${required_commands[@]}"; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "$command is not installed or not on PATH" >&2
    exit 1
  }
done
RBNX_CLI=""
if [[ "$START_UI_CLIENT_ONLY" == false ]]; then
  RBNX_CLI="$(command -v rbnx)"
  RBNX_CLI="$(readlink -f -- "$RBNX_CLI")"
  [[ -x "$RBNX_CLI" ]] || {
    echo "selected rbnx CLI is not an executable file: $RBNX_CLI" >&2
    exit 1
  }
fi

[[ -n "${GO2_NETWORK_INTERFACE:-}" ]] || {
  echo "GO2_NETWORK_INTERFACE is unset; copy .env.example to .env and set the dedicated wired NIC" >&2
  exit 1
}
[[ "$GO2_NETWORK_INTERFACE" =~ ^[A-Za-z0-9_.:-]+$ ]] || {
  echo "GO2_NETWORK_INTERFACE contains unsupported characters" >&2
  exit 1
}
ip link show dev "$GO2_NETWORK_INTERFACE" >/dev/null 2>&1 || {
  echo "network interface does not exist: $GO2_NETWORK_INTERFACE" >&2
  exit 1
}
INTERFACE_SYSFS="$(readlink -f -- "/sys/class/net/$GO2_NETWORK_INTERFACE")"
[[ -n "$INTERFACE_SYSFS" && "$INTERFACE_SYSFS" != *"/virtual/"* ]] || {
  echo "GO2 network interface must be a physical device: $GO2_NETWORK_INTERFACE" >&2
  exit 1
}
if [[ -d "/sys/class/net/$GO2_NETWORK_INTERFACE/wireless" ]]; then
  # The corrected full no-motion profile is the sole Wi-Fi exception.  Its
  # one-way marker above has already forced local placement and motion=false;
  # every other profile, including every motion route, remains wired-only.
  [[ "${GO2_FORCE_NOMOTION_PROFILE:-}" == \
      workstation-full-nomotion-corrected-v1 \
    && "$GO2_RUNTIME_PLACEMENT" == workstation-local \
    && "$GO2_ALLOW_MOTION" == false ]] || {
      echo "GO2 network interface must be wired outside the corrected full no-motion profile: $GO2_NETWORK_INTERFACE" >&2
      exit 1
    }
  if ! active_connection_uuid="$(
    nmcli --get-values GENERAL.CON-UUID \
      device show "$GO2_NETWORK_INTERFACE" 2>/dev/null
  )"; then
    echo "could not resolve the active NetworkManager connection for the Go2 Wi-Fi interface" >&2
    exit 1
  fi
  [[ "$active_connection_uuid" =~ \
    ^[[:xdigit:]]{8}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{12}$ ]] || {
      echo "Go2 Wi-Fi interface has no valid active NetworkManager connection UUID" >&2
      exit 1
    }
  if ! active_wifi_ssid="$(
    nmcli --get-values 802-11-wireless.ssid \
      connection show uuid "$active_connection_uuid" 2>/dev/null
  )"; then
    echo "could not read the active Go2 Wi-Fi connection profile SSID" >&2
    exit 1
  fi
  [[ "$active_wifi_ssid" == "$NOMOTION_PRIVATE_WIFI_SSID" ]] || {
    echo "active Go2 Wi-Fi profile SSID must exactly equal $NOMOTION_PRIVATE_WIFI_SSID" >&2
    exit 1
  }
fi
[[ "$(cat -- "/sys/class/net/$GO2_NETWORK_INTERFACE/operstate")" == "up" ]] || {
  echo "GO2 network interface is not UP: $GO2_NETWORK_INTERFACE" >&2
  exit 1
}
mapfile -t GO2_IPV4_ADDRESSES < <(
  ip -o -4 addr show dev "$GO2_NETWORK_INTERFACE" \
    | awk '{print $4}'
)
if (( ${#GO2_IPV4_ADDRESSES[@]} != 1 )) \
  || [[ "${GO2_IPV4_ADDRESSES[0]:-}" != "192.168.123.99/24" ]]; then
  echo "GO2 interface must have exactly 192.168.123.99/24 and no other IPv4 address" >&2
  exit 1
fi
mapfile -t GO2_IPV6_ADDRESSES < <(
  ip -o -6 addr show dev "$GO2_NETWORK_INTERFACE" \
    | awk '{print $4}'
)
if (( ${#GO2_IPV6_ADDRESSES[@]} != 0 )); then
  echo "GO2 interface must have IPv6 disabled and no IPv6 addresses" >&2
  exit 1
fi
if ip -4 route show default dev "$GO2_NETWORK_INTERFACE" | grep -q .; then
  echo "GO2 interface must not have an IPv4 default route" >&2
  exit 1
fi
if ip -6 route show default dev "$GO2_NETWORK_INTERFACE" | grep -q .; then
  echo "GO2 interface must not have an IPv6 default route" >&2
  exit 1
fi
if ! GO2_NM_STATE="$(
  nmcli --get-values IP4.GATEWAY,IP4.DNS,IP6.GATEWAY,IP6.DNS \
    device show "$GO2_NETWORK_INTERFACE" 2>/dev/null \
    | sed '/^[[:space:]]*$/d'
)"; then
  echo "could not verify NetworkManager gateway/DNS state for the Go2 interface" >&2
  exit 1
fi
[[ -z "$GO2_NM_STATE" ]] || {
  echo "GO2 interface must not have any NetworkManager gateway or DNS entry" >&2
  exit 1
}

case "${GO2_ALLOW_MOTION,,}" in
  1|true|yes|on)
    [[ "${GO2_OPERATOR_PRESENT,,}" =~ ^(1|true|yes|on)$ ]] || {
      echo "motion requested but GO2_OPERATOR_PRESENT is not true" >&2
      exit 1
    }
    [[ "$GO2_SAFETY_ACK" == "I_UNDERSTAND_GO2_CAN_MOVE" ]] || {
      echo "motion requested but GO2_SAFETY_ACK does not match the required phrase" >&2
      exit 1
    }
    [[ -n "${GO2_ALLOWED_MODES:-}" ]] || {
      echo "motion requested but GO2_ALLOWED_MODES has not been recorded by the read-only audit" >&2
      exit 1
    }
    export GO2_ALLOW_MOTION=true
    export GO2_ALLOWED_MODES
    export SEMANTIC_INTENT_EXECUTION_MODE=live
    # This value is owned by the completed motion gate, never by .env or the
    # inherited shell. Navigation may reach canonical /cmd_vel only here.
    export ROBONIX_VELOCITY_OUTPUT_TOPIC=/cmd_vel
    echo "================================================================"
    echo " GO2 MOTION TRANSPORT ENABLED, BUT CHASSIS IS STILL DISARMED"
    echo " An operator must explicitly call the adapter arm service."
    echo "================================================================"
    ;;
  *)
    export GO2_ALLOW_MOTION=false
    # Liaison always forwards a valid ASR final to Pilot. Keep the deployment-
    # owned semantic model in empty-RTDL preview mode so a browser voice demo
    # can show intent and blockers without calling Navigation at all.
    export SEMANTIC_INTENT_EXECUTION_MODE=preview
    export ROBONIX_VELOCITY_OUTPUT_TOPIC=/robonix/nomotion/cmd_vel
    echo "================================================================"
    echo " GO2 MOTION LOCKED - READ-ONLY STATE/SENSOR MODE"
    echo " Boot will not arm, stand, sit, or move the robot."
    echo "================================================================"
    ;;
esac

GO2_CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_NETWORK_INTERFACE\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
if [[ -n "${CYCLONEDDS_URI:-}" && "$CYCLONEDDS_URI" != "$GO2_CYCLONEDDS_URI" ]]; then
  echo "Ignoring inherited CYCLONEDDS_URI; this deployment owns the dedicated Go2 NIC binding" >&2
fi
export CYCLONEDDS_URI="$GO2_CYCLONEDDS_URI"

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "missing /opt/ros/humble/setup.bash" >&2
  exit 1
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ "$START_UI_CLIENT_ONLY" == true ]]; then
  : # The dashboard consumes only standard ROS interfaces.
elif [[ -f "$UNITREE_ROS2_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$UNITREE_ROS2_SETUP"
else
  echo "missing Unitree ROS 2 message overlay: $UNITREE_ROS2_SETUP; run ./build.sh first" >&2
  exit 1
fi
set -u

command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 is not available after sourcing ROS 2 Humble" >&2
  exit 1
}
mkdir -p "$DEPLOY_DIR/logs" "$DEPLOY_DIR/rbnx-build/run"
chmod 700 "$DEPLOY_DIR/rbnx-build/run"
# shellcheck disable=SC1091
source "$DEPLOY_DIR/scripts/runtime_lease.sh"
# shellcheck disable=SC1091
source "$DEPLOY_DIR/scripts/runtime_signal_guard.sh"
go2_runtime_lease_acquire "$DEPLOY_DIR/rbnx-build/run" "$GO2_RUNTIME_PLACEMENT"
if [[ "$START_UI_CLIENT_ONLY" == false \
  && ( -e "$RBNX_STATE_PATH" || -L "$RBNX_STATE_PATH" ) ]]; then
  echo "existing boot state belongs to an earlier launch: $RBNX_STATE_PATH" >&2
  echo "refusing to overwrite it; inspect it and run ./stop.sh before retrying" >&2
  exit 76
fi
bash "$DEPLOY_DIR/scripts/check_runtime_ownership.sh" "$OWNERSHIP_PREFLIGHT" pre

if [[ "$START_UI_CLIENT_ONLY" == true ]]; then
  exec bash "$DEPLOY_DIR/scripts/start_ui_client_only.sh"
fi

bash "$DEPLOY_DIR/scripts/verify_upstream_compatibility.sh"

SEMANTIC_ROUTER_PID=""
SEMANTIC_ROUTER_START_TICKS=""
SEMANTIC_ROUTER_TOKEN=""
SEMANTIC_ROUTER_LEASE="$DEPLOY_DIR/rbnx-build/run/semantic-intent-router.lease"
SEMANTIC_ROUTER_LOCK="$DEPLOY_DIR/rbnx-build/run/semantic-intent-router.lock"
RBNX_BOOT_PID=""

process_is_same_and_live() {
  local pid="$1" expected_start="$2" stat rest
  local -a fields
  IFS= read -r stat < "/proc/${pid}/stat" 2>/dev/null || return 1
  rest="${stat#*) }"
  [[ "$rest" != "$stat" ]] || return 1
  read -r -a fields <<< "$rest"
  [[ "${fields[0]:-}" != Z && "${fields[19]:-}" == "$expected_start" ]]
}

remove_own_semantic_lease() {
  local recorded_token=""
  if [[ -r "$SEMANTIC_ROUTER_LEASE" ]]; then
    recorded_token="$(sed -n 's/^token=//p' "$SEMANTIC_ROUTER_LEASE" 2>/dev/null || true)"
  fi
  if [[ -n "$SEMANTIC_ROUTER_TOKEN" \
    && "$recorded_token" == "$SEMANTIC_ROUTER_TOKEN" ]]; then
    rm -f -- "$SEMANTIC_ROUTER_LEASE"
  fi
}

terminate_semantic_router() {
  if [[ -n "$SEMANTIC_ROUTER_PID" ]] \
    && process_is_same_and_live "$SEMANTIC_ROUTER_PID" "$SEMANTIC_ROUTER_START_TICKS"; then
    kill -TERM "$SEMANTIC_ROUTER_PID" 2>/dev/null || true
  fi
  if [[ -n "$SEMANTIC_ROUTER_PID" ]]; then
    wait "$SEMANTIC_ROUTER_PID" 2>/dev/null || true
  fi
  remove_own_semantic_lease
}

terminate_boot() {
  [[ -n "$RBNX_BOOT_PID" ]] || return 0
  trap - EXIT INT TERM
  if kill -0 "$RBNX_BOOT_PID" 2>/dev/null; then
    kill -TERM "$RBNX_BOOT_PID" 2>/dev/null || true
  fi
  wait "$RBNX_BOOT_PID" 2>/dev/null || true
  if [[ -e "$RBNX_STATE_PATH" || -L "$RBNX_STATE_PATH" ]]; then
    if ! python3 "$DEPLOY_DIR/scripts/shutdown_rbnx_boot_state.py" \
      --manifest "$MANIFEST" \
      --rbnx "$RBNX_CLI" \
      --expected-boot-pid "$RBNX_BOOT_PID"; then
      echo "warning: exact rbnx boot-state cleanup failed; state retained at $RBNX_STATE_PATH" >&2
    fi
  fi
  RBNX_BOOT_PID=""
}

cleanup_runtime() {
  trap - EXIT INT TERM
  terminate_boot
  terminate_semantic_router
}
go2_runtime_install_cleanup_traps

start_semantic_router() {
  local owner_start temporary expected_vlm_url deadline stable
  export SEMANTIC_INTENT_HOST=127.0.0.1
  export SEMANTIC_INTENT_PORT="${SEMANTIC_INTENT_PORT:-18080}"
  [[ "$SEMANTIC_INTENT_PORT" =~ ^[0-9]+$ ]] \
    && (( SEMANTIC_INTENT_PORT >= 1 && SEMANTIC_INTENT_PORT <= 65535 )) || {
      echo "SEMANTIC_INTENT_PORT must be an integer in 1..65535" >&2
      return 75
    }
  expected_vlm_url="http://127.0.0.1:${SEMANTIC_INTENT_PORT}/v1"
  if [[ -n "${VLM_BASE_URL:-}" && "$VLM_BASE_URL" != "$expected_vlm_url" ]]; then
    echo "VLM_BASE_URL conflicts with the deployment-owned semantic router" >&2
    return 75
  fi
  export VLM_BASE_URL="$expected_vlm_url"
  export VLM_API_KEY=local-no-secret
  export VLM_MODEL=go2-semantic-router

  exec {SEMANTIC_ROUTER_LOCK_FD}>"$SEMANTIC_ROUTER_LOCK"
  if ! flock --exclusive --nonblock "$SEMANTIC_ROUTER_LOCK_FD"; then
    echo "another semantic intent router launcher already holds the runtime lock" >&2
    return 75
  fi
  SEMANTIC_ROUTER_TOKEN="$(go2_runtime_new_token)"
  owner_start="$(go2_runtime_process_start_ticks "$$")"
  (
    go2_runtime_close_parent_only_fds
    exec bash "$DEPLOY_DIR/packages/semantic_intent_router/scripts/start.sh"
  ) &
  SEMANTIC_ROUTER_PID=$!
  SEMANTIC_ROUTER_START_TICKS="$(go2_runtime_process_start_ticks "$SEMANTIC_ROUTER_PID")" || {
    echo "semantic intent router exited before identity capture" >&2
    return 75
  }
  temporary="$(mktemp "${SEMANTIC_ROUTER_LEASE}.tmp.XXXXXX")"
  chmod 0600 "$temporary"
  {
    printf 'format=go2-semantic-router-lease-v1\n'
    printf 'token=%s\n' "$SEMANTIC_ROUTER_TOKEN"
    printf 'owner_pid=%s\n' "$$"
    printf 'owner_start_ticks=%s\n' "$owner_start"
    printf 'child_pid=%s\n' "$SEMANTIC_ROUTER_PID"
    printf 'child_start_ticks=%s\n' "$SEMANTIC_ROUTER_START_TICKS"
  } > "$temporary"
  mv -f -- "$temporary" "$SEMANTIC_ROUTER_LEASE"

  deadline=$((SECONDS + 15))
  stable=0
  while (( SECONDS <= deadline )); do
    if ! process_is_same_and_live "$SEMANTIC_ROUTER_PID" "$SEMANTIC_ROUTER_START_TICKS"; then
      echo "semantic intent router exited before becoming healthy" >&2
      return 75
    fi
    if python3 "$DEPLOY_DIR/scripts/check_semantic_intent_health.py" \
      "${expected_vlm_url}/models"; then
      stable=$((stable + 1))
      if (( stable >= 3 )); then
        echo "Semantic intent router health gate passed: ${expected_vlm_url}/models"
        return 0
      fi
    else
      stable=0
    fi
    sleep 0.2
  done
  echo "semantic intent router /v1/models health deadline expired" >&2
  return 75
}

start_semantic_router

(
  go2_runtime_close_parent_only_fds
  exec "$RBNX_CLI" boot --no-update-check -f "$MANIFEST" "$@"
) &
RBNX_BOOT_PID=$!
if ! bash "$DEPLOY_DIR/scripts/check_runtime_ownership.sh" "$OWNERSHIP_PREFLIGHT" post; then
  echo "post-start publisher ownership failed; stopping this boot process" >&2
  exit 74
fi
if ! process_is_same_and_live "$SEMANTIC_ROUTER_PID" "$SEMANTIC_ROUTER_START_TICKS"; then
  echo "semantic intent router exited during Robonix startup" >&2
  exit 75
fi
go2_runtime_wait_for_first_exit "$RBNX_BOOT_PID" "$SEMANTIC_ROUTER_PID"
if [[ -z "$EXITED_RUNTIME_PID" ]]; then
  echo "runtime wait ended without identifying a managed child; failing closed" >&2
  exit 75
fi
trap - EXIT INT TERM
if [[ "$EXITED_RUNTIME_PID" == "$SEMANTIC_ROUTER_PID" ]]; then
  echo "semantic intent router exited unexpectedly; stopping Robonix boot" >&2
  terminate_boot
  remove_own_semantic_lease
  exit 75
fi
terminate_semantic_router
exit "$RUNTIME_STATUS"
