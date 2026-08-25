#!/usr/bin/env bash
set -euo pipefail
umask 077

# One reviewed stage-1 NavigateToPose goal on a loaded map.  This launcher is
# intentionally not a reusable teleoperation or voice entry point.  It accepts
# no goal arguments: both the chassis and the exact short goal are bound into
# separate one-time permits before this process starts.

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(CDPATH= cd -- "$ROOT/../.." && pwd)"
PYTHON="$WORKSPACE_ROOT/.tools/rbnx-python/bin/python3"
RBNX="$WORKSPACE_ROOT/.tools/rbnx/bin/rbnx"
UNITREE_SETUP="${UNITREE_ROS2_SETUP:-$ROOT/rbnx-build/unitree_ros2/install/setup.bash}"
SEMANTIC_NAVIGATION_IDL_SETUP="$ROOT/packages/semantic_navigation/rbnx-build/codegen/ros2_idl/install/setup.bash"
MOTION_STATE_RELAY_BINARY="$ROOT/packages/go2_motion_state_relay/.build/ros/install/go2_motion_state_relay/lib/go2_motion_state_relay/workstation_motion_state_relay"
PROFILE=workstation-staged-nav2-corrected-v1
STAGE=stage1
MOTION_ACK=I_APPROVE_GO2_STAGED_NAV2_MOTION
STANDARD_MODE="${GO2_STAGED_NAV2_STANDARD_MODE:-true}"
PERSISTENT_MODE="${GO2_PERSISTENT_NAV2_MODE:-false}"
WIRELESS_INTERFACE=wlx500ff54809b8
WIRELESS_CONNECTION_NAME=Robonix-Go2
WIRELESS_CONNECTION_UUID=ce767234-9037-4a53-a5f4-aa7b6cbf743f
INTERNET_INTERFACE=wlo1
ROBOT_PRIVATE_IP=192.168.123.161
ORIN_PRIVATE_IP=192.168.123.18
PRIVATE_IPV4=192.168.123.99/24
# 2010 was re-observed on this Go2 on 2026-07-27 after the operator selected
# the official Classic gait. This broader explicit set validates only the
# inherited passive/no-motion source profile; it is not motion authorization.
PASSIVE_SOURCE_MARKERS=100,1002,2010
# The Classic gait on this physical Go2 was measured switching 2010 -> 100
# during the 2026-07-27 voice goal. The reverse value is also the previously
# observed Classic startup state. This exact pair is the only motion transition
# set; the current marker must still be one member of it and mode stays singular.
CLASSIC_MOTION_STATE_MARKERS=100,2010
NAV2_IMAGE_NAME=robonix-nav2
NAV2_STOPPER="$ROOT/third_party/service-navigation-rbnx/scripts/stop.sh"

die() {
  local status="$1"
  shift
  printf '%s\n' "$*" >&2
  exit "$status"
}

[[ "$#" -eq 0 ]] || die 2 "this staged launcher accepts no arguments"

# Never source .env or start.sh.  One explicit runtime acknowledgement is the
# motion gate.  Normal operation defaults to live planning from the current
# localization; the historical one-time evidence bundle remains available by
# setting GO2_STAGED_NAV2_STANDARD_MODE=false.
[[ "${GO2_STAGED_NAV2_RUN_ACK:-}" == "$MOTION_ACK" ]] \
  || die 2 "GO2_STAGED_NAV2_RUN_ACK must exactly equal $MOTION_ACK"
[[ "${GO2_NETWORK_INTERFACE:-}" == "$WIRELESS_INTERFACE" ]] \
  || die 2 "staged motion must use $WIRELESS_INTERFACE"

case "$STANDARD_MODE" in
  true|false) ;;
  *) die 2 "GO2_STAGED_NAV2_STANDARD_MODE must be true or false" ;;
esac
case "$PERSISTENT_MODE" in
  true|false) ;;
  *) die 2 "GO2_PERSISTENT_NAV2_MODE must be true or false" ;;
esac
[[ "$PERSISTENT_MODE" == false || "$STANDARD_MODE" == true ]] \
  || die 2 "persistent Nav2 requires standard mode"

if [[ "$STANDARD_MODE" == true && -z "${GO2_TIMESTAMP_APPROVAL_FILE:-}" ]]; then
  while IFS= read -r candidate; do
    if "$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_approval.py" \
      --require-affine "$candidate" >/dev/null 2>&1; then
      GO2_TIMESTAMP_APPROVAL_FILE="$candidate"
      export GO2_TIMESTAMP_APPROVAL_FILE
      break
    fi
  done < <(
    find "$ROOT/rbnx-build/run" -maxdepth 1 -type f \
      -name '*approval.json' -printf '%T@ %p\n' \
      | sort -nr | cut -d' ' -f2-
  )
fi

required_paths=(GO2_TIMESTAMP_APPROVAL_FILE)
if [[ "$STANDARD_MODE" == false ]]; then
  required_paths+=(
    GO2_STAGED_NAV2_PERMIT_FILE
    GO2_STAGED_NAV2_GOAL_PERMIT_FILE
    GO2_STAGED_NAV2_DDS_IDENTITY_EVIDENCE
    GO2_STAGED_NAV2_STATE_EVIDENCE
    GO2_STAGED_NAV2_GOAL_EVIDENCE
  )
fi
for variable in "${required_paths[@]}"; do
  value="${!variable:-}"
  [[ "$value" == /* && -f "$value" && ! -L "$value" ]] \
    || die 2 "$variable must name an absolute regular non-symlink file"
done
if [[ "$STANDARD_MODE" == false ]]; then
  [[ "$GO2_STAGED_NAV2_PERMIT_FILE" != "$GO2_STAGED_NAV2_GOAL_PERMIT_FILE" ]] \
    || die 2 "chassis and goal permits must be different files"
fi

for command in awk bash basename cat chmod cmp dirname docker env flock git \
  grep id install ip kill mkdir mktemp mv nmcli readlink rm sed seq sleep stat \
  timeout tr; do
  command -v "$command" >/dev/null 2>&1 \
    || die 3 "missing required command: $command"
done
[[ "$(readlink -f -- "$(command -v docker)")" == /usr/bin/docker ]] \
  || die 3 "staged Nav2 requires the audited /usr/bin/docker CLI"
[[ -x "$PYTHON" && -x "$RBNX" ]] \
  || die 3 "workspace-local Robonix Python/rbnx is missing"
[[ -f /opt/ros/humble/setup.bash && -f "$UNITREE_SETUP" \
  && -f "$SEMANTIC_NAVIGATION_IDL_SETUP" ]] \
  || die 3 "ROS Humble or a required generated message overlay is missing"
[[ -x "$MOTION_STATE_RELAY_BINARY" && -f "$MOTION_STATE_RELAY_BINARY" \
  && ! -L "$MOTION_STATE_RELAY_BINARY" ]] \
  || die 3 "built C++ motion-state relay is missing"

# Pin every Docker check to the local daemon.  A shell proxy or remote Docker
# context cannot redirect inspection or cleanup.
unset DOCKER_HOST DOCKER_TLS_VERIFY DOCKER_CERT_PATH
export DOCKER_CONTEXT=default
docker version --format '{{.Server.Version}}' >/dev/null 2>&1 \
  || die 3 "the local Docker daemon is unavailable"
docker_endpoint="$(
  docker context inspect default --format '{{.Endpoints.docker.Host}}'
)" || die 3 "the default Docker context endpoint is unreadable"
[[ "$docker_endpoint" == unix:///var/run/docker.sock ]] \
  || die 3 "staged Nav2 requires default Docker at /var/run/docker.sock"

export ROBONIX_DEPLOY_DIR="$ROOT"
export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"
export ROBONIX_SOURCE_ROOT="$WORKSPACE_ROOT/upstream/robonix-go2-build"
export ROBONIX_SOURCE_PATH="$ROBONIX_SOURCE_ROOT"
export CARGO_HOME="$WORKSPACE_ROOT/.tools/cargo"
export CARGO_TARGET_DIR="$WORKSPACE_ROOT/.tools/cargo-target/robonix"
export RUSTUP_HOME="$WORKSPACE_ROOT/.tools/rustup"
export ROBONIX_BUILD_PROFILE="${ROBONIX_BUILD_PROFILE:-debug}"
case "$ROBONIX_BUILD_PROFILE" in
  debug|release) ;;
  *) die 3 "ROBONIX_BUILD_PROFILE must be debug or release" ;;
esac
export ROBONIX_SYSTEM_BIN_DIR="$CARGO_TARGET_DIR/$ROBONIX_BUILD_PROFILE"
export PATH="$WORKSPACE_ROOT/.tools/rbnx-python/bin:$WORKSPACE_ROOT/.tools/rbnx/bin:$ROBONIX_SYSTEM_BIN_DIR:$PATH"
"$PYTHON" "$ROOT/scripts/validate_robonix_home.py" \
  "$ROBONIX_HOME/config.yaml" "$ROBONIX_SOURCE_ROOT" >/dev/null

ROBONIX_API_DIR="$("$RBNX" path robonix-api)" \
  || die 3 "could not resolve the pinned robonix-api package"
ROBONIX_API_DIR="$(readlink -f -- "$ROBONIX_API_DIR")"
[[ -d "$ROBONIX_API_DIR" && ! -L "$ROBONIX_API_DIR" ]] \
  || die 3 "robonix-api package path is not a canonical directory"
if [[ "$PERSISTENT_MODE" == true ]]; then
  [[ -d "$ROBONIX_SOURCE_PATH/services/speech" \
    && -d "$ROBONIX_SOURCE_PATH/examples/webots/primitives/audio_client_bridge" ]] \
    || die 3 "audited Robonix source packages are missing"
fi
readonly ROBONIX_API_DIR

validate_wireless_topology() {
  "$PYTHON" "$ROOT/scripts/validate_first_motion_network.py" \
    --interface "$WIRELESS_INTERFACE" \
    --transport wireless-private \
    --internet-interface "$INTERNET_INTERFACE" \
    --robot-ip "$ROBOT_PRIVATE_IP" \
    --orin-ip "$ORIN_PRIVATE_IP" \
    --connection-uuid "$WIRELESS_CONNECTION_UUID" \
    --connection-name "$WIRELESS_CONNECTION_NAME"
}

# This command is strictly observational.  It does not activate a connection,
# change an address, add a route, or modify NetworkManager.
validate_wireless_topology \
  || die 4 "private Wi-Fi topology failed its staged preflight"

ip link show dev "$WIRELESS_INTERFACE" >/dev/null 2>&1 \
  || die 4 "private Wi-Fi interface does not exist"
interface_sysfs="$(readlink -f -- "/sys/class/net/$WIRELESS_INTERFACE")"
[[ -n "$interface_sysfs" && "$interface_sysfs" != *"/virtual/"* \
  && -d "/sys/class/net/$WIRELESS_INTERFACE/wireless" ]] \
  || die 4 "private Go2 interface must be a physical Wi-Fi device"
mapfile -t private_ipv4 < <(
  ip -o -4 addr show dev "$WIRELESS_INTERFACE" | awk '{print $4}'
)
[[ "${#private_ipv4[@]}" -eq 1 && "${private_ipv4[0]}" == "$PRIVATE_IPV4" ]] \
  || die 4 "private Wi-Fi must have exactly $PRIVATE_IPV4"

"$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_approval.py" \
  --require-affine "$GO2_TIMESTAMP_APPROVAL_FILE" >/dev/null \
  || die 5 "a current affine timestamp approval is required"

if [[ "$STANDARD_MODE" == true ]]; then
  if [[ "$PERSISTENT_MODE" == true ]]; then
    [[ -n "${GO2_PERSISTENT_NAV2_ALLOWED_MODE:-}" \
      && -n "${GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER:-}" ]] \
      || die 5 "persistent Nav2 requires explicit current read-only mode and state marker"
    observed_mode="$GO2_PERSISTENT_NAV2_ALLOWED_MODE"
    observed_marker="$GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER"
  else
    observed_mode="${GO2_STAGED_NAV2_ALLOWED_MODE:-0}"
    observed_marker="${GO2_STAGED_NAV2_ALLOWED_STATE_MARKER:-100}"
  fi
  observed_gait=0
else
  read -r observed_mode observed_marker observed_gait < <(
    "$PYTHON" "$ROOT/scripts/validate_first_motion_state_evidence.py" \
      "$GO2_STAGED_NAV2_STATE_EVIDENCE"
  ) || die 5 "stationary staged state marker evidence is required"
fi
for value in "$observed_mode" "$observed_marker" "$observed_gait"; do
  [[ "$value" =~ ^[0-9]+$ ]] \
    || die 5 "validated staged state contains a malformed integer"
done
case ",$CLASSIC_MOTION_STATE_MARKERS," in
  *,"$observed_marker",*) ;;
  *) die 5 "staged Nav2 current marker is outside the reviewed Classic set" ;;
esac

export GO2_ALLOWED_MODES="$observed_mode"
if [[ "$PERSISTENT_MODE" == true ]]; then
  export GO2_ALLOWED_STATE_MARKERS="$CLASSIC_MOTION_STATE_MARKERS"
else
  # One-shot staged permits bind the exact observed marker.  Keep that legacy
  # runtime claim unchanged so permit validation and provider consumption see
  # the same singleton allowlist.
  export GO2_ALLOWED_STATE_MARKERS="$observed_marker"
fi
export GO2_STAGED_NAV2_ALLOWED_MODE="$observed_mode"
export GO2_STAGED_NAV2_ALLOWED_STATE_MARKER="$observed_marker"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID <= 232 )) \
  || die 5 "ROS_DOMAIN_ID must be an integer in 0..232"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$WIRELESS_INTERFACE\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
export UNITREE_ROS2_SETUP="$UNITREE_SETUP"
export GO2_ALLOW_MOTION=true
export GO2_OPERATOR_PRESENT=true
export GO2_SAFETY_ACK=I_UNDERSTAND_GO2_CAN_MOVE
export GO2_MOTION_PROFILE="$PROFILE"
export GO2_STAGED_NAV2_STAGE="$STAGE"
export GO2_STAGED_NAV2_GUARD_ACK="$MOTION_ACK"
if [[ "$STANDARD_MODE" == true ]]; then
  export GO2_STAGED_NAV2_RUNTIME_ACK="$MOTION_ACK"
else
  unset GO2_STAGED_NAV2_RUNTIME_ACK
fi
export GO2_RUNTIME_PLACEMENT=workstation-local
export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1
export ROBONIX_ADVERTISE_HOST=127.0.0.1
export ROBONIX_NAV2_FORCE=docker
export ROBONIX_NAV2_IMAGE="$NAV2_IMAGE_NAME"
if [[ "$PERSISTENT_MODE" == true ]]; then
  export ROBONIX_VELOCITY_OUTPUT_TOPIC=/go2/staged_nav2/cmd_vel
  export SEMANTIC_INTENT_EXECUTION_MODE=live
else
  export ROBONIX_VELOCITY_OUTPUT_TOPIC=/cmd_vel_guard_input
fi
readonly NAV2_EXPECTED_VELOCITY_TOPIC="$ROBONIX_VELOCITY_OUTPUT_TOPIC"
# Do not set ROBONIX_CAPABILITY_ID globally here.  The official Mapping and
# Navigation package launchers own their distinct defaults (mapping and nav2);
# exporting nav2 for the whole rbnx boot makes the two providers collide.
export ROBONIX_PKG_HOST_DIR="$ROOT/third_party/service-navigation-rbnx"
export ROBONIX_FORCE_CPU=1
export SCENE_CG_FORCE_CPU=1

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$UNITREE_SETUP"
# MapLifecycle is a generated workspace interface used by both the plan-only
# evidence helper and the live staged readiness checks.  Source its normal
# install prefix explicitly so a fresh shell does not depend on a developer's
# inherited PYTHONPATH.
# shellcheck disable=SC1090
source "$SEMANTIC_NAVIGATION_IDL_SETUP"
set -u

require_no_chassis_adapter() {
  local existing_nodes
  existing_nodes="$(
    LC_ALL=C timeout --signal=INT --kill-after=1s 3s \
      ros2 node list --no-daemon 2>/dev/null
  )" || return 1
  ! grep -Fxq /go2_chassis_adapter <<<"$existing_nodes"
}
require_no_chassis_adapter \
  || die 5 "a chassis adapter already exists before staged ownership"

verify_staged_nav2_stop_hook() {
  local package="$ROOT/third_party/service-navigation-rbnx"
  local mode="" expected_pin="" stage="" indexed_path="" actual_pin="" stopper=""
  read -r mode expected_pin stage indexed_path < <(
    git -C "$ROOT" ls-files --stage -- third_party/service-navigation-rbnx
  )
  [[ "$mode" == 160000 && "$stage" == 0 \
    && "$indexed_path" == third_party/service-navigation-rbnx \
    && "$expected_pin" =~ ^[0-9a-f]{40}$ ]] || return 1
  actual_pin="$(git -C "$package" rev-parse HEAD 2>/dev/null)" || return 1
  [[ "$actual_pin" == "$expected_pin" ]] || return 1
  git -C "$package" diff --quiet -- scripts/start.sh scripts/stop.sh \
    || return 1
  git -C "$package" diff --cached --quiet -- scripts/start.sh scripts/stop.sh \
    || return 1
  stopper="$(readlink -f -- "$NAV2_STOPPER")" || return 1
  [[ "$stopper" == "$package/scripts/stop.sh" \
    && -f "$NAV2_STOPPER" && ! -L "$NAV2_STOPPER" ]]
}
verify_staged_nav2_stop_hook \
  || die 5 "pinned Nav2 Docker stop hook failed integrity validation"

cleanup_owned_staged_nav2_namespace() {
  local id="" identity="" name="" image="" shape="" entry=""
  local invocation="" package_host="" velocity="" capability=""
  local invocation_count=0 package_count=0 velocity_count=0 capability_count=0
  local prefix="$ROOT/rbnx-build/run/stg." suffix="" remaining=""
  local -a ids=() environment=()
  mapfile -t ids < <(
    docker ps --all --quiet --no-trunc \
      --filter 'name=^/robonix_nav2_staged_'
  ) || return 1
  for id in "${ids[@]}"; do
    [[ "$id" =~ ^[0-9a-f]{64}$ ]] || return 1
    identity="$(docker inspect --type container \
      --format '{{.Name}}|{{.Config.Image}}' "$id")" || return 1
    IFS='|' read -r name image <<<"$identity"
    [[ "$name" =~ ^/robonix_nav2_staged_[0-9a-f]{32}$ \
      && "$image" == "$NAV2_IMAGE_NAME" ]] || return 1
    shape="$(docker inspect --type container \
      --format '{{.HostConfig.AutoRemove}}|{{.HostConfig.NetworkMode}}|{{.HostConfig.IpcMode}}|{{.HostConfig.RestartPolicy.Name}}|{{.HostConfig.Privileged}}|{{.HostConfig.PidMode}}' \
      "$id")" || return 1
    [[ "$shape" == "true|host|host|no|false|" ]] || return 1
    invocation=""
    package_host=""
    velocity=""
    capability=""
    invocation_count=0
    package_count=0
    velocity_count=0
    capability_count=0
    mapfile -t environment < <(
      docker inspect --type container \
        --format '{{range .Config.Env}}{{println .}}{{end}}' "$id"
    ) || return 1
    for entry in "${environment[@]}"; do
      case "$entry" in
        RBNX_INVOCATION_CWD=*)
          invocation="${entry#RBNX_INVOCATION_CWD=}"
          invocation_count=$((invocation_count + 1))
          ;;
        ROBONIX_PKG_HOST_DIR=*)
          package_host="${entry#ROBONIX_PKG_HOST_DIR=}"
          package_count=$((package_count + 1))
          ;;
        ROBONIX_VELOCITY_OUTPUT_TOPIC=*)
          velocity="${entry#ROBONIX_VELOCITY_OUTPUT_TOPIC=}"
          velocity_count=$((velocity_count + 1))
          ;;
        ROBONIX_CAPABILITY_ID=*)
          capability="${entry#ROBONIX_CAPABILITY_ID=}"
          capability_count=$((capability_count + 1))
          ;;
      esac
    done
    [[ "$invocation_count" -eq 1 && "$package_count" -eq 1 \
      && "$velocity_count" -eq 1 && "$capability_count" -eq 1 ]] \
      || return 1
    [[ "$invocation" == "$prefix"* ]] || return 1
    suffix="${invocation#"$prefix"}"
    [[ "$suffix" =~ ^[A-Za-z0-9]{6}$ \
      && -d "$invocation" && ! -L "$invocation" \
      && "$package_host" == "$ROOT/third_party/service-navigation-rbnx" \
      && ( "$velocity" == /cmd_vel_guard_input \
        || "$velocity" == /go2/staged_nav2/cmd_vel ) \
      && "$capability" == nav2 ]] \
      || return 1
    echo "Recovering owned stale staged Nav2 container: ${name#/}" >&2
    env -i PATH=/usr/bin:/bin DOCKER_CONTEXT=default \
      ROBONIX_NAV2_FORCE=docker \
      ROBONIX_NAV2_CONTAINER="$id" \
      RBNX_PACKAGE_ROOT="$ROOT/third_party/service-navigation-rbnx" \
      bash "$NAV2_STOPPER" || return 1
    remaining="$(docker ps --all --quiet --no-trunc --filter "id=$id")" \
      || return 1
    [[ -z "$remaining" ]] || return 1
  done
}

cleanup_owned_staged_nav2_namespace \
  || die 5 "staged Nav2 namespace contains an unowned or invalid container"
existing_staged_names="$(
  docker ps --all --format '{{.Names}}' \
    --filter 'name=^/robonix_nav2_staged_'
)" || die 5 "could not enumerate the staged Nav2 namespace"
[[ -z "$existing_staged_names" ]] \
  || die 5 "owned staged Nav2 namespace cleanup did not complete"

mkdir -p "$ROOT/rbnx-build/run"
chmod 700 "$ROOT/rbnx-build/run"
# shellcheck disable=SC1091
source "$ROOT/scripts/runtime_lease.sh"
go2_runtime_lease_acquire "$ROOT/rbnx-build/run" "$PROFILE"

RUN_DIR="$(mktemp -d "$ROOT/rbnx-build/run/stg.XXXXXX")"
chmod 700 "$RUN_DIR"
export GO2_STAGED_NAV2_RUN_DIR="$RUN_DIR"
export GO2_STAGED_NAV2_ACTION_RESULT_FILE="$RUN_DIR/goal-action-result.json"
export GO2_STAGED_NAV2_RESULT_FILE="$RUN_DIR/measured-result.json"
SESSION_TOKEN="$(tr -d '-' < /proc/sys/kernel/random/uuid)"
[[ "$SESSION_TOKEN" =~ ^[0-9a-f]{32}$ ]] \
  || die 6 "could not create a private staged container token"
NAV2_CONTAINER_NAME="robonix_nav2_staged_${SESSION_TOKEN}"
readonly NAV2_CONTAINER_NAME
export ROBONIX_NAV2_CONTAINER="$NAV2_CONTAINER_NAME"
export GO2_SDK_SOCKET="$RUN_DIR/s"
(( ${#GO2_SDK_SOCKET} < 108 )) \
  || die 6 "private staged SDK socket exceeds the Unix sun_path limit"

BUNDLE=()
CONSUMED_CHASSIS_PERMIT=""
CONSUMED_GOAL_PERMIT=""
if [[ "$STANDARD_MODE" == true ]]; then
  export GO2_STAGED_NAV2_SESSION_ID="nav2-${SESSION_TOKEN}"
  export GO2_STAGED_NAV2_PAIR_ID="pair-${SESSION_TOKEN}"
  export GO2_STAGED_NAV2_MAP_ID="${GO2_NAV2_MAP_ID:-go2_second_location_refined_20260724_01}"
  export GO2_STAGED_NAV2_MAP_GENERATION="${GO2_NAV2_MAP_GENERATION:-1}"
  export GO2_STAGED_NAV2_GOAL_SOURCE=operator_reviewed_short_goal
  export GO2_STAGED_NAV2_TARGET_ID="${GO2_NAV2_TARGET_ID:-runtime_forward20cm}"
  export GO2_STAGED_NAV2_EXPECTED_GOAL_X=0
  export GO2_STAGED_NAV2_EXPECTED_GOAL_Y=0
  export GO2_STAGED_NAV2_EXPECTED_GOAL_YAW=0
  export GO2_STAGED_NAV2_EXPECTED_START_X=0
  export GO2_STAGED_NAV2_EXPECTED_START_Y=0
  export GO2_STAGED_NAV2_EXPECTED_START_YAW=0
  export GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256="$(printf '0%.0s' {1..64})"
else
  mapfile -t BUNDLE < <(
    "$PYTHON" "$ROOT/scripts/validate_staged_nav2_permit_bundle.py" \
      --chassis-permit "$GO2_STAGED_NAV2_PERMIT_FILE" \
      --goal-permit "$GO2_STAGED_NAV2_GOAL_PERMIT_FILE" \
      --package-root "$ROOT" \
      --network-interface "$WIRELESS_INTERFACE" \
      --allowed-mode "$observed_mode" \
      --allowed-state-marker "$observed_marker" \
      --ipc-socket "$GO2_SDK_SOCKET" \
      --dds-evidence "$GO2_STAGED_NAV2_DDS_IDENTITY_EVIDENCE" \
      --state-evidence "$GO2_STAGED_NAV2_STATE_EVIDENCE" \
      --time-evidence "$GO2_TIMESTAMP_APPROVAL_FILE" \
      --goal-evidence "$GO2_STAGED_NAV2_GOAL_EVIDENCE"
  ) || die 6 "matched staged permit bundle was rejected"
  [[ "${#BUNDLE[@]}" -eq 15 ]] \
    || die 6 "staged permit validator returned an unexpected claim count"
  export GO2_STAGED_NAV2_SESSION_ID="${BUNDLE[0]}"
  export GO2_STAGED_NAV2_PAIR_ID="${BUNDLE[1]}"
  export GO2_STAGED_NAV2_MAP_ID="${BUNDLE[2]}"
  export GO2_STAGED_NAV2_MAP_GENERATION="${BUNDLE[3]}"
  export GO2_STAGED_NAV2_GOAL_SOURCE="${BUNDLE[4]}"
  export GO2_STAGED_NAV2_TARGET_ID="${BUNDLE[5]}"
  export GO2_STAGED_NAV2_EXPECTED_GOAL_X="${BUNDLE[6]}"
  export GO2_STAGED_NAV2_EXPECTED_GOAL_Y="${BUNDLE[7]}"
  export GO2_STAGED_NAV2_EXPECTED_GOAL_YAW="${BUNDLE[8]}"
  export GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256="${BUNDLE[9]}"
  CHASSIS_PERMIT_ID="${BUNDLE[10]}"
  GOAL_PERMIT_ID="${BUNDLE[11]}"
  export GO2_STAGED_NAV2_EXPECTED_START_X="${BUNDLE[12]}"
  export GO2_STAGED_NAV2_EXPECTED_START_Y="${BUNDLE[13]}"
  export GO2_STAGED_NAV2_EXPECTED_START_YAW="${BUNDLE[14]}"
  CONSUMED_CHASSIS_PERMIT="${GO2_STAGED_NAV2_PERMIT_FILE}.consumed-${CHASSIS_PERMIT_ID}"
  CONSUMED_GOAL_PERMIT="${GO2_STAGED_NAV2_GOAL_PERMIT_FILE}.consumed-${GOAL_PERMIT_ID}"
  [[ ! -e "$CONSUMED_CHASSIS_PERMIT" && ! -L "$CONSUMED_CHASSIS_PERMIT" \
    && ! -e "$CONSUMED_GOAL_PERMIT" && ! -L "$CONSUMED_GOAL_PERMIT" ]] \
    || die 6 "a consumed-permit destination already exists"
fi
export GO2_MAP_ID="$GO2_STAGED_NAV2_MAP_ID"

CONFIG_DIR="$RUN_DIR/config"
install -d -m 700 -- "$CONFIG_DIR"
materialize_runtime_config() {
  local source="$1" destination="$2"
  [[ -f "$source" && ! -L "$source" ]] \
    || die 6 "runtime config must be a regular non-symlink file: $source"
  install -m 600 -- "$source" "$destination"
}
materialize_runtime_config \
  "$ROOT/config/rtabmap_params.yaml" "$CONFIG_DIR/rtabmap_params.yaml"
"$PYTHON" "$ROOT/scripts/render_staged_nav2_params.py" \
  --source "$ROOT/config/nav2_params_go2.yaml" \
  --output "$CONFIG_DIR/nav2_params_go2.yaml" >/dev/null
materialize_runtime_config \
  "$ROOT/config/navigate.xml" "$CONFIG_DIR/navigate.xml"
materialize_runtime_config \
  "$ROOT/config/navigate_through_poses.xml" \
  "$CONFIG_DIR/navigate_through_poses.xml"
if [[ "$PERSISTENT_MODE" == true ]]; then
  semantic_landmarks_source="${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}"
  if [[ "$semantic_landmarks_source" != /* ]]; then
    semantic_landmarks_source="$ROOT/$semantic_landmarks_source"
  fi
  materialize_runtime_config \
    "$semantic_landmarks_source" \
    "$CONFIG_DIR/semantic_landmarks.yaml"
fi

MANIFEST="$RUN_DIR/robonix_manifest.yaml"
MANIFEST_RENDERER="$ROOT/deploy/time-sync/render_workstation_staged_nav2_manifest.py"
if [[ "$PERSISTENT_MODE" == true ]]; then
  MANIFEST_RENDERER="$ROOT/deploy/time-sync/render_workstation_persistent_nav2_manifest.py"
fi
"$PYTHON" "$MANIFEST_RENDERER" \
  --base "$ROOT/robonix_manifest.yaml" \
  --state-marker "$observed_marker" \
  --passive-state-markers "$PASSIVE_SOURCE_MARKERS" \
  --output "$MANIFEST" >/dev/null

IDENTITY_READY="$RUN_DIR/identity-ready.json"
IDENTITY_FAULT="$RUN_DIR/identity-fault.json"
STAMP_READY="$RUN_DIR/stamp-ready.json"
STAMP_FAULT="$RUN_DIR/stamp-fault.json"
MOTION_STATE_READY="$RUN_DIR/motion-state-ready.json"
MOTION_STATE_FAULT="$RUN_DIR/motion-state-fault.json"
CLOUD_READY="$RUN_DIR/cloud-ready.json"
CLOUD_FAULT="$RUN_DIR/cloud-fault.json"
PRE_GUARD_RECEIPT="$RUN_DIR/pre-guard-readiness.json"
LIVE_GOAL_EVIDENCE="$RUN_DIR/live-goal-evidence.json"
POST_GUARD_RECEIPT="$RUN_DIR/post-guard-readiness.json"

IDENTITY_PID=""
STAMP_PID=""
MOTION_STATE_PID=""
CLOUD_PID=""
SEMANTIC_PID=""
BOOT_PID=""
GUARD_PID=""
DISPATCH_PID=""
cleanup_started=false

terminate_child() {
  local pid="$1" count
  [[ -n "$pid" && "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM "$pid" 2>/dev/null || true
  for count in $(seq 1 50); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL "$pid" 2>/dev/null || true
}

terminate_boot_child() {
  local pid="$1" count
  [[ -n "$pid" && "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM "$pid" 2>/dev/null || true
  # rbnx's canonical TERM path tears down recorded component process groups.
  # Do not truncate that multi-component path with the generic 5 second child
  # deadline.  A forced KILL is an explicit cleanup failure.
  for count in $(seq 1 450); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL "$pid" 2>/dev/null || true
  return 1
}

verify_current_nav2_container() {
  local id="$1" require_running="${2:-true}"
  local identity="" shape=""
  local running="" entry="" invocation="" package_host="" velocity="" capability=""
  local invocation_count=0 package_count=0 velocity_count=0 capability_count=0
  local -a environment=()
  [[ "$id" =~ ^[0-9a-f]{64}$ ]] || return 1
  identity="$(docker inspect --type container \
    --format '{{.Name}}|{{.Config.Image}}' "$id")" || return 1
  [[ "$identity" == "/${NAV2_CONTAINER_NAME}|${NAV2_IMAGE_NAME}" ]] || return 1
  # Match the official package's runtime requirements without pinning the
  # mutable local image tag or its complete bind-mount list.  Those stricter
  # local checks rejected an otherwise valid official Nav2 container after an
  # image rebuild and added no controller-ownership guarantee.
  shape="$(docker inspect --type container \
    --format '{{.HostConfig.AutoRemove}}|{{.HostConfig.NetworkMode}}|{{.HostConfig.IpcMode}}|{{.HostConfig.RestartPolicy.Name}}|{{.HostConfig.Privileged}}|{{.HostConfig.PidMode}}' \
    "$id")" || return 1
  [[ "$shape" == "true|host|host|no|false|" ]] || return 1
  running="$(docker inspect --type container --format '{{.State.Running}}' "$id")" \
    || return 1
  [[ "$running" == true || "$running" == false ]] || return 1
  [[ "$require_running" == false || "$running" == true ]] || return 1
  mapfile -t environment < <(
    docker inspect --type container \
      --format '{{range .Config.Env}}{{println .}}{{end}}' "$id"
  )
  for entry in "${environment[@]}"; do
    case "$entry" in
      RBNX_INVOCATION_CWD=*)
        invocation="${entry#RBNX_INVOCATION_CWD=}"
        invocation_count=$((invocation_count + 1))
        ;;
      ROBONIX_PKG_HOST_DIR=*)
        package_host="${entry#ROBONIX_PKG_HOST_DIR=}"
        package_count=$((package_count + 1))
        ;;
      ROBONIX_VELOCITY_OUTPUT_TOPIC=*)
        velocity="${entry#ROBONIX_VELOCITY_OUTPUT_TOPIC=}"
        velocity_count=$((velocity_count + 1))
        ;;
      ROBONIX_CAPABILITY_ID=*)
        capability="${entry#ROBONIX_CAPABILITY_ID=}"
        capability_count=$((capability_count + 1))
        ;;
    esac
  done
  [[ "$invocation_count" -eq 1 && "$package_count" -eq 1 \
    && "$velocity_count" -eq 1 && "$capability_count" -eq 1 ]] || return 1
  [[ "$invocation" == "$RUN_DIR" \
    && "$package_host" == "$ROOT/third_party/service-navigation-rbnx" \
    && "$velocity" == "$NAV2_EXPECTED_VELOCITY_TOPIC" \
    && "$capability" == nav2 ]] \
    || return 1
}

cleanup_current_nav2_container() {
  local id="" remaining="" replacement=""
  if ! id="$(docker inspect --type container --format '{{.Id}}' \
    "$NAV2_CONTAINER_NAME" 2>/dev/null)"; then
    replacement="$(docker ps --all --quiet --no-trunc \
      --filter "name=^/${NAV2_CONTAINER_NAME}$")" || return 1
    [[ -z "$replacement" ]]
    return
  fi
  verify_current_nav2_container "$id" false || return 1
  verify_staged_nav2_stop_hook || return 1
  env -i PATH=/usr/bin:/bin DOCKER_CONTEXT=default \
    ROBONIX_NAV2_FORCE=docker \
    ROBONIX_NAV2_CONTAINER="$id" \
    RBNX_PACKAGE_ROOT="$ROOT/third_party/service-navigation-rbnx" \
    bash "$NAV2_STOPPER" || return 1
  remaining="$(docker ps --all --quiet --no-trunc --filter "id=$id")" \
    || return 1
  [[ -z "$remaining" ]] || return 1
  replacement="$(docker ps --all --quiet --no-trunc \
    --filter "name=^/${NAV2_CONTAINER_NAME}$")" || return 1
  [[ -z "$replacement" ]]
}

cleanup() {
  local status=$? cleanup_status=0 guard_status=0 shutdown_status=0
  [[ "$cleanup_started" == false ]] || exit "$status"
  cleanup_started=true
  trap - EXIT HUP INT TERM
  set +e

  # Command production ends first.  The guard stays alive long enough to
  # cancel, publish zero and confirm disarm.
  terminate_child "$DISPATCH_PID"
  [[ -z "$DISPATCH_PID" ]] || wait "$DISPATCH_PID" 2>/dev/null
  terminate_child "$GUARD_PID"
  if [[ -n "$GUARD_PID" ]]; then
    wait "$GUARD_PID" 2>/dev/null
    guard_status=$?
    (( guard_status == 0 )) || cleanup_status=1
  fi

  if [[ -n "$BOOT_PID" && ( -e "$RUN_DIR/rbnx-boot/state.json" \
    || -L "$RUN_DIR/rbnx-boot/state.json" ) ]]; then
    timeout --signal=TERM --kill-after=5s 45s \
      "$PYTHON" "$ROOT/scripts/shutdown_rbnx_boot_state.py" \
      --manifest "$MANIFEST" --rbnx "$RBNX" \
      --expected-boot-pid "$BOOT_PID"
    shutdown_status=$?
    (( shutdown_status == 0 )) || cleanup_status=1
  fi
  terminate_boot_child "$BOOT_PID" || cleanup_status=1
  [[ -z "$BOOT_PID" ]] || wait "$BOOT_PID" 2>/dev/null
  if [[ -e "$RUN_DIR/rbnx-boot/state.json" \
    || -L "$RUN_DIR/rbnx-boot/state.json" ]]; then
    cleanup_status=1
  fi
  cleanup_current_nav2_container || cleanup_status=1

  terminate_child "$CLOUD_PID"
  terminate_child "$SEMANTIC_PID"
  terminate_child "$MOTION_STATE_PID"
  terminate_child "$STAMP_PID"
  terminate_child "$IDENTITY_PID"
  [[ -z "$CLOUD_PID" ]] || wait "$CLOUD_PID" 2>/dev/null
  [[ -z "$SEMANTIC_PID" ]] || wait "$SEMANTIC_PID" 2>/dev/null
  [[ -z "$MOTION_STATE_PID" ]] || wait "$MOTION_STATE_PID" 2>/dev/null
  [[ -z "$STAMP_PID" ]] || wait "$STAMP_PID" 2>/dev/null
  [[ -z "$IDENTITY_PID" ]] || wait "$IDENTITY_PID" 2>/dev/null

  if (( status == 0 && cleanup_status != 0 )); then
    status=9
  fi
  exit "$status"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

fault_first() {
  local label="" path=""
  while (( $# )); do
    label="$1"
    path="$2"
    shift 2
    if [[ -s "$path" ]]; then
      echo "$label faulted; detail: $path" >&2
      return 1
    fi
  done
}

require_state_chain_alive() {
  local pid=""
  fault_first \
    "writer identity monitor" "$IDENTITY_FAULT" \
    "timestamp discipline" "$STAMP_FAULT" \
    "motion-state relay" "$MOTION_STATE_FAULT" \
    "cloud relay" "$CLOUD_FAULT" || return 1
  for pid in "$IDENTITY_PID" "$STAMP_PID" "$MOTION_STATE_PID" "$CLOUD_PID"; do
    [[ -z "$pid" ]] || kill -0 "$pid" 2>/dev/null || return 1
  done
}

wait_private_ready() {
  local label="$1" pid="$2" ready="$3" fault="$4" seconds="$5"
  local deadline=$((SECONDS + seconds))
  while true; do
    require_state_chain_alive \
      || die 7 "a corrected state-chain owner faulted while starting $label"
    [[ ! -s "$fault" ]] || die 7 "$label faulted; detail: $fault"
    kill -0 "$pid" 2>/dev/null \
      || die 7 "$label exited before readiness"
    [[ ! -s "$ready" ]] || break
    (( SECONDS < deadline )) || die 7 "$label readiness deadline expired"
    sleep 0.1
  done
  [[ ! -s "$fault" ]] || die 7 "$label faulted at READY boundary"
}

(
  go2_runtime_close_parent_only_fds
  exec "$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_identity_monitor.py" \
    --approval-file "$GO2_TIMESTAMP_APPROVAL_FILE" \
    --ready-file "$IDENTITY_READY" --fault-file "$IDENTITY_FAULT"
) >"$RUN_DIR/identity.log" 2>&1 &
IDENTITY_PID=$!
wait_private_ready \
  "writer identity monitor" "$IDENTITY_PID" "$IDENTITY_READY" "$IDENTITY_FAULT" 20

(
  go2_runtime_close_parent_only_fds
  exec "$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_stamp_node.py" \
    --mode affine --profile motion \
    --approval-file "$GO2_TIMESTAMP_APPROVAL_FILE" \
    --ready-file "$STAMP_READY" --fault-file "$STAMP_FAULT"
) >"$RUN_DIR/stamp.log" 2>&1 &
STAMP_PID=$!
wait_private_ready \
  "timestamp discipline" "$STAMP_PID" "$STAMP_READY" "$STAMP_FAULT" 90

require_no_chassis_adapter \
  || die 7 "chassis adapter appeared before corrected state relay readiness"

(
  go2_runtime_close_parent_only_fds
  exec "$PYTHON" "$ROOT/deploy/time-sync/workstation_motion_state_relay.py" \
    --approval-file "$GO2_TIMESTAMP_APPROVAL_FILE" \
    --stamp-ready-file "$STAMP_READY" \
    --worker-binary "$MOTION_STATE_RELAY_BINARY" \
    --ready-file "$MOTION_STATE_READY" --fault-file "$MOTION_STATE_FAULT"
) >"$RUN_DIR/motion-state.log" 2>&1 &
MOTION_STATE_PID=$!
wait_private_ready \
  "motion-state relay" "$MOTION_STATE_PID" \
  "$MOTION_STATE_READY" "$MOTION_STATE_FAULT" 10

(
  go2_runtime_close_parent_only_fds
  exec "$PYTHON" \
    "$ROOT/deploy/time-sync/workstation_nomotion_cloud_relay.py" \
    --profile motion \
    --approval-file "$GO2_TIMESTAMP_APPROVAL_FILE" \
    --stamp-ready-file "$STAMP_READY" \
    --stamp-fault-file "$STAMP_FAULT" \
    --identity-fault-file "$IDENTITY_FAULT" \
    --stamp-pid "$STAMP_PID" --identity-pid "$IDENTITY_PID" \
    --ready-file "$CLOUD_READY" --fault-file "$CLOUD_FAULT"
) >"$RUN_DIR/cloud.log" 2>&1 &
CLOUD_PID=$!
wait_private_ready "cloud relay" "$CLOUD_PID" "$CLOUD_READY" "$CLOUD_FAULT" 20
require_state_chain_alive || die 7 "corrected sensor/state chain is not healthy"
validate_wireless_topology \
  || die 7 "private Wi-Fi topology changed before Robonix boot"

if [[ "$PERSISTENT_MODE" == true ]]; then
  (
    go2_runtime_close_parent_only_fds
    exec bash "$ROOT/packages/semantic_intent_router/scripts/start.sh"
  ) >"$RUN_DIR/semantic-intent-router.log" 2>&1 &
  SEMANTIC_PID=$!
  semantic_deadline=$((SECONDS + 15))
  while true; do
    kill -0 "$SEMANTIC_PID" 2>/dev/null \
      || die 7 "semantic intent router exited before readiness"
    if "$PYTHON" "$ROOT/scripts/check_semantic_intent_health.py" \
      "${VLM_BASE_URL}/models" >/dev/null 2>&1; then
      break
    fi
    (( SECONDS < semantic_deadline )) \
      || die 7 "semantic intent router readiness deadline expired"
    sleep 0.2
  done
fi

(
  go2_runtime_close_parent_only_fds
  # rbnx forwards its invocation directory to provider start hooks.  Keep the
  # parent launcher in the repository, but make this exact child use the
  # private run directory so relative config and the read-only Docker bind are
  # both tied to this session.
  cd -- "$RUN_DIR"
  exec "$RBNX" boot --no-update-check -f "$MANIFEST"
) >"$RUN_DIR/rbnx-boot.log" 2>&1 &
BOOT_PID=$!

deadline=$((SECONDS + 60))
while true; do
  require_state_chain_alive \
    || die 8 "corrected sensor/state chain failed during Robonix startup"
  kill -0 "$BOOT_PID" 2>/dev/null \
    || die 8 "rbnx boot exited before staged Nav2 readiness"
  if LC_ALL=C timeout --signal=INT --kill-after=1s 3s \
    ros2 node list --no-daemon 2>/dev/null \
    | grep -Fxq /go2_chassis_adapter \
    && LC_ALL=C timeout --signal=INT --kill-after=1s 3s \
    ros2 node list --no-daemon 2>/dev/null \
    | grep -Fxq /bt_navigator \
    && LC_ALL=C timeout --signal=INT --kill-after=1s 3s \
    ros2 node list --no-daemon 2>/dev/null \
    | grep -Fxq /velocity_smoother; then
    break
  fi
  (( SECONDS < deadline )) \
    || die 8 "staged Nav2 graph did not become ready within 60 seconds"
  sleep 0.2
done

NAV2_CONTAINER_ID=""
deadline=$((SECONDS + 10))
while true; do
  require_state_chain_alive \
    || die 8 "corrected state chain failed while binding the Nav2 container"
  kill -0 "$BOOT_PID" 2>/dev/null \
    || die 8 "rbnx boot exited before the Nav2 container could be bound"
  if NAV2_CONTAINER_ID="$(
    docker inspect --type container --format '{{.Id}}' \
      "$NAV2_CONTAINER_NAME" 2>/dev/null
  )"; then
    verify_current_nav2_container "$NAV2_CONTAINER_ID" \
      || die 8 "running staged Nav2 container failed exact validation"
    break
  fi
  (( SECONDS < deadline )) \
    || die 8 "staged Nav2 container did not appear within 10 seconds"
  sleep 0.1
done

if [[ "$PERSISTENT_MODE" == true ]]; then
  echo "================================================================"
  echo " PERSISTENT ROBONIX VOICE / NAV2 STACK READY — CHASSIS DISARMED"
  echo " map:       $GO2_MAP_ID"
  echo " dashboard: http://127.0.0.1:8092/"
  echo " velocity:  official Navigation guard -> $NAV2_EXPECTED_VELOCITY_TOPIC"
  echo " motion:    DISARMED; this launcher never calls the arm service"
  echo " stop:      Ctrl-C (owned children only)"
  echo " evidence:  $RUN_DIR"
  echo "================================================================"
  set +e
  EXITED_PID=""
  wait -n -p EXITED_PID \
    "$BOOT_PID" "$CLOUD_PID" "$MOTION_STATE_PID" "$STAMP_PID" \
    "$IDENTITY_PID" "$SEMANTIC_PID"
  persistent_status=$?
  set -e
  die "$persistent_status" \
    "persistent stack owner exited unexpectedly: pid=${EXITED_PID:-unknown}"
fi

if [[ "$STANDARD_MODE" == false ]]; then
  deadline=$((SECONDS + 15))
  while true; do
    if [[ -f "$CONSUMED_CHASSIS_PERMIT" && ! -L "$CONSUMED_CHASSIS_PERMIT" \
      && ! -e "$GO2_STAGED_NAV2_PERMIT_FILE" \
      && ! -L "$GO2_STAGED_NAV2_PERMIT_FILE" ]]; then
      break
    fi
    kill -0 "$BOOT_PID" 2>/dev/null \
      || die 8 "rbnx boot exited before chassis permit consumption"
    (( SECONDS < deadline )) \
      || die 8 "chassis did not atomically consume its exact permit"
    sleep 0.1
  done
  [[ -f "$GO2_STAGED_NAV2_GOAL_PERMIT_FILE" \
    && ! -L "$GO2_STAGED_NAV2_GOAL_PERMIT_FILE" ]] \
    || die 8 "goal permit was consumed before the dispatcher gate"
fi

# Robonix providers and the corrected sensor relays can briefly settle at
# different rates after one boot.  A single incomplete observation while the
# chassis is still disarmed is not a terminal stack failure: keep the official
# providers alive and reacquire a complete live receipt automatically.  A
# collector/configuration error still fails immediately, and every retry keeps
# checking the state-chain and exact Nav2 container owners.
PRE_GUARD_ATTEMPT=0
while true; do
  require_state_chain_alive \
    || die 8 "corrected sensor/state chain failed before guard readiness"
  kill -0 "$BOOT_PID" 2>/dev/null \
    || die 8 "rbnx boot stopped before guard readiness"
  CURRENT_NAV2_CONTAINER_ID="$(
    docker inspect --type container --format '{{.Id}}' \
      "$NAV2_CONTAINER_NAME" 2>/dev/null
  )" || die 8 "staged Nav2 container disappeared before guard readiness"
  [[ "$CURRENT_NAV2_CONTAINER_ID" == "$NAV2_CONTAINER_ID" ]] \
    || die 8 "staged Nav2 container identity changed before guard readiness"
  verify_current_nav2_container "$CURRENT_NAV2_CONTAINER_ID" \
    || die 8 "staged Nav2 container changed before guard readiness"

  PRE_GUARD_ATTEMPT=$((PRE_GUARD_ATTEMPT + 1))
  PRE_GUARD_CANDIDATE="$RUN_DIR/pre-guard-readiness-attempt-${PRE_GUARD_ATTEMPT}.json"
  if "$PYTHON" "$ROOT/scripts/staged_nav2_readiness.py" \
    --phase pre_guard \
    --session-id "$GO2_STAGED_NAV2_SESSION_ID" \
    --network-interface "$WIRELESS_INTERFACE" \
    --map-id "$GO2_STAGED_NAV2_MAP_ID" \
    --map-generation "$GO2_STAGED_NAV2_MAP_GENERATION" \
    --allowed-mode "$observed_mode" \
    --allowed-state-marker "$observed_marker" \
    --duration 5 --output "$PRE_GUARD_CANDIDATE" >/dev/null; then
    "$PYTHON" "$ROOT/scripts/validate_staged_nav2_readiness_receipt.py" \
      --receipt "$PRE_GUARD_CANDIDATE" --phase pre_guard \
      --session-id "$GO2_STAGED_NAV2_SESSION_ID" \
      --network-interface "$WIRELESS_INTERFACE" \
      --map-id "$GO2_STAGED_NAV2_MAP_ID" \
      --map-generation "$GO2_STAGED_NAV2_MAP_GENERATION" \
      --allowed-mode "$observed_mode" \
      --allowed-state-marker "$observed_marker" \
      || die 8 "passing pre-guard receipt failed independent validation"
    mv -- "$PRE_GUARD_CANDIDATE" "$PRE_GUARD_RECEIPT"
    break
  else
    readiness_status=$?
  fi
  (( readiness_status == 3 )) \
    || die 8 "pre-guard readiness collector failed (status=$readiness_status)"
  echo "waiting for live startup streams to settle (attempt $PRE_GUARD_ATTEMPT)"
  sleep 0.2
done

# This is a fresh, independent, plan-only receipt.  Its unique per-run path
# cannot overwrite the permit-bound goal evidence and its hash never replaces
# GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256.
goal_prepare=(
  "$PYTHON" "$ROOT/scripts/prepare_staged_nav2_goal_evidence.py"
  --map-id "$GO2_STAGED_NAV2_MAP_ID"
  --map-generation "$GO2_STAGED_NAV2_MAP_GENERATION"
  --target-id "$GO2_STAGED_NAV2_TARGET_ID"
)
if [[ "$STANDARD_MODE" == true ]]; then
  goal_prepare+=(
    --forward-distance "${GO2_NAV2_FORWARD_DISTANCE_M:-0.20}"
  )
else
  goal_prepare+=(
    --x "$GO2_STAGED_NAV2_EXPECTED_GOAL_X"
    --y "$GO2_STAGED_NAV2_EXPECTED_GOAL_Y"
    --yaw "$GO2_STAGED_NAV2_EXPECTED_GOAL_YAW"
  )
fi
"${goal_prepare[@]}" --output "$LIVE_GOAL_EVIDENCE" >/dev/null

if [[ "$STANDARD_MODE" == true ]]; then
  mapfile -t LIVE_TARGET < <(
    "$PYTHON" - "$LIVE_GOAL_EVIDENCE" <<'PY'
import json
import sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
pose = payload["goal"]["pose"]
for key in ("x", "y", "yaw"):
    print(format(float(pose[key]), ".17g"))
PY
  )
  [[ "${#LIVE_TARGET[@]}" -eq 3 ]] \
    || die 8 "live relative goal did not contain one finite pose"
  export GO2_STAGED_NAV2_EXPECTED_GOAL_X="${LIVE_TARGET[0]}"
  export GO2_STAGED_NAV2_EXPECTED_GOAL_Y="${LIVE_TARGET[1]}"
  export GO2_STAGED_NAV2_EXPECTED_GOAL_YAW="${LIVE_TARGET[2]}"
fi

goal_validation=(
  "$PYTHON" "$ROOT/scripts/validate_staged_nav2_live_goal_evidence.py"
  --evidence "$LIVE_GOAL_EVIDENCE"
  --map-id "$GO2_STAGED_NAV2_MAP_ID"
  --map-generation "$GO2_STAGED_NAV2_MAP_GENERATION"
  --target-id "$GO2_STAGED_NAV2_TARGET_ID"
  --goal-x "$GO2_STAGED_NAV2_EXPECTED_GOAL_X"
  --goal-y "$GO2_STAGED_NAV2_EXPECTED_GOAL_Y"
  --goal-yaw "$GO2_STAGED_NAV2_EXPECTED_GOAL_YAW"
)
if [[ "$STANDARD_MODE" == false ]]; then
  goal_validation+=(
    --permit-start-x "$GO2_STAGED_NAV2_EXPECTED_START_X"
    --permit-start-y "$GO2_STAGED_NAV2_EXPECTED_START_Y"
  )
fi
mapfile -t LIVE_GOAL < <(
  "${goal_validation[@]}"
) || die 8 "fresh live goal confirmation was rejected"
[[ "${#LIVE_GOAL[@]}" -eq 4 ]] \
  || die 8 "live goal validator returned an unexpected claim count"
LIVE_START_X="${LIVE_GOAL[0]}"
LIVE_START_Y="${LIVE_GOAL[1]}"
LIVE_START_YAW="${LIVE_GOAL[2]}"
LIVE_GOAL_RECEIPT_SHA256="${LIVE_GOAL[3]}"
if [[ "$STANDARD_MODE" == true ]]; then
  export GO2_STAGED_NAV2_EXPECTED_START_X="$LIVE_START_X"
  export GO2_STAGED_NAV2_EXPECTED_START_Y="$LIVE_START_Y"
  export GO2_STAGED_NAV2_EXPECTED_START_YAW="$LIVE_START_YAW"
  export GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256="$LIVE_GOAL_RECEIPT_SHA256"
fi
export GO2_STAGED_NAV2_LIVE_GOAL_RECEIPT_SHA256="$LIVE_GOAL_RECEIPT_SHA256"
export GO2_STAGED_NAV2_LIVE_START_YAW="$LIVE_START_YAW"

(
  go2_runtime_close_parent_only_fds
  exec "$PYTHON" "$ROOT/scripts/staged_nav2_motion_guard.py"
) >"$RUN_DIR/motion-guard.log" 2>&1 &
GUARD_PID=$!

POST_GUARD_ATTEMPT=0
while true; do
  require_state_chain_alive \
    || die 8 "corrected sensor/state chain failed during guard readiness"
  kill -0 "$BOOT_PID" 2>/dev/null \
    || die 8 "rbnx boot stopped during guard readiness"
  kill -0 "$GUARD_PID" 2>/dev/null \
    || die 8 "staged motion guard exited before readiness"

  POST_GUARD_ATTEMPT=$((POST_GUARD_ATTEMPT + 1))
  POST_GUARD_CANDIDATE="$RUN_DIR/post-guard-readiness-attempt-${POST_GUARD_ATTEMPT}.json"
  if "$PYTHON" "$ROOT/scripts/staged_nav2_readiness.py" \
    --phase post_guard \
    --session-id "$GO2_STAGED_NAV2_SESSION_ID" \
    --network-interface "$WIRELESS_INTERFACE" \
    --map-id "$GO2_STAGED_NAV2_MAP_ID" \
    --map-generation "$GO2_STAGED_NAV2_MAP_GENERATION" \
    --allowed-mode "$observed_mode" \
    --allowed-state-marker "$observed_marker" \
    --expected-start-x "$LIVE_START_X" --expected-start-y "$LIVE_START_Y" \
    --duration 5 --output "$POST_GUARD_CANDIDATE" >/dev/null; then
    "$PYTHON" "$ROOT/scripts/validate_staged_nav2_readiness_receipt.py" \
      --receipt "$POST_GUARD_CANDIDATE" --phase post_guard \
      --session-id "$GO2_STAGED_NAV2_SESSION_ID" \
      --network-interface "$WIRELESS_INTERFACE" \
      --map-id "$GO2_STAGED_NAV2_MAP_ID" \
      --map-generation "$GO2_STAGED_NAV2_MAP_GENERATION" \
      --allowed-mode "$observed_mode" \
      --allowed-state-marker "$observed_marker" \
      --expected-start-x "$LIVE_START_X" --expected-start-y "$LIVE_START_Y" \
      --permit-start-x "$GO2_STAGED_NAV2_EXPECTED_START_X" \
      --permit-start-y "$GO2_STAGED_NAV2_EXPECTED_START_Y" \
      || die 8 "passing post-guard receipt failed independent validation"
    mv -- "$POST_GUARD_CANDIDATE" "$POST_GUARD_RECEIPT"
    break
  else
    readiness_status=$?
  fi
  (( readiness_status == 3 )) \
    || die 8 "post-guard readiness collector failed (status=$readiness_status)"
  echo "waiting for disarmed navigation streams to settle (attempt $POST_GUARD_ATTEMPT)"
  sleep 0.2
done
kill -0 "$GUARD_PID" 2>/dev/null \
  || die 8 "staged motion guard exited before dispatcher authorization"

# Revalidate both exact permits immediately before dispatch.  The chassis
# permit is now the same private consumed file and the goal permit remains
# unconsumed.  Live state, localization, route and controller checks below are
# authoritative; the permit's historical expiry metadata is not a runtime
# gate.
if [[ "$STANDARD_MODE" == false ]]; then
  mapfile -t FINAL_BUNDLE < <(
    "$PYTHON" "$ROOT/scripts/validate_staged_nav2_permit_bundle.py" \
      --chassis-permit "$CONSUMED_CHASSIS_PERMIT" \
      --goal-permit "$GO2_STAGED_NAV2_GOAL_PERMIT_FILE" \
      --package-root "$ROOT" \
      --network-interface "$WIRELESS_INTERFACE" \
      --allowed-mode "$observed_mode" \
      --allowed-state-marker "$observed_marker" \
      --ipc-socket "$GO2_SDK_SOCKET" \
      --dds-evidence "$GO2_STAGED_NAV2_DDS_IDENTITY_EVIDENCE" \
      --state-evidence "$GO2_STAGED_NAV2_STATE_EVIDENCE" \
      --time-evidence "$GO2_TIMESTAMP_APPROVAL_FILE" \
      --goal-evidence "$GO2_STAGED_NAV2_GOAL_EVIDENCE"
  ) || die 8 "staged permits changed before dispatch"
  [[ "${#FINAL_BUNDLE[@]}" -eq 15 ]] \
    || die 8 "final staged permit validation returned an unexpected claim count"
  for index in "${!BUNDLE[@]}"; do
    [[ "${FINAL_BUNDLE[$index]}" == "${BUNDLE[$index]}" ]] \
      || die 8 "staged permit claim changed before dispatch"
  done
fi

require_state_chain_alive \
  || die 8 "corrected sensor/state chain failed before dispatch"
kill -0 "$BOOT_PID" 2>/dev/null \
  || die 8 "rbnx boot stopped before dispatch"
kill -0 "$GUARD_PID" 2>/dev/null \
  || die 8 "motion guard stopped before dispatch"
CURRENT_NAV2_CONTAINER_ID="$(
  docker inspect --type container --format '{{.Id}}' \
    "$NAV2_CONTAINER_NAME" 2>/dev/null
)" || die 8 "staged Nav2 container disappeared before dispatch"
[[ "$CURRENT_NAV2_CONTAINER_ID" == "$NAV2_CONTAINER_ID" ]] \
  || die 8 "staged Nav2 container identity changed before dispatch"
verify_current_nav2_container "$CURRENT_NAV2_CONTAINER_ID" \
  || die 8 "staged Nav2 container changed before dispatch"
validate_wireless_topology \
  || die 8 "private Wi-Fi topology changed before dispatcher"

echo "================================================================"
echo " STAGED NAV2 GOAL AUTHORIZED"
echo " target: $GO2_STAGED_NAV2_TARGET_ID"
echo " official Go2 Nav2 envelope: 0.30 m/s linear; 0.40 rad/s angular"
echo " keep the official remote stop in hand; evidence: $RUN_DIR"
echo "================================================================"

(
  go2_runtime_close_parent_only_fds
  exec "$PYTHON" "$ROOT/scripts/staged_nav2_goal_dispatch.py"
) >"$RUN_DIR/goal-dispatch.log" 2>&1 &
DISPATCH_PID=$!

set +e
EXITED_PID=""
wait -n -p EXITED_PID \
  "$DISPATCH_PID" "$GUARD_PID" "$BOOT_PID" "$CLOUD_PID" \
  "$MOTION_STATE_PID" "$STAMP_PID" "$IDENTITY_PID"
status=$?
set -e
if [[ "$EXITED_PID" != "$DISPATCH_PID" && "$EXITED_PID" != "$GUARD_PID" ]]; then
  echo "a safety/state/Robonix owner exited during staged navigation" >&2
  exit 70
fi
if (( status != 0 )); then
  echo "staged goal or motion guard failed (status=$status)" >&2
  exit "$status"
fi

# The paired component should observe the same terminal success promptly.
paired_pid="$GUARD_PID"
[[ "$EXITED_PID" == "$GUARD_PID" ]] && paired_pid="$DISPATCH_PID"
deadline=$((SECONDS + 5))
while kill -0 "$paired_pid" 2>/dev/null; do
  (( SECONDS < deadline )) \
    || die 70 "staged goal and guard did not converge on terminal success"
  sleep 0.1
done
set +e
wait "$paired_pid"
paired_status=$?
set -e
(( paired_status == 0 )) \
  || die 70 "paired staged goal component failed (status=$paired_status)"
# Both paired processes have been reaped and the guard already completed its
# cancel/zero/disarm terminal sequence.  Do not let EXIT cleanup wait twice.
DISPATCH_PID=""
GUARD_PID=""

if [[ "$STANDARD_MODE" == false ]]; then
  [[ -f "$CONSUMED_GOAL_PERMIT" && ! -L "$CONSUMED_GOAL_PERMIT" \
    && ! -e "$GO2_STAGED_NAV2_GOAL_PERMIT_FILE" \
    && ! -L "$GO2_STAGED_NAV2_GOAL_PERMIT_FILE" ]] \
    || die 70 "goal dispatcher did not atomically consume its exact permit"
fi

"$PYTHON" "$ROOT/scripts/validate_staged_nav2_result.py" \
  || die 70 "staged goal lacks a valid measured terminal result"

echo "staged Nav2 goal completed with measured stop evidence; cleanup will stop the stack"
exit 0
