#!/usr/bin/env bash
set -euo pipefail
umask 077

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(CDPATH= cd -- "$ROOT/../.." && pwd)"
PYTHON="$WORKSPACE_ROOT/.tools/rbnx-python/bin/python3"
UNITREE_SETUP="${UNITREE_ROS2_SETUP:-$ROOT/rbnx-build/unitree_ros2/install/setup.bash}"
APPROVAL_FILE="${GO2_TIMESTAMP_APPROVAL_FILE:-}"
INHERITED_NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-}"
PROFILE=workstation-full-nomotion-corrected-v1
SESSION_HELPER="$ROOT/scripts/workstation_nomotion_session.sh"
NAV2_STOPPER="$ROOT/third_party/service-navigation-rbnx/scripts/stop.sh"
readonly STACK_NICE_LEVEL=5
readonly NAV2_LEGACY_CONTAINER_NAME=robonix_nav2
readonly NAV2_IMAGE_NAME=robonix-nav2

if [[ "$#" -ne 0 ]]; then
  echo "this launcher accepts no arguments" >&2
  exit 2
fi
if [[ -f "$ROOT/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/.env"
  set +a
fi
# This no-motion wrapper inspects and later removes an exact local container.
# Pin every Docker operation to the same local daemon instead of accepting an
# inherited remote/TLS endpoint from the shell or .env.
unset DOCKER_HOST DOCKER_TLS_VERIFY DOCKER_CERT_PATH
export DOCKER_CONTEXT=default
D435I_PREVIEW_ENABLED="${GO2_D435I_PREVIEW_ENABLED:-false}"
case "$D435I_PREVIEW_ENABLED" in
  true|false) ;;
  *)
    echo "GO2_D435I_PREVIEW_ENABLED must be exactly true or false" >&2
    exit 2
    ;;
esac
readonly D435I_PREVIEW_ENABLED
export GO2_D435I_PREVIEW_ENABLED="$D435I_PREVIEW_ENABLED"
# A one-shot caller override is needed when the same audited workstation uses
# Ethernet for commissioning and the isolated Robonix-Go2 Wi-Fi for passive
# mapping.  It is restored only in this no-motion wrapper and is still subject
# to every physical-interface, address, SSID, route, DNS and IPv6 check below.
if [[ -n "$INHERITED_NETWORK_INTERFACE" ]]; then
  export GO2_NETWORK_INTERFACE="$INHERITED_NETWORK_INTERFACE"
fi
# Profile-local evidence policy only: marker 1002 was observed on 2026-07-22
# while the operator used the paired official remote during the approved,
# motion-disabled manual-mapping session; 100 is the stationary marker from
# the immediately preceding subscriber-only evidence. This must never be
# inherited by a motion-capable launcher or promoted to a generic default.
readonly PASSIVE_MANUAL_MAPPING_STATE_MARKERS="100,1002"
unset NOMOTION_PRIVATE_WIFI_SSID
readonly NOMOTION_PRIVATE_WIFI_SSID=Robonix-Go2
NOMOTION_STATE_EVIDENCE="${GO2_NOMOTION_STATE_EVIDENCE:-}"
# Never inherit an activation request from .env; this wrapper is the only
# entrypoint and still requires a separate private approval file.
export GO2_FORCE_NOMOTION_PROFILE="$PROFILE"
export GO2_RUNTIME_PLACEMENT=workstation-local
export GO2_ALLOW_MOTION=false
export GO2_OPERATOR_PRESENT=false
export GO2_SAFETY_ACK=""
export GO2_ALLOWED_MODES=""
export GO2_ALLOWED_STATE_MARKERS=""
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

[[ -x "$PYTHON" ]] || {
  echo "missing workspace-local Robonix Python: $PYTHON" >&2
  exit 3
}
[[ -n "$APPROVAL_FILE" && "$APPROVAL_FILE" == /* ]] || {
  echo "GO2_TIMESTAMP_APPROVAL_FILE must name an absolute private approval file" >&2
  exit 3
}
"$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_approval.py" \
  --require-affine "$APPROVAL_FILE"
[[ -n "$NOMOTION_STATE_EVIDENCE" && "$NOMOTION_STATE_EVIDENCE" == /* ]] || {
  echo "GO2_NOMOTION_STATE_EVIDENCE must name an absolute evidence file" >&2
  exit 3
}
validated_state_marker="$("$PYTHON" \
  "$ROOT/scripts/validate_nomotion_state_marker_evidence.py" \
  "$NOMOTION_STATE_EVIDENCE")" || {
    status=$?
    echo "fresh subscriber-only SportModeState evidence is required" >&2
    exit "$status"
  }
[[ "$validated_state_marker" =~ ^[0-9]+$ ]] || {
  echo "validated SportModeState marker has an invalid representation" >&2
  exit 3
}
# The fresh marker remains the startup witness. Refuse to apply the passive
# mapping exception if the current robot state is outside the exact reviewed
# set, even though both historical values were observed on this robot.
case ",$PASSIVE_MANUAL_MAPPING_STATE_MARKERS," in
  *",$validated_state_marker,"*) ;;
  *)
    echo "fresh SportModeState marker is not in the passive mapping allowlist" >&2
    exit 3
    ;;
esac
export GO2_ALLOWED_STATE_MARKERS="$PASSIVE_MANUAL_MAPPING_STATE_MARKERS"
"$PYTHON" "$ROOT/scripts/check_workstation_nomotion_tcp_ports.py" \
  --manifest "$ROOT/robonix_manifest.yaml" \
  --dashboard-port "${GO2_DASHBOARD_PORT:-8092}" \
  --semantic-port "${SEMANTIC_INTENT_PORT:-18080}" || {
    status=$?
    echo "workstation no-motion startup stopped before ROS or Docker activation" >&2
    exit "$status"
  }
[[ -n "${GO2_NETWORK_INTERFACE:-}" ]] || {
  echo "GO2_NETWORK_INTERFACE is required" >&2
  exit 4
}
[[ "$GO2_NETWORK_INTERFACE" =~ ^[A-Za-z0-9_.:-]+$ ]] || {
  echo "GO2_NETWORK_INTERFACE contains unsupported characters" >&2
  exit 4
}
for command in basename chmod cmp dirname docker env flock git id install ip mktemp mv nmcli \
  readlink renice rm stat tr; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing required command: $command" >&2
    exit 4
  }
done
[[ "$(readlink -f -- "$(command -v docker)")" == /usr/bin/docker ]] || {
  echo "the corrected no-motion profile requires the audited /usr/bin/docker CLI" >&2
  exit 4
}
ip link show dev "$GO2_NETWORK_INTERFACE" >/dev/null 2>&1 || {
  echo "network interface does not exist: $GO2_NETWORK_INTERFACE" >&2
  exit 4
}
interface_sysfs="$(readlink -f -- "/sys/class/net/$GO2_NETWORK_INTERFACE")"
[[ -n "$interface_sysfs" && "$interface_sysfs" != *"/virtual/"* ]] || {
  echo "Go2 interface must be a physical device" >&2
  exit 4
}
if [[ -d "/sys/class/net/$GO2_NETWORK_INTERFACE/wireless" ]]; then
  # This wrapper owns the exact one-way no-motion marker.  Accept Wi-Fi only
  # while all three forced invariants still hold; there is no generic override
  # that a motion-capable launcher can inherit.
  [[ "$GO2_FORCE_NOMOTION_PROFILE" == "$PROFILE" \
    && "$GO2_RUNTIME_PLACEMENT" == workstation-local \
    && "$GO2_ALLOW_MOTION" == false ]] || {
      echo "Wi-Fi is allowed only for the corrected full no-motion profile" >&2
      exit 4
    }
  if ! active_connection_uuid="$(
    nmcli --get-values GENERAL.CON-UUID \
      device show "$GO2_NETWORK_INTERFACE" 2>/dev/null
  )"; then
    echo "could not resolve the active NetworkManager connection for the Go2 Wi-Fi interface" >&2
    exit 4
  fi
  [[ "$active_connection_uuid" =~ \
    ^[[:xdigit:]]{8}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{12}$ ]] || {
      echo "Go2 Wi-Fi interface has no valid active NetworkManager connection UUID" >&2
      exit 4
    }
  if ! active_wifi_ssid="$(
    nmcli --get-values 802-11-wireless.ssid \
      connection show uuid "$active_connection_uuid" 2>/dev/null
  )"; then
    echo "could not read the active Go2 Wi-Fi connection profile SSID" >&2
    exit 4
  fi
  [[ "$active_wifi_ssid" == "$NOMOTION_PRIVATE_WIFI_SSID" ]] || {
    echo "active Go2 Wi-Fi profile SSID must exactly equal $NOMOTION_PRIVATE_WIFI_SSID" >&2
    exit 4
  }
fi
[[ "$(cat -- "/sys/class/net/$GO2_NETWORK_INTERFACE/operstate")" == up ]] || {
  echo "Go2 interface is not UP" >&2
  exit 4
}
mapfile -t ipv4 < <(ip -o -4 addr show dev "$GO2_NETWORK_INTERFACE" | awk '{print $4}')
[[ "${#ipv4[@]}" -eq 1 && "${ipv4[0]}" == 192.168.123.99/24 ]] || {
  echo "Go2 interface must have exactly 192.168.123.99/24" >&2
  exit 4
}
mapfile -t ipv6 < <(ip -o -6 addr show dev "$GO2_NETWORK_INTERFACE" | awk '{print $4}')
[[ "${#ipv6[@]}" -eq 0 ]] || {
  echo "Go2 interface must have no IPv6 address" >&2
  exit 4
}
if ip -4 route show default dev "$GO2_NETWORK_INTERFACE" | grep -q . \
  || ip -6 route show default dev "$GO2_NETWORK_INTERFACE" | grep -q .; then
  echo "Go2 interface must not have a default route" >&2
  exit 4
fi
nm_state="$(
  nmcli --get-values IP4.GATEWAY,IP4.DNS,IP6.GATEWAY,IP6.DNS \
    device show "$GO2_NETWORK_INTERFACE" 2>/dev/null | sed '/^[[:space:]]*$/d'
)" || {
  echo "could not verify NetworkManager gateway/DNS state" >&2
  exit 4
}
[[ -z "$nm_state" ]] || {
  echo "Go2 interface must have no gateway or DNS" >&2
  exit 4
}
export GO2_NOMOTION_NETWORK_INTERFACE="$GO2_NETWORK_INTERFACE"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_NETWORK_INTERFACE\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"

[[ -f /opt/ros/humble/setup.bash && -f "$UNITREE_SETUP" ]] || {
  echo "ROS Humble or the built Unitree message overlay is missing" >&2
  exit 5
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$UNITREE_SETUP"
set -u

mkdir -p "$ROOT/rbnx-build/run"
chmod 700 "$ROOT/rbnx-build/run"
exec {DISCIPLINE_LOCK_FD}>"$ROOT/rbnx-build/run/workstation-nomotion-stamp.lock"
chmod 600 "$ROOT/rbnx-build/run/workstation-nomotion-stamp.lock"
flock --exclusive --nonblock "$DISCIPLINE_LOCK_FD" || {
  echo "another workstation timestamp discipline already owns the lock" >&2
  exit 6
}
RUN_DIR="$(mktemp -d "$ROOT/rbnx-build/run/workstation-nomotion-stamp.XXXXXX")"
chmod 700 "$RUN_DIR"
CURRENT_SESSION="$ROOT/rbnx-build/run/workstation-nomotion-current.session"
SESSION_INDEX_LOCK="$ROOT/rbnx-build/run/workstation-nomotion-session-index.lock"
SESSION_METADATA="$RUN_DIR/session.meta"
# shellcheck disable=SC1090
source "$SESSION_HELPER"
WRAPPER_START_TICKS="$(go2_nomotion_process_start_ticks "$$")" || {
  echo "could not read corrected wrapper process identity" >&2
  exit 6
}
SESSION_TOKEN="$(tr 'A-F' 'a-f' < /proc/sys/kernel/random/uuid)"
NAV2_SESSION_CONTAINER_NAME="robonix_nav2_nomotion_${SESSION_TOKEN//-/}"
readonly NAV2_SESSION_CONTAINER_NAME
export ROBONIX_NAV2_CONTAINER="$NAV2_SESSION_CONTAINER_NAME"
exec {SESSION_INDEX_FD}>"$SESSION_INDEX_LOCK"
chmod 600 "$SESSION_INDEX_LOCK"
flock --exclusive "$SESSION_INDEX_FD"
if [[ -e "$CURRENT_SESSION" || -L "$CURRENT_SESSION" ]]; then
  echo "a current-session pointer already exists; stop or recover it explicitly" >&2
  exit 6
fi
go2_nomotion_atomic_write_metadata \
  "$SESSION_METADATA" "$SESSION_TOKEN" "$$" "$WRAPPER_START_TICKS" "$RUN_DIR"
go2_nomotion_atomic_write_metadata \
  "$CURRENT_SESSION" "$SESSION_TOKEN" "$$" "$WRAPPER_START_TICKS" "$RUN_DIR"
cleanup_registered_pointer_early() {
  if go2_nomotion_private_regular_file "$CURRENT_SESSION" "$(id -u)" \
    && go2_nomotion_private_regular_file "$SESSION_METADATA" "$(id -u)" \
    && cmp -s -- "$CURRENT_SESSION" "$SESSION_METADATA"; then
    rm -f -- "$CURRENT_SESSION"
  fi
}
on_early_exit() {
  local status=$?
  trap - EXIT INT TERM HUP
  cleanup_registered_pointer_early
  exit "$status"
}
on_early_signal() {
  local status="$1"
  trap - EXIT INT TERM HUP
  cleanup_registered_pointer_early
  exit "$status"
}
trap on_early_exit EXIT
trap 'on_early_signal 129' HUP
trap 'on_early_signal 130' INT
trap 'on_early_signal 143' TERM
flock --unlock "$SESSION_INDEX_FD"
exec {SESSION_INDEX_FD}>&-
STAMP_PID=""
IDENTITY_PID=""
CLOUD_RELAY_PID=""
STACK_PID=""
verify_nomotion_nav2_stop_hook() {
  local nav2_package="$ROOT/third_party/service-navigation-rbnx"
  local mode="" expected_pin="" stage="" indexed_path="" actual_pin=""
  local canonical_stopper=""

  read -r mode expected_pin stage indexed_path < <(
    git -C "$ROOT" ls-files --stage -- third_party/service-navigation-rbnx
  )
  [[ "$mode" == 160000 && "$stage" == 0 \
    && "$indexed_path" == third_party/service-navigation-rbnx \
    && "$expected_pin" =~ ^[0-9a-f]{40}$ ]] || {
      echo "refusing Nav2 cleanup: parent repository has no unique pinned Nav2 gitlink" >&2
      return 1
    }
  actual_pin="$(git -C "$nav2_package" rev-parse HEAD 2>/dev/null)" || {
    echo "refusing Nav2 cleanup: pinned Nav2 package HEAD is unreadable" >&2
    return 1
  }
  [[ "$actual_pin" == "$expected_pin" ]] || {
    echo "refusing Nav2 cleanup: checked-out Nav2 package differs from its gitlink" >&2
    return 1
  }
  git -C "$nav2_package" diff --quiet -- scripts/start.sh scripts/stop.sh || {
    echo "refusing Nav2 cleanup: Nav2 lifecycle hooks have unstaged changes" >&2
    return 1
  }
  git -C "$nav2_package" diff --cached --quiet -- scripts/start.sh scripts/stop.sh || {
    echo "refusing Nav2 cleanup: Nav2 lifecycle hooks have staged changes" >&2
    return 1
  }
  canonical_stopper="$(readlink -f -- "$NAV2_STOPPER")" || {
    echo "refusing Nav2 cleanup: pinned stop hook cannot be resolved" >&2
    return 1
  }
  [[ "$canonical_stopper" == \
    "$nav2_package/scripts/stop.sh" && -f "$NAV2_STOPPER" \
    && ! -L "$NAV2_STOPPER" ]] || {
      echo "refusing Nav2 cleanup: pinned stop hook path is not canonical" >&2
      return 1
    }
}
cleanup_nomotion_nav2_container() {
  local container_name="${1:-}"
  local expected_run_dir="${2:-}"
  local container_id="" visible_ids="" container_identity=""
  local container_shape="" container_running="" remaining_ids=""
  local container_image_id="" audited_image_id=""
  local invocation_cwd="" package_host="" velocity_output="" capability_id=""
  local invocation_count=0 package_count=0 velocity_count=0 capability_count=0
  local run_parent="" run_basename="" canonical_run_dir="" run_stat=""
  local session_file="" observed_pid_uid="" observed_start_ticks=""
  local entry="" mount_type="" mount_source="" mount_destination="" mount_rw=""
  local nav_mount_count=0 run_mount_count=0 replacement_ids=""
  local -a container_environment=()
  local -a container_mounts=()

  [[ "$container_name" == "$NAV2_LEGACY_CONTAINER_NAME" \
    || "$container_name" =~ ^robonix_nav2_nomotion_[0-9a-f]{32}$ ]] || {
      echo "refusing Nav2 cleanup: container name is outside the no-motion namespace" >&2
      return 1
    }
  if ! docker version --format '{{.Server.Version}}' >/dev/null 2>&1; then
    echo "Docker daemon is unavailable for exact no-motion Nav2 cleanup" >&2
    return 1
  fi
  if ! container_id="$(
    docker inspect --type container --format '{{.Id}}' \
      "$container_name" 2>/dev/null
  )"; then
    if ! visible_ids="$(
      docker ps --all --quiet --no-trunc \
        --filter "name=^/${container_name}$"
    )"; then
      echo "could not verify absence of the no-motion Nav2 container" >&2
      return 1
    fi
    [[ -z "$visible_ids" ]] || {
      echo "no-motion Nav2 container became ambiguous during inspection" >&2
      return 1
    }
    return 0
  fi
  [[ "$container_id" =~ ^[[:xdigit:]]{64}$ ]] || {
    echo "refusing Nav2 cleanup: Docker returned an invalid container ID" >&2
    return 1
  }
  if ! container_identity="$(
    docker inspect --type container \
      --format '{{.Name}}|{{.Config.Image}}' "$container_id"
  )"; then
    echo "refusing Nav2 cleanup: exact container identity disappeared" >&2
    return 1
  fi
  [[ "$container_identity" == \
    "/${container_name}|${NAV2_IMAGE_NAME}" ]] || {
      echo "refusing Nav2 cleanup: container name or image is not the audited no-motion instance" >&2
      return 1
    }
  container_image_id="$(
    docker inspect --type container --format '{{.Image}}' "$container_id"
  )" || {
    echo "refusing Nav2 cleanup: immutable container image ID is unreadable" >&2
    return 1
  }
  audited_image_id="$(
    docker image inspect --format '{{.Id}}' "$NAV2_IMAGE_NAME"
  )" || {
    echo "refusing Nav2 cleanup: audited Nav2 image tag is unavailable" >&2
    return 1
  }
  [[ "$container_image_id" =~ ^sha256:[[:xdigit:]]{64}$ \
    && "$container_image_id" == "$audited_image_id" ]] || {
      echo "refusing Nav2 cleanup: container image ID differs from the audited tag" >&2
      return 1
    }
  if ! container_shape="$(
    docker inspect --type container \
      --format '{{.HostConfig.AutoRemove}}|{{.HostConfig.NetworkMode}}|{{.HostConfig.IpcMode}}|{{.HostConfig.RestartPolicy.Name}}|{{.HostConfig.Privileged}}|{{.HostConfig.PidMode}}' \
      "$container_id"
  )"; then
    echo "refusing Nav2 cleanup: container runtime shape is unreadable" >&2
    return 1
  fi
  [[ "$container_shape" == "true|host|host|no|false|" ]] || {
    echo "refusing Nav2 cleanup: container runtime shape is not the audited no-motion instance" >&2
    return 1
  }
  if ! container_running="$(
    docker inspect --type container --format '{{.State.Running}}' "$container_id"
  )"; then
    echo "refusing Nav2 cleanup: container state is unreadable" >&2
    return 1
  fi
  [[ "$container_running" == true ]] || {
    echo "refusing Nav2 cleanup: audited no-motion container is not running" >&2
    return 1
  }

  mapfile -t container_environment < <(
    docker inspect --type container \
      --format '{{range .Config.Env}}{{println .}}{{end}}' "$container_id"
  )
  for entry in "${container_environment[@]}"; do
    case "$entry" in
      RBNX_INVOCATION_CWD=*)
        invocation_cwd="${entry#RBNX_INVOCATION_CWD=}"
        invocation_count=$((invocation_count + 1))
        ;;
      ROBONIX_PKG_HOST_DIR=*)
        package_host="${entry#ROBONIX_PKG_HOST_DIR=}"
        package_count=$((package_count + 1))
        ;;
      ROBONIX_VELOCITY_OUTPUT_TOPIC=*)
        velocity_output="${entry#ROBONIX_VELOCITY_OUTPUT_TOPIC=}"
        velocity_count=$((velocity_count + 1))
        ;;
      ROBONIX_CAPABILITY_ID=*)
        capability_id="${entry#ROBONIX_CAPABILITY_ID=}"
        capability_count=$((capability_count + 1))
        ;;
    esac
  done
  [[ "$invocation_count" -eq 1 && "$package_count" -eq 1 \
    && "$velocity_count" -eq 1 && "$capability_count" -eq 1 ]] || {
      echo "refusing Nav2 cleanup: required no-motion environment is missing or duplicated" >&2
      return 1
    }
  [[ "$package_host" == "$ROOT/third_party/service-navigation-rbnx" ]] || {
    echo "refusing Nav2 cleanup: package host directory is not the pinned Nav2 package" >&2
    return 1
  }
  [[ "$velocity_output" == /robonix/nomotion/cmd_vel ]] || {
    echo "refusing Nav2 cleanup: velocity output is not the no-motion sink" >&2
    return 1
  }
  [[ "$capability_id" == nav2 ]] || {
    echo "refusing Nav2 cleanup: capability identity is not nav2" >&2
    return 1
  }

  run_parent="$(dirname -- "$invocation_cwd")"
  run_basename="$(basename -- "$invocation_cwd")"
  [[ "$run_parent" == "$ROOT/rbnx-build/run" \
    && "$run_basename" =~ ^workstation-nomotion-stamp\.[[:alnum:]]{6}$ ]] || {
      echo "refusing Nav2 cleanup: invocation directory is outside the private no-motion run root" >&2
      return 1
    }
  [[ -d "$invocation_cwd" && ! -L "$invocation_cwd" ]] || {
    echo "refusing Nav2 cleanup: invocation directory is absent or is a symlink" >&2
    return 1
  }
  canonical_run_dir="$(readlink -f -- "$invocation_cwd")"
  [[ "$canonical_run_dir" == "$invocation_cwd" ]] || {
    echo "refusing Nav2 cleanup: invocation directory is not canonical" >&2
    return 1
  }
  go2_nomotion_private_directory "$invocation_cwd" "$(id -u)" || {
    echo "refusing Nav2 cleanup: invocation directory is not private and user-owned" >&2
    return 1
  }
  [[ -z "$expected_run_dir" || "$invocation_cwd" == "$expected_run_dir" ]] || {
    echo "refusing Nav2 cleanup: container does not belong to the current no-motion run" >&2
    return 1
  }
  session_file="$invocation_cwd/session.meta"
  go2_nomotion_private_regular_file "$session_file" "$(id -u)" || {
    echo "refusing Nav2 cleanup: private session metadata is missing" >&2
    return 1
  }
  go2_nomotion_read_session "$session_file" || {
    echo "refusing Nav2 cleanup: private session metadata is invalid" >&2
    return 1
  }
  [[ "$GO2_SESSION_RUN_DIR" == "$invocation_cwd" ]] || {
    echo "refusing Nav2 cleanup: session metadata names another run directory" >&2
    return 1
  }
  if [[ -n "$expected_run_dir" ]]; then
    [[ "$GO2_SESSION_PID" == "$$" \
      && "$GO2_SESSION_START_TICKS" == "$WRAPPER_START_TICKS" \
      && "$GO2_SESSION_TOKEN" == "$SESSION_TOKEN" ]] || {
        echo "refusing Nav2 cleanup: current session metadata identity does not match" >&2
        return 1
      }
  elif [[ -d "/proc/$GO2_SESSION_PID" ]]; then
    observed_pid_uid="$(stat -Lc '%u' -- "/proc/$GO2_SESSION_PID" 2>/dev/null)" || {
      echo "refusing Nav2 cleanup: stale wrapper process identity is unreadable" >&2
      return 1
    }
    observed_start_ticks="$(
      go2_nomotion_process_start_ticks "$GO2_SESSION_PID"
    )" || {
      echo "refusing Nav2 cleanup: stale wrapper start time is unreadable" >&2
      return 1
    }
    if [[ "$observed_pid_uid" == "$(id -u)" \
      && "$observed_start_ticks" == "$GO2_SESSION_START_TICKS" ]]; then
      echo "refusing Nav2 cleanup: the recorded no-motion wrapper is still alive" >&2
      return 1
    fi
  fi

  mapfile -t container_mounts < <(
    docker inspect --type container \
      --format '{{range .Mounts}}{{.Type}}|{{.Source}}|{{.Destination}}|{{.RW}}{{println}}{{end}}' \
      "$container_id"
  )
  for entry in "${container_mounts[@]}"; do
    IFS='|' read -r mount_type mount_source mount_destination mount_rw <<< "$entry"
    if [[ "$mount_type" == bind \
      && "$mount_source" == "$ROOT/third_party/service-navigation-rbnx" \
      && "$mount_destination" == /nav2 && "$mount_rw" == true ]]; then
      nav_mount_count=$((nav_mount_count + 1))
    fi
    if [[ "$mount_type" == bind && "$mount_source" == "$invocation_cwd" \
      && "$mount_destination" == "$invocation_cwd" && "$mount_rw" == false ]]; then
      run_mount_count=$((run_mount_count + 1))
    fi
  done
  [[ "$nav_mount_count" -eq 1 && "$run_mount_count" -eq 1 ]] || {
    echo "refusing Nav2 cleanup: audited package/run-directory mounts do not match" >&2
    return 1
  }
  verify_nomotion_nav2_stop_hook || return 1

  # Pass the already verified immutable ID, not the reusable container name,
  # so a concurrent replacement cannot be removed by the pinned stop hook.
  if ! env -i PATH=/usr/bin:/bin DOCKER_CONTEXT=default \
    ROBONIX_NAV2_FORCE=docker \
    ROBONIX_NAV2_CONTAINER="$container_id" \
    RBNX_PACKAGE_ROOT="$ROOT/third_party/service-navigation-rbnx" \
    bash "$NAV2_STOPPER"; then
    echo "exact no-motion Nav2 stop hook failed" >&2
    return 1
  fi
  if ! remaining_ids="$(
    docker ps --all --quiet --no-trunc --filter "id=$container_id"
  )"; then
    echo "could not verify exact no-motion Nav2 removal" >&2
    return 1
  fi
  [[ -z "$remaining_ids" ]] || {
    echo "exact no-motion Nav2 container survived its stop hook" >&2
    return 1
  }
  if ! replacement_ids="$(
    docker ps --all --quiet --no-trunc \
      --filter "name=^/${container_name}$"
  )"; then
    echo "could not verify no-motion Nav2 name release" >&2
    return 1
  fi
  [[ -z "$replacement_ids" ]] || {
    echo "Nav2 container name was reused during exact cleanup: $replacement_ids" >&2
    return 1
  }
}
cleanup_stale_nomotion_nav2_containers() {
  local names_text="" candidate=""
  local -a candidate_names=()

  cleanup_nomotion_nav2_container "$NAV2_LEGACY_CONTAINER_NAME"
  if ! names_text="$(
    docker ps --all --format '{{.Names}}' \
      --filter 'name=^/robonix_nav2_nomotion_'
  )"; then
    echo "could not enumerate stale no-motion Nav2 containers" >&2
    return 1
  fi
  mapfile -t candidate_names <<< "$names_text"
  for candidate in "${candidate_names[@]}"; do
    [[ -z "$candidate" ]] && continue
    [[ "$candidate" =~ ^robonix_nav2_nomotion_[0-9a-f]{32}$ ]] || {
      echo "refusing startup: ambiguous container occupies the no-motion Nav2 namespace" >&2
      return 1
    }
    cleanup_nomotion_nav2_container "$candidate"
  done
}
cleanup_owned_session_pointer() {
  go2_nomotion_remove_pointer_if_owned \
    "$CURRENT_SESSION" "$SESSION_METADATA" "$SESSION_INDEX_LOCK" "$(id -u)" || true
}
stop_owned_children() {
  local nav_cleanup_status=0
  if [[ -n "$STACK_PID" ]] && kill -0 "$STACK_PID" 2>/dev/null; then
    kill -TERM "$STACK_PID" 2>/dev/null || true
    wait "$STACK_PID" 2>/dev/null || true
  fi
  cleanup_nomotion_nav2_container \
    "$NAV2_SESSION_CONTAINER_NAME" "$RUN_DIR" || nav_cleanup_status=$?
  if [[ -n "$CLOUD_RELAY_PID" ]] && kill -0 "$CLOUD_RELAY_PID" 2>/dev/null; then
    kill -TERM "$CLOUD_RELAY_PID" 2>/dev/null || true
    wait "$CLOUD_RELAY_PID" 2>/dev/null || true
  fi
  if [[ -n "$STAMP_PID" ]] && kill -0 "$STAMP_PID" 2>/dev/null; then
    kill -TERM "$STAMP_PID" 2>/dev/null || true
    wait "$STAMP_PID" 2>/dev/null || true
  fi
  if [[ -n "$IDENTITY_PID" ]] && kill -0 "$IDENTITY_PID" 2>/dev/null; then
    kill -TERM "$IDENTITY_PID" 2>/dev/null || true
    wait "$IDENTITY_PID" 2>/dev/null || true
  fi
  cleanup_owned_session_pointer
  return "$nav_cleanup_status"
}
on_exit() {
  local status=$? cleanup_status=0
  trap - EXIT INT TERM HUP
  stop_owned_children || cleanup_status=$?
  if (( status == 0 && cleanup_status != 0 )); then
    status=9
  fi
  exit "$status"
}
on_signal() {
  local status="$1" cleanup_status=0
  trap - EXIT INT TERM HUP
  stop_owned_children || cleanup_status=$?
  if (( cleanup_status != 0 )); then
    echo "warning: exact no-motion Nav2 cleanup failed during signal handling" >&2
  fi
  exit "$status"
}
trap on_exit EXIT
trap 'on_signal 129' HUP
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

# Remove only a fully authenticated orphan from an earlier no-motion run before
# timestamp qualification.  Unknown or ambiguous containers fail closed.
verify_nomotion_nav2_stop_hook
cleanup_stale_nomotion_nav2_containers

RUNTIME_CONFIG_DIR="$RUN_DIR/config"
install -d -m 700 -- "$RUNTIME_CONFIG_DIR"
materialize_runtime_config() {
  local source="$1"
  local destination="$2"
  [[ -f "$source" && ! -L "$source" ]] || {
    echo "audited runtime config must be a regular non-symlink file: $source" >&2
    exit 6
  }
  install -m 600 -- "$source" "$destination"
}
materialize_runtime_config \
  "$ROOT/config/rtabmap_params.yaml" \
  "$RUNTIME_CONFIG_DIR/rtabmap_params.yaml"
materialize_runtime_config \
  "$ROOT/config/nav2_params_go2.yaml" \
  "$RUNTIME_CONFIG_DIR/nav2_params_go2.yaml"
materialize_runtime_config \
  "$ROOT/config/navigate.xml" \
  "$RUNTIME_CONFIG_DIR/navigate.xml"
materialize_runtime_config \
  "$ROOT/config/navigate_through_poses.xml" \
  "$RUNTIME_CONFIG_DIR/navigate_through_poses.xml"
READY_FILE="$RUN_DIR/ready.json"
FAULT_FILE="$RUN_DIR/fault.json"
IDENTITY_READY_FILE="$RUN_DIR/identity-ready.json"
IDENTITY_FAULT_FILE="$RUN_DIR/identity-fault.json"
CLOUD_RELAY_READY_FILE="$RUN_DIR/cloud-relay-ready.json"
CLOUD_RELAY_FAULT_FILE="$RUN_DIR/cloud-relay-fault.json"
MANIFEST="$RUN_DIR/robonix_manifest.yaml"
SCENE_START_CONFIG="$RUNTIME_CONFIG_DIR/scene-start-config.json"
NODE_LOG="$RUN_DIR/stamp-node.log"
IDENTITY_LOG="$RUN_DIR/identity-monitor.log"
CLOUD_RELAY_LOG="$RUN_DIR/cloud-relay.log"
MANIFEST_RENDERER="$ROOT/deploy/time-sync/render_workstation_nomotion_manifest.py"
if [[ "$D435I_PREVIEW_ENABLED" == true ]]; then
  MANIFEST_RENDERER="$ROOT/deploy/time-sync/render_workstation_nomotion_d435i_preview_manifest.py"
fi
readonly MANIFEST_RENDERER

"$PYTHON" "$MANIFEST_RENDERER" \
  --base "$ROOT/robonix_manifest.yaml" \
  --state-marker "$validated_state_marker" \
  --passive-state-markers "$PASSIVE_MANUAL_MAPPING_STATE_MARKERS" \
  --output "$MANIFEST" >/dev/null

scene_config_args=()
if [[ "$D435I_PREVIEW_ENABLED" == true ]]; then
  scene_config_args+=(--require-d435i-preview)
fi
"$PYTHON" "$ROOT/scripts/materialize_scene_start_config.py" \
  --manifest "$MANIFEST" \
  --output "$SCENE_START_CONFIG" \
  "${scene_config_args[@]}" >/dev/null

(
  go2_nomotion_close_inherited_fd "$DISCIPLINE_LOCK_FD"
  exec "$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_identity_monitor.py" \
    --approval-file "$APPROVAL_FILE" \
    --ready-file "$IDENTITY_READY_FILE" \
    --fault-file "$IDENTITY_FAULT_FILE"
) >"$IDENTITY_LOG" 2>&1 &
IDENTITY_PID=$!

deadline=$((SECONDS + 20))
while true; do
  if [[ -s "$IDENTITY_FAULT_FILE" ]]; then
    echo "writer identity monitor faulted; detail: $IDENTITY_FAULT_FILE" >&2
    exit 7
  fi
  if ! kill -0 "$IDENTITY_PID" 2>/dev/null; then
    identity_status=0
    wait "$IDENTITY_PID" || identity_status=$?
    echo "writer identity monitor exited before readiness (status=$identity_status)" >&2
    [[ ! -s "$IDENTITY_FAULT_FILE" ]] || echo "fault detail: $IDENTITY_FAULT_FILE" >&2
    exit 7
  fi
  [[ ! -s "$IDENTITY_READY_FILE" ]] || break
  if (( SECONDS >= deadline )); then
    echo "exact per-stream writer GID discovery did not become ready within 20 seconds" >&2
    exit 7
  fi
  sleep 0.1
done

(
  go2_nomotion_close_inherited_fd "$DISCIPLINE_LOCK_FD"
  exec "$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_stamp_node.py" \
    --mode affine \
    --profile nomotion \
    --approval-file "$APPROVAL_FILE" \
    --ready-file "$READY_FILE" \
    --fault-file "$FAULT_FILE"
) >"$NODE_LOG" 2>&1 &
STAMP_PID=$!

deadline=$((SECONDS + 90))
while true; do
  if [[ -s "$FAULT_FILE" ]]; then
    echo "timestamp qualification faulted; detail: $FAULT_FILE" >&2
    exit 7
  fi
  if [[ -s "$IDENTITY_FAULT_FILE" ]]; then
    echo "writer identity monitor faulted; detail: $IDENTITY_FAULT_FILE" >&2
    exit 7
  fi
  if ! kill -0 "$IDENTITY_PID" 2>/dev/null; then
    echo "writer identity monitor stopped during timestamp qualification" >&2
    [[ ! -s "$IDENTITY_FAULT_FILE" ]] || echo "fault detail: $IDENTITY_FAULT_FILE" >&2
    exit 7
  fi
  if ! kill -0 "$STAMP_PID" 2>/dev/null; then
    wait "$STAMP_PID" || status=$?
    echo "timestamp discipline exited before readiness (status=${status:-0})" >&2
    [[ ! -s "$FAULT_FILE" ]] || echo "fault detail: $FAULT_FILE" >&2
    exit 7
  fi
  [[ ! -s "$READY_FILE" ]] || break
  if (( SECONDS >= deadline )); then
    echo "timestamp qualification did not become ready within 90 seconds" >&2
    exit 7
  fi
  sleep 0.1
done

# Fault files win even at the final boundary before the no-motion stack is
# started.  READY can never mask a simultaneously visible terminal FAULT.
if [[ -s "$FAULT_FILE" ]]; then
  echo "timestamp qualification faulted; detail: $FAULT_FILE" >&2
  exit 7
fi
if [[ -s "$IDENTITY_FAULT_FILE" ]]; then
  echo "writer identity monitor faulted; detail: $IDENTITY_FAULT_FILE" >&2
  exit 7
fi

(
  go2_nomotion_close_inherited_fd "$DISCIPLINE_LOCK_FD"
  exec "$PYTHON" \
    "$ROOT/deploy/time-sync/workstation_nomotion_cloud_relay.py" \
    --approval-file "$APPROVAL_FILE" \
    --stamp-ready-file "$READY_FILE" \
    --stamp-fault-file "$FAULT_FILE" \
    --identity-fault-file "$IDENTITY_FAULT_FILE" \
    --stamp-pid "$STAMP_PID" \
    --identity-pid "$IDENTITY_PID" \
    --ready-file "$CLOUD_RELAY_READY_FILE" \
    --fault-file "$CLOUD_RELAY_FAULT_FILE"
) >"$CLOUD_RELAY_LOG" 2>&1 &
CLOUD_RELAY_PID=$!

deadline=$((SECONDS + 20))
while true; do
  if [[ -s "$FAULT_FILE" ]]; then
    echo "timestamp discipline faulted while starting cloud relay; detail: $FAULT_FILE" >&2
    exit 7
  fi
  if [[ -s "$IDENTITY_FAULT_FILE" ]]; then
    echo "writer identity monitor faulted while starting cloud relay; detail: $IDENTITY_FAULT_FILE" >&2
    exit 7
  fi
  if [[ -s "$CLOUD_RELAY_FAULT_FILE" ]]; then
    echo "isolated corrected cloud relay faulted; detail: $CLOUD_RELAY_FAULT_FILE" >&2
    exit 7
  fi
  if ! kill -0 "$STAMP_PID" 2>/dev/null; then
    echo "timestamp discipline stopped while starting cloud relay" >&2
    exit 7
  fi
  if ! kill -0 "$IDENTITY_PID" 2>/dev/null; then
    echo "writer identity monitor stopped while starting cloud relay" >&2
    exit 7
  fi
  if ! kill -0 "$CLOUD_RELAY_PID" 2>/dev/null; then
    cloud_status=0
    wait "$CLOUD_RELAY_PID" || cloud_status=$?
    echo "isolated corrected cloud relay exited before readiness (status=$cloud_status)" >&2
    [[ ! -s "$CLOUD_RELAY_FAULT_FILE" ]] || \
      echo "fault detail: $CLOUD_RELAY_FAULT_FILE" >&2
    exit 7
  fi
  [[ ! -s "$CLOUD_RELAY_READY_FILE" ]] || break
  if (( SECONDS >= deadline )); then
    echo "isolated corrected cloud relay did not become ready within 20 seconds" >&2
    exit 7
  fi
  sleep 0.1
done

# A simultaneously visible relay fault always wins over READY.
if [[ -s "$CLOUD_RELAY_FAULT_FILE" ]]; then
  echo "isolated corrected cloud relay faulted; detail: $CLOUD_RELAY_FAULT_FILE" >&2
  exit 7
fi

echo "Affine timestamp correction locked; chassis owns canonical /odom + TF; motion remains disabled."
(
  go2_nomotion_close_inherited_fd "$DISCIPLINE_LOCK_FD"
  export ROBONIX_MANIFEST="$MANIFEST"
  # rbnx delivers package config through Driver(CMD_INIT), while Scene is a
  # driverless system provider whose camera pins and perception policy are
  # needed before registration.  Only Scene consumes this legacy variable in
  # the audited workspace.  Scope the allowlisted private file to the stack
  # child tree; do not export the full deployment manifest or any secret.
  export RBNX_CONFIG_FILE="$SCENE_START_CONFIG"
  # Keep the timestamp and writer-identity sentinels at the normal scheduler
  # priority. Robonix startup can briefly saturate CPU while Docker, RTAB-Map
  # and speech initialize; lower only the stack subtree so a safe 100 ms
  # corrected-state freshness gate is not defeated by avoidable scheduling
  # inversion. Increasing niceness is unprivileged and inherited fail-closed.
  renice -n "$STACK_NICE_LEVEL" -p "$BASHPID" >/dev/null || {
    echo "could not lower no-motion stack scheduling priority" >&2
    exit 8
  }
  exec bash "$ROOT/start.sh"
) &
STACK_PID=$!

WAIT_PIDS=("$IDENTITY_PID" "$STAMP_PID" "$CLOUD_RELAY_PID" "$STACK_PID")
while ((${#WAIT_PIDS[@]} > 0)); do
  set +e
  EXITED_PID=""
  wait -n -p EXITED_PID "${WAIT_PIDS[@]}"
  status=$?
  set -e
  if [[ "$EXITED_PID" == "$STACK_PID" ]]; then
    exit "$status"
  fi
  if [[ "$EXITED_PID" == "$STAMP_PID" ]]; then
    echo "WARNING: timestamp discipline stopped; the no-motion UI/map stack remains available in degraded mode" >&2
    STAMP_PID=""
  elif [[ "$EXITED_PID" == "$IDENTITY_PID" ]]; then
    echo "WARNING: writer identity monitor stopped; the no-motion UI/map stack remains available in degraded mode" >&2
    IDENTITY_PID=""
  elif [[ "$EXITED_PID" == "$CLOUD_RELAY_PID" ]]; then
    echo "WARNING: isolated corrected cloud relay stopped; the no-motion UI/map stack remains available in degraded mode" >&2
    CLOUD_RELAY_PID=""
  else
    echo "managed-child wait ended without an exact child identity" >&2
    exit 75
  fi
  remaining_pids=()
  for managed_pid in "${WAIT_PIDS[@]}"; do
    [[ "$managed_pid" == "$EXITED_PID" ]] || remaining_pids+=("$managed_pid")
  done
  WAIT_PIDS=("${remaining_pids[@]}")
done
exit 75
