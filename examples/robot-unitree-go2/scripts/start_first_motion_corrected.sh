#!/usr/bin/env bash
set -euo pipefail
umask 077

# Supervised one-shot commissioning launcher.  This is not teleoperation and
# not a navigation entry point.  It can run only the fixed 0.05 m/s, 2 s,
# 0.10 m forward probe on /go2/commissioning/cmd_vel.

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(CDPATH= cd -- "$ROOT/../.." && pwd)"
PYTHON="$WORKSPACE_ROOT/.tools/rbnx-python/bin/python3"
RBNX="$WORKSPACE_ROOT/.tools/rbnx/bin/rbnx"
UNITREE_SETUP="${UNITREE_ROS2_SETUP:-$ROOT/rbnx-build/unitree_ros2/install/setup.bash}"
MOTION_STATE_RELAY_BINARY="$ROOT/packages/go2_motion_state_relay/.build/ros/install/go2_motion_state_relay/lib/go2_motion_state_relay/workstation_motion_state_relay"
PROFILE=workstation-first-motion-corrected-v1
SITE_ACK=I_CONFIRM_GO2_CLEAR_2M_REMOTE_STOP_READY
MOTION_ACK=I_APPROVE_GO2_FIRST_10CM_MOTION
WIRELESS_ACK=I_APPROVE_GO2_WIRELESS_PRIVATE_LAN_FIRST_10CM
TRANSPORT="${GO2_FIRST_MOTION_TRANSPORT:-wired}"
WIRELESS_INTERFACE=wlx500ff54809b8
WIRELESS_CONNECTION_NAME=Robonix-Go2
WIRELESS_CONNECTION_UUID=ce767234-9037-4a53-a5f4-aa7b6cbf743f
INTERNET_INTERFACE=wlo1
ROBOT_PRIVATE_IP=192.168.123.161
ORIN_PRIVATE_IP=192.168.123.18

if [[ "$#" -ne 0 ]]; then
  echo "this launcher accepts no arguments" >&2
  exit 2
fi

# Do not source .env.  A reusable config file must never fill a physical-site
# confirmation, one-time permit path, or motion acknowledgement.
[[ "${GO2_ALLOW_MOTION:-}" == true ]] || {
  echo "GO2_ALLOW_MOTION must exactly equal true" >&2
  exit 2
}
[[ "${GO2_OPERATOR_PRESENT:-}" == true ]] || {
  echo "GO2_OPERATOR_PRESENT must exactly equal true" >&2
  exit 2
}
[[ "${GO2_SAFETY_ACK:-}" == I_UNDERSTAND_GO2_CAN_MOVE ]] || {
  echo "GO2_SAFETY_ACK does not match the exact runtime gate" >&2
  exit 2
}
[[ "${GO2_FIRST_MOTION_ACK:-}" == "$MOTION_ACK" ]] || {
  echo "GO2_FIRST_MOTION_ACK must exactly equal $MOTION_ACK" >&2
  exit 2
}
[[ "${GO2_SITE_SAFETY_ACK:-}" == "$SITE_ACK" ]] || {
  echo "GO2_SITE_SAFETY_ACK must exactly equal $SITE_ACK" >&2
  exit 2
}
case "$TRANSPORT" in
  wired)
    ;;
  wireless-private)
    [[ "${GO2_WIRELESS_MOTION_ACK:-}" == "$WIRELESS_ACK" ]] || {
      echo "GO2_WIRELESS_MOTION_ACK must exactly equal $WIRELESS_ACK" >&2
      exit 2
    }
    [[ "${GO2_NETWORK_INTERFACE:-}" == "$WIRELESS_INTERFACE" ]] || {
      echo "wireless first motion must use $WIRELESS_INTERFACE" >&2
      exit 2
    }
    ;;
  *)
    echo "GO2_FIRST_MOTION_TRANSPORT must be wired or wireless-private" >&2
    exit 2
    ;;
esac

for variable in GO2_NETWORK_INTERFACE GO2_FIRST_MOTION_PERMIT_FILE \
  GO2_FIRST_MOTION_STATE_EVIDENCE GO2_FIRST_MOTION_DDS_IDENTITY_EVIDENCE \
  GO2_TIMESTAMP_APPROVAL_FILE; do
  value="${!variable:-}"
  [[ -n "$value" ]] || {
    echo "$variable is required" >&2
    exit 2
  }
done
[[ "$GO2_NETWORK_INTERFACE" =~ ^[A-Za-z0-9_.:-]+$ ]] || {
  echo "GO2_NETWORK_INTERFACE contains unsupported characters" >&2
  exit 2
}
for variable in GO2_FIRST_MOTION_PERMIT_FILE \
  GO2_FIRST_MOTION_STATE_EVIDENCE GO2_FIRST_MOTION_DDS_IDENTITY_EVIDENCE \
  GO2_TIMESTAMP_APPROVAL_FILE; do
  value="${!variable}"
  [[ "$value" == /* && -f "$value" && ! -L "$value" ]] || {
    echo "$variable must name an absolute regular non-symlink file" >&2
    exit 2
  }
done

[[ -x "$PYTHON" && -x "$RBNX" ]] || {
  echo "workspace-local Robonix Python/rbnx is missing" >&2
  exit 3
}
"$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_approval.py" \
  --require-affine "$GO2_TIMESTAMP_APPROVAL_FILE" >/dev/null
[[ -f /opt/ros/humble/setup.bash && -f "$UNITREE_SETUP" ]] || {
  echo "ROS Humble or the built Unitree message overlay is missing" >&2
  exit 3
}
[[ -x "$MOTION_STATE_RELAY_BINARY" && -f "$MOTION_STATE_RELAY_BINARY" \
  && ! -L "$MOTION_STATE_RELAY_BINARY" ]] || {
  echo "built C++ motion-state GID guard is missing; run packages/go2_motion_state_relay/scripts/build_ros.sh" >&2
  exit 3
}
for command in awk basename cat chmod cmp dirname flock grep id install ip \
  mkdir mktemp mv nmcli readlink rm sed seq sleep stat timeout tr; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing required command: $command" >&2
    exit 3
  }
done

# Read-only network topology validation.  This launcher never changes an
# address, route, DNS server, interface state, or NetworkManager profile.
ip link show dev "$GO2_NETWORK_INTERFACE" >/dev/null 2>&1 || {
  echo "network interface does not exist: $GO2_NETWORK_INTERFACE" >&2
  exit 4
}
interface_sysfs="$(readlink -f -- "/sys/class/net/$GO2_NETWORK_INTERFACE")"
[[ -n "$interface_sysfs" && "$interface_sysfs" != *"/virtual/"* ]] || {
  echo "Go2 interface must be a physical device" >&2
  exit 4
}
if [[ "$TRANSPORT" == wired ]]; then
  [[ ! -d "/sys/class/net/$GO2_NETWORK_INTERFACE/wireless" ]] || {
    echo "Go2 interface must be a physical wired device" >&2
    exit 4
  }
else
  [[ -d "/sys/class/net/$GO2_NETWORK_INTERFACE/wireless" ]] || {
    echo "wireless-private transport requires a physical Wi-Fi device" >&2
    exit 4
  }
fi
[[ "$(cat -- "/sys/class/net/$GO2_NETWORK_INTERFACE/operstate")" == up ]] || {
  echo "Go2 interface is not UP" >&2
  exit 4
}
mapfile -t ipv4 < <(
  ip -o -4 addr show dev "$GO2_NETWORK_INTERFACE" | awk '{print $4}'
)
[[ "${#ipv4[@]}" -eq 1 && "${ipv4[0]}" == 192.168.123.99/24 ]] || {
  echo "Go2 interface must have exactly 192.168.123.99/24" >&2
  exit 4
}
mapfile -t ipv6 < <(
  ip -o -6 addr show dev "$GO2_NETWORK_INTERFACE" | awk '{print $4}'
)
[[ "${#ipv6[@]}" -eq 0 ]] || {
  echo "Go2 interface must have no IPv6 address" >&2
  exit 4
}
if ip -4 route show default dev "$GO2_NETWORK_INTERFACE" | grep -q . \
  || ip -6 route show default dev "$GO2_NETWORK_INTERFACE" | grep -q .; then
  echo "Go2 interface must have no default route" >&2
  exit 4
fi
nm_state="$(
  nmcli --get-values IP4.GATEWAY,IP4.DNS,IP6.GATEWAY,IP6.DNS \
    device show "$GO2_NETWORK_INTERFACE" 2>/dev/null \
    | sed '/^[[:space:]]*$/d'
)" || {
  echo "could not verify NetworkManager gateway/DNS state" >&2
  exit 4
}
[[ -z "$nm_state" ]] || {
  echo "Go2 interface must have no gateway or DNS" >&2
  exit 4
}
first_motion_validate_wireless_topology() {
  [[ "$TRANSPORT" == wireless-private ]] || return 0
  "$PYTHON" "$ROOT/scripts/validate_first_motion_network.py" \
    --interface "$GO2_NETWORK_INTERFACE" \
    --transport wireless-private \
    --internet-interface "$INTERNET_INTERFACE" \
    --robot-ip "$ROBOT_PRIVATE_IP" \
    --orin-ip "$ORIN_PRIVATE_IP" \
    --connection-uuid "$WIRELESS_CONNECTION_UUID" \
    --connection-name "$WIRELESS_CONNECTION_NAME"
}
first_motion_validate_wireless_topology || exit 4

# Derive, never guess, the exact mode and opaque marker from a stationary
# subscriber-only capture no older than two minutes.
read -r observed_mode observed_marker observed_gait < <(
  "$PYTHON" "$ROOT/scripts/validate_first_motion_state_evidence.py" \
    "$GO2_FIRST_MOTION_STATE_EVIDENCE"
) || {
  echo "stationary first-motion state marker evidence is required" >&2
  exit 5
}
for value in "$observed_mode" "$observed_marker" "$observed_gait"; do
  [[ "$value" =~ ^[0-9]+$ ]] || {
    echo "validated first-motion state contains a malformed integer" >&2
    exit 5
  }
done
export GO2_ALLOWED_MODES="$observed_mode"
if [[ "$observed_marker" == 0 ]]; then
  export GO2_ALLOWED_STATE_MARKERS=""
else
  export GO2_ALLOWED_STATE_MARKERS="$observed_marker"
fi

"$PYTHON" "$ROOT/scripts/validate_first_motion_permit_for_launch.py" \
  --permit "$GO2_FIRST_MOTION_PERMIT_FILE" \
  --package-root "$ROOT" \
  --network-interface "$GO2_NETWORK_INTERFACE" \
  --allowed-mode "$observed_mode" \
  --allowed-state-marker "$observed_marker" \
  --dds-evidence "$GO2_FIRST_MOTION_DDS_IDENTITY_EVIDENCE" \
  --state-evidence "$GO2_FIRST_MOTION_STATE_EVIDENCE" \
  --time-evidence "$GO2_TIMESTAMP_APPROVAL_FILE" >/dev/null

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232" >&2
  exit 5
}
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_NETWORK_INTERFACE\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
export UNITREE_ROS2_SETUP="$UNITREE_SETUP"
export GO2_ALLOW_MOTION=true
export GO2_OPERATOR_PRESENT=true
export GO2_SAFETY_ACK=I_UNDERSTAND_GO2_CAN_MOVE
export GO2_FIRST_MOTION_ACK="$MOTION_ACK"
export GO2_MOTION_PROFILE="$PROFILE"

export ROBONIX_DEPLOY_DIR="$ROOT"
export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"
export ROBONIX_SOURCE_ROOT="$WORKSPACE_ROOT/upstream/robonix-go2-build"
export CARGO_HOME="$WORKSPACE_ROOT/.tools/cargo"
export CARGO_TARGET_DIR="$WORKSPACE_ROOT/.tools/cargo-target/robonix"
export RUSTUP_HOME="$WORKSPACE_ROOT/.tools/rustup"
export ROBONIX_BUILD_PROFILE="${ROBONIX_BUILD_PROFILE:-debug}"
[[ "$ROBONIX_BUILD_PROFILE" == debug || "$ROBONIX_BUILD_PROFILE" == release ]] || {
  echo "ROBONIX_BUILD_PROFILE must be debug or release" >&2
  exit 5
}
export ROBONIX_SYSTEM_BIN_DIR="$CARGO_TARGET_DIR/$ROBONIX_BUILD_PROFILE"
export PATH="$WORKSPACE_ROOT/.tools/rbnx-python/bin:$WORKSPACE_ROOT/.tools/rbnx/bin:$ROBONIX_SYSTEM_BIN_DIR:$PATH"
"$PYTHON" "$ROOT/scripts/validate_robonix_home.py" \
  "$ROBONIX_HOME/config.yaml" "$ROBONIX_SOURCE_ROOT" >/dev/null

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$UNITREE_SETUP"
set -u

# The private relay does not publish its first 30 qualification samples, and
# the chassis adapter must not exist until relay READY has been accepted.
first_motion_require_no_adapter() {
  local existing_ros_nodes
  existing_ros_nodes="$(
    LC_ALL=C timeout --signal=INT --kill-after=1s 3s \
      ros2 node list --no-daemon 2>/dev/null
  )" || {
    echo "could not prove the pre-relay ROS node graph is free of a chassis adapter" >&2
    return 1
  }
  if grep -Fxq /go2_chassis_adapter <<<"$existing_ros_nodes"; then
    echo "a chassis adapter already exists before motion-state relay READY" >&2
    return 1
  fi
}
first_motion_require_no_adapter || exit 5

mkdir -p "$ROOT/rbnx-build/run"
chmod 700 "$ROOT/rbnx-build/run"
# shellcheck disable=SC1091
source "$ROOT/scripts/runtime_lease.sh"
go2_runtime_lease_acquire "$ROOT/rbnx-build/run" "$PROFILE"

RUN_DIR="$(mktemp -d "$ROOT/rbnx-build/run/first-motion-corrected.XXXXXX")"
chmod 700 "$RUN_DIR"
MANIFEST="$RUN_DIR/robonix_manifest.yaml"
SOMA="$RUN_DIR/soma.yaml"
STAMP_READY="$RUN_DIR/stamp-ready.json"
STAMP_FAULT="$RUN_DIR/stamp-fault.json"
IDENTITY_READY="$RUN_DIR/identity-ready.json"
IDENTITY_FAULT="$RUN_DIR/identity-fault.json"
MOTION_STATE_READY="$RUN_DIR/motion-state-ready.json"
MOTION_STATE_FAULT="$RUN_DIR/motion-state-fault.json"
SPORT_REQUEST_BASELINE="$RUN_DIR/sport-request-baseline.json"

"$PYTHON" "$ROOT/deploy/time-sync/render_workstation_first_motion_manifest.py" \
  --base "$ROOT/robonix_manifest.yaml" \
  --base-soma "$ROOT/soma.yaml" \
  --package-root "$ROOT" \
  --output "$MANIFEST" \
  --soma-output "$SOMA" >/dev/null

IDENTITY_PID=""
STAMP_PID=""
RELAY_PID=""
BOOT_PID=""
PROBE_PID=""
cleanup_started=false

terminate_child() {
  local pid="$1" signal_name="${2:-TERM}" count
  [[ -n "$pid" && "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -"$signal_name" "$pid" 2>/dev/null || true
  for count in $(seq 1 50); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  # Losing the probe publisher makes both independent watchdogs stop.  A
  # bounded KILL here is safer than leaving an unresponsive controller alive.
  kill -KILL "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  [[ "$cleanup_started" == false ]] || exit "$status"
  cleanup_started=true
  trap - EXIT HUP INT TERM
  # Stop command production first, then Robonix (adapter/daemon), then the
  # state relays.  Every layer independently treats loss as a stop condition.
  terminate_child "$PROBE_PID" TERM
  [[ -z "$PROBE_PID" ]] || wait "$PROBE_PID" 2>/dev/null || true
  terminate_child "$BOOT_PID" TERM
  [[ -z "$BOOT_PID" ]] || wait "$BOOT_PID" 2>/dev/null || true
  if [[ -n "$BOOT_PID" && ( -e "$RUN_DIR/rbnx-boot/state.json" \
    || -L "$RUN_DIR/rbnx-boot/state.json" ) ]]; then
    "$PYTHON" "$ROOT/scripts/shutdown_rbnx_boot_state.py" \
      --manifest "$MANIFEST" --rbnx "$RBNX" \
      --expected-boot-pid "$BOOT_PID" || true
  fi
  terminate_child "$RELAY_PID" TERM
  terminate_child "$STAMP_PID" TERM
  terminate_child "$IDENTITY_PID" TERM
  [[ -z "$RELAY_PID" ]] || wait "$RELAY_PID" 2>/dev/null || true
  [[ -z "$STAMP_PID" ]] || wait "$STAMP_PID" 2>/dev/null || true
  [[ -z "$IDENTITY_PID" ]] || wait "$IDENTITY_PID" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

first_motion_fault_first() {
  local label path
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

"$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_identity_monitor.py" \
  --approval-file "$GO2_TIMESTAMP_APPROVAL_FILE" \
  --ready-file "$IDENTITY_READY" --fault-file "$IDENTITY_FAULT" \
  >"$RUN_DIR/identity-monitor.log" 2>&1 &
IDENTITY_PID=$!

deadline=$((SECONDS + 20))
while true; do
  first_motion_fault_first \
    "writer identity monitor" "$IDENTITY_FAULT" || exit 6
  kill -0 "$IDENTITY_PID" 2>/dev/null || {
    echo "writer identity monitor exited before readiness" >&2
    exit 6
  }
  [[ ! -s "$IDENTITY_READY" ]] || break
  (( SECONDS < deadline )) || {
    echo "writer identity monitor readiness deadline expired" >&2
    exit 6
  }
  sleep 0.1
done

"$PYTHON" "$ROOT/deploy/time-sync/workstation_nomotion_stamp_node.py" \
  --mode affine \
  --profile motion \
  --approval-file "$GO2_TIMESTAMP_APPROVAL_FILE" \
  --ready-file "$STAMP_READY" --fault-file "$STAMP_FAULT" \
  >"$RUN_DIR/stamp-node.log" 2>&1 &
STAMP_PID=$!

deadline=$((SECONDS + 90))
while true; do
  first_motion_fault_first \
    "timestamp qualification" "$STAMP_FAULT" \
    "writer identity monitor" "$IDENTITY_FAULT" || exit 6
  kill -0 "$IDENTITY_PID" 2>/dev/null || {
    echo "writer identity monitor stopped during timestamp qualification" >&2
    exit 6
  }
  kill -0 "$STAMP_PID" 2>/dev/null || {
    echo "timestamp discipline exited before readiness" >&2
    exit 6
  }
  [[ ! -s "$STAMP_READY" ]] || break
  (( SECONDS < deadline )) || {
    echo "timestamp qualification deadline expired" >&2
    exit 6
  }
  sleep 0.1
done

first_motion_require_no_adapter || exit 6

"$PYTHON" "$ROOT/deploy/time-sync/workstation_motion_state_relay.py" \
  --approval-file "$GO2_TIMESTAMP_APPROVAL_FILE" \
  --stamp-ready-file "$STAMP_READY" \
  --worker-binary "$MOTION_STATE_RELAY_BINARY" \
  --ready-file "$MOTION_STATE_READY" --fault-file "$MOTION_STATE_FAULT" \
  >"$RUN_DIR/motion-state-relay.log" 2>&1 &
RELAY_PID=$!

deadline=$((SECONDS + 10))
while true; do
  first_motion_fault_first \
    "corrected motion-state relay" "$MOTION_STATE_FAULT" \
    "timestamp qualification" "$STAMP_FAULT" \
    "writer identity monitor" "$IDENTITY_FAULT" || exit 6
  for pid in "$IDENTITY_PID" "$STAMP_PID" "$RELAY_PID"; do
    kill -0 "$pid" 2>/dev/null || {
      echo "corrected motion-state chain exited before readiness" >&2
      exit 6
    }
  done
  [[ ! -s "$MOTION_STATE_READY" ]] || break
  (( SECONDS < deadline )) || {
    echo "corrected motion-state relay readiness deadline expired" >&2
    exit 6
  }
  sleep 0.1
done

first_motion_fault_first \
  "corrected motion-state relay" "$MOTION_STATE_FAULT" \
  "timestamp qualification" "$STAMP_FAULT" \
  "writer identity monitor" "$IDENTITY_FAULT" || exit 6
for pid in "$IDENTITY_PID" "$STAMP_PID" "$RELAY_PID"; do
  kill -0 "$pid" 2>/dev/null || {
    echo "a required state-chain process stopped before Robonix boot" >&2
    exit 6
  }
done

# The Go2 firmware itself exposes multiple built-in sport request writers.
# Record that stable pre-boot set, then require Robonix boot to add exactly one
# new workstation SDK writer while the one-shot probe is alive.
"$PYTHON" "$ROOT/scripts/capture_motion_rpc_graph_baseline.py" \
  --output "$SPORT_REQUEST_BASELINE" >/dev/null || {
  echo "could not capture a stable pre-boot sport request writer baseline" >&2
  exit 6
}
export GO2_SPORT_REQUEST_BASELINE_FILE="$SPORT_REQUEST_BASELINE"

# Only this point can launch the manifest containing go2_chassis_adapter.  The
# worker's first 30 GID-verified samples therefore had no intended motion-chain
# subscriber, and READY/fault ordering above remains fault-first.
echo "================================================================"
echo " PRIVATE FIRST-MOTION GRAPH READY"
echo " max forward: 0.05 m/s"
echo " soft stop: 1.8 s / 0.09 m; hard envelope: 2.0 s / 0.10 m"
echo " /cmd_vel remains isolated; remote stop must stay in hand"
echo " runtime evidence: $RUN_DIR"
echo "================================================================"

"$RBNX" boot --no-update-check -f "$MANIFEST" &
BOOT_PID=$!

deadline=$((SECONDS + 30))
while true; do
  first_motion_fault_first \
    "corrected motion-state relay" "$MOTION_STATE_FAULT" \
    "timestamp qualification" "$STAMP_FAULT" \
    "writer identity monitor" "$IDENTITY_FAULT" || exit 7
  for pid in "$IDENTITY_PID" "$STAMP_PID" "$RELAY_PID" "$BOOT_PID"; do
    kill -0 "$pid" 2>/dev/null || {
      echo "a required first-motion process exited before adapter readiness" >&2
      exit 7
    }
  done
  if LC_ALL=C timeout --signal=INT --kill-after=1s 3s \
    ros2 node list --no-daemon 2>/dev/null \
    | grep -Fxq /go2_chassis_adapter; then
    break
  fi
  (( SECONDS < deadline )) || {
    echo "guarded chassis adapter readiness deadline expired" >&2
    exit 7
  }
  sleep 0.2
done

first_motion_fault_first \
  "corrected motion-state relay" "$MOTION_STATE_FAULT" \
  "timestamp qualification" "$STAMP_FAULT" \
  "writer identity monitor" "$IDENTITY_FAULT" || exit 7
for pid in "$IDENTITY_PID" "$STAMP_PID" "$RELAY_PID" "$BOOT_PID"; do
  kill -0 "$pid" 2>/dev/null || {
    echo "a required first-motion process stopped before probe launch" >&2
    exit 7
  }
done
first_motion_validate_wireless_topology || {
  echo "wireless topology changed before the first-motion probe" >&2
  exit 7
}

"$PYTHON" "$ROOT/scripts/first_motion_probe.py" &
PROBE_PID=$!
set +e
EXITED_PID=""
wait -n -p EXITED_PID \
  "$PROBE_PID" "$BOOT_PID" "$RELAY_PID" "$STAMP_PID" "$IDENTITY_PID"
status=$?
set -e
if [[ "${EXITED_PID:-}" != "$PROBE_PID" ]]; then
  echo "a safety/state/Robonix owner exited before the one-shot probe" >&2
  exit 70
fi
exit "$status"
