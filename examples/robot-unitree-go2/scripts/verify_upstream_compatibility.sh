#!/usr/bin/env bash
# Static, read-only compatibility gate for safety-critical upstream packages.

set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBONIX_ROOT="$(rbnx path root)"
EVIDENCE_DIR="$DEPLOY_DIR/rbnx-build"
EVIDENCE_FILE="$EVIDENCE_DIR/upstream-lock.txt"

manifest_repo_path() {
  local provider_name="$1"
  local path

  path="$(awk -v wanted="$provider_name" '
    /^  - name:[[:space:]]*/ {
      name = $0
      sub(/^  - name:[[:space:]]*/, "", name)
      active = (name == wanted)
      next
    }
    active && /^    path:[[:space:]]*/ {
      path = $0
      sub(/^    path:[[:space:]]*/, "", path)
      print path
      exit
    }
  ' "$DEPLOY_DIR/robonix_manifest.yaml")"
  path="${path#\"}"
  path="${path%\"}"
  path="${path#\'}"
  path="${path%\'}"
  [[ -n "$path" ]] || {
    echo "manifest provider has no pinned path: $provider_name" >&2
    exit 1
  }
  path="${path//'${ROBONIX_DEPLOY_DIR}'/$DEPLOY_DIR}"
  [[ "$path" = /* ]] || path="$DEPLOY_DIR/$path"
  printf '%s\n' "$path"
}

MAP_ROOT="$(manifest_repo_path mapping)"
NAV_ROOT="$(manifest_repo_path nav2)"

require_file() {
  local path="$1"
  [[ -f "$path" ]] || {
    echo "missing audited upstream file: $path" >&2
    exit 1
  }
}

require_text() {
  local path="$1"
  local expected="$2"
  local label="$3"
  require_file "$path"
  grep -Fq -- "$expected" "$path" || {
    echo "incompatible upstream: $label is absent from $path" >&2
    echo "Merge/update the linked upstream compatibility PR before deployment." >&2
    exit 1
  }
}

forbid_text() {
  local path="$1"
  local forbidden="$2"
  local label="$3"
  require_file "$path"
  if grep -Fq -- "$forbidden" "$path"; then
    echo "incompatible upstream: $label is present in $path" >&2
    echo "Merge/update the linked upstream compatibility PR before deployment." >&2
    exit 1
  fi
}

require_clean_git_checkout() {
  local label="$1"
  local path="$2"
  local dirty

  git -C "$path" rev-parse --is-inside-work-tree >/dev/null 2>&1 || {
    echo "audited upstream is not a git checkout: $label ($path)" >&2
    exit 1
  }
  dirty="$(git -C "$path" status --porcelain --untracked-files=normal)"
  [[ -z "$dirty" ]] && return 0

  # These exceptions are label-specific, explicit, and exact. Each private
  # receipt pins HEAD, raw porcelain status, and the complete tracked diff;
  # each verifier also hard-codes its independently audited patch paths.
  # Robonix has no dirty-receipt path and remains clean-only.
  if [[ "$label" == "mapping" && -n "${ROBONIX_MAPPING_DIRTY_AUDIT_RECEIPT:-}" ]]; then
    python3 "$DEPLOY_DIR/scripts/verify_dirty_upstream_audit.py" \
      --workspace "$DEPLOY_DIR" \
      --repo "$path" \
      --receipt "$ROBONIX_MAPPING_DIRTY_AUDIT_RECEIPT" && return 0
  fi
  if [[ "$label" == "navigation" && -n "${ROBONIX_NAVIGATION_DIRTY_AUDIT_RECEIPT:-}" ]]; then
    python3 "$DEPLOY_DIR/scripts/verify_navigation_dirty_upstream_audit.py" \
      --workspace "$DEPLOY_DIR" \
      --repo "$path" \
      --receipt "$ROBONIX_NAVIGATION_DIRTY_AUDIT_RECEIPT" && return 0
  fi

  echo "audited upstream checkout is dirty: $label ($path)" >&2
  printf '%s\n' "$dirty" >&2
  echo "Commit, discard, or separately audit those changes before deployment." >&2
  exit 1
}

# Scene must discover this robot's canonical contracts, subscribe with the
# publisher-declared QoS, and carry the host's CycloneDDS NIC binding into its
# x86 container.
require_text "$ROBONIX_ROOT/pylib/robonix-api/robonix_api/capability.py" \
  'ROBONIX_PROVIDER_BIND_HOST' "Robonix provider bind-host control"
require_text "$ROBONIX_ROOT/pylib/robonix-api/robonix_api/capability.py" \
  'host=self._bind_host' "Robonix provider HTTP bind-host enforcement"
require_text "$ROBONIX_ROOT/system/scene/scene_service/service.py" \
  "robonix/primitive/lidar/lidar3d" "Scene canonical lidar3d contract"
require_text "$ROBONIX_ROOT/system/scene/scene_service/service.py" \
  "robonix/primitive/chassis/odom" "Scene chassis odometry contract"
require_text "$ROBONIX_ROOT/system/scene/scene_service/perception_policy.py" \
  "def perception_enabled" "Scene strict perception policy"
require_text "$ROBONIX_ROOT/system/scene/scene_service/service.py" \
  "if not perception_enabled(config):" "Scene preview-only perception gate"
require_text "$ROBONIX_ROOT/system/scene/scene_service/ingest/ros_subscribers.py" \
  "def topic_qos_policy" "Scene Atlas QoS policy"
require_text "$ROBONIX_ROOT/system/scene/docker/Dockerfile" \
  'ros-$ROS_DISTRO-rmw-cyclonedds-cpp' "Scene CycloneDDS RMW image dependency"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"' "Scene CycloneDDS URI forwarding"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"' \
  "Scene provider bind-host forwarding"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"' \
  "Scene provider advertise-host forwarding"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]]; then' \
  "Scene Docker GPU opt-out gate"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'SCENE_HOST_DATA_DIR="${SCENE_DATA_DIR:-' \
  "Scene host persistence directory"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  '-v "$SCENE_HOST_DATA_DIR":/data/robonix' \
  "Scene persistent data mount"

# Mapping and Navigation are exact gitlink-pinned local paths.
require_text "$MAP_ROOT/docker/Dockerfile" \
  "ros-humble-rmw-cyclonedds-cpp" "Mapping CycloneDDS RMW image dependency"
require_text "$MAP_ROOT/scripts/start.sh" \
  'CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"' "Mapping CycloneDDS URI forwarding"
require_text "$MAP_ROOT/scripts/start.sh" \
  'ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"' \
  "Mapping provider bind-host forwarding"
require_text "$MAP_ROOT/scripts/start.sh" \
  'ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"' \
  "Mapping provider advertise-host forwarding"
require_text "$MAP_ROOT/scripts/start.sh" \
  'MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"' \
  "Mapping visualization explicit opt-in"
require_text "$MAP_ROOT/scripts/start.sh" \
  'MAPPING_WEBUI_HOST="${MAPPING_WEBUI_HOST:-127.0.0.1}"' \
  "Mapping loopback WebUI forwarding"
require_text "$MAP_ROOT/scripts/start.sh" \
  'if [[ "$VIZ_ENABLED" == true ]]; then' \
  "Mapping X11 opt-in gate"
require_text "$MAP_ROOT/scripts/start.sh" \
  'XHOST_AUTHORIZED=true' "Mapping X11 authorization tracking"
require_text "$MAP_ROOT/scripts/start.sh" \
  'xhost -local:docker' "Mapping X11 authorization cleanup"
require_text "$MAP_ROOT/scripts/build_ros2_overlay.sh" \
  'colcon build --packages-select map' \
  "Mapping generated system-interface isolation"
require_text "$MAP_ROOT/src/mapping_rbnx/webui.py" \
  'os.environ.get("MAPPING_WEBUI_HOST", "127.0.0.1")' \
  "Mapping loopback WebUI default"

require_text "$NAV_ROOT/docker/Dockerfile" \
  "ros-humble-rmw-cyclonedds-cpp" "Navigation CycloneDDS RMW image dependency"
require_text "$NAV_ROOT/scripts/start.sh" \
  'CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"' "Navigation CycloneDDS URI forwarding"
require_text "$NAV_ROOT/scripts/start.sh" \
  'ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"' \
  "Navigation provider bind-host forwarding"
require_text "$NAV_ROOT/scripts/start.sh" \
  'ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"' \
  "Navigation provider advertise-host forwarding"
require_text "$NAV_ROOT/nav2_wrapper/atlas_bridge.py" \
  "cancel queued until goal acceptance" "Navigation pending-cancel latch"
bash "$DEPLOY_DIR/scripts/check_navigation_velocity_contract.sh" "$NAV_ROOT"

# rbnx may retain an older URL cache even after the manifest changes to an
# exact local path. Any Navigation-looking runtime cache must satisfy the same
# no-motion routing contract or startup/build fails before it can be executed.
shopt -s nullglob
NAV_CACHE_ROOTS=("$DEPLOY_DIR"/rbnx-boot/cache/*navigation*)
shopt -u nullglob
for cache_root in "${NAV_CACHE_ROOTS[@]}"; do
  [[ -d "$cache_root" ]] || continue
  bash "$DEPLOY_DIR/scripts/check_navigation_velocity_contract.sh" "$cache_root"
done
require_text "$NAV_ROOT/scripts/build.sh" \
  'rbnx codegen --mcp' "Navigation MCP-only code generation"
forbid_text "$NAV_ROOT/scripts/build.sh" \
  '--ros2' "Navigation generated ROS system-interface overlay"
forbid_text "$NAV_ROOT/scripts/start_native.sh" \
  'codegen/ros2_idl' "Navigation native generated ROS overlay source"
forbid_text "$NAV_ROOT/docker/entrypoint.sh" \
  'codegen/ros2_idl' "Navigation container generated ROS overlay source"

require_clean_git_checkout "robonix" "$ROBONIX_ROOT"
require_clean_git_checkout "mapping" "$MAP_ROOT"
require_clean_git_checkout "navigation" "$NAV_ROOT"

mkdir -p "$EVIDENCE_DIR"
{
  printf 'Audited upstream compatibility evidence\n'
  printf 'Generated UTC: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  for entry in "robonix:$ROBONIX_ROOT" "mapping:$MAP_ROOT" "navigation:$NAV_ROOT"; do
    name="${entry%%:*}"
    path="${entry#*:}"
    revision="$(git -C "$path" rev-parse HEAD)"
    printf '%s %s\n' "$name" "$revision"
  done
} >"$EVIDENCE_FILE"

echo "[upstream-compat] PASS; revisions recorded in $EVIDENCE_FILE"
