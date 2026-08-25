#!/usr/bin/env bash
# Bounded bootstrap for the subscription-only canonical /odom observer.

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_root="$(cd -- "${script_dir}/.." && pwd)"
readonly ros_setup="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
readonly unitree_setup="${UNITREE_ROS2_SETUP:-${project_root}/rbnx-build/unitree_ros2/install/setup.bash}"
readonly stamp="$(date -u +%Y%m%dT%H%M%SZ)"

if (( $# > 3 )); then
  printf '用法：%s [logs 内新输出目录] [持续秒数] [最大间隙秒数]\n' "$0" >&2
  exit 2
fi

output_dir="${1:-${project_root}/logs/go2-readonly/${stamp}-canonical-odom-stationary}"
duration_seconds="${2:-${GO2_CANONICAL_ODOM_DURATION_SECONDS:-120}}"
gap_threshold_seconds="${3:-${GO2_CANONICAL_ODOM_GAP_THRESHOLD_SECONDS:-0.2}}"

if [[ ! "${duration_seconds}" =~ ^[1-9][0-9]*$ ]] \
  || (( duration_seconds < 1 || duration_seconds > 3600 )); then
  printf '持续秒数必须是 1..3600 的整数，收到 %q。\n' \
    "${duration_seconds}" >&2
  exit 2
fi
if [[ ! "${gap_threshold_seconds}" =~ ^(0|[1-9][0-9]*)(\.[0-9]+)?$ ]]; then
  printf '最大间隙秒数必须是非负十进制数，收到 %q。\n' \
    "${gap_threshold_seconds}" >&2
  exit 2
fi

for command in python3 realpath timeout; do
  command -v "${command}" >/dev/null 2>&1 || {
    printf '缺少依赖：%s；未创建任何 ROS 订阅。\n' "${command}" >&2
    exit 127
  }
done

if ! python3 -c '
import math
import sys

duration = int(sys.argv[1])
gap = float(sys.argv[2])
raise SystemExit(
    0 if math.isfinite(gap) and 0.001 <= gap <= 10.0 and 2.0 * gap < duration else 1
)
' "${duration_seconds}" "${gap_threshold_seconds}"; then
  printf '最大间隙秒数必须在 0.001..10.0 内，且 2 * 间隙必须严格小于持续秒数。\n' >&2
  exit 2
fi

output_dir="$(realpath -m -- "${output_dir}")"
case "${output_dir}" in
  "${project_root}/logs/"*) ;;
  *)
    printf '输出目录必须是 %s/logs/ 内的新目录。\n' \
      "${project_root}" >&2
    exit 2
    ;;
esac
if [[ -e "${output_dir}" ]]; then
  printf '拒绝复用已有输出目录：%s\n' "${output_dir}" >&2
  exit 2
fi

source_setup_or_exit() {
  local label="$1"
  local setup_file="$2"
  local source_status

  if [[ ! -r "${setup_file}" ]]; then
    printf '缺少 %s 环境脚本：%s；未创建任何 ROS 订阅。\n' \
      "${label}" "${setup_file}" >&2
    exit 127
  fi

  set +u
  # shellcheck disable=SC1090
  if source "${setup_file}"; then
    source_status=0
  else
    source_status=$?
  fi
  set -u
  if (( source_status != 0 )); then
    printf '无法加载 %s 环境脚本（状态 %d）：%s；未创建任何 ROS 订阅。\n' \
      "${label}" "${source_status}" "${setup_file}" >&2
    exit 127
  fi
}

source_setup_or_exit "ROS 2" "${ros_setup}"
source_setup_or_exit "Unitree ROS 2 overlay" "${unitree_setup}"

cat <<'BANNER'
=======================================================================
  READ-ONLY / 只读取证：canonical /odom stationary observer
  固定只订阅 /odom；不创建 publisher/service/action/Unitree client，
  不授权运动。外层 timeout 会在内部期限失效时终止进程。
=======================================================================
BANNER

readonly outer_timeout_seconds=$((duration_seconds + 15))
set +e
timeout --signal=TERM --kill-after=5s \
  "${outer_timeout_seconds}s" \
  python3 "${script_dir}/observe_canonical_odom_stationary_readonly.py" \
    --output-dir "${output_dir}" \
    --duration-seconds "${duration_seconds}" \
    --gap-threshold-seconds "${gap_threshold_seconds}"
status=$?
set -e

if (( status == 124 || status == 137 )); then
  printf '外层 timeout 触发（状态 %d）；证据可能不完整：%s\n' \
    "${status}" "${output_dir}" >&2
  exit 1
fi
exit "${status}"
