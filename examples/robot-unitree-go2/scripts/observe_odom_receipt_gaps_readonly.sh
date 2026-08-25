#!/usr/bin/env bash
# Bounded wrapper for the one-, two-, or three-subscription receipt observer.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${1:-${PROJECT_ROOT}/logs/go2-readonly/${STAMP}-odom-receipt-gaps}"
DURATION_SECONDS="${2:-${GO2_ODOM_GAP_DURATION_SECONDS:-900}}"
MAX_EVENTS="${GO2_ODOM_GAP_MAX_EVENTS:-100000}"
RETAINED_GAPS="${GO2_ODOM_GAP_RETAINED_GAPS:-200000}"
INCLUDE_SPORT_PRIMARY="${GO2_ODOM_GAP_INCLUDE_SPORT_PRIMARY:-0}"
INCLUDE_CLOUD="${GO2_ODOM_GAP_INCLUDE_CLOUD:-0}"
CLOUD_QOS_RELIABILITY="${GO2_ODOM_GAP_CLOUD_QOS_RELIABILITY:-best_effort}"

cat <<'BANNER'
=======================================================================
  READ-ONLY / 只读取证：raw Go2 receipt-gap observer
  默认只订阅 /utlidar/robot_odom；显式开关可追加
  /sportmodestate 和 /utlidar/cloud。所有回调都不读取/复制消息负载，
  不创建 publisher/service/action/Unitree client，不授权任何运动。
  记录 receipt gap 严格大于 100 / 150 / 200 ms 的事件和统计。
=======================================================================
BANNER

sport_primary_args=()
case "${INCLUDE_SPORT_PRIMARY}" in
  0) ;;
  1)
    sport_primary_args=(--include-sport-primary)
    printf '已显式启用第二个只读订阅：/sportmodestate。\n'
    ;;
  *)
    printf 'GO2_ODOM_GAP_INCLUDE_SPORT_PRIMARY 只允许 0 或 1，收到 %q。\n' \
      "${INCLUDE_SPORT_PRIMARY}" >&2
    exit 2
    ;;
esac

case "${CLOUD_QOS_RELIABILITY}" in
  best_effort|reliable) ;;
  *)
    printf 'GO2_ODOM_GAP_CLOUD_QOS_RELIABILITY 只允许 best_effort 或 reliable，收到 %q。\n' \
      "${CLOUD_QOS_RELIABILITY}" >&2
    exit 2
    ;;
esac

cloud_args=()
case "${INCLUDE_CLOUD}" in
  0)
    if [[ "${CLOUD_QOS_RELIABILITY}" != best_effort ]]; then
      printf '只有 GO2_ODOM_GAP_INCLUDE_CLOUD=1 时才能选择 cloud reliable QoS。\n' >&2
      exit 2
    fi
    ;;
  1)
    cloud_args=(
      --include-cloud
      --cloud-qos-reliability "${CLOUD_QOS_RELIABILITY}"
    )
    printf '已显式启用只读 cloud 订阅：/utlidar/cloud（QoS reliability=%s）。\n' \
      "${CLOUD_QOS_RELIABILITY}"
    ;;
  *)
    printf 'GO2_ODOM_GAP_INCLUDE_CLOUD 只允许 0 或 1，收到 %q。\n' \
      "${INCLUDE_CLOUD}" >&2
    exit 2
    ;;
esac

for value in "${DURATION_SECONDS}" "${MAX_EVENTS}" "${RETAINED_GAPS}"; do
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    printf 'duration/max-events/retained-gaps 必须是正整数，收到 %q。\n' \
      "${value}" >&2
    exit 2
  fi
done
if (( DURATION_SECONDS > 86400 || MAX_EVENTS > 1000000 || RETAINED_GAPS > 1000000 )); then
  printf '拒绝无界取证：duration<=86400，max-events/retained-gaps<=1000000。\n' >&2
  exit 2
fi
if (( RETAINED_GAPS < 100 )); then
  printf 'retained-gaps 必须至少为 100。\n' >&2
  exit 2
fi

for command in python3 realpath timeout; do
  command -v "${command}" >/dev/null 2>&1 || {
    printf '缺少依赖：%s。未执行取证。\n' "${command}" >&2
    exit 127
  }
done

OUTPUT_DIR="$(realpath -m -- "${OUTPUT_DIR}")"
case "${OUTPUT_DIR}" in
  "${PROJECT_ROOT}/logs/"*) ;;
  *)
    printf '输出目录必须位于 %s/logs/ 内。\n' "${PROJECT_ROOT}" >&2
    exit 2
    ;;
esac

# Keep all ROS runtime logs inside the repository evidence directory too.
ROS_LOG_DIR="$(realpath -m -- "${ROS_LOG_DIR:-${OUTPUT_DIR}/ros-logs}")"
case "${ROS_LOG_DIR}" in
  "${PROJECT_ROOT}"/*) ;;
  *)
    printf 'ROS_LOG_DIR 必须位于工作区包 %s 内。\n' "${PROJECT_ROOT}" >&2
    exit 2
    ;;
esac
export ROS_LOG_DIR
mkdir -p -- "${OUTPUT_DIR}" "${ROS_LOG_DIR}"
chmod 700 -- "${OUTPUT_DIR}" "${ROS_LOG_DIR}"

outer_timeout=$((DURATION_SECONDS + 15))
set +e
timeout --signal=TERM --kill-after=15s --preserve-status "${outer_timeout}s" \
  python3 "${SCRIPT_DIR}/observe_odom_receipt_gaps_readonly.py" \
    --output-dir "${OUTPUT_DIR}" \
    --duration-seconds "${DURATION_SECONDS}" \
    --max-events "${MAX_EVENTS}" \
    --retained-gaps "${RETAINED_GAPS}" \
    --topic "${GO2_ODOM_GAP_TOPIC:-/utlidar/robot_odom}" \
    "${sport_primary_args[@]}" \
    "${cloud_args[@]}"
status=$?
set -e

if (( status == 130 || status == 143 )); then
  if [[ -s "${OUTPUT_DIR}/summary.json" ]]; then
    printf 'observer 收到退出信号（状态 %d）；证据已收尾：%s\n' \
      "${status}" "${OUTPUT_DIR}/summary.json" >&2
  else
    printf 'observer 收到退出信号（状态 %d），但 summary.json 缺失：%s\n' \
      "${status}" "${OUTPUT_DIR}" >&2
  fi
  exit "${status}"
fi
if (( status == 124 || status == 137 )); then
  printf '外层 timeout 触发（状态 %d）；证据可能不完整：%s\n' \
    "${status}" "${OUTPUT_DIR}" >&2
  exit 1
fi
exit "${status}"
