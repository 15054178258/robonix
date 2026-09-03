#!/usr/bin/env bash
# Check requested Go2 state and sensor categories by subscribing only.

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${1:-${PROJECT_ROOT}/logs/go2-readonly/${STAMP}-sensors}"
OUTPUT_DIR="$(realpath -m -- "${OUTPUT_DIR}")"
INVENTORY_DIR="${OUTPUT_DIR}/inventory"
REPORT="${OUTPUT_DIR}/sensor-category-report.txt"

case "${OUTPUT_DIR}" in
  "${PROJECT_ROOT}/logs/"*) ;;
  *)
    printf '错误：输出目录必须位于 %s/logs/ 内。\n' "${PROJECT_ROOT}" >&2
    exit 2
    ;;
esac

cat <<'BANNER'
=======================================================================
  READ-ONLY / 只读审计：Go2 状态与传感器检查
  仅发现并订阅状态、IMU、相机、视频、雷达、点云、扫描、手柄和 TF。
  不发送控制数据；所有消息采样均由下层脚本使用 timeout 限制。
=======================================================================
BANNER

mkdir -p -- "${OUTPUT_DIR}"
umask 077

if ! bash "${SCRIPT_DIR}/list_go2_topics.sh" "${INVENTORY_DIR}"; then
  printf '话题清单生成失败；请查看 %s。\n' "${INVENTORY_DIR}" >&2
  exit 1
fi

SELECTED="${INVENTORY_DIR}/selected-topics.tsv"
if [[ ! -f "${SELECTED}" ]]; then
  printf '缺少预期清单：%s\n' "${SELECTED}" >&2
  exit 1
fi

{
  printf 'READ-ONLY Go2 state/sensor category report\n'
  printf 'UTC timestamp: %s\n' "${STAMP}"
  printf 'Raw type/QoS/frequency/frame_id evidence: %s\n' "${INVENTORY_DIR}"
} >"${REPORT}"

report_category() {
  local label="$1"
  local pattern="$2"
  local matches

  matches="$(tail -n +2 "${SELECTED}" | grep -Ei "${pattern}" || true)"
  {
    printf '\n## %s\n' "${label}"
    if [[ -n "${matches}" ]]; then
      printf 'FOUND\n%s\n' "${matches}"
    else
      printf 'NOT FOUND in this discovery window\n'
    fi
  } >>"${REPORT}"
}

report_category "SportModeState" 'sportmodestate'
report_category "LowState" 'lowstate'
report_category "IMU" '(^|[/_[:space:]])imu([/_[:space:]]|$)|sensor_msgs/msg/imu'
report_category "Robot state" 'robot[_/-]?state|robotstate'
report_category "Camera / image" 'camera|image|sensor_msgs/msg/(compressedimage|image|camerainfo)'
report_category "Video" 'video'
report_category "Lidar" 'lidar'
report_category "PointCloud2" 'point[_/-]?cloud|pointcloud2|sensor_msgs/msg/pointcloud2'
report_category "LaserScan" 'laserscan|sensor_msgs/msg/laserscan|(^|/)scan([[:space:]]|$|/)'
report_category "Wireless controller state" 'wireless[_/-]?controller|wirelesscontroller'
report_category "TF" '(^/tf([[:space:]_]|$))|tf2_msgs/msg/tfmessage'

printf '只读传感器报告已保存：%s\n' "${REPORT}"
