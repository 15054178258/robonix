#!/usr/bin/env bash
# Inventory Go2 state/sensor/TF topics. Every message sample is timeout-bounded.

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${1:-${PROJECT_ROOT}/logs/go2-readonly/${STAMP}-topics}"
OUTPUT_DIR="$(realpath -m -- "${OUTPUT_DIR}")"
DETAIL_DIR="${OUTPUT_DIR}/details"
DISCOVERY_TIMEOUT="${GO2_DISCOVERY_TIMEOUT:-15}"
INFO_TIMEOUT="${GO2_INFO_TIMEOUT:-8}"
HZ_TIMEOUT="${GO2_HZ_TIMEOUT:-8}"
ECHO_TIMEOUT="${GO2_ECHO_TIMEOUT:-6}"

case "${OUTPUT_DIR}" in
  "${PROJECT_ROOT}/logs/"*) ;;
  *)
    printf '错误：输出目录必须位于 %s/logs/ 内。\n' "${PROJECT_ROOT}" >&2
    exit 2
    ;;
esac

cat <<'BANNER'
=======================================================================
  READ-ONLY / 只读审计：Go2 ROS 2 话题清单
  只订阅状态、传感器和 TF；不会发送控制数据或调用运动接口。
  所有 ros2 topic echo 均受 timeout 限制。
=======================================================================
BANNER

for value in "${DISCOVERY_TIMEOUT}" "${INFO_TIMEOUT}" "${HZ_TIMEOUT}" "${ECHO_TIMEOUT}"; do
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    printf '错误：超时值必须是正整数秒，收到 %q。\n' "${value}" >&2
    exit 2
  fi
done

for required in ros2 timeout; do
  if ! command -v "${required}" >/dev/null 2>&1; then
    printf '缺失依赖：%s。未执行 ROS 2 审计。\n' "${required}" >&2
    exit 127
  fi
done

mkdir -p -- "${DETAIL_DIR}"
umask 077

ALL_TOPICS="${OUTPUT_DIR}/all-topics.txt"
DISCOVERY_LOG="${OUTPUT_DIR}/topic-discovery.stderr.txt"
SELECTED="${OUTPUT_DIR}/selected-topics.tsv"
SUMMARY="${OUTPUT_DIR}/summary.tsv"
README="${OUTPUT_DIR}/README.txt"

printf 'topic\ttype\n' >"${SELECTED}"
printf 'topic\ttype\trate\tframe_ids\tinfo_status\thz_status\techo_status\n' >"${SUMMARY}"

timeout --signal=INT --kill-after=2s "${DISCOVERY_TIMEOUT}s" \
  ros2 topic list --no-daemon -t >"${ALL_TOPICS}" 2>"${DISCOVERY_LOG}"
discovery_status=$?
if (( discovery_status != 0 )); then
  printf 'ros2 topic list -t 失败或超时（状态 %d）；详情：%s\n' "${discovery_status}" "${DISCOVERY_LOG}" >&2
  exit 1
fi

while IFS= read -r line; do
  [[ -n "${line}" ]] || continue
  topic="${line%% *}"
  type="${line#*[}"
  type="${type%]}"
  [[ "${type}" != "${line}" ]] || type="<type-not-reported>"
  haystack="${topic} ${type}"
  lower="${haystack,,}"

  if [[ "${lower}" =~ sportmodestate|lowstate|(^|[/_])imu($|[/_[:space:]])|sensor_msgs/msg/imu|robot[_/-]?state|camera|image|video|lidar|point[_/-]?cloud|pointcloud2|laserscan|(^|/)scan($|[/[:space:]])|wireless[_/-]?controller|tf2_msgs/msg/tfmessage ]] \
    || [[ "${topic}" == "/tf" || "${topic}" == "/tf_static" ]]; then
    printf '%s\t%s\n' "${topic}" "${type}" >>"${SELECTED}"
  fi
done <"${ALL_TOPICS}"

if (( $(wc -l <"${SELECTED}") <= 1 )); then
  {
    printf 'READ-ONLY Go2 topic inventory\n'
    printf 'No requested state/sensor/TF topics were discovered.\n'
    printf 'Check ROS 2 sourcing, RMW_IMPLEMENTATION, CYCLONEDDS_URI, interface selection and robot power/state.\n'
  } >"${README}"
  printf '未发现目标状态/传感器/TF 话题；原始清单：%s\n' "${ALL_TOPICS}"
  exit 0
fi

while IFS=$'\t' read -r topic type; do
  [[ "${topic}" != "topic" ]] || continue
  safe_name="$(printf '%s' "${topic}" | sed -e 's#^/##' -e 's#[^A-Za-z0-9_.-]#_#g')"
  [[ -n "${safe_name}" ]] || safe_name="root"
  info_file="${DETAIL_DIR}/${safe_name}.info.txt"
  hz_file="${DETAIL_DIR}/${safe_name}.hz.txt"
  sample_file="${DETAIL_DIR}/${safe_name}.sample.txt"
  frames_file="${DETAIL_DIR}/${safe_name}.frames.txt"

  timeout --signal=INT --kill-after=2s "${INFO_TIMEOUT}s" \
    ros2 topic info --no-daemon --verbose "${topic}" >"${info_file}" 2>&1
  info_status=$?

  timeout --signal=INT --kill-after=2s "${HZ_TIMEOUT}s" \
    ros2 topic hz --window 20 "${topic}" >"${hz_file}" 2>&1
  hz_status=$?

  if [[ "${type,,}" != *"tf2_msgs/msg/tfmessage"* ]]; then
    timeout --signal=INT --kill-after=2s "${ECHO_TIMEOUT}s" \
      ros2 topic echo --no-daemon --once --no-arr "${topic}" >"${sample_file}" 2>&1
  elif [[ "${topic}" == "/tf_static" ]]; then
    timeout --signal=INT --kill-after=2s "${ECHO_TIMEOUT}s" \
      ros2 topic echo --no-daemon --once --qos-durability transient_local "${topic}" >"${sample_file}" 2>&1
  else
    timeout --signal=INT --kill-after=2s "${ECHO_TIMEOUT}s" \
      ros2 topic echo --no-daemon --once "${topic}" >"${sample_file}" 2>&1
  fi
  echo_status=$?

  sed -nE 's/^[[:space:]]*(frame_id|child_frame_id):[[:space:]]*(.*)$/\1=\2/p' \
    "${sample_file}" >"${frames_file}"
  if [[ ! -s "${frames_file}" ]]; then
    printf '<not observed in bounded sample>\n' >"${frames_file}"
  fi

  rate="$(awk '/average rate:/ {value=$3 " Hz"} END {print value}' "${hz_file}" 2>/dev/null)"
  [[ -n "${rate}" ]] || rate="<not observed in ${HZ_TIMEOUT}s>"
  frame_ids="$(paste -sd';' "${frames_file}" 2>/dev/null)"
  [[ -n "${frame_ids}" ]] || frame_ids="<not observed>"

  printf '%s\t%s\t%s\t%s\t%d\t%d\t%d\n' \
    "${topic}" "${type}" "${rate}" "${frame_ids}" \
    "${info_status}" "${hz_status}" "${echo_status}" >>"${SUMMARY}"
done <"${SELECTED}"

{
  printf 'READ-ONLY Go2 topic inventory\n'
  printf 'UTC timestamp: %s\n' "${STAMP}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-<unset>}"
  printf 'CYCLONEDDS_URI=%s\n' "${CYCLONEDDS_URI:-<unset>}"
  printf 'all-topics.txt: names and reported types for every discovered topic\n'
  printf 'selected-topics.tsv: requested Go2 state/sensor/TF names and types\n'
  printf 'summary.tsv: bounded frequency and frame_id results\n'
  printf 'details/*.info.txt: endpoint types and QoS from ros2 topic info --verbose\n'
  printf 'details/*.hz.txt: bounded frequency observations\n'
  printf 'details/*.sample.txt: one bounded, array-suppressed message sample (TF arrays retained)\n'
  printf 'A timeout status is expected for continuous ros2 topic hz sampling.\n'
} >"${README}"

printf '只读话题审计已保存：%s\n' "${OUTPUT_DIR}"
