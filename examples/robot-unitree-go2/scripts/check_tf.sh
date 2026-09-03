#!/usr/bin/env bash
# Inspect TF topics and expected navigation frame relationships without publishing.

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${1:-${PROJECT_ROOT}/logs/go2-readonly/${STAMP}-tf}"
OUTPUT_DIR="$(realpath -m -- "${OUTPUT_DIR}")"
TOPIC_TIMEOUT="${GO2_TF_TOPIC_TIMEOUT:-8}"
ECHO_TIMEOUT="${GO2_TF_ECHO_TIMEOUT:-6}"
MONITOR_TIMEOUT="${GO2_TF_MONITOR_TIMEOUT:-10}"
MAP_FRAME="${GO2_MAP_FRAME:-map}"
ODOM_FRAME="${GO2_ODOM_FRAME:-odom}"
BASE_FRAME="${GO2_BASE_FRAME:-base_link}"

case "${OUTPUT_DIR}" in
  "${PROJECT_ROOT}/logs/"*) ;;
  *)
    printf '错误：输出目录必须位于 %s/logs/ 内。\n' "${PROJECT_ROOT}" >&2
    exit 2
    ;;
esac

cat <<'BANNER'
=======================================================================
  READ-ONLY / 只读审计：Go2 TF 检查
  仅订阅 /tf 与 /tf_static，并有界检查 map→odom→base_link。
  不发送控制数据；所有 ros2 topic echo 均受 timeout 限制。
=======================================================================
BANNER

for value in "${TOPIC_TIMEOUT}" "${ECHO_TIMEOUT}" "${MONITOR_TIMEOUT}"; do
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    printf '错误：超时值必须是正整数秒，收到 %q。\n' "${value}" >&2
    exit 2
  fi
done

for required in ros2 timeout; do
  if ! command -v "${required}" >/dev/null 2>&1; then
    printf '缺失依赖：%s。未执行 TF 审计。\n' "${required}" >&2
    exit 127
  fi
done

mkdir -p -- "${OUTPUT_DIR}/details"
umask 077

timeout --signal=INT --kill-after=2s "${TOPIC_TIMEOUT}s" \
  ros2 topic list --no-daemon -t >"${OUTPUT_DIR}/all-topics.txt" 2>"${OUTPUT_DIR}/topic-discovery.stderr.txt"
discovery_status=$?
if (( discovery_status != 0 )); then
  printf 'TF 话题发现失败或超时（状态 %d）。\n' "${discovery_status}" >&2
  exit 1
fi

printf 'topic\ttype\tinfo_status\thz_status\techo_status\tframe_ids\n' >"${OUTPUT_DIR}/tf-summary.tsv"

for topic in /tf /tf_static; do
  line="$(awk -v target="${topic}" '$1 == target {print; exit}' "${OUTPUT_DIR}/all-topics.txt")"
  if [[ -z "${line}" ]]; then
    printf '%s\t<not discovered>\t-\t-\t-\t<none>\n' "${topic}" >>"${OUTPUT_DIR}/tf-summary.tsv"
    continue
  fi

  type="${line#*[}"
  type="${type%]}"
  safe_name="${topic#/}"

  timeout --signal=INT --kill-after=2s "${TOPIC_TIMEOUT}s" \
    ros2 topic info --no-daemon --verbose "${topic}" >"${OUTPUT_DIR}/details/${safe_name}.info.txt" 2>&1
  info_status=$?

  timeout --signal=INT --kill-after=2s "${TOPIC_TIMEOUT}s" \
    ros2 topic hz --window 20 "${topic}" >"${OUTPUT_DIR}/details/${safe_name}.hz.txt" 2>&1
  hz_status=$?

  if [[ "${topic}" == "/tf_static" ]]; then
    timeout --signal=INT --kill-after=2s "${ECHO_TIMEOUT}s" \
      ros2 topic echo --no-daemon --once --qos-durability transient_local "${topic}" \
      >"${OUTPUT_DIR}/details/${safe_name}.sample.txt" 2>&1
  else
    timeout --signal=INT --kill-after=2s "${ECHO_TIMEOUT}s" \
      ros2 topic echo --no-daemon --once "${topic}" \
      >"${OUTPUT_DIR}/details/${safe_name}.sample.txt" 2>&1
  fi
  echo_status=$?

  frames="$(sed -nE 's/^[[:space:]]*(frame_id|child_frame_id):[[:space:]]*(.*)$/\1=\2/p' \
    "${OUTPUT_DIR}/details/${safe_name}.sample.txt" | paste -sd';' -)"
  [[ -n "${frames}" ]] || frames="<not observed in bounded sample>"
  printf '%s\t%s\t%d\t%d\t%d\t%s\n' \
    "${topic}" "${type}" "${info_status}" "${hz_status}" "${echo_status}" "${frames}" \
    >>"${OUTPUT_DIR}/tf-summary.tsv"
done

if ros2 pkg prefix tf2_ros >/dev/null 2>&1; then
  timeout --signal=INT --kill-after=2s "${MONITOR_TIMEOUT}s" \
    ros2 run tf2_ros tf2_monitor >"${OUTPUT_DIR}/tf2-monitor.txt" 2>&1
  printf '%d\n' "$?" >"${OUTPUT_DIR}/tf2-monitor.status"

  for pair in "${MAP_FRAME} ${ODOM_FRAME}" "${ODOM_FRAME} ${BASE_FRAME}"; do
    read -r parent child <<<"${pair}"
    timeout --signal=INT --kill-after=2s "${MONITOR_TIMEOUT}s" \
      ros2 run tf2_ros tf2_echo "${parent}" "${child}" \
      >"${OUTPUT_DIR}/tf2-echo-${parent}-to-${child}.txt" 2>&1
    printf '%d\n' "$?" >"${OUTPUT_DIR}/tf2-echo-${parent}-to-${child}.status"
  done
else
  printf 'tf2_ros package is not available; topic evidence was still collected.\n' >"${OUTPUT_DIR}/tf2-monitor.txt"
fi

{
  printf 'READ-ONLY TF audit\n'
  printf 'Expected navigation chain: %s -> %s -> %s\n' "${MAP_FRAME}" "${ODOM_FRAME}" "${BASE_FRAME}"
  printf 'Topic type/QoS: details/*.info.txt\n'
  printf 'Bounded frequency: details/*.hz.txt\n'
  printf 'frame_id/child_frame_id: tf-summary.tsv and details/*.sample.txt\n'
  printf 'Continuous monitor commands normally end by timeout; their timeout status is not itself a TF failure.\n'
} >"${OUTPUT_DIR}/README.txt"

printf '只读 TF 报告已保存：%s\n' "${OUTPUT_DIR}"
