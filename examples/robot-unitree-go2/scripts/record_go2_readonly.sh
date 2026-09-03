#!/usr/bin/env bash
# Record an explicit allowlist of Go2 state/sensor/TF topics for a bounded duration.

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DURATION="${GO2_RECORD_SECONDS:-30}"
LABEL="readonly"

usage() {
  printf 'Usage: bash %s [--duration SECONDS] [--label SAFE_LABEL]\n' "${0##*/}"
  printf 'Records only the discovered state/sensor/TF allowlist under logs/go2-readonly/.\n'
}

while (( $# > 0 )); do
  case "$1" in
    --duration)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      DURATION="$2"
      shift 2
      ;;
    --label)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      LABEL="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "${DURATION}" =~ ^[1-9][0-9]*$ ]] || (( DURATION > 600 )); then
  printf '错误：--duration 必须是 1 到 600 秒的整数。\n' >&2
  exit 2
fi
if [[ ! "${LABEL}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  printf '错误：--label 只能包含字母、数字、点、下划线和连字符。\n' >&2
  exit 2
fi

cat <<BANNER
=======================================================================
  READ-ONLY / 只读录制：Go2 状态与传感器
  将仅订阅正向白名单中的状态、传感器和 TF，最长 ${DURATION} 秒。
  不使用全话题录制，不发送控制数据，不调用运动接口。
=======================================================================
BANNER

for required in ros2 timeout; do
  if ! command -v "${required}" >/dev/null 2>&1; then
    printf '缺失依赖：%s。未开始录制。\n' "${required}" >&2
    exit 127
  fi
done

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SESSION_DIR="${PROJECT_ROOT}/logs/go2-readonly/${STAMP}-${LABEL}-recording"
INVENTORY_DIR="${SESSION_DIR}/inventory"
BAG_DIR="${SESSION_DIR}/bag"
mkdir -p -- "${SESSION_DIR}"
umask 077

if ! bash "${SCRIPT_DIR}/list_go2_topics.sh" "${INVENTORY_DIR}"; then
  printf '预录制话题审计失败；没有启动 rosbag。\n' >&2
  exit 1
fi

SELECTED="${INVENTORY_DIR}/selected-topics.tsv"
mapfile -t candidate_topics < <(tail -n +2 "${SELECTED}" | cut -f1)
topics=()
for topic in "${candidate_topics[@]}"; do
  [[ -n "${topic}" ]] || continue
  lower="${topic,,}"
  if [[ "${lower}" =~ (^|/)(request|cmd|command|control|setpoint)($|/) ]]; then
    continue
  fi
  topics+=("${topic}")
done

if (( ${#topics[@]} == 0 )); then
  printf '未发现可安全录制的状态/传感器/TF 话题；没有启动 rosbag。\n' >&2
  exit 1
fi

{
  printf 'READ-ONLY rosbag topic allowlist\n'
  printf 'Duration limit: %s seconds\n' "${DURATION}"
  printf '%s\n' "${topics[@]}"
} >"${SESSION_DIR}/recorded-topics.txt"

timeout --signal=INT --kill-after=15s "${DURATION}s" \
  ros2 bag record --output "${BAG_DIR}" "${topics[@]}" \
  >"${SESSION_DIR}/rosbag.stdout.txt" 2>"${SESSION_DIR}/rosbag.stderr.txt"
record_status=$?
printf '%d\n' "${record_status}" >"${SESSION_DIR}/rosbag.status"

if (( record_status == 124 )); then
  printf '已达到 %s 秒只读录制上限并停止：%s\n' "${DURATION}" "${SESSION_DIR}"
elif (( record_status == 0 )); then
  printf '只读录制已正常结束：%s\n' "${SESSION_DIR}"
else
  printf '只读录制异常结束（状态 %d）；请检查：%s\n' "${record_status}" "${SESSION_DIR}" >&2
  exit 1
fi
