#!/usr/bin/env bash
# Shared, side-effect-free helpers for the corrected no-motion wrapper session.

GO2_NOMOTION_SESSION_FORMAT=go2-workstation-nomotion-session-v1

# Bash-created descriptors are inherited across fork/exec unless the child
# explicitly closes them.  The corrected no-motion wrapper must retain its
# discipline lock for its whole lifetime, but none of its long-lived child
# trees may keep that lock alive after the wrapper exits.  Call this only from
# the child subshell immediately before exec.
go2_nomotion_close_inherited_fd() {
  local fd="${1:-}"
  [[ "$fd" =~ ^[0-9]+$ && "$fd" -ge 3 ]] || return 2
  exec {fd}>&-
}

go2_nomotion_process_start_ticks() {
  local pid="$1" stat rest
  local -a fields
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 2
  IFS= read -r stat < "/proc/$pid/stat" 2>/dev/null || return 1
  rest="${stat#*) }"
  [[ "$rest" != "$stat" ]] || return 2
  read -r -a fields <<< "$rest"
  [[ "${fields[0]:-}" != Z && "${fields[19]:-}" =~ ^[0-9]+$ ]] || return 2
  printf '%s\n' "${fields[19]}"
}

go2_nomotion_private_regular_file() {
  local path="$1" uid="$2"
  [[ -f "$path" && ! -L "$path" ]] || return 1
  [[ "$(stat -Lc '%u:%a:%h' -- "$path" 2>/dev/null)" == "$uid:600:1" ]]
}

go2_nomotion_private_directory() {
  local path="$1" uid="$2"
  [[ -d "$path" && ! -L "$path" ]] || return 1
  [[ "$(stat -Lc '%u:%a' -- "$path" 2>/dev/null)" == "$uid:700" ]]
}

go2_nomotion_atomic_write_metadata() {
  local destination="$1" token="$2" pid="$3" start_ticks="$4" run_dir="$5"
  local temporary
  [[ ! -e "$destination" && ! -L "$destination" ]] || return 1
  temporary="$(mktemp "${destination}.tmp.XXXXXX")" || return 1
  chmod 600 -- "$temporary" || { rm -f -- "$temporary"; return 1; }
  if ! printf '%s\n' \
    "format=$GO2_NOMOTION_SESSION_FORMAT" \
    "token=$token" \
    "wrapper_pid=$pid" \
    "wrapper_start_ticks=$start_ticks" \
    "run_dir=$run_dir" > "$temporary"; then
    rm -f -- "$temporary"
    return 1
  fi
  mv -T -- "$temporary" "$destination"
}

go2_nomotion_read_session() {
  local path="$1" line key value
  unset GO2_SESSION_FORMAT GO2_SESSION_TOKEN GO2_SESSION_PID \
    GO2_SESSION_START_TICKS GO2_SESSION_RUN_DIR
  declare -A seen=()
  while IFS= read -r line || [[ -n "$line" ]]; do
    [[ "$line" == *=* ]] || return 1
    key="${line%%=*}"
    value="${line#*=}"
    [[ -z "${seen[$key]:-}" ]] || return 1
    seen[$key]=1
    case "$key" in
      format) GO2_SESSION_FORMAT="$value" ;;
      token) GO2_SESSION_TOKEN="$value" ;;
      wrapper_pid) GO2_SESSION_PID="$value" ;;
      wrapper_start_ticks) GO2_SESSION_START_TICKS="$value" ;;
      run_dir) GO2_SESSION_RUN_DIR="$value" ;;
      *) return 1 ;;
    esac
  done < "$path"
  [[ "${GO2_SESSION_FORMAT:-}" == "$GO2_NOMOTION_SESSION_FORMAT" ]]
  [[ "${GO2_SESSION_TOKEN:-}" =~ ^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$ ]]
  [[ "${GO2_SESSION_PID:-}" =~ ^[1-9][0-9]*$ ]]
  [[ "${GO2_SESSION_START_TICKS:-}" =~ ^[0-9]+$ ]]
  [[ "${GO2_SESSION_RUN_DIR:-}" == /* ]]
}

go2_nomotion_validate_session_files() {
  local run_root="$1" pointer="$2" uid="$3" base session_file canonical
  go2_nomotion_private_directory "$run_root" "$uid" || return 1
  go2_nomotion_private_regular_file "$pointer" "$uid" || return 1
  go2_nomotion_read_session "$pointer" || return 1
  [[ "$(dirname -- "$GO2_SESSION_RUN_DIR")" == "$run_root" ]] || return 1
  base="$(basename -- "$GO2_SESSION_RUN_DIR")"
  [[ "$base" =~ ^workstation-nomotion-stamp\.[A-Za-z0-9]{6}$ ]] || return 1
  go2_nomotion_private_directory "$GO2_SESSION_RUN_DIR" "$uid" || return 1
  canonical="$(readlink -f -- "$GO2_SESSION_RUN_DIR" 2>/dev/null)" || return 1
  [[ "$canonical" == "$GO2_SESSION_RUN_DIR" ]] || return 1
  session_file="$GO2_SESSION_RUN_DIR/session.meta"
  go2_nomotion_private_regular_file "$session_file" "$uid" || return 1
  cmp -s -- "$pointer" "$session_file" || return 1
  GO2_SESSION_METADATA_FILE="$session_file"
}

go2_nomotion_process_identity_matches() {
  local pid="$1" expected_start="$2" uid="$3" observed
  [[ -d "/proc/$pid" ]] || return 1
  [[ "$(stat -Lc '%u' -- "/proc/$pid" 2>/dev/null)" == "$uid" ]] || return 2
  observed="$(go2_nomotion_process_start_ticks "$pid")" || return 2
  [[ "$observed" == "$expected_start" ]] || return 2
}

go2_nomotion_process_holds_lock() {
  local pid="$1" lock_file="$2" expected candidate
  [[ -f "$lock_file" && ! -L "$lock_file" ]] || return 1
  expected="$(stat -Lc '%d:%i' -- "$lock_file" 2>/dev/null)" || return 1
  for candidate in "/proc/$pid/fd/"*; do
    [[ -e "$candidate" ]] || continue
    [[ "$(stat -Lc '%d:%i' -- "$candidate" 2>/dev/null || true)" == "$expected" ]] && return 0
  done
  return 1
}

go2_nomotion_remove_pointer_if_owned() {
  local pointer="$1" session_file="$2" index_lock="$3" uid="$4"
  exec {session_index_fd}>"$index_lock" || return 1
  chmod 600 -- "$index_lock" || return 1
  flock --exclusive "$session_index_fd" || return 1
  if go2_nomotion_private_regular_file "$pointer" "$uid" \
    && go2_nomotion_private_regular_file "$session_file" "$uid" \
    && cmp -s -- "$pointer" "$session_file"; then
    rm -f -- "$pointer"
  fi
  flock --unlock "$session_index_fd"
  exec {session_index_fd}>&-
}
