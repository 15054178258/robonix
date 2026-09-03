#!/usr/bin/env bash

# Atomic, process-lifetime placement leases.  The kernel-held flock is the
# authority; the adjacent metadata is audit-only and is atomically replaced
# only after the lock has been acquired.  A crash therefore releases the lock
# without requiring PID guessing or stale-file deletion.

go2_runtime_process_start_ticks() {
  local pid="$1"
  local stat rest
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 1
  IFS= read -r stat < "/proc/${pid}/stat" || return 1
  rest="${stat#*) }"
  [[ "$rest" != "$stat" ]] || return 1
  # starttime is proc(5) field 22, i.e. field 20 after the closing comm ')'.
  local -a fields
  read -r -a fields <<< "$rest"
  [[ "${fields[19]:-}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${fields[19]}"
}

go2_runtime_new_token() {
  local token
  IFS= read -r token < /proc/sys/kernel/random/uuid || return 1
  [[ "$token" =~ ^[0-9a-f-]{36}$ ]] || return 1
  printf '%s\n' "$token"
}

go2_runtime_write_metadata() {
  local destination="$1"
  local profile="$2"
  local token="$3"
  local owner_pid="$4"
  local owner_start_ticks="$5"
  local temporary
  temporary="$(mktemp "${destination}.tmp.XXXXXX")" || return 1
  chmod 0600 "$temporary"
  {
    printf 'format=go2-runtime-lease-v1\n'
    printf 'profile=%s\n' "$profile"
    printf 'token=%s\n' "$token"
    printf 'owner_pid=%s\n' "$owner_pid"
    printf 'owner_start_ticks=%s\n' "$owner_start_ticks"
  } > "$temporary"
  mv -f -- "$temporary" "$destination"
}

go2_runtime_lease_acquire() {
  local runtime_root="$1"
  local profile="$2"
  local lease_lock_path lease_metadata_path lease_owner_start_ticks
  [[ "$profile" =~ ^[a-z0-9-]+$ ]] || {
    echo "invalid runtime lease profile: ${profile}" >&2
    return 2
  }
  command -v flock >/dev/null 2>&1 || {
    echo "flock is required for atomic runtime placement ownership" >&2
    return 2
  }
  mkdir -p -- "$runtime_root"
  chmod 0700 "$runtime_root"
  lease_lock_path="${runtime_root}/runtime-placement.lock"
  lease_metadata_path="${runtime_root}/runtime-placement.lease"

  # The launcher owns this descriptor for its process lifetime.  Managed
  # children that are not lease owners must call
  # go2_runtime_close_parent_only_fds in a subshell before exec so a surviving
  # child cannot extend the launcher's lease after the launcher exits.
  exec {GO2_RUNTIME_LEASE_FD}>"$lease_lock_path"
  if ! flock --exclusive --nonblock "$GO2_RUNTIME_LEASE_FD"; then
    echo "another Go2 runtime placement already holds ${lease_lock_path}" >&2
    return 73
  fi
  GO2_RUNTIME_LEASE_TOKEN="$(go2_runtime_new_token)" || return 2
  lease_owner_start_ticks="$(go2_runtime_process_start_ticks "$$")" || return 2
  export GO2_RUNTIME_LEASE_FD GO2_RUNTIME_LEASE_TOKEN
  go2_runtime_write_metadata \
    "$lease_metadata_path" "$profile" "$GO2_RUNTIME_LEASE_TOKEN" "$$" \
    "$lease_owner_start_ticks"
  echo "Atomic runtime placement lease acquired: ${profile}"
}

go2_runtime_close_parent_only_fds() {
  local runtime_fd="${GO2_RUNTIME_LEASE_FD:-}"
  local semantic_fd="${SEMANTIC_ROUTER_LOCK_FD:-}"
  local fd

  for fd in "$runtime_fd" "$semantic_fd"; do
    [[ -z "$fd" || "$fd" =~ ^[0-9]+$ ]] || {
      echo "refusing to spawn with an invalid parent-only lock descriptor" >&2
      return 2
    }
  done

  if [[ -n "$runtime_fd" ]]; then
    exec {GO2_RUNTIME_LEASE_FD}>&- || return 2
  fi
  if [[ -n "$semantic_fd" ]]; then
    exec {SEMANTIC_ROUTER_LOCK_FD}>&- || return 2
  fi

  unset GO2_RUNTIME_LEASE_FD GO2_RUNTIME_LEASE_TOKEN
  unset SEMANTIC_ROUTER_LOCK_FD
}
