#!/usr/bin/env bash
# Start the SDK-only process for IPC diagnostics without initializing SDK2.
# Motion-capable startup is intentionally available only through the provider.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
BINARY="${GO2_SDK_DAEMON_BIN:-$PKG/rbnx-build/sdk/install/bin/go2_sport_daemon}"
SOCKET="${GO2_SDK_SOCKET:-/tmp/robonix-go2-disabled.sock}"
ALLOW_MOTION_RAW="${GO2_ALLOW_MOTION:-false}"

case "${ALLOW_MOTION_RAW,,}" in
  1|true|yes|on) ALLOW_MOTION=1 ;;
  0|false|no|off|"") ALLOW_MOTION=0 ;;
  *)
    echo "GO2_ALLOW_MOTION must be an explicit boolean." >&2
    exit 3
    ;;
esac

if [ "$ALLOW_MOTION" = "1" ]; then
  echo "Refusing direct motion-enabled SDK daemon startup." >&2
  echo "Use Robonix Driver(CMD_INIT) so all gates and audited modes are validated." >&2
  exit 3
fi

if [ ! -x "$BINARY" ]; then
  echo "go2_sport_daemon is not built: $BINARY" >&2
  exit 2
fi

echo "============================================================"
echo "GO2 SDK DAEMON: NO-MOTION IPC DIAGNOSTIC MODE"
echo "SDK2 will not be initialized; arm and move requests are rejected."
echo "============================================================"

arguments=(--socket "$SOCKET" --watchdog-ms "${GO2_SDK_WATCHDOG_MS:-300}")
exec "$BINARY" "${arguments[@]}"
