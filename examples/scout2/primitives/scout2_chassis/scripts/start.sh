#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# scout2/chassis native runtime — runs directly on the host.
# ROS2 humble drivers (/odom, /cmd_vel) publish on localhost;
# this script bootstraps a Python 3.10 env (conda scout2-py) with
# ROS2 sys.path + driver code path, then calls Driver(CMD_INIT).
#
# Lifecycle is owned by rbnx/Soma: Driver(CMD_SHUTDOWN), then this
# package's manifest stop hook, then wrapper PGID TERM/KILL.
set -euo pipefail

BOOT="$(cd "$(dirname "$0")" && pwd)/bootstrap.py"

# Conda executable — common paths on this machine
CONDA=""
for candidate in /opt/conda/bin/conda /opt/miniconda3/bin/conda /home/szh/miniforge3/bin/conda; do
  [ -x "$candidate" ] && CONDA="$candidate" && break
done
if [ -z "${CONDA:-}" ]; then
  echo "[scout2-chassis] error: could not find conda binary."
  exit 1
fi

# Get Python binary from conda env
CONDA_PY="$("$CONDA" run -n scout2-py python3 -c 'import sys; print(sys.executable)' 2>/dev/null)"
if [ -z "${CONDA_PY:-}" ]; then
  echo "[scout2-chassis] error: failed to get Python from conda env 'scout2-py'."
  exit 1
fi

env \
  ROBONIX_ATLAS="${ROBONIX_SIM_ATLAS:-${ROBONIX_ATLAS:-127.0.0.1:50051}}" \
  ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-127.0.0.1}" \
  "$CONDA_PY" "$BOOT"
