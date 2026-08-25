#!/usr/bin/env python3
"""Tear down only the boot state created by one exact ``rbnx boot``.

The launcher already owns an exclusive placement lease.  This helper adds two
more fail-closed bindings before it invokes the canonical Robonix teardown:

* the state file must live beside the exact manifest passed to ``rbnx boot``;
* the persisted ``boot_pid`` must match the child PID started by this launcher.

It never discovers or signals processes itself.  Process-group and container
cleanup remains owned by ``rbnx shutdown -f <manifest>`` and its persisted
component records.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
from typing import Any


CONFIG_ERROR = 78


def _positive_pid(value: str) -> int:
    try:
        pid = int(value, 10)
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected boot PID must be an integer") from error
    if pid <= 0:
        raise argparse.ArgumentTypeError("expected boot PID must be positive")
    return pid


def _regular_file(
    path: Path, label: str, *, require_current_owner: bool = False
) -> os.stat_result:
    try:
        metadata = path.lstat()
    except OSError as error:
        raise ValueError(f"cannot inspect {label} {path}: {error}") from error
    if not stat.S_ISREG(metadata.st_mode):
        raise ValueError(f"{label} must be a regular non-symlink file: {path}")
    if require_current_owner and metadata.st_uid != os.geteuid():
        raise ValueError(f"{label} is not owned by the current user: {path}")
    return metadata


def _canonical_file(path: Path, label: str) -> Path:
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise ValueError(f"cannot resolve {label} {path}: {error}") from error
    _regular_file(resolved, label)
    return resolved


def _read_state(state_path: Path) -> dict[str, Any]:
    if state_path.is_symlink():
        raise ValueError(f"boot state must be a regular non-symlink file: {state_path}")
    metadata = _regular_file(
        state_path, "boot state", require_current_owner=True
    )
    if metadata.st_size > 1024 * 1024:
        raise ValueError(f"boot state exceeds the 1 MiB safety bound: {state_path}")
    try:
        value = json.loads(state_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot parse boot state {state_path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"boot state must be a JSON object: {state_path}")
    return value


def shutdown_recorded_boot(manifest: Path, rbnx: Path, expected_boot_pid: int) -> int:
    manifest = _canonical_file(manifest, "manifest")
    rbnx = _canonical_file(rbnx, "rbnx executable")
    if not os.access(rbnx, os.X_OK):
        raise ValueError(f"rbnx executable is not executable: {rbnx}")

    state_path = manifest.parent / "rbnx-boot" / "state.json"
    if not state_path.exists() and not state_path.is_symlink():
        return 0

    state = _read_state(state_path)
    recorded_manifest = state.get("manifest_path")
    recorded_boot_pid = state.get("boot_pid")
    if not isinstance(recorded_manifest, str) or not recorded_manifest:
        raise ValueError("boot state has no non-empty manifest_path")
    try:
        recorded_manifest_path = Path(recorded_manifest).resolve(strict=True)
    except OSError as error:
        raise ValueError(
            f"boot state manifest_path cannot be resolved: {recorded_manifest}"
        ) from error
    if recorded_manifest_path != manifest:
        raise ValueError(
            "boot state belongs to another manifest: "
            f"recorded={recorded_manifest_path} expected={manifest}"
        )
    if isinstance(recorded_boot_pid, bool) or not isinstance(recorded_boot_pid, int):
        raise ValueError("boot state boot_pid must be an integer")
    if recorded_boot_pid != expected_boot_pid:
        raise ValueError(
            "boot state belongs to another boot process: "
            f"recorded={recorded_boot_pid} expected={expected_boot_pid}"
        )

    # `rbnx shutdown --help` documents exactly `shutdown -f <manifest>`; there
    # is no `--no-update-check` option on this subcommand.
    completed = subprocess.run(
        [str(rbnx), "shutdown", "-f", str(manifest)],
        check=False,
    )
    return completed.returncode


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="tear down the persisted state for one exact rbnx boot"
    )
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--rbnx", required=True, type=Path)
    parser.add_argument("--expected-boot-pid", required=True, type=_positive_pid)
    args = parser.parse_args(argv)
    try:
        return shutdown_recorded_boot(
            args.manifest, args.rbnx, args.expected_boot_pid
        )
    except ValueError as error:
        print(f"refusing rbnx cleanup: {error}", file=sys.stderr)
        return CONFIG_ERROR


if __name__ == "__main__":
    raise SystemExit(main())
