#!/usr/bin/env python3
"""List and verify workspace-local Robonix system executables.

The deployment manifest decides which Rust system processes ``rbnx boot``
will spawn.  This helper deliberately ignores Python-managed systems such as
Scene and never searches a user or system installation as a fallback.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import stat
import sys

import yaml


SYSTEM_BINARIES: tuple[tuple[str, str], ...] = (
    ("atlas", "robonix-atlas"),
    ("executor", "robonix-executor"),
    ("pilot", "robonix-pilot"),
    ("liaison", "robonix-liaison"),
    ("soma", "robonix-soma"),
    ("vitals", "robonix-vitals"),
)


class ArtifactError(RuntimeError):
    """Raised for a malformed manifest or an invalid local artifact."""


def _load_required(manifest_path: Path) -> list[str]:
    if not manifest_path.is_absolute():
        raise ArtifactError("manifest path must be absolute")
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ArtifactError(f"cannot read manifest: {error}") from error
    if not isinstance(manifest, dict):
        raise ArtifactError("manifest top level must be a mapping")
    systems = manifest.get("system")
    if not isinstance(systems, dict):
        raise ArtifactError("manifest system must be a mapping")
    binaries = [binary for name, binary in SYSTEM_BINARIES if name in systems]
    if not binaries:
        raise ArtifactError("manifest selects no Rust Robonix system binaries")
    return binaries


def _verify(
    manifest_path: Path,
    bin_dir: Path,
    *,
    require_path: bool,
) -> list[str]:
    if not bin_dir.is_absolute():
        raise ArtifactError("binary directory must be absolute")
    try:
        resolved_dir = bin_dir.resolve(strict=True)
    except OSError as error:
        raise ArtifactError(f"binary directory is unavailable: {error}") from error
    if not resolved_dir.is_dir():
        raise ArtifactError(f"binary directory is not a directory: {resolved_dir}")

    binaries = _load_required(manifest_path)
    for binary in binaries:
        candidate = bin_dir / binary
        try:
            resolved = candidate.resolve(strict=True)
            mode = resolved.stat().st_mode
        except OSError as error:
            raise ArtifactError(f"missing Robonix system binary {candidate}: {error}") from error
        if resolved.parent != resolved_dir:
            raise ArtifactError(
                f"Robonix system binary escapes the workspace artifact directory: {candidate}"
            )
        if not stat.S_ISREG(mode) or not mode & 0o111:
            raise ArtifactError(f"Robonix system binary is not executable: {candidate}")
        if require_path:
            discovered = shutil.which(binary)
            if discovered is None:
                raise ArtifactError(f"Robonix system binary is absent from PATH: {binary}")
            try:
                discovered_path = Path(discovered).resolve(strict=True)
            except OSError as error:
                raise ArtifactError(
                    f"cannot resolve PATH entry for {binary}: {error}"
                ) from error
            if discovered_path != resolved:
                raise ArtifactError(
                    f"PATH resolves {binary} outside the selected artifact directory: "
                    f"{discovered_path}"
                )
    return binaries


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list")
    list_parser.add_argument("--manifest", type=Path, required=True)

    verify_parser = subparsers.add_parser("verify")
    verify_parser.add_argument("--manifest", type=Path, required=True)
    verify_parser.add_argument("--bin-dir", type=Path, required=True)
    verify_parser.add_argument("--require-path", action="store_true")
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        if args.command == "list":
            binaries = _load_required(args.manifest)
        else:
            binaries = _verify(
                args.manifest,
                args.bin_dir,
                require_path=args.require_path,
            )
    except ArtifactError as error:
        print(f"Robonix system artifact error: {error}", file=sys.stderr)
        return 78
    for binary in binaries:
        print(binary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
