#!/usr/bin/env python3
"""Validate the deployment-owned Robonix config without home-directory fallback."""

from __future__ import annotations

from pathlib import Path
import sys

import yaml


def _fail(message: str) -> int:
    print(f"invalid workspace-local Robonix config: {message}", file=sys.stderr)
    return 78


def main() -> int:
    if len(sys.argv) != 3:
        return _fail("expected CONFIG_PATH EXPECTED_SOURCE_PATH")
    config_path = Path(sys.argv[1])
    expected_source = Path(sys.argv[2])
    if not config_path.is_absolute() or not expected_source.is_absolute():
        return _fail("paths must be absolute")
    try:
        config_resolved = config_path.resolve(strict=True)
        expected_resolved = expected_source.resolve(strict=True)
    except OSError as error:
        return _fail(str(error))
    if not config_resolved.is_file():
        return _fail(f"config is not a regular file: {config_resolved}")
    if not expected_resolved.is_dir():
        return _fail(f"Robonix source is not a directory: {expected_resolved}")
    try:
        payload = yaml.safe_load(config_resolved.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        return _fail(str(error))
    if not isinstance(payload, dict):
        return _fail("top level must be a mapping")
    configured = payload.get("robonix_source_path")
    if not isinstance(configured, str) or not configured.strip():
        return _fail("robonix_source_path must be a non-empty string")
    configured_path = Path(configured)
    if not configured_path.is_absolute():
        return _fail("robonix_source_path must be absolute")
    try:
        configured_resolved = configured_path.resolve(strict=True)
    except OSError as error:
        return _fail(str(error))
    if configured_resolved != expected_resolved:
        return _fail(
            "robonix_source_path does not point to the audited workspace source"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
