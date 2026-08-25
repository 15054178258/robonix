#!/usr/bin/env python3
"""Materialize one private, allowlisted Scene startup configuration."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile
from typing import Any

import yaml


SCENE_START_KEYS = (
    "web_host",
    "web_port",
    "provider_ids",
    "camera_frame",
    "perception_enabled",
)
D435I_PROVIDER_ID = "go2_d435i"
D435I_OPTICAL_FRAME = "d435i_color_optical_frame"
D435I_PROVIDER_PINS = {
    "rgb": D435I_PROVIDER_ID,
    "depth": D435I_PROVIDER_ID,
    "intrinsics": D435I_PROVIDER_ID,
}


class SceneConfigError(RuntimeError):
    """Raised when a rendered deployment cannot yield a safe Scene config."""


def load_scene_config(
    manifest_path: Path,
    *,
    require_d435i_preview: bool,
) -> dict[str, Any]:
    if not manifest_path.is_absolute() or not manifest_path.is_file():
        raise SceneConfigError("manifest must be an absolute regular file")
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise SceneConfigError(f"cannot read rendered manifest: {error}") from error
    if not isinstance(manifest, dict):
        raise SceneConfigError("rendered manifest top level must be a mapping")
    system = manifest.get("system")
    if not isinstance(system, dict):
        raise SceneConfigError("rendered manifest system must be a mapping")
    scene = system.get("scene")
    if not isinstance(scene, dict):
        raise SceneConfigError("rendered manifest system.scene must be a mapping")

    config = {key: scene[key] for key in SCENE_START_KEYS if key in scene}
    if not config:
        raise SceneConfigError("rendered Scene config contains no startup fields")
    if require_d435i_preview:
        if config.get("provider_ids") != D435I_PROVIDER_PINS:
            raise SceneConfigError("D435i Scene provider pins are not exact")
        if config.get("camera_frame") != D435I_OPTICAL_FRAME:
            raise SceneConfigError("D435i Scene camera frame is not exact")
        if config.get("perception_enabled") is not False:
            raise SceneConfigError("D435i Scene perception gate is not false")
    return config


def write_private_config(output_path: Path, config: dict[str, Any]) -> None:
    if not output_path.is_absolute() or output_path.exists():
        raise SceneConfigError("output must be a new absolute path")
    parent = output_path.parent
    try:
        parent_mode = parent.stat().st_mode & 0o777
    except OSError as error:
        raise SceneConfigError(f"cannot inspect output directory: {error}") from error
    if parent_mode & 0o077:
        raise SceneConfigError("output directory must not be accessible by group/other")

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.",
        suffix=".tmp",
        dir=parent,
        text=True,
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(config, stream, ensure_ascii=False, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output_path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--require-d435i-preview", action="store_true")
    args = parser.parse_args()
    try:
        config = load_scene_config(
            args.manifest,
            require_d435i_preview=args.require_d435i_preview,
        )
        write_private_config(args.output, config)
    except (OSError, SceneConfigError) as error:
        parser.error(str(error))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
