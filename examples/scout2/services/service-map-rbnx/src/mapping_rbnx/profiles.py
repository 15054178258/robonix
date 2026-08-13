"""Deployment policy helpers for Mapping configuration."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

LEGACY_PROFILE_FILE = (
    Path(__file__).resolve().parent / "legacy_profiles.json"
)


def deployment_root() -> Path:
    """Return the directory containing the active robot manifest."""
    raw = os.environ.get("RBNX_INVOCATION_CWD", "").strip()
    return Path(raw).expanduser().resolve() if raw else Path.cwd().resolve()


def validate_scalar_params(raw: dict[object, object], source: str) -> dict[str, object]:
    normalized: dict[str, object] = {}
    for key, value in raw.items():
        if not isinstance(key, str) or not key.strip():
            raise RuntimeError(f"{source} keys must be non-empty strings")
        if isinstance(value, (dict, list, tuple)) or value is None:
            raise RuntimeError(f"{source}[{key!r}] must be a scalar value")
        normalized[key] = value
    return normalized


def load_deployment_params_file(value: object | None) -> dict[str, object]:
    """Load scalar RTAB-Map parameters from a deploy-owned YAML file."""
    raw_path = str(value or "").strip()
    if not raw_path:
        return {}
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        path = deployment_root() / path
    path = path.resolve()
    if not path.is_file():
        raise RuntimeError(f"params_file not found: {path}")
    try:
        import yaml

        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except ModuleNotFoundError as exc:
        raise RuntimeError("params_file requires the PyYAML runtime dependency") from exc
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"cannot load params_file {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise RuntimeError(f"params_file must contain a YAML mapping: {path}")
    return validate_scalar_params(raw, f"params_file {path}")


def legacy_rtabmap_profiles() -> dict[str, dict[str, object]]:
    """Load frozen compatibility data. New profiles are not an extension API."""
    raw = json.loads(LEGACY_PROFILE_FILE.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise RuntimeError(f"invalid legacy profile file: {LEGACY_PROFILE_FILE}")
    return raw


def choose_provider_record(
    records: list[Any], provider_id: str, contract_id: str
) -> Any | None:
    """Select one Atlas record without depending on response ordering."""
    if not records:
        return None
    if not provider_id and len(records) > 1:
        candidates = sorted({str(record.provider_id) for record in records})
        raise RuntimeError(
            f"multiple Atlas providers expose {contract_id}: {candidates}; "
            "set config.sensor_providers for this sensor"
        )
    return records[0]


def resolve_occupancy_sources(
    raw_sources: object | None, available_sources: set[str]
) -> dict[str, object]:
    """Translate explicit 2D-grid policy into RTAB-Map parameters.

    Atlas discovery answers which streams exist. It must not decide which
    streams are stable enough to build a robot's occupancy grid.
    """
    if raw_sources is None:
        return {}
    if not isinstance(raw_sources, (list, tuple)) or not raw_sources:
        raise RuntimeError(
            "occupancy_sources must be a non-empty list containing lidar and/or depth"
        )
    sources = {str(value).strip().lower() for value in raw_sources}
    supported = {"lidar", "depth"}
    unknown = sources - supported
    if unknown:
        raise RuntimeError(
            f"unknown occupancy source(s) {sorted(unknown)}; options: {sorted(supported)}"
        )
    missing = sources - available_sources
    if missing:
        raise RuntimeError(
            f"occupancy source(s) {sorted(missing)} were requested but not resolved from Atlas"
        )
    grid_sensor = {
        frozenset({"lidar"}): 0,
        frozenset({"depth"}): 1,
        frozenset({"lidar", "depth"}): 2,
    }[frozenset(sources)]
    return {
        "Grid/Sensor": grid_sensor,
        "Grid/FromDepth": "depth" in sources,
    }


def select_rtabmap_inputs(
    raw_inputs: object | None, resolved: dict[str, str]
) -> dict[str, str]:
    """Apply an explicit RTAB-Map subscription policy to Atlas results."""
    if raw_inputs is None:
        return dict(resolved)
    if not isinstance(raw_inputs, (list, tuple)) or not raw_inputs:
        raise RuntimeError(
            "rtabmap_inputs must be a non-empty list containing lidar, rgbd, imu and/or odom"
        )
    inputs = {str(value).strip().lower() for value in raw_inputs}
    supported = {"lidar", "rgbd", "imu", "odom"}
    unknown = inputs - supported
    if unknown:
        raise RuntimeError(
            f"unknown RTAB-Map input(s) {sorted(unknown)}; options: {sorted(supported)}"
        )

    selected: dict[str, str] = {}
    if "lidar" in inputs:
        for key in ("scan_topic", "lidar_topic"):
            if resolved.get(key):
                selected[key] = resolved[key]
        if not ({"scan_topic", "lidar_topic"} & selected.keys()):
            raise RuntimeError("RTAB-Map lidar input was requested but not resolved from Atlas")
    if "rgbd" in inputs:
        if not resolved.get("rgb_topic") or not resolved.get("depth_topic"):
            raise RuntimeError("RTAB-Map rgbd input was requested but not resolved from Atlas")
        selected["rgb_topic"] = resolved["rgb_topic"]
        selected["depth_topic"] = resolved["depth_topic"]
    if "odom" in inputs:
        if not resolved.get("odom_topic"):
            raise RuntimeError("RTAB-Map odom input was requested but not resolved from Atlas")
        selected["odom_topic"] = resolved["odom_topic"]
    if "imu" in inputs:
        if not resolved.get("imu_topic"):
            raise RuntimeError("RTAB-Map imu input was requested but not resolved from Atlas")
        selected["imu_topic"] = resolved["imu_topic"]
    return selected


def resolve_rtabmap_overrides(
    raw: object | None,
    legacy_profile: str = "",
    params_file: object | None = None,
) -> dict[str, object]:
    """Merge legacy compatibility, deploy YAML, then inline overrides."""
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise RuntimeError(
            "rtabmap_params must be a mapping of parameter names to scalar values"
        )

    legacy_profiles = legacy_rtabmap_profiles()
    if legacy_profile and legacy_profile not in legacy_profiles:
        raise RuntimeError(
            f"unknown legacy rtabmap_profile {legacy_profile!r}; "
            f"known values: {sorted(legacy_profiles)}"
        )
    normalized: dict[str, object] = dict(legacy_profiles.get(legacy_profile, {}))
    normalized.update(load_deployment_params_file(params_file))
    normalized.update(validate_scalar_params(raw, "rtabmap_params"))
    return normalized
