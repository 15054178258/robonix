#!/usr/bin/env python3
"""Render the private, one-shot Robonix manifest for the first Go2 probe.

The checked-in deployment manifest is deliberately *not* motion ready.  This
renderer copies only the three Robonix processes needed to initialise the
guarded chassis primitive and replaces every chassis field with the audited
commissioning envelope.  The result belongs in a mode-0700 runtime directory;
it is never a reusable navigation manifest.
"""

from __future__ import annotations

import argparse
import copy
import os
from pathlib import Path
import tempfile
from typing import Any

import yaml


PROFILE = "workstation-first-motion-corrected-v1"
STATE_TOPIC = "/robonix/time_corrected/motion/sportmodestate"
EXTERNAL_ODOM_TOPIC = "/robonix/time_corrected/raw/utlidar/robot_odom"
COMMAND_TOPIC = "/go2/commissioning/cmd_vel"
CANONICAL_COMMAND_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
SAFETY_ACK = "I_UNDERSTAND_GO2_CAN_MOVE"


class ManifestError(ValueError):
    """The source or rendered commissioning manifest is unsafe."""


def _named(items: Any, name: str, section: str) -> dict[str, Any]:
    if not isinstance(items, list):
        raise ManifestError(f"manifest {section} must be a list")
    matches = [
        item for item in items
        if isinstance(item, dict) and item.get("name") == name
    ]
    if len(matches) != 1:
        raise ManifestError(
            f"manifest must contain exactly one {section}.{name}"
        )
    return matches[0]


def render(base: dict[str, Any], package_root: Path) -> dict[str, Any]:
    """Return a minimal first-motion manifest with an exact fixed envelope."""

    if not isinstance(base, dict) or base.get("manifestVersion") != 1:
        raise ManifestError("base manifest is not manifestVersion 1")
    root = package_root.resolve()
    chassis_path = root / "packages" / "go2_chassis"
    if not chassis_path.is_dir():
        raise ManifestError("Go2 chassis package is missing")

    source_system = base.get("system")
    if not isinstance(source_system, dict):
        raise ManifestError("base manifest system must be a mapping")
    missing_system = sorted(
        name for name in ("atlas", "executor", "soma")
        if name not in source_system
    )
    if missing_system:
        raise ManifestError(
            "base manifest is missing required system entries: "
            + ",".join(missing_system)
        )

    rendered: dict[str, Any] = {
        "manifestVersion": 1,
        "name": "robonix-go2-private-first-motion-corrected",
        "env": {
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "GO2_ALLOW_MOTION": "true",
            "GO2_MOTION_PROFILE": PROFILE,
        },
        "system": {
            name: copy.deepcopy(source_system[name])
            for name in ("atlas", "executor", "soma")
        },
        "primitive": [],
        "service": [],
        "skill": [],
    }

    source_chassis = copy.deepcopy(
        _named(base.get("primitive"), "go2_chassis", "primitive")
    )
    source_chassis["path"] = str(chassis_path)
    source_chassis.pop("url", None)
    source_chassis.pop("branch", None)
    config = source_chassis.get("config")
    if not isinstance(config, dict):
        raise ManifestError("go2_chassis config is missing")
    config.update(
        {
            "state_topic": STATE_TOPIC,
            "state_fallback_topic": "",
            "twist_in_topic": COMMAND_TOPIC,
            "odom_source": "external_verified",
            "external_odom_topic": EXTERNAL_ODOM_TOPIC,
            "odom_topic": ODOM_TOPIC,
            "publish_odom_tf": True,
            # Stationary pose hold is a no-motion validation aid and must
            # never mask real displacement during commissioning.
            "stationary_pose_hold_enabled": False,
            "stationary_hold_dwell_s": 2.0,
            "stationary_hold_sport_max_linear_mps": 0.03,
            "stationary_hold_sport_max_yaw_rps": 0.03,
            "stationary_hold_external_twist_max_linear_mps": 0.03,
            "stationary_hold_external_twist_max_yaw_rps": 0.03,
            "stationary_hold_pose_max_linear_rate_mps": 0.005,
            "stationary_hold_pose_max_yaw_rate_rps": 0.01,
            "imu_topic": "/go2/commissioning/imu",
            "network_interface": "${GO2_NETWORK_INTERFACE}",
            "allow_motion": True,
            "motion_profile": PROFILE,
            "operator_present": True,
            "safety_ack": SAFETY_ACK,
            # The provider accepts audited modes and opaque state markers only
            # from the process environment bound into the one-time permit.
            "allowed_modes": [255],
            "max_linear_x_mps": 0.05,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.0,
            "max_linear_accel_mps2": 0.05,
            "max_angular_accel_rps2": 0.10,
            "command_timeout_s": 0.20,
            "state_timeout_s": 0.20,
            # Motion requires a strictly fresher source stamp than the
            # adapter's 200 ms receipt-liveness timeout.  This mirrors the
            # dedicated motion timestamp profile and preserves a second
            # fail-closed freshness check at the chassis boundary.
            "max_source_stamp_age_s": 0.20,
            "max_source_stamp_future_skew_s": 0.05,
            "zero_preamble_s": 0.60,
            "commissioning_max_duration_s": 2.0,
            "commissioning_max_distance_m": 0.10,
        }
    )
    # A non-empty environment marker is intentionally incompatible with a
    # manifest marker list.  Omitting the key prevents a stale file from
    # authorising a different firmware state.
    config.pop("allowed_state_markers", None)
    rendered["primitive"] = [source_chassis]
    validate_rendered(rendered, root)
    return rendered


def render_soma(base: dict[str, Any], package_root: Path) -> dict[str, Any]:
    """Pin the sidecar robot model to this checkout for private rbnx boot."""

    if not isinstance(base, dict):
        raise ManifestError("base Soma document must be a mapping")
    rendered = copy.deepcopy(base)
    urdf = rendered.get("urdf")
    if not isinstance(urdf, dict):
        raise ManifestError("base Soma document has no urdf mapping")
    urdf_path = (
        package_root.resolve()
        / "packages"
        / "go2_description"
        / "urdf"
        / "go2_robonix.urdf"
    )
    if not urdf_path.is_file():
        raise ManifestError("Go2 URDF is missing")
    urdf["path"] = str(urdf_path)
    return rendered


def validate_rendered(manifest: dict[str, Any], package_root: Path) -> None:
    chassis = _named(manifest.get("primitive"), "go2_chassis", "primitive")
    config = chassis.get("config")
    required = {
        "private_name": manifest.get("name")
        == "robonix-go2-private-first-motion-corrected",
        "only_three_systems": set(manifest.get("system", {}))
        == {"atlas", "executor", "soma"},
        "no_services": manifest.get("service") == [],
        "no_skills": manifest.get("skill") == [],
        "absolute_chassis": chassis.get("path")
        == str(package_root / "packages" / "go2_chassis"),
        "config_mapping": isinstance(config, dict),
    }
    if isinstance(config, dict):
        required.update(
            {
                "corrected_state": config.get("state_topic") == STATE_TOPIC,
                "no_state_fallback": config.get("state_fallback_topic") == "",
                "private_command": config.get("twist_in_topic") == COMMAND_TOPIC,
                "not_canonical_command": config.get("twist_in_topic")
                != CANONICAL_COMMAND_TOPIC,
                "canonical_odom": config.get("odom_topic") == ODOM_TOPIC,
                "external_verified_odom": config.get("odom_source")
                == "external_verified",
                "private_external_odom": config.get("external_odom_topic")
                == EXTERNAL_ODOM_TOPIC,
                "chassis_tf_authority": config.get("publish_odom_tf") is True,
                "stationary_pose_hold_disabled": config.get(
                    "stationary_pose_hold_enabled"
                )
                is False,
                "stationary_hold_dwell": config.get(
                    "stationary_hold_dwell_s"
                )
                == 2.0,
                "stationary_hold_sport_linear": config.get(
                    "stationary_hold_sport_max_linear_mps"
                )
                == 0.03,
                "stationary_hold_sport_yaw": config.get(
                    "stationary_hold_sport_max_yaw_rps"
                )
                == 0.03,
                "stationary_hold_external_linear": config.get(
                    "stationary_hold_external_twist_max_linear_mps"
                )
                == 0.03,
                "stationary_hold_external_yaw": config.get(
                    "stationary_hold_external_twist_max_yaw_rps"
                )
                == 0.03,
                "stationary_hold_pose_linear": config.get(
                    "stationary_hold_pose_max_linear_rate_mps"
                )
                == 0.005,
                "stationary_hold_pose_yaw": config.get(
                    "stationary_hold_pose_max_yaw_rate_rps"
                )
                == 0.01,
                "motion_enabled": config.get("allow_motion") is True,
                "motion_profile": config.get("motion_profile") == PROFILE,
                "operator_present": config.get("operator_present") is True,
                "safety_ack": config.get("safety_ack") == SAFETY_ACK,
                "sentinel_mode": config.get("allowed_modes") == [255],
                "marker_not_committed": "allowed_state_markers" not in config,
                "vx": config.get("max_linear_x_mps") == 0.05,
                "vy": config.get("max_linear_y_mps") == 0.0,
                "wz": config.get("max_angular_z_rps") == 0.0,
                "watchdog": config.get("command_timeout_s") == 0.20,
                "source_stamp_age": (
                    config.get("max_source_stamp_age_s") == 0.20
                ),
                "duration": config.get("commissioning_max_duration_s") == 2.0,
                "distance": config.get("commissioning_max_distance_m") == 0.10,
            }
        )
    failed = sorted(name for name, passed in required.items() if not passed)
    if failed:
        raise ManifestError(
            "unsafe private first-motion manifest: " + ",".join(failed)
        )


def _atomic_yaml(path: Path, payload: dict[str, Any]) -> None:
    if not path.is_absolute():
        raise ManifestError("output paths must be absolute")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--base-soma", required=True, type=Path)
    parser.add_argument("--package-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--soma-output", required=True, type=Path)
    args = parser.parse_args()
    try:
        if args.output.parent.resolve() != args.soma_output.parent.resolve():
            raise ManifestError("manifest and Soma sidecar must share one directory")
        base = yaml.safe_load(args.base.read_text(encoding="utf-8"))
        base_soma = yaml.safe_load(args.base_soma.read_text(encoding="utf-8"))
        manifest = render(base, args.package_root)
        soma = render_soma(base_soma, args.package_root)
        _atomic_yaml(args.output, manifest)
        _atomic_yaml(args.soma_output, soma)
    except (OSError, yaml.YAMLError, ManifestError) as error:
        parser.error(str(error))
    print(args.output)
    print(args.soma_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
