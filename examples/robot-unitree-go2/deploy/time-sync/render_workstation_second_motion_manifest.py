#!/usr/bin/env python3
"""Render the private one-shot manifest for the independent 20 cm probe."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys
from typing import Any

import yaml


_BASE_PATH = Path(__file__).with_name(
    "render_workstation_first_motion_manifest.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "_robonix_second_motion_manifest_base", _BASE_PATH
)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError("could not load the private manifest renderer core")
_base = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _base
_SPEC.loader.exec_module(_base)

PROFILE = "workstation-second-motion-corrected-v1"
STATE_TOPIC = _base.STATE_TOPIC
EXTERNAL_ODOM_TOPIC = _base.EXTERNAL_ODOM_TOPIC
COMMAND_TOPIC = "/go2/second_motion/cmd_vel"
CANONICAL_COMMAND_TOPIC = _base.CANONICAL_COMMAND_TOPIC
ODOM_TOPIC = _base.ODOM_TOPIC
SAFETY_ACK = _base.SAFETY_ACK
PRIVATE_NAME = "robonix-go2-private-second-motion-corrected"
ManifestError = _base.ManifestError
_named = _base._named
render_soma = _base.render_soma
_atomic_yaml = _base._atomic_yaml


def render(base: dict[str, Any], package_root: Path) -> dict[str, Any]:
    """Return a minimal second-motion manifest with its exact hard envelope."""

    rendered = _base.render(base, package_root)
    rendered["name"] = PRIVATE_NAME
    rendered["env"]["GO2_MOTION_PROFILE"] = PROFILE
    chassis = _named(rendered.get("primitive"), "go2_chassis", "primitive")
    config = chassis["config"]
    config.update(
        {
            "twist_in_topic": COMMAND_TOPIC,
            "imu_topic": "/go2/second_motion/imu",
            "motion_profile": PROFILE,
            "external_odom_timeout_s": 0.20,
            "max_linear_x_mps": 0.30,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.0,
            "max_linear_accel_mps2": 0.30,
            "max_angular_accel_rps2": 0.10,
            "command_timeout_s": 0.20,
            "control_rate_hz": 20.0,
            "state_timeout_s": 0.20,
            "max_source_stamp_age_s": 0.20,
            "max_source_stamp_future_skew_s": 0.05,
            "commissioning_max_duration_s": 1.5,
            "commissioning_max_distance_m": 0.30,
        }
    )
    validate_rendered(rendered, package_root.resolve())
    return rendered


def validate_rendered(manifest: dict[str, Any], package_root: Path) -> None:
    chassis = _named(manifest.get("primitive"), "go2_chassis", "primitive")
    config = chassis.get("config")
    required = {
        "private_name": manifest.get("name") == PRIVATE_NAME,
        "profile_env": manifest.get("env", {}).get("GO2_MOTION_PROFILE")
        == PROFILE,
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
                "private_command": config.get("twist_in_topic")
                == COMMAND_TOPIC,
                "not_canonical_command": config.get("twist_in_topic")
                != CANONICAL_COMMAND_TOPIC,
                "canonical_odom": config.get("odom_topic") == ODOM_TOPIC,
                "external_verified_odom": config.get("odom_source")
                == "external_verified",
                "private_external_odom": config.get("external_odom_topic")
                == EXTERNAL_ODOM_TOPIC,
                "external_odom_timeout": config.get(
                    "external_odom_timeout_s"
                )
                == 0.20,
                "chassis_tf_authority": config.get("publish_odom_tf") is True,
                "stationary_pose_hold_disabled": config.get(
                    "stationary_pose_hold_enabled"
                )
                is False,
                "motion_enabled": config.get("allow_motion") is True,
                "motion_profile": config.get("motion_profile") == PROFILE,
                "operator_present": config.get("operator_present") is True,
                "safety_ack": config.get("safety_ack") == SAFETY_ACK,
                "sentinel_mode": config.get("allowed_modes") == [255],
                "marker_not_committed": "allowed_state_markers" not in config,
                "vx": config.get("max_linear_x_mps") == 0.30,
                "vy": config.get("max_linear_y_mps") == 0.0,
                "wz": config.get("max_angular_z_rps") == 0.0,
                "linear_accel": config.get("max_linear_accel_mps2") == 0.30,
                "angular_accel": config.get("max_angular_accel_rps2")
                == 0.10,
                "watchdog": config.get("command_timeout_s") == 0.20,
                "control_rate": config.get("control_rate_hz") == 20.0,
                "state_timeout": config.get("state_timeout_s") == 0.20,
                "source_stamp_age": config.get("max_source_stamp_age_s")
                == 0.20,
                "source_future_skew": config.get(
                    "max_source_stamp_future_skew_s"
                )
                == 0.05,
                "duration": config.get("commissioning_max_duration_s")
                == 1.5,
                "distance": config.get("commissioning_max_distance_m")
                == 0.30,
            }
        )
    failed = sorted(name for name, passed in required.items() if not passed)
    if failed:
        raise ManifestError(
            "unsafe private second-motion manifest: " + ",".join(failed)
        )


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
            raise ManifestError(
                "manifest and Soma sidecar must share one directory"
            )
        base = yaml.safe_load(args.base.read_text(encoding="utf-8"))
        base_soma = yaml.safe_load(
            args.base_soma.read_text(encoding="utf-8")
        )
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
