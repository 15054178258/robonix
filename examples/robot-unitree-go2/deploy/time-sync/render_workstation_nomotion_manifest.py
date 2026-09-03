#!/usr/bin/env python3
"""Render the one safe Robonix route for corrected workstation timestamps."""

from __future__ import annotations

import argparse
import copy
import os
from pathlib import Path
import tempfile
from typing import Any

import yaml


PROFILE = "workstation-full-nomotion-corrected-v1"
PRIVATE_SPORT = "/robonix/time_corrected/raw/sportmodestate"
PRIVATE_IMU = "/robonix/time_corrected/raw/utlidar/imu"
PRIVATE_CLOUD = "/robonix/time_corrected/raw/utlidar/cloud"
PRIVATE_LIDAR_ODOM = "/robonix/time_corrected/raw/utlidar/robot_odom"
SAFE_CHASSIS_INPUT = "/robonix/nomotion/chassis_input_disabled"
SAFE_VELOCITY_OUTPUT = "/robonix/nomotion/cmd_vel"
CANONICAL_ODOM = "/odom"
PRIVATE_CHASSIS_IMU = "/robonix/nomotion/untrusted_chassis_imu"
RUNTIME_CONFIG_ROOT = "config"
MAPPING_PARAMS_FILE = f"{RUNTIME_CONFIG_ROOT}/rtabmap_params.yaml"
NAV2_PARAMS_FILE = f"{RUNTIME_CONFIG_ROOT}/nav2_params_go2.yaml"
NAV2_BT_XML_FILE = f"{RUNTIME_CONFIG_ROOT}/navigate.xml"
NAV2_BT_THROUGH_POSES_XML_FILE = (
    f"{RUNTIME_CONFIG_ROOT}/navigate_through_poses.xml"
)
UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS = (
    "stationary_pose_hold_enabled",
    "stationary_hold_dwell_s",
    "stationary_hold_sport_max_linear_mps",
    "stationary_hold_sport_max_yaw_rps",
    "stationary_hold_external_twist_max_linear_mps",
    "stationary_hold_external_twist_max_yaw_rps",
    "stationary_hold_pose_max_linear_rate_mps",
    "stationary_hold_pose_max_yaw_rate_rps",
)


class ManifestError(ValueError):
    pass


def _named(items: Any, name: str, section: str) -> dict[str, Any]:
    if not isinstance(items, list):
        raise ManifestError(f"manifest {section} must be a list")
    matches = [entry for entry in items if isinstance(entry, dict) and entry.get("name") == name]
    if len(matches) != 1:
        raise ManifestError(f"manifest must contain exactly one {section}.{name}")
    return matches[0]


def _validate_passive_state_markers(values: Any) -> list[int]:
    if not isinstance(values, (list, tuple)) or len(values) < 2:
        raise ManifestError(
            "passive state markers must be an explicit list of at least two values"
        )
    markers: list[int] = []
    for value in values:
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 1 <= value <= 0xFFFFFFFF
        ):
            raise ManifestError(
                "passive state markers must be non-zero uint32 integers"
            )
        if value in markers:
            raise ManifestError("passive state markers must not contain duplicates")
        markers.append(value)
    return markers


def render(
    base: dict[str, Any],
    *,
    state_marker: int,
    passive_state_markers: list[int] | tuple[int, ...],
) -> dict[str, Any]:
    if not isinstance(base, dict) or base.get("manifestVersion") != 1:
        raise ManifestError("base manifest is not manifestVersion 1")
    if (
        isinstance(state_marker, bool)
        or not isinstance(state_marker, int)
        or not 0 <= state_marker <= 0xFFFFFFFF
    ):
        raise ManifestError("state marker must be a uint32 integer")
    passive_markers = _validate_passive_state_markers(passive_state_markers)
    if state_marker not in passive_markers:
        raise ManifestError(
            "fresh SportModeState marker is not in the passive mapping allowlist"
        )
    rendered = copy.deepcopy(base)
    rendered["name"] = "robonix-go2-workstation-full-nomotion-corrected"

    chassis = _named(rendered.get("primitive"), "go2_chassis", "primitive")
    chassis_cfg = chassis.get("config")
    if not isinstance(chassis_cfg, dict):
        raise ManifestError("go2_chassis config is missing")
    chassis_cfg.update(
        {
            "state_topic": PRIVATE_SPORT,
            "state_fallback_topic": "",
            "twist_in_topic": SAFE_CHASSIS_INPUT,
            # The affine timestamp layer keeps the raw Unitree/Livox odometry
            # private.  The chassis validates that corrected stream and is the
            # sole process allowed to materialize canonical odom and TF.
            "odom_source": "external_verified",
            "external_odom_topic": PRIVATE_LIDAR_ODOM,
            "odom_topic": CANONICAL_ODOM,
            "publish_odom_tf": True,
            "imu_topic": PRIVATE_CHASSIS_IMU,
            "allow_motion": False,
            "operator_present": False,
            "safety_ack": "",
            "allowed_modes": [255],
            "allowed_state_markers": passive_markers,
            "allow_passive_state_marker_transitions": True,
        }
    )
    # PassiveStationaryPoseHold is intentionally not part of the active
    # deployment yet: the source implementation is incomplete and the last
    # verified adapter binary predates its parameters.  Strip even stale base
    # values so the rendered manifest and the executable remain truthful.
    for key in UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS:
        chassis_cfg.pop(key, None)

    sensors = _named(rendered.get("primitive"), "go2_sensors", "primitive")
    sensor_cfg = sensors.get("config")
    if not isinstance(sensor_cfg, dict):
        raise ManifestError("go2_sensors config is missing")
    sensor_cfg.update(
        {
            "source_mode": "local",
            "lidar_input_topic": PRIVATE_CLOUD,
            "imu_input_topic": PRIVATE_IMU,
        }
    )

    mapping = _named(rendered.get("service"), "mapping", "service")
    mapping_cfg = mapping.get("config")
    if not isinstance(mapping_cfg, dict):
        raise ManifestError("mapping config is missing")
    # Mapping consumes the one chassis-owned canonical odometry stream.  It
    # must not start a competing ICP odometry authority.
    mapping_cfg["rtabmap_inputs"] = ["lidar", "imu", "odom"]
    # rbnx deliberately sets RBNX_INVOCATION_CWD to the directory containing
    # the active manifest.  The launcher materializes audited, private copies
    # of all three referenced files below that runtime directory before rbnx
    # starts, so both host and container providers resolve the same files.
    mapping_cfg["params_file"] = MAPPING_PARAMS_FILE
    providers = mapping_cfg.get("sensor_providers")
    if not isinstance(providers, dict):
        raise ManifestError("mapping sensor_providers is missing")
    providers["odom"] = "go2_chassis"

    nav = _named(rendered.get("service"), "nav2", "service")
    nav_cfg = nav.get("config")
    if not isinstance(nav_cfg, dict):
        raise ManifestError("nav2 config is missing")
    nav_cfg["params_file"] = NAV2_PARAMS_FILE
    nav_cfg["bt_xml_file"] = NAV2_BT_XML_FILE
    nav_cfg["bt_through_poses_xml_file"] = NAV2_BT_THROUGH_POSES_XML_FILE
    nav_cfg["velocity_output_topic"] = SAFE_VELOCITY_OUTPUT
    # This profile is a no-motion startup-load experiment.  Keep composition
    # opt-in here instead of changing the base or future motion deployment.
    nav_cfg["use_composition"] = True
    provider_ids = nav_cfg.get("provider_ids")
    if not isinstance(provider_ids, dict) or provider_ids.get("odom") != "go2_chassis":
        raise ManifestError("Nav2 must retain the independent go2_chassis odom gate")

    dashboard = _named(rendered.get("service"), "go2_dashboard", "service")
    dashboard_cfg = dashboard.get("config")
    if not isinstance(dashboard_cfg, dict):
        raise ManifestError("go2_dashboard config is missing")
    dashboard_cfg["odom_topic"] = CANONICAL_ODOM

    env = rendered.setdefault("env", {})
    if not isinstance(env, dict):
        raise ManifestError("manifest env must be a mapping")
    env["GO2_TIMESTAMP_CORRECTION_PROFILE"] = PROFILE
    env["GO2_TIMESTAMP_PRIVATE_LIDAR_ODOM"] = PRIVATE_LIDAR_ODOM
    env["GO2_ALLOW_MOTION"] = "false"
    # This profile must remain runnable when the host exposes nvidia-smi but
    # Docker has no NVIDIA CDI/runtime integration.  The upstream scene
    # launcher honours ROBONIX_FORCE_CPU by omitting `--gpus all`; the second
    # flag also makes the perception backend selection explicit.
    env["ROBONIX_FORCE_CPU"] = "1"
    env["SCENE_CG_FORCE_CPU"] = "1"

    validate_rendered(rendered)
    return rendered


def validate_rendered(manifest: dict[str, Any]) -> None:
    chassis = _named(manifest.get("primitive"), "go2_chassis", "primitive")["config"]
    sensors = _named(manifest.get("primitive"), "go2_sensors", "primitive")["config"]
    mapping = _named(manifest.get("service"), "mapping", "service")["config"]
    nav = _named(manifest.get("service"), "nav2", "service")["config"]
    dashboard = _named(
        manifest.get("service"), "go2_dashboard", "service"
    )["config"]
    required = {
        "state": chassis.get("state_topic") == PRIVATE_SPORT,
        "no_fallback": chassis.get("state_fallback_topic") == "",
        "no_motion": chassis.get("allow_motion") is False,
        "sentinel_mode": chassis.get("allowed_modes") == [255],
        "passive_marker_transitions": chassis.get(
            "allow_passive_state_marker_transitions"
        )
        is True,
        "safe_chassis_input": chassis.get("twist_in_topic") == SAFE_CHASSIS_INPUT,
        "external_verified_odom": chassis.get("odom_source")
        == "external_verified",
        "private_external_odom": chassis.get("external_odom_topic")
        == PRIVATE_LIDAR_ODOM,
        "canonical_chassis_odom": chassis.get("odom_topic") == CANONICAL_ODOM,
        "chassis_tf_authority": chassis.get("publish_odom_tf") is True,
        "no_unfinished_stationary_pose_hold": not any(
            key in chassis
            for key in UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS
        ),
        "private_chassis_imu": chassis.get("imu_topic") == PRIVATE_CHASSIS_IMU,
        "corrected_cloud": sensors.get("lidar_input_topic") == PRIVATE_CLOUD,
        "corrected_imu": sensors.get("imu_input_topic") == PRIVATE_IMU,
        "mapping_chassis_odom": mapping.get("sensor_providers", {}).get("odom")
        == "go2_chassis",
        "mapping_inputs": mapping.get("rtabmap_inputs")
        == ["lidar", "imu", "odom"],
        "mapping_params": mapping.get("params_file") == MAPPING_PARAMS_FILE,
        "nav_params": nav.get("params_file") == NAV2_PARAMS_FILE,
        "nav_bt": nav.get("bt_xml_file") == NAV2_BT_XML_FILE,
        "nav_through_poses_bt": nav.get("bt_through_poses_xml_file")
        == NAV2_BT_THROUGH_POSES_XML_FILE,
        "safe_nav_output": nav.get("velocity_output_topic") == SAFE_VELOCITY_OUTPUT,
        "owned_nav_composition": nav.get("use_composition") is True,
        "nav_keeps_chassis_gate": nav.get("provider_ids", {}).get("odom") == "go2_chassis",
        "dashboard_uses_canonical_odom": dashboard.get("odom_topic")
        == CANONICAL_ODOM,
        "scene_docker_cpu_only": manifest.get("env", {}).get("ROBONIX_FORCE_CPU")
        == "1",
        "scene_perception_cpu_only": manifest.get("env", {}).get(
            "SCENE_CG_FORCE_CPU"
        )
        == "1",
        "marker_is_structured": isinstance(
            chassis.get("allowed_state_markers"), list
        )
        and len(chassis.get("allowed_state_markers")) >= 2
        and all(
            isinstance(value, int)
            and not isinstance(value, bool)
            and 1 <= value <= 0xFFFFFFFF
            for value in chassis.get("allowed_state_markers")
        )
        and len(chassis.get("allowed_state_markers"))
        == len(set(chassis.get("allowed_state_markers"))),
    }
    failed = sorted(name for name, passed in required.items() if not passed)
    if failed:
        raise ManifestError("unsafe corrected no-motion route: " + ",".join(failed))


def write_manifest(path: Path, manifest: dict[str, Any]) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            yaml.safe_dump(manifest, handle, sort_keys=False, allow_unicode=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--state-marker", required=True, type=int)
    parser.add_argument("--passive-state-markers", required=True)
    args = parser.parse_args()
    if not args.base.is_file() or not args.output.is_absolute():
        parser.error("base must be a file and output must be absolute")
    try:
        base = yaml.safe_load(args.base.read_text(encoding="utf-8"))
        marker_tokens = [
            token.strip() for token in args.passive_state_markers.split(",")
        ]
        if any(not token or not token.isdecimal() for token in marker_tokens):
            raise ManifestError(
                "passive-state-markers must be comma-separated decimal integers"
            )
        rendered = render(
            base,
            state_marker=args.state_marker,
            passive_state_markers=[int(token, 10) for token in marker_tokens],
        )
        write_manifest(args.output, rendered)
    except (OSError, yaml.YAMLError, ManifestError) as error:
        parser.error(str(error))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
