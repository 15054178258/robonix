#!/usr/bin/env python3
"""Render the guarded workstation stage-1 localization/Nav2 manifest.

The corrected full no-motion renderer remains the source of the sensor,
odometry, Mapping, and Navigation route.  This renderer changes only the
pieces required by the independently audited staged chassis profile and then
removes every semantic/voice execution surface.

The result is motion-configured, not motion-authorized.  ``go2_chassis`` still
atomically consumes its private one-time staged permit before it starts either
the adapter or SDK daemon.  Permit paths, reviewed modes, and firmware markers
must therefore come from the owning launcher environment and are deliberately
absent from this manifest.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml

from render_workstation_nomotion_manifest import (
    CANONICAL_ODOM,
    MAPPING_PARAMS_FILE,
    NAV2_BT_THROUGH_POSES_XML_FILE,
    NAV2_BT_XML_FILE,
    NAV2_PARAMS_FILE,
    PRIVATE_CLOUD,
    PRIVATE_IMU,
    PRIVATE_LIDAR_ODOM,
    UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS,
    ManifestError,
    _named,
    render as render_base_nomotion,
    write_manifest,
)


PROFILE = "workstation-staged-nav2-corrected-v1"
TIMESTAMP_DISCIPLINE_PROFILE = "motion"
CLOUD_RELAY_PROFILE = "motion"
STATE_TOPIC = "/robonix/time_corrected/motion/sportmodestate"
CHASSIS_COMMAND_TOPIC = "/go2/staged_nav2/cmd_vel"
NAV2_GUARD_INPUT_TOPIC = "/cmd_vel_guard_input"
PRIVATE_CHASSIS_IMU = "/robonix/staged_nav2/untrusted_chassis_imu"
SAFETY_ACK = "I_UNDERSTAND_GO2_CAN_MOVE"
IPC_SOCKET_ENV = "${GO2_SDK_SOCKET}"
DASHBOARD_PORT = 8092

EXPECTED_SYSTEMS = ("atlas", "executor", "soma")
EXPECTED_PRIMITIVES = ("go2_chassis", "go2_sensors", "robot_description")
EXPECTED_SERVICES = ("mapping", "nav2", "go2_dashboard")
FORBIDDEN_PERMIT_ENV = (
    "GO2_ALLOWED_MODES",
    "GO2_ALLOWED_STATE_MARKERS",
    "GO2_STAGED_NAV2_PERMIT_FILE",
    "GO2_STAGED_NAV2_GOAL_PERMIT_FILE",
    "GO2_STAGED_NAV2_SESSION_ID",
    "GO2_STAGED_NAV2_PAIR_ID",
    "GO2_STAGED_NAV2_GUARD_ACK",
)
VOICE_DASHBOARD_KEYS = (
    "liaison_endpoint",
    "audio_bridge_url",
    "browser_mic_provider",
)


def _exact_named_entries(
    items: Any,
    expected: tuple[str, ...],
    section: str,
) -> bool:
    if not isinstance(items, list):
        return False
    names = tuple(
        entry.get("name") if isinstance(entry, dict) else None
        for entry in items
    )
    return names == expected


def _config(manifest: dict[str, Any], section: str, name: str) -> dict[str, Any]:
    config = _named(manifest.get(section), name, section).get("config")
    if not isinstance(config, dict):
        raise ManifestError(f"{section}.{name} config is missing")
    return config


def render(
    base: dict[str, Any],
    *,
    state_marker: int,
    passive_state_markers: list[int] | tuple[int, ...],
) -> dict[str, Any]:
    """Derive one exact staged profile from the corrected no-motion route.

    The marker arguments are used only to validate the already-reviewed
    corrected no-motion source epoch.  They are removed from the result so the
    staged provider can accept them only from its one-time permit environment.
    """

    rendered = render_base_nomotion(
        base,
        state_marker=state_marker,
        passive_state_markers=passive_state_markers,
    )
    rendered["name"] = "robonix-go2-workstation-staged-nav2-corrected"

    catalog = rendered.get("catalog")
    if not isinstance(catalog, dict):
        raise ManifestError("manifest catalog must be a mapping")
    catalog["description"] = (
        "Guarded stage-1 localization and Nav2 deployment for Unitree Go2."
    )
    tags = catalog.get("tags")
    if isinstance(tags, list):
        catalog["tags"] = [
            tag
            for tag in tags
            if tag not in {"speech", "semantic", "voice"}
        ]

    systems = rendered.get("system")
    if not isinstance(systems, dict):
        raise ManifestError("manifest system must be a mapping")
    rendered["system"] = {
        name: systems[name]
        for name in EXPECTED_SYSTEMS
        if name in systems
    }

    primitives = rendered.get("primitive")
    if not isinstance(primitives, list):
        raise ManifestError("manifest primitive must be a list")
    rendered["primitive"] = [
        entry
        for entry in primitives
        if isinstance(entry, dict)
        and entry.get("name") in EXPECTED_PRIMITIVES
    ]

    services = rendered.get("service")
    if not isinstance(services, list):
        raise ManifestError("manifest service must be a list")
    rendered["service"] = [
        entry
        for entry in services
        if isinstance(entry, dict)
        and entry.get("name") in EXPECTED_SERVICES
    ]
    rendered["skill"] = []

    chassis = _config(rendered, "primitive", "go2_chassis")
    chassis.update(
        {
            "state_topic": STATE_TOPIC,
            "state_fallback_topic": "",
            "twist_in_topic": CHASSIS_COMMAND_TOPIC,
            "odom_source": "external_verified",
            "external_odom_topic": PRIVATE_LIDAR_ODOM,
            "external_odom_timeout_s": 1.0,
            "max_external_odom_yaw_jump_rad": 1.0,
            "odom_topic": CANONICAL_ODOM,
            "publish_odom_tf": True,
            "imu_topic": PRIVATE_CHASSIS_IMU,
            "arm_service": "/go2_chassis/arm",
            # The launcher owns one short, private socket path for the
            # provider, goal dispatcher, and both permit validations.  rbnx
            # resolves this substitution before the chassis provider starts.
            "ipc_socket": IPC_SOCKET_ENV,
            "allow_motion": True,
            "motion_profile": PROFILE,
            "operator_present": True,
            "safety_ack": SAFETY_ACK,
            # The provider replaces this impossible sentinel only from the
            # audited runtime environment bound into the consumed permit.
            "allowed_modes": [255],
            "allow_passive_state_marker_transitions": False,
            "allow_motion_state_marker_transitions": False,
            "max_linear_x_mps": 0.30,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.40,
            "max_linear_accel_mps2": 0.30,
            "max_angular_accel_rps2": 0.80,
            "command_timeout_s": 0.20,
            "state_timeout_s": 1.0,
            "max_source_stamp_age_s": 0.20,
            "max_source_stamp_future_skew_s": 0.05,
            "zero_preamble_s": 0.60,
            "commissioning_max_duration_s": 0.0,
            "commissioning_max_distance_m": 0.0,
        }
    )
    chassis.pop("allowed_state_markers", None)
    for key in UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS:
        chassis.pop(key, None)

    sensors = _config(rendered, "primitive", "go2_sensors")
    sensors.update(
        {
            "source_mode": "local",
            "lidar_input_topic": PRIVATE_CLOUD,
            "imu_input_topic": PRIVATE_IMU,
            # RGB remains published and registered for UI/semantic consumers.
            # A transient missing frame or quality warning must not tear down
            # lidar/IMU/odometry Nav2.  Camera children remain started so the
            # stream appears automatically when the source recovers.
            "camera_required": False,
            "camera_quality_required": False,
        }
    )

    mapping = _config(rendered, "service", "mapping")
    mapping.update(
        {
            "map_mode": "localization",
            "map_id": "${GO2_MAP_ID}",
            "reset_map": False,
            "rtabmap_inputs": ["lidar", "imu", "odom"],
            "params_file": MAPPING_PARAMS_FILE,
        }
    )
    mapping["sensor_providers"] = {
        "lidar3d": "go2_sensors",
        "imu": "go2_sensors",
        "odom": "go2_chassis",
    }

    nav = _config(rendered, "service", "nav2")
    nav.update(
        {
            "params_file": NAV2_PARAMS_FILE,
            "bt_xml_file": NAV2_BT_XML_FILE,
            "bt_through_poses_xml_file": NAV2_BT_THROUGH_POSES_XML_FILE,
            "velocity_output_topic": NAV2_GUARD_INPUT_TOPIC,
            "external_velocity_guard": True,
            "use_composition": True,
        }
    )
    nav.pop("topic_remap", None)
    nav["provider_ids"] = {
        "map": "mapping",
        "odom": "go2_chassis",
        "scan": "go2_sensors",
        "scan_cloud": "go2_sensors",
    }

    dashboard = _config(rendered, "service", "go2_dashboard")
    dashboard.update(
        {
            # The staged launcher deliberately does not source the repository
            # .env.  Materialize the official dashboard port so rbnx never
            # hands the provider a literal ${GO2_DASHBOARD_PORT} string.
            "port": DASHBOARD_PORT,
            "odom_topic": CANONICAL_ODOM,
            "browser_voice_enabled": False,
        }
    )
    for key in VOICE_DASHBOARD_KEYS:
        dashboard.pop(key, None)

    environment = rendered.get("env")
    if not isinstance(environment, dict):
        raise ManifestError("manifest env must be a mapping")
    environment.update(
        {
            "GO2_TIMESTAMP_CORRECTION_PROFILE": PROFILE,
            "GO2_TIMESTAMP_DISCIPLINE_PROFILE": TIMESTAMP_DISCIPLINE_PROFILE,
            "GO2_TIMESTAMP_PRIVATE_LIDAR_ODOM": PRIVATE_LIDAR_ODOM,
            "GO2_CLOUD_RELAY_PROFILE": CLOUD_RELAY_PROFILE,
            "GO2_ALLOW_MOTION": "true",
            "GO2_MOTION_PROFILE": PROFILE,
            "ROBONIX_VELOCITY_OUTPUT_TOPIC": NAV2_GUARD_INPUT_TOPIC,
            "ROBONIX_FORCE_CPU": "1",
            "SCENE_CG_FORCE_CPU": "1",
        }
    )
    environment.pop("SPEECH_BACKEND", None)
    for key in tuple(environment):
        if key in FORBIDDEN_PERMIT_ENV or key.startswith("GO2_STAGED_NAV2_"):
            environment.pop(key, None)

    validate_rendered(rendered)
    return rendered


def validate_rendered(manifest: dict[str, Any]) -> None:
    """Reject every route other than the exact one-goal staged topology."""

    chassis = _config(manifest, "primitive", "go2_chassis")
    sensors = _config(manifest, "primitive", "go2_sensors")
    mapping = _config(manifest, "service", "mapping")
    nav = _config(manifest, "service", "nav2")
    dashboard = _config(manifest, "service", "go2_dashboard")
    environment = manifest.get("env")
    systems = manifest.get("system")

    chassis_exact = {
        "state_topic": STATE_TOPIC,
        "state_fallback_topic": "",
        "twist_in_topic": CHASSIS_COMMAND_TOPIC,
        "odom_source": "external_verified",
        "external_odom_topic": PRIVATE_LIDAR_ODOM,
        "external_odom_timeout_s": 1.0,
        "max_external_odom_yaw_jump_rad": 1.0,
        "odom_topic": CANONICAL_ODOM,
        "publish_odom_tf": True,
        "imu_topic": PRIVATE_CHASSIS_IMU,
        "arm_service": "/go2_chassis/arm",
        "ipc_socket": IPC_SOCKET_ENV,
        "allow_motion": True,
        "motion_profile": PROFILE,
        "operator_present": True,
        "safety_ack": SAFETY_ACK,
        "allowed_modes": [255],
        "allow_passive_state_marker_transitions": False,
        "allow_motion_state_marker_transitions": False,
        "max_linear_x_mps": 0.30,
        "max_linear_y_mps": 0.0,
        "max_angular_z_rps": 0.40,
        "max_linear_accel_mps2": 0.30,
        "max_angular_accel_rps2": 0.80,
        "command_timeout_s": 0.20,
        "state_timeout_s": 1.0,
        "max_source_stamp_age_s": 0.20,
        "max_source_stamp_future_skew_s": 0.05,
        "zero_preamble_s": 0.60,
        "commissioning_max_duration_s": 0.0,
        "commissioning_max_distance_m": 0.0,
    }
    environment_exact = {
        "GO2_TIMESTAMP_CORRECTION_PROFILE": PROFILE,
        "GO2_TIMESTAMP_DISCIPLINE_PROFILE": TIMESTAMP_DISCIPLINE_PROFILE,
        "GO2_TIMESTAMP_PRIVATE_LIDAR_ODOM": PRIVATE_LIDAR_ODOM,
        "GO2_CLOUD_RELAY_PROFILE": CLOUD_RELAY_PROFILE,
        "GO2_ALLOW_MOTION": "true",
        "GO2_MOTION_PROFILE": PROFILE,
        "ROBONIX_VELOCITY_OUTPUT_TOPIC": NAV2_GUARD_INPUT_TOPIC,
        "ROBONIX_FORCE_CPU": "1",
        "SCENE_CG_FORCE_CPU": "1",
    }
    required = {
        "manifest_version": manifest.get("manifestVersion") == 1,
        "private_name": manifest.get("name")
        == "robonix-go2-workstation-staged-nav2-corrected",
        "exact_systems": isinstance(systems, dict)
        and tuple(systems) == EXPECTED_SYSTEMS,
        "exact_primitives": _exact_named_entries(
            manifest.get("primitive"), EXPECTED_PRIMITIVES, "primitive"
        ),
        "exact_services": _exact_named_entries(
            manifest.get("service"), EXPECTED_SERVICES, "service"
        ),
        "no_skills": manifest.get("skill") == [],
        "exact_chassis": all(
            chassis.get(key) == value for key, value in chassis_exact.items()
        ),
        "permit_markers_not_committed": "allowed_state_markers" not in chassis,
        "no_stationary_pose_hold": not any(
            key in chassis
            for key in UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS
        ),
        "corrected_sensors": sensors.get("source_mode") == "local"
        and sensors.get("lidar_input_topic") == PRIVATE_CLOUD
        and sensors.get("imu_input_topic") == PRIVATE_IMU
        and sensors.get("camera_required") is False
        and sensors.get("camera_quality_required") is False,
        "localization_only": mapping.get("map_mode") == "localization"
        and mapping.get("map_id") == "${GO2_MAP_ID}"
        and mapping.get("reset_map") is False,
        "mapping_inputs": mapping.get("rtabmap_inputs")
        == ["lidar", "imu", "odom"],
        "mapping_providers": mapping.get("sensor_providers")
        == {
            "lidar3d": "go2_sensors",
            "imu": "go2_sensors",
            "odom": "go2_chassis",
        },
        "mapping_params": mapping.get("params_file") == MAPPING_PARAMS_FILE,
        "nav_params": nav.get("params_file") == NAV2_PARAMS_FILE
        and nav.get("bt_xml_file") == NAV2_BT_XML_FILE
        and nav.get("bt_through_poses_xml_file")
        == NAV2_BT_THROUGH_POSES_XML_FILE,
        "nav_private_guard_input": nav.get("velocity_output_topic")
        == NAV2_GUARD_INPUT_TOPIC,
        "nav_external_guard": nav.get("external_velocity_guard") is True,
        "nav_not_chassis_output": nav.get("velocity_output_topic")
        != CHASSIS_COMMAND_TOPIC,
        "nav_no_topic_remap": "topic_remap" not in nav,
        "nav_owned_composition": nav.get("use_composition") is True,
        "nav_providers": nav.get("provider_ids")
        == {
            "map": "mapping",
            "odom": "go2_chassis",
            "scan": "go2_sensors",
            "scan_cloud": "go2_sensors",
        },
        "dashboard_canonical_odom": dashboard.get("odom_topic")
        == CANONICAL_ODOM,
        "dashboard_port": dashboard.get("port") == DASHBOARD_PORT,
        "dashboard_voice_disabled": dashboard.get("browser_voice_enabled")
        is False,
        "dashboard_has_no_voice_route": not any(
            key in dashboard for key in VOICE_DASHBOARD_KEYS
        ),
        "environment_mapping": isinstance(environment, dict),
    }
    if isinstance(environment, dict):
        required.update(
            {
                "exact_environment": all(
                    environment.get(key) == value
                    for key, value in environment_exact.items()
                ),
                "permit_not_embedded": not any(
                    key in environment
                    for key in FORBIDDEN_PERMIT_ENV
                )
                and not any(
                    key.startswith("GO2_STAGED_NAV2_")
                    for key in environment
                ),
                "speech_environment_absent": "SPEECH_BACKEND" not in environment,
            }
        )
    failed = sorted(name for name, passed in required.items() if not passed)
    if failed:
        raise ManifestError(
            "unsafe staged-nav2 manifest: " + ",".join(failed)
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--state-marker", required=True, type=int)
    parser.add_argument("--passive-state-markers", required=True)
    args = parser.parse_args()
    if not args.base.is_file() or not args.output.is_absolute():
        parser.error("base must be a file and output must be absolute")
    try:
        marker_tokens = [
            token.strip() for token in args.passive_state_markers.split(",")
        ]
        if any(not token or not token.isdecimal() for token in marker_tokens):
            raise ManifestError(
                "passive-state-markers must be comma-separated decimal integers"
            )
        base = yaml.safe_load(args.base.read_text(encoding="utf-8"))
        rendered = render(
            base,
            state_marker=args.state_marker,
            passive_state_markers=[
                int(token, 10) for token in marker_tokens
            ],
        )
        write_manifest(args.output, rendered)
    except (OSError, yaml.YAMLError, ManifestError) as error:
        parser.error(str(error))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
