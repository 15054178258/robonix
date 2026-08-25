#!/usr/bin/env python3
"""Render the persistent full Robonix voice/Nav2 profile for the Go2.

This is the minimal migration from the verified corrected no-motion route:
the official Navigation provider owns its final velocity guard, while the Go2
adapter and SDK daemon retain the independent runtime gate, clamps, fresh-state
checks and command watchdog.  Historical one-goal permit/evidence tooling is
left untouched and is not part of this normal long-running profile.
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path
from typing import Any

import yaml

from render_workstation_nomotion_d435i_preview_manifest import (
    D435I_CAMERA_INFO_TOPIC,
    D435I_DEPTH_TOPIC,
    D435I_PROVIDER_ID,
    D435I_RGB_TOPIC,
    ManifestError,
    _named,
    render as render_d435i_nomotion,
    write_manifest,
)
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
)


PROFILE = "workstation-staged-nav2-corrected-v1"
MANIFEST_NAME = "robonix-go2-workstation-persistent-voice-nav2"
STATE_TOPIC = "/robonix/time_corrected/motion/sportmodestate"
CHASSIS_COMMAND_TOPIC = "/go2/staged_nav2/cmd_vel"
PRIVATE_CHASSIS_IMU = "/robonix/staged_nav2/untrusted_chassis_imu"
SAFETY_ACK = "I_UNDERSTAND_GO2_CAN_MOVE"
IPC_SOCKET_ENV = "${GO2_SDK_SOCKET}"
MAPS_DIR = "${ROBONIX_DEPLOY_DIR}/rbnx-build/data/maps"
FORBIDDEN_COMMITTED_RUNTIME_ENV = {
    "GO2_ALLOWED_MODES",
    "GO2_ALLOWED_STATE_MARKERS",
    "GO2_STAGED_NAV2_RUNTIME_ACK",
}
LOCALIZATION_RTABMAP_OVERRIDES = {
    "RGBD/NeighborLinkRefining": False,
    "RGBD/ProximityBySpace": True,
    "RGBD/ProximityMaxGraphDepth": 0,
    "RGBD/ProximityPathMaxNeighbors": 1,
    "RGBD/ProximityOdomGuess": True,
    # Start with conservative node-to-node matching. A global assembled scan
    # map would widen the matching context in the repetitive corridor and is a
    # separate localization experiment, not part of this first correction.
    "RGBD/ProximityGlobalScanMap": False,
}
TIGHT_LOCALIZATION_ICP_PARAMS = {
    "Icp/CorrespondenceRatio": 0.20,
    "Icp/MaxCorrespondenceDistance": 0.15,
    "Icp/MaxTranslation": 0.10,
    "Icp/MaxRotation": 0.10,
}


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
    rendered = render_d435i_nomotion(
        base,
        state_marker=state_marker,
        passive_state_markers=passive_state_markers,
    )
    rendered["name"] = MANIFEST_NAME

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
            "ipc_socket": IPC_SOCKET_ENV,
            "allow_motion": True,
            "motion_profile": PROFILE,
            # Keep the operator-selected stable gait across normal Robonix
            # navigation runs. Fault/watchdog/disconnect stops never issue the
            # follow-up mode command.
            "preserve_classic_walk": True,
            "operator_present": True,
            "safety_ack": SAFETY_ACK,
            "allowed_modes": [255],
            "allow_passive_state_marker_transitions": False,
            "allow_motion_state_marker_transitions": True,
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
            # Zero disables the historical one-goal commissioning duration
            # and distance cutoffs; watchdogs and explicit disarm stay active.
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
            "camera_required": False,
            "camera_quality_required": False,
        }
    )

    base_mapping = _config(base, "service", "mapping")
    base_rtabmap_params = base_mapping.get("rtabmap_params")
    if not isinstance(base_rtabmap_params, dict):
        raise ManifestError("base mapping rtabmap_params is missing")
    localization_rtabmap_params = copy.deepcopy(base_rtabmap_params)
    localization_rtabmap_params.update(LOCALIZATION_RTABMAP_OVERRIDES)

    mapping = _config(rendered, "service", "mapping")
    mapping.update(
        {
            "map_mode": "localization",
            "map_id": "${GO2_MAP_ID}",
            "reset_map": False,
            # The persistent localization profile uses the already conditioned
            # dense scan for conservative spatial matching against saved map
            # nodes. Keep this isolated from the shared mapping profile, whose
            # neighbor-refinement policy remains unchanged.
            "dense_scan_refine_neighbors": False,
            "rtabmap_params": localization_rtabmap_params,
            "rtabmap_inputs": ["lidar", "imu", "odom"],
            "params_file": MAPPING_PARAMS_FILE,
            "sensor_providers": {
                "lidar3d": "go2_sensors",
                "imu": "go2_sensors",
                "odom": "go2_chassis",
            },
        }
    )

    nav = _config(rendered, "service", "nav2")
    nav.update(
        {
            "params_file": NAV2_PARAMS_FILE,
            "bt_xml_file": NAV2_BT_XML_FILE,
            "bt_through_poses_xml_file": NAV2_BT_THROUGH_POSES_XML_FILE,
            # The provider-owned velocity_guard is the final Nav2 producer;
            # the chassis adapter is the only subscriber allowed downstream.
            "velocity_output_topic": CHASSIS_COMMAND_TOPIC,
            "external_velocity_guard": False,
            "use_composition": True,
            "provider_ids": {
                "map": "mapping",
                "odom": "go2_chassis",
                "scan": "go2_sensors",
                "scan_cloud": "go2_sensors",
            },
        }
    )
    nav.pop("topic_remap", None)

    dashboard = _config(rendered, "service", "go2_dashboard")
    dashboard.update(
        {
            "odom_topic": CANONICAL_ODOM,
            "d435i_color_topic": D435I_RGB_TOPIC,
            "d435i_depth_topic": D435I_DEPTH_TOPIC,
            "d435i_camera_info_topic": D435I_CAMERA_INFO_TOPIC,
            "initial_pose_topic": "/initialpose",
            "map_lifecycle_topic": "/robonix/map/lifecycle",
            "initial_pose_maps_dir": MAPS_DIR,
            # Restoring an old operator estimate without first returning the
            # robot to its marked physical start would be unsafe localization.
            "initial_pose_auto_restore": False,
            "browser_voice_enabled": True,
        }
    )

    environment = rendered.get("env")
    if not isinstance(environment, dict):
        raise ManifestError("manifest env must be a mapping")
    environment.update(
        {
            "GO2_TIMESTAMP_CORRECTION_PROFILE": PROFILE,
            "GO2_TIMESTAMP_DISCIPLINE_PROFILE": "motion",
            "GO2_TIMESTAMP_PRIVATE_LIDAR_ODOM": PRIVATE_LIDAR_ODOM,
            "GO2_CLOUD_RELAY_PROFILE": "motion",
            "GO2_ALLOW_MOTION": "true",
            "GO2_MOTION_PROFILE": PROFILE,
            "ROBONIX_VELOCITY_OUTPUT_TOPIC": CHASSIS_COMMAND_TOPIC,
            "ROBONIX_FORCE_CPU": "1",
            "SCENE_CG_FORCE_CPU": "1",
        }
    )
    for key in FORBIDDEN_COMMITTED_RUNTIME_ENV:
        environment.pop(key, None)

    validate_rendered(rendered)
    return rendered


def validate_rendered(manifest: dict[str, Any]) -> None:
    chassis = _config(manifest, "primitive", "go2_chassis")
    sensors = _config(manifest, "primitive", "go2_sensors")
    mapping = _config(manifest, "service", "mapping")
    nav = _config(manifest, "service", "nav2")
    dashboard = _config(manifest, "service", "go2_dashboard")
    names = {
        section: {
            entry.get("name")
            for entry in manifest.get(section, [])
            if isinstance(entry, dict)
        }
        for section in ("primitive", "service", "skill")
    }
    environment = manifest.get("env", {})
    required = {
        "name": manifest.get("name") == MANIFEST_NAME,
        "full_systems": all(
            name in manifest.get("system", {})
            for name in ("atlas", "executor", "pilot", "liaison", "soma", "scene")
        ),
        "speech_and_semantic": "speech" in names["service"]
        and "semantic_navigation" in names["skill"],
        "dual_camera_providers": "go2_sensors" in names["primitive"]
        and D435I_PROVIDER_ID in names["primitive"],
        "chassis_runtime_gate": chassis.get("allow_motion") is True
        and chassis.get("motion_profile") == PROFILE
        and chassis.get("preserve_classic_walk") is True
        and chassis.get("operator_present") is True
        and chassis.get("safety_ack") == SAFETY_ACK
        and chassis.get("allowed_modes") == [255]
        and chassis.get("allow_passive_state_marker_transitions") is False
        and chassis.get("allow_motion_state_marker_transitions") is True
        and "allowed_state_markers" not in chassis,
        "chassis_watchdogs": chassis.get("command_timeout_s") == 0.20
        and chassis.get("state_timeout_s") == 1.0
        and chassis.get("max_source_stamp_age_s") == 0.20,
        "single_velocity_route": chassis.get("twist_in_topic")
        == CHASSIS_COMMAND_TOPIC
        and nav.get("velocity_output_topic") == CHASSIS_COMMAND_TOPIC
        and nav.get("external_velocity_guard") is False,
        "provider_owned_final_guard": nav.get("use_composition") is True,
        "camera_not_motion_gate": sensors.get("camera_required") is False
        and sensors.get("camera_quality_required") is False,
        "localization_map": mapping.get("map_mode") == "localization"
        and mapping.get("map_id") == "${GO2_MAP_ID}"
        and mapping.get("reset_map") is False,
        "lidar_only_mapping": mapping.get("rtabmap_inputs")
        == ["lidar", "imu", "odom"]
        and set(mapping.get("sensor_providers", {}))
        == {"lidar3d", "imu", "odom"},
        "pose_generation_binding": dashboard.get("initial_pose_maps_dir")
        == MAPS_DIR
        and dashboard.get("map_lifecycle_topic")
        == "/robonix/map/lifecycle"
        and dashboard.get("initial_pose_auto_restore") is False,
        "dual_preview_topics": dashboard.get("d435i_color_topic")
        == D435I_RGB_TOPIC
        and dashboard.get("d435i_depth_topic") == D435I_DEPTH_TOPIC
        and dashboard.get("d435i_camera_info_topic")
        == D435I_CAMERA_INFO_TOPIC,
        "voice_enabled": dashboard.get("browser_voice_enabled") is True,
        "runtime_ack_not_committed": isinstance(environment, dict)
        and not any(key in environment for key in FORBIDDEN_COMMITTED_RUNTIME_ENV),
        "velocity_environment": isinstance(environment, dict)
        and environment.get("ROBONIX_VELOCITY_OUTPUT_TOPIC")
        == CHASSIS_COMMAND_TOPIC,
    }
    failed = sorted(name for name, passed in required.items() if not passed)
    if failed:
        raise ManifestError(
            "unsafe persistent voice-nav2 manifest: " + ",".join(failed)
        )


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
        markers = [token.strip() for token in args.passive_state_markers.split(",")]
        if any(not token or not token.isdecimal() for token in markers):
            raise ManifestError(
                "passive-state-markers must be comma-separated decimal integers"
            )
        base = yaml.safe_load(args.base.read_text(encoding="utf-8"))
        manifest = render(
            base,
            state_marker=args.state_marker,
            passive_state_markers=[int(token, 10) for token in markers],
        )
        write_manifest(args.output, manifest)
    except (OSError, yaml.YAMLError, ManifestError) as error:
        parser.error(str(error))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
