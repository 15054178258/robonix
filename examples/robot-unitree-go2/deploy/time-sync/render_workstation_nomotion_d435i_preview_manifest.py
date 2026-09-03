#!/usr/bin/env python3
"""Render the corrected no-motion stack with opt-in D435i preview ingest."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml

from render_workstation_nomotion_manifest import (
    ManifestError,
    _named,
    render as render_base_nomotion,
    validate_rendered as validate_base_nomotion,
    write_manifest,
)


PROFILE = "workstation-full-nomotion-corrected-d435i-preview-v1"
D435I_PROVIDER_ID = "go2_d435i"
D435I_PACKAGE_PATH = "${ROBONIX_DEPLOY_DIR}/packages/go2_d435i"
D435I_RGB_TOPIC = "/go2/d435i/color/image_raw"
D435I_DEPTH_TOPIC = "/go2/d435i/aligned_depth_to_color/image_raw"
D435I_CAMERA_INFO_TOPIC = "/go2/d435i/color/camera_info"
D435I_OPTICAL_FRAME = "d435i_color_optical_frame"
# The workstation bridge intentionally uses the 320x240 low-bandwidth preview
# profile.  Its sustained rate is about 4 Hz on the robot Wi-Fi path, so keep a
# meaningful rate gate without imposing the provider's generic 5 Hz default.
D435I_MIN_RATE_HZ = 3.0


def _d435i_primitive() -> dict[str, Any]:
    """Return the external-only camera registrar configuration.

    The Orin camera bridge remains a separately started process. This
    workstation primitive creates subscriptions only and fails activation
    unless its complete RGB-D quality gate passes.
    """

    return {
        "name": D435I_PROVIDER_ID,
        "path": D435I_PACKAGE_PATH,
        "config": {
            "source_mode": "external",
            "rgb_topic": D435I_RGB_TOPIC,
            "depth_topic": D435I_DEPTH_TOPIC,
            "camera_info_topic": D435I_CAMERA_INFO_TOPIC,
            "rgb_frame": D435I_OPTICAL_FRAME,
            "depth_frame": D435I_OPTICAL_FRAME,
            "sentinel_timeout_s": 30.0,
            "quality_window_s": 5.0,
            "min_rate_hz": D435I_MIN_RATE_HZ,
            "max_stamp_age_s": 0.50,
            "max_future_skew_s": 0.05,
            "max_rgb_depth_skew_s": 0.01,
        },
    }


def render(
    base: dict[str, Any],
    *,
    state_marker: int,
    passive_state_markers: list[int] | tuple[int, ...],
) -> dict[str, Any]:
    """Add D435i preview ingest without changing the mapping or motion route."""

    rendered = render_base_nomotion(
        base,
        state_marker=state_marker,
        passive_state_markers=passive_state_markers,
    )
    rendered["name"] = "robonix-go2-workstation-full-nomotion-d435i-preview"

    primitives = rendered.get("primitive")
    if not isinstance(primitives, list):
        raise ManifestError("manifest primitive must be a list")
    if any(
        isinstance(entry, dict) and entry.get("name") == D435I_PROVIDER_ID
        for entry in primitives
    ):
        raise ManifestError("base manifest already contains go2_d435i")
    primitives.append(_d435i_primitive())

    systems = rendered.get("system")
    if not isinstance(systems, dict):
        raise ManifestError("manifest system must be a mapping")
    scene = systems.get("scene")
    if not isinstance(scene, dict):
        raise ManifestError("manifest system.scene config is missing")
    scene["provider_ids"] = {
        "rgb": D435I_PROVIDER_ID,
        "depth": D435I_PROVIDER_ID,
        "intrinsics": D435I_PROVIDER_ID,
    }
    scene["camera_frame"] = D435I_OPTICAL_FRAME
    # The D435i-to-base extrinsic has not been measured.  Keep raw RGB/depth
    # available to Scene's passive /cam view, but fail closed before either
    # detector can turn those pixels into untrustworthy world coordinates.
    scene["perception_enabled"] = False

    environment = rendered.setdefault("env", {})
    if not isinstance(environment, dict):
        raise ManifestError("manifest env must be a mapping")
    environment["GO2_D435I_PROFILE"] = PROFILE

    validate_rendered(rendered)
    return rendered


def validate_rendered(manifest: dict[str, Any]) -> None:
    """Prove D435i is preview-only and the hardened no-motion base is intact."""

    validate_base_nomotion(manifest)
    d435i = _named(manifest.get("primitive"), D435I_PROVIDER_ID, "primitive")
    expected = _d435i_primitive()
    if d435i != expected:
        raise ManifestError("go2_d435i preview configuration is not exact")

    scene = manifest.get("system", {}).get("scene", {})
    expected_pins = {
        "rgb": D435I_PROVIDER_ID,
        "depth": D435I_PROVIDER_ID,
        "intrinsics": D435I_PROVIDER_ID,
    }
    if scene.get("provider_ids") != expected_pins:
        raise ManifestError("Scene must pin one coherent go2_d435i RGB-D set")
    if scene.get("camera_frame") != D435I_OPTICAL_FRAME:
        raise ManifestError("Scene D435i optical frame is not exact")
    if scene.get("perception_enabled") is not False:
        raise ManifestError(
            "Scene perception must remain disabled until D435i extrinsics pass"
        )

    mapping = _named(manifest.get("service"), "mapping", "service")["config"]
    if mapping.get("rtabmap_inputs") != ["lidar", "imu", "odom"]:
        raise ManifestError("D435i preview must not enter RTAB-Map")
    providers = mapping.get("sensor_providers")
    if not isinstance(providers, dict):
        raise ManifestError("mapping sensor_providers is missing")
    if "rgb" in providers or "depth" in providers:
        raise ManifestError("D435i preview must not bind mapping RGB-D providers")
    if manifest.get("env", {}).get("GO2_D435I_PROFILE") != PROFILE:
        raise ManifestError("D435i preview profile marker is missing")


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
            passive_state_markers=[int(token, 10) for token in marker_tokens],
        )
        write_manifest(args.output, rendered)
    except (OSError, yaml.YAMLError, ManifestError) as error:
        parser.error(str(error))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
