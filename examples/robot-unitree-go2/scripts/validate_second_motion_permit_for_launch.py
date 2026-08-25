#!/usr/bin/env python3
"""Read-only preflight for the exact second-motion permit and evidence paths."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.runtime_config import (  # noqa: E402
    ConfigError,
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    ODOM_SOURCE_EXTERNAL_VERIFIED,
    SECOND_MOTION_COMMAND_TOPIC,
    SECOND_MOTION_CONTROL_RATE_HZ,
    SECOND_MOTION_MAX_ANGULAR_ACCEL_RPS2,
    SECOND_MOTION_MAX_DISTANCE_M,
    SECOND_MOTION_MAX_DURATION_S,
    SECOND_MOTION_MAX_LINEAR_ACCEL_MPS2,
    SECOND_MOTION_MAX_VX_MPS,
    SECOND_MOTION_MAX_VY_MPS,
    SECOND_MOTION_MAX_WZ_RPS,
    SECOND_MOTION_ODOM_TOPIC,
    SECOND_MOTION_PROFILE,
    normalize_config,
)
from go2_chassis.second_motion_permit import (  # noqa: E402
    PermitError,
    _read_json,
    validate_permit,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--permit", required=True, type=Path)
    parser.add_argument("--package-root", required=True, type=Path)
    parser.add_argument("--network-interface", required=True)
    parser.add_argument("--allowed-mode", required=True, type=int)
    parser.add_argument("--allowed-state-marker", required=True, type=int)
    parser.add_argument("--dds-evidence", required=True, type=Path)
    parser.add_argument("--state-evidence", required=True, type=Path)
    parser.add_argument("--time-evidence", required=True, type=Path)
    parser.add_argument("--first-motion-pass-evidence", required=True, type=Path)
    args = parser.parse_args()
    try:
        runtime = normalize_config(
            {
                "allow_motion": True,
                "motion_profile": SECOND_MOTION_PROFILE,
                "operator_present": True,
                "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
                "network_interface": args.network_interface,
                "state_topic": (
                    "/robonix/time_corrected/motion/sportmodestate"
                ),
                "state_fallback_topic": "",
                "twist_in_topic": SECOND_MOTION_COMMAND_TOPIC,
                "odom_source": ODOM_SOURCE_EXTERNAL_VERIFIED,
                "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
                "external_odom_timeout_s": 0.20,
                "odom_topic": SECOND_MOTION_ODOM_TOPIC,
                "publish_odom_tf": True,
                "stationary_pose_hold_enabled": False,
                "max_linear_x_mps": SECOND_MOTION_MAX_VX_MPS,
                "max_linear_y_mps": SECOND_MOTION_MAX_VY_MPS,
                "max_angular_z_rps": SECOND_MOTION_MAX_WZ_RPS,
                "max_linear_accel_mps2": (
                    SECOND_MOTION_MAX_LINEAR_ACCEL_MPS2
                ),
                "max_angular_accel_rps2": (
                    SECOND_MOTION_MAX_ANGULAR_ACCEL_RPS2
                ),
                "command_timeout_s": 0.20,
                "control_rate_hz": SECOND_MOTION_CONTROL_RATE_HZ,
                "state_timeout_s": 0.20,
                "max_source_stamp_age_s": 0.20,
                "max_source_stamp_future_skew_s": 0.05,
                "commissioning_max_duration_s": (
                    SECOND_MOTION_MAX_DURATION_S
                ),
                "commissioning_max_distance_m": (
                    SECOND_MOTION_MAX_DISTANCE_M
                ),
            },
            {
                "GO2_ALLOWED_MODES": str(args.allowed_mode),
                "GO2_ALLOWED_STATE_MARKERS": (
                    ""
                    if args.allowed_state_marker == 0
                    else str(args.allowed_state_marker)
                ),
            },
            args.package_root,
        )
        payload = _read_json(args.permit, private=True)
        permit_id = validate_permit(payload, runtime, args.package_root)
        evidence = payload["evidence"]
        expected = {
            "dds_identity": args.dds_evidence.resolve(),
            "state": args.state_evidence.resolve(),
            "time": args.time_evidence.resolve(),
            "first_motion_pass": args.first_motion_pass_evidence.resolve(),
        }
        for name, path in expected.items():
            if Path(evidence[name]["path"]).resolve() != path:
                raise PermitError(
                    f"permit {name} evidence is not the launcher-selected file"
                )
    except (OSError, ConfigError, PermitError, KeyError, TypeError) as error:
        print(
            f"second-motion permit preflight failed: {error}",
            file=sys.stderr,
        )
        return 2
    print(permit_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
