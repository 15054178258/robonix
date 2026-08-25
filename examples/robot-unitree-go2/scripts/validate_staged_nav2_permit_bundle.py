#!/usr/bin/env python3
"""Validate, but never consume, one matched staged Nav2 permit pair."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.runtime_config import (  # noqa: E402
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    STAGED_NAV2_COMMAND_TOPIC,
    STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
    STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
    STAGED_NAV2_MAX_DISTANCE_M,
    STAGED_NAV2_MAX_DURATION_S,
    STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
    STAGED_NAV2_MAX_VX_MPS,
    STAGED_NAV2_MAX_VY_MPS,
    STAGED_NAV2_MAX_WZ_RPS,
    STAGED_NAV2_PROFILE,
    STAGED_NAV2_STATE_TIMEOUT_S,
    ConfigError,
    normalize_config,
)
from go2_chassis.staged_nav2_permit import (  # noqa: E402
    PermitError,
    validate_staged_nav2_permit_bundle,
)


STATE_TOPIC = "/robonix/time_corrected/motion/sportmodestate"


def decimal(value: str, maximum: int) -> int:
    if not value.isdecimal():
        raise argparse.ArgumentTypeError("must be a decimal integer")
    result = int(value, 10)
    if not 0 <= result <= maximum:
        raise argparse.ArgumentTypeError("is outside the allowed range")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chassis-permit", type=Path, required=True)
    parser.add_argument("--goal-permit", type=Path, required=True)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--network-interface", required=True)
    parser.add_argument(
        "--allowed-mode",
        required=True,
        type=lambda value: decimal(value, 254),
    )
    parser.add_argument(
        "--allowed-state-marker",
        required=True,
        type=lambda value: decimal(value, 4_294_967_295),
    )
    parser.add_argument("--ipc-socket", type=Path, required=True)
    parser.add_argument("--dds-evidence", type=Path, required=True)
    parser.add_argument("--state-evidence", type=Path, required=True)
    parser.add_argument("--time-evidence", type=Path, required=True)
    parser.add_argument("--goal-evidence", type=Path, required=True)
    return parser.parse_args()


def _runtime(args: argparse.Namespace):
    marker_text = "" if args.allowed_state_marker == 0 else str(
        args.allowed_state_marker
    )
    environ = {
        "GO2_ALLOW_MOTION": "true",
        "GO2_OPERATOR_PRESENT": "true",
        "GO2_SAFETY_ACK": "I_UNDERSTAND_GO2_CAN_MOVE",
        "GO2_MOTION_PROFILE": STAGED_NAV2_PROFILE,
        "GO2_ALLOWED_MODES": str(args.allowed_mode),
        "GO2_ALLOWED_STATE_MARKERS": marker_text,
    }
    return normalize_config(
        {
            "allow_motion": True,
            "motion_profile": STAGED_NAV2_PROFILE,
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": args.network_interface,
            "ipc_socket": str(args.ipc_socket),
            "state_topic": STATE_TOPIC,
            "state_fallback_topic": "",
            "twist_in_topic": STAGED_NAV2_COMMAND_TOPIC,
            "odom_source": "external_verified",
            "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
            "external_odom_timeout_s": STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
            "odom_topic": "/odom",
            "publish_odom_tf": True,
            "max_linear_x_mps": STAGED_NAV2_MAX_VX_MPS,
            "max_linear_y_mps": STAGED_NAV2_MAX_VY_MPS,
            "max_angular_z_rps": STAGED_NAV2_MAX_WZ_RPS,
            "max_linear_accel_mps2": STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
            "max_angular_accel_rps2": STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
            "command_timeout_s": 0.20,
            "state_timeout_s": STAGED_NAV2_STATE_TIMEOUT_S,
            "max_source_stamp_age_s": 0.20,
            "max_source_stamp_future_skew_s": 0.05,
            "commissioning_max_duration_s": STAGED_NAV2_MAX_DURATION_S,
            "commissioning_max_distance_m": STAGED_NAV2_MAX_DISTANCE_M,
        },
        environ,
        args.package_root,
    )


def main() -> int:
    args = parse_args()
    for label, path in (
        ("chassis permit", args.chassis_permit),
        ("goal permit", args.goal_permit),
        ("package root", args.package_root),
        ("IPC socket", args.ipc_socket),
        ("DDS evidence", args.dds_evidence),
        ("state evidence", args.state_evidence),
        ("time evidence", args.time_evidence),
        ("goal evidence", args.goal_evidence),
    ):
        if not path.is_absolute():
            print(f"{label} path must be absolute", file=sys.stderr)
            return 2
    try:
        runtime = _runtime(args)
        bundle = validate_staged_nav2_permit_bundle(
            args.chassis_permit,
            args.goal_permit,
            runtime,
            args.package_root,
            {
                "dds_identity": args.dds_evidence,
                "state": args.state_evidence,
                "time": args.time_evidence,
                "goal": args.goal_evidence,
            },
        )
    except (PermitError, ConfigError, OSError, ValueError) as error:
        print(f"staged permit bundle rejected: {error}", file=sys.stderr)
        return 1

    # Fixed newline-delimited order for Bash mapfile.  Every text value is
    # already constrained to safe ASCII and cannot contain a newline.
    values = (
        bundle.session_id,
        bundle.pair_id,
        bundle.map_id,
        str(bundle.map_generation),
        bundle.goal_source,
        bundle.target_id,
        format(bundle.goal_x, ".17g"),
        format(bundle.goal_y, ".17g"),
        format(bundle.goal_yaw, ".17g"),
        bundle.goal_evidence.sha256,
        bundle.chassis_permit_id,
        bundle.goal_permit_id,
        format(bundle.goal_evidence.start_x, ".17g"),
        format(bundle.goal_evidence.start_y, ".17g"),
        format(bundle.goal_evidence.start_yaw, ".17g"),
    )
    print("\n".join(values))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
