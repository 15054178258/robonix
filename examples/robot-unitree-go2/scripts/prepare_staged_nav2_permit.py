#!/usr/bin/env python3
"""Create, but never execute, one private staged Nav2 stage-1 permit."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import secrets
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.runtime_config import (  # noqa: E402
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    STAGED_NAV2_COMMAND_TIMEOUT_S,
    STAGED_NAV2_COMMAND_TOPIC,
    STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
    STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
    STAGED_NAV2_MAX_DISTANCE_M,
    STAGED_NAV2_MAX_DURATION_S,
    STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
    STAGED_NAV2_MAX_VX_MPS,
    STAGED_NAV2_MAX_VY_MPS,
    STAGED_NAV2_MAX_WZ_RPS,
    STAGED_NAV2_NAV_COMMAND_TOPIC,
    STAGED_NAV2_ODOM_TOPIC,
    STAGED_NAV2_PROFILE,
    STAGED_NAV2_STAGE,
    STAGED_NAV2_STATE_TIMEOUT_S,
)
from go2_chassis.staged_nav2_permit import (  # noqa: E402
    CHASSIS_ROLE,
    GOAL_DISPATCH_ROLE,
    OPERATOR_GOAL_SOURCE,
    PERMIT_SCHEMA,
    STAGED_NAV2_ACK,
    sha256_file,
    validate_short_goal_evidence,
)

SESSION_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{7,63}$")
IDENTIFIER_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$")


def _finite(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("value must be finite")
    return result


def _decimal_list(value: str, *, maximum: int, allow_empty: bool) -> list[int]:
    if not value.strip():
        if allow_empty:
            return []
        raise argparse.ArgumentTypeError("at least one decimal value is required")
    result: list[int] = []
    for token in value.split(","):
        if not token.isdecimal():
            raise argparse.ArgumentTypeError(
                "values must be comma-separated decimals"
            )
        item = int(token, 10)
        if not 0 <= item <= maximum:
            raise argparse.ArgumentTypeError("value is outside its allowed range")
        if item not in result:
            result.append(item)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--goal-dispatch-output", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--network-interface", required=True)
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--allowed-modes", required=True)
    parser.add_argument("--allowed-state-markers", default="")
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--map-generation", type=int, required=True)
    parser.add_argument(
        "--goal-source",
        choices=(OPERATOR_GOAL_SOURCE,),
        required=True,
    )
    parser.add_argument("--target-id", required=True)
    parser.add_argument("--goal-x", type=_finite, required=True)
    parser.add_argument("--goal-y", type=_finite, required=True)
    parser.add_argument("--goal-yaw", type=_finite, required=True)
    parser.add_argument("--dds-identity-evidence", type=Path, required=True)
    parser.add_argument("--state-evidence", type=Path, required=True)
    parser.add_argument("--time-evidence", type=Path, required=True)
    parser.add_argument("--goal-evidence", type=Path, required=True)
    parser.add_argument("--lifetime-seconds", type=int, default=300)
    parser.add_argument("--ack", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.ack != STAGED_NAV2_ACK:
        raise SystemExit(f"--ack must exactly equal {STAGED_NAV2_ACK}")
    if not 30 <= args.lifetime_seconds <= 300:
        raise SystemExit("--lifetime-seconds must be between 30 and 300")
    if args.map_generation <= 0:
        raise SystemExit("--map-generation must be positive")
    if SESSION_RE.fullmatch(args.session_id) is None:
        raise SystemExit("--session-id must be 8-64 safe ASCII characters")
    if SESSION_RE.fullmatch(args.map_id) is None:
        raise SystemExit("--map-id must be 8-64 safe ASCII characters")
    if IDENTIFIER_RE.fullmatch(args.target_id) is None:
        raise SystemExit("--target-id must be 1-64 safe ASCII characters")
    if not -math.pi <= args.goal_yaw <= math.pi:
        raise SystemExit("--goal-yaw must be within [-pi, pi]")
    outputs = (args.output, args.goal_dispatch_output)
    if outputs[0] == outputs[1]:
        raise SystemExit("chassis and goal-dispatch permit paths must differ")
    for output in outputs:
        if not output.is_absolute():
            raise SystemExit("permit output paths must be absolute")
        if output.exists():
            raise SystemExit("refusing to overwrite an existing permit")
        if not output.parent.is_dir():
            raise SystemExit("permit parent directory does not exist")
        parent = os.lstat(output.parent)
        if parent.st_uid != os.geteuid() or (parent.st_mode & 0o777) != 0o700:
            raise SystemExit(
                "permit parent must be owned by this UID with mode 0700"
            )

    modes = _decimal_list(args.allowed_modes, maximum=254, allow_empty=False)
    markers = _decimal_list(
        args.allowed_state_markers,
        maximum=4_294_967_295,
        allow_empty=True,
    )
    if 0 in markers:
        raise SystemExit("zero must not appear in allowed-state-markers")
    evidence_paths = {
        "dds_identity": args.dds_identity_evidence,
        "state": args.state_evidence,
        "time": args.time_evidence,
        "goal": args.goal_evidence,
    }
    evidence = {
        name: {
            "path": str(path.resolve()),
            "sha256": sha256_file(path.resolve()),
        }
        for name, path in evidence_paths.items()
    }
    validate_short_goal_evidence(
        args.goal_evidence.resolve(),
        expected_sha256=evidence["goal"]["sha256"],
        map_id=args.map_id,
        map_generation=args.map_generation,
        goal_source=args.goal_source,
        target_id=args.target_id,
        goal_pose={
            "x": args.goal_x,
            "y": args.goal_y,
            "yaw": args.goal_yaw,
        },
    )
    now_ns = time.time_ns()
    pair_id = "pair-" + secrets.token_hex(16)
    common_payload = {
        "schema": PERMIT_SCHEMA,
        "pair_id": pair_id,
        "session_id": args.session_id,
        "one_time": True,
        "issued_unix_ns": now_ns,
        "expires_unix_ns": now_ns + args.lifetime_seconds * 1_000_000_000,
        "profile": STAGED_NAV2_PROFILE,
        "stage": STAGED_NAV2_STAGE,
        "operator_ack": STAGED_NAV2_ACK,
        "guard_ack": STAGED_NAV2_ACK,
        "network_interface": args.network_interface,
        "state_topic": args.state_topic,
        "nav_command_topic": STAGED_NAV2_NAV_COMMAND_TOPIC,
        "command_topic": STAGED_NAV2_COMMAND_TOPIC,
        "odom_topic": STAGED_NAV2_ODOM_TOPIC,
        "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
        "arm_service": "/go2_chassis/arm",
        "allowed_modes": modes,
        "allowed_state_markers": markers,
        "max_linear_x_mps": STAGED_NAV2_MAX_VX_MPS,
        "max_linear_y_mps": STAGED_NAV2_MAX_VY_MPS,
        "max_angular_z_rps": STAGED_NAV2_MAX_WZ_RPS,
        "max_linear_accel_mps2": STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
        "max_angular_accel_rps2": STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
        "max_duration_s": STAGED_NAV2_MAX_DURATION_S,
        "max_distance_m": STAGED_NAV2_MAX_DISTANCE_M,
        "command_timeout_s": STAGED_NAV2_COMMAND_TIMEOUT_S,
        "state_timeout_s": STAGED_NAV2_STATE_TIMEOUT_S,
        "external_odom_timeout_s": STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
        "map_id": args.map_id,
        "map_generation": args.map_generation,
        "goal_source": args.goal_source,
        "target_id": args.target_id,
        "goal_pose": {
            "x": args.goal_x,
            "y": args.goal_y,
            "yaw": args.goal_yaw,
        },
        "evidence": evidence,
    }

    def write_permit(path: Path, role: str) -> None:
        payload = {
            **common_payload,
            "permit_id": "permit-" + secrets.token_hex(16),
            "permit_role": role,
        }
        descriptor = os.open(
            path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
            0o600,
        )
        try:
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                json.dump(
                    payload,
                    handle,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
        except Exception:
            path.unlink(missing_ok=True)
            raise

    try:
        write_permit(args.output, CHASSIS_ROLE)
        write_permit(args.goal_dispatch_output, GOAL_DISPATCH_ROLE)
    except Exception:
        args.output.unlink(missing_ok=True)
        args.goal_dispatch_output.unlink(missing_ok=True)
        raise
    print(args.output)
    print(args.goal_dispatch_output)
    print(
        "STAGED PERMIT CREATED ONLY; no ROS node, publisher, arm, or motion "
        "was started"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
