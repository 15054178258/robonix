#!/usr/bin/env python3
"""Create, but never execute, one short-lived first-motion permit.

The resulting private JSON is consumed atomically by the chassis provider. It
does not start ROS, arm the adapter, publish Twist, or call a Unitree API.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import secrets
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.first_motion_permit import (  # noqa: E402
    FIRST_MOTION_ACK,
    PERMIT_SCHEMA,
    sha256_file,
)
from go2_chassis.runtime_config import (  # noqa: E402
    FIRST_MOTION_COMMAND_TOPIC,
    FIRST_MOTION_COMMAND_TIMEOUT_S,
    FIRST_MOTION_MAX_DISTANCE_M,
    FIRST_MOTION_MAX_DURATION_S,
    FIRST_MOTION_MAX_VX_MPS,
    FIRST_MOTION_MAX_VY_MPS,
    FIRST_MOTION_MAX_WZ_RPS,
    FIRST_MOTION_ODOM_TOPIC,
    FIRST_MOTION_PROFILE,
)


def _decimal_list(value: str, *, maximum: int, allow_empty: bool) -> list[int]:
    if not value.strip():
        if allow_empty:
            return []
        raise argparse.ArgumentTypeError("at least one decimal value is required")
    result: list[int] = []
    for token in value.split(","):
        if not token.isdecimal():
            raise argparse.ArgumentTypeError("values must be comma-separated decimals")
        item = int(token, 10)
        if not 0 <= item <= maximum:
            raise argparse.ArgumentTypeError("value is outside its allowed range")
        if item not in result:
            result.append(item)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--network-interface", required=True)
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--allowed-modes", required=True)
    parser.add_argument("--allowed-state-markers", default="")
    parser.add_argument("--dds-identity-evidence", type=Path, required=True)
    parser.add_argument("--state-evidence", type=Path, required=True)
    parser.add_argument("--time-evidence", type=Path, required=True)
    parser.add_argument("--lifetime-seconds", type=int, default=120)
    parser.add_argument("--ack", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.ack != FIRST_MOTION_ACK:
        raise SystemExit(f"--ack must exactly equal {FIRST_MOTION_ACK}")
    if not 30 <= args.lifetime_seconds <= 300:
        raise SystemExit("--lifetime-seconds must be between 30 and 300")
    if not args.output.is_absolute():
        raise SystemExit("--output must be absolute")
    if args.output.exists():
        raise SystemExit("refusing to overwrite an existing permit")
    if not args.output.parent.is_dir():
        raise SystemExit("permit parent directory does not exist")
    parent = os.lstat(args.output.parent)
    if parent.st_uid != os.geteuid() or (parent.st_mode & 0o777) != 0o700:
        raise SystemExit("permit parent must be owned by this UID with mode 0700")

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
    }
    evidence = {
        name: {"path": str(path.resolve()), "sha256": sha256_file(path.resolve())}
        for name, path in evidence_paths.items()
    }
    now_ns = time.time_ns()
    payload = {
        "schema": PERMIT_SCHEMA,
        "permit_id": "permit-" + secrets.token_hex(16),
        "session_id": args.session_id,
        "one_time": True,
        "issued_unix_ns": now_ns,
        "expires_unix_ns": now_ns + args.lifetime_seconds * 1_000_000_000,
        "profile": FIRST_MOTION_PROFILE,
        "operator_ack": FIRST_MOTION_ACK,
        "network_interface": args.network_interface,
        "state_topic": args.state_topic,
        "command_topic": FIRST_MOTION_COMMAND_TOPIC,
        "odom_topic": FIRST_MOTION_ODOM_TOPIC,
        "arm_service": "/go2_chassis/arm",
        "allowed_modes": modes,
        "allowed_state_markers": markers,
        "max_linear_x_mps": FIRST_MOTION_MAX_VX_MPS,
        "max_linear_y_mps": FIRST_MOTION_MAX_VY_MPS,
        "max_angular_z_rps": FIRST_MOTION_MAX_WZ_RPS,
        "max_duration_s": FIRST_MOTION_MAX_DURATION_S,
        "max_distance_m": FIRST_MOTION_MAX_DISTANCE_M,
        "command_timeout_s": FIRST_MOTION_COMMAND_TIMEOUT_S,
        "evidence": evidence,
    }
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    descriptor = os.open(args.output, flags, 0o600)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
    except Exception:
        try:
            args.output.unlink()
        except FileNotFoundError:
            pass
        raise
    print(args.output)
    print("PERMIT CREATED ONLY; no ROS node, publisher, arm, or motion was started")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
