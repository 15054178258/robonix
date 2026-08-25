#!/usr/bin/env python3
"""Validate one post-boot live short-goal receipt and print its start claim."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.staged_nav2_permit import (  # noqa: E402
    OPERATOR_GOAL_SOURCE,
    PermitError,
    sha256_file,
    validate_short_goal_evidence,
)

MAX_PERMIT_START_DRIFT_M = 0.05


def finite(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("must be finite")
    return result


def positive(value: str) -> int:
    result = int(value, 10)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--map-generation", type=positive, required=True)
    parser.add_argument("--target-id", required=True)
    parser.add_argument("--goal-x", type=finite, required=True)
    parser.add_argument("--goal-y", type=finite, required=True)
    parser.add_argument("--goal-yaw", type=finite, required=True)
    parser.add_argument("--permit-start-x", type=finite)
    parser.add_argument("--permit-start-y", type=finite)
    args = parser.parse_args()
    if not args.evidence.is_absolute():
        parser.error("--evidence must be absolute")
    try:
        evidence_hash = sha256_file(args.evidence)
        validated = validate_short_goal_evidence(
            args.evidence,
            expected_sha256=evidence_hash,
            map_id=args.map_id,
            map_generation=args.map_generation,
            goal_source=OPERATOR_GOAL_SOURCE,
            target_id=args.target_id,
            goal_pose={
                "x": args.goal_x,
                "y": args.goal_y,
                "yaw": args.goal_yaw,
            },
        )
    except (OSError, PermitError, ValueError) as error:
        print(f"live staged goal evidence rejected: {error}", file=sys.stderr)
        return 1
    if (args.permit_start_x is None) != (args.permit_start_y is None):
        parser.error("permit start coordinates must be supplied together")
    if args.permit_start_x is not None:
        permit_start_drift = math.hypot(
            validated.start_x - args.permit_start_x,
            validated.start_y - args.permit_start_y,
        )
        if (
            not math.isfinite(permit_start_drift)
            or permit_start_drift > MAX_PERMIT_START_DRIFT_M
        ):
            print(
                "live staged goal evidence rejected: "
                "fresh path start drifted more than 0.05 m from permit-bound start",
                file=sys.stderr,
            )
            return 1
    print(
        "\n".join(
            (
                format(validated.start_x, ".17g"),
                format(validated.start_y, ".17g"),
                format(validated.start_yaw, ".17g"),
                validated.sha256,
            )
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
