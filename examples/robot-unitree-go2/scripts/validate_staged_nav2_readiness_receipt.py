#!/usr/bin/env python3
"""Strictly validate one staged Nav2 readiness receipt without ROS."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import stat
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from staged_nav2_readiness import (  # noqa: E402
    MAX_START_DISTANCE_M,
    ReadinessError,
    validate_readiness_receipt,
)


MAX_RECEIPT_BYTES = 256 * 1024


def _duplicate_safe_object(
    pairs: list[tuple[str, Any]],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _finite(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("value must be finite")
    return result


def _decimal(value: str, maximum: int) -> int:
    if not value.isdecimal():
        raise argparse.ArgumentTypeError("must be a decimal integer")
    result = int(value, 10)
    if not 0 <= result <= maximum:
        raise argparse.ArgumentTypeError("value is outside its allowed range")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--phase", choices=("pre_guard", "post_guard"), required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--network-interface", required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--map-generation", type=int, required=True)
    parser.add_argument(
        "--allowed-mode",
        type=lambda value: _decimal(value, 254),
        required=True,
    )
    parser.add_argument(
        "--allowed-state-marker",
        type=lambda value: _decimal(value, 4_294_967_295),
        required=True,
    )
    parser.add_argument("--expected-start-x", type=_finite)
    parser.add_argument("--expected-start-y", type=_finite)
    parser.add_argument("--permit-start-x", type=_finite)
    parser.add_argument("--permit-start-y", type=_finite)
    return parser.parse_args()


def _load_private_receipt(path: Path) -> object:
    if not path.is_absolute():
        raise ValueError("receipt path must be absolute")
    metadata = path.lstat()
    if (
        not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != os.geteuid()
        or stat.S_IMODE(metadata.st_mode) != 0o600
        or metadata.st_size > MAX_RECEIPT_BYTES
    ):
        raise ValueError(
            "receipt must be a current-user 0600 regular file within size limit"
        )
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream, object_pairs_hook=_duplicate_safe_object)


def main() -> int:
    args = parse_args()
    if args.map_generation <= 0:
        print("readiness receipt rejected: map generation must be positive", file=sys.stderr)
        return 2
    if args.phase == "post_guard":
        if (
            args.expected_start_x is None
            or args.expected_start_y is None
            or args.permit_start_x is None
            or args.permit_start_y is None
        ):
            print(
                "readiness receipt rejected: post_guard requires live and "
                "permit-bound starts",
                file=sys.stderr,
            )
            return 2
    elif any(
        value is not None
        for value in (
            args.expected_start_x,
            args.expected_start_y,
            args.permit_start_x,
            args.permit_start_y,
        )
    ):
        print(
            "readiness receipt rejected: pre_guard must not receive start claims",
            file=sys.stderr,
        )
        return 2
    try:
        payload = _load_private_receipt(args.receipt)
        validated = validate_readiness_receipt(
            payload,
            phase=args.phase,
            session_id=args.session_id,
            network_interface=args.network_interface,
            map_id=args.map_id,
            map_generation=args.map_generation,
            allowed_mode=args.allowed_mode,
            allowed_state_marker=args.allowed_state_marker,
            expected_start_x=args.expected_start_x,
            expected_start_y=args.expected_start_y,
            now_realtime_ns=time.time_ns(),
        )
        if args.phase == "post_guard":
            localization = validated.get("localization")
            if not isinstance(localization, dict):
                raise ReadinessError(
                    "post_guard receipt has no localization witness"
                )
            localization_x = localization.get("x_m")
            localization_y = localization.get("y_m")
            if (
                isinstance(localization_x, bool)
                or not isinstance(localization_x, (int, float))
                or isinstance(localization_y, bool)
                or not isinstance(localization_y, (int, float))
            ):
                raise ReadinessError(
                    "post_guard localization witness is not numeric"
                )
            permit_distance = math.hypot(
                float(localization_x) - args.permit_start_x,
                float(localization_y) - args.permit_start_y,
            )
            if (
                not math.isfinite(permit_distance)
                or permit_distance > MAX_START_DISTANCE_M
            ):
                raise ReadinessError(
                    "post_guard localization is more than 0.05 m from "
                    "the permit-bound path start"
                )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError, ReadinessError) as error:
        print(f"readiness receipt rejected: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
