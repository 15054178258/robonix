#!/usr/bin/env python3
"""Strictly validate the final measured result for one staged Nav2 run."""

from __future__ import annotations

import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.staged_nav2_result import (  # noqa: E402
    ResultError,
    read_private_json,
    result_paths,
    validate_measured_result,
)


def main() -> int:
    environ = dict(os.environ)
    try:
        action_path, measured_path = result_paths(environ)
        payload = read_private_json(measured_path)
        metrics = validate_measured_result(payload, environ, action_path)
    except (OSError, ResultError) as error:
        print(f"staged Nav2 measured result rejected: {error}", file=sys.stderr)
        return 1
    print(
        "staged Nav2 measured result PASS "
        f"forward={metrics['forward_m']:.6f}m "
        f"total={metrics['total_m']:.6f}m "
        f"lateral={metrics['lateral_m']:.6f}m "
        f"yaw={metrics['yaw_change_rad']:.6f}rad"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
