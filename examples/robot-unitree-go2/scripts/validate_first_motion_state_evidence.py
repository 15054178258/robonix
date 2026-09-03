#!/usr/bin/env python3
"""Validate stationary SportModeState marker evidence for motion startup.

Successful stdout is exactly three decimal integers: ``mode marker gait``.
The caller must bind the first two into both the private permit and runtime
environment. The artifact selects an allowed state; it is not a runtime
freshness witness. The motion chain independently requires live state within
its 200 ms watchdog before and throughout motion. No health meaning is
assigned to the opaque marker.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from validate_nomotion_state_marker_evidence import (  # noqa: E402
    EvidenceError,
    _load_private_regular_file,
    validate_evidence,
)


def validate_first_motion_evidence(
    path: Path,
    *,
    now_realtime_ns: int | None = None,
) -> tuple[int, int, int]:
    marker = validate_evidence(
        path,
        max_age_seconds=None,
        now_realtime_ns=now_realtime_ns,
    )
    payload = _load_private_regular_file(path)
    observations: set[tuple[int, int, int]] = set()
    for stream in payload["streams"]:
        state = stream["states"][0]
        observations.add(
            (int(state["mode"]), int(state["error_code"]), int(state["gait_type"]))
        )
    if len(observations) != 1:
        raise EvidenceError("SportModeState evidence is not one stable state")
    mode, observed_marker, gait_type = observations.pop()
    if observed_marker != marker:
        raise EvidenceError("validated state marker changed during inspection")
    if not 0 <= mode <= 254:
        raise EvidenceError("observed mode must be a real uint8 mode, not sentinel 255")
    if not 0 <= gait_type <= 255:
        raise EvidenceError("observed gait_type is outside uint8 range")
    return mode, marker, gait_type


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path)
    args = parser.parse_args()
    try:
        mode, marker, gait_type = validate_first_motion_evidence(args.evidence)
    except EvidenceError as error:
        print(f"invalid first-motion state evidence: {error}", file=sys.stderr)
        return 2
    print(f"{mode} {marker} {gait_type}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
