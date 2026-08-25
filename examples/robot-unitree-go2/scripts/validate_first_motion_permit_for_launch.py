#!/usr/bin/env python3
"""Read-only preflight for the exact first-motion permit and evidence paths."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.first_motion_permit import (  # noqa: E402
    PermitError,
    _read_private_json,
    validate_permit,
)
from go2_chassis.runtime_config import (  # noqa: E402
    ConfigError,
    normalize_config,
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
    args = parser.parse_args()
    try:
        runtime = normalize_config(
            {
                "allow_motion": True,
                "motion_profile": "workstation-first-motion-corrected-v1",
                "operator_present": True,
                "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
                "network_interface": args.network_interface,
                "state_topic": "/robonix/time_corrected/motion/sportmodestate",
                "state_fallback_topic": "",
                "twist_in_topic": "/go2/commissioning/cmd_vel",
                "odom_topic": "/odom",
                "max_linear_x_mps": 0.05,
                "max_linear_y_mps": 0.0,
                "max_angular_z_rps": 0.0,
                "command_timeout_s": 0.20,
                "commissioning_max_duration_s": 2.0,
                "commissioning_max_distance_m": 0.10,
            },
            {
                "GO2_ALLOWED_MODES": str(args.allowed_mode),
                "GO2_ALLOWED_STATE_MARKERS": (
                    "" if args.allowed_state_marker == 0
                    else str(args.allowed_state_marker)
                ),
            },
            args.package_root,
        )
        payload = _read_private_json(args.permit)
        permit_id = validate_permit(payload, runtime, args.package_root)
        evidence = payload["evidence"]
        expected = {
            "dds_identity": args.dds_evidence.resolve(),
            "state": args.state_evidence.resolve(),
            "time": args.time_evidence.resolve(),
        }
        for name, path in expected.items():
            if Path(evidence[name]["path"]).resolve() != path:
                raise PermitError(
                    f"permit {name} evidence is not the launcher-selected file"
                )
    except (OSError, ConfigError, PermitError, KeyError, TypeError) as error:
        print(f"first-motion permit preflight failed: {error}", file=sys.stderr)
        return 2
    print(permit_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
