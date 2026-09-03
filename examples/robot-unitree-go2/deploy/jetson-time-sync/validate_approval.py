#!/usr/bin/env python3
"""Offline validator for the original Go2 clock evidence bundle."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "deploy" / "time-sync"))

from evidence_bundle import canonical_json, verify_bundle  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Recompute and validate an inactive time evidence bundle"
    )
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument(
        "--emit-canonical",
        action="store_true",
        help="print only the verified canonical payload for the formal launcher",
    )
    arguments = parser.parse_args(argv)
    try:
        payload = verify_bundle(arguments.bundle)
    except (OSError, ValueError) as error:
        print(f"invalid evidence bundle: {error}", file=sys.stderr)
        return 2
    if arguments.emit_canonical:
        print(canonical_json(payload))
    else:
        topics = [item["topic"] for item in payload["approved_writers"]]
        print("recomputed and validated topics: " + ", ".join(sorted(topics)))
        print("validation does not activate SYS_TIME or clock discipline")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
