#!/usr/bin/env python3
"""Create/update a local, ignored, physically verified landmark pose."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import tempfile

import yaml


DEFAULT_ARRIVAL_RADIUS_M = 0.35


def uint64(value: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a base-10 uint64") from exc
    if parsed < 0 or parsed > (1 << 64) - 1:
        raise argparse.ArgumentTypeError("must be within uint64 range")
    return parsed


def region_point(value: str) -> list[float]:
    fields = value.split(",")
    if len(fields) != 2:
        raise argparse.ArgumentTypeError("must use X,Y")
    try:
        point = [float(fields[0]), float(fields[1])]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("X and Y must be numbers") from exc
    if not all(math.isfinite(item) for item in point):
        raise argparse.ArgumentTypeError("X and Y must be finite")
    return point


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Save one measured semantic location; this only edits the local YAML "
            "file and never moves the robot."
        )
    )
    parser.add_argument(
        "--id",
        help=(
            "stable location id; required to add a new entry. Without --id, "
            "the legacy 自动售货机 entry is updated by name."
        ),
    )
    parser.add_argument(
        "--name",
        help="Chinese display name; required for a new --id, optional when updating",
    )
    parser.add_argument(
        "--alias",
        action="append",
        default=[],
        help="replace aliases with this value; repeat for multiple aliases",
    )
    parser.add_argument(
        "--kind",
        choices=("navigation", "marker"),
        default="navigation",
        help="markers are named map references and can never dispatch navigation",
    )
    parser.add_argument("--map-id", required=True)
    parser.add_argument(
        "--generation",
        required=True,
        type=uint64,
        help="generation from the live robonix/service/map/lifecycle sample",
    )
    parser.add_argument("--x", required=True, type=float)
    parser.add_argument("--y", required=True, type=float)
    parser.add_argument("--yaw", required=True, type=float, help="radians in map frame")
    parser.add_argument(
        "--arrival-radius",
        type=float,
        help=(
            "navigation arrival radius in metres; an existing value is kept, "
            f"otherwise {DEFAULT_ARRIVAL_RADIUS_M:.2f} is used"
        ),
    )
    parser.add_argument(
        "--region-point",
        type=region_point,
        action="append",
        default=[],
        metavar="X,Y",
        help=(
            "optional named region polygon vertex in map metres; repeat at least "
            "three times, in boundary order"
        ),
    )
    parser.add_argument("--measured-by", required=True)
    parser.add_argument(
        "--confirm-free-space",
        required=True,
        choices=["YES_POSE_AND_FOOTPRINT_ARE_CLEAR"],
        help="explicit physical clearance acknowledgement",
    )
    parser.add_argument(
        "--output",
        default="config/semantic_landmarks.local.yaml",
        help="local file is intentionally gitignored",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.map_id.strip():
        raise SystemExit("map-id must not be empty")
    for label, value in (("x", args.x), ("y", args.y), ("yaw", args.yaw)):
        if not math.isfinite(value):
            raise SystemExit(f"{label} must be finite")
    root = Path(__file__).resolve().parent.parent
    template = root / "config" / "semantic_landmarks.yaml"
    output = Path(args.output)
    if not output.is_absolute():
        output = root / output
    source = output if output.is_file() else template
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SystemExit(f"landmark document must be a mapping: {source}")
    data["schema_version"] = 2
    data["map_id"] = args.map_id.strip()
    data["map_generation"] = args.generation
    rows = data.get("landmarks")
    if not isinstance(rows, list):
        raise SystemExit(f"landmarks must be a list: {source}")
    if args.id:
        landmark_id = args.id.strip()
        if not landmark_id:
            raise SystemExit("id must not be empty")
        matches = [
            row for row in rows if isinstance(row, dict) and row.get("id") == landmark_id
        ]
        if len(matches) > 1:
            raise SystemExit(
                f"expected at most one landmark id {landmark_id!r}, found {len(matches)}"
            )
        if matches:
            row = matches[0]
        else:
            if not args.name or not args.name.strip():
                raise SystemExit("--name is required when adding a new --id")
            row = {
                "id": landmark_id,
                "name": args.name.strip(),
                "aliases": [],
                "verified": False,
            }
            rows.append(row)
    else:
        lookup_name = (args.name or "自动售货机").strip()
        matches = [
            row
            for row in rows
            if isinstance(row, dict) and row.get("name") == lookup_name
        ]
        if len(matches) != 1:
            raise SystemExit(
                f"expected exactly one landmark named {lookup_name!r}, found {len(matches)}"
            )
        row = matches[0]
    if args.name:
        if not args.name.strip():
            raise SystemExit("name must not be empty")
        row["name"] = args.name.strip()
    if args.alias:
        aliases = [value.strip() for value in args.alias]
        if any(not value for value in aliases):
            raise SystemExit("aliases must not be empty")
        row["aliases"] = aliases
    row["kind"] = args.kind
    row["pose"] = {
        "x": round(args.x, 6),
        "y": round(args.y, 6),
        "yaw": round(math.atan2(math.sin(args.yaw), math.cos(args.yaw)), 6),
    }
    if args.kind == "navigation":
        radius = (
            args.arrival_radius
            if args.arrival_radius is not None
            else row.get("arrival_radius", DEFAULT_ARRIVAL_RADIUS_M)
        )
        if (
            isinstance(radius, bool)
            or not isinstance(radius, (int, float))
            or not math.isfinite(float(radius))
            or not 0.05 <= float(radius) <= 10.0
        ):
            raise SystemExit("arrival-radius must be between 0.05 and 10.0 metres")
        row["arrival_radius"] = round(float(radius), 6)
    else:
        if args.arrival_radius is not None:
            raise SystemExit("arrival-radius is only valid for navigation entries")
        row.pop("arrival_radius", None)
    if args.region_point:
        if len(args.region_point) < 3 or len({tuple(point) for point in args.region_point}) < 3:
            raise SystemExit("region requires at least three distinct --region-point values")
        twice_area = sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(
                args.region_point, args.region_point[1:] + args.region_point[:1]
            )
        )
        if abs(twice_area) <= 1e-9:
            raise SystemExit("region polygon must have non-zero area")
        row["region"] = {
            "points": [
                [round(point[0], 6), round(point[1], 6)]
                for point in args.region_point
            ]
        }
    row["verified"] = True
    row["metadata"] = {
        "measured_by": args.measured_by,
        "measured_at": datetime.now(timezone.utc).isoformat(),
        "source": "physical_map_clearance_check",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=output.parent, delete=False) as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    temporary.replace(output)
    print(f"saved verified landmark to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
