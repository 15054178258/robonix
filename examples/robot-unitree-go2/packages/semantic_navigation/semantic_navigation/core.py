"""ROS/Robonix-independent landmark loading and matching."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
import unicodedata
from typing import Any

import yaml

from .map_lifecycle import LifecycleBindingError, parse_generation


class LandmarkError(ValueError):
    """A landmark configuration or lookup failed closed."""


_SEPARATORS = re.compile(r"[\s\u3000,，。.!！?？、;；:：'\"“”‘’()（）\[\]【】<>《》_-]+")
_LANDMARK_KINDS = frozenset({"navigation", "marker"})
DEFAULT_ARRIVAL_RADIUS_M = 0.35


def normalize_text(value: str) -> str:
    """Normalize common Chinese ASR spacing/punctuation without translating."""

    return _SEPARATORS.sub("", unicodedata.normalize("NFKC", value).casefold())


def normalize_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Landmark:
    id: str
    name: str
    aliases: tuple[str, ...]
    kind: str
    map_id: str
    map_generation: int
    frame_id: str
    x: float | None
    y: float | None
    yaw: float | None
    arrival_radius: float | None
    region: tuple[tuple[float, float], ...] | None
    verified: bool
    metadata: dict[str, Any]

    @property
    def terms(self) -> tuple[str, ...]:
        return (self.name, *self.aliases)

    @property
    def navigable(self) -> bool:
        return self.kind == "navigation"


def _finite_number(value: Any, *, label: str) -> float:
    if isinstance(value, bool):
        raise LandmarkError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise LandmarkError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise LandmarkError(f"{label} must be a finite number")
    return result


def _parse_region(value: Any, *, landmark_id: str) -> tuple[tuple[float, float], ...] | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise LandmarkError(f"landmark {landmark_id!r} region must be a mapping")
    points = value.get("points")
    if not isinstance(points, list) or len(points) < 3:
        raise LandmarkError(
            f"landmark {landmark_id!r} region.points must contain at least three points"
        )
    parsed: list[tuple[float, float]] = []
    for index, point in enumerate(points):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise LandmarkError(
                f"landmark {landmark_id!r} region.points[{index}] must be [x, y]"
            )
        parsed.append(
            (
                _finite_number(
                    point[0], label=f"landmark {landmark_id!r} region.points[{index}].x"
                ),
                _finite_number(
                    point[1], label=f"landmark {landmark_id!r} region.points[{index}].y"
                ),
            )
        )
    if len(set(parsed)) < 3:
        raise LandmarkError(
            f"landmark {landmark_id!r} region must contain three distinct points"
        )
    twice_area = sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(parsed, parsed[1:] + parsed[:1])
    )
    if abs(twice_area) <= 1e-9:
        raise LandmarkError(f"landmark {landmark_id!r} region polygon has zero area")
    return tuple(parsed)


class LandmarkStore:
    def __init__(
        self,
        map_id: str,
        map_generation: int,
        frame_id: str,
        landmarks: tuple[Landmark, ...],
    ):
        if not map_id.strip():
            raise LandmarkError("landmark file has an empty map_id")
        if frame_id != "map":
            raise LandmarkError(f"landmark frame must be 'map', got {frame_id!r}")
        if not landmarks:
            raise LandmarkError("landmark file contains no landmarks")
        ids = [item.id for item in landmarks]
        if len(ids) != len(set(ids)):
            raise LandmarkError("landmark ids must be unique")
        self.map_id = map_id
        self.map_generation = map_generation
        self.frame_id = frame_id
        self.landmarks = landmarks

    @classmethod
    def from_path(cls, path: str | Path) -> "LandmarkStore":
        source = Path(path)
        if not source.is_file():
            raise LandmarkError(f"landmark file not found: {source}")
        try:
            raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise LandmarkError(f"cannot read landmark file {source}: {exc}") from exc
        return cls.from_mapping(raw)

    @classmethod
    def from_mapping(cls, raw: Any) -> "LandmarkStore":
        if not isinstance(raw, dict):
            raise LandmarkError("landmark document must be a mapping")
        if raw.get("schema_version") != 2:
            raise LandmarkError("unsupported semantic landmark schema_version")
        map_id = str(raw.get("map_id", "")).strip()
        try:
            map_generation = parse_generation(
                raw.get("map_generation"), label="landmark map_generation"
            )
        except LifecycleBindingError as exc:
            raise LandmarkError(str(exc)) from exc
        frame_id = str(raw.get("frame_id", "map")).strip()
        rows = raw.get("landmarks")
        if not isinstance(rows, list):
            raise LandmarkError("landmarks must be a list")

        items: list[Landmark] = []
        seen_terms: dict[str, str] = {}
        for index, row in enumerate(rows):
            if not isinstance(row, dict):
                raise LandmarkError(f"landmarks[{index}] must be a mapping")
            landmark_id = str(row.get("id", "")).strip()
            name = str(row.get("name", "")).strip()
            aliases_raw = row.get("aliases") or []
            kind = str(row.get("kind", "navigation")).strip()
            pose = row.get("pose")
            arrival_radius_raw = row.get(
                "arrival_radius", DEFAULT_ARRIVAL_RADIUS_M
            )
            region = _parse_region(row.get("region"), landmark_id=landmark_id)
            verified = row.get("verified", False)
            metadata = row.get("metadata") or {}
            if not landmark_id or not name:
                raise LandmarkError(f"landmarks[{index}] requires id and name")
            if kind not in _LANDMARK_KINDS:
                raise LandmarkError(
                    f"landmark {landmark_id!r} kind must be one of "
                    f"{sorted(_LANDMARK_KINDS)}"
                )
            if not isinstance(aliases_raw, list):
                raise LandmarkError(f"landmark {landmark_id!r} aliases must be a list")
            if not isinstance(verified, bool):
                raise LandmarkError(
                    f"landmark {landmark_id!r} verified must be a YAML boolean"
                )
            if not isinstance(metadata, dict):
                raise LandmarkError(f"landmark {landmark_id!r} metadata must be a mapping")
            aliases = tuple(str(value).strip() for value in aliases_raw if str(value).strip())
            x: float | None = None
            y: float | None = None
            yaw: float | None = None
            if pose is not None:
                if not isinstance(pose, dict):
                    raise LandmarkError(f"landmark {landmark_id!r} pose must be a mapping")
                try:
                    x = _finite_number(
                        pose["x"], label=f"landmark {landmark_id!r} pose.x"
                    )
                    y = _finite_number(
                        pose["y"], label=f"landmark {landmark_id!r} pose.y"
                    )
                    yaw = normalize_yaw(
                        _finite_number(
                            pose["yaw"], label=f"landmark {landmark_id!r} pose.yaw"
                        )
                    )
                except KeyError as exc:
                    raise LandmarkError(
                        f"landmark {landmark_id!r} pose requires x, y and yaw"
                    ) from exc
            if kind == "navigation" and pose is None:
                raise LandmarkError(
                    f"navigation landmark {landmark_id!r} requires a point pose"
                )
            if kind == "marker" and pose is None and region is None:
                raise LandmarkError(
                    f"marker {landmark_id!r} requires a point pose or region"
                )
            arrival_radius: float | None = None
            if kind == "navigation":
                arrival_radius = _finite_number(
                    arrival_radius_raw,
                    label=f"landmark {landmark_id!r} arrival_radius",
                )
                if not 0.05 <= arrival_radius <= 10.0:
                    raise LandmarkError(
                        f"landmark {landmark_id!r} arrival_radius must be "
                        "between 0.05 and 10.0 metres"
                    )
            elif "arrival_radius" in row:
                raise LandmarkError(
                    f"non-navigation marker {landmark_id!r} cannot set arrival_radius"
                )
            item = Landmark(
                id=landmark_id,
                name=name,
                aliases=aliases,
                kind=kind,
                map_id=map_id,
                map_generation=map_generation,
                frame_id=frame_id,
                x=x,
                y=y,
                yaw=yaw,
                arrival_radius=arrival_radius,
                region=region,
                verified=verified,
                metadata=dict(metadata),
            )
            for term in item.terms:
                normalized = normalize_text(term)
                if not normalized:
                    raise LandmarkError(f"landmark {landmark_id!r} contains an empty term")
                owner = seen_terms.get(normalized)
                if owner is not None and owner != landmark_id:
                    raise LandmarkError(
                        f"ambiguous normalized landmark term {term!r}: {owner!r} and {landmark_id!r}"
                    )
                seen_terms[normalized] = landmark_id
            items.append(item)
        return cls(
            map_id=map_id,
            map_generation=map_generation,
            frame_id=frame_id,
            landmarks=tuple(items),
        )

    def resolve(
        self,
        utterance: str,
        *,
        expected_map_id: str,
        expected_generation: int,
        require_verified: bool = True,
    ) -> Landmark:
        query = normalize_text(utterance)
        if not query:
            raise LandmarkError("landmark name is empty")
        if self.map_id != expected_map_id:
            raise LandmarkError(
                f"landmark map_id {self.map_id!r} does not match active map {expected_map_id!r}"
            )
        if self.map_generation != expected_generation:
            raise LandmarkError(
                f"landmark map generation {self.map_generation} does not match active map "
                f"generation {expected_generation}"
            )

        matches: list[tuple[int, Landmark, str]] = []
        marker_matches: list[tuple[int, Landmark, str]] = []
        for item in self.landmarks:
            for term in item.terms:
                normalized = normalize_text(term)
                if normalized and normalized in query:
                    target = matches if item.navigable else marker_matches
                    target.append((len(normalized), item, term))
        if not matches:
            if marker_matches:
                markers = sorted({item.name for _, item, _ in marker_matches})
                raise LandmarkError(
                    f"matched non-navigation marker(s) {markers}; "
                    "markers cannot be dispatched as navigation goals"
                )
            raise LandmarkError(f"unknown semantic landmark in {utterance!r}")
        # Mentioning two different saved places is not made safe by choosing
        # the longer spelling. A speech command such as "先去门口再去售货机"
        # requires an explicit multi-goal policy, which this single-goal
        # capability intentionally does not implement.
        matched = {item.id: item for _, item, _ in matches}
        if len(matched) != 1:
            raise LandmarkError(
                f"ambiguous semantic landmark in {utterance!r}: {sorted(matched)}"
            )
        item = next(iter(matched.values()))
        if require_verified and not item.verified:
            raise LandmarkError(
                f"landmark {item.name!r} has no physically verified approach pose"
            )
        return item
