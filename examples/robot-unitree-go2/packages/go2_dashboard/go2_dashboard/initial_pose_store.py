"""Map-generation-bound persistence for operator localization seeds.

The store is deliberately ROS-independent.  It never creates a navigation
goal or a chassis command; it only validates and persists the operator's
``PoseWithCovarianceStamped`` localization estimate next to the immutable map
library.  A saved seed is usable only while the live ``(map_id, generation)``
tuple matches exactly and the mapping service is in localization mode.
"""

from __future__ import annotations

import copy
import math
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import re
import shutil
import threading
from typing import Any, Mapping

import yaml


SCHEMA = "robonix-go2-operator-initial-pose-v1"
UINT64_MAX = (1 << 64) - 1
_MAP_ID = re.compile(r"^[A-Za-z0-9._-]{1,160}$")


class InitialPoseError(RuntimeError):
    """The localization seed cannot be saved or restored safely."""


@dataclass(frozen=True)
class MapIdentity:
    map_id: str
    mode: str
    generation: int


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise InitialPoseError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise InitialPoseError(f"{label} must be a finite number")
    return result


def _map_id(value: Any) -> str:
    result = value.strip() if isinstance(value, str) else ""
    if not _MAP_ID.fullmatch(result) or result in {".", ".."}:
        raise InitialPoseError("map_id is empty or outside the safe map-id grammar")
    return result


def _generation(value: Any, label: str = "generation") -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise InitialPoseError(f"{label} must be a uint64 integer")
    if value < 0 or value > UINT64_MAX:
        raise InitialPoseError(f"{label} must be within uint64 range")
    return value


def _timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")


def normalize_pose(value: Mapping[str, Any], *, frame_id: str = "map") -> dict[str, Any]:
    """Validate and copy a bounded PoseWithCovarianceStamped-like mapping."""

    if str(value.get("frame_id", "")) != frame_id:
        raise InitialPoseError(f"initial pose frame_id must exactly match {frame_id!r}")
    position = value.get("position")
    orientation = value.get("orientation")
    covariance = value.get("covariance")
    if not isinstance(position, Mapping) or not isinstance(orientation, Mapping):
        raise InitialPoseError("initial pose position and orientation are required")
    if not isinstance(covariance, (list, tuple)) or len(covariance) != 36:
        raise InitialPoseError("initial pose covariance must contain exactly 36 values")
    normalized_position = {
        key: _finite(position.get(key, 0.0), f"position.{key}")
        for key in ("x", "y", "z")
    }
    normalized_orientation = {
        key: _finite(orientation.get(key), f"orientation.{key}")
        for key in ("x", "y", "z", "w")
    }
    norm = math.sqrt(sum(component * component for component in normalized_orientation.values()))
    if norm < 1e-9 or abs(norm - 1.0) > 0.02:
        raise InitialPoseError("initial pose quaternion must be normalized")
    normalized_covariance = [
        _finite(entry, f"covariance[{index}]")
        for index, entry in enumerate(covariance)
    ]
    return {
        "frame_id": frame_id,
        "position": normalized_position,
        "orientation": normalized_orientation,
        "covariance": normalized_covariance,
    }


class InitialPoseStore:
    """Persist and restore one operator seed per exact map generation."""

    def __init__(self, maps_dir: str | Path, *, frame_id: str = "map") -> None:
        root = Path(maps_dir)
        if not root.is_absolute():
            raise InitialPoseError("maps_dir must be absolute")
        self._root = root
        self._frame_id = frame_id
        self._lock = threading.RLock()
        self._identity: MapIdentity | None = None
        self._error = "waiting for a named MapLifecycle sample"
        self._saved: dict[str, Any] | None = None
        self._last_action = "idle"

    def _map_dir(self, map_id: str) -> Path:
        return self._root / _map_id(map_id)

    def _sidecar(self, map_id: str) -> Path:
        return self._root / f"{_map_id(map_id)}.initial-pose.operator.yaml"

    def _verify_artifact(self, identity: MapIdentity) -> None:
        map_dir = self._map_dir(identity.map_id)
        if not map_dir.is_dir() or not (map_dir / "rtabmap.db").is_file():
            raise InitialPoseError(
                f"saved map artifact is missing for {identity.map_id!r}"
            )
        generation_path = map_dir / "generation"
        try:
            disk_generation = int(generation_path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError) as error:
            raise InitialPoseError(
                f"saved map generation is unreadable for {identity.map_id!r}"
            ) from error
        if disk_generation != identity.generation:
            raise InitialPoseError(
                "live MapLifecycle generation does not match the saved map artifact"
            )

    def _read_sidecar(self, identity: MapIdentity) -> dict[str, Any] | None:
        path = self._sidecar(identity.map_id)
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None
        except (OSError, yaml.YAMLError) as error:
            raise InitialPoseError(f"initial pose sidecar is unreadable: {error}") from error
        if not isinstance(document, Mapping) or document.get("schema") != SCHEMA:
            raise InitialPoseError("initial pose sidecar has an unsupported schema")
        if _map_id(document.get("map_id")) != identity.map_id:
            raise InitialPoseError("initial pose sidecar map_id does not match live map")
        if _generation(document.get("map_generation"), "map_generation") != identity.generation:
            raise InitialPoseError(
                "saved initial pose belongs to a different map generation"
            )
        pose = document.get("pose")
        if not isinstance(pose, Mapping):
            raise InitialPoseError("initial pose sidecar has no pose")
        covariance_hint = document.get("covariance_hint") or {}
        covariance = pose.get("covariance")
        if covariance is None:
            xy_variance = _finite(covariance_hint.get("xy_variance", 0.25), "xy_variance")
            yaw_variance = _finite(covariance_hint.get("yaw_variance", 0.06853891909122467), "yaw_variance")
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = xy_variance
            covariance[35] = yaw_variance
        normalized = normalize_pose(
            {
                "frame_id": document.get("frame_id"),
                "position": pose.get("position"),
                "orientation": pose.get("orientation"),
                "covariance": covariance,
            },
            frame_id=self._frame_id,
        )
        return {
            **normalized,
            "map_id": identity.map_id,
            "generation": identity.generation,
            "source": copy.deepcopy(document.get("source") or {}),
            "path": str(path),
        }

    def observe_lifecycle(self, map_id: Any, mode: Any, generation: Any) -> dict[str, Any]:
        with self._lock:
            self._saved = None
            try:
                identity = MapIdentity(
                    map_id=_map_id(map_id),
                    mode=str(mode).strip().casefold(),
                    generation=_generation(generation),
                )
                if identity.mode not in {"mapping", "localization"}:
                    raise InitialPoseError("MapLifecycle mode is unsupported")
                self._verify_artifact(identity)
                self._identity = identity
                self._saved = self._read_sidecar(identity)
                self._error = "" if self._saved else "no operator initial pose saved for this map generation"
            except InitialPoseError as error:
                self._identity = None
                self._error = str(error)
            return self.status()

    def save_operator_pose(self, pose: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            identity = self._require_localization_identity()
            normalized = normalize_pose(pose, frame_id=self._frame_id)
            path = self._sidecar(identity.map_id)
            if path.exists():
                archive = path.with_name(
                    f"{identity.map_id}.initial-pose.operator.{_timestamp()}.yaml"
                )
                shutil.copy2(path, archive)
            document = {
                "schema": SCHEMA,
                "map_id": identity.map_id,
                "map_generation": identity.generation,
                "frame_id": self._frame_id,
                "pose": {
                    "position": normalized["position"],
                    "orientation": normalized["orientation"],
                    "covariance": normalized["covariance"],
                },
                "source": {
                    "kind": "operator_initialpose_observed_by_dashboard",
                    "captured_at": datetime.now(timezone.utc).isoformat(),
                },
                "use": {
                    "note": (
                        "Localization seed only. Restore only when the robot is "
                        "physically at the same marked start pose. This is not a "
                        "navigation goal or motion command."
                    )
                },
            }
            temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}-{threading.get_ident()}")
            try:
                temporary.write_text(
                    yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
                    encoding="utf-8",
                )
                os.replace(temporary, path)
            finally:
                try:
                    temporary.unlink()
                except FileNotFoundError:
                    pass
            self._saved = self._read_sidecar(identity)
            self._error = ""
            self._last_action = "saved_operator_pose"
            return self.status()

    def restore_pose(self, *, map_id: Any, generation: Any) -> dict[str, Any]:
        with self._lock:
            identity = self._require_localization_identity()
            if _map_id(map_id) != identity.map_id or _generation(generation) != identity.generation:
                raise InitialPoseError(
                    "restore confirmation does not match live map_id and generation"
                )
            saved = self._read_sidecar(identity)
            if saved is None:
                raise InitialPoseError("no saved operator initial pose is available")
            self._saved = saved
            self._error = ""
            self._last_action = "restore_requested"
            return copy.deepcopy(saved)

    def reset(self, *, confirm_map_id: Any, generation: Any) -> dict[str, Any]:
        with self._lock:
            identity = self._require_identity()
            if _map_id(confirm_map_id) != identity.map_id or _generation(generation) != identity.generation:
                raise InitialPoseError(
                    "reset confirmation does not match live map_id and generation"
                )
            path = self._sidecar(identity.map_id)
            if not path.is_file():
                raise InitialPoseError("no active initial pose sidecar exists")
            disabled = path.with_name(
                f"{identity.map_id}.initial-pose.operator.disabled.{_timestamp()}.yaml"
            )
            os.replace(path, disabled)
            self._saved = None
            self._error = "operator initial pose reset; archived recoverably"
            self._last_action = "reset_archived"
            return self.status()

    def _require_identity(self) -> MapIdentity:
        if self._identity is None:
            raise InitialPoseError(self._error or "live map identity is unavailable")
        self._verify_artifact(self._identity)
        return self._identity

    def _require_localization_identity(self) -> MapIdentity:
        identity = self._require_identity()
        if identity.mode != "localization":
            raise InitialPoseError("initial pose persistence requires localization mode")
        return identity

    def status(self) -> dict[str, Any]:
        with self._lock:
            identity = self._identity
            saved = self._saved
            return {
                "enabled": True,
                "map_id": identity.map_id if identity else "",
                "mode": identity.mode if identity else "",
                "generation": identity.generation if identity else None,
                "saved": saved is not None,
                "saved_pose": (
                    None
                    if saved is None
                    else {
                        "frame_id": saved["frame_id"],
                        "position": copy.deepcopy(saved["position"]),
                        "orientation": copy.deepcopy(saved["orientation"]),
                        "source": copy.deepcopy(saved.get("source") or {}),
                    }
                ),
                "sidecar_path": "" if saved is None else str(saved["path"]),
                "error": self._error,
                "last_action": self._last_action,
                "localization_only": True,
                "motion_command": False,
            }
