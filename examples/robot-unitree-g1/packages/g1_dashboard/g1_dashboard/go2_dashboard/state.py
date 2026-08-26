"""Thread-safe state shared by the ROS observer and HTTP server."""

from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping


SEMANTIC_STATUSES = frozenset(
    {
        "idle",
        "received",
        "resolving",
        "resolved",
        "navigating",
        "succeeded",
        "canceled",
        "failed",
    }
)

VOICE_STATUSES = frozenset(
    {
        "disabled",
        "idle",
        "accepted",
        "connecting",
        "liaison",
        "recording",
        "asr",
        "recognized",
        "authorized",
        "pilot",
        "completed",
        "failed",
    }
)

VOICE_EXECUTION_MODES = frozenset({"preview", "live"})


@dataclass(frozen=True)
class TopicSpec:
    label: str
    topic: str
    stale_after_s: float | None


DEFAULT_TOPIC_SPECS = {
    "camera": TopicSpec("Go2 内置相机", "/camera/color/image_raw", 2.0),
    "d435i_color": TopicSpec(
        "D435i 彩色", "/go2/d435i/color/image_raw", 2.0
    ),
    "d435i_depth": TopicSpec(
        "D435i 对齐深度",
        "/go2/d435i/aligned_depth_to_color/image_raw",
        2.0,
    ),
    "d435i_info": TopicSpec(
        "D435i 标定", "/go2/d435i/color/camera_info", 10.0
    ),
    "laser_scan": TopicSpec("二维雷达", "/scanner/scan", 2.0),
    "point_cloud": TopicSpec("三维雷达", "/scanner/cloud", 2.0),
    # OccupancyGrid is a transient-local, event-driven snapshot.  A mapping
    # engine need not republish an unchanged grid while the robot is still, so
    # receipt age alone cannot invalidate a successfully decoded map.
    "map": TopicSpec("地图", "/map", None),
    "odom": TopicSpec("里程计", "/odom", 1.5),
    "pose_map": TopicSpec("机器人地图位姿", "/robonix/map/pose", 1.5),
    "nav_status": TopicSpec(
        "导航任务", "/navigate_to_pose/_action/status", 8.0
    ),
    "map_lifecycle": TopicSpec(
        "地图代际", "/robonix/map/lifecycle", None
    ),
    "initial_pose_input": TopicSpec("初始位姿输入", "/initialpose", None),
}


DASHBOARD_PROFILES = frozenset({"integrated", "readonly-diagnostic"})


def dashboard_profile(value: Any) -> dict[str, Any]:
    """Return explicit UI scope metadata without inferring readiness.

    The diagnostic profile is intentionally stronger than a cosmetic label:
    consumers can use these machine-readable fields to avoid presenting raw
    receive-side telemetry as a mapping or navigation acceptance result.
    """

    normalized = str(value or "integrated").strip().lower()
    if normalized not in DASHBOARD_PROFILES:
        raise ValueError(f"unsupported dashboard profile: {normalized}")
    if normalized == "readonly-diagnostic":
        return {
            "id": normalized,
            "diagnostic_only": True,
            "navigation_stack_started": False,
            "navigation_ready": False,
            "source_time_trusted": False,
            "warning": "READ-ONLY DIAGNOSTIC / NOT NAVIGATION READY",
        }
    return {
        "id": normalized,
        "diagnostic_only": False,
        "navigation_stack_started": None,
        "navigation_ready": None,
        "source_time_trusted": None,
        "warning": "",
    }


def _bounded_text(value: Any, limit: int) -> str:
    return str(value or "").strip()[:limit]


class DashboardState:
    """Own the last bounded preview and health metadata for every input."""

    def __init__(
        self,
        topic_specs: Mapping[str, TopicSpec] | None = None,
        monotonic_fn: Callable[[], float] = time.monotonic,
        wall_fn: Callable[[], float] = time.time,
        deployment_profile: str = "integrated",
    ) -> None:
        self._lock = threading.RLock()
        self._monotonic = monotonic_fn
        self._wall = wall_fn
        self._started_monotonic = float(monotonic_fn())
        self._topic_specs = dict(topic_specs or DEFAULT_TOPIC_SPECS)
        self._profile = dashboard_profile(deployment_profile)
        self._topic_health = {
            key: {
                "last_seen_monotonic": None,
                "last_seen_at": None,
                "source_age_s_at_receipt": None,
                "sequence": 0,
                "frame_id": "",
                "error": "",
            }
            for key in self._topic_specs
        }
        self._payload: dict[str, Any] = {
            "camera": None,
            "camera_quality": None,
            "d435i_color": None,
            "d435i_color_quality": None,
            "d435i_depth": None,
            "d435i_depth_quality": None,
            "d435i_info": None,
            "lidar": None,
            "map": None,
            "pose_map": None,
            "odom": None,
            "navigation": {
                "status": "idle",
                "status_code": 0,
                "goal_id": "",
                "stamp": None,
            },
        }
        self._last_lidar_key: str | None = None
        self._camera_jpeg: dict[str, bytes | None] = {
            "camera": None,
            "d435i_color": None,
            "d435i_depth": None,
        }
        self._map_png: bytes | None = None
        self._bridge = {
            "running": False,
            "connected": False,
            "error": "尚未启动 ROS 观察线程",
        }
        self._semantic_task = {
            "revision": 0,
            "task_id": "",
            "target_name": "",
            "status": "idle",
            "message": "",
            "pose": None,
            "updated_at": float(wall_fn()),
            "source": "status-api",
            "read_only_effect": True,
        }
        self._voice = {
            "enabled": False,
            "active": False,
            "status": "disabled",
            "execution_mode": "preview",
            "session_id": "",
            "message": "浏览器语音入口未启用",
            "transcript": "",
            "intent_target": "",
            "intent_summary": "",
            "blocked_reason": "",
            "pilot_task_status": "",
            "capability_calls_observed": 0,
            "updated_at": float(wall_fn()),
            "limits": {},
            "delegates_to": "robonix/system/liaison/voice",
            "direct_robot_control": False,
            "audio_persisted": False,
        }
        self._initial_pose = {
            "enabled": False,
            "map_id": "",
            "mode": "",
            "generation": None,
            "saved": False,
            "saved_pose": None,
            "sidecar_path": "",
            "error": "initial pose persistence is not configured",
            "last_action": "disabled",
            "localization_only": True,
            "motion_command": False,
        }

    def set_bridge(
        self, *, running: bool, connected: bool, error: str = ""
    ) -> None:
        with self._lock:
            self._bridge = {
                "running": bool(running),
                "connected": bool(connected),
                "error": _bounded_text(error, 400),
            }

    def observe(
        self,
        key: str,
        payload: Any,
        *,
        frame_id: str = "",
        monotonic_now: float | None = None,
        wall_now: float | None = None,
        source_age_s: float | None = None,
    ) -> int:
        normalized_source_age: float | None = None
        if source_age_s is not None:
            if isinstance(source_age_s, bool):
                raise TypeError("source_age_s must be a finite number, not bool")
            normalized_source_age = float(source_age_s)
            if not math.isfinite(normalized_source_age):
                raise ValueError("source_age_s must be finite")
        with self._lock:
            if key not in self._topic_health:
                raise KeyError(f"unknown telemetry key: {key}")
            health = self._topic_health[key]
            health["last_seen_monotonic"] = float(
                self._monotonic() if monotonic_now is None else monotonic_now
            )
            health["last_seen_at"] = float(
                self._wall() if wall_now is None else wall_now
            )
            health["source_age_s_at_receipt"] = normalized_source_age
            health["sequence"] += 1
            health["frame_id"] = _bounded_text(frame_id, 160)
            health["error"] = ""
            if key in {"laser_scan", "point_cloud"}:
                self._last_lidar_key = key
                self._payload["lidar"] = copy.deepcopy(payload)
            elif key == "nav_status":
                self._payload["navigation"] = copy.deepcopy(payload)
            else:
                self._payload[key] = copy.deepcopy(payload)
            return int(health["sequence"])

    def note_error(self, key: str, error: Any) -> None:
        with self._lock:
            if key not in self._topic_health:
                raise KeyError(f"unknown telemetry key: {key}")
            self._topic_health[key]["error"] = _bounded_text(error, 400)

    def set_camera(
        self, jpeg: bytes, metadata: Mapping[str, Any], *, frame_id: str
    ) -> int:
        return self.set_camera_stream(
            "camera", jpeg, metadata, frame_id=frame_id
        )

    def set_camera_stream(
        self,
        key: str,
        jpeg: bytes,
        metadata: Mapping[str, Any],
        *,
        frame_id: str,
    ) -> int:
        """Store one bounded JPEG preview for an explicitly known stream."""

        if not jpeg:
            raise ValueError("camera preview is empty")
        if key not in self._camera_jpeg:
            raise KeyError(f"unknown camera stream: {key}")
        with self._lock:
            sequence = self.observe(key, dict(metadata), frame_id=frame_id)
            self._camera_jpeg[key] = bytes(jpeg)
            self._payload[key]["sequence"] = sequence
            return sequence

    def set_camera_quality(
        self, quality: Mapping[str, Any], *, key: str = "camera"
    ) -> None:
        quality_key = {
            "camera": "camera_quality",
            "d435i_color": "d435i_color_quality",
            "d435i_depth": "d435i_depth_quality",
        }.get(key)
        if quality_key is None:
            raise KeyError(f"unknown camera quality stream: {key}")
        with self._lock:
            payload = copy.deepcopy(dict(quality))
            payload["message"] = _bounded_text(payload.get("message"), 240)
            payload["api_code_semantics"] = _bounded_text(
                payload.get("api_code_semantics"), 100
            )
            payload["updated_at"] = float(self._wall())
            self._payload[quality_key] = payload

    def set_map(
        self, png: bytes, metadata: Mapping[str, Any], *, frame_id: str
    ) -> int:
        if not png:
            raise ValueError("map preview is empty")
        with self._lock:
            sequence = self.observe("map", dict(metadata), frame_id=frame_id)
            self._map_png = bytes(png)
            self._payload["map"]["sequence"] = sequence
            return sequence

    def camera_image(self, key: str = "camera") -> tuple[bytes | None, int]:
        if key not in self._camera_jpeg:
            raise KeyError(f"unknown camera stream: {key}")
        with self._lock:
            sequence = int(self._topic_health[key]["sequence"])
            return self._camera_jpeg[key], sequence

    def map_image(self) -> tuple[bytes | None, int]:
        with self._lock:
            sequence = int(self._topic_health["map"]["sequence"])
            return self._map_png, sequence

    def semantic_task(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._semantic_task)

    def configure_voice(
        self,
        enabled: bool,
        limits: Mapping[str, Any],
        execution_mode: str = "preview",
    ) -> dict[str, Any]:
        """Declare the optional Liaison-only browser voice surface."""

        normalized_mode = _bounded_text(execution_mode, 16).lower()
        if normalized_mode not in VOICE_EXECUTION_MODES:
            raise ValueError("browser voice execution mode must be preview or live")

        with self._lock:
            self._voice.update(
                {
                    "enabled": bool(enabled),
                    "active": False,
                    "status": "idle" if enabled else "disabled",
                    "execution_mode": normalized_mode,
                    "session_id": "",
                    "message": (
                        "等待浏览器按键录音"
                        if enabled
                        else "浏览器语音入口未启用"
                    ),
                    "transcript": "",
                    "intent_target": "",
                    "intent_summary": "",
                    "blocked_reason": "",
                    "pilot_task_status": "",
                    "capability_calls_observed": 0,
                    "updated_at": float(self._wall()),
                    "limits": copy.deepcopy(dict(limits)),
                }
            )
            return copy.deepcopy(self._voice)

    def voice_status(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._voice)

    def set_initial_pose_status(self, status: Mapping[str, Any]) -> None:
        """Expose bounded localization-seed state without embedding file data."""

        with self._lock:
            update = copy.deepcopy(dict(status))
            update["map_id"] = _bounded_text(update.get("map_id"), 160)
            update["mode"] = _bounded_text(update.get("mode"), 32)
            update["sidecar_path"] = _bounded_text(
                update.get("sidecar_path"), 4096
            )
            update["error"] = _bounded_text(update.get("error"), 500)
            update["last_action"] = _bounded_text(
                update.get("last_action"), 80
            )
            self._initial_pose = update

    def initial_pose_status(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._initial_pose)

    def update_voice(
        self,
        *,
        session_id: str,
        status: str,
        message: str,
        transcript: str | None = None,
        active: bool,
        intent_target: str | None = None,
        intent_summary: str | None = None,
        blocked_reason: str | None = None,
        pilot_task_status: str | None = None,
        capability_calls_observed: int | None = None,
    ) -> dict[str, Any]:
        normalized_status = _bounded_text(status, 32).lower()
        if normalized_status not in VOICE_STATUSES - {"disabled", "idle"}:
            raise ValueError(f"unsupported browser voice status: {normalized_status}")
        normalized_session = _bounded_text(session_id, 64)
        if not normalized_session:
            raise ValueError("browser voice session id is required")
        normalized_capability_calls: int | None = None
        if capability_calls_observed is not None:
            normalized_capability_calls = int(capability_calls_observed)
            if not 0 <= normalized_capability_calls <= 10_000:
                raise ValueError("capability call count is out of range")
        with self._lock:
            if not self._voice["enabled"]:
                raise ValueError("browser voice is disabled")
            current_session = str(self._voice["session_id"])
            if current_session and current_session != normalized_session:
                if self._voice["active"] or self._voice["status"] not in {
                    "completed",
                    "failed",
                }:
                    raise ValueError("stale browser voice session update")
            update = {
                "active": bool(active),
                "status": normalized_status,
                "session_id": normalized_session,
                "message": _bounded_text(message, 400),
                "updated_at": float(self._wall()),
            }
            optional_text = (
                ("transcript", transcript, 300),
                ("intent_target", intent_target, 128),
                ("intent_summary", intent_summary, 300),
                ("blocked_reason", blocked_reason, 500),
                ("pilot_task_status", pilot_task_status, 64),
            )
            for key, value, limit in optional_text:
                if value is not None:
                    update[key] = _bounded_text(value, limit)
            if normalized_capability_calls is not None:
                previous_calls = (
                    0
                    if current_session != normalized_session
                    else int(self._voice["capability_calls_observed"])
                )
                update["capability_calls_observed"] = max(
                    previous_calls,
                    normalized_capability_calls,
                )
            self._voice.update(update)
            return copy.deepcopy(self._voice)

    def update_semantic_task(self, update: Mapping[str, Any]) -> dict[str, Any]:
        status = _bounded_text(update.get("status"), 32).lower()
        if status not in SEMANTIC_STATUSES:
            raise ValueError(f"unsupported semantic task status: {status}")
        pose = update.get("pose")
        normalized_pose = None
        if pose is not None:
            normalized_pose = {
                "frame_id": _bounded_text(pose.get("frame_id", "map"), 80)
                or "map",
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose["yaw"]),
            }
            if not all(
                math.isfinite(value)
                for key, value in normalized_pose.items()
                if key != "frame_id"
            ):
                raise ValueError("semantic pose contains a non-finite value")
        with self._lock:
            self._semantic_task = {
                "revision": int(self._semantic_task["revision"]) + 1,
                "task_id": _bounded_text(update.get("task_id"), 128),
                "target_name": _bounded_text(update.get("target_name"), 128),
                "status": status,
                "message": _bounded_text(update.get("message"), 500),
                "pose": normalized_pose,
                "updated_at": float(self._wall()),
                "source": "status-api",
                "read_only_effect": True,
            }
            return copy.deepcopy(self._semantic_task)

    def snapshot(self) -> dict[str, Any]:
        now = float(self._monotonic())
        wall_now = float(self._wall())
        with self._lock:
            topics: dict[str, Any] = {}
            for key, spec in self._topic_specs.items():
                health = self._topic_health[key]
                last_seen = health["last_seen_monotonic"]
                receipt_age = (
                    None if last_seen is None else max(0.0, now - float(last_seen))
                )
                source_age_at_receipt = health["source_age_s_at_receipt"]
                current_source_age = (
                    None
                    if receipt_age is None or source_age_at_receipt is None
                    else float(source_age_at_receipt) + receipt_age
                )
                # A source timestamp ahead of the receiver clock must not make
                # an already-stale observation become fresh as wall time catches
                # up.  Keep the signed delta above for diagnostics, but use a
                # conservative monotonic upper bound for the health decision.
                source_freshness_bound = (
                    None
                    if receipt_age is None or source_age_at_receipt is None
                    else abs(float(source_age_at_receipt)) + receipt_age
                )
                age = (
                    receipt_age
                    if source_freshness_bound is None
                    else max(receipt_age, source_freshness_bound)
                )
                if health["error"]:
                    state = "error"
                elif age is None:
                    state = "missing"
                elif spec.stale_after_s is None:
                    state = "fresh"
                elif age <= spec.stale_after_s:
                    state = "fresh"
                else:
                    state = "stale"
                topics[key] = {
                    "label": spec.label,
                    "topic": spec.topic,
                    "state": state,
                    "age_s": None if age is None else round(age, 3),
                    "receipt_age_s": (
                        None if receipt_age is None else round(receipt_age, 3)
                    ),
                    "source_age_s": (
                        None
                        if current_source_age is None
                        else round(current_source_age, 3)
                    ),
                    "stale_after_s": spec.stale_after_s,
                    "last_seen_at": health["last_seen_at"],
                    "sequence": health["sequence"],
                    "frame_id": health["frame_id"],
                    "error": health["error"],
                }
            result = {
                "schema_version": 1,
                "generated_at": wall_now,
                "uptime_s": round(max(0.0, now - self._started_monotonic), 3),
                "read_only": not bool(self._voice["enabled"]),
                "telemetry_read_only": True,
                "profile": copy.deepcopy(self._profile),
                "bridge": copy.deepcopy(self._bridge),
                "topics": topics,
                "camera": copy.deepcopy(self._payload["camera"]),
                "camera_quality": copy.deepcopy(self._payload["camera_quality"]),
                "cameras": {
                    "go2": {
                        "color": copy.deepcopy(self._payload["camera"]),
                        "quality": copy.deepcopy(
                            self._payload["camera_quality"]
                        ),
                    },
                    "d435i": {
                        "color": copy.deepcopy(self._payload["d435i_color"]),
                        "color_quality": copy.deepcopy(
                            self._payload["d435i_color_quality"]
                        ),
                        "depth": copy.deepcopy(self._payload["d435i_depth"]),
                        "depth_quality": copy.deepcopy(
                            self._payload["d435i_depth_quality"]
                        ),
                        "camera_info": copy.deepcopy(
                            self._payload["d435i_info"]
                        ),
                    },
                },
                "lidar": copy.deepcopy(self._payload["lidar"]),
                "lidar_source": self._last_lidar_key,
                "map": copy.deepcopy(self._payload["map"]),
                "pose_map": copy.deepcopy(self._payload["pose_map"]),
                "odom": copy.deepcopy(self._payload["odom"]),
                "navigation": copy.deepcopy(self._payload["navigation"]),
                "initial_pose": copy.deepcopy(self._initial_pose),
                "semantic_task": copy.deepcopy(self._semantic_task),
                "voice": copy.deepcopy(self._voice),
            }
        return result
