#!/usr/bin/env python3
"""Validate external D435i ROS streams and register them with Robonix Atlas."""

from __future__ import annotations

import math
import threading
from typing import Any

from robonix_api import Deferred, Err, Ok, Primitive

from .observer import observe_external_d435i


provider = Primitive(id="go2_d435i", namespace="robonix/primitive/camera")

# Keep the runtime table literal and unconditional. The package manifest
# promises exactly these three data-plane endpoints after the quality gate.
DATA_CAPABILITIES = (
    ("robonix/primitive/camera/rgb", "rgb_topic", "best_effort"),
    ("robonix/primitive/camera/depth", "depth_topic", "best_effort"),
    ("robonix/primitive/camera/intrinsics", "camera_info_topic", "best_effort"),
)

_lock = threading.Lock()
_config: dict[str, Any] = {}
_declared = False
_active = False
_last_quality_evidence: dict[str, Any] = {}
_quality_observer = observe_external_d435i


def _absolute_topic(value: Any, name: str, default: str) -> str:
    topic = str(value or default)
    if (
        not topic.startswith("/")
        or topic == "/"
        or topic.endswith("/")
        or "//" in topic
        or any(character.isspace() for character in topic)
        or len(topic) > 255
    ):
        raise ValueError(f"{name} must be one normalized absolute ROS topic")
    return topic


def _relative_frame(value: Any, name: str, default: str) -> str:
    frame = str(value or default)
    if (
        not frame
        or frame.startswith("/")
        or frame.endswith("/")
        or "//" in frame
        or frame in {".", ".."}
        or any(character.isspace() for character in frame)
        or len(frame) > 255
    ):
        raise ValueError(f"{name} must be one normalized relative TF frame")
    return frame


def _bounded_float(
    value: Any,
    name: str,
    default: float,
    minimum: float,
    maximum: float,
    *,
    minimum_inclusive: bool = False,
) -> float:
    result = float(default if value is None else value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    below = result < minimum if minimum_inclusive else result <= minimum
    if below or result > maximum:
        bracket = "[" if minimum_inclusive else "("
        raise ValueError(f"{name} must be in {bracket}{minimum}, {maximum}]")
    return result


def _normalize_config(config: dict[str, Any]) -> dict[str, Any]:
    if config.get("source_mode") != "external":
        raise ValueError("source_mode must be exactly 'external'")

    rgb_topic = _absolute_topic(
        config.get("rgb_topic"),
        "rgb_topic",
        "/go2/d435i/color/image_raw",
    )
    depth_topic = _absolute_topic(
        config.get("depth_topic"),
        "depth_topic",
        "/go2/d435i/aligned_depth_to_color/image_raw",
    )
    camera_info_topic = _absolute_topic(
        config.get("camera_info_topic"),
        "camera_info_topic",
        "/go2/d435i/color/camera_info",
    )
    if len({rgb_topic, depth_topic, camera_info_topic}) != 3:
        raise ValueError("RGB, depth, and CameraInfo topics must be distinct")

    rgb_frame = _relative_frame(
        config.get("rgb_frame"),
        "rgb_frame",
        "d435i_color_optical_frame",
    )
    depth_frame = _relative_frame(
        config.get("depth_frame"),
        "depth_frame",
        "d435i_color_optical_frame",
    )
    if depth_frame != rgb_frame:
        raise ValueError(
            "aligned depth and RGB must use the same configured optical frame"
        )

    sentinel_timeout_s = _bounded_float(
        config.get("sentinel_timeout_s"),
        "sentinel_timeout_s",
        30.0,
        1.0,
        60.0,
    )
    quality_window_s = _bounded_float(
        config.get("quality_window_s"),
        "quality_window_s",
        5.0,
        1.0,
        30.0,
    )
    if quality_window_s >= sentinel_timeout_s:
        raise ValueError("quality_window_s must be shorter than sentinel_timeout_s")

    normalized = {
        "source_mode": "external",
        "rgb_topic": rgb_topic,
        "depth_topic": depth_topic,
        "camera_info_topic": camera_info_topic,
        "rgb_frame": rgb_frame,
        "depth_frame": depth_frame,
        "sentinel_timeout_s": sentinel_timeout_s,
        "quality_window_s": quality_window_s,
        "min_rate_hz": _bounded_float(
            config.get("min_rate_hz"),
            "min_rate_hz",
            5.0,
            0.1,
            30.0,
        ),
        "max_stamp_age_s": _bounded_float(
            config.get("max_stamp_age_s"),
            "max_stamp_age_s",
            0.50,
            0.05,
            2.0,
        ),
        "max_future_skew_s": _bounded_float(
            config.get("max_future_skew_s"),
            "max_future_skew_s",
            0.05,
            0.0,
            0.10,
            minimum_inclusive=True,
        ),
        "max_rgb_depth_skew_s": _bounded_float(
            config.get("max_rgb_depth_skew_s"),
            "max_rgb_depth_skew_s",
            0.05,
            0.001,
            0.20,
        ),
    }
    return normalized


@provider.on_init
def initialize(config: dict[str, Any]):
    global _config
    if not isinstance(config, dict):
        return Err("D435i config must be a JSON object")
    with _lock:
        if _active:
            return Err("cannot reinitialize while the D435i registrar is active")
    try:
        normalized = _normalize_config(config)
    except (TypeError, ValueError) as error:
        return Err(str(error))
    if _declared and _config and normalized != _config:
        return Err("D435i endpoints were already declared; restart to change config")
    _config = normalized
    return Ok()


@provider.on_activate
def activate():
    global _active, _declared, _last_quality_evidence
    if not _config:
        return Err("D435i registrar was not initialized")
    with _lock:
        if _active:
            return Deferred("D435i registrar is already active")

    try:
        result = _quality_observer(_config)
    except Exception as error:
        return Err(f"D435i quality gate failed: {type(error).__name__}: {error}")
    _last_quality_evidence = dict(result.evidence)
    if not result.ok:
        return Err(f"D435i external stream quality gate failed: {result.detail}")

    try:
        if not _declared:
            for contract, topic_key, qos in DATA_CAPABILITIES:
                provider.declare_ros2_topic(
                    contract,
                    _config[topic_key],
                    qos=qos,
                )
            _declared = True
        with _lock:
            _active = True
    except Exception as error:
        return Err(f"D435i Atlas registration failed: {type(error).__name__}: {error}")
    return Ok()


@provider.on_deactivate
def deactivate():
    global _active
    with _lock:
        _active = False
    return Ok()


@provider.on_shutdown
def shutdown():
    global _active
    with _lock:
        _active = False
    return Ok()


if __name__ == "__main__":
    provider.run()
