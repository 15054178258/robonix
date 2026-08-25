"""Process ownership and validated configuration for the dashboard provider.

This module has no Robonix or ROS imports so its lifecycle behavior can be
tested offline.  It starts exactly one direct child and only signals the
process object that it created.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Callable, Mapping
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener

from .voice_gateway import VoiceConfig


_SAFE_ROS_NAME = re.compile(r"^[A-Za-z0-9_~/.-]+$")
_LOG_LEVELS = frozenset({"critical", "error", "warning", "info", "debug"})
_SENSITIVE_ENV_MARKERS = (
    "API_KEY",
    "ACCESS_KEY",
    "TOKEN",
    "CREDENTIAL",
    "PASSWORD",
    "PRIVATE_KEY",
    "SECRET",
)


def _text(value: Any, key: str, *, limit: int = 200) -> str:
    result = str(value).strip()
    if not result or len(result) > limit or any(char.isspace() for char in result):
        raise ValueError(f"invalid {key}")
    return result


def _topic(value: Any, key: str) -> str:
    result = _text(value, key)
    if not result.startswith("/") or not _SAFE_ROS_NAME.fullmatch(result):
        raise ValueError(f"{key} must be an absolute ROS topic")
    return result


def _frame(value: Any, key: str) -> str:
    result = _text(value, key)
    if result.startswith("/") or not _SAFE_ROS_NAME.fullmatch(result):
        raise ValueError(f"invalid {key}")
    return result


def _bounded_float(value: Any, key: str, minimum: float, maximum: float) -> float:
    result = float(value)
    if not math.isfinite(result) or result < minimum or result > maximum:
        raise ValueError(f"{key} must be between {minimum} and {maximum}")
    return result


def _port(value: Any) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError("port must be an integer") from error
    if result < 1 or result > 65535:
        raise ValueError("port must be between 1 and 65535")
    return result


def _explicit_switch(value: Any, key: str) -> bool:
    if value is False or value == 0 or value == "0":
        return False
    if value is True or value == 1 or value == "1":
        return True
    raise ValueError(f"{key} must be exactly 0 or 1")


def _public_url(value: Any) -> str:
    result = str(value or "").strip().rstrip("/")
    if not result:
        return ""
    parsed = urlsplit(result)
    if (
        parsed.scheme not in {"http", "https"}
        or not parsed.hostname
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
    ):
        raise ValueError("public_url must be an HTTP(S) URL without credentials")
    return result


def _absolute_path(value: Any, key: str, *, allow_empty: bool = False) -> str:
    result = str(value or "").strip()
    if not result and allow_empty:
        return ""
    if (
        not result
        or len(result) > 4096
        or "\x00" in result
        or not Path(result).is_absolute()
    ):
        raise ValueError(f"{key} must be an absolute path")
    return result


@dataclass(frozen=True)
class DashboardConfig:
    host: str = "127.0.0.1"
    port: int = 8092
    public_url: str = ""
    camera_topic: str = "/camera/color/image_raw"
    d435i_color_topic: str = "/go2/d435i/color/image_raw"
    d435i_depth_topic: str = "/go2/d435i/aligned_depth_to_color/image_raw"
    d435i_camera_info_topic: str = "/go2/d435i/color/camera_info"
    scan_topic: str = "/scanner/scan"
    cloud_topic: str = "/scanner/cloud"
    map_topic: str = "/map"
    pose_topic: str = "/robonix/map/pose"
    odom_topic: str = "/odom"
    nav_status_topic: str = "/navigate_to_pose/_action/status"
    initial_pose_topic: str = "/initialpose"
    map_lifecycle_topic: str = "/robonix/map/lifecycle"
    initial_pose_maps_dir: str = ""
    initial_pose_auto_restore: bool = False
    map_frame: str = "map"
    base_frame: str = "base_link"
    log_level: str = "info"
    startup_timeout_s: float = 8.0
    stop_timeout_s: float = 5.0
    browser_voice_enabled: bool = False
    liaison_endpoint: str = "127.0.0.1:50081"
    audio_bridge_url: str = "ws://127.0.0.1:60002/client"
    browser_mic_provider: str = "audio_client_bridge"

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any] | None) -> "DashboardConfig":
        values = dict(raw or {})
        if "image_topic" in values:
            if (
                "camera_topic" in values
                and str(values["camera_topic"]) != str(values["image_topic"])
            ):
                raise ValueError("camera_topic and image_topic disagree")
            values["camera_topic"] = values.pop("image_topic")
        allowed = set(cls.__dataclass_fields__)
        unknown = sorted(set(values) - allowed)
        if unknown:
            raise ValueError(f"unknown dashboard config keys: {', '.join(unknown)}")
        defaults = cls()
        host = _text(values.get("host", defaults.host), "host")
        if host != "127.0.0.1":
            raise ValueError("host must be 127.0.0.1; use an SSH tunnel remotely")
        log_level = _text(
            values.get("log_level", defaults.log_level), "log_level"
        ).lower()
        if log_level not in _LOG_LEVELS:
            raise ValueError("invalid log_level")
        voice = VoiceConfig(
            enabled=_explicit_switch(
                values.get(
                    "browser_voice_enabled", defaults.browser_voice_enabled
                ),
                "browser_voice_enabled",
            ),
            liaison_endpoint=str(
                values.get("liaison_endpoint", defaults.liaison_endpoint)
            ).strip(),
            audio_bridge_url=str(
                values.get("audio_bridge_url", defaults.audio_bridge_url)
            ).strip(),
            mic_provider_id=str(
                values.get(
                    "browser_mic_provider", defaults.browser_mic_provider
                )
            ).strip(),
        )
        return cls(
            host=host,
            port=_port(values.get("port", defaults.port)),
            public_url=_public_url(values.get("public_url", defaults.public_url)),
            camera_topic=_topic(
                values.get("camera_topic", defaults.camera_topic), "camera_topic"
            ),
            d435i_color_topic=_topic(
                values.get("d435i_color_topic", defaults.d435i_color_topic),
                "d435i_color_topic",
            ),
            d435i_depth_topic=_topic(
                values.get("d435i_depth_topic", defaults.d435i_depth_topic),
                "d435i_depth_topic",
            ),
            d435i_camera_info_topic=_topic(
                values.get(
                    "d435i_camera_info_topic", defaults.d435i_camera_info_topic
                ),
                "d435i_camera_info_topic",
            ),
            scan_topic=_topic(
                values.get("scan_topic", defaults.scan_topic), "scan_topic"
            ),
            cloud_topic=_topic(
                values.get("cloud_topic", defaults.cloud_topic), "cloud_topic"
            ),
            map_topic=_topic(values.get("map_topic", defaults.map_topic), "map_topic"),
            pose_topic=_topic(
                values.get("pose_topic", defaults.pose_topic), "pose_topic"
            ),
            odom_topic=_topic(
                values.get("odom_topic", defaults.odom_topic), "odom_topic"
            ),
            nav_status_topic=_topic(
                values.get("nav_status_topic", defaults.nav_status_topic),
                "nav_status_topic",
            ),
            initial_pose_topic=_topic(
                values.get("initial_pose_topic", defaults.initial_pose_topic),
                "initial_pose_topic",
            ),
            map_lifecycle_topic=_topic(
                values.get(
                    "map_lifecycle_topic", defaults.map_lifecycle_topic
                ),
                "map_lifecycle_topic",
            ),
            initial_pose_maps_dir=_absolute_path(
                values.get(
                    "initial_pose_maps_dir", defaults.initial_pose_maps_dir
                ),
                "initial_pose_maps_dir",
                allow_empty=True,
            ),
            initial_pose_auto_restore=_explicit_switch(
                values.get(
                    "initial_pose_auto_restore",
                    defaults.initial_pose_auto_restore,
                ),
                "initial_pose_auto_restore",
            ),
            map_frame=_frame(
                values.get("map_frame", defaults.map_frame), "map_frame"
            ),
            base_frame=_frame(
                values.get("base_frame", defaults.base_frame), "base_frame"
            ),
            log_level=log_level,
            startup_timeout_s=_bounded_float(
                values.get("startup_timeout_s", defaults.startup_timeout_s),
                "startup_timeout_s",
                0.5,
                30.0,
            ),
            stop_timeout_s=_bounded_float(
                values.get("stop_timeout_s", defaults.stop_timeout_s),
                "stop_timeout_s",
                0.5,
                30.0,
            ),
            browser_voice_enabled=voice.enabled,
            liaison_endpoint=voice.liaison_endpoint,
            audio_bridge_url=voice.audio_bridge_url,
            browser_mic_provider=voice.mic_provider_id,
        )

    @property
    def url(self) -> str:
        if self.public_url:
            return self.public_url
        host = self.host
        if ":" in host and not host.startswith("["):
            host = f"[{host}]"
        return f"http://{host}:{self.port}"

    @property
    def health_url(self) -> str:
        host = self.host
        if ":" in host and not host.startswith("["):
            host = f"[{host}]"
        return f"http://{host}:{self.port}/healthz"

    def child_environment(self, base: Mapping[str, str]) -> dict[str, str]:
        environment = {
            key: value
            for key, value in base.items()
            if key.upper() != "SSH_AUTH_SOCK"
            and not key.upper().startswith(("PIP_", "UV_", "NPM_"))
            and not key.upper().endswith("_PROXY")
            and not any(marker in key.upper() for marker in _SENSITIVE_ENV_MARKERS)
        }
        environment.update(
            {
                "GO2_DASHBOARD_HOST": self.host,
                "GO2_DASHBOARD_PORT": str(self.port),
                "GO2_DASHBOARD_LOG_LEVEL": self.log_level,
                "GO2_DASHBOARD_CAMERA_TOPIC": self.camera_topic,
                "GO2_DASHBOARD_D435I_COLOR_TOPIC": self.d435i_color_topic,
                "GO2_DASHBOARD_D435I_DEPTH_TOPIC": self.d435i_depth_topic,
                "GO2_DASHBOARD_D435I_CAMERA_INFO_TOPIC": (
                    self.d435i_camera_info_topic
                ),
                "GO2_DASHBOARD_SCAN_TOPIC": self.scan_topic,
                "GO2_DASHBOARD_CLOUD_TOPIC": self.cloud_topic,
                "GO2_DASHBOARD_MAP_TOPIC": self.map_topic,
                "GO2_DASHBOARD_POSE_TOPIC": self.pose_topic,
                "GO2_DASHBOARD_ODOM_TOPIC": self.odom_topic,
                "GO2_DASHBOARD_NAV_STATUS_TOPIC": self.nav_status_topic,
                "GO2_DASHBOARD_INITIAL_POSE_TOPIC": self.initial_pose_topic,
                "GO2_DASHBOARD_MAP_LIFECYCLE_TOPIC": self.map_lifecycle_topic,
                "GO2_DASHBOARD_INITIAL_POSE_MAPS_DIR": (
                    self.initial_pose_maps_dir
                ),
                "GO2_DASHBOARD_INITIAL_POSE_AUTO_RESTORE": (
                    "1" if self.initial_pose_auto_restore else "0"
                ),
                "GO2_DASHBOARD_MAP_FRAME": self.map_frame,
                "GO2_DASHBOARD_BASE_FRAME": self.base_frame,
                "GO2_DASHBOARD_BROWSER_VOICE_ENABLED": (
                    "1" if self.browser_voice_enabled else "0"
                ),
                "GO2_DASHBOARD_LIAISON_ENDPOINT": self.liaison_endpoint,
                "GO2_DASHBOARD_AUDIO_BRIDGE_URL": self.audio_bridge_url,
                "GO2_DASHBOARD_BROWSER_MIC_PROVIDER": self.browser_mic_provider,
            }
        )
        return environment

    def public_dict(self) -> dict[str, Any]:
        return asdict(self)


def _read_health(url: str, timeout_s: float = 0.6) -> dict[str, Any]:
    request = Request(url, method="GET", headers={"Accept": "application/json"})
    opener = build_opener(ProxyHandler({}))
    with opener.open(request, timeout=timeout_s) as response:
        payload = response.read(65_536)
    parsed = json.loads(payload.decode("utf-8"))
    if not isinstance(parsed, dict):
        raise ValueError("health response is not an object")
    return parsed


class DashboardProcess:
    """Start, observe, and stop one directly-owned dashboard subprocess."""

    def __init__(
        self,
        *,
        popen_factory: Callable[..., Any] = subprocess.Popen,
        health_reader: Callable[[str], dict[str, Any]] = _read_health,
        monotonic_fn: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        self._popen_factory = popen_factory
        self._health_reader = health_reader
        self._monotonic = monotonic_fn
        self._sleep = sleep_fn
        self._lock = threading.RLock()
        self._process: Any | None = None
        self._config: DashboardConfig | None = None
        self._last_exit_code: int | None = None
        self._last_error = "not initialized"

    def configure(self, raw: Mapping[str, Any] | None) -> DashboardConfig:
        config = DashboardConfig.from_mapping(raw)
        with self._lock:
            self._config = config
            self._last_error = "configured"
        return config

    def start(self) -> None:
        with self._lock:
            config = self._config
            if config is None:
                raise RuntimeError("dashboard is not configured")
            if self._process is not None and self._process.poll() is None:
                return
            if self._process is not None:
                self._last_exit_code = self._process.poll()
            environment = config.child_environment(os.environ)
            self._process = self._popen_factory(
                [sys.executable, "-m", "go2_dashboard.main"],
                cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                env=environment,
                stdin=subprocess.DEVNULL,
                start_new_session=False,
            )
            process = self._process
            self._last_error = "waiting for dashboard health endpoint"

        deadline = self._monotonic() + config.startup_timeout_s
        error_text = "dashboard health endpoint did not become ready"
        while self._monotonic() < deadline:
            exit_code = process.poll()
            if exit_code is not None:
                error_text = f"dashboard child exited during startup with code {exit_code}"
                break
            try:
                health = self._health_reader(config.health_url)
                if bool(health.get("ok")):
                    with self._lock:
                        self._last_error = ""
                    return
                error_text = "dashboard health endpoint returned ok=false"
            except Exception as error:
                error_text = str(error)[:300]
            self._sleep(0.1)

        self._stop_process(process, config.stop_timeout_s)
        with self._lock:
            self._last_error = error_text
            self._last_exit_code = process.poll()
        raise RuntimeError(error_text)

    def ensure_running(self) -> None:
        with self._lock:
            process = self._process
        if process is None or process.poll() is not None:
            self.start()

    def stop(self) -> None:
        with self._lock:
            process = self._process
            config = self._config
            self._process = None
        if process is not None:
            self._stop_process(
                process, config.stop_timeout_s if config is not None else 5.0
            )
            with self._lock:
                self._last_exit_code = process.poll()
                self._last_error = "stopped"

    @staticmethod
    def _stop_process(process: Any, timeout_s: float) -> None:
        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)

    def status(self) -> dict[str, Any]:
        with self._lock:
            process = self._process
            config = self._config
            last_error = self._last_error
            last_exit_code = self._last_exit_code
        current_exit_code = process.poll() if process is not None else None
        running = process is not None and current_exit_code is None
        if process is not None and current_exit_code is not None:
            last_exit_code = int(current_exit_code)
            with self._lock:
                self._last_exit_code = last_exit_code
        health: dict[str, Any] | None = None
        if running and config is not None:
            try:
                health = self._health_reader(config.health_url)
                last_error = ""
            except Exception as error:
                last_error = str(error)[:300]
        ros_connected = bool(health and health.get("ros_connected"))
        web_ok = bool(health and health.get("ok"))
        ok = bool(running and web_ok and ros_connected)
        if not running:
            detail = "dashboard child is not running"
        elif not web_ok:
            detail = f"dashboard child is running but HTTP health failed: {last_error}"
        elif not ros_connected:
            detail = "dashboard web server is online; ROS observer is disconnected"
        else:
            detail = "dashboard web server and ROS observer are online"
        return {
            "ok": ok,
            "url": config.url if config is not None else "",
            "detail": detail,
            "status": {
                "read_only": not bool(
                    config and config.browser_voice_enabled
                ),
                "telemetry_read_only": True,
                "browser_voice_enabled": bool(
                    config and config.browser_voice_enabled
                ),
                "process_running": running,
                "pid": int(process.pid) if running else None,
                "last_exit_code": last_exit_code,
                "last_error": last_error,
                "health": health,
                "config": config.public_dict() if config is not None else None,
            },
        }


def status_json(status: Mapping[str, Any]) -> str:
    return json.dumps(status.get("status", {}), ensure_ascii=False, separators=(",", ":"))
