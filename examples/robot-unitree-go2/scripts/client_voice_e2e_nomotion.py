#!/usr/bin/env python3
"""Capture one official-Client voice E2E run behind strict no-motion gates.

The only accepted phrase is ``机器狗保持静止``.  Live mode queues one bounded
16 kHz mono PCM-s16le stream on the loopback Audio Device Server, then uses a
Chrome DevTools Protocol mouse event to click the visible official Client
``Voice`` button.  It never calls a ROS service/action, creates a ROS
publisher, changes networking, enables Hands-free, or enables motion.

Without ``--execute-live`` this program can only perform the read-only
preflight.  Every live attempt additionally requires all of these assertions:

* the exact current corrected workstation no-motion wrapper is alive;
* its rendered manifest says ``allow_motion: false`` and routes Nav2 to the
  isolated no-motion topic;
* Dashboard reports semantic execution mode ``preview``;
* official Client Hands-free is disabled and Executor has zero active plans;
* the canonical ROS topic ``/cmd_vel`` is absent in three bounded graph reads;
* the visible Client page has no active turn and an enabled Voice button.

Evidence is written only below this package's ``logs/operator-ui-nomotion``
tree.  A failed assertion prevents injection and clicking.  If a failure
occurs after PCM was queued but before the click, the queue is explicitly
cleared before exit.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import re
import socket
import stat
import subprocess
import sys
import time
from typing import Any, Callable, Mapping, Sequence

import websockets
import yaml


ROOT = Path(__file__).resolve().parents[1]
RUN_ROOT = ROOT / "rbnx-build" / "run"
CURRENT_SESSION = RUN_ROOT / "workstation-nomotion-current.session"
OUTPUT_ROOT = ROOT / "logs" / "operator-ui-nomotion"
LOOPBACK = "127.0.0.1"
EXPECTED_PHRASE = "机器狗保持静止"
CANONICAL_CMD_VEL = "/cmd_vel"
ISOLATED_CMD_VEL = "/robonix/nomotion/cmd_vel"

CLIENT_PORT = 7860
DASHBOARD_PORT = 8092
AUDIO_PORT = 60000
CDP_PORT = 9223
SAMPLE_RATE = 16_000
CHANNELS = 1
SAMPLE_WIDTH_BYTES = 2
FRAME_SAMPLES = SAMPLE_RATE // 10
FRAME_BYTES = FRAME_SAMPLES * SAMPLE_WIDTH_BYTES
MIN_PCM_SECONDS = 0.5
MAX_PCM_SECONDS = 8.0
TRAILING_SILENCE_SECONDS = 1.0
MAX_HTTP_BYTES = 4 * 1024 * 1024
MAX_CDP_MESSAGE_BYTES = 16 * 1024 * 1024
ROS_GRAPH_SAMPLES = 3
ROS_GRAPH_TIMEOUT_SECONDS = 5.0
VOICE_TIMEOUT_MIN_SECONDS = 20.0
VOICE_TIMEOUT_MAX_SECONDS = 180.0

CLIENT_URL = f"http://{LOOPBACK}:{CLIENT_PORT}/"
VOICE_WS_URL = f"ws://{LOOPBACK}:{CLIENT_PORT}/ws/voice"

ALLOWED_HTTP = frozenset(
    {
        (DASHBOARD_PORT, "GET", "/healthz"),
        (DASHBOARD_PORT, "GET", "/api/status"),
        (CLIENT_PORT, "GET", "/api/defaults"),
        (CLIENT_PORT, "GET", "/api/settings"),
        (CLIENT_PORT, "POST", "/api/handsfree/status"),
        (CLIENT_PORT, "POST", "/api/executor/active-plans"),
        (CDP_PORT, "GET", "/json/list"),
    }
)

ACTIVE_HANDSFREE_STATES = frozenset(
    {"starting", "listening", "triggered", "acknowledging", "in_voice"}
)
REQUIRED_VOICE_EVENTS = frozenset(
    {
        "session_started",
        "recording_started",
        "recording_done",
        "asr_final",
        "tts_started",
        "tts_done",
        "session_done",
    }
)


class GateError(RuntimeError):
    """A no-motion or evidence-integrity gate failed."""


def utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def require(condition: bool, message: str) -> None:
    if not condition:
        raise GateError(message)


def false_value(value: Any) -> bool:
    return value is False or (
        isinstance(value, str) and value.strip().lower() == "false"
    )


def normalize_transcript(value: str) -> str:
    """Normalize only spacing and terminal punctuation, never paraphrases."""

    compact = re.sub(r"\s+", "", value or "")
    return compact.rstrip("，。！？,.!?")


def json_write(path: Path, payload: Mapping[str, Any]) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def bytes_write(path: Path, payload: bytes) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags, 0o600)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def prepare_output(requested: Path) -> Path:
    require(requested.is_absolute(), "output directory must be absolute")
    target = Path(os.path.abspath(os.fspath(requested)))
    root = Path(os.path.abspath(os.fspath(OUTPUT_ROOT)))
    require(root.is_dir() and not root.is_symlink(), f"unsafe output root: {root}")
    try:
        relative = target.relative_to(root)
    except ValueError as exc:
        raise GateError(f"output directory must be below {root}") from exc
    require(relative != Path("."), "output directory must be a new child")

    current = root
    for part in relative.parts[:-1]:
        current = current / part
        require(current.exists(), f"output parent does not exist: {current}")
        mode = os.lstat(current).st_mode
        require(stat.S_ISDIR(mode) and not stat.S_ISLNK(mode), f"unsafe output parent: {current}")
    require(not target.exists() and not target.is_symlink(), f"refusing to overwrite: {target}")
    target.mkdir(mode=0o700)
    os.chmod(target, 0o700)
    return target


def private_regular(path: Path, *, expected_mode: int | None = None) -> None:
    try:
        info = os.lstat(path)
    except OSError as exc:
        raise GateError(f"required file unavailable: {path}: {exc}") from exc
    require(stat.S_ISREG(info.st_mode), f"required path is not a regular file: {path}")
    require(not stat.S_ISLNK(info.st_mode), f"required file is a symlink: {path}")
    require(info.st_uid == os.geteuid(), f"required file is not owned by current user: {path}")
    if expected_mode is not None:
        require(
            stat.S_IMODE(info.st_mode) == expected_mode,
            f"required file mode is not {expected_mode:04o}: {path}",
        )


def parse_session_meta(path: Path) -> dict[str, str]:
    private_regular(path, expected_mode=0o600)
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        require("=" in line, f"malformed session metadata: {path}")
        key, value = line.split("=", 1)
        require(key not in result, f"duplicate session metadata key: {key}")
        require(
            key in {"format", "token", "wrapper_pid", "wrapper_start_ticks", "run_dir"},
            f"unknown session metadata key: {key}",
        )
        result[key] = value
    require(
        set(result) == {"format", "token", "wrapper_pid", "wrapper_start_ticks", "run_dir"},
        "session metadata keys are incomplete",
    )
    require(result["format"] == "go2-workstation-nomotion-session-v1", "wrong session format")
    require(
        re.fullmatch(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}", result["token"])
        is not None,
        "invalid session token",
    )
    require(result["wrapper_pid"].isdigit() and int(result["wrapper_pid"]) > 0, "invalid wrapper pid")
    require(result["wrapper_start_ticks"].isdigit(), "invalid wrapper start ticks")
    return result


def process_start_ticks(pid: int) -> tuple[str, str]:
    try:
        line = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except OSError as exc:
        raise GateError(f"no-motion wrapper is not alive: pid={pid}: {exc}") from exc
    marker = line.rfind(") ")
    require(marker >= 0, "malformed wrapper process stat")
    fields = line[marker + 2 :].split()
    require(len(fields) > 19 and fields[0] != "Z", "no-motion wrapper is a zombie")
    return fields[0], fields[19]


def safe_json_read(path: Path) -> dict[str, Any]:
    private_regular(path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise GateError(f"invalid JSON evidence {path}: {exc}") from exc
    require(isinstance(payload, dict), f"JSON evidence is not an object: {path}")
    return payload


def safe_yaml_read(path: Path) -> dict[str, Any]:
    private_regular(path)
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise GateError(f"invalid manifest {path}: {exc}") from exc
    require(isinstance(payload, dict), "rendered manifest is not a mapping")
    return payload


def named_component(entries: Any, name: str, section: str) -> dict[str, Any]:
    require(isinstance(entries, list), f"manifest {section} is not a list")
    matches = [entry for entry in entries if isinstance(entry, dict) and entry.get("name") == name]
    require(len(matches) == 1, f"manifest must contain exactly one {section}.{name}")
    return matches[0]


def validate_nomotion_manifest(manifest: Mapping[str, Any]) -> dict[str, Any]:
    require(manifest.get("manifestVersion") == 1, "manifestVersion is not 1")
    require(
        manifest.get("name") == "robonix-go2-workstation-full-nomotion-corrected",
        "rendered manifest is not the corrected workstation no-motion profile",
    )
    environment = manifest.get("env")
    require(isinstance(environment, Mapping), "manifest env is missing")
    require(false_value(environment.get("GO2_ALLOW_MOTION")), "manifest GO2_ALLOW_MOTION is not false")

    chassis = named_component(manifest.get("primitive"), "go2_chassis", "primitive")
    chassis_config = chassis.get("config")
    require(isinstance(chassis_config, Mapping), "go2_chassis config is missing")
    require(chassis_config.get("allow_motion") is False, "go2_chassis allow_motion is not false")
    require(chassis_config.get("operator_present") is False, "go2_chassis operator_present is not false")
    require(chassis_config.get("safety_ack") == "", "go2_chassis safety_ack is not empty")
    require(
        chassis_config.get("twist_in_topic") == "/robonix/nomotion/chassis_input_disabled",
        "go2_chassis is not isolated from velocity input",
    )

    nav2 = named_component(manifest.get("service"), "nav2", "service")
    nav2_config = nav2.get("config")
    require(isinstance(nav2_config, Mapping), "nav2 config is missing")
    require(
        nav2_config.get("velocity_output_topic") == ISOLATED_CMD_VEL,
        "Nav2 velocity output is not isolated to the no-motion topic",
    )
    return {
        "name": manifest["name"],
        "env_go2_allow_motion": environment.get("GO2_ALLOW_MOTION"),
        "chassis_allow_motion": chassis_config.get("allow_motion"),
        "chassis_operator_present": chassis_config.get("operator_present"),
        "chassis_twist_in_topic": chassis_config.get("twist_in_topic"),
        "nav2_velocity_output_topic": nav2_config.get("velocity_output_topic"),
    }


def validate_current_session() -> dict[str, Any]:
    private_regular(CURRENT_SESSION, expected_mode=0o600)
    pointer_meta = parse_session_meta(CURRENT_SESSION)
    run_dir = Path(pointer_meta["run_dir"])
    require(run_dir.is_absolute(), "current run directory is not absolute")
    require(run_dir.parent == RUN_ROOT, "current run directory is outside the run root")
    require(
        re.fullmatch(r"workstation-nomotion-stamp\.[A-Za-z0-9]{6}", run_dir.name)
        is not None,
        "current run directory name is invalid",
    )
    run_info = os.lstat(run_dir)
    require(stat.S_ISDIR(run_info.st_mode) and not stat.S_ISLNK(run_info.st_mode), "current run path is unsafe")
    require(run_info.st_uid == os.geteuid(), "current run directory has another owner")
    require(stat.S_IMODE(run_info.st_mode) == 0o700, "current run directory mode is not 0700")

    session_meta_path = run_dir / "session.meta"
    run_meta = parse_session_meta(session_meta_path)
    require(pointer_meta == run_meta, "current-session pointer does not match run metadata")
    require(run_dir.resolve(strict=True) == run_dir, "current run directory is not canonical")

    pid = int(pointer_meta["wrapper_pid"])
    _, observed_ticks = process_start_ticks(pid)
    require(observed_ticks == pointer_meta["wrapper_start_ticks"], "wrapper process identity changed")
    proc_info = os.stat(f"/proc/{pid}")
    require(proc_info.st_uid == os.geteuid(), "no-motion wrapper belongs to another user")
    command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace")
    require(
        "start_workstation_full_nomotion_corrected.sh" in command,
        "current wrapper command is not the corrected no-motion launcher",
    )

    ready = safe_json_read(run_dir / "ready.json")
    identity = safe_json_read(run_dir / "identity-ready.json")
    require(not (run_dir / "fault.json").exists(), "current no-motion session has fault.json")
    require(ready.get("time_discipline_ready") is True, "time discipline is not ready")
    require(ready.get("motion_ready") is False, "stamp layer unexpectedly reports motion_ready")
    require(identity.get("identity_bound") is True, "DDS writer identity is not bound")
    require(identity.get("motion_ready") is False, "identity layer unexpectedly reports motion_ready")
    require(identity.get("session_id") == ready.get("session_id"), "ready files disagree on session id")

    manifest_path = run_dir / "robonix_manifest.yaml"
    manifest_summary = validate_nomotion_manifest(safe_yaml_read(manifest_path))
    return {
        "run_dir": str(run_dir),
        "session_token": pointer_meta["token"],
        "wrapper_pid": pid,
        "wrapper_start_ticks": observed_ticks,
        "session_id": ready.get("session_id"),
        "time_discipline_ready": ready.get("time_discipline_ready"),
        "stamp_motion_ready": ready.get("motion_ready"),
        "identity_bound": identity.get("identity_bound"),
        "identity_motion_ready": identity.get("motion_ready"),
        "manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "manifest": manifest_summary,
    }


def request_json(
    port: int,
    method: str,
    path: str,
    payload: Mapping[str, Any] | None = None,
    *,
    timeout: float = 3.0,
) -> Any:
    operation = (port, method, path)
    require(operation in ALLOWED_HTTP, f"HTTP operation is outside the read-only allowlist: {operation}")
    body = None
    headers = {"Accept": "application/json", "Connection": "close"}
    if payload is not None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        headers["Content-Type"] = "application/json"
    connection = http.client.HTTPConnection(LOOPBACK, port, timeout=timeout)
    try:
        connection.request(method, path, body=body, headers=headers)
        response = connection.getresponse()
        raw = response.read(MAX_HTTP_BYTES + 1)
    except (OSError, http.client.HTTPException) as exc:
        raise GateError(f"loopback request failed: {port}{path}: {exc}") from exc
    finally:
        connection.close()
    require(response.status == 200, f"loopback request returned HTTP {response.status}: {port}{path}")
    require(len(raw) <= MAX_HTTP_BYTES, f"loopback response exceeds limit: {port}{path}")
    try:
        return json.loads(raw)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise GateError(f"loopback response is not JSON: {port}{path}") from exc


def validate_dashboard(health: Any, status: Any) -> dict[str, Any]:
    require(isinstance(health, Mapping) and health.get("ok") is True, "Dashboard health is not ok")
    require(health.get("telemetry_read_only") is True, "Dashboard telemetry_read_only is not true")
    require(isinstance(status, Mapping), "Dashboard status is not an object")
    require(status.get("telemetry_read_only") is True, "Dashboard status is not telemetry read-only")
    voice = status.get("voice")
    semantic = status.get("semantic_task")
    navigation = status.get("navigation")
    require(isinstance(voice, Mapping), "Dashboard voice status is missing")
    require(voice.get("enabled") is True, "Dashboard browser voice is not enabled")
    require(voice.get("active") is False, "Dashboard reports an active voice session")
    require(voice.get("execution_mode") == "preview", "semantic execution mode is not preview")
    require(voice.get("direct_robot_control") is False, "Dashboard voice exposes direct robot control")
    require(voice.get("capability_calls_observed") == 0, "Dashboard already observed capability calls")
    require(isinstance(semantic, Mapping) and semantic.get("read_only_effect") is True, "semantic task is not read-only effect")
    require(isinstance(navigation, Mapping) and navigation.get("status") == "idle", "navigation is not idle")
    return {
        "health_ok": health.get("ok"),
        "telemetry_read_only": status.get("telemetry_read_only"),
        "voice": dict(voice),
        "semantic_task": dict(semantic),
        "navigation": dict(navigation),
    }


def validate_page_state(page: Any) -> dict[str, Any]:
    require(isinstance(page, Mapping), "Client page state is not an object")
    require(page.get("url") == CLIENT_URL, "CDP target is not the loopback official Client")
    require(page.get("documentReady") == "complete", "Client document is not fully loaded")
    require(page.get("voiceActive") is False, "Client already has an active voice session")
    require(page.get("hasActiveTurn") is False, "Client has an active task turn")
    require(page.get("busy") is False and page.get("taskRunning") is False, "Client is busy")
    require(page.get("activeTurnId") == "", "Client active turn id is not empty")
    handsfree = page.get("handsfree")
    require(isinstance(handsfree, Mapping), "Client Hands-free state is missing")
    require(handsfree.get("available") is True, "Client Hands-free status is unavailable")
    require(handsfree.get("enabled") is False, "Client Hands-free is enabled")
    require(str(handsfree.get("state") or "").lower() not in ACTIVE_HANDSFREE_STATES, "Client Hands-free owns the microphone")
    require(page.get("executorPlansReady") is True, "Client Executor plan view is not ready")
    require(page.get("executorPlanCount") == 0, "Client page contains active Executor plans")
    button = page.get("button")
    require(isinstance(button, Mapping), "visible Voice button was not found")
    require(
        button.get("visible") is True
        and button.get("topmost") is True
        and button.get("disabled") is False,
        "Voice button is not safely clickable",
    )
    settings = page.get("settings")
    require(isinstance(settings, Mapping), "Client page settings are missing")
    require(settings.get("atlasEndpoint") == f"{LOOPBACK}:50051", "Client Atlas endpoint is not audited loopback")
    return dict(page)


def validate_handsfree(payload: Any) -> dict[str, Any]:
    require(isinstance(payload, Mapping), "Hands-free status is not an object")
    require(payload.get("available") is True, "Hands-free provider is unavailable")
    require(payload.get("enabled") is False, "Hands-free is enabled")
    require(str(payload.get("state") or "").lower() not in ACTIVE_HANDSFREE_STATES, "Hands-free is active")
    return dict(payload)


def validate_active_plans(payload: Any) -> dict[str, Any]:
    require(isinstance(payload, Mapping), "active-plans response is not an object")
    require(payload.get("available") is True, "Executor active-plans query is unavailable")
    require(type(payload.get("count")) is int, "Executor active-plans count is malformed")
    require(payload.get("count") == 0, "Executor has active plans")
    plans = payload.get("plans")
    require(isinstance(plans, list) and not plans, "Executor active-plans list is not empty")
    return dict(payload)


def ros_topic_samples() -> dict[str, Any]:
    samples: list[list[str]] = []
    for _ in range(ROS_GRAPH_SAMPLES):
        try:
            result = subprocess.run(
                ["ros2", "topic", "list", "--no-daemon"],
                check=False,
                capture_output=True,
                text=True,
                timeout=ROS_GRAPH_TIMEOUT_SECONDS,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise GateError(f"bounded ROS graph read failed: {exc}") from exc
        require(result.returncode == 0, f"ROS graph read failed: {result.stderr.strip()[:300]}")
        topics = sorted({line.strip() for line in result.stdout.splitlines() if line.strip()})
        require(CANONICAL_CMD_VEL not in topics, f"canonical {CANONICAL_CMD_VEL} exists")
        samples.append(topics)
    return {
        "query": ["ros2", "topic", "list", "--no-daemon"],
        "samples": samples,
        "sample_count": len(samples),
        "canonical_cmd_vel": CANONICAL_CMD_VEL,
        "canonical_cmd_vel_absent": all(CANONICAL_CMD_VEL not in item for item in samples),
    }


def validate_pcm(path: Path) -> tuple[bytes, dict[str, Any]]:
    require(path.is_absolute(), "PCM path must be absolute")
    canonical = path.resolve(strict=True)
    require(canonical == path, "PCM path must be canonical and contain no symlink")
    try:
        canonical.relative_to(ROOT)
    except ValueError as exc:
        raise GateError(f"PCM must be stored below {ROOT}") from exc
    private_regular(canonical)
    source_raw = canonical.read_bytes()
    require(len(source_raw) % SAMPLE_WIDTH_BYTES == 0, "PCM byte count is not sample-aligned")
    source_duration = len(source_raw) / (SAMPLE_RATE * SAMPLE_WIDTH_BYTES)
    require(
        MIN_PCM_SECONDS
        <= source_duration
        <= MAX_PCM_SECONDS - TRAILING_SILENCE_SECONDS,
        "source PCM duration must be 0.5..7.0 seconds",
    )
    samples = memoryview(source_raw).cast("h")
    peak = max((abs(value) for value in samples), default=0)
    sum_squares = sum(float(value) * float(value) for value in samples)
    rms = math.sqrt(sum_squares / len(samples)) if samples else 0.0
    require(peak > 0 and rms > 1.0, "PCM is silent")
    trailing_silence = b"\0" * int(
        SAMPLE_RATE * SAMPLE_WIDTH_BYTES * TRAILING_SILENCE_SECONDS
    )
    raw = source_raw + trailing_silence
    frame_padding_bytes = (-len(raw)) % FRAME_BYTES
    raw += b"\0" * frame_padding_bytes
    duration = len(raw) / (SAMPLE_RATE * SAMPLE_WIDTH_BYTES)
    require(duration <= MAX_PCM_SECONDS, "framed PCM exceeds 8.0 seconds")
    return raw, {
        "path": str(canonical),
        "source_sha256": hashlib.sha256(source_raw).hexdigest(),
        "injected_sha256": hashlib.sha256(raw).hexdigest(),
        "source_bytes": len(source_raw),
        "source_duration_seconds": round(source_duration, 6),
        "bytes": len(raw),
        "duration_seconds": round(duration, 3),
        "sample_rate_hz": SAMPLE_RATE,
        "channels": CHANNELS,
        "sample_format": "pcm_s16le",
        "frame_bytes": FRAME_BYTES,
        "frames": len(raw) // FRAME_BYTES,
        "trailing_silence_seconds": TRAILING_SILENCE_SECONDS,
        "frame_padding_bytes": frame_padding_bytes,
        "peak": peak,
        "rms": round(rms, 3),
        "expected_phrase": EXPECTED_PHRASE,
    }


@dataclass
class CdpClient:
    uri: str
    ws: Any = None
    reader_task: asyncio.Task[Any] | None = None
    next_id: int = 0
    pending: dict[int, asyncio.Future[Any]] = field(default_factory=dict)
    events: asyncio.Queue[dict[str, Any]] = field(default_factory=asyncio.Queue)
    network_records: list[dict[str, Any]] = field(default_factory=list)

    async def __aenter__(self) -> "CdpClient":
        self.ws = await websockets.connect(
            self.uri,
            open_timeout=3.0,
            close_timeout=2.0,
            max_size=MAX_CDP_MESSAGE_BYTES,
            proxy=None,
        )
        self.reader_task = asyncio.create_task(self._reader())
        return self

    async def __aexit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        if self.ws is not None:
            await self.ws.close()
        if self.reader_task is not None:
            with contextlib_suppress(asyncio.CancelledError, Exception):
                await self.reader_task

    async def _reader(self) -> None:
        try:
            async for raw in self.ws:
                message = json.loads(raw)
                if "id" in message:
                    future = self.pending.pop(int(message["id"]), None)
                    if future is not None and not future.done():
                        future.set_result(message)
                    continue
                method = str(message.get("method") or "")
                if method.startswith("Network.webSocket"):
                    record = {
                        "observed_at": utc_now(),
                        "monotonic_ns": time.monotonic_ns(),
                        "method": method,
                        "params": message.get("params") or {},
                    }
                    self.network_records.append(record)
                    await self.events.put(record)
        except Exception as exc:
            for future in self.pending.values():
                if not future.done():
                    future.set_exception(exc)
            self.pending.clear()

    async def call(self, method: str, params: Mapping[str, Any] | None = None, *, timeout: float = 5.0) -> Any:
        self.next_id += 1
        identifier = self.next_id
        future = asyncio.get_running_loop().create_future()
        self.pending[identifier] = future
        await self.ws.send(json.dumps({"id": identifier, "method": method, "params": dict(params or {})}))
        response = await asyncio.wait_for(future, timeout=timeout)
        if "error" in response:
            raise GateError(f"CDP {method} failed: {response['error']}")
        return response.get("result") or {}

    async def evaluate(self, expression: str) -> Any:
        result = await self.call(
            "Runtime.evaluate",
            {"expression": expression, "returnByValue": True, "awaitPromise": True},
        )
        require("exceptionDetails" not in result, f"Client page evaluation failed: {result.get('exceptionDetails')}")
        remote = result.get("result") or {}
        require("value" in remote, "Client page evaluation returned no value")
        return remote["value"]

    async def screenshot(self) -> bytes:
        result = await self.call(
            "Page.captureScreenshot",
            {"format": "png", "fromSurface": True, "captureBeyondViewport": False},
            timeout=10.0,
        )
        data = result.get("data")
        require(isinstance(data, str) and data, "CDP screenshot is empty")
        return base64.b64decode(data, validate=True)


class contextlib_suppress:
    """Tiny local equivalent used to keep this script's imports explicit."""

    def __init__(self, *exceptions: type[BaseException]) -> None:
        self.exceptions = exceptions

    def __enter__(self) -> None:
        return None

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> bool:
        return exc_type is not None and issubclass(exc_type, self.exceptions)


PAGE_STATE_EXPRESSION = r"""
(() => {
  const button = document.querySelector("[data-page-action='voice-start']");
  const rect = button ? button.getBoundingClientRect() : null;
  const style = button ? getComputedStyle(button) : null;
  return {
    url: location.href,
    documentReady: document.readyState,
    voiceActive: Boolean(state.voiceActive),
    hasActiveTurn: Boolean(hasActiveTurn()),
    busy: Boolean(state.busy),
    taskRunning: Boolean(state.taskRunning),
    activeTurnId: String(state.activeTurnId || ""),
    handsfree: {
      available: Boolean(state.handsfree.available),
      enabled: Boolean(state.handsfree.enabled),
      state: String(state.handsfree.state || ""),
      busy: Boolean(state.handsfree.busy),
    },
    executorPlansReady: Boolean(state.executorPlansReady),
    executorPlanCount: Array.isArray(state.executorPlans) ? state.executorPlans.length : -1,
    settings: collectSettings(),
    button: button && rect ? {
      disabled: Boolean(button.disabled),
      visible: rect.width > 0 && rect.height > 0 && rect.left >= 0 && rect.top >= 0 &&
        rect.right <= innerWidth && rect.bottom <= innerHeight &&
        style.visibility !== "hidden" && style.display !== "none",
      topmost: (() => {
        const hit = document.elementFromPoint(rect.left + rect.width / 2, rect.top + rect.height / 2);
        return hit === button || Boolean(hit && button.contains(hit));
      })(),
      x: rect.left + rect.width / 2,
      y: rect.top + rect.height / 2,
      text: String(button.textContent || "").trim(),
    } : null,
  };
})()
"""


async def page_state(cdp: CdpClient) -> dict[str, Any]:
    return validate_page_state(await cdp.evaluate(PAGE_STATE_EXPRESSION))


async def find_client_target() -> dict[str, Any]:
    payload = request_json(CDP_PORT, "GET", "/json/list")
    require(isinstance(payload, list), "CDP target list is not an array")
    matches = [
        item
        for item in payload
        if isinstance(item, Mapping)
        and item.get("type") == "page"
        and item.get("url") == CLIENT_URL
        and isinstance(item.get("webSocketDebuggerUrl"), str)
    ]
    require(len(matches) == 1, "expected exactly one official Client CDP page")
    return dict(matches[0])


async def audio_health() -> dict[str, Any]:
    uri = f"ws://{LOOPBACK}:{AUDIO_PORT}/health"
    try:
        async with websockets.connect(uri, open_timeout=3.0, close_timeout=2.0, proxy=None) as ws:
            payload = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
    except Exception as exc:
        raise GateError(f"Audio Device Server health failed: {exc}") from exc
    require(isinstance(payload, Mapping) and payload.get("ok") is True, "Audio Device Server is not healthy")
    require(payload.get("sample_rate") == SAMPLE_RATE, "Audio Device Server sample rate is not 16 kHz")
    require(payload.get("frame_bytes") == FRAME_BYTES, "Audio Device Server frame size is unexpected")
    return dict(payload)


async def inject_pcm(raw: bytes) -> dict[str, Any]:
    uri = f"ws://{LOOPBACK}:{AUDIO_PORT}/inject_mic"
    async with websockets.connect(uri, open_timeout=3.0, close_timeout=2.0, proxy=None) as ws:
        await ws.send(json.dumps({"clear": True}))
        await ws.send(raw)
        await ws.send(json.dumps({"commit": True}))
        ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
    require(isinstance(ack, Mapping) and ack.get("ok") is True, "PCM injection was not acknowledged")
    require(ack.get("cleared") is True, "PCM injection did not clear the prior queue")
    require(ack.get("frames") == len(raw) // FRAME_BYTES, "PCM injection frame count mismatch")
    return dict(ack)


async def clear_pcm_queue() -> dict[str, Any]:
    uri = f"ws://{LOOPBACK}:{AUDIO_PORT}/inject_mic"
    async with websockets.connect(uri, open_timeout=3.0, close_timeout=2.0, proxy=None) as ws:
        await ws.send(json.dumps({"clear": True, "commit": True}))
        ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
    require(isinstance(ack, Mapping) and ack.get("ok") is True and ack.get("cleared") is True, "could not clear PCM queue")
    return dict(ack)


def client_runtime_checks(settings: Mapping[str, Any]) -> dict[str, Any]:
    handsfree = validate_handsfree(
        request_json(CLIENT_PORT, "POST", "/api/handsfree/status", {"settings": settings})
    )
    plans = validate_active_plans(
        request_json(CLIENT_PORT, "POST", "/api/executor/active-plans", {"settings": settings})
    )
    return {"handsfree": handsfree, "active_plans": plans}


async def full_preflight(cdp: CdpClient) -> dict[str, Any]:
    session = validate_current_session()
    dashboard = validate_dashboard(
        request_json(DASHBOARD_PORT, "GET", "/healthz"),
        request_json(DASHBOARD_PORT, "GET", "/api/status"),
    )
    page = await page_state(cdp)
    runtime = client_runtime_checks(page["settings"])
    topics = ros_topic_samples()
    audio = await audio_health()
    # Close races introduced while the slower ROS graph samples ran.
    page = await page_state(cdp)
    runtime = client_runtime_checks(page["settings"])
    session = validate_current_session()
    dashboard = validate_dashboard(
        request_json(DASHBOARD_PORT, "GET", "/healthz"),
        request_json(DASHBOARD_PORT, "GET", "/api/status"),
    )
    return {
        "observed_at": utc_now(),
        "session": session,
        "dashboard": dashboard,
        "client_page": page,
        "client_runtime": runtime,
        "ros_graph": topics,
        "audio_server": audio,
    }


async def fast_click_gate(cdp: CdpClient) -> dict[str, Any]:
    session = validate_current_session()
    dashboard = validate_dashboard(
        request_json(DASHBOARD_PORT, "GET", "/healthz"),
        request_json(DASHBOARD_PORT, "GET", "/api/status"),
    )
    page = await page_state(cdp)
    runtime = client_runtime_checks(page["settings"])
    return {
        "observed_at": utc_now(),
        "session": session,
        "dashboard": dashboard,
        "client_page": page,
        "client_runtime": runtime,
    }


async def click_voice(cdp: CdpClient, page: Mapping[str, Any]) -> dict[str, float]:
    button = page["button"]
    x = float(button["x"])
    y = float(button["y"])
    await cdp.call("Page.bringToFront")
    await cdp.call("Input.dispatchMouseEvent", {"type": "mouseMoved", "x": x, "y": y})
    await cdp.call(
        "Input.dispatchMouseEvent",
        {"type": "mousePressed", "x": x, "y": y, "button": "left", "clickCount": 1},
    )
    await cdp.call(
        "Input.dispatchMouseEvent",
        {"type": "mouseReleased", "x": x, "y": y, "button": "left", "clickCount": 1},
    )
    return {"x": x, "y": y}


@dataclass
class VoiceTrace:
    request_id: str = ""
    accepted: bool = False
    done: bool = False
    closed: bool = False
    sent_frames: int = 0
    received_frames: int = 0
    event_kinds: list[str] = field(default_factory=list)
    transcripts: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    recording_screenshot_taken: bool = False

    def observe_payload(self, payload: Any) -> None:
        self.received_frames += 1
        if not isinstance(payload, Mapping):
            self.errors.append("malformed_non_object_frame")
            return
        message_type = payload.get("type")
        if message_type == "accepted":
            self.accepted = True
        elif message_type == "done":
            self.done = True
        elif message_type == "error":
            self.errors.append(str(payload.get("error") or "unknown Client error"))
        elif message_type == "voice_event":
            event = payload.get("event")
            if not isinstance(event, Mapping):
                self.errors.append("malformed_voice_event")
                return
            kind = str(event.get("kind") or "").strip().lower()
            if not kind:
                self.errors.append("voice_event_without_kind")
                return
            self.event_kinds.append(kind)
            if kind == "asr_final":
                self.transcripts.append(str(event.get("text") or ""))
            if kind == "error":
                self.errors.append(str(event.get("error") or event.get("statusMessage") or "voice error"))

    def acceptance_reasons(self) -> list[str]:
        reasons: list[str] = []
        if not self.request_id:
            reasons.append("voice_websocket_not_created")
        if self.sent_frames < 1:
            reasons.append("voice_websocket_request_not_sent")
        if not self.accepted:
            reasons.append("voice_websocket_not_accepted")
        if not self.done:
            reasons.append("voice_websocket_done_missing")
        if not self.closed:
            reasons.append("voice_websocket_close_missing")
        missing = sorted(REQUIRED_VOICE_EVENTS.difference(self.event_kinds))
        if missing:
            reasons.append("missing_voice_events:" + ",".join(missing))
        normalized = [normalize_transcript(item) for item in self.transcripts]
        if normalized != [EXPECTED_PHRASE]:
            reasons.append("asr_final_did_not_match_fixed_phrase")
        if self.errors:
            reasons.append("voice_errors_observed")
        return reasons


async def observe_voice(cdp: CdpClient, output: Path, timeout: float) -> VoiceTrace:
    trace = VoiceTrace()
    deadline = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < deadline:
        remaining = deadline - asyncio.get_running_loop().time()
        try:
            record = await asyncio.wait_for(cdp.events.get(), timeout=remaining)
        except asyncio.TimeoutError as exc:
            raise GateError(f"voice WebSocket did not complete within {timeout:g}s") from exc
        method = record["method"]
        params = record["params"]
        if method == "Network.webSocketCreated":
            if params.get("url") == VOICE_WS_URL:
                require(not trace.request_id, "more than one /ws/voice socket was created")
                trace.request_id = str(params.get("requestId") or "")
            continue
        if not trace.request_id or str(params.get("requestId") or "") != trace.request_id:
            continue
        if method == "Network.webSocketFrameSent":
            trace.sent_frames += 1
        elif method == "Network.webSocketFrameReceived":
            response = params.get("response")
            payload_data = response.get("payloadData") if isinstance(response, Mapping) else None
            try:
                payload = json.loads(payload_data) if isinstance(payload_data, str) else None
            except json.JSONDecodeError:
                payload = None
            trace.observe_payload(payload)
            if (
                "recording_started" in trace.event_kinds
                and not trace.recording_screenshot_taken
            ):
                bytes_write(output / "client-recording.png", await cdp.screenshot())
                trace.recording_screenshot_taken = True
        elif method in {"Network.webSocketClosed", "Network.webSocketFrameError"}:
            if method == "Network.webSocketClosed":
                trace.closed = True
            else:
                trace.errors.append(str(params.get("errorMessage") or "CDP WebSocket frame error"))
        if trace.done and trace.closed:
            return trace
    raise GateError(f"voice WebSocket did not complete within {timeout:g}s")


def compact_network_records(records: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    voice_request_ids: set[str] = set()
    for item in records:
        params = item.get("params")
        if not isinstance(params, Mapping):
            continue
        method = item.get("method")
        request_id = str(params.get("requestId") or "")
        if method == "Network.webSocketCreated" and params.get("url") == VOICE_WS_URL:
            voice_request_ids.add(request_id)
        if request_id not in voice_request_ids:
            continue
        result.append(dict(item))
    return result


def postflight_acceptance(trace: VoiceTrace, postflight: Mapping[str, Any]) -> list[str]:
    reasons = trace.acceptance_reasons()
    voice = postflight["dashboard"]["voice"]
    if voice.get("capability_calls_observed") != 0:
        reasons.append("capability_calls_observed_after_voice")
    if voice.get("execution_mode") != "preview":
        reasons.append("semantic_execution_mode_changed")
    if postflight["client_runtime"]["active_plans"].get("count") != 0:
        reasons.append("active_plans_after_voice")
    if not postflight["ros_graph"].get("canonical_cmd_vel_absent"):
        reasons.append("canonical_cmd_vel_after_voice")
    return reasons


async def run(args: argparse.Namespace) -> int:
    output = prepare_output(args.output_dir)
    summary: dict[str, Any] = {
        "schema": "robonix-go2-official-client-voice-nomotion-e2e-v1",
        "started_at": utc_now(),
        "mode": "execute-live" if args.execute_live else "preflight-only",
        "expected_phrase": EXPECTED_PHRASE,
        "motion_authorized": False,
        "network_configuration_changed": False,
        "injection_attempted": False,
        "voice_button_clicked": False,
        "passed": False,
    }
    queued = False
    clicked = False
    cdp: CdpClient | None = None
    try:
        target = await find_client_target()
        async with CdpClient(str(target["webSocketDebuggerUrl"])) as connected:
            cdp = connected
            await cdp.call("Page.enable")
            await cdp.call("Runtime.enable")
            await cdp.call("Network.enable")
            preflight = await full_preflight(cdp)
            summary["preflight"] = preflight
            bytes_write(output / "client-preflight.png", await cdp.screenshot())
            json_write(output / "preflight.json", preflight)

            if not args.execute_live:
                summary["passed"] = True
                summary["finished_at"] = utc_now()
                json_write(output / "summary.json", summary)
                print(f"PASS: no-motion Client voice preflight only; evidence={output}")
                return 0

            raw, pcm_summary = validate_pcm(args.pcm)
            summary["pcm"] = pcm_summary
            summary["injection_attempted"] = True
            injection = await inject_pcm(raw)
            queued = True
            summary["injection"] = injection

            # No action is allowed after queuing until every cheap live gate is
            # checked again.  A failure enters the finally block and clears the
            # one-shot queue before returning.
            click_gate = await fast_click_gate(cdp)
            summary["click_gate"] = click_gate
            json_write(output / "click-gate.json", click_gate)

            summary["click"] = await click_voice(cdp, click_gate["client_page"])
            clicked = True
            queued = False  # The next /mic request now owns the one-shot stream.
            summary["voice_button_clicked"] = True
            trace = await observe_voice(cdp, output, args.voice_timeout_seconds)
            bytes_write(output / "client-final.png", await cdp.screenshot())

            postflight = await fast_click_gate(cdp)
            postflight["ros_graph"] = ros_topic_samples()
            summary["postflight"] = postflight
            json_write(output / "postflight.json", postflight)
            frames = compact_network_records(cdp.network_records)
            json_write(output / "voice-websocket-frames.json", {"records": frames})
            summary["voice_trace"] = {
                "request_id": trace.request_id,
                "accepted": trace.accepted,
                "done": trace.done,
                "closed": trace.closed,
                "sent_frames": trace.sent_frames,
                "received_frames": trace.received_frames,
                "event_kinds": trace.event_kinds,
                "transcripts": trace.transcripts,
                "errors": trace.errors,
                "recording_screenshot_taken": trace.recording_screenshot_taken,
            }
            reasons = postflight_acceptance(trace, postflight)
            summary["acceptance"] = {"passed": not reasons, "failure_reasons": reasons}
            summary["passed"] = not reasons
    except (GateError, OSError, ValueError, asyncio.TimeoutError) as exc:
        summary["failure"] = {"type": type(exc).__name__, "message": str(exc)}
        if cdp is not None and cdp.network_records:
            with contextlib_suppress(Exception):
                json_write(
                    output / "voice-websocket-frames.json",
                    {"records": compact_network_records(cdp.network_records)},
                )
    finally:
        if queued and not clicked:
            try:
                summary["queue_cleanup"] = await clear_pcm_queue()
            except Exception as exc:
                summary["queue_cleanup"] = {"ok": False, "error": str(exc)}
        summary["finished_at"] = utc_now()
        summary_path = output / "summary.json"
        if not summary_path.exists():
            json_write(summary_path, summary)

    if summary.get("passed") is True:
        print(f"PASS: fixed-phrase official Client voice E2E remained no-motion; evidence={output}")
        return 0
    print(f"FAIL CLOSED: official Client voice E2E did not pass; evidence={output}", file=sys.stderr)
    return 1


def bounded_voice_timeout(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("voice timeout must be a number") from exc
    if not VOICE_TIMEOUT_MIN_SECONDS <= parsed <= VOICE_TIMEOUT_MAX_SECONDS:
        raise argparse.ArgumentTypeError(
            f"voice timeout must be {VOICE_TIMEOUT_MIN_SECONDS:g}..{VOICE_TIMEOUT_MAX_SECONDS:g} seconds"
        )
    return parsed


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    mode = result.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--preflight-only",
        action="store_true",
        help="run only read-only gates and a preflight screenshot",
    )
    mode.add_argument(
        "--execute-live",
        action="store_true",
        help="after all gates pass, queue PCM and physically click Voice via CDP",
    )
    result.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help=f"new absolute directory below {OUTPUT_ROOT}",
    )
    result.add_argument(
        "--pcm",
        type=Path,
        help="absolute canonical raw PCM path below this package; live mode only",
    )
    result.add_argument(
        "--voice-timeout-seconds",
        default=120.0,
        type=bounded_voice_timeout,
        help="bounded wait for the Client voice WebSocket (20..180s; default 120)",
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.execute_live and args.pcm is None:
        parser().error("--execute-live requires --pcm")
    if args.preflight_only and args.pcm is not None:
        parser().error("--pcm is accepted only with --execute-live")
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("interrupted; no motion was authorized", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
