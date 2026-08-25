"""Fail-closed browser-audio handoff to the Robonix Liaison voice contract.

The dashboard never performs ASR and never talks to ROS, Nav2, or Unitree from
this module.  When explicitly enabled, it temporarily acts as the local client
of the already-deployed ``audio_client_bridge`` and asks Liaison to run its
normal voice session with that primitive pinned as the microphone provider.

Browser audio is accepted only as bounded 16 kHz mono PCM-in-WAV, retained in
memory for one session, and overwritten after the worker exits.
"""

from __future__ import annotations

import asyncio
import hmac
import io
import ipaddress
import json
import math
import os
import re
import secrets
import threading
import uuid
import wave
from dataclasses import dataclass
from typing import Any, Callable, Mapping
from urllib.parse import urlsplit

from .state import DashboardState


_PROVIDER_ID = re.compile(r"^[A-Za-z0-9_.:-]{1,80}$")
_SESSION_ID = re.compile(r"^[a-f0-9]{32}$")
_ALLOWED_MIME_TYPES = frozenset({"audio/wav", "audio/x-wav"})
_TERMINAL_VOICE_STATES = frozenset({"completed", "failed"})
_EXECUTION_MODES = frozenset({"preview", "live"})
_PREVIEW_GOAL_PREFIX = "语义导航预览："


class VoiceGatewayError(RuntimeError):
    """Base class for user-safe voice gateway failures."""


class VoiceGatewayDisabled(VoiceGatewayError):
    """The explicit browser-voice switch is off."""


class VoiceGatewayBusy(VoiceGatewayError):
    """A single allowed browser voice session is already active."""


class VoiceInputError(ValueError):
    """The request or its audio payload violates the input contract."""


def _strict_switch(value: str | None, name: str) -> bool:
    normalized = "0" if value is None else str(value).strip()
    if normalized == "0":
        return False
    if normalized == "1":
        return True
    raise ValueError(f"{name} must be exactly 0 or 1")


def _bounded_float(
    value: str | float | int,
    name: str,
    minimum: float,
    maximum: float,
) -> float:
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return result


def _bounded_int(
    value: str | int,
    name: str,
    minimum: int,
    maximum: int,
) -> int:
    result = int(value)
    if not minimum <= result <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return result


def _execution_mode(value: Any) -> str:
    result = str(value or "preview").strip().lower()
    if result not in _EXECUTION_MODES:
        raise ValueError("SEMANTIC_INTENT_EXECUTION_MODE must be preview or live")
    return result


def _loopback_grpc_endpoint(value: str) -> str:
    endpoint = str(value).strip()
    match = re.fullmatch(r"127\.0\.0\.1:([0-9]{1,5})", endpoint)
    if match is None:
        raise ValueError("Liaison endpoint must be literal loopback IPv4 host:port")
    port = int(match.group(1))
    if not 1 <= port <= 65535:
        raise ValueError("Liaison endpoint port is out of range")
    return endpoint


def _loopback_bridge_url(value: str) -> str:
    result = str(value).strip()
    parsed = urlsplit(result)
    try:
        port = parsed.port
    except ValueError as error:
        raise ValueError("audio bridge URL has an invalid port") from error
    if (
        parsed.scheme != "ws"
        or parsed.hostname != "127.0.0.1"
        or port is None
        or not 1 <= port <= 65535
        or parsed.path != "/client"
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
    ):
        raise ValueError(
            "audio bridge URL must be ws://127.0.0.1:<port>/client without credentials"
        )
    return result


@dataclass(frozen=True)
class VoiceConfig:
    """Validated, loopback-only settings for the optional voice handoff."""

    enabled: bool = False
    execution_mode: str = "preview"
    liaison_endpoint: str = "127.0.0.1:50081"
    audio_bridge_url: str = "ws://127.0.0.1:60002/client"
    mic_provider_id: str = "audio_client_bridge"
    max_upload_bytes: int = 300_000
    min_audio_seconds: float = 0.25
    max_audio_seconds: float = 8.0
    bridge_open_timeout_s: float = 3.0
    mic_start_timeout_s: float = 6.0
    session_timeout_s: float = 45.0

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise ValueError("voice enabled flag must be a boolean")
        object.__setattr__(
            self, "execution_mode", _execution_mode(self.execution_mode)
        )
        _loopback_grpc_endpoint(self.liaison_endpoint)
        _loopback_bridge_url(self.audio_bridge_url)
        if _PROVIDER_ID.fullmatch(self.mic_provider_id) is None:
            raise ValueError("invalid browser microphone provider id")
        _bounded_int(
            self.max_upload_bytes, "max_upload_bytes", 64_000, 400_000
        )
        _bounded_float(
            self.min_audio_seconds, "min_audio_seconds", 0.1, 2.0
        )
        _bounded_float(
            self.max_audio_seconds, "max_audio_seconds", 1.0, 10.0
        )
        if self.min_audio_seconds >= self.max_audio_seconds:
            raise ValueError("voice minimum duration must be below maximum duration")
        _bounded_float(
            self.bridge_open_timeout_s, "bridge_open_timeout_s", 0.5, 5.0
        )
        _bounded_float(
            self.mic_start_timeout_s, "mic_start_timeout_s", 1.0, 10.0
        )
        _bounded_float(
            self.session_timeout_s, "session_timeout_s", 10.0, 60.0
        )

    @classmethod
    def from_environment(
        cls, environment: Mapping[str, str] | None = None
    ) -> "VoiceConfig":
        env = os.environ if environment is None else environment
        enabled = _strict_switch(
            env.get("GO2_DASHBOARD_BROWSER_VOICE_ENABLED"),
            "GO2_DASHBOARD_BROWSER_VOICE_ENABLED",
        )
        liaison = _loopback_grpc_endpoint(
            env.get("GO2_DASHBOARD_LIAISON_ENDPOINT", cls.liaison_endpoint)
        )
        bridge = _loopback_bridge_url(
            env.get("GO2_DASHBOARD_AUDIO_BRIDGE_URL", cls.audio_bridge_url)
        )
        provider = str(
            env.get("GO2_DASHBOARD_BROWSER_MIC_PROVIDER", cls.mic_provider_id)
        ).strip()
        if _PROVIDER_ID.fullmatch(provider) is None:
            raise ValueError("invalid browser microphone provider id")
        minimum = _bounded_float(
            env.get(
                "GO2_DASHBOARD_VOICE_MIN_SECONDS", str(cls.min_audio_seconds)
            ),
            "GO2_DASHBOARD_VOICE_MIN_SECONDS",
            0.1,
            2.0,
        )
        maximum = _bounded_float(
            env.get(
                "GO2_DASHBOARD_VOICE_MAX_SECONDS", str(cls.max_audio_seconds)
            ),
            "GO2_DASHBOARD_VOICE_MAX_SECONDS",
            1.0,
            10.0,
        )
        if minimum >= maximum:
            raise ValueError("voice minimum duration must be below maximum duration")
        return cls(
            enabled=enabled,
            execution_mode=_execution_mode(
                env.get(
                    "SEMANTIC_INTENT_EXECUTION_MODE",
                    cls.execution_mode,
                )
            ),
            liaison_endpoint=liaison,
            audio_bridge_url=bridge,
            mic_provider_id=provider,
            max_upload_bytes=_bounded_int(
                env.get(
                    "GO2_DASHBOARD_VOICE_MAX_UPLOAD_BYTES",
                    str(cls.max_upload_bytes),
                ),
                "GO2_DASHBOARD_VOICE_MAX_UPLOAD_BYTES",
                64_000,
                400_000,
            ),
            min_audio_seconds=minimum,
            max_audio_seconds=maximum,
            bridge_open_timeout_s=_bounded_float(
                env.get(
                    "GO2_DASHBOARD_VOICE_BRIDGE_TIMEOUT_S",
                    str(cls.bridge_open_timeout_s),
                ),
                "GO2_DASHBOARD_VOICE_BRIDGE_TIMEOUT_S",
                0.5,
                5.0,
            ),
            mic_start_timeout_s=_bounded_float(
                env.get(
                    "GO2_DASHBOARD_VOICE_MIC_TIMEOUT_S",
                    str(cls.mic_start_timeout_s),
                ),
                "GO2_DASHBOARD_VOICE_MIC_TIMEOUT_S",
                1.0,
                10.0,
            ),
            session_timeout_s=_bounded_float(
                env.get(
                    "GO2_DASHBOARD_VOICE_SESSION_TIMEOUT_S",
                    str(cls.session_timeout_s),
                ),
                "GO2_DASHBOARD_VOICE_SESSION_TIMEOUT_S",
                10.0,
                60.0,
            ),
        )

    def public_limits(self) -> dict[str, Any]:
        return {
            "mime_types": sorted(_ALLOWED_MIME_TYPES),
            "sample_rate_hz": 16_000,
            "channels": 1,
            "sample_format": "pcm_s16le",
            "max_upload_bytes": self.max_upload_bytes,
            "min_duration_s": self.min_audio_seconds,
            "max_duration_s": self.max_audio_seconds,
        }


def validate_browser_request(
    *,
    client_host: str | None,
    host_header: str | None,
    origin_header: str | None,
    forwarded_header: str | None,
    x_forwarded_for: str | None,
    sec_fetch_site: str | None,
    server_port: int,
) -> None:
    """Require a direct same-origin request to the loopback dashboard."""

    try:
        client_address = ipaddress.ip_address(str(client_host or ""))
    except ValueError as error:
        raise VoiceInputError("voice upload client is not an IP address") from error
    if not client_address.is_loopback:
        raise VoiceInputError("voice upload is restricted to loopback clients")
    if forwarded_header or x_forwarded_for:
        raise VoiceInputError("forwarded voice uploads are not accepted")

    host = urlsplit(f"//{str(host_header or '').strip()}")
    try:
        host_port = host.port
    except ValueError as error:
        raise VoiceInputError("invalid Host header") from error
    if (
        host.hostname not in {"127.0.0.1", "localhost"}
        or host_port != server_port
        or host.username is not None
        or host.password is not None
    ):
        raise VoiceInputError("voice upload Host must be the dashboard loopback origin")

    origin = urlsplit(str(origin_header or "").strip())
    try:
        origin_port = origin.port
    except ValueError as error:
        raise VoiceInputError("invalid Origin header") from error
    if (
        origin.scheme != "http"
        or origin.hostname not in {"127.0.0.1", "localhost"}
        or origin_port != server_port
        or origin.username is not None
        or origin.password is not None
        or origin.path not in {"", "/"}
        or origin.query
        or origin.fragment
    ):
        raise VoiceInputError("voice upload Origin must match the loopback dashboard")
    if sec_fetch_site and sec_fetch_site.lower() != "same-origin":
        raise VoiceInputError("cross-site voice uploads are not accepted")


def validate_wav_upload(
    payload: bytes,
    content_type: str,
    config: VoiceConfig,
) -> tuple[bytearray, float]:
    """Return transient PCM and duration after strict WAV validation."""

    mime = str(content_type or "").strip().lower()
    if mime not in _ALLOWED_MIME_TYPES:
        raise VoiceInputError("voice upload must use audio/wav")
    if not payload:
        raise VoiceInputError("voice upload is empty")
    if len(payload) > config.max_upload_bytes:
        raise VoiceInputError("voice upload exceeds the byte limit")
    try:
        with wave.open(io.BytesIO(payload), "rb") as source:
            if source.getnchannels() != 1:
                raise VoiceInputError("voice WAV must be mono")
            if source.getsampwidth() != 2:
                raise VoiceInputError("voice WAV must use 16-bit PCM")
            if source.getframerate() != 16_000:
                raise VoiceInputError("voice WAV must use a 16 kHz sample rate")
            if source.getcomptype() != "NONE":
                raise VoiceInputError("compressed WAV is not accepted")
            frames = int(source.getnframes())
            if frames <= 0:
                raise VoiceInputError("voice WAV has no samples")
            duration = frames / 16_000.0
            if not config.min_audio_seconds <= duration <= config.max_audio_seconds:
                raise VoiceInputError(
                    "voice WAV duration is outside the configured bounds"
                )
            pcm = source.readframes(frames + 1)
    except VoiceInputError:
        raise
    except (EOFError, wave.Error) as error:
        raise VoiceInputError("malformed PCM WAV") from error
    if len(pcm) != frames * 2 or len(pcm) > int(config.max_audio_seconds * 32_000):
        raise VoiceInputError("voice WAV sample count is inconsistent")
    return bytearray(pcm), duration


def _event_status(event: Mapping[str, Any]) -> tuple[str, str, str]:
    kind = int(event.get("kind", -1))
    text = str(event.get("text") or "").strip()[:300]
    status_message = str(event.get("status_message") or "").strip()[:400]
    error = str(event.get("error") or "").strip()[:400]
    if kind == 0:
        return "liaison", status_message or "Liaison 会话已建立", ""
    if kind == 1:
        return "recording", "正在把浏览器音频转交 Liaison", ""
    if kind in {2, 3}:
        return "asr", text or status_message or "中文语音识别中", text
    if kind == 4:
        return "recognized", text or "语音识别完成", text
    if kind == 5:
        return "authorized", status_message or "Liaison 身份/访问门已完成", ""
    if kind == 6:
        return "pilot", text or status_message or "Pilot 正在处理任务", ""
    if kind in {7, 8}:
        return "pilot", status_message or "Liaison 响应处理中", ""
    if kind == 9:
        return "completed", status_message or "Liaison 会话完成", ""
    if kind == 10:
        return "failed", error or status_message or "Liaison 会话失败", ""
    return "liaison", status_message or f"Liaison 事件 {kind}", ""


def _pilot_payload(pilot: Any) -> dict[str, Any]:
    """Copy bounded display-only fields from one nested Pilot event.

    This intentionally does not deserialize RTDL arguments or connect to any
    capability.  Counting call-bearing plan leaves gives the preview UI a
    fail-closed tripwire if Pilot ever returns an executable plan.
    """

    kind = int(getattr(pilot, "event_kind", -1))
    result: dict[str, Any] = {
        "event_kind": kind,
        "text": "",
        "task_goal": "",
        "success_criterion": "",
        "task_status": "",
        "intent_target": "",
        "capability_calls": 0,
    }
    if kind == 0:
        result["text"] = str(getattr(pilot, "text_chunk", "") or "").strip()[:300]
    elif kind == 1:
        plan = getattr(pilot, "plan", None)
        nodes = list(getattr(plan, "nodes", ()) or ())
        result["capability_calls"] = sum(
            1
            for node in nodes
            if any(
                str(getattr(getattr(node, "call", None), field, "") or "").strip()
                for field in ("call_id", "provider_id", "contract_id", "args_json")
            )
        )
    elif kind == 3:
        status = getattr(pilot, "status", None)
        result["text"] = str(getattr(status, "message", "") or "").strip()[:300]
    elif kind == 4:
        result["text"] = str(getattr(pilot, "final_text", "") or "").strip()[:300]
    elif kind == 6:
        task = getattr(pilot, "task_state", None)
        goal = str(getattr(task, "goal", "") or "").strip()[:300]
        criterion = str(
            getattr(task, "success_criterion", "") or ""
        ).strip()[:500]
        task_status = str(getattr(task, "status", "") or "").strip()[:64]
        result.update(
            {
                "task_goal": goal,
                "success_criterion": criterion,
                "task_status": task_status,
            }
        )
        if goal.startswith(_PREVIEW_GOAL_PREFIX):
            result["intent_target"] = goal[len(_PREVIEW_GOAL_PREFIX) :].strip()[:128]
    return result


def _voice_event_payload(event: Any) -> dict[str, Any]:
    """Return the bounded in-memory representation consumed by the UI worker."""

    result = {
        "kind": int(getattr(event, "event_kind", -1)),
        "text": str(getattr(event, "text", "") or "").strip()[:300],
        "status_message": str(
            getattr(event, "status_message", "") or ""
        ).strip()[:400],
        "error": str(getattr(event, "error", "") or "").strip()[:400],
    }
    if result["kind"] == 6:
        result["pilot"] = _pilot_payload(getattr(event, "pilot", None))
    return result


def _event_update(event: Mapping[str, Any], execution_mode: str) -> dict[str, Any]:
    """Translate one Liaison event into display-only state fields."""

    status, message, transcript = _event_status(event)
    result: dict[str, Any] = {"status": status, "message": message}
    if transcript:
        result["transcript"] = transcript
    pilot = event.get("pilot")
    if not isinstance(pilot, Mapping):
        return result

    pilot_kind = int(pilot.get("event_kind", -1))
    pilot_text = str(pilot.get("text") or "").strip()[:300]
    if pilot_text:
        result["message"] = pilot_text
    calls = int(pilot.get("capability_calls", 0) or 0)
    result["capability_calls_observed"] = calls
    if pilot_kind == 6:
        target = str(pilot.get("intent_target") or "").strip()[:128]
        goal = str(pilot.get("task_goal") or "").strip()[:300]
        task_status = str(pilot.get("task_status") or "").strip()[:64]
        criterion = str(pilot.get("success_criterion") or "").strip()[:500]
        if goal:
            result["intent_summary"] = goal
        if target:
            result["intent_target"] = target
            result["message"] = f"Pilot 已解析语义目标：{target}（未执行）"
        if task_status:
            result["pilot_task_status"] = task_status
        if _execution_mode(execution_mode) == "preview" and criterion:
            result["blocked_reason"] = criterion
    return result


def _grpc_voice_worker(
    config: VoiceConfig,
    session_id: str,
    record_seconds: int,
    emit_event: Callable[[dict[str, Any]], None],
    cancel: threading.Event,
) -> None:
    """Consume the official Liaison server stream on a bounded worker."""

    import grpc  # type: ignore
    import liaison_pb2  # type: ignore
    import robonix_contracts_pb2_grpc  # type: ignore

    channel = grpc.insecure_channel(config.liaison_endpoint)
    call = None
    try:
        grpc.channel_ready_future(channel).result(
            timeout=config.bridge_open_timeout_s
        )
        stub = robonix_contracts_pb2_grpc.RobonixSystemLiaisonVoiceStub(channel)
        request = liaison_pb2.StartVoiceSession_Request(
            session_id=session_id,
            client_user_id="",
            record_seconds=record_seconds,
            language="zh-CN",
            tts_enabled=False,
            mic_node_id=config.mic_provider_id,
            asr_node_id="",
            voiceprint_node_id="",
            tts_node_id="",
            speaker_node_id="",
            context_json=json.dumps(
                {
                    "source": "go2_dashboard_browser",
                    "transport": "loopback",
                    "audio_persisted": False,
                    "semantic_intent_execution_mode": config.execution_mode,
                },
                separators=(",", ":"),
            ),
        )
        call = stub.StartVoiceSession(request, timeout=config.session_timeout_s)
        for response in call:
            if cancel.is_set():
                call.cancel()
                raise VoiceGatewayError("voice session canceled during shutdown")
            event = response.event
            emit_event(_voice_event_payload(event))
    finally:
        if cancel.is_set() and call is not None:
            call.cancel()
        channel.close()


async def _send_pcm(
    websocket: Any,
    pcm: bytearray,
    stream_id: str,
    cancel: threading.Event,
) -> None:
    frame_bytes = 3_200
    for offset in range(0, len(pcm), frame_bytes):
        if cancel.is_set():
            raise VoiceGatewayError("voice session canceled during shutdown")
        frame = bytes(pcm[offset : offset + frame_bytes])
        await websocket.send(frame)
        await asyncio.sleep(len(frame) / 32_000.0)
    await websocket.send(
        json.dumps(
            {"type": "mic_end", "stream_id": stream_id},
            separators=(",", ":"),
        )
    )


async def _run_network_session(
    config: VoiceConfig,
    session_id: str,
    pcm: bytearray,
    duration_s: float,
    update: Callable[..., None],
    cancel: threading.Event,
) -> None:
    import websockets  # type: ignore

    loop = asyncio.get_running_loop()
    events: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=128)

    def enqueue(event: dict[str, Any]) -> None:
        def put() -> None:
            if events.full():
                try:
                    events.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            events.put_nowait(event)

        loop.call_soon_threadsafe(put)

    update(status="connecting", message="正在连接本机音频桥和 Liaison")
    record_seconds = max(1, min(10, int(math.ceil(duration_s)) + 1))
    deadline = loop.time() + config.session_timeout_s
    mic_deadline = loop.time() + config.mic_start_timeout_s
    terminal = False
    mic_started = False
    sender: asyncio.Task[None] | None = None

    async with websockets.connect(
        config.audio_bridge_url,
        proxy=None,
        open_timeout=config.bridge_open_timeout_s,
        close_timeout=1.0,
        ping_interval=10.0,
        ping_timeout=3.0,
        max_size=65_536,
        max_queue=8,
        compression=None,
    ) as websocket:
        update(status="liaison", message="音频桥已连接，正在启动 Liaison voice")
        grpc_task = asyncio.create_task(
            asyncio.to_thread(
                _grpc_voice_worker,
                config,
                session_id,
                record_seconds,
                enqueue,
                cancel,
            )
        )
        receive_task = asyncio.create_task(websocket.recv())
        event_task = asyncio.create_task(events.get())
        try:
            while True:
                if cancel.is_set():
                    raise VoiceGatewayError("voice session canceled during shutdown")
                now = loop.time()
                if now >= deadline:
                    raise TimeoutError("Liaison voice session exceeded its deadline")
                if not mic_started and now >= mic_deadline:
                    raise TimeoutError("audio bridge did not request microphone audio")

                wait_for: set[asyncio.Task[Any]] = {
                    grpc_task,
                    receive_task,
                    event_task,
                }
                if sender is not None:
                    wait_for.add(sender)
                done, _ = await asyncio.wait(
                    wait_for,
                    timeout=min(0.25, deadline - now),
                    return_when=asyncio.FIRST_COMPLETED,
                )

                if receive_task in done:
                    message = receive_task.result()
                    if isinstance(message, bytes):
                        raise VoiceGatewayError(
                            "unexpected speaker audio from TTS-disabled Liaison session"
                        )
                    try:
                        control = json.loads(message)
                    except json.JSONDecodeError as error:
                        raise VoiceGatewayError("invalid audio bridge control frame") from error
                    if not isinstance(control, dict):
                        raise VoiceGatewayError("invalid audio bridge control payload")
                    kind = str(control.get("type") or "")
                    if kind == "mic_start":
                        stream_id = str(control.get("stream_id") or "")
                        if (
                            mic_started
                            or not stream_id
                            or len(stream_id) > 80
                            or int(control.get("sample_rate", 0)) != 16_000
                            or int(control.get("channels", 0)) != 1
                        ):
                            raise VoiceGatewayError("invalid or duplicate mic_start frame")
                        mic_started = True
                        update(
                            status="recording",
                            message="浏览器音频正在转交 Liaison（内存中，不落盘）",
                        )
                        sender = asyncio.create_task(
                            _send_pcm(websocket, pcm, stream_id, cancel)
                        )
                    elif kind == "mic_stop":
                        if sender is not None and not sender.done():
                            sender.cancel()
                    elif kind in {"speaker_stop", "speaker_end"}:
                        raise VoiceGatewayError(
                            "unexpected speaker control from TTS-disabled session"
                        )
                    else:
                        raise VoiceGatewayError("unexpected audio bridge control frame")
                    receive_task = asyncio.create_task(websocket.recv())

                if event_task in done:
                    event = event_task.result()
                    event_update = _event_update(event, config.execution_mode)
                    update(**event_update)
                    if (
                        config.execution_mode == "preview"
                        and int(event_update.get("capability_calls_observed", 0)) > 0
                    ):
                        raise VoiceGatewayError(
                            "预览安全策略违规：Pilot 返回了可执行 capability 计划，"
                            "会话已阻断"
                        )
                    if event_update["status"] == "failed":
                        raise VoiceGatewayError(str(event_update["message"]))
                    terminal = event_update["status"] == "completed"
                    event_task = asyncio.create_task(events.get())

                if sender is not None and sender in done:
                    sender.result()
                    sender = None

                if grpc_task in done:
                    grpc_task.result()
                    if not terminal:
                        raise VoiceGatewayError(
                            "Liaison stream ended without SESSION_DONE"
                        )
                    return
        finally:
            for task in (receive_task, event_task, sender):
                if task is not None and not task.done():
                    task.cancel()
            if not grpc_task.done():
                cancel.set()
                try:
                    await asyncio.wait_for(asyncio.shield(grpc_task), timeout=1.0)
                except (asyncio.TimeoutError, Exception):
                    pass


def run_liaison_session(
    config: VoiceConfig,
    session_id: str,
    pcm: bytearray,
    duration_s: float,
    update: Callable[..., None],
    cancel: threading.Event,
) -> None:
    """Run one official Liaison voice turn; suitable for a daemon worker."""

    if not _SESSION_ID.fullmatch(session_id):
        raise VoiceGatewayError("invalid internal session id")
    asyncio.run(
        _run_network_session(config, session_id, pcm, duration_s, update, cancel)
    )


class BrowserVoiceGateway:
    """Own at most one transient browser-audio Liaison session."""

    def __init__(
        self,
        state: DashboardState,
        config: VoiceConfig,
        *,
        runner: Callable[..., None] = run_liaison_session,
    ) -> None:
        self._state = state
        self.config = config
        self._runner = runner
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._cancel = threading.Event()
        self._nonce = secrets.token_urlsafe(32)
        self._state.configure_voice(
            config.enabled,
            config.public_limits(),
            config.execution_mode,
        )

    def browser_status(self) -> dict[str, Any]:
        result = self._state.voice_status()
        result["csrf_nonce"] = self._nonce if self.config.enabled else ""
        return result

    def verify_nonce(self, supplied: str | None) -> bool:
        value = str(supplied or "")
        return bool(value) and hmac.compare_digest(value, self._nonce)

    def submit(self, pcm: bytearray, duration_s: float) -> dict[str, Any]:
        if not self.config.enabled:
            raise VoiceGatewayDisabled("browser voice input is disabled")
        session_id = uuid.uuid4().hex
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise VoiceGatewayBusy("another browser voice session is active")
            self._cancel = threading.Event()
            self._state.update_voice(
                session_id=session_id,
                status="accepted",
                message="音频已通过严格校验，等待转交 Liaison",
                transcript="",
                active=True,
                intent_target="",
                intent_summary="",
                blocked_reason="",
                pilot_task_status="",
                capability_calls_observed=0,
            )
            # One successful handoff consumes the browser nonce. The page
            # fetches a fresh value before the next capture, so replaying the
            # previous POST cannot start another Liaison turn.
            self._nonce = secrets.token_urlsafe(32)
            self._thread = threading.Thread(
                target=self._run,
                args=(session_id, pcm, float(duration_s), self._cancel),
                name="go2-dashboard-liaison-voice",
                daemon=True,
            )
            self._thread.start()
        return self._state.voice_status()

    def _run(
        self,
        session_id: str,
        pcm: bytearray,
        duration_s: float,
        cancel: threading.Event,
    ) -> None:
        def update(
            *,
            status: str,
            message: str,
            transcript: str | None = None,
            intent_target: str | None = None,
            intent_summary: str | None = None,
            blocked_reason: str | None = None,
            pilot_task_status: str | None = None,
            capability_calls_observed: int | None = None,
        ) -> None:
            self._state.update_voice(
                session_id=session_id,
                status=status,
                message=message,
                transcript=transcript,
                active=status not in _TERMINAL_VOICE_STATES,
                intent_target=intent_target,
                intent_summary=intent_summary,
                blocked_reason=blocked_reason,
                pilot_task_status=pilot_task_status,
                capability_calls_observed=capability_calls_observed,
            )

        try:
            self._runner(
                self.config,
                session_id,
                pcm,
                duration_s,
                update,
                cancel,
            )
            current = self._state.voice_status()
            if current["status"] not in _TERMINAL_VOICE_STATES:
                update(status="completed", message="Liaison 会话完成")
        except Exception as error:  # noqa: BLE001 - fail closed at worker boundary
            update(status="failed", message=str(error)[:400] or "语音会话失败")
        finally:
            pcm[:] = b"\x00" * len(pcm)
            pcm.clear()

    def close(self, timeout_s: float = 1.0) -> None:
        self._cancel.set()
        with self._lock:
            thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, min(float(timeout_s), 2.0)))
