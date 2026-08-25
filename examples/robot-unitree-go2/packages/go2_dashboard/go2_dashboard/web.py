"""FastAPI application for the read-only Go2 telemetry dashboard."""

from __future__ import annotations

import math
import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response
from pydantic import BaseModel, Field

from . import __version__
from .initial_pose_store import InitialPoseError
from .ros_bridge import RosBridge, RosConfig
from .state import DashboardState
from .voice_gateway import (
    BrowserVoiceGateway,
    VoiceConfig,
    VoiceGatewayBusy,
    VoiceInputError,
    validate_browser_request,
    validate_wav_upload,
)


class SemanticPose(BaseModel):
    frame_id: str = Field(default="map", min_length=1, max_length=80)
    x: float
    y: float
    yaw: float


class SemanticTaskUpdate(BaseModel):
    """Status metadata only; this schema has no command or goal endpoint."""

    task_id: str = Field(default="", max_length=128)
    target_name: str = Field(default="", max_length=128)
    status: Literal[
        "idle",
        "received",
        "resolving",
        "resolved",
        "navigating",
        "succeeded",
        "canceled",
        "failed",
    ]
    message: str = Field(default="", max_length=500)
    pose: SemanticPose | None = None


class InitialPoseRestoreRequest(BaseModel):
    map_id: str = Field(min_length=1, max_length=160)
    generation: int = Field(ge=0, le=(1 << 64) - 1)


class InitialPoseResetRequest(BaseModel):
    confirm_map_id: str = Field(min_length=1, max_length=160)
    generation: int = Field(ge=0, le=(1 << 64) - 1)


def _model_dict(model: BaseModel) -> dict[str, Any]:
    if hasattr(model, "model_dump"):
        return model.model_dump()  # type: ignore[attr-defined,no-any-return]
    return model.dict()


def create_app(
    state: DashboardState | None = None,
    config: RosConfig | None = None,
    voice_config: VoiceConfig | None = None,
    voice_gateway: BrowserVoiceGateway | None = None,
) -> FastAPI:
    ros_config = config or RosConfig.from_environment()
    dashboard_state = state or DashboardState(
        ros_config.topic_specs(),
        deployment_profile=os.environ.get("GO2_DASHBOARD_PROFILE", "integrated"),
    )
    bridge = RosBridge(dashboard_state, ros_config)
    voice = voice_gateway or BrowserVoiceGateway(
        dashboard_state, voice_config or VoiceConfig.from_environment()
    )
    try:
        dashboard_port = int(os.environ.get("GO2_DASHBOARD_PORT", "8092"))
    except ValueError as error:
        raise ValueError("GO2_DASHBOARD_PORT must be an integer") from error
    if not 1 <= dashboard_port <= 65535:
        raise ValueError("GO2_DASHBOARD_PORT is out of range")

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        bridge.start()
        try:
            yield
        finally:
            voice.close()
            bridge.stop()

    app = FastAPI(
        title="Robonix Go2 Telemetry and Liaison Voice Dashboard",
        version=__version__,
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
        lifespan=lifespan,
    )
    app.state.dashboard_state = dashboard_state
    app.state.ros_bridge = bridge
    app.state.browser_voice = voice

    @app.get("/", response_class=HTMLResponse)
    async def dashboard() -> HTMLResponse:
        page = (
            Path(__file__).resolve().parent / "static" / "index.html"
        ).read_text(encoding="utf-8")
        return HTMLResponse(
            page,
            headers={
                "Cache-Control": "no-store",
                "Content-Security-Policy": (
                    "default-src 'self'; script-src 'self' 'unsafe-inline'; "
                    "style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; "
                    "connect-src 'self'; media-src 'none'; object-src 'none'; "
                    "base-uri 'none'; form-action 'none'; frame-ancestors 'none'"
                ),
                "Referrer-Policy": "no-referrer",
                "Permissions-Policy": "microphone=(self)",
                "X-Content-Type-Options": "nosniff",
                "X-Frame-Options": "DENY",
            },
        )

    @app.get("/healthz")
    async def health() -> JSONResponse:
        snapshot = dashboard_state.snapshot()
        return JSONResponse(
            {
                "ok": True,
                "read_only": not bool(snapshot["voice"]["enabled"]),
                "telemetry_read_only": True,
                "ros_connected": bool(snapshot["bridge"]["connected"]),
                "browser_voice_enabled": bool(snapshot["voice"]["enabled"]),
                "profile": snapshot["profile"],
                "version": __version__,
            },
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/status")
    async def status() -> JSONResponse:
        return JSONResponse(
            dashboard_state.snapshot(), headers={"Cache-Control": "no-store"}
        )

    @app.get("/api/camera.jpg")
    async def camera() -> Response:
        image, sequence = dashboard_state.camera_image()
        if image is None:
            raise HTTPException(status_code=503, detail="camera frame unavailable")
        return Response(
            content=image,
            media_type="image/jpeg",
            headers={
                "Cache-Control": "no-store",
                "X-Telemetry-Sequence": str(sequence),
            },
        )

    @app.get("/api/cameras/{stream}.jpg")
    async def camera_stream(
        stream: Literal["go2", "d435i-color", "d435i-depth"],
        sequence: int | None = None,
    ) -> Response:
        key = {
            "go2": "camera",
            "d435i-color": "d435i_color",
            "d435i-depth": "d435i_depth",
        }[stream]
        image, current_sequence = dashboard_state.camera_image(key)
        if image is None:
            raise HTTPException(
                status_code=503, detail=f"{stream} camera frame unavailable"
            )
        if sequence is not None and sequence != current_sequence:
            raise HTTPException(
                status_code=409,
                detail=(
                    "camera snapshot changed; refresh status before requesting "
                    "the image"
                ),
                headers={"X-Telemetry-Sequence": str(current_sequence)},
            )
        return Response(
            content=image,
            media_type="image/jpeg",
            headers={
                "Cache-Control": "no-store",
                "X-Telemetry-Sequence": str(current_sequence),
            },
        )

    @app.get("/api/map.png")
    async def map_preview(sequence: int | None = None) -> Response:
        image, current_sequence = dashboard_state.map_image()
        if image is None:
            raise HTTPException(status_code=503, detail="map unavailable")
        if sequence is not None and sequence != current_sequence:
            raise HTTPException(
                status_code=409,
                detail=(
                    "map snapshot changed; refresh status before requesting the image"
                ),
                headers={"X-Telemetry-Sequence": str(current_sequence)},
            )
        return Response(
            content=image,
            media_type="image/png",
            headers={
                "Cache-Control": "no-store",
                "X-Telemetry-Sequence": str(current_sequence),
            },
        )

    @app.get("/api/semantic-task")
    async def semantic_task() -> JSONResponse:
        return JSONResponse(
            dashboard_state.semantic_task(),
            headers={"Cache-Control": "no-store"},
        )

    @app.post("/api/semantic-task")
    async def update_semantic_task(update: SemanticTaskUpdate) -> JSONResponse:
        payload = _model_dict(update)
        pose = payload.get("pose")
        if pose is not None:
            numeric_values = (pose["x"], pose["y"], pose["yaw"])
            if not all(math.isfinite(float(value)) for value in numeric_values):
                raise HTTPException(
                    status_code=422, detail="pose values must be finite"
                )
        try:
            result = dashboard_state.update_semantic_task(payload)
        except (KeyError, TypeError, ValueError) as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        return JSONResponse(result, headers={"Cache-Control": "no-store"})

    @app.get("/api/initial-pose")
    async def initial_pose_status() -> JSONResponse:
        return JSONResponse(
            bridge.initial_pose_status(), headers={"Cache-Control": "no-store"}
        )

    @app.post("/api/initial-pose/restore", status_code=202)
    async def restore_initial_pose(
        request: InitialPoseRestoreRequest,
    ) -> JSONResponse:
        try:
            status = bridge.request_initial_pose_restore(
                **_model_dict(request)
            )
        except InitialPoseError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        return JSONResponse(
            status, status_code=202, headers={"Cache-Control": "no-store"}
        )

    @app.post("/api/initial-pose/reset")
    async def reset_initial_pose(
        request: InitialPoseResetRequest,
    ) -> JSONResponse:
        try:
            status = bridge.reset_initial_pose(**_model_dict(request))
        except InitialPoseError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        return JSONResponse(status, headers={"Cache-Control": "no-store"})

    @app.get("/api/voice")
    async def browser_voice_status() -> JSONResponse:
        return JSONResponse(
            voice.browser_status(), headers={"Cache-Control": "no-store"}
        )

    @app.post("/api/voice", status_code=202)
    async def browser_voice_upload(request: Request) -> JSONResponse:
        if not voice.config.enabled:
            raise HTTPException(status_code=404, detail="browser voice is disabled")
        try:
            validate_browser_request(
                client_host=request.client.host if request.client else None,
                host_header=request.headers.get("host"),
                origin_header=request.headers.get("origin"),
                forwarded_header=request.headers.get("forwarded"),
                x_forwarded_for=request.headers.get("x-forwarded-for"),
                sec_fetch_site=request.headers.get("sec-fetch-site"),
                server_port=dashboard_port,
            )
        except VoiceInputError as error:
            raise HTTPException(status_code=403, detail=str(error)) from error
        if not voice.verify_nonce(request.headers.get("x-go2-voice-nonce")):
            raise HTTPException(status_code=403, detail="invalid voice request nonce")
        if request.headers.get("content-encoding"):
            raise HTTPException(status_code=415, detail="encoded request bodies are rejected")
        if request.headers.get("content-type", "").strip().lower() not in {
            "audio/wav",
            "audio/x-wav",
        }:
            raise HTTPException(status_code=415, detail="voice upload must use audio/wav")
        raw_length = request.headers.get("content-length")
        try:
            content_length = int(raw_length or "")
        except ValueError as error:
            raise HTTPException(
                status_code=411, detail="a valid Content-Length is required"
            ) from error
        if content_length <= 0:
            raise HTTPException(status_code=411, detail="Content-Length must be positive")
        if content_length > voice.config.max_upload_bytes:
            raise HTTPException(status_code=413, detail="voice upload is too large")

        body = bytearray()
        async for chunk in request.stream():
            body.extend(chunk)
            if len(body) > voice.config.max_upload_bytes:
                body[:] = b"\x00" * len(body)
                body.clear()
                raise HTTPException(status_code=413, detail="voice upload is too large")
        if len(body) != content_length:
            body[:] = b"\x00" * len(body)
            body.clear()
            raise HTTPException(status_code=400, detail="voice upload length mismatch")
        try:
            pcm, duration = validate_wav_upload(
                bytes(body), request.headers.get("content-type", ""), voice.config
            )
        except VoiceInputError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        finally:
            body[:] = b"\x00" * len(body)
            body.clear()
        try:
            accepted = voice.submit(pcm, duration)
        except VoiceGatewayBusy as error:
            pcm[:] = b"\x00" * len(pcm)
            pcm.clear()
            raise HTTPException(status_code=409, detail=str(error)) from error
        return JSONResponse(
            accepted,
            status_code=202,
            headers={"Cache-Control": "no-store"},
        )

    return app
