from __future__ import annotations

import io
import struct
import tempfile
import threading
import unittest
import wave
from pathlib import Path

try:
    import fastapi  # noqa: F401 - availability gate for the first build pass
    import httpx
except ImportError:  # First offline pass intentionally has no PyPI imports.
    httpx = None  # type: ignore[assignment]


def _wav(seconds: float = 0.4) -> bytes:
    frames = int(16_000 * seconds)
    with io.BytesIO() as output:
        with wave.open(output, "wb") as target:
            target.setnchannels(1)
            target.setsampwidth(2)
            target.setframerate(16_000)
            target.writeframes(struct.pack("<h", 900) * frames)
        return output.getvalue()


@unittest.skipUnless(httpx is not None, "FastAPI HTTP test dependencies not installed")
class VoiceHttpTests(unittest.IsolatedAsyncioTestCase):
    def _client(self, *, enabled: bool, client_host: str = "127.0.0.1"):
        from go2_dashboard.state import DashboardState
        from go2_dashboard.voice_gateway import BrowserVoiceGateway, VoiceConfig
        from go2_dashboard.web import create_app

        state = DashboardState()
        ran = threading.Event()

        def runner(config, session_id, pcm, duration, update, cancel):
            del config, session_id, pcm, duration, cancel
            update(status="completed", message="offline test completed")
            ran.set()

        gateway = BrowserVoiceGateway(
            state, VoiceConfig(enabled=enabled), runner=runner
        )
        app = create_app(state=state, voice_gateway=gateway)
        transport = httpx.ASGITransport(
            app=app,
            client=(client_host, 41000),
        )
        client = httpx.AsyncClient(
            transport=transport,
            base_url="http://127.0.0.1:8092",
        )
        self.addAsyncCleanup(client.aclose)
        return client, gateway, ran

    @staticmethod
    def _headers(nonce: str) -> dict[str, str]:
        return {
            "Origin": "http://127.0.0.1:8092",
            "Sec-Fetch-Site": "same-origin",
            "Content-Type": "audio/wav",
            "X-Go2-Voice-Nonce": nonce,
        }

    async def test_disabled_post_is_hidden_and_ui_is_not_globally_read_only_when_enabled(self) -> None:
        disabled, _, _ = self._client(enabled=False)
        page = await disabled.get("/")
        self.assertEqual(page.headers.get("permissions-policy"), "microphone=(self)")
        self.assertEqual(
            (await disabled.post("/api/voice", content=_wav())).status_code, 404
        )
        self.assertTrue((await disabled.get("/api/status")).json()["read_only"])

        enabled, _, _ = self._client(enabled=True)
        status = (await enabled.get("/api/status")).json()
        self.assertFalse(status["read_only"])
        self.assertTrue(status["telemetry_read_only"])

    async def test_post_rejects_non_loopback_forwarding_and_bad_nonce(self) -> None:
        remote, _, _ = self._client(enabled=True, client_host="192.168.1.50")
        nonce = (await remote.get("/api/voice")).json()["csrf_nonce"]
        response = await remote.post(
            "/api/voice", content=_wav(), headers=self._headers(nonce)
        )
        self.assertEqual(response.status_code, 403)

        local, _, _ = self._client(enabled=True)
        response = await local.post(
            "/api/voice", content=_wav(), headers=self._headers("wrong")
        )
        self.assertEqual(response.status_code, 403)
        nonce = (await local.get("/api/voice")).json()["csrf_nonce"]
        headers = self._headers(nonce)
        headers["Forwarded"] = "for=127.0.0.1"
        self.assertEqual(
            (await local.post("/api/voice", content=_wav(), headers=headers)).status_code,
            403,
        )

    async def test_post_requires_length_mime_and_enforces_declared_size(self) -> None:
        client, _, _ = self._client(enabled=True)
        nonce = (await client.get("/api/voice")).json()["csrf_nonce"]
        headers = self._headers(nonce)
        headers["Content-Length"] = "not-an-integer"
        self.assertEqual(
            (await client.post("/api/voice", content=_wav(), headers=headers)).status_code,
            411,
        )

        nonce = (await client.get("/api/voice")).json()["csrf_nonce"]
        headers = self._headers(nonce)
        headers["Content-Length"] = "300001"
        self.assertEqual(
            (await client.post("/api/voice", content=b"small", headers=headers)).status_code,
            413,
        )

        nonce = (await client.get("/api/voice")).json()["csrf_nonce"]
        headers = self._headers(nonce)
        headers["Content-Type"] = "audio/webm"
        self.assertEqual(
            (await client.post("/api/voice", content=b"not-wav", headers=headers)).status_code,
            415,
        )

    async def test_valid_post_is_accepted_once_and_consumes_nonce(self) -> None:
        client, gateway, ran = self._client(enabled=True)
        nonce = (await client.get("/api/voice")).json()["csrf_nonce"]
        response = await client.post(
            "/api/voice", content=_wav(), headers=self._headers(nonce)
        )
        self.assertEqual(response.status_code, 202, response.text)
        self.assertTrue(ran.wait(timeout=1.0))
        self.assertFalse(gateway.verify_nonce(nonce))

    async def test_map_preview_rejects_a_status_image_sequence_race(self) -> None:
        from go2_dashboard.state import DashboardState
        from go2_dashboard.web import create_app

        state = DashboardState()
        state.set_map(b"first-map", {"width": 1, "height": 1}, frame_id="map")
        app = create_app(state=state)
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(
            transport=transport,
            base_url="http://127.0.0.1:8092",
        ) as client:
            old_sequence = (await client.get("/api/status")).json()["topics"][
                "map"
            ]["sequence"]
            state.set_map(
                b"second-map", {"width": 2, "height": 1}, frame_id="map"
            )

            conflict = await client.get(
                f"/api/map.png?sequence={old_sequence}"
            )
            self.assertEqual(conflict.status_code, 409)
            self.assertEqual(conflict.headers["X-Telemetry-Sequence"], "2")

            current = await client.get("/api/map.png?sequence=2")
            self.assertEqual(current.status_code, 200)
            self.assertEqual(current.headers["X-Telemetry-Sequence"], "2")
            self.assertEqual(current.content, b"second-map")

    async def test_independent_camera_endpoints_and_sequence_guard(self) -> None:
        from go2_dashboard.state import DashboardState
        from go2_dashboard.web import create_app

        state = DashboardState()
        state.set_camera(b"go2", {"width": 1}, frame_id="go2")
        state.set_camera_stream(
            "d435i_color", b"color", {"width": 1}, frame_id="color"
        )
        state.set_camera_stream(
            "d435i_depth", b"depth", {"width": 1}, frame_id="depth"
        )
        transport = httpx.ASGITransport(app=create_app(state=state))
        async with httpx.AsyncClient(
            transport=transport,
            base_url="http://127.0.0.1:8092",
        ) as client:
            for stream, content in (
                ("go2", b"go2"),
                ("d435i-color", b"color"),
                ("d435i-depth", b"depth"),
            ):
                response = await client.get(f"/api/cameras/{stream}.jpg?sequence=1")
                self.assertEqual(response.status_code, 200)
                self.assertEqual(response.content, content)
            state.set_camera_stream(
                "d435i_color", b"color-2", {"width": 2}, frame_id="color"
            )
            stale = await client.get("/api/cameras/d435i-color.jpg?sequence=1")
            self.assertEqual(stale.status_code, 409)
            self.assertEqual(stale.headers["X-Telemetry-Sequence"], "2")
            self.assertEqual(
                (await client.get("/api/cameras/unknown.jpg")).status_code,
                422,
            )

    async def test_initial_pose_http_actions_require_exact_live_identity(self) -> None:
        from go2_dashboard.ros_bridge import RosConfig
        from go2_dashboard.state import DashboardState
        from go2_dashboard.web import create_app

        with tempfile.TemporaryDirectory() as temporary:
            maps = Path(temporary)
            map_dir = maps / "lab"
            map_dir.mkdir()
            (map_dir / "rtabmap.db").write_bytes(b"db")
            (map_dir / "generation").write_text("3\n", encoding="utf-8")
            state = DashboardState()
            app = create_app(
                state=state,
                config=RosConfig(initial_pose_maps_dir=str(maps)),
            )
            store = app.state.ros_bridge._initial_pose_store
            self.assertIsNotNone(store)
            store.observe_lifecycle("lab", "localization", 3)
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = 0.25
            covariance[35] = 0.068
            store.save_operator_pose(
                {
                    "frame_id": "map",
                    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 1.0,
                    },
                    "covariance": covariance,
                }
            )
            state.set_initial_pose_status(store.status())
            transport = httpx.ASGITransport(app=app)
            async with httpx.AsyncClient(
                transport=transport,
                base_url="http://127.0.0.1:8092",
            ) as client:
                status = (await client.get("/api/initial-pose")).json()
                self.assertTrue(status["saved"])
                mismatch = await client.post(
                    "/api/initial-pose/restore",
                    json={"map_id": "lab", "generation": 4},
                )
                self.assertEqual(mismatch.status_code, 409)
                accepted = await client.post(
                    "/api/initial-pose/restore",
                    json={"map_id": "lab", "generation": 3},
                )
                self.assertEqual(accepted.status_code, 202)
                reset = await client.post(
                    "/api/initial-pose/reset",
                    json={"confirm_map_id": "lab", "generation": 3},
                )
                self.assertEqual(reset.status_code, 200)
                self.assertFalse(reset.json()["saved"])


if __name__ == "__main__":
    unittest.main()
