from __future__ import annotations

import io
import struct
import threading
import time
import unittest
import wave
from types import SimpleNamespace

from go2_dashboard.state import DashboardState
from go2_dashboard.voice_gateway import (
    BrowserVoiceGateway,
    VoiceConfig,
    VoiceGatewayBusy,
    VoiceGatewayDisabled,
    VoiceInputError,
    _event_update,
    _voice_event_payload,
    validate_browser_request,
    validate_wav_upload,
)


def _wav(seconds: float = 0.5, *, rate: int = 16_000, channels: int = 1) -> bytes:
    frames = max(1, int(seconds * rate))
    sample = struct.pack("<h", 800)
    with io.BytesIO() as output:
        with wave.open(output, "wb") as target:
            target.setnchannels(channels)
            target.setsampwidth(2)
            target.setframerate(rate)
            target.writeframes(sample * frames * channels)
        return output.getvalue()


class VoiceConfigTests(unittest.TestCase):
    def test_feature_is_disabled_by_default_and_requires_exact_switch(self) -> None:
        default = VoiceConfig.from_environment({})
        self.assertFalse(default.enabled)
        self.assertEqual(default.execution_mode, "preview")
        self.assertTrue(
            VoiceConfig.from_environment(
                {"GO2_DASHBOARD_BROWSER_VOICE_ENABLED": "1"}
            ).enabled
        )
        with self.assertRaisesRegex(ValueError, "exactly 0 or 1"):
            VoiceConfig.from_environment(
                {"GO2_DASHBOARD_BROWSER_VOICE_ENABLED": "true"}
            )

    def test_execution_mode_is_strict_and_visible_to_gateway_state(self) -> None:
        live = VoiceConfig.from_environment(
            {"SEMANTIC_INTENT_EXECUTION_MODE": "live"}
        )
        self.assertEqual(live.execution_mode, "live")
        with self.assertRaisesRegex(ValueError, "preview or live"):
            VoiceConfig.from_environment(
                {"SEMANTIC_INTENT_EXECUTION_MODE": "disabled"}
            )
        state = DashboardState()
        BrowserVoiceGateway(state, VoiceConfig(enabled=True))
        self.assertEqual(state.voice_status()["execution_mode"], "preview")

    def test_all_network_targets_are_literal_loopback(self) -> None:
        with self.assertRaisesRegex(ValueError, "literal loopback"):
            VoiceConfig.from_environment(
                {"GO2_DASHBOARD_LIAISON_ENDPOINT": "192.168.123.18:50081"}
            )
        with self.assertRaisesRegex(ValueError, "ws://127.0.0.1"):
            VoiceConfig.from_environment(
                {"GO2_DASHBOARD_AUDIO_BRIDGE_URL": "ws://localhost:60002/client"}
            )
        with self.assertRaisesRegex(ValueError, "credentials"):
            VoiceConfig.from_environment(
                {
                    "GO2_DASHBOARD_AUDIO_BRIDGE_URL": (
                        "ws://user:secret@127.0.0.1:60002/client"
                    )
                }
            )
        with self.assertRaisesRegex(ValueError, "literal loopback"):
            VoiceConfig(enabled=True, liaison_endpoint="example.test:50081")

    def test_duration_and_timeout_limits_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "below"):
            VoiceConfig.from_environment(
                {
                    "GO2_DASHBOARD_VOICE_MIN_SECONDS": "2",
                    "GO2_DASHBOARD_VOICE_MAX_SECONDS": "1",
                }
            )
        with self.assertRaisesRegex(ValueError, "between"):
            VoiceConfig.from_environment(
                {"GO2_DASHBOARD_VOICE_SESSION_TIMEOUT_S": "600"}
            )


class BrowserBoundaryTests(unittest.TestCase):
    def test_only_direct_same_origin_loopback_is_accepted(self) -> None:
        validate_browser_request(
            client_host="127.0.0.1",
            host_header="127.0.0.1:8092",
            origin_header="http://127.0.0.1:8092",
            forwarded_header=None,
            x_forwarded_for=None,
            sec_fetch_site="same-origin",
            server_port=8092,
        )
        with self.assertRaisesRegex(VoiceInputError, "loopback clients"):
            validate_browser_request(
                client_host="192.168.123.99",
                host_header="127.0.0.1:8092",
                origin_header="http://127.0.0.1:8092",
                forwarded_header=None,
                x_forwarded_for=None,
                sec_fetch_site="same-origin",
                server_port=8092,
            )
        with self.assertRaisesRegex(VoiceInputError, "forwarded"):
            validate_browser_request(
                client_host="127.0.0.1",
                host_header="127.0.0.1:8092",
                origin_header="http://127.0.0.1:8092",
                forwarded_header="for=127.0.0.1",
                x_forwarded_for=None,
                sec_fetch_site="same-origin",
                server_port=8092,
            )
        with self.assertRaisesRegex(VoiceInputError, "Origin"):
            validate_browser_request(
                client_host="127.0.0.1",
                host_header="127.0.0.1:8092",
                origin_header="https://attacker.example",
                forwarded_header=None,
                x_forwarded_for=None,
                sec_fetch_site="cross-site",
                server_port=8092,
            )

    def test_wav_contract_enforces_mime_shape_rate_size_and_duration(self) -> None:
        config = VoiceConfig()
        pcm, duration = validate_wav_upload(_wav(), "audio/wav", config)
        self.assertEqual(len(pcm), 16_000)
        self.assertAlmostEqual(duration, 0.5)
        with self.assertRaisesRegex(VoiceInputError, "audio/wav"):
            validate_wav_upload(_wav(), "audio/webm", config)
        with self.assertRaisesRegex(VoiceInputError, "mono"):
            validate_wav_upload(_wav(channels=2), "audio/wav", config)
        with self.assertRaisesRegex(VoiceInputError, "16 kHz"):
            validate_wav_upload(_wav(rate=48_000), "audio/wav", config)
        with self.assertRaisesRegex(VoiceInputError, "duration"):
            validate_wav_upload(_wav(seconds=0.1), "audio/wav", config)
        with self.assertRaisesRegex(VoiceInputError, "byte limit"):
            validate_wav_upload(b"x" * 300_001, "audio/wav", config)


class PilotDisplayEventTests(unittest.TestCase):
    def test_preview_task_state_exposes_target_and_blocker_without_a_call(self) -> None:
        pilot = SimpleNamespace(
            event_kind=6,
            task_state=SimpleNamespace(
                goal="语义导航预览：自动售货机",
                success_criterion=(
                    "阻塞原因：GO2_ALLOW_MOTION=false；未调用 Robonix navigation、"
                    "Nav2 或 Unitree API"
                ),
                status="done",
            ),
        )
        event = SimpleNamespace(
            event_kind=6,
            text="",
            status_message="",
            error="",
            pilot=pilot,
        )
        update = _event_update(_voice_event_payload(event), "preview")
        self.assertEqual(update["intent_target"], "自动售货机")
        self.assertEqual(update["pilot_task_status"], "done")
        self.assertIn("GO2_ALLOW_MOTION=false", update["blocked_reason"])
        self.assertEqual(update["capability_calls_observed"], 0)
        self.assertIn("未执行", update["message"])

    def test_preview_plan_call_is_counted_for_fail_closed_tripwire(self) -> None:
        call = SimpleNamespace(
            call_id="call-1",
            provider_id="provider",
            contract_id="contract",
            args_json="{}",
        )
        pilot = SimpleNamespace(
            event_kind=1,
            plan=SimpleNamespace(nodes=[SimpleNamespace(call=call)]),
        )
        event = SimpleNamespace(
            event_kind=6,
            text="",
            status_message="",
            error="",
            pilot=pilot,
        )
        update = _event_update(_voice_event_payload(event), "preview")
        self.assertEqual(update["capability_calls_observed"], 1)


class VoiceGatewayTests(unittest.TestCase):
    def test_disabled_gateway_rejects_audio_without_starting_worker(self) -> None:
        state = DashboardState()
        gateway = BrowserVoiceGateway(state, VoiceConfig())
        with self.assertRaises(VoiceGatewayDisabled):
            gateway.submit(bytearray(b"pcm"), 0.5)
        self.assertEqual(state.voice_status()["status"], "disabled")
        self.assertEqual(gateway.browser_status()["csrf_nonce"], "")

    def test_single_session_delegates_and_overwrites_transient_pcm(self) -> None:
        started = threading.Event()
        release = threading.Event()
        received: list[tuple[str, float]] = []

        def runner(config, session_id, pcm, duration, update, cancel):
            del config, cancel
            received.append((session_id, duration))
            update(status="recognized", message="识别完成", transcript="自动售货机")
            started.set()
            release.wait(timeout=2.0)
            update(status="completed", message="完成")

        state = DashboardState()
        gateway = BrowserVoiceGateway(
            state, VoiceConfig(enabled=True), runner=runner
        )
        original_nonce = gateway.browser_status()["csrf_nonce"]
        pcm = bytearray(b"\x01\x02" * 8_000)
        accepted = gateway.submit(pcm, 0.5)
        self.assertIn(accepted["status"], {"accepted", "recognized"})
        self.assertFalse(gateway.verify_nonce(original_nonce))
        self.assertTrue(started.wait(timeout=1.0))
        with self.assertRaises(VoiceGatewayBusy):
            gateway.submit(bytearray(b"next"), 0.5)
        self.assertEqual(state.voice_status()["transcript"], "自动售货机")
        release.set()
        deadline = time.monotonic() + 2.0
        while state.voice_status()["active"] and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(state.voice_status()["status"], "completed")
        self.assertEqual(pcm, bytearray())
        self.assertEqual(len(received), 1)


if __name__ == "__main__":
    unittest.main()
