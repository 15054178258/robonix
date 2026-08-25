#!/usr/bin/env python3
"""Offline safety and classification tests for Client voice E2E evidence."""

from __future__ import annotations

import ast
import copy
import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
from types import ModuleType
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "client_voice_e2e_nomotion.py"


def load_module():
    name = "client_voice_e2e_nomotion_under_test"
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    if importlib.util.find_spec("websockets") is None:
        websocket_stub = ModuleType("websockets")
        with mock.patch.dict(sys.modules, {"websockets": websocket_stub}):
            spec.loader.exec_module(module)
    else:
        spec.loader.exec_module(module)
    return module


voice = load_module()


def safe_manifest():
    return {
        "manifestVersion": 1,
        "name": "robonix-go2-workstation-full-nomotion-corrected",
        "env": {"GO2_ALLOW_MOTION": "false"},
        "primitive": [
            {
                "name": "go2_chassis",
                "config": {
                    "allow_motion": False,
                    "operator_present": False,
                    "safety_ack": "",
                    "twist_in_topic": "/robonix/nomotion/chassis_input_disabled",
                },
            }
        ],
        "service": [
            {
                "name": "nav2",
                "config": {
                    "velocity_output_topic": "/robonix/nomotion/cmd_vel",
                },
            }
        ],
    }


def safe_page_state():
    return {
        "url": "http://127.0.0.1:7860/",
        "documentReady": "complete",
        "voiceActive": False,
        "hasActiveTurn": False,
        "busy": False,
        "taskRunning": False,
        "activeTurnId": "",
        "handsfree": {
            "available": True,
            "enabled": False,
            "state": "disabled",
            "busy": False,
        },
        "executorPlansReady": True,
        "executorPlanCount": 0,
        "settings": {"atlasEndpoint": "127.0.0.1:50051"},
        "button": {
            "disabled": False,
            "visible": True,
            "topmost": True,
            "x": 100.0,
            "y": 200.0,
            "text": "Voice",
        },
    }


class StaticSafetyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SCRIPT.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source, filename=str(SCRIPT))

    def test_mutating_surfaces_are_exactly_audio_injection_and_cdp_click(self) -> None:
        self.assertIn("/inject_mic", self.source)
        self.assertIn('"Input.dispatchMouseEvent"', self.source)
        self.assertIn('"Network.webSocketFrameReceived"', self.source)
        self.assertNotIn("startVoice()", self.source)
        for forbidden in (
            "ros2 topic pub",
            "ros2 service call",
            "ros2 action send_goal",
            "/api/sport/request",
            "/lowcmd",
            "nmcli connection",
            "sudo ",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_live_order_is_preflight_pcm_injection_recheck_then_click(self) -> None:
        run_source = ast.get_source_segment(
            self.source,
            next(
                node
                for node in self.tree.body
                if isinstance(node, ast.AsyncFunctionDef) and node.name == "run"
            ),
        )
        self.assertIsNotNone(run_source)
        assert run_source is not None
        self.assertLess(run_source.index("full_preflight(cdp)"), run_source.index("if not args.execute_live"))
        self.assertLess(run_source.index("if not args.execute_live"), run_source.index("inject_pcm(raw)"))
        self.assertLess(run_source.index("inject_pcm(raw)"), run_source.index("fast_click_gate(cdp)"))
        self.assertLess(run_source.index("fast_click_gate(cdp)"), run_source.index("click_voice(cdp"))
        self.assertIn("if queued and not clicked", run_source)
        self.assertIn("clear_pcm_queue()", run_source)

    def test_ros_subprocess_is_only_a_bounded_graph_list(self) -> None:
        calls = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "subprocess"
        ]
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0].func.attr, "run")
        self.assertIn("timeout", {keyword.arg for keyword in calls[0].keywords})

    def test_read_only_http_allowlist_is_exact(self) -> None:
        self.assertEqual(
            voice.ALLOWED_HTTP,
            frozenset(
                {
                    (8092, "GET", "/healthz"),
                    (8092, "GET", "/api/status"),
                    (7860, "GET", "/api/defaults"),
                    (7860, "GET", "/api/settings"),
                    (7860, "POST", "/api/handsfree/status"),
                    (7860, "POST", "/api/executor/active-plans"),
                    (9223, "GET", "/json/list"),
                }
            ),
        )
        with self.assertRaisesRegex(voice.GateError, "outside the read-only allowlist"):
            voice.request_json(7860, "POST", "/api/handsfree/set", {})


class ManifestGateTest(unittest.TestCase):
    def test_exact_nomotion_manifest_passes(self) -> None:
        result = voice.validate_nomotion_manifest(safe_manifest())
        self.assertIs(result["chassis_allow_motion"], False)
        self.assertEqual(result["nav2_velocity_output_topic"], voice.ISOLATED_CMD_VEL)

    def test_every_motion_isolation_field_fails_closed(self) -> None:
        mutations = (
            (lambda item: item["env"].__setitem__("GO2_ALLOW_MOTION", "true"), "GO2_ALLOW_MOTION"),
            (lambda item: item["primitive"][0]["config"].__setitem__("allow_motion", True), "allow_motion"),
            (lambda item: item["primitive"][0]["config"].__setitem__("operator_present", True), "operator_present"),
            (lambda item: item["primitive"][0]["config"].__setitem__("safety_ack", "approved"), "safety_ack"),
            (lambda item: item["primitive"][0]["config"].__setitem__("twist_in_topic", "/cmd_vel"), "isolated"),
            (lambda item: item["service"][0]["config"].__setitem__("velocity_output_topic", "/cmd_vel"), "isolated"),
        )
        for mutate, message in mutations:
            with self.subTest(message=message):
                manifest = copy.deepcopy(safe_manifest())
                mutate(manifest)
                with self.assertRaisesRegex(voice.GateError, message):
                    voice.validate_nomotion_manifest(manifest)


class RuntimeGateTest(unittest.TestCase):
    def test_safe_page_handsfree_and_plans_pass(self) -> None:
        self.assertEqual(voice.validate_page_state(safe_page_state())["executorPlanCount"], 0)
        self.assertFalse(
            voice.validate_handsfree(
                {"available": True, "enabled": False, "state": "disabled"}
            )["enabled"]
        )
        self.assertEqual(
            voice.validate_active_plans(
                {"available": True, "count": 0, "plans": []}
            )["count"],
            0,
        )

    def test_page_refuses_handsfree_task_plan_and_disabled_button(self) -> None:
        mutations = (
            (lambda item: item["handsfree"].__setitem__("enabled", True), "Hands-free is enabled"),
            (lambda item: item.__setitem__("hasActiveTurn", True), "active task turn"),
            (lambda item: item.__setitem__("executorPlanCount", 1), "active Executor plans"),
            (lambda item: item["button"].__setitem__("disabled", True), "not safely clickable"),
        )
        for mutate, message in mutations:
            with self.subTest(message=message):
                page = copy.deepcopy(safe_page_state())
                mutate(page)
                with self.assertRaisesRegex(voice.GateError, message):
                    voice.validate_page_state(page)

    def test_dashboard_requires_preview_zero_calls_idle_and_read_only(self) -> None:
        health = {"ok": True, "telemetry_read_only": True}
        status = {
            "telemetry_read_only": True,
            "voice": {
                "enabled": True,
                "active": False,
                "execution_mode": "preview",
                "direct_robot_control": False,
                "capability_calls_observed": 0,
            },
            "semantic_task": {"read_only_effect": True},
            "navigation": {"status": "idle"},
        }
        self.assertEqual(
            voice.validate_dashboard(health, status)["voice"]["execution_mode"],
            "preview",
        )
        for key, value, message in (
            ("execution_mode", "live", "not preview"),
            ("capability_calls_observed", 1, "capability calls"),
            ("direct_robot_control", True, "direct robot control"),
        ):
            changed = copy.deepcopy(status)
            changed["voice"][key] = value
            with self.assertRaisesRegex(voice.GateError, message):
                voice.validate_dashboard(health, changed)

    def test_plan_count_rejects_boolean_and_nonempty_list(self) -> None:
        for payload in (
            {"available": True, "count": False, "plans": []},
            {"available": True, "count": 0, "plans": [{}]},
            {"available": False, "count": 0, "plans": []},
        ):
            with self.assertRaises(voice.GateError):
                voice.validate_active_plans(payload)


class PcmAndTraceTest(unittest.TestCase):
    def test_fixed_phrase_normalization_is_narrow(self) -> None:
        self.assertEqual(voice.normalize_transcript(" 机器 狗保持静止。 "), voice.EXPECTED_PHRASE)
        self.assertNotEqual(voice.normalize_transcript("机器狗不要移动"), voice.EXPECTED_PHRASE)

    def test_pcm_requires_framed_non_silent_workspace_audio(self) -> None:
        with tempfile.NamedTemporaryFile(
            dir=ROOT / "rbnx-build", suffix=".pcm", delete=False
        ) as stream:
            path = Path(stream.name)
            frame = struct.pack("<1600h", *([1200] * 1600))
            stream.write(frame * 5)
        try:
            raw, result = voice.validate_pcm(path)
            self.assertEqual(len(raw), voice.FRAME_BYTES * 15)
            self.assertEqual(result["source_duration_seconds"], 0.5)
            self.assertEqual(result["duration_seconds"], 1.5)
            self.assertEqual(result["trailing_silence_seconds"], 1.0)
            self.assertEqual(result["expected_phrase"], voice.EXPECTED_PHRASE)
        finally:
            path.unlink()

    def test_voice_trace_accepts_one_exact_complete_event_sequence(self) -> None:
        trace = voice.VoiceTrace(
            request_id="request-1",
            accepted=True,
            done=True,
            closed=True,
            sent_frames=1,
        )
        for kind in (
            "session_started",
            "recording_started",
            "recording_done",
            "asr_final",
            "tts_started",
            "tts_done",
            "session_done",
        ):
            event = {"kind": kind}
            if kind == "asr_final":
                event["text"] = "机器狗保持静止。"
            trace.observe_payload({"type": "voice_event", "event": event})
        self.assertEqual(trace.acceptance_reasons(), [])

    def test_voice_trace_fails_wrong_phrase_missing_events_and_errors(self) -> None:
        trace = voice.VoiceTrace(
            request_id="request-1",
            accepted=True,
            done=True,
            closed=True,
            sent_frames=1,
        )
        trace.observe_payload(
            {"type": "voice_event", "event": {"kind": "asr_final", "text": "向前走"}}
        )
        trace.observe_payload({"type": "error", "error": "test failure"})
        reasons = trace.acceptance_reasons()
        self.assertIn("asr_final_did_not_match_fixed_phrase", reasons)
        self.assertIn("voice_errors_observed", reasons)
        self.assertTrue(any(item.startswith("missing_voice_events:") for item in reasons))


if __name__ == "__main__":
    unittest.main()
