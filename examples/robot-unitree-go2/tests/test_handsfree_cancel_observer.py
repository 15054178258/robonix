#!/usr/bin/env python3
"""Offline tests for the bounded Hands-free cancellation observer."""

from __future__ import annotations

import ast
import asyncio
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
OBSERVER_PATH = ROOT / "scripts" / "observe_handsfree_cancel_readonly.py"


def load_observer():
    module_name = "handsfree_cancel_observer_under_test"
    spec = importlib.util.spec_from_file_location(module_name, OBSERVER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {OBSERVER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


observer = load_observer()


class HandsfreeCancelSafetyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        (ROOT / "logs").mkdir(parents=True, exist_ok=True)
        cls.source = OBSERVER_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source, filename=str(OBSERVER_PATH))

    def test_endpoint_allowlists_are_exact_and_source_has_no_other_routes(self) -> None:
        self.assertEqual(
            observer.ALLOWED_HTTP_OPERATIONS,
            frozenset(
                {
                    ("GET", "/api/settings"),
                    ("POST", "/api/handsfree/status"),
                    ("POST", "/api/executor/active-plans"),
                }
            ),
        )
        self.assertEqual(
            observer.ALLOWED_WEBSOCKET_PATHS,
            frozenset({"/ws/handsfree-events"}),
        )
        route_literals = {
            node.value
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and node.value.startswith(("/api/", "/ws/"))
        }
        self.assertEqual(
            route_literals,
            {
                "/api/settings",
                "/api/handsfree/status",
                "/api/executor/active-plans",
                "/ws/handsfree-events",
            },
        )

    def test_source_cannot_start_processes_or_address_robot_motion(self) -> None:
        imported_modules = {
            alias.name
            for node in ast.walk(self.tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in node.names
        }
        self.assertNotIn("subprocess", imported_modules)
        process_calls = {
            (node.func.value.id, node.func.attr)
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
        }
        for forbidden_call in (
            ("subprocess", "Popen"),
            ("subprocess", "run"),
            ("os", "system"),
            ("os", "execv"),
            ("os", "execve"),
        ):
            self.assertNotIn(forbidden_call, process_calls)
        for forbidden_literal in (
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "ros2",
        ):
            self.assertNotIn(forbidden_literal, self.source)

    def test_http_and_websocket_guards_reject_non_allowlisted_surfaces(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "outside the read-only allowlist"):
            asyncio.run(observer._request_json("DELETE", "/not-allowed", port=7860))
        with self.assertRaisesRegex(RuntimeError, "outside the read-only allowlist"):
            observer._websocket_uri(7860, "/ws/not-allowed")

    def test_http_observation_is_async_and_has_no_background_thread_fallback(self) -> None:
        self.assertNotIn("http.client", self.source)
        self.assertNotIn("asyncio.to_thread", self.source)
        request_nodes = [
            node
            for node in self.tree.body
            if isinstance(node, ast.AsyncFunctionDef)
            and node.name == "_request_json"
        ]
        self.assertEqual(len(request_nodes), 1)

    def test_async_http_observation_parses_a_bounded_loopback_response(self) -> None:
        async def exercise() -> None:
            async def handler(
                reader: asyncio.StreamReader, writer: asyncio.StreamWriter
            ) -> None:
                await reader.readuntil(b"\r\n\r\n")
                body = json.dumps(
                    {"ok": True, "settings": {"robotHost": "127.0.0.1"}}
                ).encode("utf-8")
                writer.write(
                    b"HTTP/1.1 200 OK\r\n"
                    + f"Content-Length: {len(body)}\r\n".encode("ascii")
                    + b"Content-Type: application/json\r\nConnection: close\r\n\r\n"
                    + body
                )
                await writer.drain()
                writer.close()

            server = await asyncio.start_server(handler, observer.LOOPBACK_HOST, 0)
            port = server.sockets[0].getsockname()[1]
            try:
                response = await asyncio.wait_for(
                    observer._request_json("GET", observer.SETTINGS_PATH, port=port),
                    timeout=0.5,
                )
                self.assertTrue(response["ok"])
                self.assertEqual(response["settings"]["robotHost"], "127.0.0.1")
            finally:
                server.close()
                await server.wait_closed()

        asyncio.run(exercise())

    def test_cancelled_http_observation_closes_its_loopback_stream(self) -> None:
        async def exercise() -> None:
            client_closed = asyncio.Event()

            async def handler(
                reader: asyncio.StreamReader, writer: asyncio.StreamWriter
            ) -> None:
                try:
                    await reader.readuntil(b"\r\n\r\n")
                    await reader.read()
                finally:
                    client_closed.set()
                    writer.close()

            server = await asyncio.start_server(handler, observer.LOOPBACK_HOST, 0)
            port = server.sockets[0].getsockname()[1]
            try:
                with self.assertRaises(asyncio.TimeoutError):
                    await asyncio.wait_for(
                        observer._request_json(
                            "GET", observer.SETTINGS_PATH, port=port
                        ),
                        timeout=0.05,
                    )
                await asyncio.wait_for(client_closed.wait(), timeout=0.5)
            finally:
                server.close()
                await server.wait_closed()

        asyncio.run(exercise())

    def test_settings_response_requires_explicit_success(self) -> None:
        self.assertEqual(
            observer._settings_from_response(
                {"ok": True, "settings": {"robotHost": "127.0.0.1"}}
            ),
            {"robotHost": "127.0.0.1"},
        )
        for payload in (
            {"ok": False, "settings": {}},
            {"settings": {}},
            {"ok": True, "settings": None},
        ):
            with self.assertRaises(RuntimeError):
                observer._settings_from_response(payload)

    def test_websocket_observation_clears_audio_routes_without_mutating_settings(self) -> None:
        settings = {
            "atlasEndpoint": "127.0.0.1:50051",
            "sessionId": "test-session",
            "micNodeId": "audio-bridge",
            "micDeviceId": "default",
            "speakerNodeId": "audio-bridge",
            "speakerDeviceId": "default",
        }
        observed = observer._settings_for_event_observation(settings)
        self.assertEqual(settings["micNodeId"], "audio-bridge")
        self.assertEqual(settings["speakerNodeId"], "audio-bridge")
        for key in observer.AUDIO_ROUTE_SETTING_KEYS:
            self.assertEqual(observed[key], "")
        self.assertEqual(observed["atlasEndpoint"], "127.0.0.1:50051")
        self.assertEqual(observed["sessionId"], "test-session")

    def test_websocket_connect_is_preconnected_loopback_and_proxy_disabled(self) -> None:
        connect_calls = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "connect"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "websockets"
        ]
        self.assertEqual(len(connect_calls), 1)
        keywords = {keyword.arg: keyword.value for keyword in connect_calls[0].keywords}
        self.assertIn("sock", keywords)
        self.assertIsInstance(keywords.get("proxy"), ast.Constant)
        self.assertIsNone(keywords["proxy"].value)

    def test_live_observation_requires_an_explicit_runtime_gate(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as parent:
            output = Path(parent) / "must-not-exist"
            stderr = io.StringIO()
            with redirect_stderr(stderr):
                result = observer.main(
                    [
                        "--output-dir",
                        str(output),
                        "--duration-seconds",
                        "30",
                    ]
                )
            self.assertEqual(result, 2)
            self.assertIn("--observe-live", stderr.getvalue())
            self.assertFalse(output.exists())

    def test_ready_requires_stream_enabled_state_and_zero_plan_baseline(self) -> None:
        class RecordingWriter:
            def __init__(self) -> None:
                self.records = []

            def write(self, record) -> None:
                self.records.append(record)

        writer = RecordingWriter()
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        output = io.StringIO()
        with redirect_stdout(output):
            observer._announce_ready_if_needed(
                writer, tracker, observed_monotonic_ns=1_100_000_000
            )
            tracker.websocket_accepted = True
            observer._announce_ready_if_needed(
                writer, tracker, observed_monotonic_ns=1_200_000_000
            )
            tracker.saw_enabled = True
            observer._announce_ready_if_needed(
                writer, tracker, observed_monotonic_ns=1_300_000_000
            )
            tracker.pre_disable_plan_samples = 1
            observer._announce_ready_if_needed(
                writer, tracker, observed_monotonic_ns=1_400_000_000
            )
        self.assertEqual(output.getvalue().count("READY:"), 1)
        self.assertEqual([item["record_type"] for item in writer.records], ["operator_ready"])


class HandsfreeCancelClassificationTest(unittest.TestCase):
    def test_voice_event_classifier_accepts_only_well_formed_voice_events(self) -> None:
        self.assertEqual(
            observer.classify_voice_event(
                {"type": "voice_event", "event": {"kind": " ASR_Final "}}
            ),
            "asr_final",
        )
        for payload in (
            {},
            {"type": "accepted"},
            {"type": "voice_event", "event": None},
            {"type": "voice_event", "event": {"kind": 7}},
            {"type": "voice_event", "event": {"kind": ""}},
        ):
            self.assertIsNone(observer.classify_voice_event(payload))

    def test_malformed_status_and_websocket_schema_fail_closed(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        status = tracker.observe_status(
            {"available": True, "enabled": True, "state": "listening"},
            observed_monotonic_ns=2_000_000_000,
            observed_at_utc="2026-07-20T00:00:01.000Z",
        )
        malformed_event = tracker.observe_websocket(
            {"type": "voice_event", "event": {}},
            observed_monotonic_ns=2_100_000_000,
        )
        unknown_event = tracker.observe_websocket(
            {"type": "voice_event", "event": {"kind": "future_kind"}},
            observed_monotonic_ns=2_200_000_000,
        )
        self.assertTrue(status["malformed"])
        self.assertTrue(malformed_event["malformed"])
        self.assertTrue(unknown_event["unknown_event_kind"])
        summary = tracker.summary(
            requested_duration_seconds=5.0,
            poll_interval_seconds=0.5,
            finished_monotonic_ns=3_000_000_000,
            interrupted=False,
        )
        reasons = summary["acceptance"]["failure_reasons"]
        self.assertIn("handsfree_status_schema_malformed", reasons)
        self.assertIn("websocket_message_schema_malformed", reasons)
        self.assertIn("websocket_unknown_voice_event_kind", reasons)

    def test_endpoint_sample_gap_over_one_second_fails_closed(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        status_payload = {
            "available": True,
            "enabled": True,
            "state": "listening",
            "lastTranscript": "",
        }
        tracker.observe_status(
            status_payload,
            observed_monotonic_ns=2_000_000_000,
            observed_at_utc="2026-07-20T00:00:01.000Z",
        )
        tracker.observe_status(
            status_payload,
            observed_monotonic_ns=3_100_000_000,
            observed_at_utc="2026-07-20T00:00:02.100Z",
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=2_000_000_000,
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=3_100_000_000,
        )
        summary = tracker.summary(
            requested_duration_seconds=5.0,
            poll_interval_seconds=0.5,
            finished_monotonic_ns=3_100_000_000,
            interrupted=False,
        )
        reasons = summary["acceptance"]["failure_reasons"]
        self.assertIn("handsfree_status_sample_gap_exceeded", reasons)
        self.assertIn("active_plans_sample_gap_exceeded", reasons)

    def test_disable_boundary_and_all_late_categories_are_counted(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        tracker.settings_loaded = True
        tracker.websocket_connected = True
        tracker.observe_websocket(
            {"type": "accepted"}, observed_monotonic_ns=1_100_000_000
        )
        tracker.observe_status(
            {
                "available": True,
                "enabled": True,
                "state": "listening",
                "lastTranscript": "开始监听",
            },
            observed_monotonic_ns=2_000_000_000,
            observed_at_utc="2026-07-20T00:00:01.000Z",
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=2_100_000_000,
        )
        tracker.observe_websocket(
            {"type": "voice_event", "event": {"kind": "asr_final"}},
            observed_monotonic_ns=2_200_000_000,
        )

        transition = tracker.observe_status(
            {
                "available": True,
                "enabled": False,
                "state": "disabled",
                "lastTranscript": "开始监听",
            },
            observed_monotonic_ns=3_000_000_000,
            observed_at_utc="2026-07-20T00:00:02.000Z",
        )
        self.assertTrue(transition["disable_transition"])
        self.assertEqual(tracker.disable_elapsed_seconds, 2.0)

        for index, kind in enumerate(observer.LATE_EVENT_KINDS, start=1):
            tracker.observe_websocket(
                {"type": "voice_event", "event": {"kind": kind}},
                observed_monotonic_ns=3_000_000_000 + index,
            )
        tracker.observe_status(
            {
                "available": True,
                "enabled": False,
                "state": "disabled",
                "lastTranscript": "关闭以后不应更新",
            },
            observed_monotonic_ns=4_000_000_000,
            observed_at_utc="2026-07-20T00:00:03.000Z",
        )
        tracker.observe_plans(
            {"available": False}, observed_monotonic_ns=4_100_000_000
        )
        tracker.observe_plans(
            {"available": True, "count": 2, "plans": [{}, {}]},
            observed_monotonic_ns=4_200_000_000,
        )

        summary = tracker.summary(
            requested_duration_seconds=5.0,
            poll_interval_seconds=0.5,
            finished_monotonic_ns=6_000_000_000,
            interrupted=False,
        )
        late = summary["post_disable_voice_events"]
        for kind in observer.LATE_EVENT_KINDS:
            self.assertEqual(late[kind], 1)
        self.assertEqual(late["tts"], 2)
        self.assertEqual(late["total"], len(observer.LATE_EVENT_KINDS))
        self.assertEqual(summary["status"]["post_disable_transcript_change_count"], 1)
        self.assertEqual(
            summary["active_plans"]["post_disable_unavailable_samples"], 1
        )
        self.assertEqual(summary["active_plans"]["post_disable_nonzero_samples"], 1)
        self.assertFalse(summary["acceptance"]["passed"])

    def test_clean_true_to_false_window_passes(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        tracker.settings_loaded = True
        tracker.websocket_connected = True
        tracker.websocket_covered_to_deadline = True
        tracker.operator_ready_announced = True
        tracker.operator_ready_at_monotonic_ns = 2_000_000_000
        tracker.operator_ready_at_utc = "2026-07-20T00:00:01.000Z"
        tracker.observe_websocket(
            {"type": "accepted"}, observed_monotonic_ns=1_100_000_000
        )
        tracker.observe_status(
            {
                "available": True,
                "enabled": True,
                "state": "listening",
                "lastTranscript": "原文本",
            },
            observed_monotonic_ns=2_000_000_000,
            observed_at_utc="2026-07-20T00:00:01.000Z",
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=2_100_000_000,
        )
        tracker.observe_status(
            {
                "available": True,
                "enabled": False,
                "state": "disabled",
                "lastTranscript": "原文本",
            },
            observed_monotonic_ns=3_000_000_000,
            observed_at_utc="2026-07-20T00:00:02.000Z",
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=3_100_000_000,
        )
        for status_ns in range(
            3_500_000_000,
            28_000_000_000,
            500_000_000,
        ):
            tracker.observe_status(
                {
                    "available": True,
                    "enabled": False,
                    "state": "disabled",
                    "lastTranscript": "原文本",
                },
                observed_monotonic_ns=status_ns,
                observed_at_utc="2026-07-20T00:00:03.000Z",
            )
            tracker.observe_plans(
                {"available": True, "count": 0, "plans": []},
                observed_monotonic_ns=status_ns + 100_000_000,
            )
        summary = tracker.summary(
            requested_duration_seconds=30.0,
            poll_interval_seconds=0.5,
            finished_monotonic_ns=28_000_000_000,
            interrupted=False,
        )
        self.assertTrue(summary["acceptance"]["passed"])
        self.assertEqual(summary["acceptance"]["failure_reasons"], [])
        self.assertEqual(
            summary["disable_boundary"]["post_disable_observation_seconds"],
            observer.MIN_POST_DISABLE_OBSERVATION_SECONDS,
        )

    def test_websocket_must_cover_the_full_window(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        tracker.settings_loaded = True
        tracker.websocket_connected = True
        tracker.websocket_accepted = True
        tracker.operator_ready_announced = True
        tracker.saw_enabled = True
        tracker.previous_enabled = True
        tracker.pre_disable_plan_samples = 1
        tracker.disable_observed_at_monotonic_ns = 3_000_000_000
        tracker.post_disable_status_samples = 1
        tracker.post_disable_plan_samples = 1
        summary = tracker.summary(
            requested_duration_seconds=30.0,
            poll_interval_seconds=0.5,
            finished_monotonic_ns=28_000_000_000,
            interrupted=False,
        )
        self.assertFalse(summary["acceptance"]["passed"])
        self.assertIn(
            "websocket_did_not_cover_full_window",
            summary["acceptance"]["failure_reasons"],
        )

    def test_malformed_plan_response_is_not_counted_as_nonzero(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        analysis = tracker.observe_plans(
            {"available": True, "count": "0", "plans": []},
            observed_monotonic_ns=2_000_000_000,
        )
        self.assertTrue(analysis["malformed"])
        self.assertFalse(analysis["nonzero"])
        self.assertEqual(tracker.pre_disable_plan_issue_samples, 1)

    def test_event_between_last_enabled_and_first_disabled_sample_fails(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        tracker.settings_loaded = True
        tracker.websocket_connected = True
        tracker.observe_websocket(
            {"type": "accepted"}, observed_monotonic_ns=1_100_000_000
        )
        tracker.observe_status(
            {
                "available": True,
                "enabled": True,
                "state": "listening",
                "lastTranscript": "唤醒中",
            },
            observed_monotonic_ns=2_000_000_000,
            observed_at_utc="2026-07-20T00:00:01.000Z",
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=2_100_000_000,
        )
        tracker.observe_websocket(
            {"type": "voice_event", "event": {"kind": "asr_final"}},
            observed_monotonic_ns=2_600_000_000,
        )
        tracker.observe_status(
            {
                "available": True,
                "enabled": False,
                "state": "disabled",
                "lastTranscript": "唤醒中",
            },
            observed_monotonic_ns=3_000_000_000,
            observed_at_utc="2026-07-20T00:00:02.000Z",
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=3_100_000_000,
        )

        summary = tracker.summary(
            requested_duration_seconds=30.0,
            poll_interval_seconds=0.5,
            finished_monotonic_ns=28_000_000_000,
            interrupted=False,
        )
        self.assertFalse(summary["acceptance"]["passed"])
        self.assertIn(
            "voice_events_in_disable_transition_window",
            summary["acceptance"]["failure_reasons"],
        )
        self.assertEqual(
            summary["disable_transition_voice_events"]["asr_final"], 1
        )
        self.assertEqual(summary["post_disable_voice_events"]["asr_final"], 0)

    def test_stale_enabled_response_cannot_erase_transition_event(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        tracker.settings_loaded = True
        tracker.websocket_connected = True
        tracker.observe_websocket(
            {"type": "accepted"}, observed_monotonic_ns=1_100_000_000
        )
        tracker.observe_status(
            {
                "available": True,
                "enabled": True,
                "state": "listening",
                "lastTranscript": "等待禁用",
            },
            observed_monotonic_ns=2_000_000_000,
            observed_at_utc="2026-07-20T00:00:01.000Z",
        )
        tracker.operator_ready_announced = True

        tracker.observe_websocket(
            {"type": "voice_event", "event": {"kind": "asr_final"}},
            observed_monotonic_ns=2_250_000_000,
        )
        # Model a status request sent before the UI click but completed after
        # the WebSocket event.  Its stale enabled=true response must not clear
        # the conservative transition window.
        tracker.observe_status(
            {
                "available": True,
                "enabled": True,
                "state": "listening",
                "lastTranscript": "等待禁用",
            },
            observed_monotonic_ns=2_300_000_000,
            observed_at_utc="2026-07-20T00:00:01.300Z",
        )
        self.assertEqual(tracker.pending_transition_event_counts["asr_final"], 1)

        transition = tracker.observe_status(
            {
                "available": True,
                "enabled": False,
                "state": "disabled",
                "lastTranscript": "等待禁用",
            },
            observed_monotonic_ns=2_500_000_000,
            observed_at_utc="2026-07-20T00:00:01.500Z",
        )
        self.assertTrue(transition["disable_transition"])
        self.assertEqual(tracker.transition_window_event_counts["asr_final"], 1)

    def test_stale_enabled_response_cannot_rebase_ready_transcript(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        tracker.observe_status(
            {
                "available": True,
                "enabled": True,
                "state": "listening",
                "lastTranscript": "关闭前基线",
            },
            observed_monotonic_ns=2_000_000_000,
            observed_at_utc="2026-07-20T00:00:01.000Z",
        )
        tracker.operator_ready_announced = True

        # Model a pre-click request that completes after a transcript change.
        # READY freezes the baseline, so this stale enabled response must not
        # make the subsequent disabled response look unchanged.
        tracker.observe_status(
            {
                "available": True,
                "enabled": True,
                "state": "listening",
                "lastTranscript": "关闭边界新文本",
            },
            observed_monotonic_ns=2_300_000_000,
            observed_at_utc="2026-07-20T00:00:01.300Z",
        )
        self.assertEqual(tracker.last_enabled_transcript, "关闭前基线")

        transition = tracker.observe_status(
            {
                "available": True,
                "enabled": False,
                "state": "disabled",
                "lastTranscript": "关闭边界新文本",
            },
            observed_monotonic_ns=2_500_000_000,
            observed_at_utc="2026-07-20T00:00:01.500Z",
        )
        self.assertTrue(transition["disable_transition"])
        self.assertTrue(tracker.transition_window_transcript_changed)

    def test_short_post_disable_window_cannot_pass(self) -> None:
        tracker = observer.AcceptanceTracker(started_monotonic_ns=1_000_000_000)
        tracker.settings_loaded = True
        tracker.websocket_connected = True
        tracker.observe_websocket(
            {"type": "accepted"}, observed_monotonic_ns=1_100_000_000
        )
        tracker.observe_status(
            {
                "available": True,
                "enabled": True,
                "state": "listening",
                "lastTranscript": "原文本",
            },
            observed_monotonic_ns=2_000_000_000,
            observed_at_utc="2026-07-20T00:00:01.000Z",
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=2_100_000_000,
        )
        tracker.observe_status(
            {
                "available": True,
                "enabled": False,
                "state": "disabled",
                "lastTranscript": "原文本",
            },
            observed_monotonic_ns=3_000_000_000,
            observed_at_utc="2026-07-20T00:00:02.000Z",
        )
        tracker.observe_plans(
            {"available": True, "count": 0, "plans": []},
            observed_monotonic_ns=3_100_000_000,
        )

        summary = tracker.summary(
            requested_duration_seconds=10.0,
            poll_interval_seconds=0.5,
            finished_monotonic_ns=11_000_000_000,
            interrupted=False,
        )
        self.assertFalse(summary["acceptance"]["passed"])
        self.assertIn(
            "post_disable_observation_too_short",
            summary["acceptance"]["failure_reasons"],
        )


class HandsfreeCancelBoundsAndEvidenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        (ROOT / "logs").mkdir(parents=True, exist_ok=True)

    def _parse_raises(self, arguments: list[str]) -> None:
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            observer.build_argument_parser().parse_args(arguments)

    def test_duration_poll_and_port_are_strictly_bounded(self) -> None:
        base = [
            "--observe-live",
            "--output-dir",
            str(ROOT / "logs" / "unused-evidence-path"),
        ]
        arguments = observer.build_argument_parser().parse_args(
            base
            + [
                "--duration-seconds",
                "30",
                "--poll-interval-seconds",
                "0.5",
                "--client-port",
                "7860",
            ]
        )
        self.assertEqual(arguments.duration_seconds, 30.0)
        self.assertEqual(arguments.poll_interval_seconds, 0.5)
        self.assertEqual(arguments.client_port, 7860)
        self._parse_raises(base + ["--duration-seconds", "29.99"])
        self._parse_raises(base + ["--duration-seconds", "300.01"])
        self._parse_raises(
            base
            + [
                "--duration-seconds",
                "30",
                "--poll-interval-seconds",
                "0.49",
            ]
        )
        self._parse_raises(
            base
            + [
                "--duration-seconds",
                "30",
                "--poll-interval-seconds",
                "0.51",
            ]
        )
        self._parse_raises(
            base
            + [
                "--duration-seconds",
                "30",
                "--client-port",
                "0",
            ]
        )

    def test_output_rejects_escape_symlink_relative_path_and_overwrite(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "must be below"):
            observer._prepare_output(Path("/tmp/handsfree-cancel-evidence"))
        with self.assertRaisesRegex(RuntimeError, "absolute path"):
            observer._prepare_output(Path("logs/relative-evidence"))

        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as parent_name:
            parent = Path(parent_name)
            output = observer._prepare_output(parent / "evidence")
            self.assertEqual(output.stat().st_mode & 0o777, 0o700)
            with self.assertRaisesRegex(RuntimeError, "refusing to overwrite"):
                observer._prepare_output(output)

            real_parent = parent / "real"
            real_parent.mkdir(mode=0o700)
            symlink_parent = parent / "alias"
            symlink_parent.symlink_to(real_parent, target_is_directory=True)
            with self.assertRaisesRegex(RuntimeError, "contains a symlink"):
                observer._prepare_output(symlink_parent / "escaped")

    def test_jsonl_and_summary_are_atomic_private_and_non_overwriting(self) -> None:
        parent = Path(tempfile.mkdtemp(prefix="handsfree-writer-", dir=ROOT / "logs"))
        try:
            output = observer._prepare_output(parent / "evidence")
            writer = observer.EvidenceWriter(output)
            writer.write({"record_type": "offline_test", "ok": True})
            writer.finalize({"schema_version": 1, "passed": True})

            log_path = output / observer.EvidenceWriter.LOG_NAME
            summary_path = output / observer.EvidenceWriter.SUMMARY_NAME
            self.assertEqual(log_path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(summary_path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(
                json.loads(log_path.read_text(encoding="utf-8")),
                {"record_type": "offline_test", "ok": True},
            )
            self.assertEqual(
                json.loads(summary_path.read_text(encoding="utf-8")),
                {"schema_version": 1, "passed": True},
            )
            self.assertFalse((output / observer.EvidenceWriter.LOG_TEMP_NAME).exists())
            self.assertFalse(
                (output / observer.EvidenceWriter.SUMMARY_TEMP_NAME).exists()
            )
        finally:
            shutil.rmtree(parent)

    def test_finalize_never_replaces_a_raced_final_name(self) -> None:
        parent = Path(tempfile.mkdtemp(prefix="handsfree-race-", dir=ROOT / "logs"))
        try:
            output = observer._prepare_output(parent / "evidence")
            writer = observer.EvidenceWriter(output)
            writer.write({"record_type": "offline_test", "ok": True})
            final_log = output / observer.EvidenceWriter.LOG_NAME
            final_log.write_text("sentinel\n", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "refusing to overwrite"):
                writer.finalize({"schema_version": 1, "passed": True})
            self.assertEqual(final_log.read_text(encoding="utf-8"), "sentinel\n")
            self.assertFalse((output / observer.EvidenceWriter.SUMMARY_NAME).exists())
        finally:
            shutil.rmtree(parent)


if __name__ == "__main__":
    unittest.main()
