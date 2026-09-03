from __future__ import annotations

import math
import unittest

from go2_dashboard.state import DashboardState, dashboard_profile


class _Clock:
    def __init__(self, value: float) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value


class StateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.monotonic = _Clock(10.0)
        self.wall = _Clock(1_700_000_000.0)
        self.state = DashboardState(
            monotonic_fn=self.monotonic, wall_fn=self.wall
        )

    def test_topic_transitions_from_missing_to_fresh_to_stale(self) -> None:
        self.assertEqual(self.state.snapshot()["topics"]["odom"]["state"], "missing")
        self.state.observe("odom", {"position": {"x": 0.0}}, frame_id="odom")
        current = self.state.snapshot()["topics"]["odom"]
        self.assertEqual(current["state"], "fresh")
        self.assertEqual(current["sequence"], 1)
        self.assertEqual(current["age_s"], 0.0)
        self.assertEqual(current["receipt_age_s"], 0.0)
        self.assertIsNone(current["source_age_s"])
        self.monotonic.value = 12.0
        self.assertEqual(self.state.snapshot()["topics"]["odom"]["state"], "stale")

    def test_source_age_drives_stale_and_grows_with_receipt_age(self) -> None:
        self.state.observe(
            "pose_map",
            {"position": {"x": 0.0}},
            frame_id="map",
            source_age_s=0.25,
        )
        initial = self.state.snapshot()["topics"]["pose_map"]
        self.assertEqual(initial["state"], "fresh")
        self.assertEqual(initial["age_s"], 0.25)
        self.assertEqual(initial["receipt_age_s"], 0.0)
        self.assertEqual(initial["source_age_s"], 0.25)

        self.monotonic.value = 11.0
        aged = self.state.snapshot()["topics"]["pose_map"]
        self.assertEqual(aged["state"], "fresh")
        self.assertEqual(aged["age_s"], 1.25)
        self.assertEqual(aged["receipt_age_s"], 1.0)
        self.assertEqual(aged["source_age_s"], 1.25)

        self.monotonic.value = 12.0
        stale = self.state.snapshot()["topics"]["pose_map"]
        self.assertEqual(stale["state"], "stale")
        self.assertEqual(stale["age_s"], 2.25)
        self.assertEqual(stale["source_age_s"], 2.25)

    def test_old_or_future_source_is_stale_and_fresh_update_recovers(self) -> None:
        for source_age_s in (4.0, -4.0):
            with self.subTest(source_age_s=source_age_s):
                self.state.observe(
                    "pose_map",
                    {"position": {"x": 1.0}},
                    frame_id="map",
                    source_age_s=source_age_s,
                )
                topic = self.state.snapshot()["topics"]["pose_map"]
                self.assertEqual(topic["state"], "stale")
                self.assertEqual(topic["age_s"], 4.0)

        self.state.observe(
            "pose_map",
            {"position": {"x": 2.0}},
            frame_id="map",
            source_age_s=0.1,
        )
        recovered = self.state.snapshot()["topics"]["pose_map"]
        self.assertEqual(recovered["state"], "fresh")
        self.assertEqual(recovered["age_s"], 0.1)
        self.assertEqual(recovered["source_age_s"], 0.1)

    def test_future_source_cannot_age_back_from_stale_to_fresh(self) -> None:
        self.state.observe(
            "pose_map",
            {"position": {"x": 1.0}},
            frame_id="map",
            source_age_s=-2.0,
        )
        observed_ages = []
        for elapsed_s in (0.0, 0.5, 1.0, 1.5, 2.0):
            self.monotonic.value = 10.0 + elapsed_s
            topic = self.state.snapshot()["topics"]["pose_map"]
            self.assertEqual(topic["state"], "stale")
            observed_ages.append(topic["age_s"])
        self.assertEqual(observed_ages, sorted(observed_ages))
        self.assertEqual(observed_ages, [2.0, 2.5, 3.0, 3.5, 4.0])

    def test_source_age_rejects_non_finite_and_boolean_values(self) -> None:
        for source_age_s in (math.nan, math.inf, -math.inf, True):
            with self.subTest(source_age_s=source_age_s):
                with self.assertRaises((TypeError, ValueError)):
                    self.state.observe(
                        "pose_map",
                        {},
                        frame_id="map",
                        source_age_s=source_age_s,
                    )
        self.assertEqual(
            self.state.snapshot()["topics"]["pose_map"]["sequence"], 0
        )

    def test_transient_local_map_snapshot_does_not_expire_by_receipt_age(self) -> None:
        initial = self.state.snapshot()
        self.assertEqual(initial["topics"]["map"]["state"], "missing")
        self.assertIsNone(initial["topics"]["map"]["stale_after_s"])

        self.state.set_bridge(running=True, connected=True)
        self.state.set_map(
            b"png-data",
            {"width": 2, "height": 2},
            frame_id="map",
        )
        self.monotonic.value = 10_000.0
        current = self.state.snapshot()
        self.assertEqual(current["topics"]["map"]["state"], "fresh")
        self.assertEqual(current["topics"]["map"]["age_s"], 9_990.0)
        self.assertIsNone(current["topics"]["map"]["source_age_s"])
        self.assertTrue(current["bridge"]["connected"])

        self.state.set_bridge(running=False, connected=False, error="ROS stopped")
        disconnected = self.state.snapshot()
        self.assertEqual(disconnected["topics"]["map"]["state"], "fresh")
        self.assertFalse(disconnected["bridge"]["connected"])
        self.state.note_error("map", "invalid occupancy data")
        self.assertEqual(self.state.snapshot()["topics"]["map"]["state"], "error")

    def test_parser_error_is_visible_and_next_frame_clears_it(self) -> None:
        self.state.note_error("camera", "unsupported encoding")
        topic = self.state.snapshot()["topics"]["camera"]
        self.assertEqual(topic["state"], "error")
        self.assertIn("unsupported", topic["error"])
        self.state.set_camera(b"jpeg", {"width": 1, "height": 1}, frame_id="camera")
        self.assertEqual(self.state.snapshot()["topics"]["camera"]["state"], "fresh")

    def test_binary_previews_are_not_embedded_in_json_snapshot(self) -> None:
        self.state.set_camera(b"jpeg-data", {"width": 1}, frame_id="camera")
        self.state.set_map(b"png-data", {"width": 2, "height": 2}, frame_id="map")
        snapshot = self.state.snapshot()
        self.assertNotIn(b"jpeg-data", snapshot.values())
        self.assertEqual(self.state.camera_image(), (b"jpeg-data", 1))
        self.assertEqual(self.state.map_image(), (b"png-data", 1))

    def test_three_camera_previews_are_independent(self) -> None:
        self.state.set_camera(
            b"go2-jpeg", {"width": 640, "height": 480}, frame_id="go2"
        )
        self.state.set_camera_stream(
            "d435i_color",
            b"d435-color-jpeg",
            {"width": 640, "height": 480},
            frame_id="d435_color",
        )
        self.state.set_camera_stream(
            "d435i_depth",
            b"d435-depth-jpeg",
            {"width": 640, "height": 480},
            frame_id="d435_depth",
        )

        snapshot = self.state.snapshot()
        self.assertEqual(snapshot["cameras"]["go2"]["color"]["sequence"], 1)
        self.assertEqual(snapshot["cameras"]["d435i"]["color"]["sequence"], 1)
        self.assertEqual(snapshot["cameras"]["d435i"]["depth"]["sequence"], 1)
        self.assertEqual(self.state.camera_image("camera"), (b"go2-jpeg", 1))
        self.assertEqual(
            self.state.camera_image("d435i_color"), (b"d435-color-jpeg", 1)
        )
        self.assertEqual(
            self.state.camera_image("d435i_depth"), (b"d435-depth-jpeg", 1)
        )

    def test_camera_quality_is_exposed_without_binary_data(self) -> None:
        self.state.set_camera_quality(
            {
                "rate_hz": 1.88,
                "quality_error_ratio": 0.25,
                "healthy": False,
                "message": "x" * 500,
                "api_code_semantics": "opaque vendor return code; not interpreted",
            }
        )
        quality = self.state.snapshot()["camera_quality"]
        self.assertEqual(quality["rate_hz"], 1.88)
        self.assertFalse(quality["healthy"])
        self.assertEqual(len(quality["message"]), 240)
        self.assertIn("not interpreted", quality["api_code_semantics"])

    def test_semantic_status_update_is_metadata_only_and_bounded(self) -> None:
        task = self.state.update_semantic_task(
            {
                "task_id": "task-1",
                "target_name": "自动售货机",
                "status": "resolved",
                "message": "matched",
                "pose": {"frame_id": "map", "x": 1, "y": 2, "yaw": 0.3},
            }
        )
        self.assertTrue(task["read_only_effect"])
        self.assertEqual(task["status"], "resolved")
        self.assertEqual(task["revision"], 1)
        self.assertEqual(task["pose"]["x"], 1.0)
        with self.assertRaisesRegex(ValueError, "unsupported"):
            self.state.update_semantic_task({"status": "execute"})
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.state.update_semantic_task(
                {
                    "status": "resolved",
                    "pose": {"x": math.inf, "y": 0, "yaw": 0},
                }
            )

    def test_bridge_error_is_bounded(self) -> None:
        self.state.set_bridge(running=False, connected=False, error="x" * 1000)
        self.assertEqual(len(self.state.snapshot()["bridge"]["error"]), 400)

    def test_readonly_diagnostic_profile_never_claims_navigation_readiness(self) -> None:
        state = DashboardState(deployment_profile="readonly-diagnostic")
        profile = state.snapshot()["profile"]
        self.assertTrue(profile["diagnostic_only"])
        self.assertFalse(profile["navigation_stack_started"])
        self.assertFalse(profile["navigation_ready"])
        self.assertFalse(profile["source_time_trusted"])
        self.assertIn("NOT NAVIGATION READY", profile["warning"])
        with self.assertRaisesRegex(ValueError, "unsupported dashboard profile"):
            dashboard_profile("demo-that-looks-ready")

    def test_voice_state_is_disabled_by_default_and_session_scoped(self) -> None:
        voice = self.state.snapshot()["voice"]
        self.assertFalse(voice["enabled"])
        self.assertEqual(voice["status"], "disabled")
        self.assertFalse(voice["direct_robot_control"])
        self.state.configure_voice(
            True, {"max_duration_s": 8.0}, execution_mode="preview"
        )
        current = self.state.update_voice(
            session_id="a" * 32,
            status="recognized",
            message="ok",
            transcript="x" * 500,
            active=True,
            intent_target="自动售货机",
            intent_summary="语义导航预览：自动售货机",
            blocked_reason="GO2_ALLOW_MOTION=false",
            pilot_task_status="done",
            capability_calls_observed=0,
        )
        self.assertEqual(len(current["transcript"]), 300)
        self.assertEqual(current["execution_mode"], "preview")
        self.assertEqual(current["intent_target"], "自动售货机")
        self.assertIn("GO2_ALLOW_MOTION=false", current["blocked_reason"])
        self.assertEqual(current["capability_calls_observed"], 0)
        with self.assertRaisesRegex(ValueError, "stale"):
            self.state.update_voice(
                session_id="b" * 32,
                status="pilot",
                message="wrong session",
                active=True,
            )
        self.state.update_voice(
            session_id="a" * 32,
            status="completed",
            message="done",
            active=False,
        )
        self.assertEqual(
            self.state.voice_status()["transcript"], "x" * 300
        )
        replacement = self.state.update_voice(
            session_id="b" * 32,
            status="accepted",
            message="next",
            active=True,
        )
        self.assertEqual(replacement["session_id"], "b" * 32)

    def test_voice_execution_mode_and_call_count_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "preview or live"):
            self.state.configure_voice(True, {}, execution_mode="off")
        self.state.configure_voice(True, {}, execution_mode="preview")
        current = self.state.update_voice(
            session_id="a" * 32,
            status="pilot",
            message="plan observed",
            active=True,
            capability_calls_observed=1,
        )
        self.assertEqual(current["capability_calls_observed"], 1)
        current = self.state.update_voice(
            session_id="a" * 32,
            status="failed",
            message="blocked",
            active=False,
            capability_calls_observed=0,
        )
        self.assertEqual(current["capability_calls_observed"], 1)


if __name__ == "__main__":
    unittest.main()
