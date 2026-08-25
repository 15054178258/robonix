from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "start_workstation_persistent_voice_nav2.sh"
BASE = ROOT / "scripts" / "start_workstation_staged_nav2_corrected.sh"
ARM = ROOT / "scripts" / "persistent_nav2_arm.py"


def _load_arm():
    spec = importlib.util.spec_from_file_location("persistent_nav2_arm", ARM)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load persistent arm module")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PersistentVoiceNav2LauncherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.launcher = LAUNCHER.read_text(encoding="utf-8")
        cls.base = BASE.read_text(encoding="utf-8")
        cls.arm = _load_arm()

    def test_start_is_explicit_disarmed_and_dispatches_no_goal(self) -> None:
        self.assertIn("I_AM_ON_SITE_SITE_CLEAR_REMOTE_STOP_READY", self.launcher)
        self.assertIn("GO2_PERSISTENT_NAV2_MODE=true", self.launcher)
        self.assertIn("startup is DISARMED and sends no goal", self.launcher)
        self.assertNotIn("ros2 service call", self.launcher)
        self.assertNotIn("send_goal", self.launcher)
        self.assertIn("PERSISTENT ROBONIX VOICE / NAV2 STACK READY", self.base)
        persistent_ready = self.base.index(
            "PERSISTENT ROBONIX VOICE / NAV2 STACK READY"
        )
        external_guard = self.base.index("staged_nav2_motion_guard.py")
        dispatcher = self.base.index("staged_nav2_goal_dispatch.py")
        self.assertLess(persistent_ready, external_guard)
        self.assertLess(persistent_ready, dispatcher)

    def test_start_requires_exact_map_generation_and_readonly_state_values(self) -> None:
        for value in (
            "GO2_PERSISTENT_NAV2_MAP_ID",
            "GO2_PERSISTENT_NAV2_MAP_GENERATION",
            "GO2_PERSISTENT_NAV2_ALLOWED_MODE",
            "GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER",
            'disk_generation" == "$GO2_PERSISTENT_NAV2_MAP_GENERATION',
        ):
            self.assertIn(value, self.launcher)

        self.assertIn("PASSIVE_SOURCE_MARKERS=100,1002,2010", self.base)
        self.assertIn("CLASSIC_MOTION_STATE_MARKERS=100,2010", self.base)
        self.assertIn(
            'export GO2_ALLOWED_STATE_MARKERS="$CLASSIC_MOTION_STATE_MARKERS"',
            self.base,
        )
        self.assertIn(
            "staged Nav2 current marker is outside the reviewed Classic set",
            self.base,
        )

    def test_persistent_mode_materializes_selected_landmarks_file(self) -> None:
        self.assertIn(
            '${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}',
            self.base,
        )
        self.assertIn(
            'materialize_runtime_config \\\n    "$semantic_landmarks_source"',
            self.base,
        )

    def test_arm_helper_only_publishes_bounded_zero_preamble(self) -> None:
        source = ARM.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(ARM))
        publish_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "publish"
        ]
        self.assertEqual(len(publish_calls), 1)
        self.assertEqual(ast.unparse(publish_calls[0].args[0]), "Twist()")
        self.assertNotIn("ActionClient", source)
        self.assertNotIn("send_goal", source)
        self.assertNotIn('"/cmd_vel"', source)
        self.assertIn(self.arm.ARM_ACK, source)
        self.assertEqual(self.arm.SNAPSHOT_TIMEOUT_S, 15.0)
        self.assertEqual(self.arm.INITIAL_GOAL_STATUS_OBSERVATION_S, 6.0)
        self.assertEqual(self.arm.ZERO_PREPARATION_S, 1.0)

    def test_snapshot_rejects_active_goal_or_generation_drift(self) -> None:
        now = 100.0
        diagnostics = {
            "guard_state": "DISARMED",
            "allow_motion": "true",
            "motion_profile": self.arm.PROFILE,
            "external_odom_valid": "true",
            "external_odom_fault_latched": "false",
            "daemon_armed": "false",
            "state_valid": "true",
            "opaque_state_marker_change_latched": "false",
            "passive_state_marker_transitions_enabled": "false",
            "motion_state_marker_transitions_enabled": "true",
            "sport_mode": "0",
            "sport_error_code": "100",
        }
        samples = {
            "lifecycle": (now, ("lab", "localization", 7)),
            "pose": (now, "map"),
            "odom": (now, ("odom", "base_link")),
            "scan": (now, "base_link"),
            "goals": (now, []),
            "diagnostics": (now, diagnostics),
        }
        self.arm.validate_snapshot(
            samples,
            map_id="lab",
            generation=7,
            allowed_mode=0,
            allowed_marker=100,
            now=now,
        )
        samples["goals"] = (now, ["active"])
        with self.assertRaisesRegex(self.arm.ArmError, "goal"):
            self.arm.validate_snapshot(
                samples,
                map_id="lab",
                generation=7,
                allowed_mode=0,
                allowed_marker=100,
                now=now,
            )
        samples["goals"] = (now, [])
        with self.assertRaisesRegex(self.arm.ArmError, "MapLifecycle"):
            self.arm.validate_snapshot(
                samples,
                map_id="lab",
                generation=8,
                allowed_mode=0,
                allowed_marker=100,
                now=now,
            )

    def test_rearm_accepts_only_the_alternate_classic_marker(self) -> None:
        now = 100.0
        diagnostics = {
            "guard_state": "DISARMED",
            "allow_motion": "true",
            "motion_profile": self.arm.PROFILE,
            "external_odom_valid": "true",
            "external_odom_fault_latched": "false",
            "daemon_armed": "false",
            "state_valid": "true",
            "opaque_state_marker_change_latched": "false",
            "passive_state_marker_transitions_enabled": "false",
            "motion_state_marker_transitions_enabled": "true",
            "sport_mode": "0",
            "sport_error_code": "100",
        }
        samples = {
            "lifecycle": (now, ("lab", "localization", 7)),
            "pose": (now, "map"),
            "odom": (now, ("odom", "base_link")),
            "scan": (now, "base_link"),
            "goals": (now, []),
            "diagnostics": (now, diagnostics),
        }
        self.arm.validate_snapshot(
            samples,
            map_id="lab",
            generation=7,
            allowed_mode=0,
            allowed_marker=2010,
            now=now,
        )

        diagnostics["sport_error_code"] = "1002"
        with self.assertRaisesRegex(self.arm.ArmError, "Classic set"):
            self.arm.validate_snapshot(
                samples,
                map_id="lab",
                generation=7,
                allowed_mode=0,
                allowed_marker=2010,
                now=now,
            )
        diagnostics["sport_error_code"] = "100"
        diagnostics["motion_state_marker_transitions_enabled"] = "false"
        with self.assertRaisesRegex(self.arm.ArmError, "diagnostic gate"):
            self.arm.validate_snapshot(
                samples,
                map_id="lab",
                generation=7,
                allowed_mode=0,
                allowed_marker=2010,
                now=now,
            )

    def test_initial_empty_goal_status_requires_exact_bt_navigator(self) -> None:
        samples = {}
        publisher = SimpleNamespace(node_namespace="/", node_name="bt_navigator")
        self.arm.infer_initial_idle_goal_state(samples, [publisher], now=12.0)
        self.assertEqual(samples["goals"], (12.0, []))

        for publishers in ([], [SimpleNamespace(node_namespace="/", node_name="other")]):
            with self.subTest(publishers=publishers):
                with self.assertRaisesRegex(self.arm.ArmError, "status publishers"):
                    self.arm.infer_initial_idle_goal_state({}, publishers, now=12.0)


if __name__ == "__main__":
    unittest.main()
