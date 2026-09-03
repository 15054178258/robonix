import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
SCRIPT = REPO / "scripts" / "staged_nav2_goal_dispatch.py"
sys.path.insert(0, str(ROOT))

from go2_chassis.staged_nav2_permit import (  # noqa: E402
    ConsumedGoalPermit,
    PermitError,
    STAGED_NAV2_ACK,
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "staged_nav2_goal_dispatch", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class StagedNav2GoalDispatchTest(unittest.TestCase):
    def setUp(self) -> None:
        self.module = _load_module()
        self.environment = {
            "GO2_ALLOW_MOTION": "true",
            "GO2_MOTION_PROFILE": (
                "workstation-staged-nav2-corrected-v1"
            ),
            "GO2_STAGED_NAV2_STAGE": "stage1",
            "GO2_STAGED_NAV2_GUARD_ACK": STAGED_NAV2_ACK,
            "GO2_NETWORK_INTERFACE": "wlan-stage",
            "GO2_SDK_SOCKET": "/tmp/go2-stage.sock",
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "100",
            "GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256": "a" * 64,
        }
        self.permit = ConsumedGoalPermit(
            path=Path("/tmp/consumed"),
            permit_id="permit-0123456789abcdef",
            pair_id="pair-0123456789abcdef",
            session_id="session-0123456789abcdef",
            map_id="go2_test_map_01",
            map_generation=2,
            goal_source="operator_reviewed_short_goal",
            target_id="short-goal-01",
            goal_x=0.30,
            goal_y=-0.10,
            goal_yaw=0.20,
        )

    def test_exact_environment_builds_fixed_stage1_runtime(self) -> None:
        runtime = self.module._runtime_from_environment(self.environment)
        self.assertEqual(
            runtime.twist_in_topic, "/go2/staged_nav2/cmd_vel"
        )
        self.assertEqual(runtime.max_linear_x_mps, 0.30)
        self.assertEqual(runtime.max_angular_z_rps, 0.40)
        self.assertEqual(runtime.commissioning_max_duration_s, 0.0)
        self.assertEqual(runtime.commissioning_max_distance_m, 0.0)

    def test_claim_is_exact_and_bound_to_consumed_permit(self) -> None:
        claim = self.module.build_goal_claim(
            self.permit, "0123456789abcdef0123456789abcdef", self.environment
        )
        self.assertEqual(
            set(claim),
            {
                "schema",
                "session_id",
                "pair_id",
                "source",
                "target_id",
                "map_id",
                "generation",
                "pose",
                "goal_evidence_sha256",
                "goal_uuid",
            },
        )
        self.assertEqual(claim["pose"], {"x": 0.3, "y": -0.1, "yaw": 0.2})
        self.assertEqual(claim["pair_id"], self.permit.pair_id)
        self.assertEqual(claim["goal_evidence_sha256"], "a" * 64)

    def test_missing_claim_or_malformed_uuid_fails_closed(self) -> None:
        for mutation in (
            {"GO2_ALLOW_MOTION": "1"},
            {"GO2_STAGED_NAV2_STAGE": "stage2"},
            {"GO2_ALLOWED_MODES": ""},
        ):
            with self.subTest(mutation=mutation), self.assertRaises(
                (PermitError, ValueError)
            ):
                self.module._runtime_from_environment(
                    {**self.environment, **mutation}
                )
        with self.assertRaises(PermitError):
            self.module.build_goal_claim(
                self.permit, "not-a-uuid", self.environment
            )

    def test_ros_endpoints_are_built_only_after_atomic_goal_consumption(self):
        source = SCRIPT.read_text(encoding="utf-8")
        main = source.index("def main()")
        paths = source.index("result_paths(environ)", main)
        consume = source.index("consume_staged_nav2_goal_permit(", main)
        run = source.index(
            "return _run_ros(consumed, environ, action_result_path)",
            consume,
        )
        self.assertLess(paths, consume)
        self.assertLess(consume, run)
        run_ros = source.index("def _run_ros(")
        ros_import = source.index("    import rclpy", run_ros)
        self.assertGreater(ros_import, run_ros)
        self.assertNotIn("create_client(SetBool", source)
        self.assertNotIn("create_publisher(Twist", source)


if __name__ == "__main__":
    unittest.main()
