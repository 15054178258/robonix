from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC = ROOT / "deploy" / "time-sync"
CHASSIS_PACKAGE = ROOT / "packages" / "go2_chassis"
sys.path.insert(0, str(TIME_SYNC))
sys.path.insert(0, str(CHASSIS_PACKAGE))

from go2_chassis import runtime_config  # noqa: E402
import render_workstation_staged_nav2_manifest as manifest  # noqa: E402
import workstation_nomotion_cloud_relay as cloud_relay  # noqa: E402


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


motion_guard = _load(
    "staged_nav2_motion_guard_cross_contract",
    ROOT / "scripts" / "staged_nav2_motion_guard.py",
)
stage1_params = _load(
    "render_staged_nav2_params_cross_contract",
    ROOT / "scripts" / "render_staged_nav2_params.py",
)
goal_dispatch = _load(
    "staged_nav2_goal_dispatch_cross_contract",
    ROOT / "scripts" / "staged_nav2_goal_dispatch.py",
)


class StagedNav2CrossContractTest(unittest.TestCase):
    def test_both_staged_consumers_share_robot_root_evidence_scope(self) -> None:
        main_source = (
            ROOT
            / "packages"
            / "go2_chassis"
            / "go2_chassis"
            / "main.py"
        ).read_text(encoding="utf-8")
        dispatcher_source = (
            ROOT / "scripts" / "staged_nav2_goal_dispatch.py"
        ).read_text(encoding="utf-8")
        launcher_source = (
            ROOT / "scripts" / "start_workstation_staged_nav2_corrected.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "consume_staged_nav2_permit(",
            main_source,
        )
        self.assertIn(
            "consume_first_motion_permit(\n"
            "                    runtime, os.environ, _package_root.parents[1]\n"
            "                )",
            main_source,
        )
        self.assertIn(
            "consume_staged_nav2_goal_permit(",
            dispatcher_source,
        )
        self.assertEqual(
            launcher_source.count('--package-root "$ROOT"'),
            2,
        )

    def test_command_graph_topics_and_owners_are_exact(self) -> None:
        self.assertEqual(
            motion_guard.INPUT_TOPIC,
            runtime_config.STAGED_NAV2_NAV_COMMAND_TOPIC,
        )
        self.assertEqual(
            motion_guard.INPUT_TOPIC,
            manifest.NAV2_GUARD_INPUT_TOPIC,
        )
        self.assertEqual(
            motion_guard.OUTPUT_TOPIC,
            runtime_config.STAGED_NAV2_COMMAND_TOPIC,
        )
        self.assertEqual(
            motion_guard.OUTPUT_TOPIC,
            manifest.CHASSIS_COMMAND_TOPIC,
        )
        self.assertEqual(motion_guard.CANONICAL_COMMAND_TOPIC, "/cmd_vel")
        self.assertEqual(
            motion_guard.EXPECTED_CONTROLLER_NODE, "/velocity_smoother"
        )
        self.assertEqual(
            motion_guard.EXPECTED_CHASSIS_NODE, "/go2_chassis_adapter"
        )

    def test_goal_dispatch_claim_is_the_guards_only_goal_identity(self) -> None:
        self.assertEqual(
            goal_dispatch.CLAIM_TOPIC, motion_guard.GOAL_CLAIM_TOPIC
        )
        self.assertEqual(
            goal_dispatch.CLAIM_SCHEMA, motion_guard.GOAL_CLAIM_SCHEMA
        )
        self.assertEqual(
            f"/{goal_dispatch.NODE_NAME}",
            motion_guard.EXPECTED_GOAL_DISPATCH_NODE,
        )
        self.assertEqual(
            goal_dispatch.ACTION_NAME,
            motion_guard.GOAL_STATUS_TOPIC.removesuffix(
                "/_action/status"
            ),
        )

    def test_sensor_state_map_and_scan_route_is_one_consistent_contract(
        self,
    ) -> None:
        self.assertEqual(cloud_relay.OUTPUT_TOPIC, manifest.PRIVATE_CLOUD)
        self.assertEqual(
            motion_guard.STATE_TOPIC, manifest.STATE_TOPIC
        )
        self.assertEqual(motion_guard.ODOM_TOPIC, manifest.CANONICAL_ODOM)
        self.assertEqual(motion_guard.SCAN_TOPIC, "/scanner/scan")
        self.assertEqual(motion_guard.MAP_TOPIC, "/map")
        self.assertEqual(
            motion_guard.LOCALIZATION_TOPIC, "/robonix/map/pose"
        )
        self.assertEqual(
            motion_guard.MAP_LIFECYCLE_TOPIC,
            "/robonix/map/lifecycle",
        )
        self.assertLess(
            cloud_relay.MOTION_PUBLISH_PERIOD_NS,
            round(
                motion_guard.FRESHNESS_LIMIT_S["scan"]
                * 1_000_000_000
            ),
        )
        self.assertEqual(
            cloud_relay.MOTION_CLOUD_MAX_CORRECTED_AGE_NS,
            round(
                motion_guard.SOURCE_STAMP_LIMIT_S["scan"]
                * 1_000_000_000
            ),
        )

    def test_profile_and_stage1_envelope_match_end_to_end(self) -> None:
        self.assertEqual(
            motion_guard.PROFILE, runtime_config.STAGED_NAV2_PROFILE
        )
        self.assertEqual(
            motion_guard.STAGE, runtime_config.STAGED_NAV2_STAGE
        )
        expected = {
            "max_linear_x_mps": runtime_config.STAGED_NAV2_MAX_VX_MPS,
            "max_linear_y_mps": runtime_config.STAGED_NAV2_MAX_VY_MPS,
            "max_angular_z_rps": runtime_config.STAGED_NAV2_MAX_WZ_RPS,
            "max_linear_accel_mps2": (
                runtime_config.STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2
            ),
            "max_angular_accel_rps2": (
                runtime_config.STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2
            ),
            "max_distance_m": runtime_config.STAGED_NAV2_MAX_DISTANCE_M,
            "max_duration_s": runtime_config.STAGED_NAV2_MAX_DURATION_S,
            "command_timeout_s": (
                runtime_config.STAGED_NAV2_COMMAND_TIMEOUT_S
            ),
        }
        for field, value in expected.items():
            with self.subTest(field=field):
                self.assertEqual(getattr(motion_guard.LIMITS, field), value)

    def test_materialized_nav2_producers_fit_guard_envelope(self) -> None:
        source = yaml.safe_load(
            (ROOT / "config" / "nav2_params_go2.yaml").read_text(
                encoding="utf-8"
            )
        )
        rendered = stage1_params.render_stage1_params(source)
        controller = rendered["controller_server"]["ros__parameters"][
            "FollowPath"
        ]
        smoother = rendered["velocity_smoother"]["ros__parameters"]
        limits = motion_guard.LIMITS

        self.assertEqual(
            controller["plugin"],
            "nav2_rotation_shim_controller::RotationShimController",
        )
        self.assertEqual(
            controller["primary_controller"],
            "dwb_core::DWBLocalPlanner",
        )
        self.assertTrue(controller["rotate_to_goal_heading"])
        self.assertFalse(controller["closed_loop"])
        self.assertLessEqual(
            controller["rotate_to_heading_angular_vel"],
            limits.max_angular_z_rps,
        )
        self.assertLessEqual(
            controller["max_angular_accel"],
            limits.max_angular_accel_rps2,
        )
        self.assertEqual(controller["min_vel_x"], 0.0)
        self.assertEqual(controller["min_vel_y"], 0.0)
        self.assertEqual(controller["max_vel_y"], 0.0)
        self.assertEqual(controller["min_speed_xy"], 0.01)
        self.assertEqual(controller["xy_goal_tolerance"], 0.35)
        self.assertEqual(controller["acc_lim_y"], 0.0)
        self.assertEqual(controller["decel_lim_y"], 0.0)
        self.assertGreaterEqual(controller["max_vel_x"], 0.0)
        self.assertLessEqual(
            controller["max_vel_x"], limits.max_linear_x_mps
        )
        self.assertLessEqual(
            controller["max_speed_xy"], limits.max_linear_x_mps
        )
        self.assertLessEqual(
            controller["max_speed_xy"], controller["max_vel_x"]
        )
        self.assertGreaterEqual(controller["min_speed_theta"], 0.0)
        self.assertLessEqual(
            controller["min_speed_theta"],
            controller["max_vel_theta"],
        )
        self.assertLessEqual(
            controller["max_vel_theta"], limits.max_angular_z_rps
        )
        self.assertGreaterEqual(controller["acc_lim_x"], 0.0)
        self.assertLessEqual(
            controller["acc_lim_x"], limits.max_linear_accel_mps2
        )
        self.assertGreaterEqual(controller["acc_lim_theta"], 0.0)
        self.assertLessEqual(
            controller["acc_lim_theta"],
            limits.max_angular_accel_rps2,
        )
        self.assertLessEqual(controller["decel_lim_x"], 0.0)
        self.assertLessEqual(
            abs(controller["decel_lim_x"]),
            limits.max_linear_accel_mps2,
        )
        self.assertLessEqual(controller["decel_lim_theta"], 0.0)
        self.assertLessEqual(
            abs(controller["decel_lim_theta"]),
            limits.max_angular_accel_rps2,
        )

        max_velocity = smoother["max_velocity"]
        min_velocity = smoother["min_velocity"]
        max_accel = smoother["max_accel"]
        max_decel = smoother["max_decel"]
        for vector in (
            max_velocity,
            min_velocity,
            max_accel,
            max_decel,
        ):
            self.assertEqual(len(vector), 3)
            self.assertEqual(vector[1], 0.0)
        self.assertEqual(min_velocity[0], 0.0)
        self.assertGreaterEqual(max_velocity[0], 0.0)
        self.assertLessEqual(max_velocity[0], limits.max_linear_x_mps)
        self.assertLessEqual(
            max_velocity[0], controller["max_vel_x"]
        )
        self.assertGreaterEqual(max_velocity[2], 0.0)
        self.assertLessEqual(max_velocity[2], limits.max_angular_z_rps)
        self.assertLessEqual(
            max_velocity[2], controller["max_vel_theta"]
        )
        self.assertLessEqual(min_velocity[2], 0.0)
        self.assertLessEqual(
            abs(min_velocity[2]), limits.max_angular_z_rps
        )
        self.assertLessEqual(
            abs(min_velocity[2]), controller["max_vel_theta"]
        )
        self.assertGreaterEqual(max_accel[0], 0.0)
        self.assertLessEqual(
            max_accel[0], limits.max_linear_accel_mps2
        )
        self.assertLessEqual(max_accel[0], controller["acc_lim_x"])
        self.assertGreaterEqual(max_accel[2], 0.0)
        self.assertLessEqual(
            max_accel[2], limits.max_angular_accel_rps2
        )
        self.assertLessEqual(max_accel[2], controller["acc_lim_theta"])
        self.assertLessEqual(max_decel[0], 0.0)
        self.assertLessEqual(
            abs(max_decel[0]), limits.max_linear_accel_mps2
        )
        self.assertLessEqual(
            abs(max_decel[0]), abs(controller["decel_lim_x"])
        )
        self.assertLessEqual(max_decel[2], 0.0)
        self.assertLessEqual(
            abs(max_decel[2]), limits.max_angular_accel_rps2
        )
        self.assertLessEqual(
            abs(max_decel[2]), abs(controller["decel_lim_theta"])
        )


if __name__ == "__main__":
    unittest.main()
