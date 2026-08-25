from __future__ import annotations

from copy import deepcopy
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "render_staged_nav2_params.py"
SOURCE = ROOT / "config" / "nav2_params_go2.yaml"
SPEC = importlib.util.spec_from_file_location(
    "render_staged_nav2_params", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class StagedNav2Stage1ParamsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.document = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))

    def test_stage1_tightens_motion_producers_without_mutating_source(
        self,
    ) -> None:
        original = deepcopy(self.document)
        rendered = module.render_stage1_params(self.document)
        self.assertEqual(self.document, original)
        controller = rendered["controller_server"]["ros__parameters"][
            "FollowPath"
        ]
        goal_checker = rendered["controller_server"]["ros__parameters"][
            "go2_goal_checker"
        ]
        smoother = rendered["velocity_smoother"]["ros__parameters"]
        self.assertEqual(goal_checker["xy_goal_tolerance"], 0.35)
        self.assertEqual(goal_checker["yaw_goal_tolerance"], 0.35)
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
        self.assertEqual(controller["rotate_to_heading_angular_vel"], 0.30)
        self.assertEqual(controller["max_angular_accel"], 0.80)
        self.assertEqual(controller["min_vel_y"], 0.0)
        self.assertEqual(controller["min_vel_x"], 0.0)
        self.assertEqual(controller["max_vel_x"], 0.30)
        self.assertEqual(controller["max_vel_y"], 0.0)
        self.assertEqual(controller["max_vel_theta"], 0.40)
        self.assertEqual(controller["xy_goal_tolerance"], 0.35)
        self.assertEqual(controller["min_speed_xy"], 0.01)
        self.assertEqual(controller["max_speed_xy"], 0.30)
        self.assertEqual(controller["min_speed_theta"], 0.20)
        self.assertLessEqual(
            controller["min_speed_theta"],
            controller["max_vel_theta"],
        )
        self.assertEqual(controller["acc_lim_x"], 0.30)
        self.assertEqual(controller["acc_lim_y"], 0.0)
        self.assertEqual(controller["acc_lim_theta"], 0.80)
        self.assertEqual(controller["decel_lim_x"], -0.30)
        self.assertEqual(controller["decel_lim_y"], 0.0)
        self.assertEqual(controller["decel_lim_theta"], -0.80)
        self.assertEqual(smoother["max_velocity"], [0.30, 0.0, 0.40])
        self.assertEqual(smoother["min_velocity"], [0.0, 0.0, -0.40])
        self.assertEqual(smoother["max_accel"], [0.30, 0.0, 0.80])
        self.assertEqual(smoother["max_decel"], [-0.30, 0.0, -0.80])

        expected_changed = set()
        self.assertEqual(
            module._changed_paths(self.document, rendered),
            expected_changed,
        )
        self.assertTrue(
            expected_changed <= module.STAGE1_ALLOWED_CHANGE_PATHS
        )

    def test_shared_nomotion_parameters_remain_at_generic_tolerance(self) -> None:
        self.assertEqual(
            self.document["controller_server"]["ros__parameters"][
                "go2_goal_checker"
            ]["xy_goal_tolerance"],
            0.35,
        )

    def test_global_static_denoise_is_preserved_without_touching_local_costmap(
        self,
    ) -> None:
        rendered = module.render_stage1_params(self.document)
        global_params = rendered["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        self.assertEqual(
            global_params["plugins"],
            [
                "static_layer",
                "static_denoise_layer",
                "obstacle_layer",
                "inflation_layer",
            ],
        )
        self.assertEqual(
            global_params["static_denoise_layer"],
            {
                "plugin": "nav2_costmap_2d::DenoiseLayer",
                "enabled": True,
                "minimal_group_size": 2,
                "group_connectivity_type": 8,
            },
        )
        local_plugins = rendered["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]["plugins"]
        self.assertNotIn("static_denoise_layer", local_plugins)

    def test_private_yaml_keeps_soma_footprint_substitution_a_string(
        self,
    ) -> None:
        rendered = module.render_stage1_params(self.document)
        with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as temp:
            output = Path(temp) / "nav2_params.yaml"
            module.write_private_yaml(output, rendered)
            text = output.read_text(encoding="utf-8")
        self.assertIn(
            'footprint: "__ROBONIX_FOOTPRINT__"', text
        )
        materialized = text.replace(
            module.FOOTPRINT_TOKEN,
            "[ [0.30, 0.20], [0.30, -0.20], [-0.30, -0.20] ]",
        )
        document = yaml.safe_load(materialized)
        for costmap in ("local_costmap", "global_costmap"):
            footprint = document[costmap][costmap]["ros__parameters"][
                "footprint"
            ]
            self.assertIsInstance(footprint, str)
            self.assertTrue(footprint.startswith("[ ["))

    def test_tighter_source_limits_are_never_expanded(self) -> None:
        tighter = deepcopy(self.document)
        controller = tighter["controller_server"]["ros__parameters"][
            "FollowPath"
        ]
        controller.update(
            {
                "max_vel_x": 0.04,
                "max_vel_theta": 0.12,
                "max_speed_xy": 0.045,
                "min_speed_theta": 0.14,
                "acc_lim_x": 0.03,
                "acc_lim_theta": 0.10,
                "decel_lim_x": -0.02,
                "decel_lim_theta": -0.08,
            }
        )
        smoother = tighter["velocity_smoother"]["ros__parameters"]
        smoother["max_velocity"] = [0.045, 0.02, 0.13]
        smoother["min_velocity"] = [-0.02, -0.01, -0.11]
        smoother["max_accel"] = [0.04, 0.20, 0.11]
        smoother["max_decel"] = [-0.04, -0.20, -0.07]

        rendered = module.render_stage1_params(tighter)
        controller = rendered["controller_server"]["ros__parameters"][
            "FollowPath"
        ]
        smoother = rendered["velocity_smoother"]["ros__parameters"]
        self.assertEqual(controller["max_vel_x"], 0.04)
        self.assertEqual(controller["max_speed_xy"], 0.04)
        self.assertEqual(controller["max_vel_theta"], 0.12)
        self.assertEqual(controller["min_speed_theta"], 0.12)
        self.assertEqual(controller["acc_lim_x"], 0.03)
        self.assertEqual(controller["acc_lim_theta"], 0.10)
        self.assertEqual(controller["decel_lim_x"], -0.02)
        self.assertEqual(controller["decel_lim_theta"], -0.08)
        self.assertEqual(smoother["max_velocity"], [0.04, 0.0, 0.12])
        self.assertEqual(smoother["min_velocity"], [0.0, 0.0, -0.11])
        self.assertEqual(smoother["max_accel"], [0.03, 0.0, 0.10])
        self.assertEqual(smoother["max_decel"], [-0.02, 0.0, -0.07])

    def test_renderer_rejects_missing_malformed_or_unsafe_contract(
        self,
    ) -> None:
        missing = deepcopy(self.document)
        del missing["controller_server"]["ros__parameters"]["FollowPath"][
            "max_vel_x"
        ]
        with self.assertRaises(module.Stage1ParamsError):
            module.render_stage1_params(missing)

        malformed = deepcopy(self.document)
        malformed["velocity_smoother"]["ros__parameters"][
            "max_velocity"
        ] = [0.30, 0.0]
        with self.assertRaises(module.Stage1ParamsError):
            module.render_stage1_params(malformed)

        non_finite = deepcopy(self.document)
        non_finite["controller_server"]["ros__parameters"]["FollowPath"][
            "acc_lim_x"
        ] = float("nan")
        with self.assertRaises(module.Stage1ParamsError):
            module.render_stage1_params(non_finite)

        wrong_sign = deepcopy(self.document)
        wrong_sign["velocity_smoother"]["ros__parameters"][
            "max_decel"
        ] = [0.30, 0.0, -0.80]
        with self.assertRaises(module.Stage1ParamsError):
            module.render_stage1_params(wrong_sign)

        tighter = deepcopy(self.document)
        tighter["controller_server"]["ros__parameters"][
            "go2_goal_checker"
        ]["xy_goal_tolerance"] = 0.34
        with self.assertRaises(module.Stage1ParamsError):
            module.render_stage1_params(tighter)


if __name__ == "__main__":
    unittest.main()
