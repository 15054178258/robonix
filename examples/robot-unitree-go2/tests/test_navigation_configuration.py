import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_yaml(relative: str):
    return yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))


def load_json(relative: str):
    return json.loads((ROOT / relative).read_text(encoding="utf-8"))


class NavigationConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.nav = load_yaml("config/nav2_params_go2.yaml")
        cls.mapping = load_yaml("config/rtabmap_params.yaml")
        cls.dense_mapping = load_json(
            "config/mapping_dense2d_refined_nomotion.json"
        )
        cls.manifest = load_yaml("robonix_manifest.yaml")
        cls.tree = ET.parse(ROOT / "config" / "navigate.xml")
        cls.through_poses_tree = ET.parse(
            ROOT / "config" / "navigate_through_poses.xml"
        )

    def service_config(self, name: str) -> dict:
        return next(row["config"] for row in self.manifest["service"] if row["name"] == name)

    def primitive_config(self, name: str) -> dict:
        return next(row.get("config", {}) for row in self.manifest["primitive"] if row["name"] == name)

    def test_humble_uses_the_deploy_owned_safe_tree(self) -> None:
        params = self.nav["bt_navigator"]["ros__parameters"]
        token = "__ROBONIX_BT_XML__"
        through_token = "__ROBONIX_BT_THROUGH_POSES_XML__"
        self.assertEqual(params["default_nav_to_pose_bt_xml"], token)
        self.assertEqual(params["default_bt_xml_filename"], token)
        self.assertEqual(
            params["default_nav_through_poses_bt_xml"], through_token
        )
        nav_cfg = self.service_config("nav2")
        self.assertEqual(nav_cfg["bt_xml_file"], "config/navigate.xml")
        self.assertEqual(
            nav_cfg["bt_through_poses_xml_file"],
            "config/navigate_through_poses.xml",
        )
        self.assertEqual(params["global_frame"], "map")
        self.assertEqual(params["robot_base_frame"], "base_link")
        self.assertEqual(params["odom_topic"], "__ROBONIX_ODOM_TOPIC__")

    def test_tree_has_only_non_motion_recovery_actions(self) -> None:
        for tree, planner_node in (
            (self.tree, "ComputePathToPose"),
            (self.through_poses_tree, "ComputePathThroughPoses"),
        ):
            tags = {node.tag for node in tree.iter()}
            for forbidden in ("Spin", "BackUp", "DriveOnHeading", "AssistedTeleop"):
                self.assertNotIn(forbidden, tags)
            for required in (
                planner_node,
                "FollowPath",
                "ClearEntireCostmap",
                "GoalUpdated",
                "Wait",
            ):
                self.assertIn(required, tags)
            retries = [
                int(node.attrib["number_of_retries"])
                for node in tree.iter("RecoveryNode")
            ]
            self.assertTrue(retries)
            self.assertTrue(all(value <= 1 for value in retries))

    def test_behavior_server_covers_every_tree_recovery_action(self) -> None:
        behavior = self.nav["behavior_server"]["ros__parameters"]
        configured = set(behavior["behavior_plugins"])
        action_plugins = {
            "Spin": "spin",
            "BackUp": "backup",
            "DriveOnHeading": "drive_on_heading",
            "AssistedTeleop": "assisted_teleop",
            "Wait": "wait",
        }
        referenced = {
            plugin
            for tree in (self.tree, self.through_poses_tree)
            for tag, plugin in action_plugins.items()
            if any(node.tag == tag for node in tree.iter())
        }
        self.assertEqual(referenced, {"wait"})
        self.assertTrue(referenced <= configured)

    def test_known_space_planning_and_live_obstacle_layers(self) -> None:
        planner = self.nav["planner_server"]["ros__parameters"]["GridBased"]
        self.assertFalse(planner["allow_unknown"])

        for costmap_name in ("local_costmap", "global_costmap"):
            params = self.nav[costmap_name][costmap_name]["ros__parameters"]
            self.assertIn("obstacle_layer", params["plugins"])
            self.assertIn("inflation_layer", params["plugins"])
            scan = params["obstacle_layer"]["scan"]
            self.assertEqual(scan["topic"], "__ROBONIX_SCAN_TOPIC__")
            self.assertEqual(scan["data_type"], "LaserScan")
            self.assertTrue(scan["marking"])
            self.assertTrue(scan["clearing"])
            self.assertTrue(scan["inf_is_valid"])
            self.assertEqual(scan["observation_persistence"], 0.0)
            self.assertGreater(scan["expected_update_rate"], 0.0)
            self.assertLessEqual(scan["expected_update_rate"], 1.0)
            self.assertGreater(scan["raytrace_max_range"], scan["obstacle_max_range"])
            self.assertLessEqual(scan["min_obstacle_height"], 0.0)
            self.assertNotIn("qos", scan)

        global_params = self.nav["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertEqual(
            global_params["plugins"],
            [
                "static_layer",
                "static_denoise_layer",
                "obstacle_layer",
                "inflation_layer",
            ],
        )
        self.assertEqual(global_params["static_layer"]["map_topic"], "__ROBONIX_MAP_TOPIC__")
        denoise = global_params["static_denoise_layer"]
        self.assertEqual(denoise["plugin"], "nav2_costmap_2d::DenoiseLayer")
        self.assertTrue(denoise["enabled"])
        self.assertEqual(denoise["minimal_group_size"], 2)
        self.assertEqual(denoise["group_connectivity_type"], 8)

    def test_humble_local_costmap_dimensions_are_integer_parameters(self) -> None:
        """Costmap2DROS rejects decimal width/height scalars on Humble."""
        params = self.nav["local_costmap"]["local_costmap"]["ros__parameters"]
        for name in ("width", "height"):
            self.assertIs(type(params[name]), int, name)
            self.assertGreater(params[name], 0)

    def test_velocity_smoother_is_bounded_and_times_out(self) -> None:
        controller_params = self.nav["controller_server"]["ros__parameters"]
        controller = controller_params["FollowPath"]
        smoother = self.nav["velocity_smoother"]["ros__parameters"]
        chassis = self.primitive_config("go2_chassis")

        progress = controller_params["progress_checker"]
        self.assertEqual(
            progress["plugin"], "nav2_controller::PoseProgressChecker"
        )
        self.assertGreater(progress["required_movement_angle"], 0.0)
        self.assertEqual(
            controller_params["go2_goal_checker"]["yaw_goal_tolerance"],
            0.35,
        )

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
        self.assertGreater(
            controller["angular_dist_threshold"],
            controller["angular_disengage_threshold"],
        )
        self.assertEqual(controller["rotate_to_heading_angular_vel"], 0.30)
        self.assertEqual(controller["max_angular_accel"], 0.80)
        self.assertNotIn("rotate_to_heading_linear_vel", controller)

        self.assertEqual(smoother["max_velocity"], [0.30, 0.0, 0.40])
        self.assertEqual(smoother["min_velocity"], [0.0, 0.0, -0.40])
        self.assertLessEqual(smoother["max_velocity"][0], controller["max_vel_x"])
        self.assertLessEqual(smoother["max_velocity"][2], controller["max_vel_theta"])
        self.assertLessEqual(smoother["max_velocity"][0], 0.30)
        self.assertLessEqual(smoother["max_velocity"][2], chassis["max_angular_z_rps"])
        self.assertLessEqual(smoother["max_accel"][0], chassis["max_linear_accel_mps2"])
        self.assertLessEqual(smoother["max_accel"][2], chassis["max_angular_accel_rps2"])
        self.assertLessEqual(abs(smoother["max_decel"][0]), chassis["max_linear_accel_mps2"])
        self.assertLessEqual(abs(smoother["max_decel"][2]), chassis["max_angular_accel_rps2"])
        self.assertLessEqual(smoother["velocity_timeout"], chassis["command_timeout_s"])
        self.assertEqual(smoother["odom_topic"], "__ROBONIX_ODOM_TOPIC__")

        self.assertLessEqual(abs(controller["decel_lim_x"]), chassis["max_linear_accel_mps2"])
        self.assertLessEqual(abs(controller["decel_lim_theta"]), chassis["max_angular_accel_rps2"])
        self.assertEqual(controller["xy_goal_tolerance"], 0.35)
        self.assertEqual(controller["min_speed_theta"], 0.20)
        self.assertNotIn("RotateToGoal.xy_goal_tolerance", controller)
        self.assertNotIn("RotateToGoal.yaw_goal_tolerance", controller)

        managed = set(self.nav["lifecycle_manager"]["ros__parameters"]["node_names"])
        self.assertTrue({"smoother_server", "velocity_smoother", "bt_navigator"} <= managed)

    def test_provider_roles_are_pinned_to_this_robot(self) -> None:
        chassis = self.primitive_config("go2_chassis")
        self.assertEqual(chassis["odom_source"], "sport_state")
        self.assertEqual(
            chassis["external_odom_topic"],
            "/robonix/time_corrected/raw/utlidar/robot_odom",
        )
        self.assertNotEqual(chassis["external_odom_topic"], chassis["odom_topic"])
        self.assertTrue(chassis["publish_odom_tf"])

        nav = self.service_config("nav2")
        self.assertEqual(
            nav["provider_ids"],
            {
                "map": "mapping",
                "odom": "go2_chassis",
                "scan": "go2_sensors",
                "scan_cloud": "go2_sensors",
            },
        )
        self.assertTrue(nav["scan_projection"]["enabled"])
        self.assertEqual(nav["scan_projection"]["target_frame"], "base_link")
        self.assertLess(nav["scan_projection"]["min_height_m"], 0.0)
        self.assertGreater(nav["scan_projection"]["max_height_m"], 0.0)
        self.assertLessEqual(nav["scan_projection"]["self_filter_margin_m"], 0.02)
        self.assertFalse(nav["scan_projection"]["deskewing"])

        mapping = self.service_config("mapping")
        self.assertEqual(
            mapping["sensor_providers"],
            {"lidar3d": "go2_sensors", "imu": "go2_sensors", "odom": "go2_chassis"},
        )
        self.assertEqual(mapping["rtabmap_inputs"], ["lidar", "imu", "odom"])
        self.assertTrue(mapping["deskew_lidar"])

    def test_terminal_guard_matches_goal_window(self) -> None:
        goal = self.nav["controller_server"]["ros__parameters"]["go2_goal_checker"]
        guard = self.service_config("nav2")
        self.assertLessEqual(guard["guard_terminal_xy_m"], goal["xy_goal_tolerance"])
        self.assertGreater(guard["guard_terminal_timeout_s"], 0.0)
        self.assertGreater(guard["guard_no_progress_s"], 0.0)
        self.assertGreater(guard["guard_global_spin_timeout_s"], 0.0)
        self.assertGreater(guard["guard_global_spin_limit_rad"], 0.0)

    def test_rtabmap_profile_is_planar_lidar_only_and_scalar(self) -> None:
        self.assertTrue(all(not isinstance(value, (dict, list)) for value in self.mapping.values()))
        self.assertEqual(self.mapping["Grid/Sensor"], 0)
        self.assertFalse(self.mapping["Grid/FromDepth"])
        self.assertFalse(self.mapping["Grid/3D"])
        self.assertLessEqual(self.mapping["Grid/RangeMax"], 6.0)
        # A UniLiDAR packet is not a complete planar scan.  Treating it as one
        # produces radial free-space spokes in the occupancy grid.
        self.assertFalse(self.mapping["Grid/RayTracing"])
        # Unitree UniLiDAR packets are non-repetitive and sparse. Per-packet normal
        # estimation can discard every obstacle return, producing a free-only
        # OccupancyGrid.  The real-robot profile must use the deterministic
        # passthrough height split instead.
        self.assertFalse(self.mapping["Grid/NormalsSegmentation"])
        scan_projection = self.service_config("nav2")["scan_projection"]
        self.assertEqual(
            self.mapping["Grid/MaxGroundHeight"],
            scan_projection["min_height_m"],
        )
        self.assertLess(self.mapping["Grid/MaxGroundHeight"], 0.0)
        self.assertGreater(
            self.mapping["Grid/MaxObstacleHeight"],
            self.mapping["Grid/MaxGroundHeight"],
        )
        # Real Mid-360 replay showed that the conservative radius filter
        # removes isolated non-repetitive returns without erasing thin walls.
        self.assertAlmostEqual(self.mapping["Grid/NoiseFilteringRadius"], 0.20)
        self.assertEqual(self.mapping["Grid/NoiseFilteringMinNeighbors"], 3)
        self.assertTrue(self.mapping["Reg/Force3DoF"])
        # The verified chassis odometry remains authoritative until ICP and
        # proximity closure have a separately validated accumulated/deskewed
        # registration input.
        self.assertFalse(self.mapping["RGBD/NeighborLinkRefining"])
        self.assertFalse(self.mapping["RGBD/ProximityBySpace"])
        self.assertFalse(self.mapping["Mem/NotLinkedNodesKept"])
        self.assertGreaterEqual(self.mapping["Grid/FootprintLength"], 0.80)
        self.assertGreaterEqual(self.mapping["Grid/FootprintWidth"], 0.50)

    def test_dense_refined_nomotion_mapping_profile_is_explicit_and_strict(self) -> None:
        profile = self.dense_mapping
        self.assertEqual(profile["algo"], "rtabmap")
        self.assertEqual(profile["map_mode"], "mapping")
        self.assertTrue(profile["reset_map"])
        self.assertTrue(profile["deskew_lidar"])
        self.assertTrue(profile["dense_scan_2d"])
        self.assertTrue(profile["dense_scan_refine_neighbors"])
        self.assertFalse(profile["use_sim_time"])
        self.assertEqual(profile["base_frame"], "base_link")
        self.assertEqual(profile["odom_frame"], "odom")
        self.assertEqual(profile["rtabmap_inputs"], ["lidar", "imu", "odom"])
        self.assertEqual(
            profile["sensor_providers"],
            {
                "lidar3d": "go2_sensors",
                "imu": "go2_sensors",
                "odom": "go2_chassis",
            },
        )
        self.assertEqual(profile["params_file"], "config/rtabmap_params.yaml")
        self.assertTrue((ROOT / profile["params_file"]).is_file())
        self.assertEqual(profile["webui_host"], "127.0.0.1")
        self.assertEqual(profile["webui_port"], 8091)
        self.assertTrue(
            {
                "motion_enabled",
                "enable_motion",
                "cmd_vel_topic",
                "command_topic",
            }.isdisjoint(profile),
        )

        params = profile["rtabmap_params"]
        self.assertEqual(params["Reg/Strategy"], 1)
        self.assertTrue(params["Reg/Force3DoF"])
        self.assertTrue(params["Icp/PointToPlane"])
        self.assertEqual(params["Icp/PointToPlaneK"], 5)
        self.assertAlmostEqual(params["Icp/PointToPlaneMinComplexity"], 0.02)
        self.assertEqual(params["Icp/PointToPlaneLowComplexityStrategy"], 1)
        self.assertAlmostEqual(params["Icp/CorrespondenceRatio"], 0.20)
        self.assertAlmostEqual(params["Icp/MaxCorrespondenceDistance"], 0.15)
        self.assertAlmostEqual(params["Icp/MaxTranslation"], 0.10)
        self.assertAlmostEqual(params["Icp/MaxRotation"], 0.10)
        self.assertAlmostEqual(params["Icp/RangeMin"], 0.492)
        self.assertAlmostEqual(params["Icp/RangeMax"], 6.0)
        self.assertTrue(params["RGBD/NeighborLinkRefining"])
        self.assertFalse(params["RGBD/ProximityBySpace"])
        self.assertEqual(params["RGBD/ProximityPathMaxNeighbors"], 0)


if __name__ == "__main__":
    unittest.main()
