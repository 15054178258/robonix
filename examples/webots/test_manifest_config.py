import unittest
from pathlib import Path
from xml.etree import ElementTree

import yaml


ROOT = Path(__file__).resolve().parent

SCENE_EVAL_CLIP_RERANK_LABELS = {
    "window",
    "picture frame",
    "cereal box",
    "book",
    "box",
    "plate",
    "monitor",
    "television",
}


def entries(document, section):
    return {entry["name"]: entry for entry in document.get(section, [])}


class WebotsDeployConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = yaml.safe_load((ROOT / "robonix_manifest.yaml").read_text())

    def test_explore_progress_watchdog_is_explicit_and_metric(self):
        expected = {
            "no_progress_timeout_s": 12.0,
            "min_translation_progress_m": 0.08,
            "min_rotation_progress_rad": 0.14,
        }
        for filename in (
            "robonix_manifest.yaml",
            "robonix_manifest.mapping-nav-eval.yaml",
        ):
            document = yaml.safe_load((ROOT / filename).read_text())
            explore = entries(document, "skill")["explore"]
            self.assertEqual(
                explore["config"]["progress_watchdog"],
                expected,
                filename,
            )
            self.assertEqual(
                explore["config"]["frontier_recovery"],
                {
                    "retry_failed_after_s": (
                        240.0
                        if filename == "robonix_manifest.mapping-nav-eval.yaml"
                        else 30.0
                    ),
                    "endpoint_clearance_m": 0.10,
                },
                filename,
            )

    def test_scene_same_class_identity_gates_are_explicit(self):
        expected = {
            "coobserved_duplicate_min_shared_frames": 3,
            "coobserved_duplicate_min_median_iou": 0.85,
            "coobserved_duplicate_max_extent_ratio": 2.0,
            "coobserved_duplicate_min_visual_similarity": 0.90,
            "same_class_centroid_max_m": 0.15,
            "same_class_min_voxel_coverage": 0.50,
            "same_class_max_extent_ratio": 1.75,
            "same_class_disjoint_min_unique_frames": 2,
            "same_class_disjoint_max_frame_gap": 1,
            "same_class_disjoint_max_center_major_extent_ratio": 0.20,
            "same_class_disjoint_min_visual_similarity": 0.85,
            "same_class_merge_interval_ticks": 1,
        }
        for filename in (
            "robonix_manifest.yaml",
            "robonix_manifest.scene-eval.yaml",
            "robonix_manifest.mapping-nav-eval.yaml",
        ):
            document = yaml.safe_load((ROOT / filename).read_text())
            association = document["system"]["scene"]["config"]["perception"][
                "association"
            ]
            self.assertEqual(
                {key: association.get(key) for key in expected},
                expected,
                filename,
            )
        standalone = yaml.safe_load((ROOT / "scene-eval-config.yaml").read_text())
        association = standalone["perception"]["association"]
        self.assertEqual(
            {key: association.get(key) for key in expected},
            expected,
            "scene-eval-config.yaml",
        )

    def test_scene_eval_clip_rerank_is_scoped_and_consistent(self):
        configs = {}
        for filename in (
            "robonix_manifest.scene-eval.yaml",
            "robonix_manifest.mapping-nav-eval.yaml",
        ):
            document = yaml.safe_load((ROOT / filename).read_text())
            configs[filename] = document["system"]["scene"]["config"]["perception"]
        configs["scene-eval-config.yaml"] = yaml.safe_load(
            (ROOT / "scene-eval-config.yaml").read_text()
        )["perception"]

        for filename, perception in configs.items():
            rerank = perception["label"]["clip_rerank"]
            groups = rerank["groups"]
            self.assertEqual(
                groups,
                [
                    {
                        "labels": ["window", "picture frame"],
                        "min_margin": 0.002,
                    },
                    {
                        "labels": ["cereal box", "book", "box", "plate"],
                        "min_margin": 0.05,
                    },
                    {
                        "labels": ["monitor", "television"],
                        "min_margin": 0.0,
                    },
                ],
                filename,
            )
            self.assertEqual(
                rerank["routes"],
                {
                    "chair": {
                        "labels": ["chair", "monitor"],
                        "min_margin": 0.02,
                    },
                    "shelf": {
                        "labels": ["shelf", "cabinet"],
                        "min_margin": 0.06,
                    },
                    "table": {
                        "labels": ["table", "chair"],
                        "min_margin": 0.02,
                    },
                    "cup": {
                        "labels": ["cup", "can"],
                        "min_margin": 0.02,
                    },
                    "cabinet": {
                        "labels": ["cabinet", "window"],
                        "min_margin": 0.03,
                    },
                    "refrigerator": {
                        "labels": ["refrigerator", "cabinet"],
                        "min_margin": 0.03,
                    },
                },
                filename,
            )
            flattened = [
                label for group in groups for label in group["labels"]
            ]
            self.assertEqual(len(flattened), len(set(flattened)), filename)
            self.assertEqual(set(flattened), SCENE_EVAL_CLIP_RERANK_LABELS, filename)
            self.assertEqual(rerank["min_score"], 0.20, filename)
            self.assertEqual(rerank["min_margin"], 0.05, filename)
            self.assertEqual(
                rerank["persistent_geometry"],
                {
                    "score_bonus": 0.06,
                    "labels": {
                        "monitor": {
                            "source_labels": ["monitor", "television"],
                            "max_horizontal_extent_m": 0.68,
                        },
                    },
                },
                filename,
            )
            # Visual reranking may compare many detector labels, but identity
            # association admits only the two explicitly reviewed flicker
            # domains. It must not inherit directed routes such as
            # chair -> monitor, which would merge nearby physical objects.
            self.assertEqual(
                perception["association"]["confusable_class_groups"],
                [
                    ["window", "picture frame"],
                    ["monitor", "television"],
                ],
                filename,
            )
            self.assertFalse(
                perception["association"]["allow_cross_class_merge"],
                filename,
            )
            self.assertEqual(
                perception["association"][
                    "identity_rebind_max_distance_m"
                ],
                0.45,
                filename,
            )

    def test_scene_visibility_dropout_thresholds_are_consistent(self):
        configs = {}
        for filename in (
            "robonix_manifest.yaml",
            "robonix_manifest.scene-eval.yaml",
            "robonix_manifest.mapping-nav-eval.yaml",
        ):
            document = yaml.safe_load((ROOT / filename).read_text())
            configs[filename] = document["system"]["scene"]["config"]["perception"]
        configs["scene-eval-config.yaml"] = yaml.safe_load(
            (ROOT / "scene-eval-config.yaml").read_text()
        )["perception"]

        for filename, perception in configs.items():
            self.assertEqual(
                perception["confirmation_min_unique_frames"],
                2,
                filename,
            )
            self.assertEqual(
                perception[
                    "confirmation_singleton_min_mean_confidence"
                ],
                0.0,
                filename,
            )
            self.assertEqual(
                perception["visibility_depth_margin_m"],
                0.10,
                filename,
            )
            self.assertEqual(
                perception["visibility_min_clear_samples"],
                3,
                filename,
            )
            self.assertEqual(
                perception["visibility_min_clear_fraction"],
                0.60,
                filename,
            )

    def test_scene_eval_surface_snap_is_conservative_and_eval_only(self):
        expected = {
            "labels": ["window", "picture frame"],
            "max_distance_m": 0.60,
            "tangent_padding_m": 0.25,
            "min_shift_m": 0.05,
            "min_support_cells": 30,
            "min_dominant_share": 0.55,
            "min_tangent_coverage": 0.50,
            "occupancy_threshold": 50,
        }
        for filename in (
            "robonix_manifest.scene-eval.yaml",
            "robonix_manifest.mapping-nav-eval.yaml",
        ):
            document = yaml.safe_load((ROOT / filename).read_text())
            geometry = document["system"]["scene"]["config"]["perception"][
                "geometry"
            ]
            self.assertEqual(geometry["surface_snap"], expected, filename)
        standalone = yaml.safe_load((ROOT / "scene-eval-config.yaml").read_text())
        self.assertEqual(
            standalone["perception"]["geometry"]["surface_snap"],
            expected,
            "scene-eval-config.yaml",
        )
        default_geometry = self.document["system"]["scene"]["config"][
            "perception"
        ]["geometry"]
        self.assertNotIn("surface_snap", default_geometry)

    def test_scene_keeps_rep105_map_geometry_fixed(self):
        for filename in (
            "robonix_manifest.yaml",
            "robonix_manifest.scene-eval.yaml",
            "robonix_manifest.mapping-nav-eval.yaml",
        ):
            document = yaml.safe_load((ROOT / filename).read_text())
            geometry = document["system"]["scene"]["config"]["perception"][
                "geometry"
            ]
            self.assertIs(geometry["rebase_map_corrections"], False, filename)
        standalone = yaml.safe_load((ROOT / "scene-eval-config.yaml").read_text())
        self.assertIs(
            standalone["perception"]["geometry"]["rebase_map_corrections"],
            False,
        )

    def test_scene_benchmark_checks_selected_manifest_not_fixed_ab_values(self):
        script = (
            ROOT.parent.parent / "testing" / "run_webots_scene_benchmark.sh"
        ).read_text()
        self.assertIn(
            'python3 - "$world_dir/startup-state.json" "$MANIFEST"',
            script,
        )
        self.assertIn(
            'export SCENE_DATA_DIR="$world_dir/scene-data"',
            script,
        )
        self.assertIn("expected_group_count = len(groups)", script)
        self.assertIn("expected_route_count = len(routes)", script)
        self.assertIn("expected_label_count = len(labels)", script)
        self.assertIn('rerank.get("min_score", 0.0)', script)
        self.assertIn('rerank.get("min_margin", 0.0)', script)
        self.assertIn("expected_surface_snap", script)
        self.assertIn(
            'surface_snap.get("max_distance_m", 0.60)',
            script,
        )
        self.assertIn(
            'actual_surface_snap = quality.get("surface_snap") or {}',
            script,
        )
        self.assertIn("expected_rebase_map_corrections", script)
        self.assertIn("actual_rebase_map_corrections", script)
        self.assertNotIn("expected 2", script)
        self.assertNotIn("expected 0.0", script)

    def test_mapping_uses_deploy_owned_file(self):
        mapping = entries(self.document, "service")["mapping"]["config"]
        self.assertEqual(mapping["params_file"], "config/rtabmap_params.yaml")
        self.assertNotIn("rtabmap_profile", mapping)
        self.assertNotIn("rtabmap_params", mapping)
        params = yaml.safe_load((ROOT / mapping["params_file"]).read_text())
        self.assertEqual(params["Reg/Strategy"], 1)
        self.assertEqual(params["Rtabmap/DetectionRate"], 5.0)
        self.assertEqual(params["RGBD/LinearUpdate"], 0.05)
        self.assertEqual(params["RGBD/AngularUpdate"], 0.05)
        self.assertTrue(params["RGBD/NeighborLinkRefining"])
        self.assertTrue(params["RGBD/ProximityBySpace"])
        # Keep the RTAB-Map 0.23 loop-closure defaults used by the historical
        # stable Webots configuration. In particular, LoopThr=1 disables the
        # appearance loop path and previously allowed map drift to accumulate.
        self.assertNotIn("RGBD/ProximityByTime", params)
        self.assertNotIn("RGBD/ProximityPathMaxNeighbors", params)
        self.assertNotIn("Rtabmap/LoopThr", params)

    def test_sim_fuses_wheel_speed_with_imu_heading_for_public_odom(self):
        resource = ROOT / "sim" / "ros_ws" / "src" / "eaios_webots" / "resource"
        urdf = (resource / "tiago_webots.urdf").read_text()
        control = yaml.safe_load((resource / "ros2_control.yml").read_text())
        ekf = yaml.safe_load((resource / "ekf.yaml").read_text())[
            "ekf_filter_node"
        ]["ros__parameters"]
        launch = (
            ROOT
            / "sim"
            / "ros_ws"
            / "src"
            / "eaios_webots"
            / "launch"
            / "robot_launch.py"
        ).read_text()

        self.assertIn("webots_ros2_driver::Ros2IMU", urdf)
        self.assertIn("<topicName>/imu</topicName>", urdf)
        self.assertFalse(
            control["diffdrive_controller"]["ros__parameters"]["enable_odom_tf"]
        )
        self.assertEqual(ekf["odom0"], "/wheel_odom")
        self.assertEqual(ekf["imu0"], "/imu")
        # Wheel pose/yaw are intentionally excluded; only body-frame vx is
        # integrated using the relative IMU yaw.
        self.assertFalse(any(ekf["odom0_config"][:6]))
        self.assertTrue(ekf["odom0_config"][6])
        self.assertTrue(ekf["imu0_config"][5])
        self.assertTrue(ekf["imu0_config"][11])
        self.assertTrue(ekf["imu0_relative"])
        self.assertIn("('/diffdrive_controller/odom', '/wheel_odom')", launch)
        self.assertIn("('/odometry/filtered', '/odom')", launch)

    def test_navigation_uses_deploy_owned_file(self):
        navigation = entries(self.document, "service")["nav2"]["config"]
        self.assertEqual(navigation["params_file"], "config/nav2_params.yaml")
        self.assertNotIn("params_profile", navigation)
        self.assertTrue((ROOT / navigation["params_file"]).is_file())
        params_text = (ROOT / navigation["params_file"]).read_text()
        self.assertEqual(params_text.count('footprint: "__ROBONIX_FOOTPRINT__"'), 2)
        self.assertNotIn("robot_radius:", params_text)
        params = yaml.safe_load(params_text)
        for costmap_name in ("local_costmap", "global_costmap"):
            costmap = params[costmap_name][costmap_name]["ros__parameters"]
            self.assertIn("scan_obstacle_layer", costmap["plugins"])
            self.assertIn("depth_obstacle_layer", costmap["plugins"])
            self.assertEqual(
                costmap["scan_obstacle_layer"]["observation_sources"],
                "scan",
            )
            self.assertEqual(
                costmap["depth_obstacle_layer"]["observation_sources"],
                "depth_points",
            )

    def test_scene_explore_eval_returns_unreachable_frontiers_promptly(self):
        document = yaml.safe_load(
            (ROOT / "robonix_manifest.mapping-nav-eval.yaml").read_text()
        )
        navigation = entries(document, "service")["nav2"]["config"]
        self.assertEqual(
            navigation["params_file"],
            "config/nav2_params.explore.yaml",
        )
        self.assertEqual(
            navigation["bt_xml_file"],
            "config/navigate_to_pose_explore.xml",
        )
        params_text = (ROOT / navigation["params_file"]).read_text()
        self.assertIn(
            'default_nav_to_pose_bt_xml: "__ROBONIX_BT_XML__"',
            params_text,
        )
        tree = ElementTree.parse(ROOT / navigation["bt_xml_file"])
        retries = tree.findall(".//RecoveryNode")
        self.assertTrue(retries)
        self.assertEqual(retries[0].attrib["number_of_retries"], "1")
        tags = {node.tag for node in tree.iter()}
        self.assertNotIn("Spin", tags)
        self.assertNotIn("Wait", tags)
        self.assertNotIn("BackUp", tags)
        clear_services = {
            node.attrib.get("service_name")
            for node in tree.findall(".//ClearEntireCostmap")
        }
        self.assertEqual(
            clear_services,
            {
                "local_costmap/clear_entirely_local_costmap",
                "global_costmap/clear_entirely_global_costmap",
            },
        )



if __name__ == "__main__":
    unittest.main()
