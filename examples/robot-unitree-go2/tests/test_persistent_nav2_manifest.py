from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC))


def _load_renderer():
    path = TIME_SYNC / "render_workstation_persistent_nav2_manifest.py"
    spec = importlib.util.spec_from_file_location("persistent_nav2_renderer", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load persistent Nav2 renderer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PersistentNav2ManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.renderer = _load_renderer()
        cls.base = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )

    def render(self):
        return self.renderer.render(
            self.base,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )

    @staticmethod
    def config(manifest, section, name):
        return next(
            item["config"] for item in manifest[section] if item["name"] == name
        )

    def test_full_voice_dual_camera_and_semantic_stack_is_retained(self) -> None:
        manifest = self.render()
        self.assertTrue(
            {"atlas", "executor", "pilot", "liaison", "soma", "scene"}
            <= set(manifest["system"])
        )
        self.assertIn("speech", {item["name"] for item in manifest["service"]})
        self.assertIn(
            "semantic_navigation", {item["name"] for item in manifest["skill"]}
        )
        self.assertIn(
            "go2_d435i", {item["name"] for item in manifest["primitive"]}
        )
        dashboard = self.config(manifest, "service", "go2_dashboard")
        self.assertIs(dashboard["browser_voice_enabled"], True)
        self.assertEqual(
            dashboard["d435i_color_topic"], "/go2/d435i/color/image_raw"
        )
        self.assertEqual(
            dashboard["d435i_depth_topic"],
            "/go2/d435i/aligned_depth_to_color/image_raw",
        )
        self.assertEqual(
            dashboard["initial_pose_maps_dir"],
            "${ROBONIX_DEPLOY_DIR}/rbnx-build/data/maps",
        )
        self.assertIs(dashboard["initial_pose_auto_restore"], False)

    def test_official_provider_owns_final_guard_and_chassis_keeps_watchdogs(self) -> None:
        manifest = self.render()
        chassis = self.config(manifest, "primitive", "go2_chassis")
        nav = self.config(manifest, "service", "nav2")
        self.assertEqual(
            chassis["twist_in_topic"], self.renderer.CHASSIS_COMMAND_TOPIC
        )
        self.assertEqual(
            nav["velocity_output_topic"], self.renderer.CHASSIS_COMMAND_TOPIC
        )
        self.assertIs(nav["external_velocity_guard"], False)
        self.assertIs(nav["use_composition"], True)
        self.assertIs(chassis["allow_motion"], True)
        self.assertIs(chassis["preserve_classic_walk"], True)
        self.assertEqual(chassis["allowed_modes"], [255])
        self.assertNotIn("allowed_state_markers", chassis)
        self.assertIs(chassis["allow_passive_state_marker_transitions"], False)
        self.assertIs(chassis["allow_motion_state_marker_transitions"], True)
        self.assertEqual(chassis["command_timeout_s"], 0.20)
        self.assertEqual(chassis["state_timeout_s"], 1.0)
        self.assertEqual(chassis["commissioning_max_duration_s"], 0.0)
        self.assertEqual(chassis["commissioning_max_distance_m"], 0.0)

    def test_persistent_localization_gets_isolated_spatial_icp_params(self) -> None:
        base_mapping = self.config(self.base, "service", "mapping")
        base_params = base_mapping["rtabmap_params"]
        original_base_params = copy.deepcopy(base_params)

        manifest = self.render()
        mapping = self.config(manifest, "service", "mapping")
        params = mapping["rtabmap_params"]

        self.assertIsNot(params, base_params)
        self.assertEqual(base_params, original_base_params)
        self.assertIs(base_mapping["dense_scan_refine_neighbors"], True)
        self.assertIs(mapping["dense_scan_refine_neighbors"], False)
        for key, value in self.renderer.LOCALIZATION_RTABMAP_OVERRIDES.items():
            self.assertEqual(params[key], value)
        for key, value in self.renderer.TIGHT_LOCALIZATION_ICP_PARAMS.items():
            self.assertEqual(params[key], value)

        self.assertIs(base_params["RGBD/NeighborLinkRefining"], True)
        self.assertIs(base_params["RGBD/ProximityBySpace"], False)
        self.assertEqual(base_params["RGBD/ProximityPathMaxNeighbors"], 0)

    def test_renderer_rejects_voice_guard_or_pose_binding_drift(self) -> None:
        for mutate in (
            lambda manifest: self.config(
                manifest, "service", "go2_dashboard"
            ).update({"browser_voice_enabled": False}),
            lambda manifest: self.config(manifest, "service", "nav2").update(
                {"external_velocity_guard": True}
            ),
            lambda manifest: self.config(
                manifest, "service", "go2_dashboard"
            ).update({"initial_pose_auto_restore": True}),
            lambda manifest: self.config(
                manifest, "primitive", "go2_chassis"
            ).update({"allow_motion_state_marker_transitions": False}),
        ):
            manifest = self.render()
            mutate(manifest)
            with self.assertRaises(self.renderer.ManifestError):
                self.renderer.validate_rendered(manifest)

    def test_runtime_authorization_values_are_not_committed_to_manifest(self) -> None:
        environment = self.render()["env"]
        for key in self.renderer.FORBIDDEN_COMMITTED_RUNTIME_ENV:
            self.assertNotIn(key, environment)


if __name__ == "__main__":
    unittest.main()
