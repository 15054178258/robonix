from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC))


def load_renderer():
    path = TIME_SYNC / "render_workstation_nomotion_d435i_preview_manifest.py"
    spec = importlib.util.spec_from_file_location("d435i_preview_renderer", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load D435i preview renderer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class D435iPreviewManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.renderer = load_renderer()
        cls.base = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )

    def render(self):
        return self.renderer.render(
            self.base,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )

    def test_preview_adds_external_camera_and_pins_scene(self) -> None:
        manifest = self.render()
        d435i = next(
            entry
            for entry in manifest["primitive"]
            if entry["name"] == "go2_d435i"
        )
        self.assertEqual(d435i, self.renderer._d435i_primitive())
        self.assertEqual(
            d435i["config"]["min_rate_hz"],
            self.renderer.D435I_MIN_RATE_HZ,
        )
        self.assertEqual(d435i["config"]["min_rate_hz"], 3.0)
        self.assertEqual(
            manifest["system"]["scene"]["provider_ids"],
            {
                "rgb": "go2_d435i",
                "depth": "go2_d435i",
                "intrinsics": "go2_d435i",
            },
        )
        self.assertEqual(
            manifest["system"]["scene"]["camera_frame"],
            "d435i_color_optical_frame",
        )
        self.assertIs(
            manifest["system"]["scene"]["perception_enabled"],
            False,
        )

    def test_preview_keeps_mapping_lidar_only_and_motion_disabled(self) -> None:
        manifest = self.render()
        mapping = next(
            entry
            for entry in manifest["service"]
            if entry["name"] == "mapping"
        )["config"]
        chassis = next(
            entry
            for entry in manifest["primitive"]
            if entry["name"] == "go2_chassis"
        )["config"]
        self.assertEqual(mapping["rtabmap_inputs"], ["lidar", "imu", "odom"])
        self.assertNotIn("rgb", mapping["sensor_providers"])
        self.assertNotIn("depth", mapping["sensor_providers"])
        self.assertIs(chassis["allow_motion"], False)
        self.assertEqual(
            chassis["twist_in_topic"],
            "/robonix/nomotion/chassis_input_disabled",
        )

    def test_hardened_base_renderer_remains_d435i_free(self) -> None:
        manifest = self.renderer.render_base_nomotion(
            self.base,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )
        self.assertNotIn(
            "go2_d435i",
            {
                entry.get("name")
                for entry in manifest["primitive"]
                if isinstance(entry, dict)
            },
        )
        self.assertNotIn("provider_ids", manifest["system"]["scene"])
        self.assertNotIn("GO2_D435I_PROFILE", manifest["env"])

    def test_launcher_selects_only_the_fixed_opt_in_renderer(self) -> None:
        source = (
            ROOT / "scripts" / "start_workstation_full_nomotion_corrected.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'GO2_D435I_PREVIEW_ENABLED must be exactly true or false',
            source,
        )
        self.assertIn(
            'if [[ "$D435I_PREVIEW_ENABLED" == true ]]; then',
            source,
        )
        self.assertIn(
            'MANIFEST_RENDERER="$ROOT/deploy/time-sync/'
            'render_workstation_nomotion_d435i_preview_manifest.py"',
            source,
        )
        self.assertIn('"$PYTHON" "$MANIFEST_RENDERER" \\', source)
        self.assertNotIn("GO2_NOMOTION_MANIFEST_RENDERER", source)

    def test_validation_rejects_mapping_fusion_or_pin_drift(self) -> None:
        mapping_enabled = self.render()
        mapping = next(
            entry
            for entry in mapping_enabled["service"]
            if entry["name"] == "mapping"
        )["config"]
        mapping["rtabmap_inputs"] = ["lidar", "rgbd", "imu", "odom"]
        mapping["sensor_providers"]["rgb"] = "go2_d435i"
        mapping["sensor_providers"]["depth"] = "go2_d435i"
        with self.assertRaises(self.renderer.ManifestError):
            self.renderer.validate_rendered(mapping_enabled)

        pin_drift = self.render()
        pin_drift["system"]["scene"]["provider_ids"]["depth"] = "go2_sensors"
        with self.assertRaises(self.renderer.ManifestError):
            self.renderer.validate_rendered(pin_drift)

        perception_enabled = self.render()
        perception_enabled["system"]["scene"]["perception_enabled"] = True
        with self.assertRaises(self.renderer.ManifestError):
            self.renderer.validate_rendered(perception_enabled)

        perception_omitted = self.render()
        del perception_omitted["system"]["scene"]["perception_enabled"]
        with self.assertRaises(self.renderer.ManifestError):
            self.renderer.validate_rendered(perception_omitted)

        perception_string = self.render()
        perception_string["system"]["scene"]["perception_enabled"] = "false"
        with self.assertRaises(self.renderer.ManifestError):
            self.renderer.validate_rendered(perception_string)

        changed_topic = self.render()
        camera = next(
            entry
            for entry in changed_topic["primitive"]
            if entry["name"] == "go2_d435i"
        )
        camera["config"]["rgb_topic"] = "/unexpected/camera"
        with self.assertRaises(self.renderer.ManifestError):
            self.renderer.validate_rendered(changed_topic)


if __name__ == "__main__":
    unittest.main()
