from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC))
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))


def load_renderer():
    path = TIME_SYNC / "render_workstation_staged_nav2_manifest.py"
    spec = importlib.util.spec_from_file_location(
        "staged_nav2_manifest_renderer", path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load staged Nav2 manifest renderer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class StagedNav2ManifestRendererTests(unittest.TestCase):
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

    @staticmethod
    def named(manifest, section, name):
        return next(
            entry for entry in manifest[section] if entry["name"] == name
        )

    def test_exact_staged_route_is_derived_from_corrected_nomotion(self) -> None:
        result = self.render()
        self.assertEqual(
            tuple(result["system"]),
            self.renderer.EXPECTED_SYSTEMS,
        )
        self.assertEqual(
            tuple(entry["name"] for entry in result["primitive"]),
            self.renderer.EXPECTED_PRIMITIVES,
        )
        self.assertEqual(
            tuple(entry["name"] for entry in result["service"]),
            self.renderer.EXPECTED_SERVICES,
        )
        self.assertEqual(result["skill"], [])

        chassis = self.named(result, "primitive", "go2_chassis")["config"]
        self.assertEqual(chassis["state_topic"], self.renderer.STATE_TOPIC)
        self.assertEqual(chassis["state_fallback_topic"], "")
        self.assertEqual(
            chassis["twist_in_topic"],
            self.renderer.CHASSIS_COMMAND_TOPIC,
        )
        self.assertTrue(chassis["allow_motion"])
        self.assertEqual(chassis["motion_profile"], self.renderer.PROFILE)
        self.assertEqual(chassis["arm_service"], "/go2_chassis/arm")
        self.assertEqual(
            chassis["ipc_socket"],
            self.renderer.IPC_SOCKET_ENV,
        )
        self.assertEqual(chassis["allowed_modes"], [255])
        self.assertNotIn("allowed_state_markers", chassis)
        self.assertFalse(chassis["allow_passive_state_marker_transitions"])
        self.assertFalse(chassis["allow_motion_state_marker_transitions"])
        self.assertEqual(chassis["odom_source"], "external_verified")
        self.assertEqual(
            chassis["external_odom_topic"],
            self.renderer.PRIVATE_LIDAR_ODOM,
        )
        self.assertEqual(chassis["odom_topic"], "/odom")
        self.assertTrue(chassis["publish_odom_tf"])
        self.assertEqual(chassis["max_linear_x_mps"], 0.30)
        self.assertEqual(chassis["max_linear_y_mps"], 0.0)
        self.assertEqual(chassis["max_angular_z_rps"], 0.40)
        self.assertEqual(chassis["max_linear_accel_mps2"], 0.30)
        self.assertEqual(chassis["max_angular_accel_rps2"], 0.80)
        self.assertEqual(chassis["command_timeout_s"], 0.20)
        self.assertEqual(chassis["state_timeout_s"], 1.0)
        self.assertEqual(chassis["external_odom_timeout_s"], 1.0)
        self.assertEqual(chassis["commissioning_max_duration_s"], 0.0)
        self.assertEqual(chassis["commissioning_max_distance_m"], 0.0)

        sensors = self.named(result, "primitive", "go2_sensors")["config"]
        self.assertEqual(
            sensors["lidar_input_topic"], self.renderer.PRIVATE_CLOUD
        )
        self.assertEqual(
            sensors["imu_input_topic"], self.renderer.PRIVATE_IMU
        )
        self.assertIs(sensors["camera_quality_required"], False)
        self.assertIs(sensors["camera_required"], False)
        self.assertEqual(
            result["env"]["GO2_TIMESTAMP_DISCIPLINE_PROFILE"],
            "motion",
        )
        self.assertEqual(
            result["env"]["GO2_TIMESTAMP_CORRECTION_PROFILE"],
            self.renderer.PROFILE,
        )
        self.assertEqual(
            result["env"]["GO2_CLOUD_RELAY_PROFILE"],
            "motion",
        )
        mapping = self.named(result, "service", "mapping")["config"]
        self.assertEqual(mapping["map_mode"], "localization")
        self.assertEqual(mapping["map_id"], "${GO2_MAP_ID}")
        self.assertIs(mapping["reset_map"], False)
        self.assertEqual(
            mapping["sensor_providers"],
            {
                "lidar3d": "go2_sensors",
                "imu": "go2_sensors",
                "odom": "go2_chassis",
            },
        )
        nav = self.named(result, "service", "nav2")["config"]
        self.assertEqual(
            nav["velocity_output_topic"],
            self.renderer.NAV2_GUARD_INPUT_TOPIC,
        )
        self.assertIs(nav["external_velocity_guard"], True)
        self.assertNotEqual(
            nav["velocity_output_topic"],
            self.renderer.CHASSIS_COMMAND_TOPIC,
        )
        self.assertNotIn("topic_remap", nav)
        dashboard = self.named(
            result, "service", "go2_dashboard"
        )["config"]
        self.assertEqual(
            dashboard["port"], self.renderer.DASHBOARD_PORT
        )

    def test_provider_runtime_accepts_profile_but_still_requires_permit(
        self,
    ) -> None:
        from go2_chassis.runtime_config import normalize_config
        from go2_chassis.staged_nav2_permit import (
            PermitError,
            consume_staged_nav2_permit,
        )

        result = self.render()
        config = dict(
            self.named(result, "primitive", "go2_chassis")["config"]
        )
        # rbnx resolves this manifest substitution before invoking the
        # provider.  Exercise the provider with the corresponding concrete
        # post-substitution value.
        config["network_interface"] = "enp1s0"
        config["ipc_socket"] = "/tmp/staged-nav2-renderer-test.sock"
        runtime = normalize_config(
            config,
            {
                "GO2_ALLOWED_MODES": "3",
                "GO2_ALLOWED_STATE_MARKERS": "100",
            },
            ROOT / "packages" / "go2_chassis",
        )
        self.assertTrue(runtime.allow_motion)
        self.assertEqual(runtime.motion_profile, self.renderer.PROFILE)
        self.assertEqual(
            runtime.twist_in_topic,
            self.renderer.CHASSIS_COMMAND_TOPIC,
        )
        self.assertEqual(
            str(runtime.ipc_socket),
            "/tmp/staged-nav2-renderer-test.sock",
        )
        daemon_argv = runtime.daemon_argv(Path("/tmp/go2_sport_daemon"))
        self.assertEqual(
            daemon_argv[daemon_argv.index("--socket") + 1],
            str(runtime.ipc_socket),
        )
        adapter_argv = runtime.adapter_argv(
            Path("/tmp/go2_chassis_adapter_node"),
            Path("/tmp/adapter.yaml"),
        )
        self.assertIn(
            f"sdk_socket:={runtime.ipc_socket}",
            adapter_argv,
        )
        with self.assertRaisesRegex(PermitError, "PERMIT_FILE"):
            consume_staged_nav2_permit(
                runtime,
                {
                    "GO2_ALLOWED_MODES": "3",
                    "GO2_ALLOWED_STATE_MARKERS": "100",
                },
                ROOT,
            )

    def test_corrected_cloud_route_has_explicit_motion_profile(self) -> None:
        import workstation_nomotion_cloud_relay as cloud_relay

        result = self.render()
        sensors = self.named(
            result, "primitive", "go2_sensors"
        )["config"]
        self.assertIn(
            result["env"]["GO2_CLOUD_RELAY_PROFILE"],
            cloud_relay.RELAY_PROFILES,
        )
        self.assertEqual(
            result["env"]["GO2_CLOUD_RELAY_PROFILE"],
            "motion",
        )
        self.assertEqual(
            cloud_relay.OUTPUT_TOPIC,
            sensors["lidar_input_topic"],
        )
        self.assertEqual(cloud_relay.MOTION_PUBLISH_PERIOD_NS, 100_000_000)
        self.assertEqual(
            cloud_relay.MOTION_CLOUD_MAX_CORRECTED_AGE_NS,
            250_000_000,
        )

    def test_semantic_live_and_voice_execution_surfaces_are_absent(self) -> None:
        result = self.render()
        self.assertNotIn("pilot", result["system"])
        self.assertNotIn("liaison", result["system"])
        self.assertNotIn("scene", result["system"])
        self.assertNotIn(
            "audio_client_bridge",
            {entry["name"] for entry in result["primitive"]},
        )
        self.assertNotIn(
            "speech",
            {entry["name"] for entry in result["service"]},
        )
        self.assertEqual(result["skill"], [])
        dashboard = self.named(
            result, "service", "go2_dashboard"
        )["config"]
        self.assertIs(dashboard["browser_voice_enabled"], False)
        for key in self.renderer.VOICE_DASHBOARD_KEYS:
            self.assertNotIn(key, dashboard)
        self.assertNotIn("SPEECH_BACKEND", result["env"])

    def test_renderer_repairs_poisoned_base_critical_routes(self) -> None:
        poisoned = yaml.safe_load(yaml.safe_dump(self.base))
        chassis = self.named(poisoned, "primitive", "go2_chassis")["config"]
        chassis.update(
            {
                "allow_motion": False,
                "motion_profile": "general-navigation",
                "twist_in_topic": "/cmd_vel",
                "ipc_socket": "/tmp/poisoned-staged-nav2.sock",
                "allowed_state_markers": [1, 2],
                "allow_motion_state_marker_transitions": False,
                "max_linear_x_mps": 9.0,
                "stationary_pose_hold_enabled": True,
            }
        )
        nav = self.named(poisoned, "service", "nav2")["config"]
        nav["velocity_output_topic"] = "/cmd_vel"
        nav["external_velocity_guard"] = False
        nav["topic_remap"] = {"cmd_vel": "/cmd_vel"}
        mapping = self.named(poisoned, "service", "mapping")["config"]
        mapping["map_mode"] = "mapping"

        result = self.renderer.render(
            poisoned,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )
        chassis = self.named(result, "primitive", "go2_chassis")["config"]
        nav = self.named(result, "service", "nav2")["config"]
        mapping = self.named(result, "service", "mapping")["config"]
        self.assertTrue(chassis["allow_motion"])
        self.assertEqual(chassis["motion_profile"], self.renderer.PROFILE)
        self.assertEqual(
            chassis["twist_in_topic"],
            self.renderer.CHASSIS_COMMAND_TOPIC,
        )
        self.assertEqual(
            chassis["ipc_socket"],
            self.renderer.IPC_SOCKET_ENV,
        )
        self.assertNotIn("allowed_state_markers", chassis)
        self.assertFalse(chassis["allow_motion_state_marker_transitions"])
        self.assertNotIn("stationary_pose_hold_enabled", chassis)
        self.assertEqual(
            nav["velocity_output_topic"],
            self.renderer.NAV2_GUARD_INPUT_TOPIC,
        )
        self.assertIs(nav["external_velocity_guard"], True)
        self.assertNotIn("topic_remap", nav)
        self.assertEqual(mapping["map_mode"], "localization")

    def test_validator_rejects_every_control_route_or_feature_drift(self) -> None:
        mutations = (
            (
                "chassis_profile",
                lambda manifest: self.named(
                    manifest, "primitive", "go2_chassis"
                )["config"].__setitem__("motion_profile", "other"),
            ),
            (
                "chassis_command",
                lambda manifest: self.named(
                    manifest, "primitive", "go2_chassis"
                )["config"].__setitem__("twist_in_topic", "/cmd_vel"),
            ),
            (
                "chassis_socket",
                lambda manifest: self.named(
                    manifest, "primitive", "go2_chassis"
                )["config"].__setitem__(
                    "ipc_socket", "/tmp/wrong-staged-nav2.sock"
                ),
            ),
            (
                "chassis_limit",
                lambda manifest: self.named(
                    manifest, "primitive", "go2_chassis"
                )["config"].__setitem__("max_angular_z_rps", 0.401),
            ),
            (
                "marker_bypass",
                lambda manifest: self.named(
                    manifest, "primitive", "go2_chassis"
                )["config"].__setitem__("allowed_state_markers", [2010]),
            ),
            (
                "classic_marker_transition_enabled",
                lambda manifest: self.named(
                    manifest, "primitive", "go2_chassis"
                )["config"].__setitem__(
                    "allow_motion_state_marker_transitions", True
                ),
            ),
            (
                "sensor_raw",
                lambda manifest: self.named(
                    manifest, "primitive", "go2_sensors"
                )["config"].__setitem__(
                    "lidar_input_topic", "/utlidar/cloud"
                ),
            ),
            (
                "mapping_mode",
                lambda manifest: self.named(
                    manifest, "service", "mapping"
                )["config"].__setitem__("map_mode", "mapping"),
            ),
            (
                "nav_bypass",
                lambda manifest: self.named(
                    manifest, "service", "nav2"
                )["config"].__setitem__(
                    "velocity_output_topic",
                    self.renderer.CHASSIS_COMMAND_TOPIC,
                ),
            ),
            (
                "nav_remap",
                lambda manifest: self.named(
                    manifest, "service", "nav2"
                )["config"].__setitem__(
                    "topic_remap", {"cmd_vel": "/cmd_vel"}
                ),
            ),
            (
                "nav_external_guard_disabled",
                lambda manifest: self.named(
                    manifest, "service", "nav2"
                )["config"].__setitem__("external_velocity_guard", False),
            ),
            (
                "voice",
                lambda manifest: self.named(
                    manifest, "service", "go2_dashboard"
                )["config"].__setitem__("browser_voice_enabled", True),
            ),
            (
                "dashboard_port",
                lambda manifest: self.named(
                    manifest, "service", "go2_dashboard"
                )["config"].__setitem__("port", "${GO2_DASHBOARD_PORT}"),
            ),
            (
                "semantic_skill",
                lambda manifest: manifest["skill"].append(
                    {"name": "semantic_navigation", "config": {}}
                ),
            ),
            (
                "permit_embedded",
                lambda manifest: manifest["env"].__setitem__(
                    "GO2_STAGED_NAV2_MAP_ID", "embedded-map"
                ),
            ),
        )
        for label, mutate in mutations:
            with self.subTest(label=label):
                candidate = yaml.safe_load(yaml.safe_dump(self.render()))
                mutate(candidate)
                with self.assertRaisesRegex(
                    self.renderer.ManifestError,
                    "unsafe staged-nav2 manifest",
                ):
                    self.renderer.validate_rendered(candidate)

    def test_output_is_private_and_first_motion_renderer_is_not_reused(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as temporary:
            output = Path(temporary) / "robonix_manifest.yaml"
            self.renderer.write_manifest(output, self.render())
            self.assertEqual(output.stat().st_mode & 0o777, 0o600)
        source = (
            TIME_SYNC / "render_workstation_staged_nav2_manifest.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("render_workstation_first_motion_manifest", source)


if __name__ == "__main__":
    unittest.main()
