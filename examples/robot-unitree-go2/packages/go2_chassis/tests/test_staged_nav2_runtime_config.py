from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from go2_chassis.runtime_config import (  # noqa: E402
    ConfigError,
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    FIRST_MOTION_PROFILE,
    STAGED_NAV2_COMMAND_TOPIC,
    STAGED_NAV2_PROFILE,
    normalize_config,
)


def staged_config(**overrides):
    config = {
        "allow_motion": True,
        "motion_profile": STAGED_NAV2_PROFILE,
        "operator_present": True,
        "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
        "network_interface": "wlan-stage",
        "ipc_socket": "/tmp/go2-staged-test.sock",
        "state_topic": "/robonix/time_corrected/motion/sportmodestate",
        "state_fallback_topic": "",
        "twist_in_topic": STAGED_NAV2_COMMAND_TOPIC,
        "odom_source": "external_verified",
        "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
        "external_odom_timeout_s": 1.0,
        "odom_topic": "/odom",
        "publish_odom_tf": True,
        "max_linear_x_mps": 0.30,
        "max_linear_y_mps": 0.0,
        "max_angular_z_rps": 0.40,
        "max_linear_accel_mps2": 0.30,
        "max_angular_accel_rps2": 0.80,
        "command_timeout_s": 0.20,
        "state_timeout_s": 1.0,
        "max_source_stamp_age_s": 0.20,
        "max_source_stamp_future_skew_s": 0.05,
        "commissioning_max_duration_s": 0.0,
        "commissioning_max_distance_m": 0.0,
    }
    config.update(overrides)
    return config


def first_motion_config():
    return {
        "allow_motion": True,
        "motion_profile": FIRST_MOTION_PROFILE,
        "operator_present": True,
        "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
        "network_interface": "enp1s0",
        "ipc_socket": "/tmp/go2-first-test.sock",
        "twist_in_topic": "/go2/commissioning/cmd_vel",
        "odom_topic": "/odom",
        "max_linear_x_mps": 0.05,
        "max_linear_y_mps": 0.0,
        "max_angular_z_rps": 0.0,
        "command_timeout_s": 0.20,
        "commissioning_max_duration_s": 2.0,
        "commissioning_max_distance_m": 0.10,
    }


class StagedNav2RuntimeConfigTest(unittest.TestCase):
    def test_exact_stage1_profile_reaches_both_processes(self) -> None:
        runtime = normalize_config(
            staged_config(),
            {
                "GO2_ALLOWED_MODES": "0",
                "GO2_ALLOWED_STATE_MARKERS": "100",
            },
            ROOT,
        )
        self.assertEqual(runtime.motion_profile, STAGED_NAV2_PROFILE)
        self.assertFalse(runtime.preserve_classic_walk)
        adapter = runtime.adapter_argv(
            Path("/tmp/adapter"), Path("/tmp/params.yaml")
        )
        daemon = runtime.daemon_argv(Path("/tmp/daemon"))
        rendered = "\n".join(adapter)
        for expected in (
            f"motion_profile:={STAGED_NAV2_PROFILE}",
            f"cmd_vel_topic:={STAGED_NAV2_COMMAND_TOPIC}",
            "odom_source:=external_verified",
            f"external_odom_topic:={DEFAULT_EXTERNAL_ODOM_TOPIC}",
            "max_vx:=0.3",
            "max_vy:=0.0",
            "max_wz:=0.4",
            "max_linear_acceleration:=0.3",
            "max_angular_acceleration:=0.8",
            "command_timeout_sec:=0.2",
            "commissioning_max_duration_sec:=0.0",
            "commissioning_max_distance_m:=0.0",
        ):
            self.assertIn(expected, rendered)
        self.assertEqual(
            daemon[daemon.index("--motion-profile") + 1],
            STAGED_NAV2_PROFILE,
        )
        self.assertEqual(daemon[daemon.index("--max-vx") + 1], "0.3")
        self.assertEqual(daemon[daemon.index("--max-vy") + 1], "0.0")
        self.assertEqual(daemon[daemon.index("--max-wz") + 1], "0.4")
        self.assertEqual(daemon[daemon.index("--max-motion-ms") + 1], "0")
        self.assertNotIn("--preserve-classic-walk", daemon)

    def test_classic_walk_preservation_is_explicit_and_reaches_both_processes(self) -> None:
        runtime = normalize_config(
            staged_config(preserve_classic_walk=True),
            {
                "GO2_ALLOWED_MODES": "0",
                "GO2_ALLOWED_STATE_MARKERS": "100,2010",
            },
            ROOT,
        )
        self.assertTrue(runtime.preserve_classic_walk)
        adapter = runtime.adapter_argv(
            Path("/tmp/adapter"), Path("/tmp/params.yaml")
        )
        daemon = runtime.daemon_argv(Path("/tmp/daemon"))
        self.assertIn("preserve_classic_walk:=true", adapter)
        self.assertIn("--preserve-classic-walk", daemon)

        with self.assertRaisesRegex(ConfigError, "staged Nav2"):
            normalize_config(
                {**first_motion_config(), "preserve_classic_walk": True},
                {"GO2_ALLOWED_MODES": "0"},
                ROOT,
            )

    def test_every_stage1_route_and_envelope_mutation_fails_closed(self) -> None:
        mutations = (
            {"twist_in_topic": "/cmd_vel"},
            {"odom_topic": "/other_odom"},
            {"publish_odom_tf": False},
            {"odom_source": "sport_state"},
            {"external_odom_topic": "/raw/odom"},
            {"external_odom_timeout_s": 0.19},
            {"max_linear_x_mps": 0.301},
            {"max_linear_x_mps": 0.249},
            {"max_linear_y_mps": 0.001},
            {"max_angular_z_rps": 0.401},
            {"max_angular_z_rps": 0.399},
            {"max_linear_accel_mps2": 0.301},
            {"max_linear_accel_mps2": 0.299},
            {"max_angular_accel_rps2": 0.801},
            {"max_angular_accel_rps2": 0.799},
            {"command_timeout_s": 0.19},
            {"state_timeout_s": 0.19},
            {"max_source_stamp_age_s": 0.19},
            {"max_source_stamp_future_skew_s": 0.04},
            {"commissioning_max_duration_s": 0.1},
            {"commissioning_max_distance_m": 0.1},
        )
        environment = {
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "100",
        }
        for mutation in mutations:
            with self.subTest(mutation=mutation), self.assertRaises(ConfigError):
                normalize_config(staged_config(**mutation), environment, ROOT)

    def test_legacy_first_motion_argv_has_no_new_profile_selector(self) -> None:
        runtime = normalize_config(
            first_motion_config(), {"GO2_ALLOWED_MODES": "0"}, ROOT
        )
        adapter = runtime.adapter_argv(
            Path("/tmp/adapter"), Path("/tmp/params.yaml")
        )
        daemon = runtime.daemon_argv(Path("/tmp/daemon"))
        self.assertFalse(
            any(argument.startswith("motion_profile:=") for argument in adapter)
        )
        self.assertNotIn("--motion-profile", daemon)
        self.assertEqual(daemon[daemon.index("--max-motion-ms") + 1], "2000")
        self.assertEqual(daemon[daemon.index("--max-wz") + 1], "0.0")


if __name__ == "__main__":
    unittest.main()
