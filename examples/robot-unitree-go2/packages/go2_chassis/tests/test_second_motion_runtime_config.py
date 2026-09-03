from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from go2_chassis.runtime_config import (  # noqa: E402
    ConfigError,
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    SECOND_MOTION_COMMAND_TOPIC,
    SECOND_MOTION_CONTROL_RATE_HZ,
    SECOND_MOTION_PROFILE,
    normalize_config,
)


def second_motion_config(**overrides):
    config = {
        "allow_motion": True,
        "motion_profile": SECOND_MOTION_PROFILE,
        "operator_present": True,
        "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
        "network_interface": "wlan-second",
        "ipc_socket": "/tmp/go2-second-test.sock",
        "state_topic": "/robonix/time_corrected/motion/sportmodestate",
        "state_fallback_topic": "",
        "twist_in_topic": SECOND_MOTION_COMMAND_TOPIC,
        "odom_source": "external_verified",
        "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
        "external_odom_timeout_s": 0.20,
        "odom_topic": "/odom",
        "publish_odom_tf": True,
        "stationary_pose_hold_enabled": False,
        "max_linear_x_mps": 0.30,
        "max_linear_y_mps": 0.0,
        "max_angular_z_rps": 0.0,
        "max_linear_accel_mps2": 0.30,
        "max_angular_accel_rps2": 0.10,
        "command_timeout_s": 0.20,
        "control_rate_hz": SECOND_MOTION_CONTROL_RATE_HZ,
        "state_timeout_s": 0.20,
        "max_source_stamp_age_s": 0.20,
        "max_source_stamp_future_skew_s": 0.05,
        "commissioning_max_duration_s": 1.5,
        "commissioning_max_distance_m": 0.30,
    }
    config.update(overrides)
    return config


class SecondMotionRuntimeConfigTest(unittest.TestCase):
    def test_exact_second_profile_reaches_both_processes(self) -> None:
        runtime = normalize_config(
            second_motion_config(),
            {
                "GO2_ALLOWED_MODES": "0",
                "GO2_ALLOWED_STATE_MARKERS": "100",
            },
            ROOT,
        )
        self.assertEqual(runtime.motion_profile, SECOND_MOTION_PROFILE)
        adapter = runtime.adapter_argv(
            Path("/tmp/adapter"), Path("/tmp/params.yaml")
        )
        daemon = runtime.daemon_argv(Path("/tmp/daemon"))
        rendered = "\n".join(adapter)
        for expected in (
            f"motion_profile:={SECOND_MOTION_PROFILE}",
            f"cmd_vel_topic:={SECOND_MOTION_COMMAND_TOPIC}",
            "odom_source:=external_verified",
            f"external_odom_topic:={DEFAULT_EXTERNAL_ODOM_TOPIC}",
            "external_odom_timeout_sec:=0.2",
            "max_vx:=0.3",
            "max_vy:=0.0",
            "max_wz:=0.0",
            "max_linear_acceleration:=0.3",
            "max_angular_acceleration:=0.1",
            "command_timeout_sec:=0.2",
            "control_rate_hz:=20.0",
            "commissioning_max_duration_sec:=1.5",
            "commissioning_max_distance_m:=0.3",
        ):
            self.assertIn(expected, rendered)
        self.assertEqual(
            daemon[daemon.index("--motion-profile") + 1],
            SECOND_MOTION_PROFILE,
        )
        self.assertEqual(daemon[daemon.index("--max-vx") + 1], "0.3")
        self.assertEqual(daemon[daemon.index("--max-vy") + 1], "0.0")
        self.assertEqual(daemon[daemon.index("--max-wz") + 1], "0.0")
        self.assertEqual(
            daemon[daemon.index("--max-motion-ms") + 1], "1500"
        )

    def test_every_second_route_and_envelope_mutation_fails_closed(self) -> None:
        mutations = (
            {"twist_in_topic": "/go2/commissioning/cmd_vel"},
            {"twist_in_topic": "/cmd_vel"},
            {"odom_topic": "/other_odom"},
            {"publish_odom_tf": False},
            {"odom_source": "sport_state"},
            {"external_odom_topic": "/raw/odom"},
            {"external_odom_timeout_s": 0.19},
            {"max_linear_x_mps": 0.301},
            {"max_linear_x_mps": 0.299},
            {"max_linear_y_mps": 0.001},
            {"max_angular_z_rps": 0.001},
            {"max_linear_accel_mps2": 0.301},
            {"max_linear_accel_mps2": 0.299},
            {"max_angular_accel_rps2": 0.11},
            {"max_angular_accel_rps2": 0.09},
            {"command_timeout_s": 0.19},
            {"control_rate_hz": 19.9},
            {"control_rate_hz": 50.0},
            {"state_timeout_s": 0.19},
            {"max_source_stamp_age_s": 0.10},
            {"max_source_stamp_future_skew_s": 0.04},
            {"commissioning_max_duration_s": 1.49},
            {"commissioning_max_distance_m": 0.29},
        )
        environment = {
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "100",
        }
        for mutation in mutations:
            with self.subTest(mutation=mutation), self.assertRaises(
                ConfigError
            ):
                normalize_config(
                    second_motion_config(**mutation),
                    environment,
                    ROOT,
                )


if __name__ == "__main__":
    unittest.main()
