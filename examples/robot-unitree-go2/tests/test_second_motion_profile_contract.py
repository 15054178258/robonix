from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = (
    ROOT / "deploy" / "workstation-second-motion-corrected" / "profile.yaml"
)


class SecondMotionProfileContractTest(unittest.TestCase):
    def test_profile_is_nonrunnable_independent_and_bounded(self) -> None:
        data = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
        self.assertNotIn("manifestVersion", data)
        self.assertEqual(
            data["profile"],
            "workstation-second-motion-corrected-v1",
        )
        self.assertEqual(
            data["command_topic"],
            "/go2/second_motion/cmd_vel",
        )
        self.assertEqual(data["canonical_command_topic"], "/cmd_vel")
        self.assertEqual(data["odom_source"], "external_verified")
        self.assertEqual(
            data["external_odom_topic"],
            "/robonix/time_corrected/raw/utlidar/robot_odom",
        )
        self.assertEqual(data["external_odom_timeout_s"], 0.20)
        self.assertEqual(
            data["timestamp_discipline"]["max_source_receipt_delta_s"],
            0.20,
        )
        self.assertEqual(
            data["motion"],
            {
                "max_linear_x_mps": 0.30,
                "max_linear_y_mps": 0.0,
                "max_angular_z_rps": 0.0,
                "max_linear_accel_mps2": 0.30,
                "max_angular_accel_rps2": 0.10,
                "soft_stop_duration_s": 1.2,
                "soft_stop_distance_m": 0.20,
                "max_duration_s": 1.5,
                "max_distance_m": 0.30,
                "command_timeout_s": 0.20,
                "control_rate_hz": 20.0,
            },
        )
        self.assertEqual(
            data["acceptance"],
            {
                "minimum_forward_displacement_m": 0.15,
                "maximum_total_displacement_m_exclusive": 0.30,
                "maximum_lateral_displacement_m": 0.05,
                "maximum_yaw_change_rad": 0.20,
                "commissioning_motion_active_observed": True,
            },
        )
        self.assertEqual(
            data["ownership"]["command_publisher"],
            "/go2_second_motion_probe",
        )
        self.assertEqual(data["ownership"]["canonical_cmd_vel_publishers"], 0)
        self.assertEqual(
            data["permit"]["schema"],
            "robonix-go2-second-motion-permit-v1",
        )
        self.assertEqual(
            data["permit"]["evidence"],
            ["dds_identity", "state", "time", "first_motion_pass"],
        )
        self.assertTrue(data["permit"]["one_time"])
        self.assertEqual(data["post_stop"]["guard_state"], "DISARMED")
        self.assertEqual(data["post_stop"]["minimum_odom_samples"], 10)
        self.assertEqual(
            data["post_stop"]["continuous_stationary_observation_s"],
            1.0,
        )


if __name__ == "__main__":
    unittest.main()
