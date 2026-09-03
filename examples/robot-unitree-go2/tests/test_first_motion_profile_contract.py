from pathlib import Path
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deploy" / "time-sync"))

from navigation_stamp_discipline import (  # noqa: E402
    go2_workstation_motion_config,
)

PROFILE = (
    ROOT / "deploy" / "workstation-first-motion-corrected" / "profile.yaml"
)


class FirstMotionProfileContractTest(unittest.TestCase):
    def test_profile_is_not_directly_runnable_and_has_fixed_envelope(self) -> None:
        data = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
        self.assertNotIn("manifestVersion", data)
        self.assertEqual(data["profile"], "workstation-first-motion-corrected-v1")
        self.assertEqual(
            data["state_topic"],
            "/robonix/time_corrected/motion/sportmodestate",
        )
        self.assertEqual(data["command_topic"], "/go2/commissioning/cmd_vel")
        self.assertEqual(data["canonical_command_topic"], "/cmd_vel")
        self.assertEqual(data["odom_source"], "external_verified")
        self.assertEqual(
            data["external_odom_topic"],
            "/robonix/time_corrected/raw/utlidar/robot_odom",
        )
        self.assertEqual(data["odom_topic"], "/odom")
        self.assertTrue(data["publish_odom_tf"])
        self.assertEqual(
            data["timestamp_discipline"],
            {
                "profile": "motion",
                "mode": "affine",
                "required_streams": [
                    "sport_primary",
                    "mid360_imu",
                    "mid360_odom",
                ],
                "excluded_streams": ["mid360_cloud"],
                "max_stream_age_s": 0.20,
                "max_source_receipt_delta_s": 0.20,
            },
        )
        timestamp_contract = data["timestamp_discipline"]
        runtime = go2_workstation_motion_config()
        policies = {policy.name: policy for policy in runtime.streams}
        self.assertEqual(
            set(policies), set(timestamp_contract["required_streams"])
        )
        self.assertTrue(
            set(timestamp_contract["excluded_streams"]).isdisjoint(policies)
        )
        for policy in policies.values():
            self.assertEqual(
                policy.max_corrected_age_ns,
                int(timestamp_contract["max_stream_age_s"] * 1_000_000_000),
            )
            effective_delta_ns = (
                policy.max_source_receipt_delta_error_ns
                if policy.max_source_receipt_delta_error_ns is not None
                else runtime.max_source_receipt_delta_error_ns
            )
            self.assertEqual(
                effective_delta_ns,
                int(
                    timestamp_contract["max_source_receipt_delta_s"]
                    * 1_000_000_000
                ),
            )
        self.assertEqual(
            data["motion"],
            {
                "max_linear_x_mps": 0.05,
                "max_linear_y_mps": 0.0,
                "max_angular_z_rps": 0.0,
                "max_duration_s": 2.0,
                "max_distance_m": 0.10,
                "command_timeout_s": 0.20,
            },
        )
        self.assertEqual(data["ownership"]["canonical_cmd_vel_publishers"], 0)
        self.assertTrue(data["permit"]["one_time"])
        self.assertEqual(data["permit"]["max_lifetime_s"], 300)
        self.assertTrue(data["post_stop"]["explicit_disarm_required"])
        self.assertEqual(data["post_stop"]["daemon_armed"], False)
        self.assertEqual(
            data["post_stop"]["continuous_stationary_observation_s"], 1.0
        )
        self.assertEqual(data["post_stop"]["max_linear_speed_mps"], 0.03)
        self.assertEqual(data["post_stop"]["max_yaw_rate_rps"], 0.03)
        self.assertEqual(data["post_stop"]["max_pose_drift_m"], 0.02)


if __name__ == "__main__":
    unittest.main()
