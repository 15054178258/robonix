import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from go2_chassis.runtime_config import (  # noqa: E402
    CLASSIC_MOTION_STATE_MARKERS,
    ConfigError,
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    ODOM_SOURCE_EXTERNAL_VERIFIED,
    ODOM_SOURCE_SPORT_STATE,
    STAGED_NAV2_COMMAND_TOPIC,
    STAGED_NAV2_PROFILE,
    UNKNOWN_MODE,
    normalize_config,
    prepare_private_directory,
)


def first_motion_config(**overrides):
    config = {
        "allow_motion": True,
        "motion_profile": "workstation-first-motion-corrected-v1",
        "operator_present": True,
        "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
        "network_interface": "enp1s0",
        "ipc_socket": "/tmp/go2-test.sock",
        "twist_in_topic": "/go2/commissioning/cmd_vel",
        "odom_topic": "/odom",
        "max_linear_x_mps": 0.05,
        "max_linear_y_mps": 0.0,
        "max_angular_z_rps": 0.0,
        "command_timeout_s": 0.20,
        "commissioning_max_duration_s": 2.0,
        "commissioning_max_distance_m": 0.10,
    }
    config.update(overrides)
    return config


def staged_nav2_config(**overrides):
    config = first_motion_config(
        motion_profile=STAGED_NAV2_PROFILE,
        twist_in_topic=STAGED_NAV2_COMMAND_TOPIC,
        odom_source=ODOM_SOURCE_EXTERNAL_VERIFIED,
        external_odom_topic=DEFAULT_EXTERNAL_ODOM_TOPIC,
        external_odom_timeout_s=1.0,
        state_timeout_s=1.0,
        max_linear_x_mps=0.30,
        max_linear_y_mps=0.0,
        max_angular_z_rps=0.40,
        commissioning_max_duration_s=0.0,
        commissioning_max_distance_m=0.0,
    )
    config.update(overrides)
    return config


class RuntimeConfigTest(unittest.TestCase):
    def test_shared_params_omit_unfinished_stationary_pose_hold(self) -> None:
        params = (ROOT / "config" / "adapter.yaml").read_text(encoding="utf-8")
        for unfinished_parameter in (
            "stationary_pose_hold_enabled",
            "stationary_hold_dwell_sec",
            "stationary_hold_sport_max_linear_mps",
            "stationary_hold_sport_max_yaw_rps",
            "stationary_hold_external_twist_max_linear_mps",
            "stationary_hold_external_twist_max_yaw_rps",
            "stationary_hold_pose_max_linear_rate_mps",
            "stationary_hold_pose_max_yaw_rate_rps",
        ):
            self.assertNotIn(unfinished_parameter, params)

    def test_private_directory_is_created_and_secured(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "runtime" / "socket"
            prepare_private_directory(directory)
            info = os.lstat(directory)
            self.assertTrue(stat.S_ISDIR(info.st_mode))
            self.assertEqual(info.st_uid, os.geteuid())
            self.assertEqual(stat.S_IMODE(info.st_mode), 0o700)

    def test_private_directory_rejects_symlink_without_chmod_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "target"
            target.mkdir()
            os.chmod(target, 0o755)
            link = root / "runtime"
            link.symlink_to(target, target_is_directory=True)

            with self.assertRaisesRegex(ConfigError, "symlink"):
                prepare_private_directory(link)
            self.assertEqual(stat.S_IMODE(os.lstat(target).st_mode), 0o755)

    def test_private_directory_rejects_foreign_uid_without_chmod(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "runtime"
            directory.mkdir()
            os.chmod(directory, 0o755)
            simulated_foreign_uid = os.geteuid() + 1

            with mock.patch(
                "go2_chassis.runtime_config.os.geteuid",
                return_value=simulated_foreign_uid,
            ), self.assertRaisesRegex(ConfigError, "current UID"):
                prepare_private_directory(directory)
            self.assertEqual(stat.S_IMODE(os.lstat(directory).st_mode), 0o755)

    def test_sdk_daemon_environment_isolated_from_ros_library_path(self) -> None:
        runtime = normalize_config(
            first_motion_config(), {"GO2_ALLOWED_MODES": "0"}, ROOT
        )
        daemon = ROOT / "rbnx-build" / "sdk" / "install" / "bin" / "go2_sport_daemon"
        with mock.patch.dict(
            os.environ,
            {"LD_LIBRARY_PATH": "/opt/ros/humble/lib:/unexpected"},
            clear=False,
        ):
            environment = runtime.sdk_daemon_env(daemon)
        self.assertEqual(
            environment["LD_LIBRARY_PATH"],
            str(daemon.resolve().parent.parent / "lib"),
        )
        self.assertNotIn("/opt/ros", environment["LD_LIBRARY_PATH"])
        self.assertNotIn("/unexpected", environment["LD_LIBRARY_PATH"])
        self.assertEqual(environment["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")

    def test_default_is_read_only_with_impossible_mode(self) -> None:
        runtime = normalize_config({}, {}, ROOT)
        self.assertFalse(runtime.allow_motion)
        self.assertFalse(runtime.starts_sdk_daemon)
        self.assertTrue(runtime.publish_odom_tf)
        self.assertEqual(runtime.odom_source, ODOM_SOURCE_SPORT_STATE)
        self.assertEqual(runtime.external_odom_topic, DEFAULT_EXTERNAL_ODOM_TOPIC)
        self.assertFalse(runtime.stationary_pose_hold_enabled)
        self.assertEqual(runtime.stationary_hold_dwell_s, 2.0)
        self.assertEqual(runtime.stationary_hold_sport_max_linear_mps, 0.03)
        self.assertEqual(runtime.stationary_hold_sport_max_yaw_rps, 0.03)
        self.assertEqual(
            runtime.stationary_hold_external_twist_max_linear_mps, 0.03
        )
        self.assertEqual(
            runtime.stationary_hold_external_twist_max_yaw_rps, 0.03
        )
        self.assertEqual(runtime.stationary_hold_pose_max_linear_rate_mps, 0.005)
        self.assertEqual(runtime.stationary_hold_pose_max_yaw_rate_rps, 0.01)
        self.assertEqual(runtime.allowed_modes, (UNKNOWN_MODE,))
        self.assertEqual(runtime.allowed_state_markers, ())
        self.assertFalse(runtime.allow_passive_state_marker_transitions)
        self.assertFalse(runtime.allow_motion_state_marker_transitions)
        self.assertFalse(runtime.preserve_classic_walk)
        self.assertEqual(runtime.process_env()["GO2_ALLOWED_STATE_MARKERS"], "")
        self.assertEqual(runtime.max_source_stamp_age_s, 0.20)
        self.assertEqual(runtime.max_source_stamp_future_skew_s, 0.05)
        self.assertTrue(str(runtime.ipc_socket).startswith(
            f"/tmp/robonix-go2-"
        ))
        self.assertLess(len(str(runtime.ipc_socket)), 108)
        with self.assertRaises(ConfigError):
            runtime.daemon_argv(Path("/tmp/daemon"))

    def test_driver_config_reaches_every_adapter_parameter(self) -> None:
        config = {
            "state_topic": "/go2/high_state",
            "state_fallback_topic": "/go2/low_state",
            "twist_in_topic": "/nav/cmd_vel",
            "odom_topic": "/go2/odom",
            "publish_odom_tf": False,
            "imu_topic": "/go2/imu",
            "odom_frame": "go2_odom",
            "base_frame": "go2_base",
            "imu_frame": "go2_imu",
            "velocity_frame": "base_link",
            "ipc_socket": "/tmp/go2-test.sock",
            "max_linear_x_mps": 0.1,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.2,
            "max_linear_accel_mps2": 0.2,
            "max_angular_accel_rps2": 0.4,
            "command_timeout_s": 0.2,
            "state_timeout_s": 0.15,
            "max_source_stamp_age_s": 0.15,
            "max_source_stamp_future_skew_s": 0.02,
            "zero_preamble_s": 1.0,
        }
        runtime = normalize_config(config, {}, ROOT)
        argv = runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        rendered = "\n".join(argv)
        for expected in (
            "sport_state_topic:=/go2/high_state",
            "state_fallback_topic:=/go2/low_state",
            "cmd_vel_topic:=/nav/cmd_vel",
            "odom_topic:=/go2/odom",
            "odom_source:=sport_state",
            "external_odom_topic:=" + DEFAULT_EXTERNAL_ODOM_TOPIC,
            "external_odom_timeout_sec:=0.2",
            "max_external_odom_yaw_jump_rad:=1.0",
            "publish_odom_tf:=false",
            "imu_topic:=/go2/imu",
            "odom_frame:=go2_odom",
            "base_frame:=go2_base",
            "imu_frame:=go2_imu",
            "state_velocity_frame:=base_link",
            "sdk_socket:=/tmp/go2-test.sock",
            "max_vx:=0.1",
            "max_vy:=0.0",
            "max_wz:=0.2",
            "max_source_stamp_age_sec:=0.15",
            "max_source_stamp_future_skew_sec:=0.02",
            "allowed_modes:=[255]",
            "allow_passive_state_marker_transitions:=false",
        ):
            self.assertIn(expected, rendered)
        for unfinished_parameter in (
            "stationary_pose_hold_enabled:=",
            "stationary_hold_dwell_sec:=",
            "stationary_hold_sport_max_linear_mps:=",
            "stationary_hold_sport_max_yaw_rps:=",
            "stationary_hold_external_twist_max_linear_mps:=",
            "stationary_hold_external_twist_max_yaw_rps:=",
            "stationary_hold_pose_max_linear_rate_mps:=",
            "stationary_hold_pose_max_yaw_rate_rps:=",
        ):
            self.assertNotIn(unfinished_parameter, rendered)
        self.assertNotIn("allowed_state_markers:=", rendered)

    def test_stationary_pose_hold_is_narrow_and_reaches_every_parameter(
        self,
    ) -> None:
        runtime = normalize_config(
            {
                "allow_motion": False,
                "odom_source": ODOM_SOURCE_EXTERNAL_VERIFIED,
                "stationary_pose_hold_enabled": True,
                "stationary_hold_dwell_s": 3.0,
                "stationary_hold_sport_max_linear_mps": 0.02,
                "stationary_hold_sport_max_yaw_rps": 0.02,
                "stationary_hold_external_twist_max_linear_mps": 0.02,
                "stationary_hold_external_twist_max_yaw_rps": 0.02,
                "stationary_hold_pose_max_linear_rate_mps": 0.004,
                "stationary_hold_pose_max_yaw_rate_rps": 0.009,
            },
            {},
            ROOT,
        )
        self.assertTrue(runtime.stationary_pose_hold_enabled)
        rendered = "\n".join(
            runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        )
        for expected in (
            "stationary_pose_hold_enabled:=true",
            "stationary_hold_dwell_sec:=3.0",
            "stationary_hold_sport_max_linear_mps:=0.02",
            "stationary_hold_sport_max_yaw_rps:=0.02",
            "stationary_hold_external_twist_max_linear_mps:=0.02",
            "stationary_hold_external_twist_max_yaw_rps:=0.02",
            "stationary_hold_pose_max_linear_rate_mps:=0.004",
            "stationary_hold_pose_max_yaw_rate_rps:=0.009",
        ):
            self.assertIn(expected, rendered)

    def test_stationary_pose_hold_rejects_sport_state_and_motion(self) -> None:
        with self.assertRaisesRegex(ConfigError, "stationary_pose_hold_enabled"):
            normalize_config({"stationary_pose_hold_enabled": True}, {}, ROOT)

        with self.assertRaisesRegex(ConfigError, "stationary_pose_hold_enabled"):
            normalize_config(
                first_motion_config(
                    odom_source=ODOM_SOURCE_EXTERNAL_VERIFIED,
                    stationary_pose_hold_enabled=True,
                ),
                {"GO2_ALLOWED_MODES": "0"},
                ROOT,
            )

    def test_stationary_pose_hold_thresholds_are_finite_and_cannot_widen(
        self,
    ) -> None:
        invalid = (
            {"stationary_hold_dwell_s": 0.99},
            {"stationary_hold_dwell_s": 10.01},
            {"stationary_hold_sport_max_linear_mps": 0.0},
            {"stationary_hold_sport_max_linear_mps": 0.031},
            {"stationary_hold_sport_max_yaw_rps": float("nan")},
            {"stationary_hold_sport_max_yaw_rps": 0.031},
            {"stationary_hold_external_twist_max_linear_mps": 0.031},
            {"stationary_hold_external_twist_max_yaw_rps": 0.031},
            {"stationary_hold_pose_max_linear_rate_mps": 0.0051},
            {"stationary_hold_pose_max_yaw_rate_rps": 0.0101},
        )
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(ConfigError):
                normalize_config(config, {}, ROOT)

    def test_external_verified_odom_is_private_distinct_and_chassis_owned(self) -> None:
        runtime = normalize_config(
            {
                "odom_source": ODOM_SOURCE_EXTERNAL_VERIFIED,
                "external_odom_topic": (
                    "/robonix/time_corrected/private/go2/lidar_odom"
                ),
                "external_odom_timeout_s": 0.15,
                "max_external_odom_yaw_jump_rad": 0.5,
            },
            {},
            ROOT,
        )
        self.assertEqual(runtime.odom_source, ODOM_SOURCE_EXTERNAL_VERIFIED)
        rendered = "\n".join(
            runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        )
        for expected in (
            "odom_source:=external_verified",
            "external_odom_topic:=/robonix/time_corrected/private/go2/lidar_odom",
            "external_odom_timeout_sec:=0.15",
            "max_external_odom_yaw_jump_rad:=0.5",
            "odom_topic:=/odom",
            "publish_odom_tf:=true",
        ):
            self.assertIn(expected, rendered)

    def test_external_verified_odom_rejects_feedback_or_public_source(self) -> None:
        invalid = (
            {"external_odom_topic": "/odom"},
            {"external_odom_topic": "/utlidar/robot_odom"},
            {"odom_topic": "/go2/odom"},
            {"publish_odom_tf": False},
            {"external_odom_timeout_s": 0.21},
            {"max_external_odom_yaw_jump_rad": 1.01},
        )
        for override in invalid:
            config = {"odom_source": ODOM_SOURCE_EXTERNAL_VERIFIED, **override}
            with self.subTest(config=config), self.assertRaises(ConfigError):
                normalize_config(config, {}, ROOT)

    def test_first_motion_may_use_external_verified_odom_without_widening(self) -> None:
        runtime = normalize_config(
            first_motion_config(odom_source=ODOM_SOURCE_EXTERNAL_VERIFIED),
            {"GO2_ALLOWED_MODES": "0"},
            ROOT,
        )
        self.assertTrue(runtime.allow_motion)
        self.assertEqual(runtime.odom_source, ODOM_SOURCE_EXTERNAL_VERIFIED)
        self.assertEqual(runtime.external_odom_timeout_s, 0.20)
        self.assertEqual(runtime.max_linear_x_mps, 0.05)
        self.assertEqual(runtime.commissioning_max_distance_m, 0.10)

    def test_unknown_odom_source_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigError, "odom_source"):
            normalize_config({"odom_source": "external_unchecked"}, {}, ROOT)

    def test_disabled_fallback_is_encoded_as_an_empty_yaml_string(self) -> None:
        runtime = normalize_config({"state_fallback_topic": ""}, {}, ROOT)
        rendered = "\n".join(
            runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        )
        self.assertIn("state_fallback_topic:=''", rendered)
        self.assertNotIn("state_fallback_topic:=\n", rendered)

    def test_opaque_state_marker_allowlist_is_explicit_and_not_a_mode(self) -> None:
        runtime = normalize_config(
            {}, {"GO2_ALLOWED_STATE_MARKERS": "2010, 100,2010"}, ROOT
        )
        self.assertFalse(runtime.allow_motion)
        self.assertEqual(runtime.allowed_modes, (UNKNOWN_MODE,))
        self.assertEqual(runtime.allowed_state_markers, (2010, 100))
        self.assertEqual(
            runtime.process_env()["GO2_ALLOWED_STATE_MARKERS"], "2010,100"
        )
        rendered = "\n".join(
            runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        )
        self.assertIn("allowed_modes:=[255]", rendered)
        self.assertIn("allowed_state_markers:=[2010,100]", rendered)

    def test_passive_private_manifest_may_deliver_evidence_marker(self) -> None:
        runtime = normalize_config(
            {"allow_motion": False, "allowed_state_markers": [100]}, {}, ROOT
        )
        self.assertFalse(runtime.allow_motion)
        self.assertEqual(runtime.allowed_state_markers, (100,))
        self.assertIn(
            "allowed_state_markers:=[100]",
            "\n".join(runtime.adapter_argv(Path("/tmp/a"), Path("/tmp/p"))),
        )

    def test_passive_external_odom_may_explicitly_allow_reviewed_transitions(
        self,
    ) -> None:
        runtime = normalize_config(
            {
                "allow_motion": False,
                "odom_source": ODOM_SOURCE_EXTERNAL_VERIFIED,
                "allowed_state_markers": [100, 1002],
                "allow_passive_state_marker_transitions": True,
            },
            {"GO2_ALLOWED_STATE_MARKERS": "100,1002"},
            ROOT,
        )
        self.assertFalse(runtime.allow_motion)
        self.assertEqual(runtime.odom_source, ODOM_SOURCE_EXTERNAL_VERIFIED)
        self.assertEqual(runtime.allowed_state_markers, (100, 1002))
        self.assertTrue(runtime.allow_passive_state_marker_transitions)
        rendered = "\n".join(
            runtime.adapter_argv(Path("/tmp/a"), Path("/tmp/p"))
        )
        self.assertIn("allowed_state_markers:=[100,1002]", rendered)
        self.assertIn(
            "allow_passive_state_marker_transitions:=true", rendered
        )

    def test_passive_marker_transition_exception_is_narrow_and_fail_closed(
        self,
    ) -> None:
        invalid = (
            {
                "allowed_state_markers": [100, 1002],
                "allow_passive_state_marker_transitions": True,
            },
            {
                "odom_source": ODOM_SOURCE_EXTERNAL_VERIFIED,
                "allowed_state_markers": [],
                "allow_passive_state_marker_transitions": True,
            },
            {
                "odom_source": ODOM_SOURCE_EXTERNAL_VERIFIED,
                "allowed_state_markers": [100],
                "allow_passive_state_marker_transitions": True,
            },
        )
        for config in invalid:
            with self.subTest(config=config), self.assertRaisesRegex(
                ConfigError, "allow_passive_state_marker_transitions"
            ):
                normalize_config(config, {}, ROOT)

        motion = first_motion_config(
            odom_source=ODOM_SOURCE_EXTERNAL_VERIFIED,
            allow_passive_state_marker_transitions=True,
        )
        with self.assertRaisesRegex(
            ConfigError, "allow_passive_state_marker_transitions"
        ):
            normalize_config(
                motion,
                {
                    "GO2_ALLOWED_MODES": "0",
                    "GO2_ALLOWED_STATE_MARKERS": "100,1002",
                },
                ROOT,
            )

    def test_classic_motion_marker_transition_is_staged_and_exact(self) -> None:
        runtime = normalize_config(
            staged_nav2_config(allow_motion_state_marker_transitions=True),
            {
                "GO2_ALLOWED_MODES": "0",
                "GO2_ALLOWED_STATE_MARKERS": "100,2010",
            },
            ROOT,
        )
        self.assertTrue(runtime.allow_motion_state_marker_transitions)
        self.assertFalse(runtime.allow_passive_state_marker_transitions)
        self.assertEqual(
            frozenset(runtime.allowed_state_markers),
            CLASSIC_MOTION_STATE_MARKERS,
        )
        rendered = "\n".join(
            runtime.adapter_argv(Path("/tmp/a"), Path("/tmp/p"))
        )
        self.assertIn("allowed_modes:=[0]", rendered)
        self.assertIn("allowed_state_markers:=[100,2010]", rendered)
        self.assertIn("allow_motion_state_marker_transitions:=true", rendered)

    def test_classic_motion_marker_transition_rejects_scope_drift(self) -> None:
        invalid = (
            (
                first_motion_config(allow_motion_state_marker_transitions=True),
                {"GO2_ALLOWED_MODES": "0", "GO2_ALLOWED_STATE_MARKERS": "100,2010"},
            ),
            (
                staged_nav2_config(allow_motion_state_marker_transitions=True),
                {"GO2_ALLOWED_MODES": "0", "GO2_ALLOWED_STATE_MARKERS": "100,1002"},
            ),
            (
                staged_nav2_config(allow_motion_state_marker_transitions=True),
                {"GO2_ALLOWED_MODES": "0,1", "GO2_ALLOWED_STATE_MARKERS": "100,2010"},
            ),
            (
                staged_nav2_config(
                    allow_motion_state_marker_transitions=True,
                    allow_passive_state_marker_transitions=True,
                ),
                {"GO2_ALLOWED_MODES": "0", "GO2_ALLOWED_STATE_MARKERS": "100,2010"},
            ),
            (
                staged_nav2_config(
                    odom_source=ODOM_SOURCE_SPORT_STATE,
                    allow_motion_state_marker_transitions=True,
                ),
                {"GO2_ALLOWED_MODES": "0", "GO2_ALLOWED_STATE_MARKERS": "100,2010"},
            ),
        )
        for config, environment in invalid:
            with self.subTest(config=config), self.assertRaises(ConfigError):
                normalize_config(config, environment, ROOT)

    def test_motion_rejects_state_markers_from_driver_config(self) -> None:
        config = first_motion_config(allowed_state_markers=[100])
        with self.assertRaisesRegex(ConfigError, "audited runtime permit"):
            normalize_config(
                config,
                {
                    "GO2_ALLOWED_MODES": "0",
                    "GO2_ALLOWED_STATE_MARKERS": "100",
                },
                ROOT,
            )

    def test_manifest_modes_cannot_enable_motion_without_audit_env(self) -> None:
        dangerous = first_motion_config(allowed_modes=[0, 1, 3])
        with self.assertRaisesRegex(ConfigError, "allowed_modes"):
            normalize_config(dangerous, {}, ROOT)

        dangerous["allowed_modes"] = [UNKNOWN_MODE]
        with self.assertRaisesRegex(ConfigError, "GO2_ALLOWED_MODES"):
            normalize_config(dangerous, {}, ROOT)

    def test_all_motion_gates_and_audited_modes_are_required(self) -> None:
        base = first_motion_config(
            operator_present=False,
            safety_ack="",
        )
        with self.assertRaisesRegex(ConfigError, "operator_present"):
            normalize_config(base, {"GO2_ALLOWED_MODES": "0"}, ROOT)
        base["operator_present"] = True
        with self.assertRaisesRegex(ConfigError, "safety_ack"):
            normalize_config(base, {"GO2_ALLOWED_MODES": "0"}, ROOT)
        base["safety_ack"] = "I_UNDERSTAND_GO2_CAN_MOVE"
        runtime = normalize_config(
            base,
            {
                "GO2_ALLOWED_MODES": "3, 1,3",
                "GO2_ALLOWED_STATE_MARKERS": "2010",
            },
            ROOT,
        )
        self.assertTrue(runtime.allow_motion)
        self.assertTrue(runtime.starts_sdk_daemon)
        self.assertEqual(runtime.allowed_modes, (3, 1))
        self.assertEqual(runtime.allowed_state_markers, (2010,))
        self.assertIn("--allow-motion", runtime.daemon_argv(Path("/tmp/daemon")))
        daemon = runtime.daemon_argv(Path("/tmp/daemon"))
        self.assertEqual(daemon[daemon.index("--max-vx") + 1], "0.05")
        self.assertEqual(daemon[daemon.index("--max-vy") + 1], "0.0")
        self.assertEqual(daemon[daemon.index("--max-wz") + 1], "0.0")
        self.assertEqual(daemon[daemon.index("--max-motion-ms") + 1], "2000")
        self.assertIn("allowed_modes:=[3,1]", "\n".join(
            runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        ))
        self.assertIn("allowed_state_markers:=[2010]", "\n".join(
            runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        ))

    def test_unknown_and_dangerous_config_fails_closed(self) -> None:
        bad_configs = (
            {"max_linear_x_mps": 0.26},
            {"max_linear_y_mps": 0.01},
            {"zero_preamble_s": 0.49},
            {"command_timeout_s": float("nan")},
            {"max_source_stamp_age_s": 0.0},
            {"max_source_stamp_age_s": 0.21},
            {"state_timeout_s": 0.21},
            {"max_source_stamp_future_skew_s": -0.01},
            {"max_source_stamp_future_skew_s": 0.051},
            {"velocity_frame": "map"},
            {"twist_in_topic": "cmd_vel"},
            {"surprise_motion_flag": True},
        )
        for config in bad_configs:
            with self.subTest(config=config), self.assertRaises(ConfigError):
                normalize_config(config, {}, ROOT)

    def test_first_motion_profile_rejects_any_wider_or_different_route(self) -> None:
        environment = {"GO2_ALLOWED_MODES": "0"}
        unsafe = (
            {"motion_profile": "general-navigation"},
            {"twist_in_topic": "/cmd_vel"},
            {"odom_topic": "/other_odom"},
            {"publish_odom_tf": False},
            {"max_linear_x_mps": 0.051},
            {"max_angular_z_rps": 0.01},
            {"commissioning_max_duration_s": 1.99},
            {"commissioning_max_distance_m": 0.09},
            {"command_timeout_s": 0.21},
        )
        for override in unsafe:
            with self.subTest(override=override), self.assertRaises(ConfigError):
                normalize_config(first_motion_config(**override), environment, ROOT)

    def test_invalid_audited_modes_fail_closed(self) -> None:
        config = {
            "allow_motion": True,
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": "enp1s0",
            "ipc_socket": "/tmp/go2-test.sock",
        }
        for value in ("", "255", "-1", "1,,3", "standing"):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                normalize_config(config, {"GO2_ALLOWED_MODES": value}, ROOT)

    def test_invalid_opaque_state_markers_fail_closed(self) -> None:
        for value in (
            "0",
            "-1",
            "1,,3",
            "mode-2010",
            "4294967296",
        ):
            with self.subTest(value=value), self.assertRaisesRegex(
                ConfigError, "GO2_ALLOWED_STATE_MARKERS"
            ):
                normalize_config({}, {"GO2_ALLOWED_STATE_MARKERS": value}, ROOT)


if __name__ == "__main__":
    unittest.main()
