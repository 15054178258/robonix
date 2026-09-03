from __future__ import annotations

import ast
import importlib.util
import math
from pathlib import Path
import time
import types
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_module():
    path = ROOT / "scripts" / "icp_odometry_nomotion_eval.py"
    spec = importlib.util.spec_from_file_location("icp_nomotion_eval", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


evaluation = load_module()


def manifest() -> dict:
    return {
        "primitive": [
            {
                "name": "go2_chassis",
                "config": {
                    "allow_motion": False,
                    "operator_present": False,
                    "twist_in_topic": "/robonix/nomotion/chassis_input_disabled",
                },
            },
            {
                "name": "go2_sensors",
                "config": {
                    "lidar_output_topic": "/scanner/cloud",
                    "imu_output_topic": "/scanner/imu",
                },
            },
        ]
    }


def evidence(now_ns: int, duration_seconds: int = 600):
    gids = {"mid360_cloud": "aa", "mid360_imu": "bb"}
    approval = {
        "schema": "robonix-go2-workstation-nomotion-stamp-offset-v3",
        "motion_enabled": False,
        "identity_evidence_verified": True,
        "not_before_unix_ns": now_ns - 1,
        "expires_unix_ns": now_ns + (duration_seconds + 91) * 1_000_000_000,
        "session_id": "session-a",
        "writer_gids": gids,
    }
    ready = {
        "time_discipline_ready": True,
        "motion_ready": False,
        "canonical_odom_ready": False,
        "session_id": "session-a",
    }
    identity = {
        "identity_bound": True,
        "motion_ready": False,
        "canonical_odom_ready": False,
        "session_id": "session-a",
        "writer_gids": gids,
    }
    return approval, ready, identity


class IcpNomotionContractTest(unittest.TestCase):
    def test_config_is_private_three_dof_and_never_publishes_tf(self) -> None:
        contract = evaluation.static_config_contract()
        self.assertEqual(contract["node_fqn"], "/robonix/nomotion/icp_eval/icp_odometry")
        self.assertEqual(contract["odom_topic"], "/robonix/nomotion/icp_eval/odom")
        self.assertEqual(contract["odom_frame"], "robonix_nomotion_icp_odom")
        self.assertFalse(contract["publish_tf"])
        payload = yaml.safe_load((ROOT / "config" / "icp_odometry_nomotion_eval.yaml").read_text())
        params = payload[contract["node_fqn"]]["ros__parameters"]
        self.assertEqual(params["Reg/Force3DoF"], "true")
        self.assertEqual(params["Icp/MaxCorrespondenceDistance"], "0.20")
        self.assertEqual(params["Odom/GuessMotion"], "false")
        self.assertEqual(params["Odom/ResetCountdown"], "1")
        self.assertTrue(params["wait_imu_to_init"])

    def test_command_has_only_private_outputs_and_exact_sensor_inputs(self) -> None:
        command = evaluation.build_icp_command(Path("/opt/ros/humble/lib/rtabmap_odom/icp_odometry"))
        joined = " ".join(command)
        self.assertIn("scan_cloud:=/scanner/cloud", joined)
        self.assertIn("imu:=/scanner/imu", joined)
        self.assertIn("odom:=/robonix/nomotion/icp_eval/odom", joined)
        self.assertIn("/tf:=/robonix/nomotion/icp_eval/tf_disabled", command)
        self.assertNotIn("tf:=/robonix/nomotion/icp_eval/tf_disabled", command)
        self.assertNotIn("/tf:=/tf", command)
        self.assertIn("/tf_static:=/tf_static", command)
        self.assertIn("--disable-rosout-logs", command)
        self.assertNotIn("odom:=/odom", command)
        for topic in evaluation.FORBIDDEN_MOTION_TOPICS:
            self.assertNotIn(f":={topic}", joined)

    def test_cloud_only_profile_is_an_exact_private_imu_ablation(self) -> None:
        contract = evaluation.static_config_contract(
            evaluation.CLOUD_ONLY_CONFIG_PATH,
            expected_wait_imu=False,
        )
        self.assertFalse(contract["wait_imu_to_init"])
        command = evaluation.build_icp_command(
            Path("/opt/ros/humble/lib/rtabmap_odom/icp_odometry"),
            evaluation.CLOUD_ONLY_CONFIG_PATH,
            evaluation.DISABLED_IMU_TOPIC,
        )
        joined = " ".join(command)
        self.assertIn(f"imu:={evaluation.DISABLED_IMU_TOPIC}", joined)
        self.assertNotIn("imu:=/scanner/imu", joined)
        self.assertIn("scan_cloud:=/scanner/cloud", joined)
        self.assertIn("/tf_static:=/tf_static", command)

        base = yaml.safe_load(
            (ROOT / "config" / "icp_odometry_nomotion_eval.yaml").read_text()
        )[evaluation.ICP_NODE_FQN]["ros__parameters"]
        cloud_only = yaml.safe_load(
            evaluation.CLOUD_ONLY_CONFIG_PATH.read_text()
        )[evaluation.ICP_NODE_FQN]["ros__parameters"]
        differing = {
            key
            for key in set(base) | set(cloud_only)
            if base.get(key) != cloud_only.get(key)
        }
        self.assertEqual(
            differing,
            {"wait_imu_to_init", "always_check_imu_tf"},
        )

    def test_point_to_point_profile_changes_only_the_icp_error_model(self) -> None:
        contract = evaluation.static_config_contract(
            evaluation.POINT_TO_POINT_CONFIG_PATH,
            expected_wait_imu=True,
        )
        self.assertTrue(contract["wait_imu_to_init"])
        command = evaluation.build_icp_command(
            Path("/opt/ros/humble/lib/rtabmap_odom/icp_odometry"),
            evaluation.POINT_TO_POINT_CONFIG_PATH,
            evaluation.SOURCE_IMU_TOPIC,
        )
        joined = " ".join(command)
        self.assertIn("imu:=/scanner/imu", joined)
        self.assertIn("scan_cloud:=/scanner/cloud", joined)
        self.assertNotIn("odom:=/odom", joined)

        base = yaml.safe_load(
            (ROOT / "config" / "icp_odometry_nomotion_eval.yaml").read_text()
        )[evaluation.ICP_NODE_FQN]["ros__parameters"]
        point_to_point = yaml.safe_load(
            evaluation.POINT_TO_POINT_CONFIG_PATH.read_text()
        )[evaluation.ICP_NODE_FQN]["ros__parameters"]
        differing = {
            key
            for key in set(base) | set(point_to_point)
            if base.get(key) != point_to_point.get(key)
        }
        self.assertEqual(differing, {"Icp/PointToPlane"})
        self.assertEqual(point_to_point["Icp/PointToPlane"], "false")

    def test_point_to_point_fine_profile_changes_only_voxel_size(self) -> None:
        contract = evaluation.static_config_contract(
            evaluation.POINT_TO_POINT_FINE_CONFIG_PATH,
            expected_wait_imu=True,
        )
        self.assertTrue(contract["wait_imu_to_init"])
        point_to_point = yaml.safe_load(
            evaluation.POINT_TO_POINT_CONFIG_PATH.read_text()
        )[evaluation.ICP_NODE_FQN]["ros__parameters"]
        fine = yaml.safe_load(
            evaluation.POINT_TO_POINT_FINE_CONFIG_PATH.read_text()
        )[evaluation.ICP_NODE_FQN]["ros__parameters"]
        differing = {
            key
            for key in set(point_to_point) | set(fine)
            if point_to_point.get(key) != fine.get(key)
        }
        self.assertEqual(differing, {"Icp/VoxelSize"})
        self.assertEqual(fine["Icp/VoxelSize"], "0.05")

    def test_point_to_point_keyframe_profile_changes_only_keyframe_threshold(self) -> None:
        contract = evaluation.static_config_contract(
            evaluation.POINT_TO_POINT_KEYFRAME_CONFIG_PATH,
            expected_wait_imu=True,
        )
        self.assertTrue(contract["wait_imu_to_init"])
        point_to_point = yaml.safe_load(
            evaluation.POINT_TO_POINT_CONFIG_PATH.read_text()
        )[evaluation.ICP_NODE_FQN]["ros__parameters"]
        keyframe = yaml.safe_load(
            evaluation.POINT_TO_POINT_KEYFRAME_CONFIG_PATH.read_text()
        )[evaluation.ICP_NODE_FQN]["ros__parameters"]
        differing = {
            key
            for key in set(point_to_point) | set(keyframe)
            if point_to_point.get(key) != keyframe.get(key)
        }
        self.assertEqual(differing, {"Odom/ScanKeyFrameThr"})
        self.assertEqual(keyframe["Odom/ScanKeyFrameThr"], "0.8")

    def test_approval_must_cover_duration_plus_margin(self) -> None:
        now_ns = time.time_ns()
        approval, ready, identity = evidence(now_ns)
        result = evaluation.validate_evidence_payloads(
            approval, ready, identity, manifest(), now_ns=now_ns, duration_seconds=600
        )
        self.assertEqual(result["session_id"], "session-a")
        approval["expires_unix_ns"] = now_ns + 689 * 1_000_000_000
        with self.assertRaisesRegex(evaluation.GateError, "does not cover"):
            evaluation.validate_evidence_payloads(
                approval, ready, identity, manifest(), now_ns=now_ns, duration_seconds=600
            )

    def test_gate_rejects_motion_manifest_and_identity_mismatch(self) -> None:
        now_ns = time.time_ns()
        approval, ready, identity = evidence(now_ns)
        unsafe = manifest()
        unsafe["primitive"][0]["config"]["allow_motion"] = True
        with self.assertRaisesRegex(evaluation.GateError, "enables chassis motion"):
            evaluation.validate_evidence_payloads(
                approval, ready, identity, unsafe, now_ns=now_ns, duration_seconds=600
            )
        identity["writer_gids"] = {"mid360_cloud": "changed"}
        with self.assertRaisesRegex(evaluation.GateError, "writer GIDs differ"):
            evaluation.validate_evidence_payloads(
                approval, ready, identity, manifest(), now_ns=now_ns, duration_seconds=600
            )

    def test_pose_summary_unwraps_yaw_and_computes_drift(self) -> None:
        series = evaluation.PoseSeries("private", "base_link")
        start = 1_000_000_000
        for index, yaw in enumerate((3.13, -3.13, -3.12, -3.11)):
            half = yaw / 2.0
            message = types.SimpleNamespace(
                header=types.SimpleNamespace(
                    frame_id="private",
                    stamp=types.SimpleNamespace(sec=index + 1, nanosec=0),
                ),
                child_frame_id="base_link",
                pose=types.SimpleNamespace(
                    pose=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=index * 0.001, y=0.0, z=0.0),
                        orientation=types.SimpleNamespace(
                            x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)
                        ),
                    )
                ),
            )
            series.observe(message, start + index * 1_000_000_000)
        summary = series.summary(0)
        self.assertTrue(summary["analyzed"])
        self.assertLess(abs(summary["yaw_drift_deg"]), 3.0)
        self.assertAlmostEqual(summary["translation_drift_m"], 0.003)

    def test_pose_summary_separates_valid_freshness_from_null_messages(self) -> None:
        series = evaluation.PoseSeries("private", "base_link")

        def pose(x: float, quaternion_w: float):
            return types.SimpleNamespace(
                header=types.SimpleNamespace(
                    frame_id="private",
                    stamp=types.SimpleNamespace(sec=1, nanosec=0),
                ),
                child_frame_id="base_link",
                pose=types.SimpleNamespace(
                    pose=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=x, y=0.0, z=0.0),
                        orientation=types.SimpleNamespace(
                            x=0.0, y=0.0, z=0.0, w=quaternion_w
                        ),
                    )
                ),
            )

        series.observe(pose(0.0, 1.0), 1_000_000_000)
        series.observe(pose(0.0, 0.0), 2_000_000_000)
        series.observe(pose(0.0, 0.0), 3_000_000_000)
        series.observe(pose(0.001, 1.0), 4_000_000_000)
        summary = series.summary(0, 5_000_000_000)
        self.assertEqual(summary["received_messages"], 4)
        self.assertEqual(summary["valid_messages"], 2)
        self.assertEqual(summary["invalid_messages"], 2)
        self.assertEqual(summary["invalid_reasons"]["invalid_quaternion_norm"], 2)
        self.assertEqual(summary["maximum_valid_gap_ns"], 3_000_000_000)
        self.assertEqual(summary["last_valid_age_ns"], 1_000_000_000)
        self.assertEqual(summary["longest_invalid_run"], 2)
        self.assertEqual(summary["trailing_invalid_run"], 0)
        self.assertEqual(summary["recovery_count"], 1)

    def test_odom_info_summary_tracks_loss_recovery_and_guess(self) -> None:
        series = evaluation.OdomInfoSeries()

        def info(lost: bool, guess_x: float):
            return types.SimpleNamespace(
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=1, nanosec=0)
                ),
                lost=lost,
                icp_inliers_ratio=0.0 if lost else 0.5,
                icp_translation=0.1,
                icp_rotation=0.2,
                icp_correspondences=10,
                local_scan_map_size=100,
                guess=types.SimpleNamespace(
                    translation=types.SimpleNamespace(
                        x=guess_x, y=0.0, z=0.0
                    )
                ),
            )

        for index, lost in enumerate((False, True, True, False)):
            series.observe(info(lost, float(index)), (index + 1) * 1_000_000_000)
        summary = series.summary(5_000_000_000)
        self.assertEqual(summary["lost_messages"], 2)
        self.assertEqual(summary["longest_lost_run"], 2)
        self.assertEqual(summary["trailing_lost_run"], 0)
        self.assertEqual(summary["recovery_count"], 1)
        self.assertEqual(summary["guess_translation_max_m"], 3.0)

    def test_private_disabled_tf_must_have_endpoint_but_zero_messages(self) -> None:
        arguments = evaluation.build_argument_parser().parse_args(
            [
                "--session-dir", "session",
                "--approval", "approval.json",
                "--duration-seconds", "60",
                "--warmup-seconds", "10",
                "--ack", evaluation.ACK_TOKEN,
            ]
        )
        summary = {
            "private_tf_disabled_messages": 1,
            "streams": {"private_icp": {}},
        }
        gates = {
            gate["name"]: gate for gate in evaluation.evaluate_gates(summary, arguments)
        }
        self.assertFalse(gates["private_tf_disabled_messages_zero"]["passed"])
        self.assertEqual(
            gates["private_tf_disabled_messages_zero"]["observed"], 1
        )

    def test_hidden_tf_listener_is_matched_by_dds_participant_prefix(self) -> None:
        prefix = bytes.fromhex("00112233445566778899aabb")
        odom = types.SimpleNamespace(
            node_name="icp_odometry",
            node_namespace="/robonix/nomotion/icp_eval",
            topic_type="nav_msgs/msg/Odometry",
            endpoint_gid=prefix + bytes.fromhex("000001c10000000000000000"),
        )
        hidden_listener = types.SimpleNamespace(
            node_name="transform_listener_impl_1234",
            node_namespace="/robonix/nomotion/icp_eval",
            topic_type="tf2_msgs/msg/TFMessage",
            endpoint_gid=prefix + bytes.fromhex("000002c70000000000000000"),
        )
        other = types.SimpleNamespace(
            node_name="other",
            node_namespace="/",
            topic_type="tf2_msgs/msg/TFMessage",
            endpoint_gid=bytes.fromhex("ffeeddccbbaa998877665544000002c70000000000000000"),
        )
        participant = evaluation.endpoint_participant_prefix(odom)
        self.assertTrue(evaluation.same_participant(hidden_listener, participant))
        self.assertFalse(evaluation.same_participant(other, participant))
        self.assertEqual(
            evaluation.endpoint_record(hidden_listener)["participant_prefix"],
            prefix.hex(),
        )

    def test_source_has_one_producer_and_observer_is_subscription_only(self) -> None:
        source = (ROOT / "scripts" / "icp_odometry_nomotion_eval.py").read_text()
        tree = ast.parse(source)
        calls = [
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        ]
        self.assertEqual(calls.count("Popen"), 1)
        self.assertEqual(calls.count("create_subscription"), 4)
        for forbidden in (
            "create_publisher",
            "create_client",
            "create_service",
            "publish",
            "call_async",
            "send_goal_async",
        ):
            self.assertNotIn(forbidden, calls)
        self.assertIn("publish_tf: false", (ROOT / "config" / "icp_odometry_nomotion_eval.yaml").read_text())
        self.assertIn("private_tf_disabled_messages_zero", source)
        self.assertIn("canonical_odom_icp_publishers", source)
        self.assertIn("tf_icp_publishers", source)

    def test_cli_duration_is_bounded_and_ack_is_explicit(self) -> None:
        parser = evaluation.build_argument_parser()
        args = parser.parse_args(
            [
                "--session-dir", "session",
                "--approval", "approval.json",
                "--ack", evaluation.ACK_TOKEN,
            ]
        )
        self.assertEqual(args.duration_seconds, 600)
        with self.assertRaises(SystemExit):
            parser.parse_args(
                [
                    "--session-dir", "session",
                    "--approval", "approval.json",
                    "--duration-seconds", "901",
                    "--ack", evaluation.ACK_TOKEN,
                ]
            )
        with self.assertRaises(SystemExit):
            parser.parse_args(
                [
                    "--session-dir", "session",
                    "--approval", "approval.json",
                    "--max-yaw-drift-deg", "2.1",
                    "--ack", evaluation.ACK_TOKEN,
                ]
            )


if __name__ == "__main__":
    unittest.main()
