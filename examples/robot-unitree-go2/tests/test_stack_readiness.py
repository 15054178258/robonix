from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "stack_readiness.py"
SPEC = importlib.util.spec_from_file_location("stack_readiness", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
gate = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = gate
SPEC.loader.exec_module(gate)


def provider(provider_id: str, caps: set[str], state: str = "ACTIVE") -> dict:
    return {
        "provider_id": provider_id,
        "namespace": "robonix/test",
        "state": state,
        "state_detail": "",
        "capabilities": [
            {
                "contract_id": contract_id,
                "transport": "grpc",
                "namespace_mismatch": False,
            }
            for contract_id in sorted(caps)
        ],
    }


def required_provider_payload() -> list[dict]:
    payload = [
        provider(provider_id, set(capabilities))
        for provider_id, capabilities in gate.REQUIRED_CAPABILITIES.items()
    ]
    go2_sensors = next(
        record for record in payload if record["provider_id"] == "go2_sensors"
    )
    go2_sensors["namespace"] = "robonix/primitive/lidar"
    for capability in go2_sensors["capabilities"]:
        if capability["contract_id"] in {
            "robonix/primitive/imu/imu",
            "robonix/primitive/camera/rgb",
        }:
            capability["transport"] = "ros2"
            capability["namespace_mismatch"] = True
    return payload


class ReadinessUnitTests(unittest.TestCase):
    def test_all_required_active_providers_pass(self) -> None:
        payload = required_provider_payload()
        result = gate.validate_providers(payload)
        self.assertEqual(result.status, gate.PASS)
        expected = frozenset(
            {
                (
                    "go2_sensors",
                    "robonix/primitive/lidar",
                    "robonix/primitive/imu/imu",
                    "ros2",
                ),
                (
                    "go2_sensors",
                    "robonix/primitive/lidar",
                    "robonix/primitive/camera/rgb",
                    "ros2",
                ),
            }
        )
        self.assertEqual(gate.ALLOWED_NAMESPACE_DIAGNOSTICS, expected)
        accepted = {
            (
                item["provider_id"],
                item["runtime_namespace"],
                item["contract_id"],
                item["transport"],
            )
            for item in result.evidence["accepted_namespace_diagnostics"]
        }
        self.assertEqual(accepted, expected)

    def test_missing_capability_and_error_provider_fail(self) -> None:
        payload = required_provider_payload()
        payload[0]["state"] = "ERROR"
        payload[1]["capabilities"] = []
        result = gate.validate_providers(payload)
        self.assertEqual(result.status, gate.FAIL)
        self.assertTrue(result.evidence["problems"])
        self.assertEqual(
            len(result.evidence["accepted_namespace_diagnostics"]), 2
        )

    def test_namespace_diagnostic_allowlist_is_exact_and_fail_closed(self) -> None:
        def assert_unexpected(payload: list[dict]) -> None:
            result = gate.validate_providers(payload)
            self.assertEqual(result.status, gate.FAIL)
            self.assertTrue(
                any(
                    "unexpected_namespace_diagnostics" in problem
                    for problem in result.evidence["problems"].values()
                    if isinstance(problem, dict)
                )
            )

        with self.subTest(label="wrong_provider"):
            payload = required_provider_payload()
            scene = next(
                record for record in payload if record["provider_id"] == "scene"
            )
            scene["namespace"] = "robonix/primitive/lidar"
            scene["capabilities"].append(
                {
                    "contract_id": "robonix/primitive/camera/rgb",
                    "transport": "ros2",
                    "namespace_mismatch": True,
                }
            )
            assert_unexpected(payload)

        with self.subTest(label="wrong_namespace"):
            payload = required_provider_payload()
            go2_sensors = next(
                record
                for record in payload
                if record["provider_id"] == "go2_sensors"
            )
            go2_sensors["namespace"] = "robonix/primitive"
            assert_unexpected(payload)

        with self.subTest(label="wrong_transport"):
            payload = required_provider_payload()
            camera = next(
                capability
                for record in payload
                if record["provider_id"] == "go2_sensors"
                for capability in record["capabilities"]
                if capability["contract_id"] == "robonix/primitive/camera/rgb"
            )
            camera["transport"] = "grpc"
            assert_unexpected(payload)

        with self.subTest(label="extra_contract"):
            payload = required_provider_payload()
            lidar = next(
                capability
                for record in payload
                if record["provider_id"] == "go2_sensors"
                for capability in record["capabilities"]
                if capability["contract_id"]
                == "robonix/primitive/lidar/lidar3d"
            )
            lidar.update({"transport": "ros2", "namespace_mismatch": True})
            assert_unexpected(payload)

    def test_go2_sensors_runtime_namespace_cannot_bypass_atlas_diagnostic(self) -> None:
        payload = required_provider_payload()
        go2_sensors = next(
            record for record in payload if record["provider_id"] == "go2_sensors"
        )
        go2_sensors["namespace"] = "robonix/primitive"
        for capability in go2_sensors["capabilities"]:
            capability["namespace_mismatch"] = False
        result = gate.validate_providers(payload)
        self.assertEqual(result.status, gate.FAIL)
        self.assertEqual(
            result.evidence["problems"]["go2_sensors"]["provider_namespace"],
            {
                "expected": "robonix/primitive/lidar",
                "actual": "robonix/primitive",
            },
        )

    def test_namespace_mismatch_flag_must_be_an_exact_boolean(self) -> None:
        for label, mutation in (
            ("missing", lambda item: item.pop("namespace_mismatch")),
            ("string", lambda item: item.__setitem__("namespace_mismatch", "true")),
            ("integer", lambda item: item.__setitem__("namespace_mismatch", 1)),
        ):
            with self.subTest(label=label):
                payload = required_provider_payload()
                camera = next(
                    capability
                    for record in payload
                    if record["provider_id"] == "go2_sensors"
                    for capability in record["capabilities"]
                    if capability["contract_id"]
                    == "robonix/primitive/camera/rgb"
                )
                mutation(camera)
                result = gate.validate_providers(payload)
                self.assertEqual(result.status, gate.FAIL)
                self.assertIn(
                    "invalid_namespace_diagnostic_flags",
                    result.evidence["problems"]["go2_sensors"],
                )

    def test_exact_namespace_diagnostics_are_required_once_each(self) -> None:
        with self.subTest(label="missing_true_diagnostic"):
            payload = required_provider_payload()
            camera = next(
                capability
                for record in payload
                if record["provider_id"] == "go2_sensors"
                for capability in record["capabilities"]
                if capability["contract_id"]
                == "robonix/primitive/camera/rgb"
            )
            camera["namespace_mismatch"] = False
            result = gate.validate_providers(payload)
            self.assertEqual(result.status, gate.FAIL)
            exact = result.evidence["problems"]["go2_sensors"][
                "namespace_diagnostics_exact_set"
            ]
            self.assertEqual(len(exact["expected"]), 2)
            self.assertEqual(len(exact["actual"]), 1)

        with self.subTest(label="duplicate_true_diagnostic"):
            payload = required_provider_payload()
            go2_sensors = next(
                record
                for record in payload
                if record["provider_id"] == "go2_sensors"
            )
            camera = next(
                capability
                for capability in go2_sensors["capabilities"]
                if capability["contract_id"]
                == "robonix/primitive/camera/rgb"
            )
            go2_sensors["capabilities"].append(dict(camera))
            result = gate.validate_providers(payload)
            self.assertEqual(result.status, gate.FAIL)
            exact = result.evidence["problems"]["go2_sensors"][
                "namespace_diagnostics_exact_set"
            ]
            self.assertEqual(len(exact["expected"]), 2)
            self.assertEqual(len(exact["actual"]), 3)

    def test_topic_header_requires_exact_fresh_timestamp_and_frame(self) -> None:
        requirement = gate.TOPICS[0]
        now = time.time()
        sec = int(now)
        header = {
            "stamp": {"sec": sec, "nanosec": int((now - sec) * 1_000_000_000)},
            "frame_id": "front_camera",
        }
        self.assertEqual(
            gate.validate_topic_header(requirement, header, now).status, gate.PASS
        )
        header["frame_id"] = "wrong"
        self.assertEqual(
            gate.validate_topic_header(requirement, header, now).status, gate.FAIL
        )
        header["frame_id"] = "front_camera"
        header["stamp"] = {"sec": int(now) - 20, "nanosec": 0}
        self.assertEqual(
            gate.validate_topic_header(requirement, header, now).status, gate.FAIL
        )

    def test_chassis_health_requires_safe_marker_policy_and_only_tighten_limits(
        self,
    ) -> None:
        document = {
            "status": [
                {
                    "name": "go2_chassis_adapter",
                    # Passive/read-only chassis intentionally reports WARN.
                    "level": 1,
                    "values": [
                        {"key": "sport_error_code", "value": "0"},
                        {
                            "key": "opaque_state_marker_explicitly_allowed",
                            "value": "false",
                        },
                        {
                            "key": "opaque_state_marker_change_latched",
                            "value": "false",
                        },
                        {"key": "opaque_state_marker_bound", "value": "none"},
                        {"key": "state_valid", "value": "true"},
                        {"key": "source_stamp_status", "value": "fresh"},
                        {"key": "state_age_sec", "value": "0.01"},
                        {"key": "state_timeout_sec", "value": "0.20"},
                        {"key": "max_source_stamp_age_sec", "value": "0.20"},
                        {
                            "key": "max_source_stamp_future_skew_sec",
                            "value": "0.05",
                        },
                    ],
                }
            ]
        }
        self.assertEqual(
            gate.validate_chassis_health_document(document).status, gate.PASS
        )
        for serialized_level in (chr(0), chr(1)):
            with self.subTest(serialized_level=ord(serialized_level)):
                character_level = json.loads(json.dumps(document))
                character_level["status"][0]["level"] = serialized_level
                self.assertEqual(
                    gate.validate_chassis_health_document(character_level).status,
                    gate.PASS,
                )
        error_level = json.loads(json.dumps(document))
        error_level["status"][0]["level"] = 2
        self.assertEqual(
            gate.validate_chassis_health_document(error_level).status,
            gate.FAIL,
        )
        character_error_level = json.loads(json.dumps(document))
        character_error_level["status"][0]["level"] = chr(2)
        self.assertEqual(
            gate.validate_chassis_health_document(character_error_level).status,
            gate.FAIL,
        )
        malformed_level = json.loads(json.dumps(document))
        malformed_level["status"][0]["level"] = "not-a-byte"
        self.assertEqual(
            gate.validate_chassis_health_document(malformed_level).status,
            gate.UNKNOWN,
        )

        for key, unsafe_value in (
            ("state_timeout_sec", "0.50"),
            ("max_source_stamp_age_sec", "0.50"),
            ("max_source_stamp_future_skew_sec", "0.10"),
        ):
            with self.subTest(key=key):
                changed = json.loads(json.dumps(document))
                values = changed["status"][0]["values"]
                next(item for item in values if item["key"] == key)["value"] = unsafe_value
                self.assertEqual(
                    gate.validate_chassis_health_document(changed).status,
                    gate.FAIL,
                )

    def test_diagnostic_uint8_decoder_is_strict(self) -> None:
        self.assertEqual(gate._diagnostic_uint8(chr(0), "level"), 0)
        self.assertEqual(gate._diagnostic_uint8(chr(1), "level"), 1)
        self.assertEqual(gate._diagnostic_uint8(255, "level"), 255)
        for malformed in (True, -1, 256, "", "warn", chr(256)):
            with self.subTest(malformed=repr(malformed)):
                with self.assertRaises(ValueError):
                    gate._diagnostic_uint8(malformed, "level")
        # Decimal text remains valid for DiagnosticStatus key/value strings,
        # but is not reinterpreted as a serialized uint8 character.
        self.assertEqual(gate._diagnostic_integer("2010", "marker"), 2010)
        self.assertEqual(gate._diagnostic_uint8("1", "level"), ord("1"))

    def test_chassis_health_accepts_only_exact_explicit_nonzero_marker(self) -> None:
        document = {
            "status": [
                {
                    "name": "go2_chassis_adapter",
                    "level": 1,
                    "values": [
                        {"key": "sport_error_code", "value": "2010"},
                        {
                            "key": "opaque_state_marker_explicitly_allowed",
                            "value": "true",
                        },
                        {
                            "key": "opaque_state_marker_change_latched",
                            "value": "false",
                        },
                        {
                            "key": "opaque_state_marker_bound",
                            "value": "2010",
                        },
                        {"key": "state_valid", "value": "true"},
                        {"key": "source_stamp_status", "value": "fresh"},
                        {"key": "state_age_sec", "value": "0.01"},
                        {"key": "state_timeout_sec", "value": "0.20"},
                        {
                            "key": "max_source_stamp_age_sec",
                            "value": "0.20",
                        },
                        {
                            "key": "max_source_stamp_future_skew_sec",
                            "value": "0.05",
                        },
                    ],
                }
            ]
        }
        result = gate.validate_chassis_health_document(document)
        self.assertEqual(result.status, gate.PASS)
        self.assertEqual(result.evidence["opaque_state_marker_bound"], "2010")

        for key, unsafe_value in (
            ("opaque_state_marker_explicitly_allowed", "false"),
            ("opaque_state_marker_change_latched", "true"),
            ("opaque_state_marker_bound", "100"),
            ("opaque_state_marker_bound", "none"),
        ):
            with self.subTest(key=key, unsafe_value=unsafe_value):
                changed = json.loads(json.dumps(document))
                values = changed["status"][0]["values"]
                next(item for item in values if item["key"] == key)["value"] = (
                    unsafe_value
                )
                self.assertEqual(
                    gate.validate_chassis_health_document(changed).status,
                    gate.FAIL,
                )

        missing_proof = json.loads(json.dumps(document))
        missing_proof["status"][0]["values"] = [
            entry
            for entry in missing_proof["status"][0]["values"]
            if not entry["key"].startswith("opaque_state_marker_")
        ]
        self.assertEqual(
            gate.validate_chassis_health_document(missing_proof).status,
            gate.FAIL,
        )

    def test_chassis_health_read_is_bounded_and_filtered(self) -> None:
        runner = gate.CommandRunner(1.0)
        result = gate.CommandResult((), 124, "", "", True)
        with mock.patch.object(runner, "ros", return_value=result) as ros:
            self.assertEqual(gate.read_chassis_health(runner).status, gate.UNKNOWN)
        argv = ros.call_args.args[0]
        self.assertIn("--once", argv)
        self.assertIn("--filter", argv)
        self.assertNotIn("pub", argv)

    def test_readiness_requires_the_projected_scan_consumed_by_nav2(self) -> None:
        scan = next(item for item in gate.TOPICS if item.label == "scan")
        self.assertEqual(scan.topic, "/scanner/scan")
        self.assertEqual(scan.message_type, "sensor_msgs/msg/LaserScan")
        self.assertEqual(scan.expected_frame, "base_link")
        self.assertLessEqual(scan.max_age_s, 1.0)

    def test_mid360_imu_frame_matches_the_relay_and_static_tf(self) -> None:
        imu = next(item for item in gate.TOPICS if item.label == "imu")
        self.assertEqual(imu.topic, "/scanner/imu")
        self.assertEqual(imu.expected_frame, "utlidar_imu")
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn(
            '("base_to_imu", "base_link", "utlidar_imu", False)', source
        )

    def test_event_driven_map_snapshot_has_no_stream_age_ceiling(self) -> None:
        map_requirement = next(item for item in gate.TOPICS if item.label == "map")
        self.assertTrue(map_requirement.transient_local)
        self.assertIsNone(map_requirement.max_age_s)
        now = time.time()
        old_snapshot = {
            "stamp": {"sec": int(now) - 3600, "nanosec": 0},
            "frame_id": "map",
        }
        result = gate.validate_topic_header(map_requirement, old_snapshot, now)
        self.assertEqual(result.status, gate.PASS)
        self.assertIn("event-driven snapshot", result.detail)
        future_snapshot = {
            "stamp": {"sec": int(now) + 2, "nanosec": 0},
            "frame_id": "map",
        }
        self.assertEqual(
            gate.validate_topic_header(map_requirement, future_snapshot, now).status,
            gate.FAIL,
        )

    def test_map_lifecycle_requires_exact_tuple(self) -> None:
        expected = {"map_id": "lab_go2", "mode": "localization", "generation": 8}
        self.assertEqual(
            gate.validate_map_lifecycle_payload(dict(expected), expected).status,
            gate.PASS,
        )
        actual = dict(expected)
        actual["generation"] = 9
        self.assertEqual(
            gate.validate_map_lifecycle_payload(actual, expected).status,
            gate.FAIL,
        )
        actual["generation"] = True
        self.assertEqual(
            gate.validate_map_lifecycle_payload(actual, expected).status,
            gate.UNKNOWN,
        )

    def test_landmark_binding_must_be_verified_and_measured(self) -> None:
        document = {
            "schema_version": 2,
            "map_id": "lab_go2",
            "map_generation": 8,
            "frame_id": "map",
            "landmarks": [
                {
                    "id": "vending_machine_front",
                    "verified": True,
                    "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "landmarks.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            check, binding = gate.load_landmark_binding(path)
            self.assertEqual(check.status, gate.PASS)
            self.assertEqual(binding["generation"], 8)
            document["landmarks"][0]["verified"] = False
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            check, binding = gate.load_landmark_binding(path)
            self.assertEqual(check.status, gate.FAIL)
            self.assertIsNone(binding)

    def test_landmark_binding_accepts_any_verified_navigation_destination(self) -> None:
        document = {
            "schema_version": 2,
            "map_id": "lab_go2",
            "map_generation": 8,
            "frame_id": "map",
            "landmarks": [
                {
                    "id": "initial_reference",
                    "name": "机器人初始位置",
                    "kind": "marker",
                    "verified": True,
                    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                },
                {
                    "id": "east_meeting_room",
                    "name": "东侧会议室",
                    "kind": "navigation",
                    "verified": True,
                    "arrival_radius": 0.5,
                    "pose": {"x": 3.0, "y": 1.0, "yaw": 1.57},
                },
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "landmarks.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            check, binding = gate.load_landmark_binding(path)
            self.assertEqual(check.status, gate.PASS)
            self.assertEqual(binding["map_id"], "lab_go2")
            self.assertIn(
                "east_meeting_room",
                check.evidence["verified_navigation_landmarks"],
            )
            self.assertNotIn(
                "initial_reference",
                check.evidence["verified_navigation_landmarks"],
            )

    def test_action_requires_one_server_and_exact_type(self) -> None:
        output = """Action: /navigate_to_pose
Action clients: 1
    /nav_client [nav2_msgs/action/NavigateToPose]
Action servers: 1
    /bt_navigator [nav2_msgs/action/NavigateToPose]
"""
        result = gate.CommandResult((), 0, output, "")
        self.assertEqual(
            gate.validate_action_output(
                "/navigate_to_pose", "nav2_msgs/action/NavigateToPose", result
            ).status,
            gate.PASS,
        )
        output = output.replace("Action servers: 1", "Action servers: 2")
        result = gate.CommandResult((), 0, output, "")
        self.assertEqual(
            gate.validate_action_output(
                "/navigate_to_pose", "nav2_msgs/action/NavigateToPose", result
            ).status,
            gate.FAIL,
        )

    def test_dynamic_tf_requires_fresh_time(self) -> None:
        now = time.time()
        output = f"At time {now - 0.1:.6f}\n- Translation: [0, 0, 0]\n- Rotation: in Quaternion [0, 0, 0, 1]\n"
        result = gate.CommandResult((), 124, output, "", True)
        self.assertEqual(
            gate.validate_tf_output(
                "map_to_odom", result, dynamic=True, now_s=now
            ).status,
            gate.PASS,
        )
        stale = output.replace(f"{now - 0.1:.6f}", f"{now - 20:.6f}")
        result = gate.CommandResult((), 124, stale, "", True)
        self.assertEqual(
            gate.validate_tf_output(
                "map_to_odom", result, dynamic=True, now_s=now
            ).status,
            gate.FAIL,
        )

    def test_ros_runner_always_prefixes_external_timeout(self) -> None:
        runner = gate.CommandRunner(4.2)
        with mock.patch.object(runner, "run") as run:
            run.return_value = gate.CommandResult((), 0, "", "")
            runner.ros(("ros2", "topic", "echo", "/safe", "--once"))
        argv = run.call_args.args[0]
        self.assertEqual(argv[:3], ("timeout", "--signal=TERM", "5s"))
        self.assertEqual(argv[3], "ros2")

    def test_dashboard_requires_connected_and_fresh_topics(self) -> None:
        topics = {
            name: {"state": "fresh"}
            for name in ("camera", "point_cloud", "map", "odom", "pose_map")
        }
        status = {
            "bridge": {"connected": True},
            "telemetry_read_only": True,
            "topics": topics,
            "camera_quality": {
                "ready": True,
                "healthy": True,
                "level": 0,
                "message": "camera quality gate passed",
                "rate_hz": 1.54,
                "quality_error_ratio": 0.0,
            },
        }
        health = {"ros_connected": True, "telemetry_read_only": True}
        with mock.patch.object(
            gate, "get_json_loopback", side_effect=[(status, "status"), (health, "health")]
        ):
            self.assertEqual(gate.validate_dashboard(8092).status, gate.PASS)
        status["topics"]["camera"]["state"] = "stale"
        with mock.patch.object(
            gate, "get_json_loopback", side_effect=[(status, "status"), (health, "health")]
        ):
            self.assertEqual(gate.validate_dashboard(8092).status, gate.FAIL)

    def test_dashboard_camera_quality_warn_and_error_fail_closed(self) -> None:
        topics = {
            name: {"state": "fresh"}
            for name in ("camera", "point_cloud", "map", "odom", "pose_map")
        }
        health = {"ros_connected": True, "telemetry_read_only": True}
        for level, message, rate_hz, error_ratio in (
            (
                1,
                "camera is usable but quality errors occurred in the active window",
                1.18,
                0.056,
            ),
            (2, "camera stream stale", 0.74, 0.24),
        ):
            with self.subTest(level=level):
                status = {
                    "bridge": {"connected": True},
                    "telemetry_read_only": True,
                    "topics": topics,
                    "camera_quality": {
                        "ready": True,
                        "healthy": False,
                        "level": level,
                        "message": message,
                        "rate_hz": rate_hz,
                        "quality_error_ratio": error_ratio,
                    },
                }
                with mock.patch.object(
                    gate,
                    "get_json_loopback",
                    side_effect=[(status, "status"), (health, "health")],
                ):
                    result = gate.validate_dashboard(8092)
                self.assertEqual(result.status, gate.FAIL)
                self.assertIn(f"level={level!r}", result.detail)
                self.assertIn(f"message={message!r}", result.detail)
                self.assertIn(f"rate_hz={rate_hz!r}", result.detail)
                self.assertIn(
                    f"quality_error_ratio={error_ratio!r}", result.detail
                )
                self.assertEqual(
                    result.evidence["camera_quality"],
                    status["camera_quality"],
                )

    def test_dashboard_camera_quality_flags_and_shape_are_required(self) -> None:
        topics = {
            name: {"state": "fresh"}
            for name in ("camera", "point_cloud", "map", "odom", "pose_map")
        }
        health = {"ros_connected": True, "telemetry_read_only": True}
        quality = {
            "ready": True,
            "healthy": True,
            "level": 0,
            "message": "camera quality gate passed",
            "rate_hz": 1.54,
            "quality_error_ratio": 0.0,
        }

        for label, mutation in (
            ("not_ready", {"ready": False}),
            ("not_healthy", {"healthy": False}),
            ("warn_with_inconsistent_healthy", {"level": 1}),
            ("inconsistent_message", {"message": "camera stream stale"}),
            ("missing_rate", {"rate_hz": None}),
            ("invalid_ratio", {"quality_error_ratio": float("nan")}),
        ):
            with self.subTest(label=label):
                status = {
                    "bridge": {"connected": True},
                    "telemetry_read_only": True,
                    "topics": topics,
                    "camera_quality": {**quality, **mutation},
                }
                with mock.patch.object(
                    gate,
                    "get_json_loopback",
                    side_effect=[(status, "status"), (health, "health")],
                ):
                    self.assertEqual(
                        gate.validate_dashboard(8092).status,
                        gate.FAIL,
                    )

        status = {
            "bridge": {"connected": True},
            "telemetry_read_only": True,
            "topics": topics,
        }
        with mock.patch.object(
            gate,
            "get_json_loopback",
            side_effect=[(status, "status"), (health, "health")],
        ):
            self.assertEqual(gate.validate_dashboard(8092).status, gate.FAIL)

    def test_speech_log_requires_non_mock_streaming_asr_ok(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "speech.log"
            record = {
                "ts": "2026-07-17 00:00:00.000000000",
                "level": "info",
                "tag": "speech",
                "msg": "Backend status: mode=local asr=Whisper (UNAVAILABLE) asr_stream=FunASR streaming ASR (OK) tts=Edge (OK)",
            }
            path.write_text(json.dumps(record) + "\n", encoding="utf-8")
            self.assertEqual(gate.validate_speech_backend(path).status, gate.PASS)
            record["msg"] = record["msg"].replace("mode=local", "mode=mock")
            path.write_text(json.dumps(record) + "\n", encoding="utf-8")
            self.assertEqual(gate.validate_speech_backend(path).status, gate.FAIL)


class ReadinessStaticSafetyTests(unittest.TestCase):
    def test_scripts_contain_no_write_or_motion_cli(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        wrapper = (ROOT / "scripts" / "check_stack_readiness.sh").read_text(
            encoding="utf-8"
        )
        combined = source + wrapper
        forbidden = (
            "ros2 topic pub",
            "ros2 action send_goal",
            "ros2 service call",
            "SportClient",
            "sport_mode_ctrl",
            "go2_sport_client",
            "low_level_ctrl",
        )
        for needle in forbidden:
            self.assertNotIn(needle, combined)

    def test_every_ros_command_is_routed_through_bounded_runner(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn('subprocess.run(("ros2"', source)
        self.assertGreaterEqual(source.count("runner.ros("), 5)
        self.assertIn('("timeout", "--signal=TERM"', source)

    def test_wrapper_is_read_only_and_executable_after_install_step(self) -> None:
        wrapper = (ROOT / "scripts" / "check_stack_readiness.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("READ-ONLY", wrapper)
        self.assertNotIn("sudo", wrapper)
        self.assertNotIn("apt ", wrapper)
        self.assertNotIn("nmcli con", wrapper)

    def test_wrapper_validates_corrected_session_before_using_runtime_dir(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        wrapper = (ROOT / "scripts" / "check_stack_readiness.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('source "$SESSION_HELPER"', wrapper)
        self.assertIn("go2_nomotion_validate_session_files", wrapper)
        self.assertIn("go2_nomotion_process_identity_matches", wrapper)
        self.assertIn("go2_nomotion_process_holds_lock", wrapper)
        self.assertIn('RUNTIME_DIR="$GO2_SESSION_RUN_DIR"', wrapper)
        self.assertIn('--runtime-dir "$RUNTIME_DIR"', wrapper)
        self.assertIn('runtime_dir / "rbnx-boot" / "state.json"', source)
        self.assertIn(
            'runtime_dir / "rbnx-boot" / "logs" / "speech.log"', source
        )


if __name__ == "__main__":
    unittest.main()
