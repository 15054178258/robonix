from __future__ import annotations

import ast
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC = ROOT / "deploy" / "time-sync"
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(TIME_SYNC))

from navigation_stamp_discipline import (  # noqa: E402
    go2_workstation_motion_config,
    go2_workstation_nomotion_config,
)
from workstation_nomotion_approval import (  # noqa: E402
    ACK,
    EXPECTED_CLOCK_DOMAIN,
    load_approval,
)
import workstation_nomotion_cloud_relay as cloud_relay  # noqa: E402
import workstation_nomotion_stamp_node as stamp_node  # noqa: E402


NS = 1_000_000_000
SOURCE_BASE = 1_700_000_000 * NS
OFFSET = 748_294_000_000
DRIFT_PPM = 13.0
SCALE = 1.0 / (1.0 - DRIFT_PPM * 1e-6)


def approval_payload(now_ns: int) -> dict:
    return {
        "schema": "robonix-go2-workstation-nomotion-stamp-offset-v3",
        "session_id": "go2-cloud-relay-test-session",
        "motion_enabled": False,
        "identity_evidence_verified": True,
        "expected_clock_domain": EXPECTED_CLOCK_DOMAIN,
        "writer_gids": {
            "sport_primary": "01" * 24,
            "mid360_imu": "02" * 24,
            "mid360_cloud": "03" * 24,
            "mid360_odom": "04" * 24,
        },
        "writer_source_ipv4": "192.168.123.161",
        "offset_evidence_sha256": "ab" * 32,
        "fixed_local_minus_source_offset_ns": OFFSET,
        "affine_drift_algorithm": "one-second-lower-envelope-theil-sen-v1",
        "affine_drift_window_ns": 30 * NS,
        "affine_window_common_drifts_ppm": {
            "first": 12.0,
            "second": 14.0,
        },
        "approved_affine_common_drift_ppm": DRIFT_PPM,
        "affine_window_common_drift_deviation_ppm": 2.0,
        "not_before_unix_ns": now_ns - 10 * NS,
        "expires_unix_ns": now_ns + 600 * NS,
        "operator_ack": ACK,
    }


def timestamp_safety_limits() -> dict[str, int | float | None]:
    config = go2_workstation_nomotion_config(EXPECTED_CLOCK_DOMAIN)
    return timestamp_safety_limits_for(config)


def timestamp_safety_limits_for(config) -> dict[str, int | float | None]:
    return {
        "offset_guard_ns": config.offset_guard_ns,
        "affine_anchor_past_guard_ns": config.affine_anchor_past_guard_ns,
        "max_corrected_future_ns": config.max_corrected_future_ns,
        "minimum_locked_corrected_age_ns": (
            config.minimum_locked_corrected_age_ns
        ),
        "max_pairwise_drift_ppm": config.max_pairwise_drift_ppm,
        "max_approved_affine_drift_deviation_ppm": (
            config.max_approved_affine_drift_deviation_ppm
        ),
        "affine_qualification_window_ns": (
            config.affine_qualification_window_ns
        ),
        "max_affine_window_common_drift_deviation_ppm": (
            config.max_affine_window_common_drift_deviation_ppm
        ),
        "max_locked_affine_drift_deviation_ppm": (
            config.max_locked_affine_drift_deviation_ppm
        ),
    }


def stamp_ready_payload() -> dict:
    anchor_local_ns = SOURCE_BASE + OFFSET
    return {
        "schema": stamp_node.AFFINE_READY_SCHEMA,
        "session_id": "go2-cloud-relay-test-session",
        "correction_mode": "affine",
        "discipline_profile": "nomotion",
        "corrected_topics": dict(stamp_node.CORRECTED_TOPICS),
        "time_discipline_ready": True,
        "motion_ready": False,
        "canonical_odom_ready": False,
        "lidar_odom_semantics": "private_mapping_input_not_chassis_odom",
        "timestamp_safety_limits": timestamp_safety_limits(),
        "approval_reference_offset_ns": OFFSET,
        "approval_affine_common_drift_ppm": DRIFT_PPM,
        "noncore_delta_drop_count": 0,
        "last_noncore_delta_drop": None,
        "affine_model": {
            "anchor_source_ns": SOURCE_BASE,
            "anchor_local_ns": anchor_local_ns,
            "drift_ppm": DRIFT_PPM,
            "source_to_local_scale": SCALE,
            "core_stream_drifts_ppm": {
                name: DRIFT_PPM for name in stamp_node.AFFINE_CORE_STREAMS
            },
            "stream_baseline_corrected_age_ns": {
                name: 20_000_000 for name in stamp_node.CORRECTED_TOPICS
            },
            "frozen": True,
        },
        "post_evaluation_commit": {"committed": True},
    }


def motion_stamp_ready_payload() -> dict:
    payload = stamp_ready_payload()
    config = go2_workstation_motion_config(EXPECTED_CLOCK_DOMAIN)
    payload["discipline_profile"] = "motion"
    payload["corrected_topics"] = dict(stamp_node.MOTION_CORRECTED_TOPICS)
    payload["timestamp_safety_limits"] = timestamp_safety_limits_for(config)
    payload["affine_model"]["stream_baseline_corrected_age_ns"] = {
        name: 20_000_000 for name in stamp_node.MOTION_CORRECTED_TOPICS
    }
    return payload


def write_private_json(path: Path, payload: dict) -> None:
    path.write_text(
        json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    path.chmod(0o600)


class CloudRelayContractTests(unittest.TestCase):
    def load_contract(self):
        now_ns = time.time_ns()
        with tempfile.TemporaryDirectory() as temporary:
            approval_path = Path(temporary) / "approval.json"
            write_private_json(approval_path, approval_payload(now_ns))
            approval = load_approval(
                approval_path,
                require_affine=True,
                now_realtime_ns=now_ns,
            )
        return cloud_relay.validate_stamp_ready(stamp_ready_payload(), approval)

    def load_approval(self):
        now_ns = time.time_ns()
        temporary = tempfile.TemporaryDirectory()
        approval_path = Path(temporary.name) / "approval.json"
        write_private_json(approval_path, approval_payload(now_ns))
        approval = load_approval(
            approval_path,
            require_affine=True,
            now_realtime_ns=now_ns,
        )
        return temporary, approval

    def test_valid_ready_reuses_exact_frozen_affine_correction(self) -> None:
        contract = self.load_contract()
        source_ns = SOURCE_BASE + 7 * NS
        expected_ns = SOURCE_BASE + OFFSET + round(7 * NS * SCALE)

        self.assertEqual(
            contract.corrected_stamp_ns(source_ns, expected_ns + 20_000_000),
            expected_ns,
        )
        self.assertEqual(
            contract.model.corrected_stamp_ns(source_ns),
            expected_ns,
        )
        self.assertEqual(contract.session_id, "go2-cloud-relay-test-session")

    def test_current_qualified_drift_is_not_rejected_by_historical_median(self) -> None:
        temporary, approval = self.load_approval()
        self.addCleanup(temporary.cleanup)
        payload = stamp_ready_payload()
        current_drift_ppm = DRIFT_PPM + 20.0
        payload["affine_model"]["drift_ppm"] = current_drift_ppm
        payload["affine_model"]["source_to_local_scale"] = 1.0 / (
            1.0 - current_drift_ppm * 1e-6
        )
        payload["affine_model"]["core_stream_drifts_ppm"] = {
            name: current_drift_ppm for name in stamp_node.AFFINE_CORE_STREAMS
        }

        contract = cloud_relay.validate_stamp_ready(payload, approval)

        self.assertEqual(contract.model.drift_ppm, current_drift_ppm)
        self.assertEqual(contract.max_corrected_age_ns, 250_000_000)
        self.assertEqual(contract.hard_corrected_age_ns, 2_000_000_000)
        self.assertEqual(contract.minimum_corrected_age_ns, 5_000_000)
        self.assertEqual(contract.clock_pair_age_acquisition_attempts, 3)

    def test_corrected_age_boundaries_fail_closed(self) -> None:
        contract = self.load_contract()
        source_ns = SOURCE_BASE + NS
        corrected_ns = contract.model.corrected_stamp_ns(source_ns)

        self.assertEqual(
            contract.corrected_stamp_ns(
                source_ns, corrected_ns + contract.minimum_corrected_age_ns
            ),
            corrected_ns,
        )
        self.assertEqual(
            contract.corrected_stamp_ns(
                source_ns, corrected_ns + contract.max_corrected_age_ns
            ),
            corrected_ns,
        )
        for receipt_ns, reason in (
            (
                corrected_ns + contract.minimum_corrected_age_ns - 1,
                "age margin",
            ),
            (
                corrected_ns + contract.max_corrected_age_ns + 1,
                "transiently stale",
            ),
            (
                corrected_ns - contract.max_corrected_future_ns - 1,
                "future",
            ),
        ):
            with self.subTest(reason=reason):
                with self.assertRaisesRegex(cloud_relay.RelayError, reason):
                    contract.corrected_stamp_ns(source_ns, receipt_ns)
        with self.assertRaisesRegex(
            cloud_relay.RelayError, "hard stale ceiling"
        ):
            contract.corrected_stamp_ns(
                source_ns,
                corrected_ns + contract.hard_corrected_age_ns + 1,
            )
        with self.assertRaises(cloud_relay.TransientCloudStale) as transient:
            contract.corrected_stamp_ns(
                source_ns,
                corrected_ns + contract.hard_corrected_age_ns,
            )
        self.assertEqual(
            transient.exception.corrected_age_ns,
            2_000_000_000,
        )

    def test_ready_rejects_model_limit_and_baseline_tampering(self) -> None:
        temporary, approval = self.load_approval()
        self.addCleanup(temporary.cleanup)
        mutations = (
            (
                ("affine_model", "source_to_local_scale"),
                SCALE + 1e-6,
                "scale is inconsistent",
            ),
            (
                ("affine_model", "frozen"),
                False,
                "not frozen",
            ),
            (
                ("affine_model", "core_stream_drifts_ppm", "mid360_imu"),
                DRIFT_PPM + 26.0,
                "drift disagreement",
            ),
            (
                ("timestamp_safety_limits", "max_corrected_future_ns"),
                timestamp_safety_limits()["max_corrected_future_ns"] + 1,
                "safety limit changed",
            ),
            (
                (
                    "affine_model",
                    "stream_baseline_corrected_age_ns",
                    "mid360_cloud",
                ),
                4_999_999,
                "baseline corrected age is unsafe",
            ),
            (
                (
                    "affine_model",
                    "stream_baseline_corrected_age_ns",
                    "mid360_cloud",
                ),
                250_000_001,
                "baseline corrected age is unsafe",
            ),
        )
        for path, value, reason in mutations:
            with self.subTest(path=path, value=value):
                payload = deepcopy(stamp_ready_payload())
                target = payload
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = value
                with self.assertRaisesRegex(cloud_relay.RelayError, reason):
                    cloud_relay.validate_stamp_ready(payload, approval)

    def test_noncore_drop_evidence_is_profile_bound_and_validated(self) -> None:
        temporary, approval = self.load_approval()
        self.addCleanup(temporary.cleanup)
        valid_last = {
            "stream": "mid360_cloud",
            "reason": (
                f"{cloud_relay.NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud"
            ),
            "receipt_monotonic_ns": 10 * NS,
            "source_delta_ns": 60_000_000,
            "receipt_delta_ns": 220_000_001,
            "delta_error_ns": 160_000_001,
            "delta_error_limit_ns": 150_000_000,
        }
        payload = stamp_ready_payload()
        payload["noncore_delta_drop_count"] = 2
        payload["last_noncore_delta_drop"] = valid_last
        cloud_relay.validate_stamp_ready(payload, approval)

        mutations = (
            ("count", False, None, "count"),
            ("count", -1, None, "count"),
            ("last", 0, valid_last, "null last"),
            ("last", 1, None, "fields"),
            (
                "identity",
                1,
                {**valid_last, "stream": "mid360_imu"},
                "identity",
            ),
            (
                "timing type",
                1,
                {**valid_last, "source_delta_ns": True},
                "positive integers",
            ),
            (
                "timing relation",
                1,
                {**valid_last, "delta_error_ns": 160_000_000},
                "inconsistent",
            ),
        )
        for name, count, last, reason in mutations:
            with self.subTest(name=name):
                changed = stamp_ready_payload()
                changed["noncore_delta_drop_count"] = count
                changed["last_noncore_delta_drop"] = last
                with self.assertRaisesRegex(cloud_relay.RelayError, reason):
                    cloud_relay.validate_stamp_ready(changed, approval)

        motion = motion_stamp_ready_payload()
        motion["noncore_delta_drop_count"] = 1
        motion["last_noncore_delta_drop"] = valid_last
        with self.assertRaisesRegex(
            cloud_relay.RelayError, "motion stamp READY"
        ):
            cloud_relay.validate_stamp_ready(
                motion, approval, profile="motion"
            )

    def test_motion_ready_reuses_core_model_without_claiming_motion(self) -> None:
        temporary, approval = self.load_approval()
        self.addCleanup(temporary.cleanup)
        contract = cloud_relay.validate_stamp_ready(
            motion_stamp_ready_payload(), approval, profile="motion"
        )

        self.assertEqual(contract.profile, "motion")
        self.assertEqual(contract.max_corrected_age_ns, 250_000_000)
        self.assertEqual(contract.hard_corrected_age_ns, 250_000_000)
        self.assertIsNone(contract.minimum_corrected_age_ns)
        self.assertEqual(contract.publish_period_ns, 100_000_000)
        self.assertEqual(contract.clock_pair_age_acquisition_attempts, 3)
        self.assertTrue(contract.drop_all_stale)
        self.assertFalse(contract.require_fresh_recovery)
        self.assertEqual(
            dict(contract.model.stream_baseline_corrected_age_ns),
            {
                name: 20_000_000
                for name in stamp_node.MOTION_CORRECTED_TOPICS
            },
        )

    def test_motion_cloud_drops_every_sample_over_250ms(self) -> None:
        temporary, approval = self.load_approval()
        self.addCleanup(temporary.cleanup)
        contract = cloud_relay.validate_stamp_ready(
            motion_stamp_ready_payload(), approval, profile="motion"
        )
        source_ns = SOURCE_BASE + NS
        corrected_ns = contract.model.corrected_stamp_ns(source_ns)

        self.assertEqual(
            contract.corrected_stamp_ns(
                source_ns, corrected_ns + 250_000_000
            ),
            corrected_ns,
        )
        for age_ns in (250_000_001, 500_000_001, 10 * NS):
            with self.subTest(age_ns=age_ns):
                with self.assertRaises(cloud_relay.TransientCloudStale):
                    contract.corrected_stamp_ns(
                        source_ns, corrected_ns + age_ns
                    )

    def test_profiles_cannot_consume_each_others_ready_contract(self) -> None:
        temporary, approval = self.load_approval()
        self.addCleanup(temporary.cleanup)
        for payload, profile in (
            (stamp_ready_payload(), "motion"),
            (motion_stamp_ready_payload(), "nomotion"),
        ):
            with self.subTest(profile=profile):
                with self.assertRaisesRegex(
                    cloud_relay.RelayError, "timestamp profile"
                ):
                    cloud_relay.validate_stamp_ready(
                        payload, approval, profile=profile
                    )

        with self.assertRaisesRegex(
            cloud_relay.RelayError, "unsupported cloud relay profile"
        ):
            cloud_relay.validate_stamp_ready(
                stamp_ready_payload(), approval, profile="unsafe"
            )


class ParentContractGuardTests(unittest.TestCase):
    def make_guard(
        self, directory: Path
    ) -> tuple[cloud_relay.ParentContractGuard, dict[str, Path]]:
        paths = {
            "approval": directory / "approval.json",
            "ready": directory / "ready.json",
            "stamp_fault": directory / "stamp-fault.json",
            "identity_fault": directory / "identity-fault.json",
        }
        write_private_json(paths["approval"], {"kind": "approval", "version": 1})
        write_private_json(paths["ready"], {"kind": "ready", "version": 1})
        _, approval_snapshot = cloud_relay._read_private_json(paths["approval"])
        _, ready_snapshot = cloud_relay._read_private_json(paths["ready"])
        identity = cloud_relay.process_identity(os.getpid())
        guard = cloud_relay.ParentContractGuard(
            approval_file=paths["approval"],
            approval_snapshot=approval_snapshot,
            stamp_ready_file=paths["ready"],
            stamp_ready_snapshot=ready_snapshot,
            stamp_fault_file=paths["stamp_fault"],
            identity_fault_file=paths["identity_fault"],
            stamp_process=identity,
            identity_process=identity,
        )
        return guard, paths

    def test_guard_rejects_ready_replacement(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            guard, paths = self.make_guard(Path(temporary))
            guard.check()
            replacement = paths["ready"].with_suffix(".replacement")
            write_private_json(
                replacement, {"kind": "ready", "version": 2}
            )
            os.replace(replacement, paths["ready"])
            with self.assertRaisesRegex(
                cloud_relay.RelayError, "READY changed"
            ):
                guard.check()

    def test_guard_rejects_each_parent_fault_even_if_empty(self) -> None:
        for fault_name, expected in (
            ("stamp_fault", "timestamp fault became visible"),
            ("identity_fault", "identity fault became visible"),
        ):
            with self.subTest(fault_name=fault_name):
                with tempfile.TemporaryDirectory() as temporary:
                    guard, paths = self.make_guard(Path(temporary))
                    paths[fault_name].touch(mode=0o600)
                    with self.assertRaisesRegex(
                        cloud_relay.RelayError, expected
                    ):
                        guard.check()


class PublishRateLimiterTests(unittest.TestCase):
    def test_rate_limiter_allows_at_most_one_publish_per_period(self) -> None:
        limiter = cloud_relay._PublishRateLimiter(period_ns=200)
        self.assertTrue(limiter.allow(1_000))
        self.assertFalse(limiter.allow(1_199))
        self.assertTrue(limiter.allow(1_200))
        self.assertFalse(limiter.allow(1_399))
        self.assertTrue(limiter.allow(1_400))

    def test_rate_limiter_rejects_monotonic_regression(self) -> None:
        limiter = cloud_relay._PublishRateLimiter(period_ns=200)
        self.assertTrue(limiter.allow(1_000))
        with self.assertRaisesRegex(cloud_relay.RelayError, "regressed"):
            limiter.allow(999)
        self.assertFalse(limiter.allow(1_100))


class CloudStampSequenceGuardTests(unittest.TestCase):
    def test_source_and_corrected_stamps_must_strictly_advance(self) -> None:
        sequence = cloud_relay._CloudStampSequenceGuard()
        sequence.observe(1_000, 2_000)
        sequence.observe(1_001, 2_001)
        for source_ns, corrected_ns, reason in (
            (1_001, 2_002, "source timestamp"),
            (1_000, 2_002, "source timestamp"),
            (1_002, 2_001, "corrected timestamp"),
            (1_002, 2_000, "corrected timestamp"),
        ):
            with self.subTest(
                source_ns=source_ns, corrected_ns=corrected_ns
            ):
                with self.assertRaisesRegex(
                    cloud_relay.RelayError, reason
                ):
                    sequence.observe(source_ns, corrected_ns)


class RuntimeFaultSchemaTests(unittest.TestCase):
    def test_motion_fault_is_distinct_and_still_never_grants_motion(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        contract = object.__new__(cloud_relay.CloudCorrectionContract)
        object.__setattr__(contract, "session_id", "motion-session")
        object.__setattr__(contract, "profile", "motion")
        fault_file = Path(temporary.name) / "fault.json"
        runtime = cloud_relay._Runtime(
            contract,
            Path(temporary.name) / "ready.json",
            fault_file,
        )

        runtime.latch_fault("test fault")

        payload = json.loads(fault_file.read_text(encoding="utf-8"))
        self.assertEqual(payload["schema"], cloud_relay.MOTION_FAULT_SCHEMA)
        self.assertIs(payload["motion_ready"], False)
        self.assertIs(payload["canonical_odom_ready"], False)


class FreshCloudRecoveryGateTests(unittest.TestCase):
    def test_transient_stale_requires_two_consecutive_fresh_samples(self) -> None:
        gate = cloud_relay._FreshCloudRecoveryGate()
        self.assertTrue(gate.allow_fresh())
        gate.observe_stale()
        self.assertFalse(gate.allow_fresh())
        gate.observe_stale()
        self.assertFalse(gate.allow_fresh())
        self.assertTrue(gate.allow_fresh())
        self.assertTrue(gate.allow_fresh())


class CloudRelayStaticSafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.relay_path = (
            TIME_SYNC / "workstation_nomotion_cloud_relay.py"
        )
        self.stamp_path = (
            TIME_SYNC / "workstation_nomotion_stamp_node.py"
        )
        self.relay_source = self.relay_path.read_text(encoding="utf-8")
        self.stamp_source = self.stamp_path.read_text(encoding="utf-8")
        self.relay_tree = ast.parse(
            self.relay_source, filename=str(self.relay_path)
        )

    def test_pointcloud2_has_one_corrected_publisher_and_depth_one_io(self) -> None:
        self.assertEqual(
            self.relay_source.count(
                "PointCloud2, OUTPUT_TOPIC, corrected_qos"
            ),
            1,
        )
        self.assertEqual(
            self.relay_source.count(
                "PointCloud2, INPUT_TOPIC, self._cloud, raw_qos"
            ),
            1,
        )
        self.assertIn('if stream != "mid360_cloud"', self.stamp_source)
        self.assertNotIn(
            'corrected_message_copy(\n                    message, "mid360_cloud"',
            self.stamp_source,
        )
        self.assertEqual(self.relay_source.count("depth=1"), 2)
        self.assertEqual(
            cloud_relay.OUTPUT_TOPIC,
            stamp_node.CORRECTED_TOPICS["mid360_cloud"],
        )
        first_freshness = self.relay_source.index(
            "corrected_ns = contract.corrected_stamp_ns("
        )
        limiter = self.relay_source.index(
            "self._limiter.allow(clocks.monotonic_ns)"
        )
        sequence = self.relay_source.index(
            "self._stamp_sequence.observe(source_ns, corrected_ns)"
        )
        self.assertIn(
            'if contract.profile == "motion":',
            self.relay_source[first_freshness:sequence],
        )
        copy_message = self.relay_source.index(
            'corrected_message_copy(\n'
            '                    message, "mid360_cloud", corrected_ns'
        )
        second_freshness = self.relay_source.index(
            "publish_corrected_ns = contract.corrected_stamp_ns("
        )
        publish = self.relay_source.index("self._publisher.publish(output)")
        self.assertLess(first_freshness, sequence)
        self.assertLess(sequence, limiter)
        self.assertLess(limiter, copy_message)
        self.assertLess(copy_message, second_freshness)
        self.assertLess(second_freshness, publish)

    def test_raw_cloud_is_reliable_only_for_nomotion_mapping(self) -> None:
        self.assertTrue(cloud_relay._raw_subscription_is_reliable("nomotion"))
        self.assertFalse(cloud_relay._raw_subscription_is_reliable("motion"))
        with self.assertRaisesRegex(
            cloud_relay.RelayError, "unsupported cloud relay profile"
        ):
            cloud_relay._raw_subscription_is_reliable("unexpected")
        self.assertIn(
            "if _raw_subscription_is_reliable(contract.profile)",
            self.relay_source,
        )

    def test_has_no_command_service_action_or_unitree_api_surface(self) -> None:
        forbidden_calls = {
            "create_client",
            "create_service",
            "create_action_client",
            "create_action_server",
        }
        called_attributes = {
            node.func.attr
            for node in ast.walk(self.relay_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
        }
        self.assertTrue(forbidden_calls.isdisjoint(called_attributes))

        imported_modules: set[str] = set()
        for node in ast.walk(self.relay_tree):
            if isinstance(node, ast.Import):
                imported_modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.add(node.module)
        self.assertFalse(
            any(
                module.startswith(("unitree", "rclpy.action"))
                for module in imported_modules
            )
        )
        string_literals = {
            node.value
            for node in ast.walk(self.relay_tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        for forbidden_topic in (
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
        ):
            self.assertNotIn(forbidden_topic, string_literals)

    def test_ros_imports_are_confined_to_run_ros(self) -> None:
        run_ros_node = next(
            node
            for node in self.relay_tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "run_ros"
        )
        ros_imports = []
        for node in ast.walk(self.relay_tree):
            modules: tuple[str, ...] = ()
            if isinstance(node, ast.Import):
                modules = tuple(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                modules = (node.module,)
            if any(
                module == "rclpy"
                or module.startswith("rclpy.")
                or module == "sensor_msgs"
                or module.startswith("sensor_msgs.")
                for module in modules
            ):
                ros_imports.append(node)
        self.assertGreaterEqual(len(ros_imports), 2)
        for node in ros_imports:
            self.assertGreaterEqual(node.lineno, run_ros_node.lineno)
            self.assertLessEqual(node.end_lineno, run_ros_node.end_lineno)

    def test_motion_mode_is_explicit_and_never_grants_motion(self) -> None:
        self.assertIn('"--profile"', self.relay_source)
        self.assertIn('default="nomotion"', self.relay_source)
        self.assertIn(
            '"stale_policy": "drop_without_publish"', self.relay_source
        )
        self.assertEqual(
            self.relay_source.count('"motion_ready": False'),
            2,
        )

    def test_launcher_binds_relay_to_parents_and_manages_lifecycle(self) -> None:
        source = (
            SCRIPTS / "start_workstation_full_nomotion_corrected.sh"
        ).read_text(encoding="utf-8")
        relay_start = source.index(
            '"$ROOT/deploy/time-sync/workstation_nomotion_cloud_relay.py"'
        )
        relay_pid = source.index("CLOUD_RELAY_PID=$!", relay_start)
        relay_block = source[relay_start:relay_pid]
        for required in (
            '--approval-file "$APPROVAL_FILE"',
            '--stamp-ready-file "$READY_FILE"',
            '--stamp-fault-file "$FAULT_FILE"',
            '--identity-fault-file "$IDENTITY_FAULT_FILE"',
            '--stamp-pid "$STAMP_PID"',
            '--identity-pid "$IDENTITY_PID"',
            '--ready-file "$CLOUD_RELAY_READY_FILE"',
            '--fault-file "$CLOUD_RELAY_FAULT_FILE"',
        ):
            self.assertIn(required, relay_block)
        child_start = source.rfind("(\n", 0, relay_start)
        self.assertGreaterEqual(child_start, 0)
        self.assertIn(
            'go2_nomotion_close_inherited_fd "$DISCIPLINE_LOCK_FD"',
            source[child_start:relay_start],
        )

        relay_wait = source[relay_pid : source.index(
            'echo "Affine timestamp correction locked', relay_pid
        )]
        for fault_file in (
            "$FAULT_FILE",
            "$IDENTITY_FAULT_FILE",
            "$CLOUD_RELAY_FAULT_FILE",
        ):
            self.assertIn(f'[[ -s "{fault_file}" ]]', relay_wait)
        self.assertLess(
            relay_wait.rindex('[[ -s "$CLOUD_RELAY_FAULT_FILE" ]]'),
            relay_wait.index(
                'echo "Affine timestamp correction locked'
            )
            if 'echo "Affine timestamp correction locked' in relay_wait
            else len(relay_wait),
        )
        self.assertIn(
            'WAIT_PIDS=("$IDENTITY_PID" "$STAMP_PID" '
            '"$CLOUD_RELAY_PID" "$STACK_PID")',
            source,
        )
        self.assertIn(
            'elif [[ "$EXITED_PID" == "$CLOUD_RELAY_PID" ]]',
            source,
        )
        cleanup_start = source.index("stop_owned_children() {")
        cleanup = source[
            cleanup_start : source.index("\n}\n", cleanup_start)
        ]
        self.assertIn('kill -TERM "$CLOUD_RELAY_PID"', cleanup)
        self.assertIn('wait "$CLOUD_RELAY_PID"', cleanup)


if __name__ == "__main__":
    unittest.main()
