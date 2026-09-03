from __future__ import annotations

import ast
from dataclasses import replace
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC))

from navigation_stamp_discipline import (  # noqa: E402
    CorrectionResult,
    DisciplineState,
    NavigationStampDiscipline,
    NONCORE_DELTA_DROP_REASON_PREFIX,
    go2_navigation_config,
    go2_workstation_motion_config,
    go2_workstation_nomotion_config,
)
from render_workstation_nomotion_manifest import (  # noqa: E402
    CANONICAL_ODOM,
    MAPPING_PARAMS_FILE,
    ManifestError,
    NAV2_BT_THROUGH_POSES_XML_FILE,
    NAV2_BT_XML_FILE,
    NAV2_PARAMS_FILE,
    PRIVATE_CHASSIS_IMU,
    PRIVATE_CLOUD,
    PRIVATE_IMU,
    PRIVATE_LIDAR_ODOM,
    PRIVATE_SPORT,
    SAFE_CHASSIS_INPUT,
    SAFE_VELOCITY_OUTPUT,
    UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS,
    render,
    validate_rendered,
)
from workstation_nomotion_approval import (  # noqa: E402
    ACK,
    EXPECTED_CLOCK_DOMAIN,
    ApprovalError,
    load_approval,
    require_strict_affine_approval,
)
import workstation_nomotion_identity_monitor as identity_monitor  # noqa: E402
import workstation_nomotion_stamp_node as stamp_node  # noqa: E402
from workstation_nomotion_stamp_node import (  # noqa: E402
    CORRECTED_TOPICS,
    RAW_TOPICS,
    corrected_message_copy,
    corrected_stamp_parts,
)


NS = 1_000_000_000
SOURCE_BASE = 1_700_000_000 * NS
MONO_BASE = 100 * NS
OFFSET = 748_294_000_000
CLOCK_BASE = SOURCE_BASE + OFFSET - MONO_BASE


def approval_payload(now_ns: int) -> dict:
    return {
        "schema": "robonix-go2-workstation-nomotion-stamp-offset-v3",
        "session_id": "go2-session-20260718-a",
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
        "approved_affine_common_drift_ppm": 13.0,
        "affine_window_common_drift_deviation_ppm": 2.0,
        "not_before_unix_ns": now_ns - 10 * NS,
        "expires_unix_ns": now_ns + 600 * NS,
        "operator_ack": ACK,
    }


class ApprovalTests(unittest.TestCase):
    def write(self, directory: Path, payload: dict, mode: int = 0o600) -> Path:
        path = directory / "approval.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        path.chmod(mode)
        return path

    def test_private_short_lived_nomotion_approval_loads(self) -> None:
        now_ns = time.time_ns()
        with tempfile.TemporaryDirectory() as temporary:
            path = self.write(Path(temporary), approval_payload(now_ns))
            result = load_approval(path, now_realtime_ns=now_ns)
        self.assertEqual(result.fixed_local_minus_source_offset_ns, OFFSET)
        self.assertEqual(result.approved_affine_common_drift_ppm, 13.0)
        self.assertEqual(result.expected_clock_domain, EXPECTED_CLOCK_DOMAIN)

    def test_legacy_v2_is_fixed_only_and_v3_affine_fields_are_strict(self) -> None:
        now_ns = time.time_ns()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            legacy = approval_payload(now_ns)
            legacy["schema"] = (
                "robonix-go2-workstation-nomotion-stamp-offset-v2"
            )
            for key in (
                "affine_drift_algorithm",
                "affine_drift_window_ns",
                "affine_window_common_drifts_ppm",
                "approved_affine_common_drift_ppm",
                "affine_window_common_drift_deviation_ppm",
            ):
                legacy.pop(key)
            legacy_path = self.write(directory, legacy)
            fixed = load_approval(legacy_path, now_realtime_ns=now_ns)
            self.assertIsNone(fixed.approved_affine_common_drift_ppm)
            with self.assertRaisesRegex(ApprovalError, "schema-v3"):
                load_approval(
                    legacy_path,
                    now_realtime_ns=now_ns,
                    require_affine=True,
                )

        mutations = (
            ({"approved_affine_common_drift_ppm": 13.1}, "two-window median"),
            (
                {"affine_window_common_drift_deviation_ppm": 1.9},
                "inconsistent",
            ),
            (
                {
                    "affine_window_common_drifts_ppm": {
                        "first": 10.0,
                        "second": 26.0,
                    },
                    "approved_affine_common_drift_ppm": 18.0,
                    "affine_window_common_drift_deviation_ppm": 16.0,
                },
                "exceeds 15 ppm",
            ),
        )
        for update, reason in mutations:
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as temporary:
                payload = approval_payload(now_ns)
                payload.update(update)
                path = self.write(Path(temporary), payload)
                with self.assertRaisesRegex(ApprovalError, reason):
                    load_approval(
                        path,
                        now_realtime_ns=now_ns,
                        require_affine=True,
                    )

    def test_startup_half_window_drift_limit_is_independent_fifteen_ppm(
        self,
    ) -> None:
        now_ns = time.time_ns()
        payload = approval_payload(now_ns)
        payload.update(
            {
                "affine_window_common_drifts_ppm": {
                    "first": 5.0,
                    "second": 20.0,
                },
                "approved_affine_common_drift_ppm": 12.5,
                "affine_window_common_drift_deviation_ppm": 15.0,
            }
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = self.write(Path(temporary), payload)
            approval = load_approval(
                path,
                now_realtime_ns=now_ns,
                require_affine=True,
            )
        self.assertIs(
            require_strict_affine_approval(
                approval,
                now_realtime_ns=now_ns,
            ),
            approval,
        )

        over_limit = replace(
            approval,
            affine_window_common_drifts_ppm=(5.0, 20.000_001),
            approved_affine_common_drift_ppm=12.500_000_5,
        )
        with self.assertRaisesRegex(ApprovalError, "exceeds 15 ppm"):
            require_strict_affine_approval(
                over_limit,
                now_realtime_ns=now_ns,
            )

    def test_approval_rejects_motion_future_issue_and_open_permissions(self) -> None:
        now_ns = time.time_ns()
        mutations = (
            ({"motion_enabled": True}, 0o600, "motion_enabled"),
            (
                {
                    "not_before_unix_ns": now_ns + NS,
                    "expires_unix_ns": now_ns + 600 * NS,
                },
                0o600,
                "issue time is in the future",
            ),
            ({}, 0o644, "group or others"),
        )
        for update, mode, reason in mutations:
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as temporary:
                payload = approval_payload(now_ns)
                payload.update(update)
                path = self.write(Path(temporary), payload, mode)
                with self.assertRaisesRegex(ApprovalError, reason):
                    load_approval(path, now_realtime_ns=now_ns)

    def test_approval_rejects_symlink_and_duplicate_keys(self) -> None:
        now_ns = time.time_ns()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            target = self.write(directory, approval_payload(now_ns))
            link = directory / "approval-link.json"
            link.symlink_to(target)
            with self.assertRaisesRegex(ApprovalError, "symlink"):
                load_approval(link, now_realtime_ns=now_ns)
            duplicate = directory / "duplicate.json"
            duplicate.write_text(
                '{"schema":"a","schema":"b"}', encoding="utf-8"
            )
            duplicate.chmod(0o600)
            with self.assertRaisesRegex(ApprovalError, "duplicate"):
                load_approval(duplicate, now_realtime_ns=now_ns)

    def test_approval_requires_distinct_gid_for_every_stream(self) -> None:
        now_ns = time.time_ns()
        mutations = []
        missing = approval_payload(now_ns)
        missing["writer_gids"].pop("mid360_odom")
        mutations.append((missing, "every required stream"))
        duplicate = approval_payload(now_ns)
        duplicate["writer_gids"]["mid360_odom"] = duplicate["writer_gids"][
            "mid360_imu"
        ]
        mutations.append((duplicate, "distinct writer GID"))
        for payload, reason in mutations:
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as temporary:
                path = self.write(Path(temporary), payload)
                with self.assertRaisesRegex(ApprovalError, reason):
                    load_approval(path, now_realtime_ns=now_ns)


class FixedOffsetCoreTests(unittest.TestCase):
    def config(self):
        original = go2_workstation_nomotion_config(EXPECTED_CLOCK_DOMAIN)
        return replace(
            original,
            streams=tuple(replace(policy, min_samples=100) for policy in original.streams),
            minimum_qualification_span_ns=10 * NS,
            retained_samples_per_stream=1000,
            statistics_evaluation_period_ns=20_000_000,
        )

    def test_live_qualification_is_rate_limited_outside_sensor_callbacks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval = SimpleNamespace(
                expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
                session_id="rate-limit-test",
                fixed_local_minus_source_offset_ns=OFFSET,
            )
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
            )
            discipline = mock.Mock()
            discipline.config.statistics_evaluation_period_ns = NS
            discipline.qualification_reasons.return_value = [
                "insufficient_span:sport_primary"
            ]
            runtime.discipline = discipline

            runtime.maybe_lock(10 * NS)
            runtime.maybe_lock(10 * NS + NS // 2)
            runtime.maybe_lock(11 * NS)

            self.assertEqual(discipline.qualification_reasons.call_count, 2)
            discipline.lock_fixed.assert_not_called()

    def test_stamp_node_does_not_shadow_rclpy_publisher_storage(self) -> None:
        source = (TIME_SYNC / "workstation_nomotion_stamp_node.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("self._publishers = {", source)
        self.assertIn("self._corrected_publishers = {", source)

    def test_raw_subscriptions_drop_queued_sensor_samples(self) -> None:
        """The timestamp breaker must see the newest raw sample under load."""

        path = TIME_SYNC / "workstation_nomotion_stamp_node.py"
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=path.name)

        qos_assignments = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "raw_subscription_qos"
                for target in node.targets
            )
        ]
        self.assertEqual(len(qos_assignments), 1)
        qos_mapping = qos_assignments[0].value
        self.assertIsInstance(qos_mapping, ast.DictComp)
        assert isinstance(qos_mapping, ast.DictComp)
        self.assertEqual(ast.unparse(qos_mapping.key), "stream")
        self.assertEqual(
            ast.unparse(qos_mapping.generators[0].iter), "runtime.streams"
        )
        qos_call = qos_mapping.value
        self.assertIsInstance(qos_call, ast.Call)
        assert isinstance(qos_call, ast.Call)
        self.assertIsInstance(qos_call.func, ast.Name)
        self.assertEqual(qos_call.func.id, "QoSProfile")
        keywords = {keyword.arg: keyword.value for keyword in qos_call.keywords}
        self.assertEqual(ast.literal_eval(keywords["depth"]), 1)
        self.assertEqual(ast.unparse(keywords["history"]), "HistoryPolicy.KEEP_LAST")
        reliability = keywords["reliability"]
        self.assertIsInstance(reliability, ast.IfExp)
        assert isinstance(reliability, ast.IfExp)
        self.assertEqual(
            ast.unparse(reliability.test),
            "_raw_subscription_is_reliable(runtime.profile, stream)",
        )
        self.assertEqual(
            ast.unparse(reliability.body), "ReliabilityPolicy.RELIABLE"
        )
        self.assertEqual(
            ast.unparse(reliability.orelse),
            "ReliabilityPolicy.BEST_EFFORT",
        )
        self.assertEqual(
            ast.unparse(keywords["durability"]), "DurabilityPolicy.VOLATILE"
        )

        subscriptions = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "create_subscription"
        ]
        # Endpoints are deliberately generated only for runtime.streams: the
        # no-motion profile gets all four streams, while the strict first-
        # motion profile has no PointCloud2 endpoint at all.  Verify the one
        # shared construction site and its bounded QoS instead of requiring
        # four duplicated calls in source code.
        self.assertEqual(len(subscriptions), 1)
        subscription = subscriptions[0]
        self.assertGreaterEqual(len(subscription.args), 4)
        self.assertEqual(ast.unparse(subscription.args[1]), "RAW_TOPICS[stream]")
        self.assertEqual(
            ast.unparse(subscription.args[3]), "raw_subscription_qos[stream]"
        )
        comprehensions = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.ListComp)
            and node.elt is subscription
        ]
        self.assertEqual(len(comprehensions), 1)
        self.assertEqual(
            ast.unparse(comprehensions[0].generators[0].iter),
            "runtime.streams",
        )

    def test_stamp_process_has_no_corrected_cloud_publisher(self) -> None:
        source = (
            TIME_SYNC / "workstation_nomotion_stamp_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("corrected_publisher_qos = QoSProfile(", source)
        self.assertIn("depth=1", source)
        self.assertIn(
            "reliability=ReliabilityPolicy.BEST_EFFORT", source
        )
        self.assertIn("durability=DurabilityPolicy.VOLATILE", source)
        self.assertNotIn("_LatestOnlyPublisherWorker", source)
        self.assertNotIn("_cloud_publish_worker", source)
        self.assertNotIn("_LatestSampleRateLimiter", source)
        self.assertIn('if stream != "mid360_cloud"', source)
        self.assertIn(
            "# This process remains the authoritative cloud timing", source
        )
        self.assertLess(
            source.index("result = runtime.discipline.observe("),
            source.index(
                "# This process remains the authoritative cloud timing"
            ),
        )

    def test_affine_receipt_recovery_pauses_both_runtime_profiles(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="receipt-recovery-profile-test",
            fixed_local_minus_source_offset_ns=OFFSET,
            approved_affine_common_drift_ppm=13.0,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with mock.patch.object(
                stamp_node, "require_strict_affine_approval"
            ):
                nomotion = stamp_node._Runtime(
                    approval,
                    directory / "nomotion-ready.json",
                    directory / "nomotion-fault.json",
                    mode="affine",
                    profile="nomotion",
                )
                motion = stamp_node._Runtime(
                    approval,
                    directory / "motion-ready.json",
                    directory / "motion-fault.json",
                    mode="affine",
                    profile="motion",
                )
        self.assertTrue(nomotion.discipline._allow_locked_receipt_recovery)
        self.assertTrue(motion.discipline._allow_locked_receipt_recovery)
        self.assertEqual(
            nomotion.discipline._locked_core_delta_recovery_streams,
            frozenset(stamp_node.AFFINE_CORE_STREAMS),
        )
        self.assertEqual(
            motion.discipline._locked_core_delta_recovery_streams,
            frozenset(stamp_node.AFFINE_CORE_STREAMS),
        )
    def test_reliable_raw_qos_is_nomotion_cloud_only(self) -> None:
        self.assertTrue(
            stamp_node._raw_subscription_is_reliable(
                "nomotion", "mid360_cloud"
            )
        )
        for stream in (
            "sport_primary",
            "mid360_imu",
            "mid360_odom",
        ):
            with self.subTest(profile="nomotion", stream=stream):
                self.assertFalse(
                    stamp_node._raw_subscription_is_reliable(
                        "nomotion", stream
                    )
                )
        for stream in RAW_TOPICS:
            with self.subTest(profile="motion", stream=stream):
                self.assertFalse(
                    stamp_node._raw_subscription_is_reliable("motion", stream)
                )

    def test_clock_pair_retry_keeps_the_latest_bounded_sample_with_age(self) -> None:
        with mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            side_effect=(
                100,
                2_000_100,
                10_000_000,
                10_040_000,
                20_000_000,
                20_200_000,
                20_250_000,
            ),
        ), mock.patch.object(
            stamp_node.time,
            "time_ns",
            side_effect=(1_000, 2_000, 3_000),
        ):
            pair = stamp_node.paired_receipt_clocks()

        self.assertEqual(pair.realtime_ns, 3_000)
        self.assertEqual(pair.monotonic_ns, 20_100_000)
        self.assertEqual(pair.read_span_ns, 200_000)
        self.assertEqual(pair.selected_pair_age_ns, 150_000)

    def test_old_tight_pair_after_retry_stall_is_rejected_as_stale(self) -> None:
        with mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            side_effect=(
                100,
                200,
                10_000_000,
                12_000_001,
                20_000_000,
                22_000_001,
                22_000_100,
                22_000_200,
            ),
        ), mock.patch.object(
            stamp_node.time,
            "time_ns",
            side_effect=(1_000, 2_000, 3_000),
        ), self.assertRaisesRegex(RuntimeError, "clock_pair_age_exceeded"):
            stamp_node.fresh_paired_receipt_clocks(
                max_clock_read_span_ns=1_000_000,
                age_acquisition_attempts=1,
            )

    def test_nomotion_reacquires_after_observed_clock_pair_age_stall(self) -> None:
        stale = stamp_node.ReceiptClockPair(
            realtime_ns=1_000,
            monotonic_ns=10_000_000,
            read_span_ns=200,
            selected_pair_age_ns=1_044_070,
        )
        fresh = stamp_node.ReceiptClockPair(
            realtime_ns=2_000,
            monotonic_ns=20_000_000,
            read_span_ns=300,
            selected_pair_age_ns=100,
        )
        with mock.patch.object(
            stamp_node,
            "paired_receipt_clocks",
            side_effect=(stale, fresh),
        ) as paired, mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            side_effect=(11_049_481, 20_000_200),
        ):
            result = stamp_node.fresh_paired_receipt_clocks(
                max_clock_read_span_ns=1_000_000,
                age_acquisition_attempts=(
                    stamp_node.NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS
                ),
            )

        self.assertEqual(result, fresh._replace(selected_pair_age_ns=200))
        self.assertEqual(paired.call_args_list, [mock.call(3), mock.call(3)])

    def test_nomotion_clock_pair_age_reacquisition_is_bounded(self) -> None:
        pairs = tuple(
            stamp_node.ReceiptClockPair(
                realtime_ns=index,
                monotonic_ns=index * 10_000_000,
                read_span_ns=200,
                selected_pair_age_ns=1_044_070,
            )
            for index in range(1, 4)
        )
        with mock.patch.object(
            stamp_node,
            "paired_receipt_clocks",
            side_effect=pairs,
        ) as paired, mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            side_effect=tuple(
                pair.monotonic_ns + 1_049_481 for pair in pairs
            ),
        ), self.assertRaisesRegex(
            RuntimeError,
            "clock_pair_age_exceeded:age_ns=1049481:"
            "return_age_ns=1044070:limit_ns=1000000",
        ):
            stamp_node.fresh_paired_receipt_clocks(
                max_clock_read_span_ns=1_000_000,
                age_acquisition_attempts=(
                    stamp_node.NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS
                ),
            )

        self.assertEqual(
            paired.call_count,
            stamp_node.NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS,
        )

    def test_clock_pair_reacquires_after_scheduler_delayed_read_span(self) -> None:
        delayed = stamp_node.ReceiptClockPair(
            realtime_ns=1_000,
            monotonic_ns=10_000,
            read_span_ns=8_738_179,
            selected_pair_age_ns=100,
        )
        fresh = stamp_node.ReceiptClockPair(
            realtime_ns=2_000,
            monotonic_ns=20_000,
            read_span_ns=200,
            selected_pair_age_ns=100,
        )
        with mock.patch.object(
            stamp_node,
            "paired_receipt_clocks",
            side_effect=(delayed, fresh),
        ) as paired, mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            side_effect=(10_200, 20_200),
        ):
            result = stamp_node.fresh_paired_receipt_clocks(
                max_clock_read_span_ns=1_000_000,
                age_acquisition_attempts=3,
            )

        self.assertEqual(result, fresh._replace(selected_pair_age_ns=200))
        self.assertEqual(paired.call_args_list, [mock.call(3), mock.call(3)])

    def test_clock_pair_read_span_limit_is_exact(self) -> None:
        at_limit = stamp_node.ReceiptClockPair(
            realtime_ns=1_000,
            monotonic_ns=10_000,
            read_span_ns=1_000_000,
            selected_pair_age_ns=100,
        )
        above_limit = stamp_node.ReceiptClockPair(
            realtime_ns=2_000,
            monotonic_ns=20_000,
            read_span_ns=1_000_001,
            selected_pair_age_ns=100,
        )
        fresh = stamp_node.ReceiptClockPair(
            realtime_ns=3_000,
            monotonic_ns=30_000,
            read_span_ns=200,
            selected_pair_age_ns=100,
        )
        with mock.patch.object(
            stamp_node,
            "paired_receipt_clocks",
            return_value=at_limit,
        ) as paired, mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            return_value=10_200,
        ):
            result = stamp_node.fresh_paired_receipt_clocks(
                max_clock_read_span_ns=1_000_000,
                age_acquisition_attempts=3,
            )
        self.assertEqual(result, at_limit._replace(selected_pair_age_ns=200))
        paired.assert_called_once_with(3)

        with mock.patch.object(
            stamp_node,
            "paired_receipt_clocks",
            side_effect=(above_limit, fresh),
        ) as paired, mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            side_effect=(20_200, 30_200),
        ):
            result = stamp_node.fresh_paired_receipt_clocks(
                max_clock_read_span_ns=1_000_000,
                age_acquisition_attempts=3,
            )
        self.assertEqual(result, fresh._replace(selected_pair_age_ns=200))
        self.assertEqual(paired.call_args_list, [mock.call(3), mock.call(3)])

    def test_clock_pair_read_span_reacquisition_is_bounded(self) -> None:
        pairs = tuple(
            stamp_node.ReceiptClockPair(
                realtime_ns=index,
                monotonic_ns=index * 10_000,
                read_span_ns=1_000_000 + index,
                selected_pair_age_ns=100,
            )
            for index in range(1, 4)
        )
        with mock.patch.object(
            stamp_node,
            "paired_receipt_clocks",
            side_effect=pairs,
        ) as paired, mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            side_effect=tuple(pair.monotonic_ns + 200 for pair in pairs),
        ), self.assertRaisesRegex(
            RuntimeError,
            "clock_pair_read_span_exceeded:span_ns=1000003:limit_ns=1000000",
        ):
            stamp_node.fresh_paired_receipt_clocks(
                max_clock_read_span_ns=1_000_000,
                age_acquisition_attempts=3,
            )

        self.assertEqual(paired.call_count, 3)

    def test_clock_pair_reacquisition_never_retries_hard_clock_faults(self) -> None:
        cases = (
            (
                "negative read span",
                stamp_node.ReceiptClockPair(1, 10_000, -1, 100),
                10_200,
                "clock_pair_read_span_exceeded",
            ),
            (
                "negative return age",
                stamp_node.ReceiptClockPair(1, 10_000, 100, -1),
                10_200,
                "clock_pair_age_exceeded",
            ),
            (
                "monotonic age regression",
                stamp_node.ReceiptClockPair(1, 10_000, 100, 200),
                10_100,
                "clock_pair_age_exceeded",
            ),
            (
                "monotonic regression with delayed read span",
                stamp_node.ReceiptClockPair(1, 10_000, 1_000_001, 200),
                10_100,
                "clock_pair_age_exceeded",
            ),
        )
        for name, pair, use_guard_ns, reason in cases:
            with self.subTest(name=name), mock.patch.object(
                stamp_node,
                "paired_receipt_clocks",
                return_value=pair,
            ) as paired, mock.patch.object(
                stamp_node.time,
                "monotonic_ns",
                return_value=use_guard_ns,
            ), self.assertRaisesRegex(RuntimeError, reason):
                stamp_node.fresh_paired_receipt_clocks(
                    max_clock_read_span_ns=1_000_000,
                    age_acquisition_attempts=(
                        stamp_node.NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS
                    ),
                )
            paired.assert_called_once_with(3)

    def test_clock_pair_age_acquisition_count_is_bounded(self) -> None:
        for invalid in (True, 0, 4):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(
                ValueError, "age acquisition attempts"
            ):
                stamp_node.fresh_paired_receipt_clocks(
                    max_clock_read_span_ns=1_000_000,
                    age_acquisition_attempts=invalid,
                )

    def test_motion_reacquires_a_fresh_pair_without_widening_limits(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="clock-pair-profile-test",
            fixed_local_minus_source_offset_ns=OFFSET,
            approved_affine_common_drift_ppm=13.0,
        )
        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(
            stamp_node, "require_strict_affine_approval"
        ):
            directory = Path(temporary)
            nomotion = stamp_node._Runtime(
                approval,
                directory / "nomotion-ready.json",
                directory / "nomotion-fault.json",
                mode="affine",
                profile="nomotion",
            )
            motion = stamp_node._Runtime(
                approval,
                directory / "motion-ready.json",
                directory / "motion-fault.json",
                mode="affine",
                profile="motion",
            )

        self.assertEqual(
            nomotion.clock_pair_age_acquisition_attempts,
            stamp_node.NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS,
        )
        self.assertEqual(
            motion.clock_pair_age_acquisition_attempts,
            stamp_node.MOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS,
        )
        self.assertEqual(
            nomotion.discipline.config.max_clock_read_span_ns,
            stamp_node.CLOCK_PAIR_HARD_LIMIT_NS,
        )
        self.assertEqual(
            motion.discipline.config.max_clock_read_span_ns,
            stamp_node.CLOCK_PAIR_HARD_LIMIT_NS,
        )

        stale = stamp_node.ReceiptClockPair(
            realtime_ns=1_000,
            monotonic_ns=10_000_000,
            read_span_ns=200,
            selected_pair_age_ns=1_044_070,
        )
        would_be_fresh = stamp_node.ReceiptClockPair(
            realtime_ns=2_000,
            monotonic_ns=20_000_000,
            read_span_ns=200,
            selected_pair_age_ns=100,
        )
        with mock.patch.object(
            stamp_node,
            "paired_receipt_clocks",
            side_effect=(stale, would_be_fresh),
        ) as paired, mock.patch.object(
            stamp_node.time,
            "monotonic_ns",
            side_effect=(11_049_481, 20_000_200),
        ):
            result = stamp_node.fresh_paired_receipt_clocks(
                max_clock_read_span_ns=(
                    motion.discipline.config.max_clock_read_span_ns
                ),
                age_acquisition_attempts=(
                    motion.clock_pair_age_acquisition_attempts
                ),
            )
        self.assertEqual(
            result,
            would_be_fresh._replace(selected_pair_age_ns=200),
        )
        self.assertEqual(paired.call_args_list, [mock.call(3), mock.call(3)])

    def test_clock_pair_retry_count_is_bounded(self) -> None:
        for invalid in (True, 0, 9):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(
                ValueError, "attempts"
            ):
                stamp_node.paired_receipt_clocks(invalid)

    def test_commit_observe_and_poll_only_use_fresh_clock_pair_api(self) -> None:
        tree = ast.parse(
            (TIME_SYNC / "workstation_nomotion_stamp_node.py").read_text(
                encoding="utf-8"
            )
        )
        calls = [
            node.func.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id
            in {"paired_receipt_clocks", "fresh_paired_receipt_clocks"}
        ]
        self.assertEqual(calls.count("paired_receipt_clocks"), 1)
        self.assertEqual(calls.count("fresh_paired_receipt_clocks"), 3)

        functions = {
            node.name: node
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        expected_retry_sources = {
            "maybe_lock": "self.clock_pair_age_acquisition_attempts",
            "_observe": "runtime.clock_pair_age_acquisition_attempts",
            "_poll": "runtime.clock_pair_age_acquisition_attempts",
        }
        for function_name, expected_source in expected_retry_sources.items():
            with self.subTest(function=function_name):
                function_calls = [
                    node
                    for node in ast.walk(functions[function_name])
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == "fresh_paired_receipt_clocks"
                ]
                self.assertEqual(len(function_calls), 1)
                keywords = {
                    keyword.arg: ast.unparse(keyword.value)
                    for keyword in function_calls[0].keywords
                }
                self.assertEqual(
                    keywords["age_acquisition_attempts"], expected_source
                )

    def test_phase_latency_evidence_retains_exact_last_and_maximum(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="phase-latency-test",
            fixed_local_minus_source_offset_ns=OFFSET,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )

            expected_zeroes = {
                name: 0 for name in stamp_node.PHASE_LATENCY_NAMES
            }
            self.assertEqual(runtime.phase_latency_last_ns, expected_zeroes)
            self.assertEqual(runtime.phase_latency_max_ns, expected_zeroes)

            runtime.record_phase_latency("discipline_observe", 41)
            runtime.record_phase_latency("discipline_observe", 17)
            runtime.record_phase_latency("message_copy", 23)

            self.assertEqual(runtime.phase_latency_last_ns["discipline_observe"], 17)
            self.assertEqual(runtime.phase_latency_max_ns["discipline_observe"], 41)
            self.assertEqual(runtime.phase_latency_last_ns["message_copy"], 23)
            self.assertEqual(runtime.phase_latency_max_ns["message_copy"], 23)
            self.assertEqual(set(runtime.phase_latency_last_ns), set(expected_zeroes))
            self.assertEqual(set(runtime.phase_latency_max_ns), set(expected_zeroes))

    def test_phase_latency_evidence_rejects_invalid_name_and_value(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="phase-latency-invalid-test",
            fixed_local_minus_source_offset_ns=OFFSET,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )
            for invalid_name in ("unknown", 1, None):
                with self.subTest(name=invalid_name), self.assertRaisesRegex(
                    ValueError, "phase"
                ):
                    runtime.record_phase_latency(invalid_name, 1)
            for invalid_value in (True, -1, 1.5, None):
                with self.subTest(value=invalid_value), self.assertRaisesRegex(
                    ValueError, "nonnegative integer"
                ):
                    runtime.record_phase_latency("publish", invalid_value)

            expected_zeroes = {
                name: 0 for name in stamp_node.PHASE_LATENCY_NAMES
            }
            self.assertEqual(runtime.phase_latency_last_ns, expected_zeroes)
            self.assertEqual(runtime.phase_latency_max_ns, expected_zeroes)

    def feed(self, discipline: NavigationStampDiscipline) -> int:
        delays = {
            "sport_primary": 5_000_000,
            "mid360_imu": 4_000_000,
            "mid360_cloud": 75_000_000,
            "mid360_odom": 6_000_000,
        }
        latest = MONO_BASE
        for index in range(121):
            source = SOURCE_BASE + index * 100_000_000
            jitter = (index % 5) * 100_000
            for stream, delay in delays.items():
                realtime = source + OFFSET + delay + jitter
                monotonic = MONO_BASE + index * 100_000_000 + delay + jitter
                latest = max(latest, monotonic)
                discipline.observe(
                    stream,
                    source,
                    realtime,
                    monotonic,
                    clock_domain=EXPECTED_CLOCK_DOMAIN,
                    clock_read_span_ns=10_000,
                )
        return latest

    def test_four_stream_profile_includes_private_lidar_odom_witness(self) -> None:
        config = go2_workstation_nomotion_config()
        policies = {policy.name: policy for policy in config.streams}
        self.assertEqual(
            set(policies),
            {"sport_primary", "mid360_imu", "mid360_cloud", "mid360_odom"},
        )
        self.assertEqual(
            policies["mid360_odom"].hard_age_ceiling_ns, 2_000_000_000
        )
        self.assertEqual(
            {policy.hard_age_ceiling_ns for policy in policies.values()},
            {2_000_000_000},
        )
        self.assertEqual(policies["mid360_odom"].max_corrected_age_ns, 200_000_000)
        self.assertEqual(policies["mid360_odom"].stale_receipt_timeout_ns, 500_000_000)
        self.assertEqual(
            policies["mid360_cloud"].max_source_receipt_delta_error_ns,
            150_000_000,
        )
        self.assertEqual(
            policies["mid360_imu"].max_source_receipt_delta_error_ns,
            150_000_000,
        )
        self.assertEqual(
            policies["mid360_odom"].max_source_receipt_delta_error_ns,
            150_000_000,
        )
        self.assertEqual(
            policies["sport_primary"].max_source_receipt_delta_error_ns,
            150_000_000,
        )
        self.assertEqual(
            policies["sport_primary"].max_corrected_age_ns, 150_000_000
        )
        self.assertEqual(
            policies["mid360_imu"].max_corrected_age_ns, 200_000_000
        )
        self.assertEqual(
            {policy.stale_receipt_timeout_ns for policy in policies.values()},
            {500_000_000},
        )
        self.assertEqual(config.max_source_receipt_delta_error_ns, 50_000_000)
        self.assertEqual(config.offset_guard_ns, 5_000_000)
        self.assertEqual(config.affine_anchor_past_guard_ns, 20_000_000)
        self.assertEqual(config.max_corrected_future_ns, 2_000_000)
        self.assertEqual(config.minimum_locked_corrected_age_ns, 5_000_000)
        self.assertEqual(config.max_pairwise_drift_ppm, 25.0)
        self.assertEqual(config.max_locked_affine_drift_deviation_ppm, 25.0)

        # The normal navigation/motion-capable profile is not widened.  It has
        # no lidar-odometry witness at all and retains the strict state/IMU
        # limits used by the guarded chassis path.
        motion_policies = {
            policy.name: policy for policy in go2_navigation_config().streams
        }
        self.assertNotIn("mid360_odom", motion_policies)
        self.assertEqual(
            motion_policies["sport_primary"].max_corrected_age_ns, 100_000_000
        )
        self.assertEqual(
            motion_policies["mid360_imu"].max_corrected_age_ns, 100_000_000
        )
        self.assertEqual(
            motion_policies["mid360_cloud"].max_corrected_age_ns, 250_000_000
        )
        self.assertIsNone(
            motion_policies["mid360_imu"].max_source_receipt_delta_error_ns
        )
        self.assertEqual(
            go2_navigation_config().max_source_receipt_delta_error_ns,
            50_000_000,
        )
        motion_config = go2_workstation_motion_config()
        self.assertEqual(motion_config.offset_guard_ns, 5_000_000)
        self.assertEqual(motion_config.affine_anchor_past_guard_ns, 20_000_000)
        self.assertEqual(motion_config.max_corrected_future_ns, 2_000_000)
        self.assertIsNone(motion_config.minimum_locked_corrected_age_ns)
        self.assertEqual(motion_config.max_pairwise_drift_ppm, 25.0)
        self.assertEqual(
            motion_config.max_locked_affine_drift_deviation_ppm,
            25.0,
        )
        self.assertFalse(motion_config.locked_statistics_enabled)
        self.assertTrue(
            go2_workstation_nomotion_config().locked_statistics_enabled
        )
        motion_profile_policies = {
            policy.name: policy for policy in motion_config.streams
        }
        self.assertEqual(
            {
                policy.stale_receipt_timeout_ns
                for policy in motion_profile_policies.values()
            },
            {200_000_000},
        )
        self.assertEqual(
            {
                policy.hard_age_ceiling_ns
                for policy in motion_profile_policies.values()
            },
            {200_000_000},
        )
        self.assertEqual(
            {
                policy.max_corrected_age_ns
                for policy in motion_profile_policies.values()
            },
            {200_000_000},
        )
        self.assertEqual(
            {
                policy.max_source_receipt_delta_error_ns
                for policy in motion_profile_policies.values()
            },
            {200_000_000},
        )
    def test_nomotion_receipt_recovery_is_500ms_bounded_and_motion_is_200ms(
        self,
    ) -> None:
        now_monotonic_ns = 2 * NS

        nomotion = NavigationStampDiscipline(go2_workstation_nomotion_config())
        for state in nomotion._streams.values():
            state.last_advancing_receipt_monotonic_ns = (
                now_monotonic_ns - 500_000_000
            )
        self.assertIsNone(nomotion._liveness_health(now_monotonic_ns))
        nomotion._streams[
            "sport_primary"
        ].last_advancing_receipt_monotonic_ns = (
            now_monotonic_ns - 500_000_001
        )
        self.assertEqual(
            nomotion._liveness_health(now_monotonic_ns),
            "stream_stale:sport_primary:receipt_age_ns=500000001:"
            "limit_ns=500000000",
        )

        motion = NavigationStampDiscipline(go2_workstation_motion_config())
        for state in motion._streams.values():
            state.last_advancing_receipt_monotonic_ns = (
                now_monotonic_ns - 200_000_000
            )
        self.assertIsNone(motion._liveness_health(now_monotonic_ns))
        motion._streams[
            "sport_primary"
        ].last_advancing_receipt_monotonic_ns = (
            now_monotonic_ns - 200_000_001
        )
        self.assertEqual(
            motion._liveness_health(now_monotonic_ns),
            "stream_stale:sport_primary:receipt_age_ns=200000001:"
            "limit_ns=200000000",
        )

    def test_motion_callback_delta_uses_shared_200ms_liveness_boundary(self) -> None:
        """The normal motion profile adds no tighter workstation-only gate."""

        source_delta_ns = 4_172_376
        measured_receipt_delta_ns = 55_134_506

        measured = NavigationStampDiscipline(
            go2_workstation_motion_config(EXPECTED_CLOCK_DOMAIN)
        )
        first = measured.observe(
            "sport_primary",
            SOURCE_BASE,
            SOURCE_BASE + OFFSET,
            MONO_BASE,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(first.reason, "offset_not_locked")
        observed = measured.observe(
            "sport_primary",
            SOURCE_BASE + source_delta_ns,
            SOURCE_BASE + OFFSET + measured_receipt_delta_ns,
            MONO_BASE + measured_receipt_delta_ns,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(observed.state, DisciplineState.QUALIFYING)
        self.assertEqual(observed.reason, "offset_not_locked")

        over_limit = NavigationStampDiscipline(
            go2_workstation_motion_config(EXPECTED_CLOCK_DOMAIN)
        )
        over_limit.observe(
            "sport_primary",
            SOURCE_BASE,
            SOURCE_BASE + OFFSET,
            MONO_BASE,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        rejected = over_limit.observe(
            "sport_primary",
            SOURCE_BASE + source_delta_ns,
            SOURCE_BASE + OFFSET + source_delta_ns + 200_000_001,
            MONO_BASE + source_delta_ns + 200_000_001,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(rejected.state, DisciplineState.FAULTED)
        self.assertEqual(
            rejected.reason,
            "source_receipt_delta_discontinuity:sport_primary",
        )
        self.assertEqual(
            dict(rejected.diagnostics),
            {
                "source_delta_ns": source_delta_ns,
                "receipt_delta_ns": source_delta_ns + 200_000_001,
                "delta_error_ns": 200_000_001,
                "delta_error_limit_ns": 200_000_000,
            },
        )

    def test_physical_361ms_executor_stall_replays_only_in_nomotion(self) -> None:
        now_monotonic_ns = 1_000_000_000
        observed_receipt_ages_ns = {
            "sport_primary": 368_399_077,
            "mid360_imu": 366_326_991,
            "mid360_cloud": 373_264_429,
            "mid360_odom": 364_314_200,
        }

        nomotion = NavigationStampDiscipline(go2_workstation_nomotion_config())
        for name, state in nomotion._streams.items():
            state.last_advancing_receipt_monotonic_ns = (
                now_monotonic_ns - observed_receipt_ages_ns[name]
            )
        self.assertIsNone(nomotion._liveness_health(now_monotonic_ns))

        motion = NavigationStampDiscipline(go2_workstation_motion_config())
        for name, state in motion._streams.items():
            state.last_advancing_receipt_monotonic_ns = (
                now_monotonic_ns - observed_receipt_ages_ns[name]
            )
        self.assertEqual(
            motion._liveness_health(now_monotonic_ns),
            "stream_stale:sport_primary:receipt_age_ns=368399077:"
            "limit_ns=200000000",
        )

    def test_nomotion_imu_recovery_is_bounded_without_widening_motion(self) -> None:
        """A fresh recovered IMU sample may follow one delayed callback."""

        nomotion = NavigationStampDiscipline(self.config())
        latest = self.feed(nomotion)
        nomotion.lock_fixed(
            OFFSET, identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = nomotion._streams["mid360_imu"]
        assert state.last_source_ns is not None
        assert state.last_advancing_receipt_monotonic_ns is not None

        # The previous feed sample was about 4.4 ms old.  This current sample
        # is 70 ms old, so the consecutive source/receipt deltas disagree by
        # about 65.6 ms.  It is still fresh under the unchanged 100 ms IMU-age
        # ceiling and is accepted only by the private no-motion relay profile.
        source = state.last_source_ns + 100_000_000
        receipt_monotonic = (
            state.last_advancing_receipt_monotonic_ns + 165_600_000
        )
        recovered = nomotion.observe(
            "mid360_imu",
            source,
            source + OFFSET + 70_000_000,
            receipt_monotonic,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(recovered.accepted, recovered.reason)
        self.assertEqual(recovered.reason, "navigation_safe")

        # The normal navigation/motion-capable profile still latches the same
        # 65.6 ms transition against its unmodified strict 50 ms breaker.
        motion = NavigationStampDiscipline(
            go2_navigation_config(EXPECTED_CLOCK_DOMAIN)
        )
        first = motion.observe(
            "mid360_imu",
            SOURCE_BASE,
            SOURCE_BASE + OFFSET + 4_400_000,
            MONO_BASE + 4_400_000,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(first.state, DisciplineState.QUALIFYING)
        rejected = motion.observe(
            "mid360_imu",
            SOURCE_BASE + 100_000_000,
            SOURCE_BASE + 100_000_000 + OFFSET + 70_000_000,
            MONO_BASE + 170_000_000,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(rejected.state, DisciplineState.FAULTED)
        self.assertEqual(
            rejected.reason,
            "source_receipt_delta_discontinuity:mid360_imu",
        )

    def test_nomotion_imu_age_uses_hard_ceiling_without_widening_motion(self) -> None:
        """Only the private no-motion relay accepts a 150 ms IMU sample."""

        nomotion = NavigationStampDiscipline(self.config())
        latest = self.feed(nomotion)
        nomotion.lock_fixed(
            OFFSET, identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = nomotion._streams["mid360_imu"]
        assert state.last_source_ns is not None
        assert state.last_advancing_receipt_monotonic_ns is not None
        source = state.last_source_ns + 100_000_000
        receipt_monotonic = source + OFFSET + 150_000_000 - CLOCK_BASE
        # Model the other three streams continuing normally while this one
        # IMU callback waited behind full-stack startup work.
        for name, peer in nomotion._streams.items():
            if name != "mid360_imu":
                peer.last_advancing_receipt_monotonic_ns = receipt_monotonic
        accepted = nomotion.observe(
            "mid360_imu",
            source,
            source + OFFSET + 150_000_000,
            receipt_monotonic,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(accepted.accepted, accepted.reason)

        motion = NavigationStampDiscipline(
            go2_navigation_config(EXPECTED_CLOCK_DOMAIN)
        )
        motion.locked_offset_ns = OFFSET
        motion.state = DisciplineState.LOCKED
        stale = motion.observe(
            "mid360_imu",
            SOURCE_BASE,
            SOURCE_BASE + OFFSET + 150_000_000,
            MONO_BASE + 150_000_000,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertFalse(stale.accepted)
        self.assertEqual(
            stale.reason,
            "corrected_timestamp_stale:mid360_imu:"
            "corrected_age_ns=150000000:limit_ns=100000000",
        )

    def test_nomotion_sport_age_and_motion_cold_start_boundaries(self) -> None:
        """No-motion keeps 150 ms; motion uses the shared 200 ms liveness bound."""

        nomotion = NavigationStampDiscipline(self.config())
        latest = self.feed(nomotion)
        nomotion.lock_fixed(
            OFFSET, identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = nomotion._streams["sport_primary"]
        assert state.last_source_ns is not None
        assert state.last_advancing_receipt_monotonic_ns is not None
        source = state.last_source_ns + 100_000_000
        observed_startup_age_ns = 106_436_812
        receipt_monotonic = (
            source + OFFSET + observed_startup_age_ns - CLOCK_BASE
        )
        for name, peer in nomotion._streams.items():
            if name != "sport_primary":
                peer.last_advancing_receipt_monotonic_ns = receipt_monotonic
        accepted = nomotion.observe(
            "sport_primary",
            source,
            source + OFFSET + observed_startup_age_ns,
            receipt_monotonic,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(accepted.accepted, accepted.reason)

        boundary_source = source + 100_000_000
        boundary_age_ns = 150_000_000
        boundary_receipt_monotonic = (
            boundary_source + OFFSET + boundary_age_ns - CLOCK_BASE
        )
        boundary = nomotion.observe(
            "sport_primary",
            boundary_source,
            boundary_source + OFFSET + boundary_age_ns,
            boundary_receipt_monotonic,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(boundary.accepted, boundary.reason)

        motion = NavigationStampDiscipline(
            go2_workstation_motion_config(EXPECTED_CLOCK_DOMAIN)
        )
        motion.locked_offset_ns = OFFSET
        motion.state = DisciplineState.LOCKED
        for name, peer in motion._streams.items():
            if name != "sport_primary":
                peer.last_advancing_receipt_monotonic_ns = (
                    MONO_BASE + observed_startup_age_ns
                )
        motion._last_statistics_evaluation_monotonic_ns = (
            MONO_BASE + observed_startup_age_ns
        )
        startup = motion.observe(
            "sport_primary",
            SOURCE_BASE,
            SOURCE_BASE + OFFSET + observed_startup_age_ns,
            MONO_BASE + observed_startup_age_ns,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(startup.accepted, startup.reason)

        motion_boundary = NavigationStampDiscipline(
            go2_workstation_motion_config(EXPECTED_CLOCK_DOMAIN)
        )
        motion_boundary.locked_offset_ns = OFFSET
        motion_boundary.state = DisciplineState.LOCKED
        for name, peer in motion_boundary._streams.items():
            if name != "sport_primary":
                peer.last_advancing_receipt_monotonic_ns = (
                    MONO_BASE + 200_000_000
                )
        motion_boundary._last_statistics_evaluation_monotonic_ns = (
            MONO_BASE + 200_000_000
        )
        boundary = motion_boundary.observe(
            "sport_primary",
            SOURCE_BASE,
            SOURCE_BASE + OFFSET + 200_000_000,
            MONO_BASE + 200_000_000,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(boundary.accepted, boundary.reason)

        motion_fault = NavigationStampDiscipline(
            go2_workstation_motion_config(EXPECTED_CLOCK_DOMAIN)
        )
        motion_fault.locked_offset_ns = OFFSET
        motion_fault.state = DisciplineState.LOCKED
        for name, peer in motion_fault._streams.items():
            if name != "sport_primary":
                peer.last_advancing_receipt_monotonic_ns = (
                    MONO_BASE + 200_000_001
                )
        motion_fault._last_statistics_evaluation_monotonic_ns = (
            MONO_BASE + 200_000_001
        )
        stale = motion_fault.observe(
            "sport_primary",
            SOURCE_BASE,
            SOURCE_BASE + OFFSET + 200_000_001,
            MONO_BASE + 200_000_001,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertFalse(stale.accepted)
        self.assertEqual(
            stale.reason,
            "corrected_timestamp_stale:sport_primary:"
            "corrected_age_ns=200000001:limit_ns=200000000",
        )

        no_motion_limit_fault = NavigationStampDiscipline(self.config())
        no_motion_limit_fault.locked_offset_ns = OFFSET
        no_motion_limit_fault.state = DisciplineState.LOCKED
        too_old = no_motion_limit_fault.observe(
            "sport_primary",
            SOURCE_BASE,
            SOURCE_BASE + OFFSET + 150_000_001,
            MONO_BASE + 150_000_001,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertFalse(too_old.accepted)
        self.assertEqual(
            too_old.reason,
            "corrected_timestamp_stale:sport_primary:"
            "corrected_age_ns=150000001:limit_ns=150000000",
        )

    def test_nomotion_cloud_age_retains_250ms_navigation_bound(self) -> None:
        discipline = NavigationStampDiscipline(self.config())
        latest = self.feed(discipline)
        discipline.lock_fixed(
            OFFSET, identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = discipline._streams["mid360_cloud"]
        assert state.last_source_ns is not None
        assert state.last_advancing_receipt_monotonic_ns is not None
        # Keep the separate 150 ms source/receipt-delta breaker satisfied.
        state.last_advancing_receipt_monotonic_ns = (
            state.last_source_ns + OFFSET + 110_000_000 - CLOCK_BASE
        )
        boundary_source = state.last_source_ns + 100_000_000
        boundary_age_ns = 250_000_000
        boundary_receipt_monotonic = (
            boundary_source + OFFSET + boundary_age_ns - CLOCK_BASE
        )
        for name, peer in discipline._streams.items():
            if name != "mid360_cloud":
                peer.last_advancing_receipt_monotonic_ns = (
                    boundary_receipt_monotonic
                )
        boundary = discipline.observe(
            "mid360_cloud",
            boundary_source,
            boundary_source + OFFSET + boundary_age_ns,
            boundary_receipt_monotonic,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(boundary.accepted, boundary.reason)

        stale_source = boundary_source + 100_000_000
        stale_age_ns = 250_000_001
        stale = discipline.observe(
            "mid360_cloud",
            stale_source,
            stale_source + OFFSET + stale_age_ns,
            boundary_receipt_monotonic + 100_000_001,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertFalse(stale.accepted)
        self.assertEqual(
            stale.reason,
            "corrected_timestamp_stale:mid360_cloud:"
            "corrected_age_ns=250000001:limit_ns=250000000",
        )

    def test_lidar_odom_callback_burst_is_bounded_by_freshness(self) -> None:
        discipline = NavigationStampDiscipline(self.config())
        source = SOURCE_BASE
        receipt_monotonic = MONO_BASE + 6_000_000
        first = discipline.observe(
            "mid360_odom",
            source,
            source + OFFSET + 6_000_000,
            receipt_monotonic,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(first.state, DisciplineState.QUALIFYING)

        # A valid 150 Hz source sample handled 70 ms later after another
        # callback occupied the single-threaded executor has a roughly 63 ms
        # source/receipt delta disagreement.  It remains a qualification
        # witness because it is inside the odometry stream's bounded relay
        # allowance; no corrected message is emitted before lock.
        queued = discipline.observe(
            "mid360_odom",
            source + 6_666_667,
            source + OFFSET + 76_000_000,
            receipt_monotonic + 70_000_000,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(queued.state, DisciplineState.QUALIFYING)
        self.assertFalse(queued.accepted)
        self.assertEqual(queued.reason, "offset_not_locked")

        # Full-stack startup may occupy the single-threaded private relay long
        # enough for one lidar-odometry callback to arrive 120 ms old.  The
        # workstation no-motion profile accepts it below the existing 200 ms
        # hard ceiling; it still cannot create canonical chassis odometry.
        locked = NavigationStampDiscipline(self.config())
        latest = self.feed(locked)
        locked.lock_fixed(
            OFFSET, identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = locked._streams["mid360_odom"]
        assert state.last_source_ns is not None
        assert state.last_advancing_receipt_monotonic_ns is not None
        next_source = state.last_source_ns + 6_666_667
        delayed_receipt = state.last_advancing_receipt_monotonic_ns + 120_000_000
        startup_burst = locked.observe(
            "mid360_odom",
            next_source,
            next_source + OFFSET + 120_000_000,
            delayed_receipt,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(startup_burst.accepted, startup_burst.reason)
        self.assertEqual(startup_burst.reason, "navigation_safe")

        # A subsequent sample beyond 200 ms still fails closed.  The chosen
        # deltas remain inside the independent 150 ms callback-scheduling
        # breaker so this specifically exercises the corrected-age ceiling.
        state = locked._streams["mid360_odom"]
        assert state.last_source_ns is not None
        assert state.last_advancing_receipt_monotonic_ns is not None
        stale_source = state.last_source_ns + 100_000_000
        stale_receipt = state.last_advancing_receipt_monotonic_ns + 181_000_000
        stale = locked.observe(
            "mid360_odom",
            stale_source,
            stale_source + OFFSET + 201_000_000,
            stale_receipt,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(stale.state, DisciplineState.FAULTED)
        self.assertEqual(
            stale.reason,
            "corrected_timestamp_stale:mid360_odom:"
            "corrected_age_ns=201000000:limit_ns=200000000",
        )

    def test_approved_offset_locks_exactly_and_never_adapts(self) -> None:
        discipline = NavigationStampDiscipline(self.config())
        latest = self.feed(discipline)
        locked = discipline.lock_fixed(
            OFFSET,
            identity_evidence_verified=True,
            now_monotonic_ns=latest,
        )
        self.assertEqual(locked, OFFSET)
        source = SOURCE_BASE + 121 * 100_000_000
        realtime = source + OFFSET + 5_000_000
        result = discipline.observe(
            "sport_primary",
            source,
            realtime,
            realtime - CLOCK_BASE,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertTrue(result.accepted, result.reason)
        self.assertEqual(result.corrected_stamp_ns, source + OFFSET)
        self.assertEqual(discipline.locked_offset_ns, OFFSET)

    def test_unapproved_offset_disconnect_and_clock_jump_fail_closed(self) -> None:
        disagreement = NavigationStampDiscipline(self.config())
        latest = self.feed(disagreement)
        with self.assertRaisesRegex(RuntimeError, "disagrees"):
            disagreement.lock_fixed(
                OFFSET + 30_000_000,
                identity_evidence_verified=True,
                now_monotonic_ns=latest,
            )

        disconnected = NavigationStampDiscipline(self.config())
        latest = self.feed(disconnected)
        disconnected.lock_fixed(
            OFFSET, identity_evidence_verified=True, now_monotonic_ns=latest
        )
        result = disconnected.poll(latest + 600_000_000)
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertTrue(result.reason.startswith("stream_stale:"), result.reason)

        jumped = NavigationStampDiscipline(self.config())
        latest = self.feed(jumped)
        jumped.lock_fixed(
            OFFSET, identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = jumped._streams["sport_primary"]
        assert state.last_source_ns is not None
        source = state.last_source_ns + 100_000_000
        realtime = source + OFFSET + 105_000_000
        result = jumped.observe(
            "sport_primary",
            source,
            realtime,
            source + OFFSET + 5_000_000 - CLOCK_BASE,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
        )
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(result.reason, "local_realtime_discontinuity")

    def test_post_lock_duplicate_fraction_latches_fault(self) -> None:
        discipline = NavigationStampDiscipline(self.config())
        latest = self.feed(discipline)
        discipline.lock_fixed(
            OFFSET, identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = discipline._streams["sport_primary"]
        assert state.last_source_ns is not None
        for index in range(20):
            monotonic = latest + (index + 1) * 100_000
            result = discipline.observe(
                "sport_primary",
                state.last_source_ns,
                monotonic + CLOCK_BASE,
                monotonic,
                clock_domain=EXPECTED_CLOCK_DOMAIN,
            )
            self.assertEqual(result.state, DisciplineState.LOCKED)
        result = discipline.poll(latest + 20_000_000)
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(result.reason, "duplicate_fraction_exceeded:sport_primary")

    def test_stamp_arithmetic_uses_source_plus_fixed_offset(self) -> None:
        sec, nanosec = corrected_stamp_parts(10 * NS + 123, 748 * NS + 456)
        self.assertEqual((sec, nanosec), (758, 579))

    def test_sport_copy_preserves_error_code_and_does_not_mutate_raw(self) -> None:
        raw = SimpleNamespace(
            stamp=SimpleNamespace(sec=10, nanosec=20),
            error_code=100,
            mode=7,
            position=[1.0, 2.0, 3.0],
        )
        corrected = corrected_message_copy(raw, "sport_primary", 50 * NS + 30)
        self.assertIsNot(corrected, raw)
        self.assertEqual((raw.stamp.sec, raw.stamp.nanosec), (10, 20))
        self.assertEqual((corrected.stamp.sec, corrected.stamp.nanosec), (50, 30))
        self.assertEqual(corrected.error_code, 100)
        self.assertEqual(corrected.mode, 7)
        self.assertEqual(corrected.position, [1.0, 2.0, 3.0])

    def test_nomotion_cloud_copy_shares_only_read_only_payload(self) -> None:
        payload = bytearray(b"point-cloud-payload")
        raw = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=10, nanosec=20),
                frame_id="utlidar_lidar",
            ),
            data=payload,
            width=123,
            height=1,
        )
        original_payload = bytes(payload)

        corrected = corrected_message_copy(raw, "mid360_cloud", 50 * NS + 30)

        self.assertIsNot(corrected, raw)
        self.assertIsNot(corrected.header, raw.header)
        self.assertIsNot(corrected.header.stamp, raw.header.stamp)
        self.assertIs(corrected.data, raw.data)
        self.assertEqual(bytes(raw.data), original_payload)
        self.assertEqual((raw.header.stamp.sec, raw.header.stamp.nanosec), (10, 20))
        self.assertEqual(
            (corrected.header.stamp.sec, corrected.header.stamp.nanosec),
            (50, 30),
        )
        self.assertEqual(corrected.header.frame_id, "utlidar_lidar")

    def test_motion_static_stream_set_has_no_cloud(self) -> None:
        self.assertNotIn("mid360_cloud", stamp_node.MOTION_CORRECTED_TOPICS)


class RouteAndStaticSafetyTests(unittest.TestCase):
    def test_nomotion_launcher_affine_preflight_precedes_endpoint_checks(self) -> None:
        source = (ROOT / "scripts/start_workstation_full_nomotion_corrected.sh").read_text(
            encoding="utf-8"
        )
        self.assertLess(
            source.index("--require-affine"),
            source.index("check_workstation_nomotion_tcp_ports.py"),
        )
        self.assertIn("deadline=$((SECONDS + 90))", source)
        deadline = source.index("deadline=$((SECONDS + 90))")
        wait_loop = source.index("while true; do", deadline)
        loop_end = source.index("\ndone", wait_loop)
        loop = source[wait_loop:loop_end]
        fault_check = source.index('[[ -s "$FAULT_FILE" ]]', wait_loop)
        ready_accept = source.index('[[ ! -s "$READY_FILE" ]] || break', wait_loop)
        timeout_check = source.index('SECONDS >= deadline', wait_loop)
        self.assertLess(fault_check, ready_accept)
        self.assertLess(fault_check, timeout_check)
        self.assertIn('[[ -s "$IDENTITY_FAULT_FILE" ]]', loop)
        self.assertIn(
            "timestamp qualification did not become ready within 90 seconds",
            source,
        )

    def test_nomotion_stack_is_lower_priority_than_timestamp_sentinels(self) -> None:
        source = (
            ROOT / "scripts/start_workstation_full_nomotion_corrected.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("readonly STACK_NICE_LEVEL=5", source)
        stack_child = source[source.index('export ROBONIX_MANIFEST="$MANIFEST"') :]
        self.assertLess(
            stack_child.index('renice -n "$STACK_NICE_LEVEL" -p "$BASHPID"'),
            stack_child.index('exec bash "$ROOT/start.sh"'),
        )
        monitor_prefix = source[: source.index('export ROBONIX_MANIFEST="$MANIFEST"')]
        self.assertNotIn("STACK_NICE_LEVEL", monitor_prefix.split("readonly STACK_NICE_LEVEL=5", 1)[1])

    def test_nomotion_nav2_cleanup_is_exact_private_and_id_bound(self) -> None:
        source = (
            ROOT / "scripts/start_workstation_full_nomotion_corrected.sh"
        ).read_text(encoding="utf-8")
        cleanup_start = source.index("cleanup_nomotion_nav2_container() {")
        cleanup_end = source.index("\n}\n", cleanup_start) + len("\n}\n")
        cleanup = source[cleanup_start:cleanup_end]

        self.assertIn(
            'NAV2_STOPPER="$ROOT/third_party/service-navigation-rbnx/scripts/stop.sh"',
            source,
        )
        self.assertIn("readonly NAV2_LEGACY_CONTAINER_NAME=robonix_nav2", source)
        self.assertIn("readonly NAV2_IMAGE_NAME=robonix-nav2", source)
        self.assertIn(
            'NAV2_SESSION_CONTAINER_NAME="robonix_nav2_nomotion_${SESSION_TOKEN//-/}"',
            source,
        )
        for field in (
            "{{.Id}}",
            "{{.Name}}",
            "{{.Config.Image}}",
            "{{.Image}}",
            "{{.HostConfig.AutoRemove}}",
            "{{.HostConfig.NetworkMode}}",
            "{{.HostConfig.IpcMode}}",
            "{{.HostConfig.RestartPolicy.Name}}",
            "{{.HostConfig.Privileged}}",
            "{{.HostConfig.PidMode}}",
        ):
            self.assertIn(field, cleanup)
        self.assertIn("^[[:xdigit:]]{64}$", cleanup)
        self.assertIn('"true|host|host|no|false|"', cleanup)
        self.assertIn("RBNX_INVOCATION_CWD=*", cleanup)
        self.assertIn("ROBONIX_PKG_HOST_DIR=*", cleanup)
        self.assertIn("ROBONIX_VELOCITY_OUTPUT_TOPIC=*", cleanup)
        self.assertIn("ROBONIX_CAPABILITY_ID=*", cleanup)
        self.assertIn(
            '"$package_host" == "$ROOT/third_party/service-navigation-rbnx"',
            cleanup,
        )
        self.assertIn(
            '"$velocity_output" == /robonix/nomotion/cmd_vel',
            cleanup,
        )
        self.assertIn('"$capability_id" == nav2', cleanup)
        self.assertIn('"$run_parent" == "$ROOT/rbnx-build/run"', cleanup)
        self.assertIn(
            "^workstation-nomotion-stamp\\.[[:alnum:]]{6}$",
            cleanup,
        )
        self.assertIn(
            'go2_nomotion_private_directory "$invocation_cwd" "$(id -u)"',
            cleanup,
        )
        self.assertIn(
            'go2_nomotion_private_regular_file "$session_file" "$(id -u)"',
            cleanup,
        )
        self.assertIn('"$canonical_run_dir" == "$invocation_cwd"', cleanup)
        self.assertIn(
            '"$invocation_cwd" == "$expected_run_dir"',
            cleanup,
        )
        self.assertIn("ROBONIX_NAV2_FORCE=docker", cleanup)
        self.assertIn('ROBONIX_NAV2_CONTAINER="$container_id"', cleanup)
        self.assertNotIn(
            'ROBONIX_NAV2_CONTAINER="$NAV2_LEGACY_CONTAINER_NAME"',
            cleanup,
        )
        self.assertNotIn(
            'ROBONIX_NAV2_CONTAINER="$NAV2_SESSION_CONTAINER_NAME"',
            cleanup,
        )
        self.assertIn('bash "$NAV2_STOPPER"', cleanup)
        self.assertIn('--filter "id=$container_id"', cleanup)
        self.assertIn('--filter "name=^/${container_name}$"', cleanup)

        main_start = (ROOT / "start.sh").read_text(encoding="utf-8")
        self.assertIn(
            'INHERITED_NOMOTION_NAV2_CONTAINER="${ROBONIX_NAV2_CONTAINER:-}"',
            main_start,
        )
        self.assertIn(
            'export ROBONIX_NAV2_CONTAINER="$INHERITED_NOMOTION_NAV2_CONTAINER"',
            main_start,
        )

    def test_nomotion_nav2_cleanup_precedes_sentinels_and_has_no_broad_mutation(
        self,
    ) -> None:
        source = (
            ROOT / "scripts/start_workstation_full_nomotion_corrected.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "for command in basename chmod cmp dirname docker env flock git",
            source,
        )
        cleanup_definition = source.index("cleanup_stale_nomotion_nav2_containers() {")
        cleanup_definition_end = source.index("\n}\n", cleanup_definition)
        startup_cleanup = source.index(
            "\ncleanup_stale_nomotion_nav2_containers\n",
            cleanup_definition_end,
        )
        self.assertLess(source.index("trap on_exit EXIT"), startup_cleanup)
        self.assertLess(
            startup_cleanup,
            source.index("workstation_nomotion_identity_monitor.py"),
        )
        self.assertLess(
            startup_cleanup,
            source.index("workstation_nomotion_stamp_node.py"),
        )

        stop_start = source.index("stop_owned_children() {")
        stop_end = source.index("\n}\n", stop_start)
        stop = source[stop_start:stop_end]
        self.assertLess(
            stop.index('wait "$STACK_PID"'),
            stop.index(
                'cleanup_nomotion_nav2_container \\\n'
                '    "$NAV2_SESSION_CONTAINER_NAME" "$RUN_DIR"'
            ),
        )
        self.assertLess(
            stop.index('"$NAV2_SESSION_CONTAINER_NAME" "$RUN_DIR"'),
            stop.index('kill -TERM "$STAMP_PID"'),
        )
        self.assertLess(
            stop.index('kill -TERM "$STAMP_PID"'),
            stop.index("cleanup_owned_session_pointer"),
        )
        for forbidden in (
            "docker rm",
            "docker container rm",
            "docker stop",
            "docker kill",
            "pkill",
            "killall",
            'kill -- "-$$"',
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)

    def test_nomotion_ready_and_fault_simultaneously_visible_is_fault_first(
        self,
    ) -> None:
        source = (
            ROOT / "scripts/start_workstation_full_nomotion_corrected.sh"
        ).read_text(encoding="utf-8")
        cases = (
            (
                "$IDENTITY_READY_FILE",
                '[[ -s "$IDENTITY_FAULT_FILE" ]]',
            ),
            ("$READY_FILE", '[[ -s "$FAULT_FILE" ]]'),
        )
        cursor = 0
        for ready, fault in cases:
            with self.subTest(ready=ready):
                loop_start = source.index("while true; do", cursor)
                loop_end = source.index("\ndone", loop_start)
                loop = source[loop_start:loop_end]
                self.assertLess(
                    loop.index(fault),
                    loop.index(f'[[ ! -s "{ready}" ]] || break'),
                    "simultaneously visible READY+FAULT must take FAULT",
                )
                cursor = loop_end + len("\ndone")

    def setUp(self) -> None:
        self.base = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        self.rendered = render(
            self.base,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )

    @staticmethod
    def named(manifest: dict, section: str, name: str) -> dict:
        return next(item for item in manifest[section] if item["name"] == name)

    def test_rendered_route_is_structurally_nomotion_and_private(self) -> None:
        chassis = self.named(self.rendered, "primitive", "go2_chassis")["config"]
        sensors = self.named(self.rendered, "primitive", "go2_sensors")["config"]
        mapping = self.named(self.rendered, "service", "mapping")["config"]
        nav = self.named(self.rendered, "service", "nav2")["config"]
        dashboard = self.named(
            self.rendered, "service", "go2_dashboard"
        )["config"]
        self.assertEqual(chassis["state_topic"], PRIVATE_SPORT)
        self.assertEqual(chassis["state_fallback_topic"], "")
        self.assertEqual(chassis["twist_in_topic"], SAFE_CHASSIS_INPUT)
        self.assertEqual(chassis["odom_source"], "external_verified")
        self.assertEqual(chassis["external_odom_topic"], PRIVATE_LIDAR_ODOM)
        self.assertEqual(chassis["odom_topic"], CANONICAL_ODOM)
        self.assertTrue(chassis["publish_odom_tf"])
        for key in UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS:
            self.assertNotIn(key, chassis)
        self.assertEqual(chassis["imu_topic"], PRIVATE_CHASSIS_IMU)
        self.assertFalse(chassis["allow_motion"])
        self.assertEqual(chassis["allowed_modes"], [255])
        self.assertEqual(chassis["allowed_state_markers"], [100, 1002])
        self.assertIs(
            chassis["allow_passive_state_marker_transitions"], True
        )
        self.assertEqual(sensors["lidar_input_topic"], PRIVATE_CLOUD)
        self.assertEqual(sensors["imu_input_topic"], PRIVATE_IMU)
        self.assertEqual(mapping["sensor_providers"]["odom"], "go2_chassis")
        self.assertEqual(mapping["rtabmap_inputs"], ["lidar", "imu", "odom"])
        self.assertEqual(mapping["params_file"], MAPPING_PARAMS_FILE)
        self.assertEqual(nav["params_file"], NAV2_PARAMS_FILE)
        self.assertEqual(nav["bt_xml_file"], NAV2_BT_XML_FILE)
        self.assertEqual(
            nav["bt_through_poses_xml_file"],
            NAV2_BT_THROUGH_POSES_XML_FILE,
        )
        self.assertEqual(nav["velocity_output_topic"], SAFE_VELOCITY_OUTPUT)
        self.assertIs(nav["use_composition"], True)
        self.assertEqual(nav["provider_ids"]["odom"], "go2_chassis")
        self.assertEqual(dashboard["odom_topic"], CANONICAL_ODOM)
        self.assertEqual(dashboard["pose_topic"], "/robonix/map/pose")
        self.assertEqual(
            self.rendered["env"]["GO2_TIMESTAMP_PRIVATE_LIDAR_ODOM"],
            PRIVATE_LIDAR_ODOM,
        )
        self.assertEqual(self.rendered["env"]["ROBONIX_FORCE_CPU"], "1")
        self.assertEqual(self.rendered["env"]["SCENE_CG_FORCE_CPU"], "1")

    def test_renderer_repairs_scene_gpu_requests_to_cpu_only(self) -> None:
        base = yaml.safe_load(yaml.safe_dump(self.base))
        base["env"]["ROBONIX_FORCE_CPU"] = "0"
        base["env"]["SCENE_CG_FORCE_CPU"] = ""

        rendered = render(
            base,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )

        self.assertEqual(rendered["env"]["ROBONIX_FORCE_CPU"], "1")
        self.assertEqual(rendered["env"]["SCENE_CG_FORCE_CPU"], "1")

    def test_renderer_strips_stale_unfinished_stationary_pose_hold(self) -> None:
        base = yaml.safe_load(yaml.safe_dump(self.base))
        chassis = self.named(base, "primitive", "go2_chassis")["config"]
        for key in UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS:
            chassis[key] = True if key == "stationary_pose_hold_enabled" else 9.0

        rendered = render(
            base,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )
        rendered_chassis = self.named(
            rendered, "primitive", "go2_chassis"
        )["config"]
        for key in UNFINISHED_STATIONARY_POSE_HOLD_CONFIG_KEYS:
            self.assertNotIn(key, rendered_chassis)

    def test_passive_marker_policy_requires_explicit_reviewed_set(self) -> None:
        invalid = (
            (100, []),
            (100, [100]),
            (100, [100, 100]),
            (1002, [100, 2010]),
            (100, [0, 100]),
        )
        for current, markers in invalid:
            with self.subTest(current=current, markers=markers), self.assertRaises(
                ManifestError
            ):
                render(
                    self.base,
                    state_marker=current,
                    passive_state_markers=markers,
                )

    def test_rendered_policy_cannot_be_silently_disabled_or_motion_enabled(
        self,
    ) -> None:
        passive_disabled = yaml.safe_load(yaml.safe_dump(self.rendered))
        chassis = self.named(
            passive_disabled, "primitive", "go2_chassis"
        )["config"]
        chassis["allow_passive_state_marker_transitions"] = False
        with self.assertRaisesRegex(ManifestError, "passive_marker_transitions"):
            validate_rendered(passive_disabled)

        motion_enabled = yaml.safe_load(yaml.safe_dump(self.rendered))
        self.named(motion_enabled, "primitive", "go2_chassis")["config"][
            "allow_motion"
        ] = True
        with self.assertRaisesRegex(ManifestError, "no_motion"):
            validate_rendered(motion_enabled)

        unfinished_stationary_hold = yaml.safe_load(
            yaml.safe_dump(self.rendered)
        )
        self.named(
            unfinished_stationary_hold, "primitive", "go2_chassis"
        )["config"]["stationary_pose_hold_enabled"] = True
        with self.assertRaisesRegex(
            ManifestError, "no_unfinished_stationary_pose_hold"
        ):
            validate_rendered(unfinished_stationary_hold)

    def test_runtime_manifest_configs_are_private_materialized_copies(self) -> None:
        self.assertEqual(MAPPING_PARAMS_FILE, "config/rtabmap_params.yaml")
        self.assertEqual(NAV2_PARAMS_FILE, "config/nav2_params_go2.yaml")
        self.assertEqual(NAV2_BT_XML_FILE, "config/navigate.xml")
        self.assertEqual(
            NAV2_BT_THROUGH_POSES_XML_FILE,
            "config/navigate_through_poses.xml",
        )
        for relative in (
            MAPPING_PARAMS_FILE,
            NAV2_PARAMS_FILE,
            NAV2_BT_XML_FILE,
            NAV2_BT_THROUGH_POSES_XML_FILE,
        ):
            source = ROOT / relative
            with self.subTest(relative=relative):
                self.assertTrue(source.is_file())
                self.assertFalse(source.is_symlink())

        launcher = (
            ROOT / "scripts" / "start_workstation_full_nomotion_corrected.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('install -d -m 700 -- "$RUNTIME_CONFIG_DIR"', launcher)
        self.assertIn('install -m 600 -- "$source" "$destination"', launcher)
        for filename in (
            "rtabmap_params.yaml",
            "nav2_params_go2.yaml",
            "navigate.xml",
            "navigate_through_poses.xml",
        ):
            self.assertIn(f'"$RUNTIME_CONFIG_DIR/{filename}"', launcher)

    def test_private_raw_topic_is_not_the_canonical_chassis_output(self) -> None:
        self.assertEqual(RAW_TOPICS["mid360_odom"], "/utlidar/robot_odom")
        self.assertEqual(CORRECTED_TOPICS["mid360_odom"], PRIVATE_LIDAR_ODOM)
        self.assertNotIn("fallback", RAW_TOPICS)
        self.assertNotIn("/odom", CORRECTED_TOPICS.values())

    def test_new_runtime_sources_have_no_command_or_clock_setting_surface(self) -> None:
        paths = (
            TIME_SYNC / "workstation_nomotion_stamp_node.py",
            TIME_SYNC / "workstation_nomotion_identity_monitor.py",
            ROOT / "scripts" / "start_workstation_full_nomotion_corrected.sh",
            TIME_SYNC / "workstation_nomotion_cloud_relay.py",
        )
        forbidden = (
            "SportClient",
            "create_client",
            "create_service",
            "clock_settime",
            "settimeofday",
            "/api/sport/request",
            "/lowcmd",
            '"/cmd_vel"',
            "ros2 topic pub",
            "sudo",
            "nmcli connection modify",
            "ip addr add",
        )
        for path in paths:
            source = path.read_text(encoding="utf-8")
            for token in forbidden:
                with self.subTest(path=path.name, token=token):
                    self.assertNotIn(token, source)

        identity_source = paths[1].read_text(encoding="utf-8")
        for token in ("create_subscription", "create_publisher", "create_action"):
            with self.subTest(path=paths[1].name, token=token):
                self.assertNotIn(token, identity_source)

        launcher_source = paths[2].read_text(encoding="utf-8")
        self.assertIn("workstation_nomotion_identity_monitor.py", launcher_source)
        self.assertIn("workstation_nomotion_cloud_relay.py", launcher_source)
        self.assertIn("--mode affine", launcher_source)
        self.assertIn("--profile nomotion", launcher_source)
        self.assertEqual(
            launcher_source.count("workstation_nomotion_stamp_node.py"),
            1,
        )
        self.assertNotIn(
            "recover_workstation_full_nomotion_corrected.sh",
            launcher_source,
        )
        self.assertIn('export GO2_ALLOWED_STATE_MARKERS=""', launcher_source)
        self.assertIn(
            'wait -n -p EXITED_PID "${WAIT_PIDS[@]}"',
            launcher_source,
        )
        self.assertIn(
            "the no-motion UI/map stack remains available in degraded mode",
            launcher_source,
        )

    def test_private_context_nodes_use_matching_explicit_executors(self) -> None:
        """Never regress to rclpy's global/default-context executor."""

        for filename in (
            "workstation_nomotion_identity_monitor.py",
            "workstation_nomotion_stamp_node.py",
            "workstation_nomotion_cloud_relay.py",
        ):
            source = (TIME_SYNC / filename).read_text(encoding="utf-8")
            with self.subTest(filename=filename):
                self.assertIn(
                    "from rclpy.executors import SingleThreadedExecutor", source
                )
                self.assertIn(
                    "executor = SingleThreadedExecutor(context=context)", source
                )
                self.assertIn("added = executor.add_node(node)", source)
                self.assertIn("executor.spin()", source)
                self.assertIn("executor.shutdown()", source)
                self.assertIn("if added and context.ok():", source)
                tree = ast.parse(source, filename=filename)
                global_spins = [
                    node
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "spin"
                    and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == "rclpy"
                ]
                self.assertEqual(global_spins, [])

    def test_startup_ownership_requires_one_canonical_odom_in_this_profile(self) -> None:
        checker = (ROOT / "scripts" / "check_runtime_ownership.py").read_text(
            encoding="utf-8"
        )
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        self.assertIn('("workstation-full-nomotion-corrected", "post")', checker)
        self.assertIn("(1, 1, 1, 1, 1, 1)", checker)
        self.assertIn("workstation-full-nomotion-corrected-v1", start)
        self.assertIn("export GO2_ALLOW_MOTION=false", start)

    def test_graph_identity_requires_one_exact_gid_per_stream(self) -> None:
        payload = approval_payload(time.time_ns())
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "approval.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            path.chmod(0o600)
            approval = load_approval(path)
        observations = {
            stream: [SimpleNamespace(endpoint_gid=list(bytes.fromhex(gid)))]
            for stream, gid in approval.writer_gids
        }
        self.assertEqual(identity_monitor.identity_failures(approval, observations), [])
        observations["mid360_odom"] = []
        self.assertEqual(
            identity_monitor.identity_failures(approval, observations),
            ["mid360_odom:publisher_count:0"],
        )
        observations["mid360_odom"] = [
            SimpleNamespace(endpoint_gid=list(bytes.fromhex("05" * 24)))
        ]
        self.assertEqual(
            identity_monitor.identity_failures(approval, observations),
            ["mid360_odom:writer_gid_mismatch"],
        )

    def test_runtime_identity_binds_current_unique_writers_after_reboot(self) -> None:
        current = {
            stream: f"{index + 10:02x}" * 24
            for index, stream in enumerate(identity_monitor.EXPECTED_RAW_TOPICS)
        }
        observations = {
            stream: [SimpleNamespace(endpoint_gid=list(bytes.fromhex(gid)))]
            for stream, gid in current.items()
        }
        observed, failures = identity_monitor.observed_unique_writer_gids(
            observations
        )
        self.assertEqual(failures, [])
        self.assertEqual(observed, current)

        historical = {
            stream: "01" * 24
            for stream in identity_monitor.EXPECTED_RAW_TOPICS
        }
        self.assertTrue(
            all(
                failure.endswith(":writer_gid_mismatch")
                for failure in identity_monitor.writer_identity_failures(
                    historical,
                    observations,
                )
            )
        )
        self.assertEqual(
            identity_monitor.writer_identity_failures(observed, observations),
            [],
        )

    def test_runtime_identity_rejects_ambiguous_current_writer_set(self) -> None:
        observations = {
            stream: [SimpleNamespace(endpoint_gid=list(bytes([index + 1]) * 24))]
            for index, stream in enumerate(identity_monitor.EXPECTED_RAW_TOPICS)
        }
        observations["mid360_cloud"].append(
            SimpleNamespace(endpoint_gid=list(b"x" * 24))
        )
        observed, failures = identity_monitor.observed_unique_writer_gids(
            observations
        )
        self.assertEqual(observed, {})
        self.assertEqual(failures, ["mid360_cloud:publisher_count:2"])

    def test_ready_identity_tolerates_only_zero_writer_rediscovery(self) -> None:
        self.assertTrue(
            identity_monitor.recoverable_ready_writer_absence(
                [
                    "sport_primary:publisher_count:0",
                    "mid360_cloud:publisher_count:0",
                ]
            )
        )
        for failures in (
            [],
            ["sport_primary:publisher_count:2"],
            ["sport_primary:writer_gid_mismatch"],
            ["graph_check_failed:RuntimeError:test"],
            ["unknown:publisher_count:0"],
        ):
            with self.subTest(failures=failures):
                self.assertFalse(
                    identity_monitor.recoverable_ready_writer_absence(
                        failures
                    )
                )

    def test_identity_session_revalidates_content_without_expiry_kill(self) -> None:
        now_ns = time.time_ns()
        payload = approval_payload(now_ns)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "approval.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            path.chmod(0o600)
            approval = load_approval(path, now_realtime_ns=now_ns)
            expired_ns = approval.expires_unix_ns + 1

            self.assertEqual(
                identity_monitor.load_session_approval(
                    path,
                    approval,
                    ready=True,
                    now_realtime_ns=expired_ns,
                ),
                approval,
            )
            self.assertEqual(
                identity_monitor.load_session_approval(
                    path,
                    approval,
                    ready=False,
                    now_realtime_ns=expired_ns,
                ),
                approval,
            )

    def test_stamp_approval_issue_time_is_checked_only_before_ready(self) -> None:
        source = (
            TIME_SYNC / "workstation_nomotion_stamp_node.py"
        ).read_text(encoding="utf-8")
        poll_start = source.index("        def _poll(self) -> None:")
        poll_end = source.index("\n    # This node uses an isolated Context", poll_start)
        poll = source[poll_start:poll_end]
        ready_gate = poll.index("if not runtime.ready:")
        approval_check = poll.index(
            "approval.require_valid_at(clocks.realtime_ns)"
        )
        discipline_poll = poll.index(
            "result = runtime.discipline.poll(clocks.monotonic_ns)"
        )
        self.assertLess(ready_gate, approval_check)
        self.assertLess(approval_check, discipline_poll)
        self.assertEqual(poll.count("approval.require_valid_at"), 1)

    def test_fault_status_write_failure_still_shuts_down(self) -> None:
        now_ns = time.time_ns()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval_path = directory / "approval.json"
            approval_path.write_text(
                json.dumps(approval_payload(now_ns)), encoding="utf-8"
            )
            approval_path.chmod(0o600)
            approval = load_approval(approval_path, now_realtime_ns=now_ns)
            runtime = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            with mock.patch.object(
                stamp_node, "_atomic_json", side_effect=OSError("disk full")
            ):
                with self.assertRaises(OSError):
                    runtime.latch_fault("test_fault")
        self.assertEqual(shutdowns, [True])

    def test_fault_payload_always_includes_bounded_phase_latency_evidence(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="phase-latency-fault-test",
            fixed_local_minus_source_offset_ns=OFFSET,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )
            runtime.record_phase_latency("discipline_observe", 31)
            runtime.record_phase_latency("discipline_observe", 19)
            runtime.record_phase_latency("publish", 7)
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)

            runtime.latch_fault("test_fault")

            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
            expected_last = {
                "discipline_observe": 19,
                "message_copy": 0,
                "publish": 7,
                "discipline_poll": 0,
            }
            expected_max = {
                "discipline_observe": 31,
                "message_copy": 0,
                "publish": 7,
                "discipline_poll": 0,
            }
            self.assertEqual(fault["phase_latency_last_ns"], expected_last)
            self.assertEqual(fault["phase_latency_max_ns"], expected_max)
            self.assertEqual(
                set(fault["phase_latency_last_ns"]),
                set(stamp_node.PHASE_LATENCY_NAMES),
            )
            self.assertEqual(
                set(fault["phase_latency_max_ns"]),
                set(stamp_node.PHASE_LATENCY_NAMES),
            )
            self.assertEqual(fault["noncore_delta_drop_count"], 0)
            self.assertIsNone(fault["last_noncore_delta_drop"])
            self.assertEqual(
                fault["timestamp_safety_limits"],
                {
                    "offset_guard_ns": 5_000_000,
                    "affine_anchor_past_guard_ns": 20_000_000,
                    "max_corrected_future_ns": 2_000_000,
                    "minimum_locked_corrected_age_ns": 5_000_000,
                    "max_pairwise_drift_ppm": 25.0,
                    "max_approved_affine_drift_deviation_ppm": 5.0,
                    "affine_qualification_window_ns": 30 * NS,
                    "max_affine_window_common_drift_deviation_ppm": 15.0,
                    "max_locked_affine_drift_deviation_ppm": 25.0,
                },
            )
            self.assertEqual(shutdowns, [True])

    def test_noncore_drop_warning_and_subsequent_fault_evidence_are_bounded(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval_path = directory / "approval.json"
            payload = approval_payload(time.time_ns())
            payload["session_id"] = "noncore-drop-evidence-test"
            approval_path.write_text(json.dumps(payload), encoding="utf-8")
            approval_path.chmod(0o600)
            approval = load_approval(approval_path, require_affine=True)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
                profile="nomotion",
            )
            first = CorrectionResult(
                False,
                None,
                False,
                f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud",
                DisciplineState.LOCKED,
                (
                    ("source_delta_ns", 64_474_582),
                    ("receipt_delta_ns", 288_418_372),
                    ("delta_error_ns", 223_943_790),
                    ("delta_error_limit_ns", 150_000_000),
                ),
            )
            second = replace(
                first,
                diagnostics=(
                    ("source_delta_ns", 60_000_000),
                    ("receipt_delta_ns", 220_000_001),
                    ("delta_error_ns", 160_000_001),
                    ("delta_error_limit_ns", 150_000_000),
                ),
            )
            with mock.patch("builtins.print") as warning:
                runtime.record_noncore_delta_drop(
                    "mid360_cloud", first, 10 * NS
                )
                runtime.record_noncore_delta_drop(
                    "mid360_cloud", second, 11 * NS
                )
            self.assertEqual(runtime.noncore_delta_drop_count, 2)
            self.assertEqual(warning.call_count, 2)
            self.assertIn(
                "WARNING discarded non-core timestamp sample",
                warning.call_args.args[0],
            )
            self.assertIn(stamp_node.NONCORE_DROP_SCHEMA, warning.call_args.args[0])
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)

            runtime.latch_fault("test_fault_after_noncore_drop")

            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
            self.assertEqual(fault["noncore_delta_drop_count"], 2)
            self.assertEqual(
                fault["last_noncore_delta_drop"],
                {
                    "stream": "mid360_cloud",
                    "reason": (
                        f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud"
                    ),
                    "receipt_monotonic_ns": 11 * NS,
                    "source_delta_ns": 60_000_000,
                    "receipt_delta_ns": 220_000_001,
                    "delta_error_ns": 160_000_001,
                    "delta_error_limit_ns": 150_000_000,
                },
            )
            self.assertEqual(shutdowns, [True])

    def test_qualifying_noncore_drop_warning_is_bounded_and_profile_isolated(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval_path = directory / "approval.json"
            payload = approval_payload(time.time_ns())
            payload["session_id"] = "qualifying-noncore-drop-test"
            approval_path.write_text(json.dumps(payload), encoding="utf-8")
            approval_path.chmod(0o600)
            approval = load_approval(approval_path, require_affine=True)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
                profile="nomotion",
            )
            result = CorrectionResult(
                False,
                None,
                False,
                f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud",
                DisciplineState.QUALIFYING,
                (
                    ("source_delta_ns", 194_688_081),
                    ("receipt_delta_ns", 347_867_528),
                    ("delta_error_ns", 153_179_447),
                    ("delta_error_limit_ns", 150_000_000),
                ),
            )
            with mock.patch("builtins.print") as warning:
                runtime.record_noncore_delta_drop(
                    "mid360_cloud", result, 10 * NS
                )
            self.assertEqual(runtime.noncore_delta_drop_count, 1)
            self.assertEqual(warning.call_count, 1)
            self.assertEqual(
                runtime.last_noncore_delta_drop,
                {
                    "stream": "mid360_cloud",
                    "reason": result.reason,
                    "receipt_monotonic_ns": 10 * NS,
                    **dict(result.diagnostics),
                },
            )

            over_ceiling = replace(
                result,
                diagnostics=(
                    ("source_delta_ns", 1),
                    ("receipt_delta_ns", 200_000_002),
                    ("delta_error_ns", 200_000_001),
                    ("delta_error_limit_ns", 150_000_000),
                ),
            )
            with self.assertRaisesRegex(ValueError, "exceeds ceilings"):
                runtime.record_noncore_delta_drop(
                    "mid360_cloud", over_ceiling, 11 * NS
                )

            motion = stamp_node._Runtime(
                approval,
                directory / "motion-ready.json",
                directory / "motion-fault.json",
                mode="affine",
                profile="motion",
            )
            with self.assertRaisesRegex(ValueError, "invalid non-core"):
                motion.record_noncore_delta_drop(
                    "mid360_cloud", result, 12 * NS
                )

    def test_ready_payload_retains_noncore_drop_evidence(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="ready-retains-noncore-drop-test",
            fixed_local_minus_source_offset_ns=OFFSET,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )
            runtime.noncore_delta_drop_count = 1
            runtime.last_noncore_delta_drop = {
                "stream": "mid360_cloud",
                "reason": (
                    f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud"
                ),
                "receipt_monotonic_ns": 10 * NS,
                "source_delta_ns": 194_688_081,
                "receipt_delta_ns": 347_867_528,
                "delta_error_ns": 153_179_447,
                "delta_error_limit_ns": 150_000_000,
            }
            with mock.patch.object(
                runtime.discipline,
                "qualification_reasons",
                return_value=[],
            ), mock.patch.object(
                runtime.discipline, "lock_fixed", return_value=OFFSET
            ):
                runtime.maybe_lock(10 * NS)

            ready = json.loads(
                runtime.ready_file.read_text(encoding="utf-8")
            )
            self.assertEqual(ready["noncore_delta_drop_count"], 1)
            self.assertEqual(
                ready["last_noncore_delta_drop"],
                runtime.last_noncore_delta_drop,
            )

    def test_stale_fault_records_trigger_age_limit_and_all_stream_ages(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="stale-observability-test",
            fixed_local_minus_source_offset_ns=OFFSET,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )
            fault_monotonic_ns = 10 * NS
            for state in runtime.discipline._streams.values():
                state.last_advancing_receipt_monotonic_ns = (
                    fault_monotonic_ns - 50_000_000
                )
            runtime.discipline._streams[
                "mid360_odom"
            ].last_advancing_receipt_monotonic_ns = (
                fault_monotonic_ns - 500_000_001
            )
            reason = runtime.discipline._liveness_health(fault_monotonic_ns)
            self.assertEqual(
                reason,
                "stream_stale:mid360_odom:receipt_age_ns=500000001:"
                "limit_ns=500000000",
            )
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            runtime.latch_fault(reason, fault_monotonic_ns=fault_monotonic_ns)

            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
            self.assertEqual(fault["schema"], stamp_node.FAULT_SCHEMA)
            self.assertEqual(fault["reason"], reason)
            self.assertEqual(fault["fault_monotonic_ns"], fault_monotonic_ns)
            self.assertEqual(fault["trigger_stream"], "mid360_odom")
            self.assertEqual(fault["trigger_receipt_age_ns"], 500_000_001)
            self.assertEqual(fault["trigger_receipt_age_limit_ns"], 500_000_000)
            self.assertEqual(
                set(fault["stream_receipt_liveness"]),
                set(runtime.discipline._streams),
            )
            self.assertEqual(
                fault["stream_receipt_liveness"]["sport_primary"][
                    "receipt_age_ns"
                ],
                50_000_000,
            )
            self.assertTrue(
                fault["stream_receipt_liveness"]["sport_primary"]["live"]
            )
            self.assertFalse(
                fault["stream_receipt_liveness"]["mid360_odom"]["live"]
            )
            self.assertEqual(shutdowns, [True])

    def test_source_receipt_fault_records_quantitative_diagnostics(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="source-receipt-diagnostic-test",
            fixed_local_minus_source_offset_ns=OFFSET,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            diagnostics = (
                ("source_delta_ns", 266_000_000),
                ("receipt_delta_ns", 65_000_000),
                ("delta_error_ns", 201_000_000),
                ("delta_error_limit_ns", 150_000_000),
            )
            runtime.latch_fault(
                "source_receipt_delta_discontinuity:mid360_cloud",
                fault_monotonic_ns=10 * NS,
                diagnostics=diagnostics,
            )

            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
            self.assertEqual(
                fault["reason"],
                "source_receipt_delta_discontinuity:mid360_cloud",
            )
            self.assertEqual(fault["fault_monotonic_ns"], 10 * NS)
            self.assertEqual(fault["trigger_stream"], "mid360_cloud")
            for key, value in diagnostics:
                self.assertEqual(fault[key], value)
            self.assertEqual(shutdowns, [True])

    def test_stale_diagnostic_failure_still_shuts_down(self) -> None:
        approval = SimpleNamespace(
            expected_clock_domain=EXPECTED_CLOCK_DOMAIN,
            session_id="stale-diagnostic-failure-test",
            fixed_local_minus_source_offset_ns=OFFSET,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            with mock.patch.object(
                runtime.discipline,
                "receipt_liveness_snapshot",
                side_effect=RuntimeError("diagnostic failure"),
            ):
                with self.assertRaisesRegex(RuntimeError, "diagnostic failure"):
                    runtime.latch_fault(
                        "stream_stale:mid360_odom:receipt_age_ns=201000000:"
                        "limit_ns=200000000",
                        fault_monotonic_ns=10 * NS,
                    )
            self.assertEqual(shutdowns, [True])


if __name__ == "__main__":
    unittest.main()
