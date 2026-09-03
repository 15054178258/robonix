from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deploy" / "time-sync"))

from navigation_stamp_discipline import (  # noqa: E402
    DisciplineConfig,
    DisciplineState,
    NavigationStampDiscipline,
    StreamPolicy,
    go2_navigation_config,
    ui_receipt_retimestamp,
)


NS = 1_000_000_000
DOMAIN = "unitree-main-computer@192.168.123.161"
BASE_SOURCE = 1_700_000_000 * NS
BASE_MONOTONIC = 100 * NS
OFFSET = 748 * NS
CLOCK_BASE = BASE_SOURCE + OFFSET - BASE_MONOTONIC


def compact_config(**overrides: object) -> DisciplineConfig:
    policies = (
        StreamPolicy(
            "sport_primary", 100_000_000, 200_000_000, 200_000_000, 10_000_000, 100
        ),
        StreamPolicy(
            "mid360_imu", 100_000_000, 200_000_000, 200_000_000, 20_000_000, 100
        ),
        StreamPolicy(
            "mid360_cloud", 250_000_000, 500_000_000, 500_000_000, 150_000_000, 100
        ),
    )
    values = dict(
        reference_stream="sport_primary",
        expected_clock_domain=DOMAIN,
        streams=policies,
        minimum_qualification_span_ns=10 * NS,
        statistics_evaluation_period_ns=20_000_000,
        retained_samples_per_stream=1000,
    )
    values.update(overrides)
    return DisciplineConfig(**values)


def feed_qualification(
    discipline: NavigationStampDiscipline,
    *,
    offset_ns: int = OFFSET,
    start_monotonic_ns: int = BASE_MONOTONIC,
    count: int = 121,
    interval_ns: int = 100_000_000,
) -> int:
    stream_delays = (
        ("mid360_imu", 1_500_000),
        ("sport_primary", 2_000_000),
        ("mid360_cloud", 75_000_000),
    )
    latest = start_monotonic_ns
    for index in range(count):
        source = BASE_SOURCE + index * interval_ns
        # Deterministic 0..0.4 ms delivery jitter, common clock rate.
        jitter = (index % 5) * 100_000
        for stream, delay in stream_delays:
            realtime = source + offset_ns + delay + jitter
            receipt_monotonic = (
                start_monotonic_ns + index * interval_ns + delay + jitter
            )
            latest = receipt_monotonic
            result = discipline.observe(
                stream,
                source,
                realtime,
                receipt_monotonic,
                clock_domain=DOMAIN,
                clock_read_span_ns=10_000,
            )
            assert not result.navigation_eligible
    return latest


class NavigationStampDisciplineTest(unittest.TestCase):
    def test_core_has_no_ros_network_process_or_clock_setting_surface(self) -> None:
        source = (
            ROOT / "deploy" / "time-sync" / "navigation_stamp_discipline.py"
        ).read_text(encoding="utf-8")
        for forbidden in (
            "import rclpy",
            "create_publisher",
            "create_subscription",
            "import socket",
            "import subprocess",
            "clock_settime",
            "settimeofday",
            "timedatectl",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "SportClient",
        ):
            self.assertNotIn(forbidden, source)

    def test_default_limits_stay_inside_existing_freshness_guards(self) -> None:
        config = go2_navigation_config()
        config.validate()
        policies = {policy.name: policy for policy in config.streams}
        self.assertEqual(policies["sport_primary"].hard_age_ceiling_ns, 200_000_000)
        self.assertEqual(policies["mid360_imu"].hard_age_ceiling_ns, 200_000_000)
        self.assertEqual(policies["mid360_cloud"].hard_age_ceiling_ns, 500_000_000)
        for policy in policies.values():
            self.assertLess(policy.max_corrected_age_ns, policy.hard_age_ceiling_ns)

    def test_single_offset_locks_explicitly_and_applies_to_all_streams(self) -> None:
        discipline = NavigationStampDiscipline(compact_config())
        latest = feed_qualification(discipline)
        with self.assertRaisesRegex(RuntimeError, "identity evidence"):
            discipline.lock(identity_evidence_verified=False, now_monotonic_ns=latest)
        offset = discipline.lock(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertGreater(offset, 747 * NS)
        self.assertLess(offset, 749 * NS)

        source = BASE_SOURCE + 121 * 100_000_000
        for stream, delay in (
            ("mid360_imu", 1_500_000),
            ("sport_primary", 2_000_000),
            ("mid360_cloud", 75_000_000),
        ):
            now_mono = source + OFFSET + delay - CLOCK_BASE
            result = discipline.observe(
                stream,
                source,
                source + OFFSET + delay,
                now_mono,
                clock_domain=DOMAIN,
            )
            self.assertTrue(result.accepted, result.reason)
            self.assertTrue(result.navigation_eligible)
            self.assertEqual(result.corrected_stamp_ns, source + offset)

    def test_ui_receipt_retimestamp_is_never_navigation_eligible(self) -> None:
        stamp = ui_receipt_retimestamp(123 * NS)
        self.assertEqual(stamp.stamp_ns, 123 * NS)
        self.assertFalse(stamp.navigation_eligible)
        self.assertIn("ui_only", stamp.provenance)

    def test_duplicate_does_not_refresh_liveness_or_fault_immediately(self) -> None:
        discipline = NavigationStampDiscipline(compact_config())
        latest = feed_qualification(discipline)
        discipline.lock(identity_evidence_verified=True, now_monotonic_ns=latest)
        state = discipline._streams["sport_primary"]
        last_source = state.last_source_ns
        last_receipt = state.last_advancing_receipt_monotonic_ns
        assert last_source is not None and last_receipt is not None
        duplicate = discipline.observe(
            "sport_primary",
            last_source,
            last_source + 748 * NS + 2_000_000,
            last_receipt + 10_000_000,
            clock_domain=DOMAIN,
        )
        self.assertFalse(duplicate.accepted)
        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertEqual(state.last_advancing_receipt_monotonic_ns, last_receipt)

    def test_regression_clock_step_and_domain_change_latch_fault(self) -> None:
        for mutation, expected in (
            ("regression", "source_timestamp_regression"),
            ("clock_step", "local_realtime_discontinuity"),
            ("domain", "clock_domain_mismatch"),
        ):
            discipline = NavigationStampDiscipline(compact_config())
            latest = feed_qualification(discipline)
            discipline.lock(identity_evidence_verified=True, now_monotonic_ns=latest)
            state = discipline._streams["sport_primary"]
            assert state.last_source_ns is not None
            source = state.last_source_ns + 100_000_000
            realtime = source + OFFSET + 2_000_000
            receipt_monotonic = realtime - CLOCK_BASE
            domain = DOMAIN
            if mutation == "regression":
                source = state.last_source_ns - 1
                realtime = source + OFFSET + 2_000_000
                receipt_monotonic = realtime - CLOCK_BASE
            elif mutation == "clock_step":
                realtime += 100_000_000
            else:
                domain = "unverified-other-clock"
            result = discipline.observe(
                "sport_primary",
                source,
                realtime,
                receipt_monotonic,
                clock_domain=domain,
            )
            self.assertEqual(result.state, DisciplineState.FAULTED)
            self.assertIn(expected, result.reason)
            again = discipline.observe(
                "sport_primary",
                state.last_source_ns + 200_000_000,
                state.last_source_ns + 200_000_000 + OFFSET,
                state.last_source_ns + 200_000_000 + OFFSET - CLOCK_BASE,
                clock_domain=DOMAIN,
            )
            self.assertFalse(again.accepted)
            self.assertEqual(again.state, DisciplineState.FAULTED)

    def test_excessive_stream_lag_refuses_lock(self) -> None:
        config = compact_config()
        discipline = NavigationStampDiscipline(config)
        latest = feed_qualification(discipline)
        cloud = discipline._streams["mid360_cloud"]
        cloud.samples = type(cloud.samples)(
            (
                (source, mono, age + 200_000_000)
                for source, mono, age in cloud.samples
            ),
            maxlen=cloud.retained_limit,
        )
        with self.assertRaisesRegex(RuntimeError, "stream_lag_exceeded"):
            discipline.lock(identity_evidence_verified=True, now_monotonic_ns=latest)

    def test_qualification_time_cannot_precede_latest_stream(self) -> None:
        discipline = NavigationStampDiscipline(compact_config())
        latest = feed_qualification(discipline)
        reasons = discipline.qualification_reasons(latest - 100_000_000)
        self.assertTrue(any(reason.startswith("stream_not_live:") for reason in reasons))

    def test_fixed_offset_is_not_adapted_and_deviation_faults(self) -> None:
        config = compact_config(max_locked_offset_deviation_ns=5_000_000)
        discipline = NavigationStampDiscipline(config)
        latest = feed_qualification(discipline)
        locked = discipline.lock(identity_evidence_verified=True, now_monotonic_ns=latest)
        state = discipline._streams["sport_primary"]
        state.samples = type(state.samples)(
            (
                (source, mono, age + 10_000_000)
                for source, mono, age in state.samples
            ),
            maxlen=state.retained_limit,
        )
        assert state.last_source_ns is not None
        source = state.last_source_ns + 100_000_000
        realtime = source + OFFSET + 12_000_000
        result = discipline.observe(
            "sport_primary",
            source,
            realtime,
            realtime - CLOCK_BASE,
            clock_domain=DOMAIN,
        )
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(result.reason, "locked_offset_deviation_exceeded")
        self.assertEqual(discipline.locked_offset_ns, locked)

    def test_missing_required_stream_latches_fault_after_its_deadline(self) -> None:
        discipline = NavigationStampDiscipline(compact_config())
        latest = feed_qualification(discipline)
        discipline.lock(identity_evidence_verified=True, now_monotonic_ns=latest)
        state = discipline._streams["sport_primary"]
        assert state.last_source_ns is not None
        source = state.last_source_ns + 600_000_000
        realtime = source + OFFSET + 2_000_000
        result = discipline.observe(
            "sport_primary",
            source,
            realtime,
            realtime - CLOCK_BASE,
            clock_domain=DOMAIN,
        )
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertTrue(result.reason.startswith("stream_stale:"), result.reason)

    def test_source_receipt_delta_jump_latches_fault(self) -> None:
        discipline = NavigationStampDiscipline(compact_config())
        latest = feed_qualification(discipline)
        discipline.lock(identity_evidence_verified=True, now_monotonic_ns=latest)
        state = discipline._streams["sport_primary"]
        assert state.last_source_ns is not None
        source = state.last_source_ns + NS
        receipt_monotonic = latest + 100_000_000
        result = discipline.observe(
            "sport_primary",
            source,
            CLOCK_BASE + receipt_monotonic,
            receipt_monotonic,
            clock_domain=DOMAIN,
        )
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertIn("source_receipt_delta_discontinuity", result.reason)
        self.assertEqual(
            dict(result.diagnostics),
            {
                "source_delta_ns": 1_000_000_000,
                "receipt_delta_ns": 173_000_000,
                "delta_error_ns": 827_000_000,
                "delta_error_limit_ns": 50_000_000,
            },
        )

    def test_per_stream_delta_limit_is_isolated_and_still_bounded(self) -> None:
        policies = tuple(
            replace(policy, max_source_receipt_delta_error_ns=150_000_000)
            if policy.name == "mid360_cloud"
            else policy
            for policy in compact_config().streams
        )

        cloud = NavigationStampDiscipline(compact_config(streams=policies))
        first = cloud.observe(
            "mid360_cloud",
            BASE_SOURCE,
            BASE_SOURCE + OFFSET + 75_000_000,
            BASE_MONOTONIC + 75_000_000,
            clock_domain=DOMAIN,
        )
        self.assertEqual(first.state, DisciplineState.QUALIFYING)
        # Two ~15 Hz samples delivered 1 ms apart after one callback was
        # queued differ by about 65.7 ms in source-vs-receipt deltas.  That is
        # above the strict 50 ms default but inside the cloud-only allowance.
        burst = cloud.observe(
            "mid360_cloud",
            BASE_SOURCE + 66_666_667,
            BASE_SOURCE + OFFSET + 76_000_000,
            BASE_MONOTONIC + 76_000_000,
            clock_domain=DOMAIN,
        )
        self.assertEqual(burst.state, DisciplineState.QUALIFYING)
        self.assertEqual(burst.reason, "offset_not_locked")

        state = NavigationStampDiscipline(compact_config(streams=policies))
        state.observe(
            "sport_primary",
            BASE_SOURCE,
            BASE_SOURCE + OFFSET + 2_000_000,
            BASE_MONOTONIC + 2_000_000,
            clock_domain=DOMAIN,
        )
        strict = state.observe(
            "sport_primary",
            BASE_SOURCE + 100_000_000,
            BASE_SOURCE + OFFSET + 202_000_000,
            BASE_MONOTONIC + 202_000_000,
            clock_domain=DOMAIN,
        )
        self.assertEqual(strict.state, DisciplineState.FAULTED)
        self.assertEqual(
            strict.reason, "source_receipt_delta_discontinuity:sport_primary"
        )

        exceeded = NavigationStampDiscipline(compact_config(streams=policies))
        exceeded.observe(
            "mid360_cloud",
            BASE_SOURCE,
            BASE_SOURCE + OFFSET + 75_000_000,
            BASE_MONOTONIC + 75_000_000,
            clock_domain=DOMAIN,
        )
        rejected = exceeded.observe(
            "mid360_cloud",
            BASE_SOURCE + 66_666_667,
            BASE_SOURCE + OFFSET + 295_000_000,
            BASE_MONOTONIC + 295_000_000,
            clock_domain=DOMAIN,
        )
        self.assertEqual(rejected.state, DisciplineState.FAULTED)
        self.assertEqual(
            rejected.reason, "source_receipt_delta_discontinuity:mid360_cloud"
        )

    def test_per_stream_delta_limit_cannot_exceed_safety_bounds(self) -> None:
        config = compact_config()
        for invalid in (True, 999_999, 250_000_001):
            with self.subTest(invalid=invalid):
                cloud = replace(
                    config.streams[2],
                    max_source_receipt_delta_error_ns=invalid,
                )
                with self.assertRaisesRegex(ValueError, "must be 1..250 ms"):
                    NavigationStampDiscipline(
                        replace(config, streams=config.streams[:2] + (cloud,))
                    )

    def test_poll_latches_fault_when_all_streams_stop(self) -> None:
        discipline = NavigationStampDiscipline(compact_config())
        latest = feed_qualification(discipline)
        discipline.lock(identity_evidence_verified=True, now_monotonic_ns=latest)
        fault_time = latest + 600_000_000
        result = discipline.poll(fault_time)
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertTrue(result.reason.startswith("stream_stale:"), result.reason)
        self.assertIn("receipt_age_ns=", result.reason)
        self.assertIn("limit_ns=200000000", result.reason)
        trigger_stream = result.reason.split(":", 2)[1]
        trigger = discipline.receipt_liveness_snapshot(fault_time)[trigger_stream]
        self.assertGreater(
            trigger["receipt_age_ns"], trigger["stale_receipt_timeout_ns"]
        )
        self.assertFalse(trigger["live"])

    def test_configuration_cannot_widen_checked_in_hard_age_ceiling(self) -> None:
        config = compact_config()
        widened = replace(
            config.streams[0], max_corrected_age_ns=201_000_000
        )
        bad = replace(config, streams=(widened,) + config.streams[1:])
        with self.assertRaisesRegex(ValueError, "hard ceiling"):
            NavigationStampDiscipline(bad)


if __name__ == "__main__":
    unittest.main()
