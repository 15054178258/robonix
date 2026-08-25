from __future__ import annotations

import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deploy" / "time-sync"))

import navigation_stamp_discipline as discipline_module  # noqa: E402
from navigation_stamp_discipline import (  # noqa: E402
    AffineClockModel,
    AffineNavigationStampDiscipline,
    AffineStreamMetrics,
    DisciplineConfig,
    DisciplineState,
    LOCKED_STALE_SAMPLE_DROP_REASON_PREFIX,
    NONCORE_DELTA_DROP_REASON_PREFIX,
    StreamPolicy,
    go2_workstation_motion_config,
    go2_workstation_nomotion_config,
)
import workstation_nomotion_stamp_node as stamp_node  # noqa: E402


NS = 1_000_000_000
DOMAIN = "unitree-main-computer@192.168.123.161"
BASE_SOURCE = 1_700_000_000 * NS
BASE_LOCAL = BASE_SOURCE + 748 * NS
BASE_MONOTONIC = 100 * NS
CORE_STREAMS = ("sport_primary", "mid360_imu", "mid360_odom")


def fresh_clock_pair(
    realtime_ns: int, monotonic_ns: int, read_span_ns: int = 0
) -> stamp_node.ReceiptClockPair:
    return stamp_node.ReceiptClockPair(
        realtime_ns,
        monotonic_ns,
        read_span_ns,
        0,
    )


def strict_affine_approval(
    session_id: str,
    *,
    offset_ns: int = 748 * NS,
    drift_ppm: float = 13.0,
):
    now_ns = time.time_ns()
    return stamp_node.FixedOffsetApproval(
        schema="robonix-go2-workstation-nomotion-stamp-offset-v3",
        session_id=session_id,
        expected_clock_domain=DOMAIN,
        writer_gids=tuple(
            (name, f"{index:02x}" * 24)
            for index, name in enumerate(stamp_node.RAW_TOPICS, start=1)
        ),
        writer_source_ipv4="192.168.123.161",
        offset_evidence_sha256="ab" * 32,
        fixed_local_minus_source_offset_ns=offset_ns,
        approved_affine_common_drift_ppm=drift_ppm,
        affine_window_common_drifts_ppm=(drift_ppm, drift_ppm),
        not_before_unix_ns=now_ns - NS,
        expires_unix_ns=now_ns + 600 * NS,
    )


def affine_config(**overrides: object) -> DisciplineConfig:
    policies = tuple(
        StreamPolicy(
            name,
            max_age,
            hard_age,
            hard_age,
            relative_lag,
            100,
            max_source_receipt_delta_error_ns=150_000_000,
        )
        for name, max_age, hard_age, relative_lag in (
            ("sport_primary", 100_000_000, 200_000_000, 10_000_000),
            ("mid360_imu", 100_000_000, 200_000_000, 20_000_000),
            ("mid360_cloud", 250_000_000, 500_000_000, 150_000_000),
            ("mid360_odom", 100_000_000, 200_000_000, 20_000_000),
        )
    )
    values = dict(
        reference_stream="sport_primary",
        expected_clock_domain=DOMAIN,
        streams=policies,
        minimum_qualification_span_ns=10 * NS,
        max_absolute_drift_ppm=50.0,
        max_pairwise_drift_ppm=25.0,
        max_locked_offset_deviation_ns=20_000_000,
        statistics_evaluation_period_ns=20_000_000,
        retained_samples_per_stream=5000,
    )
    values.update(overrides)
    return DisciplineConfig(**values)


def nomotion_drop_affine_config(**overrides: object) -> DisciplineConfig:
    values = {
        "affine_locked_noncore_delta_drop_streams": ("mid360_cloud",),
        "affine_qualifying_noncore_delta_drop_streams": ("mid360_cloud",),
        **overrides,
    }
    return replace(affine_config(), **values)


def nomotion_core_recovery_affine_config(
    **overrides: object,
) -> DisciplineConfig:
    values = {
        "affine_locked_core_delta_recovery_streams": CORE_STREAMS,
        **overrides,
    }
    return replace(affine_config(), **values)


def hardened_nomotion_affine_config(**overrides: object) -> DisciplineConfig:
    """Compact test profile matching the no-motion affine age budgets."""

    base = affine_config()
    values = {
        "streams": tuple(
            replace(
                policy,
                max_corrected_age_ns=(
                    150_000_000
                    if policy.name == "sport_primary"
                    else policy.hard_age_ceiling_ns
                ),
            )
            if policy.name in {"sport_primary", "mid360_imu", "mid360_odom"}
            else policy
            for policy in base.streams
        ),
        "affine_anchor_past_guard_ns": 20_000_000,
        "minimum_locked_corrected_age_ns": 5_000_000,
        "max_locked_affine_drift_deviation_ppm": 5.0,
        **overrides,
    }
    return replace(base, **values)


def source_at(local_elapsed_ns: int, drift_ppm: float) -> int:
    return BASE_SOURCE + round(local_elapsed_ns * (1.0 - drift_ppm * 1e-6))


def observe_set(
    discipline: AffineNavigationStampDiscipline,
    *,
    local_elapsed_ns: int,
    drift_ppm: float = 13.0,
    cloud_acquisition_lag_ns: int = 70_000_000,
    callback_burst_ns: int = 0,
) -> int:
    latest = 0
    for stream, callback_delay_ns, acquisition_lag_ns in (
        ("mid360_imu", 1_500_000, 0),
        ("sport_primary", 2_000_000, 0),
        ("mid360_odom", 4_000_000, 0),
        ("mid360_cloud", 5_000_000, cloud_acquisition_lag_ns),
    ):
        source_elapsed_ns = local_elapsed_ns - acquisition_lag_ns
        source_ns = source_at(source_elapsed_ns, drift_ppm)
        delay_ns = callback_delay_ns + callback_burst_ns
        receipt_realtime_ns = BASE_LOCAL + local_elapsed_ns + delay_ns
        receipt_monotonic_ns = BASE_MONOTONIC + local_elapsed_ns + delay_ns
        result = discipline.observe(
            stream,
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_domain=DOMAIN,
            clock_read_span_ns=10_000,
        )
        if discipline.state == DisciplineState.FAULTED:
            return receipt_monotonic_ns
        latest = max(latest, receipt_monotonic_ns)
        if discipline.state == DisciplineState.QUALIFYING:
            assert not result.navigation_eligible
    return latest


def qualify(
    discipline: AffineNavigationStampDiscipline,
    *,
    drift_ppm: float = 13.0,
    cloud_acquisition_lag_ns: int = 70_000_000,
) -> int:
    latest = 0
    for index in range(131):
        # A large non-negative callback delay affects several samples, but at
        # least one low-delay observation remains in every one-second bin.
        burst_ns = 80_000_000 if index % 10 in {4, 5} else 0
        latest = observe_set(
            discipline,
            local_elapsed_ns=index * 100_000_000,
            drift_ppm=drift_ppm,
            cloud_acquisition_lag_ns=cloud_acquisition_lag_ns,
            callback_burst_ns=burst_ns,
        )
        if discipline.state == DisciplineState.FAULTED:
            raise AssertionError(discipline.fault_reason)
    return latest


def qualify_smooth(
    discipline: AffineNavigationStampDiscipline,
    *,
    drift_ppm: float = 13.0,
) -> int:
    latest = 0
    for index in range(131):
        latest = observe_set(
            discipline,
            local_elapsed_ns=index * 100_000_000,
            drift_ppm=drift_ppm,
            callback_burst_ns=0,
        )
        if discipline.state == DisciplineState.FAULTED:
            raise AssertionError(discipline.fault_reason)
    return latest


def qualify_two_windows(
    discipline: AffineNavigationStampDiscipline,
    *,
    first_slew_ppm: float,
    second_slew_ppm: float,
    drift_ppm: float = 13.0,
) -> int:
    """Feed two fixed 10 s windows with a continuous local-clock slew."""

    latest = 0
    split_ns = 10 * NS
    for index in range(201):
        elapsed_ns = index * 100_000_000
        if elapsed_ns <= split_ns:
            slew_ns = round(elapsed_ns * first_slew_ppm * 1e-6)
        else:
            slew_ns = round(split_ns * first_slew_ppm * 1e-6) + round(
                (elapsed_ns - split_ns) * second_slew_ppm * 1e-6
            )
        for stream, delay_ns in (
            ("mid360_imu", 1_500_000),
            ("sport_primary", 2_000_000),
            ("mid360_odom", 4_000_000),
            ("mid360_cloud", 5_000_000),
        ):
            source_ns = source_at(elapsed_ns, drift_ppm)
            receipt_realtime_ns = (
                BASE_LOCAL + elapsed_ns + delay_ns + slew_ns
            )
            receipt_monotonic_ns = BASE_MONOTONIC + elapsed_ns + delay_ns
            result = discipline.observe(
                stream,
                source_ns,
                receipt_realtime_ns,
                receipt_monotonic_ns,
                clock_domain=DOMAIN,
                clock_read_span_ns=10_000,
            )
            if result.state == DisciplineState.FAULTED:
                raise AssertionError(result.reason)
            latest = max(latest, receipt_monotonic_ns)
    return latest


def legacy_locked_statistical_health(
    discipline: AffineNavigationStampDiscipline,
) -> str | None:
    """Exact pre-optimization locked-health decision oracle.

    Keep this intentionally direct: it preserves the former redundant full
    generic metrics pass, all-stream affine metrics pass, and per-evaluation
    corrected-age rebuild so the optimized path can be checked for identical
    decisions and first-fault ordering.
    """

    model = discipline.locked_affine_model
    if model is None:
        return "affine_model_missing_after_lock"
    metrics = discipline.metrics()
    for name, policy in discipline._policies.items():
        stream_metrics = metrics[name]
        if stream_metrics is None:
            return f"stream_lost:{name}"
        if stream_metrics.duplicate_fraction > policy.max_duplicate_fraction:
            return f"duplicate_fraction_exceeded:{name}"

    affine_metrics = discipline.affine_metrics()
    current_core_drifts: list[tuple[str, float]] = []
    for name in discipline.core_streams:
        drift = affine_metrics[name].drift_ppm
        if drift is None or abs(drift) > discipline.config.max_absolute_drift_ppm:
            return f"clock_drift_exceeded:{name}"
        current_core_drifts.append((name, drift))
    common_drift = float(
        discipline_module.statistics.median(
            value for _, value in current_core_drifts
        )
    )
    if abs(common_drift - model.drift_ppm) > (
        discipline.config.max_locked_affine_drift_deviation_ppm
    ):
        return "affine_model_drift_deviation_exceeded"
    pairwise_disagreements = discipline_module._pairwise_drift_disagreements(
        current_core_drifts, discipline.config.max_pairwise_drift_ppm
    )
    if pairwise_disagreements:
        return f"pairwise_drift_disagreement:{pairwise_disagreements[0]}"

    baseline_ages = dict(model.stream_baseline_corrected_age_ns)
    reference_baseline = baseline_ages[discipline.config.reference_stream]
    current_lower_ages: dict[str, int] = {}
    for name, state in discipline._streams.items():
        corrected_ages = [
            age_ns - model.offset_at_source_ns(source_ns)
            for source_ns, _, age_ns in state.samples
        ]
        if not corrected_ages:
            return f"stream_lost:{name}"
        current_lower = discipline_module._quantile(
            corrected_ages, discipline.config.lower_quantile
        )
        current_lower_ages[name] = current_lower
        if abs(current_lower - baseline_ages[name]) > (
            discipline.config.max_locked_offset_deviation_ns
        ):
            return f"affine_model_residual_exceeded:{name}"

    reference_current = current_lower_ages[discipline.config.reference_stream]
    for name, policy in discipline._policies.items():
        baseline_relative = baseline_ages[name] - reference_baseline
        current_relative = current_lower_ages[name] - reference_current
        if abs(current_relative - baseline_relative) > (
            discipline.config.max_locked_offset_deviation_ns
        ):
            return f"stream_relative_lag_changed:{name}"
        if current_relative < -discipline.config.max_relative_lead_ns:
            return f"stream_leads_reference:{name}"
        if current_relative > policy.max_relative_lag_ns:
            return f"stream_lag_exceeded:{name}"
    return None


def shift_locked_stream_ages(
    discipline: AffineNavigationStampDiscipline,
    stream: str,
    delta_ns: int,
) -> None:
    """Shift raw and cached ages together for deterministic boundary tests."""

    state = discipline._streams[stream]
    shifted_samples = [
        (source_ns, monotonic_ns, age_ns + delta_ns)
        for source_ns, monotonic_ns, age_ns in state.samples
    ]
    state.samples.clear()
    state.samples.extend(shifted_samples)
    windows = discipline._locked_corrected_ages
    assert windows is not None
    shifted_cache = [value + delta_ns for value in windows[stream]]
    windows[stream].clear()
    windows[stream].extend(shifted_cache)


def install_synthetic_locked_history(
    discipline: AffineNavigationStampDiscipline,
    *,
    duration_ns: int = 15 * 60 * NS,
    rates_hz: dict[str, int] | None = None,
    initial_drift_ppm: float = 13.0,
    switched_drift_ppm: float | None = None,
    switch_elapsed_ns: int | None = None,
) -> None:
    """Install the exact retained tail of a long, rate-diverse locked run."""

    assert discipline.state == DisciplineState.LOCKED
    model = discipline.locked_affine_model
    windows = discipline._locked_corrected_ages
    assert model is not None
    assert windows is not None
    rates = rates_hz or {
        "sport_primary": 150,
        "mid360_imu": 500,
        "mid360_odom": 250,
        "mid360_cloud": 15,
    }
    delays_ns = {
        "sport_primary": 2_000_000,
        "mid360_imu": 1_500_000,
        "mid360_odom": 4_000_000,
        "mid360_cloud": 5_000_000,
    }
    acquisition_lags_ns = {
        "sport_primary": 0,
        "mid360_imu": 0,
        "mid360_odom": 0,
        "mid360_cloud": 70_000_000,
    }

    def integrated_source_elapsed(local_elapsed_ns: int) -> int:
        if switched_drift_ppm is None or switch_elapsed_ns is None:
            return round(
                local_elapsed_ns * (1.0 - initial_drift_ppm * 1e-6)
            )
        if local_elapsed_ns <= switch_elapsed_ns:
            return round(
                local_elapsed_ns * (1.0 - initial_drift_ppm * 1e-6)
            )
        return round(
            switch_elapsed_ns * (1.0 - initial_drift_ppm * 1e-6)
            + (local_elapsed_ns - switch_elapsed_ns)
            * (1.0 - switched_drift_ppm * 1e-6)
        )

    for name, state in discipline._streams.items():
        rate_hz = rates[name]
        total_samples = duration_ns * rate_hz // NS + 1
        retained_limit = state.samples.maxlen
        assert retained_limit is not None
        first_retained_index = max(0, total_samples - retained_limit)
        state.samples.clear()
        windows[name].clear()
        for index in range(first_retained_index, total_samples):
            elapsed_ns = index * NS // rate_hz
            receipt_monotonic_ns = (
                BASE_MONOTONIC + elapsed_ns + delays_ns[name]
            )
            receipt_realtime_ns = BASE_LOCAL + elapsed_ns + delays_ns[name]
            sensor_elapsed_ns = elapsed_ns - acquisition_lags_ns[name]
            source_ns = BASE_SOURCE + integrated_source_elapsed(
                sensor_elapsed_ns
            )
            age_ns = receipt_realtime_ns - source_ns
            state.samples.append(
                (source_ns, receipt_monotonic_ns, age_ns)
            )
            windows[name].append(
                age_ns - model.offset_at_source_ns(source_ns)
            )
        latest_source_ns, latest_receipt_ns, _ = state.samples[-1]
        state.last_source_ns = latest_source_ns
        state.last_advancing_receipt_monotonic_ns = latest_receipt_ns
        state.advancing = total_samples
        state.duplicates = 0
        discipline._observed_continuity[name] = (
            latest_source_ns,
            latest_receipt_ns,
        )
        discipline._observed_continuity_is_last_good[name] = True
    discipline._last_clock_base_ns = BASE_LOCAL - BASE_MONOTONIC


class AffineNavigationStampDisciplineTest(unittest.TestCase):
    def make_discipline(self) -> AffineNavigationStampDiscipline:
        return AffineNavigationStampDiscipline(
            affine_config(), core_streams=CORE_STREAMS
        )

    def make_nomotion_drop_discipline(self) -> AffineNavigationStampDiscipline:
        return AffineNavigationStampDiscipline(
            nomotion_drop_affine_config(), core_streams=CORE_STREAMS
        )

    def assert_locked_health_matches_legacy(
        self,
        discipline: AffineNavigationStampDiscipline,
        expected: str | None,
    ) -> None:
        legacy_reason = legacy_locked_statistical_health(discipline)
        optimized_reason = discipline._statistical_health()
        self.assertEqual(legacy_reason, expected)
        self.assertEqual(optimized_reason, legacy_reason)

    def make_two_window_discipline(self) -> AffineNavigationStampDiscipline:
        return AffineNavigationStampDiscipline(
            replace(
                affine_config(),
                affine_qualification_window_ns=10 * NS,
                max_affine_window_common_drift_deviation_ppm=5.0,
            ),
            core_streams=CORE_STREAMS,
            qualification_start_monotonic_ns=BASE_MONOTONIC,
        )

    def test_two_complete_nonoverlapping_windows_gate_before_full_freeze(
        self,
    ) -> None:
        discipline = self.make_two_window_discipline()
        latest = qualify_two_windows(
            discipline, first_slew_ppm=0.0, second_slew_ppm=3.0
        )
        self.assertEqual(discipline.qualification_reasons(latest), [])
        evidence = discipline.affine_qualification_evidence()
        assert evidence is not None
        first, second, complete = evidence
        self.assertEqual(first.end_monotonic_ns, second.start_monotonic_ns)
        self.assertFalse(first.end_inclusive)
        self.assertTrue(second.end_inclusive)
        self.assertAlmostEqual(first.common_drift_ppm or 0.0, 13.0, delta=0.2)
        self.assertAlmostEqual(second.common_drift_ppm or 0.0, 16.0, delta=0.2)
        model = discipline.lock_affine(
            identity_evidence_verified=True,
            now_monotonic_ns=latest,
            approved_common_drift_ppm=14.5,
        )
        self.assertAlmostEqual(
            model.drift_ppm,
            complete.common_drift_ppm or 0.0,
            delta=1e-12,
        )

    def test_gradual_ntp_slew_transient_is_rejected_by_window_gate(self) -> None:
        discipline = self.make_two_window_discipline()
        latest = qualify_two_windows(
            discipline, first_slew_ppm=0.0, second_slew_ppm=8.0
        )
        reasons = discipline.qualification_reasons(latest)
        self.assertIn(
            "affine_window_common_drift_deviation_exceeded", reasons
        )
        with self.assertRaisesRegex(RuntimeError, "window_common"):
            discipline.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=latest,
                approved_common_drift_ppm=17.0,
            )
        self.assertEqual(discipline.state, DisciplineState.QUALIFYING)

    def test_terminal_window_failure_faults_once_with_complete_evidence(
        self,
    ) -> None:
        discipline = self.make_two_window_discipline()
        latest = qualify_two_windows(
            discipline, first_slew_ppm=0.0, second_slew_ppm=8.0
        )
        approval = strict_affine_approval(
            "terminal-window-fault-test", drift_ppm=17.0
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            runtime.discipline = discipline
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            commit_realtime_ns = BASE_LOCAL + (
                latest - BASE_MONOTONIC
            )
            with mock.patch.object(
                stamp_node,
                "fresh_paired_receipt_clocks",
                return_value=fresh_clock_pair(commit_realtime_ns, latest),
            ), mock.patch.object(
                stamp_node, "require_strict_affine_approval"
            ):
                runtime.maybe_lock(latest)
            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
            runtime.maybe_lock(latest + NS)

        self.assertEqual(
            fault["reason"], "affine_qualification_terminal_failed"
        )
        qualification_fault = fault["qualification_fault"]
        self.assertEqual(
            qualification_fault["terminal_phase"],
            "fixed_two_window_evaluation",
        )
        self.assertIn(
            "affine_window_common_drift_deviation_exceeded",
            qualification_fault["reasons"],
        )
        self.assertEqual(
            set(qualification_fault["windows"]),
            {"first", "second", "complete"},
        )
        expected_core_fields = {
            "envelope_points",
            "first_receipt_offset_ns",
            "last_receipt_offset_ns",
            "max_receipt_gap_ns",
            "drift_ppm",
        }
        for window in qualification_fault["windows"].values():
            self.assertEqual(set(window["cores"]), set(CORE_STREAMS))
            for core in window["cores"].values():
                self.assertEqual(set(core), expected_core_fields)
        self.assertEqual(discipline.state, DisciplineState.FAULTED)
        self.assertIsNone(discipline.locked_affine_model)
        self.assertEqual(runtime.affine_terminal_full_evaluation_count, 1)
        self.assertEqual(shutdowns, [True])

    def test_terminal_snapshot_uses_ten_lower_envelope_scans_and_cached_lock(
        self,
    ) -> None:
        discipline = self.make_two_window_discipline()
        latest = qualify_two_windows(
            discipline, first_slew_ppm=0.0, second_slew_ppm=3.0
        )
        snapshot = discipline.capture_affine_qualification_snapshot(latest)
        with mock.patch.object(
            discipline_module,
            "_lower_envelope_points",
            wraps=discipline_module._lower_envelope_points,
        ) as lower_envelope:
            evaluation = discipline.evaluate_affine_qualification_snapshot(
                snapshot
            )
        self.assertEqual(lower_envelope.call_count, 10)
        self.assertEqual(evaluation.reasons, ())
        with mock.patch.object(
            discipline, "qualification_reasons"
        ) as qualification_reasons, mock.patch.object(
            discipline, "_provisional_affine_model"
        ) as provisional_model:
            model = discipline.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=latest,
                approved_common_drift_ppm=14.5,
                qualification_evaluation=evaluation,
            )
        qualification_reasons.assert_not_called()
        provisional_model.assert_not_called()
        self.assertIs(model, discipline.locked_affine_model)

    def test_terminal_commit_consumes_immutable_cache_without_live_rescan(
        self,
    ) -> None:
        discipline = self.make_two_window_discipline()
        latest = qualify_two_windows(
            discipline, first_slew_ppm=0.0, second_slew_ppm=3.0
        )
        snapshot = discipline.capture_affine_qualification_snapshot(latest)
        evaluation = discipline.evaluate_affine_qualification_snapshot(snapshot)
        cached = dict(evaluation.corrected_age_windows)
        self.assertEqual(set(cached), set(discipline._streams))
        self.assertTrue(all(isinstance(values, tuple) for values in cached.values()))
        with mock.patch.object(
            AffineClockModel,
            "offset_at_source_ns",
            side_effect=AssertionError("commit rescanned/recomputed live samples"),
        ):
            discipline.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=latest + 1,
                approved_common_drift_ppm=14.5,
                qualification_evaluation=evaluation,
            )
        assert discipline._locked_corrected_ages is not None
        for name, values in cached.items():
            self.assertEqual(tuple(discipline._locked_corrected_ages[name]), values)
            self.assertIsNot(discipline._locked_corrected_ages[name], values)

        changed = self.make_two_window_discipline()
        changed_latest = qualify_two_windows(
            changed, first_slew_ppm=0.0, second_slew_ppm=3.0
        )
        changed_snapshot = changed.capture_affine_qualification_snapshot(
            changed_latest
        )
        changed_evaluation = changed.evaluate_affine_qualification_snapshot(
            changed_snapshot
        )
        changed._streams["sport_primary"].samples.append(
            changed._streams["sport_primary"].samples[-1]
        )
        with self.assertRaisesRegex(RuntimeError, "live samples changed"):
            changed.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=changed_latest + 1,
                approved_common_drift_ppm=14.5,
                qualification_evaluation=changed_evaluation,
            )

    def test_post_evaluation_liveness_boundary_is_fresh_then_stale(self) -> None:
        discipline = self.make_two_window_discipline()
        latest = qualify_two_windows(
            discipline, first_slew_ppm=0.0, second_slew_ppm=3.0
        )
        for state in discipline._streams.values():
            state.last_advancing_receipt_monotonic_ns = latest
        snapshot = discipline.capture_affine_qualification_snapshot(latest)
        evaluation = discipline.evaluate_affine_qualification_snapshot(snapshot)
        clock_base_ns = BASE_LOCAL - BASE_MONOTONIC

        boundary_ns = latest + 200_000_000
        reasons, diagnostics = discipline.affine_post_evaluation_commit_gate(
            evaluation,
            receipt_realtime_ns=clock_base_ns + boundary_ns,
            receipt_monotonic_ns=boundary_ns,
            clock_read_span_ns=1_000_000,
        )
        self.assertEqual(reasons, ())
        self.assertEqual(
            diagnostics["stream_receipt_liveness"]["sport_primary"][
                "receipt_age_ns"
            ],
            200_000_000,
        )

        stale_ns = latest + 200_000_001
        reasons, diagnostics = discipline.affine_post_evaluation_commit_gate(
            evaluation,
            receipt_realtime_ns=clock_base_ns + stale_ns,
            receipt_monotonic_ns=stale_ns,
            clock_read_span_ns=0,
        )
        self.assertTrue(
            any(reason.startswith("stream_stale:sport_primary:") for reason in reasons)
        )
        self.assertFalse(
            diagnostics["stream_receipt_liveness"]["sport_primary"]["live"]
        )

    def test_crytst_outlier_rejected_and_191601_candidate_accepted(self) -> None:
        old = self.make_discipline()
        old_latest = qualify(old, drift_ppm=0.31853891075308655)
        with self.assertRaisesRegex(
            RuntimeError, "approved_affine_common_drift_deviation_exceeded"
        ):
            old.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=old_latest,
                approved_common_drift_ppm=12.960795339710383,
            )
        self.assertEqual(old.state, DisciplineState.FAULTED)
        self.assertIsNone(old.locked_affine_model)
        with self.assertRaisesRegex(
            RuntimeError, "approved_affine_common_drift_deviation_exceeded"
        ):
            old.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=old_latest,
                approved_common_drift_ppm=0.31853891075308655,
            )

        new = self.make_discipline()
        new_latest = qualify(new, drift_ppm=9.314399888579858)
        model = new.lock_affine(
            identity_evidence_verified=True,
            now_monotonic_ns=new_latest,
            approved_common_drift_ppm=8.078323139204173,
        )
        self.assertEqual(new.state, DisciplineState.LOCKED)
        self.assertAlmostEqual(model.drift_ppm, 9.314399888579858, delta=0.2)
        self.assertLessEqual(
            abs(model.drift_ppm - 8.078323139204173), 5.0
        )

    def test_runtime_locks_current_live_model_without_old_approval_compare(
        self,
    ) -> None:
        discipline = self.make_two_window_discipline()
        latest = qualify_two_windows(
            discipline,
            first_slew_ppm=0.0,
            second_slew_ppm=0.0,
            drift_ppm=0.31853891075308655,
        )
        snapshot = discipline.capture_affine_qualification_snapshot(latest)
        evaluation = discipline.evaluate_affine_qualification_snapshot(snapshot)
        candidate = evaluation.model
        assert candidate is not None
        approval = strict_affine_approval(
            "crytst-pre-ready-regression",
            offset_ns=candidate.offset_at_source_ns(candidate.anchor_source_ns),
            drift_ppm=12.960795339710383,
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            runtime.discipline = discipline
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            commit_realtime_ns = BASE_LOCAL + (
                latest - BASE_MONOTONIC
            )
            with mock.patch.object(
                stamp_node,
                "fresh_paired_receipt_clocks",
                return_value=fresh_clock_pair(commit_realtime_ns, latest),
            ), mock.patch.object(
                stamp_node, "require_strict_affine_approval"
            ):
                runtime.maybe_lock(latest)
            ready = json.loads(runtime.ready_file.read_text(encoding="utf-8"))

        self.assertTrue(runtime.ready)
        self.assertFalse(runtime.fault_file.exists())
        self.assertEqual(ready["correction_mode"], "affine")
        self.assertAlmostEqual(
            ready["affine_model"]["drift_ppm"],
            0.31853891075308655,
            delta=0.2,
        )
        self.assertEqual(shutdowns, [])
        runtime.maybe_lock(latest + NS)
        self.assertEqual(shutdowns, [])
        self.assertEqual(runtime.affine_terminal_full_evaluation_count, 1)

    def test_post_evaluation_stale_faults_once_without_ready_or_relock(self) -> None:
        discipline = self.make_two_window_discipline()
        latest = qualify_two_windows(
            discipline, first_slew_ppm=0.0, second_slew_ppm=3.0
        )
        for state in discipline._streams.values():
            state.last_advancing_receipt_monotonic_ns = latest
        approval = strict_affine_approval("post-eval-stale-test")
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            runtime.discipline = discipline
            runtime.shutdown = lambda: None
            stale_ns = latest + 200_000_001
            clock_base_ns = BASE_LOCAL - BASE_MONOTONIC
            with mock.patch.object(
                stamp_node,
                "fresh_paired_receipt_clocks",
                return_value=fresh_clock_pair(clock_base_ns + stale_ns, stale_ns),
            ) as paired:
                runtime.maybe_lock(latest)
                runtime.maybe_lock(stale_ns + NS)
            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
        self.assertTrue(fault["reason"].startswith("stream_stale:"))
        self.assertEqual(
            fault["qualification_fault"]["terminal_phase"],
            "post_evaluation_commit_gate",
        )
        self.assertFalse(runtime.ready)
        self.assertIsNone(discipline.locked_affine_model)
        self.assertEqual(runtime.affine_terminal_full_evaluation_count, 1)
        paired.assert_called_once()

    def test_post_evaluation_expiry_metadata_does_not_block_live_commit(
        self,
    ) -> None:
        now_ns = time.time_ns()
        approval = replace(
            strict_affine_approval("post-eval-expiry-test"),
            not_before_unix_ns=now_ns - NS,
            expires_unix_ns=now_ns + NS,
        )
        model = AffineClockModel(
            anchor_source_ns=BASE_SOURCE,
            anchor_local_ns=BASE_LOCAL,
            drift_ppm=13.0,
            source_to_local_scale=1.0 / (1.0 - 13.0e-6),
            core_stream_drifts_ppm=tuple(
                (name, 13.0) for name in CORE_STREAMS
            ),
            stream_baseline_corrected_age_ns=tuple(
                (name, 5_000_000)
                for name in (
                    "sport_primary",
                    "mid360_imu",
                    "mid360_cloud",
                    "mid360_odom",
                )
            ),
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            evaluation = SimpleNamespace(reasons=())
            runtime.discipline.affine_qualification_deadline_monotonic_ns = (
                mock.Mock(return_value=60 * NS)
            )
            runtime.discipline.capture_affine_qualification_snapshot = mock.Mock(
                return_value=mock.sentinel.snapshot
            )
            runtime.discipline.evaluate_affine_qualification_snapshot = mock.Mock(
                return_value=evaluation
            )
            runtime.discipline.affine_post_evaluation_commit_gate = mock.Mock(
                return_value=((), {"phase": "post_evaluation_commit_gate"})
            )
            runtime.discipline.affine_qualification_fault_diagnostics = mock.Mock(
                return_value={"terminal_phase": "post_evaluation_commit_gate"}
            )
            runtime.discipline.reject_affine_qualification = mock.Mock()
            runtime.discipline.lock_affine = mock.Mock(return_value=model)
            runtime.shutdown = lambda: None
            with mock.patch.object(
                stamp_node,
                "fresh_paired_receipt_clocks",
                return_value=fresh_clock_pair(
                    approval.expires_unix_ns + 1, 60 * NS + 1
                ),
            ) as paired:
                runtime.maybe_lock(60 * NS)
                runtime.maybe_lock(61 * NS)
            ready = json.loads(runtime.ready_file.read_text(encoding="utf-8"))
        self.assertTrue(runtime.ready)
        self.assertFalse(runtime.fault_file.exists())
        self.assertEqual(ready["correction_mode"], "affine")
        runtime.discipline.lock_affine.assert_called_once()
        paired.assert_called_once()

    def test_post_evaluation_stale_pair_faults_before_gate_or_approval(self) -> None:
        approval = strict_affine_approval("post-eval-pair-age-test")
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            evaluation = SimpleNamespace(reasons=())
            runtime.discipline.affine_qualification_deadline_monotonic_ns = (
                mock.Mock(return_value=60 * NS)
            )
            runtime.discipline.capture_affine_qualification_snapshot = mock.Mock(
                return_value=mock.sentinel.snapshot
            )
            runtime.discipline.evaluate_affine_qualification_snapshot = mock.Mock(
                return_value=evaluation
            )
            runtime.discipline.affine_post_evaluation_commit_gate = mock.Mock()
            runtime.discipline.affine_qualification_fault_diagnostics = mock.Mock(
                return_value={"terminal_phase": "post_evaluation_commit_gate"}
            )
            runtime.discipline.reject_affine_qualification = mock.Mock()
            runtime.discipline.lock_affine = mock.Mock()
            runtime.shutdown = lambda: None
            with mock.patch.object(
                stamp_node,
                "fresh_paired_receipt_clocks",
                side_effect=RuntimeError(
                    "clock_pair_age_exceeded:age_ns=2000000:limit_ns=1000000"
                ),
            ) as paired, mock.patch.object(
                stamp_node, "require_strict_affine_approval"
            ) as require_approval:
                runtime.maybe_lock(60 * NS)
                runtime.maybe_lock(61 * NS)
            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))

        self.assertIn("clock_pair_age_exceeded", fault["reason"])
        self.assertFalse(runtime.ready)
        self.assertFalse(runtime.ready_file.exists())
        runtime.discipline.affine_post_evaluation_commit_gate.assert_not_called()
        runtime.discipline.lock_affine.assert_not_called()
        require_approval.assert_not_called()
        paired.assert_called_once()

    def test_motion_profile_does_not_inherit_nomotion_relaxations(self) -> None:
        motion = go2_workstation_motion_config(DOMAIN)
        nomotion = go2_workstation_nomotion_config(DOMAIN)
        motion.validate()
        nomotion.validate()
        self.assertFalse(motion.locked_statistics_enabled)
        self.assertTrue(nomotion.locked_statistics_enabled)
        with self.assertRaisesRegex(ValueError, "0.1..20 ppm"):
            replace(
                nomotion,
                max_affine_window_common_drift_deviation_ppm=20.000_001,
            ).validate()
        motion_policies = {policy.name: policy for policy in motion.streams}
        nomotion_policies = {policy.name: policy for policy in nomotion.streams}
        self.assertEqual(
            set(motion_policies), set(stamp_node.MOTION_CORRECTED_TOPICS)
        )
        self.assertNotIn("mid360_cloud", motion_policies)
        for policy in motion_policies.values():
            self.assertEqual(
                policy.max_source_receipt_delta_error_ns,
                200_000_000,
            )
        self.assertEqual(
            motion_policies["mid360_imu"].max_corrected_age_ns,
            200_000_000,
        )
        self.assertEqual(
            motion_policies["mid360_odom"].max_corrected_age_ns,
            200_000_000,
        )
        self.assertEqual(
            motion_policies["sport_primary"].max_corrected_age_ns,
            200_000_000,
        )
        self.assertEqual(
            nomotion_policies["mid360_imu"].max_corrected_age_ns,
            200_000_000,
        )
        self.assertEqual(
            nomotion_policies["mid360_odom"].max_corrected_age_ns,
            200_000_000,
        )
        self.assertEqual(
            nomotion_policies["sport_primary"].max_corrected_age_ns,
            150_000_000,
        )
        self.assertEqual(
            nomotion_policies["mid360_imu"].max_source_receipt_delta_error_ns,
            150_000_000,
        )
        self.assertEqual(
            nomotion.affine_locked_noncore_delta_drop_streams,
            ("mid360_cloud",),
        )
        self.assertEqual(motion.affine_locked_noncore_delta_drop_streams, ())
        self.assertEqual(
            nomotion.affine_qualifying_noncore_delta_drop_streams,
            ("mid360_cloud",),
        )
        self.assertEqual(
            motion.affine_qualifying_noncore_delta_drop_streams, ()
        )
        self.assertFalse(
            nomotion.affine_restart_qualification_on_core_delta_discontinuity
        )
        self.assertTrue(
            motion.affine_restart_qualification_on_core_delta_discontinuity
        )
        self.assertEqual(
            nomotion.affine_locked_core_delta_recovery_streams,
            CORE_STREAMS,
        )
        self.assertEqual(
            motion.affine_locked_core_delta_recovery_streams,
            CORE_STREAMS,
        )
        self.assertEqual(nomotion.offset_guard_ns, 5_000_000)
        self.assertEqual(nomotion.affine_anchor_past_guard_ns, 20_000_000)
        self.assertEqual(nomotion.max_corrected_future_ns, 2_000_000)
        self.assertEqual(nomotion.minimum_locked_corrected_age_ns, 5_000_000)
        self.assertEqual(nomotion.max_pairwise_drift_ppm, 25.0)
        self.assertEqual(
            nomotion.max_approved_affine_drift_deviation_ppm, 5.0
        )
        self.assertEqual(nomotion.affine_qualification_window_ns, 30 * NS)
        self.assertEqual(
            nomotion.max_affine_window_common_drift_deviation_ppm, 15.0
        )
        self.assertTrue(nomotion.affine_enforce_subwindow_pairwise_drift)
        self.assertTrue(nomotion.affine_enforce_subwindow_common_drift)
        self.assertEqual(
            nomotion.max_locked_affine_drift_deviation_ppm,
            25.0,
        )
        self.assertEqual(motion.offset_guard_ns, 5_000_000)
        self.assertEqual(motion.affine_anchor_past_guard_ns, 20_000_000)
        self.assertEqual(motion.max_corrected_future_ns, 2_000_000)
        self.assertIsNone(motion.minimum_locked_corrected_age_ns)
        self.assertEqual(motion.max_pairwise_drift_ppm, 25.0)
        self.assertEqual(
            motion.max_approved_affine_drift_deviation_ppm, 5.0
        )
        self.assertEqual(motion.minimum_qualification_span_ns, 10 * NS)
        self.assertEqual(motion.affine_qualification_window_ns, 10 * NS)
        self.assertEqual(
            motion.max_affine_window_common_drift_deviation_ppm, 15.0
        )
        self.assertFalse(motion.affine_enforce_subwindow_pairwise_drift)
        self.assertFalse(motion.affine_enforce_subwindow_common_drift)
        self.assertEqual(
            motion.max_locked_affine_drift_deviation_ppm,
            25.0,
        )
        self.assertEqual(nomotion.retained_samples_per_stream, 40_000)
        self.assertEqual(motion.retained_samples_per_stream, 40_000)

        approval = strict_affine_approval("motion-stream-test")
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
                profile="motion",
            )
            self.assertEqual(
                runtime.streams, tuple(stamp_node.MOTION_CORRECTED_TOPICS)
            )
            self.assertNotIn("mid360_cloud", runtime.streams)
            rejected = runtime.discipline.observe(
                "mid360_cloud",
                BASE_SOURCE,
                BASE_LOCAL,
                BASE_MONOTONIC,
                clock_domain=DOMAIN,
            )
            self.assertEqual(rejected.reason, "unexpected_stream:mid360_cloud")
            self.assertEqual(rejected.state, DisciplineState.FAULTED)

    def test_motion_qualifier_restarts_after_prelock_core_delivery_pause(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            go2_workstation_motion_config(), core_streams=CORE_STREAMS
        )
        discipline.observe(
            "mid360_imu",
            BASE_SOURCE,
            BASE_LOCAL,
            BASE_MONOTONIC,
            clock_domain=DOMAIN,
        )
        result = discipline.observe(
            "mid360_imu",
            BASE_SOURCE + 50_000_000,
            BASE_LOCAL + 307_000_000,
            BASE_MONOTONIC + 307_000_000,
            clock_domain=DOMAIN,
        )

        self.assertEqual(result.state, DisciplineState.QUALIFYING)
        self.assertEqual(
            result.reason,
            "affine_qualification_restarted_after_delivery_pause:mid360_imu",
        )
        self.assertEqual(discipline.metrics()["mid360_imu"].samples, 1)
        self.assertIsNone(discipline.metrics()["sport_primary"])
        self.assertIsNone(discipline.metrics()["mid360_odom"])

    def test_motion_qualifier_restarts_after_prelock_all_clock_gap(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            go2_workstation_motion_config(), core_streams=CORE_STREAMS
        )
        for stream in CORE_STREAMS:
            discipline.observe(
                stream,
                BASE_SOURCE,
                BASE_LOCAL,
                BASE_MONOTONIC,
                clock_domain=DOMAIN,
            )

        result = discipline.observe(
            "sport_primary",
            BASE_SOURCE + 7_500_000_000,
            BASE_LOCAL + 7_500_000_000,
            BASE_MONOTONIC + 7_500_000_000,
            clock_domain=DOMAIN,
        )

        self.assertEqual(result.state, DisciplineState.QUALIFYING)
        self.assertEqual(
            result.reason,
            "affine_qualification_restarted_after_delivery_pause:"
            "sport_primary",
        )
        self.assertEqual(discipline.metrics()["sport_primary"].samples, 1)
        self.assertIsNone(discipline.metrics()["mid360_imu"])
        self.assertIsNone(discipline.metrics()["mid360_odom"])

    def test_motion_lock_reuses_approved_drift_with_current_live_anchor(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            go2_workstation_motion_config(), core_streams=CORE_STREAMS
        )
        latest = 0
        for index in range(12):
            elapsed_ns = index * 100_000_000
            for stream, delay_ns in (
                ("mid360_imu", 1_500_000),
                ("sport_primary", 2_000_000),
                ("mid360_odom", 4_000_000),
            ):
                result = discipline.observe(
                    stream,
                    source_at(elapsed_ns, 13.0),
                    BASE_LOCAL + elapsed_ns + delay_ns,
                    BASE_MONOTONIC + elapsed_ns + delay_ns,
                    clock_domain=DOMAIN,
                    clock_read_span_ns=10_000,
                )
                self.assertEqual(result.state, DisciplineState.QUALIFYING)
                latest = max(
                    latest, BASE_MONOTONIC + elapsed_ns + delay_ns
                )

        with mock.patch.object(
            discipline, "_provisional_affine_model"
        ) as provisional_model, mock.patch.object(
            discipline, "qualification_reasons"
        ) as qualification_reasons:
            model = discipline.lock_affine_from_approved_drift(
                approved_common_drift_ppm=13.0,
                identity_evidence_verified=True,
                now_monotonic_ns=latest,
            )

        provisional_model.assert_not_called()
        qualification_reasons.assert_not_called()
        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertEqual(model.drift_ppm, 13.0)
        self.assertEqual(
            dict(model.core_stream_drifts_ppm),
            {name: 13.0 for name in CORE_STREAMS},
        )
        self.assertEqual(
            discipline.affine_drift_diagnostics()["comparison_phase"],
            "approved_drift_live_anchor",
        )

    def test_motion_past_guard_survives_observed_persistent_drift_fault(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            go2_workstation_motion_config(), core_streams=CORE_STREAMS
        )
        approved_drift_ppm = 14.017305090331774
        continuing_drift_ppm = 10.707
        latest = 0
        for index in range(12):
            elapsed_ns = index * 100_000_000
            for stream, delay_ns in (
                ("mid360_imu", 1_500_000),
                ("sport_primary", 2_000_000),
                ("mid360_odom", 4_000_000),
            ):
                result = discipline.observe(
                    stream,
                    source_at(elapsed_ns, approved_drift_ppm),
                    BASE_LOCAL + elapsed_ns + delay_ns,
                    BASE_MONOTONIC + elapsed_ns + delay_ns,
                    clock_domain=DOMAIN,
                    clock_read_span_ns=10_000,
                )
                self.assertEqual(result.state, DisciplineState.QUALIFYING)
                latest = max(
                    latest, BASE_MONOTONIC + elapsed_ns + delay_ns
                )

        model = discipline.lock_affine_from_approved_drift(
            approved_common_drift_ppm=approved_drift_ppm,
            identity_evidence_verified=True,
            now_monotonic_ns=latest,
        )

        def observe_imu_at(seconds: int) -> tuple[int, object]:
            elapsed_ns = seconds * NS
            receipt_realtime_ns = BASE_LOCAL + elapsed_ns + 1_500_000
            receipt_monotonic_ns = BASE_MONOTONIC + elapsed_ns + 1_500_000
            for name, state in discipline._streams.items():
                if name != "mid360_imu":
                    state.last_advancing_receipt_monotonic_ns = (
                        receipt_monotonic_ns
                    )
            source_ns = source_at(elapsed_ns, continuing_drift_ppm)
            corrected_age_ns = (
                receipt_realtime_ns - model.corrected_stamp_ns(source_ns)
            )
            result = discipline.observe(
                "mid360_imu",
                source_ns,
                receipt_realtime_ns,
                receipt_monotonic_ns,
                clock_domain=DOMAIN,
                clock_read_span_ns=10_000,
            )
            return corrected_age_ns, result

        observed_age_ns, accepted = observe_imu_at(2_055)
        self.assertGreaterEqual(
            observed_age_ns, -discipline.config.max_corrected_future_ns
        )
        self.assertTrue(accepted.accepted, accepted.reason)
        self.assertEqual(accepted.state, DisciplineState.LOCKED)

        exhausted_age_ns, exhausted = observe_imu_at(7_000)
        self.assertLess(
            exhausted_age_ns, -discipline.config.max_corrected_future_ns
        )
        self.assertEqual(exhausted.state, DisciplineState.FAULTED)
        self.assertEqual(
            exhausted.reason,
            "corrected_timestamp_in_future:mid360_imu",
        )

    def test_motion_runtime_ready_uses_approved_drift_fast_path(self) -> None:
        approval = strict_affine_approval(
            "motion-approved-anchor-test", drift_ppm=13.0
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
                profile="motion",
            )
            latest = 0
            for index in range(12):
                elapsed_ns = index * 100_000_000
                for stream, delay_ns in (
                    ("mid360_imu", 1_500_000),
                    ("sport_primary", 2_000_000),
                    ("mid360_odom", 4_000_000),
                ):
                    runtime.discipline.observe(
                        stream,
                        source_at(elapsed_ns, 13.0),
                        BASE_LOCAL + elapsed_ns + delay_ns,
                        BASE_MONOTONIC + elapsed_ns + delay_ns,
                        clock_domain=DOMAIN,
                        clock_read_span_ns=10_000,
                    )
                    latest = max(
                        latest, BASE_MONOTONIC + elapsed_ns + delay_ns
                    )
            with mock.patch.object(
                stamp_node,
                "fresh_paired_receipt_clocks",
                return_value=fresh_clock_pair(
                    BASE_LOCAL + (latest - BASE_MONOTONIC), latest
                ),
            ), mock.patch.object(
                stamp_node, "require_strict_affine_approval"
            ):
                runtime.maybe_lock(latest)
            ready = json.loads(
                runtime.ready_file.read_text(encoding="utf-8")
            )

        self.assertTrue(runtime.ready)
        self.assertFalse(runtime.fault_file.exists())
        self.assertEqual(runtime.affine_terminal_full_evaluation_count, 0)
        self.assertEqual(ready["affine_model"]["drift_ppm"], 13.0)
        commit = ready["post_evaluation_commit"]
        self.assertEqual(
            commit["clock_base_ns"],
            commit["commit_check_realtime_ns"]
            - commit["commit_check_monotonic_ns"],
        )
        self.assertTrue(
            all(
                details["live"]
                for details in commit["stream_receipt_liveness"].values()
            )
        )

    def test_nomotion_cloud_delta_drop_contract_is_cloud_only_and_noncore(self) -> None:
        with self.assertRaisesRegex(ValueError, "only mid360_cloud"):
            AffineNavigationStampDiscipline(
                replace(
                    affine_config(),
                    affine_locked_noncore_delta_drop_streams=("mid360_imu",),
                ),
                core_streams=CORE_STREAMS,
            )
        with self.assertRaisesRegex(ValueError, "core streams cannot"):
            AffineNavigationStampDiscipline(
                nomotion_drop_affine_config(),
                core_streams=CORE_STREAMS + ("mid360_cloud",),
            )
        with self.assertRaisesRegex(ValueError, "only mid360_cloud"):
            AffineNavigationStampDiscipline(
                replace(
                    affine_config(),
                    affine_qualifying_noncore_delta_drop_streams=(
                        "mid360_imu",
                    ),
                ),
                core_streams=CORE_STREAMS,
            )

    def test_nomotion_core_delta_recovery_contract_is_core_only(self) -> None:
        with self.assertRaisesRegex(ValueError, "only affine core streams"):
            AffineNavigationStampDiscipline(
                replace(
                    affine_config(),
                    affine_locked_core_delta_recovery_streams=(
                        "sport_primary",
                        "mid360_cloud",
                    ),
                ),
                core_streams=CORE_STREAMS,
                allow_locked_receipt_recovery=True,
            )
        with self.assertRaisesRegex(ValueError, "must be disjoint"):
            replace(
                affine_config(),
                affine_locked_noncore_delta_drop_streams=("mid360_cloud",),
                affine_locked_core_delta_recovery_streams=("mid360_cloud",),
            ).validate()

    def test_nomotion_cloud_bounded_delta_discontinuity_is_dropped_while_qualifying(
        self,
    ) -> None:
        discipline = self.make_nomotion_drop_discipline()
        first = discipline.observe(
            "mid360_cloud",
            BASE_SOURCE,
            BASE_LOCAL,
            BASE_MONOTONIC,
            clock_domain=DOMAIN,
        )
        self.assertEqual(first.state, DisciplineState.QUALIFYING)
        state = discipline._streams["mid360_cloud"]
        before_samples = list(state.samples)
        before_last_good = (
            state.last_source_ns,
            state.last_advancing_receipt_monotonic_ns,
            state.advancing,
        )
        result = discipline.observe(
            "mid360_cloud",
            BASE_SOURCE + 1,
            BASE_LOCAL + 150_000_002,
            BASE_MONOTONIC + 150_000_002,
            clock_domain=DOMAIN,
        )
        self.assertEqual(result.state, DisciplineState.QUALIFYING)
        self.assertFalse(result.accepted)
        self.assertIsNone(result.corrected_stamp_ns)
        self.assertFalse(result.navigation_eligible)
        self.assertEqual(
            result.reason,
            f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud",
        )
        self.assertEqual(
            result.diagnostics,
            (
                ("source_delta_ns", 1),
                ("receipt_delta_ns", 150_000_002),
                ("delta_error_ns", 150_000_001),
                ("delta_error_limit_ns", 150_000_000),
            ),
        )
        self.assertEqual(list(state.samples), before_samples)
        self.assertEqual(
            (
                state.last_source_ns,
                state.last_advancing_receipt_monotonic_ns,
                state.advancing,
            ),
            before_last_good,
        )
        self.assertEqual(
            discipline._observed_continuity["mid360_cloud"],
            (BASE_SOURCE + 1, BASE_MONOTONIC + 150_000_002),
        )
        self.assertFalse(
            discipline._observed_continuity_is_last_good["mid360_cloud"]
        )

        recovered = discipline.observe(
            "mid360_cloud",
            BASE_SOURCE + 20_000_001,
            BASE_LOCAL + 170_000_002,
            BASE_MONOTONIC + 170_000_002,
            clock_domain=DOMAIN,
        )
        self.assertEqual(recovered.state, DisciplineState.QUALIFYING)
        self.assertEqual(recovered.reason, "affine_not_locked")
        self.assertEqual(len(state.samples), len(before_samples) + 1)
        self.assertEqual(
            state.last_advancing_receipt_monotonic_ns,
            BASE_MONOTONIC + 170_000_002,
        )

    def test_qualifying_nomotion_cloud_drop_ceilings_are_inclusive_and_bounded(
        self,
    ) -> None:
        cases = (
            (500_000_000, 300_000_000, False),
            (300_000_000, 500_000_000, False),
            (500_000_001, 350_000_000, True),
            (350_000_000, 500_000_001, True),
            (200_000_002, 1, True),
        )
        for source_delta_ns, receipt_delta_ns, faults in cases:
            with self.subTest(
                source_delta_ns=source_delta_ns,
                receipt_delta_ns=receipt_delta_ns,
            ):
                discipline = self.make_nomotion_drop_discipline()
                discipline.observe(
                    "mid360_cloud",
                    BASE_SOURCE,
                    BASE_LOCAL,
                    BASE_MONOTONIC,
                    clock_domain=DOMAIN,
                )
                result = discipline.observe(
                    "mid360_cloud",
                    BASE_SOURCE + source_delta_ns,
                    BASE_LOCAL + receipt_delta_ns,
                    BASE_MONOTONIC + receipt_delta_ns,
                    clock_domain=DOMAIN,
                )
                if faults:
                    self.assertEqual(result.state, DisciplineState.FAULTED)
                    self.assertEqual(
                        result.reason,
                        "source_receipt_delta_discontinuity:mid360_cloud",
                    )
                else:
                    self.assertEqual(
                        result.state, DisciplineState.QUALIFYING
                    )
                    self.assertEqual(
                        result.reason,
                        f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud",
                    )

    def test_qualifying_noncore_drop_is_isolated_from_core_and_motion_profiles(
        self,
    ) -> None:
        nomotion = self.make_nomotion_drop_discipline()
        nomotion.observe(
            "mid360_imu",
            BASE_SOURCE,
            BASE_LOCAL,
            BASE_MONOTONIC,
            clock_domain=DOMAIN,
        )
        core = nomotion.observe(
            "mid360_imu",
            BASE_SOURCE + 1,
            BASE_LOCAL + 150_000_002,
            BASE_MONOTONIC + 150_000_002,
            clock_domain=DOMAIN,
        )
        self.assertEqual(core.state, DisciplineState.FAULTED)
        self.assertEqual(
            core.reason, "source_receipt_delta_discontinuity:mid360_imu"
        )

        motion = AffineNavigationStampDiscipline(
            affine_config(), core_streams=CORE_STREAMS
        )
        motion.observe(
            "mid360_cloud",
            BASE_SOURCE,
            BASE_LOCAL,
            BASE_MONOTONIC,
            clock_domain=DOMAIN,
        )
        cloud = motion.observe(
            "mid360_cloud",
            BASE_SOURCE + 1,
            BASE_LOCAL + 150_000_002,
            BASE_MONOTONIC + 150_000_002,
            clock_domain=DOMAIN,
        )
        self.assertEqual(cloud.state, DisciplineState.FAULTED)
        self.assertEqual(
            cloud.reason,
            "source_receipt_delta_discontinuity:mid360_cloud",
        )

    def test_locked_nomotion_cloud_delta_boundary_drop_and_recovery(self) -> None:
        # The configured threshold remains inclusive.  This observation is
        # accepted at exactly 150 ms rather than being silently widened.
        at_limit = self.make_nomotion_drop_discipline()
        latest = qualify(at_limit)
        at_limit.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        continuity = at_limit._observed_continuity["mid360_cloud"]
        assert continuity is not None
        source_ns, receipt_monotonic_ns = continuity
        exact = at_limit.observe(
            "mid360_cloud",
            source_ns + 1,
            BASE_LOCAL
            + (receipt_monotonic_ns + 150_000_001 - BASE_MONOTONIC),
            receipt_monotonic_ns + 150_000_001,
            clock_domain=DOMAIN,
        )
        self.assertTrue(exact.accepted, exact.reason)
        self.assertEqual(exact.reason, "navigation_safe_affine")

        discipline = self.make_nomotion_drop_discipline()
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = discipline._streams["mid360_cloud"]
        windows = discipline._locked_corrected_ages
        assert windows is not None
        continuity = discipline._observed_continuity["mid360_cloud"]
        assert continuity is not None
        last_good = (
            state.last_source_ns,
            state.last_advancing_receipt_monotonic_ns,
            state.advancing,
        )
        before_samples = list(state.samples)
        before_cache = list(windows["mid360_cloud"])
        source_ns, receipt_monotonic_ns = continuity
        dropped_source_ns = source_ns + 1
        dropped_receipt_ns = receipt_monotonic_ns + 150_000_002
        dropped = discipline.observe(
            "mid360_cloud",
            dropped_source_ns,
            BASE_LOCAL + (dropped_receipt_ns - BASE_MONOTONIC),
            dropped_receipt_ns,
            clock_domain=DOMAIN,
        )

        self.assertEqual(dropped.state, DisciplineState.LOCKED)
        self.assertFalse(dropped.accepted)
        self.assertIsNone(dropped.corrected_stamp_ns)
        self.assertFalse(dropped.navigation_eligible)
        self.assertEqual(
            dropped.reason,
            f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud",
        )
        self.assertEqual(
            dropped.diagnostics,
            (
                ("source_delta_ns", 1),
                ("receipt_delta_ns", 150_000_002),
                ("delta_error_ns", 150_000_001),
                ("delta_error_limit_ns", 150_000_000),
            ),
        )
        self.assertEqual(list(state.samples), before_samples)
        self.assertEqual(list(windows["mid360_cloud"]), before_cache)
        self.assertEqual(
            (
                state.last_source_ns,
                state.last_advancing_receipt_monotonic_ns,
                state.advancing,
            ),
            last_good,
        )
        self.assertEqual(
            discipline._observed_continuity["mid360_cloud"],
            (dropped_source_ns, dropped_receipt_ns),
        )

        recovered_source_ns = dropped_source_ns + 20_000_000
        recovered_receipt_ns = dropped_receipt_ns + 20_000_000
        recovered = discipline.observe(
            "mid360_cloud",
            recovered_source_ns,
            BASE_LOCAL + (recovered_receipt_ns - BASE_MONOTONIC),
            recovered_receipt_ns,
            clock_domain=DOMAIN,
        )
        self.assertTrue(recovered.accepted, recovered.reason)
        self.assertEqual(recovered.reason, "navigation_safe_affine")
        self.assertEqual(len(state.samples), len(before_samples) + 1)
        self.assertEqual(len(windows["mid360_cloud"]), len(before_cache) + 1)
        self.assertEqual(len(state.samples), len(windows["mid360_cloud"]))
        self.assertEqual(state.last_source_ns, recovered_source_ns)
        self.assertEqual(
            state.last_advancing_receipt_monotonic_ns,
            recovered_receipt_ns,
        )

    def test_locked_core_delta_limit_plus_one_still_faults(self) -> None:
        for stream in CORE_STREAMS:
            with self.subTest(stream=stream):
                discipline = self.make_nomotion_drop_discipline()
                latest = qualify(discipline)
                discipline.lock_affine(
                    identity_evidence_verified=True,
                    now_monotonic_ns=latest,
                )
                continuity = discipline._observed_continuity[stream]
                assert continuity is not None
                source_ns, receipt_monotonic_ns = continuity
                result = discipline.observe(
                    stream,
                    source_ns + 1,
                    BASE_LOCAL
                    + (receipt_monotonic_ns + 150_000_002 - BASE_MONOTONIC),
                    receipt_monotonic_ns + 150_000_002,
                    clock_domain=DOMAIN,
                )
                self.assertEqual(result.state, DisciplineState.FAULTED)
                self.assertEqual(
                    result.reason,
                    f"source_receipt_delta_discontinuity:{stream}",
                )
                self.assertEqual(
                    dict(result.diagnostics)["delta_error_ns"], 150_000_001
                )
                self.assertEqual(
                    dict(result.diagnostics)["delta_error_limit_ns"],
                    150_000_000,
                )

    def test_cloud_delta_drop_defers_to_retained_hard_faults(self) -> None:
        cases = (
            (
                "missing_model",
                150_000_002,
                "affine_model_missing_after_lock",
            ),
            (
                "missing_cache",
                150_000_002,
                "affine_corrected_age_cache_missing_after_lock",
            ),
            (
                "misaligned_cache",
                150_000_002,
                "affine_corrected_age_cache_misaligned:mid360_cloud",
            ),
            (
                "stale_peer",
                200_000_001,
                "stream_stale:sport_primary:receipt_age_ns=200000001:"
                "limit_ns=200000000",
            ),
            (
                "scheduled_statistics",
                150_000_002,
                "forced_scheduled_statistics_fault",
            ),
        )
        for case, receipt_advance_ns, expected_reason in cases:
            with self.subTest(case=case):
                discipline = self.make_nomotion_drop_discipline()
                latest = qualify(discipline)
                discipline.lock_affine(
                    identity_evidence_verified=True,
                    now_monotonic_ns=latest,
                )
                state = discipline._streams["mid360_cloud"]
                windows = discipline._locked_corrected_ages
                assert windows is not None
                cloud_cache = windows["mid360_cloud"]
                continuity_before = discipline._observed_continuity[
                    "mid360_cloud"
                ]
                assert continuity_before is not None

                if case == "missing_model":
                    discipline.locked_affine_model = None
                elif case == "missing_cache":
                    discipline._locked_corrected_ages = None
                elif case == "misaligned_cache":
                    cloud_cache.pop()

                samples_before = list(state.samples)
                cache_before = list(cloud_cache)
                last_good_before = (
                    state.last_source_ns,
                    state.last_advancing_receipt_monotonic_ns,
                    state.advancing,
                    state.duplicates,
                )
                source_ns, receipt_monotonic_ns = continuity_before
                dropped_receipt_ns = (
                    receipt_monotonic_ns + receipt_advance_ns
                )

                if case == "stale_peer":
                    peer_receipt_ns = discipline._streams[
                        "sport_primary"
                    ].last_advancing_receipt_monotonic_ns
                    assert peer_receipt_ns is not None
                    expected_reason = (
                        "stream_stale:sport_primary:receipt_age_ns="
                        f"{dropped_receipt_ns - peer_receipt_ns}:"
                        "limit_ns=200000000"
                    )

                if case == "scheduled_statistics":
                    health = mock.patch.object(
                        discipline,
                        "_scheduled_statistics_health",
                        return_value=expected_reason,
                    )
                else:
                    health = mock.patch.object(
                        discipline,
                        "_scheduled_statistics_health",
                        wraps=discipline._scheduled_statistics_health,
                    )
                with health:
                    result = discipline.observe(
                        "mid360_cloud",
                        source_ns + 1,
                        BASE_LOCAL
                        + (dropped_receipt_ns - BASE_MONOTONIC),
                        dropped_receipt_ns,
                        clock_domain=DOMAIN,
                    )

                self.assertEqual(result.state, DisciplineState.FAULTED)
                self.assertEqual(result.reason, expected_reason)
                self.assertEqual(
                    discipline._observed_continuity["mid360_cloud"],
                    continuity_before,
                )
                self.assertEqual(list(state.samples), samples_before)
                self.assertEqual(list(cloud_cache), cache_before)
                self.assertEqual(
                    (
                        state.last_source_ns,
                        state.last_advancing_receipt_monotonic_ns,
                        state.advancing,
                        state.duplicates,
                    ),
                    last_good_before,
                )

    def test_duplicate_of_dropped_cloud_does_not_pollute_statistics(
        self,
    ) -> None:
        discipline = self.make_nomotion_drop_discipline()
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True,
            now_monotonic_ns=latest,
        )
        state = discipline._streams["mid360_cloud"]
        windows = discipline._locked_corrected_ages
        assert windows is not None
        continuity = discipline._observed_continuity["mid360_cloud"]
        assert continuity is not None
        source_ns, receipt_monotonic_ns = continuity
        dropped_source_ns = source_ns + 1
        dropped_receipt_ns = receipt_monotonic_ns + 150_000_002

        dropped = discipline.observe(
            "mid360_cloud",
            dropped_source_ns,
            BASE_LOCAL + (dropped_receipt_ns - BASE_MONOTONIC),
            dropped_receipt_ns,
            clock_domain=DOMAIN,
        )
        self.assertEqual(
            dropped.reason,
            f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud",
        )
        samples_after_drop = list(state.samples)
        cache_after_drop = list(windows["mid360_cloud"])
        last_good_after_drop = (
            state.last_source_ns,
            state.last_advancing_receipt_monotonic_ns,
            state.advancing,
        )
        duplicates_after_drop = state.duplicates
        metrics_after_drop = discipline._metrics("mid360_cloud")
        assert metrics_after_drop is not None

        duplicate = discipline.observe(
            "mid360_cloud",
            dropped_source_ns,
            BASE_LOCAL + (dropped_receipt_ns + 1 - BASE_MONOTONIC),
            dropped_receipt_ns + 1,
            clock_domain=DOMAIN,
        )
        self.assertEqual(duplicate.state, DisciplineState.LOCKED)
        self.assertEqual(
            duplicate.reason,
            "duplicate_source_timestamp:mid360_cloud",
        )
        self.assertEqual(state.duplicates, duplicates_after_drop)
        self.assertEqual(list(state.samples), samples_after_drop)
        self.assertEqual(list(windows["mid360_cloud"]), cache_after_drop)
        self.assertEqual(
            (
                state.last_source_ns,
                state.last_advancing_receipt_monotonic_ns,
                state.advancing,
            ),
            last_good_after_drop,
        )
        metrics_after_duplicate = discipline._metrics("mid360_cloud")
        assert metrics_after_duplicate is not None
        self.assertEqual(
            metrics_after_duplicate.duplicate_fraction,
            metrics_after_drop.duplicate_fraction,
        )

        recovered_source_ns = dropped_source_ns + 20_000_000
        recovered_receipt_ns = dropped_receipt_ns + 20_000_000
        recovered = discipline.observe(
            "mid360_cloud",
            recovered_source_ns,
            BASE_LOCAL + (recovered_receipt_ns - BASE_MONOTONIC),
            recovered_receipt_ns,
            clock_domain=DOMAIN,
        )
        self.assertTrue(recovered.accepted, recovered.reason)
        accepted_duplicate = discipline.observe(
            "mid360_cloud",
            recovered_source_ns,
            BASE_LOCAL + (recovered_receipt_ns + 1 - BASE_MONOTONIC),
            recovered_receipt_ns + 1,
            clock_domain=DOMAIN,
        )
        self.assertEqual(
            accepted_duplicate.reason,
            "duplicate_source_timestamp:mid360_cloud",
        )
        self.assertEqual(state.duplicates, duplicates_after_drop + 1)
        final_metrics = discipline._metrics("mid360_cloud")
        assert final_metrics is not None
        self.assertGreater(
            final_metrics.duplicate_fraction,
            metrics_after_duplicate.duplicate_fraction,
        )

    def test_continuous_cloud_drops_do_not_refresh_liveness(self) -> None:
        discipline = self.make_nomotion_drop_discipline()
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = discipline._streams["mid360_cloud"]
        windows = discipline._locked_corrected_ages
        assert windows is not None
        continuity = discipline._observed_continuity["mid360_cloud"]
        assert continuity is not None
        source_ns, receipt_monotonic_ns = continuity
        last_good_receipt_ns = state.last_advancing_receipt_monotonic_ns
        assert last_good_receipt_ns is not None
        before_samples = list(state.samples)
        before_cache = list(windows["mid360_cloud"])

        for _ in range(5):
            source_ns += 250_000_001
            receipt_monotonic_ns += 100_000_000
            # The non-core drop path deliberately evaluates every hard gate
            # before discarding a cloud callback.  Keep the three core
            # witnesses genuinely advancing so this test isolates the cloud's
            # unchanged last-good liveness clock instead of correctly tripping
            # a stale peer first.
            for peer_name in CORE_STREAMS:
                peer_continuity = discipline._observed_continuity[peer_name]
                assert peer_continuity is not None
                peer_source_ns, peer_receipt_ns = peer_continuity
                peer_delta_ns = receipt_monotonic_ns - peer_receipt_ns
                self.assertGreater(peer_delta_ns, 0)
                peer_result = discipline.observe(
                    peer_name,
                    peer_source_ns + peer_delta_ns,
                    BASE_LOCAL + (receipt_monotonic_ns - BASE_MONOTONIC),
                    receipt_monotonic_ns,
                    clock_domain=DOMAIN,
                )
                self.assertTrue(peer_result.accepted, peer_result.reason)
                self.assertEqual(peer_result.state, DisciplineState.LOCKED)
            result = discipline.observe(
                "mid360_cloud",
                source_ns,
                BASE_LOCAL + (receipt_monotonic_ns - BASE_MONOTONIC),
                receipt_monotonic_ns,
                clock_domain=DOMAIN,
            )
            self.assertEqual(result.state, DisciplineState.LOCKED)
            self.assertEqual(
                result.reason,
                f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud",
            )

        self.assertEqual(list(state.samples), before_samples)
        self.assertEqual(list(windows["mid360_cloud"]), before_cache)
        self.assertEqual(
            state.last_advancing_receipt_monotonic_ns,
            last_good_receipt_ns,
        )
        poll_ns = last_good_receipt_ns + 500_000_001
        result = discipline.poll(poll_ns)
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(
            result.reason,
            "stream_stale:mid360_cloud:receipt_age_ns=500000001:"
            "limit_ns=500000000",
        )

    def test_13ppm_qualifies_despite_callback_delay_bursts_and_freezes(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        self.assertEqual(discipline.qualification_reasons(latest), [])
        model = discipline.lock_affine(
            identity_evidence_verified=True,
            now_monotonic_ns=latest,
        )
        self.assertAlmostEqual(model.drift_ppm, 13.0, delta=0.2)
        self.assertAlmostEqual(
            model.source_to_local_scale,
            1.0 / (1.0 - 13.0e-6),
            delta=1e-10,
        )
        self.assertEqual(
            set(dict(model.core_stream_drifts_ppm)), set(CORE_STREAMS)
        )

        frozen = model
        for index in range(131, 171):
            latest = observe_set(
                discipline,
                local_elapsed_ns=index * 100_000_000,
                callback_burst_ns=80_000_000 if index % 10 == 4 else 0,
            )
            self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertIs(discipline.locked_affine_model, frozen)
        self.assertIs(
            discipline.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=latest,
            ),
            frozen,
        )

    def test_cloud_uses_common_model_and_preserves_acquisition_lag(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline, cloud_acquisition_lag_ns=70_000_000)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        self.assertNotIn("mid360_cloud", dict(model.core_stream_drifts_ppm))

        elapsed_ns = 13_100_000_000
        source_ns = source_at(elapsed_ns - 70_000_000, 13.0)
        receipt_realtime_ns = BASE_LOCAL + elapsed_ns + 5_000_000
        receipt_monotonic_ns = BASE_MONOTONIC + elapsed_ns + 5_000_000
        result = discipline.observe(
            "mid360_cloud",
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_domain=DOMAIN,
        )
        self.assertTrue(result.navigation_eligible, result.reason)
        assert result.corrected_stamp_ns is not None
        corrected_age_ns = receipt_realtime_ns - result.corrected_stamp_ns
        # The reference has a 2 ms minimum callback delay and the model keeps a
        # 5 ms reference guard.  Cloud therefore remains 70 + 5 + (5 - 2) =
        # 78 ms old; importantly, it is not independently pulled to 5 ms.
        self.assertGreater(corrected_age_ns, 77_000_000)
        self.assertLess(corrected_age_ns, 79_000_000)

    def test_persistent_corrected_staleness_faults_on_last_good_liveness(
        self,
    ) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )

        local_elapsed_ns = 13_100_000_000
        source_ns = source_at(local_elapsed_ns, 13.0)
        receipt_realtime_ns = BASE_LOCAL + local_elapsed_ns + 105_000_000
        receipt_monotonic_ns = BASE_MONOTONIC + local_elapsed_ns + 105_000_000
        expected_age_ns = receipt_realtime_ns - model.corrected_stamp_ns(source_ns)
        result = discipline.observe(
            "sport_primary",
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_domain=DOMAIN,
        )
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertTrue(
            result.reason.startswith("stream_stale:sport_primary:"),
            result.reason,
        )

        approval = strict_affine_approval(
            "affine-corrected-age-fault-test"
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            runtime.discipline = discipline
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            runtime.latch_fault(
                result.reason,
                fault_monotonic_ns=receipt_monotonic_ns,
                diagnostics=result.diagnostics,
            )

            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
        self.assertEqual(fault["reason"], result.reason)
        self.assertEqual(fault["trigger_stream"], "sport_primary")
        self.assertGreater(
            fault["trigger_receipt_age_ns"],
            fault["trigger_receipt_age_limit_ns"],
        )
        self.assertEqual(fault["trigger_receipt_age_limit_ns"], 200_000_000)
        self.assertEqual(fault["fault_monotonic_ns"], receipt_monotonic_ns)
        self.assertEqual(
            set(fault["stream_receipt_liveness"]),
            {"sport_primary", "mid360_imu", "mid360_cloud", "mid360_odom"},
        )
        self.assertGreater(expected_age_ns, 100_000_000)
        self.assertEqual(shutdowns, [True])

    def test_corrected_future_fault_records_exact_affine_age_and_limit(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )

        local_elapsed_ns = 13_100_000_000
        source_ns = source_at(local_elapsed_ns, 13.0)
        receipt_realtime_ns = BASE_LOCAL + local_elapsed_ns - 10_000_000
        receipt_monotonic_ns = BASE_MONOTONIC + local_elapsed_ns - 10_000_000
        expected_age_ns = receipt_realtime_ns - model.corrected_stamp_ns(source_ns)
        result = discipline.observe(
            "sport_primary",
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_domain=DOMAIN,
        )
        self.assertLess(
            expected_age_ns, -discipline.config.max_corrected_future_ns
        )
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(
            result.reason,
            "corrected_timestamp_in_future:sport_primary",
        )

        approval = strict_affine_approval(
            "affine-corrected-future-fault-test"
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            runtime.discipline = discipline
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            runtime.latch_fault(
                result.reason,
                fault_monotonic_ns=receipt_monotonic_ns,
                diagnostics=result.diagnostics,
            )

            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
        self.assertEqual(fault["reason"], result.reason)
        self.assertEqual(fault["trigger_stream"], "sport_primary")
        self.assertEqual(fault["trigger_corrected_age_ns"], expected_age_ns)
        self.assertEqual(
            fault["trigger_corrected_future_limit_ns"],
            discipline.config.max_corrected_future_ns,
        )
        self.assertEqual(fault["fault_monotonic_ns"], receipt_monotonic_ns)
        self.assertEqual(
            set(fault["stream_corrected_ages"]),
            {"sport_primary", "mid360_imu", "mid360_cloud", "mid360_odom"},
        )
        self.assertEqual(
            fault["stream_corrected_ages"]["sport_primary"][
                "corrected_age_ns"
            ],
            expected_age_ns,
        )
        self.assertFalse(
            fault["stream_corrected_ages"]["sport_primary"]["within_bounds"]
        )
        self.assertEqual(shutdowns, [True])

    def test_three_ppm_model_error_reaches_future_gate_after_long_extrapolation(
        self,
    ) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline, drift_ppm=13.0)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        self.assertAlmostEqual(model.drift_ppm, 13.0, places=6)

        # A 30 s-class fit which overestimates the true source-clock drift by
        # only 3 ppm consumes roughly 6.6 ms over 2,200 s.  That is enough to
        # exhaust the model's roughly 4.5 ms past-side guard and cross the
        # unchanged 2 ms future ceiling, while the single source/receipt delta
        # remains well inside this no-motion test profile's 150 ms breaker.
        local_elapsed_ns = 2_200 * NS
        source_ns = source_at(local_elapsed_ns, 10.0)
        receipt_realtime_ns = BASE_LOCAL + local_elapsed_ns + 1_500_000
        receipt_monotonic_ns = BASE_MONOTONIC + local_elapsed_ns + 1_500_000
        state = discipline._streams["mid360_imu"]
        assert state.last_source_ns is not None
        assert state.last_advancing_receipt_monotonic_ns is not None
        delta_error_ns = abs(
            (source_ns - state.last_source_ns)
            - (
                receipt_monotonic_ns
                - state.last_advancing_receipt_monotonic_ns
            )
        )
        limit_ns = discipline._policies[
            "mid360_imu"
        ].max_source_receipt_delta_error_ns
        assert limit_ns is not None
        self.assertLess(delta_error_ns, limit_ns)

        expected_age_ns = receipt_realtime_ns - model.corrected_stamp_ns(source_ns)
        result = discipline.observe(
            "mid360_imu",
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_domain=DOMAIN,
        )
        self.assertLess(
            expected_age_ns, -discipline.config.max_corrected_future_ns
        )
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(
            result.reason,
            "corrected_timestamp_in_future:mid360_imu",
        )

    def test_nomotion_past_guard_survives_observed_2205s_drift_then_soft_faults(
        self,
    ) -> None:
        config = hardened_nomotion_affine_config()
        discipline = AffineNavigationStampDiscipline(
            config, core_streams=CORE_STREAMS
        )
        latest = qualify(discipline, drift_ppm=13.0)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )

        def observe_imu_at(seconds: int) -> tuple[int, object]:
            local_elapsed_ns = seconds * NS
            source_ns = source_at(local_elapsed_ns, 10.0)
            receipt_realtime_ns = BASE_LOCAL + local_elapsed_ns + 1_500_000
            receipt_monotonic_ns = (
                BASE_MONOTONIC + local_elapsed_ns + 1_500_000
            )
            # This test isolates the per-sample affine-age breakers.  Model the
            # other high-rate streams as live and defer the periodic statistics
            # pass at this exact callback instant.
            for name, state in discipline._streams.items():
                if name != "mid360_imu":
                    state.last_advancing_receipt_monotonic_ns = (
                        receipt_monotonic_ns
                    )
            discipline._last_statistics_evaluation_monotonic_ns = (
                receipt_monotonic_ns
            )
            expected_age_ns = (
                receipt_realtime_ns - model.corrected_stamp_ns(source_ns)
            )
            result = discipline.observe(
                "mid360_imu",
                source_ns,
                receipt_realtime_ns,
                receipt_monotonic_ns,
                clock_domain=DOMAIN,
            )
            return expected_age_ns, result

        age_at_observed_fault_ns, accepted = observe_imu_at(2_205)
        self.assertGreaterEqual(age_at_observed_fault_ns, 5_000_000)
        self.assertTrue(accepted.accepted, accepted.reason)
        self.assertEqual(accepted.state, DisciplineState.LOCKED)

        exhausted_age_ns, exhausted = observe_imu_at(5_000)
        self.assertGreaterEqual(
            exhausted_age_ns, -config.max_corrected_future_ns
        )
        self.assertLess(
            exhausted_age_ns, config.minimum_locked_corrected_age_ns
        )
        self.assertEqual(exhausted.state, DisciplineState.FAULTED)
        self.assertEqual(
            exhausted.reason,
            "affine_corrected_age_margin_exhausted:mid360_imu",
        )
        self.assertEqual(
            dict(exhausted.diagnostics),
            {
                "corrected_age_ns": exhausted_age_ns,
                "minimum_locked_corrected_age_ns": 5_000_000,
            },
        )

        approval = strict_affine_approval(
            "affine-soft-margin-fault-test"
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            runtime.discipline = discipline
            shutdowns: list[bool] = []
            runtime.shutdown = lambda: shutdowns.append(True)
            runtime.latch_fault(
                exhausted.reason,
                fault_monotonic_ns=5_000 * NS,
                diagnostics=exhausted.diagnostics,
            )
            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))

        self.assertEqual(fault["trigger_stream"], "mid360_imu")
        self.assertEqual(fault["trigger_corrected_age_ns"], exhausted_age_ns)
        self.assertEqual(
            fault["trigger_minimum_locked_corrected_age_ns"],
            5_000_000,
        )
        self.assertEqual(fault["trigger_corrected_future_limit_ns"], 2_000_000)
        self.assertFalse(
            fault["stream_corrected_ages"]["mid360_imu"]["within_bounds"]
        )
        self.assertEqual(
            fault["timestamp_safety_limits"],
            {
                "offset_guard_ns": 5_000_000,
                "affine_anchor_past_guard_ns": 20_000_000,
                "max_corrected_future_ns": 2_000_000,
                "minimum_locked_corrected_age_ns": 5_000_000,
                "max_pairwise_drift_ppm": 25.0,
                "max_approved_affine_drift_deviation_ppm": 5.0,
                "affine_qualification_window_ns": None,
                "max_affine_window_common_drift_deviation_ppm": 5.0,
                "max_locked_affine_drift_deviation_ppm": 5.0,
            },
        )
        self.assertEqual(shutdowns, [True])

    def test_hard_future_fault_keeps_priority_over_affine_soft_margin(self) -> None:
        config = hardened_nomotion_affine_config()
        discipline = AffineNavigationStampDiscipline(
            config, core_streams=CORE_STREAMS
        )
        latest = qualify(discipline, drift_ppm=13.0)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        local_elapsed_ns = 8_000 * NS
        source_ns = source_at(local_elapsed_ns, 10.0)
        receipt_realtime_ns = BASE_LOCAL + local_elapsed_ns + 1_500_000
        receipt_monotonic_ns = BASE_MONOTONIC + local_elapsed_ns + 1_500_000
        for name, state in discipline._streams.items():
            if name != "mid360_imu":
                state.last_advancing_receipt_monotonic_ns = receipt_monotonic_ns
        discipline._last_statistics_evaluation_monotonic_ns = receipt_monotonic_ns

        expected_age_ns = receipt_realtime_ns - model.corrected_stamp_ns(source_ns)
        result = discipline.observe(
            "mid360_imu",
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_domain=DOMAIN,
        )

        self.assertLess(expected_age_ns, -config.max_corrected_future_ns)
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(
            result.reason,
            "corrected_timestamp_in_future:mid360_imu",
        )

    def test_affine_soft_floor_is_a_qualification_gate(self) -> None:
        insufficient = AffineNavigationStampDiscipline(
            affine_config(minimum_locked_corrected_age_ns=5_000_000),
            core_streams=CORE_STREAMS,
        )
        latest = qualify(insufficient)
        self.assertIn(
            "candidate_locked_corrected_age_below_minimum:mid360_imu",
            insufficient.qualification_reasons(latest),
        )
        with self.assertRaisesRegex(RuntimeError, "below_minimum"):
            insufficient.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=latest,
            )

        guarded = AffineNavigationStampDiscipline(
            hardened_nomotion_affine_config(),
            core_streams=CORE_STREAMS,
        )
        latest = qualify(guarded)
        self.assertEqual(guarded.qualification_reasons(latest), [])

    def test_locked_model_drift_threshold_is_independent_of_pairwise_gate(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(
                max_pairwise_drift_ppm=25.0,
                max_locked_affine_drift_deviation_ppm=5.0,
            ),
            core_streams=CORE_STREAMS,
        )
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        assert discipline.locked_affine_model is not None
        discipline.locked_affine_model = replace(
            discipline.locked_affine_model, drift_ppm=13.0
        )
        self.assertEqual(discipline.config.max_pairwise_drift_ppm, 25.0)

        for common_drift, expected in (
            (18.0, None),
            (18.000_001, "affine_model_drift_deviation_exceeded"),
        ):
            with self.subTest(common_drift=common_drift):
                core_metrics = {
                    name: AffineStreamMetrics(14, common_drift)
                    for name in CORE_STREAMS
                }
                with mock.patch.object(
                    discipline,
                    "_affine_metrics_for",
                    return_value=core_metrics,
                ):
                    self.assertEqual(discipline._statistical_health(), expected)
        diagnostics = discipline.affine_drift_diagnostics()
        assert diagnostics is not None
        self.assertEqual(diagnostics["comparison_phase"], "locked_health")
        self.assertEqual(diagnostics["locked_common_drift_ppm"], 13.0)
        self.assertEqual(
            diagnostics["current_common_drift_ppm"], 18.000_001
        )
        self.assertAlmostEqual(
            diagnostics["common_drift_deviation_ppm"], 5.000_001
        )
        self.assertEqual(
            set(diagnostics["locked_core_stream_drifts_ppm"]),
            set(CORE_STREAMS),
        )
        self.assertEqual(
            set(diagnostics["current_core_stream_drifts_ppm"]),
            set(CORE_STREAMS),
        )
        approval = strict_affine_approval(
            "locked-drift-fault-json-test"
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            runtime.discipline = discipline
            runtime.shutdown = lambda: None
            runtime.latch_fault("affine_model_drift_deviation_exceeded")
            fault = json.loads(runtime.fault_file.read_text(encoding="utf-8"))
        self.assertEqual(fault["affine_drift_diagnostics"], diagnostics)

    def test_identity_is_required_and_approved_offset_is_only_a_bound(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        with self.assertRaisesRegex(RuntimeError, "identity evidence"):
            discipline.lock_affine(
                identity_evidence_verified=False, now_monotonic_ns=latest
            )
        provisional = discipline._provisional_affine_model()
        assert provisional is not None
        approved = provisional.offset_at_source_ns(provisional.anchor_source_ns)
        model = discipline.lock_affine(
            identity_evidence_verified=True,
            now_monotonic_ns=latest,
            approved_reference_offset_ns=approved + 1_000_000,
        )
        self.assertEqual(model, discipline.locked_affine_model)

        rejected = self.make_discipline()
        latest = qualify(rejected)
        provisional = rejected._provisional_affine_model()
        assert provisional is not None
        with self.assertRaisesRegex(RuntimeError, "disagrees"):
            rejected.lock_affine(
                identity_evidence_verified=True,
                now_monotonic_ns=latest,
                approved_reference_offset_ns=(
                    provisional.offset_at_source_ns(provisional.anchor_source_ns)
                    + 21_000_000
                ),
            )

    def test_pairwise_core_rate_disagreement_refuses_qualification(self) -> None:
        discipline = self.make_discipline()
        latest = 0
        for index in range(131):
            elapsed_ns = index * 100_000_000
            for stream, delay_ns, drift_ppm in (
                ("sport_primary", 2_000_000, 13.0),
                ("mid360_imu", 1_500_000, 13.0),
                ("mid360_odom", 4_000_000, 45.0),
                ("mid360_cloud", 75_000_000, 13.0),
            ):
                source_ns = source_at(elapsed_ns, drift_ppm)
                receipt_realtime_ns = BASE_LOCAL + elapsed_ns + delay_ns
                receipt_monotonic_ns = BASE_MONOTONIC + elapsed_ns + delay_ns
                result = discipline.observe(
                    stream,
                    source_ns,
                    receipt_realtime_ns,
                    receipt_monotonic_ns,
                    clock_domain=DOMAIN,
                )
                self.assertNotEqual(result.state, DisciplineState.FAULTED)
                latest = max(latest, receipt_monotonic_ns)
        reasons = discipline.qualification_reasons(latest)
        self.assertIn("pairwise_drift_disagreement:mid360_odom", reasons)

    def test_pairwise_range_not_distance_from_median_is_enforced(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        split_metrics = {
            "sport_primary": AffineStreamMetrics(14, -25.0),
            "mid360_imu": AffineStreamMetrics(14, 0.0),
            "mid360_odom": AffineStreamMetrics(14, 25.0),
            "mid360_cloud": AffineStreamMetrics(14, 0.0),
        }
        with mock.patch.object(
            discipline, "affine_metrics", return_value=split_metrics
        ):
            reasons = discipline.qualification_reasons(latest)
        self.assertTrue(
            any(reason.startswith("pairwise_drift_disagreement:") for reason in reasons),
            reasons,
        )

        locked = self.make_discipline()
        latest = qualify(locked)
        locked.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        core_split_metrics = {
            name: split_metrics[name] for name in locked.core_streams
        }
        with mock.patch.object(
            locked, "_affine_metrics_for", return_value=core_split_metrics
        ) as runtime_metrics:
            runtime_reason = locked._statistical_health()
        runtime_metrics.assert_called_once()
        self.assertEqual(runtime_metrics.call_args.args, (locked.core_streams,))
        common_samples = runtime_metrics.call_args.kwargs["samples_by_stream"]
        self.assertEqual(set(common_samples), set(CORE_STREAMS))
        self.assertIsNotNone(runtime_reason)
        assert runtime_reason is not None
        self.assertTrue(
            runtime_reason.startswith("pairwise_drift_disagreement:"),
            runtime_reason,
        )

    def test_locked_health_skips_full_metrics_and_non_core_drift(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        original_lower_envelope = discipline_module._lower_envelope_points
        drift_streams: list[str] = []

        def tracked_lower_envelope(
            samples, bin_ns, *, anchor_monotonic_ns=0
        ):
            materialized = tuple(samples)
            drift_streams.append(CORE_STREAMS[len(drift_streams)])
            return original_lower_envelope(
                materialized,
                bin_ns,
                anchor_monotonic_ns=anchor_monotonic_ns,
            )

        with mock.patch.object(
            discipline,
            "metrics",
            side_effect=AssertionError("locked health called full metrics"),
        ), mock.patch.object(
            discipline_module,
            "_lower_envelope_points",
            side_effect=tracked_lower_envelope,
        ):
            self.assertIsNone(discipline._statistical_health())

        self.assertEqual(drift_streams, list(CORE_STREAMS))
        self.assertNotIn("mid360_cloud", drift_streams)

    def test_locked_health_uses_one_common_window_after_fifteen_minutes(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(retained_samples_per_stream=40_000),
            core_streams=CORE_STREAMS,
        )
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        install_synthetic_locked_history(discipline)

        self.assertIsNone(discipline._statistical_health())
        diagnostics = discipline.affine_drift_diagnostics()
        assert diagnostics is not None
        self.assertEqual(
            diagnostics["drift_window_span_ns"],
            discipline_module.LOCKED_AFFINE_DRIFT_WINDOW_NS,
        )
        self.assertEqual(
            diagnostics["bin_anchor_monotonic_ns"],
            discipline._affine_qualification_start_monotonic_ns,
        )
        stream_diagnostics = diagnostics["streams"]
        self.assertEqual(set(stream_diagnostics), set(CORE_STREAMS))
        retained_spans = set()
        current_drifts = []
        common_bounds = set()
        for name in CORE_STREAMS:
            details = stream_diagnostics[name]
            self.assertTrue(details["buffer_saturated_or_evicted"])
            self.assertGreater(details["retained_samples_evicted"], 0)
            self.assertEqual(details["retained_samples"], 40_000)
            self.assertGreaterEqual(details["envelope_points"], 60)
            retained_spans.add(details["retained_span_ns"])
            current_drifts.append(details["current_drift_ppm"])
            common_bounds.add(
                (
                    details["window_start_monotonic_ns"],
                    details["window_end_monotonic_ns"],
                )
            )
        self.assertEqual(len(common_bounds), 1)
        self.assertEqual(len(retained_spans), len(CORE_STREAMS))
        self.assertLess(max(current_drifts) - min(current_drifts), 0.01)
        self.assertEqual(
            diagnostics["current_clock_base_ns"],
            BASE_LOCAL - BASE_MONOTONIC,
        )
        self.assertGreater(diagnostics["locked_elapsed_ns"], 14 * 60 * NS)

    def test_common_window_keeps_rate_step_consistent_across_stream_rates(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(retained_samples_per_stream=40_000),
            core_streams=CORE_STREAMS,
        )
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        install_synthetic_locked_history(
            discipline,
            initial_drift_ppm=13.0,
            switched_drift_ppm=17.0,
            switch_elapsed_ns=14 * 60 * NS + 30 * NS,
        )

        unequal_full_buffer_drifts = [
            value.drift_ppm
            for value in discipline._affine_metrics_for(
                CORE_STREAMS
            ).values()
        ]
        self.assertIsNone(discipline._statistical_health())
        first = discipline.affine_drift_diagnostics()
        assert first is not None
        current = list(first["current_core_stream_drifts_ppm"].values())
        self.assertGreater(
            max(unequal_full_buffer_drifts)
            - min(unequal_full_buffer_drifts),
            0.5,
        )
        self.assertLess(max(current) - min(current), 0.1)
        self.assertLessEqual(
            first["common_drift_deviation_ppm"],
            discipline.config.max_locked_affine_drift_deviation_ppm,
        )
        self.assertIsNone(discipline._statistical_health())
        second = discipline.affine_drift_diagnostics()
        assert second is not None
        self.assertEqual(
            second["previous_core_stream_drifts_ppm"],
            first["current_core_stream_drifts_ppm"],
        )
        self.assertEqual(
            second["previous_common_drift_ppm"],
            first["current_common_drift_ppm"],
        )

    def test_lower_envelope_bin_anchor_does_not_roll_with_deque_head(self) -> None:
        anchor_ns = 100 * NS
        samples = (
            (BASE_SOURCE, anchor_ns + 900_000_000, 1),
            (BASE_SOURCE + 1, anchor_ns + NS + 100_000_000, 9),
            (BASE_SOURCE + 2, anchor_ns + NS + 900_000_000, 2),
            (BASE_SOURCE + 3, anchor_ns + 2 * NS + 100_000_000, 3),
        )
        before = discipline_module._lower_envelope_points(
            samples,
            NS,
            anchor_monotonic_ns=anchor_ns,
        )
        after_eviction = discipline_module._lower_envelope_points(
            samples[1:],
            NS,
            anchor_monotonic_ns=anchor_ns,
        )

        self.assertEqual(before, (samples[0], samples[2], samples[3]))
        self.assertEqual(after_eviction, before[1:])
        self.assertEqual(
            discipline_module._lower_envelope_points(samples[1:], NS),
            after_eviction,
        )

    def test_locked_common_drift_five_ppm_boundary_is_exact(self) -> None:
        base = AffineNavigationStampDiscipline(
            affine_config(max_locked_affine_drift_deviation_ppm=5.0),
            core_streams=CORE_STREAMS,
        )
        latest = qualify(base)
        base.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        assert base.locked_affine_model is not None
        base.locked_affine_model = replace(
            base.locked_affine_model, drift_ppm=13.0
        )

        for current_drift, expected in (
            (18.0, None),
            (
                math.nextafter(18.0, math.inf),
                "affine_model_drift_deviation_exceeded",
            ),
        ):
            with self.subTest(current_drift=current_drift):
                discipline = copy.deepcopy(base)
                core_metrics = {
                    name: AffineStreamMetrics(60, current_drift)
                    for name in CORE_STREAMS
                }
                with mock.patch.object(
                    discipline,
                    "_affine_metrics_for",
                    return_value=core_metrics,
                ):
                    self.assertEqual(
                        discipline._statistical_health(), expected
                    )
                diagnostics = discipline.affine_drift_diagnostics()
                assert diagnostics is not None
                self.assertEqual(
                    diagnostics["common_drift_deviation_limit_ppm"],
                    5.0,
                )

    def test_locked_common_window_500hz_health_check_performance(self) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(retained_samples_per_stream=40_000),
            core_streams=CORE_STREAMS,
        )
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        install_synthetic_locked_history(
            discipline,
            rates_hz={
                "sport_primary": 500,
                "mid360_imu": 500,
                "mid360_odom": 500,
                "mid360_cloud": 15,
            },
        )

        started = time.perf_counter()
        reason = discipline._statistical_health()
        elapsed = time.perf_counter() - started

        self.assertIsNone(reason)
        self.assertLess(elapsed, 1.0)
        diagnostics = discipline.affine_drift_diagnostics()
        assert diagnostics is not None
        for details in diagnostics["streams"].values():
            self.assertEqual(details["retained_samples"], 40_000)
            self.assertGreaterEqual(details["envelope_points"], 60)

    def test_locked_health_optimization_matches_legacy_oracle_at_boundaries(
        self,
    ) -> None:
        base = self.make_discipline()
        latest = qualify(base)
        base.lock_affine(identity_evidence_verified=True, now_monotonic_ns=latest)
        self.assert_locked_health_matches_legacy(base, None)

        missing_model = copy.deepcopy(base)
        missing_model.locked_affine_model = None
        self.assert_locked_health_matches_legacy(
            missing_model, "affine_model_missing_after_lock"
        )

        lost = copy.deepcopy(base)
        lost._streams["mid360_cloud"].samples.clear()
        lost._locked_corrected_ages["mid360_cloud"].clear()
        self.assert_locked_health_matches_legacy(lost, "stream_lost:mid360_cloud")

        duplicate_limit = copy.deepcopy(base)
        duplicate_limit._streams["sport_primary"].advancing = 90
        duplicate_limit._streams["sport_primary"].duplicates = 10
        self.assert_locked_health_matches_legacy(duplicate_limit, None)

        duplicate_exceeded = copy.deepcopy(base)
        duplicate_exceeded._streams["sport_primary"].advancing = 89
        duplicate_exceeded._streams["sport_primary"].duplicates = 11
        shift_locked_stream_ages(
            duplicate_exceeded,
            "sport_primary",
            duplicate_exceeded.config.max_locked_offset_deviation_ns + 1,
        )
        self.assert_locked_health_matches_legacy(
            duplicate_exceeded, "duplicate_fraction_exceeded:sport_primary"
        )

        residual_limit = copy.deepcopy(base)
        for name in residual_limit._streams:
            shift_locked_stream_ages(
                residual_limit,
                name,
                residual_limit.config.max_locked_offset_deviation_ns,
            )
        self.assert_locked_health_matches_legacy(residual_limit, None)

        residual_exceeded = copy.deepcopy(base)
        for name in residual_exceeded._streams:
            shift_locked_stream_ages(
                residual_exceeded,
                name,
                residual_exceeded.config.max_locked_offset_deviation_ns + 1,
            )
        self.assert_locked_health_matches_legacy(
            residual_exceeded, "affine_model_residual_exceeded:sport_primary"
        )

        relative_limit = copy.deepcopy(base)
        half_limit = relative_limit.config.max_locked_offset_deviation_ns // 2
        shift_locked_stream_ages(relative_limit, "sport_primary", -half_limit)
        shift_locked_stream_ages(relative_limit, "mid360_cloud", half_limit)
        self.assert_locked_health_matches_legacy(relative_limit, None)

        relative_exceeded = copy.deepcopy(base)
        shift_locked_stream_ages(relative_exceeded, "sport_primary", -half_limit)
        shift_locked_stream_ages(
            relative_exceeded, "mid360_cloud", half_limit + 1
        )
        self.assert_locked_health_matches_legacy(
            relative_exceeded, "stream_relative_lag_changed:mid360_cloud"
        )

    def test_locked_health_optimized_drift_gates_match_legacy_oracle(self) -> None:
        base = self.make_discipline()
        latest = qualify(base)
        base.lock_affine(identity_evidence_verified=True, now_monotonic_ns=latest)

        cases = (
            ({name: 13.0 for name in CORE_STREAMS}, 13.0, None),
            ({name: 50.0 for name in CORE_STREAMS}, 50.0, None),
            (
                {name: 50.000_001 for name in CORE_STREAMS},
                50.0,
                "clock_drift_exceeded:sport_primary",
            ),
            ({name: 38.0 for name in CORE_STREAMS}, 13.0, None),
            (
                {name: 38.000_001 for name in CORE_STREAMS},
                13.0,
                "affine_model_drift_deviation_exceeded",
            ),
            (
                {
                    "sport_primary": 0.0,
                    "mid360_imu": 25.0,
                    "mid360_odom": 25.0,
                },
                25.0,
                None,
            ),
            (
                {
                    "sport_primary": 0.0,
                    "mid360_imu": 25.000_001,
                    "mid360_odom": 25.0,
                },
                25.0,
                "pairwise_drift_disagreement:sport_primary",
            ),
        )
        for core_drifts, model_drift, expected in cases:
            with self.subTest(core_drifts=core_drifts, model_drift=model_drift):
                discipline = copy.deepcopy(base)
                assert discipline.locked_affine_model is not None
                discipline.locked_affine_model = replace(
                    discipline.locked_affine_model, drift_ppm=model_drift
                )
                all_metrics = {
                    name: AffineStreamMetrics(
                        14,
                        core_drifts.get(name),
                    )
                    for name in discipline._streams
                }
                core_metrics = {
                    name: all_metrics[name] for name in discipline.core_streams
                }
                with mock.patch.object(
                    discipline, "affine_metrics", return_value=all_metrics
                ), mock.patch.object(
                    discipline, "_affine_metrics_for", return_value=core_metrics
                ):
                    self.assert_locked_health_matches_legacy(discipline, expected)

        non_core_invalid = copy.deepcopy(base)
        all_metrics = {
            name: AffineStreamMetrics(14, 13.0)
            for name in non_core_invalid._streams
        }
        all_metrics["mid360_cloud"] = AffineStreamMetrics(14, None)
        core_metrics = {
            name: all_metrics[name] for name in non_core_invalid.core_streams
        }
        with mock.patch.object(
            non_core_invalid, "affine_metrics", return_value=all_metrics
        ), mock.patch.object(
            non_core_invalid, "_affine_metrics_for", return_value=core_metrics
        ):
            self.assert_locked_health_matches_legacy(non_core_invalid, None)

    def test_locked_corrected_age_cache_is_exact(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        windows = discipline._locked_corrected_ages
        assert windows is not None

        self.assertEqual(set(windows), set(discipline._streams))
        for name, state in discipline._streams.items():
            with self.subTest(stream=name):
                self.assertEqual(windows[name].maxlen, state.samples.maxlen)
                self.assertEqual(
                    list(windows[name]),
                    [
                        age_ns - model.offset_at_source_ns(source_ns)
                        for source_ns, _, age_ns in state.samples
                    ],
                )

    def test_locked_corrected_age_cache_rolls_with_samples(self) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(
                retained_samples_per_stream=120,
                statistics_evaluation_period_ns=5 * NS,
            ),
            core_streams=CORE_STREAMS,
        )
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )

        for index in range(131, 171):
            latest = observe_set(
                discipline,
                local_elapsed_ns=index * 100_000_000,
            )
            self.assertEqual(discipline.state, DisciplineState.LOCKED)

        windows = discipline._locked_corrected_ages
        assert windows is not None
        for name, state in discipline._streams.items():
            with self.subTest(stream=name):
                self.assertEqual(len(state.samples), 120)
                self.assertEqual(len(windows[name]), 120)
                self.assertEqual(windows[name].maxlen, state.samples.maxlen)
                self.assertEqual(
                    list(windows[name]),
                    [
                        age_ns - model.offset_at_source_ns(source_ns)
                        for source_ns, _, age_ns in state.samples
                    ],
                )

    def test_locked_corrected_age_cache_mismatch_faults_closed(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        windows = discipline._locked_corrected_ages
        assert windows is not None
        windows["mid360_cloud"].pop()
        discipline._last_statistics_evaluation_monotonic_ns = None

        result = discipline.poll(latest + 1)

        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(
            result.reason,
            "affine_corrected_age_cache_misaligned:mid360_cloud",
        )

    def test_cache_guard_precedes_candidate_age_handling(self) -> None:
        for callback_delay_ns, expected_kind in (
            (-10_000_000, "future"),
            (105_000_000, "stale"),
        ):
            with self.subTest(expected_kind=expected_kind):
                discipline = self.make_discipline()
                latest = qualify(discipline)
                model = discipline.lock_affine(
                    identity_evidence_verified=True, now_monotonic_ns=latest
                )
                windows = discipline._locked_corrected_ages
                assert windows is not None
                windows["sport_primary"].pop()

                elapsed_ns = 13_100_000_000
                source_ns = source_at(elapsed_ns, 13.0)
                receipt_realtime_ns = (
                    BASE_LOCAL + elapsed_ns + callback_delay_ns
                )
                receipt_monotonic_ns = (
                    BASE_MONOTONIC + elapsed_ns + callback_delay_ns
                )
                corrected_age_ns = (
                    receipt_realtime_ns - model.corrected_stamp_ns(source_ns)
                )
                result = discipline.observe(
                    "sport_primary",
                    source_ns,
                    receipt_realtime_ns,
                    receipt_monotonic_ns,
                    clock_domain=DOMAIN,
                )

                self.assertEqual(result.state, DisciplineState.FAULTED)
                self.assertEqual(
                    result.reason,
                    "affine_corrected_age_cache_misaligned:sport_primary",
                )
                if expected_kind == "future":
                    self.assertLess(
                        corrected_age_ns,
                        -discipline.config.max_corrected_future_ns,
                    )
                else:
                    self.assertGreater(
                        corrected_age_ns,
                        discipline._policies[
                            "sport_primary"
                        ].max_corrected_age_ns,
                    )

    def test_regression_and_forward_jump_latch_fail_closed(self) -> None:
        for mutation, expected in (
            ("regression", "source_timestamp_regression:sport_primary"),
            ("jump", "source_receipt_delta_discontinuity:sport_primary"),
        ):
            with self.subTest(mutation=mutation):
                discipline = self.make_discipline()
                latest = qualify(discipline)
                discipline.lock_affine(
                    identity_evidence_verified=True, now_monotonic_ns=latest
                )
                state = discipline._streams["sport_primary"]
                assert state.last_source_ns is not None
                assert state.last_advancing_receipt_monotonic_ns is not None
                source_ns = (
                    state.last_source_ns - 1
                    if mutation == "regression"
                    else state.last_source_ns + NS
                )
                receipt_monotonic_ns = (
                    state.last_advancing_receipt_monotonic_ns + 100_000_000
                )
                receipt_realtime_ns = (
                    BASE_LOCAL
                    + (receipt_monotonic_ns - BASE_MONOTONIC)
                )
                result = discipline.observe(
                    "sport_primary",
                    source_ns,
                    receipt_realtime_ns,
                    receipt_monotonic_ns,
                    clock_domain=DOMAIN,
                )
                self.assertEqual(result.state, DisciplineState.FAULTED)
                self.assertEqual(result.reason, expected)
                again = discipline.poll(receipt_monotonic_ns + 1)
                self.assertEqual(again.state, DisciplineState.FAULTED)
                self.assertEqual(again.reason, expected)

    def test_rate_change_after_lock_is_not_adapted_and_faults(self) -> None:
        config = affine_config(
            max_absolute_drift_ppm=100.0,
            max_pairwise_drift_ppm=10.0,
        )
        discipline = AffineNavigationStampDiscipline(
            config, core_streams=CORE_STREAMS
        )
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )

        # Continue from the exact source value at the switch, then change the
        # source rate by 40 ppm.  Per-message deltas remain plausible, while
        # the frozen-model statistical breaker must eventually reject it.
        switch_elapsed_ns = 13_000_000_000
        switch_source_ns = source_at(switch_elapsed_ns, 13.0)
        switched_drift_ppm = 53.0
        for step in range(1, 601):
            local_elapsed_ns = switch_elapsed_ns + step * 100_000_000
            source_elapsed_after_switch = round(
                step * 100_000_000 * (1.0 - switched_drift_ppm * 1e-6)
            )
            common_source_ns = switch_source_ns + source_elapsed_after_switch
            for stream, delay_ns, acquisition_lag_ns in (
                ("mid360_imu", 1_500_000, 0),
                ("sport_primary", 2_000_000, 0),
                ("mid360_odom", 4_000_000, 0),
                ("mid360_cloud", 5_000_000, 70_000_000),
            ):
                source_ns = common_source_ns - round(
                    acquisition_lag_ns * (1.0 - switched_drift_ppm * 1e-6)
                )
                receipt_realtime_ns = BASE_LOCAL + local_elapsed_ns + delay_ns
                receipt_monotonic_ns = (
                    BASE_MONOTONIC + local_elapsed_ns + delay_ns
                )
                discipline.observe(
                    stream,
                    source_ns,
                    receipt_realtime_ns,
                    receipt_monotonic_ns,
                    clock_domain=DOMAIN,
                )
                if discipline.state == DisciplineState.FAULTED:
                    break
            if discipline.state == DisciplineState.FAULTED:
                break
        self.assertEqual(discipline.state, DisciplineState.FAULTED)
        self.assertIn(
            discipline.fault_reason,
            {
                "affine_model_drift_deviation_exceeded",
                "affine_model_residual_exceeded:sport_primary",
            },
        )
        self.assertIs(discipline.locked_affine_model, model)

    def test_poll_faults_on_stream_outage(self) -> None:
        discipline = self.make_discipline()
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        result = discipline.poll(latest + 600_000_000)
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertTrue(result.reason.startswith("stream_stale:"), result.reason)

    def test_incomplete_locked_window_pauses_only_nomotion_profile(self) -> None:
        reason = "locked_affine_common_window_incomplete:sport_primary"
        nomotion = AffineNavigationStampDiscipline(
            affine_config(),
            core_streams=CORE_STREAMS,
            allow_locked_receipt_recovery=True,
        )
        latest = qualify(nomotion)
        nomotion.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        with mock.patch.object(
            nomotion, "_scheduled_statistics_health", return_value=reason
        ):
            paused = nomotion.poll(latest)
        self.assertEqual(paused.state, DisciplineState.LOCKED)
        self.assertFalse(paused.accepted)
        self.assertTrue(nomotion.receipt_recovery_active)
        self.assertIsNone(nomotion.fault_reason)
        self.assertEqual(
            paused.reason,
            "locked_receipt_recovery:" + reason,
        )

        motion = self.make_discipline()
        latest = qualify(motion)
        motion.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        with mock.patch.object(
            motion, "_scheduled_statistics_health", return_value=reason
        ):
            faulted = motion.poll(latest)
        self.assertEqual(faulted.state, DisciplineState.FAULTED)
        self.assertFalse(faulted.accepted)
        self.assertEqual(motion.fault_reason, reason)

    def test_nomotion_locked_receipt_stall_recovers_without_relock(self) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(),
            core_streams=CORE_STREAMS,
            allow_locked_receipt_recovery=True,
        )
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )

        # Replay the physical failure shape: one synchronous publish occupied
        # the executor for ~545 ms and the next timer observed every stream
        # just beyond its receipt deadline.
        paused = discipline.poll(latest + 550_000_000)
        self.assertEqual(paused.state, DisciplineState.LOCKED)
        self.assertTrue(
            paused.reason.startswith("locked_receipt_recovery:stream_stale:"),
            paused.reason,
        )
        self.assertTrue(discipline.receipt_recovery_active)
        self.assertIsNone(discipline.fault_reason)

        observe_set(discipline, local_elapsed_ns=13_550_000_000)
        self.assertTrue(discipline.receipt_recovery_active)
        self.assertTrue(
            all(
                count == 1
                for count in discipline._receipt_recovery_sample_counts.values()
            )
        )
        observe_set(discipline, local_elapsed_ns=13_650_000_000)

        self.assertFalse(discipline.receipt_recovery_active)
        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertIsNone(discipline.fault_reason)
        self.assertIs(discipline.locked_affine_model, model)
        self.assertEqual(discipline._receipt_recovery_count, 1)
        self.assertEqual(
            {gap[2] for gap in discipline._receipt_recovery_gap_waivers.values()},
            {550_000_000},
        )

        # The exact waiver remains tied to the recorded receipt pair while it
        # is inside the rolling window; the next health evaluation must not
        # reinterpret the same gap as a new outage.
        observe_set(discipline, local_elapsed_ns=13_750_000_000)
        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertIsNone(discipline.fault_reason)

    def test_motion_locked_receipt_stall_resumes_after_fresh_streams(self) -> None:
        config = replace(affine_config(), locked_statistics_enabled=False)
        discipline = AffineNavigationStampDiscipline(
            config,
            core_streams=CORE_STREAMS,
            allow_locked_receipt_recovery=True,
        )
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )

        paused = discipline.poll(latest + 550_000_000)
        self.assertEqual(paused.state, DisciplineState.LOCKED)
        self.assertTrue(discipline.receipt_recovery_active)

        with mock.patch.object(
            discipline, "_statistical_health"
        ) as statistical_health:
            observe_set(discipline, local_elapsed_ns=13_550_000_000)
            self.assertTrue(discipline.receipt_recovery_active)
            observe_set(discipline, local_elapsed_ns=13_650_000_000)

        statistical_health.assert_not_called()
        self.assertFalse(discipline.receipt_recovery_active)
        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertIsNone(discipline.fault_reason)
        self.assertIs(discipline.locked_affine_model, model)
        self.assertEqual(discipline._receipt_recovery_count, 1)

    def test_nomotion_recovery_waiver_survives_rolling_window_left_edge(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(),
            core_streams=CORE_STREAMS,
            allow_locked_receipt_recovery=True,
        )
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )

        paused = discipline.poll(latest + 550_000_000)
        self.assertEqual(paused.state, DisciplineState.LOCKED)
        observe_set(discipline, local_elapsed_ns=13_550_000_000)
        observe_set(discipline, local_elapsed_ns=13_650_000_000)
        self.assertFalse(discipline.receipt_recovery_active)

        # Once the common 60 s window's left edge moves just beyond the
        # pre-gap sample, the exact approved gap straddles that boundary.
        # The first in-window receipt is then farther from the boundary than
        # the normal liveness budget even though no second outage occurred.
        # Keep recognizing that exact recorded pair until its right edge also
        # rolls out of the window.
        for elapsed_ns in range(
            13_750_000_000,
            73_200_000_001,
            50_000_000,
        ):
            observe_set(discipline, local_elapsed_ns=elapsed_ns)
            if discipline.state == DisciplineState.FAULTED:
                break

        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertIsNone(discipline.fault_reason)
        self.assertIs(discipline.locked_affine_model, model)
        self.assertTrue(discipline._receipt_recovery_gap_waivers)

    def test_locked_transient_corrected_staleness_drops_without_refresh(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(),
            core_streams=CORE_STREAMS,
            allow_locked_receipt_recovery=True,
        )
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        state = discipline._streams["sport_primary"]
        windows = discipline._locked_corrected_ages
        assert windows is not None
        samples_before = tuple(state.samples)
        ages_before = tuple(windows["sport_primary"])
        last_good_receipt_before = (
            state.last_advancing_receipt_monotonic_ns
        )
        advancing_before = state.advancing

        # Keep receipt/source continuity within its independent 150 ms limit,
        # but make one Sport callback old enough to cross the 100 ms
        # operational age limit while remaining below the 200 ms hard ceiling.
        elapsed_ns = 13_050_000_000
        source_ns = source_at(elapsed_ns, 13.0)
        receipt_realtime_ns = BASE_LOCAL + elapsed_ns + 102_000_000
        receipt_monotonic_ns = BASE_MONOTONIC + elapsed_ns + 102_000_000
        paused = discipline.observe(
            "sport_primary",
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_domain=DOMAIN,
            clock_read_span_ns=10_000,
        )
        self.assertEqual(paused.state, DisciplineState.LOCKED)
        self.assertFalse(paused.accepted)
        self.assertIsNone(paused.corrected_stamp_ns)
        self.assertEqual(
            paused.reason,
            f"{LOCKED_STALE_SAMPLE_DROP_REASON_PREFIX}:sport_primary",
        )
        self.assertFalse(discipline.receipt_recovery_active)
        self.assertIsNone(discipline.fault_reason)
        self.assertEqual(tuple(state.samples), samples_before)
        self.assertEqual(tuple(windows["sport_primary"]), ages_before)
        self.assertEqual(
            state.last_advancing_receipt_monotonic_ns,
            last_good_receipt_before,
        )
        self.assertEqual(state.advancing, advancing_before)
        self.assertEqual(
            discipline._observed_continuity["sport_primary"],
            (source_ns, receipt_monotonic_ns),
        )
        self.assertFalse(
            discipline._observed_continuity_is_last_good["sport_primary"]
        )

        observe_set(discipline, local_elapsed_ns=13_180_000_000)
        self.assertFalse(discipline.receipt_recovery_active)
        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertIsNone(discipline.fault_reason)
        self.assertIs(discipline.locked_affine_model, model)
        self.assertGreater(state.advancing, advancing_before)
        self.assertEqual(
            len(windows["sport_primary"]), len(state.samples)
        )

    def test_locked_stale_samples_never_use_a_separate_hard_age_gate(
        self,
    ) -> None:
        base = nomotion_core_recovery_affine_config()
        config = replace(
            base,
            streams=tuple(
                replace(
                    policy,
                    max_corrected_age_ns=(
                        150_000_000
                        if policy.name == "sport_primary"
                        else policy.max_corrected_age_ns
                    ),
                    hard_age_ceiling_ns=2_000_000_000,
                    stale_receipt_timeout_ns=500_000_000,
                )
                for policy in base.streams
            ),
        )

        for corrected_age_ns in (
            608_872_160,
            2_000_000_000,
            2_000_000_001,
        ):
            with self.subTest(corrected_age_ns=corrected_age_ns):
                discipline = AffineNavigationStampDiscipline(
                    config,
                    core_streams=CORE_STREAMS,
                    allow_locked_receipt_recovery=True,
                )
                latest = qualify(discipline)
                model = discipline.lock_affine(
                    identity_evidence_verified=True,
                    now_monotonic_ns=latest,
                )
                stream = "sport_primary"
                previous_source_ns, _ = discipline._observed_continuity[stream]
                assert previous_source_ns is not None
                source_ns = source_at(13_100_000_000, 13.0)
                corrected_ns = model.corrected_stamp_ns(source_ns)
                receipt_realtime_ns = corrected_ns + corrected_age_ns
                receipt_monotonic_ns = receipt_realtime_ns - (
                    BASE_LOCAL - BASE_MONOTONIC
                )
                source_delta_ns = source_ns - previous_source_ns
                discipline._observed_continuity[stream] = (
                    previous_source_ns,
                    receipt_monotonic_ns - source_delta_ns,
                )
                for state in discipline._streams.values():
                    state.last_advancing_receipt_monotonic_ns = (
                        receipt_monotonic_ns
                    )

                result = discipline.observe(
                    stream,
                    source_ns,
                    receipt_realtime_ns,
                    receipt_monotonic_ns,
                    clock_domain=DOMAIN,
                    clock_read_span_ns=10_000,
                )
                self.assertFalse(result.accepted)
                self.assertIsNone(result.corrected_stamp_ns)
                self.assertEqual(result.state, DisciplineState.LOCKED)
                self.assertFalse(discipline.receipt_recovery_active)
                self.assertEqual(
                    result.reason,
                    f"{LOCKED_STALE_SAMPLE_DROP_REASON_PREFIX}:"
                    "sport_primary",
                )
                self.assertIsNone(discipline.fault_reason)

    def test_nomotion_receipt_recovery_keeps_regression_hard(self) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(),
            core_streams=CORE_STREAMS,
            allow_locked_receipt_recovery=True,
        )
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        paused = discipline.poll(latest + 550_000_000)
        self.assertEqual(paused.state, DisciplineState.LOCKED)
        continuity = discipline._observed_continuity["sport_primary"]
        assert continuity is not None

        result = discipline.observe(
            "sport_primary",
            continuity[0] - 1,
            BASE_LOCAL + 13_550_000_000,
            BASE_MONOTONIC + 13_550_000_000,
            clock_domain=DOMAIN,
        )
        self.assertEqual(result.state, DisciplineState.FAULTED)
        self.assertEqual(
            result.reason, "source_timestamp_regression:sport_primary"
        )

    def test_nomotion_recovery_discards_catchup_delta_without_output(self) -> None:
        discipline = AffineNavigationStampDiscipline(
            affine_config(),
            core_streams=CORE_STREAMS,
            allow_locked_receipt_recovery=True,
        )
        latest = qualify(discipline)
        discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        paused = discipline.poll(latest + 550_000_000)
        self.assertEqual(paused.state, DisciplineState.LOCKED)

        continuity = discipline._observed_continuity["sport_primary"]
        assert continuity is not None
        state = discipline._streams["sport_primary"]
        last_good = (
            state.last_source_ns,
            state.last_advancing_receipt_monotonic_ns,
            len(state.samples),
        )
        source_ns = continuity[0] + 113_843_761
        receipt_ns = continuity[1] + 754_704_469
        result = discipline.observe(
            "sport_primary",
            source_ns,
            receipt_ns + (BASE_LOCAL - BASE_MONOTONIC),
            receipt_ns,
            clock_domain=DOMAIN,
        )
        self.assertEqual(result.state, DisciplineState.LOCKED)
        self.assertFalse(result.accepted)
        self.assertIsNone(result.corrected_stamp_ns)
        self.assertTrue(
            result.reason.startswith(
                "locked_receipt_recovery:"
                "locked_receipt_recovery_delta_discontinuity_dropped:"
                "sport_primary"
            ),
            result.reason,
        )
        self.assertEqual(
            (
                state.last_source_ns,
                state.last_advancing_receipt_monotonic_ns,
                len(state.samples),
            ),
            last_good,
        )
        self.assertEqual(
            discipline._observed_continuity["sport_primary"],
            (source_ns, receipt_ns),
        )
        self.assertFalse(
            discipline._observed_continuity_is_last_good["sport_primary"]
        )
        self.assertIsNone(discipline.fault_reason)

    def test_nomotion_core_delta_jitter_pauses_and_requalifies_without_relock(
        self,
    ) -> None:
        discipline = AffineNavigationStampDiscipline(
            nomotion_core_recovery_affine_config(),
            core_streams=CORE_STREAMS,
            allow_locked_receipt_recovery=True,
        )
        latest = qualify(discipline)
        model = discipline.lock_affine(
            identity_evidence_verified=True, now_monotonic_ns=latest
        )
        continuity = discipline._observed_continuity["sport_primary"]
        assert continuity is not None
        state = discipline._streams["sport_primary"]
        last_good = (
            state.last_source_ns,
            state.last_advancing_receipt_monotonic_ns,
            len(state.samples),
        )

        # Scale the physical 296 ms source / 483 ms receipt callback wobble to
        # this compact profile: every peer remains inside its 200 ms liveness
        # ceiling, while the one Sport delta error is 170 ms > the unchanged
        # 150 ms operational breaker.
        jitter_source_ns = continuity[0] + 20_000_000
        jitter_receipt_ns = continuity[1] + 190_000_000
        paused = discipline.observe(
            "sport_primary",
            jitter_source_ns,
            jitter_receipt_ns + (BASE_LOCAL - BASE_MONOTONIC),
            jitter_receipt_ns,
            clock_domain=DOMAIN,
        )
        self.assertEqual(paused.state, DisciplineState.LOCKED)
        self.assertFalse(paused.accepted)
        self.assertIsNone(paused.corrected_stamp_ns)
        self.assertEqual(
            paused.reason,
            "locked_receipt_recovery:"
            "locked_receipt_recovery_delta_discontinuity_dropped:"
            "sport_primary",
        )
        self.assertEqual(
            dict(paused.diagnostics)["delta_error_ns"], 170_000_000
        )
        self.assertTrue(discipline.receipt_recovery_active)
        self.assertIsNone(discipline.fault_reason)
        self.assertEqual(
            (
                state.last_source_ns,
                state.last_advancing_receipt_monotonic_ns,
                len(state.samples),
            ),
            last_good,
        )
        self.assertEqual(
            discipline._observed_continuity["sport_primary"],
            (jitter_source_ns, jitter_receipt_ns),
        )
        self.assertFalse(
            discipline._observed_continuity_is_last_good["sport_primary"]
        )

        # The first catch-up callback is discarded too.  It advances only the
        # structural baseline and cannot count toward the two fresh samples.
        catchup_source_ns = jitter_source_ns + 170_000_000
        catchup_receipt_ns = jitter_receipt_ns + 10_000_000
        catchup = discipline.observe(
            "sport_primary",
            catchup_source_ns,
            catchup_receipt_ns + (BASE_LOCAL - BASE_MONOTONIC),
            catchup_receipt_ns,
            clock_domain=DOMAIN,
        )
        self.assertFalse(catchup.accepted)
        self.assertTrue(
            catchup.reason.startswith(
                "locked_receipt_recovery:"
                "locked_receipt_recovery_delta_discontinuity_dropped:"
                "sport_primary"
            ),
            catchup.reason,
        )
        self.assertEqual(len(state.samples), last_good[2])

        # Bring all four streams to two common fresh receipt instants while
        # preserving each stream's last observed source/receipt delta.  No
        # corrected output may resume before the complete set and locked
        # statistics are healthy.
        first_target_ns = max(
            value[1]
            for value in discipline._observed_continuity.values()
            if value is not None
        ) + 10_000_000
        observed_results = []
        for target_ns in (first_target_ns, first_target_ns + 30_000_000):
            for stream in discipline._streams:
                stream_continuity = discipline._observed_continuity[stream]
                assert stream_continuity is not None
                source_ns, receipt_ns = stream_continuity
                delta_ns = target_ns - receipt_ns
                self.assertGreater(delta_ns, 0)
                observed_results.append(
                    discipline.observe(
                        stream,
                        source_ns + delta_ns,
                        target_ns + (BASE_LOCAL - BASE_MONOTONIC),
                        target_ns,
                        clock_domain=DOMAIN,
                    )
                )

        self.assertTrue(
            any(not result.accepted for result in observed_results[:-1])
        )
        self.assertFalse(discipline.receipt_recovery_active)
        self.assertEqual(discipline.state, DisciplineState.LOCKED)
        self.assertIsNone(discipline.fault_reason)
        self.assertIs(discipline.locked_affine_model, model)
        self.assertEqual(discipline._receipt_recovery_count, 1)

    def test_core_delta_recovery_never_weakens_motion_or_hard_ceiling(
        self,
    ) -> None:
        for config, allow_recovery, source_delta_ns, receipt_delta_ns in (
            # Motion/default affine configuration has no recovery stream opt-in.
            (affine_config(), False, 20_000_000, 190_000_000),
            # Even the no-motion contract cannot recover a source jump whose
            # delta error exceeds the stream's unchanged 200 ms hard ceiling.
            (
                nomotion_core_recovery_affine_config(),
                True,
                511_000_000,
                10_000_000,
            ),
        ):
            with self.subTest(
                allow_recovery=allow_recovery,
                source_delta_ns=source_delta_ns,
            ):
                discipline = AffineNavigationStampDiscipline(
                    config,
                    core_streams=CORE_STREAMS,
                    allow_locked_receipt_recovery=allow_recovery,
                )
                latest = qualify(discipline)
                discipline.lock_affine(
                    identity_evidence_verified=True,
                    now_monotonic_ns=latest,
                )
                continuity = discipline._observed_continuity["sport_primary"]
                assert continuity is not None
                result = discipline.observe(
                    "sport_primary",
                    continuity[0] + source_delta_ns,
                    continuity[1]
                    + receipt_delta_ns
                    + (BASE_LOCAL - BASE_MONOTONIC),
                    continuity[1] + receipt_delta_ns,
                    clock_domain=DOMAIN,
                )
                self.assertEqual(result.state, DisciplineState.FAULTED)
                self.assertEqual(
                    result.reason,
                    "source_receipt_delta_discontinuity:sport_primary",
                )
                self.assertFalse(discipline.receipt_recovery_active)

    def test_locked_receipt_recovery_flag_is_explicit_and_boolean(self) -> None:
        with self.assertRaisesRegex(ValueError, "boolean"):
            AffineNavigationStampDiscipline(
                affine_config(),
                core_streams=CORE_STREAMS,
                allow_locked_receipt_recovery=1,
            )

    def test_fixed_offset_class_remains_unchanged(self) -> None:
        # This focuses the compatibility contract: the affine class is opt-in
        # and does not weaken or replace the existing fixed-offset class.
        from navigation_stamp_discipline import NavigationStampDiscipline

        fixed = NavigationStampDiscipline(
            replace(
                affine_config(),
                streams=affine_config().streams[:3],
            )
        )
        self.assertFalse(hasattr(fixed, "locked_affine_model"))

    def test_stamp_node_affine_mode_is_explicit_and_reports_frozen_model(self) -> None:
        approval = strict_affine_approval("affine-node-test")
        model = AffineClockModel(
            anchor_source_ns=BASE_SOURCE,
            anchor_local_ns=BASE_LOCAL - 3_000_000,
            drift_ppm=13.0,
            source_to_local_scale=1.0 / (1.0 - 13.0e-6),
            core_stream_drifts_ppm=tuple((name, 13.0) for name in CORE_STREAMS),
            stream_baseline_corrected_age_ns=(
                ("sport_primary", 20_000_000),
                ("mid360_imu", 19_500_000),
                ("mid360_cloud", 93_000_000),
                ("mid360_odom", 22_000_000),
            ),
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
            )
            self.assertIsInstance(
                runtime.discipline, AffineNavigationStampDiscipline
            )
            snapshot = mock.sentinel.qualification_snapshot
            evaluation = SimpleNamespace(reasons=())
            runtime.discipline.affine_qualification_deadline_monotonic_ns = (
                mock.Mock(return_value=BASE_MONOTONIC + 60 * NS)
            )
            runtime.discipline.capture_affine_qualification_snapshot = mock.Mock(
                return_value=snapshot
            )
            runtime.discipline.evaluate_affine_qualification_snapshot = mock.Mock(
                return_value=evaluation
            )
            post_commit = {
                "snapshot_evaluation_monotonic_ns": BASE_MONOTONIC + 60 * NS,
                "commit_check_realtime_ns": time.time_ns(),
                "commit_check_monotonic_ns": BASE_MONOTONIC + 60 * NS,
                "evaluation_to_commit_check_ns": 0,
                "clock_read_span_ns": 0,
                "clock_read_span_limit_ns": 1_000_000,
                "previous_clock_base_ns": 1,
                "clock_base_ns": 1,
                "clock_base_delta_ns": 0,
                "clock_pair_discontinuity_limit_ns": 20_000_000,
                "stream_receipt_liveness": {},
            }
            runtime.discipline.affine_post_evaluation_commit_gate = mock.Mock(
                return_value=((), post_commit)
            )
            runtime.discipline.qualification_reasons = mock.Mock()
            runtime.discipline.lock_affine = mock.Mock(return_value=model)
            runtime.maybe_lock(BASE_MONOTONIC + 60 * NS - 1)
            runtime.discipline.qualification_reasons.assert_not_called()
            runtime.discipline.capture_affine_qualification_snapshot.assert_not_called()
            with mock.patch.object(
                stamp_node,
                "fresh_paired_receipt_clocks",
                return_value=fresh_clock_pair(
                    time.time_ns(), BASE_MONOTONIC + 60 * NS
                ),
            ):
                runtime.maybe_lock(BASE_MONOTONIC + 60 * NS)
            runtime.maybe_lock(BASE_MONOTONIC + 61 * NS)

            ready = json.loads(runtime.ready_file.read_text(encoding="utf-8"))
            self.assertEqual(ready["schema"], stamp_node.AFFINE_READY_SCHEMA)
            self.assertEqual(ready["correction_mode"], "affine")
            self.assertEqual(ready["discipline_profile"], "nomotion")
            self.assertTrue(ready["affine_model"]["frozen"])
            self.assertEqual(ready["affine_model"]["drift_ppm"], 13.0)
            self.assertEqual(
                ready["timestamp_safety_limits"],
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
            self.assertEqual(
                set(ready["corrected_topics"]),
                {
                    "sport_primary",
                    "mid360_imu",
                    "mid360_cloud",
                    "mid360_odom",
                },
            )
            self.assertTrue(ready["time_discipline_ready"])
            self.assertFalse(ready["motion_ready"])
            self.assertFalse(ready["canonical_odom_ready"])
            self.assertEqual(
                ready["approval_affine_common_drift_ppm"], 13.0
            )
            self.assertNotIn("fixed_local_minus_source_offset_ns", ready)
            runtime.discipline.lock_affine.assert_called_once_with(
                identity_evidence_verified=True,
                now_monotonic_ns=BASE_MONOTONIC + 60 * NS,
                approved_reference_offset_ns=None,
                approved_common_drift_ppm=None,
                qualification_evaluation=evaluation,
            )
            self.assertEqual(runtime.affine_terminal_full_evaluation_count, 1)
            runtime.discipline.evaluate_affine_qualification_snapshot.assert_called_once_with(
                snapshot
            )

        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            fixed = stamp_node._Runtime(
                approval, directory / "ready.json", directory / "fault.json"
            )
            self.assertEqual(fixed.mode, "fixed")
            self.assertEqual(fixed.profile, "nomotion")
            self.assertNotIsInstance(
                fixed.discipline, AffineNavigationStampDiscipline
            )
        with self.assertRaisesRegex(ValueError, "unsupported"):
            stamp_node._Runtime(
                approval, Path("/tmp/unused-ready"), Path("/tmp/unused-fault"), mode="auto"
            )
        with self.assertRaisesRegex(ValueError, "requires affine"):
            stamp_node._Runtime(
                approval,
                Path("/tmp/unused-motion-ready"),
                Path("/tmp/unused-motion-fault"),
                mode="fixed",
                profile="motion",
            )


if __name__ == "__main__":
    unittest.main()
