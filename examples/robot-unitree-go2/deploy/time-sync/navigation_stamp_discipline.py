#!/usr/bin/env python3
"""Pure, fail-closed timestamp discipline for a Go2 navigation session.

This module deliberately has no ROS, network, subprocess, or clock-setting
dependency.  It does not make a timestamp safe merely by replacing it with
receipt time.  It provides two explicit disciplines:

* the original immutable ``local - source`` offset, retained for reviewed
  fixed-offset sessions; and
* a frozen affine source-to-local model for a source clock whose rate differs
  slightly from the workstation clock.

The affine rate is qualified from the one-second lower envelopes of multiple
core streams.  One common model is then applied to every stream, including
PointCloud2, so sensor acquisition lag is preserved instead of being aligned
away independently.

The discipline is intended to be embedded behind the existing per-message
freshness guards.  The normal startup and motion path do not use it; one
explicit no-motion workstation wrapper uses it only for private corrected
topics.  A caller must also prove the source clock-domain identity (for
example from retained RTPS writer/locator evidence) before a lock can succeed.
"""

from __future__ import annotations

from collections import deque
import copy
from dataclasses import dataclass, field, replace
from enum import Enum
import math
import statistics
from typing import Iterable


NANOSECONDS_PER_SECOND = 1_000_000_000
# Post-lock clock-rate evidence is intentionally bounded by elapsed receipt
# time, not by a sample count.  The checked-in 40,000-sample buffers cover at
# least this interval at the documented 500 Hz ceiling, while lower-rate core
# streams are evaluated over the exact same monotonic interval.
LOCKED_AFFINE_DRIFT_WINDOW_NS = 60 * NANOSECONDS_PER_SECOND
# A previously reviewed affine rate does not need to be re-fitted after every
# robot or private-Wi-Fi restart.  One recent second is enough to bind that
# approved rate to the current source/local intercept while all ordinary
# monotonicity, corrected-age and receipt-liveness checks remain active.
APPROVED_AFFINE_ANCHOR_WINDOW_NS = NANOSECONDS_PER_SECOND
NONCORE_DELTA_DROP_REASON_PREFIX = (
    "noncore_source_receipt_delta_discontinuity_dropped"
)
LOCKED_STALE_SAMPLE_DROP_REASON_PREFIX = (
    "locked_corrected_timestamp_stale_dropped"
)
LOCKED_RECEIPT_RECOVERY_REASON_PREFIX = "locked_receipt_recovery"
LOCKED_RECEIPT_RECOVERY_DELTA_DROP_REASON_PREFIX = (
    "locked_receipt_recovery_delta_discontinuity_dropped"
)
LOCKED_RECEIPT_RECOVERY_REQUIRED_SAMPLES = 2
NOMOTION_TRANSIENT_STALE_RECOVERY_CEILING_NS = 2_000_000_000
# Robonix control-plane + chassis cold starts on this workstation produced a
# single 50.962 ms source/receipt period disagreement and two otherwise healthy
# IMU callbacks at 101.084288 ms and 100.929220 ms corrected age.  Later
# staged-navigation commissioning observed otherwise healthy callbacks at
# 111.761466 ms and 111.665662 ms.  Do not add a tighter workstation-only
# corrected-age gate: use the existing 200 ms motion state/liveness boundary.
# Do not add a tighter workstation-only callback-period gate either. Receipt
# loss, regressions, clock steps, and source/receipt discontinuities remain hard
# faults in the motion profile.
WORKSTATION_MOTION_MAX_CORRECTED_AGE_NS = 200_000_000
WORKSTATION_MOTION_DELTA_ERROR_LIMIT_NS = 200_000_000
# During the explicitly motion-disabled workstation qualification, a single
# large PointCloud2 callback can be scheduled just after a newer callback from
# each lightweight core stream.  Keep this non-core accommodation narrower
# than either the cloud hard-age or liveness ceiling: the callback is discarded
# only when both observed periods remain bounded to 500 ms and their mismatch
# remains bounded to 200 ms.  Core streams and every motion profile leave the
# corresponding config contract empty.
WORKSTATION_NOMOTION_QUALIFYING_NONCORE_DELTA_ERROR_CEILING_NS = 200_000_000
WORKSTATION_NOMOTION_QUALIFYING_NONCORE_PERIOD_CEILING_NS = 500_000_000


class DisciplineState(str, Enum):
    QUALIFYING = "qualifying"
    LOCKED = "locked"
    FAULTED = "faulted"


@dataclass(frozen=True)
class StreamPolicy:
    """Limits for one required navigation stream.

    ``max_relative_lag_ns`` is deliberately one-sided.  A cloud timestamp may
    describe scan acquisition tens of milliseconds before its DDS receipt; it
    may therefore lag the high-rate state clock without being a second clock.
    A stream is never allowed to lead the reference lower envelope by more
    than the global ``max_relative_lead_ns``.
    """

    name: str
    max_corrected_age_ns: int
    hard_age_ceiling_ns: int
    stale_receipt_timeout_ns: int
    max_relative_lag_ns: int
    min_samples: int
    max_duplicate_fraction: float = 0.10
    # ``None`` inherits the session-wide limit.  A per-stream override exists
    # for transports whose callback delivery is legitimately bursty (notably
    # the ~15 Hz MID-360 PointCloud2 stream).  Freshness, liveness, relative
    # lag, regression, and fixed-offset checks remain independent hard gates.
    max_source_receipt_delta_error_ns: int | None = None


@dataclass(frozen=True)
class DisciplineConfig:
    reference_stream: str
    expected_clock_domain: str
    streams: tuple[StreamPolicy, ...]
    minimum_qualification_span_ns: int = 30 * NANOSECONDS_PER_SECOND
    offset_guard_ns: int = 5_000_000
    # The affine anchor has its own guard so no-motion endurance hardening does
    # not change the legacy fixed-offset estimator or approval semantics.
    affine_anchor_past_guard_ns: int = 5_000_000
    lower_quantile: float = 0.01
    max_corrected_future_ns: int = 2_000_000
    # An affine-only past-side floor.  Unlike max_corrected_future_ns this does
    # not permit any additional future skew: it stops a frozen model while its
    # output is still this far in the past.  ``None`` keeps the strict motion
    # profile and legacy fixed-offset discipline unchanged.
    minimum_locked_corrected_age_ns: int | None = None
    max_relative_lead_ns: int = 5_000_000
    max_clock_pair_discontinuity_ns: int = 20_000_000
    max_clock_read_span_ns: int = 1_000_000
    max_source_receipt_delta_error_ns: int = 50_000_000
    max_absolute_drift_ppm: float = 50.0
    max_pairwise_drift_ppm: float = 25.0
    # Every affine-capable live process must agree with the independently
    # retained approval rate before READY.  This remains 5 ppm even for the
    # strict motion profile, whose post-lock breaker is intentionally wider.
    max_approved_affine_drift_deviation_ppm: float = 5.0
    # The workstation no-motion profile qualifies two fixed, non-overlapping
    # windows before fitting the complete interval.  Other profiles keep the
    # original one-window contract by leaving this unset.
    affine_qualification_window_ns: int | None = None
    max_affine_window_common_drift_deviation_ppm: float = 5.0
    # Short startup subwindows are used to compare their common median drift.
    # Per-stream pairwise agreement may be reserved for the complete fitted
    # interval, which is less sensitive to wireless scheduling jitter.
    affine_enforce_subwindow_pairwise_drift: bool = True
    affine_enforce_subwindow_common_drift: bool = True
    # Keep model-vs-current common-rate drift independent from qualification's
    # cross-stream agreement gate.  A no-motion profile may tighten the frozen
    # model breaker without making a noisy 30 s qualification fail spuriously.
    max_locked_affine_drift_deviation_ppm: float = 25.0
    max_locked_offset_deviation_ns: int = 20_000_000
    drift_bin_ns: int = NANOSECONDS_PER_SECOND
    statistics_evaluation_period_ns: int = NANOSECONDS_PER_SECOND
    # Some wrappers keep a frozen, independently approved affine model for the
    # lifetime of a short motion session.  They may disable the expensive
    # rolling-window re-fit after lock and rely on per-message corrected-age,
    # monotonicity, receipt-liveness and the downstream chassis watchdog.  The
    # qualification/lock checks are unchanged, and every other profile keeps
    # the rolling statistical breaker enabled by default.
    locked_statistics_enabled: bool = True
    retained_samples_per_stream: int = 20_000
    # Only the affine discipline consults this explicit no-motion degradation
    # contract.  A listed non-core stream may discard a post-lock callback
    # scheduling discontinuity without turning that callback into timing
    # evidence.  Core streams remain strict.
    affine_locked_noncore_delta_drop_streams: tuple[str, ...] = ()
    # The no-motion affine qualifier may independently discard one bounded
    # cloud scheduling discontinuity.  The advancing observation becomes only
    # the structural continuity baseline: it is not retained, does not refresh
    # liveness, and cannot contribute to the affine fit.  The fixed ceilings
    # above remain stricter than the normal hard-age/liveness limits.
    affine_qualifying_noncore_delta_drop_streams: tuple[str, ...] = ()
    # A motion-capable workstation profile may restart its still-inactive
    # affine qualifier after a core callback delivery pause.  No corrected
    # output exists before lock, so the delayed sample cannot reach Nav2 or
    # the chassis; the complete qualification window simply starts again.
    affine_restart_qualification_on_core_delta_discontinuity: bool = False
    # A separate, explicit no-motion contract may pause all corrected outputs
    # after one bounded post-lock core callback scheduling discontinuity.  The
    # triggering sample is discarded and the frozen model is not changed;
    # every stream must then supply fresh samples and pass the unchanged locked
    # statistics before output resumes.  Motion profiles leave this empty.
    affine_locked_core_delta_recovery_streams: tuple[str, ...] = ()

    def validate(self) -> None:
        if not self.reference_stream or not self.expected_clock_domain:
            raise ValueError("reference stream and expected clock domain are required")
        if not self.streams:
            raise ValueError("at least one required stream is needed")
        if not isinstance(
            self.affine_restart_qualification_on_core_delta_discontinuity,
            bool,
        ):
            raise ValueError(
                "affine qualification restart flag must be boolean"
            )
        names = [policy.name for policy in self.streams]
        if len(set(names)) != len(names) or self.reference_stream not in names:
            raise ValueError("stream names must be unique and include the reference")
        if self.minimum_qualification_span_ns < 10 * NANOSECONDS_PER_SECOND:
            raise ValueError("qualification span must be at least 10 seconds")
        if not 1_000_000 <= self.offset_guard_ns <= 20_000_000:
            raise ValueError("offset guard must be 1..20 ms")
        if not 1_000_000 <= self.affine_anchor_past_guard_ns <= 20_000_000:
            raise ValueError("affine anchor past guard must be 1..20 ms")
        if not 0.0 < self.lower_quantile <= 0.05:
            raise ValueError("lower quantile must be in (0, 0.05]")
        if not 0 <= self.max_corrected_future_ns <= 5_000_000:
            raise ValueError("navigation correction may allow at most 5 ms future skew")
        if self.offset_guard_ns < self.max_corrected_future_ns:
            raise ValueError("offset guard must cover the allowed future skew")
        if self.affine_anchor_past_guard_ns < self.max_corrected_future_ns:
            raise ValueError(
                "affine anchor past guard must cover the allowed future skew"
            )
        if self.minimum_locked_corrected_age_ns is not None:
            if (
                not isinstance(self.minimum_locked_corrected_age_ns, int)
                or isinstance(self.minimum_locked_corrected_age_ns, bool)
                or not 1_000_000
                <= self.minimum_locked_corrected_age_ns
                <= self.affine_anchor_past_guard_ns
            ):
                raise ValueError(
                    "locked corrected-age floor must be 1 ms..affine anchor guard"
                )
        if not 0 <= self.max_relative_lead_ns <= 10_000_000:
            raise ValueError("cross-stream lead allowance may be at most 10 ms")
        if not 1_000_000 <= self.max_clock_pair_discontinuity_ns <= 50_000_000:
            raise ValueError("clock discontinuity threshold must be 1..50 ms")
        if not 0 < self.max_clock_read_span_ns <= 5_000_000:
            raise ValueError("clock read span ceiling must be at most 5 ms")
        if not 1_000_000 <= self.max_source_receipt_delta_error_ns <= 250_000_000:
            raise ValueError("source/receipt delta error threshold must be 1..250 ms")
        if not 0.1 <= self.max_absolute_drift_ppm <= 100.0:
            raise ValueError("absolute drift threshold must be 0.1..100 ppm")
        if not 0.1 <= self.max_pairwise_drift_ppm <= 50.0:
            raise ValueError("pairwise drift threshold must be 0.1..50 ppm")
        if not 0.1 <= self.max_approved_affine_drift_deviation_ppm <= 5.0:
            raise ValueError(
                "approved affine drift-deviation threshold must be 0.1..5 ppm"
            )
        if self.affine_qualification_window_ns is not None:
            if (
                not isinstance(self.affine_qualification_window_ns, int)
                or isinstance(self.affine_qualification_window_ns, bool)
                or self.affine_qualification_window_ns < 10 * NANOSECONDS_PER_SECOND
                or self.affine_qualification_window_ns
                % self.drift_bin_ns
                != 0
            ):
                raise ValueError(
                    "affine qualification window must be an integer multiple "
                    "of the drift bin and at least 10 seconds"
                )
        if not 0.1 <= (
            self.max_affine_window_common_drift_deviation_ppm
        ) <= 20.0:
            raise ValueError(
                "affine window common-drift threshold must be 0.1..20 ppm"
            )
        if not isinstance(
            self.affine_enforce_subwindow_pairwise_drift, bool
        ):
            raise ValueError(
                "affine subwindow pairwise-drift flag must be boolean"
            )
        if not isinstance(
            self.affine_enforce_subwindow_common_drift, bool
        ):
            raise ValueError(
                "affine subwindow common-drift flag must be boolean"
            )
        if not 0.1 <= self.max_locked_affine_drift_deviation_ppm <= 50.0:
            raise ValueError(
                "locked affine drift-deviation threshold must be 0.1..50 ppm"
            )
        if self.max_pairwise_drift_ppm > self.max_absolute_drift_ppm:
            raise ValueError("pairwise drift threshold cannot exceed absolute drift")
        if not 1_000_000 <= self.max_locked_offset_deviation_ns <= 50_000_000:
            raise ValueError("locked offset deviation must be 1..50 ms")
        if not 100_000_000 <= self.drift_bin_ns <= 5 * NANOSECONDS_PER_SECOND:
            raise ValueError("drift bin must be 0.1..5 seconds")
        if not 10_000_000 <= self.statistics_evaluation_period_ns <= (
            5 * NANOSECONDS_PER_SECOND
        ):
            raise ValueError("statistics evaluation period must be 0.01..5 seconds")
        if not isinstance(self.locked_statistics_enabled, bool):
            raise ValueError("locked statistics enabled must be a boolean")
        if not 100 <= self.retained_samples_per_stream <= 100_000:
            raise ValueError("retained sample count must be 100..100000")
        drop_streams = self.affine_locked_noncore_delta_drop_streams
        if (
            not isinstance(drop_streams, tuple)
            or any(not isinstance(name, str) or not name for name in drop_streams)
            or len(set(drop_streams)) != len(drop_streams)
        ):
            raise ValueError(
                "affine locked non-core delta-drop streams must be unique names"
            )
        unknown_drop_streams = sorted(set(drop_streams) - set(names))
        if unknown_drop_streams:
            raise ValueError(
                "affine locked non-core delta-drop streams are unknown: "
                + ",".join(unknown_drop_streams)
            )
        unsupported_drop_streams = sorted(
            set(drop_streams) - {"mid360_cloud"}
        )
        if unsupported_drop_streams:
            raise ValueError(
                "only mid360_cloud may use the affine locked non-core "
                "delta-drop contract"
            )
        if self.reference_stream in drop_streams:
            raise ValueError("the affine reference stream cannot be delta-drop enabled")
        qualifying_drop_streams = (
            self.affine_qualifying_noncore_delta_drop_streams
        )
        if (
            not isinstance(qualifying_drop_streams, tuple)
            or any(
                not isinstance(name, str) or not name
                for name in qualifying_drop_streams
            )
            or len(set(qualifying_drop_streams))
            != len(qualifying_drop_streams)
        ):
            raise ValueError(
                "affine qualifying non-core delta-drop streams must be "
                "unique names"
            )
        unknown_qualifying_drop_streams = sorted(
            set(qualifying_drop_streams) - set(names)
        )
        if unknown_qualifying_drop_streams:
            raise ValueError(
                "affine qualifying non-core delta-drop streams are unknown: "
                + ",".join(unknown_qualifying_drop_streams)
            )
        unsupported_qualifying_drop_streams = sorted(
            set(qualifying_drop_streams) - {"mid360_cloud"}
        )
        if unsupported_qualifying_drop_streams:
            raise ValueError(
                "only mid360_cloud may use the affine qualifying non-core "
                "delta-drop contract"
            )
        if self.reference_stream in qualifying_drop_streams:
            raise ValueError(
                "the affine reference stream cannot be qualifying "
                "delta-drop enabled"
            )
        recovery_streams = self.affine_locked_core_delta_recovery_streams
        if (
            not isinstance(recovery_streams, tuple)
            or any(
                not isinstance(name, str) or not name
                for name in recovery_streams
            )
            or len(set(recovery_streams)) != len(recovery_streams)
        ):
            raise ValueError(
                "affine locked core delta-recovery streams must be unique names"
            )
        unknown_recovery_streams = sorted(
            set(recovery_streams) - set(names)
        )
        if unknown_recovery_streams:
            raise ValueError(
                "affine locked core delta-recovery streams are unknown: "
                + ",".join(unknown_recovery_streams)
            )
        overlapping_delta_streams = sorted(
            set(recovery_streams).intersection(drop_streams)
        )
        if overlapping_delta_streams:
            raise ValueError(
                "affine delta-drop and delta-recovery streams must be disjoint: "
                + ",".join(overlapping_delta_streams)
            )
        for policy in self.streams:
            if not policy.name:
                raise ValueError("stream name cannot be empty")
            if not 0 < policy.max_corrected_age_ns <= policy.hard_age_ceiling_ns:
                raise ValueError(
                    f"{policy.name}: corrected-age threshold exceeds its hard ceiling"
                )
            if not 0 < policy.stale_receipt_timeout_ns <= policy.hard_age_ceiling_ns:
                raise ValueError(
                    f"{policy.name}: receipt timeout exceeds its hard ceiling"
                )
            if not 0 <= policy.max_relative_lag_ns <= policy.hard_age_ceiling_ns:
                raise ValueError(
                    f"{policy.name}: relative lag exceeds its hard age ceiling"
                )
            if not 10 <= policy.min_samples <= self.retained_samples_per_stream:
                raise ValueError(f"{policy.name}: invalid minimum sample count")
            if not 0.0 <= policy.max_duplicate_fraction <= 0.10:
                raise ValueError(
                    f"{policy.name}: duplicate allowance may be at most 10 percent"
                )
            delta_limit = policy.max_source_receipt_delta_error_ns
            if delta_limit is not None:
                if (
                    not isinstance(delta_limit, int)
                    or isinstance(delta_limit, bool)
                    or not 1_000_000 <= delta_limit <= 250_000_000
                ):
                    raise ValueError(
                        f"{policy.name}: source/receipt delta error threshold "
                        "must be 1..250 ms"
                    )
                if delta_limit > policy.hard_age_ceiling_ns:
                    raise ValueError(
                        f"{policy.name}: source/receipt delta error threshold "
                        "exceeds its hard age ceiling"
                    )


def go2_navigation_config(
    clock_domain: str = "unitree-main-computer@192.168.123.161",
) -> DisciplineConfig:
    """Return checked-in limits that are stricter than the live freshness guards.

    The hard ceilings mirror the existing adapter: 200 ms for chassis/IMU and
    500 ms for PointCloud2.  Qualification uses half of those budgets so an
    accepted stream retains margin for downstream TF/costmap processing.
    """

    return DisciplineConfig(
        reference_stream="sport_primary",
        expected_clock_domain=clock_domain,
        streams=(
            StreamPolicy(
                "sport_primary",
                max_corrected_age_ns=100_000_000,
                hard_age_ceiling_ns=200_000_000,
                stale_receipt_timeout_ns=200_000_000,
                max_relative_lag_ns=10_000_000,
                min_samples=300,
            ),
            StreamPolicy(
                "mid360_imu",
                max_corrected_age_ns=100_000_000,
                hard_age_ceiling_ns=200_000_000,
                stale_receipt_timeout_ns=200_000_000,
                max_relative_lag_ns=20_000_000,
                min_samples=300,
            ),
            StreamPolicy(
                "mid360_cloud",
                max_corrected_age_ns=250_000_000,
                hard_age_ceiling_ns=500_000_000,
                stale_receipt_timeout_ns=500_000_000,
                max_relative_lag_ns=150_000_000,
                min_samples=200,
            ),
        ),
    )


def go2_workstation_nomotion_config(
    clock_domain: str = "unitree-main-computer@192.168.123.161",
) -> DisciplineConfig:
    """Return the fixed-offset workstation no-motion stream contract.

    This profile adds the MID-360 odometry stream as a fourth clock-domain
    witness and corrected *private* output.  It is intentionally not a
    replacement for chassis odometry: the Go2 adapter must still independently
    accept ``SportModeState`` health before it may publish canonical ``/odom``.
    """

    base = go2_navigation_config(clock_domain)
    # PointCloud2 is acquired at roughly 15 Hz and can be delivered in a short
    # burst when the single-threaded no-motion executor is busy.  The same
    # executor also relays the high-rate MID-360 IMU and lidar-odometry witness.
    # A callback which is delayed and then recovers can therefore disagree with
    # the preceding callback's source-vs-receipt period by more than 50 ms even
    # while the *current* corrected IMU age remains below its independent
    # workstation motion freshness limit (this was observed after a
    # 271 s full-stack no-motion run).
    # Give all four private no-motion relay streams a bounded per-sample
    # scheduling allowance.  The same physical full-stack run proved that
    # relaying a large cloud can also delay the high-rate SportModeState
    # callback by more than 50 ms.  During a later physical full-stack startup,
    # SportModeState reached
    # 106.4 ms corrected age while its receipt remained live and all clock-rate
    # checks stayed healthy.  Give the private no-motion state witness the same
    # bounded 150 ms scheduling budget while retaining 50 ms below its 200 ms
    # hard/liveness ceiling.  The private IMU witness uses its existing 200 ms
    # ceiling; the motion profile remains fail-closed and motion is locked.
    # A later physical mapping/UI run also showed one 361 ms workstation
    # scheduling pause.  All three high-rate core streams stopped being polled
    # together while their measured DDS receipt gaps remained below 83 ms.  A
    # 200 ms receipt watchdog therefore misclassified the host pause as three
    # independent source outages.  Permit a bounded 500 ms receipt-only recovery
    # window in this motion-disabled profile, matching the existing cloud
    # liveness window.  This does not widen any stream's max_corrected_age_ns:
    # after the executor resumes, stale queued samples still fail at 150/200 ms.
    # Regression, relative lag, drift, and immutable-offset checks also remain
    # hard gates.  In particular, the base go2_navigation_config() contract is
    # not modified here; the dedicated first-motion profile applies its own
    # explicit motion-profile override below.
    workstation_streams = tuple(
        replace(
            policy,
            # The no-motion relay and mapping stack share one Python executor.
            # On the physical NX/Go2 data plane otherwise monotonic state and
            # IMU samples were observed at 100--200 ms corrected age while all
            # ten no-motion components were starting.  Keep the motion-capable
            # motion profile's freshness contract unchanged.  The state witness gets the same
            # 150 ms scheduling budget while the IMU witness may use its
            # already-audited 200 ms hard/liveness ceiling.
            max_corrected_age_ns=(
                150_000_000
                if policy.name == "sport_primary"
                else (
                    policy.hard_age_ceiling_ns
                    if policy.name == "mid360_imu"
                    else policy.max_corrected_age_ns
                )
            ),
            # This no-motion-only ceiling is not a publish freshness budget.
            # A full Robonix startup later blocked an RMW publish for 375 ms
            # and delivered all four streams 383--609 ms old; a second run
            # reached 760--796 ms receipt gaps.  Samples beyond
            # the unchanged operational max_corrected_age_ns are discarded;
            # up to a fixed 2 s ceiling may only enter the all-stream recovery
            # pause.  This is not tuned to an observed individual sample.
            # Motion profiles retain their original 200/500 ms hard ceilings.
            hard_age_ceiling_ns=(
                NOMOTION_TRANSIENT_STALE_RECOVERY_CEILING_NS
            ),
            stale_receipt_timeout_ns=500_000_000,
            max_source_receipt_delta_error_ns=150_000_000,
        )
        if policy.name in {"sport_primary", "mid360_imu", "mid360_cloud"}
        else policy
        for policy in base.streams
    )
    return DisciplineConfig(
        reference_stream=base.reference_stream,
        expected_clock_domain=base.expected_clock_domain,
        streams=workstation_streams
        + (
            StreamPolicy(
                "mid360_odom",
                # This fourth stream exists only in the explicit workstation
                # no-motion profile and feeds a private mapping topic.  A
                # PointCloud2 copy/publish can hold the single-threaded relay
                # long enough for one otherwise valid odometry callback to
                # arrive more than 100 ms old during full-stack startup.  Keep
                # its 200 ms corrected-age breaker, but use the no-motion-only
                # 500 ms receipt recovery window described above.  The base
                # navigation (and therefore every motion-capable) profile,
                # SportModeState, and IMU budgets are unchanged.
                max_corrected_age_ns=200_000_000,
                hard_age_ceiling_ns=(
                    NOMOTION_TRANSIENT_STALE_RECOVERY_CEILING_NS
                ),
                stale_receipt_timeout_ns=500_000_000,
                max_relative_lag_ns=20_000_000,
                min_samples=200,
                max_source_receipt_delta_error_ns=150_000_000,
            ),
        ),
        minimum_qualification_span_ns=base.minimum_qualification_span_ns,
        offset_guard_ns=base.offset_guard_ns,
        # This profile is the only one permitted to bias the frozen affine
        # anchor farther into the past.  The fixed estimator remains at its
        # original 5 ms guard.  The unchanged 2 ms hard-future gate remains the
        # final breaker; the 5 ms age floor stops gradual affine extrapolation
        # erosion before that hard boundary is reached.
        affine_anchor_past_guard_ns=20_000_000,
        lower_quantile=base.lower_quantile,
        max_corrected_future_ns=base.max_corrected_future_ns,
        minimum_locked_corrected_age_ns=5_000_000,
        max_relative_lead_ns=base.max_relative_lead_ns,
        max_clock_pair_discontinuity_ns=base.max_clock_pair_discontinuity_ns,
        max_clock_read_span_ns=base.max_clock_read_span_ns,
        max_source_receipt_delta_error_ns=base.max_source_receipt_delta_error_ns,
        max_absolute_drift_ppm=base.max_absolute_drift_ppm,
        max_pairwise_drift_ppm=base.max_pairwise_drift_ppm,
        max_approved_affine_drift_deviation_ppm=5.0,
        affine_qualification_window_ns=30 * NANOSECONDS_PER_SECOND,
        # The no-motion workstation UI may start immediately after a robot
        # recharge. Repeated live qualifications on this workstation produced
        # otherwise healthy 30 s window differences up to 13.032 ppm while
        # writer identity, pairwise drift, receipt gaps and the complete-window
        # drift all remained within their separate bounds. Accept a bounded
        # 15 ppm warm-up envelope here only; the independent model comparison
        # and the 25 ppm post-lock breaker remain unchanged.
        max_affine_window_common_drift_deviation_ppm=15.0,
        # Once the no-motion model is locked, use the existing bounded 25 ppm
        # breaker from the base discipline. A short workstation scheduling
        # wobble must not tear down a mapping/UI session merely because it
        # crossed the much narrower startup-approval comparison.
        max_locked_affine_drift_deviation_ppm=(
            base.max_locked_affine_drift_deviation_ppm
        ),
        max_locked_offset_deviation_ns=base.max_locked_offset_deviation_ns,
        drift_bin_ns=base.drift_bin_ns,
        statistics_evaluation_period_ns=base.statistics_evaluation_period_ns,
        # Keep the complete fixed 60 s qualification interval even at the
        # documented 500 Hz upper-rate planning envelope.  The terminal
        # evaluator also verifies both window edges and every receipt gap, so
        # an unexpectedly exhausted buffer fails closed rather than silently
        # fitting a truncated interval.
        retained_samples_per_stream=40_000,
        # PointCloud2 is a mapping/UI consumer, not an affine clock-rate core
        # witness.  Once this no-motion profile is locked, one delayed cloud
        # callback may be discarded while its last-good liveness clock remains
        # untouched.  No motion-capable profile enables this contract.
        affine_locked_noncore_delta_drop_streams=("mid360_cloud",),
        affine_qualifying_noncore_delta_drop_streams=("mid360_cloud",),
        # A single-threaded no-motion relay can receive one delayed core
        # callback after a cloud publish or host scheduling pause while every
        # source clock remains monotonic.  Do not publish that callback and do
        # not refit the model: pause the complete private relay, require fresh
        # evidence from every stream, and rerun the unchanged locked health
        # checks.  The motion configuration below leaves this contract empty.
        affine_locked_core_delta_recovery_streams=(
            "sport_primary",
            "mid360_imu",
            "mid360_odom",
        ),
    )


def go2_workstation_motion_config(
    clock_domain: str = "unitree-main-computer@192.168.123.161",
) -> DisciplineConfig:
    """Return the strict state/IMU/odometry contract for physical motion.

    The workstation motion profile uses the shared 200 ms corrected-age,
    receipt-liveness and callback-period boundary.  Crossing that boundary
    pauses all corrected output so the independent chassis watchdog stops;
    the timestamp process may then resume the same frozen model and writer
    session after two fresh samples from every stream instead of forcing a
    full Robonix restart.  MID-360 odometry is the third private clock-domain
    witness.
    PointCloud2 is intentionally absent: first motion does not consume it, and
    copying a large cloud in the single-threaded process could delay the state
    watchdog.
    """

    base = go2_navigation_config(clock_domain)
    strict_streams = tuple(
        replace(
            policy,
            max_corrected_age_ns=WORKSTATION_MOTION_MAX_CORRECTED_AGE_NS,
            max_source_receipt_delta_error_ns=(
                WORKSTATION_MOTION_DELTA_ERROR_LIMIT_NS
            ),
        )
        for policy in base.streams
        if policy.name != "mid360_cloud"
    )
    return DisciplineConfig(
        reference_stream=base.reference_stream,
        expected_clock_domain=base.expected_clock_domain,
        streams=strict_streams
        + (
            StreamPolicy(
                "mid360_odom",
                max_corrected_age_ns=(
                    WORKSTATION_MOTION_MAX_CORRECTED_AGE_NS
                ),
                hard_age_ceiling_ns=200_000_000,
                stale_receipt_timeout_ns=200_000_000,
                max_relative_lag_ns=20_000_000,
                min_samples=200,
                max_source_receipt_delta_error_ns=(
                    WORKSTATION_MOTION_DELTA_ERROR_LIMIT_NS
                ),
            ),
        ),
        minimum_qualification_span_ns=10 * NANOSECONDS_PER_SECOND,
        offset_guard_ns=base.offset_guard_ns,
        # The persistent motion stack reproduced the same bounded affine
        # extrapolation failure already seen in the no-motion endurance run:
        # a 3.31 ppm frozen-rate overestimate consumed 6.80 ms in 2,055 s and
        # crossed the unchanged 2 ms hard-future breaker.  Biasing the common
        # motion anchor 20 ms into the past preserves the strict future,
        # stale, liveness and source/receipt breakers while giving a bounded
        # physical-navigation epoch the already-supported past-side margin.
        affine_anchor_past_guard_ns=20_000_000,
        lower_quantile=base.lower_quantile,
        max_corrected_future_ns=base.max_corrected_future_ns,
        minimum_locked_corrected_age_ns=(
            base.minimum_locked_corrected_age_ns
        ),
        max_relative_lead_ns=base.max_relative_lead_ns,
        max_clock_pair_discontinuity_ns=base.max_clock_pair_discontinuity_ns,
        max_clock_read_span_ns=base.max_clock_read_span_ns,
        max_source_receipt_delta_error_ns=base.max_source_receipt_delta_error_ns,
        max_absolute_drift_ppm=base.max_absolute_drift_ppm,
        max_pairwise_drift_ppm=base.max_pairwise_drift_ppm,
        max_approved_affine_drift_deviation_ppm=5.0,
        # Standard Robonix/Nav2 startup has no one-minute clock gate.  Keep two
        # independent live windows, but make them 10 s each: long enough for
        # the one-second lower-envelope fit while avoiding a workstation-only
        # delay on every robot restart.
        affine_qualification_window_ns=10 * NANOSECONDS_PER_SECOND,
        max_affine_window_common_drift_deviation_ppm=15.0,
        affine_enforce_subwindow_pairwise_drift=False,
        affine_enforce_subwindow_common_drift=False,
        max_locked_affine_drift_deviation_ppm=(
            base.max_locked_affine_drift_deviation_ppm
        ),
        max_locked_offset_deviation_ns=base.max_locked_offset_deviation_ns,
        drift_bin_ns=base.drift_bin_ns,
        statistics_evaluation_period_ns=base.statistics_evaluation_period_ns,
        # The affine model and writer identity were independently qualified
        # before motion.  Re-fitting a 60 s / 40k-sample rolling window every
        # second is a workstation-only startup/runtime gate, not a Robonix or
        # Nav2 requirement, and can itself starve the single-threaded relay.
        # Keep the per-message time, monotonicity and 200 ms liveness breakers;
        # the chassis watchdog remains fail-closed while output is paused.
        locked_statistics_enabled=False,
        retained_samples_per_stream=40_000,
        # A delayed transition callback is never published.  It may only
        # enter the all-stream locked-recovery pause, which requires fresh
        # evidence from every stream before output resumes.  The chassis state
        # watchdog remains the actual 200 ms motion stop boundary.
        affine_locked_core_delta_recovery_streams=(
            "sport_primary",
            "mid360_imu",
            "mid360_odom",
        ),
        affine_restart_qualification_on_core_delta_discontinuity=True,
    )


@dataclass(frozen=True)
class CorrectionResult:
    accepted: bool
    corrected_stamp_ns: int | None
    navigation_eligible: bool
    reason: str
    state: DisciplineState
    diagnostics: tuple[tuple[str, int], ...] = ()


@dataclass(frozen=True)
class UiReceiptStamp:
    """Receipt-time label for UI preview only; never eligible for navigation."""

    stamp_ns: int
    provenance: str = "local_receipt_time_ui_only"
    navigation_eligible: bool = False


def ui_receipt_retimestamp(receipt_realtime_ns: int) -> UiReceiptStamp:
    if not _positive_int(receipt_realtime_ns):
        raise ValueError("UI receipt timestamp must be a positive integer")
    return UiReceiptStamp(receipt_realtime_ns)


@dataclass(frozen=True)
class StreamMetrics:
    samples: int
    span_ns: int
    lower_age_ns: int
    median_age_ns: int
    p95_age_ns: int
    robust_drift_ppm: float | None
    duplicate_fraction: float


@dataclass(frozen=True)
class AffineStreamMetrics:
    """Lower-envelope timing evidence used only for affine qualification."""

    envelope_points: int
    drift_ppm: float | None


@dataclass(frozen=True)
class AffineDriftIntervalEvidence:
    """Lower-envelope rate evidence for one fixed monotonic interval."""

    start_monotonic_ns: int
    end_monotonic_ns: int
    end_inclusive: bool
    core_stream_drifts_ppm: tuple[tuple[str, float | None], ...]
    core_stream_envelope_points: tuple[tuple[str, int], ...]
    core_stream_first_receipt_offset_ns: tuple[tuple[str, int | None], ...]
    core_stream_last_receipt_offset_ns: tuple[tuple[str, int | None], ...]
    core_stream_max_receipt_gap_ns: tuple[tuple[str, int | None], ...]
    common_drift_ppm: float | None


@dataclass(frozen=True)
class AffineQualificationStreamSnapshot:
    """Immutable per-stream input to one terminal affine evaluation."""

    name: str
    samples: tuple[tuple[int, int, int], ...]
    last_advancing_receipt_monotonic_ns: int | None
    advancing: int
    duplicates: int


@dataclass(frozen=True)
class AffineQualificationSnapshot:
    """One immutable fixed-window view captured exactly once by the runtime."""

    qualification_start_monotonic_ns: int
    qualification_end_monotonic_ns: int
    evaluation_monotonic_ns: int
    streams: tuple[AffineQualificationStreamSnapshot, ...]

    def samples_by_stream(self) -> dict[str, tuple[tuple[int, int, int], ...]]:
        return {stream.name: stream.samples for stream in self.streams}


@dataclass(frozen=True)
class AffineQualificationEvaluation:
    """Cached result used for one terminal decision and one lock commit."""

    snapshot: AffineQualificationSnapshot
    reasons: tuple[str, ...]
    interval_evidence: tuple[
        AffineDriftIntervalEvidence,
        AffineDriftIntervalEvidence,
        AffineDriftIntervalEvidence,
    ]
    model: "AffineClockModel | None"
    # Corrected ages for the exact immutable snapshot, computed in the same
    # terminal pass as the candidate model.  The commit path may copy these
    # tuples into bounded deques, but it must never rescan mutable live samples
    # or repeat affine correction after the post-evaluation freshness check.
    corrected_age_windows: tuple[tuple[str, tuple[int, ...]], ...]


@dataclass(frozen=True)
class AffineClockModel:
    """One frozen mapping from the Go2 source clock to local realtime.

    ``drift_ppm`` is the slope of ``local - source`` against local monotonic
    time.  A positive value means the source clock advances more slowly.  The
    scale is therefore ``1 / (1 - drift_ppm * 1e-6)``.  Correction arithmetic
    always operates on a delta from the anchor, avoiding float conversion of a
    Unix-epoch nanosecond value.
    """

    anchor_source_ns: int
    anchor_local_ns: int
    drift_ppm: float
    source_to_local_scale: float
    core_stream_drifts_ppm: tuple[tuple[str, float], ...]
    stream_baseline_corrected_age_ns: tuple[tuple[str, int], ...]

    def corrected_stamp_ns(self, source_stamp_ns: int) -> int:
        if not _positive_int(source_stamp_ns):
            raise ValueError("source stamp must be a positive integer")
        corrected = self.anchor_local_ns + round(
            (source_stamp_ns - self.anchor_source_ns) * self.source_to_local_scale
        )
        if corrected <= 0 or corrected >= 2**63:
            raise ValueError("affine-corrected stamp is outside positive int64")
        return corrected

    def offset_at_source_ns(self, source_stamp_ns: int) -> int:
        return self.corrected_stamp_ns(source_stamp_ns) - source_stamp_ns


@dataclass
class _StreamState:
    retained_limit: int
    samples: deque[tuple[int, int, int]] = field(init=False)
    last_source_ns: int | None = None
    last_advancing_receipt_monotonic_ns: int | None = None
    advancing: int = 0
    duplicates: int = 0

    def __post_init__(self) -> None:
        self.samples = deque(maxlen=self.retained_limit)


def _positive_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _quantile(values: Iterable[int], fraction: float) -> int:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("cannot calculate a quantile of an empty sequence")
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return int(ordered[index])


def _robust_drift_ppm(
    samples: Iterable[tuple[int, int, int]], bin_ns: int
) -> float | None:
    """Theil-Sen slope over time-bin age medians.

    Binning prevents high-rate streams from dominating and suppresses DDS
    scheduling jitter.  Age slope is the local/source clock-rate difference.
    """

    grouped: dict[int, list[int]] = {}
    first_monotonic: int | None = None
    for _, receipt_monotonic_ns, age_ns in samples:
        if first_monotonic is None:
            first_monotonic = receipt_monotonic_ns
        key = (receipt_monotonic_ns - first_monotonic) // bin_ns
        grouped.setdefault(key, []).append(age_ns)
    points = [
        (key * bin_ns, int(statistics.median(values)))
        for key, values in sorted(grouped.items())
    ]
    if len(points) < 4:
        return None
    slopes: list[float] = []
    for index, (left_time, left_age) in enumerate(points):
        for right_time, right_age in points[index + 1 :]:
            elapsed = right_time - left_time
            if elapsed > 0:
                slopes.append((right_age - left_age) / elapsed * 1_000_000.0)
    return float(statistics.median(slopes)) if slopes else None


def _lower_envelope_points(
    samples: Iterable[tuple[int, int, int]],
    bin_ns: int,
    *,
    anchor_monotonic_ns: int = 0,
) -> tuple[tuple[int, int, int], ...]:
    """Return the minimum-age sample in each monotonic time bin.

    DDS callback delay is non-negative.  Taking the minimum observed age in a
    one-second bin rejects callback scheduling bursts without pretending that
    receipt time is sensor time.  The selected tuple remains
    ``(source_stamp_ns, receipt_monotonic_ns, local_minus_source_age_ns)``.
    """

    if (
        not isinstance(anchor_monotonic_ns, int)
        or isinstance(anchor_monotonic_ns, bool)
        or anchor_monotonic_ns < 0
    ):
        raise ValueError("lower-envelope bin anchor must be a non-negative integer")
    bins: dict[int, tuple[int, int, int]] = {}
    for source_ns, monotonic_ns, age_ns in samples:
        # Never anchor bins to the first retained deque entry.  Once a bounded
        # high-rate deque starts evicting, that entry rolls on every callback
        # and would redefine every bin independently for each stream.  An
        # explicit qualification anchor (or the stable monotonic epoch grid)
        # keeps bin membership fixed for the complete session.
        key = (monotonic_ns - anchor_monotonic_ns) // bin_ns
        candidate = (source_ns, monotonic_ns, age_ns)
        incumbent = bins.get(key)
        if incumbent is None or (age_ns, monotonic_ns, source_ns) < (
            incumbent[2],
            incumbent[1],
            incumbent[0],
        ):
            bins[key] = candidate
    return tuple(bins[key] for key in sorted(bins))


def _theil_sen_age_drift_ppm(
    points: Iterable[tuple[int, int, int]],
) -> float | None:
    """Return a robust lower-envelope age slope in ppm."""

    materialized = tuple(points)
    if len(materialized) < 4:
        return None
    slopes: list[float] = []
    for index, (_, left_monotonic_ns, left_age_ns) in enumerate(materialized):
        for _, right_monotonic_ns, right_age_ns in materialized[index + 1 :]:
            elapsed_ns = right_monotonic_ns - left_monotonic_ns
            if elapsed_ns > 0:
                slopes.append(
                    (right_age_ns - left_age_ns)
                    / elapsed_ns
                    * 1_000_000.0
                )
    return float(statistics.median(slopes)) if slopes else None


def _pairwise_drift_disagreements(
    drifts: Iterable[tuple[str, float]], limit_ppm: float
) -> tuple[str, ...]:
    """Return streams participating in any over-limit pairwise difference."""

    values = tuple(drifts)
    return tuple(
        name
        for name, drift in values
        if any(
            other_name != name and abs(drift - other_drift) > limit_ppm
            for other_name, other_drift in values
        )
    )


def _source_to_local_scale(drift_ppm: float) -> float:
    if not math.isfinite(drift_ppm):
        raise ValueError("affine drift must be finite")
    denominator = 1.0 - drift_ppm * 1e-6
    if denominator <= 0.0:
        raise ValueError("affine drift produces a non-positive clock rate")
    return 1.0 / denominator


class NavigationStampDiscipline:
    """Qualify, lock, apply, and continuously police one fixed clock offset."""

    def __init__(self, config: DisciplineConfig) -> None:
        config.validate()
        self.config = config
        self.state = DisciplineState.QUALIFYING
        self.fault_reason: str | None = None
        self.locked_offset_ns: int | None = None
        self._policies = {policy.name: policy for policy in config.streams}
        self._streams = {
            policy.name: _StreamState(config.retained_samples_per_stream)
            for policy in config.streams
        }
        self._last_clock_base_ns: int | None = None
        self._last_statistics_evaluation_monotonic_ns: int | None = None

    def _fault(
        self,
        reason: str,
        *,
        diagnostics: tuple[tuple[str, int], ...] = (),
    ) -> CorrectionResult:
        self.state = DisciplineState.FAULTED
        self.fault_reason = reason
        return CorrectionResult(
            False,
            None,
            False,
            reason,
            self.state,
            diagnostics,
        )

    def _metrics(self, stream: str) -> StreamMetrics | None:
        state = self._streams[stream]
        if not state.samples:
            return None
        ages = [sample[2] for sample in state.samples]
        span = state.samples[-1][1] - state.samples[0][1]
        total = state.advancing + state.duplicates
        return StreamMetrics(
            samples=len(state.samples),
            span_ns=max(0, span),
            lower_age_ns=_quantile(ages, self.config.lower_quantile),
            median_age_ns=_quantile(ages, 0.50),
            p95_age_ns=_quantile(ages, 0.95),
            robust_drift_ppm=_robust_drift_ppm(
                state.samples, self.config.drift_bin_ns
            ),
            duplicate_fraction=(state.duplicates / total) if total else 0.0,
        )

    def metrics(self) -> dict[str, StreamMetrics | None]:
        return {name: self._metrics(name) for name in self._streams}

    def observe(
        self,
        stream: str,
        source_stamp_ns: int,
        receipt_realtime_ns: int,
        receipt_monotonic_ns: int,
        *,
        clock_domain: str,
        clock_read_span_ns: int = 0,
    ) -> CorrectionResult:
        """Observe one sample and return a corrected stamp only while locked.

        A duplicate source timestamp is ignored and never refreshes stream
        liveness.  Regression, a clock-domain mismatch, a clock step, an
        implausible source/receipt delta, stale data, or a correction-quality
        violation latches the whole session FAULTED.
        """

        if self.state == DisciplineState.FAULTED:
            return CorrectionResult(
                False,
                None,
                False,
                self.fault_reason or "discipline faulted",
                self.state,
            )
        if stream not in self._streams:
            return self._fault(f"unexpected_stream:{stream}")
        if clock_domain != self.config.expected_clock_domain:
            return self._fault(f"clock_domain_mismatch:{stream}")
        if not all(
            _positive_int(value)
            for value in (source_stamp_ns, receipt_realtime_ns, receipt_monotonic_ns)
        ):
            return self._fault(f"invalid_timestamp_encoding:{stream}")
        if (
            not isinstance(clock_read_span_ns, int)
            or isinstance(clock_read_span_ns, bool)
            or not 0 <= clock_read_span_ns <= self.config.max_clock_read_span_ns
        ):
            return self._fault(f"clock_read_span_exceeded:{stream}")

        clock_base_ns = receipt_realtime_ns - receipt_monotonic_ns
        if self._last_clock_base_ns is not None and abs(
            clock_base_ns - self._last_clock_base_ns
        ) > self.config.max_clock_pair_discontinuity_ns:
            return self._fault("local_realtime_discontinuity")
        self._last_clock_base_ns = clock_base_ns

        state = self._streams[stream]
        if state.last_source_ns is not None:
            if source_stamp_ns < state.last_source_ns:
                return self._fault(f"source_timestamp_regression:{stream}")
            if source_stamp_ns == state.last_source_ns:
                state.duplicates += 1
                return CorrectionResult(
                    False,
                    None,
                    False,
                    f"duplicate_source_timestamp:{stream}",
                    self.state,
                )
            assert state.last_advancing_receipt_monotonic_ns is not None
            source_delta_ns = source_stamp_ns - state.last_source_ns
            receipt_delta_ns = (
                receipt_monotonic_ns - state.last_advancing_receipt_monotonic_ns
            )
            if receipt_delta_ns <= 0:
                return self._fault(f"receipt_monotonic_not_advancing:{stream}")
            policy = self._policies[stream]
            delta_limit_ns = (
                policy.max_source_receipt_delta_error_ns
                if policy.max_source_receipt_delta_error_ns is not None
                else self.config.max_source_receipt_delta_error_ns
            )
            delta_error_ns = abs(source_delta_ns - receipt_delta_ns)
            if delta_error_ns > delta_limit_ns:
                return self._fault(
                    f"source_receipt_delta_discontinuity:{stream}",
                    diagnostics=(
                        ("source_delta_ns", source_delta_ns),
                        ("receipt_delta_ns", receipt_delta_ns),
                        ("delta_error_ns", delta_error_ns),
                        ("delta_error_limit_ns", delta_limit_ns),
                    ),
                )

        age_ns = receipt_realtime_ns - source_stamp_ns
        state.samples.append((source_stamp_ns, receipt_monotonic_ns, age_ns))
        state.last_source_ns = source_stamp_ns
        state.last_advancing_receipt_monotonic_ns = receipt_monotonic_ns
        state.advancing += 1

        if self.state != DisciplineState.LOCKED:
            return CorrectionResult(
                False, None, False, "offset_not_locked", self.state
            )
        assert self.locked_offset_ns is not None
        corrected_stamp_ns = source_stamp_ns + self.locked_offset_ns
        corrected_age_ns = receipt_realtime_ns - corrected_stamp_ns
        policy = self._policies[stream]
        if corrected_age_ns < -self.config.max_corrected_future_ns:
            return self._fault(f"corrected_timestamp_in_future:{stream}")
        if corrected_age_ns > policy.max_corrected_age_ns:
            return self._fault(
                f"corrected_timestamp_stale:{stream}:"
                f"corrected_age_ns={corrected_age_ns}:"
                f"limit_ns={policy.max_corrected_age_ns}"
            )
        health_reason = self._liveness_health(receipt_monotonic_ns)
        if health_reason is None:
            health_reason = self._scheduled_statistics_health(receipt_monotonic_ns)
        if health_reason is not None:
            return self._fault(health_reason)
        return CorrectionResult(
            True, corrected_stamp_ns, True, "navigation_safe", self.state
        )

    def qualification_reasons(self, now_monotonic_ns: int) -> list[str]:
        if not _positive_int(now_monotonic_ns):
            return ["invalid_qualification_time"]
        reasons: list[str] = []
        metrics = self.metrics()
        reference = metrics[self.config.reference_stream]
        if reference is None:
            return ["missing_reference_stream"]
        for name, policy in self._policies.items():
            stream_metrics = metrics[name]
            stream_state = self._streams[name]
            if stream_metrics is None:
                reasons.append(f"missing_stream:{name}")
                continue
            if stream_metrics.samples < policy.min_samples:
                reasons.append(f"insufficient_samples:{name}")
            if stream_metrics.span_ns < self.config.minimum_qualification_span_ns:
                reasons.append(f"insufficient_span:{name}")
            last_receipt = stream_state.last_advancing_receipt_monotonic_ns
            receipt_age_ns = (
                None if last_receipt is None else now_monotonic_ns - last_receipt
            )
            if (
                receipt_age_ns is None
                or receipt_age_ns < 0
                or receipt_age_ns > policy.stale_receipt_timeout_ns
            ):
                reasons.append(f"stream_not_live:{name}")
            if stream_metrics.duplicate_fraction > policy.max_duplicate_fraction:
                reasons.append(f"excessive_duplicates:{name}")
            drift = stream_metrics.robust_drift_ppm
            if drift is None:
                reasons.append(f"insufficient_drift_bins:{name}")
            elif abs(drift) > self.config.max_absolute_drift_ppm:
                reasons.append(f"clock_drift_exceeded:{name}")

            relative_lag_ns = stream_metrics.lower_age_ns - reference.lower_age_ns
            if relative_lag_ns < -self.config.max_relative_lead_ns:
                reasons.append(f"stream_leads_reference:{name}")
            if relative_lag_ns > policy.max_relative_lag_ns:
                reasons.append(f"stream_lag_exceeded:{name}")

        reference_drift = reference.robust_drift_ppm
        if reference_drift is not None:
            for name, stream_metrics in metrics.items():
                if stream_metrics is None or stream_metrics.robust_drift_ppm is None:
                    continue
                if abs(stream_metrics.robust_drift_ppm - reference_drift) > (
                    self.config.max_pairwise_drift_ppm
                ):
                    reasons.append(f"pairwise_drift_disagreement:{name}")

        candidate_offset_ns = reference.lower_age_ns - self.config.offset_guard_ns
        for name, policy in self._policies.items():
            stream_metrics = metrics[name]
            if stream_metrics is None:
                continue
            corrected_min_ns = stream_metrics.lower_age_ns - candidate_offset_ns
            corrected_p95_ns = stream_metrics.p95_age_ns - candidate_offset_ns
            if corrected_min_ns < -self.config.max_corrected_future_ns:
                reasons.append(f"candidate_would_be_future:{name}")
            if corrected_p95_ns > policy.max_corrected_age_ns:
                reasons.append(f"candidate_would_be_stale:{name}")
        return sorted(set(reasons))

    def lock(
        self, *, identity_evidence_verified: bool, now_monotonic_ns: int
    ) -> int:
        """Explicitly freeze the one session offset; never auto-lock or adapt."""

        if self.state == DisciplineState.FAULTED:
            raise RuntimeError(self.fault_reason or "discipline faulted")
        if self.state == DisciplineState.LOCKED:
            assert self.locked_offset_ns is not None
            return self.locked_offset_ns
        if identity_evidence_verified is not True:
            raise RuntimeError("clock-domain identity evidence is not verified")
        reasons = self.qualification_reasons(now_monotonic_ns)
        if reasons:
            raise RuntimeError("timestamp qualification failed: " + ",".join(reasons))
        reference = self._metrics(self.config.reference_stream)
        assert reference is not None
        self.locked_offset_ns = reference.lower_age_ns - self.config.offset_guard_ns
        self.state = DisciplineState.LOCKED
        self._last_statistics_evaluation_monotonic_ns = now_monotonic_ns
        return self.locked_offset_ns

    def lock_fixed(
        self,
        approved_offset_ns: int,
        *,
        identity_evidence_verified: bool,
        now_monotonic_ns: int,
    ) -> int:
        """Lock an explicitly approved immutable ``local - source`` offset.

        The supplied offset is never estimated, adapted, or replaced by
        receipt time.  The live qualification still has to pass, and its
        lower-envelope candidate must agree with the approved value within the
        normal locked-offset deviation budget.  Every stream is checked
        against the approved value before the session can become LOCKED.
        """

        if self.state == DisciplineState.FAULTED:
            raise RuntimeError(self.fault_reason or "discipline faulted")
        if self.state == DisciplineState.LOCKED:
            assert self.locked_offset_ns is not None
            if approved_offset_ns != self.locked_offset_ns:
                raise RuntimeError("discipline is already locked to another offset")
            return self.locked_offset_ns
        if (
            not isinstance(approved_offset_ns, int)
            or isinstance(approved_offset_ns, bool)
            or not -(2**63) < approved_offset_ns < 2**63
        ):
            raise RuntimeError("approved offset must be a signed int64 nanosecond value")
        if identity_evidence_verified is not True:
            raise RuntimeError("clock-domain identity evidence is not verified")
        reasons = self.qualification_reasons(now_monotonic_ns)
        if reasons:
            raise RuntimeError("timestamp qualification failed: " + ",".join(reasons))

        metrics = self.metrics()
        reference = metrics[self.config.reference_stream]
        assert reference is not None
        observed_candidate_ns = reference.lower_age_ns - self.config.offset_guard_ns
        if abs(observed_candidate_ns - approved_offset_ns) > (
            self.config.max_locked_offset_deviation_ns
        ):
            raise RuntimeError("approved fixed offset disagrees with live qualification")
        fixed_reasons: list[str] = []
        for name, policy in self._policies.items():
            stream_metrics = metrics[name]
            assert stream_metrics is not None
            corrected_min_ns = stream_metrics.lower_age_ns - approved_offset_ns
            corrected_p95_ns = stream_metrics.p95_age_ns - approved_offset_ns
            if corrected_min_ns < -self.config.max_corrected_future_ns:
                fixed_reasons.append(f"approved_offset_would_be_future:{name}")
            if corrected_p95_ns > policy.max_corrected_age_ns:
                fixed_reasons.append(f"approved_offset_would_be_stale:{name}")
        if fixed_reasons:
            raise RuntimeError(
                "approved fixed offset failed stream budgets: "
                + ",".join(sorted(set(fixed_reasons)))
            )

        self.locked_offset_ns = approved_offset_ns
        self.state = DisciplineState.LOCKED
        self._last_statistics_evaluation_monotonic_ns = now_monotonic_ns
        return approved_offset_ns

    def poll(self, now_monotonic_ns: int) -> CorrectionResult:
        """Run deadline/statistical breakers when no sensor callback arrives.

        A future ROS wrapper must call this from a monotonic wall timer.  If all
        DDS sources stop, no sample callback exists to discover that fact.
        """

        if self.state == DisciplineState.FAULTED:
            return CorrectionResult(
                False,
                None,
                False,
                self.fault_reason or "discipline faulted",
                self.state,
            )
        if self.state != DisciplineState.LOCKED:
            return CorrectionResult(False, None, False, "offset_not_locked", self.state)
        if not _positive_int(now_monotonic_ns):
            return self._fault("invalid_poll_time")
        reason = self._liveness_health(now_monotonic_ns)
        if reason is None:
            reason = self._scheduled_statistics_health(now_monotonic_ns)
        if reason is not None:
            return self._fault(reason)
        return CorrectionResult(False, None, False, "healthy_no_sample", self.state)

    def receipt_liveness_snapshot(
        self, now_monotonic_ns: int
    ) -> dict[str, dict[str, int | bool | None]]:
        """Return exact receipt ages without changing any liveness decision.

        This is diagnostic evidence for a fail-closed fault record.  The
        checked-in per-stream timeout remains the sole liveness limit.
        """

        snapshot: dict[str, dict[str, int | bool | None]] = {}
        for name, policy in self._policies.items():
            last_receipt = self._streams[name].last_advancing_receipt_monotonic_ns
            receipt_age_ns = (
                None if last_receipt is None else now_monotonic_ns - last_receipt
            )
            snapshot[name] = {
                "last_advancing_receipt_monotonic_ns": last_receipt,
                "receipt_age_ns": receipt_age_ns,
                "stale_receipt_timeout_ns": policy.stale_receipt_timeout_ns,
                "live": (
                    receipt_age_ns is not None
                    and 0 <= receipt_age_ns <= policy.stale_receipt_timeout_ns
                ),
            }
        return snapshot

    def _diagnostic_corrected_stamp_ns(self, source_stamp_ns: int) -> int | None:
        """Return the frozen correction for diagnostics, never adapting it."""

        if self.locked_offset_ns is None:
            return None
        return source_stamp_ns + self.locked_offset_ns

    def corrected_age_snapshot(self) -> dict[str, dict[str, int | bool | None]]:
        """Return each stream's latest corrected age without changing state.

        Receipt realtime is reconstructed exactly from the retained
        ``local - source`` age.  This keeps fault evidence read-only and avoids
        adding another mutable timestamp alongside the data already used by
        qualification and continuous health checks.
        """

        snapshot: dict[str, dict[str, int | bool | None]] = {}
        for name, policy in self._policies.items():
            samples = self._streams[name].samples
            source_stamp_ns: int | None = None
            receipt_realtime_ns: int | None = None
            corrected_age_ns: int | None = None
            if samples:
                source_stamp_ns, _, raw_age_ns = samples[-1]
                receipt_realtime_ns = source_stamp_ns + raw_age_ns
                corrected_stamp_ns = self._diagnostic_corrected_stamp_ns(
                    source_stamp_ns
                )
                if corrected_stamp_ns is not None:
                    corrected_age_ns = receipt_realtime_ns - corrected_stamp_ns
            snapshot[name] = {
                "latest_source_stamp_ns": source_stamp_ns,
                "latest_receipt_realtime_ns": receipt_realtime_ns,
                "corrected_age_ns": corrected_age_ns,
                "max_corrected_age_ns": policy.max_corrected_age_ns,
                "within_bounds": (
                    corrected_age_ns is not None
                    and corrected_age_ns >= -self.config.max_corrected_future_ns
                    and corrected_age_ns <= policy.max_corrected_age_ns
                ),
            }
        return snapshot

    def _liveness_health(self, now_monotonic_ns: int) -> str | None:
        snapshot = self.receipt_liveness_snapshot(now_monotonic_ns)
        for name, details in snapshot.items():
            if not details["live"]:
                receipt_age_ns = details["receipt_age_ns"]
                age_text = "missing" if receipt_age_ns is None else str(receipt_age_ns)
                return (
                    f"stream_stale:{name}:receipt_age_ns={age_text}:"
                    f"limit_ns={details['stale_receipt_timeout_ns']}"
                )
        return None

    def _scheduled_statistics_health(self, now_monotonic_ns: int) -> str | None:
        previous = self._last_statistics_evaluation_monotonic_ns
        if previous is not None and now_monotonic_ns < previous:
            return "statistics_monotonic_regression"
        if not self.config.locked_statistics_enabled:
            return None
        if previous is not None and now_monotonic_ns - previous < (
            self.config.statistics_evaluation_period_ns
        ):
            return None
        self._last_statistics_evaluation_monotonic_ns = now_monotonic_ns
        return self._statistical_health()

    def _statistical_health(self) -> str | None:
        assert self.locked_offset_ns is not None
        metrics = self.metrics()
        reference = metrics[self.config.reference_stream]
        if reference is None:
            return "reference_stream_lost"
        reference_candidate = reference.lower_age_ns - self.config.offset_guard_ns
        if abs(reference_candidate - self.locked_offset_ns) > (
            self.config.max_locked_offset_deviation_ns
        ):
            return "locked_offset_deviation_exceeded"
        reference_drift = reference.robust_drift_ppm
        for name, policy in self._policies.items():
            stream_metrics = metrics[name]
            if stream_metrics is None:
                return f"stream_lost:{name}"
            if stream_metrics.duplicate_fraction > policy.max_duplicate_fraction:
                return f"duplicate_fraction_exceeded:{name}"
            drift = stream_metrics.robust_drift_ppm
            if drift is None or abs(drift) > self.config.max_absolute_drift_ppm:
                return f"clock_drift_exceeded:{name}"
            if (
                reference_drift is not None
                and abs(drift - reference_drift)
                > self.config.max_pairwise_drift_ppm
            ):
                return f"pairwise_drift_disagreement:{name}"
            relative_lag = stream_metrics.lower_age_ns - reference.lower_age_ns
            if relative_lag < -self.config.max_relative_lead_ns:
                return f"stream_leads_reference:{name}"
            if relative_lag > policy.max_relative_lag_ns:
                return f"stream_lag_exceeded:{name}"
        return None


class AffineNavigationStampDiscipline(NavigationStampDiscipline):
    """Qualify and freeze one common affine model for all required streams.

    Only ``core_streams`` contribute clock-rate estimates.  The intended Go2
    workstation set is SportModeState, MID-360 IMU, and MID-360 odometry.
    PointCloud2 is deliberately excluded from the fit: it uses the exact same
    correction model, retaining its scan-acquisition lag.

    The model is never adapted after :meth:`lock_affine`.  Continuous checks
    compare fresh lower-envelope evidence with that frozen model and latch the
    whole session FAULTED on any violation.
    """

    def __init__(
        self,
        config: DisciplineConfig,
        *,
        core_streams: tuple[str, ...],
        qualification_start_monotonic_ns: int | None = None,
        allow_locked_receipt_recovery: bool = False,
    ) -> None:
        super().__init__(config)
        if not isinstance(allow_locked_receipt_recovery, bool):
            raise ValueError("locked receipt recovery flag must be boolean")
        if len(core_streams) < 2 or len(set(core_streams)) != len(core_streams):
            raise ValueError("at least two unique affine core streams are required")
        unknown = sorted(set(core_streams) - set(self._streams))
        if unknown:
            raise ValueError("unknown affine core streams: " + ",".join(unknown))
        if config.reference_stream not in core_streams:
            raise ValueError("affine core streams must include the reference stream")
        delta_drop_streams = frozenset(
            config.affine_locked_noncore_delta_drop_streams
        )
        qualifying_delta_drop_streams = frozenset(
            config.affine_qualifying_noncore_delta_drop_streams
        )
        core_delta_recovery_streams = frozenset(
            config.affine_locked_core_delta_recovery_streams
        )
        core_drop_streams = sorted(delta_drop_streams.intersection(core_streams))
        if core_drop_streams:
            raise ValueError(
                "affine core streams cannot be delta-drop enabled: "
                + ",".join(core_drop_streams)
            )
        qualifying_core_drop_streams = sorted(
            qualifying_delta_drop_streams.intersection(core_streams)
        )
        if qualifying_core_drop_streams:
            raise ValueError(
                "affine core streams cannot be qualifying delta-drop "
                "enabled: " + ",".join(qualifying_core_drop_streams)
            )
        noncore_recovery_streams = sorted(
            core_delta_recovery_streams.difference(core_streams)
        )
        if noncore_recovery_streams:
            raise ValueError(
                "only affine core streams can use delta recovery: "
                + ",".join(noncore_recovery_streams)
            )
        self.core_streams = core_streams
        if qualification_start_monotonic_ns is not None and not _positive_int(
            qualification_start_monotonic_ns
        ):
            raise ValueError(
                "affine qualification start must be a positive integer"
            )
        self._affine_qualification_start_monotonic_ns = (
            qualification_start_monotonic_ns
        )
        self._locked_noncore_delta_drop_streams = delta_drop_streams
        self._qualifying_noncore_delta_drop_streams = (
            qualifying_delta_drop_streams
        )
        self._locked_core_delta_recovery_streams = (
            core_delta_recovery_streams
        )
        self._restart_qualification_on_core_delta_discontinuity = (
            config.affine_restart_qualification_on_core_delta_discontinuity
        )
        # This baseline tracks every structurally valid advancing observation,
        # including a deliberately discarded non-core cloud callback.  The
        # _StreamState fields remain last-good evidence: they alone feed
        # retained samples, corrected-age cache alignment, and liveness.
        self._observed_continuity: dict[
            str, tuple[int, int] | None
        ] = {name: None for name in self._streams}
        self._observed_continuity_is_last_good = {
            name: False for name in self._streams
        }
        self.locked_affine_model: AffineClockModel | None = None
        self._locked_corrected_ages: dict[str, deque[int]] | None = None
        self._last_affine_drift_diagnostics: dict[str, object] | None = None
        self._affine_lock_monotonic_ns: int | None = None
        self._affine_lock_clock_base_ns: int | None = None
        # Only the explicitly motion-disabled workstation relay enables this
        # degradation path.  A receipt outage never changes or relocks the
        # frozen affine model.  Corrected outputs pause until every stream has
        # supplied two fresh, structurally valid samples and the complete
        # locked statistical health check passes again.
        self._allow_locked_receipt_recovery = allow_locked_receipt_recovery
        self._receipt_recovery_active = False
        self._receipt_recovery_count = 0
        self._receipt_recovery_sample_counts = {
            name: 0 for name in self._streams
        }
        self._receipt_recovery_pre_gap_receipt_ns: dict[str, int | None] = {
            name: None for name in self._streams
        }
        # Keep exact receipt-pair identities after recovery completes so the
        # one audited outage gap may remain inside the rolling 60 s health
        # window without being confused with a later, unrelated gap.
        self._receipt_recovery_gap_waivers: dict[
            str, tuple[int, int, int]
        ] = {}

    def _restart_qualification_after_core_delivery_pause(
        self,
        *,
        stream: str,
        source_stamp_ns: int,
        receipt_realtime_ns: int,
        receipt_monotonic_ns: int,
        diagnostics: tuple[tuple[str, int], ...],
    ) -> CorrectionResult:
        """Discard pre-lock history and start a new complete live window."""

        self._streams = {
            name: _StreamState(self.config.retained_samples_per_stream)
            for name in self._streams
        }
        self._observed_continuity = {name: None for name in self._streams}
        self._observed_continuity_is_last_good = {
            name: False for name in self._streams
        }
        self._affine_qualification_start_monotonic_ns = None
        self._last_statistics_evaluation_monotonic_ns = None
        self._last_affine_drift_diagnostics = None

        state = self._streams[stream]
        age_ns = receipt_realtime_ns - source_stamp_ns
        state.samples.append(
            (source_stamp_ns, receipt_monotonic_ns, age_ns)
        )
        state.last_source_ns = source_stamp_ns
        state.last_advancing_receipt_monotonic_ns = receipt_monotonic_ns
        state.advancing = 1
        self._observed_continuity[stream] = (
            source_stamp_ns,
            receipt_monotonic_ns,
        )
        self._observed_continuity_is_last_good[stream] = True
        return CorrectionResult(
            False,
            None,
            False,
            f"affine_qualification_restarted_after_delivery_pause:{stream}",
            self.state,
            diagnostics,
        )

    @property
    def receipt_recovery_active(self) -> bool:
        """Whether corrected outputs are paused for a receipt-liveness gap."""

        return self._receipt_recovery_active

    def _receipt_recovery_result(
        self,
        reason: str,
        *,
        diagnostics: tuple[tuple[str, int], ...] = (),
    ) -> CorrectionResult:
        return CorrectionResult(
            False,
            None,
            False,
            f"{LOCKED_RECEIPT_RECOVERY_REASON_PREFIX}:{reason}",
            self.state,
            diagnostics,
        )

    def _record_gap_waiver(
        self,
        stream: str,
        left_receipt_ns: int,
        right_receipt_ns: int,
    ) -> None:
        gap_ns = right_receipt_ns - left_receipt_ns
        if gap_ns <= self._policies[stream].stale_receipt_timeout_ns:
            return
        candidate = (left_receipt_ns, right_receipt_ns, gap_ns)
        previous = self._receipt_recovery_gap_waivers.get(stream)
        if previous is None or candidate[2] > previous[2]:
            self._receipt_recovery_gap_waivers[stream] = candidate

    def _enter_locked_receipt_recovery(
        self,
        reason: str,
        *,
        diagnostics: tuple[tuple[str, int], ...] = (),
    ) -> CorrectionResult:
        recoverable_reason = (
            reason.startswith("stream_stale:")
            or reason.startswith("corrected_timestamp_transient_stale:")
            or reason.startswith(
                f"{LOCKED_RECEIPT_RECOVERY_DELTA_DROP_REASON_PREFIX}:"
            )
            or reason.startswith("locked_affine_common_window_incomplete:")
        )
        if (
            not self._allow_locked_receipt_recovery
            or self.state != DisciplineState.LOCKED
            or not recoverable_reason
        ):
            return self._fault(reason)
        if not self._receipt_recovery_active:
            self._receipt_recovery_active = True
            self._receipt_recovery_count += 1
            self._receipt_recovery_sample_counts = {
                name: 0 for name in self._streams
            }
            self._receipt_recovery_pre_gap_receipt_ns = {
                name: self._streams[
                    name
                ].last_advancing_receipt_monotonic_ns
                for name in self._streams
            }
            # An observe() callback may be the first code to notice that its
            # peer streams are stale.  In that case this stream's first valid
            # post-gap sample is already retained; recognize that exact pair
            # without resetting any source or receipt continuity baseline.
            for name, state in self._streams.items():
                if len(state.samples) < 2:
                    continue
                left_receipt_ns = state.samples[-2][1]
                right_receipt_ns = state.samples[-1][1]
                if (
                    right_receipt_ns - left_receipt_ns
                    > self._policies[name].stale_receipt_timeout_ns
                ):
                    self._record_gap_waiver(
                        name, left_receipt_ns, right_receipt_ns
                    )
                    self._receipt_recovery_sample_counts[name] = 1
                    self._receipt_recovery_pre_gap_receipt_ns[
                        name
                    ] = right_receipt_ns
        return self._receipt_recovery_result(
            reason, diagnostics=diagnostics
        )

    def _record_locked_receipt_recovery_sample(
        self, stream: str, receipt_monotonic_ns: int
    ) -> None:
        if not self._receipt_recovery_active:
            return
        count = self._receipt_recovery_sample_counts[stream]
        if count >= LOCKED_RECEIPT_RECOVERY_REQUIRED_SAMPLES:
            return
        if count == 0:
            previous_receipt_ns = (
                self._receipt_recovery_pre_gap_receipt_ns[stream]
            )
            if previous_receipt_ns is not None:
                self._record_gap_waiver(
                    stream, previous_receipt_ns, receipt_monotonic_ns
                )
        self._receipt_recovery_sample_counts[stream] = count + 1

    def _locked_receipt_recovery_health(
        self, now_monotonic_ns: int
    ) -> CorrectionResult | None:
        """Advance locked recovery without weakening live hard time checks."""

        reason = self._liveness_health(now_monotonic_ns)
        if reason is not None:
            return self._receipt_recovery_result(reason)
        pending = sorted(
            name
            for name, count in self._receipt_recovery_sample_counts.items()
            if count < LOCKED_RECEIPT_RECOVERY_REQUIRED_SAMPLES
        )
        if pending:
            return self._receipt_recovery_result(
                "awaiting_fresh_streams:" + ",".join(pending)
            )

        if not self.config.locked_statistics_enabled:
            # The motion profile has already passed qualification and keeps
            # the frozen model.  Fresh all-stream liveness is sufficient to
            # resume; corrected-age, clock monotonicity and source/receipt
            # delta checks still run on every accepted sample.
            self._receipt_recovery_active = False
            return None

        previous_evaluation_ns = (
            self._last_statistics_evaluation_monotonic_ns
        )
        reason = self._scheduled_statistics_health(now_monotonic_ns)
        evaluated = (
            self._last_statistics_evaluation_monotonic_ns
            != previous_evaluation_ns
        )
        if not evaluated:
            return self._receipt_recovery_result(
                "awaiting_locked_statistics"
            )
        if reason is not None:
            # A long router outage naturally leaves the exact rolling 60 s
            # fit without enough edge/bin coverage.  Stay paused until fresh
            # samples fill that same unchanged window.  Every actual drift,
            # residual, regression, clock or model violation remains terminal.
            if reason.startswith("locked_affine_common_window_incomplete:"):
                return self._receipt_recovery_result(reason)
            return self._fault(reason)

        self._receipt_recovery_active = False
        return None

    def _diagnostic_corrected_stamp_ns(self, source_stamp_ns: int) -> int | None:
        model = self.locked_affine_model
        if model is None:
            return None
        return model.corrected_stamp_ns(source_stamp_ns)

    def corrected_age_snapshot(self) -> dict[str, dict[str, int | bool | None]]:
        """Add the affine past-side floor to read-only fault diagnostics."""

        snapshot = super().corrected_age_snapshot()
        minimum_age_ns = self.config.minimum_locked_corrected_age_ns
        for details in snapshot.values():
            corrected_age_ns = details["corrected_age_ns"]
            details["minimum_locked_corrected_age_ns"] = minimum_age_ns
            details["within_bounds"] = bool(details["within_bounds"]) and (
                minimum_age_ns is None
                or (
                    isinstance(corrected_age_ns, int)
                    and not isinstance(corrected_age_ns, bool)
                    and corrected_age_ns >= minimum_age_ns
                )
            )
        return snapshot

    def _locked_noncore_drop_health(
        self, stream: str, now_monotonic_ns: int
    ) -> str | None:
        """Check retained hard state before discarding a non-core callback."""

        if self.locked_affine_model is None:
            return "affine_model_missing_after_lock"
        state = self._streams[stream]
        corrected_age_windows = self._locked_corrected_ages
        if corrected_age_windows is None or stream not in corrected_age_windows:
            return "affine_corrected_age_cache_missing_after_lock"
        corrected_ages = corrected_age_windows[stream]
        if (
            corrected_ages.maxlen != state.samples.maxlen
            or len(corrected_ages) != len(state.samples)
        ):
            return f"affine_corrected_age_cache_misaligned:{stream}"
        reason = self._liveness_health(now_monotonic_ns)
        if reason is None:
            reason = self._scheduled_statistics_health(now_monotonic_ns)
        return reason

    def _affine_metrics_for(
        self,
        streams: Iterable[str],
        *,
        samples_by_stream: dict[str, Iterable[tuple[int, int, int]]] | None = None,
    ) -> dict[str, AffineStreamMetrics]:
        result: dict[str, AffineStreamMetrics] = {}
        for name in streams:
            samples = (
                self._streams[name].samples
                if samples_by_stream is None
                else samples_by_stream[name]
            )
            points = _lower_envelope_points(
                samples,
                self.config.drift_bin_ns,
                anchor_monotonic_ns=(
                    self._affine_qualification_start_monotonic_ns or 0
                ),
            )
            result[name] = AffineStreamMetrics(
                envelope_points=len(points),
                drift_ppm=_theil_sen_age_drift_ppm(points),
            )
        return result

    def _locked_affine_drift_window(
        self,
    ) -> tuple[
        int,
        int,
        dict[str, tuple[tuple[int, int, int], ...]],
        dict[str, dict[str, int | bool | None]],
    ]:
        """Return one common recent receipt interval for every core rate fit.

        The end is the latest monotonic receipt boundary available in *all*
        core streams.  The start is exactly 60 seconds earlier, except during
        the initial post-lock period where it is clipped to the immutable
        qualification start.  Crucially, neither boundary depends on a
        deque's first retained sample, so unequal source rates and eviction
        cannot silently give the streams different fit horizons.
        """

        latest_receipts = {
            name: self._streams[name].samples[-1][1]
            for name in self.core_streams
        }
        common_end_ns = min(latest_receipts.values())
        qualification_start_ns = (
            self._affine_qualification_start_monotonic_ns
        )
        if qualification_start_ns is None:
            # A locked affine discipline can only be reached after observing
            # samples, but keep this helper fail-closed if internal state was
            # corrupted or constructed by an invalid test double.
            qualification_start_ns = common_end_ns
        common_start_ns = max(
            qualification_start_ns,
            common_end_ns - LOCKED_AFFINE_DRIFT_WINDOW_NS,
        )
        samples_by_stream: dict[
            str, tuple[tuple[int, int, int], ...]
        ] = {}
        diagnostics: dict[str, dict[str, int | bool | None]] = {}
        for name in self.core_streams:
            state = self._streams[name]
            retained = state.samples
            window_samples = tuple(
                sample
                for sample in retained
                if common_start_ns <= sample[1] <= common_end_ns
            )
            samples_by_stream[name] = window_samples
            retained_limit = retained.maxlen
            retained_samples = len(retained)
            retained_evicted = max(0, state.advancing - retained_samples)
            max_gap_pair = (
                None
                if len(window_samples) < 2
                else max(
                    zip(window_samples, window_samples[1:]),
                    key=lambda pair: pair[1][1] - pair[0][1],
                )
            )
            max_gap_ns = (
                None
                if max_gap_pair is None
                else max_gap_pair[1][1] - max_gap_pair[0][1]
            )
            diagnostics[name] = {
                "retained_samples": retained_samples,
                "retained_sample_limit": retained_limit,
                "retained_span_ns": (
                    0
                    if retained_samples < 2
                    else retained[-1][1] - retained[0][1]
                ),
                "retained_samples_evicted": retained_evicted,
                "buffer_saturated_or_evicted": bool(
                    retained_limit is not None
                    and retained_samples >= retained_limit
                    or retained_evicted > 0
                ),
                "window_start_monotonic_ns": common_start_ns,
                "window_end_monotonic_ns": common_end_ns,
                "window_samples": len(window_samples),
                "first_receipt_offset_ns": (
                    None
                    if not window_samples
                    else window_samples[0][1] - common_start_ns
                ),
                "last_receipt_offset_ns": (
                    None
                    if not window_samples
                    else common_end_ns - window_samples[-1][1]
                ),
                "max_receipt_gap_ns": max_gap_ns,
                "max_receipt_gap_start_monotonic_ns": (
                    None if max_gap_pair is None else max_gap_pair[0][1]
                ),
                "max_receipt_gap_end_monotonic_ns": (
                    None if max_gap_pair is None else max_gap_pair[1][1]
                ),
            }
        return (
            common_start_ns,
            common_end_ns,
            samples_by_stream,
            diagnostics,
        )

    def affine_metrics(self) -> dict[str, AffineStreamMetrics]:
        return self._affine_metrics_for(self._streams)

    def _interval_samples(
        self,
        start_monotonic_ns: int,
        end_monotonic_ns: int,
        *,
        end_inclusive: bool,
        samples_by_stream: dict[
            str, Iterable[tuple[int, int, int]]
        ] | None = None,
    ) -> dict[str, tuple[tuple[int, int, int], ...]]:
        def included(monotonic_ns: int) -> bool:
            return monotonic_ns >= start_monotonic_ns and (
                monotonic_ns <= end_monotonic_ns
                if end_inclusive
                else monotonic_ns < end_monotonic_ns
            )

        source_samples = (
            {name: state.samples for name, state in self._streams.items()}
            if samples_by_stream is None
            else samples_by_stream
        )
        return {
            name: tuple(
                sample for sample in source_samples[name] if included(sample[1])
            )
            for name in self._streams
        }

    def _drift_interval_evidence(
        self,
        start_monotonic_ns: int,
        end_monotonic_ns: int,
        *,
        end_inclusive: bool,
        samples_by_stream: dict[
            str, Iterable[tuple[int, int, int]]
        ] | None = None,
    ) -> AffineDriftIntervalEvidence:
        samples = self._interval_samples(
            start_monotonic_ns,
            end_monotonic_ns,
            end_inclusive=end_inclusive,
            samples_by_stream=samples_by_stream,
        )
        metrics = self._affine_metrics_for(
            self.core_streams, samples_by_stream=samples
        )
        core_drifts = tuple(
            (name, metrics[name].drift_ppm) for name in self.core_streams
        )
        finite_drifts = [value for _, value in core_drifts if value is not None]
        common = (
            float(statistics.median(finite_drifts))
            if len(finite_drifts) == len(self.core_streams)
            else None
        )
        return AffineDriftIntervalEvidence(
            start_monotonic_ns=start_monotonic_ns,
            end_monotonic_ns=end_monotonic_ns,
            end_inclusive=end_inclusive,
            core_stream_drifts_ppm=core_drifts,
            core_stream_envelope_points=tuple(
                (name, metrics[name].envelope_points)
                for name in self.core_streams
            ),
            core_stream_first_receipt_offset_ns=tuple(
                (
                    name,
                    None
                    if not samples[name]
                    else samples[name][0][1] - start_monotonic_ns,
                )
                for name in self.core_streams
            ),
            core_stream_last_receipt_offset_ns=tuple(
                (
                    name,
                    None
                    if not samples[name]
                    else end_monotonic_ns - samples[name][-1][1],
                )
                for name in self.core_streams
            ),
            core_stream_max_receipt_gap_ns=tuple(
                (
                    name,
                    None
                    if len(samples[name]) < 2
                    else max(
                        right[1] - left[1]
                        for left, right in zip(
                            samples[name], samples[name][1:]
                        )
                    ),
                )
                for name in self.core_streams
            ),
            common_drift_ppm=common,
        )

    def affine_qualification_evidence(
        self,
    ) -> tuple[
        AffineDriftIntervalEvidence,
        AffineDriftIntervalEvidence,
        AffineDriftIntervalEvidence,
    ] | None:
        """Return first, second, and complete fixed-window rate evidence."""

        window_ns = self.config.affine_qualification_window_ns
        start_ns = self._affine_qualification_start_monotonic_ns
        if window_ns is None or start_ns is None:
            return None
        split_ns = start_ns + window_ns
        end_ns = split_ns + window_ns
        return (
            self._drift_interval_evidence(
                start_ns, split_ns, end_inclusive=False
            ),
            self._drift_interval_evidence(
                split_ns, end_ns, end_inclusive=True
            ),
            self._drift_interval_evidence(
                start_ns, end_ns, end_inclusive=True
            ),
        )

    def _model_samples(
        self,
    ) -> dict[str, Iterable[tuple[int, int, int]]]:
        evidence = self.affine_qualification_evidence()
        if evidence is None:
            return {name: state.samples for name, state in self._streams.items()}
        complete = evidence[2]
        return self._interval_samples(
            complete.start_monotonic_ns,
            complete.end_monotonic_ns,
            end_inclusive=complete.end_inclusive,
        )

    def affine_drift_diagnostics(self) -> dict[str, object] | None:
        """Return a detached copy of the latest approval/locked rate check."""

        diagnostics = self._last_affine_drift_diagnostics
        if diagnostics is None:
            return None
        return copy.deepcopy(diagnostics)

    def affine_qualification_deadline_monotonic_ns(self) -> int | None:
        """Return the fixed two-window boundary using only constant-time state."""

        start_ns = self._affine_qualification_start_monotonic_ns
        window_ns = self.config.affine_qualification_window_ns
        if start_ns is None or window_ns is None:
            return None
        return start_ns + 2 * window_ns

    def approved_affine_anchor_deadline_monotonic_ns(self) -> int | None:
        """Return when a current live intercept can use an approved rate."""

        start_ns = self._affine_qualification_start_monotonic_ns
        if start_ns is None:
            return None
        return start_ns + APPROVED_AFFINE_ANCHOR_WINDOW_NS

    def capture_affine_qualification_snapshot(
        self, now_monotonic_ns: int
    ) -> AffineQualificationSnapshot:
        """Copy one immutable terminal view after both fixed windows end."""

        if not _positive_int(now_monotonic_ns):
            raise ValueError("invalid affine qualification snapshot time")
        start_ns = self._affine_qualification_start_monotonic_ns
        deadline_ns = self.affine_qualification_deadline_monotonic_ns()
        if start_ns is None or deadline_ns is None:
            raise RuntimeError("fixed affine qualification window is not initialized")
        if now_monotonic_ns < deadline_ns:
            raise RuntimeError("fixed affine qualification window is incomplete")
        return AffineQualificationSnapshot(
            qualification_start_monotonic_ns=start_ns,
            qualification_end_monotonic_ns=deadline_ns,
            evaluation_monotonic_ns=now_monotonic_ns,
            streams=tuple(
                AffineQualificationStreamSnapshot(
                    name=name,
                    samples=tuple(state.samples),
                    last_advancing_receipt_monotonic_ns=(
                        state.last_advancing_receipt_monotonic_ns
                    ),
                    advancing=state.advancing,
                    duplicates=state.duplicates,
                )
                for name, state in self._streams.items()
            ),
        )

    def affine_post_evaluation_commit_gate(
        self,
        evaluation: AffineQualificationEvaluation,
        *,
        receipt_realtime_ns: int,
        receipt_monotonic_ns: int,
        clock_read_span_ns: int,
    ) -> tuple[tuple[str, ...], dict[str, object]]:
        """Run the O(streams) gate immediately before a cached-model commit.

        The terminal fit can occupy the single-threaded executor long enough
        for receipts, the approval, or the local clock pair to become unsafe.
        This method deliberately performs no quantiles, envelopes, fitting, or
        traversal of retained samples.  It checks the fresh paired clocks and
        each stream's last advancing receipt, and records bounded evidence for
        READY or FAULT.
        """

        self._require_current_affine_snapshot(evaluation.snapshot)
        reasons: list[str] = []
        if not _positive_int(receipt_realtime_ns) or not _positive_int(
            receipt_monotonic_ns
        ):
            reasons.append("post_evaluation_clock_invalid")
        if (
            not isinstance(clock_read_span_ns, int)
            or isinstance(clock_read_span_ns, bool)
            or clock_read_span_ns < 0
            or clock_read_span_ns > self.config.max_clock_read_span_ns
        ):
            reasons.append("post_evaluation_clock_read_span_exceeded")
        if (
            _positive_int(receipt_monotonic_ns)
            and receipt_monotonic_ns
            < evaluation.snapshot.evaluation_monotonic_ns
        ):
            reasons.append("post_evaluation_monotonic_regression")

        previous_clock_base_ns = self._last_clock_base_ns
        clock_base_ns = (
            receipt_realtime_ns - receipt_monotonic_ns
            if _positive_int(receipt_realtime_ns)
            and _positive_int(receipt_monotonic_ns)
            else None
        )
        clock_base_delta_ns = (
            None
            if clock_base_ns is None or previous_clock_base_ns is None
            else abs(clock_base_ns - previous_clock_base_ns)
        )
        if previous_clock_base_ns is None:
            reasons.append("post_evaluation_clock_reference_missing")
        elif (
            clock_base_delta_ns is not None
            and clock_base_delta_ns > self.config.max_clock_pair_discontinuity_ns
        ):
            reasons.append("post_evaluation_realtime_discontinuity")

        stream_snapshots = {
            stream.name: stream for stream in evaluation.snapshot.streams
        }
        for name, state in self._streams.items():
            snapshot = stream_snapshots[name]
            snapshot_last_source_ns = (
                None if not snapshot.samples else snapshot.samples[-1][0]
            )
            if (
                len(state.samples) != len(snapshot.samples)
                or state.last_source_ns != snapshot_last_source_ns
                or state.last_advancing_receipt_monotonic_ns
                != snapshot.last_advancing_receipt_monotonic_ns
                or state.advancing != snapshot.advancing
                or state.duplicates != snapshot.duplicates
            ):
                reasons.append(f"post_evaluation_live_state_changed:{name}")

        liveness = (
            self.receipt_liveness_snapshot(receipt_monotonic_ns)
            if _positive_int(receipt_monotonic_ns)
            else {
                name: {
                    "last_advancing_receipt_monotonic_ns": (
                        state.last_advancing_receipt_monotonic_ns
                    ),
                    "receipt_age_ns": None,
                    "stale_receipt_timeout_ns": (
                        self._policies[name].stale_receipt_timeout_ns
                    ),
                    "live": False,
                }
                for name, state in self._streams.items()
            }
        )
        for name in self._streams:
            details = liveness[name]
            if not details["live"]:
                receipt_age_ns = details["receipt_age_ns"]
                age_text = "missing" if receipt_age_ns is None else str(receipt_age_ns)
                reasons.append(
                    f"stream_stale:{name}:receipt_age_ns={age_text}:"
                    f"limit_ns={details['stale_receipt_timeout_ns']}"
                )

        diagnostics: dict[str, object] = {
            "snapshot_evaluation_monotonic_ns": (
                evaluation.snapshot.evaluation_monotonic_ns
            ),
            "commit_check_realtime_ns": receipt_realtime_ns,
            "commit_check_monotonic_ns": receipt_monotonic_ns,
            "evaluation_to_commit_check_ns": (
                receipt_monotonic_ns
                - evaluation.snapshot.evaluation_monotonic_ns
                if _positive_int(receipt_monotonic_ns)
                else None
            ),
            "clock_read_span_ns": clock_read_span_ns,
            "clock_read_span_limit_ns": self.config.max_clock_read_span_ns,
            "previous_clock_base_ns": previous_clock_base_ns,
            "clock_base_ns": clock_base_ns,
            "clock_base_delta_ns": clock_base_delta_ns,
            "clock_pair_discontinuity_limit_ns": (
                self.config.max_clock_pair_discontinuity_ns
            ),
            "stream_receipt_liveness": liveness,
        }
        stable_reasons = tuple(sorted(set(reasons)))
        if not stable_reasons:
            assert clock_base_ns is not None
            self._last_clock_base_ns = clock_base_ns
        return stable_reasons, diagnostics

    def _require_current_affine_snapshot(
        self, snapshot: AffineQualificationSnapshot
    ) -> None:
        if not isinstance(snapshot, AffineQualificationSnapshot):
            raise ValueError("invalid affine qualification snapshot")
        deadline_ns = self.affine_qualification_deadline_monotonic_ns()
        if (
            snapshot.qualification_start_monotonic_ns
            != self._affine_qualification_start_monotonic_ns
            or snapshot.qualification_end_monotonic_ns != deadline_ns
            or snapshot.evaluation_monotonic_ns
            < snapshot.qualification_end_monotonic_ns
            or tuple(stream.name for stream in snapshot.streams)
            != tuple(self._streams)
        ):
            raise ValueError("affine qualification snapshot does not match runtime")

    def evaluate_affine_qualification_snapshot(
        self, snapshot: AffineQualificationSnapshot
    ) -> AffineQualificationEvaluation:
        """Perform the sole strict terminal scan over one immutable snapshot.

        All interval partitions, lower envelopes, qualification reasons, and
        the candidate model are derived together.  The runtime can therefore
        commit this cached result without calling ``qualification_reasons`` or
        rebuilding a provisional model on the executor thread.
        """

        self._require_current_affine_snapshot(snapshot)
        start_ns = snapshot.qualification_start_monotonic_ns
        end_ns = snapshot.qualification_end_monotonic_ns
        window_ns = self.config.affine_qualification_window_ns
        assert window_ns is not None
        split_ns = start_ns + window_ns
        raw_samples = snapshot.samples_by_stream()
        first_samples: dict[str, tuple[tuple[int, int, int], ...]] = {}
        second_samples: dict[str, tuple[tuple[int, int, int], ...]] = {}
        complete_samples: dict[str, tuple[tuple[int, int, int], ...]] = {}
        for name in self._streams:
            first: list[tuple[int, int, int]] = []
            second: list[tuple[int, int, int]] = []
            complete: list[tuple[int, int, int]] = []
            for sample in raw_samples[name]:
                receipt_ns = sample[1]
                if receipt_ns < start_ns or receipt_ns > end_ns:
                    continue
                complete.append(sample)
                if receipt_ns < split_ns:
                    first.append(sample)
                else:
                    second.append(sample)
            first_samples[name] = tuple(first)
            second_samples[name] = tuple(second)
            complete_samples[name] = tuple(complete)

        envelope_sets = {
            "first": {
                name: _lower_envelope_points(
                    first_samples[name],
                    self.config.drift_bin_ns,
                    anchor_monotonic_ns=start_ns,
                )
                for name in self.core_streams
            },
            "second": {
                name: _lower_envelope_points(
                    second_samples[name],
                    self.config.drift_bin_ns,
                    anchor_monotonic_ns=start_ns,
                )
                for name in self.core_streams
            },
            "complete": {
                name: _lower_envelope_points(
                    complete_samples[name],
                    self.config.drift_bin_ns,
                    anchor_monotonic_ns=start_ns,
                )
                for name in self._streams
            },
        }

        def interval_evidence(
            interval_start_ns: int,
            interval_end_ns: int,
            *,
            end_inclusive: bool,
            samples: dict[str, tuple[tuple[int, int, int], ...]],
            envelope_points: dict[str, tuple[tuple[int, int, int], ...]],
        ) -> AffineDriftIntervalEvidence:
            core_drifts = tuple(
                (name, _theil_sen_age_drift_ppm(envelope_points[name]))
                for name in self.core_streams
            )
            finite_drifts = [
                drift for _, drift in core_drifts if drift is not None
            ]
            common_drift_ppm = (
                float(statistics.median(finite_drifts))
                if len(finite_drifts) == len(self.core_streams)
                else None
            )
            return AffineDriftIntervalEvidence(
                start_monotonic_ns=interval_start_ns,
                end_monotonic_ns=interval_end_ns,
                end_inclusive=end_inclusive,
                core_stream_drifts_ppm=core_drifts,
                core_stream_envelope_points=tuple(
                    (name, len(envelope_points[name]))
                    for name in self.core_streams
                ),
                core_stream_first_receipt_offset_ns=tuple(
                    (
                        name,
                        None
                        if not samples[name]
                        else samples[name][0][1] - interval_start_ns,
                    )
                    for name in self.core_streams
                ),
                core_stream_last_receipt_offset_ns=tuple(
                    (
                        name,
                        None
                        if not samples[name]
                        else interval_end_ns - samples[name][-1][1],
                    )
                    for name in self.core_streams
                ),
                core_stream_max_receipt_gap_ns=tuple(
                    (
                        name,
                        None
                        if len(samples[name]) < 2
                        else max(
                            right[1] - left[1]
                            for left, right in zip(
                                samples[name], samples[name][1:]
                            )
                        ),
                    )
                    for name in self.core_streams
                ),
                common_drift_ppm=common_drift_ppm,
            )

        first_evidence = interval_evidence(
            start_ns,
            split_ns,
            end_inclusive=False,
            samples=first_samples,
            envelope_points=envelope_sets["first"],
        )
        second_evidence = interval_evidence(
            split_ns,
            end_ns,
            end_inclusive=True,
            samples=second_samples,
            envelope_points=envelope_sets["second"],
        )
        complete_evidence = interval_evidence(
            start_ns,
            end_ns,
            end_inclusive=True,
            samples=complete_samples,
            envelope_points={
                name: envelope_sets["complete"][name]
                for name in self.core_streams
            },
        )
        interval_evidence_set = (
            first_evidence,
            second_evidence,
            complete_evidence,
        )

        stream_snapshots = {stream.name: stream for stream in snapshot.streams}
        metrics: dict[str, StreamMetrics | None] = {}

        def ordered_quantile(ordered: list[int], fraction: float) -> int:
            index = max(
                0,
                min(
                    len(ordered) - 1,
                    math.ceil(fraction * len(ordered)) - 1,
                ),
            )
            return ordered[index]

        for name, samples in complete_samples.items():
            if not samples:
                metrics[name] = None
                continue
            ages = sorted(sample[2] for sample in samples)
            state = stream_snapshots[name]
            total = state.advancing + state.duplicates
            metrics[name] = StreamMetrics(
                samples=len(samples),
                span_ns=max(0, samples[-1][1] - samples[0][1]),
                lower_age_ns=ordered_quantile(
                    ages, self.config.lower_quantile
                ),
                median_age_ns=ordered_quantile(ages, 0.50),
                p95_age_ns=ordered_quantile(ages, 0.95),
                # The affine lower-envelope drifts below are the only clock
                # rate evidence used by this qualification path.
                robust_drift_ppm=None,
                duplicate_fraction=(state.duplicates / total) if total else 0.0,
            )

        reasons: list[str] = []
        reference = metrics[self.config.reference_stream]
        if reference is None:
            reasons.append("missing_reference_stream")
        for name, policy in self._policies.items():
            stream_metrics = metrics[name]
            stream_snapshot = stream_snapshots[name]
            if stream_metrics is None:
                reasons.append(f"missing_stream:{name}")
                continue
            if stream_metrics.samples < policy.min_samples:
                reasons.append(f"insufficient_samples:{name}")
            if stream_metrics.span_ns < self.config.minimum_qualification_span_ns:
                reasons.append(f"insufficient_span:{name}")
            last_receipt = stream_snapshot.last_advancing_receipt_monotonic_ns
            receipt_age_ns = (
                None
                if last_receipt is None
                else snapshot.evaluation_monotonic_ns - last_receipt
            )
            if (
                receipt_age_ns is None
                or receipt_age_ns < 0
                or receipt_age_ns > policy.stale_receipt_timeout_ns
            ):
                reasons.append(f"stream_not_live:{name}")
            if stream_metrics.duplicate_fraction > policy.max_duplicate_fraction:
                reasons.append(f"excessive_duplicates:{name}")
            if reference is not None:
                relative_lag_ns = (
                    stream_metrics.lower_age_ns - reference.lower_age_ns
                )
                if relative_lag_ns < -self.config.max_relative_lead_ns:
                    reasons.append(f"stream_leads_reference:{name}")
                if relative_lag_ns > policy.max_relative_lag_ns:
                    reasons.append(f"stream_lag_exceeded:{name}")

        complete_core_drifts: list[tuple[str, float]] = []
        for name, drift in complete_evidence.core_stream_drifts_ppm:
            if drift is None:
                reasons.append(f"insufficient_affine_drift_bins:{name}")
                continue
            complete_core_drifts.append((name, drift))
            if abs(drift) > self.config.max_absolute_drift_ppm:
                reasons.append(f"clock_drift_exceeded:{name}")
        if len(complete_core_drifts) == len(self.core_streams):
            for name in _pairwise_drift_disagreements(
                complete_core_drifts, self.config.max_pairwise_drift_ppm
            ):
                reasons.append(f"pairwise_drift_disagreement:{name}")

        required_points = window_ns // self.config.drift_bin_ns
        for label, evidence in (
            ("first", first_evidence),
            ("second", second_evidence),
        ):
            point_counts = dict(evidence.core_stream_envelope_points)
            first_offsets = dict(evidence.core_stream_first_receipt_offset_ns)
            last_offsets = dict(evidence.core_stream_last_receipt_offset_ns)
            max_gaps = dict(evidence.core_stream_max_receipt_gap_ns)
            window_drifts: list[tuple[str, float]] = []
            for name, drift in evidence.core_stream_drifts_ppm:
                coverage_limit_ns = self._policies[name].stale_receipt_timeout_ns
                if (
                    point_counts[name] < required_points
                    or drift is None
                    or first_offsets[name] is None
                    or first_offsets[name] > coverage_limit_ns
                    or last_offsets[name] is None
                    or last_offsets[name] > coverage_limit_ns
                    or max_gaps[name] is None
                    or max_gaps[name] > coverage_limit_ns
                ):
                    reasons.append(f"insufficient_affine_window:{label}:{name}")
                    continue
                window_drifts.append((name, drift))
                if abs(drift) > self.config.max_absolute_drift_ppm:
                    reasons.append(
                        f"affine_window_clock_drift_exceeded:{label}:{name}"
                    )
            if (
                self.config.affine_enforce_subwindow_pairwise_drift
                and len(window_drifts) == len(self.core_streams)
            ):
                for name in _pairwise_drift_disagreements(
                    window_drifts, self.config.max_pairwise_drift_ppm
                ):
                    reasons.append(
                        f"affine_window_pairwise_drift_disagreement:{label}:{name}"
                    )
        if (
            self.config.affine_enforce_subwindow_common_drift
            and
            first_evidence.common_drift_ppm is not None
            and second_evidence.common_drift_ppm is not None
            and abs(
                first_evidence.common_drift_ppm
                - second_evidence.common_drift_ppm
            )
            > self.config.max_affine_window_common_drift_deviation_ppm
        ):
            reasons.append("affine_window_common_drift_deviation_exceeded")

        model: AffineClockModel | None = None
        corrected_age_windows: tuple[tuple[str, tuple[int, ...]], ...] = ()
        if len(complete_core_drifts) == len(self.core_streams):
            common_drift_ppm = float(
                statistics.median(value for _, value in complete_core_drifts)
            )
            scale = _source_to_local_scale(common_drift_ppm)
            reference_points = envelope_sets["complete"][
                self.config.reference_stream
            ]
            if reference_points:
                anchor_source_ns = max(point[0] for point in reference_points)
                age_slope_per_source = scale - 1.0
                reference_anchor_ages = [
                    age_ns
                    - (source_ns - anchor_source_ns) * age_slope_per_source
                    for source_ns, _, age_ns in reference_points
                ]
                predicted_reference_age_ns = round(
                    statistics.median(reference_anchor_ages)
                )
                anchor_local_ns = (
                    anchor_source_ns
                    + predicted_reference_age_ns
                    - self.config.affine_anchor_past_guard_ns
                )
                candidate = AffineClockModel(
                    anchor_source_ns=anchor_source_ns,
                    anchor_local_ns=anchor_local_ns,
                    drift_ppm=common_drift_ppm,
                    source_to_local_scale=scale,
                    core_stream_drifts_ppm=tuple(complete_core_drifts),
                    stream_baseline_corrected_age_ns=(),
                )
                baselines: list[tuple[str, int]] = []
                cached_windows: list[tuple[str, tuple[int, ...]]] = []
                for name in self._streams:
                    # One pass over the immutable snapshot serves both the
                    # fixed-window candidate checks and the future locked
                    # health cache.  Samples received just after the exact
                    # second-window boundary remain in the locked cache but
                    # do not influence the qualified model or its baselines.
                    qualified_corrected_ages: list[int] = []
                    locked_corrected_ages: list[int] = []
                    for source_ns, receipt_ns, age_ns in raw_samples[name]:
                        corrected_age_ns = (
                            age_ns - candidate.offset_at_source_ns(source_ns)
                        )
                        locked_corrected_ages.append(corrected_age_ns)
                        if start_ns <= receipt_ns <= end_ns:
                            qualified_corrected_ages.append(corrected_age_ns)
                    if not qualified_corrected_ages:
                        break
                    corrected_ages = sorted(qualified_corrected_ages)
                    corrected_min_ns = ordered_quantile(
                        corrected_ages, self.config.lower_quantile
                    )
                    corrected_p95_ns = ordered_quantile(
                        corrected_ages, 0.95
                    )
                    policy = self._policies[name]
                    if corrected_min_ns < -self.config.max_corrected_future_ns:
                        reasons.append(f"candidate_would_be_future:{name}")
                    elif (
                        self.config.minimum_locked_corrected_age_ns is not None
                        and corrected_min_ns
                        < self.config.minimum_locked_corrected_age_ns
                    ):
                        reasons.append(
                            f"candidate_locked_corrected_age_below_minimum:{name}"
                        )
                    if corrected_p95_ns > policy.max_corrected_age_ns:
                        reasons.append(f"candidate_would_be_stale:{name}")
                    baselines.append((name, corrected_min_ns))
                    cached_windows.append(
                        (name, tuple(locked_corrected_ages))
                    )
                if len(baselines) == len(self._streams):
                    model = replace(
                        candidate,
                        stream_baseline_corrected_age_ns=tuple(baselines),
                    )
                    corrected_age_windows = tuple(cached_windows)

        return AffineQualificationEvaluation(
            snapshot=snapshot,
            reasons=tuple(sorted(set(reasons))),
            interval_evidence=interval_evidence_set,
            model=model,
            corrected_age_windows=corrected_age_windows,
        )

    def affine_qualification_fault_diagnostics(
        self,
        evaluation: AffineQualificationEvaluation,
        *,
        terminal_phase: str,
        reasons: Iterable[str] | None = None,
    ) -> dict[str, object]:
        """Render complete, bounded evidence for one terminal decision."""

        self._require_current_affine_snapshot(evaluation.snapshot)

        def interval_payload(
            evidence: AffineDriftIntervalEvidence,
        ) -> dict[str, object]:
            drifts = dict(evidence.core_stream_drifts_ppm)
            points = dict(evidence.core_stream_envelope_points)
            first = dict(evidence.core_stream_first_receipt_offset_ns)
            last = dict(evidence.core_stream_last_receipt_offset_ns)
            gaps = dict(evidence.core_stream_max_receipt_gap_ns)
            return {
                "start_monotonic_ns": evidence.start_monotonic_ns,
                "end_monotonic_ns": evidence.end_monotonic_ns,
                "end_inclusive": evidence.end_inclusive,
                "common_drift_ppm": evidence.common_drift_ppm,
                "cores": {
                    name: {
                        "envelope_points": points[name],
                        "first_receipt_offset_ns": first[name],
                        "last_receipt_offset_ns": last[name],
                        "max_receipt_gap_ns": gaps[name],
                        "drift_ppm": drifts[name],
                    }
                    for name in self.core_streams
                },
            }

        first, second, complete = evaluation.interval_evidence
        return {
            "terminal_phase": terminal_phase,
            "evaluation_monotonic_ns": (
                evaluation.snapshot.evaluation_monotonic_ns
            ),
            "qualification_start_monotonic_ns": (
                evaluation.snapshot.qualification_start_monotonic_ns
            ),
            "qualification_end_monotonic_ns": (
                evaluation.snapshot.qualification_end_monotonic_ns
            ),
            "reasons": list(evaluation.reasons if reasons is None else reasons),
            "windows": {
                "first": interval_payload(first),
                "second": interval_payload(second),
                "complete": interval_payload(complete),
            },
        }

    def reject_affine_qualification(
        self, evaluation: AffineQualificationEvaluation
    ) -> str:
        """Latch one failed terminal evaluation; retry/re-lock is forbidden."""

        self._require_current_affine_snapshot(evaluation.snapshot)
        if self.state == DisciplineState.LOCKED:
            raise RuntimeError("cannot reject an already locked affine model")
        if self.state == DisciplineState.FAULTED:
            return self.fault_reason or "affine_qualification_terminal_failed"
        self.state = DisciplineState.FAULTED
        self.fault_reason = "affine_qualification_terminal_failed"
        return self.fault_reason

    def _provisional_affine_model(self) -> AffineClockModel | None:
        model_samples = self._model_samples()
        envelope_by_stream = {
            name: _lower_envelope_points(
                model_samples[name],
                self.config.drift_bin_ns,
                anchor_monotonic_ns=(
                    self._affine_qualification_start_monotonic_ns or 0
                ),
            )
            for name in self._streams
        }
        core_drifts: list[tuple[str, float]] = []
        for name in self.core_streams:
            drift = _theil_sen_age_drift_ppm(envelope_by_stream[name])
            if drift is None:
                return None
            core_drifts.append((name, drift))
        common_drift_ppm = float(statistics.median(value for _, value in core_drifts))
        scale = _source_to_local_scale(common_drift_ppm)

        reference_points = envelope_by_stream[self.config.reference_stream]
        if not reference_points:
            return None
        anchor_source_ns = max(point[0] for point in reference_points)
        age_slope_per_source = scale - 1.0
        reference_anchor_ages = [
            age_ns
            - (source_ns - anchor_source_ns) * age_slope_per_source
            for source_ns, _, age_ns in reference_points
        ]
        predicted_reference_age_ns = round(statistics.median(reference_anchor_ages))
        anchor_local_ns = (
            anchor_source_ns
            + predicted_reference_age_ns
            - self.config.affine_anchor_past_guard_ns
        )
        model = AffineClockModel(
            anchor_source_ns=anchor_source_ns,
            anchor_local_ns=anchor_local_ns,
            drift_ppm=common_drift_ppm,
            source_to_local_scale=scale,
            core_stream_drifts_ppm=tuple(core_drifts),
            stream_baseline_corrected_age_ns=(),
        )
        baselines: list[tuple[str, int]] = []
        for name in self._streams:
            corrected_ages = [
                age_ns - model.offset_at_source_ns(source_ns)
                for source_ns, _, age_ns in model_samples[name]
            ]
            if not corrected_ages:
                return None
            baselines.append(
                (name, _quantile(corrected_ages, self.config.lower_quantile))
            )
        return replace(model, stream_baseline_corrected_age_ns=tuple(baselines))

    def qualification_reasons(self, now_monotonic_ns: int) -> list[str]:
        if not _positive_int(now_monotonic_ns):
            return ["invalid_qualification_time"]
        reasons: list[str] = []
        metrics = self.metrics()
        reference = metrics[self.config.reference_stream]
        if reference is None:
            return ["missing_reference_stream"]

        for name, policy in self._policies.items():
            stream_metrics = metrics[name]
            stream_state = self._streams[name]
            if stream_metrics is None:
                reasons.append(f"missing_stream:{name}")
                continue
            if stream_metrics.samples < policy.min_samples:
                reasons.append(f"insufficient_samples:{name}")
            if stream_metrics.span_ns < self.config.minimum_qualification_span_ns:
                reasons.append(f"insufficient_span:{name}")
            last_receipt = stream_state.last_advancing_receipt_monotonic_ns
            receipt_age_ns = (
                None if last_receipt is None else now_monotonic_ns - last_receipt
            )
            if (
                receipt_age_ns is None
                or receipt_age_ns < 0
                or receipt_age_ns > policy.stale_receipt_timeout_ns
            ):
                reasons.append(f"stream_not_live:{name}")
            if stream_metrics.duplicate_fraction > policy.max_duplicate_fraction:
                reasons.append(f"excessive_duplicates:{name}")

            relative_lag_ns = stream_metrics.lower_age_ns - reference.lower_age_ns
            if relative_lag_ns < -self.config.max_relative_lead_ns:
                reasons.append(f"stream_leads_reference:{name}")
            if relative_lag_ns > policy.max_relative_lag_ns:
                reasons.append(f"stream_lag_exceeded:{name}")

        affine_metrics = (
            self.affine_metrics()
            if self.config.affine_qualification_window_ns is None
            else self._affine_metrics_for(
                self._streams, samples_by_stream=self._model_samples()
            )
        )
        available_core_drifts: list[tuple[str, float]] = []
        for name in self.core_streams:
            stream_metrics = affine_metrics[name]
            drift = stream_metrics.drift_ppm
            if drift is None:
                reasons.append(f"insufficient_affine_drift_bins:{name}")
                continue
            available_core_drifts.append((name, drift))
            if abs(drift) > self.config.max_absolute_drift_ppm:
                reasons.append(f"clock_drift_exceeded:{name}")
        if len(available_core_drifts) == len(self.core_streams):
            for name in _pairwise_drift_disagreements(
                available_core_drifts, self.config.max_pairwise_drift_ppm
            ):
                reasons.append(f"pairwise_drift_disagreement:{name}")

        window_ns = self.config.affine_qualification_window_ns
        if window_ns is not None:
            start_ns = self._affine_qualification_start_monotonic_ns
            if start_ns is None:
                reasons.append("missing_affine_qualification_start")
            elif now_monotonic_ns < start_ns + 2 * window_ns:
                reasons.append("insufficient_affine_two_window_duration")
            evidence = self.affine_qualification_evidence()
            if evidence is not None:
                first, second, _complete = evidence
                required_points = window_ns // self.config.drift_bin_ns
                for label, interval in (("first", first), ("second", second)):
                    point_counts = dict(interval.core_stream_envelope_points)
                    first_offsets = dict(
                        interval.core_stream_first_receipt_offset_ns
                    )
                    last_offsets = dict(
                        interval.core_stream_last_receipt_offset_ns
                    )
                    max_gaps = dict(interval.core_stream_max_receipt_gap_ns)
                    window_drifts: list[tuple[str, float]] = []
                    for name, drift in interval.core_stream_drifts_ppm:
                        coverage_limit_ns = self._policies[
                            name
                        ].stale_receipt_timeout_ns
                        if (
                            point_counts[name] < required_points
                            or drift is None
                            or first_offsets[name] is None
                            or first_offsets[name] > coverage_limit_ns
                            or last_offsets[name] is None
                            or last_offsets[name] > coverage_limit_ns
                            or max_gaps[name] is None
                            or max_gaps[name] > coverage_limit_ns
                        ):
                            reasons.append(
                                f"insufficient_affine_window:{label}:{name}"
                            )
                            continue
                        window_drifts.append((name, drift))
                        if abs(drift) > self.config.max_absolute_drift_ppm:
                            reasons.append(
                                f"affine_window_clock_drift_exceeded:{label}:{name}"
                            )
                    if (
                        self.config.affine_enforce_subwindow_pairwise_drift
                        and len(window_drifts) == len(self.core_streams)
                    ):
                        for name in _pairwise_drift_disagreements(
                            window_drifts, self.config.max_pairwise_drift_ppm
                        ):
                            reasons.append(
                                f"affine_window_pairwise_drift_disagreement:"
                                f"{label}:{name}"
                            )
                if (
                    self.config.affine_enforce_subwindow_common_drift
                    and
                    first.common_drift_ppm is not None
                    and second.common_drift_ppm is not None
                    and abs(
                        first.common_drift_ppm - second.common_drift_ppm
                    )
                    > self.config.max_affine_window_common_drift_deviation_ppm
                ):
                    reasons.append(
                        "affine_window_common_drift_deviation_exceeded"
                    )

        candidate = self._provisional_affine_model()
        if candidate is not None:
            for name, policy in self._policies.items():
                corrected_ages = [
                    age_ns - candidate.offset_at_source_ns(source_ns)
                    for source_ns, _, age_ns in self._model_samples()[name]
                ]
                corrected_min_ns = _quantile(
                    corrected_ages, self.config.lower_quantile
                )
                corrected_p95_ns = _quantile(corrected_ages, 0.95)
                if corrected_min_ns < -self.config.max_corrected_future_ns:
                    reasons.append(f"candidate_would_be_future:{name}")
                elif (
                    self.config.minimum_locked_corrected_age_ns is not None
                    and corrected_min_ns
                    < self.config.minimum_locked_corrected_age_ns
                ):
                    reasons.append(
                        f"candidate_locked_corrected_age_below_minimum:{name}"
                    )
                if corrected_p95_ns > policy.max_corrected_age_ns:
                    reasons.append(f"candidate_would_be_stale:{name}")
        return sorted(set(reasons))

    def lock_affine(
        self,
        *,
        identity_evidence_verified: bool,
        now_monotonic_ns: int,
        approved_reference_offset_ns: int | None = None,
        approved_common_drift_ppm: float | None = None,
        qualification_evaluation: AffineQualificationEvaluation | None = None,
    ) -> AffineClockModel:
        """Freeze one affine model after all qualification gates pass."""

        if self.state == DisciplineState.FAULTED:
            raise RuntimeError(self.fault_reason or "discipline faulted")
        if self.state == DisciplineState.LOCKED:
            if self.locked_affine_model is None:
                raise RuntimeError("discipline is already locked in fixed-offset mode")
            if approved_common_drift_ppm is not None and abs(
                self.locked_affine_model.drift_ppm - approved_common_drift_ppm
            ) > self.config.max_approved_affine_drift_deviation_ppm:
                raise RuntimeError(
                    "discipline is already locked to a different affine drift"
                )
            return self.locked_affine_model
        if identity_evidence_verified is not True:
            raise RuntimeError("clock-domain identity evidence is not verified")
        cached_corrected_ages: dict[str, tuple[int, ...]] | None = None
        if qualification_evaluation is None:
            reasons = self.qualification_reasons(now_monotonic_ns)
            model = self._provisional_affine_model()
        else:
            self._require_current_affine_snapshot(
                qualification_evaluation.snapshot
            )
            if (
                not _positive_int(now_monotonic_ns)
                or now_monotonic_ns
                < qualification_evaluation.snapshot.evaluation_monotonic_ns
            ):
                raise RuntimeError(
                    "affine commit time precedes qualification evaluation"
                )
            # No callback can mutate state while the single-threaded terminal
            # evaluator runs.  Check that invariant in O(streams) before
            # consuming its immutable cache; never compare or traverse live
            # sample deques here.
            snapshots = {
                stream.name: stream
                for stream in qualification_evaluation.snapshot.streams
            }
            for name, state in self._streams.items():
                stream_snapshot = snapshots[name]
                snapshot_last_source_ns = (
                    None
                    if not stream_snapshot.samples
                    else stream_snapshot.samples[-1][0]
                )
                if (
                    len(state.samples) != len(stream_snapshot.samples)
                    or state.last_source_ns != snapshot_last_source_ns
                    or state.last_advancing_receipt_monotonic_ns
                    != stream_snapshot.last_advancing_receipt_monotonic_ns
                    or state.advancing != stream_snapshot.advancing
                    or state.duplicates != stream_snapshot.duplicates
                ):
                    raise RuntimeError(
                        "affine live samples changed after terminal snapshot"
                    )
            cached_corrected_ages = dict(
                qualification_evaluation.corrected_age_windows
            )
            if set(cached_corrected_ages) != set(self._streams) or any(
                len(cached_corrected_ages[name])
                != len(snapshots[name].samples)
                for name in self._streams
            ):
                raise RuntimeError(
                    "affine qualification corrected-age cache is incomplete"
                )
            reasons = list(qualification_evaluation.reasons)
            model = qualification_evaluation.model
        if reasons:
            raise RuntimeError("timestamp qualification failed: " + ",".join(reasons))
        if model is None:
            raise RuntimeError("timestamp qualification did not produce an affine model")
        if (
            approved_common_drift_ppm is not None
            and (
                not isinstance(approved_common_drift_ppm, (int, float))
                or isinstance(approved_common_drift_ppm, bool)
                or not math.isfinite(approved_common_drift_ppm)
                or abs(approved_common_drift_ppm)
                > self.config.max_absolute_drift_ppm
            )
        ):
            raise RuntimeError("approved affine common drift is invalid")
        if approved_common_drift_ppm is not None:
            approved_common_drift_ppm = float(approved_common_drift_ppm)
            approval_deviation_ppm = abs(
                model.drift_ppm - approved_common_drift_ppm
            )
            self._last_affine_drift_diagnostics = {
                "comparison_phase": "approval_before_ready",
                "locked_core_stream_drifts_ppm": {},
                "locked_common_drift_ppm": approved_common_drift_ppm,
                "current_core_stream_drifts_ppm": dict(
                    model.core_stream_drifts_ppm
                ),
                "current_common_drift_ppm": model.drift_ppm,
                "common_drift_deviation_ppm": approval_deviation_ppm,
                "common_drift_deviation_limit_ppm": (
                    self.config.max_approved_affine_drift_deviation_ppm
                ),
            }
            if approval_deviation_ppm > (
                self.config.max_approved_affine_drift_deviation_ppm
            ):
                self.state = DisciplineState.FAULTED
                self.fault_reason = (
                    "approved_affine_common_drift_deviation_exceeded"
                )
                raise RuntimeError(self.fault_reason)
        if approved_reference_offset_ns is not None:
            if (
                not isinstance(approved_reference_offset_ns, int)
                or isinstance(approved_reference_offset_ns, bool)
            ):
                raise RuntimeError("approved reference offset must be an integer")
            observed_offset = model.offset_at_source_ns(model.anchor_source_ns)
            if abs(observed_offset - approved_reference_offset_ns) > (
                self.config.max_locked_offset_deviation_ns
            ):
                raise RuntimeError(
                    "approved reference offset disagrees with affine qualification"
                )
        # Both deques use the same maxlen and are advanced together.  The
        # strict terminal path consumes corrected ages derived from its
        # immutable snapshot; it never recomputes from mutable live deques.
        corrected_age_windows: dict[str, deque[int]] = {}
        for name, state in self._streams.items():
            retained_limit = state.samples.maxlen
            assert retained_limit is not None
            if cached_corrected_ages is None:
                values = tuple(
                    age_ns - model.offset_at_source_ns(source_ns)
                    for source_ns, _, age_ns in state.samples
                )
            else:
                values = cached_corrected_ages[name]
            corrected_age_windows[name] = deque(values, maxlen=retained_limit)
        self.locked_affine_model = model
        self._locked_corrected_ages = corrected_age_windows
        self._affine_lock_monotonic_ns = now_monotonic_ns
        self._affine_lock_clock_base_ns = self._last_clock_base_ns
        self.state = DisciplineState.LOCKED
        self._last_statistics_evaluation_monotonic_ns = now_monotonic_ns
        return model

    def lock_affine_from_approved_drift(
        self,
        *,
        approved_common_drift_ppm: float,
        identity_evidence_verified: bool,
        now_monotonic_ns: int,
    ) -> AffineClockModel:
        """Bind an independently approved rate to the current live intercept.

        This is the motion-start fast path.  It deliberately does not estimate
        clock rate again from a short, scheduler-sensitive Wi-Fi window.  The
        approved slope is frozen exactly, while a recent all-stream window
        supplies only the current intercept and stream-relative baselines.
        """

        if self.state == DisciplineState.FAULTED:
            raise RuntimeError(self.fault_reason or "discipline faulted")
        if self.state == DisciplineState.LOCKED:
            model = self.locked_affine_model
            if model is None:
                raise RuntimeError(
                    "discipline is already locked in fixed-offset mode"
                )
            if not isinstance(approved_common_drift_ppm, (int, float)) or (
                isinstance(approved_common_drift_ppm, bool)
                or not math.isfinite(approved_common_drift_ppm)
                or abs(model.drift_ppm - float(approved_common_drift_ppm))
                > 1e-9
            ):
                raise RuntimeError(
                    "discipline is already locked to a different affine drift"
                )
            return model
        if identity_evidence_verified is not True:
            raise RuntimeError("clock-domain identity evidence is not verified")
        if (
            not isinstance(approved_common_drift_ppm, (int, float))
            or isinstance(approved_common_drift_ppm, bool)
            or not math.isfinite(approved_common_drift_ppm)
            or abs(approved_common_drift_ppm)
            > self.config.max_absolute_drift_ppm
        ):
            raise RuntimeError("approved affine common drift is invalid")
        if not _positive_int(now_monotonic_ns):
            raise RuntimeError("invalid affine commit time")
        qualification_start_ns = (
            self._affine_qualification_start_monotonic_ns
        )
        deadline_ns = self.approved_affine_anchor_deadline_monotonic_ns()
        if (
            qualification_start_ns is None
            or deadline_ns is None
            or now_monotonic_ns < deadline_ns
        ):
            raise RuntimeError("approved affine live anchor window is incomplete")

        window_start_ns = max(
            qualification_start_ns,
            now_monotonic_ns - APPROVED_AFFINE_ANCHOR_WINDOW_NS,
        )
        recent_samples: dict[
            str, tuple[tuple[int, int, int], ...]
        ] = {}
        for name, state in self._streams.items():
            samples = tuple(
                sample
                for sample in state.samples
                if window_start_ns <= sample[1] <= now_monotonic_ns
            )
            if len(samples) < 2:
                raise RuntimeError(f"approved_affine_live_samples_missing:{name}")
            policy = self._policies[name]
            first_gap_ns = samples[0][1] - window_start_ns
            last_gap_ns = now_monotonic_ns - samples[-1][1]
            maximum_gap_ns = max(
                second[1] - first[1]
                for first, second in zip(samples, samples[1:])
            )
            if (
                first_gap_ns > policy.stale_receipt_timeout_ns
                or last_gap_ns > policy.stale_receipt_timeout_ns
                or maximum_gap_ns > policy.stale_receipt_timeout_ns
            ):
                raise RuntimeError(f"approved_affine_stream_not_live:{name}")
            recent_samples[name] = samples

        approved_common_drift_ppm = float(approved_common_drift_ppm)
        scale = _source_to_local_scale(approved_common_drift_ppm)
        reference_samples = recent_samples[self.config.reference_stream]
        anchor_source_ns = max(sample[0] for sample in reference_samples)
        age_slope_per_source = scale - 1.0
        reference_anchor_ages = [
            age_ns
            - (source_ns - anchor_source_ns) * age_slope_per_source
            for source_ns, _, age_ns in reference_samples
        ]
        predicted_reference_age_ns = _quantile(
            (round(value) for value in reference_anchor_ages),
            self.config.lower_quantile,
        )
        anchor_local_ns = (
            anchor_source_ns
            + predicted_reference_age_ns
            - self.config.affine_anchor_past_guard_ns
        )
        candidate = AffineClockModel(
            anchor_source_ns=anchor_source_ns,
            anchor_local_ns=anchor_local_ns,
            drift_ppm=approved_common_drift_ppm,
            source_to_local_scale=scale,
            core_stream_drifts_ppm=tuple(
                (name, approved_common_drift_ppm)
                for name in self.core_streams
            ),
            stream_baseline_corrected_age_ns=(),
        )

        baselines: list[tuple[str, int]] = []
        reference_baseline_ns: int | None = None
        for name in self._streams:
            corrected_ages = [
                age_ns - candidate.offset_at_source_ns(source_ns)
                for source_ns, _, age_ns in recent_samples[name]
            ]
            corrected_min_ns = min(corrected_ages)
            corrected_p95_ns = _quantile(corrected_ages, 0.95)
            baseline_ns = _quantile(
                corrected_ages, self.config.lower_quantile
            )
            policy = self._policies[name]
            if corrected_min_ns < -self.config.max_corrected_future_ns:
                raise RuntimeError(f"candidate_would_be_future:{name}")
            if corrected_p95_ns > policy.max_corrected_age_ns:
                raise RuntimeError(f"candidate_would_be_stale:{name}")
            if name == self.config.reference_stream:
                reference_baseline_ns = baseline_ns
            baselines.append((name, baseline_ns))
        assert reference_baseline_ns is not None
        for name, baseline_ns in baselines:
            relative_lag_ns = baseline_ns - reference_baseline_ns
            if relative_lag_ns < -self.config.max_relative_lead_ns:
                raise RuntimeError(f"stream_leads_reference:{name}")
            if relative_lag_ns > self._policies[name].max_relative_lag_ns:
                raise RuntimeError(f"stream_lag_exceeded:{name}")

        model = replace(
            candidate,
            stream_baseline_corrected_age_ns=tuple(baselines),
        )
        corrected_age_windows: dict[str, deque[int]] = {}
        for name, state in self._streams.items():
            retained_limit = state.samples.maxlen
            assert retained_limit is not None
            corrected_age_windows[name] = deque(
                (
                    age_ns - model.offset_at_source_ns(source_ns)
                    for source_ns, _, age_ns in state.samples
                ),
                maxlen=retained_limit,
            )
        self.locked_affine_model = model
        self._locked_corrected_ages = corrected_age_windows
        self._affine_lock_monotonic_ns = now_monotonic_ns
        self._affine_lock_clock_base_ns = self._last_clock_base_ns
        self.state = DisciplineState.LOCKED
        self._last_statistics_evaluation_monotonic_ns = now_monotonic_ns
        self._last_affine_drift_diagnostics = {
            "comparison_phase": "approved_drift_live_anchor",
            "locked_core_stream_drifts_ppm": dict(
                model.core_stream_drifts_ppm
            ),
            "locked_common_drift_ppm": model.drift_ppm,
            "live_anchor_window_start_monotonic_ns": window_start_ns,
            "live_anchor_window_end_monotonic_ns": now_monotonic_ns,
        }
        return model

    def lock(
        self, *, identity_evidence_verified: bool, now_monotonic_ns: int
    ) -> int:
        raise RuntimeError("affine discipline requires lock_affine()")

    def lock_fixed(
        self,
        approved_offset_ns: int,
        *,
        identity_evidence_verified: bool,
        now_monotonic_ns: int,
    ) -> int:
        raise RuntimeError("affine discipline cannot lock a fixed offset")

    def observe(
        self,
        stream: str,
        source_stamp_ns: int,
        receipt_realtime_ns: int,
        receipt_monotonic_ns: int,
        *,
        clock_domain: str,
        clock_read_span_ns: int = 0,
    ) -> CorrectionResult:
        if self.state == DisciplineState.FAULTED:
            return CorrectionResult(
                False,
                None,
                False,
                self.fault_reason or "discipline faulted",
                self.state,
            )
        if stream not in self._streams:
            return self._fault(f"unexpected_stream:{stream}")
        if clock_domain != self.config.expected_clock_domain:
            return self._fault(f"clock_domain_mismatch:{stream}")
        if not all(
            _positive_int(value)
            for value in (source_stamp_ns, receipt_realtime_ns, receipt_monotonic_ns)
        ):
            return self._fault(f"invalid_timestamp_encoding:{stream}")
        if (
            not isinstance(clock_read_span_ns, int)
            or isinstance(clock_read_span_ns, bool)
            or not 0 <= clock_read_span_ns <= self.config.max_clock_read_span_ns
        ):
            return self._fault(f"clock_read_span_exceeded:{stream}")

        clock_base_ns = receipt_realtime_ns - receipt_monotonic_ns
        if self._last_clock_base_ns is not None and abs(
            clock_base_ns - self._last_clock_base_ns
        ) > self.config.max_clock_pair_discontinuity_ns:
            return self._fault("local_realtime_discontinuity")
        self._last_clock_base_ns = clock_base_ns
        if (
            self.state == DisciplineState.QUALIFYING
            and self._restart_qualification_on_core_delta_discontinuity
            and self._affine_qualification_start_monotonic_ns is not None
        ):
            receipt_gaps: list[int] = []
            for name, candidate in self._streams.items():
                last_receipt_ns = (
                    candidate.last_advancing_receipt_monotonic_ns
                )
                if last_receipt_ns is None:
                    continue
                gap_ns = receipt_monotonic_ns - last_receipt_ns
                if gap_ns > self._policies[name].stale_receipt_timeout_ns:
                    receipt_gaps.append(gap_ns)
            if receipt_gaps:
                return self._restart_qualification_after_core_delivery_pause(
                    stream=stream,
                    source_stamp_ns=source_stamp_ns,
                    receipt_realtime_ns=receipt_realtime_ns,
                    receipt_monotonic_ns=receipt_monotonic_ns,
                    diagnostics=(
                        ("stale_stream_count", len(receipt_gaps)),
                        ("largest_receipt_gap_ns", max(receipt_gaps)),
                    ),
                )
        if (
            self.state == DisciplineState.LOCKED
            and self._allow_locked_receipt_recovery
            and not self._receipt_recovery_active
        ):
            # Normally the 50 ms timer enters recovery before subscriptions
            # are serviced.  Keep the same behavior if executor ordering or a
            # direct caller delivers one callback first.
            liveness_reason = self._liveness_health(receipt_monotonic_ns)
            if liveness_reason is not None:
                self._enter_locked_receipt_recovery(liveness_reason)

        state = self._streams[stream]
        continuity = self._observed_continuity[stream]
        if continuity is not None:
            continuity_source_ns, continuity_receipt_monotonic_ns = continuity
            if source_stamp_ns < continuity_source_ns:
                return self._fault(f"source_timestamp_regression:{stream}")
            if source_stamp_ns == continuity_source_ns:
                # A duplicate of a discarded observation is also discarded;
                # neither may contaminate last-good duplicate statistics.
                if self._observed_continuity_is_last_good[stream]:
                    state.duplicates += 1
                return CorrectionResult(
                    False,
                    None,
                    False,
                    f"duplicate_source_timestamp:{stream}",
                    self.state,
                )
            source_delta_ns = source_stamp_ns - continuity_source_ns
            receipt_delta_ns = receipt_monotonic_ns - continuity_receipt_monotonic_ns
            if receipt_delta_ns <= 0:
                return self._fault(f"receipt_monotonic_not_advancing:{stream}")
            policy = self._policies[stream]
            delta_limit_ns = (
                policy.max_source_receipt_delta_error_ns
                if policy.max_source_receipt_delta_error_ns is not None
                else self.config.max_source_receipt_delta_error_ns
            )
            delta_error_ns = abs(source_delta_ns - receipt_delta_ns)
            if delta_error_ns > delta_limit_ns:
                diagnostics = (
                    ("source_delta_ns", source_delta_ns),
                    ("receipt_delta_ns", receipt_delta_ns),
                    ("delta_error_ns", delta_error_ns),
                    ("delta_error_limit_ns", delta_limit_ns),
                )
                if (
                    self.state == DisciplineState.QUALIFYING
                    and self._restart_qualification_on_core_delta_discontinuity
                    and stream in self.core_streams
                ):
                    return self._restart_qualification_after_core_delivery_pause(
                        stream=stream,
                        source_stamp_ns=source_stamp_ns,
                        receipt_realtime_ns=receipt_realtime_ns,
                        receipt_monotonic_ns=receipt_monotonic_ns,
                        diagnostics=diagnostics,
                    )
                if (
                    self.state == DisciplineState.QUALIFYING
                    and stream
                    in self._qualifying_noncore_delta_drop_streams
                    and delta_error_ns
                    <= (
                        WORKSTATION_NOMOTION_QUALIFYING_NONCORE_DELTA_ERROR_CEILING_NS
                    )
                    and source_delta_ns
                    <= (
                        WORKSTATION_NOMOTION_QUALIFYING_NONCORE_PERIOD_CEILING_NS
                    )
                    and receipt_delta_ns
                    <= (
                        WORKSTATION_NOMOTION_QUALIFYING_NONCORE_PERIOD_CEILING_NS
                    )
                ):
                    # This bounded cloud callback advances only structural
                    # continuity.  It cannot enter qualification statistics,
                    # refresh last-good liveness, or produce corrected output.
                    # A later normal callback is evaluated from this baseline;
                    # any over-ceiling period/error remains a hard fault.
                    self._observed_continuity[stream] = (
                        source_stamp_ns,
                        receipt_monotonic_ns,
                    )
                    self._observed_continuity_is_last_good[stream] = False
                    return CorrectionResult(
                        False,
                        None,
                        False,
                        f"{NONCORE_DELTA_DROP_REASON_PREFIX}:{stream}",
                        self.state,
                        diagnostics,
                    )
                if (
                    self.state == DisciplineState.LOCKED
                    and self._receipt_recovery_active
                    and self._allow_locked_receipt_recovery
                ):
                    # After an executor/Wi-Fi pause, KEEP_LAST(1) may first
                    # expose one transition callback whose source stamp moved
                    # much less than local receipt time (754 ms receipt versus
                    # 114 ms source was reproduced on hardware).  Advance only
                    # the structurally valid observed-continuity baseline.
                    # It cannot refresh liveness/statistics/cache or produce a
                    # corrected output; two later normal advancing samples are
                    # still required on every stream.  Regression, duplicate,
                    # clock-domain and local-clock checks above remain hard.
                    self._observed_continuity[stream] = (
                        source_stamp_ns,
                        receipt_monotonic_ns,
                    )
                    self._observed_continuity_is_last_good[stream] = False
                    return self._receipt_recovery_result(
                        f"{LOCKED_RECEIPT_RECOVERY_DELTA_DROP_REASON_PREFIX}:"
                        f"{stream}",
                        diagnostics=diagnostics,
                    )
                if (
                    self.state == DisciplineState.LOCKED
                    and self._allow_locked_receipt_recovery
                    and stream in self._locked_core_delta_recovery_streams
                    and delta_error_ns <= policy.hard_age_ceiling_ns
                ):
                    # A bounded one-callback scheduling wobble in the explicit
                    # no-motion profile must not tear down every private sensor
                    # relay.  The suspect sample is not retained, cannot refresh
                    # liveness/statistics, and cannot produce output.  Advance
                    # only the structural continuity baseline so a catch-up
                    # callback can also be discarded by the already-active
                    # recovery path.  Output resumes only after two later fresh
                    # samples from every stream and the unchanged locked
                    # statistical health check; the frozen model is never
                    # adapted or relocked.  Motion config enables no streams.
                    recovery_result = self._enter_locked_receipt_recovery(
                        f"{LOCKED_RECEIPT_RECOVERY_DELTA_DROP_REASON_PREFIX}:"
                        f"{stream}",
                        diagnostics=diagnostics,
                    )
                    if recovery_result.state != DisciplineState.FAULTED:
                        self._observed_continuity[stream] = (
                            source_stamp_ns,
                            receipt_monotonic_ns,
                        )
                        self._observed_continuity_is_last_good[stream] = False
                    return recovery_result
                if (
                    self.state == DisciplineState.LOCKED
                    and stream in self._locked_noncore_delta_drop_streams
                ):
                    health_reason = self._locked_noncore_drop_health(
                        stream, receipt_monotonic_ns
                    )
                    if health_reason is not None:
                        return self._fault(health_reason)
                    # Advance only the observed continuity baseline.  The
                    # discarded callback cannot refresh last-good liveness,
                    # enter timing statistics, disturb the locked cache, or
                    # produce a corrected output.  A following normal callback
                    # can recover relative to this observation; continued
                    # drops still hit the unchanged last-good liveness timer.
                    self._observed_continuity[stream] = (
                        source_stamp_ns,
                        receipt_monotonic_ns,
                    )
                    self._observed_continuity_is_last_good[stream] = False
                    return CorrectionResult(
                        False,
                        None,
                        False,
                        f"{NONCORE_DELTA_DROP_REASON_PREFIX}:{stream}",
                        self.state,
                        diagnostics,
                    )
                return self._fault(
                    f"source_receipt_delta_discontinuity:{stream}",
                    diagnostics=diagnostics,
                )

        corrected_stamp_ns: int | None = None
        corrected_age_ns: int | None = None
        corrected_ages: deque[int] | None = None
        if self.state == DisciplineState.LOCKED:
            model = self.locked_affine_model
            if model is None:
                return self._fault("affine_model_missing_after_lock")
            corrected_age_windows = self._locked_corrected_ages
            if (
                corrected_age_windows is None
                or stream not in corrected_age_windows
            ):
                return self._fault(
                    "affine_corrected_age_cache_missing_after_lock"
                )
            corrected_ages = corrected_age_windows[stream]
            if (
                corrected_ages.maxlen != state.samples.maxlen
                or len(corrected_ages) != len(state.samples)
            ):
                return self._fault(
                    f"affine_corrected_age_cache_misaligned:{stream}"
                )
            try:
                corrected_stamp_ns = model.corrected_stamp_ns(source_stamp_ns)
            except ValueError:
                return self._fault(f"affine_correction_out_of_range:{stream}")
            corrected_age_ns = receipt_realtime_ns - corrected_stamp_ns
            policy = self._policies[stream]
            if corrected_age_ns < -self.config.max_corrected_future_ns:
                return self._fault(
                    f"corrected_timestamp_in_future:{stream}",
                    diagnostics=(
                        ("corrected_age_ns", corrected_age_ns),
                        (
                            "max_corrected_future_ns",
                            self.config.max_corrected_future_ns,
                        ),
                    ),
                )
            minimum_age_ns = self.config.minimum_locked_corrected_age_ns
            if (
                minimum_age_ns is not None
                and corrected_age_ns < minimum_age_ns
            ):
                return self._fault(
                    f"affine_corrected_age_margin_exhausted:{stream}",
                    diagnostics=(
                        ("corrected_age_ns", corrected_age_ns),
                        ("minimum_locked_corrected_age_ns", minimum_age_ns),
                    ),
                )
            if corrected_age_ns > policy.max_corrected_age_ns:
                # A queued sample that is structurally valid but already old
                # is not a process fault.  Drop it before it can enter the
                # last-good receipt clock, samples, statistics, or corrected
                # age cache.  Advancing only structural continuity lets the
                # next current sample recover without a false delta jump.
                #
                # Repeated old samples cannot keep this process alive: because
                # they never refresh last-good receipt liveness, the existing
                # poll watchdog still fails closed at the shared 200 ms state
                # boundary.  This removes a duplicate sticky startup gate
                # without weakening the actual motion watchdog.
                liveness_reason = self._liveness_health(
                    receipt_monotonic_ns
                )
                if liveness_reason is not None:
                    return self._enter_locked_receipt_recovery(
                        liveness_reason
                    )
                self._observed_continuity[stream] = (
                    source_stamp_ns,
                    receipt_monotonic_ns,
                )
                self._observed_continuity_is_last_good[stream] = False
                return CorrectionResult(
                    False,
                    None,
                    False,
                    f"{LOCKED_STALE_SAMPLE_DROP_REASON_PREFIX}:{stream}",
                    self.state,
                    (
                        ("corrected_age_ns", corrected_age_ns),
                        (
                            "max_corrected_age_ns",
                            policy.max_corrected_age_ns,
                        ),
                    ),
                )

        previous_sample_count = len(state.samples)
        age_ns = receipt_realtime_ns - source_stamp_ns
        self._observed_continuity[stream] = (
            source_stamp_ns,
            receipt_monotonic_ns,
        )
        self._observed_continuity_is_last_good[stream] = True
        state.samples.append((source_stamp_ns, receipt_monotonic_ns, age_ns))
        state.last_source_ns = source_stamp_ns
        state.last_advancing_receipt_monotonic_ns = receipt_monotonic_ns
        state.advancing += 1

        if (
            self.state == DisciplineState.QUALIFYING
            and self._affine_qualification_start_monotonic_ns is None
            and all(candidate.samples for candidate in self._streams.values())
        ):
            # Start the fixed window only after every required subscription has
            # delivered.  ROS discovery can expose SportModeState seconds before
            # MID-360 IMU/odometry; counting that startup skew as missing clock
            # evidence made otherwise healthy restarts fail nondeterministically.
            self._affine_qualification_start_monotonic_ns = max(
                candidate.samples[0][1]
                for candidate in self._streams.values()
            )

        if self.state != DisciplineState.LOCKED:
            return CorrectionResult(False, None, False, "affine_not_locked", self.state)
        assert corrected_stamp_ns is not None
        assert corrected_age_ns is not None
        assert corrected_ages is not None
        cache_reason: str | None = None
        if len(corrected_ages) != previous_sample_count:
            cache_reason = f"affine_corrected_age_cache_misaligned:{stream}"
        else:
            corrected_ages.append(corrected_age_ns)
            if len(corrected_ages) != len(state.samples):
                cache_reason = (
                    f"affine_corrected_age_cache_misaligned:{stream}"
                )
        if cache_reason is not None:
            return self._fault(cache_reason)
        if self._receipt_recovery_active:
            self._record_locked_receipt_recovery_sample(
                stream, receipt_monotonic_ns
            )
            recovery_result = self._locked_receipt_recovery_health(
                receipt_monotonic_ns
            )
            if recovery_result is not None:
                return recovery_result
        health_reason = self._liveness_health(receipt_monotonic_ns)
        if health_reason is not None:
            return self._enter_locked_receipt_recovery(health_reason)
        health_reason = self._scheduled_statistics_health(receipt_monotonic_ns)
        if health_reason is not None:
            if health_reason.startswith(
                "locked_affine_common_window_incomplete:"
            ):
                return self._enter_locked_receipt_recovery(health_reason)
            return self._fault(health_reason)
        return CorrectionResult(
            True, corrected_stamp_ns, True, "navigation_safe_affine", self.state
        )

    def poll(self, now_monotonic_ns: int) -> CorrectionResult:
        if self.state == DisciplineState.FAULTED:
            return CorrectionResult(
                False,
                None,
                False,
                self.fault_reason or "discipline faulted",
                self.state,
            )
        if self.state != DisciplineState.LOCKED:
            return CorrectionResult(False, None, False, "affine_not_locked", self.state)
        if not _positive_int(now_monotonic_ns):
            return self._fault("invalid_poll_time")
        if self._receipt_recovery_active:
            recovery_result = self._locked_receipt_recovery_health(
                now_monotonic_ns
            )
            if recovery_result is not None:
                return recovery_result
            return CorrectionResult(
                False,
                None,
                False,
                "locked_receipt_recovered",
                self.state,
            )
        reason = self._liveness_health(now_monotonic_ns)
        if reason is not None:
            return self._enter_locked_receipt_recovery(reason)
        reason = self._scheduled_statistics_health(now_monotonic_ns)
        if reason is not None:
            if reason.startswith("locked_affine_common_window_incomplete:"):
                return self._enter_locked_receipt_recovery(reason)
            return self._fault(reason)
        return CorrectionResult(False, None, False, "healthy_no_sample", self.state)

    def _statistical_health(self) -> str | None:
        model = self.locked_affine_model
        if model is None:
            return "affine_model_missing_after_lock"
        # Locked health needs only presence and the cumulative duplicate
        # ratio from generic metrics.  Building every quantile plus a second
        # robust-drift estimate here stalls the single-threaded relay without
        # contributing to any decision below.
        for name, policy in self._policies.items():
            state = self._streams[name]
            if not state.samples:
                return f"stream_lost:{name}"
            total = state.advancing + state.duplicates
            duplicate_fraction = state.duplicates / total if total else 0.0
            if duplicate_fraction > policy.max_duplicate_fraction:
                return f"duplicate_fraction_exceeded:{name}"

        # PointCloud2 is a non-core consumer of the frozen affine model.  Its
        # acquisition lag is checked below, but it must never trigger a second
        # expensive clock-rate fit.  Core rates are fitted over one identical
        # recent monotonic interval ending at the last boundary available in
        # every core stream.  A 500 Hz deque and a 150 Hz deque can therefore
        # no longer compare 80 seconds against 267 seconds after saturation.
        (
            drift_window_start_ns,
            drift_window_end_ns,
            drift_window_samples,
            drift_stream_diagnostics,
        ) = self._locked_affine_drift_window()
        affine_metrics = self._affine_metrics_for(
            self.core_streams,
            samples_by_stream=drift_window_samples,
        )
        observed_core_drifts: list[tuple[str, float | None]] = []
        for name in self.core_streams:
            drift = affine_metrics[name].drift_ppm
            observed_core_drifts.append((name, drift))
        finite_core_drifts = [
            (name, drift)
            for name, drift in observed_core_drifts
            if drift is not None
        ]
        common_drift = (
            float(statistics.median(value for _, value in finite_core_drifts))
            if len(finite_core_drifts) == len(self.core_streams)
            else None
        )
        common_deviation_ppm = (
            None
            if common_drift is None
            else abs(common_drift - model.drift_ppm)
        )
        previous_diagnostics = self._last_affine_drift_diagnostics
        if (
            previous_diagnostics is not None
            and previous_diagnostics.get("comparison_phase") == "locked_health"
        ):
            previous_core_drifts = dict(
                previous_diagnostics.get("current_core_stream_drifts_ppm", {})
            )
            previous_common_drift = previous_diagnostics.get(
                "current_common_drift_ppm"
            )
        else:
            previous_core_drifts = dict(model.core_stream_drifts_ppm)
            previous_common_drift = model.drift_ppm
        for name in self.core_streams:
            details = drift_stream_diagnostics[name]
            details["envelope_points"] = affine_metrics[name].envelope_points
            details["previous_drift_ppm"] = previous_core_drifts.get(name)
            details["current_drift_ppm"] = affine_metrics[name].drift_ppm
        lock_monotonic_ns = self._affine_lock_monotonic_ns
        locked_elapsed_ns = (
            None
            if lock_monotonic_ns is None
            else max(0, drift_window_end_ns - lock_monotonic_ns)
        )
        lock_clock_base_ns = self._affine_lock_clock_base_ns
        current_clock_base_ns = self._last_clock_base_ns
        self._last_affine_drift_diagnostics = {
            "comparison_phase": "locked_health",
            "clock_basis": "local_monotonic_receipt_ns",
            "bin_anchor_monotonic_ns": (
                self._affine_qualification_start_monotonic_ns or 0
            ),
            "drift_window_requested_ns": LOCKED_AFFINE_DRIFT_WINDOW_NS,
            "drift_window_start_monotonic_ns": drift_window_start_ns,
            "drift_window_end_monotonic_ns": drift_window_end_ns,
            "drift_window_span_ns": (
                drift_window_end_ns - drift_window_start_ns
            ),
            "locked_monotonic_ns": lock_monotonic_ns,
            "locked_elapsed_ns": locked_elapsed_ns,
            "locked_clock_base_ns": lock_clock_base_ns,
            "current_clock_base_ns": current_clock_base_ns,
            "clock_base_delta_ns": (
                None
                if lock_clock_base_ns is None or current_clock_base_ns is None
                else current_clock_base_ns - lock_clock_base_ns
            ),
            "locked_core_stream_drifts_ppm": dict(
                model.core_stream_drifts_ppm
            ),
            "locked_common_drift_ppm": model.drift_ppm,
            "previous_core_stream_drifts_ppm": previous_core_drifts,
            "previous_common_drift_ppm": previous_common_drift,
            "current_core_stream_drifts_ppm": dict(observed_core_drifts),
            "current_common_drift_ppm": common_drift,
            "common_drift_deviation_ppm": common_deviation_ppm,
            "common_drift_deviation_limit_ppm": (
                self.config.max_locked_affine_drift_deviation_ppm
            ),
            "streams": drift_stream_diagnostics,
        }
        window_span_ns = drift_window_end_ns - drift_window_start_ns
        required_envelope_points = max(
            4, window_span_ns // self.config.drift_bin_ns
        )
        for name in self.core_streams:
            details = drift_stream_diagnostics[name]
            coverage_limit_ns = self._policies[name].stale_receipt_timeout_ns
            recorded_gap = self._receipt_recovery_gap_waivers.get(name)
            if (
                recorded_gap is not None
                and recorded_gap[1] < drift_window_start_ns
            ):
                # The exact outage pair has rolled out of the common window;
                # never let its numeric duration waive a later gap.
                self._receipt_recovery_gap_waivers.pop(name, None)
                recorded_gap = None
            maximum_gap = (
                details["max_receipt_gap_start_monotonic_ns"],
                details["max_receipt_gap_end_monotonic_ns"],
                details["max_receipt_gap_ns"],
            )
            maximum_gap_is_allowed = (
                details["max_receipt_gap_ns"] is not None
                and (
                    details["max_receipt_gap_ns"] <= coverage_limit_ns
                    or (
                        recorded_gap is not None
                        and maximum_gap == recorded_gap
                    )
                )
            )
            first_receipt_offset_ns = details["first_receipt_offset_ns"]
            recorded_gap_covers_window_start = (
                recorded_gap is not None
                and recorded_gap[0] < drift_window_start_ns
                <= recorded_gap[1]
                and first_receipt_offset_ns is not None
                and (
                    drift_window_start_ns + first_receipt_offset_ns
                    == recorded_gap[1]
                )
            )
            first_edge_is_allowed = (
                first_receipt_offset_ns is not None
                and (
                    first_receipt_offset_ns <= coverage_limit_ns
                    or recorded_gap_covers_window_start
                )
            )
            details["window_start_covered_by_recorded_gap"] = (
                recorded_gap_covers_window_start
            )
            if (
                details["window_samples"] is None
                or details["window_samples"] < 4
                or details["envelope_points"] is None
                or details["envelope_points"] < required_envelope_points
                or not first_edge_is_allowed
                or details["last_receipt_offset_ns"] is None
                or details["last_receipt_offset_ns"] > coverage_limit_ns
                or details["max_receipt_gap_ns"] is None
                or not maximum_gap_is_allowed
            ):
                return f"locked_affine_common_window_incomplete:{name}"
        for name, drift in observed_core_drifts:
            if drift is None or abs(drift) > self.config.max_absolute_drift_ppm:
                return f"clock_drift_exceeded:{name}"
        current_core_drifts = [
            (name, drift)
            for name, drift in observed_core_drifts
            if drift is not None
        ]
        assert common_deviation_ppm is not None
        if common_deviation_ppm > (
            self.config.max_locked_affine_drift_deviation_ppm
        ):
            return "affine_model_drift_deviation_exceeded"
        pairwise_disagreements = _pairwise_drift_disagreements(
            current_core_drifts, self.config.max_pairwise_drift_ppm
        )
        if pairwise_disagreements:
            return f"pairwise_drift_disagreement:{pairwise_disagreements[0]}"

        baseline_ages = dict(model.stream_baseline_corrected_age_ns)
        reference_baseline = baseline_ages[self.config.reference_stream]
        current_lower_ages: dict[str, int] = {}
        corrected_age_windows = self._locked_corrected_ages
        if corrected_age_windows is None:
            return "affine_corrected_age_cache_missing_after_lock"
        for name, state in self._streams.items():
            corrected_ages = corrected_age_windows.get(name)
            if corrected_ages is None:
                return "affine_corrected_age_cache_missing_after_lock"
            if (
                corrected_ages.maxlen != state.samples.maxlen
                or len(corrected_ages) != len(state.samples)
            ):
                return f"affine_corrected_age_cache_misaligned:{name}"
            current_lower = _quantile(
                corrected_ages, self.config.lower_quantile
            )
            current_lower_ages[name] = current_lower
            if abs(current_lower - baseline_ages[name]) > (
                self.config.max_locked_offset_deviation_ns
            ):
                return f"affine_model_residual_exceeded:{name}"

        reference_current = current_lower_ages[self.config.reference_stream]
        for name, policy in self._policies.items():
            baseline_relative = baseline_ages[name] - reference_baseline
            current_relative = current_lower_ages[name] - reference_current
            if abs(current_relative - baseline_relative) > (
                self.config.max_locked_offset_deviation_ns
            ):
                return f"stream_relative_lag_changed:{name}"
            if current_relative < -self.config.max_relative_lead_ns:
                return f"stream_leads_reference:{name}"
            if current_relative > policy.max_relative_lag_ns:
                return f"stream_lag_exceeded:{name}"
        return None
