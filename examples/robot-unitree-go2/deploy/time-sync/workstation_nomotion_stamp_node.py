#!/usr/bin/env python3
"""ROS 2 timestamp prelayer for the workstation full no-motion profile.

The legacy ``fixed`` mode transforms source stamps with one immutable approved
``local - source`` offset.  The explicit ``affine`` mode qualifies and freezes
one common source-to-local clock-rate model across SportModeState, MID-360 IMU,
and MID-360 odometry, then applies it to every stream.  Receipt time is
observation evidence and is never written into a message.  All outputs are
private raw topics; this node does not publish canonical odometry/TF, create
command interfaces, or call Unitree APIs.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
from typing import Any, Callable, NamedTuple


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from navigation_stamp_discipline import (  # noqa: E402
    AffineClockModel,
    AffineNavigationStampDiscipline,
    CorrectionResult,
    DisciplineState,
    NavigationStampDiscipline,
    NONCORE_DELTA_DROP_REASON_PREFIX,
    WORKSTATION_NOMOTION_QUALIFYING_NONCORE_DELTA_ERROR_CEILING_NS,
    WORKSTATION_NOMOTION_QUALIFYING_NONCORE_PERIOD_CEILING_NS,
    go2_workstation_motion_config,
    go2_workstation_nomotion_config,
)
from workstation_nomotion_approval import (  # noqa: E402
    ApprovalError,
    EXPECTED_RAW_TOPICS,
    FixedOffsetApproval,
    load_approval,
    require_strict_affine_approval,
)


RAW_TOPICS = dict(EXPECTED_RAW_TOPICS)
CORRECTED_TOPICS = {
    "sport_primary": "/robonix/time_corrected/raw/sportmodestate",
    "mid360_imu": "/robonix/time_corrected/raw/utlidar/imu",
    "mid360_cloud": "/robonix/time_corrected/raw/utlidar/cloud",
    "mid360_odom": "/robonix/time_corrected/raw/utlidar/robot_odom",
}
READY_SCHEMA = "robonix-go2-workstation-nomotion-stamp-ready-v1"
AFFINE_READY_SCHEMA = "robonix-go2-workstation-nomotion-affine-stamp-ready-v2"
FAULT_SCHEMA = "robonix-go2-workstation-nomotion-stamp-fault-v1"
NONCORE_DROP_SCHEMA = "robonix-go2-workstation-nomotion-noncore-drop-v1"
CORRECTION_MODES = ("fixed", "affine")
DISCIPLINE_PROFILES = ("nomotion", "motion")
AFFINE_CORE_STREAMS = ("sport_primary", "mid360_imu", "mid360_odom")
PHASE_LATENCY_NAMES = (
    "discipline_observe",
    "message_copy",
    "publish",
    "discipline_poll",
)
MOTION_CORRECTED_TOPICS = {
    name: CORRECTED_TOPICS[name] for name in AFFINE_CORE_STREAMS
}
CLOCK_PAIR_HARD_LIMIT_NS = 1_000_000
MOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS = 3
NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS = 3


def _raw_subscription_is_reliable(profile: str, stream: str) -> bool:
    """Select the one evidence-qualified reliable raw sensor subscription."""

    return profile == "nomotion" and stream == "mid360_cloud"


class ReceiptClockPair(NamedTuple):
    """One paired realtime/monotonic sample plus return-time freshness."""

    realtime_ns: int
    monotonic_ns: int
    read_span_ns: int
    selected_pair_age_ns: int


def source_stamp_ns(stamp: Any) -> int:
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if (
        not isinstance(sec, int)
        or isinstance(sec, bool)
        or not isinstance(nanosec, int)
        or isinstance(nanosec, bool)
        or sec < 0
        or not 0 <= nanosec < 1_000_000_000
    ):
        raise ValueError("source stamp is not a valid ROS time")
    value = sec * 1_000_000_000 + nanosec
    if value <= 0 or value >= 2**63:
        raise ValueError("source stamp is outside the supported positive int64 range")
    return value


def corrected_stamp_parts(source_ns: int, fixed_offset_ns: int) -> tuple[int, int]:
    """Apply only the immutable approved offset and return ROS stamp fields."""

    if (
        not isinstance(source_ns, int)
        or isinstance(source_ns, bool)
        or not isinstance(fixed_offset_ns, int)
        or isinstance(fixed_offset_ns, bool)
    ):
        raise ValueError("source stamp and fixed offset must be integers")
    corrected_ns = source_ns + fixed_offset_ns
    if corrected_ns <= 0 or corrected_ns >= 2**63:
        raise ValueError("corrected stamp is outside the positive int64 range")
    return divmod(corrected_ns, 1_000_000_000)


def set_corrected_stamp(stamp: Any, corrected_ns: int) -> None:
    sec, nanosec = corrected_stamp_parts(corrected_ns, 0)
    stamp.sec = sec
    stamp.nanosec = nanosec


def _corrected_nomotion_cloud_copy(message: Any, corrected_ns: int) -> Any:
    """Copy only the cloud envelope/header before latest-only publication.

    PointCloud2 data can be several megabytes.  The isolated no-motion cloud
    relay never mutates that payload, so a second payload allocation would
    only add latency in its independent process.  Keep the payload shared while
    the relay retains the returned message, and copy every object this function
    modifies.  The identity checks fail closed before any original object can
    be changed if a nonstandard message type has broken copy semantics.
    """

    output = copy.copy(message)
    if output is message:
        raise TypeError("cloud message copy did not create a new envelope")
    header = copy.copy(message.header)
    if header is message.header:
        raise TypeError("cloud header copy did not create a new header")
    stamp = copy.copy(message.header.stamp)
    if stamp is message.header.stamp:
        raise TypeError("cloud stamp copy did not create a new stamp")
    set_corrected_stamp(stamp, corrected_ns)
    header.stamp = stamp
    output.header = header
    return output


def corrected_message_copy(message: Any, stream: str, corrected_ns: int) -> Any:
    """Copy one raw message and change only its source/header stamp."""

    if stream not in RAW_TOPICS:
        raise ValueError("unknown corrected timestamp stream")
    if stream == "mid360_cloud":
        return _corrected_nomotion_cloud_copy(message, corrected_ns)
    output = copy.deepcopy(message)
    stamp = output.stamp if stream == "sport_primary" else output.header.stamp
    set_corrected_stamp(stamp, corrected_ns)
    return output


def paired_receipt_clocks(attempts: int = 3) -> ReceiptClockPair:
    """Return the latest bounded-span realtime/monotonic pair with age.

    A process can be preempted between the three clock syscalls even though
    neither clock moved.  Retrying does not widen the accepted span: only the
    last complete pair meeting the checked-in 1 ms ceiling is preferred.  If
    later retries stall, the return guard exposes the retained pair as stale
    instead of silently returning an old, formerly tight sample.
    """

    if (
        not isinstance(attempts, int)
        or isinstance(attempts, bool)
        or not 1 <= attempts <= 8
    ):
        raise ValueError("clock-pair attempts must be an integer from 1 to 8")
    latest: tuple[int, int, int] | None = None
    selected: tuple[int, int, int] | None = None
    for _ in range(attempts):
        before_ns = time.monotonic_ns()
        realtime_ns = time.time_ns()
        after_ns = time.monotonic_ns()
        sample = (
            realtime_ns,
            (before_ns + after_ns) // 2,
            after_ns - before_ns,
        )
        latest = sample
        if 0 <= sample[2] <= CLOCK_PAIR_HARD_LIMIT_NS:
            selected = sample
    assert latest is not None
    if selected is None:
        selected = latest
    return_guard_ns = time.monotonic_ns()
    selected_pair_age_ns = return_guard_ns - selected[1]
    return ReceiptClockPair(
        realtime_ns=selected[0],
        monotonic_ns=selected[1],
        read_span_ns=selected[2],
        selected_pair_age_ns=selected_pair_age_ns,
    )


def fresh_paired_receipt_clocks(
    *,
    max_clock_read_span_ns: int,
    max_pair_age_ns: int = CLOCK_PAIR_HARD_LIMIT_NS,
    attempts: int = 3,
    age_acquisition_attempts: int = MOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS,
) -> ReceiptClockPair:
    """Return a pair only after a second, caller-side freshness guard.

    The extra monotonic read closes the gap between the helper's return guard
    and its use by approval, receipt-liveness, or commit code.  A scheduler
    pause therefore faults before any of those consumers see the old pair.
    Both workstation profiles request a bounded number of complete new
    acquisitions after a scheduler-delayed read span or an otherwise valid
    pair aged beyond the unchanged 1 ms ceilings.  No over-limit pair is ever
    returned: a positive read-span overrun is retried from fresh clock reads,
    while negative spans and monotonic-order failures remain immediate faults.
    """

    if (
        not isinstance(max_clock_read_span_ns, int)
        or isinstance(max_clock_read_span_ns, bool)
        or not 0 < max_clock_read_span_ns <= CLOCK_PAIR_HARD_LIMIT_NS
        or not isinstance(max_pair_age_ns, int)
        or isinstance(max_pair_age_ns, bool)
        or not 0 < max_pair_age_ns <= CLOCK_PAIR_HARD_LIMIT_NS
    ):
        raise ValueError("clock-pair freshness limits must be positive and <= 1 ms")
    if (
        not isinstance(age_acquisition_attempts, int)
        or isinstance(age_acquisition_attempts, bool)
        or not 1
        <= age_acquisition_attempts
        <= NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS
    ):
        raise ValueError(
            "clock-pair age acquisition attempts must be an integer from 1 to 3"
        )
    last_retryable_error = ""
    for _ in range(age_acquisition_attempts):
        pair = paired_receipt_clocks(attempts)
        use_guard_ns = time.monotonic_ns()
        pair_age_ns = use_guard_ns - pair.monotonic_ns
        age_error = (
            "clock_pair_age_exceeded:"
            f"age_ns={pair_age_ns}:return_age_ns={pair.selected_pair_age_ns}:"
            f"limit_ns={max_pair_age_ns}"
        )
        if (
            pair.selected_pair_age_ns < 0
            or pair_age_ns < pair.selected_pair_age_ns
        ):
            raise RuntimeError(age_error)
        read_span_error = (
            "clock_pair_read_span_exceeded:"
            f"span_ns={pair.read_span_ns}:limit_ns={max_clock_read_span_ns}"
        )
        if pair.read_span_ns < 0:
            raise RuntimeError(read_span_error)
        if pair.read_span_ns > max_clock_read_span_ns:
            last_retryable_error = read_span_error
            continue
        if pair_age_ns > max_pair_age_ns:
            last_retryable_error = age_error
            continue
        return ReceiptClockPair(
            realtime_ns=pair.realtime_ns,
            monotonic_ns=pair.monotonic_ns,
            read_span_ns=pair.read_span_ns,
            selected_pair_age_ns=pair_age_ns,
        )
    raise RuntimeError(last_retryable_error)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, sort_keys=True, separators=(",", ":"))
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


class _Runtime:
    def __init__(
        self,
        approval: FixedOffsetApproval,
        ready_file: Path,
        fault_file: Path,
        mode: str = "fixed",
        profile: str = "nomotion",
    ) -> None:
        if mode not in CORRECTION_MODES:
            raise ValueError("unsupported timestamp correction mode")
        if profile not in DISCIPLINE_PROFILES:
            raise ValueError("unsupported timestamp discipline profile")
        if profile == "motion" and mode != "affine":
            raise ValueError("motion discipline profile requires affine correction")
        if mode == "affine":
            # This direct-Python boundary is security/safety relevant too;
            # callers cannot bypass schema-v3 by constructing a legacy object
            # with a manually populated finite drift.
            require_strict_affine_approval(approval)
        approval_drift = getattr(
            approval, "approved_affine_common_drift_ppm", None
        )
        if mode == "affine" and (
            not isinstance(
                approval_drift, (int, float)
            )
            or isinstance(approval_drift, bool)
            or not math.isfinite(approval_drift)
        ):
            raise ValueError("affine mode requires a schema-v3 drift approval")
        self.approval = approval
        self.ready_file = ready_file
        self.fault_file = fault_file
        self.mode = mode
        self.profile = profile
        self.clock_pair_age_acquisition_attempts = (
            MOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS
            if profile == "motion"
            else NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS
        )
        self.streams = (
            tuple(MOTION_CORRECTED_TOPICS)
            if profile == "motion"
            else tuple(CORRECTED_TOPICS)
        )
        config = (
            go2_workstation_motion_config(approval.expected_clock_domain)
            if profile == "motion"
            else go2_workstation_nomotion_config(approval.expected_clock_domain)
        )
        self.discipline = (
            NavigationStampDiscipline(config)
            if mode == "fixed"
            else AffineNavigationStampDiscipline(
                config,
                core_streams=AFFINE_CORE_STREAMS,
                # Both affine profiles pause all corrected output while a
                # transient receipt gap is requalified.  In the motion
                # profile this does not keep the chassis moving: its separate
                # 200 ms state watchdog sees the missing output and stops.
                # Keeping the timestamp process alive lets a short scheduler
                # or Wi-Fi wobble recover without tearing down Robonix.
                allow_locked_receipt_recovery=True,
            )
        )
        self.ready = False
        self.fault_reason: str | None = None
        self.shutdown: Callable[[], None] = lambda: None
        self._last_qualification_attempt_ns: int | None = None
        self._last_qualification_report_ns: int | None = None
        self._affine_terminal_evaluation_started = False
        self.affine_terminal_full_evaluation_count = 0
        self.phase_latency_last_ns = {
            name: 0 for name in PHASE_LATENCY_NAMES
        }
        self.phase_latency_max_ns = {
            name: 0 for name in PHASE_LATENCY_NAMES
        }
        self._phase_latency_lock = threading.Lock()
        self.noncore_delta_drop_count = 0
        self.last_noncore_delta_drop: dict[str, int | str] | None = None
        self.post_evaluation_commit_diagnostics: dict[str, object] | None = None

    def timestamp_safety_limits(self) -> dict[str, int | float | None]:
        """Return the immutable correction limits used by this runtime."""

        config = self.discipline.config
        return {
            "offset_guard_ns": config.offset_guard_ns,
            "affine_anchor_past_guard_ns": (
                config.affine_anchor_past_guard_ns
            ),
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

    def record_phase_latency(self, name: str, elapsed_ns: int) -> None:
        """Retain bounded last/maximum evidence for one audited call phase."""

        if not isinstance(name, str) or name not in PHASE_LATENCY_NAMES:
            raise ValueError("unknown timestamp-node latency phase")
        if (
            not isinstance(elapsed_ns, int)
            or isinstance(elapsed_ns, bool)
            or elapsed_ns < 0
        ):
            raise ValueError("phase latency must be a nonnegative integer")
        with self._phase_latency_lock:
            self.phase_latency_last_ns[name] = elapsed_ns
            if elapsed_ns > self.phase_latency_max_ns[name]:
                self.phase_latency_max_ns[name] = elapsed_ns

    def phase_latency_snapshot(self) -> tuple[dict[str, int], dict[str, int]]:
        """Return one internally consistent latency snapshot across threads."""

        with self._phase_latency_lock:
            return (
                dict(self.phase_latency_last_ns),
                dict(self.phase_latency_max_ns),
            )

    def record_noncore_delta_drop(
        self,
        stream: str,
        result: CorrectionResult,
        receipt_monotonic_ns: int,
    ) -> None:
        """Retain one bounded warning record for a discarded no-motion cloud."""

        expected_reason = f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud"
        if (
            self.mode != "affine"
            or self.profile != "nomotion"
            or stream != "mid360_cloud"
            or result.accepted
            or result.corrected_stamp_ns is not None
            or result.navigation_eligible
            or result.state
            not in (DisciplineState.QUALIFYING, DisciplineState.LOCKED)
            or result.reason != expected_reason
        ):
            raise ValueError("invalid non-core timestamp delta-drop result")
        if (
            not isinstance(receipt_monotonic_ns, int)
            or isinstance(receipt_monotonic_ns, bool)
            or receipt_monotonic_ns <= 0
        ):
            raise ValueError("non-core drop receipt time must be a positive integer")
        expected_diagnostics = {
            "source_delta_ns",
            "receipt_delta_ns",
            "delta_error_ns",
            "delta_error_limit_ns",
        }
        diagnostic_values = dict(result.diagnostics)
        if (
            len(result.diagnostics) != len(expected_diagnostics)
            or set(diagnostic_values) != expected_diagnostics
            or any(
                not isinstance(value, int)
                or isinstance(value, bool)
                or value < 0
                for value in diagnostic_values.values()
            )
        ):
            raise ValueError("invalid non-core timestamp delta-drop diagnostics")
        source_delta_ns = diagnostic_values["source_delta_ns"]
        receipt_delta_ns = diagnostic_values["receipt_delta_ns"]
        delta_error_ns = diagnostic_values["delta_error_ns"]
        delta_error_limit_ns = diagnostic_values["delta_error_limit_ns"]
        if (
            source_delta_ns <= 0
            or receipt_delta_ns <= 0
            or delta_error_limit_ns <= 0
            or delta_error_ns != abs(source_delta_ns - receipt_delta_ns)
            or delta_error_ns <= delta_error_limit_ns
        ):
            raise ValueError("inconsistent non-core timestamp delta-drop diagnostics")
        if result.state == DisciplineState.QUALIFYING and (
            delta_error_ns
            > WORKSTATION_NOMOTION_QUALIFYING_NONCORE_DELTA_ERROR_CEILING_NS
            or source_delta_ns
            > WORKSTATION_NOMOTION_QUALIFYING_NONCORE_PERIOD_CEILING_NS
            or receipt_delta_ns
            > WORKSTATION_NOMOTION_QUALIFYING_NONCORE_PERIOD_CEILING_NS
        ):
            raise ValueError(
                "qualifying non-core timestamp delta drop exceeds ceilings"
            )
        self.noncore_delta_drop_count += 1
        self.last_noncore_delta_drop = {
            "stream": stream,
            "reason": result.reason,
            "receipt_monotonic_ns": receipt_monotonic_ns,
            **diagnostic_values,
        }
        warning = {
            "schema": NONCORE_DROP_SCHEMA,
            "count": self.noncore_delta_drop_count,
            **self.last_noncore_delta_drop,
        }
        print(
            "WARNING discarded non-core timestamp sample: "
            + json.dumps(warning, sort_keys=True, separators=(",", ":")),
            flush=True,
        )

    def latch_fault(
        self,
        reason: str,
        *,
        fault_monotonic_ns: int | None = None,
        diagnostics: tuple[tuple[str, int], ...] = (),
        qualification_fault: dict[str, object] | None = None,
    ) -> None:
        if self.fault_reason is not None:
            return
        self.fault_reason = str(reason)[:240] or "unspecified_timestamp_fault"
        phase_latency_last, phase_latency_max = self.phase_latency_snapshot()
        payload: dict[str, Any] = {
            "schema": FAULT_SCHEMA,
            "session_id": self.approval.session_id,
            "reason": self.fault_reason,
            "motion_ready": False,
            "canonical_odom_ready": False,
            "phase_latency_last_ns": phase_latency_last,
            "phase_latency_max_ns": phase_latency_max,
            "timestamp_safety_limits": self.timestamp_safety_limits(),
            "noncore_delta_drop_count": self.noncore_delta_drop_count,
            "last_noncore_delta_drop": (
                None
                if self.last_noncore_delta_drop is None
                else dict(self.last_noncore_delta_drop)
            ),
        }
        if self.mode == "affine":
            payload["approval_schema"] = getattr(
                self.approval, "schema", "unknown"
            )
            payload["approved_affine_common_drift_ppm"] = (
                self.approval.approved_affine_common_drift_ppm
            )
            if isinstance(
                self.discipline, AffineNavigationStampDiscipline
            ):
                drift_diagnostics = (
                    self.discipline.affine_drift_diagnostics()
                )
                if drift_diagnostics is not None:
                    payload["affine_drift_diagnostics"] = drift_diagnostics
            if qualification_fault is not None:
                payload["qualification_fault"] = qualification_fault
            if self.post_evaluation_commit_diagnostics is not None:
                payload["post_evaluation_commit"] = (
                    self.post_evaluation_commit_diagnostics
                )
        try:
            if (
                self.fault_reason.startswith("stream_stale:")
                and isinstance(fault_monotonic_ns, int)
                and not isinstance(fault_monotonic_ns, bool)
                and fault_monotonic_ns > 0
            ):
                receipt_liveness = self.discipline.receipt_liveness_snapshot(
                    fault_monotonic_ns
                )
                trigger_stream = self.fault_reason.split(":", 2)[1]
                trigger = receipt_liveness.get(trigger_stream)
                if trigger is not None:
                    payload.update(
                        {
                            "fault_monotonic_ns": fault_monotonic_ns,
                            "trigger_stream": trigger_stream,
                            "trigger_receipt_age_ns": trigger["receipt_age_ns"],
                            "trigger_receipt_age_limit_ns": trigger[
                                "stale_receipt_timeout_ns"
                            ],
                            "stream_receipt_liveness": receipt_liveness,
                        }
                    )
            elif self.fault_reason.startswith("corrected_timestamp_stale:"):
                corrected_ages = self.discipline.corrected_age_snapshot()
                trigger_stream = self.fault_reason.split(":", 2)[1]
                trigger = corrected_ages.get(trigger_stream)
                if trigger is not None:
                    if (
                        isinstance(fault_monotonic_ns, int)
                        and not isinstance(fault_monotonic_ns, bool)
                        and fault_monotonic_ns > 0
                    ):
                        payload["fault_monotonic_ns"] = fault_monotonic_ns
                    payload.update(
                        {
                            "trigger_stream": trigger_stream,
                            "trigger_corrected_age_ns": trigger[
                                "corrected_age_ns"
                            ],
                            "trigger_corrected_age_limit_ns": trigger[
                                "max_corrected_age_ns"
                            ],
                            "stream_corrected_ages": corrected_ages,
                        }
                    )
            elif self.fault_reason.startswith("corrected_timestamp_in_future:"):
                corrected_ages = self.discipline.corrected_age_snapshot()
                trigger_stream = self.fault_reason.split(":", 2)[1]
                trigger = corrected_ages.get(trigger_stream)
                if trigger is not None:
                    diagnostic_values = dict(diagnostics)
                    trigger_age_ns = diagnostic_values.get(
                        "corrected_age_ns", trigger["corrected_age_ns"]
                    )
                    if (
                        isinstance(trigger_age_ns, int)
                        and not isinstance(trigger_age_ns, bool)
                    ):
                        trigger = dict(trigger)
                        trigger["corrected_age_ns"] = trigger_age_ns
                        trigger["within_bounds"] = False
                        corrected_ages[trigger_stream] = trigger
                    if (
                        isinstance(fault_monotonic_ns, int)
                        and not isinstance(fault_monotonic_ns, bool)
                        and fault_monotonic_ns > 0
                    ):
                        payload["fault_monotonic_ns"] = fault_monotonic_ns
                    payload.update(
                        {
                            "trigger_stream": trigger_stream,
                            "trigger_corrected_age_ns": trigger[
                                "corrected_age_ns"
                            ],
                            "trigger_corrected_future_limit_ns": (
                                self.discipline.config.max_corrected_future_ns
                            ),
                            "stream_corrected_ages": corrected_ages,
                        }
                    )
            elif self.fault_reason.startswith(
                "affine_corrected_age_margin_exhausted:"
            ):
                corrected_ages = self.discipline.corrected_age_snapshot()
                trigger_stream = self.fault_reason.split(":", 1)[1]
                trigger = corrected_ages.get(trigger_stream)
                minimum_age_ns = (
                    self.discipline.config.minimum_locked_corrected_age_ns
                )
                if trigger is not None and minimum_age_ns is not None:
                    diagnostic_values = dict(diagnostics)
                    trigger_age_ns = diagnostic_values.get(
                        "corrected_age_ns", trigger["corrected_age_ns"]
                    )
                    if (
                        isinstance(trigger_age_ns, int)
                        and not isinstance(trigger_age_ns, bool)
                    ):
                        trigger = dict(trigger)
                        trigger["corrected_age_ns"] = trigger_age_ns
                        trigger["within_bounds"] = False
                        corrected_ages[trigger_stream] = trigger
                    if (
                        isinstance(fault_monotonic_ns, int)
                        and not isinstance(fault_monotonic_ns, bool)
                        and fault_monotonic_ns > 0
                    ):
                        payload["fault_monotonic_ns"] = fault_monotonic_ns
                    payload.update(
                        {
                            "trigger_stream": trigger_stream,
                            "trigger_corrected_age_ns": trigger[
                                "corrected_age_ns"
                            ],
                            "trigger_minimum_locked_corrected_age_ns": (
                                minimum_age_ns
                            ),
                            "trigger_corrected_future_limit_ns": (
                                self.discipline.config.max_corrected_future_ns
                            ),
                            "stream_corrected_ages": corrected_ages,
                        }
                    )
            elif self.fault_reason.startswith(
                "source_receipt_delta_discontinuity:"
            ):
                trigger_stream = self.fault_reason.split(":", 2)[1]
                expected_keys = {
                    "source_delta_ns",
                    "receipt_delta_ns",
                    "delta_error_ns",
                    "delta_error_limit_ns",
                }
                diagnostic_values = dict(diagnostics)
                if set(diagnostic_values) == expected_keys and all(
                    isinstance(value, int)
                    and not isinstance(value, bool)
                    and value >= 0
                    for value in diagnostic_values.values()
                ):
                    if (
                        isinstance(fault_monotonic_ns, int)
                        and not isinstance(fault_monotonic_ns, bool)
                        and fault_monotonic_ns > 0
                    ):
                        payload["fault_monotonic_ns"] = fault_monotonic_ns
                    if trigger_stream in self.streams:
                        payload["trigger_stream"] = trigger_stream
                    payload.update(diagnostic_values)
            _atomic_json(self.fault_file, payload)
        finally:
            # Status evidence is best effort; inability to write it must never
            # keep private publishers or the surrounding stack alive.
            self.shutdown()

    def maybe_lock(self, now_monotonic_ns: int) -> None:
        if self.ready or self.fault_reason is not None:
            return
        qualification_evaluation = None

        def fresh_commit_clocks() -> ReceiptClockPair:
            return fresh_paired_receipt_clocks(
                max_clock_read_span_ns=(
                    self.discipline.config.max_clock_read_span_ns
                ),
                age_acquisition_attempts=(
                    self.clock_pair_age_acquisition_attempts
                ),
            )

        if self.mode == "affine" and self.profile == "motion":
            assert isinstance(
                self.discipline, AffineNavigationStampDiscipline
            )
            deadline_ns = (
                self.discipline.approved_affine_anchor_deadline_monotonic_ns()
            )
            if deadline_ns is None or now_monotonic_ns < deadline_ns:
                return
            if self._affine_terminal_evaluation_started:
                return
            self._affine_terminal_evaluation_started = True
            try:
                commit_clocks = fresh_commit_clocks()
                require_strict_affine_approval(
                    self.approval,
                    now_realtime_ns=commit_clocks.realtime_ns,
                )
                now_monotonic_ns = commit_clocks.monotonic_ns
                previous_clock_base_ns = self.discipline._last_clock_base_ns
                if previous_clock_base_ns is None:
                    raise RuntimeError("live anchor has no prior clock pair")
                clock_base_ns = (
                    commit_clocks.realtime_ns
                    - commit_clocks.monotonic_ns
                )
                clock_base_delta_ns = abs(
                    clock_base_ns - previous_clock_base_ns
                )
                if clock_base_delta_ns > (
                    self.discipline.config.max_clock_pair_discontinuity_ns
                ):
                    raise RuntimeError(
                        "live anchor local clock pair changed"
                    )
                liveness = self.discipline.receipt_liveness_snapshot(
                    commit_clocks.monotonic_ns
                )
                if any(not details["live"] for details in liveness.values()):
                    raise RuntimeError("live anchor stream is stale")
                self.discipline._last_clock_base_ns = clock_base_ns
                self.post_evaluation_commit_diagnostics = {
                    "snapshot_evaluation_monotonic_ns": deadline_ns,
                    "commit_check_realtime_ns": commit_clocks.realtime_ns,
                    "commit_check_monotonic_ns": commit_clocks.monotonic_ns,
                    "evaluation_to_commit_check_ns": (
                        commit_clocks.monotonic_ns - deadline_ns
                    ),
                    "clock_read_span_ns": commit_clocks.read_span_ns,
                    "clock_read_span_limit_ns": (
                        self.discipline.config.max_clock_read_span_ns
                    ),
                    "previous_clock_base_ns": previous_clock_base_ns,
                    "clock_base_ns": clock_base_ns,
                    "clock_base_delta_ns": clock_base_delta_ns,
                    "clock_pair_discontinuity_limit_ns": (
                        self.discipline.config.max_clock_pair_discontinuity_ns
                    ),
                    "stream_receipt_liveness": liveness,
                }
            except Exception as error:
                self.latch_fault(
                    "approved_affine_live_anchor_validation_failed:"
                    f"{type(error).__name__}:{error}"
                )
                return
        elif self.mode == "affine":
            assert isinstance(
                self.discipline, AffineNavigationStampDiscipline
            )
            # Before the second fixed window closes this path is deliberately
            # O(1): observe() keeps enforcing timestamp/domain/delta hard
            # faults, while the executor does no quantiles, envelopes, or
            # Theil-Sen work that could manufacture receipt gaps itself.
            deadline_ns = (
                self.discipline.affine_qualification_deadline_monotonic_ns()
            )
            if deadline_ns is None or now_monotonic_ns < deadline_ns:
                return
            if self._affine_terminal_evaluation_started:
                return
            # Latch before copying/evaluating.  Exceptions, an invalid window,
            # approval disagreement, and READY-write failure are all terminal;
            # no later timer callback may retry or re-lock this session.
            self._affine_terminal_evaluation_started = True
            self.affine_terminal_full_evaluation_count += 1
            try:
                snapshot = self.discipline.capture_affine_qualification_snapshot(
                    now_monotonic_ns
                )
                qualification_evaluation = (
                    self.discipline.evaluate_affine_qualification_snapshot(
                        snapshot
                    )
                )
            except Exception as error:
                self.latch_fault(
                    "affine_qualification_evaluation_failed:"
                    f"{type(error).__name__}:{error}"
                )
                return
            if qualification_evaluation.reasons:
                stable_reason = self.discipline.reject_affine_qualification(
                    qualification_evaluation
                )
                qualification_fault = (
                    self.discipline.affine_qualification_fault_diagnostics(
                        qualification_evaluation,
                        terminal_phase="fixed_two_window_evaluation",
                    )
                )
                self.latch_fault(
                    stable_reason,
                    fault_monotonic_ns=now_monotonic_ns,
                    qualification_fault=qualification_fault,
                )
                return
            # The full immutable-snapshot evaluation above can outlive the
            # 200 ms core receipt budget or cross the approval expiry.  Take a
            # new tight clock pair now and perform only O(streams) checks; the
            # cached model/statistics are never recomputed or re-locked.
            try:
                commit_clocks = fresh_commit_clocks()
                commit_reasons, commit_diagnostics = (
                    self.discipline.affine_post_evaluation_commit_gate(
                        qualification_evaluation,
                        receipt_realtime_ns=commit_clocks.realtime_ns,
                        receipt_monotonic_ns=commit_clocks.monotonic_ns,
                        clock_read_span_ns=commit_clocks.read_span_ns,
                    )
                )
                self.post_evaluation_commit_diagnostics = commit_diagnostics
                if commit_reasons:
                    self.discipline.reject_affine_qualification(
                        qualification_evaluation
                    )
                    self.latch_fault(
                        commit_reasons[0],
                        fault_monotonic_ns=commit_clocks.monotonic_ns,
                        qualification_fault=(
                            self.discipline.affine_qualification_fault_diagnostics(
                                qualification_evaluation,
                                terminal_phase="post_evaluation_commit_gate",
                                reasons=commit_reasons,
                            )
                        ),
                    )
                    return
                require_strict_affine_approval(
                    self.approval,
                    now_realtime_ns=commit_clocks.realtime_ns,
                )
                # The fresh commit clock, never the pre-evaluation timer
                # sample, enters the frozen discipline state.
                now_monotonic_ns = commit_clocks.monotonic_ns
            except Exception as error:
                if self.fault_reason is not None:
                    return
                qualification_fault = (
                    self.discipline.affine_qualification_fault_diagnostics(
                        qualification_evaluation,
                        terminal_phase="post_evaluation_commit_gate",
                        reasons=(
                            "post_evaluation_commit_validation_failed:"
                            f"{type(error).__name__}:{error}",
                        ),
                    )
                )
                self.discipline.reject_affine_qualification(
                    qualification_evaluation
                )
                self.latch_fault(
                    "post_evaluation_commit_validation_failed:"
                    f"{type(error).__name__}:{error}",
                    qualification_fault=qualification_fault,
                )
                return
        else:
            # The legacy fixed path is retained for explicitly reviewed v2
            # sessions.  Both deployed affine workstation profiles use the
            # terminal fixed-window branch above.
            period_ns = self.discipline.config.statistics_evaluation_period_ns
            if (
                self._last_qualification_attempt_ns is not None
                and now_monotonic_ns - self._last_qualification_attempt_ns
                < period_ns
            ):
                return
            self._last_qualification_attempt_ns = now_monotonic_ns
            reasons = self.discipline.qualification_reasons(now_monotonic_ns)
            if reasons:
                if (
                    self._last_qualification_report_ns is None
                    or now_monotonic_ns - self._last_qualification_report_ns
                    >= 5_000_000_000
                ):
                    print(
                        "timestamp qualification pending: " + ",".join(reasons),
                        flush=True,
                    )
                    self._last_qualification_report_ns = now_monotonic_ns
                return
        try:
            locked: int | AffineClockModel
            if self.mode == "fixed":
                locked = self.discipline.lock_fixed(
                    self.approval.fixed_local_minus_source_offset_ns,
                    identity_evidence_verified=True,
                    now_monotonic_ns=now_monotonic_ns,
                )
            else:
                assert isinstance(
                    self.discipline, AffineNavigationStampDiscipline
                )
                if self.profile == "motion":
                    locked = (
                        self.discipline.lock_affine_from_approved_drift(
                            approved_common_drift_ppm=(
                                self.approval.approved_affine_common_drift_ppm
                            ),
                            identity_evidence_verified=True,
                            now_monotonic_ns=now_monotonic_ns,
                        )
                    )
                else:
                    locked = self.discipline.lock_affine(
                        identity_evidence_verified=True,
                        now_monotonic_ns=now_monotonic_ns,
                        # The no-motion profile still qualifies a current
                        # two-window model for mapping evidence.
                        approved_reference_offset_ns=None,
                        approved_common_drift_ppm=None,
                        qualification_evaluation=qualification_evaluation,
                    )
        except RuntimeError as error:
            stable_reason = getattr(self.discipline, "fault_reason", None)
            qualification_fault = None
            if (
                qualification_evaluation is not None
                and isinstance(
                    self.discipline, AffineNavigationStampDiscipline
                )
            ):
                terminal_reason = stable_reason or (
                    f"affine_timestamp_lock_failed:{error}"
                )
                qualification_fault = (
                    self.discipline.affine_qualification_fault_diagnostics(
                        qualification_evaluation,
                        terminal_phase="affine_lock_rejected",
                        reasons=(terminal_reason,),
                    )
                )
            self.latch_fault(
                stable_reason
                or f"{self.mode}_timestamp_lock_failed:{error}",
                fault_monotonic_ns=now_monotonic_ns,
                qualification_fault=qualification_fault,
            )
            return
        ready_payload: dict[str, Any] = {
            "schema": (
                READY_SCHEMA if self.mode == "fixed" else AFFINE_READY_SCHEMA
            ),
            "session_id": self.approval.session_id,
            "correction_mode": self.mode,
            "discipline_profile": self.profile,
            "corrected_topics": {
                name: CORRECTED_TOPICS[name] for name in self.streams
            },
            "time_discipline_ready": True,
            "motion_ready": False,
            "canonical_odom_ready": False,
            "lidar_odom_semantics": "private_mapping_input_not_chassis_odom",
            "timestamp_safety_limits": self.timestamp_safety_limits(),
            "noncore_delta_drop_count": self.noncore_delta_drop_count,
            "last_noncore_delta_drop": (
                None
                if self.last_noncore_delta_drop is None
                else dict(self.last_noncore_delta_drop)
            ),
        }
        if isinstance(locked, int):
            ready_payload["fixed_local_minus_source_offset_ns"] = locked
        else:
            ready_payload["approval_reference_offset_ns"] = (
                self.approval.fixed_local_minus_source_offset_ns
            )
            ready_payload["approval_affine_common_drift_ppm"] = (
                self.approval.approved_affine_common_drift_ppm
            )
            ready_payload["affine_model"] = {
                "anchor_source_ns": locked.anchor_source_ns,
                "anchor_local_ns": locked.anchor_local_ns,
                "drift_ppm": locked.drift_ppm,
                "source_to_local_scale": locked.source_to_local_scale,
                "core_stream_drifts_ppm": dict(
                    locked.core_stream_drifts_ppm
                ),
                "stream_baseline_corrected_age_ns": dict(
                    locked.stream_baseline_corrected_age_ns
                ),
                "frozen": True,
            }
            ready_payload["post_evaluation_commit"] = (
                self.post_evaluation_commit_diagnostics
            )
        try:
            _atomic_json(self.ready_file, ready_payload)
        except Exception as error:
            self.latch_fault(f"ready_status_write_failed:{type(error).__name__}")
            return
        self.ready = True


def run_ros(
    approval: FixedOffsetApproval,
    ready_file: Path,
    fault_file: Path,
    stop_requested: threading.Event | None = None,
    mode: str = "fixed",
    profile: str = "nomotion",
) -> int:
    if mode == "affine":
        # Fail before importing/initializing ROS or constructing endpoints.
        require_strict_affine_approval(approval)
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Imu
    from unitree_go.msg import SportModeState

    context = Context()
    rclpy.init(args=[], context=context)
    runtime = _Runtime(
        approval,
        ready_file,
        fault_file,
        mode=mode,
        profile=profile,
    )
    stop_requested = stop_requested or threading.Event()

    class TimestampDisciplineNode(Node):
        def __init__(self) -> None:
            super().__init__("go2_workstation_nomotion_stamp_discipline", context=context)
            runtime.shutdown = lambda: context.shutdown() if context.ok() else None
            # This private prelayer must observe the newest source stamp, not
            # replay callbacks which accumulated while the single-threaded
            # executor was busy starting the rest of the stack.  A queued old
            # sport sample can otherwise look like a >50 ms source/receipt
            # discontinuity even though the source clock is healthy.  Keep the
            # audited breaker unchanged and bound every raw subscription to
            # one volatile sample.  Read-only workstation evidence qualifies
            # RELIABLE only for the no-motion mapping cloud; all state streams
            # and the complete motion profile remain BEST_EFFORT.
            raw_subscription_qos = {
                stream: QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=(
                        ReliabilityPolicy.RELIABLE
                        if _raw_subscription_is_reliable(
                            runtime.profile, stream
                        )
                        else ReliabilityPolicy.BEST_EFFORT
                    ),
                    durability=DurabilityPolicy.VOLATILE,
                )
                for stream in runtime.streams
            }
            # Do not shadow rclpy.node.Node._publishers.  Humble owns that
            # private list and destroy_node() iterates it during fail-closed
            # shutdown.
            stream_types = {
                "sport_primary": SportModeState,
                "mid360_imu": Imu,
                "mid360_odom": Odometry,
            }
            if runtime.profile == "nomotion":
                # Import and instantiate PointCloud2 endpoints only in the
                # mapping/UI profile.  The strict first-motion process has no
                # cloud endpoint and cannot spend its single executor thread
                # copying a large cloud.
                from sensor_msgs.msg import PointCloud2

                stream_types["mid360_cloud"] = PointCloud2
            # The discipline process intentionally creates corrected
            # publishers only for the three core streams.  PointCloud2 remains
            # subscribed here for timestamp/liveness validation, but its
            # corrected publisher lives in a separately managed ROS process so
            # a blocking multi-megabyte RMW publish cannot stall core
            # Sport/IMU/Odom discipline.
            corrected_publisher_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._corrected_publishers = {
                stream: self.create_publisher(
                    stream_types[stream], CORRECTED_TOPICS[stream],
                    corrected_publisher_qos
                )
                for stream in runtime.streams
                if stream != "mid360_cloud"
            }
            self._subscriptions = [
                self.create_subscription(
                    stream_types[stream],
                    RAW_TOPICS[stream],
                    lambda message, stream=stream: self._observe(
                        stream,
                        message,
                        message.stamp
                        if stream == "sport_primary"
                        else message.header.stamp,
                    ),
                    raw_subscription_qos[stream],
                )
                for stream in runtime.streams
            ]
            self._poll_timer = self.create_timer(0.05, self._poll)

        def _observe(self, stream: str, message: Any, stamp: Any) -> None:
            if runtime.fault_reason is not None:
                return
            try:
                source_ns = source_stamp_ns(stamp)
                clocks = fresh_paired_receipt_clocks(
                    max_clock_read_span_ns=(
                        runtime.discipline.config.max_clock_read_span_ns
                    ),
                    age_acquisition_attempts=(
                        runtime.clock_pair_age_acquisition_attempts
                    ),
                )
                phase_started_ns = time.monotonic_ns()
                try:
                    result = runtime.discipline.observe(
                        stream,
                        source_ns,
                        clocks.realtime_ns,
                        clocks.monotonic_ns,
                        clock_domain=approval.expected_clock_domain,
                        clock_read_span_ns=clocks.read_span_ns,
                    )
                finally:
                    runtime.record_phase_latency(
                        "discipline_observe",
                        time.monotonic_ns() - phase_started_ns,
                    )
                if result.state == DisciplineState.FAULTED:
                    runtime.latch_fault(
                        result.reason,
                        fault_monotonic_ns=clocks.monotonic_ns,
                        diagnostics=result.diagnostics,
                    )
                    return
                if result.reason == f"{NONCORE_DELTA_DROP_REASON_PREFIX}:{stream}":
                    runtime.record_noncore_delta_drop(
                        stream, result, clocks.monotonic_ns
                    )
                    return
                if not runtime.ready:
                    return
                if not result.accepted or result.corrected_stamp_ns is None:
                    return
                if stream == "mid360_cloud":
                    # This process remains the authoritative cloud timing
                    # observer.  The accepted raw sample is intentionally not
                    # copied or published here; the isolated cloud relay reads
                    # this process's frozen model from ready.json.
                    return
                phase_started_ns = time.monotonic_ns()
                try:
                    output = corrected_message_copy(
                        message, stream, result.corrected_stamp_ns
                    )
                finally:
                    runtime.record_phase_latency(
                        "message_copy", time.monotonic_ns() - phase_started_ns
                    )
                phase_started_ns = time.monotonic_ns()
                try:
                    self._corrected_publishers[stream].publish(output)
                finally:
                    runtime.record_phase_latency(
                        "publish", time.monotonic_ns() - phase_started_ns
                    )
            except Exception as error:
                runtime.latch_fault(
                    f"sample_processing_failed:{stream}:"
                    f"{type(error).__name__}:{error}"
                )

        def _poll(self) -> None:
            try:
                if stop_requested.is_set():
                    runtime.shutdown()
                    return
                clocks = fresh_paired_receipt_clocks(
                    max_clock_read_span_ns=(
                        runtime.discipline.config.max_clock_read_span_ns
                    ),
                    age_acquisition_attempts=(
                        runtime.clock_pair_age_acquisition_attempts
                    ),
                )
                if not runtime.ready:
                    # The short-lived approval authorizes this one startup and
                    # frozen-model commit. Expiring an already-consumed startup
                    # witness would deterministically tear down long mapping
                    # sessions even though the immutable model is unchanged.
                    approval.require_valid_at(clocks.realtime_ns)
                    runtime.maybe_lock(clocks.monotonic_ns)
                    return
                phase_started_ns = time.monotonic_ns()
                try:
                    result = runtime.discipline.poll(clocks.monotonic_ns)
                finally:
                    runtime.record_phase_latency(
                        "discipline_poll",
                        time.monotonic_ns() - phase_started_ns,
                    )
                if result.state == DisciplineState.FAULTED:
                    runtime.latch_fault(
                        result.reason,
                        fault_monotonic_ns=clocks.monotonic_ns,
                    )
            except Exception as error:
                runtime.latch_fault(f"poll_failed:{type(error).__name__}:{error}")

    # This node uses an isolated Context so it can fail closed without taking
    # unrelated ROS users down.  It must therefore not fall back to rclpy's
    # global executor/default Context (the Humble guard-condition handles are
    # context-specific).
    executor = SingleThreadedExecutor(context=context)
    node = TimestampDisciplineNode()
    added = False
    try:
        added = executor.add_node(node)
        if not added:
            raise RuntimeError("timestamp node could not be added to its executor")
        executor.spin()
    finally:
        try:
            # A timestamp fault calls context.shutdown() from a callback.  In
            # that case skip remove_node(), which would try to wake an already
            # invalid guard; executor.shutdown() still clears its node set.
            if added and context.ok():
                executor.remove_node(node)
        finally:
            try:
                executor.shutdown()
            finally:
                node.destroy_node()
                if context.ok():
                    context.shutdown()
    return 70 if runtime.fault_reason is not None else 0


def _private_output_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("runtime status paths must be absolute")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start the Go2 workstation no-motion timestamp prelayer"
    )
    parser.add_argument(
        "--mode",
        choices=CORRECTION_MODES,
        default="fixed",
        help="fixed is backward compatible; affine freezes a qualified clock rate",
    )
    parser.add_argument(
        "--profile",
        choices=DISCIPLINE_PROFILES,
        default="nomotion",
        help="motion uses strict freshness/delta gates; nomotion is UI/mapping only",
    )
    parser.add_argument("--approval-file", required=True, type=Path)
    parser.add_argument("--ready-file", required=True, type=_private_output_path)
    parser.add_argument("--fault-file", required=True, type=_private_output_path)
    args = parser.parse_args()
    if args.ready_file == args.fault_file:
        parser.error("ready and fault files must differ")
    try:
        approval = load_approval(
            args.approval_file, require_affine=args.mode == "affine"
        )
    except ApprovalError as error:
        parser.error(str(error))
    for path in (args.ready_file, args.fault_file):
        if path.exists() or path.is_symlink():
            parser.error(f"runtime status path already exists: {path}")
    stop_requested = threading.Event()
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(
        signal.SIGTERM,
        lambda _signal, _frame: stop_requested.set(),
    )
    try:
        return run_ros(
            approval,
            args.ready_file,
            args.fault_file,
            stop_requested=stop_requested,
            mode=args.mode,
            profile=args.profile,
        )
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
