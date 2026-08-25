#!/usr/bin/env python3
"""Pure-Python timestamp accounting shared by the read-only time tools.

This module has no ROS, network, subprocess, or clock-adjustment dependency.
It only compares a source timestamp with two clocks sampled at receipt:
``CLOCK_REALTIME`` for clock-domain offset and ``CLOCK_MONOTONIC`` for elapsed
time and drift.  Keeping the arithmetic here makes the safety properties
testable without a robot or a ROS graph.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from collections import deque
import math
import statistics
import time
from typing import Any


NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_to_nanoseconds(seconds: int, nanoseconds: int) -> int | None:
    """Return a positive, well-formed timestamp or ``None``."""

    if isinstance(seconds, bool) or isinstance(nanoseconds, bool):
        return None
    if not isinstance(seconds, int) or not isinstance(nanoseconds, int):
        return None
    if seconds < 0 or not 0 <= nanoseconds < NANOSECONDS_PER_SECOND:
        return None
    result = seconds * NANOSECONDS_PER_SECOND + nanoseconds
    return result if result > 0 else None


def read_clock_pair() -> tuple[int, int, int]:
    """Bracket realtime with monotonic reads.

    The midpoint monotonic timestamp is used for elapsed-time accounting.  The
    bracket width is included in every evidence record so slow/preempted clock
    reads remain visible rather than being silently treated as precision.
    """

    monotonic_before = time.monotonic_ns()
    realtime = time.time_ns()
    monotonic_after = time.monotonic_ns()
    return realtime, (monotonic_before + monotonic_after) // 2, (
        monotonic_after - monotonic_before
    )


def _percentile(sorted_values: list[int], fraction: float) -> float | None:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    position = fraction * (len(sorted_values) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(sorted_values[lower])
    weight = position - lower
    return (
        float(sorted_values[lower]) * (1.0 - weight)
        + float(sorted_values[upper]) * weight
    )


@dataclass(frozen=True)
class Observation:
    stream: str
    topic: str
    source_stamp_ns: int | None
    receipt_realtime_ns: int
    receipt_monotonic_ns: int
    clock_read_span_ns: int
    status: str
    age_ns: int | None
    source_minus_receipt_ns: int | None
    source_delta_ns: int | None
    receipt_monotonic_delta_ns: int | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "stream": self.stream,
            "topic": self.topic,
            "source_stamp_ns": self.source_stamp_ns,
            "receipt_realtime_ns": self.receipt_realtime_ns,
            "receipt_monotonic_ns": self.receipt_monotonic_ns,
            "clock_read_span_ns": self.clock_read_span_ns,
            "status": self.status,
            "age_ns": self.age_ns,
            "source_minus_receipt_ns": self.source_minus_receipt_ns,
            "source_delta_ns": self.source_delta_ns,
            "receipt_monotonic_delta_ns": self.receipt_monotonic_delta_ns,
        }


@dataclass
class StreamTracker:
    """Bounded timestamp statistics for one source topic."""

    name: str
    topic: str
    retained_offset_limit: int = 200_000
    received: int = 0
    valid: int = 0
    malformed: int = 0
    zero: int = 0
    duplicates: int = 0
    regressions: int = 0
    first_source_ns: int | None = None
    last_source_ns: int | None = None
    first_receipt_monotonic_ns: int | None = None
    last_receipt_monotonic_ns: int | None = None
    first_offset_ns: int | None = None
    last_offset_ns: int | None = None
    max_clock_read_span_ns: int = 0
    _offsets_ns: deque[int] = field(init=False)

    def __post_init__(self) -> None:
        if self.retained_offset_limit <= 0:
            raise ValueError("retained_offset_limit must be positive")
        # A deque is important here.  The old append-until-full list remained
        # bounded, but its statistics froze on the first samples of a long
        # run.  This is a real rolling window and remains bounded indefinitely.
        self._offsets_ns = deque(maxlen=self.retained_offset_limit)

    def observe(
        self,
        seconds: int,
        nanoseconds: int,
        receipt_realtime_ns: int,
        receipt_monotonic_ns: int,
        clock_read_span_ns: int = 0,
    ) -> Observation:
        self.received += 1
        self.max_clock_read_span_ns = max(
            self.max_clock_read_span_ns, max(0, int(clock_read_span_ns))
        )
        source_ns = stamp_to_nanoseconds(seconds, nanoseconds)
        if source_ns is None:
            if seconds == 0 and nanoseconds == 0:
                self.zero += 1
                status = "zero"
            else:
                self.malformed += 1
                status = "malformed"
            return Observation(
                self.name,
                self.topic,
                None,
                receipt_realtime_ns,
                receipt_monotonic_ns,
                clock_read_span_ns,
                status,
                None,
                None,
                None,
                None,
            )

        source_delta = (
            None if self.last_source_ns is None else source_ns - self.last_source_ns
        )
        monotonic_delta = (
            None
            if self.last_receipt_monotonic_ns is None
            else receipt_monotonic_ns - self.last_receipt_monotonic_ns
        )
        if self.last_source_ns is not None and source_ns < self.last_source_ns:
            self.regressions += 1
            status = "regression"
        elif self.last_source_ns is not None and source_ns == self.last_source_ns:
            self.duplicates += 1
            status = "duplicate"
        else:
            status = "advancing"

        age_ns = receipt_realtime_ns - source_ns
        offset_ns = -age_ns
        self.valid += 1
        if self.first_source_ns is None:
            self.first_source_ns = source_ns
            self.first_receipt_monotonic_ns = receipt_monotonic_ns
            self.first_offset_ns = offset_ns
        self.last_source_ns = source_ns
        self.last_receipt_monotonic_ns = receipt_monotonic_ns
        self.last_offset_ns = offset_ns
        self._offsets_ns.append(offset_ns)

        return Observation(
            self.name,
            self.topic,
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_read_span_ns,
            status,
            age_ns,
            offset_ns,
            source_delta,
            monotonic_delta,
        )

    @property
    def median_offset_ns(self) -> float | None:
        if not self._offsets_ns:
            return None
        return float(statistics.median(self._offsets_ns))

    def summary(self) -> dict[str, Any]:
        offsets = sorted(self._offsets_ns)
        median = self.median_offset_ns
        jitter_stddev = (
            float(statistics.pstdev(offsets)) if len(offsets) > 1 else 0.0
        ) if offsets else None
        absolute_deviations = (
            sorted(int(abs(value - median)) for value in offsets)
            if median is not None
            else []
        )
        drift_ppm: float | None = None
        if (
            self.first_offset_ns is not None
            and self.last_offset_ns is not None
            and self.first_receipt_monotonic_ns is not None
            and self.last_receipt_monotonic_ns is not None
        ):
            elapsed = self.last_receipt_monotonic_ns - self.first_receipt_monotonic_ns
            if elapsed > 0:
                drift_ppm = (
                    (self.last_offset_ns - self.first_offset_ns) / elapsed * 1_000_000.0
                )
        return {
            "stream": self.name,
            "topic": self.topic,
            "received": self.received,
            "valid": self.valid,
            "malformed": self.malformed,
            "zero": self.zero,
            "duplicates": self.duplicates,
            "regressions": self.regressions,
            "first_source_stamp_ns": self.first_source_ns,
            "last_source_stamp_ns": self.last_source_ns,
            "retained_offset_samples": len(offsets),
            "source_minus_receipt_ns_min": float(offsets[0]) if offsets else None,
            "source_minus_receipt_ns_median": median,
            "source_minus_receipt_ns_p95": _percentile(offsets, 0.95),
            "source_minus_receipt_ns_max": float(offsets[-1]) if offsets else None,
            "offset_jitter_stddev_ns": jitter_stddev,
            "offset_jitter_abs_deviation_p95_ns": _percentile(
                absolute_deviations, 0.95
            ),
            "estimated_drift_ppm": drift_ppm,
            "max_clock_read_span_ns": self.max_clock_read_span_ns,
        }


def pairwise_median_offsets(trackers: list[StreamTracker]) -> list[dict[str, Any]]:
    """Compare stream medians without claiming simultaneous acquisition."""

    result: list[dict[str, Any]] = []
    for index, left in enumerate(trackers):
        left_median = left.median_offset_ns
        if left_median is None:
            continue
        for right in trackers[index + 1 :]:
            right_median = right.median_offset_ns
            if right_median is None:
                continue
            result.append(
                {
                    "left_stream": left.name,
                    "right_stream": right.name,
                    "left_minus_right_median_offset_ns": left_median - right_median,
                    "simultaneous_measurement": False,
                }
            )
    return result
