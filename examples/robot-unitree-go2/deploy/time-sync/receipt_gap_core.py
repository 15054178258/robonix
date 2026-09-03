#!/usr/bin/env python3
"""Bounded receipt-gap accounting for subscription-only ROS diagnostics.

This module deliberately has no ROS, network, subprocess, or filesystem
dependency.  A caller supplies ``CLOCK_MONOTONIC`` receipt timestamps; the
tracker keeps exact counters plus a bounded rolling window for percentiles.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Any, Iterable


NANOSECONDS_PER_MILLISECOND = 1_000_000
DEFAULT_THRESHOLDS_NS = tuple(
    milliseconds * NANOSECONDS_PER_MILLISECOND
    for milliseconds in (100, 150, 200)
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


def _validated_thresholds(values: Iterable[int]) -> tuple[int, ...]:
    thresholds = tuple(values)
    if not thresholds:
        raise ValueError("at least one receipt-gap threshold is required")
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value <= 0
        for value in thresholds
    ):
        raise ValueError("receipt-gap thresholds must be positive integers")
    if tuple(sorted(set(thresholds))) != thresholds:
        raise ValueError("receipt-gap thresholds must be unique and increasing")
    return thresholds


@dataclass(frozen=True)
class GapObservation:
    """One interval between consecutive subscriber callback receipts."""

    interval_index: int
    previous_receipt_monotonic_ns: int
    receipt_monotonic_ns: int
    gap_ns: int
    exceeded_thresholds_ns: tuple[int, ...]

    def as_event(self, topic: str, event_written_realtime_ns: int) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "event_type": "receipt_gap_threshold_exceeded",
            "topic": topic,
            "interval_index": self.interval_index,
            "previous_receipt_monotonic_ns": self.previous_receipt_monotonic_ns,
            "receipt_monotonic_ns": self.receipt_monotonic_ns,
            "gap_ns": self.gap_ns,
            "gap_ms": self.gap_ns / NANOSECONDS_PER_MILLISECOND,
            "exceeded_thresholds_ns": list(self.exceeded_thresholds_ns),
            "exceeded_thresholds_ms": [
                value / NANOSECONDS_PER_MILLISECOND
                for value in self.exceeded_thresholds_ns
            ],
            "event_written_realtime_ns": event_written_realtime_ns,
        }


@dataclass
class ReceiptGapTracker:
    """Keep exact gap counters and bounded rolling distribution statistics."""

    thresholds_ns: tuple[int, ...] = DEFAULT_THRESHOLDS_NS
    retained_gap_limit: int = 200_000
    received_messages: int = 0
    observed_intervals: int = 0
    first_receipt_monotonic_ns: int | None = None
    last_receipt_monotonic_ns: int | None = None
    minimum_gap_ns: int | None = None
    maximum_gap_ns: int | None = None
    sum_gap_ns: int = 0
    _threshold_counts: list[int] = field(init=False)
    _retained_gaps_ns: deque[int] = field(init=False)

    def __post_init__(self) -> None:
        self.thresholds_ns = _validated_thresholds(self.thresholds_ns)
        if (
            isinstance(self.retained_gap_limit, bool)
            or not isinstance(self.retained_gap_limit, int)
            or self.retained_gap_limit <= 0
        ):
            raise ValueError("retained_gap_limit must be a positive integer")
        self._threshold_counts = [0 for _ in self.thresholds_ns]
        self._retained_gaps_ns = deque(maxlen=self.retained_gap_limit)

    def observe(self, receipt_monotonic_ns: int) -> GapObservation | None:
        if (
            isinstance(receipt_monotonic_ns, bool)
            or not isinstance(receipt_monotonic_ns, int)
            or receipt_monotonic_ns <= 0
        ):
            raise ValueError("receipt_monotonic_ns must be a positive integer")

        previous = self.last_receipt_monotonic_ns
        if previous is not None and receipt_monotonic_ns < previous:
            raise ValueError("receipt CLOCK_MONOTONIC timestamp regressed")

        self.received_messages += 1
        if self.first_receipt_monotonic_ns is None:
            self.first_receipt_monotonic_ns = receipt_monotonic_ns
        self.last_receipt_monotonic_ns = receipt_monotonic_ns
        if previous is None:
            return None

        gap_ns = receipt_monotonic_ns - previous
        self.observed_intervals += 1
        self.sum_gap_ns += gap_ns
        self.minimum_gap_ns = (
            gap_ns if self.minimum_gap_ns is None else min(self.minimum_gap_ns, gap_ns)
        )
        self.maximum_gap_ns = (
            gap_ns if self.maximum_gap_ns is None else max(self.maximum_gap_ns, gap_ns)
        )
        self._retained_gaps_ns.append(gap_ns)
        exceeded: list[int] = []
        for index, threshold_ns in enumerate(self.thresholds_ns):
            # The commissioning question is explicitly gap > threshold, not
            # gap >= threshold.  Preserve that boundary exactly.
            if gap_ns > threshold_ns:
                self._threshold_counts[index] += 1
                exceeded.append(threshold_ns)

        return GapObservation(
            interval_index=self.observed_intervals,
            previous_receipt_monotonic_ns=previous,
            receipt_monotonic_ns=receipt_monotonic_ns,
            gap_ns=gap_ns,
            exceeded_thresholds_ns=tuple(exceeded),
        )

    def summary(self) -> dict[str, Any]:
        retained = sorted(self._retained_gaps_ns)
        mean_gap_ns = (
            self.sum_gap_ns / self.observed_intervals
            if self.observed_intervals
            else None
        )
        return {
            "received_messages": self.received_messages,
            "observed_intervals": self.observed_intervals,
            "first_receipt_monotonic_ns": self.first_receipt_monotonic_ns,
            "last_receipt_monotonic_ns": self.last_receipt_monotonic_ns,
            "first_to_last_receipt_ns": (
                None
                if self.first_receipt_monotonic_ns is None
                or self.last_receipt_monotonic_ns is None
                else self.last_receipt_monotonic_ns
                - self.first_receipt_monotonic_ns
            ),
            "minimum_gap_ns": self.minimum_gap_ns,
            "mean_gap_ns": mean_gap_ns,
            "maximum_gap_ns": self.maximum_gap_ns,
            "retained_gap_limit": self.retained_gap_limit,
            "retained_gap_count": len(retained),
            "retained_gap_p50_ns": _percentile(retained, 0.50),
            "retained_gap_p95_ns": _percentile(retained, 0.95),
            "retained_gap_p99_ns": _percentile(retained, 0.99),
            "retained_gap_p99_9_ns": _percentile(retained, 0.999),
            "threshold_comparison": "strictly_greater_than",
            "threshold_exceedance_counts": [
                {"threshold_ns": threshold_ns, "count": count}
                for threshold_ns, count in zip(
                    self.thresholds_ns, self._threshold_counts, strict=True
                )
            ],
        }
