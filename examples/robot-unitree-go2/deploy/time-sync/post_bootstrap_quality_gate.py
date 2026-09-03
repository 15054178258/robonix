#!/usr/bin/env python3
"""Audit the continuous post-bootstrap Go2 clock quality window.

This process has no ROS, network, publisher, subprocess, or clock-setting
surface.  It only reads the feeder's private health JSON, waits on monotonic
time, and writes an immutable JSONL sample trail plus a summary report.
"""

from __future__ import annotations

from collections import deque
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import time
from typing import Any


EXPECTED_SCHEMA = 3
EXPECTED_POLICY = {
    "reference_topic": "/sportmodestate",
    "minimum_continuous_duration_ns": 300_000_000_000,
    "maximum_absolute_offset_ns_exclusive": 50_000_000,
    "maximum_absolute_drift_ppm_exclusive": 100.0,
    "maximum_observation_duration_ns": 900_000_000_000,
    "audit_file": "post-bootstrap-quality.json",
    "samples_file": "post-bootstrap-quality-samples.jsonl",
    "require_unsafe_latched_false": True,
    "require_current_writer_authorized": True,
    "require_feed_count_advancing": True,
    "require_chronyd_go2_selected": True,
}


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def policy_from_approval(approval: dict[str, Any]) -> dict[str, Any]:
    if approval.get("schema_version") != EXPECTED_SCHEMA:
        raise ValueError("verified schema_version=3 approval is required")
    policy = approval.get("post_bootstrap_quality_gate")
    if policy != EXPECTED_POLICY:
        raise ValueError("post-bootstrap quality policy is not exact")
    return dict(EXPECTED_POLICY)


def robust_drift_ppm(samples: list[tuple[int, int]]) -> float | None:
    if len(samples) < 4:
        return None
    slopes: list[float] = []
    for index, (left_time, left_offset) in enumerate(samples):
        for right_time, right_offset in samples[index + 1 :]:
            elapsed = right_time - left_time
            if elapsed > 0:
                slopes.append(
                    (right_offset - left_offset) / elapsed * 1_000_000.0
                )
    return float(statistics.median(slopes)) if slopes else None


class QualityGate:
    def __init__(self, policy: dict[str, Any]) -> None:
        self.policy = policy
        self.samples: deque[tuple[int, int]] = deque(maxlen=2_000)
        self.last_feed_count: int | None = None
        self.reset_count = 0
        self.last_failure: str | None = None
        self.passed_metrics: dict[str, Any] | None = None

    def _reason(self, health: dict[str, Any], now_ns: int) -> str | None:
        if health.get("unsafe_latched") is not False:
            return "unsafe_latched"
        required = set(health.get("required_topics", []))
        authorized = set(health.get("authorized_topics", []))
        reference = self.policy["reference_topic"]
        if reference not in required or required - authorized:
            return "writer_not_authorized"
        updated = health.get("updated_monotonic_ns")
        last_sample = health.get("last_valid_sample_monotonic_ns")
        if not isinstance(updated, int) or not 0 <= now_ns - updated < 5_000_000_000:
            return "health_stale"
        if (
            not isinstance(last_sample, int)
            or not 0 <= now_ns - last_sample < 2_000_000_000
        ):
            return "source_stale"
        feed_count = health.get("feed_count")
        if not isinstance(feed_count, int) or feed_count <= 0:
            return "feed_count_invalid"
        if self.last_feed_count is not None and feed_count <= self.last_feed_count:
            return "feed_count_not_advancing"
        self.last_feed_count = feed_count
        streams = health.get("stream_quality")
        stream = streams.get(reference) if isinstance(streams, dict) else None
        offset = stream.get("last_source_minus_local_ns") if isinstance(stream, dict) else None
        if not isinstance(offset, int):
            return "offset_missing"
        if abs(offset) >= self.policy["maximum_absolute_offset_ns_exclusive"]:
            return "offset_out_of_bounds"
        return None

    def observe(self, health: dict[str, Any], now_ns: int) -> str:
        reason = self._reason(health, now_ns)
        if reason is not None:
            if self.samples:
                self.reset_count += 1
            self.samples.clear()
            self.last_failure = reason
            return "qualifying"
        stream = health["stream_quality"][self.policy["reference_topic"]]
        offset = int(stream["last_source_minus_local_ns"])
        self.samples.append((now_ns, offset))
        span = self.samples[-1][0] - self.samples[0][0]
        if span < self.policy["minimum_continuous_duration_ns"]:
            return "qualifying"
        materialized = list(self.samples)
        drift = robust_drift_ppm(materialized)
        if drift is None or not math.isfinite(drift):
            self.last_failure = "drift_unavailable"
            return "qualifying"
        if abs(drift) >= self.policy["maximum_absolute_drift_ppm_exclusive"]:
            self.reset_count += 1
            self.last_failure = "drift_out_of_bounds"
            self.samples.clear()
            self.samples.append((now_ns, offset))
            return "qualifying"
        offsets = [item[1] for item in materialized]
        self.passed_metrics = {
            "continuous_duration_ns": span,
            "sample_count": len(materialized),
            "maximum_absolute_offset_ns": max(abs(value) for value in offsets),
            "robust_drift_ppm": drift,
        }
        return "passed"


def _write_private(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, sort_keys=True, indent=2) + "\n", encoding="utf-8"
    )
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def run(arguments: argparse.Namespace) -> int:
    approval = _load_json(arguments.approval_file)
    policy = policy_from_approval(approval)
    for path in (arguments.output, arguments.samples_output):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite: {path}")
        path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    gate = QualityGate(policy)
    started_ns = time.monotonic_ns()
    result = "failed"
    sample_count = 0
    with arguments.samples_output.open("x", encoding="utf-8", buffering=1) as stream:
        os.chmod(arguments.samples_output, 0o600)
        while time.monotonic_ns() - started_ns < policy[
            "maximum_observation_duration_ns"
        ]:
            now_ns = time.monotonic_ns()
            try:
                health = _load_json(arguments.health_file)
                state = gate.observe(health, now_ns)
                record = {
                    "observed_monotonic_ns": now_ns,
                    "state": state,
                    "failure": gate.last_failure,
                    "feed_count": health.get("feed_count"),
                    "stream_quality": health.get("stream_quality"),
                }
            except (OSError, ValueError, json.JSONDecodeError) as error:
                state = "qualifying"
                gate.samples.clear()
                gate.reset_count += 1
                gate.last_failure = "health_read_error"
                record = {
                    "observed_monotonic_ns": now_ns,
                    "state": state,
                    "failure": gate.last_failure,
                    "detail": str(error),
                }
            stream.write(json.dumps(record, sort_keys=True) + "\n")
            sample_count += 1
            if state == "passed":
                result = "passed"
                break
            time.sleep(arguments.poll_seconds)
    finished_ns = time.monotonic_ns()
    report = {
        "schema_version": 1,
        "gate": "go2-post-bootstrap-five-minute-quality",
        "result": result,
        "policy": policy,
        "started_monotonic_ns": started_ns,
        "finished_monotonic_ns": finished_ns,
        "observation_duration_ns": finished_ns - started_ns,
        "sample_count": sample_count,
        "reset_count": gate.reset_count,
        "last_failure": gate.last_failure,
        "passed_metrics": gate.passed_metrics,
        "approval_sha256": hashlib.sha256(
            arguments.approval_file.read_bytes()
        ).hexdigest(),
        "samples_sha256": hashlib.sha256(
            arguments.samples_output.read_bytes()
        ).hexdigest(),
        "clock_adjustment_requested_by_gate": False,
    }
    _write_private(arguments.output, report)
    return 0 if result == "passed" else 6


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--health-file", required=True, type=Path)
    parser.add_argument("--approval-file", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--samples-output", required=True, type=Path)
    parser.add_argument("--poll-seconds", type=float, default=1.0)
    arguments = parser.parse_args(argv)
    if not 0.1 <= arguments.poll_seconds <= 5.0:
        parser.error("poll-seconds must be 0.1..5.0")
    try:
        return run(arguments)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"post-bootstrap quality gate failed closed: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
