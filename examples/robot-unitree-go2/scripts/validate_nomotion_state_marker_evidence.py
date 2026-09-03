#!/usr/bin/env python3
"""Validate fresh subscriber-only SportModeState evidence for no-motion startup.

The only successful stdout value is the exact, consistently observed
``error_code`` field.  In this integration that field is treated as an opaque
state marker; this helper does not assign a health or motion-mode meaning to it.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import stat
import sys
import time
from typing import Any


EXPECTED_TOPICS = frozenset(("/sportmodestate", "/lf/sportmodestate"))
MAX_EVIDENCE_BYTES = 1_048_576
DEFAULT_MAX_AGE_SECONDS = 30 * 60
MIN_CAPTURE_SECONDS = 20
MAX_CAPTURE_SECONDS = 120
MAX_STATIONARY_SPEED = 0.05
FUTURE_TOLERANCE_NS = 5_000_000_000


class EvidenceError(ValueError):
    """Raised when evidence cannot safely authorize the passive state marker."""


def _integer(value: Any, field: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise EvidenceError(f"{field} must be an integer")
    if minimum is not None and value < minimum:
        raise EvidenceError(f"{field} must be >= {minimum}")
    return value


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError(f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise EvidenceError(f"{field} must be a finite number")
    return result


def _object_without_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise EvidenceError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _load_private_regular_file(path: Path) -> dict[str, Any]:
    if not path.is_absolute():
        raise EvidenceError("evidence path must be absolute")

    try:
        before = os.lstat(path)
    except OSError as error:
        raise EvidenceError(f"cannot stat evidence: {error.strerror}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise EvidenceError("evidence must be a regular non-symlink file")
    if before.st_uid != os.getuid():
        raise EvidenceError("evidence must be owned by the current user")
    if before.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
        raise EvidenceError("evidence must not be group- or world-writable")
    if not 0 < before.st_size <= MAX_EVIDENCE_BYTES:
        raise EvidenceError(
            f"evidence size must be in 1..{MAX_EVIDENCE_BYTES} bytes"
        )

    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise EvidenceError(f"cannot safely open evidence: {error.strerror}") from error
    try:
        opened = os.fstat(descriptor)
        if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
            raise EvidenceError("evidence changed while it was being opened")
        if not stat.S_ISREG(opened.st_mode) or opened.st_uid != os.getuid():
            raise EvidenceError("opened evidence identity is not trusted")
        if opened.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
            raise EvidenceError("opened evidence is group- or world-writable")
        if opened.st_size != before.st_size:
            raise EvidenceError("evidence size changed while it was being opened")
        with os.fdopen(descriptor, "r", encoding="utf-8") as stream:
            descriptor = -1
            text = stream.read(MAX_EVIDENCE_BYTES + 1)
    except (OSError, UnicodeError) as error:
        raise EvidenceError(f"cannot read evidence: {error}") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)

    if len(text.encode("utf-8")) > MAX_EVIDENCE_BYTES:
        raise EvidenceError("evidence exceeds the size limit")
    try:
        payload = json.loads(text, object_pairs_hook=_object_without_duplicate_keys)
    except EvidenceError:
        raise
    except json.JSONDecodeError as error:
        raise EvidenceError(f"evidence is not valid JSON: {error.msg}") from error
    if not isinstance(payload, dict):
        raise EvidenceError("evidence root must be a JSON object")
    return payload


def validate_evidence(
    path: Path,
    *,
    max_age_seconds: int | None = DEFAULT_MAX_AGE_SECONDS,
    now_realtime_ns: int | None = None,
) -> int:
    """Return the exact consistent state marker, or raise ``EvidenceError``."""

    if max_age_seconds is not None:
        if isinstance(max_age_seconds, bool) or not isinstance(max_age_seconds, int):
            raise EvidenceError("max age must be an integer number of seconds")
        if not 1 <= max_age_seconds <= 86_400:
            raise EvidenceError("max age must be in 1..86400 seconds")

    payload = _load_private_regular_file(path)
    if payload.get("schema_version") != 1:
        raise EvidenceError("schema_version must be exactly 1")
    if payload.get("mode") != "read-only-subscriber-only":
        raise EvidenceError("mode must be read-only-subscriber-only")
    if payload.get("publishers_created") is not False:
        raise EvidenceError("publishers_created must be false")
    if payload.get("unitree_clients_created") is not False:
        raise EvidenceError("unitree_clients_created must be false")

    duration_s = _integer(
        payload.get("duration_limit_s"), "duration_limit_s", minimum=MIN_CAPTURE_SECONDS
    )
    if duration_s > MAX_CAPTURE_SECONDS:
        raise EvidenceError(f"duration_limit_s must be <= {MAX_CAPTURE_SECONDS}")
    elapsed_ns = _integer(
        payload.get("elapsed_monotonic_ns"),
        "elapsed_monotonic_ns",
        minimum=MIN_CAPTURE_SECONDS * 1_000_000_000,
    )
    if elapsed_ns > (duration_s + 5) * 1_000_000_000:
        raise EvidenceError("elapsed_monotonic_ns is inconsistent with duration_limit_s")
    started_ns = _integer(
        payload.get("started_realtime_ns"), "started_realtime_ns", minimum=1
    )
    ended_ns = started_ns + elapsed_ns
    now_ns = time.time_ns() if now_realtime_ns is None else _integer(
        now_realtime_ns, "now_realtime_ns", minimum=1
    )
    if ended_ns > now_ns + FUTURE_TOLERANCE_NS:
        raise EvidenceError("evidence end time is in the future")
    if (
        max_age_seconds is not None
        and now_ns - ended_ns > max_age_seconds * 1_000_000_000
    ):
        raise EvidenceError("evidence is stale")

    streams = payload.get("streams")
    if not isinstance(streams, list) or len(streams) != len(EXPECTED_TOPICS):
        raise EvidenceError("streams must contain exactly the two expected topics")
    by_topic: dict[str, dict[str, Any]] = {}
    for index, stream in enumerate(streams):
        if not isinstance(stream, dict):
            raise EvidenceError(f"streams[{index}] must be an object")
        topic = stream.get("topic")
        if topic not in EXPECTED_TOPICS or topic in by_topic:
            raise EvidenceError("streams must contain each expected topic exactly once")
        by_topic[topic] = stream
    if set(by_topic) != EXPECTED_TOPICS:
        raise EvidenceError("streams do not match the expected topic set")

    observed_state: tuple[int, int, int] | None = None
    for topic in sorted(EXPECTED_TOPICS):
        stream = by_topic[topic]
        received = _integer(stream.get("received"), f"{topic}.received", minimum=100)
        first_ns = _integer(
            stream.get("first_source_stamp_ns"),
            f"{topic}.first_source_stamp_ns",
            minimum=1,
        )
        last_ns = _integer(
            stream.get("last_source_stamp_ns"),
            f"{topic}.last_source_stamp_ns",
            minimum=1,
        )
        if last_ns <= first_ns:
            raise EvidenceError(f"{topic} source timestamps did not advance")
        if _integer(
            stream.get("source_regressions"), f"{topic}.source_regressions", minimum=0
        ) != 0:
            raise EvidenceError(f"{topic} contains source timestamp regressions")

        for field in ("max_abs_linear_velocity", "max_abs_yaw_speed"):
            speed = _number(stream.get(field), f"{topic}.{field}")
            if speed < 0 or speed > MAX_STATIONARY_SPEED:
                raise EvidenceError(
                    f"{topic}.{field} exceeds stationary limit {MAX_STATIONARY_SPEED}"
                )

        states = stream.get("states")
        if not isinstance(states, list) or len(states) != 1:
            raise EvidenceError(f"{topic} must contain exactly one observed state")
        state = states[0]
        if not isinstance(state, dict):
            raise EvidenceError(f"{topic}.states[0] must be an object")
        marker = _integer(state.get("error_code"), f"{topic}.error_code", minimum=0)
        mode = _integer(state.get("mode"), f"{topic}.mode", minimum=0)
        gait_type = _integer(state.get("gait_type"), f"{topic}.gait_type", minimum=0)
        samples = _integer(state.get("samples"), f"{topic}.samples", minimum=100)
        if samples != received:
            raise EvidenceError(f"{topic} state sample total does not equal received")
        current = (marker, mode, gait_type)
        if observed_state is None:
            observed_state = current
        elif current != observed_state:
            raise EvidenceError("the two SportModeState streams disagree")

    if observed_state is None:  # Defensive; exact topic validation makes this unreachable.
        raise EvidenceError("no SportModeState marker was observed")
    return observed_state[0]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate fresh no-motion SportModeState evidence"
    )
    parser.add_argument("evidence", type=Path)
    parser.add_argument(
        "--max-age-seconds",
        type=int,
        default=DEFAULT_MAX_AGE_SECONDS,
        help=f"freshness window (default: {DEFAULT_MAX_AGE_SECONDS})",
    )
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    try:
        marker = validate_evidence(
            arguments.evidence,
            max_age_seconds=arguments.max_age_seconds,
        )
    except EvidenceError as error:
        print(f"invalid no-motion state evidence: {error}", file=sys.stderr)
        return 2
    print(marker)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
