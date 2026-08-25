#!/usr/bin/env python3
"""Recover an auditable summary from an interrupted read-only time probe.

The raw ``metadata.json`` and ``samples.jsonl`` files are opened read-only and
hashed before a separately named summary is created.  The recovery never uses
ROS, DDS, a Unitree client, or a publisher, and it never claims to know why an
incomplete process stopped.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
LOGS_ROOT = (ROOT / "logs").resolve()
TIME_SYNC_DIR = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC_DIR))

from go2_time_core import StreamTracker, pairwise_median_offsets  # noqa: E402


DEFAULT_RETAINED_OFFSETS = 200_000
CANONICAL_STREAM_ORDER = (
    "sport_primary",
    "sport_fallback",
    "mid360_cloud",
    "mid360_imu",
)
REQUIRED_SAFETY_FIELDS = {
    "clock_adjustment_requested": False,
    "ros_publishers_created": False,
    "unitree_clients_created": False,
}
RECOVERED_OUTPUT_PATTERN = re.compile(
    r"summary\.recovered(?:[-_.][A-Za-z0-9][A-Za-z0-9_.-]*)?\.json"
)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Recover a separate summary from read-only time evidence"
    )
    parser.add_argument("evidence_dir", type=Path)
    parser.add_argument("--output-name", default="summary.recovered.json")
    return parser


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _stat_identity(path: Path) -> tuple[int, int, int, int]:
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns


def _validated_evidence_directory(evidence_dir: Path) -> Path:
    directory = evidence_dir.expanduser().resolve()
    try:
        relative = directory.relative_to(LOGS_ROOT)
    except ValueError as error:
        raise ValueError(f"evidence_dir must resolve below {LOGS_ROOT}") from error
    if relative == Path("."):
        raise ValueError("evidence_dir must be a child directory below logs/")
    return directory


def _validated_output_name(output_name: str) -> str:
    if Path(output_name).name != output_name or not RECOVERED_OUTPUT_PATTERN.fullmatch(
        output_name
    ):
        raise ValueError(
            "output-name must be an independent summary.recovered*.json basename"
        )
    return output_name


def _require_int(record: dict[str, Any], name: str, line_number: int) -> int:
    value = record.get(name)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"line {line_number}: {name} must be an integer")
    return value


def _validate_observation(
    stored: dict[str, Any], computed: dict[str, Any], line_number: int
) -> None:
    # These fields are sufficient to detect reordering, edited timestamps, or
    # a reconstruction that does not match what the live probe recorded.
    for name in (
        "stream",
        "topic",
        "source_stamp_ns",
        "receipt_realtime_ns",
        "receipt_monotonic_ns",
        "clock_read_span_ns",
        "status",
        "age_ns",
        "source_minus_receipt_ns",
        "source_delta_ns",
        "receipt_monotonic_delta_ns",
    ):
        if stored.get(name) != computed.get(name):
            raise ValueError(
                f"line {line_number}: stored {name} does not match reconstruction"
            )


def recover_summary(evidence_dir: Path, output_name: str) -> tuple[Path, dict[str, Any]]:
    directory = _validated_evidence_directory(evidence_dir)
    output_name = _validated_output_name(output_name)
    metadata_path = directory / "metadata.json"
    samples_path = directory / "samples.jsonl"
    output_path = directory / output_name
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite recovered evidence: {output_path}")

    before_identity = {
        "metadata.json": _stat_identity(metadata_path),
        "samples.jsonl": _stat_identity(samples_path),
    }
    metadata_sha256 = _sha256(metadata_path)
    samples_sha256 = _sha256(samples_path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("mode") != "read-only-no-adjust":
        raise ValueError("metadata mode is not read-only-no-adjust")
    for name, required in REQUIRED_SAFETY_FIELDS.items():
        if metadata.get(name) is not required:
            raise ValueError(f"metadata safety field {name} is not {required!r}")

    topics = metadata.get("topics")
    if not isinstance(topics, dict) or not topics:
        raise ValueError("metadata topics must be a non-empty object")
    stream_order = metadata.get("stream_order")
    if stream_order is None:
        # Schema-v1 metadata was serialized with sort_keys=True and therefore
        # lost insertion order.  Recover the documented probe order first and
        # keep any future/unknown streams deterministic after it.
        stream_order = [name for name in CANONICAL_STREAM_ORDER if name in topics]
        stream_order.extend(sorted(name for name in topics if name not in stream_order))
    if (
        not isinstance(stream_order, list)
        or any(not isinstance(name, str) for name in stream_order)
        or len(stream_order) != len(topics)
        or set(stream_order) != set(topics)
    ):
        raise ValueError("metadata stream_order must list every topic stream once")
    retained_limit = metadata.get(
        "retained_offsets_per_stream", DEFAULT_RETAINED_OFFSETS
    )
    if isinstance(retained_limit, bool) or not isinstance(retained_limit, int):
        raise ValueError("retained_offsets_per_stream must be an integer")
    trackers = {
        stream: StreamTracker(
            stream, topics[stream], retained_offset_limit=retained_limit
        )
        for stream in stream_order
        if isinstance(stream, str) and isinstance(topics[stream], str)
    }
    if len(trackers) != len(topics):
        raise ValueError("metadata topics contain a non-string stream or topic")

    total_samples = 0
    first_receipt_monotonic_ns: int | None = None
    last_receipt_monotonic_ns: int | None = None
    last_receipt_realtime_ns: int | None = None
    with samples_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise ValueError(f"line {line_number}: empty JSONL record")
            try:
                stored = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"line {line_number}: invalid JSON: {error}") from error
            if not isinstance(stored, dict):
                raise ValueError(f"line {line_number}: record must be an object")
            stream_name = stored.get("stream")
            if stream_name not in trackers:
                raise ValueError(f"line {line_number}: unknown stream {stream_name!r}")
            if stored.get("topic") != topics[stream_name]:
                raise ValueError(f"line {line_number}: topic differs from metadata")

            source_stamp_ns = stored.get("source_stamp_ns")
            status = stored.get("status")
            if source_stamp_ns is None:
                if status == "zero":
                    seconds, nanoseconds = 0, 0
                elif status == "malformed":
                    seconds, nanoseconds = -1, 0
                else:
                    raise ValueError(
                        f"line {line_number}: null source stamp has invalid status"
                    )
            elif isinstance(source_stamp_ns, bool) or not isinstance(source_stamp_ns, int):
                raise ValueError(f"line {line_number}: source_stamp_ns must be an integer")
            else:
                seconds, nanoseconds = divmod(source_stamp_ns, 1_000_000_000)

            receipt_realtime_ns = _require_int(
                stored, "receipt_realtime_ns", line_number
            )
            receipt_monotonic_ns = _require_int(
                stored, "receipt_monotonic_ns", line_number
            )
            clock_read_span_ns = _require_int(
                stored, "clock_read_span_ns", line_number
            )
            observation = trackers[stream_name].observe(
                seconds,
                nanoseconds,
                receipt_realtime_ns,
                receipt_monotonic_ns,
                clock_read_span_ns,
            )
            _validate_observation(stored, observation.as_dict(), line_number)
            total_samples += 1
            if first_receipt_monotonic_ns is None:
                first_receipt_monotonic_ns = receipt_monotonic_ns
            if (
                last_receipt_monotonic_ns is not None
                and receipt_monotonic_ns < last_receipt_monotonic_ns
            ):
                raise ValueError(
                    f"line {line_number}: global receipt_monotonic_ns regressed"
                )
            last_receipt_monotonic_ns = receipt_monotonic_ns
            last_receipt_realtime_ns = receipt_realtime_ns

    started_monotonic_ns = metadata.get("started_monotonic_ns")
    if isinstance(started_monotonic_ns, bool) or not isinstance(started_monotonic_ns, int):
        raise ValueError("metadata started_monotonic_ns must be an integer")
    elapsed_observed_ns = (
        None
        if last_receipt_monotonic_ns is None
        else last_receipt_monotonic_ns - started_monotonic_ns
    )
    configured_duration_ns = int(metadata["duration_seconds"]) * 1_000_000_000
    duration_reached = (
        elapsed_observed_ns is not None and elapsed_observed_ns >= configured_duration_ns
    )
    max_samples_reached = total_samples >= int(metadata["max_samples"])
    summaries = [trackers[name].summary() for name in stream_order]
    anomaly_count = sum(
        tracker.zero + tracker.malformed + tracker.duplicates + tracker.regressions
        for tracker in trackers.values()
    )

    after_identity = {
        "metadata.json": _stat_identity(metadata_path),
        "samples.jsonl": _stat_identity(samples_path),
    }
    if before_identity != after_identity:
        raise RuntimeError("raw evidence changed while it was being recovered")
    if _sha256(metadata_path) != metadata_sha256 or _sha256(samples_path) != samples_sha256:
        raise RuntimeError("raw evidence digest changed while it was being recovered")

    result = {
        "schema_version": 1,
        "mode": "read-only-no-adjust-recovery",
        "exit_reason": (
            "completion_boundary_observed"
            if duration_reached or max_samples_reached
            else "incomplete_before_duration_or_sample_cap"
        ),
        "termination_cause": "unknown",
        "total_samples": total_samples,
        "started_realtime_ns": metadata.get("started_realtime_ns"),
        "started_monotonic_ns": started_monotonic_ns,
        "first_sample_receipt_monotonic_ns": first_receipt_monotonic_ns,
        "last_sample_receipt_monotonic_ns": last_receipt_monotonic_ns,
        "last_sample_receipt_realtime_ns": last_receipt_realtime_ns,
        "elapsed_observed_monotonic_ns": elapsed_observed_ns,
        "configured_duration_ns": configured_duration_ns,
        "duration_boundary_observed": duration_reached,
        "sample_cap_observed": max_samples_reached,
        "retained_offsets_per_stream": retained_limit,
        "stream_order": stream_order,
        "streams": summaries,
        "pairwise_median_offset_comparisons": pairwise_median_offsets(
            list(trackers.values())
        ),
        "timestamp_anomaly_count": anomaly_count,
        "raw_evidence": {
            "metadata.json": {
                "sha256": metadata_sha256,
                "size_bytes": before_identity["metadata.json"][2],
            },
            "samples.jsonl": {
                "sha256": samples_sha256,
                "size_bytes": before_identity["samples.jsonl"][2],
                "validated_records": total_samples,
            },
        },
        "raw_observation_validation_errors": 0,
        "safe_for_clock_discipline": False,
        "note": (
            "Recovered from immutable raw observations. No process termination "
            "cause or completion time is inferred; publisher locator, cold-boot, "
            "long-duration drift, and operator review remain mandatory."
        ),
    }

    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        os.chmod(temporary, 0o600)
        json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(output_path)
    return output_path, result


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        output_path, _summary = recover_summary(
            arguments.evidence_dir, arguments.output_name
        )
    except (OSError, ValueError, RuntimeError) as error:
        print(f"recovery failed: {error}", file=sys.stderr)
        return 1
    print(f"Recovered read-only summary saved to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
