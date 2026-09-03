#!/usr/bin/env python3
"""Prepare one short-lived workstation no-motion timestamp approval.

This tool is an offline evidence transformation.  It opens retained regular
files only: it does not initialize ROS, inspect a live graph, open a network
socket, launch another process, adjust a clock, or expose a motion surface.

The fixed offset and four writer GIDs are deliberately not CLI inputs.  They
are derived from the raw time JSONL, verbose topic-info files, and a classic
PCAP captured in the same writer session.  Stored correlation JSON is checked
against a fresh parse of the PCAP before an approval can be published.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import time
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC_DIR = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC_DIR))
sys.path.insert(0, str(ROOT / "scripts"))

from correlate_rtps_writer_locator import correlate  # noqa: E402
from evidence_bundle import (  # noqa: E402
    CONCLUSION,
    METHOD,
    _correlation_security_view,
    compact_gid,
    publisher_gids_from_topic_info,
    sha256_file,
)
from go2_time_core import StreamTracker  # noqa: E402
from navigation_stamp_discipline import (  # noqa: E402
    AffineDriftIntervalEvidence,
    AffineNavigationStampDiscipline,
    DisciplineState,
    go2_workstation_nomotion_config,
)
from workstation_nomotion_approval import (  # noqa: E402
    ACK,
    AFFINE_DRIFT_ALGORITHM,
    AFFINE_DRIFT_WINDOW_NS,
    EXPECTED_CLOCK_DOMAIN,
    EXPECTED_RAW_TOPICS,
    EXPECTED_WRITER_SOURCE,
    SCHEMA,
    ApprovalError,
    load_approval,
)


NS = 1_000_000_000
MINIMUM_CAPTURE_SECONDS = 60
MAX_TIME_EVIDENCE_BYTES = 2 * 1024 * 1024 * 1024
MAX_IDENTITY_TEXT_BYTES = 2 * 1024 * 1024
MAX_PCAP_BYTES = 512 * 1024 * 1024
EXPECTED_TIME_TOPICS = {
    "sport_primary": "/sportmodestate",
    "sport_fallback": "/lf/sportmodestate",
    "mid360_cloud": "/utlidar/cloud",
    "mid360_imu": "/utlidar/imu",
    "mid360_odom": "/utlidar/robot_odom",
}
EXPECTED_QUALIFICATION_STREAMS = [
    "sport_primary",
    "mid360_imu",
    "mid360_cloud",
    "mid360_odom",
]
EXPECTED_WITNESS_STREAMS = ["sport_fallback"]
AFFINE_CORE_STREAMS = ("sport_primary", "mid360_imu", "mid360_odom")
IDENTITY_FILENAMES = {
    stream: {
        "topic_info": f"{stream}.topic-info.txt",
        "correlation": f"{stream}.correlation.json",
    }
    for stream in EXPECTED_RAW_TOPICS
}
PCAP_FILENAME = "go2-rtps.pcap"
TIME_FILENAMES = ("metadata.json", "samples.jsonl", "summary.json")
ALLOWED_OUTPUT_ROOTS = (ROOT / "logs", ROOT / "rbnx-build")


class PreparationError(ValueError):
    """Retained evidence cannot safely support a no-motion approval."""


def _exact_int(value: Any, label: str, minimum: int | None = None) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise PreparationError(f"{label} must be an integer")
    if minimum is not None and value < minimum:
        raise PreparationError(f"{label} must be >= {minimum}")
    return value


def _absolute_without_resolving(path: Path) -> Path:
    return Path(os.path.abspath(os.path.expanduser(os.fspath(path))))


def _validated_output_path(path: Path, label: str) -> Path:
    absolute = _absolute_without_resolving(path)
    # Resolve the parent, not the final filename.  This catches an intermediate
    # symlink that escapes the repository while still letting the later O_EXCL
    # check reject an existing final symlink instead of following it.
    resolved_parent = absolute.parent.resolve(strict=False)
    candidate = resolved_parent / absolute.name
    allowed = tuple(root.resolve(strict=False) for root in ALLOWED_OUTPUT_ROOTS)
    if not any(candidate != root and candidate.is_relative_to(root) for root in allowed):
        roots = ", ".join(str(root) for root in allowed)
        raise PreparationError(f"{label} must be below an ignored local root: {roots}")
    return candidate


def _regular_evidence_file(path: Path, label: str, maximum_bytes: int) -> Path:
    path = _absolute_without_resolving(path)
    try:
        info = os.lstat(path)
    except OSError as error:
        raise PreparationError(f"cannot inspect {label}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise PreparationError(f"{label} must be a real regular file")
    if info.st_uid != os.geteuid():
        raise PreparationError(f"{label} must be owned by the current user")
    if info.st_mode & 0o022:
        raise PreparationError(f"{label} must not be group/world writable")
    if info.st_size <= 0 or info.st_size > maximum_bytes:
        raise PreparationError(f"{label} size is outside the accepted bound")
    return path


def _evidence_snapshot(path: Path) -> tuple[int, int, int, int, str]:
    info = os.lstat(path)
    return (
        info.st_dev,
        info.st_ino,
        info.st_size,
        info.st_mtime_ns,
        sha256_file(path),
    )


def _require_unchanged(
    path: Path, before: tuple[int, int, int, int, str], label: str
) -> None:
    try:
        after = _evidence_snapshot(path)
    except OSError as error:
        raise PreparationError(f"cannot recheck {label}: {error}") from error
    if after != before:
        raise PreparationError(f"{label} changed during approval preparation")


def _json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PreparationError(f"invalid {label} JSON: {error}") from error
    if not isinstance(value, dict):
        raise PreparationError(f"{label} must be a JSON object")
    return value


def _canonical_json_bytes(value: Any) -> bytes:
    try:
        rendered = json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as error:
        raise PreparationError(f"cannot canonicalize evidence manifest: {error}") from error
    return (rendered + "\n").encode("utf-8")


def _read_json_lines(path: Path) -> Iterable[tuple[int, dict[str, Any]]]:
    try:
        stream = path.open("r", encoding="utf-8")
    except OSError as error:
        raise PreparationError(f"cannot open time samples: {error}") from error
    with stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise PreparationError(f"empty time sample line {line_number}")
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise PreparationError(
                    f"invalid time sample JSON on line {line_number}: {error}"
                ) from error
            if not isinstance(value, dict):
                raise PreparationError(
                    f"time sample line {line_number} must be a JSON object"
                )
            yield line_number, value


def _verified_identity(identity_directory: Path) -> tuple[dict[str, str], dict[str, Any]]:
    identity_directory = _absolute_without_resolving(identity_directory)
    pcap = _regular_evidence_file(
        identity_directory / PCAP_FILENAME, "identity PCAP", MAX_PCAP_BYTES
    )
    pcap_snapshot = _evidence_snapshot(pcap)
    writer_gids: dict[str, str] = {}
    streams: dict[str, Any] = {}
    for stream, topic in EXPECTED_RAW_TOPICS.items():
        names = IDENTITY_FILENAMES[stream]
        topic_info = _regular_evidence_file(
            identity_directory / names["topic_info"],
            f"{stream} topic-info",
            MAX_IDENTITY_TEXT_BYTES,
        )
        correlation_path = _regular_evidence_file(
            identity_directory / names["correlation"],
            f"{stream} correlation",
            MAX_IDENTITY_TEXT_BYTES,
        )
        topic_info_snapshot = _evidence_snapshot(topic_info)
        correlation_snapshot = _evidence_snapshot(correlation_path)
        try:
            topic_info_text = topic_info.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            raise PreparationError(f"cannot read {stream} topic-info: {error}") from error
        gids = publisher_gids_from_topic_info(topic_info_text)
        if len(gids) != 1:
            raise PreparationError(
                f"{stream} topic-info must contain exactly one publisher GID"
            )
        gid = compact_gid(gids[0])
        if len(gid) != 48:
            raise PreparationError(f"{stream} writer GID must contain exactly 24 bytes")
        stored = _json_object(correlation_path, f"{stream} correlation")
        try:
            recomputed = correlate(pcap, gids[0])
        except (OSError, ValueError) as error:
            raise PreparationError(
                f"cannot recompute {stream} writer/source correlation: {error}"
            ) from error
        try:
            stored_view = _correlation_security_view(stored)
            recomputed_view = _correlation_security_view(recomputed)
        except ValueError as error:
            raise PreparationError(
                f"invalid {stream} correlation security fields: {error}"
            ) from error
        if stored_view != recomputed_view:
            raise PreparationError(
                f"{stream} correlation is not reproducible from the retained PCAP"
            )
        if recomputed.get("method") != METHOD:
            raise PreparationError(f"{stream} correlation method is not supported")
        if recomputed.get("conclusion") != CONCLUSION:
            raise PreparationError(f"{stream} raw PCAP does not prove one writer source")
        if recomputed.get("proven_source_ips") != [EXPECTED_WRITER_SOURCE]:
            raise PreparationError(
                f"{stream} writer source is not exactly {EXPECTED_WRITER_SOURCE}"
            )
        writer_gids[stream] = gid
        streams[stream] = {
            "topic": topic,
            "writer_gid": gid,
            "proven_source_ipv4": EXPECTED_WRITER_SOURCE,
            "topic_info": {
                "file": topic_info.name,
                "sha256": topic_info_snapshot[4],
            },
            "correlation": {
                "file": correlation_path.name,
                "sha256": correlation_snapshot[4],
                "method": METHOD,
                "conclusion": CONCLUSION,
            },
        }
        _require_unchanged(topic_info, topic_info_snapshot, f"{stream} topic-info")
        _require_unchanged(
            correlation_path, correlation_snapshot, f"{stream} correlation"
        )
    _require_unchanged(pcap, pcap_snapshot, "identity PCAP")
    if len(set(writer_gids.values())) != len(EXPECTED_RAW_TOPICS):
        raise PreparationError("the four required streams must have distinct writer GIDs")
    return writer_gids, {
        "pcap": {"file": pcap.name, "sha256": pcap_snapshot[4]},
        "streams": streams,
    }


def _validate_metadata(metadata: dict[str, Any]) -> None:
    if metadata.get("schema_version") != 1:
        raise PreparationError("time metadata schema_version must be 1")
    if metadata.get("mode") != "read-only-no-adjust":
        raise PreparationError("time metadata mode must be read-only-no-adjust")
    for key in (
        "clock_adjustment_requested",
        "ros_publishers_created",
        "unitree_clients_created",
    ):
        if metadata.get(key) is not False:
            raise PreparationError(f"time metadata {key} must be false")
    if metadata.get("topics") != EXPECTED_TIME_TOPICS:
        raise PreparationError("time metadata topics do not match the five-stream probe")
    if metadata.get("stream_order") != list(EXPECTED_TIME_TOPICS):
        raise PreparationError("time metadata stream_order does not match the probe")
    if metadata.get("qualification_streams") != EXPECTED_QUALIFICATION_STREAMS:
        raise PreparationError(
            "time metadata qualification_streams do not match the four-stream gate"
        )
    if metadata.get("witness_streams") != EXPECTED_WITNESS_STREAMS:
        raise PreparationError(
            "time metadata witness_streams must contain only sport_fallback"
        )
    duration = _exact_int(metadata.get("duration_seconds"), "duration_seconds", 1)
    if duration < MINIMUM_CAPTURE_SECONDS:
        raise PreparationError(
            f"time evidence must be configured for at least {MINIMUM_CAPTURE_SECONDS} seconds"
        )
    _exact_int(metadata.get("started_realtime_ns"), "started_realtime_ns", 1)
    _exact_int(metadata.get("started_monotonic_ns"), "started_monotonic_ns", 1)
    started_span = _exact_int(
        metadata.get("started_clock_read_span_ns"),
        "started_clock_read_span_ns",
        0,
    )
    if started_span > go2_workstation_nomotion_config().max_clock_read_span_ns:
        raise PreparationError("started clock read span exceeds the qualification limit")
    _exact_int(metadata.get("max_samples"), "max_samples", 1)
    _exact_int(
        metadata.get("retained_offsets_per_stream"),
        "retained_offsets_per_stream",
        100,
    )


def _replay_time_evidence(
    time_directory: Path,
) -> tuple[int, dict[str, Any]]:
    time_directory = _absolute_without_resolving(time_directory)
    paths = {
        name: _regular_evidence_file(
            time_directory / name,
            f"time evidence {name}",
            MAX_TIME_EVIDENCE_BYTES if name == "samples.jsonl" else MAX_IDENTITY_TEXT_BYTES,
        )
        for name in TIME_FILENAMES
    }
    snapshots = {name: _evidence_snapshot(path) for name, path in paths.items()}
    metadata = _json_object(paths["metadata.json"], "time metadata")
    summary = _json_object(paths["summary.json"], "time summary")
    _validate_metadata(metadata)

    retained_limit = _exact_int(
        metadata["retained_offsets_per_stream"],
        "retained_offsets_per_stream",
        100,
    )
    trackers = {
        stream: StreamTracker(stream, topic, retained_offset_limit=retained_limit)
        for stream, topic in EXPECTED_TIME_TOPICS.items()
    }
    configured_duration_ns = metadata["duration_seconds"] * NS
    qualification_start_monotonic_ns = (
        metadata["started_monotonic_ns"]
        + configured_duration_ns
        - 2 * AFFINE_DRIFT_WINDOW_NS
    )
    discipline = AffineNavigationStampDiscipline(
        go2_workstation_nomotion_config(EXPECTED_CLOCK_DOMAIN),
        core_streams=AFFINE_CORE_STREAMS,
        qualification_start_monotonic_ns=qualification_start_monotonic_ns,
    )
    total_records = 0
    latest_required_monotonic_ns = 0
    for line_number, record in _read_json_lines(paths["samples.jsonl"]):
        total_records += 1
        stream = record.get("stream")
        if stream not in EXPECTED_TIME_TOPICS:
            raise PreparationError(f"unexpected stream on time sample line {line_number}")
        if record.get("topic") != EXPECTED_TIME_TOPICS[stream]:
            raise PreparationError(f"topic mismatch on time sample line {line_number}")
        source_ns = record.get("source_stamp_ns")
        receipt_realtime_ns = _exact_int(
            record.get("receipt_realtime_ns"),
            f"receipt_realtime_ns line {line_number}",
            1,
        )
        receipt_monotonic_ns = _exact_int(
            record.get("receipt_monotonic_ns"),
            f"receipt_monotonic_ns line {line_number}",
            1,
        )
        clock_read_span_ns = _exact_int(
            record.get("clock_read_span_ns"),
            f"clock_read_span_ns line {line_number}",
            0,
        )
        if source_ns is None:
            seconds, nanoseconds = 0, 0
        else:
            source_ns = _exact_int(
                source_ns, f"source_stamp_ns line {line_number}", 1
            )
            seconds, nanoseconds = divmod(source_ns, NS)
        reconstructed = trackers[stream].observe(
            seconds,
            nanoseconds,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_read_span_ns,
        ).as_dict()
        if reconstructed != record:
            raise PreparationError(
                f"time sample line {line_number} does not match reconstruction"
            )
        if stream not in EXPECTED_RAW_TOPICS:
            continue
        if reconstructed["status"] in {"zero", "malformed", "regression"}:
            raise PreparationError(
                f"unsafe timestamp status {reconstructed['status']} in {stream}"
            )
        assert source_ns is not None
        result = discipline.observe(
            stream,
            source_ns,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            clock_domain=EXPECTED_CLOCK_DOMAIN,
            clock_read_span_ns=clock_read_span_ns,
        )
        if result.state == DisciplineState.FAULTED:
            raise PreparationError(
                f"time replay faulted at line {line_number}: {result.reason}"
            )
        latest_required_monotonic_ns = max(
            latest_required_monotonic_ns, receipt_monotonic_ns
        )

    if total_records != _exact_int(summary.get("total_samples"), "total_samples", 1):
        raise PreparationError("time summary total_samples does not match samples.jsonl")
    if total_records > _exact_int(metadata["max_samples"], "max_samples", 1):
        raise PreparationError("time evidence exceeds its configured max_samples")
    if summary.get("schema_version") != 1:
        raise PreparationError("time summary schema_version must be 1")
    if summary.get("mode") != "read-only-no-adjust":
        raise PreparationError("time summary mode must be read-only-no-adjust")
    if summary.get("exit_reason") != "duration_elapsed":
        raise PreparationError("time probe did not complete its configured duration")
    if summary.get("cleanup_errors") != []:
        raise PreparationError("time probe cleanup_errors must be empty")
    if summary.get("safe_for_clock_discipline") is not False:
        raise PreparationError("time summary safety sentinel must remain false")
    if summary.get("started_realtime_ns") != metadata["started_realtime_ns"]:
        raise PreparationError("time metadata/summary start realtime mismatch")
    configured_duration_ns = _exact_int(
        summary.get("configured_duration_ns"), "configured_duration_ns", 1
    )
    if configured_duration_ns != metadata["duration_seconds"] * NS:
        raise PreparationError("configured duration does not match metadata")
    elapsed_ns = _exact_int(
        summary.get("elapsed_monotonic_ns"), "elapsed_monotonic_ns", 1
    )
    if elapsed_ns < configured_duration_ns:
        raise PreparationError("time probe elapsed duration is incomplete")
    finished_realtime_ns = _exact_int(
        summary.get("finished_realtime_ns"), "finished_realtime_ns", 1
    )
    finished_span = _exact_int(
        summary.get("finished_clock_read_span_ns"),
        "finished_clock_read_span_ns",
        0,
    )
    config = go2_workstation_nomotion_config()
    if finished_span > config.max_clock_read_span_ns:
        raise PreparationError("finished clock read span exceeds the qualification limit")
    realtime_elapsed_ns = finished_realtime_ns - metadata["started_realtime_ns"]
    if abs(realtime_elapsed_ns - elapsed_ns) > config.max_clock_pair_discontinuity_ns:
        raise PreparationError("probe boundary clocks show a realtime discontinuity")
    summary_streams = summary.get("streams")
    if not isinstance(summary_streams, list) or len(summary_streams) != len(trackers):
        raise PreparationError("time summary must contain every probe stream exactly once")
    stored_streams: dict[str, dict[str, Any]] = {}
    for value in summary_streams:
        if not isinstance(value, dict) or not isinstance(value.get("stream"), str):
            raise PreparationError("invalid stream entry in time summary")
        name = value["stream"]
        if name in stored_streams:
            raise PreparationError(f"duplicate stream summary: {name}")
        stored_streams[name] = value
    if set(stored_streams) != set(trackers):
        raise PreparationError("time summary stream set does not match metadata")
    for stream, tracker in trackers.items():
        expected = tracker.summary()
        if stored_streams[stream] != expected:
            raise PreparationError(f"time summary does not match raw samples for {stream}")
        if stream in EXPECTED_RAW_TOPICS:
            for counter in ("zero", "malformed", "regressions"):
                if expected[counter] != 0:
                    raise PreparationError(f"{stream} summary contains {counter}")
    anomaly_count = sum(
        tracker.zero + tracker.malformed + tracker.duplicates + tracker.regressions
        for tracker in trackers.values()
    )
    if summary.get("timestamp_anomaly_count") != anomaly_count:
        raise PreparationError("time summary anomaly count does not match raw samples")
    qualification_end_monotonic_ns = (
        qualification_start_monotonic_ns + 2 * AFFINE_DRIFT_WINDOW_NS
    )
    qualification_now_monotonic_ns = max(
        latest_required_monotonic_ns, qualification_end_monotonic_ns
    )
    qualification_snapshot = discipline.capture_affine_qualification_snapshot(
        qualification_now_monotonic_ns
    )
    qualification_evaluation = (
        discipline.evaluate_affine_qualification_snapshot(
            qualification_snapshot
        )
    )
    reasons = list(qualification_evaluation.reasons)
    if reasons:
        raise PreparationError(
            "four-stream timestamp qualification failed: " + ",".join(reasons)
        )
    try:
        affine_model = discipline.lock_affine(
            identity_evidence_verified=True,
            now_monotonic_ns=qualification_now_monotonic_ns,
            qualification_evaluation=qualification_evaluation,
        )
    except RuntimeError as error:
        raise PreparationError(f"cannot derive affine clock evidence: {error}") from error
    reference_metrics = discipline.metrics()[discipline.config.reference_stream]
    assert reference_metrics is not None
    fixed_offset_ns = (
        reference_metrics.lower_age_ns - discipline.config.offset_guard_ns
    )
    first_window, second_window, complete_window = (
        qualification_evaluation.interval_evidence
    )
    if (
        first_window.common_drift_ppm is None
        or second_window.common_drift_ppm is None
        or complete_window.common_drift_ppm is None
    ):
        raise PreparationError("affine common drift evidence is incomplete")
    approved_common_drift_ppm = (
        first_window.common_drift_ppm + second_window.common_drift_ppm
    ) / 2.0
    window_common_deviation_ppm = abs(
        first_window.common_drift_ppm - second_window.common_drift_ppm
    )

    def interval_payload(value: AffineDriftIntervalEvidence) -> dict[str, Any]:
        return {
            "start_monotonic_ns": value.start_monotonic_ns,
            "end_monotonic_ns": value.end_monotonic_ns,
            "end_inclusive": value.end_inclusive,
            "core_stream_drifts_ppm": dict(value.core_stream_drifts_ppm),
            "core_stream_envelope_points": dict(
                value.core_stream_envelope_points
            ),
            "core_stream_first_receipt_offset_ns": dict(
                value.core_stream_first_receipt_offset_ns
            ),
            "core_stream_last_receipt_offset_ns": dict(
                value.core_stream_last_receipt_offset_ns
            ),
            "core_stream_max_receipt_gap_ns": dict(
                value.core_stream_max_receipt_gap_ns
            ),
            "common_drift_ppm": value.common_drift_ppm,
        }
    metrics: dict[str, Any] = {}
    for stream, value in discipline.metrics().items():
        assert value is not None
        metrics[stream] = {
            "samples": value.samples,
            "span_ns": value.span_ns,
            "lower_age_ns": value.lower_age_ns,
            "median_age_ns": value.median_age_ns,
            "p95_age_ns": value.p95_age_ns,
            "robust_drift_ppm": value.robust_drift_ppm,
            "duplicate_fraction": value.duplicate_fraction,
        }
    for name, path in paths.items():
        _require_unchanged(path, snapshots[name], f"time evidence {name}")
    return fixed_offset_ns, {
        "files": {
            name: {"file": paths[name].name, "sha256": snapshots[name][4]}
            for name in TIME_FILENAMES
        },
        "configured_duration_ns": configured_duration_ns,
        "elapsed_monotonic_ns": elapsed_ns,
        "qualification_reasons": [],
        "derived_fixed_local_minus_source_offset_ns": fixed_offset_ns,
        "derived_approved_affine_common_drift_ppm": (
            approved_common_drift_ppm
        ),
        "affine_drift_qualification": {
            "algorithm": AFFINE_DRIFT_ALGORITHM,
            "core_streams": list(AFFINE_CORE_STREAMS),
            "window_ns": AFFINE_DRIFT_WINDOW_NS,
            "first_window": interval_payload(first_window),
            "second_window": interval_payload(second_window),
            "complete_window": interval_payload(complete_window),
            "complete_window_model_common_drift_ppm": affine_model.drift_ppm,
            "window_common_drift_deviation_ppm": (
                window_common_deviation_ppm
            ),
            "max_window_common_drift_deviation_ppm": (
                discipline.config.max_affine_window_common_drift_deviation_ppm
            ),
        },
        "stream_metrics": metrics,
    }


def build_evidence_manifest(
    *,
    time_evidence_dir: Path,
    identity_evidence_dir: Path,
    session_id: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    writer_gids, identity = _verified_identity(identity_evidence_dir)
    fixed_offset_ns, timing = _replay_time_evidence(time_evidence_dir)
    manifest = {
        "schema": "robonix-go2-workstation-nomotion-evidence-manifest-v1",
        "session_id": session_id,
        "motion_enabled": False,
        "expected_clock_domain": EXPECTED_CLOCK_DOMAIN,
        "writer_source_ipv4": EXPECTED_WRITER_SOURCE,
        "identity": identity,
        "timing": timing,
    }
    derived = {
        "writer_gids": writer_gids,
        "fixed_offset_ns": fixed_offset_ns,
        "approved_affine_common_drift_ppm": timing[
            "derived_approved_affine_common_drift_ppm"
        ],
        "affine_window_common_drifts_ppm": {
            "first": timing["affine_drift_qualification"]["first_window"][
                "common_drift_ppm"
            ],
            "second": timing["affine_drift_qualification"]["second_window"][
                "common_drift_ppm"
            ],
        },
        "affine_window_common_drift_deviation_ppm": timing[
            "affine_drift_qualification"
        ]["window_common_drift_deviation_ppm"],
    }
    return manifest, derived


def _write_exclusive_private(path: Path, content: bytes) -> None:
    path = _absolute_without_resolving(path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    descriptor = os.open(path, flags, 0o600)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        path.unlink(missing_ok=True)
        raise
    finally:
        os.close(descriptor)


def _publish_non_overwriting(path: Path, content: bytes) -> tuple[int, int]:
    path = _absolute_without_resolving(path)
    temporary = path.parent / f".{path.name}.preparing-{os.getpid()}-{time.monotonic_ns()}"
    _write_exclusive_private(temporary, content)
    try:
        os.link(temporary, path)
        published = os.lstat(path)
        return published.st_dev, published.st_ino
    finally:
        temporary.unlink(missing_ok=True)


def _unlink_if_same(path: Path, identity: tuple[int, int] | None) -> None:
    if identity is None:
        return
    try:
        current = os.lstat(path)
    except FileNotFoundError:
        return
    if (current.st_dev, current.st_ino) == identity:
        path.unlink()


def prepare_approval(
    *,
    time_evidence_dir: Path,
    identity_evidence_dir: Path,
    session_id: str,
    operator_ack: str,
    validity_seconds: int,
    output: Path,
    manifest_output: Path | None = None,
    now_realtime_ns: int | None = None,
) -> tuple[Path, Path]:
    if operator_ack != ACK:
        raise PreparationError("operator acknowledgement does not match exactly")
    if not isinstance(validity_seconds, int) or isinstance(validity_seconds, bool):
        raise PreparationError("validity_seconds must be an integer")
    if not 60 <= validity_seconds <= 3600:
        raise PreparationError("validity_seconds must be in 60..3600")
    output = _validated_output_path(output, "approval output")
    if manifest_output is None:
        manifest_output = output.with_name(output.name + ".evidence-manifest.json")
    else:
        manifest_output = _validated_output_path(
            manifest_output, "evidence manifest output"
        )
    if output == manifest_output:
        raise PreparationError("approval and evidence manifest outputs must differ")
    if os.path.lexists(output) or os.path.lexists(manifest_output):
        raise FileExistsError("refusing to overwrite approval or evidence manifest")

    manifest, derived = build_evidence_manifest(
        time_evidence_dir=time_evidence_dir,
        identity_evidence_dir=identity_evidence_dir,
        session_id=session_id,
    )
    manifest_bytes = _canonical_json_bytes(manifest)
    manifest_sha256 = hashlib.sha256(manifest_bytes).hexdigest()
    now_ns = time.time_ns() if now_realtime_ns is None else now_realtime_ns
    _exact_int(now_ns, "now_realtime_ns", 1)
    approval = {
        "schema": SCHEMA,
        "session_id": session_id,
        "motion_enabled": False,
        "identity_evidence_verified": True,
        "expected_clock_domain": EXPECTED_CLOCK_DOMAIN,
        "writer_gids": derived["writer_gids"],
        "writer_source_ipv4": EXPECTED_WRITER_SOURCE,
        "offset_evidence_sha256": manifest_sha256,
        "fixed_local_minus_source_offset_ns": derived["fixed_offset_ns"],
        "affine_drift_algorithm": AFFINE_DRIFT_ALGORITHM,
        "affine_drift_window_ns": AFFINE_DRIFT_WINDOW_NS,
        "affine_window_common_drifts_ppm": derived[
            "affine_window_common_drifts_ppm"
        ],
        "approved_affine_common_drift_ppm": derived[
            "approved_affine_common_drift_ppm"
        ],
        "affine_window_common_drift_deviation_ppm": derived[
            "affine_window_common_drift_deviation_ppm"
        ],
        "not_before_unix_ns": now_ns,
        "expires_unix_ns": now_ns + validity_seconds * NS,
        "operator_ack": ACK,
    }
    approval_bytes = _canonical_json_bytes(approval)

    # Validate the exact bytes before either public filename appears.
    validation_path = output.parent / (
        f".{output.name}.validation-{os.getpid()}-{time.monotonic_ns()}"
    )
    _write_exclusive_private(validation_path, approval_bytes)
    try:
        load_approval(validation_path, now_realtime_ns=now_ns)
    except ApprovalError as error:
        raise PreparationError(f"generated approval failed validation: {error}") from error
    finally:
        validation_path.unlink(missing_ok=True)

    manifest_identity: tuple[int, int] | None = None
    approval_identity: tuple[int, int] | None = None
    try:
        manifest_identity = _publish_non_overwriting(manifest_output, manifest_bytes)
        approval_identity = _publish_non_overwriting(output, approval_bytes)
        load_approval(output, now_realtime_ns=now_ns)
        if sha256_file(manifest_output) != manifest_sha256:
            raise PreparationError("published evidence manifest digest mismatch")
    except Exception:
        _unlink_if_same(output, approval_identity)
        _unlink_if_same(manifest_output, manifest_identity)
        raise
    return output, manifest_output


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Offline derivation of one short-lived workstation no-motion "
            "fixed-offset approval"
        )
    )
    parser.add_argument("--time-evidence-dir", required=True, type=Path)
    parser.add_argument("--identity-evidence-dir", required=True, type=Path)
    parser.add_argument("--session-id", required=True)
    parser.add_argument(
        "--operator-ack",
        required=True,
        help=f"must be exactly: {ACK}",
    )
    parser.add_argument(
        "--valid-for-seconds",
        type=int,
        default=900,
        help="short validity window, 60..3600 seconds (default: 900)",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--manifest-output", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        approval, manifest = prepare_approval(
            time_evidence_dir=arguments.time_evidence_dir,
            identity_evidence_dir=arguments.identity_evidence_dir,
            session_id=arguments.session_id,
            operator_ack=arguments.operator_ack,
            validity_seconds=arguments.valid_for_seconds,
            output=arguments.output,
            manifest_output=arguments.manifest_output,
        )
    except (OSError, ValueError, RuntimeError) as error:
        print(f"approval preparation failed: {error}", file=sys.stderr)
        return 2
    print(f"workstation no-motion approval prepared: {approval}")
    print(f"deterministic evidence manifest: {manifest}")
    print("motion remains disabled; no ROS, network, or robot operation was performed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
