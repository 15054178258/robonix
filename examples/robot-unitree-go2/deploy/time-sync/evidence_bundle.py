#!/usr/bin/env python3
"""Fail-closed validation for a Go2 clock evidence bundle.

Schema v3 binds four independently reviewable things before a feeder can see
an approval:

* the current live writer's topic-info and raw RTPS PCAP correlation;
* a read-only stability run lasting at least two monotonic hours;
* three distinct cold-boot identity trials, each recomputed from raw PCAP;
* the exact post-bootstrap five-minute quality policy.

The post-bootstrap *result* cannot exist before the first clock step.  The
approval therefore commits to the exact policy and audit filenames; the
sidecar enforces it live and emits the result before reporting navigation-time
readiness.  This module is offline: it opens regular files only and never
initializes ROS, captures packets, changes a clock, or invokes a subprocess.
"""

from __future__ import annotations

import hashlib
import ipaddress
import json
from pathlib import Path
import re
import stat
import sys
from typing import Any


THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from correlate_rtps_writer_locator import correlate  # noqa: E402


SCHEMA_VERSION = 3
METHOD = "pcap_rtps_guid_prefix_and_data_writer_entity_correlation"
CONCLUSION = "single_source_proven_by_rtps_data_writer"
ALLOWED_TOPICS = {"/sportmodestate", "/lf/sportmodestate"}
PRIMARY_TOPIC = "/sportmodestate"
GO2_NETWORK = ipaddress.ip_network("192.168.123.0/24")
DISALLOWED_SOURCE_IPS = {
    ipaddress.ip_address("192.168.123.18"),
    ipaddress.ip_address("192.168.123.99"),
    GO2_NETWORK.network_address,
    GO2_NETWORK.broadcast_address,
}
MAX_PCAP_BYTES = 512 * 1024 * 1024
MAX_TEXT_BYTES = 2 * 1024 * 1024

MINIMUM_STABILITY_DURATION_NS = 2 * 60 * 60 * 1_000_000_000
MINIMUM_COLD_BOOT_TRIALS = 3
STABILITY_MAX_JITTER_P95_NS = 20_000_000
STABILITY_MAX_ABSOLUTE_DRIFT_PPM = 500.0
STABILITY_MAX_DUPLICATE_FRACTION = 0.10
STABILITY_REQUIRED_STREAMS = (
    ("sport_primary", "/sportmodestate"),
    ("mid360_cloud", "/utlidar/cloud"),
    ("mid360_imu", "/utlidar/imu"),
)

POST_BOOTSTRAP_MINIMUM_CONTINUOUS_DURATION_NS = 5 * 60 * 1_000_000_000
POST_BOOTSTRAP_MAX_ABSOLUTE_OFFSET_NS_EXCLUSIVE = 50_000_000
POST_BOOTSTRAP_MAX_ABSOLUTE_DRIFT_PPM_EXCLUSIVE = 100.0
POST_BOOTSTRAP_MAXIMUM_OBSERVATION_DURATION_NS = 15 * 60 * 1_000_000_000
POST_BOOTSTRAP_AUDIT_FILE = "post-bootstrap-quality.json"
POST_BOOTSTRAP_SAMPLES_FILE = "post-bootstrap-quality-samples.jsonl"

_TRIAL_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_BOOT_ID = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def compact_gid(value: str) -> str:
    compact = re.sub(r"[.\-:\s]", "", value).lower()
    if len(compact) < 32 or len(compact) % 2:
        raise ValueError("writer GID must contain at least 16 hexadecimal bytes")
    try:
        bytes.fromhex(compact)
    except ValueError as error:
        raise ValueError("writer GID contains non-hexadecimal characters") from error
    return compact


def publisher_gids_from_topic_info(text: str) -> list[str]:
    """Extract publisher endpoint GIDs from ``ros2 topic info --verbose``."""

    result: list[str] = []
    publisher = False
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line.lower().startswith("endpoint type:"):
            publisher = line.split(":", 1)[1].strip().upper() == "PUBLISHER"
            continue
        if publisher and line.lower().startswith("gid:"):
            result.append(line.split(":", 1)[1].strip())
            publisher = False
    return result


def _regular_file(bundle: Path, relative: Any, label: str, limit: int) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).name != relative:
        raise ValueError(f"{label} must be one plain filename inside the bundle")
    path = bundle / relative
    file_stat = path.lstat()
    if not stat.S_ISREG(file_stat.st_mode):
        raise ValueError(f"{label} is not a regular file: {relative}")
    if file_stat.st_mode & 0o022:
        raise ValueError(f"{label} must not be group/world writable: {relative}")
    if file_stat.st_size <= 0 or file_stat.st_size > limit:
        raise ValueError(f"{label} size is outside the accepted bound: {relative}")
    return path


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid {label} JSON: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _exact_digest(path: Path, expected: Any, label: str) -> str:
    actual = sha256_file(path)
    if expected != actual:
        raise ValueError(f"{label} SHA-256 mismatch")
    return actual


def _integer(value: Any, label: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{label} must be an integer >= {minimum}")
    return value


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if result != result or result in (float("inf"), float("-inf")):
        raise ValueError(f"{label} must be finite")
    return result


def _correlation_security_view(value: dict[str, Any]) -> dict[str, Any]:
    """Fields that are independently derivable from raw capture bytes."""

    return {
        "schema_version": value.get("schema_version"),
        "method": value.get("method"),
        "writer_gid": compact_gid(str(value.get("writer_gid", ""))),
        "rtps_participant_guid_prefix": value.get("rtps_participant_guid_prefix"),
        "rtps_writer_entity_id": value.get("rtps_writer_entity_id"),
        "total_packets": value.get("total_packets"),
        "decoded_ipv4_udp_packets": value.get("decoded_ipv4_udp_packets"),
        "participant_prefix_sources": value.get("participant_prefix_sources"),
        "writer_data_sources": value.get("writer_data_sources"),
        "proven_source_ips": value.get("proven_source_ips"),
        "conclusion": value.get("conclusion"),
    }


def _eligible_source(value: Any) -> str:
    try:
        address = ipaddress.ip_address(str(value))
    except ValueError as error:
        raise ValueError("correlation has an invalid source IP") from error
    if address not in GO2_NETWORK or address in DISALLOWED_SOURCE_IPS:
        raise ValueError(f"source IP is not an eligible Go2 host: {address}")
    return str(address)


def _verify_writer(
    bundle: Path,
    item: dict[str, Any],
    pcap_path: Path,
    pcap_sha256: str,
    label: str,
) -> dict[str, Any]:
    topic = item.get("topic")
    if topic not in ALLOWED_TOPICS:
        raise ValueError(f"{label} has unsupported topic: {topic!r}")
    topic_info_path = _regular_file(
        bundle, item.get("topic_info_file"), f"{label} topic-info", MAX_TEXT_BYTES
    )
    correlation_path = _regular_file(
        bundle, item.get("correlation_file"), f"{label} correlation", MAX_TEXT_BYTES
    )
    topic_info_sha256 = _exact_digest(
        topic_info_path, item.get("topic_info_sha256"), f"{label} topic-info"
    )
    correlation_sha256 = _exact_digest(
        correlation_path, item.get("correlation_sha256"), f"{label} correlation"
    )
    gids = publisher_gids_from_topic_info(topic_info_path.read_text(encoding="utf-8"))
    if len(gids) != 1:
        raise ValueError(f"{label} topic-info must contain exactly one publisher GID")
    gid = compact_gid(gids[0])
    stored_correlation = _load_json(correlation_path, f"{label} correlation")
    recomputed = correlate(pcap_path, gids[0])
    if _correlation_security_view(stored_correlation) != _correlation_security_view(
        recomputed
    ):
        raise ValueError(f"{label} stored correlation is not reproducible")
    if recomputed.get("method") != METHOD or recomputed.get("conclusion") != CONCLUSION:
        raise ValueError(f"{label} raw evidence does not prove one writer source")
    sources = recomputed.get("proven_source_ips")
    if not isinstance(sources, list) or len(sources) != 1:
        raise ValueError(f"{label} raw evidence does not have one source IP")
    source_ip = _eligible_source(sources[0])
    expected = {
        "topic": topic,
        "writer_gid": gid,
        "rtps_participant_guid_prefix": str(
            recomputed["rtps_participant_guid_prefix"]
        ).lower(),
        "source_ip": source_ip,
        "correlation_method": METHOD,
        "correlation_conclusion": CONCLUSION,
        "pcap_sha256": pcap_sha256,
        "topic_info_file": topic_info_path.name,
        "topic_info_sha256": topic_info_sha256,
        "correlation_file": correlation_path.name,
        "correlation_sha256": correlation_sha256,
    }
    for key, expected_value in expected.items():
        actual = item.get(key)
        if key == "writer_gid":
            actual = compact_gid(str(actual))
        if actual != expected_value:
            raise ValueError(f"{label} {key} mismatch")
    return expected


def _verify_stability(
    bundle: Path, value: Any, primary_writer_gid: str
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("approval requires pre_bootstrap_stability")
    metadata_path = _regular_file(
        bundle, value.get("metadata_file"), "stability metadata", MAX_TEXT_BYTES
    )
    summary_path = _regular_file(
        bundle, value.get("summary_file"), "stability summary", MAX_TEXT_BYTES
    )
    metadata_sha256 = _exact_digest(
        metadata_path, value.get("metadata_sha256"), "stability metadata"
    )
    summary_sha256 = _exact_digest(
        summary_path, value.get("summary_sha256"), "stability summary"
    )
    metadata = _load_json(metadata_path, "stability metadata")
    summary = _load_json(summary_path, "stability summary")

    if metadata.get("mode") != "read-only-no-adjust" or summary.get(
        "mode"
    ) != "read-only-no-adjust":
        raise ValueError("stability evidence must be read-only-no-adjust")
    for key in (
        "clock_adjustment_requested",
        "ros_publishers_created",
        "unitree_clients_created",
    ):
        if metadata.get(key) is not False:
            raise ValueError(f"stability metadata {key} must be false")
    configured_seconds = _integer(
        metadata.get("duration_seconds"), "stability configured duration", 1
    )
    if configured_seconds * 1_000_000_000 < MINIMUM_STABILITY_DURATION_NS:
        raise ValueError("stability configured duration is shorter than two hours")
    elapsed_ns = _integer(
        summary.get("elapsed_monotonic_ns"), "stability elapsed duration", 1
    )
    if elapsed_ns < MINIMUM_STABILITY_DURATION_NS:
        raise ValueError("stability observed duration is shorter than two hours")
    if summary.get("exit_reason") != "duration_elapsed":
        raise ValueError("stability run did not complete its configured duration")
    if summary.get("cleanup_errors") != []:
        raise ValueError("stability run has cleanup errors")
    if summary.get("started_realtime_ns") != metadata.get("started_realtime_ns"):
        raise ValueError("stability metadata/summary session mismatch")

    topics = metadata.get("topics")
    if not isinstance(topics, dict):
        raise ValueError("stability metadata topics are missing")
    streams = summary.get("streams")
    if not isinstance(streams, list):
        raise ValueError("stability summary streams are missing")
    by_name = {
        item.get("stream"): item for item in streams if isinstance(item, dict)
    }
    verified_streams: list[dict[str, Any]] = []
    for stream_name, topic in STABILITY_REQUIRED_STREAMS:
        if topics.get(stream_name) != topic:
            raise ValueError(f"stability metadata is missing {stream_name}={topic}")
        stream = by_name.get(stream_name)
        if not isinstance(stream, dict) or stream.get("topic") != topic:
            raise ValueError(f"stability summary is missing {stream_name}={topic}")
        received = _integer(stream.get("received"), f"{stream_name} received", 1)
        valid = _integer(stream.get("valid"), f"{stream_name} valid", 1)
        duplicates = _integer(stream.get("duplicates"), f"{stream_name} duplicates")
        if valid > received or duplicates > received:
            raise ValueError(f"{stream_name} counters are inconsistent")
        for field in ("zero", "malformed", "regressions"):
            if _integer(stream.get(field), f"{stream_name} {field}") != 0:
                raise ValueError(f"{stream_name} contains {field} timestamps")
        duplicate_fraction = duplicates / received
        if duplicate_fraction > STABILITY_MAX_DUPLICATE_FRACTION:
            raise ValueError(f"{stream_name} duplicate fraction exceeds 10 percent")
        jitter = _finite_number(
            stream.get("offset_jitter_abs_deviation_p95_ns"),
            f"{stream_name} offset jitter p95",
        )
        if jitter > STABILITY_MAX_JITTER_P95_NS:
            raise ValueError(f"{stream_name} offset jitter p95 exceeds 20 ms")
        drift = _finite_number(
            stream.get("estimated_drift_ppm"), f"{stream_name} drift"
        )
        if abs(drift) > STABILITY_MAX_ABSOLUTE_DRIFT_PPM:
            raise ValueError(f"{stream_name} absolute drift exceeds 500 ppm")
        verified_streams.append(
            {
                "stream": stream_name,
                "topic": topic,
                "received": received,
                "duplicates": duplicates,
                "duplicate_fraction": duplicate_fraction,
                "offset_jitter_abs_deviation_p95_ns": jitter,
                "estimated_drift_ppm": drift,
            }
        )

    expected_fields = {
        "minimum_duration_ns": MINIMUM_STABILITY_DURATION_NS,
        "observed_duration_ns": elapsed_ns,
        "writer_gid": primary_writer_gid,
        "maximum_jitter_p95_ns": STABILITY_MAX_JITTER_P95_NS,
        "maximum_absolute_drift_ppm": STABILITY_MAX_ABSOLUTE_DRIFT_PPM,
        "maximum_duplicate_fraction": STABILITY_MAX_DUPLICATE_FRACTION,
    }
    for key, expected in expected_fields.items():
        actual = value.get(key)
        if key == "writer_gid":
            actual = compact_gid(str(actual))
        if actual != expected:
            raise ValueError(f"pre_bootstrap_stability {key} mismatch")
    return {
        "metadata_file": metadata_path.name,
        "metadata_sha256": metadata_sha256,
        "summary_file": summary_path.name,
        "summary_sha256": summary_sha256,
        **expected_fields,
        "required_streams": verified_streams,
    }


def _verify_cold_boot_trials(
    bundle: Path,
    value: Any,
    current_primary: dict[str, Any],
    current_pcap_file: str,
) -> list[dict[str, Any]]:
    if not isinstance(value, list) or len(value) < MINIMUM_COLD_BOOT_TRIALS:
        raise ValueError("at least three cold_boot_identity_trials are required")
    trial_ids: set[str] = set()
    boot_ids: set[str] = set()
    writer_gids: set[str] = set()
    pcap_digests: set[str] = set()
    current_count = 0
    verified_trials: list[dict[str, Any]] = []
    for index, trial in enumerate(value, 1):
        label = f"cold boot trial {index}"
        if not isinstance(trial, dict):
            raise ValueError(f"{label} must be an object")
        trial_id = trial.get("trial_id")
        if not isinstance(trial_id, str) or not _TRIAL_ID.fullmatch(trial_id):
            raise ValueError(f"{label} has invalid trial_id")
        if trial_id in trial_ids:
            raise ValueError("cold boot trial IDs must be distinct")
        trial_ids.add(trial_id)
        if trial.get("operator_attestation") != (
            "physical-cold-boot-observed-and-read-only"
        ):
            raise ValueError(f"{label} lacks the exact cold-boot attestation")

        boot_id_path = _regular_file(
            bundle, trial.get("boot_id_file"), f"{label} boot-id", MAX_TEXT_BYTES
        )
        boot_id_sha256 = _exact_digest(
            boot_id_path, trial.get("boot_id_sha256"), f"{label} boot-id"
        )
        boot_id = boot_id_path.read_text(encoding="utf-8").strip().lower()
        if not _BOOT_ID.fullmatch(boot_id) or trial.get("boot_id") != boot_id:
            raise ValueError(f"{label} boot ID is invalid or mismatched")
        if boot_id in boot_ids:
            raise ValueError("cold boot IDs must be distinct")
        boot_ids.add(boot_id)

        pcap_path = _regular_file(
            bundle, trial.get("pcap_file"), f"{label} PCAP", MAX_PCAP_BYTES
        )
        pcap_sha256 = _exact_digest(
            pcap_path, trial.get("pcap_sha256"), f"{label} PCAP"
        )
        if pcap_sha256 in pcap_digests:
            raise ValueError("cold boot PCAP digests must be distinct")
        pcap_digests.add(pcap_sha256)
        writer = _verify_writer(bundle, trial, pcap_path, pcap_sha256, label)
        if writer["topic"] != PRIMARY_TOPIC:
            raise ValueError(f"{label} must prove the primary sport writer")
        gid = writer["writer_gid"]
        if gid in writer_gids:
            raise ValueError("cold boot writer GIDs must be distinct")
        writer_gids.add(gid)
        if writer["source_ip"] != current_primary["source_ip"]:
            raise ValueError("cold boot trials disagree on the Go2 source IP")

        current_session = trial.get("current_session")
        if not isinstance(current_session, bool):
            raise ValueError(f"{label} current_session must be boolean")
        if current_session:
            current_count += 1
            if (
                writer["writer_gid"] != current_primary["writer_gid"]
                or writer["pcap_sha256"] != current_primary["pcap_sha256"]
                or pcap_path.name != current_pcap_file
            ):
                raise ValueError(
                    "current cold boot trial does not match current writer evidence"
                )
        verified_trials.append(
            {
                "trial_id": trial_id,
                "operator_attestation": trial["operator_attestation"],
                "current_session": current_session,
                "boot_id_file": boot_id_path.name,
                "boot_id_sha256": boot_id_sha256,
                "boot_id": boot_id,
                "pcap_file": pcap_path.name,
                **writer,
            }
        )
    if current_count != 1:
        raise ValueError("exactly one cold boot trial must be the current session")
    return verified_trials


def post_bootstrap_policy() -> dict[str, Any]:
    return {
        "reference_topic": PRIMARY_TOPIC,
        "minimum_continuous_duration_ns": (
            POST_BOOTSTRAP_MINIMUM_CONTINUOUS_DURATION_NS
        ),
        "maximum_absolute_offset_ns_exclusive": (
            POST_BOOTSTRAP_MAX_ABSOLUTE_OFFSET_NS_EXCLUSIVE
        ),
        "maximum_absolute_drift_ppm_exclusive": (
            POST_BOOTSTRAP_MAX_ABSOLUTE_DRIFT_PPM_EXCLUSIVE
        ),
        "maximum_observation_duration_ns": (
            POST_BOOTSTRAP_MAXIMUM_OBSERVATION_DURATION_NS
        ),
        "audit_file": POST_BOOTSTRAP_AUDIT_FILE,
        "samples_file": POST_BOOTSTRAP_SAMPLES_FILE,
        "require_unsafe_latched_false": True,
        "require_current_writer_authorized": True,
        "require_feed_count_advancing": True,
        "require_chronyd_go2_selected": True,
    }


def verify_post_bootstrap_policy(value: Any) -> dict[str, Any]:
    expected = post_bootstrap_policy()
    if value != expected:
        raise ValueError(
            "post_bootstrap_quality_gate must exactly match the fail-closed policy"
        )
    return expected


def verify_bundle(
    bundle: Path, approval_filename: str = "go2-clock-ref-approval.json"
) -> dict[str, Any]:
    """Rebuild all security conclusions and return a verified approval."""

    bundle = bundle.expanduser().resolve()
    bundle_stat = bundle.lstat()
    if not stat.S_ISDIR(bundle_stat.st_mode):
        raise ValueError("evidence bundle is not a directory")
    approval_path = _regular_file(
        bundle, approval_filename, "approval", MAX_TEXT_BYTES
    )
    approval = _load_json(approval_path, "approval")
    if approval.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            "legacy/incomplete approval rejected; schema_version=3 required"
        )
    if approval.get("purpose") != "go2-source-clock-to-chrony-refclock":
        raise ValueError("approval purpose is not exact")
    if approval.get("activation_authorized") is not False:
        raise ValueError("approval file must remain non-authorizing")

    evidence = approval.get("evidence_bundle")
    if not isinstance(evidence, dict):
        raise ValueError("approval is missing evidence_bundle")
    pcap_path = _regular_file(
        bundle, evidence.get("pcap_file"), "PCAP", MAX_PCAP_BYTES
    )
    pcap_sha256 = _exact_digest(
        pcap_path, evidence.get("pcap_sha256"), "PCAP"
    )

    writers = approval.get("approved_writers")
    if not isinstance(writers, list) or not writers:
        raise ValueError("approval requires approved_writers")
    seen_topics: set[str] = set()
    verified_writers: list[dict[str, Any]] = []
    for index, item in enumerate(writers, 1):
        if not isinstance(item, dict):
            raise ValueError("approved writer entry must be an object")
        topic = item.get("topic")
        if topic in seen_topics:
            raise ValueError(f"duplicate approved topic: {topic!r}")
        seen_topics.add(str(topic))
        verified_writers.append(
            _verify_writer(
                bundle, item, pcap_path, pcap_sha256, f"approved writer {index}"
            )
        )
    primary = next(
        (item for item in verified_writers if item["topic"] == PRIMARY_TOPIC), None
    )
    if primary is None:
        raise ValueError("current primary /sportmodestate writer is required")

    stability = _verify_stability(
        bundle, approval.get("pre_bootstrap_stability"), primary["writer_gid"]
    )
    cold_boot_trials = _verify_cold_boot_trials(
        bundle,
        approval.get("cold_boot_identity_trials"),
        primary,
        pcap_path.name,
    )
    policy = verify_post_bootstrap_policy(
        approval.get("post_bootstrap_quality_gate")
    )

    # Return only recomputed evidence and exact non-authorizing policy fields.
    return {
        "schema_version": SCHEMA_VERSION,
        "purpose": "go2-source-clock-to-chrony-refclock",
        "activation_authorized": False,
        "evidence_bundle": {
            "pcap_file": pcap_path.name,
            "pcap_sha256": pcap_sha256,
        },
        "approved_writers": verified_writers,
        "pre_bootstrap_stability": stability,
        "cold_boot_identity_trials": cold_boot_trials,
        "post_bootstrap_quality_gate": policy,
    }


def canonical_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
