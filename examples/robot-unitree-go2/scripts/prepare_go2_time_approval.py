#!/usr/bin/env python3
"""Build a reviewable clock-reference approval from original evidence.

This is an offline transformation.  It neither contacts ROS nor activates the
chrony bridge.  It refuses to trust a correlation JSON on its own: every
writer GID is reread from topic-info and every RTPS correlation is recomputed
from the classic PCAP before a schema-v3 approval is emitted.  The two-hour
stability run and three physical cold-boot trials remain direct, immutable
files in the same evidence directory and are revalidated before the output is
published.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deploy" / "time-sync"))
sys.path.insert(0, str(ROOT / "scripts"))

from evidence_bundle import (  # noqa: E402
    ALLOWED_TOPICS,
    CONCLUSION,
    MINIMUM_STABILITY_DURATION_NS,
    METHOD,
    PRIMARY_TOPIC,
    SCHEMA_VERSION,
    STABILITY_MAX_ABSOLUTE_DRIFT_PPM,
    STABILITY_MAX_DUPLICATE_FRACTION,
    STABILITY_MAX_JITTER_P95_NS,
    _correlation_security_view,
    _eligible_source,
    compact_gid,
    post_bootstrap_policy,
    publisher_gids_from_topic_info,
    sha256_file,
    verify_bundle,
)
from correlate_rtps_writer_locator import correlate  # noqa: E402


def _mapping(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("expected TOPIC=CORRELATION_JSON")
    topic, path = value.split("=", 1)
    if topic not in ALLOWED_TOPICS:
        raise argparse.ArgumentTypeError(f"unsupported clock topic: {topic}")
    if not path:
        raise argparse.ArgumentTypeError("correlation path is empty")
    return topic, Path(path)


def _cold_boot_trial(value: str) -> dict[str, Any]:
    """Parse one deliberately explicit physical cold-boot attestation."""

    fields = value.split(",")
    if len(fields) != 7:
        raise argparse.ArgumentTypeError(
            "expected TRIAL_ID,CURRENT,ATTESTATION,BOOT_ID,PCAP,TOPIC_INFO,CORRELATION"
        )
    (
        trial_id,
        current_text,
        attestation,
        boot_id_file,
        pcap_file,
        topic_info_file,
        correlation_file,
    ) = fields
    if current_text not in {"true", "false"}:
        raise argparse.ArgumentTypeError("CURRENT must be exactly true or false")
    if not all(fields):
        raise argparse.ArgumentTypeError("cold-boot trial fields cannot be empty")
    return {
        "trial_id": trial_id,
        "current_session": current_text == "true",
        "operator_attestation": attestation,
        "boot_id_file": Path(boot_id_file),
        "pcap_file": Path(pcap_file),
        "topic_info_file": Path(topic_info_file),
        "correlation_file": Path(correlation_file),
    }


def _json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid {label} JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object: {path}")
    return value


def approved_writer(
    topic: str, topic_info_path: Path, correlation_path: Path, pcap_path: Path
) -> dict[str, Any]:
    topic_info_path = topic_info_path.expanduser().resolve()
    correlation_path = correlation_path.expanduser().resolve()
    pcap_path = pcap_path.expanduser().resolve()
    gids = publisher_gids_from_topic_info(
        topic_info_path.read_text(encoding="utf-8")
    )
    if len(gids) != 1:
        raise ValueError(
            f"topic-info must contain exactly one publisher GID for {topic}"
        )
    gid = compact_gid(gids[0])
    try:
        correlation = json.loads(correlation_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(
            f"invalid correlation JSON {correlation_path}: {error}"
        ) from error
    recomputed = correlate(pcap_path, gids[0])
    if _correlation_security_view(correlation) != _correlation_security_view(recomputed):
        raise ValueError(f"correlation is not reproducible from PCAP for {topic}")
    if recomputed.get("method") != METHOD or recomputed.get("conclusion") != CONCLUSION:
        raise ValueError(f"raw evidence does not prove one writer source for {topic}")
    sources = recomputed.get("proven_source_ips")
    if not isinstance(sources, list) or len(sources) != 1:
        raise ValueError(f"raw evidence must contain one proven source for {topic}")
    source_ip = _eligible_source(sources[0])
    return {
        "topic": topic,
        "writer_gid": gid,
        "rtps_participant_guid_prefix": recomputed["rtps_participant_guid_prefix"],
        "source_ip": source_ip,
        "correlation_method": METHOD,
        "correlation_conclusion": CONCLUSION,
        "pcap_sha256": sha256_file(pcap_path),
        "topic_info_file": topic_info_path.name,
        "topic_info_sha256": sha256_file(topic_info_path),
        "correlation_file": correlation_path.name,
        "correlation_sha256": sha256_file(correlation_path),
    }


def write_private_json(path: Path, payload: dict[str, Any]) -> None:
    resolved = path.expanduser().resolve()
    resolved.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    if resolved.exists():
        raise FileExistsError(f"refusing to overwrite: {resolved}")
    with resolved.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.chmod(resolved, 0o600)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Prepare, but do not activate, Go2 clock-source approval"
    )
    parser.add_argument(
        "--topic-correlation",
        action="append",
        required=True,
        type=_mapping,
        metavar="TOPIC=JSON",
    )
    parser.add_argument(
        "--topic-info",
        action="append",
        required=True,
        type=_mapping,
        metavar="TOPIC=TOPIC_INFO_TXT",
    )
    parser.add_argument("--pcap", required=True, type=Path)
    parser.add_argument("--stability-metadata", required=True, type=Path)
    parser.add_argument("--stability-summary", required=True, type=Path)
    parser.add_argument(
        "--cold-boot-trial",
        action="append",
        required=True,
        type=_cold_boot_trial,
        metavar=(
            "TRIAL_ID,CURRENT,ATTESTATION,BOOT_ID,PCAP,TOPIC_INFO,CORRELATION"
        ),
    )
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    output_parent = arguments.output.expanduser().resolve().parent
    pcap_path = arguments.pcap.expanduser().resolve()
    stability_metadata_path = arguments.stability_metadata.expanduser().resolve()
    stability_summary_path = arguments.stability_summary.expanduser().resolve()
    correlations = dict(arguments.topic_correlation)
    topic_infos = dict(arguments.topic_info)
    if len(correlations) != len(arguments.topic_correlation):
        print("approval preparation failed: duplicate correlation topic", file=sys.stderr)
        return 2
    if len(topic_infos) != len(arguments.topic_info):
        print("approval preparation failed: duplicate topic-info topic", file=sys.stderr)
        return 2
    if set(correlations) != set(topic_infos):
        print(
            "approval preparation failed: topic-info/correlation topics differ",
            file=sys.stderr,
        )
        return 2
    evidence_paths = [pcap_path, stability_metadata_path, stability_summary_path]
    evidence_paths.extend(path.expanduser().resolve() for path in correlations.values())
    evidence_paths.extend(path.expanduser().resolve() for path in topic_infos.values())
    for trial in arguments.cold_boot_trial:
        evidence_paths.extend(
            trial[key].expanduser().resolve()
            for key in (
                "boot_id_file",
                "pcap_file",
                "topic_info_file",
                "correlation_file",
            )
        )
    if any(path.parent != output_parent for path in evidence_paths):
        print(
            "approval preparation failed: all evidence and output must be direct files in one bundle directory",
            file=sys.stderr,
        )
        return 2
    writers: list[dict[str, Any]] = []
    try:
        for topic in sorted(correlations):
            writers.append(
                approved_writer(
                    topic, topic_infos[topic], correlations[topic], pcap_path
                )
            )
        current_primary = next(
            (writer for writer in writers if writer["topic"] == PRIMARY_TOPIC), None
        )
        if current_primary is None:
            raise ValueError("current primary /sportmodestate evidence is required")

        _json_object(stability_metadata_path, "stability metadata")
        stability_summary = _json_object(stability_summary_path, "stability summary")
        observed_duration_ns = stability_summary.get("elapsed_monotonic_ns")
        if isinstance(observed_duration_ns, bool) or not isinstance(
            observed_duration_ns, int
        ):
            raise ValueError("stability summary elapsed_monotonic_ns must be an integer")
        stability = {
            "metadata_file": stability_metadata_path.name,
            "metadata_sha256": sha256_file(stability_metadata_path),
            "summary_file": stability_summary_path.name,
            "summary_sha256": sha256_file(stability_summary_path),
            "minimum_duration_ns": MINIMUM_STABILITY_DURATION_NS,
            "observed_duration_ns": observed_duration_ns,
            "writer_gid": current_primary["writer_gid"],
            "maximum_jitter_p95_ns": STABILITY_MAX_JITTER_P95_NS,
            "maximum_absolute_drift_ppm": STABILITY_MAX_ABSOLUTE_DRIFT_PPM,
            "maximum_duplicate_fraction": STABILITY_MAX_DUPLICATE_FRACTION,
        }

        cold_boot_trials: list[dict[str, Any]] = []
        for trial in arguments.cold_boot_trial:
            boot_id_path = trial["boot_id_file"].expanduser().resolve()
            trial_pcap_path = trial["pcap_file"].expanduser().resolve()
            trial_topic_info_path = trial["topic_info_file"].expanduser().resolve()
            trial_correlation_path = trial["correlation_file"].expanduser().resolve()
            boot_id = boot_id_path.read_text(encoding="utf-8").strip().lower()
            trial_writer = approved_writer(
                PRIMARY_TOPIC,
                trial_topic_info_path,
                trial_correlation_path,
                trial_pcap_path,
            )
            cold_boot_trials.append(
                {
                    "trial_id": trial["trial_id"],
                    "operator_attestation": trial["operator_attestation"],
                    "current_session": trial["current_session"],
                    "boot_id_file": boot_id_path.name,
                    "boot_id_sha256": sha256_file(boot_id_path),
                    "boot_id": boot_id,
                    "pcap_file": trial_pcap_path.name,
                    **trial_writer,
                }
            )
        payload = {
            "schema_version": SCHEMA_VERSION,
            "purpose": "go2-source-clock-to-chrony-refclock",
            "activation_authorized": False,
            "evidence_bundle": {
                "pcap_file": pcap_path.name,
                "pcap_sha256": sha256_file(pcap_path),
            },
            "approved_writers": writers,
            "pre_bootstrap_stability": stability,
            "cold_boot_identity_trials": cold_boot_trials,
            "post_bootstrap_quality_gate": post_bootstrap_policy(),
            "note": (
                "This non-authorizing file binds original evidence; it does not grant "
                "SYS_TIME or activate clock discipline."
            ),
        }
        output_path = arguments.output.expanduser().resolve()
        if output_path.exists():
            raise FileExistsError(f"refusing to overwrite: {output_path}")
        temporary_path = output_parent / f".{output_path.name}.preparing-{os.getpid()}"
        try:
            write_private_json(temporary_path, payload)
            verify_bundle(output_parent, temporary_path.name)
            temporary_path.replace(output_path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise
    except (OSError, ValueError) as error:
        print(f"approval preparation failed: {error}", file=sys.stderr)
        return 2
    print(f"inactive schema-v3 approval prepared at {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
