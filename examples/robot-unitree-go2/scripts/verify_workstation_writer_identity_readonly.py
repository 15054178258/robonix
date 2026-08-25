#!/usr/bin/env python3
"""Boundedly compare the live ROS graph with explicit retained writer evidence.

The expected writer GIDs are never accepted as command-line values.  They are
re-derived from one explicitly named identity directory, including a fresh
parse of its retained PCAP/source correlation.  The ROS node is graph-only: it
creates no subscriptions, publishers, clients, services, actions, or Unitree
SDK objects.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import stat
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC_DIR = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC_DIR))
sys.path.insert(0, str(ROOT / "scripts"))

from prepare_workstation_nomotion_offset_approval import (  # noqa: E402
    PreparationError,
    _verified_identity,
)
from workstation_nomotion_approval import EXPECTED_RAW_TOPICS  # noqa: E402
from workstation_nomotion_identity_monitor import (  # noqa: E402
    endpoint_gid_hex,
    writer_identity_failures,
)


SCHEMA = "robonix-go2-workstation-writer-identity-check-v1"
POLL_INTERVAL_SECONDS = 0.25
ALLOWED_RECEIPT_ROOTS = (ROOT / "logs", ROOT / "rbnx-build")
_PHASE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{1,63}$")


class VerificationError(RuntimeError):
    """The explicit evidence or live graph cannot prove writer continuity."""


def _bounded_int(value: str, minimum: int, maximum: int, label: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{label} must be an integer") from error
    if not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{label} must be in {minimum}..{maximum}"
        )
    return parsed


def _absolute_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("evidence and receipt paths must be absolute")
    return path


def _phase(value: str) -> str:
    if _PHASE_RE.fullmatch(value) is None:
        raise argparse.ArgumentTypeError("phase contains unsupported characters")
    return value


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "READ-ONLY graph check: compare current raw-topic writer GIDs "
            "with one explicitly supplied retained identity directory"
        )
    )
    parser.add_argument(
        "--identity-evidence-dir", required=True, type=_absolute_path
    )
    parser.add_argument("--receipt", required=True, type=_absolute_path)
    parser.add_argument("--phase", required=True, type=_phase)
    parser.add_argument(
        "--discovery-seconds",
        type=lambda value: _bounded_int(value, 1, 30, "discovery-seconds"),
        default=15,
    )
    parser.add_argument(
        "--stable-samples",
        type=lambda value: _bounded_int(value, 1, 10, "stable-samples"),
        default=3,
    )
    return parser


def _private_identity_directory(path: Path) -> Path:
    absolute = Path(os.path.abspath(os.fspath(path)))
    try:
        resolved = absolute.resolve(strict=True)
        info = os.lstat(absolute)
    except OSError as error:
        raise VerificationError(f"cannot inspect identity evidence directory: {error}") from error
    if resolved != absolute or stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise VerificationError(
            "identity evidence directory must be an absolute real directory without symlinks"
        )
    if info.st_uid != os.geteuid():
        raise VerificationError("identity evidence directory must be owned by the current user")
    if stat.S_IMODE(info.st_mode) & 0o077:
        raise VerificationError(
            "identity evidence directory must not be accessible by group or others"
        )
    evidence_root = (ROOT / "logs").resolve(strict=True)
    if not absolute.is_relative_to(evidence_root):
        raise VerificationError("identity evidence directory must be below repository logs/")
    return absolute


def _new_private_receipt_path(path: Path) -> Path:
    absolute = Path(os.path.abspath(os.fspath(path)))
    try:
        parent = absolute.parent.resolve(strict=True)
        parent_info = os.lstat(parent)
    except OSError as error:
        raise VerificationError(f"cannot inspect receipt parent directory: {error}") from error
    if not stat.S_ISDIR(parent_info.st_mode) or parent_info.st_uid != os.geteuid():
        raise VerificationError("receipt parent must be a current-user-owned directory")
    if stat.S_IMODE(parent_info.st_mode) & 0o077:
        raise VerificationError(
            "receipt parent directory must not be accessible by group or others"
        )
    candidate = parent / absolute.name
    allowed_roots = tuple(root.resolve(strict=True) for root in ALLOWED_RECEIPT_ROOTS)
    if not any(candidate != root and candidate.is_relative_to(root) for root in allowed_roots):
        raise VerificationError("receipt must be below repository logs/ or rbnx-build/")
    if os.path.lexists(candidate):
        raise VerificationError(f"refusing to overwrite identity receipt: {candidate}")
    return candidate


def _write_exclusive_private(path: Path, payload: dict[str, Any]) -> None:
    encoded = (
        json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    descriptor = os.open(path, flags, 0o600)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        path.unlink(missing_ok=True)
        raise
    finally:
        os.close(descriptor)


def _observed_writer_gids(observations: dict[str, list[Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for stream in EXPECTED_RAW_TOPICS:
        endpoints = observations.get(stream, [])
        values: list[str] = []
        for endpoint in endpoints:
            try:
                values.append(endpoint_gid_hex(endpoint.endpoint_gid))
            except (AttributeError, ValueError) as error:
                values.append(f"invalid:{error}")
        result[stream] = values
    return result


def observe_live_graph(
    expected_writer_gids: dict[str, str],
    *,
    discovery_seconds: int,
    stable_samples: int,
) -> dict[str, Any]:
    """Return bounded graph evidence; ROS imports and init happen only here."""

    from rclpy.context import Context
    from rclpy.impl.implementation_singleton import (
        rclpy_implementation as _rclpy,
    )
    from rclpy.topic_endpoint_info import TopicEndpointInfo

    context = Context()
    context_initialized = False
    node_handle = None
    started_realtime_ns = 0
    started_monotonic_ns = 0
    consecutive_matches = 0
    check_count = 0
    last_failures: list[str] = ["graph_not_checked"]
    last_observations: dict[str, list[Any]] = {}
    passed = False
    try:
        context.init(args=[], initialize_logging=False)
        context_initialized = True
        # Humble's high-level Node unconditionally owns a /parameter_events
        # publisher and may create parameter services.  This low-level handle
        # disables rosout and parameter-service creation, so the verifier owns
        # no communication endpoint while it reads the graph.
        with context.handle:
            node_handle = _rclpy.Node(
                "go2_writer_identity_check_readonly",
                "",
                context.handle,
                None,
                False,
                False,
            )

        started_realtime_ns = time.time_ns()
        started_monotonic_ns = time.monotonic_ns()
        deadline_ns = started_monotonic_ns + discovery_seconds * 1_000_000_000
        while context.ok():
            if time.monotonic_ns() >= deadline_ns:
                break
            with node_handle:
                last_observations = {
                    stream: [
                        TopicEndpointInfo(**endpoint)
                        for endpoint in _rclpy.rclpy_get_publishers_info_by_topic(
                            node_handle,
                            topic,
                            False,
                        )
                    ]
                    for stream, topic in EXPECTED_RAW_TOPICS.items()
                }
            last_failures = writer_identity_failures(
                expected_writer_gids, last_observations
            )
            check_count += 1
            if last_failures:
                consecutive_matches = 0
            else:
                consecutive_matches += 1
                if consecutive_matches >= stable_samples:
                    passed = True
                    break
            remaining_seconds = (
                deadline_ns - time.monotonic_ns()
            ) / 1_000_000_000
            if remaining_seconds > 0:
                time.sleep(min(POLL_INTERVAL_SECONDS, remaining_seconds))
    finally:
        try:
            if node_handle is not None:
                node_handle.destroy_when_not_in_use()
        finally:
            if context_initialized:
                context.try_shutdown()

    finished_monotonic_ns = time.monotonic_ns()
    return {
        "passed": passed,
        "started_realtime_ns": started_realtime_ns,
        "started_monotonic_ns": started_monotonic_ns,
        "finished_realtime_ns": time.time_ns(),
        "finished_monotonic_ns": finished_monotonic_ns,
        "elapsed_monotonic_ns": finished_monotonic_ns - started_monotonic_ns,
        "discovery_limit_ns": discovery_seconds * 1_000_000_000,
        "stable_samples_required": stable_samples,
        "consecutive_matching_samples": consecutive_matches,
        "graph_check_count": check_count,
        "failures": [] if passed else last_failures,
        "observed_writer_gids": _observed_writer_gids(last_observations),
    }


def verify(arguments: argparse.Namespace) -> tuple[bool, Path]:
    identity_directory = _private_identity_directory(arguments.identity_evidence_dir)
    receipt_path = _new_private_receipt_path(arguments.receipt)

    try:
        expected_writer_gids, identity_manifest_before = _verified_identity(
            identity_directory
        )
    except (OSError, ValueError, PreparationError) as error:
        raise VerificationError(f"identity evidence validation failed: {error}") from error

    graph = observe_live_graph(
        expected_writer_gids,
        discovery_seconds=arguments.discovery_seconds,
        stable_samples=arguments.stable_samples,
    )
    evidence_unchanged = False
    evidence_recheck_failure: str | None = None
    try:
        rechecked_gids, identity_manifest_after = _verified_identity(identity_directory)
        evidence_unchanged = (
            rechecked_gids == expected_writer_gids
            and identity_manifest_after == identity_manifest_before
        )
        if not evidence_unchanged:
            evidence_recheck_failure = "identity_evidence_changed_during_graph_check"
    except (OSError, ValueError, PreparationError) as error:
        evidence_recheck_failure = (
            f"identity_evidence_recheck_failed:{type(error).__name__}:{error}"
        )[:500]

    passed = bool(graph["passed"] and evidence_unchanged)
    receipt = {
        "schema": SCHEMA,
        "phase": arguments.phase,
        "mode": "graph-only-read-only",
        "result": "pass" if passed else "fail",
        "identity_evidence_directory": str(identity_directory),
        "identity_evidence": identity_manifest_before,
        "expected_raw_topics": EXPECTED_RAW_TOPICS,
        "expected_writer_gids": expected_writer_gids,
        "identity_evidence_unchanged": evidence_unchanged,
        "identity_evidence_recheck_failure": evidence_recheck_failure,
        "graph": graph,
        "ros_subscriptions_created": False,
        "ros_publishers_created": False,
        "ros_clients_created": False,
        "ros_services_created": False,
        "ros_actions_created": False,
        "unitree_clients_created": False,
        "network_configuration_changed": False,
        "clock_adjustment_requested": False,
        "motion_ready": False,
        "canonical_odom_ready": False,
    }
    _write_exclusive_private(receipt_path, receipt)
    return passed, receipt_path


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        passed, receipt = verify(arguments)
    except (OSError, ValueError, VerificationError) as error:
        print(f"writer identity verification failed: {error}", file=sys.stderr)
        return 2
    if not passed:
        print(f"writer identity mismatch; failure receipt: {receipt}", file=sys.stderr)
        return 2
    print(f"PASS: current writer GIDs match explicit identity evidence: {receipt}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
