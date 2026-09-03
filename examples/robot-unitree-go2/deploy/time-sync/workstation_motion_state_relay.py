#!/usr/bin/env python3
"""Supervise the C++ affine-qualified first-motion state relay.

The strict three-stream motion timestamp discipline remains the only component
that changes a timestamp. It qualifies SportModeState, MID-360 IMU and
MID-360 odometry and has no PointCloud2 endpoint. Python validates the strict
v3 approval and immutable upstream READY contract, owns readiness/fault files,
and sends a private heartbeat to the C++ worker. It creates no ROS endpoint;
the C++ worker performs graph and per-message publisher-GID checks before it
can copy state to the commissioning-only topic. Neither side has a command
publisher, client, service, or Unitree API.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import stat
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from workstation_nomotion_approval import (  # noqa: E402
    ApprovalError,
    FixedOffsetApproval,
    load_approval,
    require_strict_affine_approval,
)
from workstation_nomotion_stamp_node import (  # noqa: E402
    AFFINE_CORE_STREAMS,
    AFFINE_READY_SCHEMA,
    CORRECTED_TOPICS,
    MOTION_CORRECTED_TOPICS,
)
from navigation_stamp_discipline import (  # noqa: E402
    WORKSTATION_MOTION_MAX_CORRECTED_AGE_NS,
    go2_workstation_motion_config,
)


INPUT_TOPIC = "/robonix/time_corrected/raw/sportmodestate"
OUTPUT_TOPIC = "/robonix/time_corrected/motion/sportmodestate"
READY_SCHEMA = "robonix-go2-workstation-motion-state-ready-v3"
FAULT_SCHEMA = "robonix-go2-workstation-motion-state-fault-v3"
WORKER_PROTOCOL = "robonix-go2-motion-state-gid-pipe-v3"
MAX_FILE_BYTES = 64 * 1024
MAX_EVENT_BYTES = 64 * 1024
MAX_CORRECTED_AGE_NS = WORKSTATION_MOTION_MAX_CORRECTED_AGE_NS
MAX_CORRECTED_FUTURE_NS = 5_000_000
STALE_RECEIPT_NS = 200_000_000
MIN_READY_SAMPLES = 30
INITIAL_READY_TIMEOUT_NS = 5_000_000_000
PUBLISHER_GID_BYTES = 24
STABLE_GRAPH_POLLS = 3
STABLE_MESSAGE_SAMPLES = 3
REQUIRED_RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
GRAPH_RECHECK_PERIOD_MS = 50
INITIAL_GRAPH_DISCOVERY_TIMEOUT_MS = 1_000
SUPERVISOR_HEARTBEAT_TIMEOUT_MS = 200
SUPERVISOR_POLL_SECONDS = 0.05
WORKER_EXIT_GRACE_SECONDS = 1.0
AFFINE_MODE = "affine"
PACKAGE_ROOT = HERE.parents[1]
DEFAULT_WORKER_BINARY = (
    PACKAGE_ROOT
    / "packages/go2_motion_state_relay/.build/ros/install"
    / "go2_motion_state_relay/lib/go2_motion_state_relay"
    / "workstation_motion_state_relay"
)
AFFINE_MODEL_KEYS = frozenset(
    {
        "anchor_source_ns",
        "anchor_local_ns",
        "drift_ppm",
        "source_to_local_scale",
        "core_stream_drifts_ppm",
        "stream_baseline_corrected_age_ns",
        "frozen",
    }
)
AFFINE_READY_KEYS = frozenset(
    {
        "schema",
        "session_id",
        "correction_mode",
        "discipline_profile",
        "corrected_topics",
        "time_discipline_ready",
        "motion_ready",
        "canonical_odom_ready",
        "lidar_odom_semantics",
        "timestamp_safety_limits",
        "approval_reference_offset_ns",
        "approval_affine_common_drift_ppm",
        "affine_model",
        "post_evaluation_commit",
        "noncore_delta_drop_count",
        "last_noncore_delta_drop",
    }
)
POST_EVALUATION_COMMIT_KEYS = frozenset(
    {
        "snapshot_evaluation_monotonic_ns",
        "commit_check_realtime_ns",
        "commit_check_monotonic_ns",
        "evaluation_to_commit_check_ns",
        "clock_read_span_ns",
        "clock_read_span_limit_ns",
        "previous_clock_base_ns",
        "clock_base_ns",
        "clock_base_delta_ns",
        "clock_pair_discontinuity_limit_ns",
        "stream_receipt_liveness",
    }
)
TIMESTAMP_SAFETY_LIMIT_KEYS = frozenset(
    {
        "offset_guard_ns",
        "affine_anchor_past_guard_ns",
        "max_corrected_future_ns",
        "minimum_locked_corrected_age_ns",
        "max_pairwise_drift_ppm",
        "max_approved_affine_drift_deviation_ppm",
        "affine_qualification_window_ns",
        "max_affine_window_common_drift_deviation_ppm",
        "max_locked_affine_drift_deviation_ppm",
    }
)


class RelayError(ValueError):
    """The corrected-state chain is stale, ambiguous, or untrusted."""


@dataclass(frozen=True)
class AffineStateCorrectionContract:
    """Immutable upstream model bound into one first-motion relay run."""

    session_id: str
    approval_reference_offset_ns: int
    anchor_source_ns: int
    anchor_local_ns: int
    drift_ppm: float
    source_to_local_scale: float
    core_stream_drifts_ppm: tuple[tuple[str, float], ...]
    stream_baseline_corrected_age_ns: tuple[tuple[str, int], ...]

    def corrected_stamp_ns(self, source_stamp: int) -> int:
        if (
            not isinstance(source_stamp, int)
            or isinstance(source_stamp, bool)
            or source_stamp <= 0
        ):
            raise RelayError("affine source stamp must be a positive integer")
        corrected = self.anchor_local_ns + round(
            (source_stamp - self.anchor_source_ns) * self.source_to_local_scale
        )
        if corrected <= 0 or corrected >= 2**63:
            raise RelayError("affine corrected stamp is outside positive int64")
        return corrected


def _object_without_duplicate_keys(
    pairs: list[tuple[str, Any]],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise RelayError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _load_private_json(path: Path) -> dict[str, Any]:
    if not path.is_absolute():
        raise RelayError("stamp ready path must be absolute")
    try:
        before = os.lstat(path)
    except OSError as error:
        raise RelayError(f"cannot inspect stamp ready file: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise RelayError("stamp ready file must be a regular non-symlink")
    if before.st_uid != os.geteuid() or stat.S_IMODE(before.st_mode) & 0o077:
        raise RelayError("stamp ready file must be private and owned by this UID")
    if not 0 < before.st_size <= MAX_FILE_BYTES:
        raise RelayError("stamp ready file size is invalid")
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        opened = os.fstat(descriptor)
        if (
            (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino)
            or opened.st_uid != before.st_uid
            or opened.st_size != before.st_size
        ):
            raise RelayError("stamp ready file changed while opening")
        raw = os.read(descriptor, MAX_FILE_BYTES + 1)
    finally:
        os.close(descriptor)
    if len(raw) > MAX_FILE_BYTES:
        raise RelayError("stamp ready file exceeds size limit")
    try:
        payload = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_object_without_duplicate_keys
        )
    except (UnicodeError, json.JSONDecodeError) as error:
        raise RelayError("stamp ready file is not valid UTF-8 JSON") from error
    if not isinstance(payload, dict):
        raise RelayError("stamp ready root must be an object")
    return payload


def _positive_integer(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise RelayError(f"{name} must be a positive integer")
    return value


def _finite_number(value: Any, name: str) -> float:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(value)
    ):
        raise RelayError(f"{name} must be a finite number")
    return float(value)


def validate_stamp_ready(
    payload: dict[str, Any], approval: FixedOffsetApproval
) -> AffineStateCorrectionContract:
    """Validate and normalize the frozen affine state-correction contract.

    The motion relay deliberately rejects the legacy fixed-offset READY file.
    A first-motion launcher now starts the upstream layer in explicit affine
    mode, and every poll must yield the exact same normalized contract.
    """

    require_strict_affine_approval(approval)
    if set(payload) != AFFINE_READY_KEYS:
        raise RelayError("upstream affine stamp-ready fields are incomplete or unknown")
    if payload.get("schema") != AFFINE_READY_SCHEMA:
        raise RelayError("unexpected upstream stamp-ready schema")
    if payload.get("correction_mode") != AFFINE_MODE:
        raise RelayError("upstream correction mode must be affine")
    if payload.get("discipline_profile") != "motion":
        raise RelayError("upstream timestamp discipline profile must be motion")
    noncore_drop_count = payload.get("noncore_delta_drop_count")
    if (
        not isinstance(noncore_drop_count, int)
        or isinstance(noncore_drop_count, bool)
        or noncore_drop_count != 0
    ):
        raise RelayError(
            "motion timestamp READY must have zero non-core delta drops"
        )
    if payload.get("last_noncore_delta_drop") is not None:
        raise RelayError(
            "motion timestamp READY must not retain non-core delta-drop evidence"
        )
    if payload.get("session_id") != approval.session_id:
        raise RelayError("stamp-ready session does not match approval")
    if payload.get("approval_reference_offset_ns") != (
        approval.fixed_local_minus_source_offset_ns
    ):
        raise RelayError("affine reference offset does not match approval")
    if payload.get("approval_affine_common_drift_ppm") != (
        approval.approved_affine_common_drift_ppm
    ):
        raise RelayError("affine common drift does not match approval")
    if payload.get("time_discipline_ready") is not True:
        raise RelayError("upstream timestamp discipline is not ready")
    if payload.get("motion_ready") is not False:
        raise RelayError("timestamp approval must not claim motion authority")
    if payload.get("canonical_odom_ready") is not False:
        raise RelayError("timestamp prelayer must not claim canonical odometry")
    if payload.get("lidar_odom_semantics") != (
        "private_mapping_input_not_chassis_odom"
    ):
        raise RelayError("upstream private lidar odometry semantics changed")
    config = go2_workstation_motion_config(approval.expected_clock_domain)
    raw_limits = payload.get("timestamp_safety_limits")
    if not isinstance(raw_limits, dict) or set(raw_limits) != (
        TIMESTAMP_SAFETY_LIMIT_KEYS
    ):
        raise RelayError("upstream timestamp safety-limit fields are unexpected")
    expected_limits: dict[str, int | float | None] = {
        "offset_guard_ns": config.offset_guard_ns,
        "affine_anchor_past_guard_ns": config.affine_anchor_past_guard_ns,
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
    for name, expected in expected_limits.items():
        observed = raw_limits[name]
        if expected is None:
            matches = observed is None
        elif isinstance(expected, int):
            matches = (
                isinstance(observed, int)
                and not isinstance(observed, bool)
                and observed == expected
            )
        else:
            matches = (
                isinstance(observed, (int, float))
                and not isinstance(observed, bool)
                and math.isfinite(observed)
                and float(observed) == expected
            )
        if not matches:
            raise RelayError(f"upstream timestamp safety limit changed: {name}")

    commit = payload.get("post_evaluation_commit")
    if not isinstance(commit, dict) or set(commit) != POST_EVALUATION_COMMIT_KEYS:
        raise RelayError("upstream post-evaluation commit evidence is unexpected")
    positive_commit_fields = (
        "snapshot_evaluation_monotonic_ns",
        "commit_check_realtime_ns",
        "commit_check_monotonic_ns",
        "clock_read_span_limit_ns",
        "previous_clock_base_ns",
        "clock_base_ns",
        "clock_pair_discontinuity_limit_ns",
    )
    for name in positive_commit_fields:
        _positive_integer(commit.get(name), f"post-evaluation {name}")
    nonnegative_commit_fields = (
        "evaluation_to_commit_check_ns",
        "clock_read_span_ns",
        "clock_base_delta_ns",
    )
    for name in nonnegative_commit_fields:
        value = commit.get(name)
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise RelayError(f"post-evaluation {name} must be nonnegative")
    if commit["commit_check_monotonic_ns"] < commit[
        "snapshot_evaluation_monotonic_ns"
    ]:
        raise RelayError("post-evaluation commit clock regressed")
    if commit["evaluation_to_commit_check_ns"] != (
        commit["commit_check_monotonic_ns"]
        - commit["snapshot_evaluation_monotonic_ns"]
    ):
        raise RelayError("post-evaluation commit duration is inconsistent")
    if commit["clock_read_span_limit_ns"] != config.max_clock_read_span_ns or (
        commit["clock_read_span_ns"] > config.max_clock_read_span_ns
    ):
        raise RelayError("post-evaluation clock read span exceeded")
    if commit["clock_pair_discontinuity_limit_ns"] != (
        config.max_clock_pair_discontinuity_ns
    ) or commit["clock_base_delta_ns"] > config.max_clock_pair_discontinuity_ns:
        raise RelayError("post-evaluation clock pair discontinuity exceeded")
    if commit["clock_base_ns"] != (
        commit["commit_check_realtime_ns"] - commit["commit_check_monotonic_ns"]
    ) or commit["clock_base_delta_ns"] != abs(
        commit["clock_base_ns"] - commit["previous_clock_base_ns"]
    ):
        raise RelayError("post-evaluation clock-pair evidence is inconsistent")
    liveness = commit.get("stream_receipt_liveness")
    if not isinstance(liveness, dict) or set(liveness) != set(
        MOTION_CORRECTED_TOPICS
    ):
        raise RelayError("post-evaluation stream liveness set is unexpected")
    policies = {policy.name: policy for policy in config.streams}
    for name, details in liveness.items():
        expected_liveness_keys = {
            "last_advancing_receipt_monotonic_ns",
            "receipt_age_ns",
            "stale_receipt_timeout_ns",
            "live",
        }
        if not isinstance(details, dict) or set(details) != expected_liveness_keys:
            raise RelayError(f"post-evaluation liveness fields changed: {name}")
        last_receipt_ns = _positive_integer(
            details["last_advancing_receipt_monotonic_ns"],
            f"post-evaluation last receipt {name}",
        )
        receipt_age_ns = details["receipt_age_ns"]
        if (
            not isinstance(receipt_age_ns, int)
            or isinstance(receipt_age_ns, bool)
            or receipt_age_ns < 0
        ):
            raise RelayError(f"post-evaluation receipt age is invalid: {name}")
        expected_limit_ns = policies[name].stale_receipt_timeout_ns
        if (
            details["stale_receipt_timeout_ns"] != expected_limit_ns
            or receipt_age_ns > expected_limit_ns
            or details["live"] is not True
            or last_receipt_ns + receipt_age_ns
            != commit["commit_check_monotonic_ns"]
        ):
            raise RelayError(f"post-evaluation stream is not live: {name}")
    topics = payload.get("corrected_topics")
    if topics != MOTION_CORRECTED_TOPICS:
        raise RelayError("upstream corrected topic contract is unexpected")
    if CORRECTED_TOPICS.get("sport_primary") != INPUT_TOPIC:
        raise RelayError("checked-in corrected SportModeState contract changed")

    model = payload.get("affine_model")
    if not isinstance(model, dict) or set(model) != AFFINE_MODEL_KEYS:
        raise RelayError("upstream affine model fields are incomplete or unknown")
    if model.get("frozen") is not True:
        raise RelayError("upstream affine model must be frozen")
    anchor_source_ns = _positive_integer(
        model.get("anchor_source_ns"), "affine anchor_source_ns"
    )
    anchor_local_ns = _positive_integer(
        model.get("anchor_local_ns"), "affine anchor_local_ns"
    )
    if anchor_source_ns >= 2**63 or anchor_local_ns >= 2**63:
        raise RelayError("affine anchors must fit positive int64")
    drift_ppm = _finite_number(model.get("drift_ppm"), "affine drift_ppm")
    source_to_local_scale = _finite_number(
        model.get("source_to_local_scale"), "affine source_to_local_scale"
    )
    if abs(drift_ppm) > config.max_absolute_drift_ppm:
        raise RelayError("affine drift exceeds the approved absolute limit")
    expected_scale = 1.0 / (1.0 - drift_ppm * 1.0e-6)
    if source_to_local_scale <= 0.0 or not math.isclose(
        source_to_local_scale,
        expected_scale,
        rel_tol=1.0e-12,
        abs_tol=1.0e-15,
    ):
        raise RelayError("affine scale does not match drift_ppm")

    raw_core_drifts = model.get("core_stream_drifts_ppm")
    if not isinstance(raw_core_drifts, dict) or set(raw_core_drifts) != set(
        AFFINE_CORE_STREAMS
    ):
        raise RelayError("affine core-stream drift contract is unexpected")
    core_drifts = tuple(
        (name, _finite_number(raw_core_drifts[name], f"core drift {name}"))
        for name in AFFINE_CORE_STREAMS
    )
    if any(abs(value) > config.max_absolute_drift_ppm for _, value in core_drifts):
        raise RelayError("affine core-stream drift exceeds the absolute limit")
    common_drift = float(statistics.median(value for _, value in core_drifts))
    if not math.isclose(drift_ppm, common_drift, rel_tol=0.0, abs_tol=1.0e-9):
        raise RelayError("affine drift is not the core-stream median")
    core_drift_values = [value for _, value in core_drifts]
    if max(core_drift_values) - min(core_drift_values) > (
        config.max_pairwise_drift_ppm
    ):
        raise RelayError("affine core-stream drift disagreement exceeds limit")

    raw_baselines = model.get("stream_baseline_corrected_age_ns")
    if not isinstance(raw_baselines, dict) or set(raw_baselines) != set(
        MOTION_CORRECTED_TOPICS
    ):
        raise RelayError("affine stream-baseline contract is unexpected")
    policies = {policy.name: policy for policy in config.streams}
    baselines: list[tuple[str, int]] = []
    for name in MOTION_CORRECTED_TOPICS:
        value = raw_baselines[name]
        if not isinstance(value, int) or isinstance(value, bool):
            raise RelayError(f"affine baseline {name} must be an integer")
        if not -config.max_corrected_future_ns <= value <= policies[
            name
        ].max_corrected_age_ns:
            raise RelayError(f"affine baseline {name} exceeds freshness limits")
        baselines.append((name, value))

    approval_reference_offset_ns = payload["approval_reference_offset_ns"]
    assert isinstance(approval_reference_offset_ns, int)
    contract = AffineStateCorrectionContract(
        session_id=approval.session_id,
        approval_reference_offset_ns=approval_reference_offset_ns,
        anchor_source_ns=anchor_source_ns,
        anchor_local_ns=anchor_local_ns,
        drift_ppm=drift_ppm,
        source_to_local_scale=source_to_local_scale,
        core_stream_drifts_ppm=core_drifts,
        stream_baseline_corrected_age_ns=tuple(baselines),
    )
    if contract.corrected_stamp_ns(anchor_source_ns) != anchor_local_ns:
        raise RelayError("affine state correction does not preserve its anchor")
    if contract.corrected_stamp_ns(anchor_source_ns + 1_000_000_000) <= (
        anchor_local_ns
    ):
        raise RelayError("affine state correction is not strictly advancing")
    return contract


def corrected_state_issue(
    *,
    now_realtime_ns: int,
    now_monotonic_ns: int,
    corrected_stamp_ns: int,
    last_corrected_stamp_ns: int | None,
    last_receipt_monotonic_ns: int | None,
) -> str:
    values = (
        now_realtime_ns,
        now_monotonic_ns,
        corrected_stamp_ns,
    )
    if any(
        not isinstance(value, int) or isinstance(value, bool) or value <= 0
        for value in values
    ):
        return "corrected-state clocks must be positive integers"
    if (
        last_corrected_stamp_ns is not None
        and corrected_stamp_ns <= last_corrected_stamp_ns
    ):
        return "corrected SportModeState stamp did not advance"
    age_ns = now_realtime_ns - corrected_stamp_ns
    if age_ns < -MAX_CORRECTED_FUTURE_NS:
        return "corrected SportModeState stamp is too far in the future"
    if age_ns > MAX_CORRECTED_AGE_NS:
        return "corrected SportModeState stamp is stale"
    if last_receipt_monotonic_ns is not None:
        receipt_age_ns = now_monotonic_ns - last_receipt_monotonic_ns
        if receipt_age_ns < 0:
            return "monotonic receipt clock regressed"
    return ""


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
        temporary.unlink(missing_ok=True)


@dataclass(frozen=True)
class WorkerReady:
    graph_publisher_gid: str
    message_publisher_gid: str
    rmw_implementation_identifier: str
    samples: int
    publisher_count: int
    stable_graph_polls: int
    stable_message_samples: int


@dataclass(frozen=True)
class WorkerFault:
    reason: str
    graph_publisher_gid: str | None
    message_publisher_gid: str | None
    observed_publisher_gid: str | None
    observed_publisher_gid_domain: str | None
    publisher_count: int
    rmw_implementation_identifier: str | None


def _publisher_gid(value: str, *, optional: bool = False) -> str | None:
    if optional and value == "-":
        return None
    if len(value) != PUBLISHER_GID_BYTES * 2 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise RelayError("worker publisher GID is not exact lowercase hex")
    if value == "0" * (PUBLISHER_GID_BYTES * 2):
        raise RelayError("worker publisher GID is zero")
    return value


def _graph_publisher_gid(value: str, *, optional: bool = False) -> str | None:
    gid = _publisher_gid(value, optional=optional)
    if gid is None:
        return None
    raw = bytes.fromhex(gid)
    if not any(raw[:16]) or any(raw[16:]):
        raise RelayError("worker graph GID has unexpected CycloneDDS shape")
    return gid


def _message_publisher_gid(
    value: str, *, optional: bool = False
) -> str | None:
    gid = _publisher_gid(value, optional=optional)
    if gid is None:
        return None
    raw = bytes.fromhex(gid)
    if not any(raw[:8]) or any(raw[8:]):
        raise RelayError("worker message GID has unexpected CycloneDDS shape")
    return gid


def _bounded_nonnegative_integer(value: str, name: str) -> int:
    if not value.isascii() or not value.isdecimal():
        raise RelayError(f"worker {name} is not a nonnegative integer")
    parsed = int(value)
    if parsed > 1_000_000_000:
        raise RelayError(f"worker {name} exceeds protocol limit")
    return parsed


def _rmw_identifier(value: str, *, optional: bool = False) -> str | None:
    if optional and value == "-":
        return None
    if not 0 < len(value) <= 80 or any(
        not (
            character.isascii()
            and (
                character.isalnum()
                or character in "_-."
            )
        )
        for character in value
    ):
        raise RelayError("worker RMW implementation identifier is invalid")
    if value != REQUIRED_RMW_IMPLEMENTATION:
        raise RelayError("worker RMW implementation is not rmw_cyclonedds_cpp")
    return value


def parse_worker_event(line: str) -> WorkerReady | WorkerFault:
    """Parse one exact, newline-free event from the local C++ worker."""

    if not line or "\n" in line or "\r" in line or "\x00" in line:
        raise RelayError("worker event framing is invalid")
    fields = line.split("\t")
    if fields[0] == "READY_V3" and len(fields) == 8:
        graph_gid = _graph_publisher_gid(fields[1])
        message_gid = _message_publisher_gid(fields[2])
        rmw_identifier = _rmw_identifier(fields[3])
        assert isinstance(graph_gid, str)
        assert isinstance(message_gid, str)
        assert isinstance(rmw_identifier, str)
        samples = _bounded_nonnegative_integer(fields[4], "sample count")
        publisher_count = _bounded_nonnegative_integer(
            fields[5], "publisher count"
        )
        stable_polls = _bounded_nonnegative_integer(
            fields[6], "stable graph polls"
        )
        stable_message_samples = _bounded_nonnegative_integer(
            fields[7], "stable message samples"
        )
        if samples < MIN_READY_SAMPLES:
            raise RelayError("worker READY has too few verified samples")
        if publisher_count != 1:
            raise RelayError("worker READY publisher count is not one")
        if stable_polls != STABLE_GRAPH_POLLS:
            raise RelayError("worker READY graph-poll count is unexpected")
        if stable_message_samples != STABLE_MESSAGE_SAMPLES:
            raise RelayError("worker READY message-sample count is unexpected")
        return WorkerReady(
            graph_gid,
            message_gid,
            rmw_identifier,
            samples,
            publisher_count,
            stable_polls,
            stable_message_samples,
        )
    if fields[0] == "FAULT_V3" and len(fields) == 8:
        reason = fields[1]
        if not 0 < len(reason) <= 80 or any(
            character not in "abcdefghijklmnopqrstuvwxyz0123456789_"
            for character in reason
        ):
            raise RelayError("worker fault reason is invalid")
        graph_gid = _graph_publisher_gid(fields[2], optional=True)
        message_gid = _message_publisher_gid(fields[3], optional=True)
        observed = _publisher_gid(fields[4], optional=True)
        observed_domain = None if fields[5] == "-" else fields[5]
        if observed_domain not in (None, "graph", "message"):
            raise RelayError("worker observed GID domain is invalid")
        if observed is not None and observed_domain is None:
            raise RelayError("worker observed GID requires its representation domain")
        publisher_count = _bounded_nonnegative_integer(
            fields[6], "publisher count"
        )
        rmw_identifier = _rmw_identifier(fields[7], optional=True)
        return WorkerFault(
            reason,
            graph_gid,
            message_gid,
            observed,
            observed_domain,
            publisher_count,
            rmw_identifier,
        )
    raise RelayError("worker event type or field count is unexpected")


def _validate_worker_binary(path: Path) -> None:
    if not path.is_absolute():
        raise RelayError("worker binary path must be absolute")
    try:
        metadata = os.lstat(path)
    except OSError as error:
        raise RelayError(f"cannot inspect worker binary: {error}") from error
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise RelayError("worker binary must be a regular non-symlink")
    if metadata.st_uid != os.geteuid():
        raise RelayError("worker binary must be owned by this UID")
    if stat.S_IMODE(metadata.st_mode) & 0o022:
        raise RelayError("worker binary must not be group/world writable")
    if not os.access(path, os.X_OK):
        raise RelayError("worker binary is not executable")


def _ready_payload(
    approval: FixedOffsetApproval, event: WorkerReady
) -> dict[str, Any]:
    return {
        "schema": READY_SCHEMA,
        "session_id": approval.session_id,
        "input_topic": INPUT_TOPIC,
        "output_topic": OUTPUT_TOPIC,
        "correction_mode": AFFINE_MODE,
        "discipline_profile": "motion",
        "affine_model_frozen": True,
        "worker_protocol": WORKER_PROTOCOL,
        "graph_publisher_gid": event.graph_publisher_gid,
        "message_publisher_gid": event.message_publisher_gid,
        "rmw_implementation_identifier": event.rmw_implementation_identifier,
        "publisher_gid_bytes": PUBLISHER_GID_BYTES,
        "publisher_count": event.publisher_count,
        "stable_graph_polls": event.stable_graph_polls,
        "stable_message_samples": event.stable_message_samples,
        "publisher_identity_exact": True,
        "publisher_qos_exact": True,
        "graph_publisher_gid_nonzero": True,
        "message_publisher_gid_nonzero": True,
        "graph_publisher_gid_frozen": True,
        "message_publisher_gid_frozen": True,
        "graph_gid_cyclonedds_shape_valid": True,
        "message_gid_cyclonedds_shape_valid": True,
        "cross_gid_representation_comparison": False,
        "cross_gid_writer_mapping_proven": False,
        "gid_assurance_scope": (
            "continuity_under_single_exact_graph_publisher"
        ),
        "message_gid_implementation_identifier_exact": True,
        "rebind_allowed": False,
        "graph_recheck_period_ms": GRAPH_RECHECK_PERIOD_MS,
        "initial_graph_discovery_timeout_ms": (
            INITIAL_GRAPH_DISCOVERY_TIMEOUT_MS
        ),
        "supervisor_heartbeat_timeout_ms": SUPERVISOR_HEARTBEAT_TIMEOUT_MS,
        "supervisor_heartbeat_active": True,
        "samples": event.samples,
        "timestamp_chain_ready": True,
        # Only the separately consumed one-shot permit can authorize motion.
        "motion_authorized": False,
    }


def _fault_payload(
    approval: FixedOffsetApproval,
    reason: str,
    *,
    graph_publisher_gid: str | None = None,
    message_publisher_gid: str | None = None,
    observed_publisher_gid: str | None = None,
    observed_publisher_gid_domain: str | None = None,
    publisher_count: int = 0,
    rmw_implementation_identifier: str | None = None,
) -> dict[str, Any]:
    return {
        "schema": FAULT_SCHEMA,
        "session_id": approval.session_id,
        "reason": str(reason)[:300] or "unspecified_motion_state_fault",
        "graph_publisher_gid": graph_publisher_gid,
        "message_publisher_gid": message_publisher_gid,
        "observed_publisher_gid": observed_publisher_gid,
        "observed_publisher_gid_domain": observed_publisher_gid_domain,
        "publisher_count": publisher_count,
        "rmw_implementation_identifier": rmw_implementation_identifier,
        "cross_gid_representation_comparison": False,
        "cross_gid_writer_mapping_proven": False,
        "gid_assurance_scope": (
            "continuity_under_single_exact_graph_publisher"
        ),
        "fault_latched": True,
        "rebind_allowed": False,
        "timestamp_chain_ready": False,
        "motion_authorized": False,
    }


def run_supervisor(
    approval_file: Path,
    approval: FixedOffsetApproval,
    stamp_ready_file: Path,
    ready_file: Path,
    fault_file: Path,
    worker_binary: Path = DEFAULT_WORKER_BINARY,
    stop_requested: threading.Event | None = None,
) -> int:
    """Supervise the only C++ endpoint owner through private one-way pipes.

    Python never creates a ROS endpoint.  It continuously revalidates the
    strict v3 approval and immutable affine READY contract, then emits a
    one-byte heartbeat.  Closing or stalling this process therefore removes
    the worker's permission to relay within 200 ms.
    """

    require_strict_affine_approval(approval)
    stamp_contract = validate_stamp_ready(
        _load_private_json(stamp_ready_file), approval
    )
    _validate_worker_binary(worker_binary)
    stop_requested = stop_requested or threading.Event()
    heartbeat_read, heartbeat_write = os.pipe2(os.O_CLOEXEC | os.O_NONBLOCK)
    event_read, event_write = os.pipe2(os.O_CLOEXEC | os.O_NONBLOCK)
    worker: subprocess.Popen[bytes] | None = None
    fault_reason: str | None = None
    frozen_graph_gid: str | None = None
    frozen_message_gid: str | None = None
    frozen_rmw_identifier: str | None = None
    last_publisher_count = 0
    event_buffer = bytearray()

    def latch_fault(
        reason: str,
        *,
        graph_gid: str | None = None,
        message_gid: str | None = None,
        observed: str | None = None,
        observed_domain: str | None = None,
        publisher_count: int | None = None,
        rmw_identifier: str | None = None,
    ) -> None:
        nonlocal fault_reason
        if fault_reason is not None:
            return
        fault_reason = str(reason)[:300] or "unspecified_motion_state_fault"
        try:
            _atomic_json(
                fault_file,
                _fault_payload(
                    approval,
                    fault_reason,
                    graph_publisher_gid=(
                        graph_gid
                        if graph_gid is not None
                        else frozen_graph_gid
                    ),
                    message_publisher_gid=(
                        message_gid
                        if message_gid is not None
                        else frozen_message_gid
                    ),
                    observed_publisher_gid=observed,
                    observed_publisher_gid_domain=observed_domain,
                    publisher_count=(
                        last_publisher_count
                        if publisher_count is None
                        else publisher_count
                    ),
                    rmw_implementation_identifier=(
                        rmw_identifier
                        if rmw_identifier is not None
                        else frozen_rmw_identifier
                    ),
                ),
            )
        except Exception as error:
            print(
                f"motion-state supervisor could not write fault status: "
                f"{type(error).__name__}: {error}",
                file=sys.stderr,
            )

    def handle_event(event: WorkerReady | WorkerFault) -> None:
        nonlocal frozen_graph_gid, frozen_message_gid
        nonlocal frozen_rmw_identifier, last_publisher_count
        if isinstance(event, WorkerFault):
            last_publisher_count = event.publisher_count
            latch_fault(
                f"worker:{event.reason}",
                graph_gid=event.graph_publisher_gid,
                message_gid=event.message_publisher_gid,
                observed=event.observed_publisher_gid,
                observed_domain=event.observed_publisher_gid_domain,
                publisher_count=event.publisher_count,
                rmw_identifier=event.rmw_implementation_identifier,
            )
            return
        if frozen_graph_gid is not None or frozen_message_gid is not None:
            latch_fault("worker emitted duplicate READY")
            return
        try:
            current = load_approval(approval_file, require_affine=True)
            if current != approval:
                raise ApprovalError("timestamp approval changed before READY")
            current_contract = validate_stamp_ready(
                _load_private_json(stamp_ready_file), approval
            )
            if current_contract != stamp_contract:
                raise RelayError(
                    "frozen affine state-correction contract changed before READY"
                )
            _atomic_json(ready_file, _ready_payload(approval, event))
        except Exception as error:
            latch_fault(
                f"ready commit failed: {type(error).__name__}: {error}"
            )
            return
        frozen_graph_gid = event.graph_publisher_gid
        frozen_message_gid = event.message_publisher_gid
        frozen_rmw_identifier = event.rmw_implementation_identifier
        last_publisher_count = event.publisher_count

    try:
        # Prime the one-way pipe before the worker can create any ROS endpoint.
        # The C++ callback drains this record before checking freshness, which
        # removes the startup race where the first state sample could precede
        # the worker's 50 ms graph/supervisor timer.  The monotonic sender
        # timestamp remains authoritative, so a delayed worker still fails
        # closed once the 200 ms freshness bound is exceeded.
        heartbeat = f"{time.monotonic_ns():016x}\n".encode("ascii")
        if len(heartbeat) != 17:
            raise RelayError("local monotonic heartbeat is out of range")
        if os.write(heartbeat_write, heartbeat) != len(heartbeat):
            raise RelayError("initial worker heartbeat write was incomplete")

        worker = subprocess.Popen(
            [
                os.fspath(worker_binary),
                "--heartbeat-fd",
                str(heartbeat_read),
                "--event-fd",
                str(event_write),
            ],
            stdin=subprocess.DEVNULL,
            close_fds=True,
            pass_fds=(heartbeat_read, event_write),
        )
        os.close(heartbeat_read)
        heartbeat_read = -1
        os.close(event_write)
        event_write = -1

        while not stop_requested.is_set() and fault_reason is None:
            event_pipe_closed = False
            pending_events: list[WorkerReady | WorkerFault] = []
            while True:
                try:
                    chunk = os.read(event_read, 4096)
                except BlockingIOError:
                    break
                except InterruptedError:
                    continue
                except OSError as error:
                    latch_fault(
                        f"worker event pipe read failed: {type(error).__name__}"
                    )
                    break
                if not chunk:
                    event_pipe_closed = True
                    break
                event_buffer.extend(chunk)
                if len(event_buffer) > MAX_EVENT_BYTES:
                    latch_fault("worker event buffer exceeded protocol limit")
                    break
                while b"\n" in event_buffer:
                    raw_line, _, remainder = event_buffer.partition(b"\n")
                    event_buffer[:] = remainder
                    try:
                        line = raw_line.decode("ascii")
                        pending_events.append(parse_worker_event(line))
                    except (UnicodeError, RelayError) as error:
                        latch_fault(f"worker event rejected: {error}")
                    if fault_reason is not None:
                        break
                if fault_reason is not None:
                    break

            if fault_reason is not None:
                break
            worker_status = worker.poll()
            worker_faults = [
                event for event in pending_events
                if isinstance(event, WorkerFault)
            ]
            worker_readies = [
                event for event in pending_events
                if isinstance(event, WorkerReady)
            ]
            # A fault in the same pipe drain always dominates READY.  This
            # prevents even a transient ready file when the worker bound and
            # faulted before the supervisor could consume its status.
            if worker_faults:
                handle_event(worker_faults[0])
                break
            if event_pipe_closed or worker_status is not None:
                latch_fault(
                    "worker exited without a complete fault event"
                    if not event_buffer
                    else "worker exited with a partial event"
                )
                break
            if len(worker_readies) > 1:
                latch_fault("worker emitted duplicate READY")
                break
            if worker_readies:
                handle_event(worker_readies[0])
                if fault_reason is not None:
                    break
            try:
                current = load_approval(approval_file, require_affine=True)
                if current != approval:
                    raise ApprovalError("timestamp approval changed during relay")
                current_contract = validate_stamp_ready(
                    _load_private_json(stamp_ready_file), approval
                )
                if current_contract != stamp_contract:
                    raise RelayError(
                        "frozen affine state-correction contract changed"
                    )
                heartbeat = f"{time.monotonic_ns():016x}\n".encode("ascii")
                if len(heartbeat) != 17:
                    raise RelayError("local monotonic heartbeat is out of range")
                if os.write(heartbeat_write, heartbeat) != len(heartbeat):
                    raise RelayError("worker heartbeat write was incomplete")
            except Exception as error:
                latch_fault(
                    f"timestamp approval/ready supervision failed: "
                    f"{type(error).__name__}: {error}"
                )
                break
            stop_requested.wait(SUPERVISOR_POLL_SECONDS)
    except Exception as error:
        latch_fault(f"worker launch/supervision failed: {type(error).__name__}: {error}")
    finally:
        for descriptor in (
            heartbeat_read,
            heartbeat_write,
            event_read,
            event_write,
        ):
            if descriptor >= 0:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
        if worker is not None:
            try:
                worker.wait(timeout=WORKER_EXIT_GRACE_SECONDS)
            except subprocess.TimeoutExpired:
                worker.terminate()
                try:
                    worker.wait(timeout=WORKER_EXIT_GRACE_SECONDS)
                except subprocess.TimeoutExpired:
                    worker.kill()
                    worker.wait(timeout=WORKER_EXIT_GRACE_SECONDS)
    return 70 if fault_reason is not None else 0


def _absolute_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("runtime paths must be absolute")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--approval-file", required=True, type=_absolute_path)
    parser.add_argument("--stamp-ready-file", required=True, type=_absolute_path)
    parser.add_argument("--ready-file", required=True, type=_absolute_path)
    parser.add_argument("--fault-file", required=True, type=_absolute_path)
    parser.add_argument(
        "--worker-binary",
        type=_absolute_path,
        default=DEFAULT_WORKER_BINARY,
    )
    args = parser.parse_args()
    if len({args.stamp_ready_file, args.ready_file, args.fault_file}) != 3:
        parser.error("stamp-ready, ready, and fault paths must differ")
    for path in (args.ready_file, args.fault_file):
        if path.exists() or path.is_symlink():
            parser.error(f"runtime status path already exists: {path}")
    try:
        approval = load_approval(args.approval_file, require_affine=True)
    except ApprovalError as error:
        parser.error(str(error))
    stop_requested = threading.Event()
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, lambda _signal, _frame: stop_requested.set())
    try:
        return run_supervisor(
            args.approval_file,
            approval,
            args.stamp_ready_file,
            args.ready_file,
            args.fault_file,
            args.worker_binary,
            stop_requested,
        )
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
