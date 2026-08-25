#!/usr/bin/env python3
"""Isolated corrected PointCloud2 relay for workstation navigation stacks.

The no-motion timestamp process still validates the raw MID-360 cloud but
deliberately has no corrected cloud publisher.  The strict motion timestamp
process omits the large cloud altogether so it cannot delay state, IMU, or
odometry watchdogs.  This separate ROS process reads either parent's immutable
affine READY contract, applies only the frozen model, and publishes one private
best-effort depth-one PointCloud2 stream.  It has no command, service, action,
TF, odometry, clock-setting, or Unitree API surface.

The default ``nomotion`` profile preserves the mapping/UI recovery contract.
The explicit ``motion`` profile may instead consume the strict motion stamp
process's frozen core-stream model.  It never republishes state, IMU, or
odometry; a cloud older than the 250 ms navigation publish budget is simply
dropped so the downstream scan freshness guard can stop and disarm motion.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import sys
import tempfile
import threading
import time
from typing import Any, Callable


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from navigation_stamp_discipline import (  # noqa: E402
    AffineClockModel,
    NOMOTION_TRANSIENT_STALE_RECOVERY_CEILING_NS,
    NONCORE_DELTA_DROP_REASON_PREFIX,
    go2_navigation_config,
    go2_workstation_motion_config,
    go2_workstation_nomotion_config,
)
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
    MOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS,
    MOTION_CORRECTED_TOPICS,
    NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS,
    RAW_TOPICS,
    corrected_message_copy,
    fresh_paired_receipt_clocks,
    source_stamp_ns,
)


READY_SCHEMA = "robonix-go2-workstation-nomotion-cloud-relay-ready-v1"
FAULT_SCHEMA = "robonix-go2-workstation-nomotion-cloud-relay-fault-v1"
MOTION_READY_SCHEMA = (
    "robonix-go2-workstation-motion-cloud-relay-ready-v1"
)
MOTION_FAULT_SCHEMA = (
    "robonix-go2-workstation-motion-cloud-relay-fault-v1"
)
INPUT_TOPIC = RAW_TOPICS["mid360_cloud"]
OUTPUT_TOPIC = CORRECTED_TOPICS["mid360_cloud"]
PUBLISH_PERIOD_NS = 200_000_000
MOTION_PUBLISH_PERIOD_NS = 100_000_000
MOTION_CLOUD_MAX_CORRECTED_AGE_NS = 250_000_000
PARENT_POLL_SECONDS = 0.05
MAX_FILE_BYTES = 64 * 1024
RELAY_PROFILES = ("nomotion", "motion")
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
STAMP_READY_KEYS = frozenset(
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
    """The parent contract, process identity, or cloud sample is untrusted."""


class TransientCloudStale(RelayError):
    """A stale cloud must be dropped while the isolated relay stays alive."""

    def __init__(
        self,
        corrected_age_ns: int,
        publish_limit_ns: int,
        recovery_ceiling_ns: int,
    ) -> None:
        self.corrected_age_ns = corrected_age_ns
        self.publish_limit_ns = publish_limit_ns
        self.recovery_ceiling_ns = recovery_ceiling_ns
        super().__init__(
            "corrected cloud timestamp is transiently stale:"
            f"corrected_age_ns={corrected_age_ns}:"
            f"publish_limit_ns={publish_limit_ns}:"
            f"recovery_ceiling_ns={recovery_ceiling_ns}"
        )


def _raw_subscription_is_reliable(profile: str) -> bool:
    """Keep lossless mapping input without retransmit backlog during motion."""

    if profile not in RELAY_PROFILES:
        raise RelayError(f"unsupported cloud relay profile: {profile}")
    return profile == "nomotion"


@dataclass(frozen=True)
class PrivateFileSnapshot:
    device: int
    inode: int
    uid: int
    mode: int
    size: int
    mtime_ns: int
    sha256: str


@dataclass(frozen=True)
class ProcessIdentity:
    pid: int
    start_ticks: int


@dataclass(frozen=True)
class CloudCorrectionContract:
    session_id: str
    profile: str
    model: AffineClockModel
    max_corrected_age_ns: int
    hard_corrected_age_ns: int
    max_corrected_future_ns: int
    minimum_corrected_age_ns: int | None
    max_clock_read_span_ns: int
    clock_pair_age_acquisition_attempts: int
    publish_period_ns: int
    drop_all_stale: bool
    require_fresh_recovery: bool

    def corrected_stamp_ns(
        self, source_ns: int, receipt_realtime_ns: int
    ) -> int:
        corrected_ns = self.model.corrected_stamp_ns(source_ns)
        corrected_age_ns = receipt_realtime_ns - corrected_ns
        if corrected_age_ns < -self.max_corrected_future_ns:
            raise RelayError(
                "corrected cloud timestamp is too far in the future"
            )
        if (
            self.minimum_corrected_age_ns is not None
            and corrected_age_ns < self.minimum_corrected_age_ns
        ):
            raise RelayError(
                "corrected cloud timestamp exhausted its affine age margin"
            )
        if corrected_age_ns > self.max_corrected_age_ns:
            if (
                self.drop_all_stale
                or corrected_age_ns <= self.hard_corrected_age_ns
            ):
                raise TransientCloudStale(
                    corrected_age_ns,
                    self.max_corrected_age_ns,
                    self.hard_corrected_age_ns,
                )
            raise RelayError(
                "corrected cloud timestamp exceeds its hard stale ceiling"
            )
        return corrected_ns


def _object_without_duplicate_keys(
    pairs: list[tuple[str, Any]],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise RelayError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _read_private_json(
    path: Path,
) -> tuple[dict[str, Any], PrivateFileSnapshot]:
    if not path.is_absolute():
        raise RelayError("runtime contract paths must be absolute")
    try:
        before = os.lstat(path)
    except OSError as error:
        raise RelayError(f"cannot inspect private JSON: {path}: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise RelayError("private JSON must be a regular non-symlink")
    if before.st_uid != os.geteuid() or stat.S_IMODE(before.st_mode) & 0o077:
        raise RelayError("private JSON must be private and owned by this UID")
    if not 0 < before.st_size <= MAX_FILE_BYTES:
        raise RelayError("private JSON size is invalid")
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        opened = os.fstat(descriptor)
        if (
            (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino)
            or opened.st_uid != before.st_uid
            or opened.st_size != before.st_size
            or opened.st_mtime_ns != before.st_mtime_ns
        ):
            raise RelayError("private JSON changed while opening")
        raw = os.read(descriptor, MAX_FILE_BYTES + 1)
    finally:
        os.close(descriptor)
    if len(raw) != before.st_size:
        raise RelayError("private JSON changed while reading")
    try:
        payload = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_object_without_duplicate_keys,
        )
    except (UnicodeError, json.JSONDecodeError) as error:
        raise RelayError("private JSON is not valid UTF-8 JSON") from error
    if not isinstance(payload, dict):
        raise RelayError("private JSON root must be an object")
    snapshot = PrivateFileSnapshot(
        device=before.st_dev,
        inode=before.st_ino,
        uid=before.st_uid,
        mode=stat.S_IMODE(before.st_mode),
        size=before.st_size,
        mtime_ns=before.st_mtime_ns,
        sha256=hashlib.sha256(raw).hexdigest(),
    )
    return payload, snapshot


def _atomic_private_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
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
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


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
        raise RelayError(f"{name} must be finite")
    return float(value)


def _exact_number(value: Any, expected: int | float | None, name: str) -> None:
    if expected is None:
        if value is not None:
            raise RelayError(f"timestamp safety limit changed: {name}")
        return
    if isinstance(expected, int):
        matches = (
            isinstance(value, int)
            and not isinstance(value, bool)
            and value == expected
        )
    else:
        matches = (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            and float(value) == expected
        )
    if not matches:
        raise RelayError(f"timestamp safety limit changed: {name}")


def _validate_noncore_delta_drop_evidence(
    payload: dict[str, Any], profile: str
) -> None:
    count = payload.get("noncore_delta_drop_count")
    if (
        not isinstance(count, int)
        or isinstance(count, bool)
        or count < 0
    ):
        raise RelayError("non-core delta-drop count must be a nonnegative integer")
    last = payload.get("last_noncore_delta_drop")
    if profile == "motion":
        if count != 0 or last is not None:
            raise RelayError(
                "motion stamp READY must not contain non-core delta-drop evidence"
            )
        return
    if count == 0:
        if last is not None:
            raise RelayError(
                "zero non-core delta-drop count must have null last evidence"
            )
        return
    expected_keys = {
        "stream",
        "reason",
        "receipt_monotonic_ns",
        "source_delta_ns",
        "receipt_delta_ns",
        "delta_error_ns",
        "delta_error_limit_ns",
    }
    if not isinstance(last, dict) or set(last) != expected_keys:
        raise RelayError("last non-core delta-drop evidence fields are unexpected")
    if last.get("stream") != "mid360_cloud" or last.get("reason") != (
        f"{NONCORE_DELTA_DROP_REASON_PREFIX}:mid360_cloud"
    ):
        raise RelayError("last non-core delta-drop identity is unexpected")
    numeric_fields = (
        "receipt_monotonic_ns",
        "source_delta_ns",
        "receipt_delta_ns",
        "delta_error_ns",
        "delta_error_limit_ns",
    )
    if any(
        not isinstance(last.get(name), int)
        or isinstance(last.get(name), bool)
        or last[name] <= 0
        for name in numeric_fields
    ):
        raise RelayError(
            "last non-core delta-drop timing fields must be positive integers"
        )
    if last["delta_error_ns"] != abs(
        last["source_delta_ns"] - last["receipt_delta_ns"]
    ) or last["delta_error_ns"] <= last["delta_error_limit_ns"]:
        raise RelayError("last non-core delta-drop timing evidence is inconsistent")


def validate_stamp_ready(
    payload: dict[str, Any],
    approval: FixedOffsetApproval,
    profile: str = "nomotion",
) -> CloudCorrectionContract:
    """Bind one immutable affine READY model to its retained session."""

    require_strict_affine_approval(approval)
    if profile not in RELAY_PROFILES:
        raise RelayError("unsupported cloud relay profile")
    if set(payload) != STAMP_READY_KEYS:
        raise RelayError("stamp READY fields are incomplete or unknown")
    if payload.get("schema") != AFFINE_READY_SCHEMA:
        raise RelayError("unexpected stamp READY schema")
    if payload.get("session_id") != approval.session_id:
        raise RelayError("stamp READY session does not match approval")
    if payload.get("correction_mode") != "affine":
        raise RelayError("cloud relay requires affine correction")
    if payload.get("discipline_profile") != profile:
        raise RelayError(
            f"cloud relay requires the {profile} timestamp profile"
        )
    _validate_noncore_delta_drop_evidence(payload, profile)
    if payload.get("time_discipline_ready") is not True:
        raise RelayError("parent timestamp discipline is not ready")
    if payload.get("motion_ready") is not False:
        raise RelayError("stamp READY must not grant motion")
    if payload.get("canonical_odom_ready") is not False:
        raise RelayError("stamp READY must not claim canonical odometry")
    if payload.get("lidar_odom_semantics") != (
        "private_mapping_input_not_chassis_odom"
    ):
        raise RelayError("private lidar odometry semantics changed")
    expected_corrected_topics = (
        MOTION_CORRECTED_TOPICS
        if profile == "motion"
        else CORRECTED_TOPICS
    )
    if payload.get("corrected_topics") != expected_corrected_topics:
        raise RelayError("corrected topic contract changed")
    if payload.get("approval_reference_offset_ns") != (
        approval.fixed_local_minus_source_offset_ns
    ):
        raise RelayError("stamp READY reference offset changed")
    if payload.get("approval_affine_common_drift_ppm") != (
        approval.approved_affine_common_drift_ppm
    ):
        raise RelayError("stamp READY approved drift changed")

    config = (
        go2_workstation_motion_config(approval.expected_clock_domain)
        if profile == "motion"
        else go2_workstation_nomotion_config(approval.expected_clock_domain)
    )
    raw_limits = payload.get("timestamp_safety_limits")
    if not isinstance(raw_limits, dict) or set(raw_limits) != (
        TIMESTAMP_SAFETY_LIMIT_KEYS
    ):
        raise RelayError("timestamp safety-limit fields are unexpected")
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
        _exact_number(raw_limits.get(name), expected, name)

    raw_model = payload.get("affine_model")
    if not isinstance(raw_model, dict) or set(raw_model) != AFFINE_MODEL_KEYS:
        raise RelayError("affine model fields are incomplete or unknown")
    if raw_model.get("frozen") is not True:
        raise RelayError("affine model is not frozen")
    anchor_source_ns = _positive_integer(
        raw_model.get("anchor_source_ns"), "anchor_source_ns"
    )
    anchor_local_ns = _positive_integer(
        raw_model.get("anchor_local_ns"), "anchor_local_ns"
    )
    drift_ppm = _finite_number(raw_model.get("drift_ppm"), "drift_ppm")
    scale = _finite_number(
        raw_model.get("source_to_local_scale"), "source_to_local_scale"
    )
    expected_scale = 1.0 / (1.0 - drift_ppm * 1e-6)
    if scale <= 0.0 or not math.isclose(
        scale, expected_scale, rel_tol=0.0, abs_tol=1e-15
    ):
        raise RelayError("affine scale is inconsistent with drift")
    # The approved drift is evidence from an earlier stationary collection,
    # not a permanent oscillator calibration.  A new boot's parent timestamp
    # process has already qualified and frozen this model against current live
    # streams.  Re-comparing it with the historical median here caused valid
    # restarts to fail after READY.  Keep the approval fields above as exact
    # session/audit metadata, while validating the live frozen model through
    # scale consistency, pairwise agreement, baselines and timestamp limits.

    raw_core_drifts = raw_model.get("core_stream_drifts_ppm")
    if not isinstance(raw_core_drifts, dict) or set(raw_core_drifts) != set(
        AFFINE_CORE_STREAMS
    ):
        raise RelayError("affine core-stream drift set changed")
    core_drifts = tuple(
        (name, _finite_number(raw_core_drifts[name], f"drift {name}"))
        for name in AFFINE_CORE_STREAMS
    )
    if any(
        abs(value - drift_ppm) > config.max_pairwise_drift_ppm
        for _, value in core_drifts
    ):
        raise RelayError("affine core-stream drift disagreement exceeded")

    policies = {policy.name: policy for policy in config.streams}
    raw_baselines = raw_model.get("stream_baseline_corrected_age_ns")
    if not isinstance(raw_baselines, dict) or set(raw_baselines) != set(
        expected_corrected_topics
    ):
        raise RelayError("affine baseline stream set changed")
    baselines: list[tuple[str, int]] = []
    minimum_age_ns = config.minimum_locked_corrected_age_ns
    if profile == "nomotion" and minimum_age_ns is None:
        raise RelayError("no-motion affine age margin is unavailable")
    for name in expected_corrected_topics:
        age_ns = _positive_integer(
            raw_baselines[name], f"baseline corrected age {name}"
        )
        policy = policies[name]
        if (
            (minimum_age_ns is not None and age_ns < minimum_age_ns)
            or policy.max_corrected_age_ns is None
            or age_ns > policy.max_corrected_age_ns
        ):
            raise RelayError(f"baseline corrected age is unsafe: {name}")
        baselines.append((name, age_ns))

    if not isinstance(payload.get("post_evaluation_commit"), dict):
        raise RelayError("post-evaluation commit evidence is missing")
    cloud_policy = (
        {
            policy.name: policy
            for policy in go2_navigation_config(
                approval.expected_clock_domain
            ).streams
        }["mid360_cloud"]
        if profile == "motion"
        else policies["mid360_cloud"]
    )
    if cloud_policy.max_corrected_age_ns is None:
        raise RelayError("cloud corrected-age limit is unavailable")
    if profile == "motion":
        if (
            cloud_policy.max_corrected_age_ns
            != MOTION_CLOUD_MAX_CORRECTED_AGE_NS
        ):
            raise RelayError("motion cloud freshness limit changed")
        hard_corrected_age_ns = MOTION_CLOUD_MAX_CORRECTED_AGE_NS
        publish_period_ns = MOTION_PUBLISH_PERIOD_NS
        clock_pair_age_acquisition_attempts = (
            MOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS
        )
        drop_all_stale = True
        require_fresh_recovery = False
    else:
        if (
            cloud_policy.hard_age_ceiling_ns
            != NOMOTION_TRANSIENT_STALE_RECOVERY_CEILING_NS
        ):
            raise RelayError("cloud transient-stale recovery ceiling changed")
        hard_corrected_age_ns = cloud_policy.hard_age_ceiling_ns
        publish_period_ns = PUBLISH_PERIOD_NS
        clock_pair_age_acquisition_attempts = (
            NOMOTION_CLOCK_PAIR_AGE_ACQUISITION_ATTEMPTS
        )
        drop_all_stale = False
        require_fresh_recovery = True
    model = AffineClockModel(
        anchor_source_ns=anchor_source_ns,
        anchor_local_ns=anchor_local_ns,
        drift_ppm=drift_ppm,
        source_to_local_scale=scale,
        core_stream_drifts_ppm=tuple(core_drifts),
        stream_baseline_corrected_age_ns=tuple(baselines),
    )
    return CloudCorrectionContract(
        session_id=approval.session_id,
        profile=profile,
        model=model,
        max_corrected_age_ns=cloud_policy.max_corrected_age_ns,
        hard_corrected_age_ns=hard_corrected_age_ns,
        max_corrected_future_ns=config.max_corrected_future_ns,
        minimum_corrected_age_ns=minimum_age_ns,
        max_clock_read_span_ns=config.max_clock_read_span_ns,
        clock_pair_age_acquisition_attempts=(
            clock_pair_age_acquisition_attempts
        ),
        publish_period_ns=publish_period_ns,
        drop_all_stale=drop_all_stale,
        require_fresh_recovery=require_fresh_recovery,
    )


def process_identity(pid: int) -> ProcessIdentity:
    if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 1:
        raise RelayError("parent PID must be greater than one")
    try:
        stat_text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except OSError as error:
        raise RelayError(f"parent process is unavailable: {pid}") from error
    close = stat_text.rfind(")")
    if close < 0:
        raise RelayError("parent process stat is malformed")
    fields = stat_text[close + 2 :].split()
    if len(fields) <= 19:
        raise RelayError("parent process stat is truncated")
    try:
        start_ticks = int(fields[19])
    except ValueError as error:
        raise RelayError("parent process start time is invalid") from error
    if start_ticks <= 0:
        raise RelayError("parent process start time is nonpositive")
    return ProcessIdentity(pid=pid, start_ticks=start_ticks)


class ParentContractGuard:
    """Continuously bind the relay to immutable files and live parent PIDs."""

    def __init__(
        self,
        *,
        approval_file: Path,
        approval_snapshot: PrivateFileSnapshot,
        stamp_ready_file: Path,
        stamp_ready_snapshot: PrivateFileSnapshot,
        stamp_fault_file: Path,
        identity_fault_file: Path,
        stamp_process: ProcessIdentity,
        identity_process: ProcessIdentity,
    ) -> None:
        self.approval_file = approval_file
        self.approval_snapshot = approval_snapshot
        self.stamp_ready_file = stamp_ready_file
        self.stamp_ready_snapshot = stamp_ready_snapshot
        self.stamp_fault_file = stamp_fault_file
        self.identity_fault_file = identity_fault_file
        self.stamp_process = stamp_process
        self.identity_process = identity_process

    @staticmethod
    def _require_fault_absent(path: Path, name: str) -> None:
        try:
            os.lstat(path)
        except FileNotFoundError:
            return
        except OSError as error:
            raise RelayError(f"cannot inspect {name} fault path: {error}") from error
        raise RelayError(f"{name} fault became visible")

    def check(self) -> None:
        _, approval_snapshot = _read_private_json(self.approval_file)
        if approval_snapshot != self.approval_snapshot:
            raise RelayError("approval file changed after relay startup")
        _, ready_snapshot = _read_private_json(self.stamp_ready_file)
        if ready_snapshot != self.stamp_ready_snapshot:
            raise RelayError("stamp READY changed after relay startup")
        self._require_fault_absent(self.stamp_fault_file, "timestamp")
        self._require_fault_absent(self.identity_fault_file, "identity")
        if process_identity(self.stamp_process.pid) != self.stamp_process:
            raise RelayError("timestamp parent process identity changed")
        if process_identity(self.identity_process.pid) != self.identity_process:
            raise RelayError("identity parent process identity changed")


class _PublishRateLimiter:
    def __init__(self, period_ns: int = PUBLISH_PERIOD_NS) -> None:
        if (
            not isinstance(period_ns, int)
            or isinstance(period_ns, bool)
            or period_ns <= 0
        ):
            raise ValueError("publish period must be positive")
        self.period_ns = period_ns
        self.last_publish_monotonic_ns: int | None = None

    def allow(self, monotonic_ns: int) -> bool:
        if (
            not isinstance(monotonic_ns, int)
            or isinstance(monotonic_ns, bool)
            or monotonic_ns <= 0
        ):
            raise ValueError("publish clock must be positive")
        previous = self.last_publish_monotonic_ns
        if previous is not None and monotonic_ns < previous:
            raise RelayError("publish monotonic clock regressed")
        if previous is not None and monotonic_ns - previous < self.period_ns:
            return False
        self.last_publish_monotonic_ns = monotonic_ns
        return True


class _CloudStampSequenceGuard:
    """Reject a fresh cloud whose frozen source/corrected time did not advance."""

    def __init__(self) -> None:
        self.last_source_ns: int | None = None
        self.last_corrected_ns: int | None = None

    def observe(self, source_ns: int, corrected_ns: int) -> None:
        if (
            not isinstance(source_ns, int)
            or isinstance(source_ns, bool)
            or source_ns <= 0
            or not isinstance(corrected_ns, int)
            or isinstance(corrected_ns, bool)
            or corrected_ns <= 0
        ):
            raise RelayError("cloud stamp sequence values must be positive")
        if (
            self.last_source_ns is not None
            and source_ns <= self.last_source_ns
        ):
            raise RelayError("cloud source timestamp did not advance")
        if (
            self.last_corrected_ns is not None
            and corrected_ns <= self.last_corrected_ns
        ):
            raise RelayError("cloud corrected timestamp did not advance")
        self.last_source_ns = source_ns
        self.last_corrected_ns = corrected_ns


class _FreshCloudRecoveryGate:
    """Require two consecutive fresh samples after a transient stale drop."""

    def __init__(self, required_fresh_samples: int = 2) -> None:
        if (
            not isinstance(required_fresh_samples, int)
            or isinstance(required_fresh_samples, bool)
            or required_fresh_samples < 2
        ):
            raise ValueError("cloud recovery requires at least two fresh samples")
        self.required_fresh_samples = required_fresh_samples
        self.active = False
        self.fresh_samples = 0

    def observe_stale(self) -> None:
        self.active = True
        self.fresh_samples = 0

    def allow_fresh(self) -> bool:
        if not self.active:
            return True
        self.fresh_samples += 1
        if self.fresh_samples < self.required_fresh_samples:
            return False
        self.active = False
        self.fresh_samples = 0
        return True


class _Runtime:
    def __init__(
        self,
        contract: CloudCorrectionContract,
        ready_file: Path,
        fault_file: Path,
    ) -> None:
        self.contract = contract
        self.ready_file = ready_file
        self.fault_file = fault_file
        self.fault_reason: str | None = None
        self.shutdown: Callable[[], None] = lambda: None

    def latch_fault(self, reason: str) -> None:
        if self.fault_reason is not None:
            return
        self.fault_reason = str(reason)[:240] or "unspecified_cloud_relay_fault"
        try:
            _atomic_private_json(
                self.fault_file,
                {
                    "schema": (
                        MOTION_FAULT_SCHEMA
                        if self.contract.profile == "motion"
                        else FAULT_SCHEMA
                    ),
                    "session_id": self.contract.session_id,
                    "reason": self.fault_reason,
                    "motion_ready": False,
                    "canonical_odom_ready": False,
                },
            )
        finally:
            self.shutdown()


def run_ros(
    contract: CloudCorrectionContract,
    guard: ParentContractGuard,
    ready_file: Path,
    fault_file: Path,
    stop_requested: threading.Event | None = None,
) -> int:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2

    context = Context()
    rclpy.init(args=[], context=context)
    stop_requested = stop_requested or threading.Event()
    runtime = _Runtime(contract, ready_file, fault_file)

    class CloudRelayNode(Node):
        def __init__(self) -> None:
            super().__init__(
                f"go2_workstation_{contract.profile}_cloud_relay",
                context=context,
            )
            runtime.shutdown = lambda: (
                context.shutdown() if context.ok() else None
            )
            raw_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                # Mapping keeps its lossless raw-cloud contract.  Physical
                # navigation instead consumes the latest sensor sample: a
                # reliable multi-megabyte PointCloud2 reader can request DDS
                # retransmits after a radio wobble and delay the independent
                # state/IMU/odometry streams that stop the chassis.  Depth one
                # plus BEST_EFFORT drops an obsolete scan rather than building
                # a network backlog; corrected-age and scan-freshness guards
                # still reject stale data downstream.
                reliability=(
                    ReliabilityPolicy.RELIABLE
                    if _raw_subscription_is_reliable(contract.profile)
                    else ReliabilityPolicy.BEST_EFFORT
                ),
                durability=DurabilityPolicy.VOLATILE,
            )
            corrected_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._publisher = self.create_publisher(
                PointCloud2, OUTPUT_TOPIC, corrected_qos
            )
            self._subscription = self.create_subscription(
                PointCloud2, INPUT_TOPIC, self._cloud, raw_qos
            )
            self._limiter = _PublishRateLimiter(contract.publish_period_ns)
            self._stamp_sequence = _CloudStampSequenceGuard()
            self._recovery = _FreshCloudRecoveryGate()
            self._guard_timer = self.create_timer(
                PARENT_POLL_SECONDS, self._poll_guard
            )

        def _poll_guard(self) -> None:
            try:
                if stop_requested.is_set():
                    runtime.shutdown()
                    return
                guard.check()
            except Exception as error:
                runtime.latch_fault(
                    f"parent_guard_failed:{type(error).__name__}:{error}"
                )

        def _cloud(self, message: Any) -> None:
            if runtime.fault_reason is not None:
                return
            try:
                guard.check()
                clocks = fresh_paired_receipt_clocks(
                    max_clock_read_span_ns=contract.max_clock_read_span_ns,
                    age_acquisition_attempts=(
                        contract.clock_pair_age_acquisition_attempts
                    ),
                )
                source_ns = source_stamp_ns(message.header.stamp)
                try:
                    corrected_ns = contract.corrected_stamp_ns(
                        source_ns, clocks.realtime_ns
                    )
                except TransientCloudStale:
                    if contract.require_fresh_recovery:
                        self._recovery.observe_stale()
                    return
                if (
                    contract.require_fresh_recovery
                    and not self._recovery.allow_fresh()
                ):
                    return
                # The no-motion parent already observes the raw cloud and owns
                # its duplicate/regression policy.  The motion stamp process
                # deliberately omits PointCloud2, so this isolated process
                # must supply the missing strict sequence check itself.
                if contract.profile == "motion":
                    self._stamp_sequence.observe(source_ns, corrected_ns)
                if not self._limiter.allow(clocks.monotonic_ns):
                    return
                output = corrected_message_copy(
                    message, "mid360_cloud", corrected_ns
                )
                guard.check()
                publish_clocks = fresh_paired_receipt_clocks(
                    max_clock_read_span_ns=contract.max_clock_read_span_ns,
                    age_acquisition_attempts=(
                        contract.clock_pair_age_acquisition_attempts
                    ),
                )
                try:
                    publish_corrected_ns = contract.corrected_stamp_ns(
                        source_ns, publish_clocks.realtime_ns
                    )
                except TransientCloudStale:
                    if contract.require_fresh_recovery:
                        self._recovery.observe_stale()
                    return
                if publish_corrected_ns != corrected_ns:
                    raise RelayError("frozen cloud correction changed")
                self._publisher.publish(output)
            except Exception as error:
                runtime.latch_fault(
                    f"cloud_processing_failed:{type(error).__name__}:{error}"
                )

    executor = SingleThreadedExecutor(context=context)
    node = CloudRelayNode()
    added = False
    try:
        added = executor.add_node(node)
        if not added:
            raise RuntimeError("cloud relay could not join its executor")
        guard.check()
        ready_payload = {
            "schema": READY_SCHEMA,
            "session_id": contract.session_id,
            "input_topic": INPUT_TOPIC,
            "output_topic": OUTPUT_TOPIC,
            "stamp_pid": guard.stamp_process.pid,
            "stamp_start_ticks": guard.stamp_process.start_ticks,
            "identity_pid": guard.identity_process.pid,
            "identity_start_ticks": guard.identity_process.start_ticks,
            "motion_ready": False,
            "canonical_odom_ready": False,
        }
        if contract.profile == "motion":
            ready_payload.update(
                {
                    "schema": MOTION_READY_SCHEMA,
                    "relay_profile": "motion",
                    "max_corrected_age_ns": (
                        MOTION_CLOUD_MAX_CORRECTED_AGE_NS
                    ),
                    "publish_period_ns": MOTION_PUBLISH_PERIOD_NS,
                    "stale_policy": "drop_without_publish",
                }
            )
        _atomic_private_json(ready_file, ready_payload)
        executor.spin()
    except Exception as error:
        runtime.latch_fault(
            f"cloud_relay_runtime_failed:{type(error).__name__}:{error}"
        )
    finally:
        try:
            if added and context.ok():
                executor.remove_node(node)
        finally:
            try:
                executor.shutdown()
            finally:
                node.destroy_node()
                if context.ok():
                    context.shutdown()
    return 70 if runtime.fault_reason is not None else 0


def _private_output_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("runtime status paths must be absolute")
    return path


def _positive_pid(value: str) -> int:
    try:
        pid = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("parent PID must be an integer") from error
    if pid <= 1:
        raise argparse.ArgumentTypeError("parent PID must be greater than one")
    return pid


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start one isolated corrected PointCloud2 relay"
    )
    parser.add_argument(
        "--profile",
        choices=RELAY_PROFILES,
        default="nomotion",
        help=(
            "timestamp READY profile to bind; motion is explicit and still "
            "does not grant motion"
        ),
    )
    parser.add_argument("--approval-file", required=True, type=Path)
    parser.add_argument(
        "--stamp-ready-file", required=True, type=_private_output_path
    )
    parser.add_argument(
        "--stamp-fault-file", required=True, type=_private_output_path
    )
    parser.add_argument(
        "--identity-fault-file", required=True, type=_private_output_path
    )
    parser.add_argument("--stamp-pid", required=True, type=_positive_pid)
    parser.add_argument("--identity-pid", required=True, type=_positive_pid)
    parser.add_argument("--ready-file", required=True, type=_private_output_path)
    parser.add_argument("--fault-file", required=True, type=_private_output_path)
    args = parser.parse_args()
    output_paths = (args.ready_file, args.fault_file)
    if len(set(output_paths)) != len(output_paths):
        parser.error("cloud relay ready and fault paths must differ")
    for path in output_paths:
        if path.exists() or path.is_symlink():
            parser.error(f"runtime status path already exists: {path}")
    try:
        approval = load_approval(args.approval_file, require_affine=True)
        approval_payload, approval_snapshot = _read_private_json(
            args.approval_file
        )
        if approval_payload.get("session_id") != approval.session_id:
            raise RelayError("approval changed while loading")
        ready_payload, ready_snapshot = _read_private_json(
            args.stamp_ready_file
        )
        contract = validate_stamp_ready(
            ready_payload, approval, profile=args.profile
        )
        guard = ParentContractGuard(
            approval_file=args.approval_file.resolve(),
            approval_snapshot=approval_snapshot,
            stamp_ready_file=args.stamp_ready_file,
            stamp_ready_snapshot=ready_snapshot,
            stamp_fault_file=args.stamp_fault_file,
            identity_fault_file=args.identity_fault_file,
            stamp_process=process_identity(args.stamp_pid),
            identity_process=process_identity(args.identity_pid),
        )
        guard.check()
    except (ApprovalError, RelayError) as error:
        parser.error(str(error))

    stop_requested = threading.Event()
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, lambda _signal, _frame: stop_requested.set())
    try:
        return run_ros(
            contract,
            guard,
            args.ready_file,
            args.fault_file,
            stop_requested=stop_requested,
        )
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
