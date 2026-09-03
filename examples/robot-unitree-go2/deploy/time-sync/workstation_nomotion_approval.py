#!/usr/bin/env python3
"""Validate one private fixed-offset evidence record for no-motion use.

The approval is deliberately a local, private file.  It binds an immutable
``local realtime - source timestamp`` offset to one reviewed writer/session
evidence set; it is not a credential and it cannot authorize motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
import math
import os
from pathlib import Path
import re
import stat
import time
from typing import Any


LEGACY_SCHEMA = "robonix-go2-workstation-nomotion-stamp-offset-v2"
SCHEMA = "robonix-go2-workstation-nomotion-stamp-offset-v3"
ACK = "I_APPROVE_THIS_FIXED_OFFSET_FOR_WORKSTATION_NOMOTION"
AFFINE_DRIFT_ALGORITHM = "one-second-lower-envelope-theil-sen-v1"
AFFINE_DRIFT_WINDOW_NS = 30 * 1_000_000_000
# This limit compares the two non-overlapping 30 s startup half-windows only.
# The runtime candidate/model-versus-approved drift limit remains independently
# fixed at 5 ppm in the navigation timestamp discipline.
MAX_STARTUP_HALF_WINDOW_COMMON_DRIFT_DEVIATION_PPM = 15.0
MAX_ABSOLUTE_AFFINE_DRIFT_PPM = 50.0
EXPECTED_CLOCK_DOMAIN = "unitree-main-computer@192.168.123.161"
EXPECTED_WRITER_SOURCE = "192.168.123.161"
EXPECTED_RAW_TOPICS = {
    "sport_primary": "/sportmodestate",
    "mid360_imu": "/utlidar/imu",
    "mid360_cloud": "/utlidar/cloud",
    "mid360_odom": "/utlidar/robot_odom",
}
MAX_FILE_BYTES = 32 * 1024
MAX_ABSOLUTE_OFFSET_NS = 60 * 60 * 1_000_000_000
_SESSION_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{7,95}$")
_HEX_48_RE = re.compile(r"^[0-9a-f]{48}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_LEGACY_KEYS = frozenset(
    {
        "schema",
        "session_id",
        "motion_enabled",
        "identity_evidence_verified",
        "expected_clock_domain",
        "writer_gids",
        "writer_source_ipv4",
        "offset_evidence_sha256",
        "fixed_local_minus_source_offset_ns",
        "not_before_unix_ns",
        "expires_unix_ns",
        "operator_ack",
    }
)
_AFFINE_KEYS = _LEGACY_KEYS | frozenset(
    {
        "affine_drift_algorithm",
        "affine_drift_window_ns",
        "affine_window_common_drifts_ppm",
        "approved_affine_common_drift_ppm",
        "affine_window_common_drift_deviation_ppm",
    }
)


class ApprovalError(ValueError):
    """The approval is missing, unsafe, or internally inconsistent."""


def _exact_int(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ApprovalError(f"{name} must be an integer")
    return value


@dataclass(frozen=True, slots=True)
class FixedOffsetApproval:
    schema: str
    session_id: str
    expected_clock_domain: str
    writer_gids: tuple[tuple[str, str], ...]
    writer_source_ipv4: str
    offset_evidence_sha256: str
    fixed_local_minus_source_offset_ns: int
    approved_affine_common_drift_ppm: float | None
    affine_window_common_drifts_ppm: tuple[float, float] | None
    not_before_unix_ns: int
    expires_unix_ns: int

    def writer_gid_for(self, stream: str) -> str:
        for name, gid in self.writer_gids:
            if name == stream:
                return gid
        raise KeyError(stream)

    def valid_at(self, realtime_ns: int) -> bool:
        # ``expires_unix_ns`` remains accepted as historical schema metadata,
        # but it is not a runtime gate.  Current writer identity and the live
        # affine qualification establish whether this record may be used.
        return self.not_before_unix_ns <= realtime_ns

    def require_valid_at(self, realtime_ns: int) -> None:
        if not self.valid_at(realtime_ns):
            raise ApprovalError("fixed-offset approval issue time is in the future")


def require_strict_affine_approval(
    approval: FixedOffsetApproval,
    *,
    now_realtime_ns: int | None = None,
) -> FixedOffsetApproval:
    """Revalidate a normalized schema-v3 approval at a direct API boundary.

    ``load_approval(..., require_affine=True)`` is the file/CLI boundary, but
    the timestamp and motion-relay modules also expose direct Python entry
    points.  Those entry points must not accept a hand-built schema-v2 object
    merely because somebody populated its optional drift field.  Rechecking
    the complete normalized contract here keeps the gate ahead of ROS init and
    endpoint creation without reopening or trusting the source JSON file.
    """

    if not isinstance(approval, FixedOffsetApproval):
        raise ApprovalError("affine approval must be a FixedOffsetApproval")
    if approval.schema != SCHEMA:
        raise ApprovalError("affine mode requires a schema-v3 drift approval")
    if (
        not isinstance(approval.session_id, str)
        or _SESSION_RE.fullmatch(approval.session_id) is None
    ):
        raise ApprovalError("session_id is invalid")
    if approval.expected_clock_domain != EXPECTED_CLOCK_DOMAIN:
        raise ApprovalError("clock domain is not the reviewed Go2 source")
    if approval.writer_source_ipv4 != EXPECTED_WRITER_SOURCE:
        raise ApprovalError("writer source IPv4 is not the reviewed Go2 source")
    if (
        not isinstance(approval.offset_evidence_sha256, str)
        or _SHA256_RE.fullmatch(approval.offset_evidence_sha256) is None
    ):
        raise ApprovalError("offset evidence SHA-256 is invalid")
    offset_ns = _exact_int(
        approval.fixed_local_minus_source_offset_ns,
        "fixed_local_minus_source_offset_ns",
    )
    if offset_ns == 0 or abs(offset_ns) > MAX_ABSOLUTE_OFFSET_NS:
        raise ApprovalError("fixed offset must be nonzero and no more than one hour")

    writer_gids = approval.writer_gids
    if (
        not isinstance(writer_gids, tuple)
        or len(writer_gids) != len(EXPECTED_RAW_TOPICS)
        or tuple(name for name, _ in writer_gids) != tuple(EXPECTED_RAW_TOPICS)
    ):
        raise ApprovalError("writer_gids must name every required stream exactly once")
    seen_gids: set[str] = set()
    for stream, gid in writer_gids:
        if not isinstance(gid, str) or _HEX_48_RE.fullmatch(gid) is None:
            raise ApprovalError(
                f"writer_gids.{stream} must be 24 lowercase hexadecimal bytes"
            )
        if gid in seen_gids:
            raise ApprovalError("each required stream must have a distinct writer GID")
        seen_gids.add(gid)

    window_drifts = approval.affine_window_common_drifts_ppm
    approved_drift = approval.approved_affine_common_drift_ppm
    if not isinstance(window_drifts, tuple) or len(window_drifts) != 2:
        raise ApprovalError("schema-v3 approval requires two affine drift windows")

    def normalized_drift(value: Any, name: str) -> float:
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
        ):
            raise ApprovalError(f"{name} must be a finite number")
        result = float(value)
        if abs(result) > MAX_ABSOLUTE_AFFINE_DRIFT_PPM:
            raise ApprovalError(f"{name} exceeds the absolute drift limit")
        return result

    first_drift = normalized_drift(window_drifts[0], "first affine window drift")
    second_drift = normalized_drift(window_drifts[1], "second affine window drift")
    normalized_approved = normalized_drift(
        approved_drift, "approved_affine_common_drift_ppm"
    )
    if not math.isclose(
        normalized_approved,
        (first_drift + second_drift) / 2.0,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ApprovalError(
            "approved affine common drift is not the two-window median"
        )
    if (
        abs(first_drift - second_drift)
        > MAX_STARTUP_HALF_WINDOW_COMMON_DRIFT_DEVIATION_PPM
    ):
        raise ApprovalError(
            "affine startup half-window common drift deviation exceeds 15 ppm"
        )

    not_before_ns = _exact_int(approval.not_before_unix_ns, "not_before_unix_ns")
    expires_ns = _exact_int(approval.expires_unix_ns, "expires_unix_ns")
    if not_before_ns <= 0 or expires_ns <= not_before_ns:
        raise ApprovalError("approval validity interval is invalid")
    approval.require_valid_at(
        time.time_ns() if now_realtime_ns is None else now_realtime_ns
    )
    return approval


def _read_private_regular_file(path: Path) -> dict[str, Any]:
    if not path.is_absolute():
        raise ApprovalError("approval path must be absolute")
    try:
        before = os.lstat(path)
    except OSError as error:
        raise ApprovalError(f"cannot inspect approval file: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise ApprovalError("approval must be a real regular file, not a symlink")
    if before.st_uid != os.geteuid():
        raise ApprovalError("approval file must be owned by the current user")
    if stat.S_IMODE(before.st_mode) & 0o077:
        raise ApprovalError("approval file must not be accessible by group or others")
    if before.st_size <= 0 or before.st_size > MAX_FILE_BYTES:
        raise ApprovalError("approval file size is invalid")

    flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise ApprovalError(f"cannot open approval file safely: {error}") from error
    try:
        opened = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened.st_mode)
            or opened.st_dev != before.st_dev
            or opened.st_ino != before.st_ino
            or opened.st_uid != before.st_uid
            or opened.st_size != before.st_size
        ):
            raise ApprovalError("approval file changed during validation")
        def no_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
            result: dict[str, Any] = {}
            for key, value in pairs:
                if key in result:
                    raise ApprovalError(f"duplicate approval key: {key}")
                result[key] = value
            return result

        with os.fdopen(descriptor, "r", encoding="utf-8", closefd=False) as handle:
            payload = json.load(handle, object_pairs_hook=no_duplicate_keys)
    except (OSError, UnicodeError, json.JSONDecodeError, ApprovalError) as error:
        raise ApprovalError(f"approval is not valid UTF-8 JSON: {error}") from error
    finally:
        os.close(descriptor)
    if not isinstance(payload, dict):
        raise ApprovalError("approval JSON must be an object")
    return payload


def load_approval(
    path: str | Path,
    *,
    now_realtime_ns: int | None = None,
    require_affine: bool = False,
) -> FixedOffsetApproval:
    payload = _read_private_regular_file(Path(path))
    schema = payload.get("schema")
    expected_keys = _AFFINE_KEYS if schema == SCHEMA else _LEGACY_KEYS
    keys = set(payload)
    if keys != expected_keys:
        missing = sorted(expected_keys - keys)
        unknown = sorted(keys - expected_keys)
        raise ApprovalError(
            f"approval keys do not match schema (missing={missing}, unknown={unknown})"
        )
    if schema != LEGACY_SCHEMA and schema != SCHEMA:
        raise ApprovalError("approval schema is not supported")
    if require_affine and schema != SCHEMA:
        raise ApprovalError("affine mode requires a schema-v3 drift approval")
    if payload["motion_enabled"] is not False:
        raise ApprovalError("approval must set motion_enabled to false")
    if payload["identity_evidence_verified"] is not True:
        raise ApprovalError("writer identity evidence is not explicitly verified")
    if payload["operator_ack"] != ACK:
        raise ApprovalError("operator acknowledgement does not match")

    session_id = payload["session_id"]
    if not isinstance(session_id, str) or _SESSION_RE.fullmatch(session_id) is None:
        raise ApprovalError("session_id is invalid")
    clock_domain = payload["expected_clock_domain"]
    if clock_domain != EXPECTED_CLOCK_DOMAIN:
        raise ApprovalError("clock domain is not the reviewed Go2 source")
    writer_gids_value = payload["writer_gids"]
    if not isinstance(writer_gids_value, dict) or set(writer_gids_value) != set(
        EXPECTED_RAW_TOPICS
    ):
        raise ApprovalError("writer_gids must name every required stream exactly once")
    writer_gids: list[tuple[str, str]] = []
    seen_gids: set[str] = set()
    for stream in EXPECTED_RAW_TOPICS:
        gid = writer_gids_value.get(stream)
        if not isinstance(gid, str) or _HEX_48_RE.fullmatch(gid) is None:
            raise ApprovalError(
                f"writer_gids.{stream} must be 24 lowercase hexadecimal bytes"
            )
        if gid in seen_gids:
            raise ApprovalError("each required stream must have a distinct writer GID")
        seen_gids.add(gid)
        writer_gids.append((stream, gid))
    writer_source = payload["writer_source_ipv4"]
    if writer_source != EXPECTED_WRITER_SOURCE:
        raise ApprovalError("writer source IPv4 is not the reviewed Go2 source")
    evidence_hash = payload["offset_evidence_sha256"]
    if not isinstance(evidence_hash, str) or _SHA256_RE.fullmatch(evidence_hash) is None:
        raise ApprovalError("offset evidence SHA-256 is invalid")

    offset_ns = _exact_int(
        payload["fixed_local_minus_source_offset_ns"],
        "fixed_local_minus_source_offset_ns",
    )
    if offset_ns == 0 or abs(offset_ns) > MAX_ABSOLUTE_OFFSET_NS:
        raise ApprovalError("fixed offset must be nonzero and no more than one hour")

    approved_affine_drift_ppm: float | None = None
    affine_window_common_drifts_ppm: tuple[float, float] | None = None
    if schema == SCHEMA:
        if payload["affine_drift_algorithm"] != AFFINE_DRIFT_ALGORITHM:
            raise ApprovalError("affine drift algorithm is not supported")
        if _exact_int(
            payload["affine_drift_window_ns"], "affine_drift_window_ns"
        ) != AFFINE_DRIFT_WINDOW_NS:
            raise ApprovalError("affine drift window must be exactly 30 seconds")
        raw_window_drifts = payload["affine_window_common_drifts_ppm"]
        if not isinstance(raw_window_drifts, dict) or set(raw_window_drifts) != {
            "first",
            "second",
        }:
            raise ApprovalError(
                "affine_window_common_drifts_ppm must contain first and second"
            )

        def finite_drift(value: Any, name: str) -> float:
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ApprovalError(f"{name} must be a finite number")
            result = float(value)
            if abs(result) > MAX_ABSOLUTE_AFFINE_DRIFT_PPM:
                raise ApprovalError(f"{name} exceeds the absolute drift limit")
            return result

        first_drift = finite_drift(
            raw_window_drifts["first"],
            "affine_window_common_drifts_ppm.first",
        )
        second_drift = finite_drift(
            raw_window_drifts["second"],
            "affine_window_common_drifts_ppm.second",
        )
        approved_affine_drift_ppm = finite_drift(
            payload["approved_affine_common_drift_ppm"],
            "approved_affine_common_drift_ppm",
        )
        reported_deviation = finite_drift(
            payload["affine_window_common_drift_deviation_ppm"],
            "affine_window_common_drift_deviation_ppm",
        )
        expected_approved = (first_drift + second_drift) / 2.0
        expected_deviation = abs(first_drift - second_drift)
        if not math.isclose(
            approved_affine_drift_ppm,
            expected_approved,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ApprovalError(
                "approved affine common drift is not the two-window median"
            )
        if not math.isclose(
            reported_deviation,
            expected_deviation,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ApprovalError("affine window drift deviation is inconsistent")
        if (
            reported_deviation
            > MAX_STARTUP_HALF_WINDOW_COMMON_DRIFT_DEVIATION_PPM
        ):
            raise ApprovalError(
                "affine startup half-window common drift deviation exceeds 15 ppm"
            )
        affine_window_common_drifts_ppm = (first_drift, second_drift)
    not_before_ns = _exact_int(payload["not_before_unix_ns"], "not_before_unix_ns")
    expires_ns = _exact_int(payload["expires_unix_ns"], "expires_unix_ns")
    if not_before_ns <= 0 or expires_ns <= not_before_ns:
        raise ApprovalError("approval validity interval is invalid")

    approval = FixedOffsetApproval(
        schema=schema,
        session_id=session_id,
        expected_clock_domain=clock_domain,
        writer_gids=tuple(writer_gids),
        writer_source_ipv4=writer_source,
        offset_evidence_sha256=evidence_hash,
        fixed_local_minus_source_offset_ns=offset_ns,
        approved_affine_common_drift_ppm=approved_affine_drift_ppm,
        affine_window_common_drifts_ppm=affine_window_common_drifts_ppm,
        not_before_unix_ns=not_before_ns,
        expires_unix_ns=expires_ns,
    )
    approval.require_valid_at(
        time.time_ns() if now_realtime_ns is None else now_realtime_ns
    )
    return approval


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate a private workstation no-motion fixed-offset approval"
    )
    parser.add_argument("approval_file", type=Path)
    parser.add_argument(
        "--require-affine",
        action="store_true",
        help="reject legacy schema-v2 approvals that do not bind clock drift",
    )
    args = parser.parse_args()
    try:
        load_approval(args.approval_file, require_affine=args.require_affine)
    except ApprovalError as error:
        parser.error(str(error))
    print("timestamp approval is valid for workstation no-motion only")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
