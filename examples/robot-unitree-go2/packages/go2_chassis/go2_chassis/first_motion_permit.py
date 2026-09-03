"""Validate and atomically consume a short-lived first-motion permit.

The permit is deliberately separate from the Robonix manifest.  A committed
manifest can describe the commissioning graph, but it cannot authorize a
physical run.  Initialization consumes one private regular file and binds it
to the exact runtime envelope plus three immutable evidence files.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import stat
import time
from typing import Mapping

from .runtime_config import (
    FIRST_MOTION_COMMAND_TOPIC,
    FIRST_MOTION_COMMAND_TIMEOUT_S,
    FIRST_MOTION_MAX_DISTANCE_M,
    FIRST_MOTION_MAX_DURATION_S,
    FIRST_MOTION_MAX_VX_MPS,
    FIRST_MOTION_MAX_VY_MPS,
    FIRST_MOTION_MAX_WZ_RPS,
    FIRST_MOTION_ODOM_TOPIC,
    FIRST_MOTION_PROFILE,
    RuntimeConfig,
)


PERMIT_SCHEMA = "robonix-go2-first-motion-permit-v1"
PERMIT_ENV = "GO2_FIRST_MOTION_PERMIT_FILE"
FIRST_MOTION_ACK = "I_APPROVE_GO2_FIRST_10CM_MOTION"
MAX_LIFETIME_NS = 5 * 60 * 1_000_000_000
MIN_REMAINING_NS = 15 * 1_000_000_000
MAX_PERMIT_BYTES = 64 * 1024
_HEX_RE = re.compile(r"^[0-9a-f]{64}$")
_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{7,127}$")


class PermitError(ValueError):
    """A permit is missing, replayed, expired, unbound, or malformed."""


def _owned_regular_file(path: Path, *, private: bool) -> os.stat_result:
    try:
        info = os.lstat(path)
    except FileNotFoundError as exc:
        raise PermitError(f"required file does not exist: {path}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise PermitError(f"file must be a non-symlink regular file: {path}")
    if info.st_uid != os.geteuid():
        raise PermitError(f"file is not owned by the current UID: {path}")
    mode = stat.S_IMODE(info.st_mode)
    if private and mode != 0o600:
        raise PermitError(f"permit mode must be exactly 0600, got {mode:04o}")
    if not private and mode & 0o022:
        raise PermitError(f"evidence file is group/world writable: {path}")
    return info


def _read_private_json(path: Path) -> dict[str, object]:
    expected = _owned_regular_file(path, private=True)
    flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW
    descriptor = os.open(path, flags)
    try:
        opened = os.fstat(descriptor)
        if (
            opened.st_dev != expected.st_dev
            or opened.st_ino != expected.st_ino
            or opened.st_uid != os.geteuid()
            or not stat.S_ISREG(opened.st_mode)
        ):
            raise PermitError("permit changed while it was being opened")
        raw = os.read(descriptor, MAX_PERMIT_BYTES + 1)
        if len(raw) > MAX_PERMIT_BYTES:
            raise PermitError("permit is unreasonably large")
    finally:
        os.close(descriptor)
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PermitError("permit is not valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise PermitError("permit root must be a JSON object")
    return payload


def sha256_file(path: Path) -> str:
    _owned_regular_file(path, private=False)
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _exact_number(value: object, expected: float, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PermitError(f"{label} must be numeric")
    if float(value) != expected:
        raise PermitError(f"{label} does not match the fixed envelope")


def validate_permit(
    payload: Mapping[str, object],
    runtime: RuntimeConfig,
    package_root: Path,
    *,
    now_unix_ns: int | None = None,
) -> str:
    """Return the permit id after validating every immutable claim."""
    now_ns = time.time_ns() if now_unix_ns is None else now_unix_ns
    if payload.get("schema") != PERMIT_SCHEMA:
        raise PermitError("unexpected first-motion permit schema")
    permit_id = payload.get("permit_id")
    session_id = payload.get("session_id")
    if not isinstance(permit_id, str) or _ID_RE.fullmatch(permit_id) is None:
        raise PermitError("permit_id is malformed")
    if not isinstance(session_id, str) or _ID_RE.fullmatch(session_id) is None:
        raise PermitError("session_id is malformed")
    if payload.get("one_time") is not True:
        raise PermitError("permit must explicitly be one_time")
    issued_ns = payload.get("issued_unix_ns")
    expires_ns = payload.get("expires_unix_ns")
    if (
        isinstance(issued_ns, bool)
        or isinstance(expires_ns, bool)
        or not isinstance(issued_ns, int)
        or not isinstance(expires_ns, int)
        or issued_ns <= 0
        or expires_ns <= issued_ns
        or expires_ns - issued_ns > MAX_LIFETIME_NS
    ):
        raise PermitError("permit lifetime is invalid or exceeds five minutes")
    if now_ns < issued_ns or expires_ns - now_ns < MIN_REMAINING_NS:
        raise PermitError("permit is not current or has less than 15 seconds left")

    exact_text = {
        "profile": FIRST_MOTION_PROFILE,
        "operator_ack": FIRST_MOTION_ACK,
        "network_interface": runtime.network_interface,
        "state_topic": runtime.state_topic,
        "command_topic": FIRST_MOTION_COMMAND_TOPIC,
        "odom_topic": FIRST_MOTION_ODOM_TOPIC,
        "arm_service": runtime.arm_service,
    }
    for name, expected in exact_text.items():
        if payload.get(name) != expected:
            raise PermitError(f"permit {name} does not match this runtime")
    if payload.get("allowed_modes") != list(runtime.allowed_modes):
        raise PermitError("permit allowed_modes does not match this runtime")
    if payload.get("allowed_state_markers") != list(
        runtime.allowed_state_markers
    ):
        raise PermitError(
            "permit allowed_state_markers does not match this runtime"
        )
    _exact_number(payload.get("max_linear_x_mps"), FIRST_MOTION_MAX_VX_MPS,
                  "max_linear_x_mps")
    _exact_number(payload.get("max_linear_y_mps"), FIRST_MOTION_MAX_VY_MPS,
                  "max_linear_y_mps")
    _exact_number(payload.get("max_angular_z_rps"), FIRST_MOTION_MAX_WZ_RPS,
                  "max_angular_z_rps")
    _exact_number(payload.get("max_duration_s"), FIRST_MOTION_MAX_DURATION_S,
                  "max_duration_s")
    _exact_number(payload.get("max_distance_m"), FIRST_MOTION_MAX_DISTANCE_M,
                  "max_distance_m")
    _exact_number(payload.get("command_timeout_s"),
                  FIRST_MOTION_COMMAND_TIMEOUT_S, "command_timeout_s")

    evidence = payload.get("evidence")
    if not isinstance(evidence, dict) or set(evidence) != {
        "dds_identity", "state", "time"
    }:
        raise PermitError("permit must bind exactly DDS identity, state, and time evidence")
    resolved_root = package_root.resolve()
    for name in ("dds_identity", "state", "time"):
        claim = evidence[name]
        if not isinstance(claim, dict) or set(claim) != {"path", "sha256"}:
            raise PermitError(f"{name} evidence claim is malformed")
        path_text = claim.get("path")
        expected_hash = claim.get("sha256")
        if not isinstance(path_text, str) or not Path(path_text).is_absolute():
            raise PermitError(f"{name} evidence path must be absolute")
        if not isinstance(expected_hash, str) or _HEX_RE.fullmatch(expected_hash) is None:
            raise PermitError(f"{name} evidence hash is malformed")
        evidence_path = Path(path_text).resolve()
        try:
            evidence_path.relative_to(resolved_root)
        except ValueError as exc:
            raise PermitError(f"{name} evidence is outside the package root") from exc
        if sha256_file(evidence_path) != expected_hash:
            raise PermitError(f"{name} evidence hash changed")
    return permit_id


def consume_first_motion_permit(
    runtime: RuntimeConfig,
    environ: Mapping[str, str],
    package_root: Path,
) -> Path:
    """Validate and rename the permit so a second initialization cannot replay it."""
    if not runtime.allow_motion:
        raise PermitError("permit consumption is only valid for a motion runtime")
    path_text = environ.get(PERMIT_ENV, "").strip()
    if not path_text:
        raise PermitError(f"motion requires {PERMIT_ENV}")
    permit_path = Path(path_text)
    if not permit_path.is_absolute():
        raise PermitError("first-motion permit path must be absolute")
    parent_info = os.lstat(permit_path.parent)
    if (
        not stat.S_ISDIR(parent_info.st_mode)
        or stat.S_ISLNK(parent_info.st_mode)
        or parent_info.st_uid != os.geteuid()
        or stat.S_IMODE(parent_info.st_mode) != 0o700
    ):
        raise PermitError("permit parent must be an owned non-symlink 0700 directory")
    payload = _read_private_json(permit_path)
    permit_id = validate_permit(payload, runtime, package_root)
    consumed_path = permit_path.with_name(f"{permit_path.name}.consumed-{permit_id}")
    if consumed_path.exists():
        raise PermitError("permit was already consumed")
    try:
        os.rename(permit_path, consumed_path)
    except FileNotFoundError as exc:
        raise PermitError("permit was consumed concurrently") from exc
    return consumed_path
