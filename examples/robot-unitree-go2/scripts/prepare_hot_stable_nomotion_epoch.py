#!/usr/bin/env python3
"""Prepare a fresh, bounded no-motion affine epoch after thermal stabilization.

This orchestrator deliberately requires an explicit retained identity directory.
It proves that the current four raw-topic writer GIDs match that evidence both
before and after a new 75-second subscription-only time capture, and only then
invokes the existing offline approval generator.  It never captures a new
identity implicitly and never starts the corrected stack or a motion surface.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import time
from typing import Any, Callable, Sequence


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC_DIR = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC_DIR))
sys.path.insert(0, str(ROOT / "scripts"))

from verify_workstation_writer_identity_readonly import (  # noqa: E402
    SCHEMA as IDENTITY_RECEIPT_SCHEMA,
    _private_identity_directory,
)
from workstation_nomotion_approval import (  # noqa: E402
    ACK,
    EXPECTED_RAW_TOPICS,
    load_approval,
)


THERMAL_STABLE_ACK = (
    "I_CONFIRM_THE_ROBOT_IS_STATIONARY_AND_TIME_SOURCES_ARE_THERMALLY_STABLE"
)
EPOCH_SCHEMA = "robonix-go2-hot-stable-nomotion-epoch-v1"
TIME_CAPTURE_SECONDS = 75
DEFAULT_VALIDITY_SECONDS = 900
MIN_VALIDITY_SECONDS = 300
MAX_VALIDITY_SECONDS = 3600
MAX_RECEIPT_BYTES = 8 * 1024 * 1024
_SESSION_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{7,95}$")

IDENTITY_WRAPPER = ROOT / "scripts" / "verify_workstation_writer_identity_readonly.sh"
TIME_PROBE_WRAPPER = ROOT / "scripts" / "probe_go2_time_readonly.sh"
APPROVAL_GENERATOR = ROOT / "scripts" / "prepare_workstation_nomotion_offset_approval.py"


class EpochError(RuntimeError):
    """A stage failed, so no usable epoch approval may be claimed."""


def _bounded_validity(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("valid-for-seconds must be an integer") from error
    if not MIN_VALIDITY_SECONDS <= parsed <= MAX_VALIDITY_SECONDS:
        raise argparse.ArgumentTypeError(
            f"valid-for-seconds must be in {MIN_VALIDITY_SECONDS}..{MAX_VALIDITY_SECONDS}"
        )
    return parsed


def _absolute_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("evidence/output paths must be absolute")
    return path


def _session_id(value: str) -> str:
    if _SESSION_RE.fullmatch(value) is None:
        raise argparse.ArgumentTypeError("session-id is invalid")
    return value


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "After an operator-confirmed thermal soak, take fresh 75-second "
            "read-only time evidence and prepare one short no-motion approval"
        )
    )
    parser.add_argument(
        "--identity-evidence-dir",
        required=True,
        type=_absolute_path,
        help="explicit retained same-writer-session identity directory; no default",
    )
    parser.add_argument(
        "--thermal-stable-ack",
        required=True,
        help=f"must be exactly: {THERMAL_STABLE_ACK}",
    )
    parser.add_argument(
        "--operator-ack",
        required=True,
        help=f"must be exactly: {ACK}",
    )
    parser.add_argument(
        "--valid-for-seconds",
        type=_bounded_validity,
        default=DEFAULT_VALIDITY_SECONDS,
        help=(
            f"bounded no-motion validity, {MIN_VALIDITY_SECONDS}.."
            f"{MAX_VALIDITY_SECONDS} seconds (default: {DEFAULT_VALIDITY_SECONDS})"
        ),
    )
    parser.add_argument("--session-id", type=_session_id)
    parser.add_argument("--epoch-dir", type=_absolute_path)
    parser.add_argument("--approval-output", type=_absolute_path)
    return parser


@dataclass(frozen=True, slots=True)
class EpochPlan:
    identity_directory: Path
    session_id: str
    epoch_directory: Path
    time_directory: Path
    pre_identity_receipt: Path
    post_identity_receipt: Path
    approval_output: Path
    approval_manifest: Path
    epoch_receipt: Path
    validity_seconds: int

    def commands(self) -> tuple[tuple[str, tuple[str, ...]], ...]:
        identity = str(self.identity_directory)
        return (
            (
                "identity-before",
                (
                    "bash",
                    str(IDENTITY_WRAPPER),
                    "--identity-evidence-dir",
                    identity,
                    "--receipt",
                    str(self.pre_identity_receipt),
                    "--phase",
                    "before-time-capture",
                ),
            ),
            (
                "time-capture-75s",
                (
                    "bash",
                    str(TIME_PROBE_WRAPPER),
                    str(self.time_directory),
                    str(TIME_CAPTURE_SECONDS),
                ),
            ),
            (
                "identity-after",
                (
                    "bash",
                    str(IDENTITY_WRAPPER),
                    "--identity-evidence-dir",
                    identity,
                    "--receipt",
                    str(self.post_identity_receipt),
                    "--phase",
                    "after-time-capture",
                ),
            ),
            (
                "approval-generation",
                (
                    sys.executable,
                    str(APPROVAL_GENERATOR),
                    "--time-evidence-dir",
                    str(self.time_directory),
                    "--identity-evidence-dir",
                    identity,
                    "--session-id",
                    self.session_id,
                    "--operator-ack",
                    ACK,
                    "--valid-for-seconds",
                    str(self.validity_seconds),
                    "--output",
                    str(self.approval_output),
                ),
            ),
        )


def _new_session_id(now: float | None = None) -> str:
    stamp = time.strftime(
        "%Y%m%dT%H%M%SZ", time.gmtime(time.time() if now is None else now)
    )
    return f"workstation-hot-stable-nomotion-{stamp}-{os.getpid()}"


def _path_below(path: Path, root: Path, label: str) -> Path:
    absolute = Path(os.path.abspath(os.fspath(path)))
    resolved_root = root.resolve(strict=True)
    parent = absolute.parent.resolve(strict=True)
    candidate = parent / absolute.name
    if candidate == resolved_root or not candidate.is_relative_to(resolved_root):
        raise EpochError(f"{label} must be below {resolved_root}")
    return candidate


def build_plan(arguments: argparse.Namespace, *, now: float | None = None) -> EpochPlan:
    if arguments.thermal_stable_ack != THERMAL_STABLE_ACK:
        raise EpochError("thermal-stable acknowledgement does not match exactly")
    if arguments.operator_ack != ACK:
        raise EpochError("no-motion operator acknowledgement does not match exactly")

    identity_directory = _private_identity_directory(arguments.identity_evidence_dir)
    session_id = arguments.session_id or _new_session_id(now)
    if _SESSION_RE.fullmatch(session_id) is None:
        raise EpochError("generated session-id is invalid")

    logs_root = (ROOT / "logs" / "go2-readonly").resolve(strict=True)
    run_root = (ROOT / "rbnx-build" / "run").resolve(strict=True)
    epoch_candidate = arguments.epoch_dir or (logs_root / f"{session_id}-epoch")
    approval_candidate = arguments.approval_output or (
        run_root / f"{session_id}-approval.json"
    )
    epoch_directory = _path_below(epoch_candidate, logs_root, "epoch directory")
    approval_output = _path_below(
        approval_candidate, run_root, "approval output"
    )
    if os.path.lexists(epoch_directory):
        raise EpochError(f"refusing to reuse epoch directory: {epoch_directory}")
    if os.path.lexists(approval_output):
        raise EpochError(f"refusing to overwrite approval: {approval_output}")
    approval_manifest = approval_output.with_name(
        approval_output.name + ".evidence-manifest.json"
    )
    if os.path.lexists(approval_manifest):
        raise EpochError(f"refusing to overwrite approval manifest: {approval_manifest}")

    return EpochPlan(
        identity_directory=identity_directory,
        session_id=session_id,
        epoch_directory=epoch_directory,
        time_directory=epoch_directory / "time-evidence",
        pre_identity_receipt=epoch_directory / "identity-before.json",
        post_identity_receipt=epoch_directory / "identity-after.json",
        approval_output=approval_output,
        approval_manifest=approval_manifest,
        epoch_receipt=epoch_directory / "epoch-receipt.json",
        validity_seconds=arguments.valid_for_seconds,
    )


def _read_private_json(path: Path, label: str) -> dict[str, Any]:
    try:
        before = os.lstat(path)
    except OSError as error:
        raise EpochError(f"cannot inspect {label}: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise EpochError(f"{label} must be a real regular file")
    if before.st_uid != os.geteuid() or stat.S_IMODE(before.st_mode) & 0o077:
        raise EpochError(f"{label} must be private and current-user-owned")
    if not 0 < before.st_size <= MAX_RECEIPT_BYTES:
        raise EpochError(f"{label} size is invalid")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise EpochError(f"cannot read {label}: {error}") from error
    after = os.lstat(path)
    if (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    ) != (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns):
        raise EpochError(f"{label} changed while it was read")
    if not isinstance(payload, dict):
        raise EpochError(f"{label} must contain a JSON object")
    return payload


def validate_identity_receipt_pair(
    before: dict[str, Any], after: dict[str, Any], identity_directory: Path
) -> dict[str, str]:
    for label, receipt, phase in (
        ("before identity receipt", before, "before-time-capture"),
        ("after identity receipt", after, "after-time-capture"),
    ):
        if receipt.get("schema") != IDENTITY_RECEIPT_SCHEMA:
            raise EpochError(f"{label} schema is invalid")
        if receipt.get("phase") != phase or receipt.get("result") != "pass":
            raise EpochError(f"{label} did not pass its declared phase")
        if receipt.get("identity_evidence_directory") != str(identity_directory):
            raise EpochError(f"{label} is not bound to the explicit identity directory")
        if receipt.get("identity_evidence_unchanged") is not True:
            raise EpochError(f"{label} did not preserve the identity evidence")
        graph = receipt.get("graph")
        if not isinstance(graph, dict) or graph.get("passed") is not True:
            raise EpochError(f"{label} graph result is not pass")
        if receipt.get("ros_subscriptions_created") is not False:
            raise EpochError(f"{label} subscription sentinel is not false")
        if receipt.get("ros_publishers_created") is not False:
            raise EpochError(f"{label} publisher sentinel is not false")
        if receipt.get("unitree_clients_created") is not False:
            raise EpochError(f"{label} Unitree-client sentinel is not false")
        if receipt.get("motion_ready") is not False:
            raise EpochError(f"{label} motion sentinel is not false")

    before_gids = before.get("expected_writer_gids")
    after_gids = after.get("expected_writer_gids")
    if (
        not isinstance(before_gids, dict)
        or set(before_gids) != set(EXPECTED_RAW_TOPICS)
        or before_gids != after_gids
    ):
        raise EpochError("writer GIDs did not remain identical around the time capture")
    if before.get("identity_evidence") != after.get("identity_evidence"):
        raise EpochError("retained identity evidence changed around the time capture")
    return {str(stream): str(gid) for stream, gid in before_gids.items()}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _write_epoch_receipt(
    plan: EpochPlan,
    expected_writer_gids: dict[str, str],
) -> None:
    approval = load_approval(plan.approval_output, require_affine=True)
    if approval.session_id != plan.session_id:
        raise EpochError("generated approval session does not match the epoch")
    if dict(approval.writer_gids) != expected_writer_gids:
        raise EpochError("generated approval GIDs do not match the bracketing checks")
    time_files = ("metadata.json", "samples.jsonl", "summary.json")
    payload = {
        "schema": EPOCH_SCHEMA,
        "session_id": plan.session_id,
        "motion_enabled": False,
        "thermal_stable_operator_assertion": True,
        "identity_directory_was_explicit": True,
        "writer_identity_bracketed_around_time_capture": True,
        "time_capture_seconds": TIME_CAPTURE_SECONDS,
        "validity_seconds": plan.validity_seconds,
        "identity_evidence_directory": str(plan.identity_directory),
        "writer_gids": expected_writer_gids,
        "artifacts": {
            "identity_before": {
                "file": str(plan.pre_identity_receipt),
                "sha256": _sha256(plan.pre_identity_receipt),
            },
            "identity_after": {
                "file": str(plan.post_identity_receipt),
                "sha256": _sha256(plan.post_identity_receipt),
            },
            "time_evidence": {
                name: {
                    "file": str(plan.time_directory / name),
                    "sha256": _sha256(plan.time_directory / name),
                }
                for name in time_files
            },
            "approval": {
                "file": str(plan.approval_output),
                "sha256": _sha256(plan.approval_output),
            },
            "approval_manifest": {
                "file": str(plan.approval_manifest),
                "sha256": _sha256(plan.approval_manifest),
            },
        },
        "not_before_unix_ns": approval.not_before_unix_ns,
        "expires_unix_ns": approval.expires_unix_ns,
        "future_allowance_changed": False,
        "automatic_relock_enabled": False,
        "stack_started": False,
        "motion_ready": False,
        "canonical_odom_ready": False,
    }
    encoded = (
        json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode("utf-8")
    descriptor = os.open(
        plan.epoch_receipt,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
        0o600,
    )
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        plan.epoch_receipt.unlink(missing_ok=True)
        raise
    finally:
        os.close(descriptor)


Runner = Callable[[Sequence[str], dict[str, str], float], int]


def _default_runner(command: Sequence[str], environment: dict[str, str], timeout: float) -> int:
    return subprocess.run(
        list(command),
        cwd=ROOT,
        env=environment,
        timeout=timeout,
        check=False,
    ).returncode


def execute_plan(plan: EpochPlan, *, runner: Runner = _default_runner) -> None:
    plan.epoch_directory.mkdir(mode=0o700, parents=False, exist_ok=False)
    os.chmod(plan.epoch_directory, 0o700)
    environment = os.environ.copy()
    environment.update(
        {
            "GO2_ALLOW_MOTION": "false",
            "GO2_MOTION_MODE": "",
            "GO2_CHASSIS_ACKNOWLEDGEMENT": "",
        }
    )
    timeouts = {
        "identity-before": 50.0,
        "time-capture-75s": 105.0,
        "identity-after": 50.0,
        "approval-generation": 180.0,
    }
    commands = plan.commands()
    for stage, command in commands[:3]:
        try:
            status = runner(command, environment, timeouts[stage])
        except subprocess.TimeoutExpired as error:
            raise EpochError(f"{stage} exceeded its outer timeout") from error
        if status != 0:
            raise EpochError(f"{stage} failed with status {status}")

    before = _read_private_json(plan.pre_identity_receipt, "before identity receipt")
    after = _read_private_json(plan.post_identity_receipt, "after identity receipt")
    expected_writer_gids = validate_identity_receipt_pair(
        before, after, plan.identity_directory
    )

    stage, command = commands[3]
    try:
        status = runner(command, environment, timeouts[stage])
    except subprocess.TimeoutExpired as error:
        raise EpochError(f"{stage} exceeded its outer timeout") from error
    if status != 0:
        raise EpochError(f"{stage} failed with status {status}")
    _write_epoch_receipt(plan, expected_writer_gids)


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        plan = build_plan(arguments)
        execute_plan(plan)
    except (OSError, ValueError, EpochError) as error:
        print(f"hot-stable no-motion epoch preparation failed: {error}", file=sys.stderr)
        return 2
    print(f"PASS: fresh no-motion epoch prepared: {plan.epoch_receipt}")
    print(f"approval: {plan.approval_output}")
    print("stack was not started; motion remains disabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
