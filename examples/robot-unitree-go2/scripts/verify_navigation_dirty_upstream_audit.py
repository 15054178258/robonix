#!/usr/bin/env python3
"""Verify an exact receipt for the audited Navigation patch."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
from datetime import datetime, timezone


SCHEMA = "robonix-navigation-dirty-audit-v1"
MAX_RECEIPT_BYTES = 16 * 1024
ALLOWED_PATHS = (
    "config.spec",
    "nav2_wrapper/atlas_bridge.py",
    "nav2_wrapper/configuration.py",
    "nav2_wrapper/guarded_launch.py",
    "test_configuration.py",
    "test_guarded_launch.py",
    "test_runtime_integration.py",
)
EXPECTED_KEYS = {
    "schema",
    "repository",
    "repository_relpath",
    "head",
    "status_porcelain_v1_z_base64",
    "tracked_diff_sha256",
    "allowed_paths",
    "issued_at_utc",
    "expires_at_utc",
}


class AuditError(RuntimeError):
    pass


def _git(repo: Path, *arguments: str) -> bytes:
    environment = os.environ.copy()
    for name in (
        "GIT_DIR",
        "GIT_WORK_TREE",
        "GIT_INDEX_FILE",
        "GIT_EXTERNAL_DIFF",
    ):
        environment.pop(name, None)
    environment.update({"LC_ALL": "C", "GIT_PAGER": "cat", "PAGER": "cat"})
    try:
        return subprocess.run(
            ["git", "-C", str(repo), *arguments],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=environment,
        ).stdout
    except (OSError, subprocess.CalledProcessError) as error:
        detail = getattr(error, "stderr", b"").decode("utf-8", "replace").strip()
        raise AuditError(f"git inspection failed: {detail or error}") from error


def _strict_json(raw: bytes) -> dict[str, object]:
    def reject_duplicates(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise AuditError(f"duplicate receipt key: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(raw.decode("utf-8"), object_pairs_hook=reject_duplicates)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AuditError(f"invalid receipt JSON: {error}") from error
    if not isinstance(value, dict) or set(value) != EXPECTED_KEYS:
        raise AuditError("receipt keys do not exactly match the audited schema")
    return value


def _parse_utc(value: object, label: str) -> datetime:
    if not isinstance(value, str) or not re.fullmatch(
        r"[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z", value
    ):
        raise AuditError(f"{label} is not strict UTC seconds")
    try:
        return datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(
            tzinfo=timezone.utc
        )
    except ValueError as error:
        raise AuditError(f"{label} is invalid") from error


def _read_private_receipt(receipt: Path, workspace: Path) -> bytes:
    workspace = workspace.resolve(strict=True)
    absolute = receipt if receipt.is_absolute() else workspace / receipt
    try:
        metadata = os.lstat(absolute)
    except OSError as error:
        raise AuditError(f"cannot inspect audit receipt: {error}") from error
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise AuditError("audit receipt must be a non-symlink regular file")
    if stat.S_IMODE(metadata.st_mode) != 0o600:
        raise AuditError("audit receipt permissions must be exactly 0600")
    if metadata.st_uid != os.geteuid() or metadata.st_nlink != 1:
        raise AuditError("audit receipt must be owned by this user and not hard-linked")
    try:
        resolved = absolute.resolve(strict=True)
        resolved.relative_to(workspace)
    except (OSError, ValueError) as error:
        raise AuditError("audit receipt must resolve inside the deployment workspace") from error

    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(absolute, flags)
        try:
            opened = os.fstat(descriptor)
            if (opened.st_dev, opened.st_ino) != (metadata.st_dev, metadata.st_ino):
                raise AuditError("audit receipt changed while it was being opened")
            raw = os.read(descriptor, MAX_RECEIPT_BYTES + 1)
        finally:
            os.close(descriptor)
    except OSError as error:
        raise AuditError(f"cannot open audit receipt safely: {error}") from error
    if len(raw) > MAX_RECEIPT_BYTES:
        raise AuditError("audit receipt is too large")
    return raw


def verify(workspace: Path, repo: Path, receipt: Path) -> None:
    workspace = workspace.resolve(strict=True)
    repo = repo.resolve(strict=True)
    try:
        repository_relpath = repo.relative_to(workspace).as_posix()
    except ValueError as error:
        raise AuditError("audited Navigation checkout must be inside the workspace") from error
    top_level = Path(_git(repo, "rev-parse", "--show-toplevel").decode().strip()).resolve()
    if top_level != repo:
        raise AuditError("audited Navigation path is not the git worktree root")

    raw_receipt = _read_private_receipt(receipt, workspace)
    data = _strict_json(raw_receipt)
    if data["schema"] != SCHEMA or data["repository"] != "navigation":
        raise AuditError("receipt is not an audited Navigation receipt")
    if data["repository_relpath"] != repository_relpath:
        raise AuditError("receipt Navigation checkout path does not match")
    if data["allowed_paths"] != list(ALLOWED_PATHS):
        raise AuditError("receipt allowed_paths is not the narrow audited Navigation set")

    now = datetime.now(timezone.utc)
    issued = _parse_utc(data["issued_at_utc"], "issued_at_utc")
    expires = _parse_utc(data["expires_at_utc"], "expires_at_utc")
    lifetime = (expires - issued).total_seconds()
    if lifetime <= 0:
        raise AuditError("receipt validity metadata is invalid")
    if issued.timestamp() > now.timestamp() + 60:
        raise AuditError("receipt issue time is in the future")

    head = _git(repo, "rev-parse", "HEAD").decode("ascii").strip()
    if not re.fullmatch(r"[0-9a-f]{40}|[0-9a-f]{64}", head):
        raise AuditError("Navigation HEAD is not a canonical object id")
    if data["head"] != head:
        raise AuditError("audit receipt HEAD does not match Navigation HEAD")

    untracked = _git(repo, "ls-files", "--others", "--exclude-standard", "-z")
    if untracked:
        raise AuditError("audited Navigation checkout contains untracked files")
    changed_paths_raw = _git(repo, "diff", "--name-only", "-z", "HEAD", "--")
    changed_paths = tuple(
        item.decode("utf-8", "surrogateescape")
        for item in changed_paths_raw.rstrip(b"\0").split(b"\0")
        if item
    )
    if changed_paths != ALLOWED_PATHS:
        raise AuditError(
            "Navigation tracked changes are not exactly the audited Navigation files"
        )
    for relative in ALLOWED_PATHS:
        target = repo / relative
        try:
            target_metadata = os.lstat(target)
        except OSError as error:
            raise AuditError(f"cannot inspect audited Navigation file: {relative}") from error
        if not stat.S_ISREG(target_metadata.st_mode) or stat.S_ISLNK(target_metadata.st_mode):
            raise AuditError(f"audited Navigation path is not a regular file: {relative}")

    status = _git(repo, "status", "--porcelain=v1", "-z", "--untracked-files=normal")
    try:
        expected_status = base64.b64decode(
            data["status_porcelain_v1_z_base64"], validate=True
        )
    except (TypeError, ValueError) as error:
        raise AuditError("receipt status encoding is invalid") from error
    if expected_status != status:
        raise AuditError("audit receipt exact porcelain status does not match")

    tracked_diff = _git(
        repo,
        "-c",
        "diff.noprefix=false",
        "diff",
        "--binary",
        "--full-index",
        "--no-ext-diff",
        "--no-textconv",
        "--no-renames",
        "--src-prefix=a/",
        "--dst-prefix=b/",
        "HEAD",
        "--",
    )
    actual_diff_hash = hashlib.sha256(tracked_diff).hexdigest()
    expected_diff_hash = data["tracked_diff_sha256"]
    if not isinstance(expected_diff_hash, str) or not re.fullmatch(
        r"[0-9a-f]{64}", expected_diff_hash
    ):
        raise AuditError("receipt tracked diff hash is invalid")
    if expected_diff_hash != actual_diff_hash:
        raise AuditError("audit receipt full tracked diff hash does not match")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--receipt", required=True, type=Path)
    arguments = parser.parse_args()
    try:
        verify(arguments.workspace, arguments.repo, arguments.receipt)
    except AuditError as error:
        print(f"dirty upstream audit rejected: {error}", file=sys.stderr)
        return 1
    print("[upstream-compat] separately audited Navigation dirty patch accepted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
