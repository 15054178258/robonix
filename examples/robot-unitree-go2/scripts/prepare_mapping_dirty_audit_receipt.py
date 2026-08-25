#!/usr/bin/env python3
"""Create a short-lived receipt for the exact audited Mapping worktree patch."""

from __future__ import annotations

import argparse
import base64
from collections.abc import Callable
from datetime import datetime, timedelta, timezone
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import stat

from verify_dirty_upstream_audit import (
    ALLOWED_PATHS,
    AuditError,
    SCHEMA,
    _git,
    verify,
)


MIN_LIFETIME_SECONDS = 60
MAX_LIFETIME_SECONDS = 4 * 60 * 60


def _inspect_exact_patch(
    workspace: Path,
    repo: Path,
    *,
    repository_label: str = "Mapping",
    allowed_paths: tuple[str, ...] = ALLOWED_PATHS,
    git=_git,
    error_type: type[Exception] = AuditError,
) -> dict[str, object]:
    workspace = workspace.resolve(strict=True)
    repo = repo.resolve(strict=True)
    try:
        repository_relpath = repo.relative_to(workspace).as_posix()
    except ValueError as error:
        raise error_type(
            f"audited {repository_label} checkout must be inside the workspace"
        ) from error

    top_level = Path(git(repo, "rev-parse", "--show-toplevel").decode().strip()).resolve()
    if top_level != repo:
        raise error_type(
            f"audited {repository_label} path is not the git worktree root"
        )

    head = git(repo, "rev-parse", "HEAD").decode("ascii").strip()
    if not re.fullmatch(r"[0-9a-f]{40}|[0-9a-f]{64}", head):
        raise error_type(f"{repository_label} HEAD is not a canonical object id")

    untracked = git(repo, "ls-files", "--others", "--exclude-standard", "-z")
    if untracked:
        raise error_type(
            f"audited {repository_label} checkout contains untracked files"
        )

    changed_paths_raw = git(repo, "diff", "--name-only", "-z", "HEAD", "--")
    changed_paths = tuple(
        item.decode("utf-8", "surrogateescape")
        for item in changed_paths_raw.rstrip(b"\0").split(b"\0")
        if item
    )
    if changed_paths != allowed_paths:
        raise error_type(
            f"{repository_label} tracked changes are not exactly the audited "
            f"{repository_label} files"
        )

    for relative in allowed_paths:
        target = repo / relative
        try:
            metadata = os.lstat(target)
        except OSError as error:
            raise error_type(
                f"cannot inspect audited {repository_label} file: {relative}"
            ) from error
        if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
            raise error_type(
                f"audited {repository_label} path is not a regular file: {relative}"
            )

    status = git(repo, "status", "--porcelain=v1", "-z", "--untracked-files=normal")
    tracked_diff = git(
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
    return {
        "repository_relpath": repository_relpath,
        "head": head,
        "status_porcelain_v1_z_base64": base64.b64encode(status).decode("ascii"),
        "tracked_diff_sha256": hashlib.sha256(tracked_diff).hexdigest(),
    }


def _atomic_private_publish(
    workspace: Path,
    output: Path,
    payload: bytes,
    *,
    validator: Callable[[Path], None],
    error_type: type[Exception] = AuditError,
) -> Path:
    """Validate a complete private temp file, then publish without replacement.

    The final pathname is never visible with partial contents.  A same-directory
    hard-link is the portable Linux/Python no-replace publication primitive:
    the link either creates the complete target atomically or fails with
    ``EEXIST``.  Cleanup is inode-qualified so a concurrently substituted path
    is never removed on our behalf.
    """
    workspace = workspace.resolve(strict=True)
    absolute = output if output.is_absolute() else workspace / output
    try:
        parent = absolute.parent.resolve(strict=True)
        parent.relative_to(workspace)
    except (OSError, ValueError) as error:
        raise error_type(
            "audit receipt parent must already exist inside the deployment workspace"
        ) from error
    target_name = absolute.name
    if target_name in {"", ".", ".."}:
        raise error_type("audit receipt output must have a regular file name")
    target = parent / target_name

    directory_flags = (
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    create_flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    read_flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )

    parent_fd = -1
    temp_fd = -1
    temp_name = ""
    created_identity: tuple[int, int] | None = None
    temp_exists = False
    target_published = False

    def _path_identity_matches(name: str) -> bool:
        if created_identity is None:
            return False
        try:
            metadata = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        except (FileNotFoundError, OSError):
            return False
        return (metadata.st_dev, metadata.st_ino) == created_identity

    def _unlink_ours(name: str) -> None:
        if name and _path_identity_matches(name):
            try:
                os.unlink(name, dir_fd=parent_fd)
            except FileNotFoundError:
                pass

    def _assert_parent_identity(expected: os.stat_result) -> None:
        try:
            current = os.lstat(parent)
        except OSError as error:
            raise error_type("audit receipt parent changed during publication") from error
        if (
            stat.S_ISLNK(current.st_mode)
            or not stat.S_ISDIR(current.st_mode)
            or (current.st_dev, current.st_ino)
            != (expected.st_dev, expected.st_ino)
        ):
            raise error_type("audit receipt parent changed during publication")

    def _assert_private_payload(name: str) -> None:
        descriptor = -1
        try:
            descriptor = os.open(name, read_flags, dir_fd=parent_fd)
            metadata = os.fstat(descriptor)
            if (
                created_identity is None
                or (metadata.st_dev, metadata.st_ino) != created_identity
                or not stat.S_ISREG(metadata.st_mode)
                or stat.S_IMODE(metadata.st_mode) != 0o600
                or metadata.st_uid != os.geteuid()
                or metadata.st_nlink != 1
            ):
                raise error_type("audit receipt file identity changed during publication")
            observed = bytearray()
            while len(observed) <= len(payload):
                chunk = os.read(descriptor, min(65536, len(payload) + 1 - len(observed)))
                if not chunk:
                    break
                observed.extend(chunk)
            if bytes(observed) != payload:
                raise error_type("audit receipt contents changed during publication")
        except OSError as error:
            raise error_type(f"cannot re-open private audit receipt: {error}") from error
        finally:
            if descriptor >= 0:
                os.close(descriptor)

    try:
        parent_before = os.lstat(parent)
        if stat.S_ISLNK(parent_before.st_mode) or not stat.S_ISDIR(parent_before.st_mode):
            raise error_type("audit receipt parent must be a non-symlink directory")
        parent_fd = os.open(parent, directory_flags)
        parent_opened = os.fstat(parent_fd)
        if (parent_opened.st_dev, parent_opened.st_ino) != (
            parent_before.st_dev,
            parent_before.st_ino,
        ):
            raise error_type("audit receipt parent changed while it was opened")
        _assert_parent_identity(parent_opened)

        try:
            os.stat(target_name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            raise error_type("audit receipt output already exists")

        for _ in range(32):
            candidate = f".robonix-audit-tmp-{secrets.token_hex(16)}"
            try:
                temp_fd = os.open(candidate, create_flags, 0o600, dir_fd=parent_fd)
            except FileExistsError:
                continue
            temp_name = candidate
            temp_exists = True
            opened_temp = os.fstat(temp_fd)
            created_identity = (opened_temp.st_dev, opened_temp.st_ino)
            break
        if temp_fd < 0:
            raise error_type("could not allocate a unique private audit receipt")

        os.fchmod(temp_fd, 0o600)
        offset = 0
        while offset < len(payload):
            written = os.write(temp_fd, payload[offset:])
            if written <= 0:
                raise OSError(errno.EIO, "short audit receipt write")
            offset += written
        os.fsync(temp_fd)
        created = os.fstat(temp_fd)
        if (
            (created.st_dev, created.st_ino) != created_identity
            or not stat.S_ISREG(created.st_mode)
            or stat.S_IMODE(created.st_mode) != 0o600
            or created.st_uid != os.geteuid()
            or created.st_nlink != 1
        ):
            raise error_type("private audit receipt metadata is invalid")
        os.close(temp_fd)
        temp_fd = -1
        os.fsync(parent_fd)

        _assert_parent_identity(parent_opened)
        validator(parent / temp_name)
        _assert_parent_identity(parent_opened)
        _assert_private_payload(temp_name)

        try:
            os.link(
                temp_name,
                target_name,
                src_dir_fd=parent_fd,
                dst_dir_fd=parent_fd,
                follow_symlinks=False,
            )
        except FileExistsError as error:
            raise error_type("audit receipt output already exists") from error
        target_published = True
        os.unlink(temp_name, dir_fd=parent_fd)
        temp_exists = False
        os.fsync(parent_fd)

        _assert_parent_identity(parent_opened)
        validator(target)
        _assert_private_payload(target_name)
        return target
    except BaseException as error:
        if temp_fd >= 0:
            try:
                os.close(temp_fd)
            except OSError:
                pass
            temp_fd = -1
        if parent_fd >= 0:
            if target_published:
                _unlink_ours(target_name)
            if temp_exists:
                _unlink_ours(temp_name)
            try:
                os.fsync(parent_fd)
            except OSError:
                pass
        if isinstance(error, (KeyboardInterrupt, SystemExit)):
            raise
        if isinstance(error, error_type):
            raise
        if isinstance(error, OSError):
            raise error_type(f"cannot create private audit receipt: {error}") from error
        raise
    finally:
        if temp_fd >= 0:
            try:
                os.close(temp_fd)
            except OSError:
                pass
        if parent_fd >= 0:
            try:
                os.close(parent_fd)
            except OSError:
                pass


def prepare(
    workspace: Path,
    repo: Path,
    output: Path,
    valid_for_seconds: int,
) -> Path:
    if not MIN_LIFETIME_SECONDS <= valid_for_seconds <= MAX_LIFETIME_SECONDS:
        raise AuditError("receipt lifetime must be between 60 seconds and four hours")

    workspace = workspace.resolve(strict=True)
    repo = repo.resolve(strict=True)
    patch = _inspect_exact_patch(workspace, repo, error_type=AuditError)
    issued = datetime.now(timezone.utc).replace(microsecond=0)
    expires = issued + timedelta(seconds=valid_for_seconds)
    data = {
        "schema": SCHEMA,
        "repository": "mapping",
        **patch,
        "allowed_paths": list(ALLOWED_PATHS),
        "issued_at_utc": issued.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "expires_at_utc": expires.strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    raw = (json.dumps(data, indent=2, sort_keys=True) + "\n").encode("utf-8")
    return _atomic_private_publish(
        workspace,
        output,
        raw,
        validator=lambda receipt: verify(workspace, repo, receipt),
        error_type=AuditError,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--valid-for-seconds", type=int, default=3600)
    arguments = parser.parse_args()
    try:
        receipt = prepare(
            arguments.workspace,
            arguments.repo,
            arguments.output,
            arguments.valid_for_seconds,
        )
    except AuditError as error:
        print(f"dirty upstream audit receipt rejected: {error}", file=os.sys.stderr)
        return 1
    print(receipt)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
