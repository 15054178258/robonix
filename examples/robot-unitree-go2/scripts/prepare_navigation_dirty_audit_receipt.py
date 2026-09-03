#!/usr/bin/env python3
"""Create a short-lived receipt for the exact audited Navigation worktree patch."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path

from prepare_mapping_dirty_audit_receipt import (
    MAX_LIFETIME_SECONDS,
    MIN_LIFETIME_SECONDS,
    _atomic_private_publish,
    _inspect_exact_patch,
)
from verify_navigation_dirty_upstream_audit import (
    ALLOWED_PATHS,
    AuditError,
    SCHEMA,
    _git,
    verify,
)


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
    patch = _inspect_exact_patch(
        workspace,
        repo,
        repository_label="Navigation",
        allowed_paths=ALLOWED_PATHS,
        git=_git,
        error_type=AuditError,
    )
    issued = datetime.now(timezone.utc).replace(microsecond=0)
    expires = issued + timedelta(seconds=valid_for_seconds)
    data = {
        "schema": SCHEMA,
        "repository": "navigation",
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
