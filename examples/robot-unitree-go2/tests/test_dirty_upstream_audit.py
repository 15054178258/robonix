from __future__ import annotations

import base64
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
VERIFIER = ROOT / "scripts" / "verify_dirty_upstream_audit.py"
ALLOWED_PATHS = [
    "CAPABILITY.md",
    "config.spec",
    "launch/rtabmap_2d.launch.py",
    "scripts/start_engine.sh",
    "src/mapping_rbnx/atlas_bridge.py",
    "src/mapping_rbnx/lifecycle.py",
    "src/mapping_rbnx/map_ops.py",
    "test_map_load.py",
    "test_rtabmap_profiles.py",
]


class DirtyUpstreamAuditTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.workspace = Path(self.temporary.name) / "workspace"
        self.repo = self.workspace / "third_party" / "service-map-rbnx"
        self.repo.mkdir(parents=True)
        self.git("init", "-q")
        self.git("config", "user.name", "Receipt Test")
        self.git("config", "user.email", "receipt-test@example.invalid")
        for relative in [*ALLOWED_PATHS, "tracked-extra.txt"]:
            target = self.repo / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(f"base {relative}\n", encoding="utf-8")
        self.git("add", ".")
        self.git("commit", "-qm", "base")
        for relative in ALLOWED_PATHS:
            with (self.repo / relative).open("a", encoding="utf-8") as stream:
                stream.write("audited change\n")
        self.receipt = self.workspace / "private" / "mapping.json"
        self.write_receipt()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def git_bytes(self, *arguments: str) -> bytes:
        return subprocess.run(
            ["git", "-C", str(self.repo), *arguments],
            check=True,
            stdout=subprocess.PIPE,
        ).stdout

    def git(self, *arguments: str) -> str:
        return self.git_bytes(*arguments).decode("utf-8").strip()

    def receipt_data(
        self,
        *,
        issued: datetime | None = None,
        expires: datetime | None = None,
    ) -> dict[str, object]:
        issued = issued or datetime.now(timezone.utc).replace(microsecond=0)
        expires = expires or issued + timedelta(hours=1)
        status = self.git_bytes(
            "status", "--porcelain=v1", "-z", "--untracked-files=normal"
        )
        tracked_diff = self.git_bytes(
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
            "schema": "robonix-mapping-dirty-audit-v1",
            "repository": "mapping",
            "repository_relpath": "third_party/service-map-rbnx",
            "head": self.git("rev-parse", "HEAD"),
            "status_porcelain_v1_z_base64": base64.b64encode(status).decode(),
            "tracked_diff_sha256": hashlib.sha256(tracked_diff).hexdigest(),
            "allowed_paths": ALLOWED_PATHS,
            "issued_at_utc": issued.strftime("%Y-%m-%dT%H:%M:%SZ"),
            "expires_at_utc": expires.strftime("%Y-%m-%dT%H:%M:%SZ"),
        }

    def write_receipt(self, data: dict[str, object] | None = None) -> None:
        self.receipt.parent.mkdir(parents=True, exist_ok=True)
        self.receipt.write_text(
            json.dumps(data or self.receipt_data(), indent=2) + "\n",
            encoding="utf-8",
        )
        self.receipt.chmod(0o600)

    def verify(self, receipt: Path | None = None) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(VERIFIER),
                "--workspace",
                str(self.workspace),
                "--repo",
                str(self.repo),
                "--receipt",
                str(receipt or self.receipt),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def assert_rejected(self, expected: str, receipt: Path | None = None) -> None:
        result = self.verify(receipt)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn(expected, result.stderr)

    def test_matching_private_workspace_receipt_is_accepted(self) -> None:
        result = self.verify()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("separately audited Mapping dirty patch accepted", result.stdout)

    def test_untracked_or_unexpected_tracked_paths_are_rejected(self) -> None:
        (self.repo / "untracked.txt").write_text("no\n", encoding="utf-8")
        self.write_receipt()
        self.assert_rejected("contains untracked files")
        (self.repo / "untracked.txt").unlink()

        with (self.repo / "tracked-extra.txt").open("a", encoding="utf-8") as stream:
            stream.write("not audited\n")
        self.write_receipt()
        self.assert_rejected("not exactly the audited Mapping files")

    def test_head_status_and_full_diff_must_each_match(self) -> None:
        data = self.receipt_data()
        data["head"] = "0" * 40
        self.write_receipt(data)
        self.assert_rejected("HEAD does not match")

        self.write_receipt()
        self.git("add", ALLOWED_PATHS[0])
        self.assert_rejected("exact porcelain status does not match")
        self.git("reset", "-q", "HEAD", "--", ALLOWED_PATHS[0])

        self.write_receipt()
        with (self.repo / ALLOWED_PATHS[0]).open("a", encoding="utf-8") as stream:
            stream.write("changed after audit\n")
        self.assert_rejected("full tracked diff hash does not match")

    def test_receipt_must_be_private_regular_owned_file_inside_workspace(self) -> None:
        self.receipt.chmod(0o640)
        self.assert_rejected("permissions must be exactly 0600")
        self.receipt.chmod(0o600)

        symlink = self.workspace / "private" / "mapping-link.json"
        symlink.symlink_to(self.receipt)
        self.assert_rejected("non-symlink regular file", symlink)

        outside = Path(self.temporary.name) / "outside.json"
        outside.write_bytes(self.receipt.read_bytes())
        outside.chmod(0o600)
        self.assert_rejected("inside the deployment workspace", outside)

    def test_expiry_metadata_does_not_replace_exact_diff_validation(self) -> None:
        issued = datetime.now(timezone.utc).replace(microsecond=0) - timedelta(hours=2)
        self.write_receipt(self.receipt_data(issued=issued, expires=issued + timedelta(hours=1)))
        self.assertEqual(self.verify().returncode, 0)

        issued = datetime.now(timezone.utc).replace(microsecond=0)
        self.write_receipt(self.receipt_data(issued=issued, expires=issued + timedelta(hours=5)))
        self.assertEqual(self.verify().returncode, 0)

    def test_shell_gate_keeps_mapping_exception_label_specific(self) -> None:
        source = (ROOT / "scripts" / "verify_upstream_compatibility.sh").read_text(
            encoding="utf-8"
        )
        self.assertEqual(source.count("ROBONIX_MAPPING_DIRTY_AUDIT_RECEIPT"), 2)
        self.assertIn('[[ "$label" == "mapping"', source)
        self.assertIn('[[ "$label" == "navigation"', source)
        self.assertNotIn("ROBONIX_ROBONIX_DIRTY_AUDIT_RECEIPT", source)


if __name__ == "__main__":
    unittest.main()
