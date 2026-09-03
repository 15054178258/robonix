from __future__ import annotations

import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
PREPARER = ROOT / "scripts" / "prepare_navigation_dirty_audit_receipt.py"
VERIFIER = ROOT / "scripts" / "verify_navigation_dirty_upstream_audit.py"
sys.path.insert(0, str(ROOT / "scripts"))
import prepare_navigation_dirty_audit_receipt as preparer  # noqa: E402

ALLOWED_PATHS = [
    "config.spec",
    "nav2_wrapper/atlas_bridge.py",
    "nav2_wrapper/configuration.py",
    "nav2_wrapper/guarded_launch.py",
    "test_configuration.py",
    "test_guarded_launch.py",
    "test_runtime_integration.py",
]


class PrepareNavigationDirtyAuditReceiptTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.workspace = Path(self.temporary.name) / "workspace"
        self.repo = self.workspace / "third_party" / "service-navigation-rbnx"
        self.private = self.workspace / "rbnx-build" / "run"
        self.repo.mkdir(parents=True)
        self.private.mkdir(parents=True)
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
        self.receipt = self.private / "navigation-audit.json"

    def git(self, *arguments: str) -> None:
        subprocess.run(
            ["git", "-C", str(self.repo), *arguments],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def prepare(
        self,
        *,
        output: Path | None = None,
        lifetime: int = 3600,
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(PREPARER),
                "--workspace",
                str(self.workspace),
                "--repo",
                str(self.repo),
                "--output",
                str(output or self.receipt),
                "--valid-for-seconds",
                str(lifetime),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def verify(self) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(VERIFIER),
                "--workspace",
                str(self.workspace),
                "--repo",
                str(self.repo),
                "--receipt",
                str(self.receipt),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def test_prepares_private_exact_receipt_accepted_by_verifier(self) -> None:
        result = self.prepare()
        self.assertEqual(result.returncode, 0, result.stderr)
        metadata = os.lstat(self.receipt)
        self.assertTrue(stat.S_ISREG(metadata.st_mode))
        self.assertFalse(stat.S_ISLNK(metadata.st_mode))
        self.assertEqual(stat.S_IMODE(metadata.st_mode), 0o600)
        self.assertEqual(metadata.st_uid, os.geteuid())
        self.assertEqual(metadata.st_nlink, 1)
        data = json.loads(self.receipt.read_text(encoding="utf-8"))
        self.assertEqual(data["schema"], "robonix-navigation-dirty-audit-v1")
        self.assertEqual(data["allowed_paths"], ALLOWED_PATHS)
        verified = self.verify()
        self.assertEqual(verified.returncode, 0, verified.stderr)

    def test_verifier_accepts_schema_keys_in_another_order(self) -> None:
        result = self.prepare()
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(self.receipt.read_text(encoding="utf-8"))
        reordered = dict(reversed(list(data.items())))
        self.receipt.write_text(
            json.dumps(reordered, indent=2) + "\n",
            encoding="utf-8",
        )
        self.receipt.chmod(0o600)

        verified = self.verify()
        self.assertEqual(verified.returncode, 0, verified.stderr)

    def test_refuses_existing_or_outside_output(self) -> None:
        self.receipt.write_text("occupied\n", encoding="utf-8")
        result = self.prepare()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already exists", result.stderr)
        self.assertNotIn("Traceback", result.stderr)
        self.assertEqual(self.receipt.read_text(encoding="utf-8"), "occupied\n")
        self.assertEqual(list(self.private.glob(".robonix-audit-tmp-*")), [])

        outside = Path(self.temporary.name) / "outside.json"
        result = self.prepare(output=outside)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("inside the deployment workspace", result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_shared_helper_raises_navigation_error_and_cleans_verify_failure(self) -> None:
        with patch.object(
            preparer,
            "verify",
            side_effect=preparer.AuditError("injected navigation verification failure"),
        ):
            with self.assertRaises(preparer.AuditError) as raised:
                preparer.prepare(self.workspace, self.repo, self.receipt, 3600)

        self.assertIn("navigation verification failure", str(raised.exception))
        self.assertFalse(self.receipt.exists())
        self.assertEqual(list(self.private.glob(".robonix-audit-tmp-*")), [])

    def test_refuses_untracked_or_unexpected_tracked_changes(self) -> None:
        (self.repo / "untracked.txt").write_text("no\n", encoding="utf-8")
        result = self.prepare()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("contains untracked files", result.stderr)
        (self.repo / "untracked.txt").unlink()

        with (self.repo / "tracked-extra.txt").open("a", encoding="utf-8") as stream:
            stream.write("not audited\n")
        result = self.prepare()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not exactly the audited Navigation files", result.stderr)

    def test_enforces_bounded_lifetime(self) -> None:
        for lifetime in (59, 14401):
            with self.subTest(lifetime=lifetime):
                result = self.prepare(lifetime=lifetime)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("between 60 seconds and four hours", result.stderr)


if __name__ == "__main__":
    unittest.main()
