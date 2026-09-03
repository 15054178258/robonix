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
PREPARER = ROOT / "scripts" / "prepare_mapping_dirty_audit_receipt.py"
VERIFIER = ROOT / "scripts" / "verify_dirty_upstream_audit.py"
sys.path.insert(0, str(ROOT / "scripts"))
import prepare_mapping_dirty_audit_receipt as preparer  # noqa: E402

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


class PrepareMappingDirtyAuditReceiptTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.workspace = Path(self.temporary.name) / "workspace"
        self.repo = self.workspace / "third_party" / "service-map-rbnx"
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
        self.receipt = self.private / "mapping-audit.json"

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
        self.assertEqual(data["schema"], "robonix-mapping-dirty-audit-v1")
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

        outside_directory = Path(self.temporary.name) / "outside-directory"
        outside_directory.mkdir()
        escape = self.workspace / "escape"
        escape.symlink_to(outside_directory, target_is_directory=True)
        result = self.prepare(output=escape / "escaped.json")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("inside the deployment workspace", result.stderr)
        self.assertFalse((outside_directory / "escaped.json").exists())

    def test_symlink_and_hardlink_outputs_are_never_replaced(self) -> None:
        victim = self.private / "victim"
        victim.write_text("keep\n", encoding="utf-8")

        symlink = self.private / "symlink.json"
        symlink.symlink_to(victim)
        result = self.prepare(output=symlink)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already exists", result.stderr)
        self.assertTrue(symlink.is_symlink())
        self.assertEqual(victim.read_text(encoding="utf-8"), "keep\n")

        hardlink = self.private / "hardlink.json"
        os.link(victim, hardlink)
        before_links = os.lstat(victim).st_nlink
        result = self.prepare(output=hardlink)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already exists", result.stderr)
        self.assertEqual(victim.read_text(encoding="utf-8"), "keep\n")
        self.assertEqual(os.lstat(victim).st_nlink, before_links)
        self.assertEqual(list(self.private.glob(".robonix-audit-tmp-*")), [])

    def test_racing_existing_target_is_not_overwritten(self) -> None:
        real_link = os.link

        def publish_after_racer(*args, **kwargs):
            self.receipt.write_text("racer\n", encoding="utf-8")
            self.receipt.chmod(0o600)
            return real_link(*args, **kwargs)

        with patch.object(preparer.os, "link", side_effect=publish_after_racer):
            with self.assertRaises(preparer.AuditError) as raised:
                preparer.prepare(self.workspace, self.repo, self.receipt, 3600)

        self.assertIn("already exists", str(raised.exception))
        self.assertEqual(self.receipt.read_text(encoding="utf-8"), "racer\n")
        self.assertEqual(list(self.private.glob(".robonix-audit-tmp-*")), [])

    def test_write_sync_chmod_and_verify_failures_leave_no_fragments(self) -> None:
        failures = (
            ("write", OSError("injected write failure")),
            ("fsync", OSError("injected fsync failure")),
            ("fchmod", OSError("injected chmod failure")),
        )
        for function, error in failures:
            with self.subTest(function=function):
                with patch.object(preparer.os, function, side_effect=error):
                    with self.assertRaises(preparer.AuditError):
                        preparer.prepare(
                            self.workspace, self.repo, self.receipt, 3600
                        )
                self.assertFalse(self.receipt.exists())
                self.assertEqual(
                    list(self.private.glob(".robonix-audit-tmp-*")), []
                )

        with patch.object(
            preparer,
            "verify",
            side_effect=preparer.AuditError("injected verification failure"),
        ):
            with self.assertRaises(preparer.AuditError):
                preparer.prepare(self.workspace, self.repo, self.receipt, 3600)
        self.assertFalse(self.receipt.exists())
        self.assertEqual(list(self.private.glob(".robonix-audit-tmp-*")), [])

    def test_patch_change_between_temp_and_final_verification_fails_closed(self) -> None:
        real_verify = preparer.verify
        calls = 0

        def verify_then_mutate(*args, **kwargs):
            nonlocal calls
            real_verify(*args, **kwargs)
            calls += 1
            if calls == 1:
                with (self.repo / ALLOWED_PATHS[0]).open(
                    "a", encoding="utf-8"
                ) as stream:
                    stream.write("changed after temp verification\n")

        with patch.object(preparer, "verify", side_effect=verify_then_mutate):
            with self.assertRaises(preparer.AuditError):
                preparer.prepare(self.workspace, self.repo, self.receipt, 3600)

        self.assertEqual(calls, 1)
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
        self.assertIn("not exactly the audited Mapping files", result.stderr)

    def test_enforces_bounded_lifetime(self) -> None:
        for lifetime in (59, 14401):
            with self.subTest(lifetime=lifetime):
                result = self.prepare(lifetime=lifetime)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("between 60 seconds and four hours", result.stderr)


if __name__ == "__main__":
    unittest.main()
