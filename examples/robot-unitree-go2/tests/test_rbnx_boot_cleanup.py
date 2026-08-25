from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "scripts" / "shutdown_rbnx_boot_state.py"


class RbnxBootCleanupTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.fake_rbnx = self.root / "bin" / "rbnx"
        self.fake_rbnx.parent.mkdir()
        self.call_log = self.root / "rbnx-call.json"
        self.fake_rbnx.write_text(
            """#!/usr/bin/env python3
import json
import os
import sys
from pathlib import Path
Path(os.environ["RBNX_CALL_LOG"]).write_text(json.dumps(sys.argv[1:]))
raise SystemExit(int(os.environ.get("RBNX_FAKE_STATUS", "0")))
""",
            encoding="utf-8",
        )
        self.fake_rbnx.chmod(0o700)
        self.env = os.environ.copy()
        self.env["RBNX_CALL_LOG"] = str(self.call_log)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _manifest(self, name: str) -> Path:
        manifest = self.root / name / "robonix_manifest.yaml"
        manifest.parent.mkdir()
        manifest.write_text("manifestVersion: 1\n", encoding="utf-8")
        return manifest.resolve()

    @staticmethod
    def _state(manifest: Path, boot_pid: int) -> Path:
        state = manifest.parent / "rbnx-boot" / "state.json"
        state.parent.mkdir()
        state.write_text(
            json.dumps(
                {
                    "manifest_path": str(manifest),
                    "boot_pid": boot_pid,
                    "started_at_ms": 1,
                    "atlas_endpoint": "127.0.0.1:50051",
                    "components": [],
                }
            ),
            encoding="utf-8",
        )
        return state

    def _run(self, manifest: Path, boot_pid: int) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(HELPER),
                "--manifest",
                str(manifest),
                "--rbnx",
                str(self.fake_rbnx),
                "--expected-boot-pid",
                str(boot_pid),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=self.env,
            timeout=5,
        )

    def test_exact_manifest_and_boot_pid_use_documented_shutdown_syntax(self) -> None:
        manifest = self._manifest("deploy-a")
        other_manifest = self._manifest("deploy-b")
        self._state(manifest, 4123)
        other_state = self._state(other_manifest, 9988)

        result = self._run(manifest, 4123)

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            json.loads(self.call_log.read_text(encoding="utf-8")),
            ["shutdown", "-f", str(manifest)],
        )
        self.assertNotIn("--no-update-check", self.call_log.read_text())
        self.assertTrue(other_state.exists(), "another manifest state was modified")

    def test_absent_state_does_not_invoke_rbnx_or_touch_another_manifest(self) -> None:
        manifest = self._manifest("deploy-a")
        other_manifest = self._manifest("deploy-b")
        other_state = self._state(other_manifest, 9988)

        result = self._run(manifest, 4123)

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.call_log.exists())
        self.assertTrue(other_state.exists())

    def test_state_from_another_manifest_fails_closed(self) -> None:
        manifest = self._manifest("deploy-a")
        other_manifest = self._manifest("deploy-b")
        state = self._state(manifest, 4123)
        value = json.loads(state.read_text())
        value["manifest_path"] = str(other_manifest)
        state.write_text(json.dumps(value), encoding="utf-8")

        result = self._run(manifest, 4123)

        self.assertEqual(result.returncode, 78)
        self.assertIn("belongs to another manifest", result.stderr)
        self.assertFalse(self.call_log.exists())

    def test_state_from_another_boot_or_placement_fails_closed(self) -> None:
        manifest = self._manifest("deploy-a")
        self._state(manifest, 4123)

        result = self._run(manifest, 5000)

        self.assertEqual(result.returncode, 78)
        self.assertIn("belongs to another boot process", result.stderr)
        self.assertFalse(self.call_log.exists())

    def test_malformed_or_symlink_state_never_invokes_rbnx(self) -> None:
        manifest = self._manifest("deploy-a")
        state = manifest.parent / "rbnx-boot" / "state.json"
        state.parent.mkdir()
        state.write_text("not json\n", encoding="utf-8")
        malformed = self._run(manifest, 4123)
        self.assertEqual(malformed.returncode, 78)
        self.assertFalse(self.call_log.exists())

        state.unlink()
        foreign = self.root / "foreign-state.json"
        foreign.write_text("{}\n", encoding="utf-8")
        state.symlink_to(foreign)
        symlinked = self._run(manifest, 4123)
        self.assertEqual(symlinked.returncode, 78)
        self.assertIn("regular non-symlink", symlinked.stderr)
        self.assertFalse(self.call_log.exists())

    def test_launcher_waits_for_boot_then_uses_exact_cleanup_helper(self) -> None:
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        wait_call = 'wait "$RBNX_BOOT_PID" 2>/dev/null || true'
        helper_call = 'python3 "$DEPLOY_DIR/scripts/shutdown_rbnx_boot_state.py"'
        self.assertLess(start.index(wait_call), start.index(helper_call))
        self.assertIn('--expected-boot-pid "$RBNX_BOOT_PID"', start)
        self.assertIn('--rbnx "$RBNX_CLI"', start)
        self.assertIn('"$RBNX_CLI" boot --no-update-check -f "$MANIFEST"', start)
        self.assertIn('-e "$RBNX_STATE_PATH" || -L "$RBNX_STATE_PATH"', start)
        self.assertNotIn("pkill", start)


if __name__ == "__main__":
    unittest.main()
