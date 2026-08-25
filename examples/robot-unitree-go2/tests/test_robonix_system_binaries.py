from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "scripts" / "robonix_system_artifacts.py"


class RobonixSystemBinaryTest(unittest.TestCase):
    def _write_manifest(self, root: Path, systems: dict[str, object]) -> Path:
        manifest = root / "manifest.yaml"
        manifest.write_text(
            yaml.safe_dump({"manifestVersion": 1, "system": systems}),
            encoding="utf-8",
        )
        return manifest

    def _run(
        self,
        *arguments: str,
        path: str | None = None,
    ) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        if path is not None:
            env["PATH"] = path
        return subprocess.run(
            [sys.executable, str(HELPER), *arguments],
            capture_output=True,
            check=False,
            env=env,
            text=True,
            timeout=3,
        )

    @staticmethod
    def _make_executable(directory: Path, name: str) -> None:
        executable = directory / name
        executable.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        executable.chmod(0o755)

    def test_manifest_drives_core_and_optional_vitals_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = self._write_manifest(
                root,
                {
                    "soma": {},
                    "scene": {},
                    "atlas": {},
                    "vitals": {},
                    "pilot": {},
                },
            )
            result = self._run("list", "--manifest", str(manifest))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(
                result.stdout.splitlines(),
                [
                    "robonix-atlas",
                    "robonix-pilot",
                    "robonix-soma",
                    "robonix-vitals",
                ],
            )
            self.assertNotIn("scene", result.stdout)

    def test_exact_executable_directory_and_path_pass(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = self._write_manifest(
                root,
                {"atlas": {}, "executor": {}, "liaison": {}},
            )
            bin_dir = root / "cargo-target" / "debug"
            bin_dir.mkdir(parents=True)
            for binary in (
                "robonix-atlas",
                "robonix-executor",
                "robonix-liaison",
            ):
                self._make_executable(bin_dir, binary)
            result = self._run(
                "verify",
                "--manifest",
                str(manifest),
                "--bin-dir",
                str(bin_dir),
                "--require-path",
                path=str(bin_dir),
            )
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_missing_redirected_or_wrong_path_artifact_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = self._write_manifest(root, {"atlas": {}})
            expected = root / "target" / "release"
            foreign = root / "foreign"
            expected.mkdir(parents=True)
            foreign.mkdir()
            self._make_executable(foreign, "robonix-atlas")

            missing = self._run(
                "verify",
                "--manifest",
                str(manifest),
                "--bin-dir",
                str(expected),
            )
            self.assertEqual(missing.returncode, 78)
            self.assertIn("missing Robonix system binary", missing.stderr)

            (expected / "robonix-atlas").symlink_to(foreign / "robonix-atlas")
            redirected = self._run(
                "verify",
                "--manifest",
                str(manifest),
                "--bin-dir",
                str(expected),
            )
            self.assertEqual(redirected.returncode, 78)
            self.assertIn("escapes the workspace artifact directory", redirected.stderr)

            (expected / "robonix-atlas").unlink()
            self._make_executable(expected, "robonix-atlas")
            wrong_path = self._run(
                "verify",
                "--manifest",
                str(manifest),
                "--bin-dir",
                str(expected),
                "--require-path",
                path=str(foreign),
            )
            self.assertEqual(wrong_path.returncode, 78)
            self.assertIn("outside the selected artifact directory", wrong_path.stderr)

    def test_build_and_start_share_exact_workspace_profile_contract(self) -> None:
        build = (ROOT / "build.sh").read_text(encoding="utf-8")
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        env_example = (ROOT / ".env.example").read_text(encoding="utf-8")

        for source in (build, start):
            self.assertIn(
                'WORKSPACE_ROBONIX_SOURCE_ROOT="$WORKSPACE_ROOT/upstream/robonix-go2-build"',
                source,
            )
            self.assertIn(
                'WORKSPACE_CARGO_TARGET_DIR="$WORKSPACE_ROOT/.tools/cargo-target/robonix"',
                source,
            )
            self.assertIn('export CARGO_HOME="$WORKSPACE_CARGO_HOME"', source)
            self.assertIn('export CARGO_TARGET_DIR="$WORKSPACE_CARGO_TARGET_DIR"', source)
            self.assertIn('export RUSTUP_HOME="$WORKSPACE_RUSTUP_HOME"', source)
            self.assertIn(
                'export ROBONIX_SYSTEM_BIN_DIR="$CARGO_TARGET_DIR/$ROBONIX_BUILD_PROFILE"',
                source,
            )
            self.assertIn('export PATH="$ROBONIX_SYSTEM_BIN_DIR:$PATH"', source)

        self.assertNotIn("rbnx path root", build)
        self.assertIn("robonix_system_artifacts.py", build)
        self.assertIn("  debug)", build)
        self.assertIn("  release)", build)
        self.assertIn("  debug|release)", start)
        selected = self._run(
            "list", "--manifest", str((ROOT / "robonix_manifest.yaml").resolve())
        )
        self.assertEqual(selected.returncode, 0, selected.stderr)
        for package in (
            "robonix-atlas",
            "robonix-executor",
            "robonix-pilot",
            "robonix-liaison",
            "robonix-soma",
        ):
            self.assertIn(package, selected.stdout.splitlines())
        self.assertLess(
            start.index("robonix_system_artifacts.py"),
            start.index('"$RBNX_CLI" boot --no-update-check -f "$MANIFEST"'),
        )
        self.assertRegex(env_example, r"(?m)^ROBONIX_BUILD_PROFILE=debug$")


if __name__ == "__main__":
    unittest.main()
