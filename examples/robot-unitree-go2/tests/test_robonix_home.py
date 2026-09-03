from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR = ROOT / "scripts" / "validate_robonix_home.py"


class RobonixHomeValidationTest(unittest.TestCase):
    def run_validator(
        self, config: Path, expected_source: Path
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["python3", str(VALIDATOR), str(config), str(expected_source)],
            capture_output=True,
            text=True,
            check=False,
            timeout=3,
        )

    def test_exact_existing_workspace_source_passes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "upstream" / "robonix-go2-build"
            source.mkdir(parents=True)
            config = root / ".tools" / "robonix-home" / "config.yaml"
            config.parent.mkdir(parents=True)
            config.write_text(f"robonix_source_path: {source}\n", encoding="utf-8")
            self.assertEqual(self.run_validator(config, source).returncode, 0)

    def test_home_or_other_source_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            expected = root / "upstream" / "robonix-go2-build"
            other = root / "other-robonix"
            expected.mkdir(parents=True)
            other.mkdir()
            config = root / ".tools" / "robonix-home" / "config.yaml"
            config.parent.mkdir(parents=True)
            config.write_text(f"robonix_source_path: {other}\n", encoding="utf-8")
            result = self.run_validator(config, expected)
            self.assertEqual(result.returncode, 78)
            self.assertIn("audited workspace source", result.stderr)

    def test_missing_config_or_source_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            result = self.run_validator(
                root / "missing-config.yaml", root / "missing-source"
            )
            self.assertEqual(result.returncode, 78)
            self.assertIn("invalid workspace-local", result.stderr)


if __name__ == "__main__":
    unittest.main()
