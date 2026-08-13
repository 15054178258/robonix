#!/usr/bin/env python3
"""Repository hygiene contracts for generated local artifacts."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent


class RepositoryHygieneTests(unittest.TestCase):
    def test_codegen_log_directory_is_ignored(self) -> None:
        rules = (ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()

        self.assertIn("logs/", rules)


if __name__ == "__main__":
    unittest.main()
