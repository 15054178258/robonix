from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_staged_nav2_live_goal_evidence.py"
SPEC = importlib.util.spec_from_file_location(
    "validate_staged_nav2_live_goal_evidence_test", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class StagedNav2LiveGoalValidatorTest(unittest.TestCase):
    def _run(self, *, start_x: float, start_y: float) -> tuple[int, str, str]:
        argv = [
            str(SCRIPT),
            "--evidence",
            str(ROOT / "rbnx-build" / "run" / "fresh.json"),
            "--map-id",
            "hall_map",
            "--map-generation",
            "2",
            "--target-id",
            "short_goal",
            "--goal-x",
            "1.0",
            "--goal-y",
            "2.0",
            "--goal-yaw",
            "0.0",
            "--permit-start-x",
            "0.0",
            "--permit-start-y",
            "0.0",
        ]
        validated = SimpleNamespace(
            start_x=start_x,
            start_y=start_y,
            start_yaw=2.8,
            sha256="a" * 64,
        )
        stdout = StringIO()
        stderr = StringIO()
        with (
            patch.object(module, "sha256_file", return_value="a" * 64),
            patch.object(
                module,
                "validate_short_goal_evidence",
                return_value=validated,
            ),
            patch("sys.argv", argv),
            redirect_stdout(stdout),
            redirect_stderr(stderr),
        ):
            status = module.main()
        return status, stdout.getvalue(), stderr.getvalue()

    def test_fresh_start_within_permit_radius_prints_exact_four_claims(self) -> None:
        status, stdout, stderr = self._run(start_x=0.03, start_y=0.04)
        self.assertEqual(status, 0)
        self.assertEqual(stderr, "")
        self.assertEqual(
            stdout.splitlines(),
            ["0.029999999999999999", "0.040000000000000001", "2.7999999999999998", "a" * 64],
        )

    def test_fresh_start_beyond_permit_radius_is_rejected(self) -> None:
        status, stdout, stderr = self._run(start_x=0.051, start_y=0.0)
        self.assertEqual(status, 1)
        self.assertEqual(stdout, "")
        self.assertIn("more than 0.05 m", stderr)


if __name__ == "__main__":
    unittest.main()
