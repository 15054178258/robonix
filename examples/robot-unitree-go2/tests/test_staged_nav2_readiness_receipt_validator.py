from __future__ import annotations

from contextlib import redirect_stderr
import importlib.util
from io import StringIO
from pathlib import Path
from unittest.mock import patch
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT / "scripts" / "validate_staged_nav2_readiness_receipt.py"
)
SPEC = importlib.util.spec_from_file_location(
    "validate_staged_nav2_readiness_receipt_test", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class StagedNav2ReadinessReceiptValidatorTest(unittest.TestCase):
    def _run(
        self,
        *,
        localization_x: float,
        localization_y: float,
        include_permit: bool = True,
    ) -> tuple[int, str]:
        argv = [
            str(SCRIPT),
            "--receipt",
            str(ROOT / "rbnx-build" / "run" / "post.json"),
            "--phase",
            "post_guard",
            "--session-id",
            "session-01234567",
            "--network-interface",
            "wlx500ff54809b8",
            "--map-id",
            "hall-map-01",
            "--map-generation",
            "2",
            "--allowed-mode",
            "0",
            "--allowed-state-marker",
            "100",
            "--expected-start-x",
            "0.03",
            "--expected-start-y",
            "0.0",
        ]
        if include_permit:
            argv.extend(
                (
                    "--permit-start-x",
                    "0.0",
                    "--permit-start-y",
                    "0.0",
                )
            )
        validated = {
            "localization": {
                "x_m": localization_x,
                "y_m": localization_y,
            }
        }
        stderr = StringIO()
        with (
            patch.object(module, "_load_private_receipt", return_value={}),
            patch.object(
                module,
                "validate_readiness_receipt",
                return_value=validated,
            ),
            patch("sys.argv", argv),
            redirect_stderr(stderr),
        ):
            status = module.main()
        return status, stderr.getvalue()

    def test_post_guard_localization_must_also_match_permit_start(self) -> None:
        status, stderr = self._run(
            localization_x=0.051,
            localization_y=0.0,
        )
        self.assertEqual(status, 1)
        self.assertIn("permit-bound path start", stderr)

    def test_post_guard_accepts_one_localization_matching_both_starts(self) -> None:
        status, stderr = self._run(
            localization_x=0.04,
            localization_y=0.0,
        )
        self.assertEqual(status, 0)
        self.assertEqual(stderr, "")

    def test_post_guard_requires_explicit_permit_start(self) -> None:
        status, stderr = self._run(
            localization_x=0.0,
            localization_y=0.0,
            include_permit=False,
        )
        self.assertEqual(status, 2)
        self.assertIn("live and permit-bound starts", stderr)


if __name__ == "__main__":
    unittest.main()
