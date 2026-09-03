from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/validate_first_motion_state_evidence.py"
SPEC = importlib.util.spec_from_file_location("first_motion_state", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
state = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = state
SPEC.loader.exec_module(state)


class FirstMotionStateEvidenceTest(unittest.TestCase):
    def payload(self, now_ns: int) -> dict[str, object]:
        streams = []
        for topic in ("/sportmodestate", "/lf/sportmodestate"):
            streams.append(
                {
                    "topic": topic,
                    "received": 100,
                    "first_source_stamp_ns": 1_000_000_000,
                    "last_source_stamp_ns": 31_000_000_000,
                    "source_regressions": 0,
                    "max_abs_linear_velocity": 0.01,
                    "max_abs_yaw_speed": 0.01,
                    "states": [
                        {
                            "error_code": 2010,
                            "mode": 0,
                            "gait_type": 0,
                            "samples": 100,
                        }
                    ],
                }
            )
        return {
            "schema_version": 1,
            "mode": "read-only-subscriber-only",
            "duration_limit_s": 30,
            "started_realtime_ns": now_ns - 31_000_000_000,
            "elapsed_monotonic_ns": 30_000_000_000,
            "publishers_created": False,
            "unitree_clients_created": False,
            "streams": streams,
        }

    def write(self, directory: Path, payload: dict[str, object]) -> Path:
        path = directory / "state.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        path.chmod(0o600)
        return path

    def test_returns_exact_mode_marker_and_gait(self) -> None:
        now_ns = time.time_ns()
        with tempfile.TemporaryDirectory() as temporary:
            path = self.write(Path(temporary), self.payload(now_ns))
            self.assertEqual(
                state.validate_first_motion_evidence(
                    path, now_realtime_ns=now_ns
                ),
                (0, 2010, 0),
            )

    def test_artifact_age_is_not_a_duplicate_runtime_gate_but_sentinel_fails(self) -> None:
        now_ns = time.time_ns()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            old = self.payload(now_ns - 121_000_000_000)
            path = self.write(directory, old)
            self.assertEqual(
                state.validate_first_motion_evidence(
                    path, now_realtime_ns=now_ns
                ),
                (0, 2010, 0),
            )
            path.unlink()
            sentinel = self.payload(now_ns)
            for stream in sentinel["streams"]:
                stream["states"][0]["mode"] = 255
            path = self.write(directory, sentinel)
            with self.assertRaisesRegex(state.EvidenceError, "sentinel"):
                state.validate_first_motion_evidence(
                    path, now_realtime_ns=now_ns
                )


if __name__ == "__main__":
    unittest.main()
