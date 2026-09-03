from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from go2_chassis.staged_nav2_result import (
    ACTION_RESULT_NAME,
    ACTION_RESULT_SCHEMA,
    MEASURED_RESULT_NAME,
    MEASURED_RESULT_SCHEMA,
    ResultError,
    read_private_json,
    result_paths,
    sha256_file,
    validate_measured_result,
    write_private_json,
)


VALIDATOR = REPO / "scripts" / "validate_staged_nav2_result.py"


class StagedNav2ResultTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.run_dir = Path(self.temporary.name).resolve()
        self.run_dir.chmod(0o700)
        self.action_path = self.run_dir / ACTION_RESULT_NAME
        self.measured_path = self.run_dir / MEASURED_RESULT_NAME
        self.environment = {
            "GO2_STAGED_NAV2_RUN_DIR": str(self.run_dir),
            "GO2_STAGED_NAV2_ACTION_RESULT_FILE": str(self.action_path),
            "GO2_STAGED_NAV2_RESULT_FILE": str(self.measured_path),
            "GO2_STAGED_NAV2_SESSION_ID": "session-0123456789",
            "GO2_STAGED_NAV2_PAIR_ID": "pair-0123456789",
            "GO2_STAGED_NAV2_MAP_ID": "corridor-map",
            "GO2_STAGED_NAV2_MAP_GENERATION": "7",
            "GO2_STAGED_NAV2_TARGET_ID": "short-goal-01",
            "GO2_STAGED_NAV2_EXPECTED_GOAL_X": "1.2",
            "GO2_STAGED_NAV2_EXPECTED_GOAL_Y": "2.01",
            "GO2_STAGED_NAV2_EXPECTED_GOAL_YAW": "0.02",
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _action(self) -> dict[str, object]:
        now = time.time_ns()
        return {
            "schema": ACTION_RESULT_SCHEMA,
            "status": "PASS",
            "session_id": self.environment[
                "GO2_STAGED_NAV2_SESSION_ID"
            ],
            "pair_id": self.environment["GO2_STAGED_NAV2_PAIR_ID"],
            "map_id": self.environment["GO2_STAGED_NAV2_MAP_ID"],
            "map_generation": 7,
            "target_id": self.environment["GO2_STAGED_NAV2_TARGET_ID"],
            "goal_uuid": "01" * 16,
            "action": "/navigate_to_pose",
            "goal_accepted": True,
            "action_status_code": 4,
            "action_status_name": "succeeded",
            "cancel_requested": False,
            "cancel_confirmed": False,
            "started_unix_ns": now - 2_000_000,
            "finished_unix_ns": now - 1_000_000,
            "error": "",
        }

    def _measured(self) -> dict[str, object]:
        now = time.time_ns()
        return {
            "schema": MEASURED_RESULT_SCHEMA,
            "status": "PASS",
            "session_id": self.environment[
                "GO2_STAGED_NAV2_SESSION_ID"
            ],
            "pair_id": self.environment["GO2_STAGED_NAV2_PAIR_ID"],
            "map": {"id": "corridor-map", "generation": 7},
            "target": {
                "id": "short-goal-01",
                "pose": {"x": 1.2, "y": 2.01, "yaw": 0.02},
            },
            "goal_uuid": "01" * 16,
            "started_unix_ns": now - 2_000_000,
            "finished_unix_ns": now - 1_000_000,
            "action": {
                "result_file": str(self.action_path),
                "result_sha256": sha256_file(self.action_path),
                "goal_accepted": True,
                "status_code": 4,
                "status_name": "succeeded",
            },
            "measurement": {
                "frame_id": "odom",
                "start_pose": {"x": 1.0, "y": 2.0, "yaw": 0.0},
                "end_pose": {"x": 1.2, "y": 2.01, "yaw": 0.02},
                "forward_m": 0.2,
                "total_m": 0.21,
                "lateral_m": 0.01,
                "yaw_change_rad": 0.02,
            },
            "stop_sequence": {
                "cancel": {
                    "required": False,
                    "requested": False,
                    "confirmed": False,
                },
                "zero": {
                    "published_count": 62,
                    "confirmed_zero": True,
                },
                "disarm": {
                    "requested": True,
                    "service_success": True,
                    "measured_disarmed": True,
                },
                "post_stop": {
                    "observation_s": 1.01,
                    "odom_samples": 20,
                    "max_drift_m": 0.001,
                    "max_linear_speed_mps": 0.002,
                    "max_yaw_rate_rps": 0.003,
                    "limits": {
                        "max_drift_m": 0.02,
                        "max_linear_speed_mps": 0.03,
                        "max_yaw_rate_rps": 0.03,
                        "min_odom_samples": 10,
                    },
                    "passed": True,
                },
            },
            "checks": {
                "action_succeeded": "pass",
                "measurement_complete": "pass",
                "cancel_complete": "pass",
                "zero_complete": "pass",
                "disarm_complete": "pass",
                "post_stop_stationary": "pass",
            },
            "failure_reason": "",
        }

    def test_exact_private_paths_and_exclusive_files(self) -> None:
        self.assertEqual(
            result_paths(self.environment),
            (self.action_path, self.measured_path),
        )
        write_private_json(self.action_path, self._action())
        self.assertEqual(
            stat.S_IMODE(os.lstat(self.action_path).st_mode), 0o600
        )
        self.assertEqual(
            read_private_json(self.action_path)["status"], "PASS"
        )
        with self.assertRaises(ResultError):
            write_private_json(self.action_path, self._action())
        self.run_dir.chmod(0o755)
        with self.assertRaises(ResultError):
            result_paths(self.environment)

    def test_complete_measured_result_validates(self) -> None:
        write_private_json(self.action_path, self._action())
        metrics = validate_measured_result(
            self._measured(), self.environment, self.action_path
        )
        self.assertAlmostEqual(metrics["forward_m"], 0.2)
        self.assertAlmostEqual(metrics["total_m"], 0.21)
        self.assertAlmostEqual(metrics["lateral_m"], 0.01)
        self.assertAlmostEqual(metrics["yaw_change_rad"], 0.02)

    def test_missing_or_mismatched_evidence_is_rejected(self) -> None:
        write_private_json(self.action_path, self._action())
        baseline = self._measured()
        mutations: list[dict[str, object]] = []

        missing = deepcopy(baseline)
        del missing["measurement"]
        mutations.append(missing)

        endpoint = deepcopy(baseline)
        endpoint["measurement"]["forward_m"] = 0.19  # type: ignore[index]
        mutations.append(endpoint)

        action_hash = deepcopy(baseline)
        action_hash["action"]["result_sha256"] = "0" * 64  # type: ignore[index]
        mutations.append(action_hash)

        post_stop = deepcopy(baseline)
        post_stop["stop_sequence"]["post_stop"]["odom_samples"] = 9  # type: ignore[index]
        mutations.append(post_stop)

        disarm = deepcopy(baseline)
        disarm["stop_sequence"]["disarm"]["service_success"] = False  # type: ignore[index]
        mutations.append(disarm)

        check = deepcopy(baseline)
        check["checks"]["zero_complete"] = "fail"  # type: ignore[index]
        mutations.append(check)

        for payload in mutations:
            with self.subTest(payload=payload), self.assertRaises(ResultError):
                validate_measured_result(
                    payload, self.environment, self.action_path
                )

    def test_missing_target_claim_is_a_contract_error(self) -> None:
        write_private_json(self.action_path, self._action())
        environment = dict(self.environment)
        del environment["GO2_STAGED_NAV2_EXPECTED_GOAL_X"]
        with self.assertRaises(ResultError):
            validate_measured_result(
                self._measured(), environment, self.action_path
            )

    def test_offline_validator_cli_accepts_only_complete_result(self) -> None:
        write_private_json(self.action_path, self._action())
        write_private_json(self.measured_path, self._measured())
        completed = subprocess.run(
            [sys.executable, str(VALIDATOR)],
            cwd=REPO,
            env={**os.environ, **self.environment},
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("measured result PASS", completed.stdout)


if __name__ == "__main__":
    unittest.main()
