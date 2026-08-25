import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from go2_chassis.runtime_config import (  # noqa: E402
    STAGED_NAV2_PROFILE,
    normalize_config,
)
from go2_chassis.staged_nav2_permit import (  # noqa: E402
    CHASSIS_ROLE,
    GOAL_DISPATCH_ROLE,
    GOAL_EVIDENCE_SHA256_ENV,
    GOAL_PERMIT_ENV,
    GOAL_SOURCE_ENV,
    GOAL_X_ENV,
    GOAL_YAW_ENV,
    GOAL_Y_ENV,
    GUARD_ACK_ENV,
    MAP_GENERATION_ENV,
    MAP_ID_ENV,
    OPERATOR_GOAL_SOURCE,
    PAIR_ID_ENV,
    PERMIT_ENV,
    PERMIT_SCHEMA,
    SESSION_ENV,
    STAGED_NAV2_ACK,
    STAGE_ENV,
    TARGET_ID_ENV,
    PermitError,
    consume_staged_nav2_goal_permit,
    consume_staged_nav2_permit,
    validate_permit,
)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _runtime():
    return normalize_config(
        {
            "allow_motion": True,
            "motion_profile": STAGED_NAV2_PROFILE,
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": "wlan-stage",
            "ipc_socket": "/tmp/go2-staged-test.sock",
            "state_topic": "/robonix/time_corrected/motion/sportmodestate",
            "state_fallback_topic": "",
            "twist_in_topic": "/go2/staged_nav2/cmd_vel",
            "odom_source": "external_verified",
            "external_odom_topic": (
                "/robonix/time_corrected/raw/utlidar/robot_odom"
            ),
            "external_odom_timeout_s": 1.0,
            "odom_topic": "/odom",
            "publish_odom_tf": True,
            "max_linear_x_mps": 0.30,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.40,
            "max_linear_accel_mps2": 0.30,
            "max_angular_accel_rps2": 0.80,
            "command_timeout_s": 0.20,
            "state_timeout_s": 1.0,
            "max_source_stamp_age_s": 0.20,
            "max_source_stamp_future_skew_s": 0.05,
            "commissioning_max_duration_s": 0.0,
            "commissioning_max_distance_m": 0.0,
        },
        {
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "100",
        },
        ROOT,
    )


class StagedNav2PermitTest(unittest.TestCase):
    def setUp(self) -> None:
        build = ROOT / "rbnx-build"
        build.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=build)
        self.directory = Path(self.temporary.name)
        self.directory.chmod(0o700)
        self.evidence = {}
        for name in (
            "dds_identity",
            "state",
            "time",
            "goal",
        ):
            path = self.directory / f"{name}.evidence"
            path.write_text(f"immutable {name}\n", encoding="utf-8")
            path.chmod(0o600)
            self.evidence[name] = {"path": str(path), "sha256": _digest(path)}
        self.now_ns = time.time_ns()
        self.common = {
            "schema": PERMIT_SCHEMA,
            "pair_id": "pair-0123456789abcdef",
            "session_id": "session-0123456789abcdef",
            "one_time": True,
            "issued_unix_ns": self.now_ns - 1_000_000_000,
            "expires_unix_ns": self.now_ns + 60_000_000_000,
            "profile": STAGED_NAV2_PROFILE,
            "stage": "stage1",
            "operator_ack": STAGED_NAV2_ACK,
            "guard_ack": STAGED_NAV2_ACK,
            "network_interface": "wlan-stage",
            "state_topic": "/robonix/time_corrected/motion/sportmodestate",
            "nav_command_topic": "/cmd_vel_guard_input",
            "command_topic": "/go2/staged_nav2/cmd_vel",
            "odom_topic": "/odom",
            "external_odom_topic": (
                "/robonix/time_corrected/raw/utlidar/robot_odom"
            ),
            "arm_service": "/go2_chassis/arm",
            "allowed_modes": [0],
            "allowed_state_markers": [100],
            "max_linear_x_mps": 0.30,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.40,
            "max_linear_accel_mps2": 0.30,
            "max_angular_accel_rps2": 0.80,
            "max_duration_s": 0.0,
            "max_distance_m": 0.0,
            "command_timeout_s": 0.20,
            "state_timeout_s": 1.0,
            "external_odom_timeout_s": 1.0,
            "map_id": "go2_test_map_01",
            "map_generation": 1,
            "goal_source": OPERATOR_GOAL_SOURCE,
            "target_id": "short-goal-01",
            "goal_pose": {"x": 0.3, "y": -0.1, "yaw": 0.2},
            "evidence": self.evidence,
        }
        self.environ = {
            SESSION_ENV: self.common["session_id"],
            PAIR_ID_ENV: self.common["pair_id"],
            STAGE_ENV: "stage1",
            GUARD_ACK_ENV: STAGED_NAV2_ACK,
            MAP_ID_ENV: self.common["map_id"],
            MAP_GENERATION_ENV: "1",
            GOAL_SOURCE_ENV: OPERATOR_GOAL_SOURCE,
            TARGET_ID_ENV: self.common["target_id"],
            GOAL_X_ENV: "0.29999999999999999",
            GOAL_Y_ENV: "-0.10000000000000001",
            GOAL_YAW_ENV: "0.20000000000000001",
            GOAL_EVIDENCE_SHA256_ENV: self.evidence["goal"]["sha256"],
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _payload(self, role: str, suffix: str) -> dict[str, object]:
        return {
            **self.common,
            "permit_id": f"permit-0123456789{suffix}",
            "permit_role": role,
        }

    def _write(self, name: str, payload: dict[str, object]) -> Path:
        path = self.directory / name
        path.write_text(json.dumps(payload), encoding="utf-8")
        path.chmod(0o600)
        return path

    def test_role_bound_pair_is_consumed_once_by_each_consumer(self) -> None:
        runtime = _runtime()
        chassis = self._payload(CHASSIS_ROLE, "abcdef")
        goal = self._payload(GOAL_DISPATCH_ROLE, "abcdea")
        self.assertEqual(
            validate_permit(
                chassis,
                runtime,
                self.environ,
                ROOT,
                expected_role=CHASSIS_ROLE,
                now_unix_ns=self.now_ns,
            ),
            chassis["permit_id"],
        )
        chassis_path = self._write("chassis.json", chassis)
        goal_path = self._write("goal.json", goal)
        consumed_chassis = consume_staged_nav2_permit(
            runtime,
            {**self.environ, PERMIT_ENV: str(chassis_path)},
            ROOT,
        )
        consumed_goal = consume_staged_nav2_goal_permit(
            runtime,
            {**self.environ, GOAL_PERMIT_ENV: str(goal_path)},
            ROOT,
        )
        self.assertFalse(chassis_path.exists())
        self.assertFalse(goal_path.exists())
        self.assertTrue(consumed_chassis.is_file())
        self.assertTrue(consumed_goal.path.is_file())
        self.assertEqual(consumed_goal.pair_id, self.common["pair_id"])
        self.assertEqual(consumed_goal.goal_x, 0.3)
        with self.assertRaisesRegex(PermitError, "does not exist"):
            consume_staged_nav2_permit(
                runtime,
                {**self.environ, PERMIT_ENV: str(chassis_path)},
                ROOT,
            )

    def test_cross_role_and_pair_or_goal_claim_mismatch_fail_closed(self) -> None:
        runtime = _runtime()
        payload = self._payload(CHASSIS_ROLE, "abcdef")
        with self.assertRaisesRegex(PermitError, "role"):
            validate_permit(
                payload,
                runtime,
                self.environ,
                ROOT,
                expected_role=GOAL_DISPATCH_ROLE,
                now_unix_ns=self.now_ns,
            )
        for name, replacement in (
            (PAIR_ID_ENV, "pair-different-012345"),
            (GOAL_X_ENV, "0.31"),
            (MAP_GENERATION_ENV, "2"),
            (GOAL_EVIDENCE_SHA256_ENV, "0" * 64),
        ):
            with self.subTest(name=name), self.assertRaises(PermitError):
                validate_permit(
                    payload,
                    runtime,
                    {**self.environ, name: replacement},
                    ROOT,
                    expected_role=CHASSIS_ROLE,
                    now_unix_ns=self.now_ns,
                )

    def test_timestamp_tamper_permissions_and_envelope_reject(self) -> None:
        runtime = _runtime()
        payload = self._payload(CHASSIS_ROLE, "abcdef")
        payload["expires_unix_ns"] = payload["issued_unix_ns"]
        with self.assertRaisesRegex(PermitError, "timestamp metadata"):
            validate_permit(
                payload,
                runtime,
                self.environ,
                ROOT,
                expected_role=CHASSIS_ROLE,
                now_unix_ns=self.now_ns,
            )
        payload["expires_unix_ns"] = self.now_ns + 60_000_000_000
        payload["max_angular_z_rps"] = 0.401
        with self.assertRaisesRegex(PermitError, "fixed stage1 envelope"):
            validate_permit(
                payload,
                runtime,
                self.environ,
                ROOT,
                expected_role=CHASSIS_ROLE,
                now_unix_ns=self.now_ns,
            )
        payload["max_angular_z_rps"] = 0.40
        Path(self.evidence["goal"]["path"]).write_text(
            "tampered\n", encoding="utf-8"
        )
        with self.assertRaisesRegex(PermitError, "goal evidence hash changed"):
            validate_permit(
                payload,
                runtime,
                self.environ,
                ROOT,
                expected_role=CHASSIS_ROLE,
                now_unix_ns=self.now_ns,
            )
        Path(self.evidence["goal"]["path"]).write_text(
            "immutable goal\n", encoding="utf-8"
        )
        path = self._write("weak.json", payload)
        path.chmod(0o644)
        with self.assertRaisesRegex(PermitError, "exactly 0600"):
            consume_staged_nav2_permit(
                runtime,
                {**self.environ, PERMIT_ENV: str(path)},
                ROOT,
            )


if __name__ == "__main__":
    unittest.main()
