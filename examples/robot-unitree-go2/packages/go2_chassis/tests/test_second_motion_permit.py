import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from go2_chassis.runtime_config import (  # noqa: E402
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    SECOND_MOTION_COMMAND_TOPIC,
    SECOND_MOTION_CONTROL_RATE_HZ,
    SECOND_MOTION_PROFILE,
    normalize_config,
)
from go2_chassis.second_motion_permit import (  # noqa: E402
    PERMIT_ENV,
    PERMIT_SCHEMA,
    SECOND_MOTION_ACK,
    PermitError,
    consume_second_motion_permit,
    validate_first_motion_pass,
    validate_permit,
)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _runtime():
    return normalize_config(
        {
            "allow_motion": True,
            "motion_profile": SECOND_MOTION_PROFILE,
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": "wlan-second",
            "ipc_socket": "/tmp/go2-second-permit-test.sock",
            "state_topic": "/robonix/time_corrected/motion/sportmodestate",
            "state_fallback_topic": "",
            "twist_in_topic": SECOND_MOTION_COMMAND_TOPIC,
            "odom_source": "external_verified",
            "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
            "external_odom_timeout_s": 0.20,
            "odom_topic": "/odom",
            "publish_odom_tf": True,
            "stationary_pose_hold_enabled": False,
            "max_linear_x_mps": 0.30,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.0,
            "max_linear_accel_mps2": 0.30,
            "max_angular_accel_rps2": 0.10,
            "command_timeout_s": 0.20,
            "control_rate_hz": SECOND_MOTION_CONTROL_RATE_HZ,
            "state_timeout_s": 0.20,
            "max_source_stamp_age_s": 0.20,
            "max_source_stamp_future_skew_s": 0.05,
            "commissioning_max_duration_s": 1.5,
            "commissioning_max_distance_m": 0.30,
        },
        {
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "100",
        },
        ROOT,
    )


class SecondMotionPermitTest(unittest.TestCase):
    def setUp(self) -> None:
        build = ROOT / "rbnx-build"
        build.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=build)
        self.directory = Path(self.temporary.name)
        self.directory.chmod(0o700)
        self.first_pass = self.directory / "first-motion-pass.json"
        self.first_pass_payload = {
            "schema": "robonix-go2-first-motion-v1",
            "status": "PASS",
            "command_topic": "/go2/commissioning/cmd_vel",
            "command_ownership": "PASS",
            "required_odom_source": "external_verified",
            "commissioning_motion_active_observed": True,
            "disarm_success": True,
            "post_stop_success": True,
            "distance_limit_m": 0.10,
            "duration_limit_s": 2.0,
            "measured_odom_forward_m": 0.031,
            "post_stop": {
                "daemon_armed": False,
                "commissioning_motion_active": False,
                "guard_state": "DISARMED",
            },
        }
        self._write_first_pass()
        self.evidence = {}
        for name in ("dds_identity", "state", "time"):
            path = self.directory / f"{name}.evidence"
            path.write_text(f"immutable {name}\n", encoding="utf-8")
            path.chmod(0o600)
            self.evidence[name] = {
                "path": str(path),
                "sha256": _digest(path),
            }
        self.evidence["first_motion_pass"] = {
            "path": str(self.first_pass),
            "sha256": _digest(self.first_pass),
        }
        self.now_ns = time.time_ns()
        self.payload = {
            "schema": PERMIT_SCHEMA,
            "permit_id": "permit-0123456789abcdef",
            "session_id": "session-0123456789abcdef",
            "one_time": True,
            "issued_unix_ns": self.now_ns - 1_000_000_000,
            "expires_unix_ns": self.now_ns + 60_000_000_000,
            "profile": SECOND_MOTION_PROFILE,
            "operator_ack": SECOND_MOTION_ACK,
            "network_interface": "wlan-second",
            "state_topic": "/robonix/time_corrected/motion/sportmodestate",
            "command_topic": SECOND_MOTION_COMMAND_TOPIC,
            "odom_topic": "/odom",
            "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
            "arm_service": "/go2_chassis/arm",
            "allowed_modes": [0],
            "allowed_state_markers": [100],
            "max_linear_x_mps": 0.30,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.0,
            "max_linear_accel_mps2": 0.30,
            "max_angular_accel_rps2": 0.10,
            "max_duration_s": 1.5,
            "max_distance_m": 0.30,
            "command_timeout_s": 0.20,
            "control_rate_hz": SECOND_MOTION_CONTROL_RATE_HZ,
            "evidence": self.evidence,
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_first_pass(self) -> None:
        self.first_pass.write_text(
            json.dumps(self.first_pass_payload),
            encoding="utf-8",
        )
        self.first_pass.chmod(0o600)

    def _write_permit(self) -> Path:
        path = self.directory / "second-motion-permit.json"
        path.write_text(json.dumps(self.payload), encoding="utf-8")
        path.chmod(0o600)
        return path

    def test_valid_permit_binds_first_pass_and_is_consumed_once(self) -> None:
        runtime = _runtime()
        self.assertEqual(
            validate_first_motion_pass(self.first_pass),
            _digest(self.first_pass),
        )
        self.assertEqual(
            validate_permit(
                self.payload,
                runtime,
                ROOT,
                now_unix_ns=self.now_ns,
            ),
            self.payload["permit_id"],
        )
        path = self._write_permit()
        consumed = consume_second_motion_permit(
            runtime,
            {PERMIT_ENV: str(path)},
            ROOT,
        )
        self.assertFalse(path.exists())
        self.assertTrue(consumed.is_file())
        self.assertEqual(oct(consumed.stat().st_mode & 0o777), "0o600")
        with self.assertRaisesRegex(PermitError, "does not exist"):
            consume_second_motion_permit(
                runtime,
                {PERMIT_ENV: str(path)},
                ROOT,
            )

    def test_first_pass_must_be_successful_and_immutable(self) -> None:
        runtime = _runtime()
        self.first_pass_payload["status"] = "FAIL"
        self._write_first_pass()
        self.evidence["first_motion_pass"]["sha256"] = _digest(
            self.first_pass
        )
        with self.assertRaisesRegex(PermitError, "invalid status"):
            validate_permit(
                self.payload,
                runtime,
                ROOT,
                now_unix_ns=self.now_ns,
            )
        self.first_pass_payload["status"] = "PASS"
        self._write_first_pass()
        with self.assertRaisesRegex(
            PermitError,
            "first_motion_pass evidence hash changed",
        ):
            validate_permit(
                self.payload,
                runtime,
                ROOT,
                now_unix_ns=self.now_ns,
            )

    def test_missing_evidence_or_changed_envelope_fails_closed(self) -> None:
        runtime = _runtime()
        first_claim = self.evidence.pop("first_motion_pass")
        with self.assertRaisesRegex(PermitError, "first-motion PASS"):
            validate_permit(
                self.payload,
                runtime,
                ROOT,
                now_unix_ns=self.now_ns,
            )
        self.evidence["first_motion_pass"] = first_claim
        for key, value in (
            ("max_linear_x_mps", 0.299),
            ("max_linear_accel_mps2", 0.299),
            ("max_angular_accel_rps2", 0.11),
            ("max_duration_s", 1.49),
            ("max_distance_m", 0.29),
            ("command_timeout_s", 0.19),
            ("control_rate_hz", 50.0),
        ):
            original = self.payload[key]
            self.payload[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(
                PermitError,
                "second-motion envelope",
            ):
                validate_permit(
                    self.payload,
                    runtime,
                    ROOT,
                    now_unix_ns=self.now_ns,
                )
            self.payload[key] = original

    def test_expiry_and_private_mode_are_enforced(self) -> None:
        runtime = _runtime()
        self.payload["expires_unix_ns"] = self.now_ns + 14_000_000_000
        with self.assertRaisesRegex(PermitError, "less than 15 seconds"):
            validate_permit(
                self.payload,
                runtime,
                ROOT,
                now_unix_ns=self.now_ns,
            )
        self.payload["expires_unix_ns"] = self.now_ns + 60_000_000_000
        path = self._write_permit()
        path.chmod(0o644)
        with self.assertRaisesRegex(PermitError, "exactly 0600"):
            consume_second_motion_permit(
                runtime,
                {PERMIT_ENV: str(path)},
                ROOT,
            )


if __name__ == "__main__":
    unittest.main()
