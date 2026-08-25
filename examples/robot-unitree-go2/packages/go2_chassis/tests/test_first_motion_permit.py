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

from go2_chassis.first_motion_permit import (  # noqa: E402
    FIRST_MOTION_ACK,
    PERMIT_ENV,
    PERMIT_SCHEMA,
    PermitError,
    consume_first_motion_permit,
    validate_permit,
)
from go2_chassis.runtime_config import normalize_config  # noqa: E402


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _runtime():
    return normalize_config(
        {
            "allow_motion": True,
            "motion_profile": "workstation-first-motion-corrected-v1",
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": "enp1s0",
            "ipc_socket": "/tmp/go2-test.sock",
            "state_topic": "/robonix/time_corrected/motion/sportmodestate",
            "state_fallback_topic": "",
            "twist_in_topic": "/go2/commissioning/cmd_vel",
            "odom_topic": "/odom",
            "max_linear_x_mps": 0.05,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.0,
            "command_timeout_s": 0.20,
            "commissioning_max_duration_s": 2.0,
            "commissioning_max_distance_m": 0.10,
        },
        {
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "100",
        },
        ROOT,
    )


class FirstMotionPermitTest(unittest.TestCase):
    def setUp(self) -> None:
        build = ROOT / "rbnx-build"
        build.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=build)
        self.directory = Path(self.temporary.name)
        self.directory.chmod(0o700)
        self.evidence = {}
        for name in ("dds_identity", "state", "time"):
            path = self.directory / f"{name}.evidence"
            path.write_text(f"immutable {name}\n", encoding="utf-8")
            path.chmod(0o600)
            self.evidence[name] = {"path": str(path), "sha256": _digest(path)}
        self.now_ns = time.time_ns()
        self.payload = {
            "schema": PERMIT_SCHEMA,
            "permit_id": "permit-0123456789abcdef",
            "session_id": "session-0123456789abcdef",
            "one_time": True,
            "issued_unix_ns": self.now_ns - 1_000_000_000,
            "expires_unix_ns": self.now_ns + 60_000_000_000,
            "profile": "workstation-first-motion-corrected-v1",
            "operator_ack": FIRST_MOTION_ACK,
            "network_interface": "enp1s0",
            "state_topic": "/robonix/time_corrected/motion/sportmodestate",
            "command_topic": "/go2/commissioning/cmd_vel",
            "odom_topic": "/odom",
            "arm_service": "/go2_chassis/arm",
            "allowed_modes": [0],
            "allowed_state_markers": [100],
            "max_linear_x_mps": 0.05,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.0,
            "max_duration_s": 2.0,
            "max_distance_m": 0.10,
            "command_timeout_s": 0.20,
            "evidence": self.evidence,
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_permit(self) -> Path:
        path = self.directory / "first-motion-permit.json"
        path.write_text(json.dumps(self.payload), encoding="utf-8")
        path.chmod(0o600)
        return path

    def test_valid_permit_is_bound_and_consumed_once(self) -> None:
        runtime = _runtime()
        self.assertEqual(
            validate_permit(self.payload, runtime, ROOT, now_unix_ns=self.now_ns),
            self.payload["permit_id"],
        )
        path = self._write_permit()
        consumed = consume_first_motion_permit(
            runtime, {PERMIT_ENV: str(path)}, ROOT
        )
        self.assertFalse(path.exists())
        self.assertTrue(consumed.is_file())
        self.assertEqual(oct(consumed.stat().st_mode & 0o777), "0o600")
        with self.assertRaisesRegex(PermitError, "does not exist"):
            consume_first_motion_permit(runtime, {PERMIT_ENV: str(path)}, ROOT)

    def test_expired_or_nearly_expired_permit_fails_closed(self) -> None:
        runtime = _runtime()
        self.payload["expires_unix_ns"] = self.now_ns + 14_000_000_000
        with self.assertRaisesRegex(PermitError, "less than 15 seconds"):
            validate_permit(self.payload, runtime, ROOT, now_unix_ns=self.now_ns)

    def test_tampered_evidence_or_envelope_fails_closed(self) -> None:
        runtime = _runtime()
        evidence_path = Path(self.evidence["state"]["path"])
        evidence_path.write_text("changed\n", encoding="utf-8")
        with self.assertRaisesRegex(PermitError, "state evidence hash changed"):
            validate_permit(self.payload, runtime, ROOT, now_unix_ns=self.now_ns)
        evidence_path.write_text("immutable state\n", encoding="utf-8")
        self.payload["max_linear_x_mps"] = 0.051
        with self.assertRaisesRegex(PermitError, "fixed envelope"):
            validate_permit(self.payload, runtime, ROOT, now_unix_ns=self.now_ns)
        self.payload["max_linear_x_mps"] = 0.05
        self.payload["command_timeout_s"] = 0.21
        with self.assertRaisesRegex(PermitError, "fixed envelope"):
            validate_permit(self.payload, runtime, ROOT, now_unix_ns=self.now_ns)

    def test_permit_permissions_are_private(self) -> None:
        runtime = _runtime()
        path = self._write_permit()
        path.chmod(0o644)
        with self.assertRaisesRegex(PermitError, "exactly 0600"):
            consume_first_motion_permit(runtime, {PERMIT_ENV: str(path)}, ROOT)


if __name__ == "__main__":
    unittest.main()
