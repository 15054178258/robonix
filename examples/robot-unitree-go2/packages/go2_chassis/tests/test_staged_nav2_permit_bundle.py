from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import tempfile
import time
import unittest

from go2_chassis.runtime_config import STAGED_NAV2_PROFILE, normalize_config
from go2_chassis.staged_nav2_permit import (
    CHASSIS_ROLE,
    GOAL_DISPATCH_ROLE,
    GOAL_EVIDENCE_REQUIRED_CHECKS,
    GOAL_EVIDENCE_SCHEMA,
    OPERATOR_GOAL_SOURCE,
    PERMIT_SCHEMA,
    STAGED_NAV2_ACK,
    PermitError,
    validate_staged_nav2_permit_bundle,
)


CHASSIS_ROOT = Path(__file__).resolve().parents[1]
ROBOT_ROOT = CHASSIS_ROOT.parents[1]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def runtime():
    return normalize_config(
        {
            "allow_motion": True,
            "motion_profile": STAGED_NAV2_PROFILE,
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": "wlx500ff54809b8",
            "ipc_socket": "/tmp/staged-nav2-bundle-test.sock",
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
        CHASSIS_ROOT,
    )


class StagedNav2PermitBundleTest(unittest.TestCase):
    def setUp(self) -> None:
        run_root = ROBOT_ROOT / "rbnx-build" / "run"
        run_root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=run_root)
        self.directory = Path(self.temporary.name)
        self.directory.chmod(0o700)
        self.goal_pose = {"x": 0.25, "y": -0.05, "yaw": 0.1}
        finished_ns = time.time_ns()
        goal_payload = {
            "schema": GOAL_EVIDENCE_SCHEMA,
            "operation": "compute_path_to_pose_only",
            "action": "/compute_path_to_pose",
            "frame_id": "map",
            "motion_disabled": True,
            "status": "pass",
            "action_status": 4,
            "action_status_name": "succeeded",
            "started_unix_ns": finished_ns - 100_000_000,
            "finished_unix_ns": finished_ns,
            "map": {
                "id": "corridor-map-01",
                "generation": 4,
                "mode": "localization",
            },
            "goal": {
                "source": OPERATOR_GOAL_SOURCE,
                "target_id": "short-goal:01",
                "pose": self.goal_pose,
            },
            "path": {
                "pose_count": 9,
                "length_m": 0.37,
                "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "end": {"x": 0.25, "y": -0.05, "yaw": 0.1},
                "sample_count": 15,
                "map_width": 100,
                "map_height": 100,
                "map_resolution_m": 0.05,
                "lethal_occupancy_threshold": 65,
                "endpoint_position_error_m": 0.0,
                "endpoint_yaw_error_rad": 0.0,
            },
            "localization": {
                "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "source_age_s": 0.01,
            },
            "stage1_geometry": {
                "goal_distance_m": math.hypot(0.25, -0.05),
                "goal_bearing_error_rad": abs(math.atan2(-0.05, 0.25)),
            },
            "checks": {
                name: "pass" for name in GOAL_EVIDENCE_REQUIRED_CHECKS
            },
        }
        self.goal_evidence = self._evidence(
            "goal.json", json.dumps(goal_payload, sort_keys=True)
        )
        self.evidence_paths = {
            name: self._evidence(f"{name}.json", f"{name}\n")
            for name in (
                "dds_identity",
                "state",
                "time",
            )
        }
        self.evidence_paths["goal"] = self.goal_evidence
        evidence = {
            name: {"path": str(path), "sha256": digest(path)}
            for name, path in self.evidence_paths.items()
        }
        now_ns = time.time_ns()
        self.common = {
            "schema": PERMIT_SCHEMA,
            "pair_id": "pair-0123456789abcdef",
            "session_id": "session-0123456789abcdef",
            "one_time": True,
            "issued_unix_ns": now_ns - 1_000_000_000,
            "expires_unix_ns": now_ns + 60_000_000_000,
            "profile": STAGED_NAV2_PROFILE,
            "stage": "stage1",
            "operator_ack": STAGED_NAV2_ACK,
            "guard_ack": STAGED_NAV2_ACK,
            "network_interface": "wlx500ff54809b8",
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
            "map_id": "corridor-map-01",
            "map_generation": 4,
            "goal_source": OPERATOR_GOAL_SOURCE,
            "target_id": "short-goal:01",
            "goal_pose": self.goal_pose,
            "evidence": evidence,
        }
        self.chassis_path = self._permit(
            "chassis-permit.json",
            {
                **self.common,
                "permit_id": "permit-0123456789abcdef",
                "permit_role": CHASSIS_ROLE,
            },
        )
        self.goal_path = self._permit(
            "goal-permit.json",
            {
                **self.common,
                "permit_id": "permit-fedcba9876543210",
                "permit_role": GOAL_DISPATCH_ROLE,
            },
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _evidence(self, name: str, text: str) -> Path:
        path = self.directory / name
        path.write_text(text, encoding="utf-8")
        path.chmod(0o600)
        return path.resolve()

    def _permit(self, name: str, payload: dict[str, object]) -> Path:
        path = self.directory / name
        path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
        path.chmod(0o600)
        return path.resolve()

    def _rewrite(self, path: Path, payload: dict[str, object]) -> None:
        path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
        path.chmod(0o600)

    def _bundle(self):
        return validate_staged_nav2_permit_bundle(
            self.chassis_path,
            self.goal_path,
            runtime(),
            ROBOT_ROOT,
            self.evidence_paths,
        )

    def test_valid_pair_derives_exact_claims_without_consuming(self) -> None:
        before = (digest(self.chassis_path), digest(self.goal_path))
        bundle = self._bundle()
        self.assertEqual(bundle.session_id, self.common["session_id"])
        self.assertEqual(bundle.pair_id, self.common["pair_id"])
        self.assertEqual(bundle.map_id, "corridor-map-01")
        self.assertEqual(bundle.map_generation, 4)
        self.assertEqual(bundle.target_id, "short-goal:01")
        self.assertEqual(bundle.goal_evidence.path_length_m, 0.37)
        self.assertEqual(
            bundle.environment["GO2_STAGED_NAV2_EXPECTED_GOAL_X"], "0.25"
        )
        self.assertTrue(self.chassis_path.is_file())
        self.assertTrue(self.goal_path.is_file())
        self.assertEqual(before, (digest(self.chassis_path), digest(self.goal_path)))
        self.assertFalse(list(self.directory.glob("*.consumed-*")))

    def test_cross_pair_or_launcher_evidence_path_mismatch_rejects(self) -> None:
        goal = json.loads(self.goal_path.read_text(encoding="utf-8"))
        goal["pair_id"] = "pair-different-1234567"
        self._rewrite(self.goal_path, goal)
        with self.assertRaisesRegex(PermitError, "exact matched pair"):
            self._bundle()
        goal["pair_id"] = self.common["pair_id"]
        self._rewrite(self.goal_path, goal)
        other = dict(self.evidence_paths)
        other["state"] = self.goal_evidence
        with self.assertRaisesRegex(PermitError, "state evidence path"):
            validate_staged_nav2_permit_bundle(
                self.chassis_path,
                self.goal_path,
                runtime(),
                ROBOT_ROOT,
                other,
            )

    def test_goal_evidence_map_pose_length_and_checks_are_strict(self) -> None:
        original = json.loads(self.goal_evidence.read_text(encoding="utf-8"))
        mutations = (
            ("map", lambda value: value["map"].update({"generation": 5})),
            ("pose", lambda value: value["goal"]["pose"].update({"x": 0.251})),
            (
                "endpoint position",
                lambda value: value["path"]["end"].update({"x": 0.271}),
            ),
            (
                "endpoint yaw",
                lambda value: value["path"]["end"].update({"yaw": 0.201}),
            ),
            (
                "start missing",
                lambda value: value["path"].pop("start"),
            ),
            (
                "start moved",
                lambda value: value["path"]["start"].update({"x": 0.051}),
            ),
            (
                "bearing summary",
                lambda value: value["stage1_geometry"].update(
                    {"goal_bearing_error_rad": 0.0}
                ),
            ),
            ("length", lambda value: value["path"].update({"length_m": 0.400001})),
            (
                "unknown check",
                lambda value: value["checks"].update(
                    {"path_known_space": "unknown"}
                ),
            ),
        )
        for label, mutation in mutations:
            with self.subTest(label=label):
                payload = json.loads(json.dumps(original))
                mutation(payload)
                self._rewrite(self.goal_evidence, payload)
                new_hash = digest(self.goal_evidence)
                for permit_path in (self.chassis_path, self.goal_path):
                    permit = json.loads(permit_path.read_text(encoding="utf-8"))
                    permit["evidence"]["goal"]["sha256"] = new_hash
                    self._rewrite(permit_path, permit)
                with self.assertRaises(PermitError):
                    self._bundle()
                self._rewrite(self.goal_evidence, original)
                original_hash = digest(self.goal_evidence)
                for permit_path in (self.chassis_path, self.goal_path):
                    permit = json.loads(
                        permit_path.read_text(encoding="utf-8")
                    )
                    permit["evidence"]["goal"]["sha256"] = original_hash
                    self._rewrite(permit_path, permit)

    def test_duplicate_keys_and_future_goal_evidence_reject(self) -> None:
        original_goal = self.goal_evidence.read_text(encoding="utf-8")
        duplicate_goal = original_goal.replace(
            '"status": "pass"', '"status": "pass", "status": "error"', 1
        )
        self.goal_evidence.write_text(duplicate_goal, encoding="utf-8")
        self.goal_evidence.chmod(0o600)
        changed_hash = digest(self.goal_evidence)
        for permit_path in (self.chassis_path, self.goal_path):
            permit = json.loads(permit_path.read_text(encoding="utf-8"))
            permit["evidence"]["goal"]["sha256"] = changed_hash
            self._rewrite(permit_path, permit)
        with self.assertRaisesRegex(PermitError, "duplicate JSON key"):
            self._bundle()

        self.goal_evidence.write_text(original_goal, encoding="utf-8")
        self.goal_evidence.chmod(0o600)
        original_hash = digest(self.goal_evidence)
        for permit_path in (self.chassis_path, self.goal_path):
            permit = json.loads(permit_path.read_text(encoding="utf-8"))
            permit["evidence"]["goal"]["sha256"] = original_hash
            self._rewrite(permit_path, permit)

        original_permit = self.chassis_path.read_text(encoding="utf-8")
        duplicate_permit = original_permit.replace(
            '"one_time": true', '"one_time": true, "one_time": false', 1
        )
        self.chassis_path.write_text(duplicate_permit, encoding="utf-8")
        self.chassis_path.chmod(0o600)
        with self.assertRaisesRegex(PermitError, "duplicate JSON key"):
            self._bundle()
        self.chassis_path.write_text(original_permit, encoding="utf-8")
        self.chassis_path.chmod(0o600)

        original = json.loads(self.goal_evidence.read_text(encoding="utf-8"))
        finished_ns = time.time_ns() + 1_000_000_000
        payload = json.loads(json.dumps(original))
        payload["started_unix_ns"] = finished_ns - 100_000_000
        payload["finished_unix_ns"] = finished_ns
        self._rewrite(self.goal_evidence, payload)
        changed_hash = digest(self.goal_evidence)
        for permit_path in (self.chassis_path, self.goal_path):
            permit = json.loads(
                permit_path.read_text(encoding="utf-8")
            )
            permit["evidence"]["goal"]["sha256"] = changed_hash
            self._rewrite(permit_path, permit)
        with self.assertRaisesRegex(
            PermitError, "timestamps are invalid"
        ):
            self._bundle()
        self._rewrite(self.goal_evidence, original)

    def test_stage1_rejects_landmark_long_identifiers_and_wide_yaw(self) -> None:
        cases = (
            ("goal_source", "verified_landmark"),
            ("session_id", "s" * 65),
            ("map_id", "m" * 65),
            ("target_id", "t" * 65),
            ("goal_pose", {"x": 0.25, "y": -0.05, "yaw": math.pi + 0.01}),
        )
        for field, value in cases:
            with self.subTest(field=field):
                chassis = json.loads(
                    self.chassis_path.read_text(encoding="utf-8")
                )
                goal = json.loads(self.goal_path.read_text(encoding="utf-8"))
                chassis[field] = value
                goal[field] = value
                self._rewrite(self.chassis_path, chassis)
                self._rewrite(self.goal_path, goal)
                with self.assertRaises(PermitError):
                    self._bundle()
                chassis[field] = self.common[field]
                goal[field] = self.common[field]
                self._rewrite(self.chassis_path, chassis)
                self._rewrite(self.goal_path, goal)


if __name__ == "__main__":
    unittest.main()
