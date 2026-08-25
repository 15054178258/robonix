from __future__ import annotations

import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "scripts" / "staged_nav2_motion_guard.py"
SPEC = importlib.util.spec_from_file_location("staged_nav2_motion_guard", SOURCE)
assert SPEC is not None and SPEC.loader is not None
guard_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = guard_module
SPEC.loader.exec_module(guard_module)
GOAL_ID = "01" * 16
OTHER_GOAL_ID = "02" * 16
EVIDENCE_HASH = "ab" * 32


class StagedNav2MotionGuardTest(unittest.TestCase):
    def _claims(self) -> dict[str, str]:
        return {
            "GO2_ALLOW_MOTION": "true",
            "GO2_MOTION_PROFILE": guard_module.PROFILE,
            "GO2_STAGED_NAV2_STAGE": guard_module.STAGE,
            "GO2_STAGED_NAV2_GUARD_ACK": guard_module.GUARD_ACK,
            "GO2_STAGED_NAV2_SESSION_ID": "session-20260724",
            "GO2_STAGED_NAV2_PAIR_ID": "pair-20260724",
            "GO2_STAGED_NAV2_GOAL_SOURCE": (
                guard_module.STAGE1_GOAL_SOURCE
            ),
            "GO2_STAGED_NAV2_TARGET_ID": "operator-click-1",
            "GO2_STAGED_NAV2_MAP_ID": "corridor-map",
            "GO2_STAGED_NAV2_MAP_GENERATION": "7",
            "GO2_STAGED_NAV2_EXPECTED_GOAL_X": "0.25",
            "GO2_STAGED_NAV2_EXPECTED_GOAL_Y": "-0.10",
            "GO2_STAGED_NAV2_EXPECTED_GOAL_YAW": "0.2",
            "GO2_STAGED_NAV2_EXPECTED_START_X": "0.0",
            "GO2_STAGED_NAV2_EXPECTED_START_Y": "0.0",
            "GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256": EVIDENCE_HASH,
            "GO2_ALLOWED_MODES": "0",
            "GO2_ALLOWED_STATE_MARKERS": "100",
        }

    def _expected(self) -> guard_module.ExpectedGoalClaim:
        return guard_module.validate_environment(self._claims()).expected_goal

    def _policy(
        self, *, enabled: bool = True
    ) -> guard_module.StagedNav2Guard:
        claims = guard_module.validate_environment(self._claims())
        return guard_module.StagedNav2Guard(
            enabled=enabled,
            expected_goal=claims.expected_goal,
            allowed_modes=claims.allowed_modes,
            allowed_state_markers=claims.allowed_state_markers,
        )

    def _goal_claim(self, goal_id: str = GOAL_ID) -> dict[str, object]:
        expected = self._expected()
        return {
            "schema": guard_module.GOAL_CLAIM_SCHEMA,
            "session_id": expected.session_id,
            "pair_id": expected.pair_id,
            "source": expected.source,
            "target_id": expected.target_id,
            "map_id": expected.map_id,
            "generation": expected.generation,
            "pose": {
                "x": expected.x_m,
                "y": expected.y_m,
                "yaw": expected.yaw_rad,
            },
            "goal_evidence_sha256": expected.evidence_sha256,
            "goal_uuid": goal_id,
        }

    def _refresh(
        self,
        policy: guard_module.StagedNav2Guard,
        now_s: float,
        *,
        omit: str = "",
    ) -> None:
        if omit != "state":
            policy.observe_state(
                now_s, mode=0, error_code=100, source_age_s=0.01
            )
        if omit != "odom":
            policy.observe_odom(now_s, 0.0, 0.0, source_age_s=0.01)
        if omit != "tf":
            policy.observe("tf", now_s)
        if omit != "scan":
            policy.observe_scan(
                now_s,
                frame_id="laser_link",
                angle_min=-1.0,
                angle_max=1.0,
                angle_increment=0.01,
                range_min=0.05,
                range_max=20.0,
                ranges=(1.0, 2.0),
                source_age_s=0.01,
            )
        if omit != "localization":
            policy.observe_localization(
                now_s,
                frame_id="map",
                x_m=0.0,
                y_m=0.0,
                yaw_rad=0.0,
                source_age_s=0.01,
            )
        if omit != "goal_status":
            policy.observe("goal_status", now_s)
        if omit != "goal_claim":
            if policy._accepted_goal_claim is None:
                policy.observe_goal_claim(
                    now_s,
                    self._goal_claim(),
                    publisher_nodes=(
                        guard_module.EXPECTED_GOAL_DISPATCH_NODE,
                    ),
                )
            else:
                policy.refresh_goal_claim(
                    now_s,
                    publisher_nodes=(
                        guard_module.EXPECTED_GOAL_DISPATCH_NODE,
                    ),
                )
        if omit != "map":
            policy.observe("map", now_s)
        if omit != "map_lifecycle":
            policy.observe_map_lifecycle(
                now_s,
                map_id="corridor-map",
                mode="localization",
                generation=7,
            )
        if omit != "ownership":
            policy.observe_ownership(
                now_s,
                controller_publishers=(guard_module.EXPECTED_CONTROLLER_NODE,),
                output_publishers=(f"/{guard_module.NODE_NAME}",),
                chassis_subscribers=(guard_module.EXPECTED_CHASSIS_NODE,),
                canonical_publishers=0,
            )
        if omit != "chassis":
            if policy.phase is guard_module.Phase.ARMED:
                guard_state, daemon_armed = "ARMED", True
            elif policy.phase is guard_module.Phase.PENDING_ARM:
                guard_state, daemon_armed = "PREPARING", False
            else:
                guard_state, daemon_armed = "DISARMED", False
            policy.observe_chassis_status(
                now_s,
                guard_state=guard_state,
                daemon_armed=daemon_armed,
                motion_configured=True,
                motion_profile=guard_module.PROFILE,
                odom_source="external_verified",
            )

    def _armed(
        self, base_s: float = 100.0
    ) -> guard_module.StagedNav2Guard:
        policy = self._policy()
        self._refresh(policy, base_s)
        decision = policy.begin_goal(GOAL_ID, base_s + 0.01)
        self.assertTrue(decision.request_arm)
        decision = policy.confirm_arm(True, base_s + 0.02)
        self.assertEqual(policy.phase, guard_module.Phase.PENDING_ARM)
        self.assertTrue(decision.is_zero)
        self._refresh(policy, base_s + 0.61)
        decision = policy.observe_chassis_status(
            base_s + 0.62,
            guard_state="ARMED",
            daemon_armed=True,
            motion_configured=True,
            motion_profile=guard_module.PROFILE,
            odom_source="external_verified",
        )
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertTrue(decision.is_zero)
        return policy

    def test_runtime_claims_are_exact_and_default_execution_is_disabled(self) -> None:
        claims = guard_module.validate_environment(self._claims())
        self.assertEqual(claims.stage, "stage1")
        self.assertEqual(claims.session_id, "session-20260724")
        self.assertEqual(claims.allowed_modes, (0,))
        self.assertEqual(claims.allowed_state_markers, (100,))
        self.assertEqual(claims.expected_goal.start_x_m, 0.0)
        self.assertEqual(claims.expected_goal.start_y_m, 0.0)

        target_with_namespace = self._claims()
        target_with_namespace["GO2_STAGED_NAV2_TARGET_ID"] = "ui:short-goal-01"
        self.assertEqual(
            guard_module.validate_environment(
                target_with_namespace
            ).expected_goal.target_id,
            "ui:short-goal-01",
        )
        short_map = self._claims()
        short_map["GO2_STAGED_NAV2_MAP_ID"] = "map"
        with self.assertRaises(guard_module.GuardError):
            guard_module.validate_environment(short_map)

        for key in self._claims():
            with self.subTest(missing=key):
                broken = self._claims()
                broken.pop(key)
                with self.assertRaises(guard_module.GuardError):
                    guard_module.validate_environment(broken)
        broken = self._claims()
        broken["GO2_STAGED_NAV2_SESSION_ID"] = "short"
        with self.assertRaises(guard_module.GuardError):
            guard_module.validate_environment(broken)
        for key, value in (
            ("GO2_ALLOWED_MODES", "255"),
            ("GO2_ALLOWED_MODES", "0,0"),
            ("GO2_ALLOWED_STATE_MARKERS", "0"),
            ("GO2_STAGED_NAV2_EXPECTED_START_X", "nan"),
        ):
            with self.subTest(key=key, value=value):
                broken = self._claims()
                broken[key] = value
                with self.assertRaises(guard_module.GuardError):
                    guard_module.validate_environment(broken)

        completed = subprocess.run(
            [sys.executable, str(SOURCE)],
            cwd=ROOT,
            env={
                "PATH": os.environ.get("PATH", ""),
                "PYTHONPATH": os.environ.get("PYTHONPATH", ""),
            },
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("guard disabled", completed.stderr)

    def test_goal_claim_must_exactly_match_permit_derived_pose_and_hash(
        self,
    ) -> None:
        expected = self._expected()
        accepted = guard_module.validate_goal_claim(
            self._goal_claim(), expected
        )
        self.assertEqual(accepted.goal_uuid, GOAL_ID)
        for mutation in (
            lambda value: value.update({"session_id": "other-session"}),
            lambda value: value.update({"map_id": "other-map"}),
            lambda value: value.update({"generation": 8}),
            lambda value: value["pose"].update({"x": 0.250001}),
            lambda value: value.update({"goal_evidence_sha256": "cd" * 32}),
            lambda value: value.update({"goal_uuid": "not-a-uuid"}),
            lambda value: value.update({"extra": True}),
        ):
            with self.subTest(mutation=mutation):
                payload = self._goal_claim()
                mutation(payload)
                with self.assertRaises(guard_module.GuardError):
                    guard_module.validate_goal_claim(payload, expected)

    def test_active_goal_waits_zero_for_exact_claim_then_locks_uuid(self) -> None:
        policy = self._policy()
        self._refresh(policy, 10.0, omit="goal_claim")
        waiting = policy.observe_goal_statuses(
            10.01, [(GOAL_ID, "executing")]
        )
        self.assertEqual(policy.phase, guard_module.Phase.IDLE)
        self.assertTrue(waiting.is_zero)
        self.assertFalse(waiting.request_arm)
        accepted = policy.observe_goal_claim(
            10.02,
            self._goal_claim(),
            publisher_nodes=(guard_module.EXPECTED_GOAL_DISPATCH_NODE,),
        )
        self.assertEqual(policy.phase, guard_module.Phase.PENDING_ARM)
        self.assertTrue(accepted.request_arm)

        policy = self._policy()
        self._refresh(policy, 12.0, omit="goal_claim")
        delivered_before_graph = policy.observe_goal_claim(
            12.01,
            self._goal_claim(),
            publisher_nodes=(),
        )
        self.assertEqual(policy.phase, guard_module.Phase.IDLE)
        self.assertEqual(
            delivered_before_graph.reason, "goal_claim_accepted"
        )

        second = policy.observe_goal_claim(
            10.03,
            self._goal_claim(),
            publisher_nodes=(guard_module.EXPECTED_GOAL_DISPATCH_NODE,),
        )
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(second.reason, "second_goal_claim")
        self.assertTrue(second.request_disarm)

        policy = self._policy()
        self._refresh(policy, 15.0, omit="goal_claim")
        owner = policy.observe_goal_claim(
            15.01,
            self._goal_claim(),
            publisher_nodes=(
                guard_module.EXPECTED_GOAL_DISPATCH_NODE,
                "/unknown_goal_sender",
            ),
        )
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(
            owner.reason, "goal_claim_owner_missing_or_ambiguous"
        )

        policy = self._policy()
        self._refresh(policy, 20.0, omit="goal_claim")
        policy.observe_goal_statuses(20.01, [(GOAL_ID, "executing")])
        mismatch = policy.observe_goal_claim(
            20.02,
            self._goal_claim(OTHER_GOAL_ID),
            publisher_nodes=(guard_module.EXPECTED_GOAL_DISPATCH_NODE,),
        )
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(mismatch.reason, "goal_claim_uuid_mismatch")
        self.assertTrue(mismatch.request_cancel)
        retry = policy.tick(20.03)
        self.assertTrue(retry.request_cancel)

        policy = self._policy()
        self._refresh(policy, 30.0, omit="goal_claim")
        policy.observe_goal_statuses(30.01, [(GOAL_ID, "executing")])
        timeout = policy.tick(
            30.01 + guard_module.UNVERIFIED_GOAL_TIMEOUT_S + 0.001
        )
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(timeout.reason, "active_goal_claim_timeout")
        self.assertTrue(timeout.request_cancel)

    def test_disabled_policy_never_requests_arm_or_forwards_command(self) -> None:
        policy = self._policy(enabled=False)
        self._refresh(policy, 10.0)
        decision = policy.begin_goal(GOAL_ID, 10.01)
        self.assertEqual(policy.phase, guard_module.Phase.DISABLED)
        self.assertTrue(decision.is_zero)
        self.assertFalse(decision.request_arm)
        self.assertTrue(decision.request_disarm)
        decision = policy.command(10.02, 0.01, 0.0, 0.0)
        self.assertTrue(decision.is_zero)

    def test_goal_waits_for_every_fresh_witness_and_exact_single_owner(self) -> None:
        for missing in guard_module.FRESHNESS_LIMIT_S:
            if missing == "goal_status":
                continue
            with self.subTest(missing=missing):
                policy = self._policy()
                self._refresh(policy, 10.0, omit=missing)
                decision = policy.begin_goal(GOAL_ID, 10.01)
                self.assertTrue(decision.is_zero)
                self.assertIn(missing, decision.reason)
                if missing == "goal_claim":
                    self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                    self.assertTrue(decision.request_disarm)
                    self.assertTrue(decision.request_cancel)
                else:
                    self.assertEqual(policy.phase, guard_module.Phase.IDLE)
                    self.assertFalse(decision.request_arm)
                    self.assertFalse(decision.request_disarm)
                    self.assertFalse(decision.request_cancel)

        policy = self._policy()
        self._refresh(policy, 20.0, omit="ownership")
        decision = policy.observe_ownership(
            20.0,
            controller_publishers=(
                guard_module.EXPECTED_CONTROLLER_NODE,
                "/behavior_server",
            ),
            output_publishers=(f"/{guard_module.NODE_NAME}",),
            chassis_subscribers=(guard_module.EXPECTED_CHASSIS_NODE,),
            canonical_publishers=0,
        )
        self.assertIn("ownership", decision.reason)
        decision = policy.begin_goal(GOAL_ID, 20.01)
        self.assertEqual(policy.phase, guard_module.Phase.IDLE)
        self.assertEqual(
            decision.reason, "goal_waiting_for_ownership_missing"
        )
        self.assertFalse(decision.request_arm)
        self.assertFalse(decision.request_cancel)

    def test_canonical_cmd_vel_bypass_and_unknown_chassis_are_rejected(self) -> None:
        cases = (
            {
                "controller_publishers": (guard_module.EXPECTED_CONTROLLER_NODE,),
                "output_publishers": (f"/{guard_module.NODE_NAME}",),
                "chassis_subscribers": (guard_module.EXPECTED_CHASSIS_NODE,),
                "canonical_publishers": 1,
            },
            {
                "controller_publishers": ("/unknown_controller",),
                "output_publishers": (f"/{guard_module.NODE_NAME}",),
                "chassis_subscribers": (guard_module.EXPECTED_CHASSIS_NODE,),
                "canonical_publishers": 0,
            },
            {
                "controller_publishers": (guard_module.EXPECTED_CONTROLLER_NODE,),
                "output_publishers": (f"/{guard_module.NODE_NAME}",),
                "chassis_subscribers": ("/other_adapter",),
                "canonical_publishers": 0,
            },
        )
        for values in cases:
            with self.subTest(values=values):
                policy = self._armed()
                decision = policy.observe_ownership(100.63, **values)
                self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                self.assertTrue(decision.is_zero)
                self.assertTrue(decision.request_disarm)
                self.assertTrue(decision.request_cancel)

    def test_only_active_goal_can_arm_and_arm_failure_latches(self) -> None:
        policy = self._policy()
        self._refresh(policy, 30.0)
        idle = policy.observe_goal_statuses(30.01, [])
        self.assertFalse(idle.request_arm)
        active = policy.observe_goal_statuses(
            30.02, [(GOAL_ID, "executing")]
        )
        self.assertTrue(active.request_arm)
        self.assertEqual(policy.phase, guard_module.Phase.PENDING_ARM)
        rejected = policy.confirm_arm(False, 30.03)
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertTrue(rejected.request_disarm)
        self.assertTrue(rejected.request_cancel)

    def test_bound_goal_waits_zero_for_fresh_state_before_arming(self) -> None:
        policy = self._policy()
        self._refresh(policy, 35.0)
        policy._witnesses["state"] = None

        waiting = policy.observe_goal_statuses(
            35.01, [(GOAL_ID, "executing")]
        )
        self.assertEqual(policy.phase, guard_module.Phase.IDLE)
        self.assertEqual(waiting.reason, "goal_waiting_for_state_missing")
        self.assertTrue(waiting.is_zero)
        self.assertFalse(waiting.request_arm)
        self.assertFalse(waiting.request_cancel)
        self.assertFalse(policy.session_spent)

        still_waiting = policy.tick(35.02)
        self.assertEqual(
            still_waiting.reason, "goal_waiting_for_state_missing"
        )
        self.assertEqual(policy.phase, guard_module.Phase.IDLE)

        policy.observe_state(
            35.03, mode=0, error_code=100, source_age_s=0.01
        )
        accepted = policy.tick(35.04)
        self.assertEqual(accepted.reason, "goal_active_request_arm")
        self.assertTrue(accepted.request_arm)
        self.assertEqual(policy.phase, guard_module.Phase.PENDING_ARM)
        self.assertTrue(policy.session_spent)

    def test_arm_ack_requires_zero_preamble_and_measured_armed_status(
        self,
    ) -> None:
        policy = self._policy()
        self._refresh(policy, 40.0)
        policy.begin_goal(GOAL_ID, 40.01)
        ack = policy.confirm_arm(True, 40.02)
        self.assertEqual(policy.phase, guard_module.Phase.PENDING_ARM)
        self.assertTrue(ack.is_zero)
        early = policy.observe_chassis_status(
            40.10,
            guard_state="ARMED",
            daemon_armed=True,
            motion_configured=True,
            motion_profile=guard_module.PROFILE,
            odom_source="external_verified",
        )
        self.assertEqual(policy.phase, guard_module.Phase.PENDING_ARM)
        self.assertTrue(early.is_zero)
        command = policy.command(40.11, 0.05, 0.0, 0.0)
        self.assertTrue(command.is_zero)
        self._refresh(policy, 40.61)
        measured = policy.observe_chassis_status(
            40.62,
            guard_state="ARMED",
            daemon_armed=True,
            motion_configured=True,
            motion_profile=guard_module.PROFILE,
            odom_source="external_verified",
        )
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(
            measured.reason, "chassis_measured_armed_after_zero_hold"
        )

        policy = self._policy()
        self._refresh(policy, 50.0)
        policy.begin_goal(GOAL_ID, 50.01)
        policy.confirm_arm(True, 50.02)
        now_s = (
            50.01
            + guard_module.LIMITS.chassis_ready_timeout_s
            + 0.001
        )
        self._refresh(policy, now_s)
        timeout = policy.tick(now_s)
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(timeout.reason, "chassis_measured_arm_timeout")

    def test_stage_envelope_and_slew_limits_are_enforced(self) -> None:
        policy = self._armed()
        first = policy.command(100.72, 0.05, 0.0, 0.15)
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertGreater(first.command[0], 0.0)
        self.assertLessEqual(first.command[0], 0.030000001)
        self.assertLessEqual(abs(first.command[2]), 0.080000001)
        second = policy.command(100.77, 0.05, 0.0, 0.15)
        self.assertLessEqual(
            second.command[0] - first.command[0],
            guard_module.LIMITS.max_linear_accel_mps2 * 0.05 + 1e-9,
        )
        self.assertLessEqual(
            second.command[2] - first.command[2],
            guard_module.LIMITS.max_angular_accel_rps2 * 0.05 + 1e-9,
        )

        invalid_commands = (
            (-0.001, 0.0, 0.0, "linear_x"),
            (0.301, 0.0, 0.0, "linear_x"),
            (0.01, 0.001, 0.0, "linear_y"),
            (0.01, 0.0, 0.401, "angular_z"),
            (math.nan, 0.0, 0.0, "non_finite"),
            (0.0, 0.0, math.inf, "non_finite"),
        )
        for vx, vy, wz, reason in invalid_commands:
            with self.subTest(command=(vx, vy, wz)):
                candidate = self._armed()
                decision = candidate.command(100.63, vx, vy, wz)
                self.assertEqual(candidate.phase, guard_module.Phase.FAULT)
                self.assertTrue(decision.is_zero)
                self.assertTrue(decision.request_disarm)
                self.assertIn(reason, decision.reason)

    def test_every_runtime_witness_stale_path_zeroes_and_disarms(self) -> None:
        for stale in guard_module.FRESHNESS_LIMIT_S:
            if stale in {
                "goal_status",
                "goal_claim",
                "scan",
                "localization",
            }:
                continue
            with self.subTest(stale=stale):
                policy = self._armed()
                now_s = 102.0 + max(
                    0.0,
                    guard_module.FRESHNESS_LIMIT_S[stale] - 1.0,
                )
                self._refresh(policy, now_s, omit=stale)
                policy._last_command_at = now_s
                decision = policy.tick(now_s)
                self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                self.assertTrue(decision.is_zero)
                self.assertTrue(decision.request_disarm)
                self.assertTrue(decision.request_cancel)
                self.assertIn(stale, decision.reason)

        policy = self._armed()
        self._refresh(policy, 100.0)
        policy._witnesses["goal_status"] = None
        policy._last_command_at = 100.0
        decision = policy.tick(100.01)
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(decision.reason, "armed_healthy")

        policy = self._armed()
        self._refresh(policy, 101.0)
        decision = policy.tick(101.0)
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(decision.reason, "command_timeout_zero_hold")
        self.assertTrue(decision.is_zero)
        self.assertFalse(decision.request_disarm)
        self.assertFalse(decision.request_cancel)

        waiting = policy.tick(101.01)
        self.assertEqual(
            waiting.reason, "armed_waiting_for_controller_command"
        )
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)

        resumed = policy.command(101.06, 0.10, 0.0, 0.0)
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(resumed.reason, "command_forwarded")
        self.assertGreater(resumed.command[0], 0.0)
        self.assertLessEqual(
            resumed.command[0],
            guard_module.LIMITS.max_linear_accel_mps2 * 0.05 + 1e-9,
        )

    def test_stale_and_future_source_stamps_fail_closed(self) -> None:
        for source in guard_module.SOURCE_STAMP_LIMIT_S:
            if source in {"scan", "localization"}:
                continue
            with self.subTest(stale=source):
                policy = self._armed()
                decision = (
                    policy.observe_odom(
                        100.63,
                        0.0,
                        0.0,
                        source_age_s=guard_module.SOURCE_STAMP_LIMIT_S[source]
                        + 0.001,
                    )
                    if source == "odom"
                    else policy.observe(
                        source,
                        100.63,
                        source_age_s=guard_module.SOURCE_STAMP_LIMIT_S[source]
                        + 0.001,
                    )
                )
                self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                self.assertTrue(decision.request_disarm)
                self.assertIn("source_stamp", decision.reason)

            with self.subTest(future=source):
                policy = self._armed()
                decision = (
                    policy.observe_odom(
                        100.63,
                        0.0,
                        0.0,
                        source_age_s=-guard_module.MAX_SOURCE_FUTURE_S - 0.001,
                    )
                    if source == "odom"
                    else policy.observe(
                        source,
                        100.63,
                        source_age_s=-guard_module.MAX_SOURCE_FUTURE_S - 0.001,
                    )
                )
                self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                self.assertTrue(decision.request_disarm)

    def test_state_source_stamp_uses_shared_200ms_boundary(self) -> None:
        limit_s = guard_module.SOURCE_STAMP_LIMIT_S["state"]
        self.assertEqual(limit_s, 0.20)

        policy = self._armed()
        decision = policy.observe_state(
            100.63,
            mode=0,
            error_code=100,
            source_age_s=limit_s,
        )
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(decision.reason, "state_fresh")
        self.assertFalse(decision.request_disarm)

        policy = self._armed()
        decision = policy.observe_state(
            100.63,
            mode=0,
            error_code=100,
            source_age_s=math.nextafter(limit_s, math.inf),
        )
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(decision.reason, "state_source_stamp_invalid")
        self.assertTrue(decision.request_disarm)

    def test_live_state_scan_localization_and_chassis_payloads_fail_closed(
        self,
    ) -> None:
        for mode, marker in ((1, 100), (0, 101)):
            with self.subTest(state=(mode, marker)):
                policy = self._armed()
                decision = policy.observe_state(
                    100.63,
                    mode=mode,
                    error_code=marker,
                    source_age_s=0.01,
                )
                self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                self.assertEqual(
                    decision.reason, "state_mode_or_marker_mismatch"
                )

        policy = self._armed()
        accepted_zero = policy.observe_state(
            100.63, mode=0, error_code=0, source_age_s=0.01
        )
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(accepted_zero.reason, "state_fresh")

        policy = self._armed()
        invalid_scan = policy.observe_scan(
            100.63,
            frame_id="laser_link",
            angle_min=-1.0,
            angle_max=1.0,
            angle_increment=0.01,
            range_min=0.05,
            range_max=20.0,
            ranges=(math.inf, math.nan),
            source_age_s=0.01,
        )
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(invalid_scan.reason, "scan_delegated_to_nav2")

        for frame_id, x_m in (("odom", 0.0), ("map", math.nan)):
            with self.subTest(localization=(frame_id, x_m)):
                policy = self._armed()
                invalid_pose = policy.observe_localization(
                    100.63,
                    frame_id=frame_id,
                    x_m=x_m,
                    y_m=0.0,
                    yaw_rad=0.0,
                    source_age_s=0.01,
                )
                self.assertEqual(policy.phase, guard_module.Phase.ARMED)
                self.assertEqual(
                    invalid_pose.reason, "localization_delegated_to_nav2"
                )

        for field, value in (
            ("motion_configured", False),
            ("motion_profile", "another-profile"),
            ("odom_source", "sport_state"),
        ):
            with self.subTest(chassis_field=field):
                policy = self._armed()
                arguments = {
                    "guard_state": "ARMED",
                    "daemon_armed": True,
                    "motion_configured": True,
                    "motion_profile": guard_module.PROFILE,
                    "odom_source": "external_verified",
                }
                arguments[field] = value
                invalid_chassis = policy.observe_chassis_status(
                    100.63, **arguments
                )
                self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                self.assertEqual(
                    invalid_chassis.reason,
                    "chassis_status_or_profile_invalid",
                )

    def test_permit_start_is_live_confirmed_until_measured_arm(self) -> None:
        policy = self._policy()
        self._refresh(policy, 10.0, omit="localization")
        policy.observe_localization(
            10.0,
            frame_id="map",
            x_m=guard_module.MAX_START_POSITION_ERROR_M + 0.001,
            y_m=0.0,
            yaw_rad=2.5,
            source_age_s=0.01,
        )
        rejected = policy.begin_goal(GOAL_ID, 10.01)
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(
            rejected.reason,
            "goal_started_outside_permitted_start_position",
        )

        policy = self._policy()
        self._refresh(policy, 20.0, omit="localization")
        policy.observe_localization(
            20.0,
            frame_id="map",
            x_m=guard_module.MAX_START_POSITION_ERROR_M,
            y_m=0.0,
            yaw_rad=2.5,
            source_age_s=0.01,
        )
        accepted = policy.begin_goal(GOAL_ID, 20.01)
        self.assertTrue(accepted.request_arm)
        self.assertEqual(policy.phase, guard_module.Phase.PENDING_ARM)
        moved = policy.observe_localization(
            20.02,
            frame_id="map",
            x_m=guard_module.MAX_START_POSITION_ERROR_M + 0.001,
            y_m=0.0,
            yaw_rad=-2.5,
            source_age_s=0.01,
        )
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(
            moved.reason,
            "localization_left_permitted_start_before_arm",
        )

    def test_bound_goal_disappearance_or_duplicate_status_fails_closed(
        self,
    ) -> None:
        for statuses, expected_reason in (
            ((), "bound_nav2_goal_status_missing_or_ambiguous"),
            (
                ((GOAL_ID, "executing"), (GOAL_ID, "executing")),
                "multiple_active_nav2_goals",
            ),
            (
                ((OTHER_GOAL_ID, "succeeded"),),
                "bound_nav2_goal_status_missing_or_ambiguous",
            ),
        ):
            with self.subTest(statuses=statuses):
                policy = self._armed()
                decision = policy.observe_goal_statuses(100.63, statuses)
                self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                self.assertEqual(decision.reason, expected_reason)
                self.assertTrue(decision.request_cancel)
                self.assertTrue(decision.request_disarm)

    def test_cancel_failure_success_disconnect_and_shutdown_stop(self) -> None:
        transitions = (
            ("canceling", guard_module.Phase.COMPLETE, "canceled"),
            ("canceled", guard_module.Phase.COMPLETE, "canceled"),
            ("succeeded", guard_module.Phase.COMPLETE, "succeeded"),
            ("aborted", guard_module.Phase.FAULT, "failed"),
            ("unknown", guard_module.Phase.FAULT, "failed"),
        )
        for status, phase, reason in transitions:
            with self.subTest(status=status):
                policy = self._armed()
                decision = policy.observe_goal_statuses(
                    100.63, [(GOAL_ID, status)]
                )
                self.assertEqual(policy.phase, phase)
                self.assertTrue(decision.is_zero)
                self.assertTrue(decision.request_disarm)
                self.assertIn(reason, decision.reason)

        for method, reason in (
            ("disconnect", "network_disconnect"),
            ("shutdown", "shutdown"),
        ):
            with self.subTest(method=method):
                policy = self._armed()
                decision = (
                    policy.disconnect(reason)
                    if method == "disconnect"
                    else policy.shutdown()
                )
                self.assertEqual(policy.phase, guard_module.Phase.FAULT)
                self.assertTrue(decision.is_zero)
                self.assertTrue(decision.request_disarm)
                self.assertTrue(decision.request_cancel)

    def test_multiple_goals_and_replay_are_blocked_without_trip_timer(self) -> None:
        policy = self._policy()
        self._refresh(policy, 1.0)
        decision = policy.observe_goal_statuses(
            1.01,
            [(GOAL_ID, "executing"), (OTHER_GOAL_ID, "accepted")],
        )
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(decision.reason, "multiple_active_nav2_goals")

        policy = self._armed()
        first_leg = policy.observe_odom(
            100.63, 0.20, 0.0, source_age_s=0.01
        )
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(first_leg.reason, "odom_fresh")
        policy.observe_odom(100.64, 0.0, 0.0, source_age_s=0.01)
        cumulative = policy.observe_odom(
            100.65, 0.10, 0.0, source_age_s=0.01
        )
        self.assertEqual(policy.phase, guard_module.Phase.ARMED)
        self.assertEqual(cumulative.reason, "odom_fresh")

        policy = self._armed()
        policy.observe_goal_statuses(100.63, [(GOAL_ID, "succeeded")])
        replay = policy.begin_goal(OTHER_GOAL_ID, 100.64)
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(replay.reason, "staged_session_is_one_goal_only")

    def test_map_lifecycle_is_named_localization_epoch_and_cannot_change(
        self,
    ) -> None:
        for values in (
            {"map_id": "", "mode": "localization", "generation": 7},
            {"map_id": "corridor-map", "mode": "mapping", "generation": 7},
            {
                "map_id": "corridor-map",
                "mode": "localization",
                "generation": 0,
            },
            {
                "map_id": "other-map",
                "mode": "localization",
                "generation": 7,
            },
        ):
            with self.subTest(values=values):
                policy = self._policy()
                decision = policy.observe_map_lifecycle(1.0, **values)
                self.assertIn("map_lifecycle", decision.reason)

        policy = self._armed()
        changed = policy.observe_map_lifecycle(
            100.63,
            map_id="corridor-map",
            mode="localization",
            generation=8,
        )
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertTrue(changed.request_disarm)

    def test_arm_response_timeout_is_fail_closed(self) -> None:
        policy = self._policy()
        self._refresh(policy, 5.0)
        policy.begin_goal(GOAL_ID, 5.01)
        now_s = 5.01 + guard_module.LIMITS.arm_response_timeout_s + 0.001
        self._refresh(policy, now_s)
        decision = policy.tick(now_s)
        self.assertEqual(policy.phase, guard_module.Phase.FAULT)
        self.assertEqual(decision.reason, "chassis_arm_response_timeout")
        self.assertTrue(decision.request_disarm)

    def test_source_has_private_route_lazy_ros_import_and_shutdown_zero_burst(
        self,
    ) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        pure_prefix = source[: source.index("def _run_ros(")]
        self.assertNotIn("import rclpy", pure_prefix)
        self.assertIn('OUTPUT_TOPIC = "/go2/staged_nav2/cmd_vel"', source)
        self.assertIn(
            'LOCALIZATION_TOPIC = "/robonix/map/pose"', source
        )
        self.assertIn(
            'MAP_LIFECYCLE_TOPIC = "/robonix/map/lifecycle"', source
        )
        self.assertIn(
            'GOAL_CLAIM_TOPIC = "/robonix/staged_nav2/goal_claim"',
            source,
        )
        self.assertIn('CANONICAL_COMMAND_TOPIC = "/cmd_vel"', source)
        self.assertNotIn(
            'create_publisher(Twist, CANONICAL_COMMAND_TOPIC', source
        )
        self.assertIn("for _ in range(10):", source)
        cancel_wait = source.index("cancel_deadline = time.monotonic() + 0.20")
        disarm_request = source.index("self._request_disarm()", cancel_wait)
        self.assertLess(cancel_wait, disarm_request)
        self.assertIn("request.data = False", source)
        self.assertIn("node.policy.shutdown()", source)
        self.assertIn("def stop_and_disarm(self) -> bool:", source)
        self.assertIn("node.policy.phase is Phase.FAULT or not disarmed", source)
        self.assertIn("POST_STOP_OBSERVATION_S", source)
        self.assertIn("POST_STOP_MIN_ODOM_SAMPLES", source)
        self.assertIn("POST_STOP_MAX_DRIFT_M", source)
        self.assertIn("POST_STOP_MAX_LINEAR_MPS", source)
        self.assertIn("POST_STOP_MAX_YAW_RPS", source)
        self.assertIn("validate_action_result(", source)
        self.assertIn("validate_measured_result(", source)
        self.assertIn("write_private_json(measured_result_path", source)

    def test_stage_measurement_reports_body_frame_endpoint_metrics(self) -> None:
        measurement = guard_module.StageOdomMeasurement()
        measurement.start(
            guard_module.OdomSnapshot(
                monotonic_s=1.0,
                x_m=1.0,
                y_m=2.0,
                yaw_rad=math.pi / 2.0,
                linear_speed_mps=0.0,
                yaw_rate_rps=0.0,
            )
        )
        measurement.observe(
            guard_module.OdomSnapshot(
                monotonic_s=1.1,
                x_m=1.0,
                y_m=2.1,
                yaw_rad=-math.pi + 0.01,
                linear_speed_mps=0.02,
                yaw_rate_rps=0.01,
            )
        )
        measurement.observe(
            guard_module.OdomSnapshot(
                monotonic_s=1.2,
                x_m=1.01,
                y_m=2.2,
                yaw_rad=-math.pi + 0.02,
                linear_speed_mps=0.02,
                yaw_rate_rps=0.01,
            )
        )
        measurement.freeze()
        summary = measurement.summary()
        self.assertAlmostEqual(summary["forward_m"], 0.2)
        self.assertAlmostEqual(summary["lateral_m"], -0.01)
        self.assertAlmostEqual(
            summary["total_m"], 0.1 + math.hypot(0.01, 0.1)
        )
        expected_yaw = math.atan2(
            math.sin((-math.pi + 0.02) - math.pi / 2.0),
            math.cos((-math.pi + 0.02) - math.pi / 2.0),
        )
        self.assertAlmostEqual(summary["yaw_change_rad"], expected_yaw)

    def test_post_stop_requires_full_second_and_all_thresholds(self) -> None:
        reference = guard_module.OdomSnapshot(
            monotonic_s=10.0,
            x_m=1.0,
            y_m=2.0,
            yaw_rad=0.0,
            linear_speed_mps=0.0,
            yaw_rate_rps=0.0,
        )
        evidence = guard_module.PostStopStationarity(reference, 10.0)
        for index in range(10):
            evidence.observe(
                guard_module.OdomSnapshot(
                    monotonic_s=10.01 + index * 0.09,
                    x_m=1.001,
                    y_m=2.0,
                    yaw_rad=0.0,
                    linear_speed_mps=0.002,
                    yaw_rate_rps=0.003,
                )
            )
        self.assertFalse(evidence.summary(10.99)["passed"])
        self.assertTrue(evidence.summary(11.0)["passed"])
        evidence.invalidate_chassis()
        self.assertFalse(evidence.summary(11.1)["passed"])


if __name__ == "__main__":
    unittest.main()
