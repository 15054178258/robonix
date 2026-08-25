#!/usr/bin/env python3
"""Final fail-closed guard for the staged physical Nav2 profile.

This module is intentionally independent of the one-shot 10 cm commissioning
probe.  It never calls Unitree APIs directly: Nav2 publishes to
``/cmd_vel_guard_input``, this guard publishes a bounded command on the private
staged topic, and the separately guarded chassis adapter remains the only
process allowed to reach the SDK daemon.

The pure :class:`StagedNav2Guard` policy is importable without ROS so every
transition can be tested offline.  ROS imports and endpoint construction occur
only after exact, launcher-derived runtime claims have been validated.  Missing
or ambiguous claims therefore leave no velocity publisher behind.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import time
from typing import Iterable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis.staged_nav2_result import (  # noqa: E402
    MEASURED_RESULT_SCHEMA,
    POST_STOP_MAX_DRIFT_M,
    POST_STOP_MAX_LINEAR_MPS,
    POST_STOP_MAX_YAW_RPS,
    POST_STOP_MIN_ODOM_SAMPLES,
    POST_STOP_OBSERVATION_S,
    ResultError,
    read_private_json,
    result_paths,
    sha256_file,
    validate_action_result,
    validate_measured_result,
    write_private_json,
)


PROFILE = "workstation-staged-nav2-corrected-v1"
STAGE = "stage1"
GUARD_ACK = "I_APPROVE_GO2_STAGED_NAV2_MOTION"

NODE_NAME = "go2_staged_nav2_motion_guard"
EXPECTED_CONTROLLER_NODE = "/velocity_smoother"
EXPECTED_CHASSIS_NODE = "/go2_chassis_adapter"

INPUT_TOPIC = "/cmd_vel_guard_input"
OUTPUT_TOPIC = "/go2/staged_nav2/cmd_vel"
CANONICAL_COMMAND_TOPIC = "/cmd_vel"
ARM_SERVICE = "/go2_chassis/arm"
STATE_TOPIC = "/robonix/time_corrected/motion/sportmodestate"
ODOM_TOPIC = "/odom"
SCAN_TOPIC = "/scanner/scan"
MAP_TOPIC = "/map"
LOCALIZATION_TOPIC = "/robonix/map/pose"
MAP_LIFECYCLE_TOPIC = "/robonix/map/lifecycle"
GOAL_STATUS_TOPIC = "/navigate_to_pose/_action/status"
GOAL_CANCEL_SERVICE = "/navigate_to_pose/_action/cancel_goal"
CHASSIS_STATUS_TOPIC = "/go2_chassis/status"
GOAL_CLAIM_TOPIC = "/robonix/staged_nav2/goal_claim"
GOAL_CLAIM_SCHEMA = "robonix-go2-staged-nav2-goal-claim-v1"
EXPECTED_GOAL_DISPATCH_NODE = "/go2_staged_nav2_goal_dispatch"
STAGE1_GOAL_SOURCE = "operator_reviewed_short_goal"

SESSION_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{7,63}")
MAP_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{7,63}")
TARGET_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}")
SHA256_RE = re.compile(r"[0-9a-f]{64}")
GOAL_UUID_RE = re.compile(r"[0-9a-f]{32}")
ACTIVE_STATUSES = frozenset({"accepted", "executing"})
TERMINAL_SUCCESS_STATUSES = frozenset({"succeeded"})
TERMINAL_CANCEL_STATUSES = frozenset({"canceling", "canceled"})
TERMINAL_FAILURE_STATUSES = frozenset({"aborted", "unknown"})


class GuardError(ValueError):
    """An invalid runtime claim or guard input."""


class Phase(str, Enum):
    DISABLED = "disabled"
    IDLE = "idle"
    PENDING_ARM = "pending_arm"
    ARMED = "armed"
    COMPLETE = "complete"
    FAULT = "fault"


@dataclass(frozen=True)
class StageLimits:
    max_linear_x_mps: float = 0.30
    max_linear_y_mps: float = 0.0
    max_angular_z_rps: float = 0.40
    max_linear_accel_mps2: float = 0.30
    max_angular_accel_rps2: float = 0.80
    # Standard navigation is bounded by velocity/acceleration limits and the
    # independent command watchdog, not a commissioning trip timer.
    max_distance_m: float = 0.0
    max_duration_s: float = 0.0
    command_timeout_s: float = 0.20
    arm_response_timeout_s: float = 1.0
    zero_preamble_s: float = 0.60
    chassis_ready_timeout_s: float = 2.0


LIMITS = StageLimits()

# A static map is not expected to republish.  The ROS wrapper refreshes the map
# witness while the validated map remains loaded and its unique publisher is
# still present.  Every other witness is refreshed by live data or TF lookup.
FRESHNESS_LIMIT_S: Mapping[str, float] = {
    # Receipt freshness is intentionally looser than source freshness.  ROS
    # callbacks on the workstation can be delayed briefly by DDS/Wi-Fi and
    # process scheduling even while each delivered sample is current.  The
    # source-stamp limits below still reject old data, and the independent
    # command/SDK watchdogs still stop the chassis within 200/300 ms.
    "state": 1.0,
    "odom": 1.0,
    "tf": 1.0,
    # Scan availability is established before the guard starts.  During an
    # active goal, the official Nav2 obstacle layer owns observation timeout
    # and stop behavior; duplicating it here canceled valid goals during normal
    # wireless projection gaps.
    "scan": 1.50,
    "localization": 0.50,
    "goal_status": 0.50,
    "goal_claim": 0.75,
    "map": 0.75,
    "map_lifecycle": 0.75,
    "ownership": 0.75,
    # Adapter status is a 200 ms observation channel, not the motion
    # watchdog. Allow scheduler jitter while lower layers retain their
    # independent 200/300 ms command/disconnect stop deadlines.
    "chassis": 1.0,
}
SOURCE_STAMP_LIMIT_S: Mapping[str, float] = {
    # Delivered state must still have been produced recently; widening the
    # callback receipt window must never make an old sample look live.
    "state": 0.20,
    "odom": 0.20,
    "scan": 0.25,
    "localization": 0.50,
}
MAX_SOURCE_FUTURE_S = 0.05
UNVERIFIED_GOAL_TIMEOUT_S = 0.50
MAX_START_POSITION_ERROR_M = 0.05


@dataclass(frozen=True)
class ExpectedGoalClaim:
    session_id: str
    pair_id: str
    source: str
    target_id: str
    map_id: str
    generation: int
    x_m: float
    y_m: float
    yaw_rad: float
    start_x_m: float
    start_y_m: float
    evidence_sha256: str


@dataclass(frozen=True)
class RuntimeClaims:
    session_id: str
    stage: str
    allowed_modes: tuple[int, ...]
    allowed_state_markers: tuple[int, ...]
    expected_goal: ExpectedGoalClaim


@dataclass(frozen=True)
class AcceptedGoalClaim:
    goal_uuid: str
    expected: ExpectedGoalClaim


@dataclass(frozen=True)
class Decision:
    command: tuple[float, float, float] = (0.0, 0.0, 0.0)
    request_arm: bool = False
    request_disarm: bool = False
    request_cancel: bool = False
    reason: str = ""

    @property
    def is_zero(self) -> bool:
        return self.command == (0.0, 0.0, 0.0)


def validate_environment(environ: Mapping[str, str]) -> RuntimeClaims:
    """Validate launcher-derived claims before importing ROS.

    The chassis process consumes the private one-time permit.  This guard does
    not compete for that file; it accepts only short-lived, non-secret claims
    derived by the owning launcher after successful permit consumption.
    """

    exact = {
        "GO2_ALLOW_MOTION": "true",
        "GO2_MOTION_PROFILE": PROFILE,
        "GO2_STAGED_NAV2_STAGE": STAGE,
        "GO2_STAGED_NAV2_GUARD_ACK": GUARD_ACK,
    }
    for key, expected in exact.items():
        if environ.get(key) != expected:
            raise GuardError(f"{key} must exactly equal {expected!r}")
    session_id = environ.get("GO2_STAGED_NAV2_SESSION_ID", "")
    if SESSION_RE.fullmatch(session_id) is None:
        raise GuardError(
            "GO2_STAGED_NAV2_SESSION_ID must be 8-64 safe ASCII characters"
        )
    pair_id = environ.get("GO2_STAGED_NAV2_PAIR_ID", "")
    source = environ.get("GO2_STAGED_NAV2_GOAL_SOURCE", "")
    target_id = environ.get("GO2_STAGED_NAV2_TARGET_ID", "")
    map_id = environ.get("GO2_STAGED_NAV2_MAP_ID", "")
    evidence_sha256 = environ.get(
        "GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256", ""
    )
    if SESSION_RE.fullmatch(pair_id) is None:
        raise GuardError(
            "GO2_STAGED_NAV2_PAIR_ID must be 8-64 safe ASCII characters"
        )
    if source != STAGE1_GOAL_SOURCE:
        raise GuardError(
            f"GO2_STAGED_NAV2_GOAL_SOURCE must equal {STAGE1_GOAL_SOURCE!r}"
        )
    if TARGET_ID_RE.fullmatch(target_id) is None:
        raise GuardError(
            "GO2_STAGED_NAV2_TARGET_ID must be a 1-64 character safe identifier"
        )
    if MAP_ID_RE.fullmatch(map_id) is None:
        raise GuardError(
            "GO2_STAGED_NAV2_MAP_ID must be an 8-64 character safe identifier"
        )
    try:
        generation = int(environ.get("GO2_STAGED_NAV2_MAP_GENERATION", ""), 10)
    except ValueError as error:
        raise GuardError(
            "GO2_STAGED_NAV2_MAP_GENERATION must be a positive integer"
        ) from error
    if generation <= 0:
        raise GuardError(
            "GO2_STAGED_NAV2_MAP_GENERATION must be a positive integer"
        )

    def goal_coordinate(key: str) -> float:
        try:
            value = float(environ.get(key, ""))
        except ValueError as error:
            raise GuardError(f"{key} must be finite") from error
        if not math.isfinite(value):
            raise GuardError(f"{key} must be finite")
        return value

    goal_x = goal_coordinate("GO2_STAGED_NAV2_EXPECTED_GOAL_X")
    goal_y = goal_coordinate("GO2_STAGED_NAV2_EXPECTED_GOAL_Y")
    goal_yaw = goal_coordinate("GO2_STAGED_NAV2_EXPECTED_GOAL_YAW")
    start_x = goal_coordinate("GO2_STAGED_NAV2_EXPECTED_START_X")
    start_y = goal_coordinate("GO2_STAGED_NAV2_EXPECTED_START_Y")
    if not -math.pi <= goal_yaw <= math.pi:
        raise GuardError(
            "GO2_STAGED_NAV2_EXPECTED_GOAL_YAW must be within [-pi, pi]"
        )

    def decimal_list(
        key: str,
        *,
        minimum: int,
        maximum: int,
        allow_empty: bool,
    ) -> tuple[int, ...]:
        raw = environ.get(key)
        if raw is None:
            raise GuardError(f"{key} is required")
        raw = raw.strip()
        if not raw:
            if allow_empty:
                return ()
            raise GuardError(f"{key} must not be empty")
        tokens = [token.strip() for token in raw.replace(";", ",").split(",")]
        if any(not token or not token.isdecimal() for token in tokens):
            raise GuardError(f"{key} must be a decimal integer list")
        values = tuple(int(token, 10) for token in tokens)
        if (
            any(value < minimum or value > maximum for value in values)
            or len(values) != len(set(values))
        ):
            raise GuardError(
                f"{key} entries must be unique and in {minimum}..{maximum}"
            )
        return values

    allowed_modes = decimal_list(
        "GO2_ALLOWED_MODES",
        minimum=0,
        maximum=254,
        allow_empty=False,
    )
    allowed_state_markers = decimal_list(
        "GO2_ALLOWED_STATE_MARKERS",
        minimum=1,
        maximum=0xFFFFFFFF,
        allow_empty=True,
    )
    if SHA256_RE.fullmatch(evidence_sha256) is None:
        raise GuardError(
            "GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256 must be lowercase SHA-256"
        )
    expected = ExpectedGoalClaim(
        session_id=session_id,
        pair_id=pair_id,
        source=source,
        target_id=target_id,
        map_id=map_id,
        generation=generation,
        x_m=goal_x,
        y_m=goal_y,
        yaw_rad=goal_yaw,
        start_x_m=start_x,
        start_y_m=start_y,
        evidence_sha256=evidence_sha256,
    )
    return RuntimeClaims(
        session_id=session_id,
        stage=STAGE,
        allowed_modes=allowed_modes,
        allowed_state_markers=allowed_state_markers,
        expected_goal=expected,
    )


def validate_goal_claim(
    payload: object,
    expected: ExpectedGoalClaim,
) -> AcceptedGoalClaim:
    """Require the dispatcher's claim to exactly match permit-derived fields."""

    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except json.JSONDecodeError as error:
            raise GuardError("goal claim is not valid JSON") from error
    if not isinstance(payload, dict):
        raise GuardError("goal claim must be a JSON object")
    exact_keys = {
        "schema",
        "session_id",
        "pair_id",
        "source",
        "target_id",
        "map_id",
        "generation",
        "pose",
        "goal_evidence_sha256",
        "goal_uuid",
    }
    if set(payload) != exact_keys:
        raise GuardError("goal claim keys do not match the exact schema")
    pose = payload.get("pose")
    if not isinstance(pose, dict) or set(pose) != {"x", "y", "yaw"}:
        raise GuardError("goal claim pose keys do not match the exact schema")
    expected_values = {
        "schema": GOAL_CLAIM_SCHEMA,
        "session_id": expected.session_id,
        "pair_id": expected.pair_id,
        "source": expected.source,
        "target_id": expected.target_id,
        "map_id": expected.map_id,
        "generation": expected.generation,
        "goal_evidence_sha256": expected.evidence_sha256,
    }
    if any(payload.get(key) != value for key, value in expected_values.items()):
        raise GuardError("goal claim does not match permit-derived fields")
    try:
        claim_pose = (
            float(pose["x"]),
            float(pose["y"]),
            float(pose["yaw"]),
        )
    except (TypeError, ValueError) as error:
        raise GuardError("goal claim pose is not numeric") from error
    expected_pose = (expected.x_m, expected.y_m, expected.yaw_rad)
    if not all(
        math.isfinite(actual)
        and math.isclose(actual, wanted, rel_tol=0.0, abs_tol=1e-9)
        for actual, wanted in zip(claim_pose, expected_pose)
    ):
        raise GuardError("goal claim pose does not match permit-derived pose")
    goal_uuid = payload.get("goal_uuid")
    if not isinstance(goal_uuid, str) or GOAL_UUID_RE.fullmatch(goal_uuid) is None:
        raise GuardError("goal claim UUID must be 16-byte lowercase hex")
    return AcceptedGoalClaim(goal_uuid=goal_uuid, expected=expected)


def _finite_time(value: float, label: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise GuardError(f"{label} must be a finite non-negative time")
    return value


def _canonical_node_name(name: str, namespace: str = "/") -> str:
    name = str(name).strip("/")
    namespace = str(namespace or "/").strip("/")
    return "/" + "/".join(part for part in (namespace, name) if part)


class StagedNav2Guard:
    """Pure one-goal safety state machine for the stage-1 Nav2 envelope."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        stage: str = STAGE,
        limits: StageLimits = LIMITS,
        expected_goal: ExpectedGoalClaim | None = None,
        allowed_modes: Sequence[int] = (),
        allowed_state_markers: Sequence[int] = (),
    ) -> None:
        if stage != STAGE:
            raise GuardError(f"unsupported staged Nav2 stage: {stage!r}")
        if enabled and (expected_goal is None or not allowed_modes):
            raise GuardError(
                "enabled staged guard requires goal and audited mode claims"
            )
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 0 <= value <= 254
            for value in allowed_modes
        ):
            raise GuardError("allowed modes must be uint8 values below 255")
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 1 <= value <= 0xFFFFFFFF
            for value in allowed_state_markers
        ):
            raise GuardError(
                "allowed state markers must be non-zero uint32 values"
            )
        if (
            len(tuple(allowed_modes)) != len(set(allowed_modes))
            or len(tuple(allowed_state_markers))
            != len(set(allowed_state_markers))
        ):
            raise GuardError("audited state claims must not contain duplicates")
        self.limits = limits
        self.expected_goal = expected_goal
        self.allowed_modes = tuple(allowed_modes)
        self.allowed_state_markers = tuple(allowed_state_markers)
        self.phase = Phase.IDLE if enabled else Phase.DISABLED
        self.session_spent = False
        self.goal_id = ""
        self.fault_reason = ""
        self.terminal_reason = ""
        self._witnesses: dict[str, float | None] = {
            name: None for name in FRESHNESS_LIMIT_S
        }
        self._ownership_valid = False
        self._map_lifecycle: tuple[str, int] | None = None
        self._locked_map_lifecycle: tuple[str, int] | None = None
        self._accepted_goal_claim: AcceptedGoalClaim | None = None
        self._goal_claim_seen = False
        self._unverified_goal_id = ""
        self._unverified_goal_at: float | None = None
        self._chassis_guard_state = "UNKNOWN"
        self._daemon_armed = False
        self._odom_xy: tuple[float, float] | None = None
        self._localization_xy: tuple[float, float] | None = None
        self._localization_yaw: float | None = None
        self._last_distance_xy: tuple[float, float] | None = None
        self._distance_traveled_m = 0.0
        self._arm_requested_at: float | None = None
        self._arm_service_accepted = False
        self._armed_at: float | None = None
        self._last_command_at: float | None = None
        self._last_output_at: float | None = None
        self._last_output = (0.0, 0.0, 0.0)

    def _decision(
        self,
        reason: str,
        *,
        command: tuple[float, float, float] = (0.0, 0.0, 0.0),
        arm: bool = False,
        disarm: bool = False,
        cancel: bool = False,
    ) -> Decision:
        return Decision(
            command=command,
            request_arm=arm,
            request_disarm=disarm,
            request_cancel=cancel,
            reason=reason,
        )

    def _trip(self, reason: str) -> Decision:
        if self.phase is not Phase.FAULT:
            self.phase = Phase.FAULT
            self.fault_reason = reason
            self.terminal_reason = reason
        self.session_spent = True
        self._last_output = (0.0, 0.0, 0.0)
        return self._decision(
            self.fault_reason or reason,
            disarm=True,
            cancel=bool(self.goal_id or self._unverified_goal_id),
        )

    def _complete(self, reason: str, *, cancel: bool = False) -> Decision:
        self.phase = Phase.COMPLETE
        self.terminal_reason = reason
        self.session_spent = True
        self._last_output = (0.0, 0.0, 0.0)
        return self._decision(reason, disarm=True, cancel=cancel)

    def observe(
        self,
        name: str,
        now_s: float,
        *,
        source_age_s: float | None = None,
    ) -> Decision:
        """Refresh one health witness, rejecting stale/future source stamps."""

        if name not in self._witnesses:
            raise GuardError(f"unknown guard witness: {name}")
        now_s = _finite_time(now_s, "monotonic time")
        if source_age_s is not None:
            source_age_s = float(source_age_s)
            maximum = SOURCE_STAMP_LIMIT_S.get(name)
            invalid = (
                not math.isfinite(source_age_s)
                or source_age_s < -MAX_SOURCE_FUTURE_S
                or maximum is None
                or source_age_s > maximum
            )
            if invalid:
                self._witnesses[name] = None
                if name in {"scan", "localization"}:
                    return self._decision(f"{name}_delegated_to_nav2")
                if self.phase in (Phase.PENDING_ARM, Phase.ARMED):
                    return self._trip(f"{name}_source_stamp_invalid")
                return self._decision(f"{name}_source_stamp_invalid")
        self._witnesses[name] = now_s
        return self._decision(f"{name}_fresh")

    def observe_odom(
        self,
        now_s: float,
        x_m: float,
        y_m: float,
        *,
        source_age_s: float | None = None,
    ) -> Decision:
        if not all(math.isfinite(value) for value in (x_m, y_m)):
            return self._trip("odom_pose_non_finite")
        result = self.observe("odom", now_s, source_age_s=source_age_s)
        if self.phase is Phase.FAULT:
            return result
        self._odom_xy = (float(x_m), float(y_m))
        if self.phase is Phase.ARMED and self._last_distance_xy is not None:
            step_m = math.hypot(
                self._odom_xy[0] - self._last_distance_xy[0],
                self._odom_xy[1] - self._last_distance_xy[1],
            )
            self._last_distance_xy = self._odom_xy
            self._distance_traveled_m += step_m
            if (
                not math.isfinite(step_m)
                or not math.isfinite(self._distance_traveled_m)
                or (
                    self.limits.max_distance_m > 0.0
                    and self._distance_traveled_m
                    >= self.limits.max_distance_m
                )
            ):
                return self._trip("stage_distance_limit")
        return result

    def observe_state(
        self,
        now_s: float,
        *,
        mode: int,
        error_code: int,
        source_age_s: float | None = None,
    ) -> Decision:
        """Refresh only an exact audited firmware mode/marker witness."""

        valid = (
            isinstance(mode, int)
            and not isinstance(mode, bool)
            and mode in self.allowed_modes
            and isinstance(error_code, int)
            and not isinstance(error_code, bool)
            and (
                error_code == 0
                or error_code in self.allowed_state_markers
            )
        )
        if not valid:
            self._witnesses["state"] = None
            if self.phase in (Phase.PENDING_ARM, Phase.ARMED):
                return self._trip("state_mode_or_marker_mismatch")
            return self._decision("state_mode_or_marker_mismatch")
        return self.observe(
            "state", now_s, source_age_s=source_age_s
        )

    def observe_scan(
        self,
        now_s: float,
        *,
        frame_id: str,
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        ranges: Sequence[float],
        source_age_s: float | None = None,
    ) -> Decision:
        """Reject a fresh timestamp carrying an unusable laser payload."""

        try:
            numeric = (
                float(angle_min),
                float(angle_max),
                float(angle_increment),
                float(range_min),
                float(range_max),
            )
            range_values = tuple(float(value) for value in ranges)
        except (TypeError, ValueError):
            numeric = (math.nan,) * 5
            range_values = ()
        valid_ranges = tuple(
            value
            for value in range_values
            if math.isfinite(value)
            and numeric[3] <= value <= numeric[4]
        )
        valid = (
            bool(str(frame_id).strip())
            and all(math.isfinite(value) for value in numeric)
            and numeric[2] > 0.0
            and numeric[1] > numeric[0]
            and 0.0 <= numeric[3] < numeric[4]
            and bool(range_values)
            and bool(valid_ranges)
        )
        if not valid:
            self._witnesses["scan"] = None
            return self._decision("scan_delegated_to_nav2")
        return self.observe("scan", now_s, source_age_s=source_age_s)

    def observe_localization(
        self,
        now_s: float,
        *,
        frame_id: str,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        source_age_s: float | None = None,
    ) -> Decision:
        """Track a finite map pose and enforce the permit start before arming."""

        valid = (
            str(frame_id) == "map"
            and all(
                math.isfinite(float(value))
                for value in (x_m, y_m, yaw_rad)
            )
            and -math.pi <= float(yaw_rad) <= math.pi
        )
        if not valid:
            self._witnesses["localization"] = None
            return self._decision("localization_delegated_to_nav2")
        self._localization_xy = (float(x_m), float(y_m))
        self._localization_yaw = float(yaw_rad)
        result = self.observe(
            "localization", now_s, source_age_s=source_age_s
        )
        if (
            self.phase is Phase.PENDING_ARM
            and not self._start_position_matches()
        ):
            return self._trip("localization_left_permitted_start_before_arm")
        return result

    def _start_position_matches(self) -> bool:
        if self.expected_goal is None or self._localization_xy is None:
            return False
        error_m = math.hypot(
            self._localization_xy[0] - self.expected_goal.start_x_m,
            self._localization_xy[1] - self.expected_goal.start_y_m,
        )
        return (
            math.isfinite(error_m)
            and error_m <= MAX_START_POSITION_ERROR_M
        )

    def observe_ownership(
        self,
        now_s: float,
        *,
        controller_publishers: Sequence[str],
        output_publishers: Sequence[str],
        chassis_subscribers: Sequence[str],
        canonical_publishers: int,
    ) -> Decision:
        """Require the exact Nav2 -> guard -> chassis graph with no bypass."""

        now_s = _finite_time(now_s, "monotonic time")
        valid = (
            tuple(controller_publishers) == (EXPECTED_CONTROLLER_NODE,)
            and tuple(output_publishers) == (f"/{NODE_NAME}",)
            and tuple(chassis_subscribers) == (EXPECTED_CHASSIS_NODE,)
            and canonical_publishers == 0
        )
        self._ownership_valid = valid
        if not valid:
            self._witnesses["ownership"] = None
            if self.phase in (Phase.PENDING_ARM, Phase.ARMED):
                return self._trip("command_ownership_mismatch")
            return self._decision("command_ownership_mismatch")
        self._witnesses["ownership"] = now_s
        return self._decision("command_ownership_fresh")

    def observe_map_lifecycle(
        self,
        now_s: float,
        *,
        map_id: str,
        mode: str,
        generation: int,
    ) -> Decision:
        """Bind motion to one named localization map and generation."""

        now_s = _finite_time(now_s, "monotonic time")
        map_id = str(map_id).strip()
        mode = str(mode).strip().lower()
        if (
            not map_id
            or mode != "localization"
            or not isinstance(generation, int)
            or isinstance(generation, bool)
            or generation <= 0
        ):
            self._witnesses["map_lifecycle"] = None
            if self.phase in (Phase.PENDING_ARM, Phase.ARMED):
                return self._trip("map_lifecycle_invalid")
            return self._decision("map_lifecycle_invalid")
        current = (map_id, generation)
        if self.expected_goal is not None and current != (
            self.expected_goal.map_id,
            self.expected_goal.generation,
        ):
            self._witnesses["map_lifecycle"] = None
            if self.phase in (Phase.PENDING_ARM, Phase.ARMED):
                return self._trip("map_lifecycle_does_not_match_goal_permit")
            return self._decision("map_lifecycle_does_not_match_goal_permit")
        if (
            self._locked_map_lifecycle is not None
            and current != self._locked_map_lifecycle
        ):
            return self._trip("map_lifecycle_changed")
        self._map_lifecycle = current
        self._witnesses["map_lifecycle"] = now_s
        return self._decision("map_lifecycle_fresh")

    def observe_chassis_status(
        self,
        now_s: float,
        *,
        guard_state: str,
        daemon_armed: object,
        motion_configured: object,
        motion_profile: str,
        odom_source: str,
    ) -> Decision:
        """Track the adapter's measured state, not merely SetBool response."""

        now_s = _finite_time(now_s, "monotonic time")
        guard_state = str(guard_state).strip().upper()
        profile_exact = (
            motion_configured is True
            and str(motion_profile) == PROFILE
            and str(odom_source) == "external_verified"
        )
        if (
            guard_state
            not in {"DISARMED", "PREPARING", "ARMED", "FAULT"}
            or not isinstance(daemon_armed, bool)
            or not profile_exact
        ):
            self._witnesses["chassis"] = None
            if self.phase in (Phase.PENDING_ARM, Phase.ARMED):
                return self._trip("chassis_status_or_profile_invalid")
            return self._decision("chassis_status_or_profile_invalid")
        self._chassis_guard_state = guard_state
        self._daemon_armed = daemon_armed
        self._witnesses["chassis"] = now_s
        if guard_state == "FAULT":
            return self._trip("chassis_fault")
        if self.phase is Phase.ARMED:
            if guard_state != "ARMED" or not daemon_armed:
                return self._trip("chassis_left_armed_state")
        elif self.phase is Phase.PENDING_ARM:
            if (
                self._arm_service_accepted
                and self._arm_requested_at is not None
                and now_s - self._arm_requested_at >= self.limits.zero_preamble_s
                and guard_state == "ARMED"
                and daemon_armed
            ):
                if not self._start_position_matches():
                    return self._trip(
                        "chassis_armed_outside_permitted_start_position"
                    )
                self.phase = Phase.ARMED
                self._armed_at = now_s
                self._last_command_at = now_s
                self._last_output_at = now_s
                self._last_output = (0.0, 0.0, 0.0)
                return self._decision("chassis_measured_armed_after_zero_hold")
            if daemon_armed and guard_state != "ARMED":
                return self._trip("daemon_armed_without_chassis_armed")
        return self._decision("chassis_status_fresh")

    def observe_goal_claim(
        self,
        now_s: float,
        payload: object,
        *,
        publisher_nodes: Sequence[str],
    ) -> Decision:
        """Accept one permit-bound claim and lock its exact goal UUID.

        DDS can deliver the transient-local payload before its graph endpoint
        becomes visible.  An empty graph snapshot is therefore accepted; any
        visible ambiguous or unexpected publisher set is still rejected.
        """

        now_s = _finite_time(now_s, "monotonic time")
        if publisher_nodes and tuple(publisher_nodes) != (
            EXPECTED_GOAL_DISPATCH_NODE,
        ):
            return self._trip("goal_claim_owner_missing_or_ambiguous")
        if self.expected_goal is None:
            return self._trip("goal_permit_claims_missing")
        if self._goal_claim_seen:
            return self._trip("second_goal_claim")
        self._goal_claim_seen = True
        try:
            claim = validate_goal_claim(payload, self.expected_goal)
        except GuardError:
            return self._trip("goal_claim_mismatch")
        self._accepted_goal_claim = claim
        self._witnesses["goal_claim"] = now_s
        if (
            self._unverified_goal_id
            and claim.goal_uuid != self._unverified_goal_id
        ):
            return self._trip("goal_claim_uuid_mismatch")
        if self._unverified_goal_id:
            return self.begin_goal(claim.goal_uuid, now_s)
        return self._decision("goal_claim_accepted")

    def refresh_goal_claim(
        self,
        now_s: float,
        *,
        publisher_nodes: Sequence[str],
    ) -> Decision:
        """Keep an already validated static claim bound to this session.

        Publisher graph liveness is not a navigation heartbeat.  Goal status,
        command ownership, the exact UUID, and the live chassis/sensor paths
        provide the runtime checks after the claim has been accepted.
        """

        now_s = _finite_time(now_s, "monotonic time")
        del publisher_nodes
        if self._accepted_goal_claim is None:
            self._witnesses["goal_claim"] = None
            return self._decision("goal_claim_missing")
        self._witnesses["goal_claim"] = now_s
        return self._decision("goal_claim_bound_static")

    def _stale_reason(self, now_s: float) -> str | None:
        for name, maximum in FRESHNESS_LIMIT_S.items():
            # ROS 2 action status is transition/event state, not a sensor
            # heartbeat.  Goal identity and terminal status are still checked
            # whenever an event arrives; liveness is provided independently
            # by command, state, odom, TF, scan and chassis watchdogs.
            if name in {"goal_status", "goal_claim"}:
                continue
            if (
                name in {"scan", "localization"}
                and self.phase in {Phase.PENDING_ARM, Phase.ARMED}
            ):
                continue
            observed = self._witnesses[name]
            if observed is None:
                return f"{name}_missing"
            age = now_s - observed
            if not math.isfinite(age) or age < 0.0 or age > maximum:
                return f"{name}_stale"
        if not self._ownership_valid:
            return "command_ownership_mismatch"
        return None

    def _hold_zero_for_command_gap(self, now_s: float) -> None:
        """Expire controller output without canceling the Nav2 goal.

        Nav2 may pause output while planning or recovering.  The timeout still
        fails closed: publish zero, forget the previous command, and ramp any
        resumed command stream from zero.  The adapter and SDK watchdogs also
        keep their independent chassis-stop behavior.
        """

        self._last_command_at = None
        self._last_output_at = now_s
        self._last_output = (0.0, 0.0, 0.0)

    def begin_goal(self, goal_id: str, now_s: float) -> Decision:
        now_s = _finite_time(now_s, "monotonic time")
        if self.phase is Phase.DISABLED:
            return self._decision("profile_disabled", disarm=True, cancel=True)
        if self.phase is Phase.FAULT:
            return self._decision(
                self.fault_reason or "fault_latched",
                disarm=True,
                cancel=True,
            )
        if self.phase is not Phase.IDLE or self.session_spent:
            return self._trip("staged_session_is_one_goal_only")
        if not goal_id or len(goal_id) > 128:
            return self._trip("goal_id_invalid")
        # The caller reached this method because Nav2 already reported an
        # active goal.  Record its identity before checking readiness so any
        # failed precondition also requests cancellation of that exact goal.
        self.goal_id = goal_id
        if self._accepted_goal_claim is None:
            return self._trip("goal_claim_missing")
        if goal_id != self._accepted_goal_claim.goal_uuid:
            return self._trip("active_goal_not_bound_to_claim")
        # A Nav2 goal-status event and the independent sensor/status callbacks
        # are not phase locked.  Binding a valid goal during a short callback
        # gap must therefore hold zero and wait, rather than consume the
        # one-goal session before the chassis has even been asked to arm.
        self._unverified_goal_id = ""
        self._unverified_goal_at = None
        return self._start_bound_goal_if_ready(now_s)

    def _start_bound_goal_if_ready(self, now_s: float) -> Decision:
        """Arm a bound goal only after all live witnesses are fresh.

        Waiting is deliberately possible only while IDLE: no velocity is
        forwarded and the chassis remains measured DISARMED.  Once arming has
        started, critical stale witnesses still fail closed; a controller gap
        holds zero so official Nav2 planning/recovery can continue.
        """

        now_s = _finite_time(now_s, "monotonic time")
        if self.phase is not Phase.IDLE or not self.goal_id:
            return self._trip("bound_goal_start_state_invalid")
        stale = self._stale_reason(now_s)
        if stale is not None:
            return self._decision(f"goal_waiting_for_{stale}")
        if self._odom_xy is None:
            return self._trip("goal_started_without_odom_pose")
        if not self._start_position_matches():
            return self._trip(
                "goal_started_outside_permitted_start_position"
            )
        if self._map_lifecycle is None:
            return self._trip("goal_started_without_map_lifecycle")
        if self._chassis_guard_state != "DISARMED" or self._daemon_armed:
            return self._trip("goal_started_while_chassis_not_disarmed")
        self.session_spent = True
        self.phase = Phase.PENDING_ARM
        self._arm_requested_at = now_s
        self._arm_service_accepted = False
        self._last_distance_xy = self._odom_xy
        self._distance_traveled_m = 0.0
        self._locked_map_lifecycle = self._map_lifecycle
        self._last_command_at = None
        self._last_output_at = now_s
        self._last_output = (0.0, 0.0, 0.0)
        return self._decision("goal_active_request_arm", arm=True)

    def observe_goal_statuses(
        self,
        now_s: float,
        statuses: Iterable[tuple[str, str]],
    ) -> Decision:
        now_s = _finite_time(now_s, "monotonic time")
        normalized = [(str(goal), str(status).lower()) for goal, status in statuses]
        self.observe("goal_status", now_s)
        active = [
            (goal, status)
            for goal, status in normalized
            if status in ACTIVE_STATUSES
        ]
        canceling = [
            (goal, status)
            for goal, status in normalized
            if status in TERMINAL_CANCEL_STATUSES
        ]
        if len(active) > 1:
            return self._trip("multiple_active_nav2_goals")
        if self.goal_id:
            current = [
                status for goal, status in normalized if goal == self.goal_id
            ]
            if len(current) != 1:
                return self._trip("bound_nav2_goal_status_missing_or_ambiguous")
            if any(status in TERMINAL_FAILURE_STATUSES for status in current):
                return self._trip("nav2_goal_failed")
            if any(status in TERMINAL_CANCEL_STATUSES for status in current):
                return self._complete("nav2_goal_canceled", cancel=True)
            if any(status in TERMINAL_SUCCESS_STATUSES for status in current):
                return self._complete("nav2_goal_succeeded")
            if active and active[0][0] != self.goal_id:
                return self._trip("active_goal_identity_changed")
            return self._decision("active_goal_status_fresh")
        if canceling:
            return self._complete("nav2_cancel_without_arm", cancel=True)
        if active:
            goal_id = active[0][0]
            if self._accepted_goal_claim is None:
                if (
                    self._unverified_goal_id
                    and self._unverified_goal_id != goal_id
                ):
                    return self._trip("second_unverified_active_goal")
                if not self._unverified_goal_id:
                    self._unverified_goal_id = goal_id
                    self._unverified_goal_at = now_s
                return self._decision("active_goal_waiting_for_exact_claim")
            return self.begin_goal(goal_id, now_s)
        return self._decision("no_active_goal")

    def confirm_arm(self, success: bool, now_s: float) -> Decision:
        now_s = _finite_time(now_s, "monotonic time")
        if self.phase is not Phase.PENDING_ARM:
            return self._trip("unexpected_arm_response")
        if not success:
            return self._trip("chassis_arm_rejected")
        stale = self._stale_reason(now_s)
        if stale is not None:
            return self._trip(f"arm_completed_with_{stale}")
        if not self._start_position_matches():
            return self._trip(
                "arm_completed_outside_permitted_start_position"
            )
        self._arm_service_accepted = True
        return self._decision("chassis_prepare_acknowledged_zero_hold")

    @staticmethod
    def _slew(target: float, previous: float, delta: float) -> float:
        return min(max(target, previous - delta), previous + delta)

    def command(
        self,
        now_s: float,
        linear_x_mps: float,
        linear_y_mps: float,
        angular_z_rps: float,
    ) -> Decision:
        now_s = _finite_time(now_s, "monotonic time")
        values = (linear_x_mps, linear_y_mps, angular_z_rps)
        if not all(math.isfinite(value) for value in values):
            return self._trip("command_non_finite")
        if self.phase is not Phase.ARMED:
            return self._decision(
                "command_while_not_armed",
                disarm=self.phase in (Phase.FAULT, Phase.COMPLETE),
                cancel=self.phase is Phase.FAULT,
            )
        stale = self._stale_reason(now_s)
        if stale is not None:
            return self._trip(stale)
        if linear_x_mps < 0.0 or linear_x_mps > self.limits.max_linear_x_mps:
            return self._trip("linear_x_outside_stage_envelope")
        if abs(linear_y_mps) > self.limits.max_linear_y_mps:
            return self._trip("linear_y_outside_stage_envelope")
        if abs(angular_z_rps) > self.limits.max_angular_z_rps:
            return self._trip("angular_z_outside_stage_envelope")
        if self._armed_at is None:
            return self._trip("stage_duration_limit")

        if self._last_command_at is not None:
            age = now_s - self._last_command_at
            if not math.isfinite(age) or age < 0.0:
                return self._trip("command_clock_invalid")
            if age > self.limits.command_timeout_s:
                self._hold_zero_for_command_gap(now_s)

        previous_at = self._last_output_at
        if previous_at is None:
            return self._trip("output_clock_missing")
        delta_s = now_s - previous_at
        if not math.isfinite(delta_s) or delta_s < 0.0:
            return self._trip("output_clock_regressed")
        output = (
            self._slew(
                linear_x_mps,
                self._last_output[0],
                self.limits.max_linear_accel_mps2 * delta_s,
            ),
            0.0,
            self._slew(
                angular_z_rps,
                self._last_output[2],
                self.limits.max_angular_accel_rps2 * delta_s,
            ),
        )
        self._last_command_at = now_s
        self._last_output_at = now_s
        self._last_output = output
        return self._decision("command_forwarded", command=output)

    def tick(self, now_s: float) -> Decision:
        now_s = _finite_time(now_s, "monotonic time")
        if self.phase is Phase.FAULT:
            return self._decision(
                self.fault_reason or "fault_latched",
                disarm=True,
                cancel=bool(self.goal_id or self._unverified_goal_id),
            )
        if self.phase is Phase.COMPLETE:
            return self._decision("session_complete", disarm=True)
        if self.phase is Phase.IDLE and self._unverified_goal_id:
            if (
                self._unverified_goal_at is None
                or now_s - self._unverified_goal_at
                > UNVERIFIED_GOAL_TIMEOUT_S
            ):
                self.goal_id = self._unverified_goal_id
                return self._trip("active_goal_claim_timeout")
            return self._decision("waiting_for_exact_goal_claim")
        if self.phase is Phase.IDLE and self.goal_id:
            return self._start_bound_goal_if_ready(now_s)
        if self.phase is Phase.PENDING_ARM:
            if self._arm_requested_at is None:
                return self._trip("chassis_arm_clock_missing")
            elapsed_s = now_s - self._arm_requested_at
            if (
                not self._arm_service_accepted
                and elapsed_s > self.limits.arm_response_timeout_s
            ):
                return self._trip("chassis_arm_response_timeout")
            if (
                self._arm_service_accepted
                and elapsed_s > self.limits.chassis_ready_timeout_s
            ):
                return self._trip("chassis_measured_arm_timeout")
            stale = self._stale_reason(now_s)
            return self._trip(stale) if stale else self._decision("pending_arm")
        if self.phase is Phase.ARMED:
            stale = self._stale_reason(now_s)
            if stale is not None:
                return self._trip(stale)
            if self._armed_at is None:
                return self._trip("stage_duration_limit")
            if self._last_command_at is None:
                self._hold_zero_for_command_gap(now_s)
                return self._decision("armed_waiting_for_controller_command")
            command_age = now_s - self._last_command_at
            if not math.isfinite(command_age) or command_age < 0.0:
                return self._trip("command_clock_invalid")
            if command_age > self.limits.command_timeout_s:
                self._hold_zero_for_command_gap(now_s)
                return self._decision("command_timeout_zero_hold")
            return self._decision("armed_healthy")
        return self._decision("idle_zero")

    def disconnect(self, reason: str = "disconnect") -> Decision:
        return self._trip(reason)

    def shutdown(self) -> Decision:
        return self._trip("shutdown")


def _stamp_age_s(node: object, stamp: object) -> float:
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    now_ns = int(node.get_clock().now().nanoseconds)
    return (now_ns - stamp_ns) / 1_000_000_000.0


def _yaw_from_quaternion(quaternion: object) -> float:
    try:
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
    except (AttributeError, TypeError, ValueError):
        return math.nan
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        return math.nan
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or not 0.5 <= norm <= 1.5:
        return math.nan
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class OdomSnapshot:
    """One finite odometry sample used by the terminal evidence recorder."""

    monotonic_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    linear_speed_mps: float
    yaw_rate_rps: float

    def __post_init__(self) -> None:
        values = (
            self.monotonic_s,
            self.x_m,
            self.y_m,
            self.yaw_rad,
            self.linear_speed_mps,
            self.yaw_rate_rps,
        )
        if (
            not all(math.isfinite(float(value)) for value in values)
            or self.monotonic_s < 0.0
            or not -math.pi <= self.yaw_rad <= math.pi
            or self.linear_speed_mps < 0.0
            or self.yaw_rate_rps < 0.0
        ):
            raise GuardError("odometry evidence sample is invalid")


class StageOdomMeasurement:
    """Accumulate one armed stage in odom and freeze it before shutdown."""

    def __init__(self) -> None:
        self.start_snapshot: OdomSnapshot | None = None
        self.end_snapshot: OdomSnapshot | None = None
        self._last_snapshot: OdomSnapshot | None = None
        self.total_m = 0.0
        self.samples = 0
        self.frozen = False

    def start(self, sample: OdomSnapshot) -> None:
        if self.start_snapshot is not None or self.frozen:
            raise GuardError("stage odometry measurement already started")
        self.start_snapshot = sample
        self.end_snapshot = sample
        self._last_snapshot = sample
        self.samples = 1

    def observe(self, sample: OdomSnapshot) -> None:
        if self.start_snapshot is None or self.frozen:
            return
        if (
            self._last_snapshot is None
            or sample.monotonic_s < self._last_snapshot.monotonic_s
        ):
            raise GuardError("stage odometry sample clock regressed")
        step_m = math.hypot(
            sample.x_m - self._last_snapshot.x_m,
            sample.y_m - self._last_snapshot.y_m,
        )
        if not math.isfinite(step_m):
            raise GuardError("stage odometry path is non-finite")
        self.total_m += step_m
        if not math.isfinite(self.total_m):
            raise GuardError("stage odometry total path is non-finite")
        self.end_snapshot = sample
        self._last_snapshot = sample
        self.samples += 1

    def freeze(self) -> None:
        self.frozen = True

    def summary(self) -> dict[str, object]:
        start = self.start_snapshot
        end = self.end_snapshot
        if start is None or end is None or self.samples < 1:
            raise GuardError("stage odometry measurement is incomplete")
        delta_x = end.x_m - start.x_m
        delta_y = end.y_m - start.y_m
        return {
            "frame_id": "odom",
            "start_pose": {
                "x": start.x_m,
                "y": start.y_m,
                "yaw": start.yaw_rad,
            },
            "end_pose": {
                "x": end.x_m,
                "y": end.y_m,
                "yaw": end.yaw_rad,
            },
            "forward_m": (
                delta_x * math.cos(start.yaw_rad)
                + delta_y * math.sin(start.yaw_rad)
            ),
            "total_m": self.total_m,
            "lateral_m": (
                -delta_x * math.sin(start.yaw_rad)
                + delta_y * math.cos(start.yaw_rad)
            ),
            "yaw_change_rad": _wrapped_angle(
                end.yaw_rad - start.yaw_rad
            ),
        }


class PostStopStationarity:
    """Measure a continuous stationary interval after measured disarm."""

    def __init__(
        self,
        reference: OdomSnapshot,
        started_monotonic_s: float,
    ) -> None:
        self.reference = reference
        self.started_monotonic_s = _finite_time(
            started_monotonic_s, "post-stop start time"
        )
        self.odom_samples = 0
        self.max_drift_m = 0.0
        self.max_linear_speed_mps = 0.0
        self.max_yaw_rate_rps = 0.0
        self.chassis_remained_disarmed = True

    def observe(self, sample: OdomSnapshot) -> None:
        if sample.monotonic_s < self.started_monotonic_s:
            return
        self.odom_samples += 1
        self.max_drift_m = max(
            self.max_drift_m,
            math.hypot(
                sample.x_m - self.reference.x_m,
                sample.y_m - self.reference.y_m,
            ),
        )
        self.max_linear_speed_mps = max(
            self.max_linear_speed_mps,
            sample.linear_speed_mps,
        )
        self.max_yaw_rate_rps = max(
            self.max_yaw_rate_rps,
            sample.yaw_rate_rps,
        )

    def invalidate_chassis(self) -> None:
        self.chassis_remained_disarmed = False

    def summary(self, finished_monotonic_s: float) -> dict[str, object]:
        observation_s = (
            _finite_time(finished_monotonic_s, "post-stop finish time")
            - self.started_monotonic_s
        )
        passed = (
            observation_s >= POST_STOP_OBSERVATION_S
            and self.odom_samples >= POST_STOP_MIN_ODOM_SAMPLES
            and self.max_drift_m <= POST_STOP_MAX_DRIFT_M
            and self.max_linear_speed_mps <= POST_STOP_MAX_LINEAR_MPS
            and self.max_yaw_rate_rps <= POST_STOP_MAX_YAW_RPS
            and self.chassis_remained_disarmed
        )
        return {
            "observation_s": observation_s,
            "odom_samples": self.odom_samples,
            "max_drift_m": self.max_drift_m,
            "max_linear_speed_mps": self.max_linear_speed_mps,
            "max_yaw_rate_rps": self.max_yaw_rate_rps,
            "limits": {
                "max_drift_m": POST_STOP_MAX_DRIFT_M,
                "max_linear_speed_mps": POST_STOP_MAX_LINEAR_MPS,
                "max_yaw_rate_rps": POST_STOP_MAX_YAW_RPS,
                "min_odom_samples": POST_STOP_MIN_ODOM_SAMPLES,
            },
            "passed": passed,
        }


def _run_ros(
    claims: RuntimeClaims,
    action_result_path: Path,
    measured_result_path: Path,
    environ: Mapping[str, str],
) -> int:
    """Run the ROS wrapper.  The pure policy above remains the authority."""

    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from map.msg import MapLifecycle
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_ros import Buffer, TransformException, TransformListener
    from unitree_go.msg import SportModeState

    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    old_sigint = signal.signal(signal.SIGINT, request_stop)
    old_sigterm = signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)

    status_names = {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }

    class GuardNode(Node):
        def __init__(self) -> None:
            super().__init__(NODE_NAME)
            self.policy = StagedNav2Guard(
                enabled=True,
                stage=claims.stage,
                expected_goal=claims.expected_goal,
                allowed_modes=claims.allowed_modes,
                allowed_state_markers=claims.allowed_state_markers,
            )
            self.map_loaded = False
            self.map_lifecycle: tuple[str, str, int] | None = None
            self.last_graph_check_s = -math.inf
            self.arm_future = None
            self.disarm_future = None
            self.cancel_future = None
            self.active_goal_raw = None
            self.fault_logged = False
            self.result_started_unix_ns = time.time_ns()
            self.latest_odom: OdomSnapshot | None = None
            self.measurement = StageOdomMeasurement()
            self.post_stop: PostStopStationarity | None = None
            self.stop_evidence: dict[str, object] = {
                "cancel": {
                    "required": False,
                    "requested": False,
                    "confirmed": False,
                },
                "zero": {
                    "published_count": 0,
                    "confirmed_zero": False,
                },
                "disarm": {
                    "requested": False,
                    "service_success": False,
                    "measured_disarmed": False,
                },
                "post_stop": {
                    "observation_s": 0.0,
                    "odom_samples": 0,
                    "max_drift_m": 0.0,
                    "max_linear_speed_mps": 0.0,
                    "max_yaw_rate_rps": 0.0,
                    "limits": {
                        "max_drift_m": POST_STOP_MAX_DRIFT_M,
                        "max_linear_speed_mps": POST_STOP_MAX_LINEAR_MPS,
                        "max_yaw_rate_rps": POST_STOP_MAX_YAW_RPS,
                        "min_odom_samples": POST_STOP_MIN_ODOM_SAMPLES,
                    },
                    "passed": False,
                },
            }

            reliable = QoSProfile(depth=10)
            reliable.reliability = ReliabilityPolicy.RELIABLE
            volatile_best_effort = QoSProfile(depth=20)
            volatile_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT
            map_qos = QoSProfile(depth=1)
            map_qos.reliability = ReliabilityPolicy.RELIABLE
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

            self.command_pub = self.create_publisher(Twist, OUTPUT_TOPIC, reliable)
            self.create_subscription(
                Twist, INPUT_TOPIC, self._on_command, reliable
            )
            self.create_subscription(
                Odometry, ODOM_TOPIC, self._on_odom, volatile_best_effort
            )
            self.create_subscription(
                LaserScan, SCAN_TOPIC, self._on_scan, volatile_best_effort
            )
            self.create_subscription(
                SportModeState, STATE_TOPIC, self._on_state, volatile_best_effort
            )
            self.create_subscription(
                OccupancyGrid, MAP_TOPIC, self._on_map, map_qos
            )
            self.create_subscription(
                MapLifecycle,
                MAP_LIFECYCLE_TOPIC,
                self._on_map_lifecycle,
                map_qos,
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                LOCALIZATION_TOPIC,
                self._on_localization,
                reliable,
            )
            self.create_subscription(
                GoalStatusArray,
                GOAL_STATUS_TOPIC,
                self._on_goal_status,
                reliable,
            )
            self.create_subscription(
                String,
                CHASSIS_STATUS_TOPIC,
                self._on_chassis_status,
                reliable,
            )
            claim_qos = QoSProfile(depth=1)
            claim_qos.reliability = ReliabilityPolicy.RELIABLE
            claim_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                String,
                GOAL_CLAIM_TOPIC,
                self._on_goal_claim,
                claim_qos,
            )
            self.arm_client = self.create_client(SetBool, ARM_SERVICE)
            self.cancel_client = self.create_client(
                CancelGoal, GOAL_CANCEL_SERVICE
            )
            self.tf_buffer = Buffer(cache_time=Duration(seconds=1.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(0.05, self._on_timer)
            self.get_logger().info(
                f"staged Nav2 guard ready session={claims.session_id} "
                f"stage={claims.stage}; output={OUTPUT_TOPIC}"
            )

        def _now_s(self) -> float:
            return time.monotonic()

        def _zero(self) -> Twist:
            return Twist()

        def _publish(self, decision: Decision) -> None:
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = decision.command
            self.command_pub.publish(message)
            if decision.request_cancel:
                self._request_cancel()
            if decision.request_arm:
                self._request_arm()
            if decision.request_disarm:
                self._request_disarm()
            if self.policy.phase is Phase.FAULT and not self.fault_logged:
                self.fault_logged = True
                self.get_logger().error(
                    f"staged Nav2 guard fault latched: {self.policy.fault_reason}"
                )

        def _request_arm(self) -> None:
            if self.arm_future is not None and not self.arm_future.done():
                return
            if not self.arm_client.service_is_ready():
                self._publish(
                    self.policy.disconnect("chassis_arm_service_unavailable")
                )
                return
            request = SetBool.Request()
            request.data = True
            self.arm_future = self.arm_client.call_async(request)

            def done(future: object) -> None:
                try:
                    response = future.result()
                    success = bool(response and response.success)
                except Exception:
                    success = False
                self._publish(self.policy.confirm_arm(success, self._now_s()))

            self.arm_future.add_done_callback(done)

        def _request_disarm(self) -> bool:
            if self.disarm_future is not None:
                return True
            if not self.arm_client.service_is_ready():
                return False
            request = SetBool.Request()
            request.data = False
            self.disarm_future = self.arm_client.call_async(request)
            return True

        def _request_cancel(self) -> bool:
            if self.cancel_future is not None:
                return not self.cancel_future.cancelled()
            if (
                self.active_goal_raw is None
                or not self.cancel_client.service_is_ready()
            ):
                return False
            request = CancelGoal.Request()
            request.goal_info.goal_id.uuid = self.active_goal_raw
            self.cancel_future = self.cancel_client.call_async(request)
            return True

        def _on_command(self, message: Twist) -> None:
            self._publish(
                self.policy.command(
                    self._now_s(),
                    float(message.linear.x),
                    float(message.linear.y),
                    float(message.angular.z),
                )
            )

        def _on_odom(self, message: Odometry) -> None:
            pose = message.pose.pose.position
            yaw_rad = _yaw_from_quaternion(message.pose.pose.orientation)
            linear = message.twist.twist.linear
            angular = message.twist.twist.angular
            try:
                sample = OdomSnapshot(
                    monotonic_s=self._now_s(),
                    x_m=float(pose.x),
                    y_m=float(pose.y),
                    yaw_rad=yaw_rad,
                    linear_speed_mps=math.hypot(
                        float(linear.x), float(linear.y)
                    ),
                    yaw_rate_rps=abs(float(angular.z)),
                )
                self.latest_odom = sample
                self.measurement.observe(sample)
                if self.post_stop is not None:
                    self.post_stop.observe(sample)
            except (GuardError, TypeError, ValueError):
                self._publish(
                    self.policy.disconnect("odom_measurement_payload_invalid")
                )
                return
            decision = self.policy.observe_odom(
                sample.monotonic_s,
                float(pose.x),
                float(pose.y),
                source_age_s=_stamp_age_s(self, message.header.stamp),
            )
            if self.policy.phase is Phase.FAULT:
                self._publish(decision)

        def _on_scan(self, message: LaserScan) -> None:
            decision = self.policy.observe_scan(
                self._now_s(),
                frame_id=str(message.header.frame_id),
                angle_min=float(message.angle_min),
                angle_max=float(message.angle_max),
                angle_increment=float(message.angle_increment),
                range_min=float(message.range_min),
                range_max=float(message.range_max),
                ranges=message.ranges,
                source_age_s=_stamp_age_s(self, message.header.stamp),
            )
            if self.policy.phase is Phase.FAULT:
                self._publish(decision)

        def _on_state(self, message: SportModeState) -> None:
            decision = self.policy.observe_state(
                self._now_s(),
                mode=int(message.mode),
                error_code=int(message.error_code),
                source_age_s=_stamp_age_s(self, message.stamp),
            )
            if self.policy.phase is Phase.FAULT:
                self._publish(decision)

        def _on_map(self, message: OccupancyGrid) -> None:
            self.map_loaded = (
                int(message.info.width) > 0
                and int(message.info.height) > 0
                and math.isfinite(float(message.info.resolution))
                and float(message.info.resolution) > 0.0
                and len(message.data)
                == int(message.info.width) * int(message.info.height)
            )
            if not self.map_loaded and self.policy.phase in (
                Phase.PENDING_ARM,
                Phase.ARMED,
            ):
                self._publish(self.policy.disconnect("map_invalid"))

        def _on_map_lifecycle(self, message: MapLifecycle) -> None:
            self.map_lifecycle = (
                str(message.map_id),
                str(message.mode),
                int(message.generation),
            )
            decision = self.policy.observe_map_lifecycle(
                self._now_s(),
                map_id=self.map_lifecycle[0],
                mode=self.map_lifecycle[1],
                generation=self.map_lifecycle[2],
            )
            if self.policy.phase is Phase.FAULT:
                self._publish(decision)

        def _on_localization(
            self, message: PoseWithCovarianceStamped
        ) -> None:
            pose = message.pose.pose
            decision = self.policy.observe_localization(
                self._now_s(),
                frame_id=str(message.header.frame_id),
                x_m=float(pose.position.x),
                y_m=float(pose.position.y),
                yaw_rad=_yaw_from_quaternion(pose.orientation),
                source_age_s=_stamp_age_s(self, message.header.stamp),
            )
            if self.policy.phase is Phase.FAULT:
                self._publish(decision)

        def _on_goal_status(self, message: GoalStatusArray) -> None:
            statuses: list[tuple[str, str]] = []
            raw_by_id: dict[str, object] = {}
            for item in message.status_list:
                raw = item.goal_info.goal_id.uuid
                goal_id = bytes(raw).hex()
                statuses.append((goal_id, status_names.get(item.status, "unknown")))
                raw_by_id[goal_id] = raw
            decision = self.policy.observe_goal_statuses(
                self._now_s(), statuses
            )
            if self.policy.goal_id in raw_by_id:
                self.active_goal_raw = raw_by_id[self.policy.goal_id]
            elif len(statuses) == 1 and statuses[0][1] in ACTIVE_STATUSES:
                # Preserve the raw UUID during the bounded status-before-claim
                # race so a claim mismatch can still cancel the exact goal.
                self.active_goal_raw = raw_by_id[statuses[0][0]]
            self._publish(decision)

        def _on_chassis_status(self, message: String) -> None:
            text = str(message.data)
            state, separator, remainder = text.partition(":")
            values: dict[str, str] = {}
            if separator:
                for item in remainder.split(";")[1:]:
                    key, equals, value = item.strip().partition("=")
                    if equals and key and key not in values:
                        values[key] = value.strip()
            decision = self.policy.observe_chassis_status(
                self._now_s(),
                guard_state=state.strip(),
                daemon_armed=(
                    values.get("daemon_armed") == "true"
                    if values.get("daemon_armed") in {"true", "false"}
                    else None
                ),
                motion_configured=(
                    values.get("motion_configured") == "true"
                    if values.get("motion_configured")
                    in {"true", "false"}
                    else None
                ),
                motion_profile=values.get("motion_profile", ""),
                odom_source=values.get("odom_source", ""),
            )
            if self.post_stop is not None and (
                state.strip().upper() != "DISARMED"
                or values.get("daemon_armed") != "false"
            ):
                self.post_stop.invalidate_chassis()
            if (
                decision.reason
                == "chassis_measured_armed_after_zero_hold"
            ):
                try:
                    if self.latest_odom is None:
                        raise GuardError(
                            "no odometry sample at measured arm transition"
                        )
                    self.measurement.start(self.latest_odom)
                except GuardError:
                    decision = self.policy.disconnect(
                        "stage_measurement_start_missing"
                    )
            if (
                self.policy.phase is Phase.FAULT
                or decision.reason == "chassis_measured_armed_after_zero_hold"
            ):
                self._publish(decision)

        def _on_goal_claim(self, message: String) -> None:
            decision = self.policy.observe_goal_claim(
                self._now_s(),
                str(message.data),
                publisher_nodes=self._endpoint_names(
                    self.get_publishers_info_by_topic(GOAL_CLAIM_TOPIC)
                ),
            )
            self._publish(decision)

        @staticmethod
        def _endpoint_names(endpoints: Sequence[object]) -> tuple[str, ...]:
            return tuple(
                sorted(
                    _canonical_node_name(item.node_name, item.node_namespace)
                    for item in endpoints
                )
            )

        def _refresh_graph(self, now_s: float) -> None:
            if now_s - self.last_graph_check_s < 0.20:
                return
            self.last_graph_check_s = now_s
            input_publishers = self._endpoint_names(
                self.get_publishers_info_by_topic(INPUT_TOPIC)
            )
            output_publishers = self._endpoint_names(
                self.get_publishers_info_by_topic(OUTPUT_TOPIC)
            )
            output_subscribers = self._endpoint_names(
                self.get_subscriptions_info_by_topic(OUTPUT_TOPIC)
            )
            canonical_publishers = len(
                self.get_publishers_info_by_topic(CANONICAL_COMMAND_TOPIC)
            )
            decision = self.policy.observe_ownership(
                now_s,
                controller_publishers=input_publishers,
                output_publishers=output_publishers,
                chassis_subscribers=output_subscribers,
                canonical_publishers=canonical_publishers,
            )
            if self.map_loaded and len(
                self.get_publishers_info_by_topic(MAP_TOPIC)
            ) == 1:
                self.policy.observe("map", now_s)
            if (
                self.map_lifecycle is not None
                and len(
                    self.get_publishers_info_by_topic(MAP_LIFECYCLE_TOPIC)
                )
                == 1
            ):
                decision = self.policy.observe_map_lifecycle(
                    now_s,
                    map_id=self.map_lifecycle[0],
                    mode=self.map_lifecycle[1],
                    generation=self.map_lifecycle[2],
                )
                if self.policy.phase is Phase.FAULT:
                    self._publish(decision)
            if self.policy._accepted_goal_claim is not None:
                decision = self.policy.refresh_goal_claim(
                    now_s,
                    publisher_nodes=self._endpoint_names(
                        self.get_publishers_info_by_topic(GOAL_CLAIM_TOPIC)
                    ),
                )
                if self.policy.phase is Phase.FAULT:
                    self._publish(decision)
            if self.policy.phase is Phase.FAULT:
                self._publish(decision)

        def _refresh_tf(self, now_s: float) -> None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_link", Time()
                )
            except TransformException:
                return
            decision = self.policy.observe(
                "tf",
                now_s,
            )
            if self.policy.phase is Phase.FAULT:
                self._publish(decision)

        def _on_timer(self) -> None:
            now_s = self._now_s()
            self._refresh_graph(now_s)
            self._refresh_tf(now_s)
            self._publish(self.policy.tick(now_s))

        def _publish_stop_zero(self) -> None:
            self.command_pub.publish(self._zero())
            zero = self.stop_evidence["zero"]
            assert isinstance(zero, dict)
            zero["published_count"] = int(zero["published_count"]) + 1

        def _cancel_was_confirmed(self, response: object) -> bool:
            if response is None or int(getattr(response, "return_code", -1)) != 0:
                return False
            if self.active_goal_raw is None:
                return False
            expected = bytes(self.active_goal_raw)
            return any(
                bytes(item.goal_id.uuid) == expected
                for item in getattr(response, "goals_canceling", ())
            )

        def stop_and_disarm(self) -> bool:
            """Persistable cancel/zero/disarm plus a full stationary second."""

            self.measurement.freeze()
            for _ in range(10):
                self._publish_stop_zero()
                rclpy.spin_once(self, timeout_sec=0.01)
            zero = self.stop_evidence["zero"]
            assert isinstance(zero, dict)
            zero["confirmed_zero"] = int(zero["published_count"]) >= 10
            cancel_required = bool(
                self.policy.phase is Phase.FAULT
                and (self.policy.goal_id or self.policy._unverified_goal_id)
            )
            cancel = self.stop_evidence["cancel"]
            assert isinstance(cancel, dict)
            cancel["required"] = cancel_required
            cancel_reached = not cancel_required
            if cancel_required:
                # Keep zero flowing while giving the action server one bounded
                # chance to acknowledge cancellation.  Disarm still follows
                # even when the action server has already disappeared.
                cancel_deadline = time.monotonic() + 0.20
                while (
                    not self.cancel_client.service_is_ready()
                    and time.monotonic() < cancel_deadline
                ):
                    self._publish_stop_zero()
                    rclpy.spin_once(self, timeout_sec=0.01)
                cancel["requested"] = self._request_cancel()
                cancel_deadline = time.monotonic() + 0.20
                while (
                    self.cancel_future is not None
                    and not self.cancel_future.done()
                    and time.monotonic() < cancel_deadline
                ):
                    self._publish_stop_zero()
                    rclpy.spin_once(self, timeout_sec=0.01)
                if self.cancel_future is not None and self.cancel_future.done():
                    try:
                        cancel_reached = self._cancel_was_confirmed(
                            self.cancel_future.result()
                        )
                    except Exception:
                        cancel_reached = False
                cancel["confirmed"] = cancel_reached
            if not self.arm_client.service_is_ready():
                self._publish_stop_zero()
                return False
            disarm = self.stop_evidence["disarm"]
            assert isinstance(disarm, dict)
            disarm["requested"] = self._request_disarm()
            deadline = time.monotonic() + 0.75
            while (
                self.disarm_future is not None
                and not self.disarm_future.done()
                and time.monotonic() < deadline
            ):
                self._publish_stop_zero()
                rclpy.spin_once(self, timeout_sec=0.02)
            self._publish_stop_zero()
            if self.disarm_future is None or not self.disarm_future.done():
                return False
            try:
                response = self.disarm_future.result()
            except Exception:
                return False
            if response is None or not response.success:
                return False
            disarm["service_success"] = True
            measured_deadline = time.monotonic() + 0.50
            while time.monotonic() < measured_deadline:
                if (
                    self.policy._chassis_guard_state == "DISARMED"
                    and not self.policy._daemon_armed
                ):
                    disarm["measured_disarmed"] = True
                    break
                self._publish_stop_zero()
                rclpy.spin_once(self, timeout_sec=0.02)
            self._publish_stop_zero()
            if not disarm["measured_disarmed"] or self.latest_odom is None:
                return False

            post_started = time.monotonic()
            self.post_stop = PostStopStationarity(
                self.latest_odom, post_started
            )
            post_deadline = post_started + POST_STOP_OBSERVATION_S
            while time.monotonic() < post_deadline:
                self._publish_stop_zero()
                rclpy.spin_once(self, timeout_sec=0.02)
            post_summary = self.post_stop.summary(time.monotonic())
            self.stop_evidence["post_stop"] = post_summary
            return bool(
                cancel_reached
                and zero["confirmed_zero"]
                and disarm["requested"]
                and disarm["service_success"]
                and disarm["measured_disarmed"]
                and post_summary["passed"]
            )

        def _wait_for_action_result(self) -> tuple[dict[str, object], str]:
            deadline = time.monotonic() + 2.0
            while not action_result_path.exists() and time.monotonic() < deadline:
                self._publish_stop_zero()
                rclpy.spin_once(self, timeout_sec=0.02)
            try:
                payload = read_private_json(action_result_path)
                goal_uuid = validate_action_result(
                    payload,
                    session_id=claims.session_id,
                    pair_id=claims.expected_goal.pair_id,
                    map_id=claims.expected_goal.map_id,
                    map_generation=claims.expected_goal.generation,
                    target_id=claims.expected_goal.target_id,
                )
            except (OSError, ResultError) as error:
                raise GuardError(
                    f"dispatcher action result invalid: {error}"
                ) from error
            return payload, goal_uuid

        def write_terminal_result(self) -> bool:
            """Write exactly one result; only a fully validated PASS returns true."""

            action_error = ""
            action_payload: dict[str, object] | None = None
            action_goal_uuid = ""
            try:
                action_payload, action_goal_uuid = self._wait_for_action_result()
            except GuardError as error:
                action_error = str(error)

            try:
                measurement = self.measurement.summary()
                measurement_complete = True
            except GuardError:
                measurement = {
                    "frame_id": "odom",
                    "start_pose": None,
                    "end_pose": None,
                    "forward_m": None,
                    "total_m": None,
                    "lateral_m": None,
                    "yaw_change_rad": None,
                }
                measurement_complete = False

            cancel = self.stop_evidence["cancel"]
            zero = self.stop_evidence["zero"]
            disarm = self.stop_evidence["disarm"]
            post_stop = self.stop_evidence["post_stop"]
            assert isinstance(cancel, dict)
            assert isinstance(zero, dict)
            assert isinstance(disarm, dict)
            assert isinstance(post_stop, dict)
            checks = {
                "action_succeeded": (
                    "pass" if action_payload is not None else "fail"
                ),
                "measurement_complete": (
                    "pass" if measurement_complete else "fail"
                ),
                "cancel_complete": (
                    "pass"
                    if (
                        not cancel["required"]
                        or (cancel["requested"] and cancel["confirmed"])
                    )
                    else "fail"
                ),
                "zero_complete": (
                    "pass"
                    if (
                        int(zero["published_count"]) >= 10
                        and zero["confirmed_zero"] is True
                    )
                    else "fail"
                ),
                "disarm_complete": (
                    "pass"
                    if disarm
                    == {
                        "requested": True,
                        "service_success": True,
                        "measured_disarmed": True,
                    }
                    else "fail"
                ),
                "post_stop_stationary": (
                    "pass" if post_stop.get("passed") is True else "fail"
                ),
            }
            normal_success = (
                self.policy.phase is Phase.COMPLETE
                and self.policy.terminal_reason == "nav2_goal_succeeded"
                and all(value == "pass" for value in checks.values())
            )
            failure_reason = ""
            if not normal_success:
                failure_reason = (
                    self.policy.fault_reason
                    or action_error
                    or (
                        "terminal state was "
                        f"{self.policy.terminal_reason or self.policy.phase.value}"
                    )
                )
            action_binding = {
                "result_file": str(action_result_path),
                "result_sha256": (
                    sha256_file(action_result_path)
                    if action_payload is not None
                    else ""
                ),
                "goal_accepted": bool(
                    action_payload
                    and action_payload.get("goal_accepted") is True
                ),
                "status_code": (
                    int(action_payload["action_status_code"])
                    if action_payload is not None
                    else -1
                ),
                "status_name": (
                    str(action_payload["action_status_name"])
                    if action_payload is not None
                    else "missing"
                ),
            }
            payload: dict[str, object] = {
                "schema": MEASURED_RESULT_SCHEMA,
                "status": "PASS" if normal_success else "FAIL",
                "session_id": claims.session_id,
                "pair_id": claims.expected_goal.pair_id,
                "map": {
                    "id": claims.expected_goal.map_id,
                    "generation": claims.expected_goal.generation,
                },
                "target": {
                    "id": claims.expected_goal.target_id,
                    "pose": {
                        "x": claims.expected_goal.x_m,
                        "y": claims.expected_goal.y_m,
                        "yaw": claims.expected_goal.yaw_rad,
                    },
                },
                "goal_uuid": action_goal_uuid,
                "started_unix_ns": self.result_started_unix_ns,
                "finished_unix_ns": time.time_ns(),
                "action": action_binding,
                "measurement": measurement,
                "stop_sequence": self.stop_evidence,
                "checks": checks,
                "failure_reason": failure_reason,
            }
            if normal_success:
                try:
                    validate_measured_result(
                        payload, environ, action_result_path
                    )
                except (KeyError, TypeError, ValueError, ResultError) as error:
                    payload["status"] = "FAIL"
                    payload["failure_reason"] = (
                        f"terminal result self-validation failed: {error}"
                    )
                    normal_success = False
            write_private_json(measured_result_path, payload)
            return normal_success

    node = GuardNode()
    exit_code = 0
    terminal_handled = False
    terminal_result_passed = False
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.policy.phase in (Phase.COMPLETE, Phase.FAULT):
                disarmed = node.stop_and_disarm()
                terminal_handled = True
                if node.policy.phase is Phase.FAULT or not disarmed:
                    exit_code = 1
                break
        if stop_requested and not terminal_handled:
            node._publish(node.policy.shutdown())
            disarmed = node.stop_and_disarm()
            terminal_handled = True
            exit_code = 1 if not disarmed else 0
    except Exception as error:
        node.get_logger().error(f"staged Nav2 guard exception: {error}")
        node._publish(node.policy.disconnect("guard_exception"))
        try:
            node.stop_and_disarm()
        except Exception:
            pass
        terminal_handled = True
        exit_code = 1
    finally:
        try:
            if not terminal_handled:
                node._publish(node.policy.shutdown())
                if not node.stop_and_disarm():
                    exit_code = 1
            try:
                terminal_result_passed = node.write_terminal_result()
            except Exception as error:
                node.get_logger().error(
                    f"could not write staged terminal result: {error}"
                )
                terminal_result_passed = False
                exit_code = 1
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            signal.signal(signal.SIGINT, old_sigint)
            signal.signal(signal.SIGTERM, old_sigterm)
    return 0 if exit_code == 0 and terminal_result_passed else 1


def main() -> int:
    try:
        claims = validate_environment(os.environ)
        action_result_path, measured_result_path = result_paths(os.environ)
    except (GuardError, ResultError) as error:
        print(f"staged Nav2 guard disabled: {error}", file=sys.stderr)
        return 2
    return _run_ros(
        claims,
        action_result_path,
        measured_result_path,
        dict(os.environ),
    )


if __name__ == "__main__":
    raise SystemExit(main())
