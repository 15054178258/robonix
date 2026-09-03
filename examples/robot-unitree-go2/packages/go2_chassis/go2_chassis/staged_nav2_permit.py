"""Validate and atomically consume one staged Nav2 motion permit.

This contract is intentionally independent from the immutable first 10 cm
commissioning permit.  One private file authorizes exactly one stage-1
navigation process, one reviewed map generation, and one verified landmark.
It never authorizes a later stage or a second arm in the same process.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import time
from dataclasses import dataclass
from typing import Mapping, Sequence

from .runtime_config import (
    DEFAULT_EXTERNAL_ODOM_TOPIC,
    STAGED_NAV2_COMMAND_TIMEOUT_S,
    STAGED_NAV2_COMMAND_TOPIC,
    STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
    STAGED_NAV2_MAX_DISTANCE_M,
    STAGED_NAV2_MAX_DURATION_S,
    STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
    STAGED_NAV2_MAX_VX_MPS,
    STAGED_NAV2_MAX_VY_MPS,
    STAGED_NAV2_MAX_WZ_RPS,
    STAGED_NAV2_NAV_COMMAND_TOPIC,
    STAGED_NAV2_ODOM_TOPIC,
    STAGED_NAV2_PROFILE,
    STAGED_NAV2_STAGE,
    STAGED_NAV2_STATE_TIMEOUT_S,
    STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
    RuntimeConfig,
)


PERMIT_SCHEMA = "robonix-go2-staged-nav2-permit-v1"
PERMIT_ENV = "GO2_STAGED_NAV2_PERMIT_FILE"
GOAL_PERMIT_ENV = "GO2_STAGED_NAV2_GOAL_PERMIT_FILE"
STAGED_NAV2_ACK = "I_APPROVE_GO2_STAGED_NAV2_MOTION"
SESSION_ENV = "GO2_STAGED_NAV2_SESSION_ID"
STAGE_ENV = "GO2_STAGED_NAV2_STAGE"
GUARD_ACK_ENV = "GO2_STAGED_NAV2_GUARD_ACK"
PAIR_ID_ENV = "GO2_STAGED_NAV2_PAIR_ID"
MAP_ID_ENV = "GO2_STAGED_NAV2_MAP_ID"
MAP_GENERATION_ENV = "GO2_STAGED_NAV2_MAP_GENERATION"
GOAL_SOURCE_ENV = "GO2_STAGED_NAV2_GOAL_SOURCE"
TARGET_ID_ENV = "GO2_STAGED_NAV2_TARGET_ID"
GOAL_X_ENV = "GO2_STAGED_NAV2_EXPECTED_GOAL_X"
GOAL_Y_ENV = "GO2_STAGED_NAV2_EXPECTED_GOAL_Y"
GOAL_YAW_ENV = "GO2_STAGED_NAV2_EXPECTED_GOAL_YAW"
GOAL_EVIDENCE_SHA256_ENV = "GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256"
CHASSIS_ROLE = "chassis_boot"
GOAL_DISPATCH_ROLE = "goal_dispatch"
OPERATOR_GOAL_SOURCE = "operator_reviewed_short_goal"
# Reserved for a future, separately reviewed stage/profile.  Stage 1 must not
# accept this value because the guard deliberately accepts only an
# operator-reviewed short goal.
LANDMARK_GOAL_SOURCE = "verified_landmark"
GOAL_EVIDENCE_SCHEMA = "robonix-go2-staged-nav2-short-goal-evidence-v1"
GOAL_EVIDENCE_MAX_PATH_M = 0.40
GOAL_EVIDENCE_MAX_ENDPOINT_ERROR_M = 0.02
GOAL_EVIDENCE_MAX_ENDPOINT_YAW_ERROR_RAD = 0.10
GOAL_EVIDENCE_MAX_START_ERROR_M = 0.05
GOAL_EVIDENCE_MAX_GOAL_BEARING_ERROR_RAD = 0.50
GOAL_EVIDENCE_REQUIRED_CHECKS = frozenset(
    {
        "map_lifecycle_exact",
        "goal_bearing_within_stage1",
        "plan_only_action",
        "path_collision_free",
        "path_endpoint_matches_goal",
        "path_finite",
        "path_frame_map",
        "path_known_space",
        "path_length_le_0_40_m",
        "path_nonempty",
        "path_start_matches_localization",
    }
)
MAX_PERMIT_BYTES = 64 * 1024
_HEX_RE = re.compile(r"^[0-9a-f]{64}$")
_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{7,63}$")
_LANDMARK_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$")


class PermitError(ValueError):
    """The staged permit is absent, replayed, expired, or mismatched."""


def _unique_json_object(pairs: Sequence[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise PermitError(f"duplicate JSON key is forbidden: {key}")
        result[key] = value
    return result


@dataclass(frozen=True, slots=True)
class ConsumedGoalPermit:
    path: Path
    permit_id: str
    pair_id: str
    session_id: str
    map_id: str
    map_generation: int
    goal_source: str
    target_id: str
    goal_x: float
    goal_y: float
    goal_yaw: float


@dataclass(frozen=True, slots=True)
class ValidatedGoalEvidence:
    path: Path
    sha256: str
    map_id: str
    map_generation: int
    goal_source: str
    target_id: str
    goal_x: float
    goal_y: float
    goal_yaw: float
    start_x: float
    start_y: float
    start_yaw: float
    path_length_m: float
    pose_count: int


@dataclass(frozen=True, slots=True)
class ValidatedPermitBundle:
    chassis_path: Path
    goal_path: Path
    chassis_permit_id: str
    goal_permit_id: str
    pair_id: str
    session_id: str
    map_id: str
    map_generation: int
    goal_source: str
    target_id: str
    goal_x: float
    goal_y: float
    goal_yaw: float
    goal_evidence: ValidatedGoalEvidence
    environment: Mapping[str, str]


def _owned_regular_file(path: Path, *, private: bool) -> os.stat_result:
    try:
        info = os.lstat(path)
    except FileNotFoundError as exc:
        raise PermitError(f"required file does not exist: {path}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise PermitError(f"file must be a non-symlink regular file: {path}")
    if info.st_uid != os.geteuid():
        raise PermitError(f"file is not owned by the current UID: {path}")
    mode = stat.S_IMODE(info.st_mode)
    if private and mode != 0o600:
        raise PermitError(f"permit mode must be exactly 0600, got {mode:04o}")
    if not private and mode & 0o022:
        raise PermitError(f"evidence file is group/world writable: {path}")
    return info


def _read_private_json(path: Path) -> dict[str, object]:
    expected = _owned_regular_file(path, private=True)
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        opened = os.fstat(descriptor)
        if (
            opened.st_dev != expected.st_dev
            or opened.st_ino != expected.st_ino
            or opened.st_uid != os.geteuid()
            or not stat.S_ISREG(opened.st_mode)
        ):
            raise PermitError("permit changed while it was being opened")
        raw = os.read(descriptor, MAX_PERMIT_BYTES + 1)
        if len(raw) > MAX_PERMIT_BYTES:
            raise PermitError("permit is unreasonably large")
    finally:
        os.close(descriptor)
    try:
        payload = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_unique_json_object
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PermitError("permit is not valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise PermitError("permit root must be a JSON object")
    return payload


def _private_parent(path: Path, label: str) -> None:
    try:
        parent_info = os.lstat(path.parent)
    except FileNotFoundError as exc:
        raise PermitError(f"{label} parent does not exist: {path.parent}") from exc
    if (
        not stat.S_ISDIR(parent_info.st_mode)
        or stat.S_ISLNK(parent_info.st_mode)
        or parent_info.st_uid != os.geteuid()
        or stat.S_IMODE(parent_info.st_mode) != 0o700
    ):
        raise PermitError(
            f"{label} parent must be an owned non-symlink 0700 directory"
        )


def _read_evidence_json(path: Path) -> dict[str, object]:
    expected = _owned_regular_file(path, private=False)
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        opened = os.fstat(descriptor)
        if (
            opened.st_dev != expected.st_dev
            or opened.st_ino != expected.st_ino
            or opened.st_uid != os.geteuid()
            or not stat.S_ISREG(opened.st_mode)
        ):
            raise PermitError("goal evidence changed while it was being opened")
        raw = os.read(descriptor, MAX_PERMIT_BYTES + 1)
        if len(raw) > MAX_PERMIT_BYTES:
            raise PermitError("goal evidence is unreasonably large")
    finally:
        os.close(descriptor)
    try:
        payload = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_unique_json_object
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PermitError("goal evidence is not valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise PermitError("goal evidence root must be a JSON object")
    return payload


def sha256_file(path: Path) -> str:
    _owned_regular_file(path, private=False)
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _exact_finite_number(value: object, expected: float, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or not math.isclose(
            float(value), expected, rel_tol=0.0, abs_tol=1e-9
        )
    ):
        raise PermitError(f"goal evidence {label} does not match the permit")
    return float(value)


def validate_short_goal_evidence(
    path: Path,
    *,
    expected_sha256: str,
    map_id: str,
    map_generation: int,
    goal_source: str,
    target_id: str,
    goal_pose: Mapping[str, float],
    now_unix_ns: int | None = None,
) -> ValidatedGoalEvidence:
    """Validate one plan-only, known-space path bound to the exact permit.

    The evidence is intentionally stricter than the generic no-motion planner
    output.  Every check must be explicitly ``pass``; absent, ``unknown``, or
    failed checks are rejected.  The 0.40 m plan limit leaves margin below the
    chassis stage's independent 0.50 m measured-distance cutoff.
    """

    if not path.is_absolute():
        raise PermitError("goal evidence path must be absolute")
    if _HEX_RE.fullmatch(expected_sha256) is None:
        raise PermitError("goal evidence hash is malformed")
    actual_sha256 = sha256_file(path)
    if actual_sha256 != expected_sha256:
        raise PermitError("goal evidence hash changed")
    payload = _read_evidence_json(path)
    exact_root = {
        "schema": GOAL_EVIDENCE_SCHEMA,
        "operation": "compute_path_to_pose_only",
        "action": "/compute_path_to_pose",
        "frame_id": "map",
        "motion_disabled": True,
        "status": "pass",
    }
    for name, expected in exact_root.items():
        if payload.get(name) != expected:
            raise PermitError(
                f"goal evidence {name} is not the exact successful plan-only claim"
            )
    if payload.get("action_status") != 4:
        raise PermitError("goal evidence action_status is not SUCCEEDED(4)")
    started_ns = payload.get("started_unix_ns")
    finished_ns = payload.get("finished_unix_ns")
    now_ns = time.time_ns() if now_unix_ns is None else now_unix_ns
    if (
        isinstance(started_ns, bool)
        or isinstance(finished_ns, bool)
        or not isinstance(started_ns, int)
        or not isinstance(finished_ns, int)
        or started_ns <= 0
        or finished_ns < started_ns
        or finished_ns > now_ns
    ):
        raise PermitError(
            "goal evidence timestamps are invalid or in the future"
        )

    map_claim = payload.get("map")
    if (
        not isinstance(map_claim, dict)
        or set(map_claim) != {"id", "generation", "mode"}
        or map_claim.get("id") != map_id
        or map_claim.get("generation") != map_generation
        or map_claim.get("mode") != "localization"
    ):
        raise PermitError("goal evidence map lifecycle does not match the permit")

    goal_claim = payload.get("goal")
    if (
        not isinstance(goal_claim, dict)
        or set(goal_claim) != {"source", "target_id", "pose"}
        or goal_claim.get("source") != goal_source
        or goal_claim.get("target_id") != target_id
    ):
        raise PermitError("goal evidence identity does not match the permit")
    evidence_pose = goal_claim.get("pose")
    if (
        not isinstance(evidence_pose, dict)
        or set(evidence_pose) != {"x", "y", "yaw"}
    ):
        raise PermitError("goal evidence pose is malformed")
    for coordinate in ("x", "y", "yaw"):
        _exact_finite_number(
            evidence_pose.get(coordinate),
            float(goal_pose[coordinate]),
            f"goal.pose.{coordinate}",
        )

    checks = payload.get("checks")
    if (
        not isinstance(checks, dict)
        or set(checks) != GOAL_EVIDENCE_REQUIRED_CHECKS
        or any(value != "pass" for value in checks.values())
    ):
        raise PermitError(
            "goal evidence checks must be exact, complete, and all pass"
        )

    path_claim = payload.get("path")
    expected_path_keys = {
        "endpoint_position_error_m",
        "endpoint_yaw_error_rad",
        "end",
        "length_m",
        "lethal_occupancy_threshold",
        "map_height",
        "map_resolution_m",
        "map_width",
        "pose_count",
        "sample_count",
        "start",
    }
    if (
        not isinstance(path_claim, dict)
        or set(path_claim) != expected_path_keys
    ):
        raise PermitError("goal evidence path summary keys are not exact")
    pose_count = path_claim.get("pose_count")
    length_m = path_claim.get("length_m")
    if (
        isinstance(pose_count, bool)
        or not isinstance(pose_count, int)
        or pose_count <= 0
    ):
        raise PermitError("goal evidence path pose_count must be positive")
    sample_count = path_claim.get("sample_count")
    map_width = path_claim.get("map_width")
    map_height = path_claim.get("map_height")
    if any(
        isinstance(value, bool)
        or not isinstance(value, int)
        or value <= 0
        for value in (sample_count, map_width, map_height)
    ) or int(sample_count) < pose_count:
        raise PermitError("goal evidence path/map counts are invalid")
    map_resolution_m = path_claim.get("map_resolution_m")
    if (
        isinstance(map_resolution_m, bool)
        or not isinstance(map_resolution_m, (int, float))
        or not math.isfinite(float(map_resolution_m))
        or float(map_resolution_m) <= 0.0
        or path_claim.get("lethal_occupancy_threshold") != 65
    ):
        raise PermitError("goal evidence map summary is invalid")
    if (
        isinstance(length_m, bool)
        or not isinstance(length_m, (int, float))
        or not math.isfinite(float(length_m))
        or float(length_m) < 0.0
        or float(length_m) > GOAL_EVIDENCE_MAX_PATH_M
    ):
        raise PermitError(
            "goal evidence path length must be finite and at most 0.40 m"
        )
    if payload.get("action_status_name") != "succeeded":
        raise PermitError("goal evidence planner action did not succeed")
    start = path_claim.get("start")
    localization = payload.get("localization")
    if (
        not isinstance(start, dict)
        or set(start) != {"x", "y", "yaw"}
        or not isinstance(localization, dict)
        or set(localization) != {"pose", "source_age_s"}
        or not isinstance(localization.get("pose"), dict)
        or set(localization["pose"]) != {"x", "y", "yaw"}
    ):
        raise PermitError(
            "goal evidence path start or live localization is malformed"
        )
    try:
        start_x = float(start["x"])
        start_y = float(start["y"])
        start_yaw = float(start["yaw"])
        localization_x = float(localization["pose"]["x"])
        localization_y = float(localization["pose"]["y"])
        localization_yaw = float(localization["pose"]["yaw"])
        localization_age_s = float(localization["source_age_s"])
    except (TypeError, ValueError) as exc:
        raise PermitError(
            "goal evidence path start or localization is not numeric"
        ) from exc
    if not all(
        math.isfinite(value)
        for value in (
            start_x,
            start_y,
            start_yaw,
            localization_x,
            localization_y,
            localization_yaw,
            localization_age_s,
        )
    ) or not -0.05 <= localization_age_s <= 0.50:
        raise PermitError(
            "goal evidence path start or localization is not fresh and finite"
        )
    start_error_m = math.hypot(
        start_x - localization_x, start_y - localization_y
    )
    if start_error_m > GOAL_EVIDENCE_MAX_START_ERROR_M:
        raise PermitError(
            "goal evidence path start does not match live localization"
        )
    stage1_geometry = payload.get("stage1_geometry")
    if (
        not isinstance(stage1_geometry, dict)
        or set(stage1_geometry)
        != {"goal_bearing_error_rad", "goal_distance_m"}
    ):
        raise PermitError("goal evidence stage1 geometry is malformed")
    delta_x = float(goal_pose["x"]) - localization_x
    delta_y = float(goal_pose["y"]) - localization_y
    goal_distance_m = math.hypot(delta_x, delta_y)
    desired_heading = (
        math.atan2(delta_y, delta_x)
        if goal_distance_m > 1e-6
        else float(goal_pose["yaw"])
    )
    goal_bearing_error = abs(
        math.atan2(
            math.sin(desired_heading - localization_yaw),
            math.cos(desired_heading - localization_yaw),
        )
    )
    try:
        recorded_goal_distance = float(stage1_geometry["goal_distance_m"])
        recorded_bearing_error = float(
            stage1_geometry["goal_bearing_error_rad"]
        )
    except (TypeError, ValueError) as exc:
        raise PermitError("goal evidence stage1 geometry is not numeric") from exc
    if (
        not math.isfinite(recorded_goal_distance)
        or not math.isfinite(recorded_bearing_error)
        or not math.isclose(
            recorded_goal_distance,
            goal_distance_m,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
        or not math.isclose(
            recorded_bearing_error,
            goal_bearing_error,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
        or goal_bearing_error > GOAL_EVIDENCE_MAX_GOAL_BEARING_ERROR_RAD
    ):
        raise PermitError(
            "goal evidence target is outside the stage1 forward bearing"
        )
    endpoint = path_claim.get("end")
    if (
        not isinstance(endpoint, dict)
        or set(endpoint) != {"x", "y", "yaw"}
    ):
        raise PermitError("goal evidence path endpoint is malformed")
    try:
        endpoint_x = float(endpoint["x"])
        endpoint_y = float(endpoint["y"])
        endpoint_yaw = float(endpoint["yaw"])
    except (TypeError, ValueError) as exc:
        raise PermitError("goal evidence path endpoint is not numeric") from exc
    if not all(
        math.isfinite(value)
        for value in (endpoint_x, endpoint_y, endpoint_yaw)
    ):
        raise PermitError("goal evidence path endpoint is not finite")
    endpoint_error_m = math.hypot(
        endpoint_x - float(goal_pose["x"]),
        endpoint_y - float(goal_pose["y"]),
    )
    yaw_error = abs(
        math.atan2(
            math.sin(endpoint_yaw - float(goal_pose["yaw"])),
            math.cos(endpoint_yaw - float(goal_pose["yaw"])),
        )
    )
    recorded_endpoint_error = path_claim.get("endpoint_position_error_m")
    recorded_yaw_error = path_claim.get("endpoint_yaw_error_rad")
    if (
        isinstance(recorded_endpoint_error, bool)
        or isinstance(recorded_yaw_error, bool)
        or not isinstance(recorded_endpoint_error, (int, float))
        or not isinstance(recorded_yaw_error, (int, float))
        or not math.isfinite(float(recorded_endpoint_error))
        or not math.isfinite(float(recorded_yaw_error))
        or not math.isclose(
            float(recorded_endpoint_error),
            endpoint_error_m,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(recorded_yaw_error),
            yaw_error,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
    ):
        raise PermitError("goal evidence endpoint error summary is inconsistent")
    if (
        endpoint_error_m > GOAL_EVIDENCE_MAX_ENDPOINT_ERROR_M
        or yaw_error > GOAL_EVIDENCE_MAX_ENDPOINT_YAW_ERROR_RAD
    ):
        raise PermitError(
            "goal evidence path endpoint does not match the permitted goal"
        )
    return ValidatedGoalEvidence(
        path=path,
        sha256=actual_sha256,
        map_id=map_id,
        map_generation=map_generation,
        goal_source=goal_source,
        target_id=target_id,
        goal_x=float(goal_pose["x"]),
        goal_y=float(goal_pose["y"]),
        goal_yaw=float(goal_pose["yaw"]),
        start_x=start_x,
        start_y=start_y,
        start_yaw=start_yaw,
        path_length_m=float(length_m),
        pose_count=pose_count,
    )


def _exact_number(value: object, expected: float, label: str) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) != expected
    ):
        raise PermitError(f"{label} does not match the fixed stage1 envelope")


def _positive_generation(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise PermitError("map_generation must be a positive integer")
    return value


def _goal_pose(value: object) -> dict[str, float]:
    if not isinstance(value, dict) or set(value) != {"x", "y", "yaw"}:
        raise PermitError("goal_pose must contain exactly x, y, and yaw")
    result: dict[str, float] = {}
    for name in ("x", "y", "yaw"):
        item = value[name]
        if (
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(float(item))
        ):
            raise PermitError(f"goal_pose.{name} must be finite")
        result[name] = float(item)
    if not -math.pi <= result["yaw"] <= math.pi:
        raise PermitError("goal_pose.yaw must be within [-pi, pi]")
    return result


def validate_permit(
    payload: Mapping[str, object],
    runtime: RuntimeConfig,
    environ: Mapping[str, str],
    package_root: Path,
    *,
    expected_role: str,
    now_unix_ns: int | None = None,
) -> str:
    """Return the permit id after validating all runtime and evidence claims."""

    now_ns = time.time_ns() if now_unix_ns is None else now_unix_ns
    if payload.get("schema") != PERMIT_SCHEMA:
        raise PermitError("unexpected staged-nav2 permit schema")
    permit_id = payload.get("permit_id")
    pair_id = payload.get("pair_id")
    session_id = payload.get("session_id")
    if not isinstance(permit_id, str) or _ID_RE.fullmatch(permit_id) is None:
        raise PermitError("permit_id is malformed")
    if not isinstance(session_id, str) or _ID_RE.fullmatch(session_id) is None:
        raise PermitError("session_id is malformed")
    if not isinstance(pair_id, str) or _ID_RE.fullmatch(pair_id) is None:
        raise PermitError("pair_id is malformed")
    if payload.get("permit_role") != expected_role:
        raise PermitError("permit role does not match this consumer")
    if payload.get("one_time") is not True:
        raise PermitError("permit must explicitly be one_time")
    issued_ns = payload.get("issued_unix_ns")
    expires_ns = payload.get("expires_unix_ns")
    if (
        isinstance(issued_ns, bool)
        or isinstance(expires_ns, bool)
        or not isinstance(issued_ns, int)
        or not isinstance(expires_ns, int)
        or issued_ns <= 0
        or expires_ns <= issued_ns
    ):
        raise PermitError("permit timestamp metadata is malformed")
    if now_ns < issued_ns:
        raise PermitError("permit issue time is in the future")

    map_id = payload.get("map_id")
    goal_source = payload.get("goal_source")
    target_id = payload.get("target_id")
    map_generation = _positive_generation(payload.get("map_generation"))
    if not isinstance(map_id, str) or _ID_RE.fullmatch(map_id) is None:
        raise PermitError("map_id is malformed")
    if goal_source != OPERATOR_GOAL_SOURCE:
        raise PermitError(
            "stage1 goal_source must be operator_reviewed_short_goal"
        )
    if not isinstance(target_id, str) or _LANDMARK_RE.fullmatch(target_id) is None:
        raise PermitError("target_id is malformed")
    goal_pose = _goal_pose(payload.get("goal_pose"))

    exact_text = {
        "profile": STAGED_NAV2_PROFILE,
        "stage": STAGED_NAV2_STAGE,
        "operator_ack": STAGED_NAV2_ACK,
        "guard_ack": STAGED_NAV2_ACK,
        "network_interface": runtime.network_interface,
        "state_topic": runtime.state_topic,
        "nav_command_topic": STAGED_NAV2_NAV_COMMAND_TOPIC,
        "command_topic": STAGED_NAV2_COMMAND_TOPIC,
        "odom_topic": STAGED_NAV2_ODOM_TOPIC,
        "external_odom_topic": DEFAULT_EXTERNAL_ODOM_TOPIC,
        "arm_service": runtime.arm_service,
    }
    for name, expected in exact_text.items():
        if payload.get(name) != expected:
            raise PermitError(f"permit {name} does not match this runtime")
    if payload.get("allowed_modes") != list(runtime.allowed_modes):
        raise PermitError("permit allowed_modes does not match this runtime")
    if payload.get("allowed_state_markers") != list(
        runtime.allowed_state_markers
    ):
        raise PermitError(
            "permit allowed_state_markers does not match this runtime"
        )

    exact_environment = {
        SESSION_ENV: session_id,
        PAIR_ID_ENV: pair_id,
        STAGE_ENV: STAGED_NAV2_STAGE,
        GUARD_ACK_ENV: STAGED_NAV2_ACK,
        MAP_ID_ENV: map_id,
        MAP_GENERATION_ENV: str(map_generation),
        GOAL_SOURCE_ENV: goal_source,
        TARGET_ID_ENV: target_id,
        GOAL_X_ENV: format(goal_pose["x"], ".17g"),
        GOAL_Y_ENV: format(goal_pose["y"], ".17g"),
        GOAL_YAW_ENV: format(goal_pose["yaw"], ".17g"),
    }
    for name, expected in exact_environment.items():
        if environ.get(name, "").strip() != expected:
            raise PermitError(f"{name} does not match the one-time permit")

    _exact_number(
        payload.get("max_linear_x_mps"),
        STAGED_NAV2_MAX_VX_MPS,
        "max_linear_x_mps",
    )
    _exact_number(
        payload.get("max_linear_y_mps"),
        STAGED_NAV2_MAX_VY_MPS,
        "max_linear_y_mps",
    )
    _exact_number(
        payload.get("max_angular_z_rps"),
        STAGED_NAV2_MAX_WZ_RPS,
        "max_angular_z_rps",
    )
    _exact_number(
        payload.get("max_linear_accel_mps2"),
        STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
        "max_linear_accel_mps2",
    )
    _exact_number(
        payload.get("max_angular_accel_rps2"),
        STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
        "max_angular_accel_rps2",
    )
    _exact_number(
        payload.get("max_duration_s"),
        STAGED_NAV2_MAX_DURATION_S,
        "max_duration_s",
    )
    _exact_number(
        payload.get("max_distance_m"),
        STAGED_NAV2_MAX_DISTANCE_M,
        "max_distance_m",
    )
    _exact_number(
        payload.get("command_timeout_s"),
        STAGED_NAV2_COMMAND_TIMEOUT_S,
        "command_timeout_s",
    )
    _exact_number(
        payload.get("state_timeout_s"),
        STAGED_NAV2_STATE_TIMEOUT_S,
        "state_timeout_s",
    )
    _exact_number(
        payload.get("external_odom_timeout_s"),
        STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
        "external_odom_timeout_s",
    )

    evidence = payload.get("evidence")
    expected_evidence = {
        "dds_identity",
        "state",
        "time",
        "goal",
    }
    if not isinstance(evidence, dict) or set(evidence) != expected_evidence:
        raise PermitError(
            "permit must bind exactly identity, state, time, and exact goal "
            "evidence; live staged readiness is a post-boot launcher gate"
        )
    goal_claim = evidence.get("goal")
    if (
        not isinstance(goal_claim, dict)
        or environ.get(GOAL_EVIDENCE_SHA256_ENV, "").strip()
        != goal_claim.get("sha256")
    ):
        raise PermitError(
            f"{GOAL_EVIDENCE_SHA256_ENV} does not match exact goal evidence"
        )
    resolved_root = package_root.resolve()
    for name in sorted(expected_evidence):
        claim = evidence[name]
        if not isinstance(claim, dict) or set(claim) != {"path", "sha256"}:
            raise PermitError(f"{name} evidence claim is malformed")
        path_text = claim.get("path")
        expected_hash = claim.get("sha256")
        if not isinstance(path_text, str) or not Path(path_text).is_absolute():
            raise PermitError(f"{name} evidence path must be absolute")
        if (
            not isinstance(expected_hash, str)
            or _HEX_RE.fullmatch(expected_hash) is None
        ):
            raise PermitError(f"{name} evidence hash is malformed")
        evidence_path = Path(path_text).resolve()
        try:
            evidence_path.relative_to(resolved_root)
        except ValueError as exc:
            raise PermitError(f"{name} evidence is outside package root") from exc
        if sha256_file(evidence_path) != expected_hash:
            raise PermitError(f"{name} evidence hash changed")
    return permit_id


def validate_staged_nav2_permit_bundle(
    chassis_path: Path,
    goal_path: Path,
    runtime: RuntimeConfig,
    package_root: Path,
    expected_evidence_paths: Mapping[str, Path],
    *,
    now_unix_ns: int | None = None,
) -> ValidatedPermitBundle:
    """Validate a matched chassis/dispatcher pair without consuming either.

    This is the launcher's only preflight path.  It derives the short-lived
    runtime claims from the private permit payloads rather than accepting those
    claims independently from the shell.  Atomic consumption remains owned by
    the chassis provider and sole goal dispatcher later in the launch.
    """

    if not runtime.allow_motion or runtime.motion_profile != STAGED_NAV2_PROFILE:
        raise PermitError(
            "bundle validation requires the exact staged-nav2 runtime"
        )
    if not chassis_path.is_absolute() or not goal_path.is_absolute():
        raise PermitError("both staged permit paths must be absolute")
    if chassis_path == goal_path:
        raise PermitError("chassis and goal permit paths must be distinct")
    _private_parent(chassis_path, "chassis permit")
    _private_parent(goal_path, "goal permit")
    chassis_payload = _read_private_json(chassis_path)
    goal_payload = _read_private_json(goal_path)
    if chassis_payload.get("permit_role") != CHASSIS_ROLE:
        raise PermitError("chassis permit has the wrong role")
    if goal_payload.get("permit_role") != GOAL_DISPATCH_ROLE:
        raise PermitError("goal permit has the wrong role")
    chassis_id = chassis_payload.get("permit_id")
    goal_id = goal_payload.get("permit_id")
    if chassis_id == goal_id:
        raise PermitError("paired permits must have distinct permit IDs")
    excluded = {"permit_id", "permit_role"}
    chassis_common = {
        key: value for key, value in chassis_payload.items() if key not in excluded
    }
    goal_common = {
        key: value for key, value in goal_payload.items() if key not in excluded
    }
    if chassis_common != goal_common:
        raise PermitError(
            "chassis and goal permits are not one exact matched pair"
        )

    pose = _goal_pose(chassis_payload.get("goal_pose"))
    evidence = chassis_payload.get("evidence")
    if not isinstance(evidence, dict):
        raise PermitError("paired permit evidence claims are malformed")
    goal_evidence_claim = evidence.get("goal")
    if not isinstance(goal_evidence_claim, dict):
        raise PermitError("paired permit goal evidence claim is malformed")
    goal_evidence_hash = goal_evidence_claim.get("sha256")
    if (
        not isinstance(goal_evidence_hash, str)
        or _HEX_RE.fullmatch(goal_evidence_hash) is None
    ):
        raise PermitError("paired permit goal evidence hash is malformed")

    session_id = str(chassis_payload.get("session_id", ""))
    pair_id = str(chassis_payload.get("pair_id", ""))
    map_id = str(chassis_payload.get("map_id", ""))
    map_generation = _positive_generation(
        chassis_payload.get("map_generation")
    )
    goal_source = str(chassis_payload.get("goal_source", ""))
    target_id = str(chassis_payload.get("target_id", ""))
    derived_environment = {
        SESSION_ENV: session_id,
        PAIR_ID_ENV: pair_id,
        STAGE_ENV: STAGED_NAV2_STAGE,
        GUARD_ACK_ENV: STAGED_NAV2_ACK,
        MAP_ID_ENV: map_id,
        MAP_GENERATION_ENV: str(map_generation),
        GOAL_SOURCE_ENV: goal_source,
        TARGET_ID_ENV: target_id,
        GOAL_X_ENV: format(pose["x"], ".17g"),
        GOAL_Y_ENV: format(pose["y"], ".17g"),
        GOAL_YAW_ENV: format(pose["yaw"], ".17g"),
        GOAL_EVIDENCE_SHA256_ENV: goal_evidence_hash,
    }
    now_ns = time.time_ns() if now_unix_ns is None else now_unix_ns
    validated_chassis_id = validate_permit(
        chassis_payload,
        runtime,
        derived_environment,
        package_root,
        expected_role=CHASSIS_ROLE,
        now_unix_ns=now_ns,
    )
    validated_goal_id = validate_permit(
        goal_payload,
        runtime,
        derived_environment,
        package_root,
        expected_role=GOAL_DISPATCH_ROLE,
        now_unix_ns=now_ns,
    )

    expected_names = {
        "dds_identity",
        "state",
        "time",
        "goal",
    }
    if set(expected_evidence_paths) != expected_names:
        raise PermitError(
            "launcher must provide exactly all four staged pre-boot evidence paths"
        )
    resolved_expected: dict[str, Path] = {}
    for name in sorted(expected_names):
        candidate = expected_evidence_paths[name]
        if not candidate.is_absolute():
            raise PermitError(f"{name} expected evidence path must be absolute")
        resolved_expected[name] = candidate.resolve()
        claim = evidence.get(name)
        if (
            not isinstance(claim, dict)
            or claim.get("path") != str(resolved_expected[name])
        ):
            raise PermitError(
                f"{name} evidence path does not match the launcher input"
            )

    goal_evidence = validate_short_goal_evidence(
        resolved_expected["goal"],
        expected_sha256=goal_evidence_hash,
        map_id=map_id,
        map_generation=map_generation,
        goal_source=goal_source,
        target_id=target_id,
        goal_pose=pose,
        now_unix_ns=now_ns,
    )
    return ValidatedPermitBundle(
        chassis_path=chassis_path,
        goal_path=goal_path,
        chassis_permit_id=validated_chassis_id,
        goal_permit_id=validated_goal_id,
        pair_id=pair_id,
        session_id=session_id,
        map_id=map_id,
        map_generation=map_generation,
        goal_source=goal_source,
        target_id=target_id,
        goal_x=pose["x"],
        goal_y=pose["y"],
        goal_yaw=pose["yaw"],
        goal_evidence=goal_evidence,
        environment=derived_environment,
    )


def consume_staged_nav2_permit(
    runtime: RuntimeConfig,
    environ: Mapping[str, str],
    package_root: Path,
) -> Path:
    """Validate and rename the staged permit before any motion process starts."""

    if not runtime.allow_motion or runtime.motion_profile != STAGED_NAV2_PROFILE:
        raise PermitError(
            "staged permit consumption requires the exact staged-nav2 runtime"
        )
    path_text = environ.get(PERMIT_ENV, "").strip()
    if not path_text:
        raise PermitError(f"motion requires {PERMIT_ENV}")
    permit_path = Path(path_text)
    if not permit_path.is_absolute():
        raise PermitError("staged-nav2 permit path must be absolute")
    _private_parent(permit_path, "permit")
    payload = _read_private_json(permit_path)
    permit_id = validate_permit(
        payload,
        runtime,
        environ,
        package_root,
        expected_role=CHASSIS_ROLE,
    )
    return _atomic_consume(permit_path, permit_id)


def _atomic_consume(permit_path: Path, permit_id: str) -> Path:
    consumed_path = permit_path.with_name(
        f"{permit_path.name}.consumed-{permit_id}"
    )
    if consumed_path.exists():
        raise PermitError("permit was already consumed")
    try:
        os.rename(permit_path, consumed_path)
    except FileNotFoundError as exc:
        raise PermitError("permit was consumed concurrently") from exc
    return consumed_path


def consume_staged_nav2_goal_permit(
    runtime: RuntimeConfig,
    environ: Mapping[str, str],
    package_root: Path,
) -> ConsumedGoalPermit:
    """Consume and return the exact goal claim for the sole action sender."""

    if not runtime.allow_motion or runtime.motion_profile != STAGED_NAV2_PROFILE:
        raise PermitError(
            "goal permit consumption requires the exact staged-nav2 runtime"
        )
    path_text = environ.get(GOAL_PERMIT_ENV, "").strip()
    if not path_text:
        raise PermitError(f"goal dispatch requires {GOAL_PERMIT_ENV}")
    permit_path = Path(path_text)
    if not permit_path.is_absolute():
        raise PermitError("staged-nav2 goal permit path must be absolute")
    _private_parent(permit_path, "goal permit")
    payload = _read_private_json(permit_path)
    permit_id = validate_permit(
        payload,
        runtime,
        environ,
        package_root,
        expected_role=GOAL_DISPATCH_ROLE,
    )
    consumed_path = _atomic_consume(permit_path, permit_id)
    pose = _goal_pose(payload["goal_pose"])
    return ConsumedGoalPermit(
        path=consumed_path,
        permit_id=permit_id,
        pair_id=str(payload["pair_id"]),
        session_id=str(payload["session_id"]),
        map_id=str(payload["map_id"]),
        map_generation=int(payload["map_generation"]),
        goal_source=str(payload["goal_source"]),
        target_id=str(payload["target_id"]),
        goal_x=pose["x"],
        goal_y=pose["y"],
        goal_yaw=pose["yaw"],
    )
