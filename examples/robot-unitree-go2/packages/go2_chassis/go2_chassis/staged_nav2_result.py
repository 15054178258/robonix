"""Private result contracts for one staged Nav2 goal.

The goal dispatcher records the exact action result.  The motion guard then
binds that action result to measured odometry and its fail-closed stop
sequence.  Only the guard's final measured result can be reported as a
successful physical stage.
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
from typing import Mapping


ACTION_RESULT_SCHEMA = "robonix-go2-staged-nav2-action-result-v1"
MEASURED_RESULT_SCHEMA = "robonix-go2-staged-nav2-measured-result-v1"
RUN_DIR_ENV = "GO2_STAGED_NAV2_RUN_DIR"
ACTION_RESULT_ENV = "GO2_STAGED_NAV2_ACTION_RESULT_FILE"
MEASURED_RESULT_ENV = "GO2_STAGED_NAV2_RESULT_FILE"
ACTION_RESULT_NAME = "goal-action-result.json"
MEASURED_RESULT_NAME = "measured-result.json"
NAV_ACTION = "/navigate_to_pose"
SUCCEEDED_STATUS_CODE = 4
MAX_DISTANCE_M = 0.50
POST_STOP_OBSERVATION_S = 1.0
POST_STOP_MIN_ODOM_SAMPLES = 10
POST_STOP_MAX_DRIFT_M = 0.02
POST_STOP_MAX_LINEAR_MPS = 0.03
POST_STOP_MAX_YAW_RPS = 0.03
_GOAL_UUID_RE = re.compile(r"^[0-9a-f]{32}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class ResultError(ValueError):
    """A result path or payload does not satisfy the staged contract."""


def _private_run_dir(environ: Mapping[str, str]) -> Path:
    raw = environ.get(RUN_DIR_ENV, "").strip()
    if not raw or not Path(raw).is_absolute():
        raise ResultError(f"{RUN_DIR_ENV} must be an absolute path")
    path = Path(raw)
    try:
        info = os.lstat(path)
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise ResultError(f"cannot inspect staged run directory: {error}") from error
    if (
        resolved != path
        or not stat.S_ISDIR(info.st_mode)
        or info.st_uid != os.geteuid()
        or stat.S_IMODE(info.st_mode) != 0o700
    ):
        raise ResultError(
            "staged run directory must be canonical, owned by this UID, "
            "and mode 0700"
        )
    return path


def result_paths(environ: Mapping[str, str]) -> tuple[Path, Path]:
    run_dir = _private_run_dir(environ)
    values = (
        (ACTION_RESULT_ENV, ACTION_RESULT_NAME),
        (MEASURED_RESULT_ENV, MEASURED_RESULT_NAME),
    )
    paths: list[Path] = []
    for variable, filename in values:
        raw = environ.get(variable, "").strip()
        expected = run_dir / filename
        if not raw or Path(raw) != expected:
            raise ResultError(f"{variable} must exactly equal {expected}")
        paths.append(expected)
    if paths[0] == paths[1]:
        raise ResultError("action and measured result paths must differ")
    return paths[0], paths[1]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_private_json(path: Path, payload: Mapping[str, object]) -> None:
    if os.path.lexists(path):
        raise ResultError(f"refusing to overwrite staged result: {path}")
    encoded = (
        json.dumps(
            payload,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
        | (os.O_NOFOLLOW if hasattr(os, "O_NOFOLLOW") else 0),
        0o600,
    )
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        path.unlink(missing_ok=True)
        raise
    finally:
        os.close(descriptor)


def read_private_json(path: Path) -> dict[str, object]:
    try:
        info = os.lstat(path)
    except OSError as error:
        raise ResultError(f"cannot inspect staged result: {error}") from error
    if (
        not stat.S_ISREG(info.st_mode)
        or info.st_uid != os.geteuid()
        or stat.S_IMODE(info.st_mode) != 0o600
        or info.st_nlink != 1
        or info.st_size <= 0
        or info.st_size > 64 * 1024
    ):
        raise ResultError(
            "staged result must be a private, single-link regular file"
        )
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ResultError(f"staged result is not valid JSON: {error}") from error
    if not isinstance(payload, dict):
        raise ResultError("staged result root must be an object")
    return payload


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ResultError(f"{label} must be a finite number")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ResultError(f"{label} must be a finite number")
    return parsed


def _positive_time_range(payload: Mapping[str, object]) -> None:
    started = payload.get("started_unix_ns")
    finished = payload.get("finished_unix_ns")
    now = time.time_ns()
    if (
        isinstance(started, bool)
        or isinstance(finished, bool)
        or not isinstance(started, int)
        or not isinstance(finished, int)
        or started <= 0
        or finished < started
        or finished > now
    ):
        raise ResultError("staged result timestamps are invalid")


def validate_action_result(
    payload: Mapping[str, object],
    *,
    session_id: str,
    pair_id: str,
    map_id: str,
    map_generation: int,
    target_id: str,
) -> str:
    exact_keys = {
        "schema",
        "status",
        "session_id",
        "pair_id",
        "map_id",
        "map_generation",
        "target_id",
        "goal_uuid",
        "action",
        "goal_accepted",
        "action_status_code",
        "action_status_name",
        "cancel_requested",
        "cancel_confirmed",
        "started_unix_ns",
        "finished_unix_ns",
        "error",
    }
    if set(payload) != exact_keys:
        raise ResultError("action result keys do not match the exact schema")
    expected = {
        "schema": ACTION_RESULT_SCHEMA,
        "status": "PASS",
        "session_id": session_id,
        "pair_id": pair_id,
        "map_id": map_id,
        "map_generation": map_generation,
        "target_id": target_id,
        "action": NAV_ACTION,
        "action_status_name": "succeeded",
        "error": "",
    }
    if (
        any(payload.get(key) != value for key, value in expected.items())
        or isinstance(payload.get("map_generation"), bool)
        or not isinstance(payload.get("map_generation"), int)
        or payload.get("map_generation") != map_generation
        or payload.get("goal_accepted") is not True
        or isinstance(payload.get("action_status_code"), bool)
        or not isinstance(payload.get("action_status_code"), int)
        or payload.get("action_status_code") != SUCCEEDED_STATUS_CODE
        or payload.get("cancel_requested") is not False
        or payload.get("cancel_confirmed") is not False
    ):
        raise ResultError("action result is not an exact successful goal")
    goal_uuid = payload.get("goal_uuid")
    if not isinstance(goal_uuid, str) or _GOAL_UUID_RE.fullmatch(goal_uuid) is None:
        raise ResultError("action result goal UUID is malformed")
    _positive_time_range(payload)
    return goal_uuid


def _pose(payload: object, label: str) -> tuple[float, float, float]:
    if not isinstance(payload, dict) or set(payload) != {"x", "y", "yaw"}:
        raise ResultError(f"{label} pose keys do not match the exact schema")
    pose = (
        _finite(payload["x"], f"{label}.x"),
        _finite(payload["y"], f"{label}.y"),
        _finite(payload["yaw"], f"{label}.yaw"),
    )
    if not -math.pi <= pose[2] <= math.pi:
        raise ResultError(f"{label}.yaw must be within [-pi, pi]")
    return pose


def validate_measured_result(
    payload: Mapping[str, object],
    environ: Mapping[str, str],
    action_path: Path,
) -> dict[str, float]:
    exact_keys = {
        "schema",
        "status",
        "session_id",
        "pair_id",
        "map",
        "target",
        "goal_uuid",
        "started_unix_ns",
        "finished_unix_ns",
        "action",
        "measurement",
        "stop_sequence",
        "checks",
        "failure_reason",
    }
    if set(payload) != exact_keys:
        raise ResultError("measured result keys do not match the exact schema")
    if (
        payload.get("schema") != MEASURED_RESULT_SCHEMA
        or payload.get("status") != "PASS"
        or payload.get("failure_reason") != ""
    ):
        raise ResultError("measured result is not PASS")
    session_id = environ.get("GO2_STAGED_NAV2_SESSION_ID", "")
    pair_id = environ.get("GO2_STAGED_NAV2_PAIR_ID", "")
    map_id = environ.get("GO2_STAGED_NAV2_MAP_ID", "")
    target_id = environ.get("GO2_STAGED_NAV2_TARGET_ID", "")
    try:
        map_generation = int(
            environ.get("GO2_STAGED_NAV2_MAP_GENERATION", ""), 10
        )
    except ValueError as error:
        raise ResultError("expected map generation is malformed") from error
    if (
        payload.get("session_id") != session_id
        or payload.get("pair_id") != pair_id
    ):
        raise ResultError("measured result session or pair does not match")
    map_claim = payload.get("map")
    if (
        not isinstance(map_claim, dict)
        or set(map_claim) != {"id", "generation"}
        or map_claim.get("id") != map_id
        or isinstance(map_claim.get("generation"), bool)
        or not isinstance(map_claim.get("generation"), int)
        or map_claim.get("generation") != map_generation
    ):
        raise ResultError("measured result map does not match")
    target = payload.get("target")
    if (
        not isinstance(target, dict)
        or set(target) != {"id", "pose"}
        or target.get("id") != target_id
    ):
        raise ResultError("measured result target does not match")
    try:
        expected_target = tuple(
            float(environ[key])
            for key in (
                "GO2_STAGED_NAV2_EXPECTED_GOAL_X",
                "GO2_STAGED_NAV2_EXPECTED_GOAL_Y",
                "GO2_STAGED_NAV2_EXPECTED_GOAL_YAW",
            )
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ResultError("expected target pose is malformed") from error
    if not all(math.isfinite(value) for value in expected_target):
        raise ResultError("expected target pose is malformed")
    target_pose = _pose(target["pose"], "target")
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9)
        for actual, expected in zip(target_pose, expected_target)
    ):
        raise ResultError("measured result target pose does not match")

    action_payload = read_private_json(action_path)
    goal_uuid = validate_action_result(
        action_payload,
        session_id=session_id,
        pair_id=pair_id,
        map_id=map_id,
        map_generation=map_generation,
        target_id=target_id,
    )
    if payload.get("goal_uuid") != goal_uuid:
        raise ResultError("measured result goal UUID does not match action result")
    action = payload.get("action")
    expected_action = {
        "result_file": str(action_path),
        "result_sha256": sha256_file(action_path),
        "goal_accepted": True,
        "status_code": SUCCEEDED_STATUS_CODE,
        "status_name": "succeeded",
    }
    if (
        not isinstance(action, dict)
        or set(action) != set(expected_action)
        or action.get("result_file") != expected_action["result_file"]
        or action.get("result_sha256")
        != expected_action["result_sha256"]
        or action.get("goal_accepted") is not True
        or isinstance(action.get("status_code"), bool)
        or not isinstance(action.get("status_code"), int)
        or action.get("status_code") != SUCCEEDED_STATUS_CODE
        or action.get("status_name") != "succeeded"
    ):
        raise ResultError("measured result action binding does not match")

    measurement = payload.get("measurement")
    if not isinstance(measurement, dict) or set(measurement) != {
        "frame_id",
        "start_pose",
        "end_pose",
        "forward_m",
        "total_m",
        "lateral_m",
        "yaw_change_rad",
    }:
        raise ResultError("measured result measurement keys are incomplete")
    if measurement.get("frame_id") != "odom":
        raise ResultError("measured result frame must be odom")
    start = _pose(measurement["start_pose"], "measurement.start")
    end = _pose(measurement["end_pose"], "measurement.end")
    forward = _finite(measurement["forward_m"], "measurement.forward_m")
    total = _finite(measurement["total_m"], "measurement.total_m")
    lateral = _finite(measurement["lateral_m"], "measurement.lateral_m")
    yaw_change = _finite(
        measurement["yaw_change_rad"], "measurement.yaw_change_rad"
    )
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    expected_forward = delta_x * math.cos(start[2]) + delta_y * math.sin(start[2])
    expected_lateral = -delta_x * math.sin(start[2]) + delta_y * math.cos(start[2])
    expected_yaw = math.atan2(
        math.sin(end[2] - start[2]), math.cos(end[2] - start[2])
    )
    for actual, expected, label in (
        (forward, expected_forward, "forward"),
        (lateral, expected_lateral, "lateral"),
        (yaw_change, expected_yaw, "yaw change"),
    ):
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6):
            raise ResultError(f"measured {label} does not match endpoint poses")
    displacement = math.hypot(delta_x, delta_y)
    if total < displacement - 1e-6 or not 0.0 <= total < MAX_DISTANCE_M:
        raise ResultError("measured total path is outside the stage envelope")

    stop = payload.get("stop_sequence")
    if not isinstance(stop, dict) or set(stop) != {
        "cancel",
        "zero",
        "disarm",
        "post_stop",
    }:
        raise ResultError("measured stop sequence keys are incomplete")
    cancel = stop["cancel"]
    if (
        not isinstance(cancel, dict)
        or set(cancel) != {"required", "requested", "confirmed"}
        or any(not isinstance(cancel[key], bool) for key in cancel)
        or (
            cancel["required"]
            and not (cancel["requested"] and cancel["confirmed"])
        )
    ):
        raise ResultError("cancel evidence is incomplete")
    zero = stop["zero"]
    if (
        not isinstance(zero, dict)
        or set(zero) != {"published_count", "confirmed_zero"}
        or isinstance(zero["published_count"], bool)
        or not isinstance(zero["published_count"], int)
        or zero["published_count"] < 10
        or zero["confirmed_zero"] is not True
    ):
        raise ResultError("zero-command evidence is incomplete")
    disarm = stop["disarm"]
    expected_disarm = {
        "requested": True,
        "service_success": True,
        "measured_disarmed": True,
    }
    if (
        not isinstance(disarm, dict)
        or set(disarm) != set(expected_disarm)
        or any(not isinstance(disarm[key], bool) for key in disarm)
        or disarm != expected_disarm
    ):
        raise ResultError("disarm evidence is incomplete")
    post_stop = stop["post_stop"]
    if not isinstance(post_stop, dict) or set(post_stop) != {
        "observation_s",
        "odom_samples",
        "max_drift_m",
        "max_linear_speed_mps",
        "max_yaw_rate_rps",
        "limits",
        "passed",
    }:
        raise ResultError("post-stop evidence keys are incomplete")
    limits = post_stop["limits"]
    if limits != {
        "max_drift_m": POST_STOP_MAX_DRIFT_M,
        "max_linear_speed_mps": POST_STOP_MAX_LINEAR_MPS,
        "max_yaw_rate_rps": POST_STOP_MAX_YAW_RPS,
        "min_odom_samples": POST_STOP_MIN_ODOM_SAMPLES,
    }:
        raise ResultError("post-stop limits do not match the staged contract")
    observation = _finite(post_stop["observation_s"], "post_stop.observation_s")
    drift = _finite(post_stop["max_drift_m"], "post_stop.max_drift_m")
    linear = _finite(
        post_stop["max_linear_speed_mps"],
        "post_stop.max_linear_speed_mps",
    )
    yaw_rate = _finite(
        post_stop["max_yaw_rate_rps"], "post_stop.max_yaw_rate_rps"
    )
    samples = post_stop["odom_samples"]
    if (
        observation < POST_STOP_OBSERVATION_S
        or isinstance(samples, bool)
        or not isinstance(samples, int)
        or samples < POST_STOP_MIN_ODOM_SAMPLES
        or drift > POST_STOP_MAX_DRIFT_M
        or linear > POST_STOP_MAX_LINEAR_MPS
        or yaw_rate > POST_STOP_MAX_YAW_RPS
        or post_stop["passed"] is not True
    ):
        raise ResultError("continuous post-stop stationary evidence failed")

    checks = payload.get("checks")
    expected_checks = {
        "action_succeeded",
        "measurement_complete",
        "cancel_complete",
        "zero_complete",
        "disarm_complete",
        "post_stop_stationary",
    }
    if (
        not isinstance(checks, dict)
        or set(checks) != expected_checks
        or any(value != "pass" for value in checks.values())
    ):
        raise ResultError("measured result checks are incomplete or failed")
    _positive_time_range(payload)
    return {
        "forward_m": forward,
        "total_m": total,
        "lateral_m": lateral,
        "yaw_change_rad": yaw_change,
    }
