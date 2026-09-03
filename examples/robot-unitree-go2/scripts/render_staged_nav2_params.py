#!/usr/bin/env python3
"""Render the stage-1-only Nav2 parameter file.

The shared Go2 Nav2 parameters remain suitable for motion-disabled Mapping and
Navigation inspection.  This renderer monotonically tightens the private
stage-1 materialization to the final motion guard's velocity and acceleration
envelope.  The physical profile keeps the shared 0.35 m XY goal tolerance; it
does not add a local short-goal completion gate.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go2_chassis"))

from go2_chassis import runtime_config  # noqa: E402


STAGE1_XY_GOAL_TOLERANCE_M = 0.35
FOOTPRINT_TOKEN = "__ROBONIX_FOOTPRINT__"
GOAL_TOLERANCE_PATH = (
    "controller_server",
    "ros__parameters",
    "go2_goal_checker",
    "xy_goal_tolerance",
)
ROTATE_GOAL_TOLERANCE_PATH = (
    "controller_server",
    "ros__parameters",
    "FollowPath",
    "xy_goal_tolerance",
)
FOLLOW_PATH = (
    "controller_server",
    "ros__parameters",
    "FollowPath",
)
VELOCITY_SMOOTHER = (
    "velocity_smoother",
    "ros__parameters",
)
CONTROLLER_ZERO_PATHS = frozenset(
    {
        (*FOLLOW_PATH, "min_vel_y"),
        (*FOLLOW_PATH, "max_vel_y"),
        (*FOLLOW_PATH, "acc_lim_y"),
        (*FOLLOW_PATH, "decel_lim_y"),
    }
)
CONTROLLER_UPPER_LIMITS = {
    (*FOLLOW_PATH, "max_vel_x"): runtime_config.STAGED_NAV2_MAX_VX_MPS,
    (*FOLLOW_PATH, "max_vel_theta"): runtime_config.STAGED_NAV2_MAX_WZ_RPS,
    (*FOLLOW_PATH, "max_speed_xy"): runtime_config.STAGED_NAV2_MAX_VX_MPS,
    (*FOLLOW_PATH, "min_speed_theta"): runtime_config.STAGED_NAV2_MAX_WZ_RPS,
    (*FOLLOW_PATH, "acc_lim_x"): (
        runtime_config.STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2
    ),
    (*FOLLOW_PATH, "acc_lim_theta"): (
        runtime_config.STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2
    ),
}
CONTROLLER_NEGATIVE_MAGNITUDE_LIMITS = {
    (*FOLLOW_PATH, "decel_lim_x"): (
        runtime_config.STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2
    ),
    (*FOLLOW_PATH, "decel_lim_theta"): (
        runtime_config.STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2
    ),
}
SMOOTHER_VECTOR_PATHS = frozenset(
    {
        (*VELOCITY_SMOOTHER, "max_velocity"),
        (*VELOCITY_SMOOTHER, "min_velocity"),
        (*VELOCITY_SMOOTHER, "max_accel"),
        (*VELOCITY_SMOOTHER, "max_decel"),
    }
)
STAGE1_ALLOWED_CHANGE_PATHS = frozenset(
    {
        GOAL_TOLERANCE_PATH,
        ROTATE_GOAL_TOLERANCE_PATH,
        *CONTROLLER_ZERO_PATHS,
        *CONTROLLER_UPPER_LIMITS,
        *CONTROLLER_NEGATIVE_MAGNITUDE_LIMITS,
        *SMOOTHER_VECTOR_PATHS,
    }
)


class Stage1ParamsError(ValueError):
    """The shared parameter file cannot be safely specialized."""


class _TokenSafeDumper(yaml.SafeDumper):
    """Keep type-sensitive runtime substitution tokens quoted."""


def _represent_string(
    dumper: yaml.SafeDumper, value: str
) -> yaml.nodes.ScalarNode:
    # Soma resolves the footprint to text such as ``[ [x, y], ... ]``.  ROS 2
    # declares the costmap footprint parameter as a string; emitting this
    # token unquoted makes the later substitution a YAML sequence and rclcpp
    # rejects the entire parameter file before Nav2 starts.
    style = '"' if value == FOOTPRINT_TOKEN else None
    return dumper.represent_scalar(
        "tag:yaml.org,2002:str", value, style=style
    )


_TokenSafeDumper.add_representer(str, _represent_string)


def _mapping_at(document: dict[str, Any], path: tuple[str, ...]) -> dict[str, Any]:
    current: Any = document
    for key in path:
        if not isinstance(current, dict) or key not in current:
            raise Stage1ParamsError(
                "shared Nav2 parameters are missing " + ".".join(path)
            )
        current = current[key]
    if not isinstance(current, dict):
        raise Stage1ParamsError(
            "shared Nav2 parameter parent is not a mapping: "
            + ".".join(path)
        )
    return current


def _changed_paths(
    before: Any,
    after: Any,
    prefix: tuple[str, ...] = (),
) -> set[tuple[str, ...]]:
    if isinstance(before, dict) and isinstance(after, dict):
        changed: set[tuple[str, ...]] = set()
        for key in set(before) | set(after):
            if key not in before or key not in after:
                changed.add((*prefix, str(key)))
            else:
                changed.update(
                    _changed_paths(before[key], after[key], (*prefix, str(key)))
                )
        return changed
    return set() if before == after else {prefix}


def _number_at(
    document: dict[str, Any],
    path: tuple[str, ...],
) -> tuple[dict[str, Any], str, float]:
    parent = _mapping_at(document, path[:-1])
    key = path[-1]
    if key not in parent:
        raise Stage1ParamsError(
            "shared Nav2 parameters are missing " + ".".join(path)
        )
    value = parent[key]
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise Stage1ParamsError(
            "shared Nav2 parameter must be a finite number: "
            + ".".join(path)
        )
    return parent, key, float(value)


def _vector_at(
    document: dict[str, Any],
    path: tuple[str, ...],
) -> tuple[dict[str, Any], str, list[float]]:
    parent = _mapping_at(document, path[:-1])
    key = path[-1]
    if key not in parent:
        raise Stage1ParamsError(
            "shared Nav2 parameters are missing " + ".".join(path)
        )
    value = parent[key]
    if not isinstance(value, list) or len(value) != 3:
        raise Stage1ParamsError(
            "shared Nav2 parameter must be a three-number vector: "
            + ".".join(path)
        )
    numbers: list[float] = []
    for item in value:
        if (
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(float(item))
        ):
            raise Stage1ParamsError(
                "shared Nav2 parameter must be a three-number vector: "
                + ".".join(path)
            )
        numbers.append(float(item))
    return parent, key, numbers


def _tighten_positive_limit(
    document: dict[str, Any],
    path: tuple[str, ...],
    limit: float,
) -> None:
    parent, key, current = _number_at(document, path)
    if current < 0.0:
        raise Stage1ParamsError(
            "shared Nav2 positive limit is negative: " + ".".join(path)
        )
    parent[key] = min(current, limit)


def _tighten_negative_magnitude_limit(
    document: dict[str, Any],
    path: tuple[str, ...],
    limit: float,
) -> None:
    parent, key, current = _number_at(document, path)
    if current > 0.0:
        raise Stage1ParamsError(
            "shared Nav2 deceleration limit is positive: " + ".".join(path)
        )
    parent[key] = max(current, -limit)


def _set_zero(
    document: dict[str, Any],
    path: tuple[str, ...],
) -> None:
    parent, key, _ = _number_at(document, path)
    parent[key] = 0.0


def _tighten_controller_relationships(document: dict[str, Any]) -> None:
    _, _, max_linear = _number_at(
        document, (*FOLLOW_PATH, "max_vel_x")
    )
    parent, key, max_speed = _number_at(
        document, (*FOLLOW_PATH, "max_speed_xy")
    )
    parent[key] = min(max_speed, max_linear)

    _, _, max_angular = _number_at(
        document, (*FOLLOW_PATH, "max_vel_theta")
    )
    parent, key, min_angular = _number_at(
        document, (*FOLLOW_PATH, "min_speed_theta")
    )
    parent[key] = min(min_angular, max_angular)


def _tighten_smoother_vectors(document: dict[str, Any]) -> None:
    _, _, controller_max_linear = _number_at(
        document, (*FOLLOW_PATH, "max_vel_x")
    )
    _, _, controller_max_speed = _number_at(
        document, (*FOLLOW_PATH, "max_speed_xy")
    )
    _, _, controller_max_angular = _number_at(
        document, (*FOLLOW_PATH, "max_vel_theta")
    )
    _, _, controller_linear_accel = _number_at(
        document, (*FOLLOW_PATH, "acc_lim_x")
    )
    _, _, controller_angular_accel = _number_at(
        document, (*FOLLOW_PATH, "acc_lim_theta")
    )
    _, _, controller_linear_decel = _number_at(
        document, (*FOLLOW_PATH, "decel_lim_x")
    )
    _, _, controller_angular_decel = _number_at(
        document, (*FOLLOW_PATH, "decel_lim_theta")
    )
    linear_speed = min(
        runtime_config.STAGED_NAV2_MAX_VX_MPS,
        controller_max_linear,
        controller_max_speed,
    )
    angular_speed = min(
        runtime_config.STAGED_NAV2_MAX_WZ_RPS,
        controller_max_angular,
    )
    linear_accel = min(
        runtime_config.STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
        controller_linear_accel,
    )
    angular_accel = min(
        runtime_config.STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
        controller_angular_accel,
    )
    linear_decel = min(
        runtime_config.STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
        abs(controller_linear_decel),
    )
    angular_decel = min(
        runtime_config.STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
        abs(controller_angular_decel),
    )

    parent, key, current = _vector_at(
        document, (*VELOCITY_SMOOTHER, "max_velocity")
    )
    if current[0] < 0.0 or current[2] < 0.0:
        raise Stage1ParamsError("velocity_smoother.max_velocity is negative")
    parent[key] = [
        min(current[0], linear_speed),
        0.0,
        min(current[2], angular_speed),
    ]

    parent, key, current = _vector_at(
        document, (*VELOCITY_SMOOTHER, "min_velocity")
    )
    if current[0] > 0.0 or current[2] > 0.0:
        raise Stage1ParamsError(
            "velocity_smoother.min_velocity has the wrong sign"
        )
    parent[key] = [0.0, 0.0, max(current[2], -angular_speed)]

    parent, key, current = _vector_at(
        document, (*VELOCITY_SMOOTHER, "max_accel")
    )
    if current[0] < 0.0 or current[2] < 0.0:
        raise Stage1ParamsError("velocity_smoother.max_accel is negative")
    parent[key] = [
        min(current[0], linear_accel),
        0.0,
        min(current[2], angular_accel),
    ]

    parent, key, current = _vector_at(
        document, (*VELOCITY_SMOOTHER, "max_decel")
    )
    if current[0] > 0.0 or current[2] > 0.0:
        raise Stage1ParamsError(
            "velocity_smoother.max_decel has the wrong sign"
        )
    parent[key] = [
        max(current[0], -linear_decel),
        0.0,
        max(current[2], -angular_decel),
    ]


def render_stage1_params(document: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(document, dict):
        raise Stage1ParamsError("shared Nav2 parameters must be a mapping")
    rendered = deepcopy(document)
    parent, key, current = _number_at(rendered, GOAL_TOLERANCE_PATH)
    if current < STAGE1_XY_GOAL_TOLERANCE_M:
        raise Stage1ParamsError(
            "stage-1 rendering must not expand an already tighter tolerance"
        )
    parent[key] = STAGE1_XY_GOAL_TOLERANCE_M
    parent, key, current = _number_at(
        rendered, ROTATE_GOAL_TOLERANCE_PATH
    )
    if current < STAGE1_XY_GOAL_TOLERANCE_M:
        raise Stage1ParamsError(
            "stage-1 rendering must not expand an already tighter "
            "RotateToGoal tolerance"
        )
    parent[key] = STAGE1_XY_GOAL_TOLERANCE_M
    for path in CONTROLLER_ZERO_PATHS:
        _set_zero(rendered, path)
    for path, limit in CONTROLLER_UPPER_LIMITS.items():
        _tighten_positive_limit(rendered, path, limit)
    for path, limit in CONTROLLER_NEGATIVE_MAGNITUDE_LIMITS.items():
        _tighten_negative_magnitude_limit(rendered, path, limit)
    _tighten_controller_relationships(rendered)
    _tighten_smoother_vectors(rendered)
    changed = _changed_paths(document, rendered)
    if not changed <= STAGE1_ALLOWED_CHANGE_PATHS:
        raise Stage1ParamsError(
            "stage-1 rendering changed fields outside the staged envelope"
        )
    return rendered


def write_private_yaml(path: Path, document: dict[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise Stage1ParamsError(f"refusing to overwrite stage params: {path}")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as handle:
        yaml.dump(
            document,
            handle,
            Dumper=_TokenSafeDumper,
            sort_keys=False,
        )
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    temporary.replace(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.source.resolve()
    output = Path(os.path.abspath(os.fspath(args.output)))
    if not source.is_file() or source.is_symlink():
        raise SystemExit("shared Nav2 parameter source must be a regular file")
    if not output.is_absolute():
        raise SystemExit("stage Nav2 parameter output must be absolute")
    with source.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    try:
        rendered = render_stage1_params(document)
        write_private_yaml(output, rendered)
    except Stage1ParamsError as error:
        raise SystemExit(str(error)) from error
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
