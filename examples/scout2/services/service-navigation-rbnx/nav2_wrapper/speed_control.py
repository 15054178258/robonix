"""Pure policy for runtime navigation speed-limit commands."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SpeedSettings:
    """Validated deployment policy for dynamic navigation speed limits."""

    max_linear_speed_mps: float
    default_percentage: float
    step_percentage: float = 20.0
    min_percentage: float = 20.0
    topic: str = "/speed_limit"


@dataclass(frozen=True)
class SpeedDecision:
    """Result of one relative or explicit speed command."""

    percentage: float
    linear_speed_mps: float
    changed: bool
    detail: str


def speed_settings(cfg: dict) -> SpeedSettings:
    """Validate the deploy-owned policy without importing ROS dependencies."""
    raw = cfg.get("dynamic_speed")
    if not isinstance(raw, dict):
        raise ValueError(
            "dynamic_speed must be a mapping with max_linear_speed_mps and "
            "default_percentage"
        )
    allowed = {
        "max_linear_speed_mps",
        "default_percentage",
        "step_percentage",
        "min_percentage",
        "topic",
    }
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"unknown dynamic_speed field(s): {sorted(unknown)}")

    missing = {
        "max_linear_speed_mps",
        "default_percentage",
    } - set(raw)
    if missing:
        raise ValueError(f"missing dynamic_speed field(s): {sorted(missing)}")

    settings = SpeedSettings(
        max_linear_speed_mps=float(raw["max_linear_speed_mps"]),
        default_percentage=float(raw["default_percentage"]),
        step_percentage=float(raw.get("step_percentage", 20.0)),
        min_percentage=float(raw.get("min_percentage", 20.0)),
        topic=str(raw.get("topic", "/speed_limit")).strip(),
    )
    values = (
        settings.max_linear_speed_mps,
        settings.default_percentage,
        settings.step_percentage,
        settings.min_percentage,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("dynamic_speed numeric fields must be finite")
    if settings.max_linear_speed_mps <= 0.0:
        raise ValueError(
            "dynamic_speed max_linear_speed_mps must be greater than zero"
        )
    if not 0.0 < settings.min_percentage <= settings.default_percentage <= 100.0:
        raise ValueError(
            "dynamic_speed requires 0 < min_percentage <= "
            "default_percentage <= 100"
        )
    if not 0.0 < settings.step_percentage <= 100.0:
        raise ValueError("dynamic_speed step_percentage must be in (0, 100]")
    if not settings.topic.startswith("/"):
        raise ValueError("dynamic_speed topic must be an absolute ROS topic")
    return settings


def _decision(
    current_percentage: float,
    target_percentage: float,
    settings: SpeedSettings,
    action: str,
) -> SpeedDecision:
    """Build one bounded decision and its deploy-configured metric limit."""
    target = round(float(target_percentage), 6)
    changed = not math.isclose(
        target, current_percentage, rel_tol=0.0, abs_tol=1e-9
    )
    linear_speed_mps = round(
        settings.max_linear_speed_mps * target / 100.0,
        6,
    )
    state = "changed" if changed else "already"
    detail = (
        f"navigation speed {state} at {target:g}% "
        f"({linear_speed_mps:g} m/s linear limit) after {action}"
    )
    return SpeedDecision(
        target,
        linear_speed_mps,
        changed,
        detail,
    )


def decide_adjustment(
    current_percentage: float,
    direction: str,
    settings: SpeedSettings,
) -> SpeedDecision:
    """Resolve faster, slower, or normal within the configured bounds."""
    command = direction.strip().lower()
    if command == "faster":
        target = min(100.0, current_percentage + settings.step_percentage)
    elif command == "slower":
        target = max(
            settings.min_percentage,
            current_percentage - settings.step_percentage,
        )
    elif command == "normal":
        target = settings.default_percentage
    else:
        raise ValueError("direction must be faster, slower, or normal")
    return _decision(current_percentage, target, settings, command)


def decide_explicit(
    current_percentage: float,
    requested_percentage: float,
    settings: SpeedSettings,
) -> SpeedDecision:
    """Resolve an explicit percentage within the configured safety bounds."""
    if not math.isfinite(requested_percentage):
        raise ValueError("percentage must be finite")
    if not settings.min_percentage <= requested_percentage <= 100.0:
        raise ValueError(
            "percentage must be between "
            f"{settings.min_percentage:g} and 100"
        )
    return _decision(
        current_percentage,
        requested_percentage,
        settings,
        "explicit set",
    )
