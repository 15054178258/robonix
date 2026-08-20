"""Bounded, transport-safe diagnostics for Nav2 Humble action results."""

from __future__ import annotations

import re


_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
_SIGNALS = (
    "failed to make progress",
    "controller patience exceeded",
    "resulting plan has 0 poses",
    "received plan with zero length",
    "no valid trajectories",
    "failed to create plan",
    "collision ahead",
    "rotation shim controller",
    "extrapolation error",
    "transform timeout",
    "timed out waiting for transform",
    "spin failed",
)


def classify_nav2_line(line: str) -> str:
    """Return one concise actionable line, or empty for routine Nav2 output."""
    clean = _ANSI.sub("", line).strip()
    lowered = clean.lower()
    if not any(signal in lowered for signal in _SIGNALS):
        return ""
    return clean[-240:]


def format_result_detail(
    status: str,
    feedback: dict[str, object] | None,
    diagnostics: list[str],
) -> str:
    """Describe an action result without inventing unavailable Humble codes."""
    parts = [status.lower()]
    if feedback:
        parts.append(
            "distance_remaining={:.3f}m recoveries={} last_pose=({:.3f},{:.3f})".format(
                float(feedback.get("distance_remaining", 0.0)),
                int(feedback.get("recoveries", 0)),
                float(feedback.get("x", 0.0)),
                float(feedback.get("y", 0.0)),
            )
        )
    unique: list[str] = []
    for item in diagnostics:
        if item and item not in unique:
            unique.append(item)
    if unique:
        parts.append("nav2=" + " | ".join(unique[-3:]))
    return "; ".join(parts)
