#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Verify visibility-aware Scene dropout with a real Webots object move."""

from __future__ import annotations

import argparse
import json
import math
import os
import time
import urllib.request
from typing import Any, Callable

from webots_object_control import send_object_control


def _state(base_url: str) -> dict[str, Any]:
    with urllib.request.urlopen(
        base_url.rstrip("/") + "/api/state",
        timeout=10,
    ) as response:
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise RuntimeError("/api/state did not return a JSON object")
    return payload


def _distance(object_: dict[str, Any], center: tuple[float, float, float]) -> float:
    pose = object_.get("pose") or {}
    return math.sqrt(
        sum(
            (float(pose[axis]) - expected) ** 2
            for axis, expected in zip(("x", "y", "z"), center)
        )
    )


def _find(
    state: dict[str, Any],
    *,
    object_id: str | None = None,
    expected_classes: set[str] | None = None,
    expected_center: tuple[float, float, float] | None = None,
) -> dict[str, Any] | None:
    candidates = [
        obj
        for obj in state.get("objects") or ()
        if isinstance(obj, dict) and obj.get("cls") != "robot"
    ]
    if object_id is not None:
        return next(
            (obj for obj in candidates if obj.get("id") == object_id),
            None,
        )
    if expected_classes:
        candidates = [
            obj
            for obj in candidates
            if str(obj.get("cls") or "") in expected_classes
        ]
    if not candidates:
        return None
    if expected_center is None:
        return max(
            candidates,
            key=lambda obj: int(obj.get("observation_count") or 0),
        )
    return min(candidates, key=lambda obj: _distance(obj, expected_center))


def _wait(
    base_url: str,
    predicate,
    timeout_s: float,
    description: str,
    *,
    phase: str,
    trace: list[dict[str, Any]],
    trace_selector: Callable[[dict[str, Any]], dict[str, Any] | None],
    checkpoint: Callable[[str], None],
) -> tuple[dict[str, Any], dict[str, Any]]:
    deadline = time.monotonic() + timeout_s
    latest_state: dict[str, Any] = {}
    latest_object: dict[str, Any] | None = None
    last_trace_signature: tuple[Any, ...] | None = None
    while time.monotonic() < deadline:
        latest_state = _state(base_url)
        traced_object = trace_selector(latest_state)
        visibility_debug = (
            traced_object.get("visibility_debug") or {}
            if traced_object
            else {}
        )
        signature = (
            traced_object.get("id") if traced_object else None,
            traced_object.get("observation_count") if traced_object else None,
            traced_object.get("consecutive_visible_misses")
            if traced_object
            else None,
            traced_object.get("missing") if traced_object else None,
            visibility_debug.get("status"),
            visibility_debug.get("valid_samples"),
            visibility_debug.get("clear_samples"),
            visibility_debug.get("clear_fraction"),
            visibility_debug.get("depth_delta_median_m"),
        )
        if signature != last_trace_signature:
            trace.append(
                {
                    "elapsed_s": round(time.monotonic() - (deadline - timeout_s), 3),
                    "phase": phase,
                    "object": _trace_object(traced_object),
                }
            )
            last_trace_signature = signature
            checkpoint(phase)
        latest_object = predicate(latest_state)
        if latest_object is not None:
            return latest_state, latest_object
        time.sleep(0.25)
    raise RuntimeError(
        f"timed out waiting for {description}; last object={latest_object!r}"
    )


def _trace_object(object_: dict[str, Any] | None) -> dict[str, Any] | None:
    if object_ is None:
        return None
    return {
        "id": str(object_.get("id") or ""),
        "cls": str(object_.get("cls") or ""),
        "pose": object_.get("pose"),
        "observation_count": int(object_.get("observation_count") or 0),
        "consecutive_visible_misses": int(
            object_.get("consecutive_visible_misses") or 0
        ),
        "visibility_debug": object_.get("visibility_debug") or {},
        "missing": bool(object_.get("missing")),
        "last_observed_unix": object_.get("last_observed_unix"),
    }


def _write_json_atomic(path: str, payload: dict[str, Any]) -> None:
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def exercise(args) -> dict[str, Any]:
    center = tuple(float(value) for value in args.expected_center)
    requested_classes = {
        item.strip()
        for value in args.expected_class
        for item in str(value).split(",")
        if item.strip()
    }
    if not requested_classes:
        raise ValueError("--expected-class must name at least one class")
    expected_classes = (
        set() if "*" in requested_classes else requested_classes
    )
    trace: list[dict[str, Any]] = []
    staging_ack: dict[str, Any] | None = None
    cleanup_ack: dict[str, Any] | None = None
    cleanup_translation: list[float] | None = None
    succeeded = False
    context: dict[str, Any] = {
        "ok": False,
        "status": "starting",
        "base_url": args.base_url,
        "target_name": args.target_name,
        "expected_classes": sorted(requested_classes),
        "expected_center": list(center),
        "timeline": trace,
    }

    def checkpoint(status: str) -> None:
        context["status"] = status
        context["timeline"] = trace
        if args.output:
            _write_json_atomic(args.output, context)

    checkpoint("baseline")
    try:
        baseline_state = _state(args.base_url)
        baseline_counts = {
            str(obj.get("id") or ""): int(obj.get("observation_count") or 0)
            for obj in baseline_state.get("objects") or ()
            if isinstance(obj, dict) and obj.get("id")
        }
        if args.staged_translation is not None:
            print("phase=stage_object", flush=True)
            staging_ack = send_object_control(
                target_name=args.target_name,
                translation=args.staged_translation,
                timeout_s=args.control_timeout_s,
            )
            if not staging_ack.get("ok"):
                raise RuntimeError(f"Webots staging move failed: {staging_ack}")
            context["staging_ack"] = staging_ack
            checkpoint("staged")
            previous = staging_ack.get("previous_translation")
            if (
                isinstance(previous, list)
                and len(previous) == 3
                and all(isinstance(value, (int, float)) for value in previous)
            ):
                cleanup_translation = [float(value) for value in previous]

        def nearest_selector(state: dict[str, Any]) -> dict[str, Any] | None:
            return _find(
                state,
                expected_classes=expected_classes,
                expected_center=center,
            )

        def target_selector(state: dict[str, Any]) -> dict[str, Any] | None:
            # A pre-existing track is not proof that the staged Webots object
            # generated it. Require N fresh positive observations after the
            # staging command; a newly created track has a baseline of zero.
            eligible = []
            for obj in state.get("objects") or ():
                if not isinstance(obj, dict):
                    continue
                object_id = str(obj.get("id") or "")
                observation_delta = int(
                    obj.get("observation_count") or 0
                ) - baseline_counts.get(object_id, 0)
                if observation_delta >= args.min_observations:
                    eligible.append(obj)
            return _find(
                {**state, "objects": eligible},
                expected_classes=expected_classes,
                expected_center=center,
            )

        print(
            "phase=wait_stable classes="
            + ("*" if not expected_classes else ",".join(sorted(expected_classes))),
            flush=True,
        )
        _, before = _wait(
            args.base_url,
            lambda state: (
                candidate
                if (
                    (candidate := target_selector(state))
                    is not None
                    and not candidate.get("missing")
                    and _distance(candidate, center)
                    <= args.max_center_distance_m
                )
                else None
            ),
            args.wait_s,
            "a stable target Scene object",
            phase="observe_staged",
            trace=trace,
            trace_selector=nearest_selector,
            checkpoint=checkpoint,
        )
        object_id = str(before["id"])
        before_count = int(before.get("observation_count") or 0)
        context.update(
            {
                "scene_object_id": object_id,
                "baseline_observation_count": baseline_counts.get(object_id, 0),
                "before": before,
            }
        )
        checkpoint("stable_target")
        tracked_selector = lambda state: _find(state, object_id=object_id)

        move_ack = send_object_control(
            target_name=args.target_name,
            translation=args.removed_translation,
            timeout_s=args.control_timeout_s,
        )
        if not move_ack.get("ok"):
            raise RuntimeError(f"Webots removal move failed: {move_ack}")
        context["move_ack"] = move_ack
        checkpoint("object_removed")

        print(f"phase=wait_missing object_id={object_id}", flush=True)
        _, missing = _wait(
            args.base_url,
            lambda state: (
                candidate
                if (
                    (candidate := tracked_selector(state)) is not None
                    and candidate.get("missing") is True
                    and int(candidate.get("consecutive_visible_misses") or 0)
                    >= args.visible_miss_threshold
                )
                else None
            ),
            args.dropout_timeout_s,
            "visibility-aware missing state",
            phase="removed",
            trace=trace,
            trace_selector=tracked_selector,
            checkpoint=checkpoint,
        )
        context["missing"] = missing
        checkpoint("missing")

        restore_ack = send_object_control(
            target_name=args.target_name,
            translation=args.restored_translation,
            timeout_s=args.control_timeout_s,
        )
        if not restore_ack.get("ok"):
            raise RuntimeError(f"Webots restore failed: {restore_ack}")
        context["restore_ack"] = restore_ack
        checkpoint("object_restored")

        print(f"phase=wait_restored object_id={object_id}", flush=True)
        _, restored = _wait(
            args.base_url,
            lambda state: (
                candidate
                if (
                    (candidate := tracked_selector(state)) is not None
                    and candidate.get("missing") is False
                    and int(candidate.get("observation_count") or 0)
                    > before_count
                    and int(candidate.get("consecutive_visible_misses") or 0)
                    == 0
                )
                else None
            ),
            args.restore_timeout_s,
            "same-ID re-observation after restore",
            phase="restored",
            trace=trace,
            trace_selector=tracked_selector,
            checkpoint=checkpoint,
        )
        context["restored"] = restored
        succeeded = True
    finally:
        if cleanup_translation is not None:
            cleanup_ack = send_object_control(
                target_name=args.target_name,
                translation=cleanup_translation,
                timeout_s=args.control_timeout_s,
            )
            context["cleanup_ack"] = cleanup_ack
            checkpoint("cleanup")

    if not succeeded:
        raise RuntimeError("dropout exercise did not reach its success state")
    result = {
        "ok": True,
        "status": "passed",
        "base_url": args.base_url,
        "target_name": args.target_name,
        "scene_object_id": object_id,
        "expected_classes": sorted(requested_classes),
        "baseline_observation_count": baseline_counts.get(object_id, 0),
        "staging_ack": staging_ack,
        "before": before,
        "move_ack": move_ack,
        "missing": missing,
        "restore_ack": restore_ack,
        "restored": restored,
        "cleanup_ack": cleanup_ack,
        "timeline": trace,
        "checks": {
            "negative_evidence_threshold_reached": True,
            "missing_marked": True,
            "stable_id_rebound": str(restored["id"]) == object_id,
            "observation_count_increased": (
                int(restored.get("observation_count") or 0) > before_count
            ),
            "visible_miss_counter_reset": (
                int(restored.get("consecutive_visible_misses") or 0) == 0
            ),
        },
    }
    if args.output:
        _write_json_atomic(args.output, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-url",
        default="http://127.0.0.1:50107",
    )
    parser.add_argument("--target-name", required=True)
    parser.add_argument(
        "--expected-class",
        action="append",
        required=True,
        help=(
            "accepted detector class for locating the staged target; repeat "
            "the flag or use a comma-separated list; '*' selects by geometry "
            "and post-staging observations only"
        ),
    )
    parser.add_argument(
        "--expected-center",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--removed-translation",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--restored-translation",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--staged-translation",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help=(
            "optional deterministic test placement applied before waiting for "
            "the target; the original Webots translation is restored at exit"
        ),
    )
    parser.add_argument("--min-observations", type=int, default=3)
    parser.add_argument("--visible-miss-threshold", type=int, default=3)
    parser.add_argument("--max-center-distance-m", type=float, default=0.8)
    parser.add_argument("--wait-s", type=float, default=180.0)
    parser.add_argument("--dropout-timeout-s", type=float, default=30.0)
    parser.add_argument("--restore-timeout-s", type=float, default=30.0)
    parser.add_argument("--control-timeout-s", type=float, default=10.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    try:
        result = exercise(args)
    except Exception as exc:  # noqa: BLE001
        result: dict[str, Any] = {}
        if args.output:
            try:
                with open(args.output, encoding="utf-8") as stream:
                    value = json.load(stream)
                if isinstance(value, dict):
                    result = value
            except (OSError, ValueError):
                pass
        result.update(
            {
                "ok": False,
                "status": "failed",
                "error": f"{type(exc).__name__}: {exc}",
            }
        )
        if args.output:
            _write_json_atomic(args.output, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
