#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Exercise Scene object corrections against a live Web UI API.

The verifier intentionally mutates only derived, non-robot objects in an
ephemeral benchmark run.  It proves that operator label and geometry edits,
single-object deletion, and a runtime flush are reflected by subsequent
``/api/state`` reads.  Saved semantic snapshots are never modified.
"""

from __future__ import annotations

import argparse
import json
import math
import time
import urllib.error
import urllib.request
from typing import Any


def _request(
    base_url: str,
    method: str,
    path: str,
    body: dict[str, Any] | None = None,
) -> dict[str, Any]:
    data = None
    headers: dict[str, str] = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(
        base_url.rstrip("/") + path,
        data=data,
        headers=headers,
        method=method,
    )
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            payload = json.load(response)
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", "replace")
        raise RuntimeError(
            f"{method} {path} returned HTTP {exc.code}: {detail}"
        ) from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"{method} {path} did not return a JSON object")
    return payload


def _derived_objects(state: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        obj
        for obj in state.get("objects") or ()
        if isinstance(obj, dict)
        and not bool(obj.get("is_robot"))
        and obj.get("cls") != "robot"
        and obj.get("id")
    ]


def _find_object(
    state: dict[str, Any],
    object_id: str,
) -> dict[str, Any] | None:
    return next(
        (
            obj
            for obj in state.get("objects") or ()
            if isinstance(obj, dict) and obj.get("id") == object_id
        ),
        None,
    )


def _epoch(state: dict[str, Any]) -> dict[str, Any]:
    binding = state.get("map_binding") or {}
    generation = binding.get("generation")
    return {
        "expected_map_id": str(binding.get("map_id") or ""),
        "expected_generation": -1 if generation is None else int(generation),
        "persist_to_snapshot": False,
    }


def _close(actual: float, expected: float) -> bool:
    return math.isclose(float(actual), float(expected), abs_tol=1e-6)


def exercise(base_url: str, wait_s: float, min_objects: int) -> dict[str, Any]:
    deadline = time.monotonic() + wait_s
    state: dict[str, Any] | None = None
    derived: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        try:
            state = _request(base_url, "GET", "/api/state")
            derived = _derived_objects(state)
        except (OSError, RuntimeError, ValueError):
            state = None
            derived = []
        if len(derived) >= min_objects:
            break
        time.sleep(1)
    if state is None or len(derived) < min_objects:
        raise RuntimeError(
            f"only {len(derived)} derived objects appeared within {wait_s:.0f}s"
        )

    epoch = _epoch(state)
    label_target = derived[0]
    delete_target = derived[1]
    original_label = str(label_target.get("cls") or "")
    test_label = "runtime_verified_object"

    label_response = _request(
        base_url,
        "POST",
        f"/api/objects/{label_target['id']}/label",
        {**epoch, "label": test_label},
    )
    state = _request(base_url, "GET", "/api/state")
    after_label = _find_object(state, str(label_target["id"]))
    if (
        after_label is None
        or after_label.get("cls") != test_label
        or after_label.get("label_source") != "operator"
    ):
        raise RuntimeError("operator label edit was not visible in /api/state")

    pose = after_label.get("pose") or {}
    bbox = after_label.get("bbox") or {}
    edited = {
        "x": float(pose["x"]) + 0.031,
        "y": float(pose["y"]) - 0.027,
        "z": float(pose["z"]) + 0.019,
        "yaw": float(bbox.get("yaw", pose.get("yaw", 0.0))) + 0.041,
        "size_x": max(0.05, float(bbox["size_x"]) + 0.023),
        "size_y": max(0.05, float(bbox["size_y"]) + 0.017),
        "size_z": max(0.05, float(bbox["size_z"]) + 0.029),
        "frame_id": str(pose["frame_id"]),
    }
    geometry_response = _request(
        base_url,
        "POST",
        f"/api/objects/{label_target['id']}/geometry",
        {**epoch, **edited},
    )
    state = _request(base_url, "GET", "/api/state")
    after_geometry = _find_object(state, str(label_target["id"]))
    if after_geometry is None:
        raise RuntimeError("geometry-edited object disappeared from /api/state")
    actual_pose = after_geometry.get("pose") or {}
    actual_bbox = after_geometry.get("bbox") or {}
    if (
        after_geometry.get("geometry_source") != "operator_bbox"
        or after_geometry.get("navigation_grade") is not False
        or any(
            not _close(actual_pose[field], edited[field])
            for field in ("x", "y", "z")
        )
        or any(
            not _close(actual_bbox[field], edited[field])
            for field in ("size_x", "size_y", "size_z", "yaw")
        )
    ):
        raise RuntimeError("operator geometry edit was not visible in /api/state")

    delete_response = _request(
        base_url,
        "DELETE",
        f"/api/objects/{delete_target['id']}",
        epoch,
    )
    state = _request(base_url, "GET", "/api/state")
    if _find_object(state, str(delete_target["id"])) is not None:
        raise RuntimeError("deleted object remained visible in /api/state")

    pre_flush_ids = {
        str(obj["id"])
        for obj in _derived_objects(state)
    }
    flush_response = _request(
        base_url,
        "POST",
        "/api/objects/flush",
        epoch,
    )
    state = _request(base_url, "GET", "/api/state")
    post_flush_ids = {
        str(obj["id"])
        for obj in _derived_objects(state)
    }
    surviving_ids = sorted(pre_flush_ids & post_flush_ids)
    if surviving_ids:
        raise RuntimeError(
            "flush left pre-existing derived objects in /api/state: "
            + ", ".join(surviving_ids[:8])
        )

    return {
        "ok": True,
        "base_url": base_url,
        "epoch": epoch,
        "initial_derived_count": len(derived),
        "label_edit": {
            "object_id": label_target["id"],
            "original_label": original_label,
            "new_label": test_label,
            "response": label_response,
        },
        "geometry_edit": {
            "object_id": label_target["id"],
            "requested": edited,
            "response": geometry_response,
        },
        "delete": {
            "object_id": delete_target["id"],
            "response": delete_response,
        },
        "flush": {
            "pre_flush_count": len(pre_flush_ids),
            "post_flush_count": len(post_flush_ids),
            "response": flush_response,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-url",
        default="http://127.0.0.1:50107",
    )
    parser.add_argument("--wait-s", type=float, default=180.0)
    parser.add_argument("--min-objects", type=int, default=3)
    parser.add_argument("--output")
    args = parser.parse_args()
    result = exercise(args.base_url, args.wait_s, args.min_objects)
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
