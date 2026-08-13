#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Standalone local tester for llm_detect orientation output.

Purpose:
    Feed a single local image + a target object name to the same LLM
    endpoint / prompt / schema the service uses, then draw the returned
    bounding box AND the coarse orientation stroke on the image and
    save the annotated result. Everything ROS-related in main.py is
    bypassed on purpose so this script can be run on any machine with
    only Python + opencv + openai + toml installed.

Usage:
    python scripts/test_local.py \\
        --image /path/to/frame.jpg \\
        --object banana \\
        --base-url https://api.openai.com/v1 \\
        --model gpt-4o \\
        --api-key sk-xxx \\
        --output /tmp/detect_out.jpg

Environment fallbacks (only used if the matching CLI flag is not given):
    LLM_BASE_URL, LLM_MODEL, LLM_API_KEY, LLM_TEMPERATURE

The script exits non-zero if the LLM refuses / fails to parse / the
target is not found, so it can also be used as a smoke test in CI.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
from pathlib import Path

# Heavy deps (cv2, openai, toml) are imported lazily inside the helpers
# so that `--help` still works on a bare Python install and users only
# see a "please install X" hint when they actually try to run detection.

# Reuse the exact orientation helpers and normalization the service uses,
# so what we render locally matches what the service would render into
# latest.jpg on the robot.
_HERE = Path(__file__).resolve().parent
_PKG_ROOT = _HERE.parent
sys.path.insert(0, str(_PKG_ROOT / "llm_detect"))

# NOTE: importing main.py at module scope would try to import robonix_api
# (a ROS-side dependency that most dev machines don't have). To keep this
# tester dependency-light we duplicate the tiny set of helpers we need.
# If you change ORIENTATION_LABELS or _orientation_endpoints in main.py,
# update the copies below as well — they are intentionally short.

ORIENTATION_LABELS = (
    "vertical", "horizontal", "diag_tlbr", "diag_trbl", "unknown",
)
_ORIENTATION_COLOR = (255, 0, 255)  # magenta BGR
_TARGET_COLOR = (0, 255, 0)         # green BGR


def _normalize_orientation(raw) -> str:
    if not isinstance(raw, str):
        return "unknown"
    v = raw.strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "up_down": "vertical", "updown": "vertical", "portrait": "vertical",
        "left_right": "horizontal", "leftright": "horizontal", "landscape": "horizontal",
        "tl_br": "diag_tlbr", "tlbr": "diag_tlbr", "top_left_bottom_right": "diag_tlbr",
        "backslash": "diag_tlbr", "\\": "diag_tlbr",
        "tr_bl": "diag_trbl", "trbl": "diag_trbl", "top_right_bottom_left": "diag_trbl",
        "slash": "diag_trbl", "/": "diag_trbl",
        "none": "unknown", "": "unknown",
    }
    v = aliases.get(v, v)
    return v if v in ORIENTATION_LABELS else "unknown"


def _require_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as e:
        print("[test_local] ERROR: opencv-python is required to run this "
              "script. Install with `pip install opencv-python`.",
              file=sys.stderr)
        raise SystemExit(70) from e
    return cv2


def _orientation_endpoints(bbox, orientation: str):
    x_min, y_min, x_max, y_max = [int(v) for v in bbox]
    if x_max <= x_min or y_max <= y_min:
        return None
    cx = (x_min + x_max) // 2
    cy = (y_min + y_max) // 2
    inset = max(2, int(0.08 * min(x_max - x_min, y_max - y_min)))
    x0, x1 = x_min + inset, x_max - inset
    y0, y1 = y_min + inset, y_max - inset
    if orientation == "vertical":
        return (cx, y0), (cx, y1)
    if orientation == "horizontal":
        return (x0, cy), (x1, cy)
    if orientation == "diag_tlbr":
        return (x0, y0), (x1, y1)
    if orientation == "diag_trbl":
        return (x1, y0), (x0, y1)
    return None


def _draw_orientation(img, bbox, orientation: str,
                      color=_ORIENTATION_COLOR, thickness: int = 3) -> bool:
    cv2 = _require_cv2()
    endpoints = _orientation_endpoints(bbox, orientation)
    if endpoints is None:
        return False
    pt1, pt2 = endpoints
    cv2.line(img, pt1, pt2, color, thickness, cv2.LINE_AA)
    return True


def _draw_one_box(img, bbox, label, color, thickness=2):
    cv2 = _require_cv2()
    x_min, y_min, x_max, y_max = [int(v) for v in bbox]
    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color, thickness)
    if label:
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ly = max(y_min - 4, th + 4)
        cv2.rectangle(img, (x_min, ly - th - 4),
                      (x_min + tw + 4, ly), color, -1)
        cv2.putText(img, label, (x_min + 2, ly - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
                    cv2.LINE_AA)


def _extract_json(text: str) -> str:
    import re
    m = re.search(r"```(?:json|JSON)?\s*(.*?)```", text, flags=re.S)
    return m.group(1).strip() if m else text


def _load_single_detect_prompt() -> str:
    prompts_path = _PKG_ROOT / "config" / "prompts.toml"
    if not prompts_path.is_file():
        raise FileNotFoundError(f"prompts.toml not found: {prompts_path}")
    try:
        import toml  # lazy import — only needed once we actually run
    except ImportError as e:
        print("[test_local] ERROR: the `toml` package is required. "
              "Install with `pip install toml`.", file=sys.stderr)
        raise SystemExit(70) from e
    data = toml.load(prompts_path)
    p = data.get("prompts", {}).get("single_detect_prompt", "")
    if not p:
        raise KeyError("prompts.single_detect_prompt missing in prompts.toml")
    return p


def _build_detection_schema() -> dict:
    """Mirror of the schema in llm_detect/main.py so the LLM is constrained
    to return the orientation field."""
    return {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "id": {"type": "integer"},
            "class_name": {"type": "string"},
            "box_center_x": {"type": "number", "minimum": 0.0, "maximum": 1.0},
            "box_center_y": {"type": "number", "minimum": 0.0, "maximum": 1.0},
            "box_width":    {"type": "number", "minimum": 0.0, "maximum": 1.0},
            "box_height":   {"type": "number", "minimum": 0.0, "maximum": 1.0},
            "orientation":  {"type": "string", "enum": list(ORIENTATION_LABELS)},
            "thinking_process": {"anyOf": [{"type": "string"}, {"type": "null"}]},
            "failed":       {"anyOf": [{"type": "boolean"}, {"type": "null"}]},
        },
        "required": [
            "id", "class_name",
            "box_center_x", "box_center_y", "box_width", "box_height",
            "orientation", "thinking_process", "failed",
        ],
    }


def run_once(image_path: Path, object_name: str,
             base_url: str, model: str, api_key: str,
             temperature: float, output_path: Path,
             json_dump_path: "Path | None" = None,
             use_json_schema: bool = True) -> int:
    cv2 = _require_cv2()
    try:
        from openai import OpenAI  # imported here so --help works without deps
    except ImportError as e:
        print("[test_local] ERROR: the `openai` package is required. "
              "Install with `pip install openai`.", file=sys.stderr)
        raise SystemExit(70) from e

    img_bgr = cv2.imread(str(image_path))
    if img_bgr is None:
        print(f"[test_local] ERROR: failed to read image: {image_path}",
              file=sys.stderr)
        return 2
    img_h, img_w = img_bgr.shape[:2]
    print(f"[test_local] image: {image_path} ({img_w}x{img_h})")

    prompt = _load_single_detect_prompt().replace("{object_name}", object_name)

    _, img_encoded = cv2.imencode(".jpg", img_bgr)
    image_b64 = base64.b64encode(img_encoded.tobytes()).decode("utf-8")

    client = OpenAI(base_url=base_url, api_key=api_key or "any",
                    timeout=60.0, max_retries=1)

    print(f"[test_local] calling LLM: base_url={base_url} model={model} "
          f"temperature={temperature} json_schema={use_json_schema}")
    kwargs = dict(
        model=model,
        messages=[{
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
            ],
        }],
        temperature=temperature,
    )
    if use_json_schema:
        kwargs["response_format"] = {
            "type": "json_schema",
            "json_schema": {
                "name": "single_target_detection",
                "strict": True,
                "schema": _build_detection_schema(),
            },
        }

    try:
        completion = client.chat.completions.create(**kwargs)
    except Exception as e:  # noqa: BLE001
        print(f"[test_local] ERROR: LLM API call failed: "
              f"{type(e).__name__}: {e}", file=sys.stderr)
        return 3

    if not completion.choices:
        print("[test_local] ERROR: LLM returned no choices", file=sys.stderr)
        return 4
    content = completion.choices[0].message.content or ""
    if not content.strip():
        print(f"[test_local] ERROR: LLM returned empty content "
              f"(finish_reason={completion.choices[0].finish_reason})",
              file=sys.stderr)
        return 4
    print(f"[test_local] LLM raw content (first 500):\n{content[:500]}")

    try:
        parsed = json.loads(_extract_json(content))
    except (json.JSONDecodeError, ValueError) as e:
        print(f"[test_local] ERROR: JSON parse failed: {e}", file=sys.stderr)
        return 5
    print(f"[test_local] parsed result: {parsed}")

    if json_dump_path is not None:
        json_dump_path.parent.mkdir(parents=True, exist_ok=True)
        json_dump_path.write_text(json.dumps(parsed, indent=2, ensure_ascii=False))
        print(f"[test_local] wrote raw JSON to {json_dump_path}")

    failed = bool(parsed.get("failed", False))
    orientation = _normalize_orientation(parsed.get("orientation"))

    annotated = img_bgr.copy()
    if failed:
        cv2.putText(annotated, f"FAILED: '{object_name}' not found",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2, cv2.LINE_AA)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), annotated)
        print(f"[test_local] target NOT found; annotated image -> {output_path}")
        return 1

    try:
        cx = float(parsed["box_center_x"])
        cy = float(parsed["box_center_y"])
        w  = float(parsed["box_width"])
        h  = float(parsed["box_height"])
    except (KeyError, TypeError, ValueError) as e:
        print(f"[test_local] ERROR: invalid bbox fields: {e}", file=sys.stderr)
        return 5

    bbox = [
        int(max(0,     (cx - w / 2) * img_w)),
        int(max(0,     (cy - h / 2) * img_h)),
        int(min(img_w, (cx + w / 2) * img_w)),
        int(min(img_h, (cy + h / 2) * img_h)),
    ]

    cls = str(parsed.get("class_name", object_name))
    label = f"{cls} dir={orientation}"
    _draw_one_box(annotated, bbox, label, color=_TARGET_COLOR, thickness=3)
    drew_stroke = _draw_orientation(annotated, bbox, orientation)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), annotated)
    print(f"[test_local] OK: class={cls!r} bbox={bbox} orientation={orientation!r} "
          f"stroke_drawn={drew_stroke}")
    print(f"[test_local] annotated image -> {output_path}")
    return 0


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Standalone tester for llm_detect orientation output.",
    )
    ap.add_argument("--image", required=True, type=Path,
                    help="path to the input image (jpg/png)")
    ap.add_argument("--object", required=True,
                    help="target object name, e.g. 'banana'")
    ap.add_argument("--base-url", default=os.environ.get("LLM_BASE_URL", ""),
                    help="OpenAI-compatible base URL "
                         "(default: $LLM_BASE_URL)")
    ap.add_argument("--model", default=os.environ.get("LLM_MODEL", ""),
                    help="model id (default: $LLM_MODEL)")
    ap.add_argument("--api-key", default=os.environ.get("LLM_API_KEY", ""),
                    help="API key (default: $LLM_API_KEY, empty means 'any')")
    ap.add_argument("--temperature", type=float,
                    default=float(os.environ.get("LLM_TEMPERATURE", "0.0")),
                    help="sampling temperature (default: 0.0)")
    ap.add_argument("--output", type=Path,
                    default=Path("/tmp/llm_detect_test_out.jpg"),
                    help="output annotated image path "
                         "(default: /tmp/llm_detect_test_out.jpg)")
    ap.add_argument("--dump-json", type=Path, default=None,
                    help="optional path to also write the raw LLM JSON result")
    ap.add_argument("--no-json-schema", action="store_true",
                    help="disable response_format=json_schema (use if your "
                         "provider does not support strict schemas)")
    return ap.parse_args()


def main() -> int:
    args = _parse_args()
    if not args.base_url:
        print("[test_local] ERROR: --base-url (or $LLM_BASE_URL) is required",
              file=sys.stderr)
        return 64
    if not args.model:
        print("[test_local] ERROR: --model (or $LLM_MODEL) is required",
              file=sys.stderr)
        return 64
    if not args.image.is_file():
        print(f"[test_local] ERROR: image not found: {args.image}",
              file=sys.stderr)
        return 66
    return run_once(
        image_path=args.image,
        object_name=args.object,
        base_url=args.base_url,
        model=args.model,
        api_key=args.api_key,
        temperature=args.temperature,
        output_path=args.output,
        json_dump_path=args.dump_json,
        use_json_schema=not args.no_json_schema,
    )


if __name__ == "__main__":
    sys.exit(main())
