#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Score persisted Scene CLIP features against configured label prototypes.

This is an offline evaluation helper.  It joins debug-only object features to
WBT matches through the stable Scene registry id, then applies the same
prototype construction and conservative switching rule as the live detector.
It never changes Scene state.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

_DEFAULT_SWEEP_MARGINS = (
    0.0,
    0.002,
    0.005,
    0.01,
    0.02,
    0.03,
    0.04,
    0.05,
    0.06,
)


def _label(value: Any) -> str:
    return str(value or "").strip().lower().replace("_", " ")


def _horizontal_extent_m(obj: dict[str, Any]) -> float | None:
    """Return the larger horizontal dimension from exported bbox corners."""
    corners = obj.get("bbox_corners") or []
    if len(corners) != 8:
        return None
    try:
        first = np.asarray(corners[0], dtype=np.float64)
        x_edge = np.asarray(corners[1], dtype=np.float64) - first
        y_edge = np.asarray(corners[2], dtype=np.float64) - first
        extent = max(
            float(np.linalg.norm(x_edge[:2])),
            float(np.linalg.norm(y_edge[:2])),
        )
    except (TypeError, ValueError):
        return None
    return extent if math.isfinite(extent) and extent > 0.0 else None


def _persistent_geometry_adjustments(
    *,
    current_label: str,
    candidates: list[str] | tuple[str, ...],
    horizontal_extent_m: float | None,
    score_bonus: float,
    constraints_by_label: dict[str, dict[str, Any]],
) -> dict[str, float]:
    """Return positive bonuses for candidates satisfying configured bounds."""
    if (
        horizontal_extent_m is None
        or not math.isfinite(horizontal_extent_m)
        or score_bonus <= 0.0
    ):
        return {}
    adjustments = {}
    for label in candidates:
        constraints = constraints_by_label.get(label)
        if not constraints:
            continue
        source_labels = constraints.get("source_labels")
        if source_labels is not None and current_label not in source_labels:
            continue
        minimum = constraints.get("min_horizontal_extent_m")
        maximum = constraints.get("max_horizontal_extent_m")
        if minimum is not None and horizontal_extent_m < minimum:
            continue
        if maximum is not None and horizontal_extent_m > maximum:
            continue
        adjustments[label] = score_bonus
    return adjustments


def _load_rerank_config(manifest_path: Path) -> dict[str, Any]:
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    return (
        manifest.get("system", {})
        .get("scene", {})
        .get("config", {})
        .get("perception", {})
        .get("label", {})
        .get("clip_rerank", {})
    )


def _text_prototypes(
    *,
    candidate_sets: list[list[str]],
    prompts_by_label: dict[str, list[str]],
    model_name: str,
    checkpoint: str,
    device: str,
) -> dict[str, np.ndarray]:
    import open_clip
    import torch

    model, _, _ = open_clip.create_model_and_transforms(
        model_name,
        pretrained=checkpoint,
    )
    model = model.to(device).eval()
    tokenizer = open_clip.get_tokenizer(model_name)
    labels = list(
        dict.fromkeys(
            label
            for candidates in candidate_sets
            for label in candidates
        )
    )
    prompt_sets = {
        label: prompts_by_label.get(label) or [f"a photo of a {label}"]
        for label in labels
    }
    prompts = [prompt for label in labels for prompt in prompt_sets[label]]
    with torch.no_grad():
        tokens = tokenizer(prompts).to(device)
        features = model.encode_text(tokens).float()
        features = features / features.norm(dim=-1, keepdim=True)
    prototypes: dict[str, np.ndarray] = {}
    offset = 0
    for label in labels:
        count = len(prompt_sets[label])
        prototype = features[offset : offset + count].mean(dim=0)
        prototype = prototype / prototype.norm()
        prototypes[label] = (
            prototype.detach().cpu().numpy().astype(np.float32)
        )
        offset += count
    return prototypes


def _selected_label(
    row: dict[str, Any],
    *,
    min_score: float,
    min_margin: float,
) -> str:
    current = str(row["current"])
    winner = str(row["winner"])
    scores = row.get("scores") or {}
    if (
        winner != current
        and winner in scores
        and current in scores
        and float(scores[winner]) >= min_score
        and float(scores[winner]) > float(scores[current])
        and float(scores[winner]) - float(scores[current]) >= min_margin
    ):
        return winner
    return current


def _margin_sweep(
    *,
    matched_rows: list[dict[str, Any]],
    feature_rows: list[dict[str, Any]],
    groups: list[list[str]],
    min_score: float,
    margins: tuple[float, ...] = _DEFAULT_SWEEP_MARGINS,
) -> list[dict[str, Any]]:
    reports = []
    for labels in groups:
        label_set = set(labels)
        matched = [
            row
            for row in matched_rows
            if row.get("scores") and row.get("current") in label_set
        ]
        features = [
            row
            for row in feature_rows
            if row.get("scores") and row.get("current") in label_set
        ]
        trials = []
        for margin in margins:
            selected = [
                _selected_label(
                    row,
                    min_score=min_score,
                    min_margin=margin,
                )
                for row in matched
            ]
            trials.append(
                {
                    "min_margin": margin,
                    "selected_correct": sum(
                        label == row["expected"]
                        for label, row in zip(selected, matched)
                    ),
                    "switches": sum(
                        label != row["current"]
                        for label, row in zip(selected, matched)
                    ),
                    "helpful_switches": sum(
                        label == row["expected"]
                        and row["current"] != row["expected"]
                        for label, row in zip(selected, matched)
                    ),
                    "harmful_switches": sum(
                        label != row["expected"]
                        and row["current"] == row["expected"]
                        for label, row in zip(selected, matched)
                    ),
                    "all_feature_switches": sum(
                        _selected_label(
                            row,
                            min_score=min_score,
                            min_margin=margin,
                        )
                        != row["current"]
                        for row in features
                    ),
                }
            )
        reports.append(
            {
                "labels": labels,
                "matched_rows": len(matched),
                "feature_rows": len(features),
                "current_correct": sum(
                    row["current"] == row["expected"] for row in matched
                ),
                "trials": trials,
            }
        )
    return reports


def _route_margin_sweep(
    *,
    matched_rows: list[dict[str, Any]],
    feature_rows: list[dict[str, Any]],
    routes: dict[str, list[str]],
    min_score: float,
    margins: tuple[float, ...] = _DEFAULT_SWEEP_MARGINS,
) -> list[dict[str, Any]]:
    """Sweep source-specific candidate sets without making them symmetric."""
    reports = []
    for source, labels in routes.items():
        matched = [
            row
            for row in matched_rows
            if row.get("scores") and row.get("current") == source
        ]
        features = [
            row
            for row in feature_rows
            if row.get("scores") and row.get("current") == source
        ]
        trials = []
        for margin in margins:
            selected = [
                _selected_label(
                    row,
                    min_score=min_score,
                    min_margin=margin,
                )
                for row in matched
            ]
            trials.append(
                {
                    "min_margin": margin,
                    "selected_correct": sum(
                        label == row["expected"]
                        for label, row in zip(selected, matched)
                    ),
                    "switches": sum(
                        label != row["current"]
                        for label, row in zip(selected, matched)
                    ),
                    "helpful_switches": sum(
                        label == row["expected"]
                        and row["current"] != row["expected"]
                        for label, row in zip(selected, matched)
                    ),
                    "harmful_switches": sum(
                        label != row["expected"]
                        and row["current"] == row["expected"]
                        for label, row in zip(selected, matched)
                    ),
                    "all_feature_switches": sum(
                        _selected_label(
                            row,
                            min_score=min_score,
                            min_margin=margin,
                        )
                        != row["current"]
                        for row in features
                    ),
                }
            )
        reports.append(
            {
                "source": source,
                "labels": labels,
                "matched_rows": len(matched),
                "feature_rows": len(features),
                "current_correct": sum(
                    row["current"] == row["expected"] for row in matched
                ),
                "trials": trials,
            }
        )
    return reports


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--objects-debug", type=Path, required=True)
    parser.add_argument("--evaluation", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument(
        "--model-name",
        default="ViT-B-32",
    )
    parser.add_argument(
        "--checkpoint",
        default="/opt/models/open_clip_pytorch_model.bin",
    )
    parser.add_argument(
        "--device",
        default="cuda",
    )
    parser.add_argument(
        "--prompt",
        action="append",
        default=[],
        metavar="LABEL=TEXT",
        help=(
            "replace one configured label prototype with these prompts; "
            "repeat the option to build an ensemble"
        ),
    )
    parser.add_argument(
        "--group",
        action="append",
        default=[],
        metavar="LABEL,LABEL,...",
        help=(
            "replace configured rerank groups for an offline experiment; "
            "repeat for disjoint groups"
        ),
    )
    parser.add_argument(
        "--route",
        action="append",
        default=[],
        metavar="SOURCE=LABEL,LABEL,...",
        help=(
            "replace configured rerank scopes with source-specific candidate "
            "sets; repeat for distinct source labels. SOURCE must be one of "
            "the candidates"
        ),
    )
    parser.add_argument("--min-score", type=float)
    parser.add_argument("--min-margin", type=float)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    debug = json.loads(args.objects_debug.read_text(encoding="utf-8"))
    if debug.get("debug_clip_features") is not True:
        raise SystemExit("objects payload is not a debug CLIP snapshot")
    evaluation = json.loads(args.evaluation.read_text(encoding="utf-8"))
    config = _load_rerank_config(args.manifest)
    groups = []
    group_min_margin_by_label: dict[str, float] = {}
    for raw_group in config.get("groups") or []:
        if isinstance(raw_group, dict):
            labels = raw_group.get("labels") or []
            margin = raw_group.get("min_margin")
        else:
            labels = raw_group
            margin = None
        group = [_label(label) for label in labels]
        groups.append(group)
        if margin is not None:
            group_min_margin_by_label.update(
                {label: float(margin) for label in group}
            )
    routes: dict[str, list[str]] = {}
    route_min_margin_by_label: dict[str, float] = {}
    raw_routes = config.get("routes") or {}
    if not isinstance(raw_routes, dict):
        parser.error("configured clip_rerank.routes must be a mapping")
    for raw_source, raw_route in raw_routes.items():
        source = _label(raw_source)
        margin = None
        if isinstance(raw_route, list):
            raw_labels = raw_route
        elif isinstance(raw_route, dict):
            unknown_keys = set(raw_route) - {"labels", "min_margin"}
            if unknown_keys:
                parser.error(
                    f"route {raw_source!r} has unknown keys: "
                    + ", ".join(sorted(unknown_keys))
                )
            raw_labels = raw_route.get("labels")
            margin = raw_route.get("min_margin")
        else:
            raw_labels = None
        if (
            not isinstance(raw_labels, list)
            or any(not isinstance(item, str) for item in raw_labels)
        ):
            parser.error(
                f"invalid configured route {raw_source!r}; labels must be "
                "a string list"
            )
        labels = list(
            dict.fromkeys(
                label
                for label in (
                    _label(item) for item in raw_labels
                )
                if label
            )
        )
        if not source or len(labels) < 2 or source not in labels:
            parser.error(
                f"invalid configured route {raw_source!r}; expected at least "
                "two distinct labels including the source"
            )
        routes[source] = labels
        if margin is not None:
            route_min_margin_by_label[source] = float(margin)
    if args.group or args.route:
        groups = []
        group_min_margin_by_label = {}
        routes = {}
        route_min_margin_by_label = {}
        for value in args.group:
            group = list(
                dict.fromkeys(
                    label
                    for label in (
                        _label(item) for item in value.split(",")
                    )
                    if label
                )
            )
            if len(group) < 2:
                parser.error(
                    f"invalid --group {value!r}; expected at least two labels"
                )
            groups.append(group)
        for value in args.route:
            raw_source, separator, raw_labels = value.partition("=")
            source = _label(raw_source)
            labels = list(
                dict.fromkeys(
                    label
                    for label in (
                        _label(item) for item in raw_labels.split(",")
                    )
                    if label
                )
            )
            if (
                not separator
                or not source
                or len(labels) < 2
                or source not in labels
            ):
                parser.error(
                    f"invalid --route {value!r}; expected "
                    "SOURCE=LABEL,LABEL,... with SOURCE among at least two "
                    "distinct labels"
                )
            if source in routes:
                parser.error(f"duplicate --route source {source!r}")
            routes[source] = labels
    repeated = sorted(
        label
        for label in {label for group in groups for label in group}
        if sum(label in group for group in groups) > 1
    )
    if repeated:
        parser.error(
            "rerank groups must be disjoint; repeated labels: "
            + ", ".join(repeated)
        )
    grouped_labels = {
        label
        for group in groups
        for label in group
    }
    ambiguous_sources = sorted(set(routes) & grouped_labels)
    if ambiguous_sources:
        parser.error(
            "route source labels must not also belong to symmetric groups: "
            + ", ".join(ambiguous_sources)
        )
    prompts = {
        _label(label): [str(prompt) for prompt in values]
        for label, values in (config.get("prompts") or {}).items()
    }
    prompt_overrides: dict[str, list[str]] = {}
    for value in args.prompt:
        raw_label, separator, text = value.partition("=")
        label = _label(raw_label)
        text = text.strip()
        if not separator or not label or not text:
            parser.error(f"invalid --prompt {value!r}; expected LABEL=TEXT")
        prompt_overrides.setdefault(label, []).append(text)
    prompts.update(prompt_overrides)
    min_score = (
        float(args.min_score)
        if args.min_score is not None
        else float(config.get("min_score", 0.20))
    )
    min_margin = (
        float(args.min_margin)
        if args.min_margin is not None
        else float(config.get("min_margin", 0.04))
    )
    min_margin_by_label = {
        **group_min_margin_by_label,
        **route_min_margin_by_label,
    }
    persistent_geometry = config.get("persistent_geometry") or {}
    geometry_score_bonus = float(
        persistent_geometry.get("score_bonus", 0.0)
    )
    geometry_constraints_by_label = {
        _label(label): {
            str(key): (
                [_label(source) for source in value]
                if str(key) == "source_labels"
                else float(value)
            )
            for key, value in constraints.items()
        }
        for label, constraints in (
            persistent_geometry.get("labels") or {}
        ).items()
    }
    prototypes = _text_prototypes(
        candidate_sets=groups + list(routes.values()),
        prompts_by_label=prompts,
        model_name=args.model_name,
        checkpoint=args.checkpoint,
        device=args.device,
    )
    candidates_by_label = {
        label: group
        for group in groups
        for label in group
    }
    candidates_by_label.update(routes)
    objects = {
        str(obj.get("registry_id")): obj
        for obj in (debug.get("objects") or [])
        if obj.get("registry_id") and obj.get("clip_feature")
    }

    def score_object(
        obj: dict[str, Any],
        *,
        current_override: str | None = None,
    ) -> dict[str, Any] | None:
        current = _label(
            current_override
            if current_override is not None
            else obj.get("cls")
        )
        feature = np.asarray(
            obj["clip_feature"],
            dtype=np.float32,
        ).reshape(-1)
        norm = float(np.linalg.norm(feature))
        if not math.isfinite(norm) or norm <= 1e-9:
            return None
        feature /= norm
        candidates = candidates_by_label.get(current) or []
        visual_scores = {
            label: float(np.dot(feature, prototypes[label]))
            for label in candidates
            if label in prototypes
        }
        horizontal_extent = _horizontal_extent_m(obj)
        geometry_adjustments = _persistent_geometry_adjustments(
            current_label=current,
            candidates=candidates,
            horizontal_extent_m=horizontal_extent,
            score_bonus=geometry_score_bonus,
            constraints_by_label=geometry_constraints_by_label,
        )
        scores = {
            label: score + geometry_adjustments.get(label, 0.0)
            for label, score in visual_scores.items()
        }
        winner = max(scores, key=scores.get) if scores else current
        margin = (
            scores[winner] - scores[current]
            if current in scores and winner in scores
            else 0.0
        )
        switched = (
            winner != current
            and scores[winner] >= min_score
            and scores[winner] > scores[current]
            and margin
            >= min_margin_by_label.get(current, min_margin)
        )
        return {
            "current": current,
            "selected": winner if switched else current,
            "winner": winner,
            "winner_margin": round(margin, 6),
            "min_margin": min_margin_by_label.get(
                current,
                min_margin,
            ),
            "scores": {
                label: round(score, 6)
                for label, score in scores.items()
            },
            "visual_scores": {
                label: round(score, 6)
                for label, score in visual_scores.items()
            },
            "horizontal_extent_m": (
                round(horizontal_extent, 6)
                if horizontal_extent is not None
                else None
            ),
            "geometry_adjustments": {
                label: round(value, 6)
                for label, value in geometry_adjustments.items()
            },
        }

    rows = []
    for target in evaluation.get("per_target") or []:
        object_id = target.get("object_id")
        if not target.get("matched") or object_id not in objects:
            continue
        expected = _label(target.get("expected_label"))
        scored = score_object(
            objects[object_id],
            current_override=_label(target.get("observed_label")),
        )
        if scored is None:
            continue
        current = scored["current"]
        selected = scored["selected"]
        rows.append(
            {
                "identity": target.get("identity"),
                "object_id": object_id,
                "expected": expected,
                "current": current,
                "selected": selected,
                "current_correct": current == expected,
                "selected_correct": selected == expected,
                "winner": scored["winner"],
                "winner_margin": scored["winner_margin"],
                "scores": scored["scores"],
                "visual_scores": scored["visual_scores"],
                "horizontal_extent_m": scored["horizontal_extent_m"],
                "geometry_adjustments": scored["geometry_adjustments"],
            }
        )

    all_feature_rows = []
    for object_id, obj in sorted(objects.items()):
        scored = score_object(obj)
        if scored is None or not scored["scores"]:
            continue
        all_feature_rows.append(
            {
                "object_id": object_id,
                "num_detections": int(obj.get("num_detections") or 0),
                **scored,
            }
        )
    scoped = [row for row in rows if row["scores"]]
    payload = {
        "model_name": args.model_name,
        "checkpoint": args.checkpoint,
        "min_score": min_score,
        "min_margin": min_margin,
        "min_margin_by_label": min_margin_by_label,
        "persistent_geometry": {
            "score_bonus": geometry_score_bonus,
            "labels": geometry_constraints_by_label,
        },
        "routes": routes,
        "prompt_overrides": prompt_overrides,
        "matched_feature_rows": len(rows),
        "rerank_scoped_rows": len(scoped),
        "current_correct": sum(row["current_correct"] for row in scoped),
        "selected_correct": sum(row["selected_correct"] for row in scoped),
        "switches": sum(
            row["selected"] != row["current"]
            for row in scoped
        ),
        "all_feature_row_count": len(all_feature_rows),
        "feature_rows": all_feature_rows,
        "all_feature_switches": [
            row
            for row in all_feature_rows
            if row["selected"] != row["current"]
        ],
        "margin_sweep": _margin_sweep(
            matched_rows=rows,
            feature_rows=all_feature_rows,
            groups=groups,
            min_score=min_score,
        ),
        "route_margin_sweep": _route_margin_sweep(
            matched_rows=rows,
            feature_rows=all_feature_rows,
            routes=routes,
            min_score=min_score,
        ),
        "rows": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
