# SPDX-License-Identifier: MulanPSL-2.0

from analyze_webots_clip_labels import (
    _horizontal_extent_m,
    _margin_sweep,
    _persistent_geometry_adjustments,
    _route_margin_sweep,
    _selected_label,
)


def _row(
    *,
    current: str,
    winner: str,
    expected: str,
    current_score: float,
    winner_score: float,
) -> dict:
    return {
        "current": current,
        "winner": winner,
        "expected": expected,
        "scores": {
            current: current_score,
            winner: winner_score,
        },
    }


def test_selected_label_respects_score_and_margin() -> None:
    row = _row(
        current="picture frame",
        winner="window",
        expected="window",
        current_score=0.25,
        winner_score=0.255,
    )
    assert (
        _selected_label(row, min_score=0.20, min_margin=0.005) == "window"
    )
    assert (
        _selected_label(row, min_score=0.20, min_margin=0.006)
        == "picture frame"
    )
    assert (
        _selected_label(row, min_score=0.26, min_margin=0.0)
        == "picture frame"
    )


def test_margin_sweep_reports_helpful_harmful_and_unmatched_switches() -> None:
    helpful = _row(
        current="picture frame",
        winner="window",
        expected="window",
        current_score=0.25,
        winner_score=0.255,
    )
    harmful = _row(
        current="picture frame",
        winner="window",
        expected="picture frame",
        current_score=0.25,
        winner_score=0.253,
    )
    unmatched = {
        **_row(
            current="picture frame",
            winner="window",
            expected="",
            current_score=0.25,
            winner_score=0.254,
        ),
        "object_id": "scene.object.picture_frame_999",
    }
    report = _margin_sweep(
        matched_rows=[helpful, harmful],
        feature_rows=[unmatched],
        groups=[["window", "picture frame"]],
        min_score=0.20,
        margins=(0.0, 0.004, 0.006),
    )[0]
    assert report["matched_rows"] == 2
    assert report["feature_rows"] == 1
    assert report["current_correct"] == 1
    assert report["trials"][0] == {
        "min_margin": 0.0,
        "selected_correct": 1,
        "switches": 2,
        "helpful_switches": 1,
        "harmful_switches": 1,
        "all_feature_switches": 1,
    }
    assert report["trials"][1]["selected_correct"] == 2
    assert report["trials"][1]["helpful_switches"] == 1
    assert report["trials"][1]["harmful_switches"] == 0
    assert report["trials"][1]["all_feature_switches"] == 1
    assert report["trials"][2]["switches"] == 0


def test_route_margin_sweep_only_applies_to_the_source_label() -> None:
    helpful = _row(
        current="chair",
        winner="monitor",
        expected="monitor",
        current_score=0.22,
        winner_score=0.25,
    )
    reverse_direction = _row(
        current="monitor",
        winner="chair",
        expected="monitor",
        current_score=0.22,
        winner_score=0.25,
    )
    report = _route_margin_sweep(
        matched_rows=[helpful, reverse_direction],
        feature_rows=[],
        routes={"chair": ["chair", "monitor"]},
        min_score=0.20,
        margins=(0.02,),
    )[0]
    assert report["source"] == "chair"
    assert report["labels"] == ["chair", "monitor"]
    assert report["matched_rows"] == 1
    assert report["trials"][0] == {
        "min_margin": 0.02,
        "selected_correct": 1,
        "switches": 1,
        "helpful_switches": 1,
        "harmful_switches": 0,
        "all_feature_switches": 0,
    }


def test_persistent_geometry_bonus_is_positive_and_range_scoped() -> None:
    obj = {
        "bbox_corners": [
            [-0.2, -0.1, 0.0],
            [0.2, -0.1, 0.0],
            [-0.2, 0.1, 0.0],
            [-0.2, -0.1, 0.3],
            [0.2, 0.1, 0.3],
            [-0.2, 0.1, 0.3],
            [0.2, -0.1, 0.3],
            [0.2, 0.1, 0.0],
        ],
    }
    extent = _horizontal_extent_m(obj)
    assert extent == 0.4
    constraints = {
        "monitor": {
            "source_labels": ["monitor", "television"],
            "max_horizontal_extent_m": 0.68,
        },
        "television": {"min_horizontal_extent_m": 0.75},
    }
    assert _persistent_geometry_adjustments(
        current_label="television",
        candidates=["monitor", "television"],
        horizontal_extent_m=extent,
        score_bonus=0.06,
        constraints_by_label=constraints,
    ) == {"monitor": 0.06}
    assert _persistent_geometry_adjustments(
        current_label="television",
        candidates=["monitor", "television"],
        horizontal_extent_m=0.72,
        score_bonus=0.06,
        constraints_by_label=constraints,
    ) == {}
    assert _persistent_geometry_adjustments(
        current_label="chair",
        candidates=["chair", "monitor"],
        horizontal_extent_m=extent,
        score_bonus=0.06,
        constraints_by_label=constraints,
    ) == {}
