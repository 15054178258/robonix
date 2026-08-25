from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "prepare_staged_nav2_goal_evidence.py"
SPEC = importlib.util.spec_from_file_location(
    "prepare_staged_nav2_goal_evidence", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def pose(x: float, y: float, yaw: float = 0.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        ),
    )


def stamped(x: float, y: float, yaw: float = 0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        pose=pose(x, y, yaw),
    )


def grid(data: list[int], *, width: int = 10, height: int = 10):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=0.1,
            origin=pose(0.0, 0.0),
        ),
        data=data,
    )


class StagedNav2GoalEvidenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SCRIPT.read_text(encoding="utf-8")

    def test_surface_is_plan_only_and_has_no_motion_endpoint(self) -> None:
        self.assertIn('ACTION_NAME = "/compute_path_to_pose"', self.source)
        self.assertIn('"GO2_ALLOW_MOTION"] = "false"', self.source)
        for forbidden in (
            "NavigateToPose",
            "FollowPath",
            "create_publisher",
            "/cmd_vel",
            "/api/sport/request",
            "/lowcmd",
            "/go2_chassis/arm",
            "unitree",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_path_and_endpoint_yaw_are_finite_and_measured(self) -> None:
        path = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            poses=[stamped(0.1, 0.1), stamped(0.35, 0.1, 0.2)],
        )
        points = module.path_points(path)
        self.assertEqual(points, [(0.1, 0.1), (0.35, 0.1)])
        self.assertAlmostEqual(module.path_length(points), 0.25)
        self.assertAlmostEqual(module.pose_yaw(path.poses[-1].pose), 0.2)

    def test_known_free_path_passes_and_unknown_or_occupied_rejects(self) -> None:
        points = [(0.1, 0.1), (0.35, 0.1)]
        result = module.validate_path_against_map(points, grid([0] * 100))
        self.assertGreater(result["sample_count"], 2)

        unknown = [0] * 100
        unknown[13] = -1
        with self.assertRaisesRegex(ValueError, "unknown"):
            module.validate_path_against_map(points, grid(unknown))

        occupied = [0] * 100
        occupied[13] = 100
        with self.assertRaisesRegex(ValueError, "occupied"):
            module.validate_path_against_map(points, grid(occupied))

    def test_exact_check_set_contains_endpoint_and_known_space(self) -> None:
        self.assertEqual(
            module.CHECK_NAMES,
            {
                "goal_bearing_within_stage1",
                "map_lifecycle_exact",
                "plan_only_action",
                "path_collision_free",
                "path_endpoint_matches_goal",
                "path_finite",
                "path_frame_map",
                "path_known_space",
                "path_length_le_0_40_m",
                "path_nonempty",
                "path_start_matches_localization",
            },
        )
        self.assertEqual(module.MAX_PATH_M, 0.40)


if __name__ == "__main__":
    unittest.main()
