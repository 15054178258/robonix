from importlib import util
from pathlib import Path
from types import SimpleNamespace
import math
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "compute_path_to_pose_nomotion.py"


def load_module():
    spec = util.spec_from_file_location("compute_path_to_pose_nomotion", SCRIPT)
    module = util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def stamped(x, y, frame_id="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


class ComputePathToPoseNomotionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module()
        cls.source = SCRIPT.read_text(encoding="utf-8")

    def test_script_has_only_planner_action_surface(self) -> None:
        self.assertIn("ComputePathToPose", self.source)
        self.assertIn('ACTION_NAME = "/compute_path_to_pose"', self.source)
        for forbidden in (
            "NavigateToPose",
            "FollowPath",
            "Twist",
            "/cmd_vel",
            "/api/sport/request",
            "/lowcmd",
            "unitree",
            "create_publisher",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_path_summary_is_finite_and_measured(self) -> None:
        path = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            poses=[stamped(1.0, 2.0), stamped(4.0, 6.0)],
        )
        summary = self.module.summarize_path(path)
        self.assertEqual(summary["pose_count"], 2)
        self.assertEqual(summary["length_m"], 5.0)
        self.assertEqual(summary["start"], {"x": 1.0, "y": 2.0})
        self.assertEqual(summary["end"], {"x": 4.0, "y": 6.0})

    def test_path_summary_rejects_wrong_frame_empty_or_nonfinite(self) -> None:
        with self.assertRaisesRegex(ValueError, "path frame"):
            self.module.summarize_path(
                SimpleNamespace(
                    header=SimpleNamespace(frame_id="odom"),
                    poses=[stamped(0.0, 0.0, frame_id="odom")],
                )
            )
        with self.assertRaisesRegex(ValueError, "empty path"):
            self.module.summarize_path(
                SimpleNamespace(
                    header=SimpleNamespace(frame_id="map"),
                    poses=[],
                )
            )
        with self.assertRaisesRegex(ValueError, "not finite"):
            self.module.summarize_path(
                SimpleNamespace(
                    header=SimpleNamespace(frame_id="map"),
                    poses=[stamped(math.nan, 0.0)],
                )
            )

    def test_output_is_confined_to_repository(self) -> None:
        relative = self.module.resolve_output("rbnx-build/run/plan-only.json")
        self.assertTrue(relative.is_relative_to(ROOT))
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "must remain below"):
                self.module.resolve_output(str(Path(directory) / "outside.json"))


if __name__ == "__main__":
    unittest.main()
