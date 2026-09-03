from importlib import util
import math
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "refine_initialpose_offline.py"


def load_module():
    spec = util.spec_from_file_location("refine_initialpose_offline", SCRIPT)
    module = util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_grid(module, *, width=120, height=100, resolution=0.05):
    data = [0] * (width * height)
    return module.GridMap(
        width,
        height,
        resolution,
        module.Pose2D(0.0, 0.0, 0.0),
        tuple(data),
    )


def with_occupied(module, grid, cells):
    data = list(grid.occupancy)
    for x, y in cells:
        data[y * grid.width + x] = 100
    return module.GridMap(
        grid.width,
        grid.height,
        grid.resolution,
        grid.origin,
        tuple(data),
    )


class RefineInitialposeOfflineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()
        cls.source = SCRIPT.read_text(encoding="utf-8")

    def test_script_is_strictly_offline_and_has_no_ros_write_surface(self):
        self.assertIn("offline tool", self.source)
        self.assertIn('"publishes_ros_topics": False', self.source)
        for forbidden in (
            "import rclpy",
            "create_node(",
            "create_publisher",
            "ros2 topic",
            "/cmd_vel",
            "/api/sport/request",
            "/lowcmd",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_pgm_loading_flips_image_rows_and_preserves_unknown(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            root = Path(directory)
            image = root / "map.pgm"
            # PGM is top row first.  0=occupied, 254=free, 205=unknown.
            image.write_bytes(b"P5\n2 2\n255\n" + bytes([0, 254, 205, 0]))
            metadata = root / "map.yaml"
            metadata.write_text(
                "\n".join(
                    (
                        "image: map.pgm",
                        "resolution: 0.1",
                        "origin: [1.0, 2.0, 0.0]",
                        "negate: 0",
                        "occupied_thresh: 0.65",
                        "free_thresh: 0.25",
                    )
                )
                + "\n",
                encoding="utf-8",
            )
            grid, loaded_image = self.module.load_map_yaml(metadata)
        self.assertEqual(loaded_image, image)
        self.assertEqual(grid.occupancy, (-1, 100, 100, 0))

    def test_unknown_and_out_of_bounds_are_fixed_denominator_penalties(self):
        grid = make_grid(self.module, width=20, height=20, resolution=1.0)
        data = list(grid.occupancy)
        data[5 * grid.width + 6] = 100
        data[5 * grid.width + 7] = -1
        grid = self.module.GridMap(
            grid.width,
            grid.height,
            grid.resolution,
            grid.origin,
            tuple(data),
        )
        field = self.module.occupied_distance_field(grid)
        beams = self.module.BeamSet(
            endpoint_x=(1.0, 2.0, 30.0),
            endpoint_y=(0.0, 0.0, 0.0),
            source_scan_count=1,
        )
        config = self.module.SearchConfig(
            footprint_radius_m=0.0,
            hit_sigma_m=0.5,
        )
        score = self.module.score_pose(
            grid,
            field,
            beams,
            self.module.Pose2D(5.0, 5.0, 0.0),
            config,
        )
        self.assertEqual(score.beam_count, 3)
        self.assertEqual(score.known_count, 1)
        self.assertEqual(score.unknown_count, 1)
        self.assertEqual(score.out_of_bounds_count, 1)
        self.assertAlmostEqual(
            score.score,
            (1.0 + config.unknown_penalty + config.out_of_bounds_penalty) / 3.0,
        )
        with self.assertRaisesRegex(ValueError, "coordinate lengths"):
            self.module.score_pose(
                grid,
                field,
                self.module.BeamSet((1.0, 2.0), (0.0,), 1),
                self.module.Pose2D(5.0, 5.0, 0.0),
                config,
            )

    def test_asymmetric_local_peak_recovers_pose(self):
        grid = make_grid(self.module, width=140, height=120, resolution=0.05)
        truth = self.module.Pose2D(3.0, 2.5, math.radians(10.0))
        angles = [math.radians(-70.0 + index * 10.0) for index in range(15)]
        ranges = [
            0.75, 0.90, 1.10, 1.35, 1.55,
            1.20, 1.70, 1.00, 1.45, 0.85,
            1.30, 1.60, 0.95, 1.40, 1.05,
        ]
        cells = set()
        for angle, distance in zip(angles, ranges):
            world_angle = truth.yaw + angle
            x = truth.x + distance * math.cos(world_angle)
            y = truth.y + distance * math.sin(world_angle)
            cell = self.module.world_to_cell(grid, x, y)
            cells.add(cell)
        grid = with_occupied(self.module, grid, cells)
        beams = self.module.BeamSet(
            tuple(distance * math.cos(angle) for angle, distance in zip(angles, ranges)),
            tuple(distance * math.sin(angle) for angle, distance in zip(angles, ranges)),
            1,
        )
        initial = self.module.Pose2D(2.90, 2.42, math.radians(6.0))
        config = self.module.SearchConfig(
            search_xy_m=0.20,
            search_yaw_rad=math.radians(8.0),
            xy_step_m=0.05,
            yaw_step_rad=math.radians(2.0),
            refinement_levels=1,
            footprint_radius_m=0.0,
            min_beams=10,
            min_score=0.60,
            min_known_fraction=0.95,
            min_hit_fraction=0.80,
            min_improvement=0.05,
            unique_translation_m=0.10,
            unique_yaw_rad=math.radians(4.0),
            min_unique_margin=0.05,
        )
        result = self.module.refine_pose(grid, beams, initial, config)
        self.assertEqual(result["decision"], "accept_refinement")
        self.assertTrue(result["apply_recommended"])
        self.assertLess(abs(result["best"]["pose"]["x"] - truth.x), 0.04)
        self.assertLess(abs(result["best"]["pose"]["y"] - truth.y), 0.04)
        self.assertLess(
            self.module.angular_distance(result["best"]["pose"]["yaw"], truth.yaw),
            math.radians(2.0),
        )
        self.assertEqual(result["initial"]["beam_count"], beams.count)
        self.assertEqual(result["best"]["beam_count"], beams.count)

    def test_parallel_wall_is_rejected_as_ambiguous_or_boundary(self):
        grid = make_grid(self.module, width=140, height=140, resolution=0.05)
        wall_x = 100
        grid = with_occupied(
            self.module,
            grid,
            {(wall_x, cell_y) for cell_y in range(10, 130)},
        )
        pose = self.module.Pose2D(3.0, 3.5, 0.0)
        angles = [math.radians(-30.0 + index * 2.0) for index in range(31)]
        wall_world_x = wall_x * grid.resolution + grid.resolution * 0.5
        ranges = [
            (wall_world_x - pose.x) / math.cos(angle)
            for angle in angles
        ]
        beams = self.module.BeamSet(
            tuple(distance * math.cos(angle) for angle, distance in zip(angles, ranges)),
            tuple(distance * math.sin(angle) for angle, distance in zip(angles, ranges)),
            1,
        )
        config = self.module.SearchConfig(
            search_xy_m=0.20,
            search_yaw_rad=math.radians(6.0),
            xy_step_m=0.05,
            yaw_step_rad=math.radians(2.0),
            refinement_levels=1,
            footprint_radius_m=0.0,
            min_score=0.5,
            min_hit_fraction=0.8,
            min_improvement=0.001,
            min_unique_margin=0.03,
        )
        result = self.module.refine_pose(grid, beams, pose, config)
        self.assertFalse(result["apply_recommended"])
        self.assertIn(
            result["decision"],
            {"keep_initial", "reject_ambiguous", "reject_boundary"},
        )
        self.assertLess(result["unique_margin"], config.min_unique_margin)

    def test_output_path_is_confined_to_repository(self):
        self.assertTrue(
            self.module.resolve_output("rbnx-build/run/refinement.json").is_relative_to(ROOT)
        )
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "must remain below"):
                self.module.resolve_output(Path(directory) / "outside.json")


if __name__ == "__main__":
    unittest.main()
