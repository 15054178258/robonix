import importlib.util
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def load_bridge():
    spec = importlib.util.spec_from_file_location(
        "map_to_odom_bridge",
        ROOT / "src" / "mapping_rbnx" / "odom_bridge_math.py",
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class MapToOdomMathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_bridge()

    def assertPoseAlmostEqual(self, actual, expected):
        self.assertAlmostEqual(actual.x, expected.x, places=7)
        self.assertAlmostEqual(actual.y, expected.y, places=7)
        self.assertAlmostEqual(
            self.bridge.wrap_angle(actual.yaw - expected.yaw), 0.0, places=7
        )

    def test_compose_with_inverse_is_identity(self):
        pose = self.bridge.Pose2(2.3, -1.7, 1.2)
        self.assertPoseAlmostEqual(
            self.bridge.compose(pose, self.bridge.inverse(pose)),
            self.bridge.Pose2(0.0, 0.0, 0.0),
        )

    def test_bridge_equation_reconstructs_rtabmap_pose(self):
        map_to_icp = self.bridge.Pose2(5.0, 2.0, math.radians(20.0))
        icp_to_base = self.bridge.Pose2(1.2, -0.1, math.radians(5.0))
        odom_to_base = self.bridge.Pose2(-0.4, 0.2, math.radians(-12.0))
        map_to_base = self.bridge.compose(map_to_icp, icp_to_base)
        map_to_odom = self.bridge.compose(
            map_to_base, self.bridge.inverse(odom_to_base)
        )
        reconstructed = self.bridge.compose(map_to_odom, odom_to_base)
        self.assertPoseAlmostEqual(reconstructed, map_to_base)

    def test_interpolation_uses_shortest_angle_across_pi(self):
        a = self.bridge.TimedPose2(
            0, self.bridge.Pose2(0.0, 0.0, math.radians(170.0))
        )
        b = self.bridge.TimedPose2(
            10, self.bridge.Pose2(10.0, 2.0, math.radians(-170.0))
        )
        result = self.bridge.interpolate(a, b, 5)
        self.assertAlmostEqual(result.x, 5.0)
        self.assertAlmostEqual(result.y, 1.0)
        self.assertAlmostEqual(abs(result.yaw), math.pi)


if __name__ == "__main__":
    unittest.main()
