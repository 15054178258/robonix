from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
from io import StringIO
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "scripts" / "check_runtime_ownership.py"
WRAPPER = ROOT / "scripts" / "check_runtime_ownership.sh"


def load_checker():
    spec = importlib.util.spec_from_file_location(
        "check_runtime_ownership_under_test", CHECKER
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {CHECKER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RuntimeOwnershipTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.checker = load_checker()

    def run_checker(
        self, profile: str, phase: str, counts: tuple[int, ...]
    ) -> tuple[int, str, str]:
        output = StringIO()
        errors = StringIO()
        clock = iter((0.0, 0.0, 0.0))

        def query_factory():
            return lambda: counts, lambda: None

        with redirect_stdout(output), redirect_stderr(errors):
            status = self.checker.run(
                profile,
                phase,
                environment={
                    "GO2_OWNERSHIP_DISCOVERY_TIMEOUT_S": "1",
                    "GO2_OWNERSHIP_STABILITY_SAMPLES": "1",
                    "GO2_OWNERSHIP_QUERY_TIMEOUT_S": "1",
                },
                query_factory=query_factory,
                monotonic=lambda: next(clock, 2.0),
                sleep=lambda _seconds: None,
            )
        return status, output.getvalue(), errors.getvalue()

    def test_low_level_checker_has_no_ros_communication_endpoints(self) -> None:
        source = CHECKER.read_text(encoding="utf-8")
        wrapper = WRAPPER.read_text(encoding="utf-8")
        self.assertIn("_rclpy.Node(", source)
        self.assertIn("get_count_publishers", source)
        self.assertNotIn("from rclpy.node import Node", source)
        self.assertNotIn("create_publisher", source)
        self.assertNotIn("create_subscription", source)
        self.assertNotIn("ros2 topic info", wrapper.replace("`", ""))

    def test_local_preflight_requires_an_empty_owned_graph(self) -> None:
        status, output, errors = self.run_checker(
            "workstation-local", "pre", (0, 0, 0, 0, 0, 0)
        )
        self.assertEqual(status, 0, errors)
        self.assertIn("/odom publishers=0", output)

    def test_nx_sensor_owner_has_exactly_one_sensor_publisher(self) -> None:
        status, output, errors = self.run_checker(
            "workstation-full-nx-sensors", "pre", (1, 1, 1, 1, 0, 0)
        )
        self.assertEqual(status, 0, errors)
        self.assertIn("/tf_static publishers=0", output)

    def test_duplicate_camera_publisher_fails_immediately(self) -> None:
        status, _output, errors = self.run_checker(
            "workstation-full-nx-sensors", "pre", (2, 1, 1, 1, 0, 0)
        )
        self.assertEqual(status, 5)
        self.assertIn("publisher ownership violation", errors)

    def test_unknown_profile_and_phase_are_rejected(self) -> None:
        status, _output, errors = self.run_checker(
            "automatic", "pre", (0, 0, 0, 0, 0, 0)
        )
        self.assertEqual(status, 2)
        self.assertIn("unknown runtime ownership profile", errors)
        status, _output, errors = self.run_checker(
            "workstation-local", "automatic", (0, 0, 0, 0, 0, 0)
        )
        self.assertEqual(status, 2)
        self.assertIn("ownership phase", errors)

    def test_post_start_requires_exactly_one_final_owner(self) -> None:
        status, output, errors = self.run_checker(
            "workstation-full-nomotion-corrected",
            "post",
            (1, 1, 1, 1, 1, 1),
        )
        self.assertEqual(status, 0, errors)
        self.assertIn("/odom publishers=1", output)

        status, _output, errors = self.run_checker(
            "workstation-full-nomotion-corrected",
            "post",
            (1, 1, 1, 1, 2, 1),
        )
        self.assertEqual(status, 5)
        self.assertIn("publisher ownership violation: /odom", errors)

        status, _output, errors = self.run_checker(
            "workstation-full-nomotion-corrected",
            "post",
            (1, 1, 1, 1, 0, 1),
        )
        self.assertEqual(status, 6)
        self.assertIn("discovery deadline expired", errors)

    def test_nx_sensors_only_post_start_omits_odom_and_tf(self) -> None:
        status, _output, errors = self.run_checker(
            "nx-sensors-only", "post", (1, 1, 1, 1, 0, 0)
        )
        self.assertEqual(status, 0, errors)


if __name__ == "__main__":
    unittest.main()
