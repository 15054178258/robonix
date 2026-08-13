import time
import unittest
from unittest.mock import patch

from explore_skill.controller import ExploreController, TaskHandle


class _Grid:
    @staticmethod
    def world_to_cell(_x, _y):
        return (4, 7)


class ExploreControlFlowTest(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = ExploreController(
            map_topic="/map",
            nav_navigate_endpoint="http://127.0.0.1:1/mcp/",
            nav_status_endpoint="http://127.0.0.1:1/mcp/",
            nav_cancel_endpoint="http://127.0.0.1:1/mcp/",
        )
        self.controller._latest_pose_xyyaw = (1.0, 2.0, 0.0)
        self.controller._latest_map = object()

    def _handle(self, legs: int = 0) -> TaskHandle:
        return TaskHandle(
            task_id="test", started_at=time.time(), timeout_s=60.0,
            max_speed_m_s=0.1, legs_completed=legs,
        )

    @patch("explore_skill.frontier.GridView.from_msg", return_value=_Grid())
    def test_sweep_requires_periodic_leg_and_missing_coverage(self, _grid) -> None:
        self.controller._viewed_sectors[(4, 7)] = set()
        self.assertFalse(self.controller._should_sweep(self._handle(1)))
        self.assertFalse(self.controller._should_sweep(self._handle(2)))
        self.assertTrue(self.controller._should_sweep(self._handle(3)))

        self.controller._viewed_sectors[(4, 7)] = set(range(6))
        self.assertFalse(self.controller._should_sweep(self._handle(3)))

    def test_navigation_timeout_cancels_the_accepted_run(self) -> None:
        calls = []

        def rpc(tool, args):
            calls.append((tool, args))
            if tool == "navigate":
                return {"accepted": True, "run_id": "run-42"}
            if tool == "status":
                return {"state": "RUNNING"}
            return {"accepted": True}

        self.controller._mcp_call_sync = rpc
        self.controller.NAV_POLL_PERIOD_S = 0.001
        ok, detail = self.controller._nav_navigate_blocking(
            1.0, 2.0, yaw=None, timeout_s=0.003,
            cancel_evt=self._handle(),
        )

        self.assertFalse(ok)
        self.assertEqual(detail, "leg timeout")
        self.assertIn(("cancel", {"run_id": "run-42"}), calls)

    def test_navigation_cancel_request_cancels_the_accepted_run(self) -> None:
        calls = []

        def rpc(tool, args):
            calls.append((tool, args))
            if tool == "navigate":
                return {"accepted": True, "run_id": "run-7"}
            return {"state": "RUNNING"}

        handle = self._handle()
        handle.cancel_requested = True
        self.controller._mcp_call_sync = rpc
        ok, detail = self.controller._nav_navigate_blocking(
            1.0, 2.0, yaw=None, timeout_s=1.0, cancel_evt=handle,
        )

        self.assertFalse(ok)
        self.assertEqual(detail, "canceled during nav")
        self.assertIn(("cancel", {"run_id": "run-7"}), calls)


if __name__ == "__main__":
    unittest.main()
