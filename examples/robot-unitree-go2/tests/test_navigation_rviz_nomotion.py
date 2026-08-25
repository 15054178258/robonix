from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "go2_navigation_nomotion.rviz"
LAUNCHER = ROOT / "scripts" / "start_navigation_rviz_nomotion.sh"


def _displays(items):
    for item in items:
        yield item
        yield from _displays(item.get("Displays", []))


class NavigationRvizNomotionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.raw_config = CONFIG.read_text(encoding="utf-8")
        cls.config = yaml.safe_load(cls.raw_config)
        cls.raw_launcher = LAUNCHER.read_text(encoding="utf-8")

    def test_fixed_frame_and_reviewed_topics(self) -> None:
        manager = self.config["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        displays = list(_displays(manager["Displays"]))
        topics = {
            display[key]["Value"]
            for display in displays
            for key in ("Topic", "Description Topic")
            if isinstance(display.get(key), dict)
        }
        self.assertTrue(
            {
                "/map",
                "/robot_description",
                "/scanner/scan",
                "/scanner/cloud",
                "/robonix/map/pose",
                "/odom",
                "/global_costmap/costmap",
                "/local_costmap/costmap",
                "/plan",
                "/local_plan",
                "/global_costmap/published_footprint",
                "/local_costmap/published_footprint",
            }.issubset(topics)
        )

    def test_default_profile_has_no_navigation_goal_or_motion_tool(self) -> None:
        tools = self.config["Visualization Manager"]["Tools"]
        classes = {tool["Class"] for tool in tools}
        self.assertNotIn("nav2_rviz_plugins/GoalTool", classes)
        self.assertNotIn("rviz_default_plugins/SetGoal", classes)
        initial_pose = [
            tool for tool in tools
            if tool["Class"] == "rviz_default_plugins/SetInitialPose"
        ]
        self.assertEqual(len(initial_pose), 1)
        self.assertEqual(initial_pose[0]["Topic"]["Value"], "/initialpose")
        for forbidden in ("/cmd_vel", "/api/sport/request", "/lowcmd"):
            self.assertNotIn(forbidden, self.raw_config)

    def test_launcher_is_observation_only(self) -> None:
        self.assertIn("export GO2_ALLOW_MOTION=false", self.raw_launcher)
        self.assertIn("exec rviz2 -d \"$RVIZ_CONFIG\"", self.raw_launcher)
        for forbidden in (
            "ros2 topic pub",
            "ros2 service call",
            "ros2 action send_goal",
            "/cmd_vel",
            "/api/sport/request",
            "/lowcmd",
        ):
            self.assertNotIn(forbidden, self.raw_launcher)
        for line in self.raw_launcher.splitlines():
            command = line.strip()
            self.assertFalse(
                command.startswith(
                    ("sudo ", "apt ", "apt-get ", "nmcli ", "ip address ", "ip route ")
                )
            )


if __name__ == "__main__":
    unittest.main()
