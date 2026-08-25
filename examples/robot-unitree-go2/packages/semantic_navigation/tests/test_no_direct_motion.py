from pathlib import Path
import unittest


class MotionSurfaceTest(unittest.TestCase):
    def test_skill_has_no_direct_motion_transport(self) -> None:
        root = Path(__file__).resolve().parents[1]
        source = "\n".join(
            path.read_text(encoding="utf-8", errors="replace")
            for path in root.rglob("*.py")
            if "tests" not in path.parts
        )
        forbidden = ("SportClient", "/lowcmd", "create_publisher", "cmd_vel")
        for token in forbidden:
            self.assertNotIn(token, source, token)


if __name__ == "__main__":
    unittest.main()
