from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from robot_description.runtime import command_for, inspect_urdf, write_params


URDF = """<robot name="test">
  <link name="base_link"/>
  <link name="sensor"/>
  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor"/>
  </joint>
</robot>"""


class RuntimeTest(unittest.TestCase):
    def test_inspect_urdf(self):
        self.assertEqual(inspect_urdf(URDF), ("base_link", 2, 1))

    def test_params_preserve_urdf(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "robot.yaml"
            write_params(path, URDF)
            text = path.read_text(encoding="utf-8")
        self.assertIn("robot_description: |", text)
        self.assertIn('<robot name="test">', text)

    def test_native_command(self):
        command = command_for("native", Path("/tmp/robot.yaml"), "robot_description")
        self.assertEqual(command[-2:], ["--params-file", "/tmp/robot.yaml"])


if __name__ == "__main__":
    unittest.main()
