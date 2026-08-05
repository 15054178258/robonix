import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent


class RangerV3DefaultsTest(unittest.TestCase):
    def test_wrapper_launches_v3_with_v3_defaults(self):
        source = (ROOT / "ranger_chassis" / "main.py").read_text()
        self.assertIn('"ranger_mini_v3.launch.xml"', source)
        self.assertIn("cfg.get('robot_model', 'ranger_mini_v3')", source)
        self.assertIn("cfg.get('port_name', 'can_ranger')", source)

    def test_scout_spawn_exists(self):
        source = (ROOT / "ranger_chassis" / "main.py").read_text()
        self.assertIn('"scout_base.launch.py"', source)
        self.assertIn('"scout_base"', source)

    def test_manifest_and_spec_match_runtime_defaults(self):
        manifest = (ROOT / "package_manifest.yaml").read_text()
        spec = (ROOT / "config.spec").read_text()
        for text in (manifest, spec):
            self.assertIn("can_ranger", text)
            self.assertIn("ranger_mini_v3", text)
        # scout tags present in manifest
        self.assertIn("scout", manifest)

    def test_build_sh_includes_scout_ros2(self):
        build = (ROOT / "scripts" / "build.sh").read_text()
        self.assertIn("scout_ros2", build)


if __name__ == "__main__":
    unittest.main()
