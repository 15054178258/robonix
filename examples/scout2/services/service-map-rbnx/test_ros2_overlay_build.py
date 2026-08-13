#!/usr/bin/env python3
"""Contracts for the generated ROS 2 interface overlay build."""

from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent
HELPER = ROOT / "scripts" / "build_ros2_overlay.sh"


class Ros2OverlayBuildTests(unittest.TestCase):
    @staticmethod
    def _write_fake_colcon(fake_bin: Path) -> None:
        fake_bin.mkdir()
        fake_colcon = fake_bin / "colcon"
        fake_colcon.write_text(
            '#!/usr/bin/env bash\n'
            'printf "cwd=%s\\n" "$PWD" > "$COLCON_LOG"\n'
            'printf "arg=%s\\n" "$@" >> "$COLCON_LOG"\n',
            encoding="utf-8",
        )
        fake_colcon.chmod(0o755)

    def test_all_targets_use_the_shared_overlay_helper(self) -> None:
        build = (ROOT / "scripts" / "build.sh").read_text(encoding="utf-8")

        self.assertEqual(build.count("build_ros2_overlay.sh"), 2)
        self.assertNotIn("(cd \"$ROS2_IDL\" && colcon build)", build)
        self.assertNotIn("colcon build --packages-up-to map", build)

    def test_helper_builds_only_the_custom_map_package(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            overlay = root / "ros2_idl"
            fake_bin = root / "bin"
            (overlay / "src" / "map").mkdir(parents=True)
            stale_standard_package = overlay / "install" / "sensor_msgs" / "marker"
            stale_standard_package.parent.mkdir(parents=True)
            stale_standard_package.write_text("stale", encoding="utf-8")
            log = root / "colcon.log"
            self._write_fake_colcon(fake_bin)

            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{fake_bin}:/usr/bin:/bin",
                    "COLCON_LOG": str(log),
                }
            )
            subprocess.run(
                ["bash", str(HELPER), str(overlay)],
                check=True,
                env=env,
                capture_output=True,
                text=True,
            )

            self.assertEqual(
                log.read_text(encoding="utf-8").splitlines(),
                [
                    f"cwd={overlay}",
                    "arg=build",
                    "arg=--packages-select",
                    "arg=map",
                ],
            )
            self.assertFalse(stale_standard_package.exists())

    def test_helper_keeps_clean_map_outputs_incremental(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            overlay = root / "ros2_idl"
            fake_bin = root / "bin"
            (overlay / "src" / "map").mkdir(parents=True)
            incremental_marker = overlay / "build" / "map" / "marker"
            incremental_marker.parent.mkdir(parents=True)
            incremental_marker.write_text("keep", encoding="utf-8")
            log = root / "colcon.log"
            self._write_fake_colcon(fake_bin)

            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{fake_bin}:/usr/bin:/bin",
                    "COLCON_LOG": str(log),
                }
            )
            subprocess.run(
                ["bash", str(HELPER), str(overlay)],
                check=True,
                env=env,
                capture_output=True,
                text=True,
            )

            self.assertTrue(incremental_marker.exists())

    def test_helper_fails_closed_when_map_was_not_generated(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                ["bash", str(HELPER), directory],
                check=False,
                capture_output=True,
                text=True,
            )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("generated map package missing", result.stderr)


if __name__ == "__main__":
    unittest.main()
