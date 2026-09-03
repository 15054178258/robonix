from __future__ import annotations

from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "set_landmark.py"
sys.path.insert(0, str(ROOT / "packages" / "semantic_navigation"))

from semantic_navigation.core import LandmarkError, LandmarkStore  # noqa: E402


class SetLandmarkMultiTest(unittest.TestCase):
    def run_script(self, output: Path, *extra: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--map-id",
                "fixture_map",
                "--generation",
                "12",
                "--x",
                "1.25",
                "--y",
                "-0.5",
                "--yaw",
                "1.57",
                "--measured-by",
                "fixture-operator",
                "--confirm-free-space",
                "YES_POSE_AND_FOOTPRINT_ARE_CLEAR",
                "--output",
                str(output),
                *extra,
            ],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

    def test_add_named_destination_and_non_navigation_marker(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "semantic-landmarks.yaml"
            destination = self.run_script(
                output,
                "--id",
                "east_meeting_room",
                "--name",
                "东侧会议室",
                "--alias",
                "一号会议室",
                "--arrival-radius",
                "0.55",
                "--region-point",
                "0,0",
                "--region-point",
                "2,0",
                "--region-point",
                "2,1",
                "--region-point",
                "0,1",
            )
            self.assertEqual(destination.returncode, 0, destination.stderr)

            marker = self.run_script(
                output,
                "--id",
                "initial_reference",
                "--name",
                "机器人初始位置",
                "--alias",
                "起点",
                "--kind",
                "marker",
            )
            self.assertEqual(marker.returncode, 0, marker.stderr)
            self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)

            store = LandmarkStore.from_path(output)
            destination_item = store.resolve(
                "去一号会议室",
                expected_map_id="fixture_map",
                expected_generation=12,
            )
            self.assertEqual(destination_item.id, "east_meeting_room")
            self.assertEqual(destination_item.arrival_radius, 0.55)
            self.assertEqual(len(destination_item.region or ()), 4)
            with self.assertRaisesRegex(LandmarkError, "non-navigation marker"):
                store.resolve(
                    "返回起点",
                    expected_map_id="fixture_map",
                    expected_generation=12,
                )

            raw = yaml.safe_load(output.read_text(encoding="utf-8"))
            initial = next(row for row in raw["landmarks"] if row["id"] == "initial_reference")
            self.assertTrue(initial["verified"])
            self.assertNotIn("arrival_radius", initial)

    def test_new_id_requires_name_and_bad_region_fails_without_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "semantic-landmarks.yaml"
            missing_name = self.run_script(output, "--id", "new_destination")
            self.assertNotEqual(missing_name.returncode, 0)
            self.assertIn("--name is required", missing_name.stderr)
            self.assertFalse(output.exists())

            bad_region = self.run_script(
                output,
                "--id",
                "new_destination",
                "--name",
                "新目的地",
                "--region-point",
                "0,0",
                "--region-point",
                "1,1",
                "--region-point",
                "2,2",
            )
            self.assertNotEqual(bad_region.returncode, 0)
            self.assertIn("non-zero area", bad_region.stderr)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
