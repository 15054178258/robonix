from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
HELPER_PATH = ROOT / "scripts" / "materialize_scene_start_config.py"
WRAPPER_PATH = ROOT / "scripts" / "start_workstation_full_nomotion_corrected.sh"

SPEC = importlib.util.spec_from_file_location("materialize_scene_start_config", HELPER_PATH)
assert SPEC is not None and SPEC.loader is not None
HELPER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = HELPER
SPEC.loader.exec_module(HELPER)


class SceneStartConfigTests(unittest.TestCase):
    def _manifest(
        self,
        directory: Path,
        scene: dict[str, object],
    ) -> Path:
        path = directory / "manifest.yaml"
        path.write_text(
            yaml.safe_dump(
                {
                    "system": {
                        "scene": scene,
                        "pilot": {"api_key": "must-not-be-copied"},
                    },
                    "env": {"VLM_API_KEY": "must-not-be-copied"},
                }
            ),
            encoding="utf-8",
        )
        return path.resolve()

    def test_extracts_only_allowlisted_preview_config_and_writes_private_file(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            os.chmod(directory, 0o700)
            scene = {
                "log": "info",
                "web_host": "127.0.0.1",
                "web_port": 50107,
                "provider_ids": dict(HELPER.D435I_PROVIDER_PINS),
                "camera_frame": HELPER.D435I_OPTICAL_FRAME,
                "perception_enabled": False,
                "future_secret": "must-not-be-copied",
            }
            manifest = self._manifest(directory, scene)
            output = (directory / "scene.json").resolve()

            config = HELPER.load_scene_config(
                manifest,
                require_d435i_preview=True,
            )
            HELPER.write_private_config(output, config)

            written = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(set(written), set(HELPER.SCENE_START_KEYS))
            self.assertEqual(written["provider_ids"], HELPER.D435I_PROVIDER_PINS)
            self.assertIs(written["perception_enabled"], False)
            self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)
            self.assertNotIn("must-not-be-copied", output.read_text(encoding="utf-8"))

    def test_preview_contract_rejects_mixed_camera_or_enabled_perception(self) -> None:
        invalid = (
            {
                "provider_ids": {
                    "rgb": "go2_sensors",
                    "depth": "go2_d435i",
                    "intrinsics": "go2_d435i",
                },
                "camera_frame": HELPER.D435I_OPTICAL_FRAME,
                "perception_enabled": False,
            },
            {
                "provider_ids": dict(HELPER.D435I_PROVIDER_PINS),
                "camera_frame": HELPER.D435I_OPTICAL_FRAME,
                "perception_enabled": True,
            },
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            os.chmod(directory, 0o700)
            for index, scene in enumerate(invalid):
                with self.subTest(scene=scene):
                    manifest = directory / f"manifest-{index}.yaml"
                    manifest.write_text(
                        yaml.safe_dump({"system": {"scene": scene}}),
                        encoding="utf-8",
                    )
                    with self.assertRaises(HELPER.SceneConfigError):
                        HELPER.load_scene_config(
                            manifest.resolve(),
                            require_d435i_preview=True,
                        )

    def test_rejects_missing_scene_existing_output_and_public_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            os.chmod(directory, 0o700)
            manifest = self._manifest(directory, {})
            with self.assertRaises(HELPER.SceneConfigError):
                HELPER.load_scene_config(
                    manifest,
                    require_d435i_preview=False,
                )

            output = directory / "scene.json"
            output.write_text("{}\n", encoding="utf-8")
            with self.assertRaises(HELPER.SceneConfigError):
                HELPER.write_private_config(output.resolve(), {"web_port": 50107})

            public = directory / "public"
            public.mkdir(mode=0o755)
            os.chmod(public, 0o755)
            with self.assertRaises(HELPER.SceneConfigError):
                HELPER.write_private_config(
                    (public / "scene.json").resolve(),
                    {"web_port": 50107},
                )

    def test_wrapper_materializes_before_private_stack_export(self) -> None:
        source = WRAPPER_PATH.read_text(encoding="utf-8")
        render = source.index('"$PYTHON" "$MANIFEST_RENDERER"')
        materialize = source.index(
            '"$PYTHON" "$ROOT/scripts/materialize_scene_start_config.py"'
        )
        export = source.index('export RBNX_CONFIG_FILE="$SCENE_START_CONFIG"')
        boot = source.index('exec bash "$ROOT/start.sh"')
        self.assertLess(render, materialize)
        self.assertLess(materialize, export)
        self.assertLess(export, boot)
        self.assertIn("--require-d435i-preview", source)


if __name__ == "__main__":
    unittest.main()
