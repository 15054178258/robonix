from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class OfficialClientSceneProfileTest(unittest.TestCase):
    def test_scene_ui_is_enabled_on_loopback_port_50107(self) -> None:
        manifest = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        scene = manifest["system"]["scene"]
        self.assertEqual(scene["web_host"], "127.0.0.1")
        self.assertEqual(scene["web_port"], 50107)

    def test_client_template_has_no_credentials_and_targets_local_atlas(self) -> None:
        path = ROOT / "config" / "robonix_client_settings.yaml"
        settings = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.assertEqual(settings["robotHost"], "127.0.0.1")
        self.assertEqual(settings["atlasPort"], 50051)
        self.assertEqual(settings["language"], "zh-CN")
        lowered = path.read_text(encoding="utf-8").lower()
        for forbidden in ("api_key", "secret_key", "password", "token"):
            self.assertNotIn(forbidden, lowered)

    def test_client_launcher_is_loopback_only_and_does_not_start_robot(self) -> None:
        source = (ROOT / "scripts" / "start_robonix_client_local.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--host 127.0.0.1", source)
        self.assertIn("--audio-server-bind-host 127.0.0.1", source)
        self.assertIn("--audio-server-ui-host 127.0.0.1", source)
        self.assertIn("ROBONIX_CLIENT_REVERSE_AUDIO", source)
        self.assertIn("rbnx-build/client", source)
        for forbidden in (
            "sudo ",
            "\napt ",
            "ros2 topic pub",
            "/api/sport/request",
            "/lowcmd",
            "sport_mode_ctrl",
        ):
            self.assertNotIn(forbidden, source)

    def test_ui_checker_is_read_only_and_covers_all_operator_surfaces(self) -> None:
        source = (ROOT / "scripts" / "check_robonix_ui_stack.sh").read_text(
            encoding="utf-8"
        )
        for port in ("50107", "8091", "8092", "7860"):
            self.assertIn(port, source)
        self.assertNotIn("curl -X", source)
        self.assertNotIn("ros2 ", source)


if __name__ == "__main__":
    unittest.main()
