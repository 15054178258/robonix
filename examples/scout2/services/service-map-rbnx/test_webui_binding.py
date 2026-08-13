from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent


class WebUiBindingTest(unittest.TestCase):
    def test_container_forwards_loopback_safe_webui_host(self):
        source = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertIn(
            '-e MAPPING_WEBUI_HOST="${MAPPING_WEBUI_HOST:-127.0.0.1}"',
            source,
        )

    def test_webui_default_is_loopback(self):
        source = (ROOT / "src" / "mapping_rbnx" / "webui.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'os.environ.get("MAPPING_WEBUI_HOST", "127.0.0.1")', source
        )
        self.assertNotIn(
            'os.environ.get("MAPPING_WEBUI_HOST", "0.0.0.0")', source
        )

    def test_driver_config_propagates_webui_host(self):
        source = (ROOT / "src" / "mapping_rbnx" / "atlas_bridge.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('cfg.get(\n        "webui_host"', source)
        self.assertIn('os.environ["MAPPING_WEBUI_HOST"] = _webui_host', source)
        self.assertIn('os.environ.pop("MAPPING_WEBUI_PORT", None)', source)

    def test_config_spec_declares_loopback_safe_webui_host(self):
        source = (ROOT / "config.spec").read_text(encoding="utf-8")
        self.assertIn("  webui_host:\n", source)
        self.assertIn("    default: 127.0.0.1\n", source)


if __name__ == "__main__":
    unittest.main()
