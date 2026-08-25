from __future__ import annotations

import importlib
import sys
import types
import unittest

from go2_d435i_provider.quality import QualityResult


class FakePrimitive:
    def __init__(self, *, id: str, namespace: str) -> None:
        self.id = id
        self.namespace = namespace
        self.declared: list[tuple[str, str, str]] = []

    def on_init(self, callback):
        self.init_callback = callback
        return callback

    def on_activate(self, callback):
        self.activate_callback = callback
        return callback

    def on_deactivate(self, callback):
        self.deactivate_callback = callback
        return callback

    def on_shutdown(self, callback):
        self.shutdown_callback = callback
        return callback

    def declare_ros2_topic(self, contract: str, topic: str, *, qos: str) -> None:
        self.declared.append((contract, topic, qos))

    def run(self) -> None:
        raise AssertionError("offline provider test must not run the registrar")


def load_provider():
    fake_api = types.ModuleType("robonix_api")
    fake_api.Primitive = FakePrimitive
    fake_api.Ok = lambda: ("ok", None)
    fake_api.Err = lambda message: ("err", message)
    fake_api.Deferred = lambda message: ("deferred", message)
    sys.modules["robonix_api"] = fake_api
    sys.modules.pop("go2_d435i_provider.main", None)
    return importlib.import_module("go2_d435i_provider.main")


def valid_config() -> dict:
    return {
        "source_mode": "external",
        "rgb_topic": "/d435i/color/image_raw",
        "depth_topic": "/d435i/aligned_depth_to_color/image_raw",
        "camera_info_topic": "/d435i/color/camera_info",
        "rgb_frame": "d435i_color_optical_frame",
        "depth_frame": "d435i_color_optical_frame",
        "sentinel_timeout_s": 10.0,
        "quality_window_s": 2.0,
        "min_rate_hz": 5.0,
        "max_stamp_age_s": 0.5,
        "max_future_skew_s": 0.05,
        "max_rgb_depth_skew_s": 0.05,
    }


class ProviderContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.module = load_provider()

    def test_identity_and_external_activation_declare_exact_data_contracts(self) -> None:
        self.assertEqual(self.module.provider.id, "go2_d435i")
        self.assertEqual(
            self.module.provider.namespace,
            "robonix/primitive/camera",
        )
        self.module._quality_observer = lambda config: QualityResult(
            True,
            "offline pass",
            {"ros_publishers_created": False, "config": dict(config)},
        )
        self.assertEqual(self.module.initialize(valid_config()), ("ok", None))
        self.assertEqual(self.module.activate(), ("ok", None))
        self.assertEqual(
            self.module.provider.declared,
            [
                (
                    "robonix/primitive/camera/rgb",
                    "/d435i/color/image_raw",
                    "best_effort",
                ),
                (
                    "robonix/primitive/camera/depth",
                    "/d435i/aligned_depth_to_color/image_raw",
                    "best_effort",
                ),
                (
                    "robonix/primitive/camera/intrinsics",
                    "/d435i/color/camera_info",
                    "best_effort",
                ),
            ],
        )
        self.assertFalse(
            self.module._last_quality_evidence["ros_publishers_created"]
        )
        self.assertEqual(
            self.module.activate(),
            ("deferred", "D435i registrar is already active"),
        )

    def test_defaults_match_the_ephemeral_orin_bridge(self) -> None:
        normalized = self.module._normalize_config({"source_mode": "external"})
        self.assertEqual(
            normalized["rgb_topic"],
            "/go2/d435i/color/image_raw",
        )
        self.assertEqual(
            normalized["depth_topic"],
            "/go2/d435i/aligned_depth_to_color/image_raw",
        )
        self.assertEqual(
            normalized["camera_info_topic"],
            "/go2/d435i/color/camera_info",
        )
        self.assertEqual(normalized["rgb_frame"], "d435i_color_optical_frame")
        self.assertEqual(normalized["depth_frame"], "d435i_color_optical_frame")

    def test_quality_failure_declares_nothing(self) -> None:
        self.module._quality_observer = lambda _config: QualityResult(
            False,
            "synthetic malformed depth",
            {"ros_publishers_created": False},
        )
        self.assertEqual(self.module.initialize(valid_config()), ("ok", None))
        result = self.module.activate()
        self.assertEqual(result[0], "err")
        self.assertIn("synthetic malformed depth", result[1])
        self.assertEqual(self.module.provider.declared, [])
        self.assertFalse(self.module._active)

    def test_only_exact_external_mode_is_accepted(self) -> None:
        for value in (None, "", "local", "automatic", "EXTERNAL"):
            with self.subTest(value=value):
                module = load_provider()
                config = valid_config()
                if value is None:
                    config.pop("source_mode")
                else:
                    config["source_mode"] = value
                self.assertEqual(
                    module.initialize(config),
                    ("err", "source_mode must be exactly 'external'"),
                )

    def test_topics_frames_and_bounds_fail_closed(self) -> None:
        mutations = (
            ("relative_topic", {"rgb_topic": "d435i/rgb"}),
            ("duplicate_topic", {"depth_topic": "/d435i/color/image_raw"}),
            ("different_frames", {"depth_frame": "d435i_depth_optical_frame"}),
            ("short_sentinel", {"sentinel_timeout_s": 0.5}),
            (
                "window_not_shorter",
                {"sentinel_timeout_s": 5.0, "quality_window_s": 5.0},
            ),
            ("unsafe_age", {"max_stamp_age_s": 3.0}),
            ("unsafe_future", {"max_future_skew_s": 0.2}),
        )
        for label, changes in mutations:
            with self.subTest(label=label):
                module = load_provider()
                config = valid_config()
                config.update(changes)
                result = module.initialize(config)
                self.assertEqual(result[0], "err")

    def test_declared_endpoints_cannot_change_without_restart(self) -> None:
        self.module._quality_observer = lambda _config: QualityResult(
            True,
            "pass",
            {"ros_publishers_created": False},
        )
        self.assertEqual(self.module.initialize(valid_config()), ("ok", None))
        self.assertEqual(self.module.activate(), ("ok", None))
        self.assertEqual(self.module.deactivate(), ("ok", None))
        changed = valid_config()
        changed["rgb_topic"] = "/other/color/image_raw"
        result = self.module.initialize(changed)
        self.assertEqual(result[0], "err")
        self.assertIn("restart to change config", result[1])


if __name__ == "__main__":
    unittest.main()
