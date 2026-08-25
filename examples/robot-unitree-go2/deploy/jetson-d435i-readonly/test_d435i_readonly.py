import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
BRIDGE_PATH = ROOT / "d435i_readonly_bridge.py"
LAUNCHER_PATH = ROOT / "run-via-ssh.sh"


def load_bridge():
    spec = importlib.util.spec_from_file_location("d435i_readonly_bridge", BRIDGE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class D435iPureFunctionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_bridge()

    def test_acknowledgement_is_exact_and_fail_closed(self):
        self.bridge.require_readonly_streaming_ack(
            self.bridge.READONLY_STREAMING_ACK
        )
        for value in (
            "",
            self.bridge.READONLY_STREAMING_ACK.lower(),
            self.bridge.READONLY_STREAMING_ACK + " ",
        ):
            with self.assertRaises(self.bridge.ConfigurationError):
                self.bridge.require_readonly_streaming_ack(value)

    def test_default_plan_is_640x480_capture_30_publish_10(self):
        plan = self.bridge.make_stream_plan()
        self.assertEqual(plan.namespace, "/go2/d435i")
        self.assertEqual((plan.width, plan.height), (640, 480))
        self.assertEqual(plan.hardware_fps, 30)
        self.assertEqual(plan.publish_hz, 10)
        self.assertEqual(plan.decimation, 3)

    def test_stream_envelope_rejects_higher_load_or_fractional_decimation(self):
        for overrides in (
            {"width": 641},
            {"height": 481},
            {"hardware_fps": 15},
            {"publish_hz": 15},
            {"publish_hz": 7},
        ):
            with self.assertRaises(self.bridge.ConfigurationError):
                self.bridge.make_stream_plan(**overrides)

    def test_decimator_selects_one_of_each_three_hardware_frames(self):
        selected = [
            index
            for index in range(10)
            if self.bridge.should_publish_frame(index, 3)
        ]
        self.assertEqual(selected, [0, 3, 6, 9])

    def test_namespace_is_absolute_and_shell_like_input_is_rejected(self):
        self.assertEqual(
            self.bridge.normalize_namespace("go2//d435i/"),
            "/go2/d435i",
        )
        for value in ("", "/", "/go2/d435i;shutdown", "/go2/d435i value"):
            with self.assertRaises(self.bridge.ConfigurationError):
                self.bridge.normalize_namespace(value)

    def test_depth_scale_must_match_ros_16uc1_millimetres(self):
        self.bridge.validate_depth_scale(0.001)
        with self.assertRaises(self.bridge.ConfigurationError):
            self.bridge.validate_depth_scale(0.0001)

    def test_image_layout_is_exact_and_bounded_before_copy(self):
        self.assertEqual(
            self.bridge.validate_image_layout(640, 480, 1920, "bgr8"),
            (921600, (480, 640, 3)),
        )
        self.assertEqual(
            self.bridge.validate_image_layout(640, 480, 1280, "16UC1"),
            (614400, (480, 640)),
        )
        for values in (
            (641, 480, 1923, "bgr8"),
            (640, 481, 1280, "16UC1"),
            (640, 480, 1919, "bgr8"),
            (640, 480, 1281, "16UC1"),
            (640, 480, 640, "mono8"),
        ):
            with self.subTest(values=values), self.assertRaises(
                (self.bridge.ConfigurationError, RuntimeError)
            ):
                self.bridge.validate_image_layout(*values)

    def test_bridge_memory_ceiling_fails_closed(self):
        self.bridge.validate_bridge_peak_rss(512.0)
        with self.assertRaises(RuntimeError):
            self.bridge.validate_bridge_peak_rss(
                self.bridge.MAX_BRIDGE_PEAK_RSS_MIB + 0.1
            )

    def test_camera_info_uses_rgb_intrinsics_for_aligned_depth_geometry(self):
        fields = self.bridge.camera_info_fields(
            640, 480, 600.0, 601.0, 319.5, 239.5, [0.1, 0.2]
        )
        self.assertEqual(fields["distortion_model"], "plumb_bob")
        self.assertEqual(fields["d"], [0.1, 0.2, 0.0, 0.0, 0.0])
        self.assertEqual(
            fields["k"],
            [600.0, 0.0, 319.5, 0.0, 601.0, 239.5, 0.0, 0.0, 1.0],
        )
        self.assertEqual(len(fields["p"]), 12)


class D435iStaticContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge_source = BRIDGE_PATH.read_text(encoding="utf-8")
        cls.launcher_source = LAUNCHER_PATH.read_text(encoding="utf-8")

    def test_runtime_imports_are_deferred_until_after_ack(self):
        ack_call = self.bridge_source.index(
            "require_readonly_streaming_ack(arguments.readonly_streaming_ack)"
        )
        runtime_call = self.bridge_source.index("return run_bridge(plan)")
        self.assertLess(ack_call, runtime_call)
        prefix = self.bridge_source.split("def _load_runtime_modules", 1)[0]
        self.assertNotIn("import pyrealsense2", prefix)
        self.assertNotIn("import rclpy", prefix)

    def test_only_rgb_depth_and_one_camera_info_are_published(self):
        self.assertEqual(self.bridge_source.count("node.create_publisher("), 3)
        self.assertIn('RGB_TOPIC = "color/image_raw"', self.bridge_source)
        self.assertIn(
            'ALIGNED_DEPTH_TOPIC = "aligned_depth_to_color/image_raw"',
            self.bridge_source,
        )
        self.assertIn(
            'CAMERA_INFO_TOPIC = "color/camera_info"',
            self.bridge_source,
        )
        self.assertEqual(
            self.bridge_source.count("camera_config.enable_stream("),
            2,
        )
        for forbidden in (
            "rs.stream.accel",
            "rs.stream.gyro",
            "TransformBroadcaster",
            "PointCloud2",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
        ):
            self.assertNotIn(forbidden, self.bridge_source)

    def test_all_publishers_use_best_effort_sensor_qos(self):
        self.assertIn(
            "from rclpy.qos import qos_profile_sensor_data",
            self.bridge_source,
        )
        self.assertIn(
            "np, rs, rclpy, sensor_qos, CameraInfo, Image = "
            "_load_runtime_modules()",
            self.bridge_source,
        )
        self.assertEqual(self.bridge_source.count(", sensor_qos)"), 1)
        self.assertEqual(self.bridge_source.count(", sensor_qos\n"), 2)

    def test_frame_copy_uses_bounded_numpy_view_and_rss_fuse(self):
        self.assertNotIn("bytes(frame.get_data())", self.bridge_source)
        self.assertIn("array = np.asanyarray(frame.get_data())", self.bridge_source)
        self.assertIn("int(array.nbytes) != expected_size", self.bridge_source)
        self.assertIn("array.tobytes(order=\"C\")", self.bridge_source)
        self.assertGreaterEqual(
            self.bridge_source.count(
                "validate_bridge_peak_rss(_peak_rss_mib())"
            ),
            2,
        )
        self.assertIn(
            "D435i read-only health: published_frame_sets={}",
            self.bridge_source,
        )
        self.assertIn(
            "published_frame_sets % (plan.publish_hz * 5) == 0",
            self.bridge_source,
        )

    def test_one_orin_ros_clock_stamp_is_shared_by_the_frame_set(self):
        self.assertIn("rclpy.init(args=[])", self.bridge_source)
        self.assertEqual(
            self.bridge_source.count("stamp = node.get_clock().now().to_msg()"),
            1,
        )
        self.assertIn(
            'Image, rgb_frame, "bgr8", stamp, plan.frame_id',
            self.bridge_source,
        )
        self.assertIn(
            'Image, depth_frame, "16UC1", stamp, plan.frame_id',
            self.bridge_source,
        )
        self.assertIn(
            "CameraInfo, intrinsics, stamp, plan.frame_id",
            self.bridge_source,
        )

    def test_shutdown_attempts_every_cleanup_layer(self):
        finally_block = self.bridge_source.split("    finally:", 1)[1]
        self.assertIn("pipeline.stop()", finally_block)
        self.assertIn("node.destroy_node()", finally_block)
        self.assertIn("rclpy.shutdown()", finally_block)
        self.assertIn("_restore_stop_handlers(previous_handlers)", finally_block)
        for signal_name in ("signal.SIGINT", "signal.SIGTERM", "signal.SIGHUP"):
            self.assertIn(signal_name, self.bridge_source)

    def test_launcher_streams_source_over_ssh_without_remote_copy(self):
        self.assertIn("ssh_options=(", self.launcher_source)
        self.assertIn("  -T", self.launcher_source)
        self.assertIn("\nssh \\\n", self.launcher_source)
        self.assertIn(
            "exec python3 - --remote-producer --readonly-streaming-ack",
            self.launcher_source,
        )
        self.assertIn("--workstation-publisher", self.launcher_source)
        self.assertIn('readonly STREAM_WIDTH=320', self.launcher_source)
        self.assertIn('readonly STREAM_HEIGHT=240', self.launcher_source)
        self.assertIn('readonly STREAM_PUBLISH_HZ=5', self.launcher_source)
        self.assertGreaterEqual(self.launcher_source.count('--width'), 2)
        self.assertGreaterEqual(self.launcher_source.count('--height'), 2)
        self.assertGreaterEqual(self.launcher_source.count('--publish-hz'), 2)
        self.assertIn('< "${BRIDGE_SOURCE}"', self.launcher_source)
        self.assertIn("PYTHONDONTWRITEBYTECODE=1", self.launcher_source)
        remote_command = self.launcher_source.split(
            'readonly REMOTE_COMMAND="', 1
        )[1].split('"', 1)[0]
        self.assertNotIn("rclpy", remote_command)
        self.assertNotIn("RMW_IMPLEMENTATION", remote_command)
        self.assertNotIn("CYCLONEDDS_URI", remote_command)
        self.assertNotIn("scp ", self.launcher_source)
        self.assertNotIn("rsync ", self.launcher_source)
        self.assertIn(
            self.bridge_source.split('READONLY_STREAMING_ACK = "', 1)[1].split('"', 1)[0],
            self.launcher_source,
        )

    def test_optional_control_socket_is_confined_to_ignored_run_directory(self):
        self.assertIn(
            'readonly SSH_CONTROL_PATH="${D435I_SSH_CONTROL_PATH:-}"',
            self.launcher_source,
        )
        self.assertIn(
            '"${REPOSITORY_ROOT}"/.run/*',
            self.launcher_source,
        )
        self.assertIn('ssh_options+=(-S "${SSH_CONTROL_PATH}")', self.launcher_source)


if __name__ == "__main__":
    unittest.main()
