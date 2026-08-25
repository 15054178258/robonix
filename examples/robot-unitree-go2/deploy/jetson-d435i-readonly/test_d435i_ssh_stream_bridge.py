import importlib.util
import io
import struct
from types import SimpleNamespace
import unittest
import zlib
from pathlib import Path


ROOT = Path(__file__).resolve().parent
BRIDGE_PATH = ROOT / "d435i_ssh_stream_bridge.py"


def load_bridge():
    spec = importlib.util.spec_from_file_location(
        "d435i_ssh_stream_bridge",
        BRIDGE_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FragmentedReader:
    def __init__(self, payload, fragment_size=1):
        self._stream = io.BytesIO(payload)
        self._fragment_size = fragment_size

    def read(self, size):
        return self._stream.read(min(size, self._fragment_size))


class D435iSshProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_bridge()
        cls.plan = cls.bridge.make_stream_plan()

    def make_header(self, sequence=0, monotonic_ns=10):
        return {
            "sequence": sequence,
            "capture_unix_ns": 1_784_783_000_000_000_000 + sequence,
            "capture_monotonic_ns": monotonic_ns,
            "width": 640,
            "height": 480,
            "frame_id": "d435i_color_optical_frame",
            "color_codec": "jpeg-bgr8",
            "depth_codec": "zlib-u16le",
            "depth_scale": 0.001,
            "fx": 600.0,
            "fy": 601.0,
            "ppx": 319.5,
            "ppy": 239.5,
            "coefficients": [0.1, 0.2, 0.0, 0.0, 0.0],
        }

    def test_default_plan_publishes_six_hz_from_thirty_hz_capture(self):
        self.assertEqual((self.plan.width, self.plan.height), (640, 480))
        self.assertEqual(
            (self.bridge.PREVIEW_WIDTH, self.bridge.PREVIEW_HEIGHT),
            (320, 240),
        )
        self.assertEqual(self.plan.hardware_fps, 30)
        self.assertEqual(self.plan.publish_hz, 6)
        self.assertEqual(self.plan.decimation, 5)

    def test_remote_header_scales_capture_intrinsics_for_transport(self):
        plan = self.bridge.make_stream_plan(
            width=320,
            height=240,
            publish_hz=5,
        )
        intrinsics = SimpleNamespace(
            fx=600.0,
            fy=602.0,
            ppx=320.0,
            ppy=240.0,
            coeffs=[0.1, 0.2, 0.0, 0.0, 0.0],
        )
        header = self.bridge._remote_header(0, 1, 2, plan, intrinsics)
        self.assertEqual((header["width"], header["height"]), (320, 240))
        self.assertEqual(
            (header["fx"], header["fy"], header["ppx"], header["ppy"]),
            (300.0, 301.0, 160.0, 120.0),
        )
        self.bridge.validate_header(header, plan)

    def make_packet(self):
        header = self.make_header()
        jpeg = b"\xff\xd8bounded-jpeg-test\xff\xd9"
        depth = zlib.compress(b"\x00\x00" * (640 * 480), level=1)
        packet = self.bridge.encode_packet(header, jpeg, depth)
        return header, jpeg, depth, packet

    def test_packet_round_trip_and_single_byte_fragmentation(self):
        header, jpeg, depth, packet = self.make_packet()
        decoded = self.bridge.read_packet(FragmentedReader(packet))
        self.assertEqual(decoded, (header, jpeg, depth))
        self.bridge.validate_header(decoded[0], self.plan)

    def test_clean_eof_and_mid_packet_eof_are_distinct(self):
        self.assertIsNone(self.bridge.read_packet(io.BytesIO(b"")))
        _header, _jpeg, _depth, packet = self.make_packet()
        for cut in (1, self.bridge.PACKET_PREFIX.size, len(packet) - 1):
            with self.subTest(cut=cut), self.assertRaises(
                self.bridge.StreamProtocolError
            ):
                self.bridge.read_packet(io.BytesIO(packet[:cut]))

    def test_crc_failure_terminates_instead_of_resynchronizing(self):
        _header, _jpeg, _depth, packet = self.make_packet()
        corrupt = bytearray(packet)
        corrupt[-1] ^= 0x01
        with self.assertRaisesRegex(
            self.bridge.StreamProtocolError,
            "CRC mismatch",
        ):
            self.bridge.read_packet(io.BytesIO(bytes(corrupt)))

    def test_prefix_length_limits_are_checked_before_payload_reads(self):
        forged = self.bridge.PACKET_PREFIX.pack(
            self.bridge.PROTOCOL_MAGIC,
            self.bridge.PROTOCOL_VERSION,
            self.bridge.MAX_HEADER_BYTES + 1,
            1,
            1,
            0,
            0,
            0,
        )
        with self.assertRaisesRegex(
            self.bridge.StreamProtocolError,
            "header length",
        ):
            self.bridge.read_packet(io.BytesIO(forged))

    def test_encode_rejects_each_oversized_component(self):
        header = self.make_header()
        good_jpeg = b"j"
        good_depth = b"d"
        cases = (
            ({"padding": "x" * self.bridge.MAX_HEADER_BYTES}, good_jpeg, good_depth),
            (header, b"j" * (self.bridge.MAX_JPEG_BYTES + 1), good_depth),
            (
                header,
                good_jpeg,
                b"d" * (self.bridge.MAX_DEPTH_ZLIB_BYTES + 1),
            ),
        )
        for values in cases:
            with self.subTest(lengths=tuple(len(value) if isinstance(value, bytes) else -1 for value in values)), self.assertRaises(
                self.bridge.StreamProtocolError
            ):
                self.bridge.encode_packet(*values)

    def test_depth_decompression_is_exact_and_rejects_trailing_member(self):
        expected = b"\x00\x01" * (640 * 480)
        payload = zlib.compress(expected, level=1)
        self.assertEqual(
            self.bridge._decompress_depth(payload, len(expected)),
            expected,
        )
        with self.assertRaises(self.bridge.StreamProtocolError):
            self.bridge._decompress_depth(payload + zlib.compress(b"x"), len(expected))
        with self.assertRaises(self.bridge.StreamProtocolError):
            self.bridge._decompress_depth(
                zlib.compress(expected + b"\x00\x00", level=1),
                len(expected),
            )

    def test_header_contract_is_exact_and_finite(self):
        self.bridge.validate_header(self.make_header(), self.plan)
        cases = []
        missing = self.make_header()
        del missing["fx"]
        cases.append(missing)
        extra = self.make_header()
        extra["unknown"] = 1
        cases.append(extra)
        wrong_frame = self.make_header()
        wrong_frame["frame_id"] = "base_link"
        cases.append(wrong_frame)
        non_finite = self.make_header()
        non_finite["fx"] = float("inf")
        cases.append(non_finite)
        for header in cases:
            with self.subTest(header=header), self.assertRaises(
                (self.bridge.StreamProtocolError, self.bridge.ConfigurationError)
            ):
                self.bridge.validate_header(header, self.plan)

    def test_protocol_prefix_is_network_order_and_fixed_size(self):
        self.assertEqual(self.bridge.PACKET_PREFIX.format, "!8s7I")
        self.assertEqual(
            self.bridge.PACKET_PREFIX.size,
            struct.calcsize("!8s7I"),
        )


class D435iSshStaticSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = BRIDGE_PATH.read_text(encoding="utf-8")

    def test_remote_role_has_no_ros_import_or_ros_publisher(self):
        remote_section = self.source.split(
            "def run_remote_producer", 1
        )[1].split("def _load_workstation_modules", 1)[0]
        self.assertNotIn("rclpy", remote_section)
        self.assertNotIn("create_publisher", remote_section)
        self.assertNotIn("sensor_msgs", remote_section)

    def test_stdout_is_reserved_for_protocol_and_logs_are_stderr(self):
        self.assertIn("protocol_fd = os.dup(sys.stdout.fileno())", self.source)
        self.assertIn(
            "os.dup2(sys.stderr.fileno(), sys.stdout.fileno())",
            self.source,
        )
        remote_section = self.source.split(
            "def run_remote_producer", 1
        )[1].split("def _load_workstation_modules", 1)[0]
        self.assertGreaterEqual(remote_section.count("file=sys.stderr"), 2)

    def test_workstation_owns_exactly_three_sensor_publishers(self):
        workstation_section = self.source.split(
            "def run_workstation_publisher", 1
        )[1].split("def build_argument_parser", 1)[0]
        self.assertEqual(workstation_section.count("node.create_publisher("), 3)
        self.assertIn('RGB_TOPIC = "color/image_raw"', self.source)
        self.assertIn(
            'ALIGNED_DEPTH_TOPIC = "aligned_depth_to_color/image_raw"',
            self.source,
        )
        self.assertIn(
            'CAMERA_INFO_TOPIC = "color/camera_info"',
            self.source,
        )

    def test_qos_is_explicit_best_effort_volatile_keep_last_one(self):
        for text in (
            "history=HistoryPolicy.KEEP_LAST",
            "depth=1",
            "reliability=ReliabilityPolicy.BEST_EFFORT",
            "durability=DurabilityPolicy.VOLATILE",
        ):
            self.assertIn(text, self.source)

    def test_shared_workstation_receipt_stamp_and_tight_layout(self):
        self.assertEqual(
            self.source.count("stamp = _message_stamp_from_now(node)"),
            1,
        )
        self.assertIn('message.step = message.width * bytes_per_pixel', self.source)
        self.assertIn('message.is_bigendian = 0', self.source)
        self.assertIn("(PREVIEW_WIDTH, PREVIEW_HEIGHT)", self.source)
        self.assertIn("interpolation=cv2.INTER_AREA", self.source)
        self.assertIn("interpolation=cv2.INTER_NEAREST", self.source)
        self.assertIn(
            "scale_x = float(output_width) / float(header[\"width\"])",
            self.source,
        )

    def test_camera_pipeline_cleanup_and_forbidden_surfaces(self):
        self.assertIn("pipeline.stop()", self.source)
        self.assertIn("node.destroy_node()", self.source)
        self.assertIn("rclpy.shutdown()", self.source)
        for forbidden in (
            "rs.stream.accel",
            "rs.stream.gyro",
            "TransformBroadcaster",
            "PointCloud2",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_remote_capture_profile_is_fixed_and_downsamples_before_transport(self):
        remote_section = self.source.split(
            "def run_remote_producer", 1
        )[1].split("def _load_workstation_modules", 1)[0]
        self.assertIn(
            "rs.stream.color,\n            CAPTURE_WIDTH,\n            CAPTURE_HEIGHT,",
            remote_section,
        )
        self.assertIn(
            "rs.stream.depth,\n            CAPTURE_WIDTH,\n            CAPTURE_HEIGHT,",
            remote_section,
        )
        self.assertIn("(plan.width, plan.height)", remote_section)
        self.assertIn("interpolation=cv2.INTER_AREA", remote_section)
        self.assertIn("interpolation=cv2.INTER_NEAREST", remote_section)

    def test_workstation_disables_rclpy_signal_handlers_and_checks_stop(self):
        self.assertIn(
            "signal_handler_options=SignalHandlerOptions.NO",
            self.source,
        )
        self.assertIn(
            "if stop_event.is_set() or not rclpy.ok():",
            self.source,
        )


if __name__ == "__main__":
    unittest.main()
