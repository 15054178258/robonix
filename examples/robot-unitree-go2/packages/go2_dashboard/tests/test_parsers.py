from __future__ import annotations

import math
import struct
import unittest

from go2_dashboard.parsers import (
    cloud_to_points,
    goal_status_label,
    image_to_rgb_bytes,
    occupancy_to_luma,
    quaternion_to_yaw,
    scan_to_points,
)


class ParserTests(unittest.TestCase):
    def test_scan_filters_invalid_and_out_of_range_values(self) -> None:
        points = scan_to_points(
            [1.0, float("nan"), 0.05, 3.0],
            angle_min=0.0,
            angle_increment=math.pi / 2,
            range_min=0.1,
            range_max=2.0,
        )
        self.assertEqual(points, [[1.0, 0.0, 0.0]])

    def test_scan_preview_is_bounded(self) -> None:
        points = scan_to_points(
            [1.0] * 100,
            angle_min=-1.0,
            angle_increment=0.02,
            range_min=0.1,
            range_max=10.0,
            max_points=9,
        )
        self.assertLessEqual(len(points), 9)

    def test_cloud_decodes_little_endian_xyz_with_padding(self) -> None:
        fields = [
            {"name": "x", "offset": 0, "datatype": 7},
            {"name": "y", "offset": 4, "datatype": 7},
            {"name": "z", "offset": 8, "datatype": 7},
        ]
        data = struct.pack("<fffIfffI", 1.0, 2.0, 3.0, 99, -1.0, -2.0, -3.0, 7)
        points = cloud_to_points(data, fields, 16, 2, 1, False)
        self.assertEqual(points, [[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0]])

    def test_cloud_decodes_big_endian_and_skips_non_finite(self) -> None:
        fields = [
            {"name": "x", "offset": 0, "datatype": 8},
            {"name": "y", "offset": 8, "datatype": 8},
            {"name": "z", "offset": 16, "datatype": 8},
        ]
        data = struct.pack(">dddddd", 0.5, 1.5, 2.5, math.inf, 0.0, 0.0)
        points = cloud_to_points(data, fields, 24, 2, 1, True)
        self.assertEqual(points, [[0.5, 1.5, 2.5]])

    def test_cloud_honors_organized_row_padding(self) -> None:
        fields = [
            {"name": "x", "offset": 0, "datatype": 7},
            {"name": "y", "offset": 4, "datatype": 7},
            {"name": "z", "offset": 8, "datatype": 7},
        ]
        row_one = struct.pack("<fff", 1.0, 2.0, 3.0) + b"padding!"
        row_two = struct.pack("<fff", 4.0, 5.0, 6.0) + b"padding!"
        points = cloud_to_points(
            row_one + row_two,
            fields,
            point_step=12,
            width=1,
            height=2,
            is_bigendian=False,
            row_step=20,
        )
        self.assertEqual(points, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])

    def test_cloud_rejects_missing_coordinate_field(self) -> None:
        with self.assertRaisesRegex(ValueError, "x, y, and z"):
            cloud_to_points(
                b"\x00" * 12,
                [
                    {"name": "x", "offset": 0, "datatype": 7},
                    {"name": "y", "offset": 4, "datatype": 7},
                ],
                12,
                1,
                1,
                False,
            )

    def test_occupancy_grid_is_vertically_flipped_for_canvas(self) -> None:
        luma = occupancy_to_luma([0, 100, -1, 50], width=2, height=2)
        self.assertEqual(list(luma), [150, 135, 250, 20])

    def test_rgb_image_removes_row_padding(self) -> None:
        data = bytes([1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 88, 88])
        rgb = image_to_rgb_bytes(data, width=2, height=2, step=8, encoding="rgb8")
        self.assertEqual(rgb, bytes(range(1, 13)))

    def test_bgr_and_mono_images_are_normalized(self) -> None:
        self.assertEqual(
            image_to_rgb_bytes(bytes([1, 2, 3]), 1, 1, 3, "bgr8"),
            bytes([3, 2, 1]),
        )
        self.assertEqual(
            image_to_rgb_bytes(bytes([17]), 1, 1, 1, "mono8"),
            bytes([17, 17, 17]),
        )

    def test_four_channel_images_drop_alpha_and_honor_row_padding(self) -> None:
        rgba = bytes(
            [1, 2, 3, 4, 5, 6, 7, 8, 99, 99, 9, 10, 11, 12, 13, 14, 15, 16, 88, 88]
        )
        self.assertEqual(
            image_to_rgb_bytes(rgba, 2, 2, 10, "rgba8"),
            bytes([1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15]),
        )

        bgra = bytes([3, 2, 1, 4, 7, 6, 5, 8])
        self.assertEqual(
            image_to_rgb_bytes(bgra, 2, 1, 8, "bgra8"),
            bytes([1, 2, 3, 5, 6, 7]),
        )

    def test_bgr_and_mono_multi_pixel_rows_are_vectorized_equivalently(self) -> None:
        bgr = bytes([3, 2, 1, 6, 5, 4, 99, 9, 8, 7, 12, 11, 10, 88])
        self.assertEqual(
            image_to_rgb_bytes(bgr, 2, 2, 7, "bgr8"),
            bytes([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]),
        )
        mono = bytes([1, 2, 99, 3, 4, 88])
        self.assertEqual(
            image_to_rgb_bytes(mono, 2, 2, 3, "mono8"),
            bytes([1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4]),
        )

    def test_quaternion_to_yaw_normalizes_input(self) -> None:
        yaw = quaternion_to_yaw(0.0, 0.0, math.sqrt(2), math.sqrt(2))
        self.assertAlmostEqual(yaw, math.pi / 2)
        with self.assertRaisesRegex(ValueError, "norm"):
            quaternion_to_yaw(0.0, 0.0, 0.0, 0.0)

    def test_goal_status_labels(self) -> None:
        self.assertEqual(goal_status_label(2), "executing")
        self.assertEqual(goal_status_label(4), "succeeded")
        self.assertEqual(goal_status_label(999), "unknown")


if __name__ == "__main__":
    unittest.main()
