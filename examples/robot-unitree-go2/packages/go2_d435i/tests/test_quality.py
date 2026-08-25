from __future__ import annotations

from types import SimpleNamespace
import unittest

from go2_d435i_provider.quality import NANOSECONDS_PER_SECOND, QualityTracker


FRAME = "d435i_color_optical_frame"
REALTIME_BASE_NS = 1_700_000_000 * NANOSECONDS_PER_SECOND
MONOTONIC_BASE_NS = 10 * NANOSECONDS_PER_SECOND


def header(stamp_ns: int, frame: str = FRAME) -> SimpleNamespace:
    return SimpleNamespace(
        stamp=SimpleNamespace(
            sec=stamp_ns // NANOSECONDS_PER_SECOND,
            nanosec=stamp_ns % NANOSECONDS_PER_SECOND,
        ),
        frame_id=frame,
    )


def rgb(stamp_ns: int, *, width: int = 4, height: int = 3) -> SimpleNamespace:
    step = width * 3
    return SimpleNamespace(
        header=header(stamp_ns),
        width=width,
        height=height,
        encoding="rgb8",
        is_bigendian=0,
        step=step,
        data=bytes([17]) * step * height,
    )


def depth(
    stamp_ns: int,
    *,
    width: int = 4,
    height: int = 3,
    nonzero: bool = True,
) -> SimpleNamespace:
    step = width * 2
    payload = bytes([1, 0]) * width * height if nonzero else bytes(step * height)
    return SimpleNamespace(
        header=header(stamp_ns),
        width=width,
        height=height,
        encoding="16UC1",
        is_bigendian=0,
        step=step,
        data=payload,
    )


def camera_info(
    stamp_ns: int,
    *,
    width: int = 4,
    height: int = 3,
    calibrated: bool = True,
) -> SimpleNamespace:
    fx = 4.0 if calibrated else 0.0
    fy = 4.0 if calibrated else 0.0
    return SimpleNamespace(
        header=header(stamp_ns),
        width=width,
        height=height,
        distortion_model="plumb_bob",
        d=[0.0] * 5,
        k=[fx, 0.0, 2.0, 0.0, fy, 1.5, 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[fx, 0.0, 2.0, 0.0, 0.0, fy, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0],
    )


def config() -> dict:
    return {
        "source_mode": "external",
        "rgb_frame": FRAME,
        "depth_frame": FRAME,
        "quality_window_s": 5.0,
        "min_rate_hz": 5.0,
        "max_stamp_age_s": 0.5,
        "max_future_skew_s": 0.05,
        "max_rgb_depth_skew_s": 0.05,
    }


def good_tracker() -> tuple[QualityTracker, int]:
    tracker = QualityTracker(config())
    for index in range(26):
        offset_ns = index * 200_000_000
        receipt_real_ns = REALTIME_BASE_NS + offset_ns
        receipt_mono_ns = MONOTONIC_BASE_NS + offset_ns
        tracker.observe(
            "rgb",
            rgb(receipt_real_ns),
            receipt_realtime_ns=receipt_real_ns,
            receipt_monotonic_ns=receipt_mono_ns,
        )
        tracker.observe(
            "depth",
            depth(receipt_real_ns),
            receipt_realtime_ns=receipt_real_ns,
            receipt_monotonic_ns=receipt_mono_ns,
        )
        if index == 0:
            tracker.observe(
                "camera_info",
                camera_info(receipt_real_ns),
                receipt_realtime_ns=receipt_real_ns,
                receipt_monotonic_ns=receipt_mono_ns,
            )
    return tracker, MONOTONIC_BASE_NS + 5 * NANOSECONDS_PER_SECOND


class QualityTrackerTests(unittest.TestCase):
    def test_valid_synchronized_rgbd_and_intrinsics_pass(self) -> None:
        tracker, finished_ns = good_tracker()
        result = tracker.finalize(
            quality_duration_ns=5 * NANOSECONDS_PER_SECOND,
            finished_monotonic_ns=finished_ns,
        )
        self.assertTrue(result.ok, result.detail)
        self.assertFalse(result.evidence["ros_publishers_created"])
        self.assertEqual(result.evidence["depth_nonzero_ratio"], 1.0)
        self.assertEqual(result.evidence["rgb_depth_sync_ratio"], 1.0)
        self.assertEqual(result.evidence["intrinsics"]["fx"], 4.0)

    def test_uncalibrated_camera_info_fails(self) -> None:
        tracker, finished_ns = good_tracker()
        tracker = QualityTracker(config())
        for index in range(26):
            offset_ns = index * 200_000_000
            realtime_ns = REALTIME_BASE_NS + offset_ns
            monotonic_ns = MONOTONIC_BASE_NS + offset_ns
            tracker.observe(
                "rgb",
                rgb(realtime_ns),
                receipt_realtime_ns=realtime_ns,
                receipt_monotonic_ns=monotonic_ns,
            )
            tracker.observe(
                "depth",
                depth(realtime_ns),
                receipt_realtime_ns=realtime_ns,
                receipt_monotonic_ns=monotonic_ns,
            )
        tracker.observe(
            "camera_info",
            camera_info(REALTIME_BASE_NS, calibrated=False),
            receipt_realtime_ns=REALTIME_BASE_NS,
            receipt_monotonic_ns=MONOTONIC_BASE_NS,
        )
        result = tracker.finalize(
            quality_duration_ns=5 * NANOSECONDS_PER_SECOND,
            finished_monotonic_ns=finished_ns,
        )
        self.assertFalse(result.ok)
        self.assertIn("uncalibrated", result.detail)

    def test_persistent_zero_depth_fails(self) -> None:
        tracker = QualityTracker(config())
        for index in range(26):
            offset_ns = index * 200_000_000
            realtime_ns = REALTIME_BASE_NS + offset_ns
            monotonic_ns = MONOTONIC_BASE_NS + offset_ns
            tracker.observe(
                "rgb",
                rgb(realtime_ns),
                receipt_realtime_ns=realtime_ns,
                receipt_monotonic_ns=monotonic_ns,
            )
            tracker.observe(
                "depth",
                depth(realtime_ns, nonzero=False),
                receipt_realtime_ns=realtime_ns,
                receipt_monotonic_ns=monotonic_ns,
            )
            if index == 0:
                tracker.observe(
                    "camera_info",
                    camera_info(realtime_ns),
                    receipt_realtime_ns=realtime_ns,
                    receipt_monotonic_ns=monotonic_ns,
                )
        result = tracker.finalize(
            quality_duration_ns=5 * NANOSECONDS_PER_SECOND,
            finished_monotonic_ns=MONOTONIC_BASE_NS
            + 5 * NANOSECONDS_PER_SECOND,
        )
        self.assertFalse(result.ok)
        self.assertIn("non-zero depth ratio", result.detail)

    def test_mismatched_geometry_and_frame_fail(self) -> None:
        tracker, finished_ns = good_tracker()
        realtime_ns = REALTIME_BASE_NS + 5_200_000_000
        malformed = depth(realtime_ns, width=2, height=2)
        malformed.header.frame_id = "d435i_depth_optical_frame"
        tracker.observe(
            "depth",
            malformed,
            receipt_realtime_ns=realtime_ns,
            receipt_monotonic_ns=finished_ns + 200_000_000,
        )
        result = tracker.finalize(
            quality_duration_ns=5_200_000_000,
            finished_monotonic_ns=finished_ns + 200_000_000,
        )
        self.assertFalse(result.ok)
        self.assertIn("unexpected frame", result.detail)
        self.assertTrue(
            any(
                problem.startswith("depth geometry was missing or changed")
                for problem in result.evidence["problems"]
            )
        )

    def test_stale_and_non_monotonic_source_stamps_fail(self) -> None:
        tracker, finished_ns = good_tracker()
        receipt_real_ns = REALTIME_BASE_NS + 5_200_000_000
        repeated_stamp_ns = REALTIME_BASE_NS + 5_000_000_000
        tracker.observe(
            "rgb",
            rgb(repeated_stamp_ns),
            receipt_realtime_ns=receipt_real_ns + NANOSECONDS_PER_SECOND,
            receipt_monotonic_ns=finished_ns + 200_000_000,
        )
        result = tracker.finalize(
            quality_duration_ns=5_200_000_000,
            finished_monotonic_ns=finished_ns + 200_000_000,
        )
        self.assertFalse(result.ok)
        self.assertTrue(
            any(
                "not strictly increasing" in problem or "old" in problem
                for problem in result.evidence["problems"]
            )
        )

    def test_rgb_depth_timestamp_desynchronization_fails(self) -> None:
        tracker = QualityTracker(config())
        for index in range(26):
            offset_ns = index * 200_000_000
            realtime_ns = REALTIME_BASE_NS + offset_ns
            monotonic_ns = MONOTONIC_BASE_NS + offset_ns
            tracker.observe(
                "rgb",
                rgb(realtime_ns),
                receipt_realtime_ns=realtime_ns,
                receipt_monotonic_ns=monotonic_ns,
            )
            tracker.observe(
                "depth",
                depth(realtime_ns - 100_000_000),
                receipt_realtime_ns=realtime_ns,
                receipt_monotonic_ns=monotonic_ns,
            )
            if index == 0:
                tracker.observe(
                    "camera_info",
                    camera_info(realtime_ns),
                    receipt_realtime_ns=realtime_ns,
                    receipt_monotonic_ns=monotonic_ns,
                )
        result = tracker.finalize(
            quality_duration_ns=5 * NANOSECONDS_PER_SECOND,
            finished_monotonic_ns=MONOTONIC_BASE_NS
            + 5 * NANOSECONDS_PER_SECOND,
        )
        self.assertFalse(result.ok)
        self.assertIn("synchronized ratio", result.detail)


if __name__ == "__main__":
    unittest.main()
