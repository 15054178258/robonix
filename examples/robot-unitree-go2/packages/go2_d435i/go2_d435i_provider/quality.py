"""Pure RGB-D quality checks used by the subscription-only runtime observer."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Mapping


NANOSECONDS_PER_SECOND = 1_000_000_000
MAX_RECORDED_ERRORS = 64
MAX_RECORDED_STAMPS = 4096
MIN_DEPTH_VALID_RATIO = 0.80
MIN_SYNC_RATIO = 0.80


@dataclass(frozen=True)
class QualityResult:
    ok: bool
    detail: str
    evidence: dict[str, Any]


@dataclass
class StreamState:
    samples: int = 0
    first_receipt_monotonic_ns: int | None = None
    last_receipt_monotonic_ns: int | None = None
    last_stamp_ns: int | None = None
    stamps_ns: list[int] = field(default_factory=list)
    frames: set[str] = field(default_factory=set)
    encodings: set[str] = field(default_factory=set)
    geometries: set[tuple[int, int]] = field(default_factory=set)
    malformed_samples: int = 0


def _bounded_text(value: Any, maximum: int = 160) -> str:
    result = str(value)
    if len(result) > maximum:
        return result[:maximum] + "..."
    return result


def _header_stamp_ns(message: Any) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if isinstance(sec, bool) or not isinstance(sec, int) or sec < 0:
        raise ValueError("header.stamp.sec must be a non-negative integer")
    if (
        isinstance(nanosec, bool)
        or not isinstance(nanosec, int)
        or not 0 <= nanosec < NANOSECONDS_PER_SECOND
    ):
        raise ValueError("header.stamp.nanosec must be in 0..999999999")
    result = sec * NANOSECONDS_PER_SECOND + nanosec
    if result <= 0:
        raise ValueError("header timestamp must be non-zero")
    return result


def _header_frame(message: Any) -> str:
    frame = getattr(getattr(message, "header", None), "frame_id", None)
    if not isinstance(frame, str) or not frame or frame.startswith("/"):
        raise ValueError("header.frame_id must be a non-empty relative frame")
    return frame


def _positive_integer(value: Any, name: str, maximum: int = 16384) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value <= maximum:
        raise ValueError(f"{name} must be an integer in 1..{maximum}")
    return value


def _image_layout(message: Any, bytes_per_pixel: int) -> tuple[int, int]:
    width = _positive_integer(getattr(message, "width", None), "image width")
    height = _positive_integer(getattr(message, "height", None), "image height")
    step = _positive_integer(
        getattr(message, "step", None),
        "image step",
        maximum=256 * 1024,
    )
    minimum_step = width * bytes_per_pixel
    if step < minimum_step:
        raise ValueError(f"image step {step} is smaller than {minimum_step}")
    data = getattr(message, "data", None)
    try:
        data_length = len(data)
    except (TypeError, AttributeError) as error:
        raise ValueError("image data must be a bounded byte sequence") from error
    expected = step * height
    if data_length != expected:
        raise ValueError(
            f"image data length {data_length} does not equal step*height {expected}"
        )
    return width, height


def _finite_sequence(value: Any, name: str, exact_length: int | None = None) -> list[float]:
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite numeric sequence") from error
    if exact_length is not None and len(result) != exact_length:
        raise ValueError(f"{name} must contain exactly {exact_length} values")
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains a non-finite value")
    return result


class QualityTracker:
    """Accumulate bounded evidence without retaining RGB-D payload bytes."""

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.config = dict(config)
        self.streams = {
            "rgb": StreamState(),
            "depth": StreamState(),
            "camera_info": StreamState(),
        }
        self.errors: list[str] = []
        self.depth_nonzero_samples = 0
        self.latest_intrinsics: dict[str, Any] | None = None

    @property
    def all_streams_seen(self) -> bool:
        return all(state.samples > 0 for state in self.streams.values())

    def _error(self, role: str, detail: str) -> None:
        state = self.streams[role]
        state.malformed_samples += 1
        message = f"{role}: {_bounded_text(detail)}"
        if len(self.errors) < MAX_RECORDED_ERRORS and message not in self.errors:
            self.errors.append(message)

    def _observe_header(
        self,
        role: str,
        message: Any,
        receipt_realtime_ns: int,
        receipt_monotonic_ns: int,
        expected_frame: str,
        *,
        require_fresh: bool,
    ) -> int | None:
        state = self.streams[role]
        state.samples += 1
        if state.first_receipt_monotonic_ns is None:
            state.first_receipt_monotonic_ns = receipt_monotonic_ns
        state.last_receipt_monotonic_ns = receipt_monotonic_ns
        try:
            stamp_ns = _header_stamp_ns(message)
            frame = _header_frame(message)
            state.frames.add(frame)
            if frame != expected_frame:
                raise ValueError(
                    f"unexpected frame {frame!r}; expected {expected_frame!r}"
                )
            if state.last_stamp_ns is not None and stamp_ns <= state.last_stamp_ns:
                raise ValueError("source timestamp is not strictly increasing")
            state.last_stamp_ns = stamp_ns
            if len(state.stamps_ns) < MAX_RECORDED_STAMPS:
                state.stamps_ns.append(stamp_ns)
            if require_fresh:
                age_ns = receipt_realtime_ns - stamp_ns
                maximum_age_ns = int(
                    float(self.config["max_stamp_age_s"]) * NANOSECONDS_PER_SECOND
                )
                maximum_future_ns = int(
                    float(self.config["max_future_skew_s"]) * NANOSECONDS_PER_SECOND
                )
                if age_ns > maximum_age_ns:
                    raise ValueError(
                        f"source timestamp is {age_ns / NANOSECONDS_PER_SECOND:.6f}s old"
                    )
                if age_ns < -maximum_future_ns:
                    raise ValueError(
                        "source timestamp leads receipt time by "
                        f"{-age_ns / NANOSECONDS_PER_SECOND:.6f}s"
                    )
            return stamp_ns
        except (TypeError, ValueError) as error:
            self._error(role, str(error))
            return None

    def observe(
        self,
        role: str,
        message: Any,
        *,
        receipt_realtime_ns: int,
        receipt_monotonic_ns: int,
    ) -> None:
        if role not in self.streams:
            raise ValueError(f"unknown D435i stream role: {role!r}")
        expected_frame = (
            self.config["rgb_frame"]
            if role in {"rgb", "camera_info"}
            else self.config["depth_frame"]
        )
        self._observe_header(
            role,
            message,
            receipt_realtime_ns,
            receipt_monotonic_ns,
            expected_frame,
            require_fresh=role in {"rgb", "depth"},
        )
        if role == "rgb":
            self._observe_rgb(message)
        elif role == "depth":
            self._observe_depth(message)
        else:
            self._observe_camera_info(message)

    def _observe_rgb(self, message: Any) -> None:
        state = self.streams["rgb"]
        try:
            encoding = str(getattr(message, "encoding", "")).lower()
            state.encodings.add(encoding)
            if encoding not in {"rgb8", "bgr8"}:
                raise ValueError(f"unsupported RGB encoding: {encoding!r}")
            state.geometries.add(_image_layout(message, 3))
        except (TypeError, ValueError) as error:
            self._error("rgb", str(error))

    def _observe_depth(self, message: Any) -> None:
        state = self.streams["depth"]
        try:
            encoding = str(getattr(message, "encoding", "")).lower()
            state.encodings.add(encoding)
            if encoding not in {"16uc1", "mono16"}:
                raise ValueError(f"unsupported metric depth encoding: {encoding!r}")
            state.geometries.add(_image_layout(message, 2))
            data = getattr(message, "data", None)
            if any(data):
                self.depth_nonzero_samples += 1
        except (TypeError, ValueError) as error:
            self._error("depth", str(error))

    def _observe_camera_info(self, message: Any) -> None:
        state = self.streams["camera_info"]
        try:
            width = _positive_integer(
                getattr(message, "width", None), "CameraInfo width"
            )
            height = _positive_integer(
                getattr(message, "height", None), "CameraInfo height"
            )
            state.geometries.add((width, height))
            k = _finite_sequence(getattr(message, "k", None), "CameraInfo.k", 9)
            _finite_sequence(getattr(message, "d", ()), "CameraInfo.d")
            _finite_sequence(getattr(message, "r", None), "CameraInfo.r", 9)
            _finite_sequence(getattr(message, "p", None), "CameraInfo.p", 12)
            if k[0] <= 0.0 or k[4] <= 0.0:
                raise ValueError("CameraInfo is uncalibrated: fx and fy must be positive")
            if not 0.0 <= k[2] <= float(width) or not 0.0 <= k[5] <= float(height):
                raise ValueError("CameraInfo principal point lies outside the image")
            distortion_model = getattr(message, "distortion_model", "")
            if not isinstance(distortion_model, str) or not distortion_model:
                raise ValueError("CameraInfo distortion_model is empty")
            self.latest_intrinsics = {
                "width": width,
                "height": height,
                "fx": k[0],
                "fy": k[4],
                "cx": k[2],
                "cy": k[5],
                "distortion_model": distortion_model,
            }
        except (TypeError, ValueError) as error:
            self._error("camera_info", str(error))

    def _stream_rate(self, role: str) -> float:
        state = self.streams[role]
        if (
            state.samples < 2
            or state.first_receipt_monotonic_ns is None
            or state.last_receipt_monotonic_ns is None
        ):
            return 0.0
        span_ns = state.last_receipt_monotonic_ns - state.first_receipt_monotonic_ns
        if span_ns <= 0:
            return 0.0
        return (state.samples - 1) * NANOSECONDS_PER_SECOND / span_ns

    def finalize(
        self,
        *,
        quality_duration_ns: int,
        finished_monotonic_ns: int,
    ) -> QualityResult:
        problems = list(self.errors)
        required_duration_ns = int(
            float(self.config["quality_window_s"]) * NANOSECONDS_PER_SECOND
        )
        if quality_duration_ns < required_duration_ns:
            problems.append(
                "quality observation window was shorter than the configured duration"
            )

        for role, state in self.streams.items():
            if state.samples == 0:
                problems.append(f"no {role} sample was received")
        for role in ("rgb", "depth"):
            state = self.streams[role]
            rate = self._stream_rate(role)
            if rate < float(self.config["min_rate_hz"]):
                problems.append(
                    f"{role} sustained rate {rate:.3f}Hz is below "
                    f"{float(self.config['min_rate_hz']):.3f}Hz"
                )
            minimum_span_ns = max(
                0,
                required_duration_ns
                - 2
                * int(
                    float(self.config["max_stamp_age_s"])
                    * NANOSECONDS_PER_SECOND
                ),
            )
            if (
                state.first_receipt_monotonic_ns is None
                or state.last_receipt_monotonic_ns is None
                or state.last_receipt_monotonic_ns
                - state.first_receipt_monotonic_ns
                < minimum_span_ns
            ):
                problems.append(f"{role} did not span the sustained quality window")
            if (
                state.last_receipt_monotonic_ns is None
                or finished_monotonic_ns - state.last_receipt_monotonic_ns
                > int(
                    float(self.config["max_stamp_age_s"])
                    * NANOSECONDS_PER_SECOND
                )
            ):
                problems.append(f"{role} was stale at the end of the quality window")

        depth_samples = self.streams["depth"].samples
        depth_valid_ratio = (
            self.depth_nonzero_samples / depth_samples if depth_samples else 0.0
        )
        if depth_valid_ratio < MIN_DEPTH_VALID_RATIO:
            problems.append(
                f"non-zero depth ratio {depth_valid_ratio:.3f} is below "
                f"{MIN_DEPTH_VALID_RATIO:.3f}"
            )

        rgb_geometry = self.streams["rgb"].geometries
        depth_geometry = self.streams["depth"].geometries
        info_geometry = self.streams["camera_info"].geometries
        if len(rgb_geometry) != 1:
            problems.append("RGB geometry was missing or changed during activation")
        if len(depth_geometry) != 1:
            problems.append("depth geometry was missing or changed during activation")
        if len(info_geometry) != 1:
            problems.append("CameraInfo geometry was missing or changed during activation")
        if (
            len(rgb_geometry) == len(depth_geometry) == len(info_geometry) == 1
            and not (
                next(iter(rgb_geometry))
                == next(iter(depth_geometry))
                == next(iter(info_geometry))
            )
        ):
            problems.append("RGB, aligned depth, and CameraInfo geometries differ")

        rgb_stamps = self.streams["rgb"].stamps_ns
        depth_stamps = self.streams["depth"].stamps_ns
        synchronized = 0
        maximum_skew_ns = int(
            float(self.config["max_rgb_depth_skew_s"]) * NANOSECONDS_PER_SECOND
        )
        if rgb_stamps and depth_stamps:
            rgb_sorted = sorted(rgb_stamps)
            rgb_index = 0
            for depth_stamp in sorted(depth_stamps):
                while (
                    rgb_index + 1 < len(rgb_sorted)
                    and abs(rgb_sorted[rgb_index + 1] - depth_stamp)
                    <= abs(rgb_sorted[rgb_index] - depth_stamp)
                ):
                    rgb_index += 1
                if abs(rgb_sorted[rgb_index] - depth_stamp) <= maximum_skew_ns:
                    synchronized += 1
        sync_ratio = synchronized / len(depth_stamps) if depth_stamps else 0.0
        if sync_ratio < MIN_SYNC_RATIO:
            problems.append(
                f"RGB-depth synchronized ratio {sync_ratio:.3f} is below "
                f"{MIN_SYNC_RATIO:.3f}"
            )

        evidence = {
            "ros_publishers_created": False,
            "source_mode": "external",
            "streams": {
                role: {
                    "samples": state.samples,
                    "malformed_samples": state.malformed_samples,
                    "frames": sorted(state.frames),
                    "encodings": sorted(state.encodings),
                    "geometries": [list(item) for item in sorted(state.geometries)],
                    "rate_hz": round(self._stream_rate(role), 6)
                    if role in {"rgb", "depth"}
                    else None,
                }
                for role, state in self.streams.items()
            },
            "depth_nonzero_ratio": round(depth_valid_ratio, 6),
            "rgb_depth_sync_ratio": round(sync_ratio, 6),
            "intrinsics": dict(self.latest_intrinsics)
            if self.latest_intrinsics is not None
            else None,
            "problems": problems,
        }
        if problems:
            return QualityResult(False, "; ".join(problems[:8]), evidence)
        return QualityResult(
            True,
            "external D435i RGB, aligned depth, and intrinsics quality gate passed",
            evidence,
        )
