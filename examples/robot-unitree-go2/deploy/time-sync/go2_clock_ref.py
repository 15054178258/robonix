#!/usr/bin/env python3
"""Subscription-only Go2 source-clock bridge for chrony's SOCK refclock.

The default mode is a bounded, observe-only run.  Feeding chrony requires all
of the following independent gates:

* ``--mode feed-chrony``;
* a root-owned enable file with an exact acknowledgement token;
* root-owned, same-session PCAP correlation approval for the current writer
  GID, which is rechecked against the ROS graph while running.

This process never adjusts a clock itself.  In feed mode it sends a source
sample to chronyd's Unix datagram socket; chronyd remains the only clock servo.
It creates ROS subscriptions only and has no Unitree SDK client.
"""

from __future__ import annotations

import argparse
from collections import deque
import ctypes
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import stat
import sys
import time
from typing import Any, Callable


THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))

from go2_time_core import StreamTracker, read_clock_pair  # noqa: E402


ENABLE_TOKEN = "ENABLE_GO2_CHRONY_REFCLOCK_V1\n"
SOCK_MAGIC = 0x534F434B
EXPECTED_CORRELATION_METHOD = (
    "pcap_rtps_guid_prefix_and_data_writer_entity_correlation"
)
EXPECTED_CORRELATION_CONCLUSION = "single_source_proven_by_rtps_data_writer"
APPROVAL_SCHEMA_VERSION = 3
MINIMUM_STABILITY_DURATION_NS = 7_200_000_000_000
MINIMUM_COLD_BOOT_TRIALS = 3
POST_BOOTSTRAP_POLICY = {
    "reference_topic": "/sportmodestate",
    "minimum_continuous_duration_ns": 300_000_000_000,
    "maximum_absolute_offset_ns_exclusive": 50_000_000,
    "maximum_absolute_drift_ppm_exclusive": 100.0,
    "maximum_observation_duration_ns": 900_000_000_000,
    "audit_file": "post-bootstrap-quality.json",
    "samples_file": "post-bootstrap-quality-samples.jsonl",
    "require_unsafe_latched_false": True,
    "require_current_writer_authorized": True,
    "require_feed_count_advancing": True,
    "require_chronyd_go2_selected": True,
}


class Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class ChronySockSample(ctypes.Structure):
    # Native layout is intentional.  This is the ABI defined by chrony's
    # refclock_sock.c for a client built on the same NX userspace.
    _fields_ = [
        ("tv", Timeval),
        ("offset", ctypes.c_double),
        ("pulse", ctypes.c_int),
        ("leap", ctypes.c_int),
        ("padding", ctypes.c_int),
        ("magic", ctypes.c_int),
    ]


def _compact_gid(value: str) -> bytes:
    compact = value.replace(".", "").replace(":", "").replace("-", "")
    compact = "".join(compact.split())
    if len(compact) < 32 or len(compact) % 2:
        raise ValueError("writer_gid must contain at least 16 hexadecimal bytes")
    try:
        return bytes.fromhex(compact)
    except ValueError as error:
        raise ValueError("writer_gid contains non-hexadecimal characters") from error


def _secure_regular_file(path: Path, label: str, expected_owner_uid: int = 0) -> str:
    """Read a root-controlled gate without following a final symlink."""

    file_stat = path.lstat()
    if not stat.S_ISREG(file_stat.st_mode):
        raise PermissionError(f"{label} is not a regular file: {path}")
    if file_stat.st_uid != expected_owner_uid:
        raise PermissionError(
            f"{label} must be owned by uid {expected_owner_uid}: {path}"
        )
    if file_stat.st_mode & 0o022:
        raise PermissionError(f"{label} must not be group/world writable: {path}")
    if file_stat.st_size > 64 * 1024:
        raise PermissionError(f"{label} is unexpectedly large: {path}")
    return path.read_text(encoding="utf-8")


def validate_enable_file(path: Path, expected_owner_uid: int = 0) -> None:
    content = _secure_regular_file(path, "enable file", expected_owner_uid)
    if content != ENABLE_TOKEN:
        raise PermissionError("enable file acknowledgement token is not exact")


def validate_approval_payload(payload: Any) -> dict[str, dict[str, Any]]:
    if (
        not isinstance(payload, dict)
        or payload.get("schema_version") != APPROVAL_SCHEMA_VERSION
    ):
        raise ValueError(
            "approval must be a recomputed schema_version=3 JSON object"
        )
    if payload.get("activation_authorized") is not False:
        raise ValueError("approval payload must remain non-authorizing")
    if payload.get("post_bootstrap_quality_gate") != POST_BOOTSTRAP_POLICY:
        raise ValueError("approval post-bootstrap quality policy is not exact")
    evidence = payload.get("evidence_bundle")
    if not isinstance(evidence, dict):
        raise ValueError("approval is missing evidence_bundle")
    pcap_digest = str(evidence.get("pcap_sha256", "")).lower()
    if len(pcap_digest) != 64 or any(
        character not in "0123456789abcdef" for character in pcap_digest
    ):
        raise ValueError("approval has an invalid PCAP digest")
    writers = payload.get("approved_writers")
    if not isinstance(writers, list) or not writers:
        raise ValueError("approval requires a non-empty approved_writers list")
    result: dict[str, dict[str, Any]] = {}
    for item in writers:
        if not isinstance(item, dict):
            raise ValueError("each approved writer must be a JSON object")
        topic = item.get("topic")
        if topic not in ("/sportmodestate", "/lf/sportmodestate"):
            raise ValueError(f"unsupported approved topic: {topic!r}")
        if topic in result:
            raise ValueError(f"duplicate approved topic: {topic}")
        gid = _compact_gid(str(item.get("writer_gid", "")))
        prefix = str(item.get("rtps_participant_guid_prefix", "")).lower()
        if prefix != gid[:12].hex():
            raise ValueError(f"GUID prefix does not match writer_gid for {topic}")
        try:
            source_ip = ipaddress.ip_address(str(item.get("source_ip", "")))
        except ValueError as error:
            raise ValueError(f"invalid approved source_ip for {topic}") from error
        expected_network = ipaddress.ip_network("192.168.123.0/24")
        if source_ip not in expected_network or source_ip in (
            expected_network.network_address,
            expected_network.broadcast_address,
            ipaddress.ip_address("192.168.123.18"),
            ipaddress.ip_address("192.168.123.99"),
        ):
            raise ValueError(f"source_ip is not an eligible Go2 subnet host: {source_ip}")
        if item.get("correlation_method") != EXPECTED_CORRELATION_METHOD:
            raise ValueError(f"unsupported correlation method for {topic}")
        if item.get("correlation_conclusion") != EXPECTED_CORRELATION_CONCLUSION:
            raise ValueError(f"writer source is not uniquely proven for {topic}")
        digest = str(item.get("correlation_sha256", "")).lower()
        if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
            raise ValueError(f"invalid correlation_sha256 for {topic}")
        if item.get("pcap_sha256") != pcap_digest:
            raise ValueError(f"writer PCAP digest does not match bundle for {topic}")
        for filename_key in ("topic_info_file", "correlation_file"):
            filename = item.get(filename_key)
            if (
                not isinstance(filename, str)
                or not filename
                or Path(filename).name != filename
            ):
                raise ValueError(f"invalid {filename_key} for {topic}")
        topic_info_digest = str(item.get("topic_info_sha256", "")).lower()
        if len(topic_info_digest) != 64 or any(
            character not in "0123456789abcdef" for character in topic_info_digest
        ):
            raise ValueError(f"invalid topic_info_sha256 for {topic}")
        result[topic] = {
            "gid": gid,
            "source_ip": str(source_ip),
            "correlation_sha256": digest,
            "pcap_sha256": pcap_digest,
        }
    if "/sportmodestate" not in result:
        raise ValueError("approval requires the primary /sportmodestate writer")
    stability = payload.get("pre_bootstrap_stability")
    if not isinstance(stability, dict):
        raise ValueError("approval requires verified pre-bootstrap stability")
    if (
        stability.get("minimum_duration_ns") != MINIMUM_STABILITY_DURATION_NS
        or not isinstance(stability.get("observed_duration_ns"), int)
        or stability["observed_duration_ns"] < MINIMUM_STABILITY_DURATION_NS
    ):
        raise ValueError("approval lacks a verified two-hour stability run")
    if _compact_gid(str(stability.get("writer_gid", ""))) != result[
        "/sportmodestate"
    ]["gid"]:
        raise ValueError("stability writer does not match the current primary writer")
    trials = payload.get("cold_boot_identity_trials")
    if not isinstance(trials, list) or len(trials) < MINIMUM_COLD_BOOT_TRIALS:
        raise ValueError("approval requires three verified cold-boot trials")
    trial_ids: set[str] = set()
    boot_ids: set[str] = set()
    writer_gids: set[bytes] = set()
    current_count = 0
    for trial in trials:
        if not isinstance(trial, dict):
            raise ValueError("cold-boot trial must be an object")
        trial_id = trial.get("trial_id")
        boot_id = trial.get("boot_id")
        gid = _compact_gid(str(trial.get("writer_gid", "")))
        if (
            not isinstance(trial_id, str)
            or not trial_id
            or trial_id in trial_ids
            or not isinstance(boot_id, str)
            or not boot_id
            or boot_id in boot_ids
            or gid in writer_gids
        ):
            raise ValueError("cold-boot identities must be complete and distinct")
        trial_ids.add(trial_id)
        boot_ids.add(boot_id)
        writer_gids.add(gid)
        if trial.get("current_session") is True:
            current_count += 1
            if gid != result["/sportmodestate"]["gid"]:
                raise ValueError("current cold-boot writer does not match approval")
    if current_count != 1:
        raise ValueError("exactly one cold-boot trial must be current")
    return result


def load_approval(
    path: Path, expected_owner_uid: int = 0
) -> dict[str, dict[str, Any]]:
    content = _secure_regular_file(path, "approval file", expected_owner_uid)
    try:
        payload = json.loads(content)
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid approval JSON: {error}") from error
    return validate_approval_payload(payload)


def approval_digest_matches(path: Path, expected_sha256: str) -> bool:
    """Optional helper for deployment validation of retained correlation JSON."""

    if not path.is_file():
        return False
    return hashlib.sha256(path.read_bytes()).hexdigest() == expected_sha256


def endpoint_gid_bytes(endpoint: Any) -> bytes:
    raw = endpoint.endpoint_gid
    if isinstance(raw, bytes):
        return raw
    if isinstance(raw, bytearray):
        return bytes(raw)
    return bytes(int(value) for value in raw)


def authorized_graph_topics(
    endpoint_lookup: Callable[[str], list[Any]],
    approvals: dict[str, dict[str, Any]],
) -> tuple[set[str], dict[str, str]]:
    """Require exactly one live publisher and a same-session GID match."""

    authorized: set[str] = set()
    reasons: dict[str, str] = {}
    for topic, approval in approvals.items():
        endpoints = endpoint_lookup(topic)
        if len(endpoints) != 1:
            reasons[topic] = f"publisher_count={len(endpoints)}"
            continue
        current_gid = endpoint_gid_bytes(endpoints[0])
        approved_gid = approval["gid"]
        if len(current_gid) < 16 or current_gid[:16] != approved_gid[:16]:
            reasons[topic] = "current_writer_gid_does_not_match_approval"
            continue
        authorized.add(topic)
        reasons[topic] = (
            "live_gid_matches_raw_pcap_source=" + approval["source_ip"]
        )
    return authorized, reasons


class ChronySocketWriter:
    """Send native chrony SOCK refclock samples; never adjust a clock."""

    def __init__(self, path: Path) -> None:
        socket_stat = path.lstat()
        if not stat.S_ISSOCK(socket_stat.st_mode):
            raise RuntimeError(f"chrony refclock path is not a Unix socket: {path}")
        if socket_stat.st_mode & 0o002:
            raise PermissionError(f"chrony refclock socket is world-writable: {path}")
        self._path = str(path)
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def close(self) -> None:
        self._socket.close()

    @staticmethod
    def encode_sample(source_stamp_ns: int, receipt_realtime_ns: int) -> bytes:
        # timeval is microsecond-resolution.  Compute offset from the exact
        # transmitted timeval so sub-microsecond source precision is retained.
        seconds, remaining_ns = divmod(receipt_realtime_ns, 1_000_000_000)
        microseconds = remaining_ns // 1_000
        timeval_ns = seconds * 1_000_000_000 + microseconds * 1_000
        sample = ChronySockSample(
            tv=Timeval(tv_sec=seconds, tv_usec=microseconds),
            offset=(source_stamp_ns - timeval_ns) / 1_000_000_000.0,
            pulse=0,
            leap=0,
            padding=0,
            magic=SOCK_MAGIC,
        )
        return bytes(sample)

    def send(self, source_stamp_ns: int, receipt_realtime_ns: int) -> int:
        payload = self.encode_sample(source_stamp_ns, receipt_realtime_ns)
        sent = self._socket.sendto(payload, self._path)
        if sent != len(payload):
            raise RuntimeError(f"partial chrony SOCK sample: {sent}/{len(payload)}")
        return sent


class QualityWindow:
    def __init__(self, size: int) -> None:
        if size <= 0:
            raise ValueError("quality window size must be positive")
        self.samples: deque[tuple[int, int]] = deque(maxlen=size)

    def add(self, offset_ns: int, monotonic_ns: int) -> None:
        self.samples.append((offset_ns, monotonic_ns))

    def reset(self) -> None:
        self.samples.clear()

    def metrics(self) -> tuple[int, float | None, float | None]:
        count = len(self.samples)
        if not count:
            return 0, None, None
        ordered = sorted(offset for offset, _ in self.samples)
        median = ordered[count // 2]
        deviations = sorted(abs(value - median) for value in ordered)
        index = min(count - 1, math.ceil(0.95 * count) - 1)
        jitter_p95_ns = float(deviations[index])
        drift_ppm: float | None = None
        if count > 1:
            first_offset_ns, first_monotonic_ns = self.samples[0]
            last_offset_ns, last_monotonic_ns = self.samples[-1]
            elapsed = last_monotonic_ns - first_monotonic_ns
            if elapsed > 0:
                drift_ppm = (
                    (last_offset_ns - first_offset_ns)
                    / elapsed
                    * 1_000_000.0
                )
        return count, jitter_p95_ns, drift_ppm


def source_timing_jump_reason(
    source_delta_ns: int | None,
    receipt_monotonic_delta_ns: int | None,
    max_delta_error_ns: int,
) -> str | None:
    """Reject both backwards and implausibly large forward source jumps."""

    if source_delta_ns is None or receipt_monotonic_delta_ns is None:
        return None
    if source_delta_ns < 0:
        return "source_timestamp_regression"
    if receipt_monotonic_delta_ns <= 0:
        return "receipt_monotonic_not_advancing"
    if abs(source_delta_ns - receipt_monotonic_delta_ns) > max_delta_error_ns:
        return "source_receipt_delta_discontinuity"
    return None


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Go2 source-clock observer; chrony feed is explicitly gated"
    )
    parser.add_argument(
        "--mode",
        choices=("observe-only", "verify-graph", "feed-chrony"),
        default="observe-only",
    )
    parser.add_argument("--primary-topic", default="/sportmodestate")
    parser.add_argument("--fallback-topic", default="/lf/sportmodestate")
    parser.add_argument(
        "--duration-seconds",
        type=int,
        default=60,
        help="0 means unbounded and is allowed only in explicit feed-chrony mode",
    )
    parser.add_argument("--enable-file", type=Path, default=Path(
        "/etc/robonix-go2/ENABLE_GO2_REFCLOCK"
    ))
    parser.add_argument("--approval-file", type=Path, default=Path(
        "/etc/robonix-go2/go2-clock-ref-approval.json"
    ))
    parser.add_argument(
        "--gate-owner-uid",
        type=int,
        default=0,
        help=(
            "expected gate-file owner; host service must keep 0, while the "
            "immutable sidecar passes its fixed non-root container UID"
        ),
    )
    parser.add_argument(
        "--chrony-socket", type=Path, default=Path("/run/chrony/go2.sock")
    )
    parser.add_argument("--warmup-samples", type=int, default=60)
    parser.add_argument("--window-samples", type=int, default=256)
    parser.add_argument("--max-jitter-ms", type=float, default=20.0)
    parser.add_argument("--max-absolute-drift-ppm", type=float, default=500.0)
    parser.add_argument("--max-feed-hz", type=float, default=10.0)
    parser.add_argument("--primary-stale-seconds", type=float, default=1.0)
    parser.add_argument("--fallback-agreement-ms", type=float, default=20.0)
    parser.add_argument("--max-source-jump-ms", type=float, default=250.0)
    parser.add_argument("--startup-timeout-seconds", type=float, default=20.0)
    parser.add_argument("--health-file", type=Path)
    return parser


def _validate_arguments(arguments: argparse.Namespace) -> None:
    if arguments.duration_seconds < 0 or arguments.duration_seconds > 86_400:
        raise ValueError("duration-seconds must be 0..86400")
    if arguments.mode == "observe-only" and arguments.duration_seconds == 0:
        raise ValueError("observe-only mode must remain time-bounded")
    if not 10 <= arguments.warmup_samples <= 10_000:
        raise ValueError("warmup-samples must be 10..10000")
    if not arguments.warmup_samples <= arguments.window_samples <= 100_000:
        raise ValueError("window-samples must be >= warmup-samples and <=100000")
    if not 0.1 <= arguments.max_jitter_ms <= 1000.0:
        raise ValueError("max-jitter-ms must be 0.1..1000")
    if not 0.1 <= arguments.max_absolute_drift_ppm <= 100_000.0:
        raise ValueError("max-absolute-drift-ppm must be 0.1..100000")
    if not 0.1 <= arguments.max_feed_hz <= 100.0:
        raise ValueError("max-feed-hz must be 0.1..100")
    if not 0.1 <= arguments.primary_stale_seconds <= 60.0:
        raise ValueError("primary-stale-seconds must be 0.1..60")
    if not 0.1 <= arguments.fallback_agreement_ms <= 1000.0:
        raise ValueError("fallback-agreement-ms must be 0.1..1000")
    if not 1.0 <= arguments.max_source_jump_ms <= 10_000.0:
        raise ValueError("max-source-jump-ms must be 1..10000")
    if not 1.0 <= arguments.startup_timeout_seconds <= 300.0:
        raise ValueError("startup-timeout-seconds must be 1..300")
    if arguments.gate_owner_uid < 0 or arguments.gate_owner_uid > 2_147_483_647:
        raise ValueError("gate-owner-uid is outside the Linux UID range")


def run(arguments: argparse.Namespace) -> int:
    _validate_arguments(arguments)
    feed_mode = arguments.mode == "feed-chrony"
    approval_mode = arguments.mode in ("verify-graph", "feed-chrony")
    approvals: dict[str, dict[str, Any]] = {}
    writer: ChronySocketWriter | None = None
    if approval_mode:
        if os.geteuid() == 0:
            raise PermissionError("go2_clock_ref must run as an unprivileged user")
        validate_enable_file(arguments.enable_file, arguments.gate_owner_uid)
        approvals = load_approval(arguments.approval_file, arguments.gate_owner_uid)
    if feed_mode:
        writer = ChronySocketWriter(arguments.chrony_socket)

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from unitree_go.msg import SportModeState
    except ImportError as error:
        if writer is not None:
            writer.close()
        print(f"missing ROS 2 or Unitree message dependency: {error}", file=sys.stderr)
        return 127

    topics = (arguments.primary_topic, arguments.fallback_topic)
    if approval_mode and not set(approvals).issubset(set(topics)):
        raise ValueError("approved topic does not match configured primary/fallback topic")

    trackers = {
        topic: StreamTracker(topic, topic, retained_offset_limit=arguments.window_samples)
        for topic in topics
    }
    quality = {topic: QualityWindow(arguments.window_samples) for topic in topics}
    last_receipt = {topic: None for topic in topics}
    last_offset = {topic: None for topic in topics}
    last_feed_monotonic_ns = 0
    authorized_topics: set[str] = set()
    graph_reasons: dict[str, str] = {}
    unsafe_latched = False
    feed_count = 0
    clock_discontinuity_count = 0
    previous_clock_pair: tuple[int, int] | None = None
    last_valid_sample_monotonic_ns = 0
    graph_ever_authorized = False

    def write_health() -> None:
        if arguments.health_file is None:
            return
        stream_quality: dict[str, dict[str, int | float | None]] = {}
        for topic in topics:
            count, jitter_p95_ns, drift_ppm = quality[topic].metrics()
            stream_quality[topic] = {
                "window_sample_count": count,
                "last_source_minus_local_ns": last_offset[topic],
                "offset_jitter_abs_deviation_p95_ns": jitter_p95_ns,
                "estimated_drift_ppm": drift_ppm,
                "last_receipt_monotonic_ns": last_receipt[topic],
            }
        payload = {
            "schema_version": 1,
            "unsafe_latched": unsafe_latched,
            "feed_count": feed_count,
            "authorized_topics": sorted(authorized_topics),
            "required_topics": sorted(approvals),
            "clock_discontinuity_count": clock_discontinuity_count,
            "stream_quality": stream_quality,
            "last_valid_sample_monotonic_ns": last_valid_sample_monotonic_ns,
            "updated_monotonic_ns": time.monotonic_ns(),
        }
        temporary = arguments.health_file.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
        os.chmod(temporary, 0o600)
        temporary.replace(arguments.health_file)

    class Go2ClockRefNode(Node):
        def __init__(self) -> None:
            super().__init__("go2_clock_ref_subscription_only")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.subscriptions = [
                self.create_subscription(
                    SportModeState,
                    topic,
                    self._callback_for(topic),
                    qos,
                )
                for topic in topics
            ]
            self.graph_timer = self.create_timer(1.0, self._refresh_graph_authority)

        def _refresh_graph_authority(self) -> None:
            nonlocal authorized_topics, graph_reasons, graph_ever_authorized
            if not approval_mode:
                return
            authorized_topics, graph_reasons = authorized_graph_topics(
                self.get_publishers_info_by_topic, approvals
            )
            if set(approvals).issubset(authorized_topics):
                graph_ever_authorized = True
            write_health()

        def _callback_for(self, topic: str) -> Callable[[Any], None]:
            def callback(message: Any) -> None:
                nonlocal unsafe_latched, last_feed_monotonic_ns, feed_count
                nonlocal clock_discontinuity_count, previous_clock_pair
                nonlocal last_valid_sample_monotonic_ns
                receipt_realtime, receipt_monotonic, clock_span = read_clock_pair()
                if previous_clock_pair is not None:
                    realtime_delta = receipt_realtime - previous_clock_pair[0]
                    monotonic_delta = receipt_monotonic - previous_clock_pair[1]
                    if abs(realtime_delta - monotonic_delta) > 50_000_000:
                        # Expected after an initial chronyd step.  Discard the
                        # pre-step quality windows and warm up again in the new
                        # clock domain; never synthesize or restamp a sample.
                        for window in quality.values():
                            window.reset()
                        clock_discontinuity_count += 1
                        self.get_logger().warning(
                            "local realtime discontinuity observed; quality "
                            "windows reset before any further chrony sample"
                        )
                previous_clock_pair = (receipt_realtime, receipt_monotonic)
                observation = trackers[topic].observe(
                    int(message.stamp.sec),
                    int(message.stamp.nanosec),
                    receipt_realtime,
                    receipt_monotonic,
                    clock_span,
                )
                if observation.source_stamp_ns is None:
                    unsafe_latched = feed_mode
                    return
                if observation.status == "regression":
                    unsafe_latched = feed_mode
                    self.get_logger().error(
                        f"source timestamp regression on {topic}; feed latched off"
                    )
                    return
                if observation.status == "duplicate":
                    return
                jump_reason = source_timing_jump_reason(
                    observation.source_delta_ns,
                    observation.receipt_monotonic_delta_ns,
                    int(arguments.max_source_jump_ms * 1_000_000),
                )
                if jump_reason is not None:
                    unsafe_latched = approval_mode
                    self.get_logger().error(
                        f"{jump_reason} on {topic}; formal path latched off"
                    )
                    write_health()
                    return
                offset_ns = observation.source_minus_receipt_ns
                assert offset_ns is not None
                last_valid_sample_monotonic_ns = receipt_monotonic
                quality[topic].add(offset_ns, receipt_monotonic)
                last_receipt[topic] = receipt_monotonic
                last_offset[topic] = offset_ns
                if not feed_mode or writer is None or unsafe_latched:
                    return
                if topic not in authorized_topics:
                    return

                active_topic = arguments.primary_topic
                primary_receipt = last_receipt[arguments.primary_topic]
                if (
                    topic == arguments.fallback_topic
                    and (
                        primary_receipt is None
                        or receipt_monotonic - primary_receipt
                        > int(arguments.primary_stale_seconds * 1_000_000_000)
                    )
                ):
                    active_topic = arguments.fallback_topic
                if topic != active_topic:
                    return
                if topic == arguments.fallback_topic:
                    primary_offset = last_offset[arguments.primary_topic]
                    if primary_offset is None or abs(offset_ns - primary_offset) > int(
                        arguments.fallback_agreement_ms * 1_000_000
                    ):
                        return

                count, jitter_p95_ns, drift_ppm = quality[topic].metrics()
                if count < arguments.warmup_samples:
                    return
                if jitter_p95_ns is None or jitter_p95_ns > (
                    arguments.max_jitter_ms * 1_000_000
                ):
                    return
                if drift_ppm is None or abs(drift_ppm) > arguments.max_absolute_drift_ppm:
                    return
                min_interval_ns = int(1_000_000_000 / arguments.max_feed_hz)
                if receipt_monotonic - last_feed_monotonic_ns < min_interval_ns:
                    return
                writer.send(observation.source_stamp_ns, receipt_realtime)
                last_feed_monotonic_ns = receipt_monotonic
                feed_count += 1
                write_health()

            return callback

    print(
        json.dumps(
            {
                "mode": arguments.mode,
                "clock_adjustment_by_this_process": False,
                "ros_publishers_created": False,
                "approved_topics": sorted(approvals),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    rclpy.init(args=None)
    node = Go2ClockRefNode()
    started_ns = time.monotonic_ns()
    graph_loss_started_ns: int | None = None
    startup_refused = False
    try:
        while rclpy.ok():
            if arguments.duration_seconds and time.monotonic_ns() - started_ns >= (
                arguments.duration_seconds * 1_000_000_000
            ):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            now_ns = time.monotonic_ns()
            if approval_mode:
                fully_authorized = set(approvals).issubset(authorized_topics)
                if not graph_ever_authorized and now_ns - started_ns > int(
                    arguments.startup_timeout_seconds * 1_000_000_000
                ):
                    startup_refused = True
                    break
                if graph_ever_authorized and not fully_authorized:
                    if graph_loss_started_ns is None:
                        graph_loss_started_ns = now_ns
                    elif now_ns - graph_loss_started_ns > 3_000_000_000:
                        unsafe_latched = True
                        break
                else:
                    graph_loss_started_ns = None
                if arguments.mode == "verify-graph" and fully_authorized:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if writer is not None:
            writer.close()

    print(
        json.dumps(
            {
                "mode": arguments.mode,
                "feed_count": feed_count,
                "unsafe_latched": unsafe_latched,
                "clock_discontinuity_count": clock_discontinuity_count,
                "graph_authorization": graph_reasons,
                "streams": [trackers[topic].summary() for topic in topics],
            },
            sort_keys=True,
        ),
        flush=True,
    )
    if startup_refused:
        return 4
    if arguments.mode == "verify-graph" and not set(approvals).issubset(
        authorized_topics
    ):
        return 4
    if feed_mode and unsafe_latched:
        return 5
    return 0


def main(argv: list[str] | None = None) -> int:
    try:
        arguments = build_argument_parser().parse_args(argv)
        return run(arguments)
    except (OSError, PermissionError, RuntimeError, ValueError) as error:
        print(f"go2_clock_ref refused to start: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
