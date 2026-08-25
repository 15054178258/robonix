from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC_DIR = ROOT / "deploy" / "time-sync"

import sys

sys.path.insert(0, str(TIME_SYNC_DIR))

from go2_time_core import StreamTracker
from workstation_nomotion_approval import ACK, load_approval


def load_script(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


generator = load_script(
    "workstation_nomotion_approval_generator",
    "prepare_workstation_nomotion_offset_approval.py",
)


NS = 1_000_000_000
OFFSET = 748_294_000_000
SOURCE_BASE = 1_700_000_000 * NS
MONOTONIC_BASE = 100 * NS
CLOCK_BASE = SOURCE_BASE + OFFSET - MONOTONIC_BASE
TOPICS = dict(generator.EXPECTED_TIME_TOPICS)
REQUIRED = dict(generator.EXPECTED_RAW_TOPICS)


def rtps_data(prefix: bytes, writer_entity: bytes) -> bytes:
    header = b"RTPS" + b"\x02\x03" + b"\x01\x0f" + prefix
    body = b"\x00\x00" + b"\x10\x00" + b"\x00" * 4 + writer_entity + b"\x00" * 8
    return header + struct.pack("<BBH", 0x15, 0x01, len(body)) + body


def ethernet_ipv4_udp(source: str, destination: str, payload: bytes) -> bytes:
    udp = struct.pack("!HHHH", 7400, 7412, 8 + len(payload), 0) + payload
    ip = struct.pack(
        "!BBHHHBBH4s4s",
        0x45,
        0,
        20 + len(udp),
        1,
        0,
        64,
        17,
        0,
        socket.inet_aton(source),
        socket.inet_aton(destination),
    ) + udp
    return (
        b"\x01\x00\x5e\x7f\x00\x01"
        + b"\x02\x00\x00\x00\x00\x01"
        + b"\x08\x00"
        + ip
    )


def write_pcap(path: Path, frames: list[bytes]) -> None:
    with path.open("wb") as stream:
        stream.write(struct.pack("<IHHIIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))
        for index, frame in enumerate(frames, 1):
            stream.write(struct.pack("<IIII", index, 0, len(frame), len(frame)))
            stream.write(frame)
    path.chmod(0o600)


def private_text(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")
    path.chmod(0o600)


def private_json(path: Path, value: object) -> None:
    private_text(path, json.dumps(value, sort_keys=True) + "\n")


class EvidenceBuilder:
    def __init__(self, root: Path, source_ip: str = "192.168.123.161") -> None:
        self.time = root / "time"
        self.identity = root / "identity"
        self.time.mkdir(mode=0o700)
        self.identity.mkdir(mode=0o700)
        self.gids: dict[str, str] = {}
        frames: list[bytes] = []
        for index, stream in enumerate(REQUIRED, 1):
            prefix = bytes([index]) * 12
            entity = bytes((0, 0, 10, index))
            gid_bytes = prefix + entity + bytes(8)
            self.gids[stream] = gid_bytes.hex()
            frames.append(
                ethernet_ipv4_udp(
                    source_ip,
                    "192.168.123.99",
                    rtps_data(prefix, entity),
                )
            )
        pcap = self.identity / generator.PCAP_FILENAME
        write_pcap(pcap, frames)
        for stream, gid in self.gids.items():
            dotted = ".".join(gid[offset : offset + 2] for offset in range(0, 48, 2))
            private_text(
                self.identity / f"{stream}.topic-info.txt",
                "Publisher count: 1\n"
                "Node name: synthetic_writer\n"
                "Endpoint type: PUBLISHER\n"
                f"GID: {dotted}\n"
                "QoS profile:\n  Reliability: BEST_EFFORT\n",
            )
            private_json(
                self.identity / f"{stream}.correlation.json",
                generator.correlate(pcap, dotted),
            )
        self._write_time_evidence()

    def _write_time_evidence(self) -> None:
        duration_seconds = 60
        metadata = {
            "schema_version": 1,
            "mode": "read-only-no-adjust",
            "clock_adjustment_requested": False,
            "ros_publishers_created": False,
            "unitree_clients_created": False,
            "hostname": "offline-test",
            "pid": 1234,
            "started_realtime_ns": SOURCE_BASE + OFFSET,
            "started_monotonic_ns": MONOTONIC_BASE,
            "started_clock_read_span_ns": 1_000,
            "duration_seconds": duration_seconds,
            "max_samples": 10_000,
            "retained_offsets_per_stream": 10_000,
            "topics": TOPICS,
            "stream_order": list(TOPICS),
            "qualification_streams": list(generator.EXPECTED_QUALIFICATION_STREAMS),
            "witness_streams": list(generator.EXPECTED_WITNESS_STREAMS),
            "interpretation": "offline synthetic test evidence",
        }
        trackers = {
            stream: StreamTracker(stream, topic, retained_offset_limit=10_000)
            for stream, topic in TOPICS.items()
        }
        delays = {
            "sport_primary": 5_000_000,
            "sport_fallback": 8_000_000,
            "mid360_cloud": 75_000_000,
            "mid360_imu": 4_000_000,
            "mid360_odom": 6_000_000,
        }
        records: list[dict] = []
        for index in range(duration_seconds * 10 + 1):
            source_ns = SOURCE_BASE + index * 100_000_000
            seconds, nanoseconds = divmod(source_ns, NS)
            for stream in TOPICS:
                delay = delays[stream]
                receipt_monotonic_ns = MONOTONIC_BASE + index * 100_000_000 + delay
                receipt_realtime_ns = receipt_monotonic_ns + CLOCK_BASE
                records.append(
                    trackers[stream].observe(
                        seconds,
                        nanoseconds,
                        receipt_realtime_ns,
                        receipt_monotonic_ns,
                        1_000,
                    ).as_dict()
                )
        private_json(self.time / "metadata.json", metadata)
        private_text(
            self.time / "samples.jsonl",
            "".join(
                json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"
                for record in records
            ),
        )
        summaries = [trackers[stream].summary() for stream in TOPICS]
        summary = {
            "schema_version": 1,
            "mode": "read-only-no-adjust",
            "exit_reason": "duration_elapsed",
            "total_samples": len(records),
            "started_realtime_ns": metadata["started_realtime_ns"],
            "finished_realtime_ns": metadata["started_realtime_ns"] + duration_seconds * NS,
            "elapsed_monotonic_ns": duration_seconds * NS,
            "configured_duration_ns": duration_seconds * NS,
            "finished_clock_read_span_ns": 1_000,
            "streams": summaries,
            "pairwise_median_offset_comparisons": [],
            "timestamp_anomaly_count": 0,
            "cleanup_errors": [],
            "safe_for_clock_discipline": False,
            "note": "offline synthetic test evidence",
        }
        private_json(self.time / "summary.json", summary)


class WorkstationNomotionApprovalGeneratorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        (ROOT / "rbnx-build").mkdir(parents=True, exist_ok=True)

    def test_derives_four_gids_and_fixed_offset_without_manual_values(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as temporary:
            directory = Path(temporary)
            evidence = EvidenceBuilder(directory)
            output = directory / "generated" / "approval.json"
            approval_path, manifest_path = generator.prepare_approval(
                time_evidence_dir=evidence.time,
                identity_evidence_dir=evidence.identity,
                session_id="go2-session-offline-test-01",
                operator_ack=ACK,
                validity_seconds=900,
                output=output,
                now_realtime_ns=SOURCE_BASE + OFFSET,
            )
            approval = load_approval(
                approval_path, now_realtime_ns=SOURCE_BASE + OFFSET
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

            self.assertEqual(dict(approval.writer_gids), evidence.gids)
            self.assertEqual(approval.fixed_local_minus_source_offset_ns, OFFSET)
            self.assertEqual(approval.approved_affine_common_drift_ppm, 0.0)
            self.assertEqual(
                approval.offset_evidence_sha256,
                hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
            )
            self.assertEqual(
                manifest["timing"]["derived_fixed_local_minus_source_offset_ns"],
                OFFSET,
            )
            self.assertEqual(manifest["timing"]["qualification_reasons"], [])
            self.assertEqual(
                manifest["timing"]["derived_approved_affine_common_drift_ppm"],
                0.0,
            )
            affine = manifest["timing"]["affine_drift_qualification"]
            self.assertEqual(affine["window_ns"], 30 * NS)
            self.assertEqual(
                affine["first_window"]["common_drift_ppm"], 0.0
            )
            self.assertEqual(
                affine["second_window"]["common_drift_ppm"], 0.0
            )
            self.assertEqual(approval_path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(manifest_path.stat().st_mode & 0o777, 0o600)
            with self.assertRaises(FileExistsError):
                generator.prepare_approval(
                    time_evidence_dir=evidence.time,
                    identity_evidence_dir=evidence.identity,
                    session_id="go2-session-offline-test-01",
                    operator_ack=ACK,
                    validity_seconds=900,
                    output=output,
                    now_realtime_ns=SOURCE_BASE + OFFSET,
                )

    def test_rejects_unproven_source_and_does_not_publish_outputs(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as temporary:
            directory = Path(temporary)
            evidence = EvidenceBuilder(directory, source_ip="192.168.123.77")
            output = directory / "approval.json"
            with self.assertRaisesRegex(
                generator.PreparationError, "source is not exactly 192.168.123.161"
            ):
                generator.prepare_approval(
                    time_evidence_dir=evidence.time,
                    identity_evidence_dir=evidence.identity,
                    session_id="go2-session-offline-test-02",
                    operator_ack=ACK,
                    validity_seconds=900,
                    output=output,
                    now_realtime_ns=SOURCE_BASE + OFFSET,
                )
            self.assertFalse(output.exists())
            self.assertFalse(
                output.with_name(output.name + ".evidence-manifest.json").exists()
            )

    def test_rejects_incomplete_cleanup_and_wrong_ack(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as temporary:
            directory = Path(temporary)
            evidence = EvidenceBuilder(directory)
            output = directory / "approval.json"
            with self.assertRaisesRegex(generator.PreparationError, "acknowledgement"):
                generator.prepare_approval(
                    time_evidence_dir=evidence.time,
                    identity_evidence_dir=evidence.identity,
                    session_id="go2-session-offline-test-03",
                    operator_ack="YES",
                    validity_seconds=900,
                    output=output,
                    now_realtime_ns=SOURCE_BASE + OFFSET,
                )
            summary_path = evidence.time / "summary.json"
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            summary["cleanup_errors"] = ["synthetic failure"]
            private_json(summary_path, summary)
            with self.assertRaisesRegex(generator.PreparationError, "cleanup_errors"):
                generator.prepare_approval(
                    time_evidence_dir=evidence.time,
                    identity_evidence_dir=evidence.identity,
                    session_id="go2-session-offline-test-03",
                    operator_ack=ACK,
                    validity_seconds=900,
                    output=output,
                    now_realtime_ns=SOURCE_BASE + OFFSET,
                )
            self.assertFalse(output.exists())

    def test_rejects_output_outside_ignored_repository_roots(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            outside = Path(temporary) / "approval.json"
            with self.assertRaisesRegex(
                generator.PreparationError, "ignored local root"
            ):
                generator.prepare_approval(
                    time_evidence_dir=Path(temporary) / "unused-time",
                    identity_evidence_dir=Path(temporary) / "unused-identity",
                    session_id="go2-session-offline-test-04",
                    operator_ack=ACK,
                    validity_seconds=900,
                    output=outside,
                    now_realtime_ns=SOURCE_BASE + OFFSET,
                )
            self.assertFalse(outside.exists())

        with tempfile.TemporaryDirectory() as outside_temporary:
            with tempfile.TemporaryDirectory(dir=ROOT / "rbnx-build") as inside:
                escape = Path(inside) / "escape"
                escape.symlink_to(outside_temporary, target_is_directory=True)
                escaped_output = escape / "approval.json"
                with self.assertRaisesRegex(
                    generator.PreparationError, "ignored local root"
                ):
                    generator.prepare_approval(
                        time_evidence_dir=Path(outside_temporary) / "unused-time",
                        identity_evidence_dir=Path(outside_temporary) / "unused-identity",
                        session_id="go2-session-offline-test-05",
                        operator_ack=ACK,
                        validity_seconds=900,
                        output=escaped_output,
                        now_realtime_ns=SOURCE_BASE + OFFSET,
                    )
                self.assertFalse((Path(outside_temporary) / "approval.json").exists())

    def test_source_is_statically_offline_and_has_no_manual_gid_or_offset_cli(self) -> None:
        source = (
            ROOT / "scripts" / "prepare_workstation_nomotion_offset_approval.py"
        ).read_text(encoding="utf-8")
        for forbidden in (
            "import rclpy",
            "create_subscription",
            "create_publisher",
            "import subprocess",
            "import socket",
            "SportClient",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "--writer-gid",
            "--fixed-offset",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
