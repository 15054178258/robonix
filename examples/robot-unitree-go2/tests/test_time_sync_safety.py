from __future__ import annotations

import ctypes
import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
TIME_DIR = ROOT / "deploy" / "time-sync"

import sys

sys.path.insert(0, str(TIME_DIR))

import go2_clock_ref
import go2_time_core
import evidence_bundle
import post_bootstrap_quality_gate


def load_script(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


locator = load_script("rtps_locator", "correlate_rtps_writer_locator.py")
approval_tool = load_script("approval_tool", "prepare_go2_time_approval.py")


GID = "01.10.8f.17.35.15.4b.ce.5e.5f.ab.49.00.00.0a.03.00.00.00.00.00.00.00.00"
PREFIX = bytes.fromhex("01108f1735154bce5e5fab49")
ENTITY = bytes.fromhex("00000a03")


def gid_for_prefix(prefix: bytes) -> str:
    return ".".join(f"{value:02x}" for value in prefix + ENTITY + b"\x00" * 8)


def rtps_data(prefix: bytes, writer_entity: bytes) -> bytes:
    header = b"RTPS" + b"\x02\x03" + b"\x01\x0f" + prefix
    body = b"\x00\x00" + b"\x10\x00" + b"\x00" * 4 + writer_entity + b"\x00" * 8
    return header + struct.pack("<BBH", 0x15, 0x01, len(body)) + body


def rtps_header(prefix: bytes) -> bytes:
    return b"RTPS" + b"\x02\x03" + b"\x01\x0f" + prefix


def rtps_data_submessage(
    writer_entity: bytes,
    *,
    submessage_id: int = 0x15,
    declared_body_length: int | None = None,
    captured_body_length: int | None = None,
) -> bytes:
    body = b"\x00\x00" + b"\x10\x00" + b"\x00" * 4 + writer_entity + b"\x00" * 32
    if captured_body_length is not None:
        body = body[:captured_body_length]
    if declared_body_length is None:
        declared_body_length = len(body)
    return struct.pack("<BBH", submessage_id, 0x01, declared_body_length) + body


def ethernet_ipv4_udp(source: str, destination: str, payload: bytes) -> bytes:
    source_bytes = socket.inet_aton(source)
    destination_bytes = socket.inet_aton(destination)
    udp = struct.pack(
        "!HHHH", 7400, 7412, 8 + len(payload), 0
    ) + payload
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
        source_bytes,
        destination_bytes,
    ) + udp
    return b"\x01\x00\x5e\x7f\x00\x01" + b"\x02\x00\x00\x00\x00\x01" + b"\x08\x00" + ip


def write_pcap(
    path: Path,
    frames: list[bytes],
    capture_lengths: list[int] | None = None,
) -> None:
    with path.open("wb") as stream:
        stream.write(struct.pack("<IHHIIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))
        for index, frame in enumerate(frames, 1):
            captured = (
                frame
                if capture_lengths is None
                else frame[: capture_lengths[index - 1]]
            )
            stream.write(
                struct.pack("<IIII", index, 0, len(captured), len(frame))
            )
            stream.write(captured)


def valid_approval_payload() -> dict:
    writer = {
        "topic": "/sportmodestate",
        "writer_gid": GID,
        "rtps_participant_guid_prefix": PREFIX.hex(),
        "source_ip": "192.168.123.161",
        "correlation_method": go2_clock_ref.EXPECTED_CORRELATION_METHOD,
        "correlation_conclusion": go2_clock_ref.EXPECTED_CORRELATION_CONCLUSION,
        "pcap_sha256": "b" * 64,
        "topic_info_file": "sport_primary.topic-info.txt",
        "topic_info_sha256": "c" * 64,
        "correlation_file": "sport_primary.correlation.json",
        "correlation_sha256": "a" * 64,
    }
    trial_prefixes = (
        PREFIX,
        bytes.fromhex("02108f1735154bce5e5fab49"),
        bytes.fromhex("03108f1735154bce5e5fab49"),
    )
    return {
        "schema_version": 3,
        "purpose": "go2-source-clock-to-chrony-refclock",
        "activation_authorized": False,
        "evidence_bundle": {
            "pcap_file": "go2-rtps.pcap",
            "pcap_sha256": "b" * 64,
        },
        "approved_writers": [writer],
        "pre_bootstrap_stability": {
            "minimum_duration_ns": evidence_bundle.MINIMUM_STABILITY_DURATION_NS,
            "observed_duration_ns": evidence_bundle.MINIMUM_STABILITY_DURATION_NS,
            "writer_gid": GID,
        },
        "cold_boot_identity_trials": [
            {
                "trial_id": f"cold-boot-{index + 1}",
                "boot_id": f"00000000-0000-4000-8000-{index + 1:012d}",
                "writer_gid": gid_for_prefix(prefix),
                "current_session": index == 0,
            }
            for index, prefix in enumerate(trial_prefixes)
        ],
        "post_bootstrap_quality_gate": dict(go2_clock_ref.POST_BOOTSTRAP_POLICY),
    }


class TimestampAccountingTest(unittest.TestCase):
    def test_stamp_validation_and_anomalies(self) -> None:
        self.assertIsNone(go2_time_core.stamp_to_nanoseconds(0, 0))
        self.assertIsNone(go2_time_core.stamp_to_nanoseconds(-1, 0))
        self.assertIsNone(go2_time_core.stamp_to_nanoseconds(1, 1_000_000_000))
        tracker = go2_time_core.StreamTracker("sport", "/sportmodestate")
        first = tracker.observe(100, 0, 101_000_000_000, 1_000_000_000)
        duplicate = tracker.observe(100, 0, 101_100_000_000, 1_100_000_000)
        regression = tracker.observe(99, 0, 101_200_000_000, 1_200_000_000)
        zero = tracker.observe(0, 0, 101_300_000_000, 1_300_000_000)
        self.assertEqual(first.status, "advancing")
        self.assertEqual(first.source_minus_receipt_ns, -1_000_000_000)
        self.assertEqual(duplicate.status, "duplicate")
        self.assertEqual(regression.status, "regression")
        self.assertEqual(zero.status, "zero")
        self.assertEqual(tracker.duplicates, 1)
        self.assertEqual(tracker.regressions, 1)
        self.assertEqual(tracker.zero, 1)

    def test_offset_drift_jitter_and_pairwise_summary(self) -> None:
        left = go2_time_core.StreamTracker("left", "/left")
        right = go2_time_core.StreamTracker("right", "/right")
        for index in range(5):
            monotonic = index * 1_000_000_000
            source = 100 + index
            left.observe(source, 0, source * 1_000_000_000 + index * 1_000, monotonic)
            right.observe(source, 0, source * 1_000_000_000 + 5_000, monotonic)
        summary = left.summary()
        self.assertAlmostEqual(summary["estimated_drift_ppm"], -1.0)
        self.assertIsNotNone(summary["offset_jitter_abs_deviation_p95_ns"])
        pairs = go2_time_core.pairwise_median_offsets([left, right])
        self.assertEqual(len(pairs), 1)
        self.assertFalse(pairs[0]["simultaneous_measurement"])

    def test_quality_window_is_truly_sliding(self) -> None:
        window = go2_clock_ref.QualityWindow(3)
        window.add(0, 0)
        window.add(1_000, 1_000_000_000)
        window.add(2_000, 2_000_000_000)
        window.add(2_000_000, 3_000_000_000)
        self.assertEqual(list(window.samples), [
            (1_000, 1_000_000_000),
            (2_000, 2_000_000_000),
            (2_000_000, 3_000_000_000),
        ])
        count, _, drift = window.metrics()
        self.assertEqual(count, 3)
        self.assertAlmostEqual(drift, (2_000_000 - 1_000) / 2_000_000_000 * 1_000_000)

    def test_forward_and_reverse_source_jumps_are_immediate(self) -> None:
        jump = go2_clock_ref.source_timing_jump_reason
        self.assertEqual(jump(-1, 10, 100), "source_timestamp_regression")
        self.assertEqual(
            jump(2_000_000_000, 100_000_000, 250_000_000),
            "source_receipt_delta_discontinuity",
        )
        self.assertIsNone(jump(100_000_000, 100_000_001, 250_000_000))

    def test_state_remains_bounded_for_two_hour_run(self) -> None:
        tracker = go2_time_core.StreamTracker(
            "sport", "/sportmodestate", retained_offset_limit=256
        )
        quality = go2_clock_ref.QualityWindow(256)
        # 50 Hz for 7200 seconds.  Counts grow, retained state does not.
        for index in range(50 * 7200 + 1):
            source_ns = 1_000_000_000 + index * 20_000_000
            observation = tracker.observe(
                source_ns // 1_000_000_000,
                source_ns % 1_000_000_000,
                source_ns + 1_000,
                index * 20_000_000,
            )
            quality.add(int(observation.source_minus_receipt_ns), index * 20_000_000)
        self.assertEqual(len(tracker._offsets_ns), 256)
        self.assertEqual(len(quality.samples), 256)
        self.assertEqual(tracker.received, 50 * 7200 + 1)


class RtpsLocatorTest(unittest.TestCase):
    def test_complete_data_submessage_exposes_writer_entity(self) -> None:
        payload = rtps_header(PREFIX) + rtps_data_submessage(ENTITY)
        self.assertEqual(locator._rtps_data_writer_ids(payload), [ENTITY])

    def test_snaplen_truncated_data_still_exposes_captured_writer(self) -> None:
        payload = rtps_header(PREFIX) + rtps_data_submessage(
            ENTITY,
            declared_body_length=512,
            captured_body_length=12,
        )
        self.assertEqual(locator._rtps_data_writer_ids(payload), [ENTITY])

    def test_snaplen_truncated_data_frag_exposes_captured_writer(self) -> None:
        payload = rtps_header(PREFIX) + rtps_data_submessage(
            ENTITY,
            submessage_id=0x16,
            declared_body_length=512,
            captured_body_length=12,
        )
        self.assertEqual(locator._rtps_data_writer_ids(payload), [ENTITY])

    def test_truncated_pcap_record_correlates_captured_writer(self) -> None:
        fixed_body = b"\x00\x00" + b"\x10\x00" + b"\x00" * 4 + ENTITY
        body = fixed_body + b"\x00" * (512 - len(fixed_body))
        payload = (
            rtps_header(PREFIX)
            + struct.pack("<BBH", 0x15, 0x01, len(body))
            + body
        )
        frame = ethernet_ipv4_udp("192.168.123.161", "192.168.123.99", payload)
        # Ethernet + IPv4 + UDP + RTPS header + submessage header + the fixed
        # twelve body bytes ending at writerEntityId.
        captured_length = 14 + 20 + 8 + 20 + 4 + 12
        with tempfile.TemporaryDirectory() as directory:
            pcap = Path(directory) / "snaplen.pcap"
            write_pcap(pcap, [frame], [captured_length])
            result = locator.correlate(pcap, GID)
        self.assertEqual(
            result["conclusion"], "single_source_proven_by_rtps_data_writer"
        )
        self.assertEqual(result["proven_source_ips"], ["192.168.123.161"])

    def test_declared_body_too_short_cannot_smuggle_writer_entity(self) -> None:
        payload = rtps_header(PREFIX) + rtps_data_submessage(
            ENTITY,
            declared_body_length=8,
            captured_body_length=12,
        )
        self.assertEqual(locator._rtps_data_writer_ids(payload), [])

    def test_multiple_submessages_reach_later_data_writer(self) -> None:
        # INFO_TS has an eight-byte body and is followed by a complete DATA.
        info_ts = struct.pack("<BBH", 0x09, 0x01, 8) + b"\x00" * 8
        payload = rtps_header(PREFIX) + info_ts + rtps_data_submessage(ENTITY)
        self.assertEqual(locator._rtps_data_writer_ids(payload), [ENTITY])

    def test_non_initial_ipv4_fragment_is_never_decoded_as_udp_rtps(self) -> None:
        frame = bytearray(
            ethernet_ipv4_udp(
                "192.168.123.161", "192.168.123.99", rtps_data(PREFIX, ENTITY)
            )
        )
        # IPv4 fragment offset one (eight bytes); payload happens to retain a
        # UDP-looking header but must never be treated as a UDP datagram.
        frame[14 + 6 : 14 + 8] = struct.pack("!H", 1)
        self.assertIsNone(locator._udp_datagram(bytes(frame[14:])))

    def test_same_session_writer_data_proves_source_ip(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            pcap = Path(directory) / "sample.pcap"
            frames = [
                ethernet_ipv4_udp(
                    "192.168.123.77", "239.255.0.1", rtps_data(PREFIX, b"\x00\x00\x0a\x04")
                ),
                ethernet_ipv4_udp(
                    "192.168.123.161", "192.168.123.99", rtps_data(PREFIX, ENTITY)
                ),
            ]
            write_pcap(pcap, frames)
            result = locator.correlate(pcap, GID)
        self.assertEqual(
            result["conclusion"], "single_source_proven_by_rtps_data_writer"
        )
        self.assertEqual(result["proven_source_ips"], ["192.168.123.161"])
        self.assertEqual(result["participant_prefix_sources"]["192.168.123.77"], 1)
        self.assertFalse(result["ros_topic_info_provides_ip"])

    def test_prefix_without_writer_entity_is_not_ip_proof(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            pcap = Path(directory) / "sample.pcap"
            write_pcap(
                pcap,
                [ethernet_ipv4_udp(
                    "192.168.123.77", "239.255.0.1", rtps_data(PREFIX, b"\x00\x00\x0a\x04")
                )],
            )
            result = locator.correlate(pcap, GID)
        self.assertEqual(
            result["conclusion"], "participant_prefix_seen_but_writer_data_not_proven"
        )
        self.assertEqual(result["proven_source_ips"], [])


class EvidenceBundleTest(unittest.TestCase):
    def make_writer_evidence(
        self, directory: Path, stem: str, prefix: bytes
    ) -> tuple[Path, dict]:
        gid = gid_for_prefix(prefix)
        pcap = directory / f"{stem}.pcap"
        write_pcap(
            pcap,
            [ethernet_ipv4_udp(
                "192.168.123.161", "192.168.123.99", rtps_data(prefix, ENTITY)
            )],
        )
        topic_info = directory / f"{stem}.topic-info.txt"
        topic_info.write_text(
            "Endpoint type: PUBLISHER\nGID: " + gid + "\n",
            encoding="utf-8",
        )
        correlation_path = directory / f"{stem}.correlation.json"
        correlation_path.write_text(
            json.dumps(locator.correlate(pcap, gid)), encoding="utf-8"
        )
        for path in (pcap, topic_info, correlation_path):
            path.chmod(0o600)
        writer = approval_tool.approved_writer(
            "/sportmodestate", topic_info, correlation_path, pcap
        )
        return pcap, writer

    def make_bundle(self, directory: Path) -> tuple[Path, dict]:
        pcap, writer = self.make_writer_evidence(directory, "go2-rtps", PREFIX)

        started_realtime_ns = 1_700_000_000_000_000_000
        stability_metadata = {
            "mode": "read-only-no-adjust",
            "clock_adjustment_requested": False,
            "ros_publishers_created": False,
            "unitree_clients_created": False,
            "duration_seconds": 7200,
            "started_realtime_ns": started_realtime_ns,
            "topics": dict(evidence_bundle.STABILITY_REQUIRED_STREAMS),
        }
        stability_streams = []
        for stream_name, topic in evidence_bundle.STABILITY_REQUIRED_STREAMS:
            stability_streams.append(
                {
                    "stream": stream_name,
                    "topic": topic,
                    "received": 10_000,
                    "valid": 10_000,
                    "duplicates": 0,
                    "zero": 0,
                    "malformed": 0,
                    "regressions": 0,
                    "offset_jitter_abs_deviation_p95_ns": 1_000_000,
                    "estimated_drift_ppm": 5.0,
                }
            )
        stability_summary = {
            "mode": "read-only-no-adjust",
            "elapsed_monotonic_ns": evidence_bundle.MINIMUM_STABILITY_DURATION_NS,
            "exit_reason": "duration_elapsed",
            "cleanup_errors": [],
            "started_realtime_ns": started_realtime_ns,
            "streams": stability_streams,
        }
        metadata_path = directory / "stability-metadata.json"
        summary_path = directory / "stability-summary.json"
        metadata_path.write_text(json.dumps(stability_metadata), encoding="utf-8")
        summary_path.write_text(json.dumps(stability_summary), encoding="utf-8")
        metadata_path.chmod(0o600)
        summary_path.chmod(0o600)
        stability = {
            "metadata_file": metadata_path.name,
            "metadata_sha256": evidence_bundle.sha256_file(metadata_path),
            "summary_file": summary_path.name,
            "summary_sha256": evidence_bundle.sha256_file(summary_path),
            "minimum_duration_ns": evidence_bundle.MINIMUM_STABILITY_DURATION_NS,
            "observed_duration_ns": evidence_bundle.MINIMUM_STABILITY_DURATION_NS,
            "writer_gid": writer["writer_gid"],
            "maximum_jitter_p95_ns": evidence_bundle.STABILITY_MAX_JITTER_P95_NS,
            "maximum_absolute_drift_ppm": (
                evidence_bundle.STABILITY_MAX_ABSOLUTE_DRIFT_PPM
            ),
            "maximum_duplicate_fraction": (
                evidence_bundle.STABILITY_MAX_DUPLICATE_FRACTION
            ),
        }

        trial_prefixes = (
            PREFIX,
            bytes.fromhex("02108f1735154bce5e5fab49"),
            bytes.fromhex("03108f1735154bce5e5fab49"),
        )
        trials = []
        for index, prefix in enumerate(trial_prefixes, 1):
            if index == 1:
                trial_pcap, trial_writer = pcap, writer
            else:
                trial_pcap, trial_writer = self.make_writer_evidence(
                    directory, f"cold-boot-{index}", prefix
                )
            boot_id = f"00000000-0000-4000-8000-{index:012d}"
            boot_id_path = directory / f"cold-boot-{index}.boot-id.txt"
            boot_id_path.write_text(boot_id + "\n", encoding="utf-8")
            boot_id_path.chmod(0o600)
            trials.append(
                {
                    "trial_id": f"cold-boot-{index}",
                    "operator_attestation": (
                        "physical-cold-boot-observed-and-read-only"
                    ),
                    "current_session": index == 1,
                    "boot_id_file": boot_id_path.name,
                    "boot_id_sha256": evidence_bundle.sha256_file(boot_id_path),
                    "boot_id": boot_id,
                    "pcap_file": trial_pcap.name,
                    **trial_writer,
                }
            )
        payload = {
            "schema_version": 3,
            "purpose": "go2-source-clock-to-chrony-refclock",
            "activation_authorized": False,
            "evidence_bundle": {
                "pcap_file": pcap.name,
                "pcap_sha256": evidence_bundle.sha256_file(pcap),
            },
            "approved_writers": [writer],
            "pre_bootstrap_stability": stability,
            "cold_boot_identity_trials": trials,
            "post_bootstrap_quality_gate": evidence_bundle.post_bootstrap_policy(),
        }
        approval = directory / "go2-clock-ref-approval.json"
        approval.write_text(json.dumps(payload), encoding="utf-8")
        approval.chmod(0o600)
        return approval, payload

    def test_raw_bundle_is_recomputed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            self.make_bundle(bundle)
            verified = evidence_bundle.verify_bundle(bundle)
            self.assertEqual(
                verified["approved_writers"][0]["source_ip"], "192.168.123.161"
            )

    def test_preparation_cli_emits_a_verified_schema_v3_bundle(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            approval, payload = self.make_bundle(bundle)
            approval.unlink()
            writer = payload["approved_writers"][0]
            arguments = [
                "--pcap",
                str(bundle / payload["evidence_bundle"]["pcap_file"]),
                "--topic-info",
                f'/sportmodestate={bundle / writer["topic_info_file"]}',
                "--topic-correlation",
                f'/sportmodestate={bundle / writer["correlation_file"]}',
                "--stability-metadata",
                str(bundle / payload["pre_bootstrap_stability"]["metadata_file"]),
                "--stability-summary",
                str(bundle / payload["pre_bootstrap_stability"]["summary_file"]),
            ]
            for trial in payload["cold_boot_identity_trials"]:
                trial_value = ",".join(
                    (
                        trial["trial_id"],
                        "true" if trial["current_session"] else "false",
                        trial["operator_attestation"],
                        str(bundle / trial["boot_id_file"]),
                        str(bundle / trial["pcap_file"]),
                        str(bundle / trial["topic_info_file"]),
                        str(bundle / trial["correlation_file"]),
                    )
                )
                arguments.extend(("--cold-boot-trial", trial_value))
            arguments.extend(("--output", str(approval)))

            self.assertEqual(approval_tool.main(arguments), 0)
            self.assertEqual(
                evidence_bundle.verify_bundle(bundle)["schema_version"], 3
            )

    def test_fake_digest_and_tampered_pcap_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            approval, payload = self.make_bundle(bundle)
            payload["evidence_bundle"]["pcap_sha256"] = "0" * 64
            approval.write_text(json.dumps(payload), encoding="utf-8")
            approval.chmod(0o600)
            with self.assertRaisesRegex(ValueError, "PCAP SHA-256 mismatch"):
                evidence_bundle.verify_bundle(bundle)

    def test_forged_source_and_self_consistent_digest_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            approval, payload = self.make_bundle(bundle)
            correlation_path = bundle / "go2-rtps.correlation.json"
            correlation = json.loads(correlation_path.read_text(encoding="utf-8"))
            correlation["proven_source_ips"] = ["192.168.123.77"]
            correlation["writer_data_sources"] = {"192.168.123.77": 1}
            correlation_path.write_text(json.dumps(correlation), encoding="utf-8")
            correlation_path.chmod(0o600)
            payload["approved_writers"][0]["source_ip"] = "192.168.123.77"
            payload["approved_writers"][0]["correlation_sha256"] = (
                evidence_bundle.sha256_file(correlation_path)
            )
            approval.write_text(json.dumps(payload), encoding="utf-8")
            approval.chmod(0o600)
            with self.assertRaisesRegex(ValueError, "not reproducible"):
                evidence_bundle.verify_bundle(bundle)

    def test_missing_original_correlation_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            self.make_bundle(bundle)
            (bundle / "go2-rtps.correlation.json").unlink()
            with self.assertRaises(OSError):
                evidence_bundle.verify_bundle(bundle)

    def test_schema_v2_and_incomplete_evidence_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            approval, payload = self.make_bundle(bundle)
            payload["schema_version"] = 2
            approval.write_text(json.dumps(payload), encoding="utf-8")
            approval.chmod(0o600)
            with self.assertRaisesRegex(ValueError, "schema_version=3 required"):
                evidence_bundle.verify_bundle(bundle)

    def test_short_stability_and_two_cold_boot_trials_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            approval, payload = self.make_bundle(bundle)
            summary_path = bundle / "stability-summary.json"
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            summary["elapsed_monotonic_ns"] -= 1
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            summary_path.chmod(0o600)
            payload["pre_bootstrap_stability"]["summary_sha256"] = (
                evidence_bundle.sha256_file(summary_path)
            )
            payload["pre_bootstrap_stability"]["observed_duration_ns"] -= 1
            approval.write_text(json.dumps(payload), encoding="utf-8")
            approval.chmod(0o600)
            with self.assertRaisesRegex(ValueError, "shorter than two hours"):
                evidence_bundle.verify_bundle(bundle)

        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            approval, payload = self.make_bundle(bundle)
            payload["cold_boot_identity_trials"] = payload[
                "cold_boot_identity_trials"
            ][:2]
            approval.write_text(json.dumps(payload), encoding="utf-8")
            approval.chmod(0o600)
            with self.assertRaisesRegex(ValueError, "at least three"):
                evidence_bundle.verify_bundle(bundle)

    def test_post_bootstrap_policy_cannot_be_weakened(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            approval, payload = self.make_bundle(bundle)
            payload["post_bootstrap_quality_gate"][
                "maximum_absolute_offset_ns_exclusive"
            ] += 1
            approval.write_text(json.dumps(payload), encoding="utf-8")
            approval.chmod(0o600)
            with self.assertRaisesRegex(ValueError, "exactly match"):
                evidence_bundle.verify_bundle(bundle)


class RefclockGateTest(unittest.TestCase):
    def test_approval_and_graph_identity_gate(self) -> None:
        approvals = go2_clock_ref.validate_approval_payload(valid_approval_payload())

        class Endpoint:
            endpoint_gid = bytes.fromhex(GID.replace(".", ""))

        authorized, reasons = go2_clock_ref.authorized_graph_topics(
            lambda topic: [Endpoint()], approvals
        )
        self.assertEqual(authorized, {"/sportmodestate"})
        self.assertEqual(
            reasons["/sportmodestate"],
            "live_gid_matches_raw_pcap_source=192.168.123.161",
        )

        class RestartedEndpoint:
            endpoint_gid = b"\xff" * 24

        authorized, reasons = go2_clock_ref.authorized_graph_topics(
            lambda topic: [RestartedEndpoint()], approvals
        )
        self.assertEqual(authorized, set())
        self.assertEqual(
            reasons["/sportmodestate"],
            "current_writer_gid_does_not_match_approval",
        )

    def test_legacy_self_asserted_approval_is_rejected(self) -> None:
        payload = valid_approval_payload()
        payload["schema_version"] = 2
        with self.assertRaises(ValueError):
            go2_clock_ref.validate_approval_payload(payload)

    def test_approval_requires_all_formal_time_gates(self) -> None:
        payload = valid_approval_payload()
        payload["cold_boot_identity_trials"] = payload[
            "cold_boot_identity_trials"
        ][:2]
        with self.assertRaisesRegex(ValueError, "three verified cold-boot"):
            go2_clock_ref.validate_approval_payload(payload)

        payload = valid_approval_payload()
        payload["pre_bootstrap_stability"]["observed_duration_ns"] -= 1
        with self.assertRaisesRegex(ValueError, "two-hour stability"):
            go2_clock_ref.validate_approval_payload(payload)

        payload = valid_approval_payload()
        payload["post_bootstrap_quality_gate"][
            "maximum_absolute_drift_ppm_exclusive"
        ] = 100.1
        with self.assertRaisesRegex(ValueError, "policy is not exact"):
            go2_clock_ref.validate_approval_payload(payload)

    def test_approval_rejects_nx_as_publisher_source(self) -> None:
        payload = valid_approval_payload()
        payload["approved_writers"][0]["source_ip"] = "192.168.123.18"
        with self.assertRaises(ValueError):
            go2_clock_ref.validate_approval_payload(payload)

    def test_exact_enable_token_and_owner(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            gate = Path(directory) / "gate"
            gate.write_text(go2_clock_ref.ENABLE_TOKEN, encoding="utf-8")
            gate.chmod(0o600)
            go2_clock_ref.validate_enable_file(gate, os.geteuid())
            gate.write_text("YES\n", encoding="utf-8")
            with self.assertRaises(PermissionError):
                go2_clock_ref.validate_enable_file(gate, os.geteuid())

    def test_chrony_sock_sample_abi_and_offset_direction(self) -> None:
        payload = go2_clock_ref.ChronySocketWriter.encode_sample(
            102_250_000_500, 100_125_000_900
        )
        self.assertEqual(len(payload), ctypes.sizeof(go2_clock_ref.ChronySockSample))
        sample = go2_clock_ref.ChronySockSample.from_buffer_copy(payload)
        self.assertEqual(sample.magic, go2_clock_ref.SOCK_MAGIC)
        self.assertEqual(sample.pulse, 0)
        self.assertEqual(sample.leap, 0)
        self.assertAlmostEqual(sample.offset, 2.1250005, places=7)

    def test_default_mode_is_bounded_observe_only(self) -> None:
        arguments = go2_clock_ref.build_argument_parser().parse_args([])
        self.assertEqual(arguments.mode, "observe-only")
        self.assertGreater(arguments.duration_seconds, 0)


class PostBootstrapQualityGateTest(unittest.TestCase):
    @staticmethod
    def health(now_ns: int, feed_count: int, offset_ns: int) -> dict:
        return {
            "unsafe_latched": False,
            "required_topics": ["/sportmodestate"],
            "authorized_topics": ["/sportmodestate"],
            "updated_monotonic_ns": now_ns,
            "last_valid_sample_monotonic_ns": now_ns,
            "feed_count": feed_count,
            "stream_quality": {
                "/sportmodestate": {
                    "last_source_minus_local_ns": offset_ns,
                }
            },
        }

    def test_continuous_window_passes_only_after_duration(self) -> None:
        policy = dict(post_bootstrap_quality_gate.EXPECTED_POLICY)
        policy["minimum_continuous_duration_ns"] = 3_000_000_000
        gate = post_bootstrap_quality_gate.QualityGate(policy)
        states = []
        for index in range(1, 5):
            now_ns = index * 1_000_000_000
            states.append(
                gate.observe(self.health(now_ns, index, index * 10), now_ns)
            )
        self.assertEqual(states, ["qualifying", "qualifying", "qualifying", "passed"])
        self.assertEqual(gate.passed_metrics["continuous_duration_ns"], 3_000_000_000)
        self.assertLess(abs(gate.passed_metrics["robust_drift_ppm"]), 100.0)

    def test_offset_limit_is_strict_and_resets_window(self) -> None:
        policy = dict(post_bootstrap_quality_gate.EXPECTED_POLICY)
        gate = post_bootstrap_quality_gate.QualityGate(policy)
        now_ns = 1_000_000_000
        state = gate.observe(
            self.health(now_ns, 1, policy["maximum_absolute_offset_ns_exclusive"]),
            now_ns,
        )
        self.assertEqual(state, "qualifying")
        self.assertEqual(gate.last_failure, "offset_out_of_bounds")
        self.assertEqual(list(gate.samples), [])

    def test_approval_policy_is_exact(self) -> None:
        payload = valid_approval_payload()
        self.assertEqual(
            post_bootstrap_quality_gate.policy_from_approval(payload),
            post_bootstrap_quality_gate.EXPECTED_POLICY,
        )
        payload["post_bootstrap_quality_gate"][
            "minimum_continuous_duration_ns"
        ] -= 1
        with self.assertRaisesRegex(ValueError, "policy is not exact"):
            post_bootstrap_quality_gate.policy_from_approval(payload)


class StaticSafetyTest(unittest.TestCase):
    def test_subscription_tools_have_no_publish_or_clock_setter(self) -> None:
        paths = [
            TIME_DIR / "go2_clock_ref.py",
            ROOT / "scripts" / "probe_go2_time_readonly.py",
        ]
        forbidden = (
            "create_publisher",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "SportClient",
            "clock_settime",
            "settimeofday",
            "subprocess",
        )
        for path in paths:
            source = path.read_text(encoding="utf-8")
            for token in forbidden:
                self.assertNotIn(token, source, f"{token} in {path}")

        refclock_source = paths[0].read_text(encoding="utf-8")
        self.assertIn("create_subscription", refclock_source)
        probe_source = paths[1].read_text(encoding="utf-8")
        self.assertIn("_rclpy.Node(", probe_source)
        self.assertIn("_rclpy.Subscription(", probe_source)
        self.assertIn(
            "_rclpy.WaitSet(5, 0, 0, 0, 0, 0, context.handle)",
            probe_source,
        )
        self.assertIn("initialize_logging=False", probe_source)
        self.assertIn('"ros_publishers_created": False', probe_source)
        for forbidden_high_level in (
            "from rclpy.node import Node",
            ".create_subscription(",
            "rclpy.spin_once(",
            "ReadOnlyTimeProbe(",
        ):
            self.assertNotIn(forbidden_high_level, probe_source)

        wrapper_source = (ROOT / "scripts" / "probe_go2_time_readonly.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'MAX_SAMPLES="${GO2_TIME_PROBE_MAX_SAMPLES:-500000}"',
            wrapper_source,
        )

    def test_locator_echoes_are_bounded_and_never_publish(self) -> None:
        source = (ROOT / "scripts" / "collect_go2_publisher_locators_readonly.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("timeout --signal=INT", source)
        self.assertIn("ros2 topic echo", source)
        self.assertIn("ros2 topic echo \"${topic}\" --no-daemon", source)
        self.assertIn("ros2 topic info --no-daemon --verbose", source)
        self.assertNotIn("ros2 topic pub", source)
        self.assertIn("ros_topic_info_provides_ip=false", source)

    def test_locator_sudo_mode_privileges_only_tcpdump_and_closes_startup_race(self) -> None:
        source = (
            ROOT / "scripts" / "collect_go2_publisher_locators_readonly.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("SUDO_NONINTERACTIVE", source)
        self.assertIn("PKEXEC", source)
        self.assertIn("tcpdump_command=(sudo -n -- tcpdump)", source)
        self.assertIn("if (( EUID == 0 )); then", source)
        self.assertIn('capture_ready=false', source)
        self.assertLess(
            source.index('if [[ "${capture_ready}" != true ]]'),
            source.index('ros2 topic echo "${topic}" --no-daemon'),
        )
        self.assertNotIn("sudo -S", source)

    def test_sidecar_runtime_is_minimal_and_probe_has_no_sys_time(self) -> None:
        run = (ROOT / "deploy" / "jetson-time-sync" / "run.sh").read_text(
            encoding="utf-8"
        )
        for required in (
            "--network host",
            "--network none",
            "--read-only",
            "--cap-drop ALL",
            "--cap-add SYS_TIME",
            "GO2_TIME_ALL_ROS_CONTAINERS_STOPPED",
            "--entrypoint /usr/sbin/chronyd",
            "GO2_TIME_ROLE=feeder",
            "type=volume,source=${socket_volume}",
            "--bundle",
        ):
            self.assertIn(required, run)
        for forbidden in (
            "--privileged",
            "--device ",
            "type=bind",
            "--volume ",
            "docker.sock",
            "--pid host",
            "--ipc host",
        ):
            self.assertNotIn(forbidden, run)
        probe_block, formal_block = run.split(
            'if [[ "${GO2_TIME_ALLOW_SYS_TIME:-}"', 1
        )
        self.assertNotIn("--cap-add SYS_TIME", probe_block)
        self.assertIn("--cap-add SYS_TIME", formal_block)

    def test_capability_checks_cover_all_kernel_sets(self) -> None:
        entrypoint = (
            ROOT / "deploy" / "jetson-time-sync" / "entrypoint.sh"
        ).read_text(encoding="utf-8")
        for name in ("CapEff", "CapPrm", "CapInh", "CapAmb", "CapBnd"):
            self.assertIn(name, entrypoint)
        self.assertIn("assert_zero_capabilities", entrypoint)
        run = (ROOT / "deploy" / "jetson-time-sync" / "run.sh").read_text()
        self.assertIn("--entrypoint /usr/sbin/chronyd", run)
        self.assertIn("--no-healthcheck", run)
        self.assertIn("--user 10002:10002", run)
        self.assertIn('"${image}" -U -F 2', run)
        self.assertIn("chronyd_caps_exact", run)
        self.assertIn("chronyd_single_process", run)
        self.assertIn("chronyd_selected_go2", run)
        self.assertNotIn("GO2_TIME_ROLE=chronyd", entrypoint)

    def test_feeder_cannot_reach_chronyd_command_socket(self) -> None:
        deploy = ROOT / "deploy" / "jetson-time-sync"
        dockerfile = (deploy / "Dockerfile").read_text(encoding="utf-8")
        run = (deploy / "run.sh").read_text(encoding="utf-8")
        healthcheck = (deploy / "healthcheck.sh").read_text(encoding="utf-8")
        status = (deploy / "status.sh").read_text(encoding="utf-8")
        self.assertIn("install -d -o 10002 -g 10001 -m 2770", dockerfile)
        self.assertIn("install -d -o 10002 -g 10002 -m 0700", dockerfile)
        self.assertIn("/run/robonix-go2-time/refclock", dockerfile)
        self.assertIn("/run/robonix-go2-time/control", dockerfile)
        self.assertIn("/run/robonix-go2-time/state", dockerfile)
        self.assertIn("--user 10002:10002", run)
        self.assertIn('"${image}" -U -F 2', run)
        self.assertNotIn("chronyc", healthcheck)
        self.assertNotIn("chronyc", status)
        self.assertNotIn("/control/", healthcheck)
        self.assertNotIn("/control/", status)
        for name in ("chrony-bootstrap.conf", "chrony-steady.conf"):
            config = (deploy / "config" / name).read_text(encoding="utf-8")
            self.assertIn(
                "refclock SOCK /run/robonix-go2-time/refclock/go2.sock", config
            )
            self.assertIn(
                "bindcmdaddress /run/robonix-go2-time/control/chronyd.sock", config
            )

    def test_competing_servos_and_readiness_are_enforced(self) -> None:
        run = (ROOT / "deploy" / "jetson-time-sync" / "run.sh").read_text()
        for token in (
            "systemd-timesyncd.service",
            "chronyd|ntpd|ntp|openntpd|systemd-timesyn|ptp4l|phc2sys|timemaster",
            "competing_clock_processes",
            "competing_clock_services",
            "competing_sys_time_containers",
            '"healthy"',
        ):
            self.assertIn(token, run)

    def test_chrony_and_systemd_templates_are_inert(self) -> None:
        deploy = ROOT / "deploy" / "time-sync"
        bootstrap = (deploy / "chrony" / "go2-refclock-bootstrap.conf.template").read_text()
        steady = (deploy / "chrony" / "go2-refclock-steady.conf.template").read_text()
        self.assertIn("makestep", bootstrap)
        self.assertNotIn("makestep", steady)
        for content in (bootstrap, steady):
            self.assertNotRegex(content, r"(?m)^\s*(server|pool|peer|local)\s")
        for service in (deploy / "systemd").glob("*.service.template"):
            content = service.read_text(encoding="utf-8")
            self.assertNotIn("[Install]", content)
            self.assertIn("ProtectClock=yes", content)


if __name__ == "__main__":
    unittest.main()
