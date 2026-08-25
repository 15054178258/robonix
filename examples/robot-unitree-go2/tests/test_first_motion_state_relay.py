from __future__ import annotations

import copy
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC = ROOT / "deploy/time-sync"
sys.path.insert(0, str(TIME_SYNC))
MODULE_PATH = TIME_SYNC / "workstation_motion_state_relay.py"
SPEC = importlib.util.spec_from_file_location("motion_state_relay", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
relay = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = relay
SPEC.loader.exec_module(relay)

import workstation_nomotion_stamp_node as stamp_node  # noqa: E402
from navigation_stamp_discipline import AffineClockModel  # noqa: E402


ANCHOR_SOURCE_NS = 1_700_000_000_000_000_000
APPROVED_OFFSET_NS = 123_456_789
DRIFT_PPM = 13.0


class FirstMotionStateRelayPolicyTest(unittest.TestCase):
    def approval(self):
        now_ns = time.time_ns()
        return relay.FixedOffsetApproval(
            schema="robonix-go2-workstation-nomotion-stamp-offset-v3",
            session_id="session-0123456789abcdef",
            expected_clock_domain="unitree-main-computer@192.168.123.161",
            writer_gids=tuple(
                (name, f"{index:02x}" * 24)
                for index, name in enumerate(stamp_node.RAW_TOPICS, start=1)
            ),
            writer_source_ipv4="192.168.123.161",
            offset_evidence_sha256="0" * 64,
            fixed_local_minus_source_offset_ns=APPROVED_OFFSET_NS,
            approved_affine_common_drift_ppm=DRIFT_PPM,
            affine_window_common_drifts_ppm=(12.0, 14.0),
            not_before_unix_ns=now_ns - 1_000_000_000,
            expires_unix_ns=now_ns + 600_000_000_000,
        )

    def model(self) -> AffineClockModel:
        return AffineClockModel(
            anchor_source_ns=ANCHOR_SOURCE_NS,
            anchor_local_ns=ANCHOR_SOURCE_NS + APPROVED_OFFSET_NS,
            drift_ppm=DRIFT_PPM,
            source_to_local_scale=1.0 / (1.0 - DRIFT_PPM * 1.0e-6),
            core_stream_drifts_ppm=tuple(
                (name, DRIFT_PPM) for name in relay.AFFINE_CORE_STREAMS
            ),
            stream_baseline_corrected_age_ns=(
                ("sport_primary", 5_000_000),
                ("mid360_imu", 5_000_000),
                ("mid360_odom", 7_000_000),
            ),
        )

    def ready(self) -> dict[str, object]:
        model = self.model()
        commit_monotonic_ns = 10_100_000_000
        commit_realtime_ns = 1_700_000_010_100_000_000
        previous_clock_base_ns = (
            commit_realtime_ns - commit_monotonic_ns - 1_000
        )
        return {
            "schema": relay.AFFINE_READY_SCHEMA,
            "session_id": "session-0123456789abcdef",
            "correction_mode": "affine",
            "discipline_profile": "motion",
            "corrected_topics": dict(relay.MOTION_CORRECTED_TOPICS),
            "time_discipline_ready": True,
            "motion_ready": False,
            "canonical_odom_ready": False,
            "lidar_odom_semantics": "private_mapping_input_not_chassis_odom",
            "timestamp_safety_limits": {
                "offset_guard_ns": 5_000_000,
                "affine_anchor_past_guard_ns": 20_000_000,
                "max_corrected_future_ns": 2_000_000,
                "minimum_locked_corrected_age_ns": None,
                "max_pairwise_drift_ppm": 25.0,
                "max_approved_affine_drift_deviation_ppm": 5.0,
                "affine_qualification_window_ns": 10_000_000_000,
                "max_affine_window_common_drift_deviation_ppm": 15.0,
                "max_locked_affine_drift_deviation_ppm": 25.0,
            },
            "approval_reference_offset_ns": APPROVED_OFFSET_NS,
            "approval_affine_common_drift_ppm": DRIFT_PPM,
            "noncore_delta_drop_count": 0,
            "last_noncore_delta_drop": None,
            "affine_model": {
                "anchor_source_ns": model.anchor_source_ns,
                "anchor_local_ns": model.anchor_local_ns,
                "drift_ppm": model.drift_ppm,
                "source_to_local_scale": model.source_to_local_scale,
                "core_stream_drifts_ppm": dict(model.core_stream_drifts_ppm),
                "stream_baseline_corrected_age_ns": dict(
                    model.stream_baseline_corrected_age_ns
                ),
                "frozen": True,
            },
            "post_evaluation_commit": {
                "snapshot_evaluation_monotonic_ns": 10_000_000_000,
                "commit_check_realtime_ns": commit_realtime_ns,
                "commit_check_monotonic_ns": commit_monotonic_ns,
                "evaluation_to_commit_check_ns": 100_000_000,
                "clock_read_span_ns": 10_000,
                "clock_read_span_limit_ns": 1_000_000,
                "previous_clock_base_ns": previous_clock_base_ns,
                "clock_base_ns": (
                    commit_realtime_ns - commit_monotonic_ns
                ),
                "clock_base_delta_ns": 1_000,
                "clock_pair_discontinuity_limit_ns": 20_000_000,
                "stream_receipt_liveness": {
                    name: {
                        "last_advancing_receipt_monotonic_ns": (
                            commit_monotonic_ns - 10_000_000
                        ),
                        "receipt_age_ns": 10_000_000,
                        "stale_receipt_timeout_ns": 200_000_000,
                        "live": True,
                    }
                    for name in relay.MOTION_CORRECTED_TOPICS
                },
            },
        }

    def approval_payload(self) -> dict[str, object]:
        approval = self.approval()
        assert approval.affine_window_common_drifts_ppm is not None
        first, second = approval.affine_window_common_drifts_ppm
        return {
            "schema": approval.schema,
            "session_id": approval.session_id,
            "motion_enabled": False,
            "identity_evidence_verified": True,
            "expected_clock_domain": approval.expected_clock_domain,
            "writer_gids": dict(approval.writer_gids),
            "writer_source_ipv4": approval.writer_source_ipv4,
            "offset_evidence_sha256": approval.offset_evidence_sha256,
            "fixed_local_minus_source_offset_ns": (
                approval.fixed_local_minus_source_offset_ns
            ),
            "affine_drift_algorithm": (
                "one-second-lower-envelope-theil-sen-v1"
            ),
            "affine_drift_window_ns": 30_000_000_000,
            "affine_window_common_drifts_ppm": {
                "first": first,
                "second": second,
            },
            "approved_affine_common_drift_ppm": (
                approval.approved_affine_common_drift_ppm
            ),
            "affine_window_common_drift_deviation_ppm": abs(first - second),
            "not_before_unix_ns": approval.not_before_unix_ns,
            "expires_unix_ns": approval.expires_unix_ns,
            "operator_ack": (
                "I_APPROVE_THIS_FIXED_OFFSET_FOR_WORKSTATION_NOMOTION"
            ),
        }

    @staticmethod
    def write_private_json(path: Path, payload: dict[str, object]) -> None:
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_text(json.dumps(payload), encoding="utf-8")
        temporary.chmod(0o600)
        temporary.replace(path)

    @staticmethod
    def write_fake_worker(path: Path, event: str) -> None:
        path.write_text(
            "#!/usr/bin/env python3\n"
            "import os, sys, time\n"
            "args = dict(zip(sys.argv[1::2], sys.argv[2::2]))\n"
            "heartbeat = int(args['--heartbeat-fd'])\n"
            "events = int(args['--event-fd'])\n"
            "sent = False\n"
            "while True:\n"
            "    try:\n"
            "        data = os.read(heartbeat, 256)\n"
            "    except BlockingIOError:\n"
            "        time.sleep(0.005)\n"
            "        continue\n"
            "    if not data:\n"
            "        break\n"
            "    if not sent:\n"
            f"        os.write(events, {event.encode('ascii')!r})\n"
            "        sent = True\n"
            "        if "
            f"{event.startswith('FAULT_V3')!r}:\n"
            "            break\n",
            encoding="utf-8",
        )
        path.chmod(0o700)

    def test_strict_affine_motion_ready_is_normalized(self) -> None:
        contract = relay.validate_stamp_ready(self.ready(), self.approval())
        self.assertEqual(contract.session_id, self.approval().session_id)
        self.assertEqual(contract.drift_ppm, DRIFT_PPM)
        self.assertEqual(
            contract.corrected_stamp_ns(ANCHOR_SOURCE_NS),
            ANCHOR_SOURCE_NS + APPROVED_OFFSET_NS,
        )

    def test_schema_v2_with_finite_drift_is_rejected_at_direct_entries(self) -> None:
        legacy = replace(
            self.approval(),
            schema="robonix-go2-workstation-nomotion-stamp-offset-v2",
        )
        with self.assertRaisesRegex(relay.ApprovalError, "schema-v3"):
            relay.validate_stamp_ready(self.ready(), legacy)
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with self.assertRaisesRegex(relay.ApprovalError, "schema-v3"):
                stamp_node._Runtime(
                    legacy,
                    directory / "ready.json",
                    directory / "fault.json",
                    mode="affine",
                    profile="motion",
                )
            with mock.patch.object(relay.subprocess, "Popen") as popen:
                with self.assertRaisesRegex(relay.ApprovalError, "schema-v3"):
                    relay.run_supervisor(
                        directory / "approval.json",
                        legacy,
                        directory / "stamp-ready.json",
                        directory / "ready.json",
                        directory / "fault.json",
                        worker_binary=directory / "worker",
                    )
                popen.assert_not_called()

    def test_weakened_or_mutated_ready_contract_is_rejected(self) -> None:
        mutations = (
            ("wrong session", lambda value: value.update(session_id="different")),
            (
                "wrong approval offset",
                lambda value: value.update(approval_reference_offset_ns=1),
            ),
            (
                "wrong approval drift",
                lambda value: value.update(
                    approval_affine_common_drift_ppm=DRIFT_PPM + 1.0
                ),
            ),
            (
                "not ready",
                lambda value: value.update(time_discipline_ready=False),
            ),
            ("claims motion", lambda value: value.update(motion_ready=True)),
            (
                "claims odom",
                lambda value: value.update(canonical_odom_ready=True),
            ),
            (
                "no-motion profile",
                lambda value: value.update(discipline_profile="nomotion"),
            ),
            (
                "motion non-core drop count",
                lambda value: value.update(noncore_delta_drop_count=1),
            ),
            (
                "motion non-core drop bool",
                lambda value: value.update(noncore_delta_drop_count=False),
            ),
            (
                "motion non-core drop evidence",
                lambda value: value.update(
                    last_noncore_delta_drop={"stream": "mid360_cloud"}
                ),
            ),
            (
                "legacy fixed schema",
                lambda value: value.update(schema=stamp_node.READY_SCHEMA),
            ),
            (
                "unknown field",
                lambda value: value.update(unreviewed_relaxation=True),
            ),
            (
                "cloud endpoint injected",
                lambda value: value["corrected_topics"].update(
                    mid360_cloud=relay.CORRECTED_TOPICS["mid360_cloud"]
                ),
            ),
            (
                "no-motion timestamp soft floor injected",
                lambda value: value["timestamp_safety_limits"].update(
                    minimum_locked_corrected_age_ns=5_000_000
                ),
            ),
            (
                "timestamp anchor guard narrowed",
                lambda value: value["timestamp_safety_limits"].update(
                    affine_anchor_past_guard_ns=5_000_000
                ),
            ),
            (
                "unknown timestamp limit",
                lambda value: value["timestamp_safety_limits"].update(
                    unreviewed_limit=1
                ),
            ),
            (
                "unfrozen model",
                lambda value: value["affine_model"].update(frozen=False),
            ),
            (
                "wrong scale",
                lambda value: value["affine_model"].update(
                    source_to_local_scale=1.0
                ),
            ),
            (
                "stale IMU baseline",
                lambda value: value["affine_model"][
                    "stream_baseline_corrected_age_ns"
                ].update(mid360_imu=200_000_001),
            ),
            (
                "pairwise drift disagreement",
                lambda value: value["affine_model"].update(
                    drift_ppm=0.0,
                    source_to_local_scale=1.0,
                    core_stream_drifts_ppm={
                        "sport_primary": -25.0,
                        "mid360_imu": 0.0,
                        "mid360_odom": 25.0,
                    },
                ),
            ),
        )
        for name, mutate in mutations:
            with self.subTest(name=name):
                changed = copy.deepcopy(self.ready())
                mutate(changed)
                with self.assertRaises(relay.RelayError):
                    relay.validate_stamp_ready(changed, self.approval())

    def test_affine_motion_stamp_ready_flows_into_relay_validator(self) -> None:
        approval = self.approval()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runtime = stamp_node._Runtime(
                approval,
                directory / "ready.json",
                directory / "fault.json",
                mode="affine",
                profile="motion",
            )
            expected_ready = self.ready()
            commit = expected_ready["post_evaluation_commit"]
            runtime.discipline.approved_affine_anchor_deadline_monotonic_ns = (
                mock.Mock(
                    return_value=commit[
                        "snapshot_evaluation_monotonic_ns"
                    ]
                )
            )
            runtime.discipline._last_clock_base_ns = commit[
                "previous_clock_base_ns"
            ]
            runtime.discipline.receipt_liveness_snapshot = mock.Mock(
                return_value=commit["stream_receipt_liveness"]
            )
            runtime.discipline.lock_affine_from_approved_drift = mock.Mock(
                return_value=self.model()
            )
            with mock.patch.object(
                stamp_node,
                "fresh_paired_receipt_clocks",
                return_value=stamp_node.ReceiptClockPair(
                    commit["commit_check_realtime_ns"],
                    commit["commit_check_monotonic_ns"],
                    commit["clock_read_span_ns"],
                    0,
                ),
            ), mock.patch.object(
                stamp_node, "require_strict_affine_approval"
            ):
                runtime.maybe_lock(commit["commit_check_monotonic_ns"])

            payload = json.loads(runtime.ready_file.read_text(encoding="utf-8"))
            contract = relay.validate_stamp_ready(payload, approval)
            self.assertEqual(payload["discipline_profile"], "motion")
            self.assertEqual(contract.drift_ppm, DRIFT_PPM)

        launcher = (ROOT / "scripts/start_first_motion_corrected.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--mode affine", launcher)
        self.assertIn("--profile motion", launcher)

    def test_corrected_state_must_advance_and_remain_strictly_fresh(self) -> None:
        now = 10_000_000_000
        self.assertEqual(
            relay.corrected_state_issue(
                now_realtime_ns=now,
                now_monotonic_ns=20_000_000_000,
                corrected_stamp_ns=now - 200_000_000,
                last_corrected_stamp_ns=now - 210_000_000,
                last_receipt_monotonic_ns=19_990_000_000,
            ),
            "",
        )
        cases = (
            (now - 20_000_000, now - 20_000_000, "did not advance"),
            (now - 200_000_001, now - 300_000_000, "stale"),
            (now + 6_000_000, now - 20_000_000, "future"),
        )
        for stamp, previous, fragment in cases:
            with self.subTest(fragment=fragment):
                issue = relay.corrected_state_issue(
                    now_realtime_ns=now,
                    now_monotonic_ns=20_000_000_000,
                    corrected_stamp_ns=stamp,
                    last_corrected_stamp_ns=previous,
                    last_receipt_monotonic_ns=19_990_000_000,
                )
                self.assertIn(fragment, issue)

    def test_cpp_worker_event_protocol_is_exact_and_fail_closed(self) -> None:
        graph_gid = "ab" * 16 + "00" * 8
        message_gid = "bc" * 8 + "00" * 16
        changed_message_gid = "cd" * 8 + "00" * 16
        rmw_identifier = "rmw_cyclonedds_cpp"
        ready = relay.parse_worker_event(
            f"READY_V3\t{graph_gid}\t{message_gid}\t"
            f"{rmw_identifier}\t30\t1\t3\t3"
        )
        self.assertEqual(ready.graph_publisher_gid, graph_gid)
        self.assertEqual(ready.message_publisher_gid, message_gid)
        self.assertEqual(ready.rmw_implementation_identifier, rmw_identifier)
        self.assertEqual(ready.samples, 30)
        fault = relay.parse_worker_event(
            f"FAULT_V3\tmessage_publisher_gid_changed\t{graph_gid}\t"
            f"{message_gid}\t{changed_message_gid}\tmessage\t1\t"
            f"{rmw_identifier}"
        )
        self.assertEqual(fault.graph_publisher_gid, graph_gid)
        self.assertEqual(fault.message_publisher_gid, message_gid)
        self.assertEqual(fault.observed_publisher_gid, changed_message_gid)
        self.assertEqual(fault.observed_publisher_gid_domain, "message")
        self.assertEqual(fault.publisher_count, 1)
        for malformed in (
            f"READY_V3\t{graph_gid}\t{message_gid}\t"
            f"{rmw_identifier}\t29\t1\t3\t3",
            f"READY_V3\t{graph_gid}\t{message_gid}\t"
            f"{rmw_identifier}\t30\t2\t3\t3",
            f"READY_V3\t{graph_gid}\t{message_gid}\t"
            f"{rmw_identifier}\t30\t1\t2\t3",
            f"READY_V3\t{graph_gid}\t{message_gid}\t"
            f"{rmw_identifier}\t30\t1\t3\t2",
            f"READY_V3\t{'0' * 48}\t{message_gid}\t"
            f"{rmw_identifier}\t30\t1\t3\t3",
            "FAULT_V3\tbad-reason\t-\t-\t-\t-\t0\t-",
            f"FAULT_V3\tmessage_publisher_gid_changed\t{graph_gid}\t"
            f"{message_gid}\t{changed_message_gid}\t-\t1\t{rmw_identifier}",
            "UNKNOWN\t-",
        ):
            with self.subTest(event=malformed):
                with self.assertRaises(relay.RelayError):
                    relay.parse_worker_event(malformed)

    def test_python_supervisor_commits_v3_ready_from_local_pipe(self) -> None:
        graph_gid = "ab" * 16 + "00" * 8
        message_gid = "bc" * 8 + "00" * 16
        rmw_identifier = "rmw_cyclonedds_cpp"
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval_file = directory / "approval.json"
            stamp_ready = directory / "stamp-ready.json"
            ready_file = directory / "relay-ready.json"
            fault_file = directory / "relay-fault.json"
            worker = directory / "fake-worker"
            self.write_private_json(approval_file, self.approval_payload())
            self.write_private_json(stamp_ready, self.ready())
            self.write_fake_worker(
                worker,
                f"READY_V3\t{graph_gid}\t{message_gid}\t"
                f"{rmw_identifier}\t30\t1\t3\t3\n",
            )
            stop_requested = threading.Event()
            results: list[int] = []

            thread = threading.Thread(
                target=lambda: results.append(
                    relay.run_supervisor(
                        approval_file,
                        relay.load_approval(approval_file, require_affine=True),
                        stamp_ready,
                        ready_file,
                        fault_file,
                        worker_binary=worker,
                        stop_requested=stop_requested,
                    )
                )
            )
            thread.start()
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not ready_file.exists():
                time.sleep(0.01)
            stop_requested.set()
            thread.join(timeout=3.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(results, [0])
            self.assertFalse(fault_file.exists())
            payload = json.loads(ready_file.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema"], relay.READY_SCHEMA)
            self.assertEqual(payload["graph_publisher_gid"], graph_gid)
            self.assertEqual(payload["message_publisher_gid"], message_gid)
            self.assertEqual(
                payload["rmw_implementation_identifier"], rmw_identifier
            )
            self.assertEqual(payload["publisher_gid_bytes"], 24)
            self.assertTrue(payload["publisher_identity_exact"])
            self.assertTrue(payload["publisher_qos_exact"])
            self.assertTrue(payload["graph_publisher_gid_frozen"])
            self.assertTrue(payload["message_publisher_gid_frozen"])
            self.assertFalse(payload["cross_gid_representation_comparison"])
            self.assertFalse(payload["rebind_allowed"])
            self.assertTrue(
                payload["message_gid_implementation_identifier_exact"]
            )
            self.assertEqual(payload["graph_recheck_period_ms"], 50)
            self.assertEqual(payload["initial_graph_discovery_timeout_ms"], 1000)
            self.assertEqual(payload["supervisor_heartbeat_timeout_ms"], 200)
            self.assertTrue(payload["supervisor_heartbeat_active"])
            self.assertFalse(payload["motion_authorized"])

    def test_cpp_fault_fields_are_preserved_in_v3_fault_file(self) -> None:
        graph_gid = "ab" * 16 + "00" * 8
        message_gid = "bc" * 8 + "00" * 16
        observed = "cd" * 8 + "00" * 16
        rmw_identifier = "rmw_cyclonedds_cpp"
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval_file = directory / "approval.json"
            stamp_ready = directory / "stamp-ready.json"
            ready_file = directory / "relay-ready.json"
            fault_file = directory / "relay-fault.json"
            worker = directory / "fake-worker"
            self.write_private_json(approval_file, self.approval_payload())
            self.write_private_json(stamp_ready, self.ready())
            self.write_fake_worker(
                worker,
                "FAULT_V3\tmessage_publisher_gid_changed\t"
                f"{graph_gid}\t{message_gid}\t{observed}\tmessage\t1\t"
                f"{rmw_identifier}\n",
            )
            status = relay.run_supervisor(
                approval_file,
                relay.load_approval(approval_file, require_affine=True),
                stamp_ready,
                ready_file,
                fault_file,
                worker_binary=worker,
            )
            self.assertEqual(status, 70)
            self.assertFalse(ready_file.exists())
            payload = json.loads(fault_file.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema"], relay.FAULT_SCHEMA)
            self.assertEqual(payload["graph_publisher_gid"], graph_gid)
            self.assertEqual(payload["message_publisher_gid"], message_gid)
            self.assertEqual(payload["observed_publisher_gid"], observed)
            self.assertEqual(
                payload["observed_publisher_gid_domain"], "message"
            )
            self.assertEqual(
                payload["rmw_implementation_identifier"], rmw_identifier
            )
            self.assertEqual(payload["publisher_count"], 1)
            self.assertTrue(payload["fault_latched"])
            self.assertFalse(payload["rebind_allowed"])
            self.assertFalse(payload["motion_authorized"])

    def test_fault_in_same_pipe_drain_dominates_ready(self) -> None:
        graph_gid = "ab" * 16 + "00" * 8
        message_gid = "bc" * 8 + "00" * 16
        changed_graph_gid = "cd" * 16 + "00" * 8
        rmw_identifier = "rmw_cyclonedds_cpp"
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval_file = directory / "approval.json"
            stamp_ready = directory / "stamp-ready.json"
            ready_file = directory / "relay-ready.json"
            fault_file = directory / "relay-fault.json"
            worker = directory / "fake-worker"
            self.write_private_json(approval_file, self.approval_payload())
            self.write_private_json(stamp_ready, self.ready())
            self.write_fake_worker(
                worker,
                f"READY_V3\t{graph_gid}\t{message_gid}\t"
                f"{rmw_identifier}\t30\t1\t3\t3\n"
                f"FAULT_V3\tgraph_publisher_gid_changed\t{graph_gid}\t"
                f"{message_gid}\t{changed_graph_gid}\tgraph\t1\t"
                f"{rmw_identifier}\n",
            )
            status = relay.run_supervisor(
                approval_file,
                relay.load_approval(approval_file, require_affine=True),
                stamp_ready,
                ready_file,
                fault_file,
                worker_binary=worker,
            )
            self.assertEqual(status, 70)
            self.assertFalse(ready_file.exists())
            payload = json.loads(fault_file.read_text(encoding="utf-8"))
            self.assertEqual(
                payload["reason"], "worker:graph_publisher_gid_changed"
            )
            self.assertEqual(payload["graph_publisher_gid"], graph_gid)
            self.assertEqual(payload["message_publisher_gid"], message_gid)
            self.assertEqual(
                payload["observed_publisher_gid"], changed_graph_gid
            )

    def test_approval_mutation_after_ready_latches_fault_and_stops_worker(
        self,
    ) -> None:
        graph_gid = "ab" * 16 + "00" * 8
        message_gid = "bc" * 8 + "00" * 16
        rmw_identifier = "rmw_cyclonedds_cpp"
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval_file = directory / "approval.json"
            stamp_ready = directory / "stamp-ready.json"
            ready_file = directory / "relay-ready.json"
            fault_file = directory / "relay-fault.json"
            worker = directory / "fake-worker"
            approval_payload = self.approval_payload()
            self.write_private_json(approval_file, approval_payload)
            self.write_private_json(stamp_ready, self.ready())
            self.write_fake_worker(
                worker,
                f"READY_V3\t{graph_gid}\t{message_gid}\t"
                f"{rmw_identifier}\t30\t1\t3\t3\n",
            )
            results: list[int] = []
            thread = threading.Thread(
                target=lambda: results.append(
                    relay.run_supervisor(
                        approval_file,
                        relay.load_approval(approval_file, require_affine=True),
                        stamp_ready,
                        ready_file,
                        fault_file,
                        worker_binary=worker,
                    )
                )
            )
            thread.start()
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not ready_file.exists():
                time.sleep(0.01)
            self.assertTrue(ready_file.exists())
            changed = copy.deepcopy(approval_payload)
            changed["session_id"] = "changed-session-0123456789abcdef"
            self.write_private_json(approval_file, changed)
            thread.join(timeout=3.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(results, [70])
            payload = json.loads(fault_file.read_text(encoding="utf-8"))
            self.assertIn("approval changed during relay", payload["reason"])
            self.assertEqual(payload["graph_publisher_gid"], graph_gid)
            self.assertEqual(payload["message_publisher_gid"], message_gid)
            self.assertFalse(payload["motion_authorized"])

    def test_worker_exit_without_event_is_a_v3_fault(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            approval_file = directory / "approval.json"
            stamp_ready = directory / "stamp-ready.json"
            ready_file = directory / "relay-ready.json"
            fault_file = directory / "relay-fault.json"
            worker = directory / "fake-worker"
            self.write_private_json(approval_file, self.approval_payload())
            self.write_private_json(stamp_ready, self.ready())
            worker.write_text("#!/usr/bin/env python3\n", encoding="utf-8")
            worker.chmod(0o700)
            status = relay.run_supervisor(
                approval_file,
                relay.load_approval(approval_file, require_affine=True),
                stamp_ready,
                ready_file,
                fault_file,
                worker_binary=worker,
            )
            self.assertEqual(status, 70)
            self.assertFalse(ready_file.exists())
            payload = json.loads(fault_file.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema"], relay.FAULT_SCHEMA)
            self.assertIn("worker exited", payload["reason"])
            self.assertIsNone(payload["graph_publisher_gid"])
            self.assertIsNone(payload["message_publisher_gid"])
            self.assertIsNone(payload["observed_publisher_gid"])
            self.assertEqual(payload["publisher_count"], 0)

    def test_relay_has_no_command_or_unitree_api_surface(self) -> None:
        supervisor = MODULE_PATH.read_text(encoding="utf-8")
        worker = (
            ROOT
            / "packages/go2_motion_state_relay/src/"
            "workstation_motion_state_relay.cpp"
        ).read_text(encoding="utf-8")
        guard = (
            ROOT
            / "packages/go2_motion_state_relay/include/"
            "go2_motion_state_relay/gid_guard.hpp"
        ).read_text(encoding="utf-8")
        self.assertNotIn("import rclpy", supervisor)
        self.assertNotIn("create_publisher", supervisor)
        self.assertGreaterEqual(supervisor.count("require_affine=True"), 3)
        self.assertEqual(relay.MAX_CORRECTED_AGE_NS, 200_000_000)
        self.assertIn("EvaluateCorrectedStampFreshness", worker)
        self.assertIn(relay.OUTPUT_TOPIC, worker)
        self.assertIn("rclcpp::MessageInfo", worker)
        self.assertIn("publisher_gid.data", worker)
        self.assertIn("publisher_gid.implementation_identifier", worker)
        self.assertIn("rmw_get_implementation_identifier", worker)
        self.assertIn("get_publishers_info_by_topic", worker)
        self.assertIn("rclcpp::SensorDataQoS", worker)
        self.assertIn("CorrectedStateInputQos", worker)
        self.assertIn("IsSafeCorrectedStatePublisherQos", worker)
        self.assertNotIn(
            "endpoint.qos_profile() == rclcpp::SensorDataQoS()", worker
        )
        self.assertIn("create_wall_timer(50ms", worker)
        self.assertIn("std::from_chars", worker)
        self.assertIn("time.monotonic_ns()", supervisor)
        worker_launch = supervisor.index("worker = subprocess.Popen")
        initial_heartbeat = supervisor.index(
            "initial worker heartbeat write was incomplete"
        )
        self.assertLess(initial_heartbeat, worker_launch)
        observe_message = worker.index("void ObserveMessage")
        stale_branch = worker.index(
            "CorrectedStampFreshness::kTooOld", observe_message
        )
        stale_branch_end = worker.index(
            "if (last_receipt_steady_ns_.has_value()", stale_branch
        )
        stale_body = worker[stale_branch:stale_branch_end]
        self.assertIn("return;", stale_body)
        self.assertNotIn("LatchFault", stale_body)
        self.assertNotIn("publisher_->publish", stale_body)
        callback_heartbeat_drain = worker.index(
            "DrainHeartbeat();", observe_message
        )
        callback_heartbeat_check = worker.index(
            "if (!HeartbeatFresh", observe_message
        )
        self.assertLess(callback_heartbeat_drain, callback_heartbeat_check)
        qualification_publish_block = worker.index(
            "if (!ready_emitted_) {", worker.index("void ObserveMessage")
        )
        ready_event = worker.index("EmitReady();", qualification_publish_block)
        qualification_return = worker.index("return;", ready_event)
        first_publish = worker.index(
            "publisher_->publish(*message);", qualification_return
        )
        self.assertLess(ready_event, qualification_return)
        self.assertLess(qualification_return, first_publish)
        self.assertIn("kRequiredStableGraphPolls = 3U", guard)
        self.assertIn("kRequiredStableMessageSamples = 3U", guard)
        self.assertIn("frozen_graph_gid_", guard)
        self.assertIn("frozen_message_gid_", guard)
        self.assertIn("RMW_GID_STORAGE_SIZE", worker)
        for forbidden in (
            "/cmd_vel",
            "/api/sport/request",
            "/lowcmd",
            "SportClient",
            "create_client",
            "create_service",
        ):
            self.assertNotIn(forbidden, supervisor)
            self.assertNotIn(forbidden, worker)


if __name__ == "__main__":
    unittest.main()
