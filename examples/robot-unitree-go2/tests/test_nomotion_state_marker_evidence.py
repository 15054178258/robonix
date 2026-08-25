#!/usr/bin/env python3
"""Offline tests for the no-motion SportModeState evidence gate."""

from __future__ import annotations

import contextlib
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR = ROOT / "scripts" / "validate_nomotion_state_marker_evidence.py"
WRAPPER = ROOT / "scripts" / "start_workstation_full_nomotion_corrected.sh"
NOW_NS = 2_000_000_000_000_000_000


def load_validator():
    spec = importlib.util.spec_from_file_location(
        "validate_nomotion_state_marker_evidence_under_test", VALIDATOR
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {VALIDATOR}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def valid_payload(marker: int = 100) -> dict:
    elapsed_ns = 25_000_000_000
    state = {
        "error_code": marker,
        "mode": 0,
        "gait_type": 0,
        "samples": 150,
    }
    return {
        "schema_version": 1,
        "mode": "read-only-subscriber-only",
        "duration_limit_s": 25,
        "started_realtime_ns": NOW_NS - elapsed_ns,
        "elapsed_monotonic_ns": elapsed_ns,
        "publishers_created": False,
        "unitree_clients_created": False,
        "streams": [
            {
                "topic": "/sportmodestate",
                "received": 150,
                "first_source_stamp_ns": 1_000_000_000,
                "last_source_stamp_ns": 25_000_000_000,
                "source_regressions": 0,
                "max_abs_linear_velocity": 0.02,
                "max_abs_yaw_speed": 0.03,
                "states": [copy.deepcopy(state)],
            },
            {
                "topic": "/lf/sportmodestate",
                "received": 150,
                "first_source_stamp_ns": 1_100_000_000,
                "last_source_stamp_ns": 25_100_000_000,
                "source_regressions": 0,
                "max_abs_linear_velocity": 0.01,
                "max_abs_yaw_speed": 0.02,
                "states": [copy.deepcopy(state)],
            },
        ],
    }


class NomotionStateMarkerEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.validator = load_validator()

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write(self, payload: dict, *, mode: int = 0o600) -> Path:
        path = self.directory / "sport-state-summary.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        path.chmod(mode)
        return path

    def validate(self, payload: dict) -> int:
        return self.validator.validate_evidence(
            self.write(payload), now_realtime_ns=NOW_NS
        )

    def test_valid_evidence_returns_only_exact_marker_on_stdout(self) -> None:
        path = self.write(valid_payload(2010))
        output = io.StringIO()
        with mock.patch.object(
            sys, "argv", [str(VALIDATOR), str(path), "--max-age-seconds", "1800"]
        ), mock.patch.object(self.validator.time, "time_ns", return_value=NOW_NS), \
            contextlib.redirect_stdout(output):
            self.assertEqual(self.validator.main(), 0)
        self.assertEqual(output.getvalue(), "2010\n")

    def test_stale_evidence_is_rejected(self) -> None:
        payload = valid_payload()
        payload["started_realtime_ns"] -= 1_801_000_000_000
        with self.assertRaisesRegex(self.validator.EvidenceError, "stale"):
            self.validate(payload)

    def test_motion_above_stationary_limit_is_rejected(self) -> None:
        payload = valid_payload()
        payload["streams"][0]["max_abs_linear_velocity"] = 0.050001
        with self.assertRaisesRegex(self.validator.EvidenceError, "stationary limit"):
            self.validate(payload)

    def test_state_changes_are_rejected(self) -> None:
        payload = valid_payload()
        changed = copy.deepcopy(payload["streams"][0]["states"][0])
        changed["error_code"] = 1015
        changed["samples"] = 1
        payload["streams"][0]["states"].append(changed)
        with self.assertRaisesRegex(self.validator.EvidenceError, "exactly one"):
            self.validate(payload)

    def test_disagreement_between_streams_is_rejected(self) -> None:
        payload = valid_payload()
        payload["streams"][1]["states"][0]["gait_type"] = 1
        with self.assertRaisesRegex(self.validator.EvidenceError, "disagree"):
            self.validate(payload)

    def test_source_timestamp_regression_is_rejected(self) -> None:
        payload = valid_payload()
        payload["streams"][0]["source_regressions"] = 1
        with self.assertRaisesRegex(self.validator.EvidenceError, "regressions"):
            self.validate(payload)

    def test_group_or_world_writable_file_is_rejected(self) -> None:
        for mode in (0o620, 0o602):
            with self.subTest(mode=oct(mode)):
                path = self.write(valid_payload(), mode=mode)
                with self.assertRaisesRegex(
                    self.validator.EvidenceError, "group- or world-writable"
                ):
                    self.validator.validate_evidence(
                        path, now_realtime_ns=NOW_NS
                    )
                path.unlink()

    def test_symbolic_link_is_rejected(self) -> None:
        target = self.write(valid_payload())
        link = self.directory / "evidence-link.json"
        link.symlink_to(target)
        self.assertTrue(stat.S_ISLNK(os.lstat(link).st_mode))
        with self.assertRaisesRegex(self.validator.EvidenceError, "non-symlink"):
            self.validator.validate_evidence(link, now_realtime_ns=NOW_NS)

    def test_wrapper_cannot_inherit_direct_motion_or_marker_bypasses(self) -> None:
        source = WRAPPER.read_text(encoding="utf-8")
        forced = (
            "export GO2_ALLOW_MOTION=false",
            "export GO2_OPERATOR_PRESENT=false",
            'export GO2_SAFETY_ACK=""',
            'export GO2_ALLOWED_MODES=""',
            'export GO2_ALLOWED_STATE_MARKERS=""',
        )
        for line in forced:
            with self.subTest(line=line):
                self.assertIn(line, source)
        self.assertNotIn("${GO2_ALLOWED_STATE_MARKERS:-", source)
        self.assertIn(
            'NOMOTION_STATE_EVIDENCE="${GO2_NOMOTION_STATE_EVIDENCE:-}"', source
        )
        self.assertIn("validate_nomotion_state_marker_evidence.py", source)
        self.assertIn(
            'readonly PASSIVE_MANUAL_MAPPING_STATE_MARKERS="100,1002"', source
        )
        self.assertIn(
            'export GO2_ALLOWED_STATE_MARKERS="$PASSIVE_MANUAL_MAPPING_STATE_MARKERS"',
            source,
        )
        self.assertIn(
            '*",$validated_state_marker,"*) ;;', source
        )
        self.assertIn("2026-07-22", source)
        self.assertIn("--passive-state-markers", source)
        self.assertLess(
            source.index('export GO2_ALLOWED_STATE_MARKERS=""'),
            source.index(
                'export GO2_ALLOWED_STATE_MARKERS="$PASSIVE_MANUAL_MAPPING_STATE_MARKERS"'
            ),
        )


if __name__ == "__main__":
    unittest.main()
