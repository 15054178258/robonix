#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_workstation_nomotion_tcp_ports.py"
SPEC = importlib.util.spec_from_file_location("nomotion_tcp_ports", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
PORTS = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PORTS
SPEC.loader.exec_module(PORTS)


PROC_HEADER = (
    "  sl  local_address rem_address   st tx_queue rx_queue tr tm->when "
    "retrnsmt   uid  timeout inode\n"
)


def proc_row(address: str, port: int, state: str, inode: int) -> str:
    return (
        f"   0: {address}:{port:04X} 00000000:0000 {state} "
        f"00000000:00000000 00:00000000 00000000 1000 0 {inode}\n"
    )


class WorkstationNomotionTcpPortTests(unittest.TestCase):
    def setUp(self) -> None:
        self.manifest = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )

    def test_manifest_audit_discovers_complete_fixed_profile(self) -> None:
        claims = PORTS.discover_port_claims(self.manifest, 8092, 18080)
        self.assertEqual(
            {(claim.owner, claim.port) for claim in claims},
            {
                ("system.atlas", 50051),
                ("system.executor", 50061),
                ("system.pilot", 50071),
                ("system.liaison", 50081),
                ("system.soma", 50091),
                ("system.scene.web", 50107),
                ("primitive.audio_client_bridge", 60002),
                ("service.mapping.webui", 8091),
                ("service.go2_dashboard", 8092),
                ("semantic_intent_router", 18080),
            },
        )

    def test_custom_dashboard_and_semantic_ports_are_audited(self) -> None:
        claims = PORTS.discover_port_claims(self.manifest, 19092, 19080)
        owners = {claim.owner: claim.port for claim in claims}
        self.assertEqual(owners["service.go2_dashboard"], 19092)
        self.assertEqual(owners["semantic_intent_router"], 19080)

    def test_duplicate_profile_assignment_fails_before_proc_read(self) -> None:
        with self.assertRaisesRegex(PORTS.PreflightError, "multiple owners"):
            PORTS.discover_port_claims(self.manifest, 8091, 18080)

    def test_scene_ui_must_remain_on_literal_loopback(self) -> None:
        self.manifest["system"]["scene"]["web_host"] = "0.0.0.0"
        with self.assertRaisesRegex(PORTS.PreflightError, "literal loopback"):
            PORTS.discover_port_claims(self.manifest, 8092, 18080)

    def test_proc_parser_reports_ipv4_and_ipv6_listeners_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            proc = Path(temporary)
            (proc / "tcp").write_text(
                PROC_HEADER
                + proc_row("0100007F", 50051, "0A", 101)
                + proc_row("0100007F", 50061, "01", 102),
                encoding="ascii",
            )
            (proc / "tcp6").write_text(
                PROC_HEADER
                + proc_row("00000000000000000000000001000000", 8092, "0A", 201),
                encoding="ascii",
            )
            listeners = PORTS.parse_proc_net(proc / "tcp", "tcp")
            listeners += PORTS.parse_proc_net(proc / "tcp6", "tcp6")
        self.assertEqual(
            {(item.port, item.address, item.family, item.inode) for item in listeners},
            {
                (50051, "127.0.0.1", "tcp", "101"),
                (8092, "::1", "tcp6", "201"),
            },
        )

    def test_cli_refuses_occupied_ports_without_process_control(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            proc = Path(temporary)
            (proc / "tcp").write_text(
                PROC_HEADER + proc_row("00000000", 18080, "0A", 301),
                encoding="ascii",
            )
            (proc / "tcp6").write_text(PROC_HEADER, encoding="ascii")
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--manifest",
                    str(ROOT / "robonix_manifest.yaml"),
                    "--dashboard-port",
                    "8092",
                    "--semantic-port",
                    "18080",
                    "--proc-net-root",
                    str(proc),
                ],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        self.assertEqual(result.returncode, 1, result)
        self.assertIn("18080 (semantic_intent_router)", result.stderr)
        self.assertIn("no process was stopped", result.stderr)

    def test_cli_passes_when_profile_ports_are_free(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            proc = Path(temporary)
            (proc / "tcp").write_text(
                PROC_HEADER + proc_row("0100007F", 12345, "0A", 401),
                encoding="ascii",
            )
            # tcp6 is optional when IPv6 support is absent from the kernel.
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--manifest",
                    str(ROOT / "robonix_manifest.yaml"),
                    "--dashboard-port",
                    "8092",
                    "--semantic-port",
                    "18080",
                    "--proc-net-root",
                    str(proc),
                ],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        self.assertEqual(result.returncode, 0, result)
        self.assertIn("TCP port preflight passed", result.stdout)

    def test_launcher_runs_port_gate_before_ros_setup_and_docker(self) -> None:
        launcher = (
            ROOT / "scripts" / "start_workstation_full_nomotion_corrected.sh"
        ).read_text(encoding="utf-8")
        gate_index = launcher.index("check_workstation_nomotion_tcp_ports.py")
        self.assertLess(gate_index, launcher.index("source /opt/ros/humble/setup.bash"))
        # Docker is started indirectly by start.sh; the wrapper must run the
        # gate before delegating to that launcher.
        self.assertLess(gate_index, launcher.index('bash "$ROOT/start.sh"'))


if __name__ == "__main__":
    unittest.main()
