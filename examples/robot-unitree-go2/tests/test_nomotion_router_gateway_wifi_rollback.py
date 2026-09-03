from __future__ import annotations

import ast
import importlib.util
import os
from pathlib import Path
import signal
import sys
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "nomotion_router_gateway_wifi_rollback.py"
SPEC = importlib.util.spec_from_file_location(
    "nomotion_router_gateway_wifi_rollback", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ReachabilityRunner:
    def __init__(self, *, ping_received: int = 3, http_status: str = "401") -> None:
        self.ping_received = ping_received
        self.http_status = http_status
        self.calls: list[tuple[tuple[str, ...], dict[str, str] | None]] = []

    def run(self, argv, *, operation, timeout, environ=None):  # noqa: ANN001, ANN201
        del operation, timeout
        command = tuple(argv)
        copied = dict(environ) if environ is not None else None
        self.calls.append((command, copied))
        if command[0] == "ping":
            loss = 0 if self.ping_received == 3 else 34
            return (
                f"3 packets transmitted, {self.ping_received} received, "
                f"{loss}% packet loss, time 2000ms\n"
            )
        if command[0] == "/usr/bin/curl":
            return (
                f"{self.http_status}\t192.168.123.99\t8.140.217.18\t0\t0\t"
                f"{MODULE.DASHSCOPE_PROBE}"
            )
        if command[:5] == ("ip", "-4", "route", "get", "8.140.217.18"):
            return (
                "8.140.217.18 via 192.168.123.1 dev wlo1 "
                "src 192.168.123.99 uid 1000\n"
            )
        raise AssertionError(f"unexpected command: {command!r}")


class RouterGatewayWifiRollbackTests(unittest.TestCase):
    def test_constants_match_physical_topology(self) -> None:
        self.assertEqual(MODULE.WIFI_INTERFACE, "wlo1")
        self.assertEqual(MODULE.WIRED_INTERFACE, "enp108s0")
        self.assertEqual(MODULE.HOST_ADDRESS, "192.168.123.99/24")
        self.assertEqual(
            MODULE.TARGET_HOSTS,
            ("192.168.123.1", "192.168.123.18", "192.168.123.161"),
        )

    def test_parser_requires_explicit_mode(self) -> None:
        parser = MODULE.build_parser()
        with self.assertRaises(SystemExit):
            parser.parse_args([])
        self.assertTrue(parser.parse_args(["--preflight"]).preflight)
        self.assertTrue(
            parser.parse_args(["--execute-after-approval-interactive"])
            .execute_after_approval_interactive
        )

    def test_hold_boundaries(self) -> None:
        self.assertEqual(MODULE.bounded_hold("0"), 0)
        self.assertEqual(MODULE.bounded_hold("120"), 120)
        for value in ("-1", "121", "not-an-int"):
            with self.assertRaises(Exception):
                MODULE.bounded_hold(value)

    def test_signal_latch_defers_only_during_rollback_boundary(self) -> None:
        latch = MODULE.SignalLatch()
        latch.capture(signal.SIGTERM)
        with latch.deferred():
            latch.check()
        with self.assertRaises(MODULE.CutoverInterrupted):
            latch.check()

    def _workflow(self, runner: ReachabilityRunner):
        latch = MODULE.SignalLatch()
        workflow = MODULE.RouterGatewayAudit(runner, latch)
        workflow._validate_runtime_state = lambda: None  # type: ignore[method-assign]
        return workflow

    def test_reachability_requires_all_three_ping_replies(self) -> None:
        workflow = self._workflow(ReachabilityRunner(ping_received=2))
        with self.assertRaisesRegex(MODULE.CutoverError, "packet loss"):
            workflow._validate_reachability()

    def test_reachability_requires_dashscope_401(self) -> None:
        workflow = self._workflow(ReachabilityRunner(http_status="302"))
        with self.assertRaisesRegex(MODULE.CutoverError, "HTTP 401"):
            workflow._validate_reachability()

    def test_direct_probe_removes_proxy_environment(self) -> None:
        runner = ReachabilityRunner()
        workflow = self._workflow(runner)
        with mock.patch.dict(
            os.environ,
            {"http_proxy": "http://127.0.0.1:1", "ALL_PROXY": "socks5://127.0.0.1:2"},
        ):
            workflow._validate_reachability()
        curl_command, curl_env = next(
            call for call in runner.calls if call[0][0] == "/usr/bin/curl"
        )
        self.assertEqual(curl_command[:2], ("/usr/bin/curl", "--disable"))
        self.assertIn("--proxy", curl_command)
        self.assertEqual(curl_command[curl_command.index("--proxy") + 1], "")
        assert curl_env is not None
        self.assertNotIn("http_proxy", curl_env)
        self.assertNotIn("ALL_PROXY", curl_env)

    def test_source_contains_no_secret_retrieval_or_shell_execution(self) -> None:
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        string_literals = {
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        self.assertNotIn("802-11-wireless-security.psk", string_literals)
        self.assertNotIn("GetSecrets", string_literals)
        self.assertNotIn("--show-secrets", string_literals)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            for keyword in node.keywords:
                if keyword.arg == "shell":
                    self.assertIsInstance(keyword.value, ast.Constant)
                    self.assertFalse(keyword.value.value)


if __name__ == "__main__":
    unittest.main()
