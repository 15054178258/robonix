from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import shutil
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import unittest


ROOT = Path(__file__).resolve().parents[1]
READINESS_PATH = ROOT / "scripts" / "operator_ui_nomotion_readiness.py"
SUPERVISOR_PATH = ROOT / "scripts" / "start_operator_ui_nomotion.sh"
SPEC = importlib.util.spec_from_file_location("operator_ui_nomotion_readiness", READINESS_PATH)
assert SPEC is not None and SPEC.loader is not None
READINESS = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = READINESS
SPEC.loader.exec_module(READINESS)


class _QuietTCPHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        self.request.recv(1)


def _handler_for(routes: dict[str, tuple[str, object]]) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args: object) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802 - stdlib callback name
            route = routes.get(self.path)
            if route is None:
                self.send_error(404)
                return
            content_type, value = route
            if content_type == "application/json":
                body = json.dumps(value).encode("utf-8")
            else:
                body = str(value).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


class OperatorReadinessTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.temp = Path(self.temporary.name)
        self.servers: list[object] = []
        self.threads: list[threading.Thread] = []

    def tearDown(self) -> None:
        for server in self.servers:
            server.shutdown()
            server.server_close()
        for thread in self.threads:
            thread.join(timeout=2)
        self.temporary.cleanup()

    def _start(self, server: object) -> int:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.servers.append(server)
        self.threads.append(thread)
        return int(server.server_address[1])

    def _http(self, routes: dict[str, tuple[str, object]]) -> int:
        server = ThreadingHTTPServer((READINESS.LOOPBACK, 0), _handler_for(routes))
        return self._start(server)

    def _rbnx(self) -> Path:
        path = self.temp / "rbnx"
        path.write_text("#!/bin/sh\nprintf '%s\\n' '[]'\n", encoding="utf-8")
        path.chmod(0o700)
        return path

    def _ready_endpoints(self) -> tuple[object, Path]:
        atlas = socketserver.TCPServer(
            (READINESS.LOOPBACK, 0), _QuietTCPHandler, bind_and_activate=True
        )
        atlas_port = self._start(atlas)
        scene_port = self._http({"/user": ("text/html", "<html>scene</html>")})
        mapping_port = self._http(
            {"/api/state": ("application/json", {"mode": "mapping", "has_map": False})}
        )
        dashboard_port = self._http(
            {
                "/healthz": (
                    "application/json",
                    {"ok": True, "telemetry_read_only": True},
                )
            }
        )
        client_port = self._http(
            {
                "/api/defaults": (
                    "application/json",
                    {"robotHost": READINESS.LOOPBACK, "atlasPort": atlas_port},
                )
            }
        )
        endpoints = READINESS.OperatorEndpoints(
            atlas_port=atlas_port,
            scene_port=scene_port,
            mapping_port=mapping_port,
            dashboard_port=dashboard_port,
            client_port=client_port,
        )
        return endpoints, self._rbnx()

    def test_full_probe_verifies_protocol_and_all_four_http_surfaces(self) -> None:
        endpoints, rbnx = self._ready_endpoints()
        READINESS.probe_full(endpoints, rbnx, timeout=0.5)

    def test_client_with_wrong_atlas_target_is_rejected(self) -> None:
        port = self._http(
            {
                "/api/defaults": (
                    "application/json",
                    {"robotHost": READINESS.LOOPBACK, "atlasPort": 12345},
                )
            }
        )
        with self.assertRaisesRegex(READINESS.ProbeError, "audited Atlas"):
            READINESS.probe_client(port, 50051, timeout=0.5)

    def test_dashboard_must_assert_read_only_telemetry(self) -> None:
        port = self._http(
            {
                "/healthz": (
                    "application/json",
                    {"ok": True, "telemetry_read_only": False},
                )
            }
        )
        with self.assertRaisesRegex(READINESS.ProbeError, "telemetry_read_only"):
            READINESS.probe_dashboard(port, timeout=0.5)

    def test_port_probe_observes_listener_without_stopping_it(self) -> None:
        server = socketserver.TCPServer(
            (READINESS.LOOPBACK, 0), _QuietTCPHandler, bind_and_activate=True
        )
        port = self._start(server)
        self.assertTrue(READINESS.port_has_listener(port))
        with __import__("socket").socket() as probe:
            probe.bind((READINESS.LOOPBACK, 0))
            free_port = int(probe.getsockname()[1])
        self.assertFalse(READINESS.port_has_listener(free_port))


class OperatorSupervisorContractTests(unittest.TestCase):
    def test_supervisor_is_explicitly_motion_disabled_and_has_exact_urls(self) -> None:
        source = SUPERVISOR_PATH.read_text(encoding="utf-8")
        self.assertIn("export GO2_ALLOW_MOTION=false", source)
        self.assertIn("export GO2_OPERATOR_PRESENT=false", source)
        self.assertIn("export ROBONIX_CLIENT_ENABLE_AUDIO=auto", source)
        self.assertIn("start_workstation_full_nomotion_corrected.sh", source)
        self.assertIn("start_robonix_client_local.sh", source)
        for endpoint in (
            "127.0.0.1:50051",
            "http://127.0.0.1:50107/user",
            "http://127.0.0.1:8091/",
            "http://127.0.0.1:8092/",
            "http://127.0.0.1:7860/",
        ):
            self.assertIn(endpoint, source)
        for forbidden in (
            "sudo ",
            "\napt ",
            "ros2 topic pub",
            "/cmd_vel",
            "/api/sport/request",
            "/lowcmd",
            "pkill",
            "killall",
            "pgrep",
            "kill -9",
            "kill -- -",
        ):
            self.assertNotIn(forbidden, source)

    def test_stack_is_ready_before_client_and_cleanup_uses_pid_and_start_ticks(self) -> None:
        source = SUPERVISOR_PATH.read_text(encoding="utf-8")
        stack_start = source.index('bash "$STACK_LAUNCHER" &')
        stack_ready = source.index("wait_for_phase stack")
        client_start = source.index('bash "$CLIENT_LAUNCHER" &')
        full_ready = source.index("wait_for_phase full")
        self.assertLess(stack_start, stack_ready)
        self.assertLess(stack_ready, client_start)
        self.assertLess(client_start, full_ready)
        self.assertIn('kill -TERM "$pid"', source)
        self.assertIn('"$ticks" == "$expected_ticks"', source)
        self.assertIn(
            'wait -n -p EXITED_PID "${WAIT_PIDS[@]}"',
            source,
        )
        self.assertIn(
            "Mapping, Scene and the no-motion stack remain running",
            source,
        )
        self.assertNotIn(
            "official Client exited; closing the owned no-motion stack",
            source,
        )

    def test_signal_flow_stops_only_the_two_owned_children(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            deploy = workspace / "packages" / "robot-unitree-go2"
            scripts = deploy / "scripts"
            tools_python = workspace / ".tools" / "rbnx-python" / "bin"
            tools_rbnx = workspace / ".tools" / "rbnx" / "bin"
            scripts.mkdir(parents=True)
            tools_python.mkdir(parents=True)
            tools_rbnx.mkdir(parents=True)
            shutil.copy2(SUPERVISOR_PATH, scripts / SUPERVISOR_PATH.name)
            (tools_python / "python3").symlink_to(sys.executable)
            fake_rbnx = tools_rbnx / "rbnx"
            fake_rbnx.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
            fake_rbnx.chmod(0o700)
            readiness = scripts / "operator_ui_nomotion_readiness.py"
            readiness.write_text("raise SystemExit(0)\n", encoding="utf-8")
            log = Path(temporary) / "flow.log"
            child_template = """#!/usr/bin/env bash
set -euo pipefail
label=LABEL
[[ "${GO2_ALLOW_MOTION:-}" == false ]]
[[ "${ROBONIX_CLIENT_ENABLE_AUDIO:-}" == auto ]]
printf '%s-start\\n' "$label" >> "$FLOW_LOG"
trap 'printf "%s-stop\\n" "$label" >> "$FLOW_LOG"; exit 0' TERM
while :; do sleep 0.05; done
"""
            for filename, label in (
                ("start_workstation_full_nomotion_corrected.sh", "stack"),
                ("start_robonix_client_local.sh", "client"),
            ):
                (scripts / filename).write_text(
                    child_template.replace("LABEL", label), encoding="utf-8"
                )
            env = os.environ.copy()
            env["FLOW_LOG"] = str(log)
            supervisor = subprocess.Popen(
                ["bash", str(scripts / SUPERVISOR_PATH.name)],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            unrelated = subprocess.Popen(["sleep", "30"])
            try:
                deadline = time.monotonic() + 5
                content = ""
                while time.monotonic() < deadline:
                    content = log.read_text(encoding="utf-8") if log.exists() else ""
                    if "stack-start" in content and "client-start" in content:
                        break
                    time.sleep(0.02)
                self.assertIn("stack-start", content)
                self.assertIn("client-start", content)
                supervisor.terminate()
                supervisor.wait(timeout=5)
                self.assertEqual(supervisor.returncode, 143)
                final = log.read_text(encoding="utf-8")
                self.assertIn("client-stop", final)
                self.assertIn("stack-stop", final)
                self.assertIsNone(unrelated.poll(), "unowned process must remain alive")
            finally:
                if supervisor.poll() is None:
                    supervisor.kill()
                    supervisor.wait(timeout=2)
                unrelated.terminate()
                unrelated.wait(timeout=2)


if __name__ == "__main__":
    unittest.main()
