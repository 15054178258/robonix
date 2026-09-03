from __future__ import annotations

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
LEASE = ROOT / "scripts" / "runtime_lease.sh"


class RuntimeLeaseTest(unittest.TestCase):
    @staticmethod
    def _prepare_workspace_config(workspace: Path) -> None:
        config_root = workspace / ".tools" / "robonix-home"
        source = workspace / "upstream" / "robonix-go2-build"
        config_root.mkdir(parents=True)
        source.mkdir(parents=True)
        (config_root / "config.yaml").write_text(
            f"robonix_source_path: {source}\n",
            encoding="utf-8",
        )

    def test_lock_is_atomic_and_recovers_after_owner_exit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            first = subprocess.Popen(
                [
                    "bash",
                    "-c",
                    'source "$1"; go2_runtime_lease_acquire "$2" workstation-local; '
                    'printf ready > "$2/ready"; sleep 30',
                    "bash",
                    str(LEASE),
                    temporary,
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            ready = Path(temporary) / "ready"
            deadline = time.monotonic() + 3.0
            while not ready.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertTrue(ready.exists())
            blocked = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1"; go2_runtime_lease_acquire "$2" workstation-local',
                    "bash",
                    str(LEASE),
                    temporary,
                ],
                capture_output=True,
                text=True,
                check=False,
                timeout=5,
            )
            self.assertEqual(blocked.returncode, 73, blocked.stderr)
            first.terminate()
            first.wait(timeout=5)
            if first.stdout is not None:
                first.stdout.close()
            if first.stderr is not None:
                first.stderr.close()

            recovered = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1"; go2_runtime_lease_acquire "$2" workstation-local',
                    "bash",
                    str(LEASE),
                    temporary,
                ],
                capture_output=True,
                text=True,
                check=False,
                timeout=5,
            )
            self.assertEqual(recovered.returncode, 0, recovered.stderr)
            metadata = (Path(temporary) / "runtime-placement.lease").read_text()
            self.assertIn("format=go2-runtime-lease-v1\n", metadata)
            self.assertRegex(metadata, r"(?m)^owner_start_ticks=[0-9]+$")

    def test_surviving_child_does_not_inherit_parent_only_locks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runtime_root = Path(temporary)
            semantic_lock = runtime_root / "semantic-intent-router.lock"
            child_pid_path = runtime_root / "surviving-child.pid"
            launcher = subprocess.run(
                [
                    "bash",
                    "-c",
                    r'''
set -euo pipefail
source "$1"
go2_runtime_lease_acquire "$2" workstation-local
exec {SEMANTIC_ROUTER_LOCK_FD}>"$3"
flock --exclusive --nonblock "$SEMANTIC_ROUTER_LOCK_FD"
(
  go2_runtime_close_parent_only_fds
  printf '%s\n' "$BASHPID" > "$4"
  exec sleep 30
) </dev/null >/dev/null 2>&1 &
deadline=$((SECONDS + 3))
while [[ ! -s "$4" && $SECONDS -le $deadline ]]; do
  sleep 0.01
done
[[ -s "$4" ]]
''',
                    "bash",
                    str(LEASE),
                    str(runtime_root),
                    str(semantic_lock),
                    str(child_pid_path),
                ],
                capture_output=True,
                text=True,
                check=False,
                timeout=5,
            )
            self.assertEqual(launcher.returncode, 0, launcher.stderr)
            child_pid = int(child_pid_path.read_text(encoding="utf-8").strip())
            try:
                os.kill(child_pid, 0)
                for lock_path in (
                    runtime_root / "runtime-placement.lock",
                    semantic_lock,
                ):
                    probe = subprocess.run(
                        ["flock", "--exclusive", "--nonblock", str(lock_path), "true"],
                        capture_output=True,
                        text=True,
                        check=False,
                        timeout=5,
                    )
                    self.assertEqual(
                        probe.returncode,
                        0,
                        f"surviving child still holds {lock_path}: {probe.stderr}",
                    )
            finally:
                try:
                    os.kill(child_pid, 15)
                except ProcessLookupError:
                    pass

    def test_full_launcher_closes_parent_only_fds_before_managed_execs(self) -> None:
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        semantic_spawn = '''(
    go2_runtime_close_parent_only_fds
    exec bash "$DEPLOY_DIR/packages/semantic_intent_router/scripts/start.sh"
  ) &'''
        boot_spawn = '''(
  go2_runtime_close_parent_only_fds
  exec "$RBNX_CLI" boot --no-update-check -f "$MANIFEST" "$@"
) &'''
        self.assertIn(semantic_spawn, start)
        self.assertIn(boot_spawn, start)
        self.assertEqual(start.count("go2_runtime_close_parent_only_fds"), 2)

    def test_malformed_active_ui_lease_does_not_skip_robonix_shutdown(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            deploy = workspace / "packages" / "robot-unitree-go2"
            (deploy / "scripts").mkdir(parents=True)
            (deploy / "rbnx-build" / "run").mkdir(parents=True)
            (deploy / "rbnx-boot").mkdir()
            self._prepare_workspace_config(workspace)
            (deploy / "rbnx-boot" / "state.json").write_text("{}\n")
            (deploy / "robonix_manifest.yaml").write_text("manifestVersion: 1\n")
            shutil.copy2(ROOT / "stop.sh", deploy / "stop.sh")
            shutil.copy2(LEASE, deploy / "scripts" / "runtime_lease.sh")
            shutil.copy2(
                ROOT / "scripts" / "validate_robonix_home.py",
                deploy / "scripts" / "validate_robonix_home.py",
            )

            lock = deploy / "rbnx-build" / "run" / "ui-client-dashboard.lock"
            malformed = deploy / "rbnx-build" / "run" / "ui-client-dashboard.lease"
            malformed.write_text("not-a-valid-lease\n")
            holder = subprocess.Popen(
                ["flock", str(lock), "sleep", "30"],
            )
            time.sleep(0.1)
            try:
                fake_bin = Path(temporary) / "bin"
                fake_bin.mkdir()
                called = Path(temporary) / "rbnx-called"
                fake_rbnx = fake_bin / "rbnx"
                fake_rbnx.write_text(
                    "#!/usr/bin/env bash\nprintf '%s\\n' \"$*\" > \"$RBNX_CALLED\"\n",
                    encoding="utf-8",
                )
                fake_rbnx.chmod(0o755)
                env = os.environ.copy()
                env["PATH"] = f"{fake_bin}:{env['PATH']}"
                env["RBNX_CALLED"] = str(called)
                result = subprocess.run(
                    ["bash", str(deploy / "stop.sh")],
                    capture_output=True,
                    text=True,
                    check=False,
                    env=env,
                    timeout=5,
                )
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("missing/malformed metadata", result.stderr)
                self.assertTrue(called.exists(), "rbnx shutdown was skipped")
                self.assertIn("shutdown", called.read_text())
                self.assertIsNone(holder.poll(), "unvalidated lock holder was killed")
            finally:
                holder.terminate()
                holder.wait(timeout=5)

    def test_validated_ui_lock_owner_is_stopped_without_cmdline_matching(self) -> None:
        build_root = ROOT / "rbnx-build" / "lease-test"
        build_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=build_root) as temporary:
            workspace = Path(temporary) / "workspace"
            deploy = workspace / "packages" / "robot-unitree-go2"
            (deploy / "scripts").mkdir(parents=True)
            self._prepare_workspace_config(workspace)
            dashboard = deploy / "packages" / "go2_dashboard"
            (dashboard / "go2_dashboard").mkdir(parents=True)
            venv_bin = dashboard / "rbnx-build" / "venv" / "bin"
            venv_bin.mkdir(parents=True)
            (venv_bin / "python").symlink_to(Path(os.sys.executable))
            (dashboard / "go2_dashboard" / "__init__.py").write_text("")
            (dashboard / "go2_dashboard" / "main.py").write_text(
                "import time\ntime.sleep(30)\n", encoding="utf-8"
            )
            shutil.copy2(LEASE, deploy / "scripts" / "runtime_lease.sh")
            shutil.copy2(
                ROOT / "scripts" / "validate_robonix_home.py",
                deploy / "scripts" / "validate_robonix_home.py",
            )
            shutil.copy2(
                ROOT / "scripts" / "start_ui_client_only.sh",
                deploy / "scripts" / "start_ui_client_only.sh",
            )
            shutil.copy2(ROOT / "stop.sh", deploy / "stop.sh")
            checker = deploy / "scripts" / "check_runtime_ownership.sh"
            checker.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            checker.chmod(0o755)
            env = os.environ.copy()
            env["GO2_RUNTIME_PLACEMENT"] = "workstation-ui-nx-full"
            launcher = subprocess.Popen(
                ["bash", str(deploy / "scripts" / "start_ui_client_only.sh")],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            lease = deploy / "rbnx-build" / "run" / "ui-client-dashboard.lease"
            deadline = time.monotonic() + 5.0
            while not lease.exists() and time.monotonic() < deadline:
                if launcher.poll() is not None:
                    break
                time.sleep(0.02)
            if not lease.exists():
                stdout, stderr = launcher.communicate(timeout=2)
                self.fail(
                    f"UI lease was not created (exit={launcher.returncode}): "
                    f"stdout={stdout!r} stderr={stderr!r}"
                )
            result = subprocess.run(
                ["bash", str(deploy / "stop.sh")],
                capture_output=True,
                text=True,
                check=False,
                env=env,
                timeout=12,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("UI/client-only dashboard stopped", result.stdout)
            launcher.wait(timeout=5)
            if launcher.stdout is not None:
                launcher.stdout.close()
            if launcher.stderr is not None:
                launcher.stderr.close()
            self.assertFalse(lease.exists())

    def test_validated_semantic_router_child_is_stopped_by_exact_lease(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            deploy = workspace / "packages" / "robot-unitree-go2"
            scripts = deploy / "scripts"
            run_root = deploy / "rbnx-build" / "run"
            scripts.mkdir(parents=True)
            run_root.mkdir(parents=True)
            self._prepare_workspace_config(workspace)
            shutil.copy2(ROOT / "stop.sh", deploy / "stop.sh")
            shutil.copy2(LEASE, scripts / "runtime_lease.sh")
            shutil.copy2(
                ROOT / "scripts" / "validate_robonix_home.py",
                scripts / "validate_robonix_home.py",
            )
            lock = run_root / "semantic-intent-router.lock"
            lease = run_root / "semantic-intent-router.lease"
            ready = run_root / "semantic-ready"
            holder_script = scripts / "semantic-holder.sh"
            holder_script.write_text(
                """#!/usr/bin/env bash
set -u
lock="$1"
lease="$2"
ready="$3"
exec {lock_fd}>"$lock"
flock --exclusive "$lock_fd"
sleep 30 &
child=$!
owner_start="$(awk '{print $22}' "/proc/$$/stat")"
child_start="$(awk '{print $22}' "/proc/$child/stat")"
{
  printf 'format=go2-semantic-router-lease-v1\\n'
  printf 'token=00000000-0000-0000-0000-000000000000\\n'
  printf 'owner_pid=%s\\n' "$$"
  printf 'owner_start_ticks=%s\\n' "$owner_start"
  printf 'child_pid=%s\\n' "$child"
  printf 'child_start_ticks=%s\\n' "$child_start"
} > "$lease"
printf ready > "$ready"
wait "$child" 2>/dev/null || true
rm -f -- "$lease"
""",
                encoding="utf-8",
            )
            holder_script.chmod(0o755)
            holder = subprocess.Popen(
                ["bash", str(holder_script), str(lock), str(lease), str(ready)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            deadline = time.monotonic() + 3.0
            while not ready.exists() and time.monotonic() < deadline:
                if holder.poll() is not None:
                    break
                time.sleep(0.02)
            if not ready.exists():
                stdout, stderr = holder.communicate(timeout=2)
                self.fail(
                    f"semantic holder did not become ready (exit={holder.returncode}): "
                    f"stdout={stdout!r} stderr={stderr!r}"
                )
            result = subprocess.run(
                ["bash", str(deploy / "stop.sh")],
                capture_output=True,
                text=True,
                check=False,
                timeout=12,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("semantic intent router stopped", result.stdout)
            holder.wait(timeout=5)
            if holder.stdout is not None:
                holder.stdout.close()
            if holder.stderr is not None:
                holder.stderr.close()
            self.assertFalse(lease.exists())


if __name__ == "__main__":
    unittest.main()
