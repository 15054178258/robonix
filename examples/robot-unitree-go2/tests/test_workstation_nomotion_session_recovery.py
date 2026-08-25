from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest
import uuid


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


def start_ticks(pid: int) -> int:
    fields = Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1].split()
    return int(fields[19])


class CorrectedSessionRecoveryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.deploy = Path(self.temporary.name) / "robot-unitree-go2"
        self.scripts = self.deploy / "scripts"
        self.run_root = self.deploy / "rbnx-build" / "run"
        self.scripts.mkdir(parents=True)
        self.run_root.mkdir(parents=True)
        self.run_root.chmod(0o700)
        for name in (
            "workstation_nomotion_session.sh",
            "stop_workstation_full_nomotion_corrected.sh",
            "recover_workstation_full_nomotion_corrected.sh",
        ):
            shutil.copy2(SCRIPTS / name, self.scripts / name)
            (self.scripts / name).chmod(0o755)
        self.holder: subprocess.Popen[str] | None = None

    def tearDown(self) -> None:
        if self.holder is not None and self.holder.poll() is None:
            self.holder.terminate()
            self.holder.wait(timeout=3)
        self.temporary.cleanup()

    def make_metadata(self, pid: int, ticks: int, *, mode: int = 0o600) -> Path:
        run_dir = self.run_root / "workstation-nomotion-stamp.A1b2C3"
        run_dir.mkdir(exist_ok=True)
        run_dir.chmod(0o700)
        content = "\n".join(
            (
                "format=go2-workstation-nomotion-session-v1",
                f"token={uuid.uuid4()}",
                f"wrapper_pid={pid}",
                f"wrapper_start_ticks={ticks}",
                f"run_dir={run_dir}",
                "",
            )
        )
        session = run_dir / "session.meta"
        pointer = self.run_root / "workstation-nomotion-current.session"
        session.write_text(content)
        pointer.write_text(content)
        session.chmod(0o600)
        pointer.chmod(mode)
        return pointer

    def start_lock_holder(self) -> subprocess.Popen[str]:
        lock = self.run_root / "workstation-nomotion-stamp.lock"
        ready = self.run_root / "holder.ready"
        self.holder = subprocess.Popen(
            [
                "bash",
                "-c",
                'exec 9>"$1"; chmod 600 "$1"; flock -x 9; '
                'printf ready > "$2"; trap "exit 0" TERM; '
                "while :; do sleep 0.05; done",
                "bash",
                str(lock),
                str(ready),
            ],
            text=True,
        )
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(ready.exists())
        return self.holder

    def run_script(self, name: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["bash", str(self.scripts / name)],
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )

    def test_exact_pid_start_ticks_and_lock_are_required_before_term(self) -> None:
        holder = self.start_lock_holder()
        pointer = self.make_metadata(holder.pid, start_ticks(holder.pid))
        result = self.run_script("stop_workstation_full_nomotion_corrected.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("TERM sent", result.stdout)
        holder.wait(timeout=3)
        recovered = self.run_script("recover_workstation_full_nomotion_corrected.sh")
        self.assertEqual(recovered.returncode, 0, recovered.stderr)
        self.assertFalse(pointer.exists())

    def test_start_ticks_mismatch_refuses_without_signalling(self) -> None:
        holder = self.start_lock_holder()
        self.make_metadata(holder.pid, start_ticks(holder.pid) + 1)
        result = self.run_script("stop_workstation_full_nomotion_corrected.sh")
        self.assertEqual(result.returncode, 74)
        self.assertIn("identity mismatch", result.stderr)
        self.assertIsNone(holder.poll())

    def test_wrong_mode_and_symlink_pointer_are_rejected(self) -> None:
        self.make_metadata(99999999, 1, mode=0o644)
        result = self.run_script("recover_workstation_full_nomotion_corrected.sh")
        self.assertEqual(result.returncode, 74)
        pointer = self.run_root / "workstation-nomotion-current.session"
        pointer.unlink()
        pointer.symlink_to(
            self.run_root / "workstation-nomotion-stamp.A1b2C3" / "session.meta"
        )
        result = self.run_script("recover_workstation_full_nomotion_corrected.sh")
        self.assertEqual(result.returncode, 74)

    def test_absent_process_recovery_removes_only_pointer(self) -> None:
        pointer = self.make_metadata(99999999, 1)
        session = pointer.parent / "workstation-nomotion-stamp.A1b2C3" / "session.meta"
        result = self.run_script("recover_workstation_full_nomotion_corrected.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(pointer.exists())
        self.assertTrue(session.exists(), "RUN_DIR evidence must be preserved")

    def test_failure_trap_removes_only_its_byte_identical_pointer(self) -> None:
        pointer = self.make_metadata(99999999, 1)
        session = pointer.parent / "workstation-nomotion-stamp.A1b2C3" / "session.meta"
        index_lock = self.run_root / "workstation-nomotion-session-index.lock"
        result = subprocess.run(
            [
                "bash",
                "-c",
                'set -euo pipefail; source "$1"; '
                'trap \'go2_nomotion_remove_pointer_if_owned "$2" "$3" "$4" '
                '"$(id -u)"\' EXIT; false',
                "bash",
                str(self.scripts / "workstation_nomotion_session.sh"),
                str(pointer),
                str(session),
                str(index_lock),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(pointer.exists())
        self.assertTrue(session.exists())

    def test_launcher_arms_cleanup_before_any_post_registration_failure(self) -> None:
        source = (SCRIPTS / "start_workstation_full_nomotion_corrected.sh").read_text()
        registration = source.index(
            '"$CURRENT_SESSION" "$SESSION_TOKEN" "$$" "$WRAPPER_START_TICKS"'
        )
        trap = source.index("trap on_early_exit EXIT")
        config = source.index('RUNTIME_CONFIG_DIR="$RUN_DIR/config"')
        self.assertLess(registration, trap)
        self.assertLess(trap, config)
        self.assertIn("trap 'on_signal 129' HUP", source)

    def test_launcher_closes_discipline_fd_in_all_long_lived_children(self) -> None:
        source = (SCRIPTS / "start_workstation_full_nomotion_corrected.sh").read_text()
        self.assertIn("workstation_nomotion_stamp_node.py\" \\\n    --mode affine", source)
        close = 'go2_nomotion_close_inherited_fd "$DISCIPLINE_LOCK_FD"'
        self.assertEqual(source.count(close), 4)
        for executable in (
            "workstation_nomotion_identity_monitor.py",
            "workstation_nomotion_stamp_node.py",
            "workstation_nomotion_cloud_relay.py",
            'exec bash "$ROOT/start.sh"',
        ):
            child = source.rfind("(\n", 0, source.index(executable))
            self.assertGreaterEqual(child, 0)
            self.assertIn(close, source[child : source.index(executable)])

    def test_child_close_helper_preserves_parent_lock_but_not_exec_child_fd(self) -> None:
        lock = self.run_root / "fd-inheritance.lock"
        result = subprocess.run(
            [
                "bash",
                "-c",
                r'''
set -euo pipefail
source "$1"
lock="$2"
exec {discipline_fd}>"$lock"
flock --exclusive "$discipline_fd"
expected="$(stat -Lc '%d:%i' -- "$lock")"

# Once the child closes its inherited copy, a separately opened descriptor
# must still be blocked by the parent wrapper's lock.
(
  go2_nomotion_close_inherited_fd "$discipline_fd"
  if flock --exclusive --nonblock "$lock" true; then
    exit 70
  fi
)

# The same close must survive the real exec boundary used by the launcher.
(
  go2_nomotion_close_inherited_fd "$discipline_fd"
  exec bash -c '
    expected="$1"
    for candidate in /proc/$$/fd/*; do
      [[ -e "$candidate" ]] || continue
      [[ "$(stat -Lc "%d:%i" -- "$candidate" 2>/dev/null || true)" != "$expected" ]] || exit 71
    done
  ' bash "$expected"
)
''',
                "bash",
                str(self.scripts / "workstation_nomotion_session.sh"),
                str(lock),
            ],
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_stop_surfaces_have_no_name_kill_or_force_kill(self) -> None:
        source = "\n".join(
            (SCRIPTS / name).read_text()
            for name in (
                "stop_workstation_full_nomotion_corrected.sh",
                "recover_workstation_full_nomotion_corrected.sh",
            )
        )
        for forbidden in ("pkill", "killall", "kill -KILL", "kill -9", "/cmdline"):
            self.assertNotIn(forbidden, source)
        self.assertIn('kill -TERM "$GO2_SESSION_PID"', source)


if __name__ == "__main__":
    unittest.main()
