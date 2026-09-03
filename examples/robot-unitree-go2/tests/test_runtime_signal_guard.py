from __future__ import annotations

import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
GUARD = ROOT / "scripts" / "runtime_signal_guard.sh"


class RuntimeSignalGuardTest(unittest.TestCase):
    def _start_waiting_harness(
        self, root: Path
    ) -> tuple[subprocess.Popen[str], Path, Path]:
        ready = root / "ready"
        cleanup = root / "cleanup"
        script = root / "harness.sh"
        script.write_text(
            f"""#!/usr/bin/env bash
set -euo pipefail
source {shlex.quote(str(GUARD))}
ready=$1
cleanup=$2
first_pid=""
second_pid=""
cleanup_runtime() {{
  trap - EXIT INT TERM
  for pid in "$first_pid" "$second_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  printf 'cleanup\n' >> "$cleanup"
}}
sleep 30 & first_pid=$!
sleep 30 & second_pid=$!
go2_runtime_install_cleanup_traps
printf 'ready\n' > "$ready"
go2_runtime_wait_for_first_exit "$first_pid" "$second_pid"
printf 'unexpected continuation: %s %s\n' \
  "${{EXITED_RUNTIME_PID:-unset}}" "${{RUNTIME_STATUS:-unset}}"
""",
            encoding="utf-8",
        )
        script.chmod(0o700)
        process = subprocess.Popen(
            [str(script), str(ready), str(cleanup)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 5
        while not ready.exists() and process.poll() is None:
            if time.monotonic() >= deadline:
                process.kill()
                self.fail("runtime wait harness did not become ready")
            time.sleep(0.01)
        self.assertIsNone(process.poll(), "runtime wait harness exited early")
        return process, ready, cleanup

    def test_term_during_wait_cleans_once_and_exits_143(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            process, _, cleanup = self._start_waiting_harness(Path(temporary))
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=5)

            self.assertEqual(process.returncode, 143, stderr)
            self.assertEqual(stdout, "")
            self.assertNotIn("unbound variable", stderr)
            self.assertEqual(cleanup.read_text(encoding="utf-8"), "cleanup\n")

    def test_int_during_wait_cleans_once_and_exits_130(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            process, _, cleanup = self._start_waiting_harness(Path(temporary))
            os.kill(process.pid, signal.SIGINT)
            stdout, stderr = process.communicate(timeout=5)

            self.assertEqual(process.returncode, 130, stderr)
            self.assertEqual(stdout, "")
            self.assertNotIn("unbound variable", stderr)
            self.assertEqual(cleanup.read_text(encoding="utf-8"), "cleanup\n")

    def test_natural_child_exit_reports_identity_and_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cleanup = root / "cleanup"
            script = root / "natural-exit.sh"
            script.write_text(
                f"""#!/usr/bin/env bash
set -euo pipefail
source {shlex.quote(str(GUARD))}
cleanup=$1
first_pid=""
second_pid=""
cleanup_runtime() {{
  trap - EXIT INT TERM
  for pid in "$first_pid" "$second_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  printf 'cleanup\n' >> "$cleanup"
}}
bash -c 'exit 17' & first_pid=$!
sleep 30 & second_pid=$!
go2_runtime_install_cleanup_traps
go2_runtime_wait_for_first_exit "$first_pid" "$second_pid"
printf '%s %s\n' "$EXITED_RUNTIME_PID" "$RUNTIME_STATUS"
""",
                encoding="utf-8",
            )
            script.chmod(0o700)

            result = subprocess.run(
                [str(script), str(cleanup)],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            exited_pid, status = result.stdout.strip().split()
            self.assertTrue(exited_pid.isdigit())
            self.assertEqual(status, "17")
            self.assertEqual(cleanup.read_text(encoding="utf-8"), "cleanup\n")


if __name__ == "__main__":
    unittest.main()
