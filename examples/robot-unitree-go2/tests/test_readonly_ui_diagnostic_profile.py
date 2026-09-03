from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "start_readonly_ui_diagnostic.sh"
CHECKER = ROOT / "scripts" / "check_readonly_ui_diagnostic_ownership.sh"


class ReadonlyUiDiagnosticProfileTest(unittest.TestCase):
    def test_static_scope_is_readonly_and_explicitly_not_navigation_ready(self) -> None:
        launcher = LAUNCHER.read_text(encoding="utf-8")
        checker = CHECKER.read_text(encoding="utf-8")
        combined = f"{launcher}\n{checker}"
        self.assertIn("READ-ONLY DIAGNOSTIC / NOT NAVIGATION READY", launcher)
        self.assertIn("GO2_DASHBOARD_PROFILE=readonly-diagnostic", launcher)
        self.assertIn("GO2_DASHBOARD_BROWSER_VOICE_ENABLED=0", launcher)
        self.assertIn("GO2_DASHBOARD_CLOUD_TOPIC=/utlidar/cloud", launcher)
        self.assertIn("GO2_DASHBOARD_ODOM_TOPIC=/utlidar/robot_odom", launcher)
        self.assertIn('GO2_DASHBOARD_HOST=127.0.0.1', launcher)
        self.assertIn('bash "$ownership_checker" pre', launcher)
        self.assertLess(
            launcher.index('bash "$ownership_checker" pre'),
            launcher.index("start_child camera-daemon"),
        )
        self.assertIn('kill -TERM -- "-${child_pgids[$index]}"', launcher)
        self.assertIn('kill -KILL -- "-${child_pgids[$index]}"', launcher)
        self.assertNotIn("pkill", combined)
        for forbidden in (
            "ros2 topic pub",
            "ros2 action send_goal",
            "sport_mode_ctrl",
            "go2_sport_client",
            "go2_stand_example",
            "low_level_ctrl",
        ):
            self.assertNotIn(forbidden, combined)
        self.assertNotRegex(combined, r"(?m)^\s*(?:/cmd_vel|/lowcmd|/api/sport/request)\s*$")
        self.assertNotIn("go2_sensor_relay", launcher)
        for absent_surface in ("/scanner/cloud", "/odom", "/tf_static", "/map"):
            self.assertIn(absent_surface, checker)
        self.assertIn("--no-daemon", checker)
        self.assertIn("timeout --signal=INT", checker)

    def _fake_ros_environment(self, directory: Path) -> dict[str, str]:
        fake_bin = directory / "bin"
        fake_bin.mkdir()
        fake_ros2 = fake_bin / "ros2"
        fake_ros2.write_text(
            "#!/usr/bin/env bash\n"
            "topic=\"${@: -1}\"\n"
            "case \"$topic\" in\n"
            "  /frontvideostream|/utlidar/cloud|/utlidar/robot_odom) count=${RAW_COUNT:-1} ;;\n"
            "  /camera/color/image_raw|/camera/color/camera_info) count=${CAMERA_COUNT:-0} ;;\n"
            "  /go2/sensors/status) count=${STATUS_COUNT:-0} ;;\n"
            "  *) count=${NAV_SURFACE_COUNT:-0} ;;\n"
            "esac\n"
            "printf 'Type: offline/Fake\\nPublisher count: %s\\nSubscription count: 0\\n' \"$count\"\n",
            encoding="utf-8",
        )
        fake_ros2.chmod(0o755)
        env = os.environ.copy()
        env.update(
            {
                "PATH": f"{fake_bin}:{env['PATH']}",
                "GO2_DIAGNOSTIC_DISCOVERY_TIMEOUT_S": "2",
                "GO2_DIAGNOSTIC_STABILITY_SAMPLES": "1",
                "GO2_DIAGNOSTIC_QUERY_TIMEOUT_S": "1",
            }
        )
        return env

    def test_graph_gate_requires_raw_sources_and_exclusive_camera_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            env = self._fake_ros_environment(Path(temporary))
            pre = subprocess.run(
                [str(CHECKER), "pre"], env=env, text=True, capture_output=True
            )
            self.assertEqual(pre.returncode, 0, pre.stderr)
            env.update({"CAMERA_COUNT": "1", "STATUS_COUNT": "1"})
            post = subprocess.run(
                [str(CHECKER), "post"], env=env, text=True, capture_output=True
            )
            self.assertEqual(post.returncode, 0, post.stderr)
            env["CAMERA_COUNT"] = "2"
            duplicate = subprocess.run(
                [str(CHECKER), "post"], env=env, text=True, capture_output=True
            )
            self.assertEqual(duplicate.returncode, 5)
            self.assertIn("publisher ownership violation", duplicate.stderr)

    def test_lifecycle_uses_exclusive_lease_and_cleans_exact_child_groups(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            deploy = Path(temporary) / "robot-unitree-go2"
            scripts = deploy / "scripts"
            sensors = deploy / "packages" / "go2_sensors"
            dashboard = deploy / "packages" / "go2_dashboard"
            markers = deploy / "markers"
            fake_bin = deploy / "fake-bin"
            scripts.mkdir(parents=True)
            markers.mkdir()
            fake_bin.mkdir()
            shutil.copy2(LAUNCHER, scripts / LAUNCHER.name)
            shutil.copy2(ROOT / "scripts" / "runtime_lease.sh", scripts / "runtime_lease.sh")

            checker = scripts / CHECKER.name
            checker.write_text(
                "#!/usr/bin/env bash\n"
                "set -euo pipefail\n"
                "phase=$1\n"
                "if [[ $phase == pre ]]; then\n"
                "  compgen -G \"$FAKE_MARKERS/*.started\" >/dev/null && exit 91\n"
                "  printf pre > \"$FAKE_MARKERS/pre.ok\"\n"
                "else\n"
                "  [[ $(find \"$FAKE_MARKERS\" -name '*.started' | wc -l) -eq 3 ]]\n"
                "  printf post > \"$FAKE_MARKERS/post.ok\"\n"
                "fi\n",
                encoding="utf-8",
            )
            checker.chmod(0o755)

            child_source = (
                "#!/usr/bin/env bash\n"
                "set -u\n"
                "name=${0##*/}\n"
                "touch \"$FAKE_MARKERS/$name.started\"\n"
                "trap 'touch \"$FAKE_MARKERS/$name.stopped\"; exit 0' TERM INT HUP\n"
                "while :; do sleep 0.1; done\n"
            )
            daemon = sensors / ".build" / "camera-daemon" / "install" / "bin" / "go2_camera_daemon"
            bridge = (
                sensors
                / ".build"
                / "ros"
                / "install"
                / "go2_sensors"
                / "lib"
                / "go2_sensors"
                / "go2_camera_bridge"
            )
            python = dashboard / "rbnx-build" / "venv" / "bin" / "python"
            for executable in (daemon, bridge, python):
                executable.parent.mkdir(parents=True, exist_ok=True)
                executable.write_text(child_source, encoding="utf-8")
                executable.chmod(0o755)
            private_lib = sensors / ".build" / "camera-daemon" / "install" / "lib"
            private_lib.mkdir(parents=True, exist_ok=True)
            for library in ("libddsc.so", "libddsc.so.0", "libddscxx.so", "libddscxx.so.0"):
                (private_lib / library).write_text("offline fixture", encoding="utf-8")
            overlay = sensors / ".build" / "ros" / "install" / "setup.bash"
            overlay.parent.mkdir(parents=True, exist_ok=True)
            overlay.write_text("# offline fixture\n", encoding="utf-8")
            config = sensors / "config" / "go2_sensors.yaml"
            config.parent.mkdir(parents=True)
            config.write_text("go2_camera_bridge: {}\n", encoding="utf-8")
            ros_setup = deploy / "ros-setup.bash"
            ros_setup.write_text("# offline fixture\n", encoding="utf-8")
            fake_ros2 = fake_bin / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            fake_ros2.chmod(0o755)

            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{fake_bin}:{env['PATH']}",
                    "FAKE_MARKERS": str(markers),
                    "GO2_NETWORK_INTERFACE": "offline0",
                    "GO2_DASHBOARD_PORT": "18092",
                    "ROS_SETUP_FILE": str(ros_setup),
                }
            )
            first = subprocess.Popen(
                ["bash", str(scripts / LAUNCHER.name)],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            child_lease = deploy / "rbnx-build" / "run" / "dui" / "children.lease"
            post_ok = markers / "post.ok"
            deadline = time.monotonic() + 5
            while (not child_lease.exists() or not post_ok.exists()) and time.monotonic() < deadline:
                if first.poll() is not None:
                    break
                time.sleep(0.02)
            if not child_lease.exists() or not post_ok.exists():
                stdout, stderr = first.communicate(timeout=2)
                self.fail(f"diagnostic launcher failed: {stdout!r} {stderr!r}")
            lease_text = child_lease.read_text(encoding="utf-8")
            pids = [int(value) for value in re.findall(r"(?m)^child_\d+_pid=(\d+)$", lease_text)]
            pgids = [int(value) for value in re.findall(r"(?m)^child_\d+_pgid=(\d+)$", lease_text)]
            self.assertEqual(len(pids), 3)
            self.assertEqual(pids, pgids)

            second = subprocess.run(
                ["bash", str(scripts / LAUNCHER.name)],
                env=env,
                text=True,
                capture_output=True,
                timeout=5,
            )
            self.assertEqual(second.returncode, 73, second.stderr)

            first.terminate()
            first.wait(timeout=8)
            if first.stdout is not None:
                first.stdout.close()
            if first.stderr is not None:
                first.stderr.close()
            self.assertFalse(child_lease.exists())
            for name in ("go2_camera_daemon", "go2_camera_bridge", "python"):
                self.assertTrue((markers / f"{name}.stopped").exists(), name)
            for pid in pids:
                self.assertFalse(Path(f"/proc/{pid}").exists(), f"child PID {pid} leaked")


if __name__ == "__main__":
    unittest.main()
