#!/usr/bin/env python3
"""Offline tests for the time-probe wrapper's ROS environment bootstrap."""

from __future__ import annotations

import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts" / "probe_go2_time_readonly.sh"


class TimeProbeWrapperBootstrapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = WRAPPER.read_text(encoding="utf-8")
        (ROOT / "logs").mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _write_executable(path: Path, source: str) -> None:
        path.write_text(source, encoding="utf-8")
        path.chmod(path.stat().st_mode | stat.S_IXUSR)

    def _run_with_mocked_timeout(
        self,
        temporary: Path,
        ros_setup_source: str,
        unitree_setup_source: str | None,
    ) -> tuple[subprocess.CompletedProcess[str], Path, Path]:
        ros_setup = temporary / "ros-setup.bash"
        unitree_setup = temporary / "unitree-setup.bash"
        timeout_marker = temporary / "timeout-called"
        unitree_marker = temporary / "unitree-sourced"
        fake_bin = temporary / "bin"
        output = temporary / "evidence"
        fake_bin.mkdir()
        ros_setup.write_text(ros_setup_source, encoding="utf-8")
        if unitree_setup_source is not None:
            unitree_setup.write_text(unitree_setup_source, encoding="utf-8")
        self._write_executable(
            fake_bin / "timeout",
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            ': "${ROS_SETUP_SENTINEL:?ROS setup was not sourced}"\n'
            ': "${UNITREE_SETUP_SENTINEL:?Unitree setup was not sourced}"\n'
            'printf "called\\n" > "${TIMEOUT_MARKER}"\n',
        )
        environment = os.environ.copy()
        environment.update(
            {
                "PATH": f"{fake_bin}:{environment['PATH']}",
                "ROS_SETUP_FILE": str(ros_setup),
                "UNITREE_ROS2_SETUP": str(unitree_setup),
                "TIMEOUT_MARKER": str(timeout_marker),
                "UNITREE_MARKER": str(unitree_marker),
            }
        )
        result = subprocess.run(
            ["bash", str(WRAPPER), str(output), "1"],
            cwd=ROOT,
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10,
            check=False,
        )
        return result, timeout_marker, unitree_marker

    def test_defaults_are_ros_humble_and_workspace_unitree_overlay(self) -> None:
        self.assertIn(
            'readonly ROS_SETUP="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"',
            self.source,
        )
        self.assertIn(
            'readonly UNITREE_SETUP="${UNITREE_ROS2_SETUP:-${PROJECT_ROOT}/'
            'rbnx-build/unitree_ros2/install/setup.bash}"',
            self.source,
        )
        self.assertLess(
            self.source.index('source_setup_or_exit "ROS 2" "${ROS_SETUP}"'),
            self.source.index(
                'source_setup_or_exit "Unitree ROS 2 overlay" "${UNITREE_SETUP}"'
            ),
        )
        self.assertLess(
            self.source.index(
                'source_setup_or_exit "Unitree ROS 2 overlay" "${UNITREE_SETUP}"'
            ),
            self.source.index("timeout --signal=TERM"),
        )
        source_index = self.source.index('if source "${setup_file}"; then')
        self.assertLess(self.source.rfind("set +u", 0, source_index), source_index)
        self.assertLess(source_index, self.source.index("set -u", source_index))

    def test_mocked_setups_are_sourced_before_probe_with_nounset_restored(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            temporary = Path(directory)
            result, timeout_marker, unitree_marker = self._run_with_mocked_timeout(
                temporary,
                "[[ ${ROS_SETUP_UNSET_FOR_TEST} == never ]] || true\n"
                "export ROS_SETUP_SENTINEL=ready\n",
                "[[ ${UNITREE_SETUP_UNSET_FOR_TEST} == never ]] || true\n"
                ': "${ROS_SETUP_SENTINEL:?ROS setup must run first}"\n'
                'printf "sourced\\n" > "${UNITREE_MARKER}"\n'
                "export UNITREE_SETUP_SENTINEL=ready\n",
            )
            self.assertEqual(result.returncode, 0, (result.stdout, result.stderr))
            self.assertTrue(timeout_marker.exists())
            self.assertTrue(unitree_marker.exists())

    def test_setup_failure_is_clear_and_probe_never_starts(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            temporary = Path(directory)
            result, timeout_marker, unitree_marker = self._run_with_mocked_timeout(
                temporary,
                "return 23\n",
                'printf "sourced\\n" > "${UNITREE_MARKER}"\n'
                "export UNITREE_SETUP_SENTINEL=ready\n",
            )
            self.assertEqual(result.returncode, 127, (result.stdout, result.stderr))
            self.assertIn("无法加载 ROS 2 环境脚本（状态 23）", result.stderr)
            self.assertIn("未创建任何 ROS 订阅", result.stderr)
            self.assertFalse(timeout_marker.exists())
            self.assertFalse(unitree_marker.exists())

    def test_missing_overlay_is_clear_and_probe_never_starts(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as directory:
            temporary = Path(directory)
            result, timeout_marker, _unitree_marker = self._run_with_mocked_timeout(
                temporary,
                "export ROS_SETUP_SENTINEL=ready\n",
                None,
            )
            self.assertEqual(result.returncode, 127, (result.stdout, result.stderr))
            self.assertIn("缺少 Unitree ROS 2 overlay 环境脚本", result.stderr)
            self.assertIn("未创建任何 ROS 订阅", result.stderr)
            self.assertFalse(timeout_marker.exists())

    def test_wrapper_remains_subscription_only_and_bounded(self) -> None:
        self.assertIn("GO2_TIME_PROBE_DURATION_SECONDS:-75", self.source)
        for forbidden in (
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "create_publisher",
            "ros2 topic pub",
            "SportClient",
        ):
            self.assertNotIn(forbidden, self.source)
        self.assertIn(
            "timeout --signal=TERM --kill-after=15s --preserve-status",
            self.source,
        )


if __name__ == "__main__":
    unittest.main()
