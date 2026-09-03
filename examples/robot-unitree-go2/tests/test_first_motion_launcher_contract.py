from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts/start_first_motion_corrected.sh"
RPC_BASELINE_CAPTURE = ROOT / "scripts/capture_motion_rpc_graph_baseline.py"


class FirstMotionLauncherContractTest(unittest.TestCase):
    def test_provider_and_launcher_share_robot_root_evidence_scope(self) -> None:
        launcher = LAUNCHER.read_text(encoding="utf-8")
        provider = (
            ROOT
            / "packages"
            / "go2_chassis"
            / "go2_chassis"
            / "main.py"
        ).read_text(encoding="utf-8")
        self.assertEqual(
            launcher.count('--package-root "$ROOT"'),
            2,
        )
        self.assertIn(
            "consume_first_motion_permit(\n"
            "                    runtime, os.environ, _package_root.parents[1]\n"
            "                )",
            provider,
        )

    def test_launcher_is_one_shot_private_and_fail_closed(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        for required in (
            "I_CONFIRM_GO2_CLEAR_2M_REMOTE_STOP_READY",
            "I_APPROVE_GO2_FIRST_10CM_MOTION",
            "validate_first_motion_state_evidence.py",
            "validate_first_motion_permit_for_launch.py",
            "workstation_nomotion_identity_monitor.py",
            "workstation_nomotion_stamp_node.py",
            "workstation_motion_state_relay.py",
            "packages/go2_motion_state_relay/.build/ros/install",
            "--worker-binary \"$MOTION_STATE_RELAY_BINARY\"",
            "render_workstation_first_motion_manifest.py",
            "capture_motion_rpc_graph_baseline.py",
            "GO2_SPORT_REQUEST_BASELINE_FILE",
            "first_motion_probe.py",
            "go2_runtime_lease_acquire",
            "shutdown_rbnx_boot_state.py",
            "terminate_child \"$PROBE_PID\"",
        ):
            self.assertIn(required, source)
        self.assertIn("0.05 m/s", source)
        self.assertIn("2.0 s", source)
        self.assertIn("0.10 m", source)
        self.assertRegex(
            source,
            re.compile(
                r"workstation_nomotion_stamp_node\.py\" \\\n"
                r"\s+--mode affine \\\n\s+--profile motion"
            ),
        )
        self.assertNotRegex(source, re.compile(r"(^|\s)source\s+\"?\$ROOT/\.env"))
        self.assertNotIn("ros2 topic pub", source)
        self.assertNotIn("ros2 service call", source)
        self.assertNotIn("sport_mode_ctrl", source)
        self.assertNotIn("go2_sport_client", source)
        self.assertNotIn("go2_stand_example", source)
        self.assertNotIn("low_level_ctrl", source)

        baseline_capture = RPC_BASELINE_CAPTURE.read_text(encoding="utf-8")
        self.assertIn("/api/sport/request", baseline_capture)
        self.assertIn("/api/sport_lease/request", baseline_capture)
        self.assertNotIn("create_publisher", baseline_capture)
        self.assertNotIn("create_subscription", baseline_capture)
        self.assertNotIn("unitree_api", baseline_capture)

    def test_launcher_never_uses_canonical_cmd_vel(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        # The banner may say that canonical cmd_vel is isolated, but it must
        # never be exported, published, or passed as an output route.
        self.assertNotIn("ROBONIX_VELOCITY_OUTPUT_TOPIC=/cmd_vel", source)
        self.assertNotRegex(source, re.compile(r"ros2\s+topic\s+pub.*?/cmd_vel"))

    def test_wireless_transport_is_explicit_and_keeps_wired_default(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        self.assertIn(
            'TRANSPORT="${GO2_FIRST_MOTION_TRANSPORT:-wired}"',
            source,
        )
        self.assertIn(
            "I_APPROVE_GO2_WIRELESS_PRIVATE_LAN_FIRST_10CM",
            source,
        )
        self.assertIn(
            '[[ "${GO2_WIRELESS_MOTION_ACK:-}" == "$WIRELESS_ACK" ]]',
            source,
        )
        self.assertIn("validate_first_motion_network.py", source)
        self.assertEqual(
            source.count("first_motion_validate_wireless_topology ||"),
            2,
        )
        self.assertIn(
            "Go2 interface must be a physical wired device",
            source,
        )
        self.assertNotIn("GO2_WIRELESS_MOTION_ACK=true", source)
        self.assertNotIn("nmcli connection up", source)
        self.assertNotIn("nmcli connection modify", source)

    def test_affine_approval_is_preflighted_before_ros_with_90s_deadline(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        approval = source.index("--require-affine")
        ros_source = source.index("source /opt/ros/humble/setup.bash")
        lease = source.index("go2_runtime_lease_acquire")
        self.assertLess(approval, ros_source)
        self.assertLess(approval, lease)
        self.assertIn("deadline=$((SECONDS + 90))", source)
        deadline = source.index("deadline=$((SECONDS + 90))")
        wait_loop = source.index("while true; do", deadline)
        loop_end = source.index("\ndone", wait_loop)
        loop = source[wait_loop:loop_end]
        fault_check = loop.index(
            '"timestamp qualification" "$STAMP_FAULT"'
        )
        ready_accept = loop.index('[[ ! -s "$STAMP_READY" ]] || break')
        timeout_check = source.index('SECONDS < deadline', wait_loop)
        self.assertLess(fault_check, ready_accept)
        self.assertLess(wait_loop + fault_check, timeout_check)

    def test_every_ready_wait_is_unconditional_and_fault_first(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        cases = (
            (
                "$IDENTITY_READY",
                '"writer identity monitor" "$IDENTITY_FAULT"',
            ),
            (
                "$STAMP_READY",
                '"timestamp qualification" "$STAMP_FAULT"',
            ),
            (
                "$MOTION_STATE_READY",
                '"corrected motion-state relay" "$MOTION_STATE_FAULT"',
            ),
        )
        cursor = 0
        for ready, fault in cases:
            with self.subTest(ready=ready):
                loop_start = source.index("while true; do", cursor)
                loop_end = source.index("\ndone", loop_start)
                loop = source[loop_start:loop_end]
                self.assertLess(
                    loop.index(fault),
                    loop.index(f'[[ ! -s "{ready}" ]] || break'),
                    "simultaneously visible READY+FAULT must take FAULT",
                )
                cursor = loop_end + len("\ndone")

    def test_chassis_adapter_cannot_exist_or_start_before_relay_ready(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        preexisting_guard = source.index(
            "a chassis adapter already exists before motion-state relay READY"
        )
        relay_launch = source.index("workstation_motion_state_relay.py\" \\")
        relay_ready_accept = source.index(
            '[[ ! -s "$MOTION_STATE_READY" ]] || break'
        )
        boot = source.index('"$RBNX" boot --no-update-check')
        adapter_ready_probe = source.index(
            "grep -Fxq /go2_chassis_adapter", boot
        )
        self.assertEqual(
            source.count("\nfirst_motion_require_no_adapter || exit"), 2
        )
        self.assertLess(preexisting_guard, relay_launch)
        self.assertLess(relay_launch, relay_ready_accept)
        self.assertLess(relay_ready_accept, boot)
        self.assertLess(boot, adapter_ready_probe)

    def test_boot_and_probe_have_final_fault_first_and_pid_gates(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        boot = source.index('"$RBNX" boot --no-update-check')
        pre_boot_pid = source.index(
            "a required state-chain process stopped before Robonix boot"
        )
        pre_boot_fault = source.rindex(
            '"corrected motion-state relay" "$MOTION_STATE_FAULT"',
            0,
            pre_boot_pid,
        )
        self.assertLess(pre_boot_fault, pre_boot_pid)
        self.assertLess(pre_boot_pid, boot)

        probe = source.index('"$PYTHON" "$ROOT/scripts/first_motion_probe.py" &')
        pre_probe_pid = source.index(
            "a required first-motion process stopped before probe launch"
        )
        pre_probe_fault = source.rindex(
            '"corrected motion-state relay" "$MOTION_STATE_FAULT"',
            boot,
            pre_probe_pid,
        )
        self.assertLess(pre_probe_fault, pre_probe_pid)
        self.assertLess(pre_probe_pid, probe)


if __name__ == "__main__":
    unittest.main()
