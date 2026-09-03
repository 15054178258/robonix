from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "start_second_motion_corrected.sh"
FIRST_LAUNCHER = ROOT / "scripts" / "start_first_motion_corrected.sh"


class SecondMotionLauncherContractTest(unittest.TestCase):
    def test_launcher_is_one_shot_private_and_binds_first_pass(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        for required in (
            "workstation-second-motion-corrected-v1",
            "I_CONFIRM_GO2_CLEAR_2M_REMOTE_STOP_READY",
            "I_APPROVE_GO2_SECOND_20CM_MOTION",
            "GO2_FIRST_MOTION_PASS_EVIDENCE",
            "--first-motion-pass-evidence",
            "validate_first_motion_state_evidence.py",
            "validate_second_motion_permit_for_launch.py",
            "workstation_nomotion_identity_monitor.py",
            "workstation_nomotion_stamp_node.py",
            "workstation_motion_state_relay.py",
            "render_workstation_second_motion_manifest.py",
            "capture_motion_rpc_graph_baseline.py",
            "GO2_SPORT_REQUEST_BASELINE_FILE",
            "second_motion_probe.py",
            "go2_runtime_lease_acquire",
            "shutdown_rbnx_boot_state.py",
            'terminate_child "$PROBE_PID"',
        ):
            self.assertIn(required, source)
        self.assertIn("0.30 m/s", source)
        self.assertIn("1.2 s / 0.20 m", source)
        self.assertIn("1.5 s / 0.30 m", source)
        self.assertEqual(source.count('--package-root "$ROOT"'), 2)
        self.assertRegex(
            source,
            re.compile(
                r"workstation_nomotion_stamp_node\.py\" \\\n"
                r"\s+--mode affine \\\n\s+--profile motion"
            ),
        )
        self.assertNotRegex(
            source,
            re.compile(r"(^|\s)source\s+\"?\$ROOT/\.env"),
        )
        for forbidden in (
            "ros2 topic pub",
            "ros2 service call",
            "sport_mode_ctrl",
            "go2_sport_client",
            "go2_stand_example",
            "low_level_ctrl",
        ):
            self.assertNotIn(forbidden, source)

    def test_wireless_stage_has_its_own_ack_and_rechecks_topology(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        self.assertIn(
            'TRANSPORT="${GO2_SECOND_MOTION_TRANSPORT:-wired}"',
            source,
        )
        self.assertIn(
            "I_APPROVE_GO2_WIRELESS_PRIVATE_LAN_SECOND_20CM",
            source,
        )
        self.assertIn(
            '[[ "${GO2_SECOND_MOTION_WIRELESS_ACK:-}" == "$WIRELESS_ACK" ]]',
            source,
        )
        self.assertEqual(
            source.count("second_motion_validate_wireless_topology ||"),
            2,
        )
        self.assertIn("validate_first_motion_network.py", source)
        self.assertNotIn("nmcli connection up", source)
        self.assertNotIn("nmcli connection modify", source)

    def test_canonical_command_and_navigation_routes_remain_isolated(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        self.assertNotIn(
            "ROBONIX_VELOCITY_OUTPUT_TOPIC=/cmd_vel",
            source,
        )
        self.assertNotRegex(
            source,
            re.compile(r"ros2\s+topic\s+pub.*?/cmd_vel"),
        )
        self.assertNotIn("staged_nav2", source)
        self.assertNotIn("NavigateToPose", source)

    def test_ready_waits_are_fault_first_and_adapter_starts_last(self) -> None:
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
            loop_start = source.index("while true; do", cursor)
            loop_end = source.index("\ndone", loop_start)
            loop = source[loop_start:loop_end]
            with self.subTest(ready=ready):
                self.assertLess(
                    loop.index(fault),
                    loop.index(f'[[ ! -s "{ready}" ]] || break'),
                )
            cursor = loop_end + len("\ndone")
        relay_ready = source.index(
            '[[ ! -s "$MOTION_STATE_READY" ]] || break'
        )
        boot = source.index('"$RBNX" boot --no-update-check')
        probe = source.index(
            '"$PYTHON" "$ROOT/scripts/second_motion_probe.py" &'
        )
        self.assertEqual(
            source.count("\nsecond_motion_require_no_adapter || exit"),
            2,
        )
        self.assertLess(relay_ready, boot)
        self.assertLess(boot, probe)

    def test_first_launcher_contract_is_not_widened(self) -> None:
        source = FIRST_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn(
            "workstation-first-motion-corrected-v1",
            source,
        )
        self.assertIn("I_APPROVE_GO2_FIRST_10CM_MOTION", source)
        self.assertIn("/go2/commissioning/cmd_vel", source)
        self.assertIn("soft stop: 1.8 s / 0.09 m", source)
        self.assertIn("hard envelope: 2.0 s / 0.10 m", source)
        self.assertNotIn("GO2_SECOND_MOTION_ACK", source)
        self.assertNotIn("second_motion_probe.py", source)


if __name__ == "__main__":
    unittest.main()
