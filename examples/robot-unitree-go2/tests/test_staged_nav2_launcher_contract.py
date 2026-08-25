from __future__ import annotations

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "start_workstation_staged_nav2_corrected.sh"
LIVE_VALIDATOR = (
    ROOT / "scripts" / "validate_staged_nav2_live_goal_evidence.py"
)
READINESS_VALIDATOR = (
    ROOT / "scripts" / "validate_staged_nav2_readiness_receipt.py"
)
RESULT_VALIDATOR = ROOT / "scripts" / "validate_staged_nav2_result.py"


class StagedNav2LauncherContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = LAUNCHER.read_text(encoding="utf-8")

    def test_one_explicit_run_approval_and_one_time_permits_exist(self) -> None:
        for required in (
            "I_APPROVE_GO2_STAGED_NAV2_MOTION",
            "GO2_STAGED_NAV2_PERMIT_FILE",
            "GO2_STAGED_NAV2_GOAL_PERMIT_FILE",
            "validate_staged_nav2_permit_bundle.py",
            "GO2_STAGED_NAV2_GUARD_ACK",
            "GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256",
        ):
            self.assertIn(required, self.source)
        self.assertIn('[[ "$#" -eq 0 ]]', self.source)
        self.assertIn('[[ "${#BUNDLE[@]}" -eq 15 ]]', self.source)
        self.assertIn('[[ "${#FINAL_BUNDLE[@]}" -eq 15 ]]', self.source)
        self.assertNotIn("GO2_STAGED_NAV2_SITE_ACK", self.source)
        self.assertNotIn("GO2_STAGED_NAV2_WIRELESS_ACK", self.source)

    def test_private_wireless_topology_is_read_only_and_rechecked(self) -> None:
        for required in (
            "wlx500ff54809b8",
            "Robonix-Go2",
            "ce767234-9037-4a53-a5f4-aa7b6cbf743f",
            "wlo1",
            "192.168.123.99/24",
            "192.168.123.161",
            "192.168.123.18",
            "validate_first_motion_network.py",
            "--transport wireless-private",
        ):
            self.assertIn(required, self.source)
        self.assertEqual(
            self.source.count("\nvalidate_wireless_topology \\"),
            3,
        )
        for forbidden in (
            "nmcli connection up",
            "nmcli connection modify",
            "nmcli device connect",
            "ip route add",
            "ip addr add",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_corrected_motion_chain_precedes_boot_and_dispatch(self) -> None:
        identity = self.source.index("workstation_nomotion_identity_monitor.py")
        stamp = self.source.index("workstation_nomotion_stamp_node.py")
        motion_state = self.source.index("workstation_motion_state_relay.py")
        cloud = self.source.index("workstation_nomotion_cloud_relay.py")
        boot = self.source.index('"$RBNX" boot --no-update-check')
        pre_guard = self.source.index("--phase pre_guard")
        live_confirm = self.source.index(
            "prepare_staged_nav2_goal_evidence.py"
        )
        guard = self.source.index("staged_nav2_motion_guard.py")
        post_guard = self.source.index("--phase post_guard")
        dispatch = self.source.index("staged_nav2_goal_dispatch.py")
        self.assertLess(identity, stamp)
        self.assertLess(stamp, motion_state)
        self.assertLess(motion_state, cloud)
        self.assertLess(cloud, boot)
        self.assertLess(boot, pre_guard)
        self.assertLess(pre_guard, live_confirm)
        self.assertLess(live_confirm, guard)
        self.assertLess(guard, post_guard)
        self.assertLess(post_guard, dispatch)
        self.assertRegex(
            self.source,
            re.compile(
                r"workstation_nomotion_stamp_node\.py\" \\\n"
                r"\s+--mode affine --profile motion"
            ),
        )
        self.assertIn("--profile motion", self.source)
        boot_block = re.search(
            r"\(\n"
            r"\s+go2_runtime_close_parent_only_fds\n"
            r"(?:\s+#.*\n)*"
            r"\s+cd -- \"\$RUN_DIR\"\n"
            r"\s+exec \"\$RBNX\" boot --no-update-check -f \"\$MANIFEST\"\n"
            r"\)",
            self.source,
        )
        self.assertIsNotNone(boot_block)
        self.assertIn('MANIFEST="$RUN_DIR/robonix_manifest.yaml"', self.source)

    def test_classic_marker_transition_is_persistent_only(self) -> None:
        self.assertIn("CLASSIC_MOTION_STATE_MARKERS=100,2010", self.source)
        self.assertIn(
            'export GO2_ALLOWED_MODES="$observed_mode"', self.source
        )
        self.assertIn(
            'if [[ "$PERSISTENT_MODE" == true ]]; then', self.source
        )
        self.assertIn(
            'export GO2_ALLOWED_STATE_MARKERS="$CLASSIC_MOTION_STATE_MARKERS"',
            self.source,
        )
        self.assertIn(
            'export GO2_ALLOWED_STATE_MARKERS="$observed_marker"', self.source
        )
        self.assertIn(
            "staged Nav2 current marker is outside the reviewed Classic set",
            self.source,
        )

    def test_stage1_goal_tolerance_is_rendered_without_mutating_shared_config(
        self,
    ) -> None:
        self.assertIn("render_staged_nav2_params.py", self.source)
        self.assertIn(
            '--source "$ROOT/config/nav2_params_go2.yaml"',
            self.source,
        )
        self.assertIn(
            '--output "$CONFIG_DIR/nav2_params_go2.yaml"',
            self.source,
        )
        self.assertNotIn(
            'materialize_runtime_config \\\n'
            '  "$ROOT/config/nav2_params_go2.yaml"',
            self.source,
        )

    def test_private_terminal_result_is_required_before_success(self) -> None:
        for required in (
            'export GO2_STAGED_NAV2_RUN_DIR="$RUN_DIR"',
            'export GO2_STAGED_NAV2_ACTION_RESULT_FILE="$RUN_DIR/goal-action-result.json"',
            'export GO2_STAGED_NAV2_RESULT_FILE="$RUN_DIR/measured-result.json"',
            "validate_staged_nav2_result.py",
            "staged Nav2 goal completed with measured stop evidence",
        ):
            self.assertIn(required, self.source)
        run_dir = self.source.index(
            'RUN_DIR="$(mktemp -d "$ROOT/rbnx-build/run/'
        )
        result_paths = self.source.index(
            "export GO2_STAGED_NAV2_RUN_DIR", run_dir
        )
        guard = self.source.index("staged_nav2_motion_guard.py")
        dispatch = self.source.index("staged_nav2_goal_dispatch.py")
        validator = self.source.index("validate_staged_nav2_result.py")
        success = self.source.index(
            "staged Nav2 goal completed with measured stop evidence"
        )
        self.assertLess(run_dir, result_paths)
        self.assertLess(result_paths, guard)
        self.assertLess(result_paths, dispatch)
        self.assertLess(dispatch, validator)
        self.assertLess(validator, success)
        self.assertTrue(RESULT_VALIDATOR.is_file())
        validator_source = RESULT_VALIDATOR.read_text(encoding="utf-8")
        self.assertIn("validate_measured_result(", validator_source)
        self.assertNotIn("import rclpy", validator_source)

    def test_private_sdk_socket_fits_the_unix_path_limit(self) -> None:
        self.assertIn(
            'RUN_DIR="$(mktemp -d "$ROOT/rbnx-build/run/stg.XXXXXX")"',
            self.source,
        )
        self.assertIn('export GO2_SDK_SOCKET="$RUN_DIR/s"', self.source)
        self.assertIn("${#GO2_SDK_SOCKET} < 108", self.source)
        socket_export = self.source.index(
            'export GO2_SDK_SOCKET="$RUN_DIR/s"'
        )
        manifest_render = self.source.index(
            "render_workstation_staged_nav2_manifest.py"
        )
        boot = self.source.index('"$RBNX" boot --no-update-check')
        dispatch = self.source.index("staged_nav2_goal_dispatch.py")
        self.assertLess(socket_export, manifest_render)
        self.assertLess(manifest_render, boot)
        self.assertLess(socket_export, dispatch)

    def test_standard_mode_uses_live_goal_and_legacy_keeps_permit_hash(
        self,
    ) -> None:
        self.assertIn(
            'LIVE_GOAL_EVIDENCE="$RUN_DIR/live-goal-evidence.json"',
            self.source,
        )
        self.assertIn(
            'export GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256="${BUNDLE[9]}"',
            self.source,
        )
        self.assertIn('STANDARD_MODE="${GO2_STAGED_NAV2_STANDARD_MODE:-true}"', self.source)
        self.assertIn(
            'export GO2_STAGED_NAV2_GOAL_EVIDENCE_SHA256="$LIVE_GOAL_RECEIPT_SHA256"',
            self.source,
        )
        self.assertIn("--forward-distance", self.source)
        self.assertIn(
            "GO2_STAGED_NAV2_LIVE_GOAL_RECEIPT_SHA256",
            self.source,
        )
        self.assertIn("--permit-start-x", self.source)
        self.assertIn("--permit-start-y", self.source)
        self.assertIn('[[ "${#LIVE_GOAL[@]}" -eq 4 ]]', self.source)
        self.assertIn(
            '--expected-start-x "$LIVE_START_X" --expected-start-y "$LIVE_START_Y"',
            self.source,
        )
        self.assertIn(
            '--permit-start-x "$GO2_STAGED_NAV2_EXPECTED_START_X"',
            self.source,
        )
        self.assertIn(
            '--permit-start-y "$GO2_STAGED_NAV2_EXPECTED_START_Y"',
            self.source,
        )
        self.assertIn(
            'export GO2_STAGED_NAV2_EXPECTED_START_X="${BUNDLE[12]}"',
            self.source,
        )
        self.assertIn(
            'export GO2_STAGED_NAV2_EXPECTED_START_Y="${BUNDLE[13]}"',
            self.source,
        )

    def test_permit_pair_is_revalidated_after_chassis_consumption(self) -> None:
        self.assertIn(
            'CONSUMED_CHASSIS_PERMIT="${GO2_STAGED_NAV2_PERMIT_FILE}.consumed-${CHASSIS_PERMIT_ID}"',
            self.source,
        )
        self.assertIn(
            '--chassis-permit "$CONSUMED_CHASSIS_PERMIT"',
            self.source,
        )
        final_bundle = self.source.index("mapfile -t FINAL_BUNDLE")
        final_container = self.source.rindex(
            'verify_current_nav2_container "$CURRENT_NAV2_CONTAINER_ID"'
        )
        final_network = self.source.rindex("validate_wireless_topology")
        dispatcher = self.source.index("staged_nav2_goal_dispatch.py")
        self.assertLess(final_bundle, final_container)
        self.assertLess(final_container, final_network)
        self.assertLess(final_bundle, final_network)
        self.assertLess(final_network, dispatcher)

    def test_nav2_container_and_cleanup_are_exact(self) -> None:
        for required in (
            "robonix_nav2_staged_",
            "cleanup_owned_staged_nav2_namespace",
            "Recovering owned stale staged Nav2 container",
            "staged Nav2 namespace contains an unowned or invalid container",
            "verify_staged_nav2_stop_hook",
            "verify_current_nav2_container",
            "{{.Id}}",
            "true|host|host|no|false|",
            "RBNX_INVOCATION_CWD",
            "ROBONIX_PKG_HOST_DIR",
            "ROBONIX_VELOCITY_OUTPUT_TOPIC",
            "ROBONIX_CAPABILITY_ID",
            "ROBONIX_NAV2_CONTAINER=\"$id\"",
            "shutdown_rbnx_boot_state.py",
            'verify_current_nav2_container "$id" false',
            "unix:///var/run/docker.sock",
            "terminate_boot_child",
            "timeout --signal=TERM --kill-after=5s 45s",
        ):
            self.assertIn(required, self.source)
        dispatcher_stop = self.source.index('terminate_child "$DISPATCH_PID"')
        guard_stop = self.source.index('terminate_child "$GUARD_PID"')
        boot_shutdown = self.source.index("shutdown_rbnx_boot_state.py")
        relay_stop = self.source.index('terminate_child "$CLOUD_PID"')
        self.assertLess(dispatcher_stop, guard_stop)
        self.assertLess(guard_stop, boot_shutdown)
        self.assertLess(boot_shutdown, relay_stop)
        self.assertNotIn('terminate_child "$BOOT_PID"', self.source)
        self.assertIn('terminate_boot_child "$BOOT_PID"', self.source)
        boot = self.source.index('"$RBNX" boot --no-update-check')
        bind_container = self.source.index(
            'verify_current_nav2_container "$NAV2_CONTAINER_ID"',
            boot,
        )
        pre_guard = self.source.index("--phase pre_guard")
        self.assertLess(boot, bind_container)
        self.assertLess(bind_container, pre_guard)
        self.assertIn("deadline=$((SECONDS + 10))", self.source)
        self.assertNotIn("exact manual recovery is required", self.source)
        self.assertNotIn("audited_image_id", self.source)
        self.assertNotIn("recognized_mounts", self.source)

    def test_disarmed_readiness_recovers_without_a_manual_restart(self) -> None:
        for required in (
            "PRE_GUARD_ATTEMPT=0",
            "POST_GUARD_ATTEMPT=0",
            "pre-guard-readiness-attempt-",
            "post-guard-readiness-attempt-",
            "waiting for live startup streams to settle",
            "waiting for disarmed navigation streams to settle",
            "readiness_status == 3",
            'mv -- "$PRE_GUARD_CANDIDATE" "$PRE_GUARD_RECEIPT"',
            'mv -- "$POST_GUARD_CANDIDATE" "$POST_GUARD_RECEIPT"',
        ):
            self.assertIn(required, self.source)
        self.assertEqual(self.source.count("--duration 5"), 2)
        self.assertNotIn("pre-guard readiness deadline", self.source)
        self.assertNotIn("post-guard readiness deadline", self.source)

    def test_launcher_avoids_unsafe_or_reusable_control_shortcuts(self) -> None:
        for forbidden in (
            "source \"$ROOT/.env\"",
            "source $ROOT/.env",
            "bash \"$ROOT/start.sh\"",
            "eval ",
            "pkill ",
            "killall ",
            "ros2 topic pub",
            "ros2 service call",
            "/api/sport/request",
            "/lowcmd",
            "sport_mode_ctrl",
            "go2_sport_client",
            "go2_stand_example",
            "low_level_ctrl",
            "export ROBONIX_CAPABILITY_ID=nav2",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_offline_validators_are_present(self) -> None:
        self.assertTrue(LIVE_VALIDATOR.is_file())
        self.assertTrue(READINESS_VALIDATOR.is_file())
        live = LIVE_VALIDATOR.read_text(encoding="utf-8")
        self.assertIn("MAX_PERMIT_START_DRIFT_M = 0.05", live)
        self.assertIn("--permit-start-x", live)
        self.assertIn("--permit-start-y", live)
        readiness = READINESS_VALIDATOR.read_text(encoding="utf-8")
        self.assertIn("validate_readiness_receipt", readiness)
        self.assertIn("object_pairs_hook=_duplicate_safe_object", readiness)


if __name__ == "__main__":
    unittest.main()
