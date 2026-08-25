from pathlib import Path
import re
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class StaticSafetyTest(unittest.TestCase):
    def test_second_motion_cpp_envelope_matches_audited_profile(self) -> None:
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        daemon = (ROOT / "sdk_daemon" / "src" / "main.cpp").read_text()
        for expected in (
            "constexpr double kSecondMotionMaxVx = 0.30;",
            "constexpr double kSecondMotionMaxLinearAcceleration = 0.30;",
            "constexpr double kSecondMotionMaxDurationSec = 1.5;",
            "constexpr double kSecondMotionMaxDistanceM = 0.30;",
            '"second-motion 1.5 second hard deadline reached"',
            '"second-motion 0.30 metre hard distance reached"',
        ):
            self.assertIn(expected, node)
        for expected in (
            "constexpr float kSecondMotionMaxVx = 0.30F;",
            "constexpr std::uint64_t kSecondMotionMaxMotionMs = 1'500U;",
        ):
            self.assertIn(expected, daemon)

    def test_public_configuration_is_motion_disabled(self) -> None:
        config = yaml.safe_load((ROOT / "config" / "adapter.yaml").read_text())
        parameters = config["go2_chassis_adapter"]["ros__parameters"]
        self.assertIs(parameters["allow_motion"], False)
        self.assertIs(parameters["publish_odom_tf"], True)
        self.assertEqual(parameters["odom_source"], "sport_state")
        self.assertNotEqual(
            parameters["external_odom_topic"], parameters["odom_topic"]
        )
        self.assertEqual(parameters["allowed_modes"], [255])
        # Empty YAML sequences have no inferable ROS parameter element type.
        # The node declares an empty vector<int64> itself and the provider adds
        # a non-empty override only after an audited marker is present.
        self.assertNotIn("allowed_state_markers", parameters)
        self.assertIs(
            parameters["allow_passive_state_marker_transitions"], False
        )
        self.assertIs(
            parameters["allow_motion_state_marker_transitions"], False
        )
        self.assertEqual(parameters["max_source_stamp_age_sec"], 0.20)
        self.assertEqual(parameters["max_source_stamp_future_skew_sec"], 0.05)

        deployment = yaml.safe_load(
            (ROOT.parents[1] / "robonix_manifest.yaml").read_text()
        )
        chassis = next(
            entry["config"]
            for entry in deployment["primitive"]
            if entry["name"] == "go2_chassis"
        )
        self.assertEqual(chassis["max_source_stamp_age_s"], 0.20)
        self.assertEqual(chassis["max_source_stamp_future_skew_s"], 0.05)

    def test_cpp_defaults_are_motion_disabled_with_unknown_mode(self) -> None:
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        self.assertIn('"allowed_modes", {255}', node)
        self.assertIn(
            '"allowed_state_markers", std::vector<std::int64_t>{}', node
        )
        self.assertIn(
            '"allow_passive_state_marker_transitions", false', node
        )
        self.assertIn(
            '"allow_motion_state_marker_transitions", false', node
        )
        self.assertIn(
            "ClassicMotionStateMarkerTransitionDeploymentEligible(", node
        )
        self.assertIn(
            '"Classic marker allowlist {100,2010}"', node
        )
        self.assertIn('"state_fallback_topic", "/lf/sportmodestate"', node)
        self.assertIn(
            'declare_parameter<bool>("publish_odom_tf", true)', node
        )
        self.assertEqual(node.count("if (publish_odom_tf_)"), 1)
        self.assertIn("if (!publish_odom_tf_)", node)
        self.assertLess(
            node.index("if (publish_odom_tf_)"),
            node.index(
                "std::make_unique<tf2_ros::TransformBroadcaster>(*this)"
            ),
        )
        self.assertIn("ValidateSourceStamp(", node)
        self.assertIn("stamp_tracker.Accept(source_stamp, allow_motion_)", node)
        self.assertLess(
            node.index("ValidateSourceStamp("),
            node.index("stamp_tracker.Accept(source_stamp, allow_motion_)"),
        )
        self.assertLess(
            node.index("stamp_tracker.Accept(source_stamp, allow_motion_)"),
            node.index("last_state_receipt_sec_ = receipt_sec"),
        )
        self.assertLess(
            node.index("ValidateSourceStamp("),
            node.index("guard_->UpdateRobotState(receipt_sec"),
        )
        self.assertIn("odometry.header.stamp.sec = message.stamp.sec", node)
        self.assertIn(
            "odometry.header.stamp.nanosec = message.stamp.nanosec", node
        )
        self.assertIn("imu.header.stamp.sec = message.stamp.sec", node)
        self.assertIn(
            "imu.header.stamp.nanosec = message.stamp.nanosec", node
        )
        self.assertNotIn("odometry.header.stamp = stamp", node)
        self.assertNotIn("imu.header.stamp = stamp", node)
        self.assertIn("state == GuardState::kFault || source_stamp_fault", node)
        self.assertIn('"source timestamp rejected: "', node)
        self.assertIn("source_stamp_diagnostics_.fault_since_last_report()", node)
        self.assertIn("source_stamp_diagnostics_.MarkReported()", node)
        self.assertIn(
            "state_marker_policy_->Observe(message.error_code, receipt_sec)",
            node,
        )
        self.assertIn("PassiveStateMarkerPauseIsRecoverable(", node)
        self.assertIn("passive_recovery_pending()", node)
        self.assertIn("state_marker_policy_->change_latched()", node)
        self.assertIn("state_marker_policy_->AcknowledgeCurrent", node)
        self.assertIn(
            "guard_->state() == GuardState::kDisarmed", node
        )
        self.assertIn("guard_->ForceFault(", node)
        for expected in (
            '"odom_source", "sport_state"',
            "OnExternalOdometry(*message)",
            "external_odom_topic_ == odom_topic_",
            "message.header.frame_id != odom_frame_",
            "message.child_frame_id != base_frame_",
            "external_odom_stamp_tracker_.Accept(source_stamp, true)",
            "ExternalOdometryPoseContinuityEligible(",
            "pose or yaw continuity exceeded verified odom limits",
            "ExternalOdometryFresh(SteadyNowSec())",
            "verified external odometry became stale",
            "PublishCanonicalOdometry(canonical)",
            "external_odom_fault_latch_.CanonicalOutputEligible(",
            "CheckExternalOdometryInterlock(now_sec)",
            "HandleExternalOdometryLivenessLoss(",
            "ExternalOdometryLivenessLossRequiresProcessRestart(",
            "allow_motion_, CommissioningSessionOpen()",
            "SportModeState fresh-sample timeout",
            "external odometry fresh-sample timeout",
            "steady receipt clock is invalid or regressed",
            "SportModeState measurements invalid while stale",
            'source_stamp_diagnostics_.Observe("too_old_ignored")',
            'source_stamp_diagnostics_.Observe("duplicate_ignored")',
            'external_odom_stamp_diagnostics_.Observe("too_old_ignored")',
            'external_odom_stamp_diagnostics_.Observe("duplicate_ignored")',
        ):
            self.assertIn(expected, node)
        self.assertEqual(node.count("SourceStampViolatesProgress("), 0)
        self.assertGreaterEqual(
            node.count("ReceiptClockInvalidOrRegressed("), 3
        )
        external_callback = node[
            node.index("void OnExternalOdometry(") : node.index(
                "static void SetOdometryCovariance"
            )
        ]
        self.assertNotIn("elapsed < 1.0", external_callback)
        self.assertLess(
            external_callback.index("message.header.frame_id != odom_frame_"),
            external_callback.index("if (!sport_state_fresh)"),
        )
        self.assertLess(
            external_callback.index("if (!valid)"),
            external_callback.index("if (!sport_state_fresh)"),
        )
        self.assertLess(
            external_callback.index("ExternalOdometryPoseContinuityEligible("),
            external_callback.index("if (!sport_state_fresh)"),
        )
        liveness_handler = node[
            node.index("void HandleExternalOdometryLivenessLoss(") : node.index(
                "bool ExternalOdometryFresh("
            )
        ]
        self.assertIn("last_external_odom_receipt_sec_ = 0.0", liveness_handler)
        self.assertIn("previous_position_receipt_sec_ = 0.0", liveness_handler)
        self.assertIn("has_previous_position_ = false", liveness_handler)
        self.assertNotIn("external_odom_stamp_tracker_.Reset", liveness_handler)
        canonical_gate = node.index("if (!state_valid)")
        self.assertLess(
            canonical_gate, node.index("odom_publisher_->publish(odometry)")
        )
        self.assertLess(
            canonical_gate,
            node.index("tf_broadcaster_->sendTransform(transform)"),
        )
        self.assertLess(
            canonical_gate, node.index("imu_publisher_->publish(imu)")
        )

    def test_freshness_configuration_can_only_tighten_defaults(self) -> None:
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        guard = (ROOT / "include" / "go2_chassis" / "safety_guard.hpp").read_text()
        self.assertIn("config.state_timeout_sec > kMaximumStateTimeoutSec", node)
        self.assertIn(
            "config.max_source_stamp_age_sec > kMaximumSourceStampAgeSec", node
        )
        self.assertIn(
            "config.max_source_stamp_future_skew_sec >\n"
            "            kMaximumSourceStampFutureSkewSec",
            node,
        )
        self.assertIn("kMaximumStateTimeoutSec = 1.0", guard)
        self.assertIn("kDefaultStateTimeoutSec = 0.20", guard)
        self.assertIn("kCommissioningStateTimeoutSec = 0.20", node)
        self.assertIn("kStagedNav2StateTimeoutSec = 1.0", node)
        self.assertIn("kMaximumSourceStampAgeSec = 0.20", guard)
        self.assertIn("kMaximumSourceStampFutureSkewSec = 0.05", guard)

    def test_corrected_motion_inputs_keep_only_the_newest_sample(self) -> None:
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        subscriptions = node[
            node.index("auto state_qos =") : node.index(
                "if (allow_motion_ &&\n"
                "        (sport_state_topic_ != kFirstMotionStateTopic"
            )
        ]
        corrected_qos = subscriptions[
            subscriptions.index("auto corrected_motion_input_qos =") :
            subscriptions.index("sport_state_subscription_ =")
        ]

        self.assertIn(
            "rclcpp::QoS(rclcpp::KeepLast(1))", corrected_qos
        )
        self.assertIn(
            "corrected_motion_input_qos.best_effort().durability_volatile();",
            corrected_qos,
        )
        self.assertIn(
            "allow_motion_ ? corrected_motion_input_qos : state_qos",
            corrected_qos,
        )
        self.assertIn(
            "sport_state_topic_, sport_state_qos", subscriptions
        )
        self.assertIn(
            "sport_state_fallback_topic_, state_qos", subscriptions
        )
        self.assertIn(
            "external_odom_topic_, corrected_motion_input_qos",
            subscriptions,
        )
        self.assertEqual(subscriptions.count("rclcpp::KeepLast(1)"), 1)
        self.assertEqual(subscriptions.count("rclcpp::KeepLast(10)"), 1)

    def test_passive_adapter_constructs_no_motion_control_entities(self) -> None:
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        constructor = node[
            node.index("Go2ChassisAdapterNode()") : node.index(
                "~Go2ChassisAdapterNode()"
            )
        ]
        initializer = node[
            node.index("void InitializeMotionControlGraph()") : node.index(
                "GuardConfig DeclareGuardConfig()"
            )
        ]

        self.assertIn(
            "if (graph_plan.has_complete_motion_control_graph())", constructor
        )
        self.assertIn("if (!graph_plan.is_consistent())", constructor)
        self.assertIn("InitializeMotionControlGraph();", constructor)
        for entity_creation in (
            "std::make_unique<SeqpacketClient>",
            "create_subscription<geometry_msgs::msg::Twist>",
            "create_service<std_srvs::srv::SetBool>",
            "control_timer_ = create_wall_timer",
        ):
            self.assertNotIn(entity_creation, constructor)
            self.assertEqual(node.count(entity_creation), 1)
            self.assertIn(entity_creation, initializer)

        destructor = node[
            node.index("~Go2ChassisAdapterNode()") : node.index(
                "void InitializeMotionControlGraph()"
            )
        ]
        self.assertIn(
            "if (motion_control_graph_initialized_ && allow_motion_)", destructor
        )
        self.assertIn("BestEffortDaemonDisarm()", destructor)
        self.assertIn("if (!allow_motion_)", initializer)

        arm_handler = node[
            node.index("void OnArmRequest(") : node.index("void OnControlTimer()")
        ]
        control_handler = node[
            node.index("void OnControlTimer()") : node.index(
                "void HandleIpcFault("
            )
        ]
        disarm_handler = node[
            node.index(
                "bool BestEffortDaemonDisarm(bool restore_classic_walk = false,"
            ) : node.index(
                "void PublishDiagnostics()"
            )
        ]
        fail_closed_check = "!allow_motion_ || !motion_control_graph_initialized_"
        self.assertIn(fail_closed_check, arm_handler)
        self.assertIn(fail_closed_check, control_handler)
        self.assertIn(fail_closed_check, disarm_handler)
        self.assertNotIn("if (!daemon_armed_ || ipc_ == nullptr)", disarm_handler)
        self.assertIn("if (ipc_ == nullptr)", disarm_handler)
        self.assertIn("if (!ipc_->connected())", disarm_handler)
        self.assertIn("ipc_->Connect(&connect_error)", disarm_handler)
        self.assertIn("if (daemon_armed_)", disarm_handler)
        self.assertIn("CommandOp::kDisarm", disarm_handler)
        self.assertIn(
            "guard_->state() != GuardState::kFault", arm_handler
        )
        self.assertIn(
            "response.success && clean_classic_walk_restore", arm_handler
        )

    def test_runtime_graph_policy_is_fail_closed(self) -> None:
        policy = (
            ROOT / "include" / "go2_chassis" / "runtime_graph.hpp"
        ).read_text()
        self.assertIn("static constexpr RuntimeGraphPlan For(bool allow_motion)", policy)
        for field in (
            "seqpacket_client",
            "cmd_vel_subscription",
            "arm_service",
            "control_timer",
        ):
            self.assertIn(field, policy)
        test = (ROOT / "tests" / "runtime_graph_test.cpp").read_text()
        self.assertIn("RuntimeGraphPlan::For(false)", test)
        self.assertIn("!passive.has_motion_control_entities()", test)
        self.assertIn("RuntimeGraphPlan::For(true)", test)
        self.assertIn("motion_enabled.has_complete_motion_control_graph()", test)

    def test_motion_state_requires_progress_and_reverse_is_not_forwarded(self) -> None:
        guard = (ROOT / "include" / "go2_chassis" / "safety_guard.hpp").read_text()
        daemon = (ROOT / "include" / "go2_chassis" / "daemon_core.hpp").read_text()
        self.assertIn("source_stamp_ns <= last_source_stamp_ns_", guard)
        self.assertIn("source_stamp_ns == 0U", guard)
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        self.assertIn(
            "state_marker_policy_->Observe(message.error_code, receipt_sec)",
            node,
        )
        self.assertIn("error_code == 0U ||", guard)
        self.assertIn("allowed_state_markers.count(error_code) == 1U", guard)
        self.assertIn("if (has_bound_marker_ && marker != bound_marker_)", guard)
        self.assertIn("change_latched_ = true", guard)
        self.assertIn("allow_allowlisted_transitions_", guard)
        self.assertIn(
            "allowed_state_markers_.count(bound_marker_) == 1U", guard
        )
        self.assertIn(
            "allow_motion_ || odom_source_ != OdomSource::kExternalVerified",
            node,
        )
        self.assertIn(
            "passive state marker transitions require at least two explicit",
            node,
        )
        self.assertIn("config_.allowed_modes.count(state_mode_)", guard)
        self.assertIn("std::clamp(velocity.vx, 0.0, config_.max_vx)", guard)
        self.assertIn("packet.vx < 0.0F || packet.vx > config_.max_vx", daemon)

    def test_first_motion_has_independent_fixed_adapter_and_daemon_limits(self) -> None:
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        daemon = (ROOT / "include" / "go2_chassis" / "daemon_core.hpp").read_text()
        daemon_main = (ROOT / "sdk_daemon" / "src" / "main.cpp").read_text()
        for expected in (
            'kFirstMotionCommandTopic =\n    "/go2/commissioning/cmd_vel"',
            "kFirstMotionMaxVx = 0.05",
            "kFirstMotionMaxVy = 0.0",
            "kFirstMotionMaxWz = 0.0",
            "kFirstMotionCommandTimeoutSec = 0.20",
            "kFirstMotionMaxDurationSec = 2.0",
            "kFirstMotionMaxDistanceM = 0.10",
            "CommissioningEnvelopeExceeded",
            "UpdateCommissioningDistance",
            "commissioning_arm_spent_",
            "FailClosedCommissioningStop",
            "CommandOp::kStop",
            "CommandOp::kDisarm",
            'DiagnosticValue(\n        "commissioning_motion_active"',
            'DiagnosticValue(\n        "commissioning_command_timeout_sec"',
        ):
            self.assertIn(expected, node)
        self.assertIn("max_motion_ns{2'000'000'000ULL}", daemon)
        self.assertIn("commissioning_arm_spent_", daemon)
        self.assertIn("now_ns - motion_start_ns_ >= config_.max_motion_ns", daemon)
        self.assertIn("void OnDisconnect()", daemon)
        for option in ("--max-vx", "--max-vy", "--max-wz", "--max-motion-ms"):
            self.assertIn(option, daemon_main)
        self.assertIn("options.max_vx != kCommissioningMaxVx", daemon_main)
        self.assertIn(
            "options.max_motion_ms != kCommissioningMaxMotionMs", daemon_main
        )

    def test_ros_process_does_not_link_sdk2(self) -> None:
        cmake = (
            ROOT / "ros2_ws" / "src" / "go2_chassis_adapter" / "CMakeLists.txt"
        ).read_text()
        self.assertNotIn("unitree_sdk2", cmake)
        self.assertIn("unitree_go", cmake)

    def test_sdk_process_does_not_link_ros(self) -> None:
        cmake = (ROOT / "sdk_daemon" / "CMakeLists.txt").read_text()
        self.assertIn("unitree_sdk2", cmake)
        self.assertIn('INSTALL_RPATH "\\$ORIGIN/../lib"', cmake)
        self.assertIn("-Wl,--disable-new-dtags", cmake)
        self.assertIn("libddsc.so libddsc.so.0 libddscxx.so libddscxx.so.0", cmake)
        for forbidden in ("rclcpp", "ament", "geometry_msgs", "nav_msgs"):
            self.assertNotIn(forbidden, cmake)

        provider = (ROOT / "go2_chassis" / "main.py").read_text()
        runtime = (ROOT / "go2_chassis" / "runtime_config.py").read_text()
        self.assertIn("runtime.sdk_daemon_env(_daemon_binary)", provider)
        self.assertIn('environment["LD_LIBRARY_PATH"]', runtime)

    def test_runtime_has_no_posture_or_low_level_api(self) -> None:
        runtime_files = list((ROOT / "include").rglob("*.hpp"))
        runtime_files += list((ROOT / "sdk_daemon").rglob("*.cpp"))
        runtime_files += list((ROOT / "ros2_ws").rglob("*.cpp"))
        source = "\n".join(path.read_text() for path in runtime_files)
        for forbidden in (
            "StandUp",
            "StandDown",
            "RecoveryStand",
            "BalanceStand",
            "lowcmd",
            "/lowcmd",
        ):
            self.assertNotIn(forbidden, source)

        # The isolated SDK daemon may subscribe to the raw sport RPC pair to
        # independently verify SDK2 request/response correlation.  It must not
        # construct a raw request publisher or use that topic anywhere else.
        sdk_client = (
            ROOT / "sdk_daemon" / "src" / "unitree_sport_client.cpp"
        ).read_text()
        self.assertEqual(sdk_client.count("rt/api/sport/request"), 1)
        self.assertIn(
            "ChannelSubscriber<unitree::robot::Request>", sdk_client
        )
        self.assertNotIn("ChannelPublisher<unitree::robot::Request>", source)
        non_observer_source = "\n".join(
            path.read_text()
            for path in runtime_files
            if path != ROOT / "sdk_daemon" / "src" / "unitree_sport_client.cpp"
        )
        self.assertNotIn("/api/sport/request", non_observer_source)

    def test_sdk_motion_rpc_requires_full_ack_and_exact_zero_lease(self) -> None:
        guard = (
            ROOT / "include" / "go2_chassis" / "rpc_response_guard.hpp"
        ).read_text()
        daemon = (
            ROOT / "include" / "go2_chassis" / "daemon_core.hpp"
        ).read_text()
        sdk_client = (
            ROOT / "sdk_daemon" / "src" / "unitree_sport_client.cpp"
        ).read_text()
        self.assertIn("response.identity_id != request.identity_id", guard)
        self.assertIn("response.api_id != request.api_id", guard)
        self.assertIn("response.status_code != 0", guard)
        self.assertIn("request.lease_id != expected.lease_id", guard)
        self.assertIn("expected.require_positive_lease", guard)
        self.assertIn(
            "request.noreply != expected.expected_noreply", guard
        )
        self.assertIn(
            "request.priority != expected.expected_priority", guard
        )
        self.assertIn(
            "request.parameter != expected.expected_parameter", guard
        )
        self.assertIn("client_.PrepareArm()", daemon)
        self.assertIn("SportClient(false)", sdk_client)
        self.assertNotIn("ServiceList(services)", sdk_client)
        self.assertNotIn("RobotStateClient", sdk_client)
        self.assertIn("ROBOT_SPORT_API_ID_STOPMOVE", sdk_client)
        self.assertNotIn("ROBOT_API_ID_INTERNAL_API_NOOP", sdk_client)
        self.assertNotIn("ProbeOwnership", sdk_client)
        self.assertNotIn("owned_lease_id_", sdk_client)
        self.assertIn(
            "{api_id, 0, false, expected_noreply, expected_priority,\n"
            "         expected_parameter}",
            sdk_client,
        )
        self.assertIn("constexpr std::int32_t kMovePriority = 0", sdk_client)
        self.assertIn(
            "constexpr std::int32_t kStopMovePriority = 1", sdk_client
        )
        self.assertIn(
            "constexpr std::int32_t kClassicWalkPriority = 0", sdk_client
        )
        self.assertIn(
            "PrepareArm accepted initialized official sport client path",
            sdk_client,
        )
        prepare_arm = sdk_client[
            sdk_client.index("std::int32_t UnitreeSportClient::PrepareArm()"):
            sdk_client.index("std::int32_t UnitreeSportClient::VerifiedCall(")
        ]
        self.assertNotIn("client_->StopMove()", prepare_arm)
        self.assertIn("header.policy().noreply()", sdk_client)
        self.assertIn("header.policy().priority()", sdk_client)
        self.assertIn("candidate.parameter = incoming.parameter()", sdk_client)
        self.assertIn("unitree::common::ToJsonString(json)", sdk_client)
        self.assertIn(
            "VerifiedCall(unitree::robot::go2::ROBOT_SPORT_API_ID_MOVE,\n"
            "                   [this, vx, vy, wz]()",
            sdk_client,
        )
        self.assertIn(
            "},\n                   kMovePriority, true, expected_parameter);",
            sdk_client,
        )
        self.assertIn(
            "Witnessed one-way Move emission: noreply=true", sdk_client
        )
        self.assertIn(
            "[this]() { return client_->StopMove(); },\n"
            "                      kStopMovePriority, false",
            sdk_client,
        )
        self.assertIn("ROBOT_SPORT_API_ID_CLASSICWALK", sdk_client)
        self.assertIn("JsonizeDataBool json", sdk_client)
        self.assertIn("client_->ClassicWalk(enabled)", sdk_client)
        self.assertIn(
            "kClassicWalkPriority, false, expected_parameter", sdk_client
        )
        self.assertIn("Verified sport RPC failed: api_id=", sdk_client)
        self.assertIn("remote_status=", sdk_client)

    def test_arm_preserves_stopped_output_until_first_move(self) -> None:
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        control = node[
            node.index("void OnControlTimer()") : node.index(
                "void HandleIpcFault("
            )
        ]
        arm_block = control[
            control.index("if (!daemon_armed_)") : control.index(
                "CommandOp operation = CommandOp::kPing"
            )
        ]
        self.assertIn("daemon_armed_ = true", arm_block)
        self.assertIn("last_output_stopped_ = true", arm_block)
        self.assertNotIn("last_output_stopped_ = false", arm_block)
        self.assertIn("return;", arm_block)
        move_block = control[
            control.index("if (decision.action == GuardAction::kMove)") :
            control.index("if (!ipc_->Exchange(")
        ]
        self.assertIn("last_output_stopped_ = false", move_block)
        self.assertIn("operation = CommandOp::kStop", move_block)
        self.assertIn("last_output_stopped_ = true", move_block)

    def test_motion_ipc_timing_covers_sdk_calls_below_watchdog(self) -> None:
        timing = (
            ROOT / "include" / "go2_chassis" / "motion_timing.hpp"
        ).read_text()
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        sdk_client = (
            ROOT / "sdk_daemon" / "src" / "unitree_sport_client.cpp"
        ).read_text()
        self.assertIn("kRpcEvidenceArrivalTimeoutMs = 50", timing)
        self.assertIn("kRpcEvidenceSettlementMs = 25", timing)
        self.assertIn("kMotionArmIpcReplyTimeoutMs = 290", timing)
        self.assertIn("kMotionCommandIpcReplyTimeoutMs = 190", timing)
        self.assertIn("kMotionPingIpcReplyTimeoutMs = 20", timing)
        self.assertIn("MotionIpcReplyTimeoutMs(operation)", node)
        self.assertIn("CommandOp::kRestoreClassicWalk", node)
        self.assertIn(
            "StopMove and ClassicWalk are\n"
            "      // both response-bearing SDK calls and cannot share the 190 ms deadline",
            node,
        )
        self.assertIn("kSdkSynchronousCallTimeoutMs", sdk_client)
        self.assertIn("kRpcEvidenceSettlementMs", sdk_client)
        self.assertNotIn("ipc_reply_timeout_ms =", node)
        self.assertIn(
            "The next timer tick must revalidate everything", node
        )

    def test_wrappers_never_publish_with_ros2_cli(self) -> None:
        for path in (ROOT / "scripts").glob("*.sh"):
            self.assertIsNone(
                re.search(r"ros2\s+topic\s+pub", path.read_text()), path
            )

    def test_robonix_registration_does_not_create_cmd_vel_publisher(self) -> None:
        provider = (ROOT / "go2_chassis" / "main.py").read_text()
        self.assertNotIn("create_publisher", provider)
        self.assertIn("declare_ros2_topic", provider)
        self.assertIn("normalize_config", provider)
        self.assertIn("prepare_private_directory(runtime.ipc_socket.parent)", provider)
        self.assertNotIn("ipc_socket.parent.chmod", provider)
        self.assertIn(".spawn(", provider)

    def test_package_manifest_uses_canonical_contract_names(self) -> None:
        manifest = yaml.safe_load((ROOT / "package_manifest.yaml").read_text())
        capabilities = manifest["capabilities"]
        self.assertEqual(
            {entry["name"] for entry in capabilities},
            {
                "robonix/primitive/chassis/driver",
                "robonix/primitive/chassis/twist_in",
                "robonix/primitive/chassis/odom",
            },
        )
        self.assertTrue(all(set(entry) == {"name"} for entry in capabilities))

    def test_package_entrypoint_only_starts_provider(self) -> None:
        start = (ROOT / "scripts" / "start.sh").read_text()
        for forbidden in ("start_adapter.sh", "start_daemon.sh", "ros2 run"):
            self.assertNotIn(forbidden, start)
        self.assertIn("RBNX_PACKAGE_ROOT", start)
        self.assertIn("export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", start)
        self.assertIn("CYCLONEDDS_URI must be bound", start)
        self.assertIn("rbnx-build/codegen/ros2_idl/install/setup.bash", start)

    def test_build_checks_all_runtime_outputs(self) -> None:
        build = (ROOT / "scripts" / "build.sh").read_text()
        self.assertIn('rbnx codegen -p "$PKG" --ros2', build)
        for output in (
            "go2_chassis_adapter_node",
            "go2_sport_daemon",
            "libddsc.so.0",
            "libddscxx.so.0",
        ):
            self.assertIn(output, build)

    def test_manual_wrappers_cannot_enable_motion(self) -> None:
        for name in ("start_adapter.sh", "start_daemon.sh"):
            wrapper = (ROOT / "scripts" / name).read_text()
            self.assertIn("Refusing direct motion-enabled", wrapper)
        daemon_wrapper = (ROOT / "scripts" / "start_daemon.sh").read_text()
        self.assertNotIn("--allow-motion", daemon_wrapper)

    def test_capability_frontmatter_has_only_description(self) -> None:
        capability = (ROOT / "CAPABILITY.md").read_text()
        match = re.match(r"\A---\n(.*?)\n---\n", capability, re.DOTALL)
        self.assertIsNotNone(match)
        self.assertEqual(set(yaml.safe_load(match.group(1))), {"description"})


if __name__ == "__main__":
    unittest.main()
