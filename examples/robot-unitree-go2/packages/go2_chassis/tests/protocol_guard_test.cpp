#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include "go2_chassis/daemon_core.hpp"
#include "go2_chassis/motion_timing.hpp"
#include "go2_chassis/protocol.hpp"
#include "go2_chassis/rpc_response_guard.hpp"
#include "go2_chassis/safety_guard.hpp"

namespace {

class FakeSportClient final : public go2_chassis::ISportClient {
 public:
  bool Initialize(const std::string &, std::string *) override {
    initialized = true;
    return true;
  }
  std::int32_t PrepareArm() override {
    ++prepare_arm_calls;
    return prepare_arm_result;
  }
  std::int32_t ClassicWalk(bool enabled) override {
    ++classic_walk_calls;
    classic_walk_enabled = enabled;
    return classic_walk_result;
  }
  std::int32_t Move(float x, float y, float yaw) override {
    ++move_calls;
    vx = x;
    vy = y;
    wz = yaw;
    return move_result;
  }
  std::int32_t StopMove() override {
    ++stop_calls;
    return stop_result;
  }

  bool initialized{false};
  int prepare_arm_calls{0};
  int classic_walk_calls{0};
  int move_calls{0};
  int stop_calls{0};
  std::int32_t prepare_arm_result{0};
  std::int32_t classic_walk_result{0};
  std::int32_t move_result{0};
  std::int32_t stop_result{0};
  float vx{0.0F};
  float vy{0.0F};
  float wz{0.0F};
  bool classic_walk_enabled{false};
};

void TestProtocol() {
  constexpr std::uint64_t now = 1'000'000'000ULL;
  auto packet = go2_chassis::MakeCommand(go2_chassis::CommandOp::kMove, 7U, now,
                                         100'000'000ULL, 0.1F, 0.0F, 0.2F);
  assert(go2_chassis::ValidateCommand(packet, now) ==
         go2_chassis::ReplyCode::kOk);
  packet.vx = std::numeric_limits<float>::quiet_NaN();
  go2_chassis::Seal(packet);
  assert(go2_chassis::ValidateCommand(packet, now) ==
         go2_chassis::ReplyCode::kMalformed);

  packet = go2_chassis::MakeCommand(go2_chassis::CommandOp::kPing, 8U, now,
                                    10'000'000ULL);
  assert(go2_chassis::ValidateCommand(packet, now + 20'000'000ULL) ==
         go2_chassis::ReplyCode::kExpired);
  packet.checksum ^= 1U;
  assert(go2_chassis::ValidateCommand(packet, now) ==
         go2_chassis::ReplyCode::kMalformed);
}

void TestMotionWatchdogProfileGate() {
  using go2_chassis::MotionWatchdogDeploymentEligible;

  // Diagnostic/no-motion startup retains the existing 100..1000 ms parser
  // range.  Only a motion-capable process is pinned to the audited 300 ms
  // cross-process deadline.
  assert(MotionWatchdogDeploymentEligible(false, 100U));
  assert(MotionWatchdogDeploymentEligible(false, 300U));
  assert(MotionWatchdogDeploymentEligible(false, 1000U));
  assert(!MotionWatchdogDeploymentEligible(true, 100U));
  assert(!MotionWatchdogDeploymentEligible(true, 299U));
  assert(MotionWatchdogDeploymentEligible(true, 300U));
  assert(!MotionWatchdogDeploymentEligible(true, 301U));
  assert(!MotionWatchdogDeploymentEligible(true, 1000U));
}

void TestMotionIpcTimingContract() {
  using go2_chassis::CommandOp;
  using go2_chassis::MotionIpcReplyTimeoutMs;

  assert(MotionIpcReplyTimeoutMs(CommandOp::kArm) == 290);
  assert(MotionIpcReplyTimeoutMs(CommandOp::kMove) == 190);
  assert(MotionIpcReplyTimeoutMs(CommandOp::kStop) == 190);
  assert(MotionIpcReplyTimeoutMs(CommandOp::kDisarm) == 190);
  assert(MotionIpcReplyTimeoutMs(CommandOp::kRestoreClassicWalk) == 190);
  assert(MotionIpcReplyTimeoutMs(CommandOp::kPing) == 20);
  assert(MotionIpcReplyTimeoutMs(CommandOp::kArm) <
         static_cast<std::int32_t>(
             go2_chassis::kAuditedMotionWatchdogMs));
}

void TestGuardDefaultsFailClosed() {
  go2_chassis::SafetyGuard guard(go2_chassis::GuardConfig{});
  guard.UpdateRobotState(1.0, 0U, true);
  guard.UpdateCommand(1.0, {});
  std::string message;
  assert(!guard.RequestArm(1.0, &message));
  assert(guard.state() == go2_chassis::GuardState::kDisabled);
  assert(guard.Tick(1.1, 0.02).action == go2_chassis::GuardAction::kStop);
}

void TestUnknownDefaultModeCannotArm() {
  go2_chassis::GuardConfig config;
  config.allow_motion = true;
  go2_chassis::SafetyGuard guard(config);
  guard.UpdateRobotState(1.0, 0U, true);
  guard.UpdateCommand(1.0, {});
  std::string message;
  assert(!guard.RequestArm(1.0, &message));
  assert(message == "SportModeState mode is not allowed");
  assert(guard.state() == go2_chassis::GuardState::kDisarmed);
}

void TestGuardPreparationLimitsAndFaultLatch() {
  go2_chassis::GuardConfig config;
  config.allow_motion = true;
  config.allowed_modes = {0U};
  go2_chassis::SafetyGuard guard(config);
  guard.UpdateRobotState(10.0, 0U, true);
  guard.UpdateCommand(10.0, {});
  std::string message;
  assert(guard.RequestArm(10.0, &message));
  guard.UpdateRobotState(10.49, 0U, true);
  guard.UpdateCommand(10.49, {});
  assert(guard.Tick(10.49, 0.02).state ==
         go2_chassis::GuardState::kPreparing);

  guard.UpdateRobotState(10.51, 0U, true);
  guard.UpdateCommand(10.51, {});
  assert(guard.Tick(10.51, 0.02).state == go2_chassis::GuardState::kArmed);

  guard.UpdateRobotState(10.52, 0U, true);
  guard.UpdateCommand(10.52, {-2.0, 1.0, 3.0});
  const auto bounded = guard.Tick(10.52, 0.02);
  assert(bounded.action == go2_chassis::GuardAction::kMove);
  assert(bounded.velocity.vx == 0.0);
  assert(bounded.velocity.vy == 0.0);
  assert(std::fabs(bounded.velocity.wz) <= 0.0161);

  assert(guard.Tick(10.80, 0.02).state == go2_chassis::GuardState::kFault);
  assert(!guard.RequestArm(10.80, &message));
  guard.UpdateRobotState(10.81, 0U, true);
  guard.UpdateCommand(10.81, {});
  assert(guard.RequestDisarm(10.81, &message));
  assert(guard.state() == go2_chassis::GuardState::kDisarmed);
}

void TestSourceStampLivenessFailClosed() {
  constexpr std::uint64_t max_age_ns = 200'000'000ULL;
  constexpr std::uint64_t max_future_skew_ns = 50'000'000ULL;
  constexpr std::int64_t reference_ns = 1'000'000'000'000LL;
  using Freshness = go2_chassis::SourceStampFreshness;

  assert(!go2_chassis::SourceStampStatusIsFault("not_received"));
  assert(!go2_chassis::SourceStampStatusIsFault("fresh"));
  assert(!go2_chassis::SourceStampStatusIsFault("too_old_ignored"));
  assert(!go2_chassis::SourceStampStatusIsFault("duplicate_ignored"));
  assert(go2_chassis::SourceStampStatusIsFault("zero"));
  assert(go2_chassis::SourceStampStatusIsFault("malformed"));
  assert(go2_chassis::SourceStampStatusIsFault("too_old"));
  assert(go2_chassis::SourceStampStatusIsFault("too_far_in_future"));
  assert(go2_chassis::SourceStampStatusIsFault("non_monotonic"));

  go2_chassis::SourceStampDiagnosticTracker stamp_diagnostics;
  assert(stamp_diagnostics.latest_status() == "not_received");
  assert(!stamp_diagnostics.active_fault());
  assert(!stamp_diagnostics.fault_since_last_report());
  stamp_diagnostics.Observe("too_old");
  assert(stamp_diagnostics.active_fault());
  assert(stamp_diagnostics.fault_since_last_report());
  stamp_diagnostics.Observe("fresh");
  assert(!stamp_diagnostics.active_fault());
  assert(stamp_diagnostics.fault_since_last_report());
  assert(stamp_diagnostics.last_fault_status() == "too_old");
  assert(stamp_diagnostics.total_faults() == 1U);
  stamp_diagnostics.MarkReported();
  assert(!stamp_diagnostics.fault_since_last_report());

  assert(go2_chassis::ValidateSourceStamp(
             0U, reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kZero);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns), 0, max_age_ns,
             max_future_skew_ns) == Freshness::kReferenceClockInvalid);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns), -1, max_age_ns,
             max_future_skew_ns) == Freshness::kReferenceClockInvalid);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) - max_age_ns,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kFresh);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) - max_age_ns - 1U,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kTooOld);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) + max_future_skew_ns,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kFresh);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) +
                 max_future_skew_ns + 1U,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kTooFarInFuture);

  // Reproduce the measured hardware failure class: a monotonically advancing
  // source clock that is roughly 739 seconds behind is still stale.
  constexpr std::uint64_t measured_offset_ns = 739'000'000'000ULL;
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) - measured_offset_ns,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kTooOld);
  assert(go2_chassis::ValidateSourceStamp(
             std::numeric_limits<std::uint64_t>::max(), reference_ns,
             max_age_ns, max_future_skew_ns) ==
         Freshness::kTooFarInFuture);

  go2_chassis::SourceStampTracker motion_stamps;
  assert(!motion_stamps.Accept(0U, true));
  assert(motion_stamps.Accept(100U, true));
  assert(!motion_stamps.Accept(100U, true));
  assert(!motion_stamps.Accept(99U, true));
  assert(motion_stamps.Accept(101U, true));

  go2_chassis::SourceStampTracker readonly_stamps;
  assert(!readonly_stamps.Accept(0U, false));
  assert(readonly_stamps.Accept(42U, false));
  assert(readonly_stamps.Accept(42U, false));
  assert(!readonly_stamps.Accept(41U, false));

  go2_chassis::GuardConfig config;
  config.allow_motion = true;
  config.allowed_modes = {0U};
  config.zero_preparation_sec = 0.0;
  config.state_timeout_sec = 0.20;
  go2_chassis::SafetyGuard guard(config);
  go2_chassis::SourceStampTracker state_stamps;
  assert(state_stamps.Accept(1U, true));
  guard.UpdateRobotState(1.0, 0U, true);
  guard.UpdateCommand(1.0, {});
  std::string message;
  assert(guard.RequestArm(1.0, &message));
  assert(guard.Tick(1.0, 0.02).state == go2_chassis::GuardState::kArmed);

  // A duplicate packet may keep arriving, and commands may remain fresh, but
  // rejecting its source stamp means it cannot extend the state watchdog.
  assert(!state_stamps.Accept(1U, true));
  guard.UpdateCommand(1.10, {});
  assert(guard.Tick(1.10, 0.02).state == go2_chassis::GuardState::kArmed);
  guard.UpdateCommand(1.21, {});
  assert(guard.Tick(1.21, 0.02).state == go2_chassis::GuardState::kFault);

  // A too-old sample is checked against retained history before it may be
  // classified as a recoverable passive liveness loss.
  assert(!go2_chassis::SourceStampViolatesProgress(10U, 0U, true));
  assert(go2_chassis::SourceStampViolatesProgress(10U, 10U, true));
  assert(!go2_chassis::SourceStampViolatesProgress(10U, 10U, false));
  assert(go2_chassis::SourceStampViolatesProgress(9U, 10U, false));
  assert(!go2_chassis::ReceiptClockInvalidOrRegressed(10.0, 9.9, 9.8));
  assert(go2_chassis::ReceiptClockInvalidOrRegressed(10.0, 10.1, 9.8));
  assert(go2_chassis::ReceiptClockInvalidOrRegressed(10.0, 9.9, 10.1));
}

void TestExternalOdometryCanonicalInterlock() {
  constexpr double timeout_sec = 0.20;
  constexpr double max_position_jump_m = 1.0;
  constexpr double max_yaw_jump_rad = 1.0;

  // The first pose establishes continuity only after the message-level
  // frame/stamp/numeric checks. Every later pose must remain inside both the
  // timeout and jump envelopes; elapsed >= 1 s is never a jump exemption.
  assert(go2_chassis::ExternalOdometryContinuityEligible(
      false, 0.0, 0.0, 0.0, timeout_sec, max_position_jump_m,
      max_yaw_jump_rad));
  assert(go2_chassis::ExternalOdometryContinuityEligible(
      true, timeout_sec, max_position_jump_m, max_yaw_jump_rad, timeout_sec,
      max_position_jump_m, max_yaw_jump_rad));
  assert(!go2_chassis::ExternalOdometryContinuityEligible(
      true, timeout_sec + 0.001, 0.0, 0.0, timeout_sec,
      max_position_jump_m, max_yaw_jump_rad));
  assert(!go2_chassis::ExternalOdometryContinuityEligible(
      true, 2.0, 2.0, 0.0, timeout_sec, max_position_jump_m,
      max_yaw_jump_rad));
  assert(!go2_chassis::ExternalOdometryContinuityEligible(
      true, 0.05, max_position_jump_m + 0.001, 0.0, timeout_sec,
      max_position_jump_m, max_yaw_jump_rad));
  assert(go2_chassis::ExternalOdometryPoseContinuityEligible(
      true, max_position_jump_m, max_yaw_jump_rad, max_position_jump_m,
      max_yaw_jump_rad));
  assert(!go2_chassis::ExternalOdometryPoseContinuityEligible(
      true, max_position_jump_m + 0.001, 0.0, max_position_jump_m,
      max_yaw_jump_rad));

  go2_chassis::ExternalOdometryFaultLatch interlock;
  assert(interlock.CanonicalOutputEligible(
      true, 10.0, true, 10.0, 10.10, timeout_sec, timeout_sec));
  // Either state invalidity or either receipt timeout blocks the exact same
  // canonical publication path, independent of whether motion is enabled.
  assert(!interlock.CanonicalOutputEligible(
      false, 10.0, true, 10.0, 10.10, timeout_sec, timeout_sec));
  assert(!interlock.CanonicalOutputEligible(
      true, 10.0, true, 10.0, 10.201, timeout_sec, timeout_sec));

  interlock.Latch();
  assert(interlock.latched());
  // Fresh replacement evidence cannot silently establish a new origin in
  // the same process. There is deliberately no reset method.
  assert(!interlock.CanonicalOutputEligible(
      true, 20.0, true, 20.0, 20.01, timeout_sec, timeout_sec));

  // Pure liveness loss may establish a new continuity epoch while disarmed.
  // Once a motion session is open it requires a reviewed restart, and hard
  // integrity faults always use the latch above.
  assert(!go2_chassis::ExternalOdometryLivenessLossRequiresProcessRestart(
      false, false));
  assert(!go2_chassis::ExternalOdometryLivenessLossRequiresProcessRestart(
      false, true));
  assert(!go2_chassis::ExternalOdometryLivenessLossRequiresProcessRestart(
      true, false));
  assert(go2_chassis::ExternalOdometryLivenessLossRequiresProcessRestart(
      true, true));
}

void TestOpaqueFirmwareStateMarkerPolicyFailsClosedOnChange() {
  assert(go2_chassis::CanonicalStateOutputEligible(true, 0U));
  assert(!go2_chassis::CanonicalStateOutputEligible(true, 1U));
  assert(!go2_chassis::CanonicalStateOutputEligible(true, 100U));
  assert(!go2_chassis::CanonicalStateOutputEligible(false, 0U));
  assert(go2_chassis::CanonicalStateOutputEligible(true, 2010U, {2010U}));
  assert(!go2_chassis::CanonicalStateOutputEligible(false, 2010U, {2010U}));

  go2_chassis::OpaqueFirmwareStateMarkerPolicy default_policy;
  assert(default_policy.Observe(0U));
  assert(default_policy.Observe(0U));
  assert(!default_policy.Observe(2010U));
  assert(default_policy.change_latched());
  std::string message;
  assert(!default_policy.AcknowledgeCurrent(&message));
  assert(message ==
         "current firmware state marker is not explicitly allowed");

  go2_chassis::OpaqueFirmwareStateMarkerPolicy audited_policy(
      {100U, 2010U});
  assert(audited_policy.Observe(2010U));
  assert(audited_policy.has_bound_marker());
  assert(audited_policy.bound_marker() == 2010U);
  assert(audited_policy.Observe(2010U));

  // A transition remains a latch even when both opaque values were explicitly
  // listed. It cannot silently inherit an existing armed session.
  assert(!audited_policy.Observe(100U));
  assert(audited_policy.change_latched());
  assert(audited_policy.current_marker_allowed());
  assert(!audited_policy.Observe(100U));
  assert(audited_policy.CanAcknowledgeCurrent(&message));
  assert(audited_policy.change_latched());
  assert(audited_policy.AcknowledgeCurrent(&message));
  assert(!audited_policy.change_latched());
  assert(audited_policy.bound_marker() == 100U);
  assert(audited_policy.Observe(100U));

  // Zero is accepted without an allowlist, but changing to it still requires
  // the same explicit disarm acknowledgement.
  assert(!audited_policy.Observe(0U));
  assert(audited_policy.change_latched());
  assert(audited_policy.AcknowledgeCurrent(&message));
  assert(audited_policy.bound_marker() == 0U);
  assert(audited_policy.Observe(0U));

  // The narrowly scoped passive-mapping exception accepts a transition only
  // when both values are members of its explicit reviewed set. The default
  // policy above remains fail-closed for the same 2010 -> 100 transition.
  go2_chassis::OpaqueFirmwareStateMarkerPolicy passive_mapping_policy(
      {100U, 1002U}, true, true);
  assert(passive_mapping_policy.allow_allowlisted_transitions());
  assert(passive_mapping_policy.allow_passive_recovery());
  assert(passive_mapping_policy.Observe(100U));
  assert(passive_mapping_policy.Observe(1002U));
  assert(!passive_mapping_policy.change_latched());
  assert(passive_mapping_policy.bound_marker() == 1002U);
  assert(passive_mapping_policy.Observe(100U));
  assert(!passive_mapping_policy.change_latched());
  assert(passive_mapping_policy.bound_marker() == 100U);

  // Zero is intrinsically eligible for ordinary state publication but is not
  // in the exceptional non-zero set. The passive profile therefore pauses
  // without binding zero or setting a process-lifetime marker latch.
  assert(!passive_mapping_policy.Observe(0U));
  assert(!passive_mapping_policy.change_latched());
  assert(passive_mapping_policy.passive_recovery_pending());
  assert(!passive_mapping_policy.has_passive_recovery_candidate());

  // Returning to an explicit marker is not enough by itself. It must be the
  // same marker for at least 0.5 seconds, at least five samples, and no sample
  // gap may exceed the independently audited 0.2 second state timeout.
  assert(!passive_mapping_policy.Observe(100U, 10.00));
  assert(!passive_mapping_policy.Observe(100U, 10.10));
  assert(!passive_mapping_policy.Observe(100U, 10.20));
  assert(!passive_mapping_policy.Observe(100U, 10.39));
  assert(passive_mapping_policy.passive_recovery_pending());
  assert(passive_mapping_policy.passive_recovery_samples() == 4U);
  assert(passive_mapping_policy.Observe(100U, 10.50));
  assert(!passive_mapping_policy.passive_recovery_pending());
  assert(passive_mapping_policy.bound_marker() == 100U);

  go2_chassis::OpaqueFirmwareStateMarkerPolicy unknown_transition_policy(
      {100U, 1002U}, true, true);
  assert(unknown_transition_policy.Observe(100U));
  assert(!unknown_transition_policy.Observe(2010U, 20.00));
  assert(!unknown_transition_policy.change_latched());
  assert(unknown_transition_policy.passive_recovery_pending());
  assert(!unknown_transition_policy.Observe(1002U, 20.10));
  // A gap longer than the state-liveness ceiling resets the stable window.
  assert(!unknown_transition_policy.Observe(1002U, 20.31));
  assert(unknown_transition_policy.passive_recovery_samples() == 1U);
  assert(!unknown_transition_policy.Observe(1002U, 20.41));
  assert(!unknown_transition_policy.Observe(1002U, 20.51));
  assert(!unknown_transition_policy.Observe(1002U, 20.61));
  assert(!unknown_transition_policy.Observe(1002U, 20.71));
  assert(unknown_transition_policy.Observe(1002U, 20.82));
  assert(!unknown_transition_policy.passive_recovery_pending());
  assert(unknown_transition_policy.bound_marker() == 1002U);

  // The same unknown value still latches the default policy. Automatic
  // recovery is impossible unless the narrow passive option was constructed.
  go2_chassis::OpaqueFirmwareStateMarkerPolicy default_unknown_policy(
      {100U, 1002U});
  assert(default_unknown_policy.Observe(100U));
  assert(!default_unknown_policy.Observe(2010U, 30.00));
  assert(default_unknown_policy.change_latched());
  assert(!default_unknown_policy.passive_recovery_pending());

  // The motion exception is narrower than the passive mapping path: one
  // audited mode, staged Nav2, external odometry, and exactly the two observed
  // Classic markers. It accepts 2010 <-> 100, but an unknown marker latches
  // immediately and cannot enter passive recovery.
  assert(go2_chassis::ClassicMotionStateMarkerTransitionDeploymentEligible(
      true, true, true, true, false, {0U}, {100U, 2010U}));
  assert(go2_chassis::ClassicMotionStateMarkerTransitionDeploymentEligible(
      false, false, false, false, true, {}, {}));
  assert(!go2_chassis::ClassicMotionStateMarkerTransitionDeploymentEligible(
      true, false, true, true, false, {0U}, {100U, 2010U}));
  assert(!go2_chassis::ClassicMotionStateMarkerTransitionDeploymentEligible(
      true, true, false, true, false, {0U}, {100U, 2010U}));
  assert(!go2_chassis::ClassicMotionStateMarkerTransitionDeploymentEligible(
      true, true, true, false, false, {0U}, {100U, 2010U}));
  assert(!go2_chassis::ClassicMotionStateMarkerTransitionDeploymentEligible(
      true, true, true, true, true, {0U}, {100U, 2010U}));
  assert(!go2_chassis::ClassicMotionStateMarkerTransitionDeploymentEligible(
      true, true, true, true, false, {0U, 1U}, {100U, 2010U}));
  assert(!go2_chassis::ClassicMotionStateMarkerTransitionDeploymentEligible(
      true, true, true, true, false, {0U}, {100U, 1002U}));

  go2_chassis::OpaqueFirmwareStateMarkerPolicy classic_motion_policy(
      {100U, 2010U}, true, false);
  assert(classic_motion_policy.Observe(2010U));
  assert(classic_motion_policy.Observe(100U));
  assert(classic_motion_policy.Observe(2010U));
  assert(!classic_motion_policy.change_latched());
  assert(!classic_motion_policy.Observe(1002U));
  assert(classic_motion_policy.change_latched());
  assert(!classic_motion_policy.passive_recovery_pending());

  // The node may downgrade only this marker quarantine to a recoverable
  // publication pause. Motion, non-external odom, disabled transition policy,
  // invalid measurements, or a true marker latch all remain hard faults.
  assert(go2_chassis::PassiveStateMarkerPauseIsRecoverable(
      false, true, true, true, true, false));
  assert(!go2_chassis::PassiveStateMarkerPauseIsRecoverable(
      true, true, true, true, true, false));
  assert(!go2_chassis::PassiveStateMarkerPauseIsRecoverable(
      false, false, true, true, true, false));
  assert(!go2_chassis::PassiveStateMarkerPauseIsRecoverable(
      false, true, false, true, true, false));
  assert(!go2_chassis::PassiveStateMarkerPauseIsRecoverable(
      false, true, true, false, true, false));
  assert(!go2_chassis::PassiveStateMarkerPauseIsRecoverable(
      false, true, true, true, false, false));
  assert(!go2_chassis::PassiveStateMarkerPauseIsRecoverable(
      false, true, true, true, true, true));
}

void TestMarkerChangeRequiresDisarmBeforeRearm() {
  go2_chassis::GuardConfig config;
  config.allow_motion = true;
  config.allowed_modes = {0U};
  config.zero_preparation_sec = 0.0;
  go2_chassis::SafetyGuard guard(config);
  go2_chassis::OpaqueFirmwareStateMarkerPolicy marker_policy({100U, 2010U});

  assert(marker_policy.Observe(2010U));
  guard.UpdateRobotState(20.0, 0U, true);
  guard.UpdateCommand(20.0, {});
  std::string message;
  assert(guard.RequestArm(20.0, &message));
  assert(guard.Tick(20.0, 0.02).state ==
         go2_chassis::GuardState::kArmed);

  assert(!marker_policy.Observe(100U));
  guard.UpdateRobotState(20.01, 0U, false);
  guard.ForceFault(
      "opaque firmware state marker changed; explicit disarm required");
  assert(guard.Tick(20.01, 0.02).state ==
         go2_chassis::GuardState::kFault);
  assert(!guard.RequestArm(20.01, &message));

  assert(marker_policy.AcknowledgeCurrent(&message));
  assert(marker_policy.Observe(100U));
  guard.UpdateRobotState(20.02, 0U, true);
  guard.UpdateCommand(20.02, {});
  assert(guard.RequestDisarm(20.02, &message));
  assert(guard.state() == go2_chassis::GuardState::kDisarmed);
  assert(guard.RequestArm(20.02, &message));
  assert(guard.Tick(20.02, 0.02).state ==
         go2_chassis::GuardState::kArmed);
}

void TestRpcAcknowledgementMustMatchFullIdentityLeaseAndStatus() {
  const go2_chassis::RpcCallExpectation expected{
      1008, 77, true, false, 0, ""};
  const go2_chassis::RpcRequestEvidence request{true, false, 12345, 1008,
                                                77, false, 0, ""};
  const go2_chassis::RpcResponseEvidence response{true, false, 12345, 1008,
                                                  0};
  auto result =
      go2_chassis::ValidateRpcCallEvidence(expected, request, response, 0);
  assert(result.ok());
  assert(go2_chassis::RpcEvidenceReturnCode(result) == 0);

  auto wrong_identity = response;
  wrong_identity.identity_id = 12346;
  result = go2_chassis::ValidateRpcCallEvidence(expected, request,
                                                 wrong_identity, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kResponseMismatch);

  auto wrong_api = response;
  wrong_api.api_id = 1003;
  result = go2_chassis::ValidateRpcCallEvidence(expected, request, wrong_api,
                                                 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kResponseMismatch);

  auto remote_error = response;
  remote_error.status_code = 7001;
  result = go2_chassis::ValidateRpcCallEvidence(expected, request,
                                                 remote_error, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRemoteStatusError);
  assert(go2_chassis::RpcEvidenceReturnCode(result) == 7001);

  auto wrong_lease = request;
  wrong_lease.lease_id = 78;
  result = go2_chassis::ValidateRpcCallEvidence(expected, wrong_lease,
                                                 response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);

  auto zero_identity = request;
  zero_identity.identity_id = 0;
  result = go2_chassis::ValidateRpcCallEvidence(expected, zero_identity,
                                                 response, 0);
  assert(result.code ==
         go2_chassis::RpcEvidenceCode::kRequestIdentityInvalid);

  auto ambiguous_request = request;
  ambiguous_request.ambiguous = true;
  result = go2_chassis::ValidateRpcCallEvidence(expected, ambiguous_request,
                                                 response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestAmbiguous);

  auto ambiguous_response = response;
  ambiguous_response.ambiguous = true;
  result = go2_chassis::ValidateRpcCallEvidence(expected, request,
                                                 ambiguous_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kResponseAmbiguous);

  result =
      go2_chassis::ValidateRpcCallEvidence(expected, request, response, 3105);
  assert(result.code == go2_chassis::RpcEvidenceCode::kSdkCallFailed);
  assert(go2_chassis::RpcEvidenceReturnCode(result) == 3105);

  const go2_chassis::RpcCallExpectation exact_stop{
      1003, 0, false, false, 1, ""};
  const go2_chassis::RpcRequestEvidence zero_lease_stop_request{
      true, false, 22345, 1003, 0, false, 1, ""};
  const go2_chassis::RpcResponseEvidence zero_lease_stop_response{
      true, false, 22345, 1003, 0};
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_stop, zero_lease_stop_request, zero_lease_stop_response, 0);
  assert(result.ok());
  assert(result.request_priority == 1);

  auto wrong_stop_priority = zero_lease_stop_request;
  wrong_stop_priority.priority = 0;
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_stop, wrong_stop_priority, zero_lease_stop_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);

  auto unexpected_positive_lease = zero_lease_stop_request;
  unexpected_positive_lease.lease_id = 1;
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_stop, unexpected_positive_lease, zero_lease_stop_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);

  const go2_chassis::RpcCallExpectation invalid_zero_default{
      1003, 0, true, false, 1, ""};
  result = go2_chassis::ValidateRpcCallEvidence(
      invalid_zero_default, zero_lease_stop_request, zero_lease_stop_response,
      0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kInvalidExpectation);

  const go2_chassis::RpcCallExpectation invalid_nonzero_exact_zero{
      1003, 1, false, false, 1, ""};
  result = go2_chassis::ValidateRpcCallEvidence(
      invalid_nonzero_exact_zero, zero_lease_stop_request,
      zero_lease_stop_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kInvalidExpectation);

  const go2_chassis::RpcCallExpectation move_noreply{
      1008, 0, false, true, 0, R"({"x":0.05,"y":0.0,"z":0.0})"};
  const go2_chassis::RpcRequestEvidence move_request{
      true, false, 32345, 1008, 0, true, 0,
      R"({"x":0.05,"y":0.0,"z":0.0})"};
  const go2_chassis::RpcResponseEvidence no_response{};
  result = go2_chassis::ValidateRpcCallEvidence(
      move_noreply, move_request, no_response, 0);
  assert(result.ok());
  assert(result.request_identity_id == 32345);
  assert(result.request_api_id == 1008);
  assert(result.request_lease_id == 0);
  assert(result.request_priority == 0);
  assert(result.request_noreply);
  assert(!result.response_observed);
  assert(result.request_parameter == R"({"x":0.05,"y":0.0,"z":0.0})");

  auto wrong_move_policy = move_request;
  wrong_move_policy.noreply = false;
  result = go2_chassis::ValidateRpcCallEvidence(
      move_noreply, wrong_move_policy, no_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);

  auto wrong_move_priority = move_request;
  wrong_move_priority.priority = 1;
  result = go2_chassis::ValidateRpcCallEvidence(
      move_noreply, wrong_move_priority, no_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);

  auto wrong_move_parameter = move_request;
  wrong_move_parameter.parameter = R"({"x":0.0,"y":0.0,"z":0.0})";
  result = go2_chassis::ValidateRpcCallEvidence(
      move_noreply, wrong_move_parameter, no_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);

  auto malformed_no_response = no_response;
  malformed_no_response.ambiguous = true;
  result = go2_chassis::ValidateRpcCallEvidence(
      move_noreply, move_request, malformed_no_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kResponseAmbiguous);

  auto optional_move_response =
      go2_chassis::RpcResponseEvidence{true, false, 32345, 1008, 0};
  result = go2_chassis::ValidateRpcCallEvidence(
      move_noreply, move_request, optional_move_response, 0);
  assert(result.ok());
  optional_move_response.status_code = 7002;
  result = go2_chassis::ValidateRpcCallEvidence(
      move_noreply, move_request, optional_move_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRemoteStatusError);

  const go2_chassis::RpcCallExpectation exact_classic_walk{
      2049, 0, false, false, 0, R"({"data":true})"};
  const go2_chassis::RpcRequestEvidence classic_walk_request{
      true, false, 42345, 2049, 0, false, 0, R"({"data":true})"};
  const go2_chassis::RpcResponseEvidence classic_walk_response{
      true, false, 42345, 2049, 0};
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_classic_walk, classic_walk_request, classic_walk_response, 0);
  assert(result.ok());
  auto wrong_classic_policy = classic_walk_request;
  wrong_classic_policy.noreply = true;
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_classic_walk, wrong_classic_policy, classic_walk_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);
  auto wrong_classic_priority = classic_walk_request;
  wrong_classic_priority.priority = 1;
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_classic_walk, wrong_classic_priority, classic_walk_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);
  auto wrong_classic_parameter = classic_walk_request;
  wrong_classic_parameter.parameter = R"({"data":false})";
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_classic_walk, wrong_classic_parameter, classic_walk_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRequestMismatch);
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_classic_walk, classic_walk_request, no_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kResponseMissing);
  auto rejected_classic_response = classic_walk_response;
  rejected_classic_response.status_code = 7002;
  result = go2_chassis::ValidateRpcCallEvidence(
      exact_classic_walk, classic_walk_request, rejected_classic_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kRemoteStatusError);

  result = go2_chassis::ValidateRpcCallEvidence(
      exact_stop, zero_lease_stop_request, no_response, 0);
  assert(result.code == go2_chassis::RpcEvidenceCode::kResponseMissing);
}

void TestDaemonCannotArmWithoutClientPreflight() {
  constexpr std::uint64_t now = 4'000'000'000ULL;
  FakeSportClient client;
  client.prepare_arm_result = -4201;
  go2_chassis::DaemonConfig config;
  config.allow_motion = true;
  go2_chassis::DaemonCore daemon(config, client);
  const auto arm = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kArm, 1U, now, 100'000'000ULL);
  const auto reply = daemon.Handle(arm, now);
  assert(reply.code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kSdkError));
  assert(client.prepare_arm_calls == 1);
  assert(!daemon.armed());
  assert(daemon.faulted());
  assert(client.move_calls == 0);
  assert(client.stop_calls == 0);

  auto disarm = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kDisarm, 2U, now + 1U, 100'000'000ULL);
  assert(daemon.Handle(disarm, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  const auto retry = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kArm, 3U, now + 2U, 100'000'000ULL);
  assert(daemon.Handle(retry, now + 2U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kFaultLatched));
  assert(client.prepare_arm_calls == 1);
}

void TestDaemonDefenseInDepthAndWatchdog() {
  constexpr std::uint64_t now = 5'000'000'000ULL;
  FakeSportClient disabled_client;
  go2_chassis::DaemonCore disabled({}, disabled_client);
  auto arm = go2_chassis::MakeCommand(go2_chassis::CommandOp::kArm, 1U, now,
                                      100'000'000ULL);
  auto reply = disabled.Handle(arm, now);
  assert(reply.code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kMotionDisabled));
  assert(disabled_client.move_calls == 0);
  assert(disabled_client.stop_calls == 0);

  const auto expect_limit = [now](float vx, float vy, float wz) {
    FakeSportClient client;
    go2_chassis::DaemonConfig config;
    config.allow_motion = true;
    go2_chassis::DaemonCore daemon(config, client);
    auto local_arm = go2_chassis::MakeCommand(
        go2_chassis::CommandOp::kArm, 1U, now, 100'000'000ULL);
    assert(daemon.Handle(local_arm, now).code ==
           static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
    auto move = go2_chassis::MakeCommand(
        go2_chassis::CommandOp::kMove, 2U, now + 1U, 100'000'000ULL,
        vx, vy, wz);
    assert(daemon.Handle(move, now + 1U).code ==
           static_cast<std::int32_t>(
               go2_chassis::ReplyCode::kLimitExceeded));
    assert(client.move_calls == 0);
    assert(client.stop_calls == 1);
    assert(daemon.faulted());
  };
  expect_limit(0.0501F, 0.0F, 0.0F);
  expect_limit(-0.01F, 0.0F, 0.0F);
  expect_limit(0.01F, 0.001F, 0.0F);
  expect_limit(0.01F, 0.0F, 0.001F);

  FakeSportClient duration_client;
  go2_chassis::DaemonConfig duration_config;
  duration_config.allow_motion = true;
  duration_config.watchdog_ns = 3'000'000'000ULL;
  go2_chassis::DaemonCore duration_daemon(duration_config, duration_client);
  assert(duration_daemon.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  auto valid_move = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kMove, 2U, now + 1U, 100'000'000ULL,
      0.05F, 0.0F, 0.0F);
  assert(duration_daemon.Handle(valid_move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(!duration_daemon.CheckWatchdog(
      now + 1U + duration_config.max_motion_ns - 1U));
  assert(duration_daemon.CheckWatchdog(
      now + 1U + duration_config.max_motion_ns));
  assert(duration_client.move_calls == 1);
  assert(duration_client.stop_calls == 1);
  assert(duration_daemon.faulted());
  assert(!duration_daemon.armed());

  FakeSportClient packet_client;
  go2_chassis::DaemonConfig packet_config;
  packet_config.allow_motion = true;
  go2_chassis::DaemonCore packet_daemon(packet_config, packet_client);
  assert(packet_daemon.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(packet_daemon.Handle(valid_move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(packet_daemon.CheckWatchdog(
      now + 1U + packet_config.watchdog_ns + 1U));
  assert(packet_client.stop_calls == 1);

  FakeSportClient disconnect_client;
  go2_chassis::DaemonConfig disconnect_config;
  disconnect_config.allow_motion = true;
  go2_chassis::DaemonCore disconnect_daemon(disconnect_config,
                                             disconnect_client);
  assert(disconnect_daemon.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(disconnect_daemon.Handle(valid_move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  disconnect_daemon.OnDisconnect();
  assert(!disconnect_daemon.armed());
  assert(!disconnect_daemon.moving());
  assert(disconnect_client.stop_calls == 1);
  assert(disconnect_daemon.Handle(arm, now + 2U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kFaultLatched));

  // A failed StopMove reply must never be treated as proof that the physical
  // robot stopped. Disconnect performs an immediate retry, then the daemon's
  // watchdog loop keeps retrying while local motion remains disarmed/faulted.
  FakeSportClient failed_stop_client;
  failed_stop_client.stop_result = -1;
  go2_chassis::DaemonConfig failed_stop_config;
  failed_stop_config.allow_motion = true;
  go2_chassis::DaemonCore failed_stop_daemon(failed_stop_config,
                                             failed_stop_client);
  assert(failed_stop_daemon.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(failed_stop_daemon.Handle(valid_move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  failed_stop_daemon.OnDisconnect();
  assert(!failed_stop_daemon.armed());
  assert(!failed_stop_daemon.moving());
  assert(failed_stop_daemon.faulted());
  assert(failed_stop_daemon.stop_unconfirmed());
  assert(failed_stop_client.stop_calls == 2);
  failed_stop_client.stop_result = 0;
  assert(failed_stop_daemon.CheckWatchdog(now + 2U));
  assert(failed_stop_client.stop_calls == 3);
  assert(!failed_stop_daemon.stop_unconfirmed());
  assert(failed_stop_daemon.faulted());
}

void TestClassicWalkPreservationUsesASeparateCleanPhase() {
  constexpr std::uint64_t now = 8'000'000'000ULL;
  go2_chassis::DaemonConfig config;
  config.allow_motion = true;
  config.repeatable_arm = true;
  config.preserve_classic_walk = true;
  config.max_vx = 0.05F;
  config.max_wz = 0.15F;
  config.max_motion_ns = 0U;

  FakeSportClient client;
  go2_chassis::DaemonCore daemon(config, client);
  const auto arm = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kArm, 1U, now, 100'000'000ULL);
  assert(daemon.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(client.prepare_arm_calls == 1);
  assert(client.classic_walk_calls == 1);
  assert(client.classic_walk_enabled);

  const auto move = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kMove, 2U, now + 1U, 100'000'000ULL,
      0.05F, 0.0F, 0.15F);
  assert(daemon.Handle(move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  const auto stop = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kStop, 3U, now + 2U, 100'000'000ULL);
  assert(daemon.Handle(stop, now + 2U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(client.stop_calls == 1);
  // StopMove remains the only SDK operation inside the stop IPC deadline.
  assert(client.classic_walk_calls == 1);

  const auto premature_restore = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kRestoreClassicWalk, 4U, now + 3U,
      100'000'000ULL);
  assert(daemon.Handle(premature_restore, now + 3U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kFaultLatched));
  assert(client.classic_walk_calls == 1);

  const auto disarm = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kDisarm, 5U, now + 4U, 100'000'000ULL);
  assert(daemon.Handle(disarm, now + 4U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(client.stop_calls == 2);
  assert(client.classic_walk_calls == 1);
  const auto restore = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kRestoreClassicWalk, 6U, now + 5U,
      100'000'000ULL);
  assert(daemon.Handle(restore, now + 5U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(client.classic_walk_calls == 2);

  FakeSportClient failed_stop_client;
  go2_chassis::DaemonCore failed_stop(config, failed_stop_client);
  assert(failed_stop.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(failed_stop.Handle(move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  failed_stop_client.stop_result = -1;
  assert(failed_stop.Handle(stop, now + 2U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kSdkError));
  assert(failed_stop.stop_unconfirmed());
  assert(failed_stop_client.classic_walk_calls == 1);
  assert(failed_stop.Handle(premature_restore, now + 3U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kFaultLatched));
  assert(failed_stop_client.classic_walk_calls == 1);

  FakeSportClient watchdog_client;
  go2_chassis::DaemonCore watchdog(config, watchdog_client);
  assert(watchdog.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(watchdog.Handle(move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(watchdog.CheckWatchdog(now + config.watchdog_ns + 2U));
  assert(watchdog_client.stop_calls == 1);
  assert(watchdog_client.classic_walk_calls == 1);

  FakeSportClient failed_classic_client;
  failed_classic_client.classic_walk_result = -1;
  go2_chassis::DaemonCore failed_classic(config, failed_classic_client);
  assert(failed_classic.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kSdkError));
  assert(!failed_classic.armed());
  assert(failed_classic.faulted());
  assert(failed_classic_client.stop_calls == 0);
  assert(failed_classic_client.classic_walk_calls == 1);

  FakeSportClient failed_restore_client;
  go2_chassis::DaemonCore failed_restore(config, failed_restore_client);
  assert(failed_restore.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(failed_restore.Handle(disarm, now + 4U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  failed_restore_client.classic_walk_result = -1;
  assert(failed_restore.Handle(restore, now + 5U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kSdkError));
  assert(!failed_restore.armed());
  assert(!failed_restore.faulted());
  assert(!failed_restore.stop_unconfirmed());
  assert(failed_restore_client.stop_calls == 1);
  assert(failed_restore_client.classic_walk_calls == 2);
}

void TestStagedNav2DaemonEnvelopeStopsAndDisarms() {
  constexpr std::uint64_t now = 9'000'000'000ULL;
  const auto staged_config = [] {
    go2_chassis::DaemonConfig config;
    config.allow_motion = true;
    config.repeatable_arm = true;
    config.max_vx = 0.05F;
    config.max_vy = 0.0F;
    config.max_wz = 0.15F;
    config.watchdog_ns = 300'000'000ULL;
    config.max_motion_ns = 0U;
    return config;
  };

  FakeSportClient accepted_client;
  const auto accepted_config = staged_config();
  go2_chassis::DaemonCore accepted(accepted_config, accepted_client);
  const auto arm = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kArm, 1U, now, 100'000'000ULL);
  assert(accepted.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  const auto move = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kMove, 2U, now + 1U, 100'000'000ULL, 0.05F,
      0.0F, 0.15F);
  assert(accepted.Handle(move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(accepted_client.move_calls == 1);
  const auto disarm = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kDisarm, 3U, now + 2U, 100'000'000ULL);
  assert(accepted.Handle(disarm, now + 2U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(accepted_client.stop_calls == 1);
  assert(!accepted.armed());
  assert(!accepted.faulted());

  const auto rearm = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kArm, 4U, now + 3U, 100'000'000ULL);
  assert(accepted.Handle(rearm, now + 3U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(accepted_client.prepare_arm_calls == 2);
  assert(accepted.armed());

  const auto expect_rejected = [now, &staged_config](float vx, float vy,
                                                     float wz) {
    FakeSportClient client;
    go2_chassis::DaemonCore daemon(staged_config(), client);
    const auto local_arm = go2_chassis::MakeCommand(
        go2_chassis::CommandOp::kArm, 1U, now, 100'000'000ULL);
    assert(daemon.Handle(local_arm, now).code ==
           static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
    const auto unsafe = go2_chassis::MakeCommand(
        go2_chassis::CommandOp::kMove, 2U, now + 1U, 100'000'000ULL, vx, vy,
        wz);
    assert(daemon.Handle(unsafe, now + 1U).code ==
           static_cast<std::int32_t>(
               go2_chassis::ReplyCode::kLimitExceeded));
    assert(client.move_calls == 0);
    assert(client.stop_calls == 1);
    assert(!daemon.armed());
    assert(daemon.faulted());
  };
  expect_rejected(-0.001F, 0.0F, 0.0F);
  expect_rejected(0.0501F, 0.0F, 0.0F);
  expect_rejected(0.01F, 0.001F, 0.0F);
  expect_rejected(0.01F, 0.0F, 0.1501F);

  FakeSportClient disconnect_client;
  go2_chassis::DaemonCore disconnect(staged_config(), disconnect_client);
  assert(disconnect.Handle(arm, now).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(disconnect.Handle(move, now + 1U).code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  disconnect.OnDisconnect();
  assert(!disconnect.armed());
  assert(!disconnect.moving());
  assert(disconnect_client.stop_calls == 1);
}

}  // namespace

int main() {
  TestProtocol();
  TestMotionWatchdogProfileGate();
  TestMotionIpcTimingContract();
  TestGuardDefaultsFailClosed();
  TestUnknownDefaultModeCannotArm();
  TestGuardPreparationLimitsAndFaultLatch();
  TestSourceStampLivenessFailClosed();
  TestExternalOdometryCanonicalInterlock();
  TestOpaqueFirmwareStateMarkerPolicyFailsClosedOnChange();
  TestMarkerChangeRequiresDisarmBeforeRearm();
  TestRpcAcknowledgementMustMatchFullIdentityLeaseAndStatus();
  TestDaemonCannotArmWithoutClientPreflight();
  TestDaemonDefenseInDepthAndWatchdog();
  TestClassicWalkPreservationUsesASeparateCleanPhase();
  TestStagedNav2DaemonEnvelopeStopsAndDisarms();
  std::cout << "protocol_guard_test: PASS\n";
  return 0;
}
