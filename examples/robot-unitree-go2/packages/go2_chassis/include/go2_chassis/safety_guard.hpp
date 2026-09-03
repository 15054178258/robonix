#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <utility>

namespace go2_chassis {

// Receipt and source freshness are separate contracts.  DDS/Wi-Fi callback
// receipt may be delayed by up to one second in the staged Nav2 deployment,
// while source timestamps remain constrained to the audited 200 ms window.
// Profile-specific envelopes below keep commissioning profiles at 200 ms.
inline constexpr double kMaximumStateTimeoutSec = 1.0;
inline constexpr double kDefaultStateTimeoutSec = 0.20;
inline constexpr double kMaximumSourceStampAgeSec = 0.20;
inline constexpr double kMaximumSourceStampFutureSkewSec = 0.05;
inline constexpr double kPassiveStateMarkerRecoveryStableSec = 0.50;
inline constexpr std::size_t kPassiveStateMarkerRecoveryMinimumSamples = 5U;

enum class GuardState {
  kDisabled,
  kDisarmed,
  kPreparing,
  kArmed,
  kFault,
};

enum class GuardAction {
  kNone,
  kMove,
  kStop,
};

struct Velocity {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct GuardConfig {
  bool allow_motion{false};
  double state_timeout_sec{kDefaultStateTimeoutSec};
  double max_source_stamp_age_sec{kMaximumSourceStampAgeSec};
  double max_source_stamp_future_skew_sec{kMaximumSourceStampFutureSkewSec};
  double command_timeout_sec{0.25};
  double zero_preparation_sec{0.50};
  double max_vx{0.25};
  double max_vy{0.0};
  double max_wz{0.40};
  double max_linear_acceleration{0.30};
  double max_angular_acceleration{0.80};
  double zero_epsilon{1.0e-4};
  // Unitree's public source does not define mode semantics. The impossible
  // uint8 sentinel prevents motion until read-only auditing supplies modes.
  std::set<std::uint8_t> allowed_modes{255U};
  // These are opaque non-zero firmware compatibility markers, not modes and
  // not a declaration that the state is healthy. Zero remains the only value
  // accepted without an explicit operator-provided allowlist.
  std::set<std::uint32_t> allowed_state_markers{};
};

// Unitree's public message defines error_code separately from mode, but the
// observed EDU firmware emits undocumented non-zero values in this field.
// Treat an explicitly configured value only as an opaque compatibility marker;
// never infer a mode, health state, or RPC result from its number.
inline bool CanonicalStateOutputEligible(bool measurements_valid,
                                         std::uint32_t error_code,
                                         const std::set<std::uint32_t>
                                             &allowed_state_markers = {}) {
  return measurements_valid &&
         (error_code == 0U ||
          allowed_state_markers.count(error_code) == 1U);
}

// The observed Classic gait firmware alternates only between opaque markers
// 100 and 2010 while retaining one audited SportModeState.mode value.  This is
// deliberately a deployment predicate rather than inferred marker semantics:
// any other marker set, multiple allowed modes, profile, or odometry route
// keeps the motion transition option unavailable.
inline bool ClassicMotionStateMarkerTransitionDeploymentEligible(
    bool transition_enabled, bool allow_motion, bool staged_nav2_profile,
    bool external_verified_odom, bool passive_transition_enabled,
    const std::set<std::uint8_t> &allowed_modes,
    const std::set<std::uint32_t> &allowed_state_markers) {
  if (!transition_enabled) {
    return true;
  }
  return allow_motion && staged_nav2_profile && external_verified_odom &&
         !passive_transition_enabled && allowed_modes.size() == 1U &&
         allowed_state_markers.size() == 2U &&
         allowed_state_markers.count(100U) == 1U &&
         allowed_state_markers.count(2010U) == 1U;
}

// Binding the first eligible value makes a remote/controller state transition
// observable. By default, any later value change latches closed, even when both
// values are in the explicit allowlist. Separately gated deployments may opt in
// to transitions only when both the old and new markers are members of that
// reviewed set. Passive mapping may additionally quarantine and recover from an
// unknown marker; motion profiles must keep that recovery disabled so an
// unknown transition still latches. This policy is ROS-independent for offline
// safety tests.
class OpaqueFirmwareStateMarkerPolicy {
 public:
  explicit OpaqueFirmwareStateMarkerPolicy(
      std::set<std::uint32_t> allowed_state_markers = {},
      bool allow_allowlisted_transitions = false,
      bool allow_passive_recovery = false)
      : allowed_state_markers_(std::move(allowed_state_markers)),
        allow_allowlisted_transitions_(allow_allowlisted_transitions),
        allow_passive_recovery_(allow_passive_recovery) {}

  bool Observe(std::uint32_t marker, double observation_sec = 0.0) {
    current_marker_ = marker;
    has_current_marker_ = true;
    current_marker_allowed_ =
        CanonicalStateOutputEligible(true, marker, allowed_state_markers_);
    if (change_latched_) {
      return false;
    }
    if (passive_recovery_pending_) {
      return ObservePassiveRecovery(marker, observation_sec);
    }
    if (has_bound_marker_ && marker != bound_marker_) {
      const bool transition_is_explicitly_allowlisted =
          current_marker_allowed_ && allow_allowlisted_transitions_ &&
          allowed_state_markers_.count(bound_marker_) == 1U &&
          allowed_state_markers_.count(marker) == 1U;
      if (!transition_is_explicitly_allowlisted) {
        if (allow_passive_recovery_) {
          BeginPassiveRecovery();
          return false;
        }
        change_latched_ = true;
        return false;
      }
      bound_marker_ = marker;
      return true;
    }
    if (!current_marker_allowed_) {
      if (allow_passive_recovery_) {
        BeginPassiveRecovery();
      }
      return false;
    }
    if (!has_bound_marker_) {
      bound_marker_ = marker;
      has_bound_marker_ = true;
      return true;
    }
    return true;
  }

  bool CanAcknowledgeCurrent(std::string *message) const {
    if (!has_current_marker_) {
      SetMessage(message, "no firmware state marker has been observed");
      return false;
    }
    if (!current_marker_allowed_) {
      SetMessage(message,
                 "current firmware state marker is not explicitly allowed");
      return false;
    }
    return true;
  }

  bool AcknowledgeCurrent(std::string *message) {
    if (!CanAcknowledgeCurrent(message)) {
      return false;
    }
    bound_marker_ = current_marker_;
    has_bound_marker_ = true;
    change_latched_ = false;
    passive_recovery_pending_ = false;
    ResetPassiveRecoveryCandidate();
    SetMessage(message,
               "current opaque firmware state marker acknowledged by explicit "
               "disarm");
    return true;
  }

  bool current_marker_allowed() const { return current_marker_allowed_; }
  bool has_current_marker() const { return has_current_marker_; }
  bool change_latched() const { return change_latched_; }
  bool passive_recovery_pending() const {
    return passive_recovery_pending_;
  }
  bool has_passive_recovery_candidate() const {
    return has_passive_recovery_candidate_;
  }
  std::uint32_t passive_recovery_candidate() const {
    return passive_recovery_candidate_;
  }
  std::size_t passive_recovery_samples() const {
    return passive_recovery_samples_;
  }
  double passive_recovery_elapsed_sec() const {
    return has_passive_recovery_candidate_
               ? passive_recovery_last_sec_ - passive_recovery_start_sec_
               : 0.0;
  }
  bool has_bound_marker() const { return has_bound_marker_; }
  std::uint32_t current_marker() const { return current_marker_; }
  std::uint32_t bound_marker() const { return bound_marker_; }
  const std::set<std::uint32_t> &allowed_state_markers() const {
    return allowed_state_markers_;
  }
  bool allow_allowlisted_transitions() const {
    return allow_allowlisted_transitions_;
  }
  bool allow_passive_recovery() const { return allow_passive_recovery_; }

 private:
  void BeginPassiveRecovery() {
    passive_recovery_pending_ = true;
    ResetPassiveRecoveryCandidate();
  }

  void ResetPassiveRecoveryCandidate() {
    has_passive_recovery_candidate_ = false;
    passive_recovery_candidate_ = 0U;
    passive_recovery_samples_ = 0U;
    passive_recovery_start_sec_ = 0.0;
    passive_recovery_last_sec_ = 0.0;
  }

  void StartPassiveRecoveryCandidate(std::uint32_t marker,
                                     double observation_sec) {
    has_passive_recovery_candidate_ = true;
    passive_recovery_candidate_ = marker;
    passive_recovery_samples_ = 1U;
    passive_recovery_start_sec_ = observation_sec;
    passive_recovery_last_sec_ = observation_sec;
  }

  bool ObservePassiveRecovery(std::uint32_t marker,
                              double observation_sec) {
    // Recovery is deliberately narrower than ordinary canonical eligibility:
    // zero is normally acceptable, but it is not allowed to silently replace
    // a bound opaque marker in this exceptional passive profile. Only a value
    // in the explicit reviewed set may complete automatic recovery.
    const bool marker_explicitly_allowlisted =
        allowed_state_markers_.count(marker) == 1U;
    if (!marker_explicitly_allowlisted ||
        !std::isfinite(observation_sec) || observation_sec <= 0.0) {
      ResetPassiveRecoveryCandidate();
      return false;
    }
    const bool candidate_changed =
        !has_passive_recovery_candidate_ ||
        passive_recovery_candidate_ != marker;
    const bool clock_regressed =
        has_passive_recovery_candidate_ &&
        observation_sec < passive_recovery_last_sec_;
    const bool sample_gap_exceeded =
        has_passive_recovery_candidate_ &&
        observation_sec - passive_recovery_last_sec_ >
            kDefaultStateTimeoutSec;
    if (candidate_changed || clock_regressed || sample_gap_exceeded) {
      StartPassiveRecoveryCandidate(marker, observation_sec);
      return false;
    }

    ++passive_recovery_samples_;
    passive_recovery_last_sec_ = observation_sec;
    const bool stable_long_enough =
        passive_recovery_last_sec_ - passive_recovery_start_sec_ >=
        kPassiveStateMarkerRecoveryStableSec;
    if (passive_recovery_samples_ <
            kPassiveStateMarkerRecoveryMinimumSamples ||
        !stable_long_enough) {
      return false;
    }

    bound_marker_ = marker;
    has_bound_marker_ = true;
    passive_recovery_pending_ = false;
    ResetPassiveRecoveryCandidate();
    return true;
  }

  static void SetMessage(std::string *target, const std::string &value) {
    if (target != nullptr) {
      *target = value;
    }
  }

  std::set<std::uint32_t> allowed_state_markers_;
  bool allow_allowlisted_transitions_{false};
  bool allow_passive_recovery_{false};
  bool has_current_marker_{false};
  bool current_marker_allowed_{false};
  bool has_bound_marker_{false};
  bool change_latched_{false};
  bool passive_recovery_pending_{false};
  bool has_passive_recovery_candidate_{false};
  std::uint32_t current_marker_{0U};
  std::uint32_t bound_marker_{0U};
  std::uint32_t passive_recovery_candidate_{0U};
  std::size_t passive_recovery_samples_{0U};
  double passive_recovery_start_sec_{0.0};
  double passive_recovery_last_sec_{0.0};
};

// Automatic marker recovery is not a general fault-reset mechanism. It is
// available only to the motion-disabled external-odometry profile, and only
// when finite measurements show that the marker quarantine is the sole reason
// state is invalid. Every motion-capable or integrity-fault path stays latched.
inline bool PassiveStateMarkerPauseIsRecoverable(
    bool allow_motion, bool external_verified,
    bool allow_passive_state_marker_transitions, bool measurements_valid,
    bool passive_recovery_pending, bool marker_change_latched) {
  return !allow_motion && external_verified &&
         allow_passive_state_marker_transitions && measurements_valid &&
         passive_recovery_pending && !marker_change_latched;
}

enum class SourceStampFreshness {
  kFresh,
  kMalformed,
  kZero,
  kReferenceClockInvalid,
  kTooOld,
  kTooFarInFuture,
};

inline const char *SourceStampFreshnessName(SourceStampFreshness freshness) {
  switch (freshness) {
    case SourceStampFreshness::kFresh:
      return "fresh";
    case SourceStampFreshness::kMalformed:
      return "malformed";
    case SourceStampFreshness::kZero:
      return "zero";
    case SourceStampFreshness::kReferenceClockInvalid:
      return "reference_clock_invalid";
    case SourceStampFreshness::kTooOld:
      return "too_old";
    case SourceStampFreshness::kTooFarInFuture:
      return "too_far_in_future";
  }
  return "unknown";
}

// "not_received" is an expected startup/read-only state. A delayed or duplicate
// best-effort DDS sample is ignored without refreshing liveness; the independent
// receipt timeout still stops an armed controller if fresh samples cease.
inline bool SourceStampStatusIsFault(const std::string &status) {
  return status != "fresh" && status != "not_received" &&
         status != "too_old_ignored" && status != "duplicate_ignored";
}

// Preserve transient source-stamp failures until at least one diagnostics
// publication observes them. Without this latch, a 300 Hz fresh sample could
// overwrite a rejection before a 5 Hz diagnostics timer reports it.
class SourceStampDiagnosticTracker {
 public:
  void Observe(std::string status) {
    latest_status_ = std::move(status);
    if (SourceStampStatusIsFault(latest_status_)) {
      last_fault_status_ = latest_status_;
      ++total_faults_;
      ++faults_since_report_;
    }
  }

  const std::string &latest_status() const { return latest_status_; }
  const std::string &last_fault_status() const { return last_fault_status_; }
  std::uint64_t total_faults() const { return total_faults_; }
  std::uint64_t faults_since_report() const { return faults_since_report_; }
  bool active_fault() const {
    return SourceStampStatusIsFault(latest_status_);
  }
  bool fault_since_last_report() const { return faults_since_report_ > 0U; }
  void MarkReported() { faults_since_report_ = 0U; }

 private:
  std::string latest_status_{"not_received"};
  std::string last_fault_status_{"none"};
  std::uint64_t total_faults_{0U};
  std::uint64_t faults_since_report_{0U};
};

// Validate an already well-formed source timestamp against the ROS clock used
// for canonical output. Subtraction is ordered to avoid unsigned underflow.
// Keeping this ROS-independent makes clock-skew and boundary cases testable
// without starting a ROS graph or contacting a robot.
inline SourceStampFreshness ValidateSourceStamp(
    std::uint64_t source_stamp_ns, std::int64_t reference_stamp_ns,
    std::uint64_t max_age_ns, std::uint64_t max_future_skew_ns) {
  if (source_stamp_ns == 0U) {
    return SourceStampFreshness::kZero;
  }
  if (reference_stamp_ns <= 0) {
    return SourceStampFreshness::kReferenceClockInvalid;
  }

  const auto reference_ns = static_cast<std::uint64_t>(reference_stamp_ns);
  if (source_stamp_ns > reference_ns) {
    return source_stamp_ns - reference_ns > max_future_skew_ns
               ? SourceStampFreshness::kTooFarInFuture
               : SourceStampFreshness::kFresh;
  }
  return reference_ns - source_stamp_ns > max_age_ns
             ? SourceStampFreshness::kTooOld
             : SourceStampFreshness::kFresh;
}

struct GuardDecision {
  GuardAction action{GuardAction::kNone};
  Velocity velocity{};
  GuardState state{GuardState::kDisarmed};
  std::string reason{"not initialized"};
};

// SportModeState is received over DDS, so receipt time alone is not proof that
// the robot is still producing live state.  Keep the source-stamp rule in this
// ROS-independent helper so the motion path can be tested offline.
class SourceStampTracker {
 public:
  bool Accept(std::uint64_t source_stamp_ns, bool require_strict_progress) {
    // Zero can never prove liveness and is rejected in every mode. The
    // freshness gate above is still mandatory before canonical publication.
    if (source_stamp_ns == 0U) {
      return false;
    }
    if (require_strict_progress) {
      if (last_source_stamp_ns_ != 0U &&
          source_stamp_ns <= last_source_stamp_ns_) {
        return false;
      }
      last_source_stamp_ns_ = source_stamp_ns;
      return true;
    }

    // Read-only bring-up permits repeated, fresh non-zero timestamps while the
    // dog is stationary. A timestamp is never allowed to move backwards.
    if (last_source_stamp_ns_ != 0U &&
        source_stamp_ns < last_source_stamp_ns_) {
      return false;
    }
    last_source_stamp_ns_ = source_stamp_ns;
    return true;
  }

  std::uint64_t last_source_stamp_ns() const {
    return last_source_stamp_ns_;
  }

 private:
  std::uint64_t last_source_stamp_ns_{0U};
};

inline bool SourceStampViolatesProgress(
    std::uint64_t source_stamp_ns, std::uint64_t last_source_stamp_ns,
    bool require_strict_progress) {
  if (source_stamp_ns == 0U || last_source_stamp_ns == 0U) {
    return false;
  }
  return require_strict_progress ? source_stamp_ns <= last_source_stamp_ns
                                 : source_stamp_ns < last_source_stamp_ns;
}

// Canonical external odometry is useful only while both its own receipt and
// the independent SportModeState health witness remain live.  Keep this
// evidence rule ROS-independent so the exact publication gate exercised by
// the adapter is covered by offline C++ tests.
inline bool FreshValidReceipt(bool valid, double last_receipt_sec,
                              double now_sec, double timeout_sec) {
  return valid && std::isfinite(last_receipt_sec) &&
         std::isfinite(now_sec) && std::isfinite(timeout_sec) &&
         last_receipt_sec > 0.0 && timeout_sec > 0.0 &&
         now_sec >= last_receipt_sec &&
         now_sec - last_receipt_sec <= timeout_sec;
}

inline bool ReceiptClockInvalidOrRegressed(
    double now_sec, double last_sport_state_receipt_sec,
    double last_external_odom_receipt_sec) {
  if (!std::isfinite(now_sec)) {
    return true;
  }
  const auto receipt_is_ahead = [now_sec](double receipt_sec) {
    return receipt_sec > 0.0 &&
           (!std::isfinite(receipt_sec) || now_sec < receipt_sec);
  };
  return receipt_is_ahead(last_sport_state_receipt_sec) ||
         receipt_is_ahead(last_external_odom_receipt_sec);
}

inline bool ExternalOdometryPoseContinuityEligible(
    bool has_previous_pose, double position_jump_m, double yaw_jump_rad,
    double max_position_jump_m, double max_yaw_jump_rad) {
  if (!has_previous_pose) {
    return true;
  }
  return std::isfinite(position_jump_m) && std::isfinite(yaw_jump_rad) &&
         std::isfinite(max_position_jump_m) &&
         std::isfinite(max_yaw_jump_rad) &&
         position_jump_m <= max_position_jump_m &&
         yaw_jump_rad <= max_yaw_jump_rad;
}

// A timeout is not a relocalization handshake for a motion-capable process.
// Every subsequent sample must remain inside the liveness and pose/yaw
// continuity envelopes. A separately gated passive process may clear only its
// pose-continuity epoch after pure liveness loss.
inline bool ExternalOdometryContinuityEligible(
    bool has_previous_pose, double elapsed_sec, double position_jump_m,
    double yaw_jump_rad, double timeout_sec, double max_position_jump_m,
    double max_yaw_jump_rad) {
  if (!has_previous_pose) {
    return true;
  }
  return std::isfinite(elapsed_sec) && std::isfinite(timeout_sec) &&
         elapsed_sec >= 0.0 && elapsed_sec <= timeout_sec &&
         ExternalOdometryPoseContinuityEligible(
             true, position_jump_m, yaw_jump_rad, max_position_jump_m,
             max_yaw_jump_rad);
}

// Passive stationary pose holding is deliberately narrower than a generic
// odometry filter.  It may only be enabled for a motion-disabled process that
// consumes the separately verified external odometry stream.  The limits are
// ceilings: deployment configuration may tighten them, but cannot widen what
// this offline-reviewed helper considers stationary.
inline constexpr double kStationaryHoldMinimumDwellSec = 1.0;
inline constexpr double kStationaryHoldMaximumDwellSec = 10.0;
inline constexpr double kStationaryHoldMaximumSportLinearMps = 0.03;
inline constexpr double kStationaryHoldMaximumSportYawRps = 0.03;
inline constexpr double kStationaryHoldMaximumExternalLinearMps = 0.03;
inline constexpr double kStationaryHoldMaximumExternalYawRps = 0.03;
inline constexpr double kStationaryHoldMaximumPoseLinearRateMps = 0.005;
inline constexpr double kStationaryHoldMaximumPoseYawRateRps = 0.01;

struct Se2Pose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double WrapYaw(double yaw) {
  constexpr double kTwoPi = 6.28318530717958647692;
  return std::remainder(yaw, kTwoPi);
}

inline bool FiniteSe2Pose(const Se2Pose &pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

inline Se2Pose ComposeSe2(const Se2Pose &left, const Se2Pose &right) {
  const double cosine = std::cos(left.yaw);
  const double sine = std::sin(left.yaw);
  return {left.x + cosine * right.x - sine * right.y,
          left.y + sine * right.x + cosine * right.y,
          WrapYaw(left.yaw + right.yaw)};
}

inline Se2Pose InverseSe2(const Se2Pose &pose) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {-cosine * pose.x - sine * pose.y,
          sine * pose.x - cosine * pose.y, WrapYaw(-pose.yaw)};
}

struct PassiveStationaryEvidence {
  bool sport_fresh_valid{false};
  double sport_linear_speed_mps{0.0};
  double sport_yaw_rate_rps{0.0};
  bool external_twist_valid{false};
  double external_linear_speed_mps{0.0};
  double external_yaw_rate_rps{0.0};
  bool external_pose_rate_valid{false};
  double external_pose_linear_rate_mps{0.0};
  double external_pose_yaw_rate_rps{0.0};
};

struct StationaryPoseHoldConfig {
  bool enabled{false};
  double dwell_sec{2.0};
  double sport_max_linear_mps{0.03};
  double sport_max_yaw_rate_rps{0.03};
  double external_twist_max_linear_mps{0.03};
  double external_twist_max_yaw_rate_rps{0.03};
  double pose_max_linear_rate_mps{0.005};
  double pose_max_yaw_rate_rps{0.01};
};

inline bool StationaryPoseHoldDeploymentEligible(
    bool enabled, bool allow_motion, bool external_verified,
    bool publish_odom_tf) {
  return !enabled ||
         (!allow_motion && external_verified && publish_odom_tf);
}

inline bool StationaryPoseHoldConfigValid(
    const StationaryPoseHoldConfig &config) {
  const auto bounded_positive = [](double value, double maximum) {
    return std::isfinite(value) && value > 0.0 && value <= maximum;
  };
  return std::isfinite(config.dwell_sec) &&
         config.dwell_sec >= kStationaryHoldMinimumDwellSec &&
         config.dwell_sec <= kStationaryHoldMaximumDwellSec &&
         bounded_positive(config.sport_max_linear_mps,
                          kStationaryHoldMaximumSportLinearMps) &&
         bounded_positive(config.sport_max_yaw_rate_rps,
                          kStationaryHoldMaximumSportYawRps) &&
         bounded_positive(config.external_twist_max_linear_mps,
                          kStationaryHoldMaximumExternalLinearMps) &&
         bounded_positive(config.external_twist_max_yaw_rate_rps,
                          kStationaryHoldMaximumExternalYawRps) &&
         bounded_positive(config.pose_max_linear_rate_mps,
                          kStationaryHoldMaximumPoseLinearRateMps) &&
         bounded_positive(config.pose_max_yaw_rate_rps,
                          kStationaryHoldMaximumPoseYawRateRps);
}

enum class StationaryPoseHoldState {
  kDisabled,
  kUnqualified,
  kQualifying,
  kHolding,
};

inline const char *StationaryPoseHoldStateName(
    StationaryPoseHoldState state) {
  switch (state) {
    case StationaryPoseHoldState::kDisabled:
      return "disabled";
    case StationaryPoseHoldState::kUnqualified:
      return "unqualified";
    case StationaryPoseHoldState::kQualifying:
      return "qualifying";
    case StationaryPoseHoldState::kHolding:
      return "holding";
  }
  return "unknown";
}

struct StationaryPoseHoldResult {
  bool valid{false};
  Se2Pose corrected{};
  bool zero_planar_twist{false};
  StationaryPoseHoldState state{StationaryPoseHoldState::kDisabled};
  bool sport_gate{false};
  bool twist_gate{false};
  bool pose_rate_gate{false};
  double qualification_elapsed_sec{0.0};
  Se2Pose correction{};
};

// Maintain a continuous SE(2) correction rather than clamping individual
// jumps.  While all three independent witnesses prove stationary for the
// dwell interval, the canonical pose remains anchored.  On the first sample
// that fails any witness, the accumulated correction is retained and the raw
// motion delta flows through without snapping back to the uncorrected origin.
class PassiveStationaryPoseHold {
 public:
  explicit PassiveStationaryPoseHold(
      StationaryPoseHoldConfig config = {})
      : config_(std::move(config)),
        state_(config_.enabled ? StationaryPoseHoldState::kUnqualified
                              : StationaryPoseHoldState::kDisabled) {}

  StationaryPoseHoldResult Update(
      double steady_now_sec, const Se2Pose &raw,
      const PassiveStationaryEvidence &evidence) {
    StationaryPoseHoldResult result;
    result.corrected = raw;
    result.state = state_;
    result.correction = correction_;
    if (!std::isfinite(steady_now_sec) || !FiniteSe2Pose(raw)) {
      BreakQualification();
      result.state = state_;
      return result;
    }

    if (!config_.enabled) {
      state_ = StationaryPoseHoldState::kDisabled;
      correction_ = {};
      last_update_sec_ = steady_now_sec;
      result.valid = true;
      result.state = state_;
      result.correction = correction_;
      return result;
    }

    const auto within = [](bool valid, double linear, double yaw,
                           double max_linear, double max_yaw) {
      return valid && std::isfinite(linear) && std::isfinite(yaw) &&
             linear >= 0.0 && yaw >= 0.0 && linear <= max_linear &&
             yaw <= max_yaw;
    };
    last_sport_gate_ =
        within(evidence.sport_fresh_valid,
               evidence.sport_linear_speed_mps,
               evidence.sport_yaw_rate_rps,
               config_.sport_max_linear_mps,
               config_.sport_max_yaw_rate_rps);
    last_twist_gate_ =
        within(evidence.external_twist_valid,
               evidence.external_linear_speed_mps,
               evidence.external_yaw_rate_rps,
               config_.external_twist_max_linear_mps,
               config_.external_twist_max_yaw_rate_rps);
    last_pose_rate_gate_ =
        within(evidence.external_pose_rate_valid,
               evidence.external_pose_linear_rate_mps,
               evidence.external_pose_yaw_rate_rps,
               config_.pose_max_linear_rate_mps,
               config_.pose_max_yaw_rate_rps);
    const bool clock_progressed =
        last_update_sec_ <= 0.0 || steady_now_sec >= last_update_sec_;
    const bool stationary = last_sport_gate_ && last_twist_gate_ &&
                            last_pose_rate_gate_ && clock_progressed;

    if (state_ == StationaryPoseHoldState::kHolding) {
      if (stationary) {
        correction_ = ComposeSe2(anchor_, InverseSe2(raw));
        result.corrected = anchor_;
        result.zero_planar_twist = true;
      } else {
        BreakQualification();
        result.corrected = ComposeSe2(correction_, raw);
      }
    } else {
      result.corrected = ComposeSe2(correction_, raw);
      if (!stationary) {
        BreakQualification();
      } else if (state_ == StationaryPoseHoldState::kQualifying) {
        qualification_elapsed_sec_ =
            steady_now_sec - qualification_start_sec_;
        if (qualification_elapsed_sec_ >= config_.dwell_sec) {
          anchor_ = result.corrected;
          correction_ = ComposeSe2(anchor_, InverseSe2(raw));
          state_ = StationaryPoseHoldState::kHolding;
          result.corrected = anchor_;
          result.zero_planar_twist = true;
        }
      } else {
        state_ = StationaryPoseHoldState::kQualifying;
        qualification_start_sec_ = steady_now_sec;
        qualification_elapsed_sec_ = 0.0;
      }
    }

    last_update_sec_ = steady_now_sec;
    result.valid = true;
    result.state = state_;
    result.sport_gate = last_sport_gate_;
    result.twist_gate = last_twist_gate_;
    result.pose_rate_gate = last_pose_rate_gate_;
    result.qualification_elapsed_sec = qualification_elapsed_sec_;
    result.correction = correction_;
    return result;
  }

  void BreakQualification() {
    state_ = config_.enabled ? StationaryPoseHoldState::kUnqualified
                             : StationaryPoseHoldState::kDisabled;
    qualification_start_sec_ = 0.0;
    qualification_elapsed_sec_ = 0.0;
  }

  bool enabled() const { return config_.enabled; }
  StationaryPoseHoldState state() const { return state_; }
  const Se2Pose &correction() const { return correction_; }
  double qualification_elapsed_sec() const {
    return qualification_elapsed_sec_;
  }
  bool sport_gate() const { return last_sport_gate_; }
  bool twist_gate() const { return last_twist_gate_; }
  bool pose_rate_gate() const { return last_pose_rate_gate_; }

 private:
  StationaryPoseHoldConfig config_;
  StationaryPoseHoldState state_{StationaryPoseHoldState::kDisabled};
  Se2Pose correction_{};
  Se2Pose anchor_{};
  double qualification_start_sec_{0.0};
  double qualification_elapsed_sec_{0.0};
  double last_update_sec_{0.0};
  bool last_sport_gate_{false};
  bool last_twist_gate_{false};
  bool last_pose_rate_gate_{false};
};

// There is intentionally no reset method.  Once this latch is set by a motion
// profile or a hard integrity/continuity failure, fresh traffic cannot silently
// re-enable canonical external odometry in the same process.
class ExternalOdometryFaultLatch {
 public:
  void Latch() { latched_ = true; }
  bool latched() const { return latched_; }

  bool CanonicalOutputEligible(
      bool sport_state_valid, double last_sport_state_receipt_sec,
      bool external_odom_valid, double last_external_odom_receipt_sec,
      double now_sec, double state_timeout_sec,
      double external_odom_timeout_sec) const {
    return !latched_ &&
           FreshValidReceipt(sport_state_valid,
                             last_sport_state_receipt_sec, now_sec,
                             state_timeout_sec) &&
           FreshValidReceipt(external_odom_valid,
                             last_external_odom_receipt_sec, now_sec,
                             external_odom_timeout_sec);
  }

 private:
 bool latched_{false};
};

// A process which is currently disarmed cannot produce motion and may
// re-acquire independently verified state/odometry streams after a pure
// liveness outage.  Once an arm/preparation session is open, the same loss
// must remain latched and require a reviewed restart.  Hard integrity faults
// always use ExternalOdometryFaultLatch directly and remain process-lifetime.
inline bool ExternalOdometryLivenessLossRequiresProcessRestart(
    bool allow_motion, bool motion_session_open) {
  return allow_motion && motion_session_open;
}

inline const char *GuardStateName(GuardState state) {
  switch (state) {
    case GuardState::kDisabled:
      return "DISABLED";
    case GuardState::kDisarmed:
      return "DISARMED";
    case GuardState::kPreparing:
      return "PREPARING";
    case GuardState::kArmed:
      return "ARMED";
    case GuardState::kFault:
      return "FAULT";
  }
  return "UNKNOWN";
}

class SafetyGuard {
 public:
  explicit SafetyGuard(GuardConfig config) : config_(std::move(config)) {
    state_ = config_.allow_motion ? GuardState::kDisarmed : GuardState::kDisabled;
    reason_ = config_.allow_motion ? "waiting for explicit arm request"
                                   : "motion disabled by configuration";
  }

  GuardState state() const { return state_; }
  const std::string &reason() const { return reason_; }
  bool motion_allowed() const { return config_.allow_motion; }

  void UpdateRobotState(double now_sec, std::uint8_t mode, bool valid) {
    has_state_ = true;
    last_state_sec_ = now_sec;
    state_mode_ = mode;
    state_valid_ = valid;
  }

  void UpdateCommand(double now_sec, const Velocity &command) {
    has_command_ = true;
    last_command_sec_ = now_sec;
    command_ = command;
    if (!Finite(command_)) {
      ForceFault("command contains NaN or infinity");
    }
  }

  bool RequestArm(double now_sec, std::string *message) {
    if (!config_.allow_motion) {
      SetMessage(message, "allow_motion is false");
      state_ = GuardState::kDisabled;
      return false;
    }
    if (state_ == GuardState::kFault) {
      SetMessage(message, "fault is latched; explicitly disarm before re-arming");
      return false;
    }
    if (state_ == GuardState::kArmed || state_ == GuardState::kPreparing) {
      SetMessage(message, "arm sequence is already active");
      return true;
    }
    const std::string health = HealthFailure(now_sec, true);
    if (!health.empty()) {
      SetMessage(message, health);
      return false;
    }
    if (!IsZero(command_)) {
      SetMessage(message, "a fresh zero command is required before arming");
      return false;
    }
    state_ = GuardState::kPreparing;
    preparation_start_sec_ = now_sec;
    output_ = {};
    reason_ = "holding zero command before arm";
    SetMessage(message, reason_);
    return true;
  }

  bool RequestDisarm(double now_sec, std::string *message) {
    const bool was_faulted = state_ == GuardState::kFault;
    output_ = {};
    preparation_start_sec_ = 0.0;
    if (!config_.allow_motion) {
      state_ = GuardState::kDisabled;
      reason_ = "motion disabled by configuration";
      SetMessage(message, reason_);
      return true;
    }
    if (was_faulted) {
      const std::string health = HealthFailure(now_sec, true);
      if (!health.empty() || !IsZero(command_)) {
        reason_ = health.empty() ? "zero command required to clear fault" : health;
        SetMessage(message, "disarmed; fault remains latched: " + reason_);
        return true;
      }
    }
    state_ = GuardState::kDisarmed;
    reason_ = was_faulted ? "fault cleared by explicit disarm" : "explicitly disarmed";
    SetMessage(message, reason_);
    return true;
  }

  GuardDecision Tick(double now_sec, double dt_sec) {
    GuardDecision decision;
    decision.state = state_;
    decision.reason = reason_;

    if (!config_.allow_motion) {
      state_ = GuardState::kDisabled;
      reason_ = "motion disabled by configuration";
      return StopDecision();
    }
    if (state_ == GuardState::kFault || state_ == GuardState::kDisabled) {
      return StopDecision();
    }
    if (state_ == GuardState::kDisarmed) {
      reason_ = "waiting for explicit arm request";
      return StopDecision();
    }

    const std::string health = HealthFailure(now_sec, true);
    if (!health.empty()) {
      ForceFault(health);
      return StopDecision();
    }

    if (state_ == GuardState::kPreparing) {
      if (!IsZero(command_)) {
        preparation_start_sec_ = now_sec;
        reason_ = "zero preparation restarted after non-zero command";
        return StopDecision();
      }
      if (now_sec - preparation_start_sec_ < config_.zero_preparation_sec) {
        reason_ = "holding zero command before arm";
        return StopDecision();
      }
      state_ = GuardState::kArmed;
      last_tick_sec_ = now_sec;
      reason_ = "armed; watchdogs active";
    }

    if (state_ != GuardState::kArmed) {
      return StopDecision();
    }

    const double safe_dt = std::clamp(
        dt_sec > 0.0 ? dt_sec : now_sec - last_tick_sec_, 0.0, 0.10);
    last_tick_sec_ = now_sec;
    Velocity target = Clamp(command_);
    output_.vx = Slew(output_.vx, target.vx,
                      config_.max_linear_acceleration * safe_dt);
    output_.vy = Slew(output_.vy, target.vy,
                      config_.max_linear_acceleration * safe_dt);
    output_.wz = Slew(output_.wz, target.wz,
                      config_.max_angular_acceleration * safe_dt);

    if (IsZero(target) && IsZero(output_)) {
      output_ = {};
      reason_ = "armed and stopped";
      return StopDecision();
    }
    decision.action = GuardAction::kMove;
    decision.velocity = output_;
    decision.state = state_;
    decision.reason = "armed; bounded velocity command";
    reason_ = decision.reason;
    return decision;
  }

  void ForceFault(const std::string &reason) {
    if (!config_.allow_motion) {
      state_ = GuardState::kDisabled;
      reason_ = "motion disabled by configuration";
      output_ = {};
      return;
    }
    state_ = GuardState::kFault;
    reason_ = reason;
    output_ = {};
  }

 private:
  static void SetMessage(std::string *target, const std::string &value) {
    if (target != nullptr) {
      *target = value;
    }
  }

  static bool Finite(const Velocity &velocity) {
    return std::isfinite(velocity.vx) && std::isfinite(velocity.vy) &&
           std::isfinite(velocity.wz);
  }

  bool IsZero(const Velocity &velocity) const {
    return std::fabs(velocity.vx) <= config_.zero_epsilon &&
           std::fabs(velocity.vy) <= config_.zero_epsilon &&
           std::fabs(velocity.wz) <= config_.zero_epsilon;
  }

  std::string HealthFailure(double now_sec, bool require_command) const {
    if (!has_state_) {
      return "no SportModeState received";
    }
    if (!state_valid_) {
      return "SportModeState is invalid";
    }
    if (now_sec - last_state_sec_ < 0.0 ||
        now_sec - last_state_sec_ > config_.state_timeout_sec) {
      return "SportModeState watchdog expired";
    }
    if (config_.allowed_modes.count(state_mode_) == 0U) {
      return "SportModeState mode is not allowed";
    }
    if (require_command && !has_command_) {
      return "no velocity command received";
    }
    if (require_command &&
        (now_sec - last_command_sec_ < 0.0 ||
         now_sec - last_command_sec_ > config_.command_timeout_sec)) {
      return "velocity command watchdog expired";
    }
    if (!Finite(command_)) {
      return "velocity command is invalid";
    }
    return {};
  }

  Velocity Clamp(const Velocity &velocity) const {
    return {
        std::clamp(velocity.vx, 0.0, config_.max_vx),
        std::clamp(velocity.vy, -config_.max_vy, config_.max_vy),
        std::clamp(velocity.wz, -config_.max_wz, config_.max_wz),
    };
  }

  static double Slew(double current, double target, double maximum_delta) {
    if (maximum_delta <= 0.0) {
      return current;
    }
    return current + std::clamp(target - current, -maximum_delta, maximum_delta);
  }

  GuardDecision StopDecision() const {
    return {GuardAction::kStop, {}, state_, reason_};
  }

  GuardConfig config_;
  GuardState state_{GuardState::kDisabled};
  std::string reason_{"motion disabled by configuration"};
  bool has_state_{false};
  bool has_command_{false};
  bool state_valid_{false};
  std::uint8_t state_mode_{0};
  double last_state_sec_{0.0};
  double last_command_sec_{0.0};
  double preparation_start_sec_{0.0};
  double last_tick_sec_{0.0};
  Velocity command_{};
  Velocity output_{};
};

}  // namespace go2_chassis
