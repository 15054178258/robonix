#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "go2_chassis/motion_timing.hpp"
#include "go2_chassis/protocol.hpp"
#include "go2_chassis/runtime_graph.hpp"
#include "go2_chassis/safety_guard.hpp"

namespace go2_chassis {
namespace {

double SteadyNowSec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::uint64_t SteadyNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool Finite(double value) { return std::isfinite(value); }

constexpr const char *kFirstMotionStateTopic =
    "/robonix/time_corrected/motion/sportmodestate";
constexpr const char *kFirstMotionCommandTopic =
    "/go2/commissioning/cmd_vel";
constexpr const char *kFirstMotionOdomTopic = "/odom";
constexpr const char *kFirstMotionProfile =
    "workstation-first-motion-corrected-v1";
constexpr double kFirstMotionMaxVx = 0.05;
constexpr double kFirstMotionMaxVy = 0.0;
constexpr double kFirstMotionMaxWz = 0.0;
constexpr double kFirstMotionCommandTimeoutSec = 0.20;
constexpr double kCommissioningStateTimeoutSec = 0.20;
constexpr double kFirstMotionMaxDurationSec = 2.0;
constexpr double kFirstMotionMaxDistanceM = 0.10;
constexpr const char *kSecondMotionProfile =
    "workstation-second-motion-corrected-v1";
constexpr const char *kSecondMotionCommandTopic =
    "/go2/second_motion/cmd_vel";
constexpr double kSecondMotionMaxVx = 0.30;
constexpr double kSecondMotionMaxVy = 0.0;
constexpr double kSecondMotionMaxWz = 0.0;
constexpr double kSecondMotionMaxLinearAcceleration = 0.30;
constexpr double kSecondMotionMaxAngularAcceleration = 0.10;
constexpr double kSecondMotionCommandTimeoutSec = 0.20;
constexpr double kSecondMotionMaxDurationSec = 1.5;
constexpr double kSecondMotionMaxDistanceM = 0.30;
constexpr const char *kStagedNav2Profile =
    "workstation-staged-nav2-corrected-v1";
constexpr const char *kStagedNav2CommandTopic =
    "/go2/staged_nav2/cmd_vel";
constexpr double kStagedNav2MaxVx = 0.30;
constexpr double kStagedNav2MaxVy = 0.0;
constexpr double kStagedNav2MaxWz = 0.40;
constexpr double kStagedNav2MaxLinearAcceleration = 0.30;
constexpr double kStagedNav2MaxAngularAcceleration = 0.80;
constexpr double kStagedNav2CommandTimeoutSec = 0.20;
constexpr double kStagedNav2StateTimeoutSec = 1.0;
constexpr double kStagedNav2ExternalOdomTimeoutSec = 1.0;
constexpr double kStagedNav2MaxDurationSec = 0.0;
constexpr double kStagedNav2MaxDistanceM = 0.0;
constexpr const char *kStagedNav2ExternalOdomTopic =
    "/robonix/time_corrected/raw/utlidar/robot_odom";
constexpr double kTwoPi = 6.28318530717958647692;
constexpr const char *kExternalOdomTopicPrefix =
    "/robonix/time_corrected/";

enum class OdomSource { kSportState, kExternalVerified };

const char *OdomSourceName(OdomSource source) {
  return source == OdomSource::kExternalVerified ? "external_verified"
                                                  : "sport_state";
}

OdomSource ParseOdomSource(const std::string &value) {
  if (value == "sport_state") {
    return OdomSource::kSportState;
  }
  if (value == "external_verified") {
    return OdomSource::kExternalVerified;
  }
  throw std::runtime_error(
      "odom_source must be 'sport_state' or 'external_verified'");
}

diagnostic_msgs::msg::KeyValue DiagnosticValue(const std::string &key,
                                                const std::string &value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

class SeqpacketClient {
 public:
  explicit SeqpacketClient(std::string socket_path)
      : socket_path_(std::move(socket_path)) {}

  ~SeqpacketClient() { Close(); }

  bool connected() const { return descriptor_ >= 0; }

  bool Connect(std::string *error) {
    if (connected()) {
      return true;
    }
    if (socket_path_.empty() || socket_path_.size() >= sizeof(sockaddr_un::sun_path)) {
      SetError(error, "IPC socket path is empty or too long");
      return false;
    }
    descriptor_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (descriptor_ < 0) {
      SetError(error, "cannot create IPC socket");
      return false;
    }
    timeval timeout {};
    timeout.tv_sec = kMotionArmIpcReplyTimeoutMs / 1000;
    timeout.tv_usec = (kMotionArmIpcReplyTimeoutMs % 1000) * 1000;
    (void)::setsockopt(descriptor_, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                       sizeof(timeout));
    (void)::setsockopt(descriptor_, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                       sizeof(timeout));

    sockaddr_un address {};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path_.c_str(),
                 sizeof(address.sun_path) - 1U);
    if (::connect(descriptor_, reinterpret_cast<const sockaddr *>(&address),
                  sizeof(address)) != 0) {
      SetError(error, "cannot connect to SDK daemon IPC socket");
      Close();
      return false;
    }

    ucred credentials {};
    socklen_t length = sizeof(credentials);
    if (::getsockopt(descriptor_, SOL_SOCKET, SO_PEERCRED, &credentials,
                     &length) != 0 ||
        length != sizeof(credentials) || credentials.uid != ::geteuid()) {
      SetError(error, "SDK daemon IPC peer UID does not match");
      Close();
      return false;
    }
    return true;
  }

  bool Exchange(CommandOp operation, const Velocity &velocity,
                std::int32_t *reply_code, std::string *error) {
    if (!connected() && !Connect(error)) {
      return false;
    }
    const std::uint64_t now = SteadyNowNs();
    const CommandPacket command = MakeCommand(
        operation, ++sequence_, now, 100'000'000ULL,
        static_cast<float>(velocity.vx), static_cast<float>(velocity.vy),
        static_cast<float>(velocity.wz));
    const ssize_t sent =
        ::send(descriptor_, &command, sizeof(command), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(sizeof(command))) {
      SetError(error, "incomplete IPC command write");
      Close();
      return false;
    }

    pollfd poll_descriptor {descriptor_, POLLIN, 0};
    const int reply_timeout_ms = MotionIpcReplyTimeoutMs(operation);
    const int poll_result =
        ::poll(&poll_descriptor, 1, reply_timeout_ms);
    if (poll_result != 1 || (poll_descriptor.revents & POLLIN) == 0) {
      SetError(error, "SDK daemon IPC reply timeout");
      Close();
      return false;
    }
    ReplyPacket reply {};
    const ssize_t received = ::recv(descriptor_, &reply, sizeof(reply), 0);
    if (received != static_cast<ssize_t>(sizeof(reply)) ||
        !ValidateReply(reply, command.sequence)) {
      SetError(error, "invalid SDK daemon IPC reply");
      Close();
      return false;
    }
    if (reply_code != nullptr) {
      *reply_code = reply.code;
    }
    if (reply.code != static_cast<std::int32_t>(ReplyCode::kOk)) {
      SetError(error, "SDK daemon rejected command with code " +
                          std::to_string(reply.code));
      return false;
    }
    return true;
  }

  void Close() {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
      descriptor_ = -1;
    }
  }

 private:
  static void SetError(std::string *target, const std::string &message) {
    if (target != nullptr) {
      *target = message;
    }
  }

  std::string socket_path_;
  int descriptor_{-1};
  std::uint64_t sequence_{0};
};

class Go2ChassisAdapterNode final : public rclcpp::Node {
 public:
  Go2ChassisAdapterNode() : Node("go2_chassis_adapter") {
    motion_profile_ =
        declare_parameter<std::string>("motion_profile", kFirstMotionProfile);
    const GuardConfig guard_config = DeclareGuardConfig();
    allow_motion_ = guard_config.allow_motion;
    preserve_classic_walk_ =
        declare_parameter<bool>("preserve_classic_walk", false);
    if (allow_motion_ && motion_profile_ != kFirstMotionProfile &&
        motion_profile_ != kSecondMotionProfile &&
        motion_profile_ != kStagedNav2Profile) {
      throw std::runtime_error(
          "motion requires an independently audited motion_profile");
    }
    if (preserve_classic_walk_ &&
        (!allow_motion_ || motion_profile_ != kStagedNav2Profile)) {
      throw std::runtime_error(
          "preserve_classic_walk requires the staged Nav2 motion profile");
    }
    guard_ = std::make_unique<SafetyGuard>(guard_config);

    sport_state_topic_ =
        declare_parameter<std::string>("sport_state_topic", "/sportmodestate");
    sport_state_fallback_topic_ = declare_parameter<std::string>(
        "state_fallback_topic", "/lf/sportmodestate");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    odom_source_ = ParseOdomSource(
        declare_parameter<std::string>("odom_source", "sport_state"));
    allow_passive_state_marker_transitions_ = declare_parameter<bool>(
        "allow_passive_state_marker_transitions", false);
    allow_motion_state_marker_transitions_ = declare_parameter<bool>(
        "allow_motion_state_marker_transitions", false);
    external_odom_topic_ = declare_parameter<std::string>(
        "external_odom_topic",
        "/robonix/time_corrected/raw/utlidar/robot_odom");
    external_odom_timeout_sec_ = declare_parameter<double>(
        "external_odom_timeout_sec", kCommissioningStateTimeoutSec);
    max_external_odom_yaw_jump_rad_ = declare_parameter<double>(
        "max_external_odom_yaw_jump_rad", 1.0);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
    StationaryPoseHoldConfig stationary_hold_config;
    stationary_hold_config.enabled = declare_parameter<bool>(
        "stationary_pose_hold_enabled", false);
    stationary_hold_config.dwell_sec = declare_parameter<double>(
        "stationary_hold_dwell_sec", 2.0);
    stationary_hold_config.sport_max_linear_mps = declare_parameter<double>(
        "stationary_hold_sport_max_linear_mps", 0.03);
    stationary_hold_config.sport_max_yaw_rate_rps =
        declare_parameter<double>("stationary_hold_sport_max_yaw_rps", 0.03);
    stationary_hold_config.external_twist_max_linear_mps =
        declare_parameter<double>(
            "stationary_hold_external_twist_max_linear_mps", 0.03);
    stationary_hold_config.external_twist_max_yaw_rate_rps =
        declare_parameter<double>(
            "stationary_hold_external_twist_max_yaw_rps", 0.03);
    stationary_hold_config.pose_max_linear_rate_mps =
        declare_parameter<double>(
            "stationary_hold_pose_max_linear_rate_mps", 0.005);
    stationary_hold_config.pose_max_yaw_rate_rps =
        declare_parameter<double>(
            "stationary_hold_pose_max_yaw_rate_rps", 0.01);
    if (!StationaryPoseHoldConfigValid(stationary_hold_config)) {
      throw std::runtime_error(
          "stationary pose hold thresholds exceed no-motion safety ceilings");
    }
    if (!StationaryPoseHoldDeploymentEligible(
            stationary_hold_config.enabled, allow_motion_,
            odom_source_ == OdomSource::kExternalVerified,
            publish_odom_tf_)) {
      throw std::runtime_error(
          "stationary_pose_hold_enabled requires motion disabled, "
          "external_verified odometry, and chassis-owned TF");
    }
    stationary_pose_hold_ =
        std::make_unique<PassiveStationaryPoseHold>(stationary_hold_config);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    status_topic_ =
        declare_parameter<std::string>("status_topic", "/go2_chassis/status");
    diagnostics_topic_ = declare_parameter<std::string>(
        "diagnostics_topic", "/diagnostics");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
    velocity_frame_ =
        declare_parameter<std::string>("state_velocity_frame", "odom");
    if (velocity_frame_ != "odom" && velocity_frame_ != "base_link") {
      throw std::runtime_error(
          "state_velocity_frame must be 'odom' or 'base_link'");
    }
    max_position_jump_m_ =
        declare_parameter<double>("max_position_jump_m", 1.0);
    if (!Finite(max_position_jump_m_) || max_position_jump_m_ <= 0.0) {
      throw std::runtime_error("max_position_jump_m must be positive");
    }
    if (!Finite(external_odom_timeout_sec_) ||
        external_odom_timeout_sec_ <= 0.0 ||
        external_odom_timeout_sec_ > kMaximumStateTimeoutSec ||
        !Finite(max_external_odom_yaw_jump_rad_) ||
        max_external_odom_yaw_jump_rad_ <= 0.0 ||
        max_external_odom_yaw_jump_rad_ > 1.0) {
      throw std::runtime_error(
          "external odom timeout/yaw-jump limits exceed safety ceilings");
    }
    if (odom_source_ == OdomSource::kExternalVerified) {
      if (external_odom_topic_ == odom_topic_) {
        throw std::runtime_error(
            "external_odom_topic must differ from canonical odom_topic");
      }
      if (external_odom_topic_.rfind(kExternalOdomTopicPrefix, 0U) != 0U) {
        throw std::runtime_error(
            "external_verified odom requires a private time-corrected topic");
      }
      if (odom_topic_ != kFirstMotionOdomTopic || !publish_odom_tf_) {
        throw std::runtime_error(
            "external_verified odom requires chassis-owned /odom and TF");
      }
    }
    const double required_external_odom_timeout =
        motion_profile_ == kStagedNav2Profile
            ? kStagedNav2ExternalOdomTimeoutSec
            : kCommissioningStateTimeoutSec;
    if (allow_motion_ &&
        (motion_profile_ == kSecondMotionProfile ||
         motion_profile_ == kStagedNav2Profile) &&
        (odom_source_ != OdomSource::kExternalVerified ||
         external_odom_topic_ != kStagedNav2ExternalOdomTopic ||
         external_odom_timeout_sec_ != required_external_odom_timeout)) {
      throw std::runtime_error(
          "later motion profiles require the exact corrected external "
          "odometry stream and profile-specific receipt timeout");
    }
    if (allow_passive_state_marker_transitions_) {
      if (allow_motion_ || odom_source_ != OdomSource::kExternalVerified) {
        throw std::runtime_error(
            "allow_passive_state_marker_transitions requires motion disabled "
            "and external_verified odometry");
      }
      if (guard_config.allowed_state_markers.size() < 2U) {
        throw std::runtime_error(
            "passive state marker transitions require at least two explicit "
            "allowed_state_markers");
      }
    }
    if (!ClassicMotionStateMarkerTransitionDeploymentEligible(
            allow_motion_state_marker_transitions_, allow_motion_,
            motion_profile_ == kStagedNav2Profile,
            odom_source_ == OdomSource::kExternalVerified,
            allow_passive_state_marker_transitions_,
            guard_config.allowed_modes, guard_config.allowed_state_markers)) {
      throw std::runtime_error(
          "allow_motion_state_marker_transitions requires staged Nav2, "
          "external_verified odometry, one audited mode, and the exact "
          "Classic marker allowlist {100,2010}");
    }
    state_marker_policy_ =
        std::make_unique<OpaqueFirmwareStateMarkerPolicy>(
            guard_config.allowed_state_markers,
            allow_passive_state_marker_transitions_ ||
                allow_motion_state_marker_transitions_,
            allow_passive_state_marker_transitions_);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic_, 10);
    if (publish_odom_tf_) {
      tf_broadcaster_ =
          std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    state_qos.best_effort().durability_volatile();
    // Match the corrected motion relays' depth-one QoS so an IPC-heavy control
    // callback cannot leave older state or odometry queued behind fresh data.
    auto corrected_motion_input_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    corrected_motion_input_qos.best_effort().durability_volatile();
    const auto &sport_state_qos =
        allow_motion_ ? corrected_motion_input_qos : state_qos;
    sport_state_subscription_ =
        create_subscription<unitree_go::msg::SportModeState>(
            sport_state_topic_, sport_state_qos,
            [this](const unitree_go::msg::SportModeState::SharedPtr message) {
              OnSportModeState(*message, true);
            });
    if (!sport_state_fallback_topic_.empty() &&
        sport_state_fallback_topic_ != sport_state_topic_) {
      sport_state_fallback_subscription_ =
          create_subscription<unitree_go::msg::SportModeState>(
              sport_state_fallback_topic_, state_qos,
              [this](
                  const unitree_go::msg::SportModeState::SharedPtr message) {
                OnSportModeState(*message, false);
              });
    }
    if (odom_source_ == OdomSource::kExternalVerified) {
      external_odom_subscription_ =
          create_subscription<nav_msgs::msg::Odometry>(
              external_odom_topic_, corrected_motion_input_qos,
              [this](const nav_msgs::msg::Odometry::SharedPtr message) {
                OnExternalOdometry(*message);
              });
    }
    if (allow_motion_ &&
        (sport_state_topic_ != kFirstMotionStateTopic ||
         !sport_state_fallback_topic_.empty() ||
         odom_topic_ != kFirstMotionOdomTopic)) {
      throw std::runtime_error(
          "motion profiles require the dedicated corrected state topic, no "
          "fallback, and canonical /odom");
    }
    const RuntimeGraphPlan graph_plan = RuntimeGraphPlan::For(allow_motion_);
    if (!graph_plan.is_consistent()) {
      throw std::logic_error("incomplete chassis motion-control graph plan");
    }
    if (graph_plan.has_complete_motion_control_graph()) {
      InitializeMotionControlGraph();
    }
    diagnostics_timer_ = create_wall_timer(
        std::chrono::milliseconds(200), [this]() { PublishDiagnostics(); });

    if (allow_motion_) {
      RCLCPP_WARN(get_logger(),
                  "Go2 chassis adapter started CONFIGURED-BUT-DISARMED. No "
                  "motion can occur without an enabled SDK daemon and "
                  "explicit arm service call.");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Go2 chassis adapter started in PASSIVE mode: no IPC client, "
                  "cmd_vel subscription, arm service, or control timer exists.");
    }
    RCLCPP_INFO(get_logger(), "Canonical odometry source: %s",
                OdomSourceName(odom_source_));
  }

  ~Go2ChassisAdapterNode() override {
    if (motion_control_graph_initialized_ && allow_motion_) {
      commissioning_motion_active_ = false;
      commissioning_stop_reason_ = "adapter_shutdown";
      (void)BestEffortDaemonDisarm();
    }
  }

 private:
  void InitializeMotionControlGraph() {
    if (!allow_motion_) {
      throw std::logic_error(
          "refusing to initialize motion-control graph while motion is disabled");
    }

    cmd_vel_topic_ =
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const std::string expected_command_topic =
        motion_profile_ == kFirstMotionProfile
            ? kFirstMotionCommandTopic
            : (motion_profile_ == kSecondMotionProfile
                   ? kSecondMotionCommandTopic
                   : kStagedNav2CommandTopic);
    if (cmd_vel_topic_ != expected_command_topic) {
      throw std::runtime_error(
          "cmd_vel_topic does not match the selected motion profile");
    }
    arm_service_name_ = declare_parameter<std::string>(
        "arm_service", "/go2_chassis/arm");
    const std::string socket_path = declare_parameter<std::string>(
        "sdk_socket", "/tmp/robonix-go2-disabled.sock");
    const double requested_control_rate =
        declare_parameter<double>("control_rate_hz", 50.0);
    if (!Finite(requested_control_rate) || requested_control_rate <= 0.0) {
      throw std::runtime_error("control_rate_hz must be finite and positive");
    }
    const double control_rate =
        std::clamp(requested_control_rate, 10.0, 100.0);
    control_period_sec_ = 1.0 / control_rate;
    commissioning_max_duration_sec_ = declare_parameter<double>(
        "commissioning_max_duration_sec", kFirstMotionMaxDurationSec);
    commissioning_max_distance_m_ = declare_parameter<double>(
        "commissioning_max_distance_m", kFirstMotionMaxDistanceM);
    const double profile_max_duration =
        motion_profile_ == kFirstMotionProfile
            ? kFirstMotionMaxDurationSec
            : (motion_profile_ == kSecondMotionProfile
                   ? kSecondMotionMaxDurationSec
                   : kStagedNav2MaxDurationSec);
    const double profile_max_distance =
        motion_profile_ == kFirstMotionProfile
            ? kFirstMotionMaxDistanceM
            : (motion_profile_ == kSecondMotionProfile
                   ? kSecondMotionMaxDistanceM
                   : kStagedNav2MaxDistanceM);
    const bool standard_navigation_profile =
        motion_profile_ == kStagedNav2Profile;
    if (!Finite(commissioning_max_duration_sec_) ||
        !Finite(commissioning_max_distance_m_) ||
        (!standard_navigation_profile &&
         commissioning_max_duration_sec_ <= 0.0) ||
        commissioning_max_duration_sec_ != profile_max_duration ||
        (!standard_navigation_profile &&
         commissioning_max_distance_m_ <= 0.0) ||
        commissioning_max_distance_m_ != profile_max_distance) {
      throw std::runtime_error(
          "motion duration/distance envelope does not match the selected profile");
    }

    ipc_ = std::make_unique<SeqpacketClient>(socket_path);
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const geometry_msgs::msg::Twist::SharedPtr message) {
          OnVelocityCommand(*message);
        });
    arm_service_ = create_service<std_srvs::srv::SetBool>(
        arm_service_name_,
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          OnArmRequest(request->data, *response);
        });
    const auto control_period =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(control_period_sec_));
    control_timer_ = create_wall_timer(
        control_period, [this]() { OnControlTimer(); });
    motion_control_graph_initialized_ = true;
  }

  GuardConfig DeclareGuardConfig() {
    GuardConfig config;
    config.allow_motion = declare_parameter<bool>("allow_motion", false);
    config.state_timeout_sec =
        declare_parameter<double>("state_timeout_sec",
                                  kCommissioningStateTimeoutSec);
    state_timeout_sec_ = config.state_timeout_sec;
    config.max_source_stamp_age_sec =
        declare_parameter<double>("max_source_stamp_age_sec",
                                  kMaximumSourceStampAgeSec);
    config.max_source_stamp_future_skew_sec = declare_parameter<double>(
        "max_source_stamp_future_skew_sec",
        kMaximumSourceStampFutureSkewSec);
    max_source_stamp_age_sec_ = config.max_source_stamp_age_sec;
    max_source_stamp_future_skew_sec_ =
        config.max_source_stamp_future_skew_sec;
    config.command_timeout_sec =
        declare_parameter<double>("command_timeout_sec", 0.25);
    command_timeout_sec_ = config.command_timeout_sec;
    config.zero_preparation_sec =
        declare_parameter<double>("zero_preparation_sec", 0.50);
    config.max_vx = declare_parameter<double>("max_vx", 0.30);
    config.max_vy = declare_parameter<double>("max_vy", 0.0);
    config.max_wz = declare_parameter<double>("max_wz", 0.40);
    config.max_linear_acceleration =
        declare_parameter<double>("max_linear_acceleration", 0.30);
    config.max_angular_acceleration =
        declare_parameter<double>("max_angular_acceleration", 0.80);
    const auto allowed = declare_parameter<std::vector<std::int64_t>>(
        "allowed_modes", {255});
    config.allowed_modes.clear();
    for (const std::int64_t value : allowed) {
      if (value >= 0 && value <= 255) {
        config.allowed_modes.insert(static_cast<std::uint8_t>(value));
      }
    }
    if (config.allowed_modes.empty()) {
      throw std::runtime_error("allowed_modes must contain at least one uint8 value");
    }
    const auto allowed_state_markers =
        declare_parameter<std::vector<std::int64_t>>(
            "allowed_state_markers", std::vector<std::int64_t>{});
    for (const std::int64_t value : allowed_state_markers) {
      if (value <= 0 || value > 4'294'967'295LL) {
        throw std::runtime_error(
            "allowed_state_markers must contain only non-zero uint32 values");
      }
      config.allowed_state_markers.insert(static_cast<std::uint32_t>(value));
    }
    if (!Finite(config.state_timeout_sec) ||
        !Finite(config.max_source_stamp_age_sec) ||
        !Finite(config.max_source_stamp_future_skew_sec) ||
        !Finite(config.command_timeout_sec) ||
        !Finite(config.zero_preparation_sec) || !Finite(config.max_vx) ||
        !Finite(config.max_vy) || !Finite(config.max_wz) ||
        !Finite(config.max_linear_acceleration) ||
        !Finite(config.max_angular_acceleration) ||
        config.state_timeout_sec <= 0.0 ||
        config.state_timeout_sec > kMaximumStateTimeoutSec ||
        config.max_source_stamp_age_sec <= 0.0 ||
        config.max_source_stamp_age_sec > kMaximumSourceStampAgeSec ||
        config.max_source_stamp_future_skew_sec < 0.0 ||
        config.max_source_stamp_future_skew_sec >
            kMaximumSourceStampFutureSkewSec ||
        config.command_timeout_sec <= 0.0 ||
        config.zero_preparation_sec < 0.0 || config.max_vx <= 0.0 ||
        config.max_vy < 0.0 || config.max_wz < 0.0 ||
        config.max_linear_acceleration <= 0.0 ||
        config.max_angular_acceleration <= 0.0) {
      throw std::runtime_error("invalid non-positive chassis safety parameter");
    }
    if (config.allow_motion) {
      const bool first_motion_envelope =
          motion_profile_ == kFirstMotionProfile &&
          config.max_vx == kFirstMotionMaxVx &&
          config.max_vy == kFirstMotionMaxVy &&
          config.max_wz == kFirstMotionMaxWz &&
          config.state_timeout_sec == kCommissioningStateTimeoutSec &&
          config.max_source_stamp_age_sec == kMaximumSourceStampAgeSec &&
          config.command_timeout_sec == kFirstMotionCommandTimeoutSec;
      const bool second_motion_envelope =
          motion_profile_ == kSecondMotionProfile &&
          config.max_vx == kSecondMotionMaxVx &&
          config.max_vy == kSecondMotionMaxVy &&
          config.max_wz == kSecondMotionMaxWz &&
          config.max_linear_acceleration ==
              kSecondMotionMaxLinearAcceleration &&
          config.max_angular_acceleration ==
              kSecondMotionMaxAngularAcceleration &&
          config.state_timeout_sec == kCommissioningStateTimeoutSec &&
          config.max_source_stamp_age_sec == kMaximumSourceStampAgeSec &&
          config.max_source_stamp_future_skew_sec ==
              kMaximumSourceStampFutureSkewSec &&
          config.command_timeout_sec == kSecondMotionCommandTimeoutSec;
      const bool staged_nav2_envelope =
          motion_profile_ == kStagedNav2Profile &&
          config.max_vx == kStagedNav2MaxVx &&
          config.max_vy == kStagedNav2MaxVy &&
          config.max_wz == kStagedNav2MaxWz &&
          config.max_linear_acceleration ==
              kStagedNav2MaxLinearAcceleration &&
          config.max_angular_acceleration ==
              kStagedNav2MaxAngularAcceleration &&
          config.state_timeout_sec == kStagedNav2StateTimeoutSec &&
          config.max_source_stamp_age_sec == kMaximumSourceStampAgeSec &&
          config.max_source_stamp_future_skew_sec ==
              kMaximumSourceStampFutureSkewSec &&
          config.command_timeout_sec == kStagedNav2CommandTimeoutSec;
      if (!first_motion_envelope && !second_motion_envelope &&
          !staged_nav2_envelope) {
        throw std::runtime_error(
            "velocity/watchdog envelope does not match the selected motion "
            "profile");
      }
    }
    return config;
  }

  void OnVelocityCommand(const geometry_msgs::msg::Twist &message) {
    if (motion_profile_ == kSecondMotionProfile &&
        (!Finite(message.linear.x) || !Finite(message.linear.y) ||
         !Finite(message.angular.z) || message.linear.x < 0.0 ||
         message.linear.x > kSecondMotionMaxVx ||
         message.linear.y != kSecondMotionMaxVy ||
         message.angular.z != kSecondMotionMaxWz)) {
      commissioning_motion_active_ = false;
      commissioning_stop_reason_ =
          "second-motion raw command exceeded fixed envelope";
      guard_->ForceFault(commissioning_stop_reason_);
      (void)BestEffortDaemonDisarm();
      return;
    }
    if (motion_profile_ == kStagedNav2Profile &&
        (!Finite(message.linear.x) || !Finite(message.linear.y) ||
         !Finite(message.angular.z) || message.linear.x < 0.0 ||
         message.linear.x > kStagedNav2MaxVx ||
         message.linear.y != kStagedNav2MaxVy ||
         std::fabs(message.angular.z) > kStagedNav2MaxWz)) {
      commissioning_motion_active_ = false;
      commissioning_stop_reason_ =
          "staged-nav2 raw command exceeded fixed envelope";
      guard_->ForceFault(commissioning_stop_reason_);
      (void)BestEffortDaemonDisarm();
      return;
    }
    guard_->UpdateCommand(
        SteadyNowSec(),
        {message.linear.x, message.linear.y, message.angular.z});
  }

  bool CommissioningSessionOpen() const {
    return commissioning_motion_active_ || daemon_armed_ ||
           guard_->state() == GuardState::kPreparing ||
           guard_->state() == GuardState::kArmed;
  }

  void ResetCommissioningMeasurements() {
    commissioning_motion_active_ = false;
    commissioning_motion_start_sec_ = 0.0;
    commissioning_elapsed_sec_ = 0.0;
    commissioning_distance_m_ = 0.0;
    commissioning_last_x_ = previous_x_;
    commissioning_last_y_ = previous_y_;
    commissioning_has_last_position_ = has_previous_position_;
  }

  bool CommissioningEnvelopeExceeded(double now_sec,
                                      std::string *reason) {
    if (!commissioning_motion_active_) {
      return false;
    }
    if (!Finite(now_sec) || now_sec < commissioning_motion_start_sec_) {
      if (reason != nullptr) {
        *reason = "steady clock regressed during commissioning motion";
      }
      return true;
    }
    commissioning_elapsed_sec_ = now_sec - commissioning_motion_start_sec_;
    if (commissioning_max_duration_sec_ > 0.0 &&
        commissioning_elapsed_sec_ >= commissioning_max_duration_sec_) {
      if (reason != nullptr) {
        *reason =
            motion_profile_ == kFirstMotionProfile
                ? "commissioning 2.0 second hard deadline reached"
                : (motion_profile_ == kSecondMotionProfile
                       ? "second-motion 1.5 second hard deadline reached"
                       : "staged-nav2 20.0 second hard deadline reached");
      }
      return true;
    }
    if (!Finite(commissioning_distance_m_) ||
        (commissioning_max_distance_m_ > 0.0 &&
         commissioning_distance_m_ >= commissioning_max_distance_m_)) {
      if (reason != nullptr) {
        *reason =
            motion_profile_ == kFirstMotionProfile
                ? "commissioning 0.10 metre hard distance reached"
                : (motion_profile_ == kSecondMotionProfile
                       ? "second-motion 0.30 metre hard distance reached"
                       : "staged-nav2 0.50 metre hard distance reached");
      }
      return true;
    }
    return false;
  }

  void FailClosedCommissioningStop(const std::string &reason) {
    commissioning_stop_reason_ = reason;
    commissioning_motion_active_ = false;
    guard_->ForceFault(reason);
    const bool stopped = BestEffortDaemonDisarm();
    if (!stopped) {
      RCLCPP_ERROR(
          get_logger(),
          "Commissioning stop was not acknowledged; daemon watchdog remains "
          "the independent stop fallback: %s",
          reason.c_str());
    }
  }

  void UpdateCommissioningDistance(double x, double y, double now_sec) {
    if (!commissioning_motion_active_) {
      return;
    }
    if (!Finite(x) || !Finite(y)) {
      FailClosedCommissioningStop(
          "non-finite commissioning position measurement");
      return;
    }
    if (!commissioning_has_last_position_) {
      commissioning_last_x_ = x;
      commissioning_last_y_ = y;
      commissioning_has_last_position_ = true;
    } else {
      commissioning_distance_m_ +=
          std::hypot(x - commissioning_last_x_, y - commissioning_last_y_);
      commissioning_last_x_ = x;
      commissioning_last_y_ = y;
    }
    std::string reason;
    if (CommissioningEnvelopeExceeded(now_sec, &reason)) {
      FailClosedCommissioningStop(reason);
    }
  }

  void BeginCommissioningMotion(double now_sec) {
    if (commissioning_motion_active_) {
      return;
    }
    if (!has_previous_position_ || !last_state_valid_ || !Finite(previous_x_) ||
        !Finite(previous_y_)) {
      FailClosedCommissioningStop(
          "cannot start commissioning motion without a valid pose");
      return;
    }
    commissioning_motion_active_ = true;
    commissioning_motion_start_sec_ = now_sec;
    commissioning_elapsed_sec_ = 0.0;
    commissioning_distance_m_ = 0.0;
    commissioning_last_x_ = previous_x_;
    commissioning_last_y_ = previous_y_;
    commissioning_has_last_position_ = true;
    commissioning_stop_reason_ = "motion_in_progress";
  }

  void OnSportModeState(const unitree_go::msg::SportModeState &message,
                        bool is_primary) {
    const double receipt_sec = SteadyNowSec();
    const rclcpp::Time reference_stamp = now();
    if (!is_primary && last_primary_state_receipt_sec_ > 0.0 &&
               receipt_sec - last_primary_state_receipt_sec_ <=
                   state_timeout_sec_) {
      return;
    }

    const auto &source_quaternion = message.imu_state.quaternion;
    const double qw = source_quaternion[0];
    const double qx = source_quaternion[1];
    const double qy = source_quaternion[2];
    const double qz = source_quaternion[3];
    const double quaternion_norm =
        std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    bool valid = Finite(quaternion_norm) && quaternion_norm > 0.5 &&
                 quaternion_norm < 1.5;
    for (const float value : message.position) {
      valid = valid && Finite(value);
    }
    for (const float value : message.velocity) {
      valid = valid && Finite(value);
    }
    for (const float value : message.imu_state.gyroscope) {
      valid = valid && Finite(value);
    }
    for (const float value : message.imu_state.accelerometer) {
      valid = valid && Finite(value);
    }
    const bool stamp_encoding_valid =
        message.stamp.sec >= 0 && message.stamp.nanosec < 1'000'000'000U;
    valid = valid && Finite(message.yaw_speed) && stamp_encoding_valid;

    std::uint64_t source_stamp = 0U;
    SourceStampFreshness stamp_freshness =
        SourceStampFreshness::kMalformed;
    if (stamp_encoding_valid) {
      source_stamp = static_cast<std::uint64_t>(message.stamp.sec) *
                         1'000'000'000ULL +
                     message.stamp.nanosec;
      const auto max_age_ns = static_cast<std::uint64_t>(
          max_source_stamp_age_sec_ * 1'000'000'000.0);
      const auto max_future_skew_ns = static_cast<std::uint64_t>(
          max_source_stamp_future_skew_sec_ * 1'000'000'000.0);
      stamp_freshness = ValidateSourceStamp(
          source_stamp, reference_stamp.nanoseconds(), max_age_ns,
          max_future_skew_ns);
    }
    SourceStampTracker &stamp_tracker =
        is_primary ? primary_stamp_tracker_ : fallback_stamp_tracker_;
    if (stamp_freshness == SourceStampFreshness::kTooOld && !valid) {
      source_stamp_diagnostics_.Observe("invalid_measurements");
      last_state_valid_ = false;
      if (odom_source_ == OdomSource::kExternalVerified) {
        RejectExternalOdometry(
            "SportModeState measurements invalid while stale", true);
      }
      if (allow_motion_ && CommissioningSessionOpen()) {
        FailClosedCommissioningStop(
            "SportModeState measurements invalid while stale");
      }
      return;
    }
    if (stamp_freshness != SourceStampFreshness::kFresh) {
      // Source clock validity is a common gate for canonical state and motion
      // liveness. Rejected samples do not update receipt times, SafetyGuard,
      // position history, or odom/TF/IMU publishers.
      if (stamp_freshness == SourceStampFreshness::kTooOld) {
        source_stamp_diagnostics_.Observe("too_old_ignored");
        // A delayed best-effort DDS sample is not a new health failure. Keep
        // the last independently accepted state until its receipt timeout;
        // never let this delayed sample refresh liveness or canonical output.
        if (!FreshValidSportModeState(receipt_sec) &&
            odom_source_ == OdomSource::kExternalVerified) {
          HandleExternalOdometryLivenessLoss(
              "SportModeState fresh-sample timeout");
        }
        return;
      }
      source_stamp_diagnostics_.Observe(SourceStampFreshnessName(stamp_freshness));
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "SportModeState rejected: source_stamp_status=%s source_ns=%llu "
          "reference_ns=%lld max_age_sec=%.3f max_future_sec=%.3f",
          SourceStampFreshnessName(stamp_freshness),
          static_cast<unsigned long long>(source_stamp),
          static_cast<long long>(reference_stamp.nanoseconds()),
          max_source_stamp_age_sec_, max_source_stamp_future_skew_sec_);
      last_state_valid_ = false;
      if (odom_source_ == OdomSource::kExternalVerified) {
        if (stamp_freshness == SourceStampFreshness::kTooOld) {
          HandleExternalOdometryLivenessLoss(
              "SportModeState source timestamp is too old");
        } else {
          RejectExternalOdometry(
              "SportModeState source timestamp rejected", true);
        }
      }
      if (allow_motion_ && CommissioningSessionOpen()) {
        FailClosedCommissioningStop(
            "SportModeState source timestamp rejected");
      }
      return;
    }

    if (!stamp_tracker.Accept(source_stamp, allow_motion_)) {
      // A repeated, zero, or regressing source stamp must not refresh the
      // motion watchdog.  If the adapter is preparing/armed, SafetyGuard will
      // fault once the last genuinely advancing state exceeds its timeout.
      if (source_stamp == stamp_tracker.last_source_stamp_ns()) {
        source_stamp_diagnostics_.Observe("duplicate_ignored");
        if (!FreshValidSportModeState(receipt_sec) &&
            odom_source_ == OdomSource::kExternalVerified) {
          HandleExternalOdometryLivenessLoss(
              "SportModeState fresh-sample timeout");
        }
        return;
      }
      source_stamp_diagnostics_.Observe("non_monotonic");
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "SportModeState rejected: non-monotonic source stamp "
          "source_ns=%llu previous_ns=%llu strict_progress=%s",
          static_cast<unsigned long long>(source_stamp),
          static_cast<unsigned long long>(stamp_tracker.last_source_stamp_ns()),
          allow_motion_ ? "true" : "false");
      last_state_valid_ = false;
      if (odom_source_ == OdomSource::kExternalVerified) {
        RejectExternalOdometry(
            "SportModeState source timestamp stopped progressing", true);
      }
      if (allow_motion_ && CommissioningSessionOpen()) {
        FailClosedCommissioningStop(
            "SportModeState source timestamp stopped progressing");
      }
      return;
    }
    source_stamp_diagnostics_.Observe("fresh");

    const int source = is_primary ? 1 : 2;
    if (source != active_state_source_) {
      // Position histories are independent. Timestamp histories remain with
      // each source so switching cannot make replayed state look new.
      active_state_source_ = source;
      if (odom_source_ == OdomSource::kSportState) {
        has_previous_position_ = false;
      }
    }
    if (is_primary) {
      last_primary_state_receipt_sec_ = receipt_sec;
    }
    last_state_receipt_sec_ = receipt_sec;
    last_mode_ = message.mode;
    last_error_code_ = message.error_code;

    if (valid && odom_source_ == OdomSource::kSportState &&
        has_previous_position_) {
      const double jump = std::hypot(
          static_cast<double>(message.position[0]) - previous_x_,
          static_cast<double>(message.position[1]) - previous_y_);
      const double elapsed = receipt_sec - previous_position_receipt_sec_;
      if (elapsed >= 0.0 && elapsed < 1.0 && jump > max_position_jump_m_) {
        valid = false;
      }
    }
    const bool state_valid =
        valid &&
        state_marker_policy_->Observe(message.error_code, receipt_sec);
    last_state_measurements_valid_ = valid;
    last_state_valid_ = state_valid;
    guard_->UpdateRobotState(receipt_sec, message.mode, state_valid);
    if (state_marker_policy_->change_latched()) {
      // A remote/controller transition must never silently inherit an armed
      // session. The marker has no inferred semantics; only its unexpected
      // change is used to latch the existing stop/fault path.
      guard_->ForceFault(
          "opaque firmware state marker changed; explicit disarm "
          "acknowledgement and re-arm required");
      if (odom_source_ == OdomSource::kSportState) {
        has_previous_position_ = false;
      }
    }
    if (!state_valid) {
      // An invalid, unconfigured, or changed marker must not let an otherwise
      // finite pose feed canonical odom, odom->base_link TF, or IMU consumers.
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "SportModeState rejected: measurements_valid=%s marker=%u "
          "marker_allowed=%s marker_change_latched=%s "
          "passive_marker_recovery_pending=%s",
          valid ? "true" : "false", message.error_code,
          state_marker_policy_->current_marker_allowed() ? "true" : "false",
          state_marker_policy_->change_latched() ? "true" : "false",
          state_marker_policy_->passive_recovery_pending() ? "true" : "false");
      if (odom_source_ == OdomSource::kExternalVerified) {
        const bool recoverable_marker_pause =
            PassiveStateMarkerPauseIsRecoverable(
                allow_motion_,
                odom_source_ == OdomSource::kExternalVerified,
                allow_passive_state_marker_transitions_, valid,
                state_marker_policy_->passive_recovery_pending(),
                state_marker_policy_->change_latched());
        if (recoverable_marker_pause) {
          HandleExternalOdometryLivenessLoss(
              "SportModeState marker quarantined; waiting for an explicitly "
              "allowed marker to remain stable");
        } else {
          RejectExternalOdometry(
              "SportModeState became invalid", true);
        }
      }
      if (allow_motion_ && CommissioningSessionOpen()) {
        FailClosedCommissioningStop(
            "SportModeState became invalid during commissioning");
      }
      return;
    }

    const double normalized_qw = qw / quaternion_norm;
    const double normalized_qx = qx / quaternion_norm;
    const double normalized_qy = qy / quaternion_norm;
    const double normalized_qz = qz / quaternion_norm;
    const double yaw = std::atan2(
        2.0 * (normalized_qw * normalized_qz +
               normalized_qx * normalized_qy),
        1.0 - 2.0 * (normalized_qy * normalized_qy +
                     normalized_qz * normalized_qz));
    if (odom_source_ == OdomSource::kSportState) {
      previous_x_ = message.position[0];
      previous_y_ = message.position[1];
      previous_position_receipt_sec_ = receipt_sec;
      has_previous_position_ = true;
      UpdateCommissioningDistance(previous_x_, previous_y_, receipt_sec);

      const double half_yaw = yaw * 0.5;
      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp.sec = message.stamp.sec;
      odometry.header.stamp.nanosec = message.stamp.nanosec;
      odometry.header.frame_id = odom_frame_;
      odometry.child_frame_id = base_frame_;
      odometry.pose.pose.position.x = message.position[0];
      odometry.pose.pose.position.y = message.position[1];
      odometry.pose.pose.position.z = 0.0;
      odometry.pose.pose.orientation.z = std::sin(half_yaw);
      odometry.pose.pose.orientation.w = std::cos(half_yaw);
      if (velocity_frame_ == "odom") {
        odometry.twist.twist.linear.x =
            std::cos(yaw) * message.velocity[0] +
            std::sin(yaw) * message.velocity[1];
        odometry.twist.twist.linear.y =
            -std::sin(yaw) * message.velocity[0] +
            std::cos(yaw) * message.velocity[1];
      } else {
        odometry.twist.twist.linear.x = message.velocity[0];
        odometry.twist.twist.linear.y = message.velocity[1];
      }
      odometry.twist.twist.angular.z = message.yaw_speed;
      SetOdometryCovariance(odometry);
      PublishCanonicalOdometry(odometry);
    }

    sensor_msgs::msg::Imu imu;
    imu.header.stamp.sec = message.stamp.sec;
    imu.header.stamp.nanosec = message.stamp.nanosec;
    imu.header.frame_id = imu_frame_;
    imu.orientation.x = normalized_qx;
    imu.orientation.y = normalized_qy;
    imu.orientation.z = normalized_qz;
    imu.orientation.w = normalized_qw;
    imu.angular_velocity.x = message.imu_state.gyroscope[0];
    imu.angular_velocity.y = message.imu_state.gyroscope[1];
    imu.angular_velocity.z = message.imu_state.gyroscope[2];
    imu.linear_acceleration.x = message.imu_state.accelerometer[0];
    imu.linear_acceleration.y = message.imu_state.accelerometer[1];
    imu.linear_acceleration.z = message.imu_state.accelerometer[2];
    SetImuCovariance(imu);
    imu_publisher_->publish(imu);
  }

  bool FreshValidSportModeState(double now_sec) const {
    return FreshValidReceipt(last_state_valid_, last_state_receipt_sec_,
                             now_sec, state_timeout_sec_);
  }

  void RejectExternalOdometry(const std::string &reason,
                              bool latch_process = true) {
    last_external_odom_valid_ = false;
    if (latch_process) {
      external_odom_fault_latch_.Latch();
    }
    external_odom_status_ = reason;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "External odometry rejected: %s", reason.c_str());
    if (allow_motion_ && CommissioningSessionOpen()) {
      FailClosedCommissioningStop("external odometry rejected: " + reason);
    }
  }

  void HandleExternalOdometryLivenessLoss(const std::string &reason) {
    if (ExternalOdometryLivenessLossRequiresProcessRestart(
            allow_motion_, CommissioningSessionOpen())) {
      RejectExternalOdometry(reason + "; process restart required", true);
      return;
    }
    RejectExternalOdometry(
        reason + "; waiting for fresh state and external odometry", false);
    // Start a new disarmed continuity epoch only after the next message passes
    // the unchanged frame, source-stamp, finite-value and quaternion checks.
    // Do not reset external_odom_stamp_tracker_: replay/regression remains a
    // permanent integrity fault even in the motion-disabled profile.
    last_external_odom_receipt_sec_ = 0.0;
    previous_position_receipt_sec_ = 0.0;
    has_previous_position_ = false;
  }

  bool ExternalOdometryFresh(double now_sec) const {
    return odom_source_ != OdomSource::kExternalVerified ||
           external_odom_fault_latch_.CanonicalOutputEligible(
               last_state_valid_, last_state_receipt_sec_,
               last_external_odom_valid_, last_external_odom_receipt_sec_,
               now_sec, state_timeout_sec_, external_odom_timeout_sec_);
  }

  void CheckExternalOdometryInterlock(double now_sec) {
    if (odom_source_ != OdomSource::kExternalVerified ||
        external_odom_fault_latch_.latched() ||
        last_external_odom_receipt_sec_ <= 0.0) {
      return;
    }
    if (ReceiptClockInvalidOrRegressed(
            now_sec, last_state_receipt_sec_,
            last_external_odom_receipt_sec_)) {
      RejectExternalOdometry(
          "steady receipt clock is invalid or regressed", true);
      return;
    }
    if (!FreshValidSportModeState(now_sec)) {
      HandleExternalOdometryLivenessLoss(
          "SportModeState invalid or stale");
      return;
    }
    if (!FreshValidReceipt(last_external_odom_valid_,
                           last_external_odom_receipt_sec_, now_sec,
                           external_odom_timeout_sec_)) {
      HandleExternalOdometryLivenessLoss("external odometry timeout");
    }
  }

  bool PublishCanonicalOdometry(const nav_msgs::msg::Odometry &odometry) {
    const double now_sec = SteadyNowSec();
    if (odom_source_ == OdomSource::kExternalVerified &&
        ReceiptClockInvalidOrRegressed(
            now_sec, last_state_receipt_sec_,
            last_external_odom_receipt_sec_)) {
      RejectExternalOdometry(
          "steady receipt clock is invalid or regressed", true);
      return false;
    }
    if (odom_source_ == OdomSource::kExternalVerified &&
        !ExternalOdometryFresh(now_sec)) {
      HandleExternalOdometryLivenessLoss(
          "canonical odometry blocked by state/odometry interlock");
      return false;
    }
    odom_publisher_->publish(odometry);
    if (!publish_odom_tf_) {
      return true;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = odometry.child_frame_id;
    transform.transform.translation.x = odometry.pose.pose.position.x;
    transform.transform.translation.y = odometry.pose.pose.position.y;
    transform.transform.translation.z = odometry.pose.pose.position.z;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
    return true;
  }

  void OnExternalOdometry(const nav_msgs::msg::Odometry &message) {
    if (odom_source_ != OdomSource::kExternalVerified) {
      return;
    }
    const double receipt_sec = SteadyNowSec();
    const rclcpp::Time reference_stamp = now();
    if (external_odom_fault_latch_.latched()) {
      return;
    }
    const bool sport_state_fresh = FreshValidSportModeState(receipt_sec);
    if (ReceiptClockInvalidOrRegressed(
            receipt_sec, last_state_receipt_sec_,
            last_external_odom_receipt_sec_)) {
      RejectExternalOdometry(
          "steady receipt clock is invalid or regressed", true);
      return;
    }
    const bool receipt_timed_out =
        last_external_odom_receipt_sec_ > 0.0 &&
        receipt_sec - last_external_odom_receipt_sec_ >
            external_odom_timeout_sec_;
    if (message.header.frame_id != odom_frame_ ||
        message.child_frame_id != base_frame_) {
      RejectExternalOdometry("frame mismatch (requires odom -> base_link)");
      return;
    }
    const bool stamp_encoding_valid =
        message.header.stamp.sec >= 0 &&
        message.header.stamp.nanosec < 1'000'000'000U;
    if (!stamp_encoding_valid) {
      RejectExternalOdometry("malformed source timestamp");
      return;
    }
    const std::uint64_t source_stamp =
        static_cast<std::uint64_t>(message.header.stamp.sec) *
            1'000'000'000ULL +
        message.header.stamp.nanosec;
    const auto max_age_ns = static_cast<std::uint64_t>(
        max_source_stamp_age_sec_ * 1'000'000'000.0);
    const auto max_future_skew_ns = static_cast<std::uint64_t>(
        max_source_stamp_future_skew_sec_ * 1'000'000'000.0);
    const SourceStampFreshness stamp_freshness = ValidateSourceStamp(
        source_stamp, reference_stamp.nanoseconds(), max_age_ns,
        max_future_skew_ns);
    if (stamp_freshness != SourceStampFreshness::kFresh) {
      if (stamp_freshness == SourceStampFreshness::kTooOld) {
        external_odom_stamp_diagnostics_.Observe("too_old_ignored");
        if (!FreshValidReceipt(
                last_external_odom_valid_, last_external_odom_receipt_sec_,
                receipt_sec, external_odom_timeout_sec_)) {
          HandleExternalOdometryLivenessLoss(
              "external odometry fresh-sample timeout");
        }
        return;
      }
      external_odom_stamp_diagnostics_.Observe(
          SourceStampFreshnessName(stamp_freshness));
      RejectExternalOdometry(
          std::string("source timestamp ") +
          SourceStampFreshnessName(stamp_freshness));
      return;
    }

    const auto &position = message.pose.pose.position;
    const auto &orientation = message.pose.pose.orientation;
    const auto &linear = message.twist.twist.linear;
    const auto &angular = message.twist.twist.angular;
    bool valid = Finite(position.x) && Finite(position.y) &&
                 Finite(position.z) && Finite(orientation.x) &&
                 Finite(orientation.y) && Finite(orientation.z) &&
                 Finite(orientation.w) && Finite(linear.x) &&
                 Finite(linear.y) && Finite(linear.z) && Finite(angular.x) &&
                 Finite(angular.y) && Finite(angular.z);
    for (const double value : message.pose.covariance) {
      valid = valid && Finite(value);
    }
    for (const double value : message.twist.covariance) {
      valid = valid && Finite(value);
    }
    const double quaternion_norm = std::sqrt(
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w);
    valid = valid && Finite(quaternion_norm) && quaternion_norm > 0.5 &&
            quaternion_norm < 1.5;
    if (!valid) {
      RejectExternalOdometry("non-finite values or invalid quaternion");
      return;
    }

    const double qx = orientation.x / quaternion_norm;
    const double qy = orientation.y / quaternion_norm;
    const double qz = orientation.z / quaternion_norm;
    const double qw = orientation.w / quaternion_norm;
    const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                  1.0 - 2.0 * (qy * qy + qz * qz));
    const double position_jump =
        std::hypot(position.x - previous_x_, position.y - previous_y_);
    const double yaw_jump =
        std::abs(std::remainder(yaw - previous_external_yaw_, kTwoPi));
    if (!ExternalOdometryPoseContinuityEligible(
            has_previous_position_, position_jump, yaw_jump,
            max_position_jump_m_,
            max_external_odom_yaw_jump_rad_)) {
      RejectExternalOdometry(
          "pose or yaw continuity exceeded verified odom limits", true);
      return;
    }

    // Only after all non-mutating message-integrity checks pass may a pure
    // liveness condition establish a new passive continuity epoch. The
    // current sample is intentionally not consumed; the next fully fresh
    // sample must independently pass every check.
    const bool chain_started = last_state_receipt_sec_ > 0.0 ||
                               last_external_odom_receipt_sec_ > 0.0;
    if (!sport_state_fresh) {
      if (chain_started) {
        HandleExternalOdometryLivenessLoss(
            "fresh valid SportModeState required for external odometry");
      } else {
        RejectExternalOdometry(
            "fresh valid SportModeState required for external odometry",
            false);
      }
      return;
    }
    if (receipt_timed_out) {
      HandleExternalOdometryLivenessLoss("external odometry timeout");
      return;
    }
    if (!external_odom_stamp_tracker_.Accept(source_stamp, true)) {
      if (source_stamp ==
          external_odom_stamp_tracker_.last_source_stamp_ns()) {
        external_odom_stamp_diagnostics_.Observe("duplicate_ignored");
        if (!FreshValidReceipt(
                last_external_odom_valid_, last_external_odom_receipt_sec_,
                receipt_sec, external_odom_timeout_sec_)) {
          HandleExternalOdometryLivenessLoss(
              "external odometry fresh-sample timeout");
        }
        return;
      }
      external_odom_stamp_diagnostics_.Observe("non_monotonic");
      RejectExternalOdometry("source timestamp did not strictly advance");
      return;
    }

    nav_msgs::msg::Odometry canonical = message;
    canonical.header.frame_id = odom_frame_;
    canonical.child_frame_id = base_frame_;
    canonical.pose.pose.orientation.x = qx;
    canonical.pose.pose.orientation.y = qy;
    canonical.pose.pose.orientation.z = qz;
    canonical.pose.pose.orientation.w = qw;
    previous_x_ = position.x;
    previous_y_ = position.y;
    previous_external_yaw_ = yaw;
    previous_position_receipt_sec_ = receipt_sec;
    has_previous_position_ = true;
    last_external_odom_receipt_sec_ = receipt_sec;
    last_external_odom_valid_ = true;
    external_odom_status_ = "fresh";
    external_odom_stamp_diagnostics_.Observe("fresh");
    UpdateCommissioningDistance(previous_x_, previous_y_, receipt_sec);
    PublishCanonicalOdometry(canonical);
  }

  static void SetOdometryCovariance(nav_msgs::msg::Odometry &odometry) {
    odometry.pose.covariance.fill(0.0);
    odometry.twist.covariance.fill(0.0);
    odometry.pose.covariance[0] = 0.04;
    odometry.pose.covariance[7] = 0.04;
    odometry.pose.covariance[14] = 999.0;
    odometry.pose.covariance[21] = 999.0;
    odometry.pose.covariance[28] = 999.0;
    odometry.pose.covariance[35] = 0.09;
    odometry.twist.covariance[0] = 0.09;
    odometry.twist.covariance[7] = 0.09;
    odometry.twist.covariance[14] = 999.0;
    odometry.twist.covariance[21] = 999.0;
    odometry.twist.covariance[28] = 999.0;
    odometry.twist.covariance[35] = 0.16;
  }

  static void SetImuCovariance(sensor_msgs::msg::Imu &imu) {
    imu.orientation_covariance.fill(0.0);
    imu.angular_velocity_covariance.fill(0.0);
    imu.linear_acceleration_covariance.fill(0.0);
    imu.orientation_covariance[0] = 0.02;
    imu.orientation_covariance[4] = 0.02;
    imu.orientation_covariance[8] = 0.04;
    imu.angular_velocity_covariance[0] = 0.01;
    imu.angular_velocity_covariance[4] = 0.01;
    imu.angular_velocity_covariance[8] = 0.01;
    imu.linear_acceleration_covariance[0] = 0.10;
    imu.linear_acceleration_covariance[4] = 0.10;
    imu.linear_acceleration_covariance[8] = 0.10;
  }

  void OnArmRequest(bool arm,
                    std_srvs::srv::SetBool::Response &response) {
    if (!allow_motion_ || !motion_control_graph_initialized_) {
      response.success = false;
      response.message = "motion-control graph is disabled";
      return;
    }
    std::string message;
    if (arm) {
      if (commissioning_arm_spent_ &&
          motion_profile_ != kStagedNav2Profile) {
        response.success = false;
        response.message =
            motion_profile_ == kFirstMotionProfile
                ? "first-motion permit is already spent in this adapter process"
                : "second-motion permit is already spent in this adapter "
                  "process";
        return;
      }
      if (!last_state_valid_ || !has_previous_position_) {
        response.success = false;
        response.message = "fresh valid odometry pose is required before arm";
        return;
      }
      if (!ExternalOdometryFresh(SteadyNowSec())) {
        response.success = false;
        response.message =
            "fresh verified external odometry is required before arm";
        return;
      }
      response.success = guard_->RequestArm(SteadyNowSec(), &message);
      if (response.success) {
        // Keep the historical diagnostic latch, but only the two
        // commissioning profiles interpret it as a one-shot permit.  The
        // standard Nav2 profile can return here after an explicit disarm.
        commissioning_arm_spent_ = true;
        ResetCommissioningMeasurements();
        commissioning_stop_reason_ = "armed_zero";
      }
      response.message = message;
      return;
    }

    const bool marker_change_pending =
        state_marker_policy_->change_latched();
    // Capture the pre-disarm guard state. RequestDisarm intentionally clears a
    // fault after StopMove is acknowledged, but a fault-originated cleanup must
    // never be reclassified as a clean ClassicWalk restoration.
    const bool clean_classic_walk_restore =
        guard_->state() != GuardState::kFault && !marker_change_pending;
    bool marker_can_be_acknowledged = true;
    if (marker_change_pending) {
      if (!last_state_measurements_valid_) {
        marker_can_be_acknowledged = false;
        message = "cannot acknowledge marker change: latest state measurements "
                  "are invalid";
      } else {
        marker_can_be_acknowledged =
            state_marker_policy_->CanAcknowledgeCurrent(&message);
      }
      if (marker_can_be_acknowledged) {
        // Give RequestDisarm the latest measurement/freshness evidence, but do
        // not clear the marker latch until the guard has actually accepted the
        // disarm. A failed health check therefore remains fail-closed.
        guard_->UpdateRobotState(last_state_receipt_sec_, last_mode_,
                                 last_state_measurements_valid_);
      }
    }
    std::string disarm_message;
    const bool disarm_request_accepted = marker_can_be_acknowledged &&
        guard_->RequestDisarm(SteadyNowSec(), &disarm_message);
    bool marker_unlock_completed =
        disarm_request_accepted &&
        (!marker_change_pending ||
         guard_->state() == GuardState::kDisarmed);
    if (marker_unlock_completed && marker_change_pending) {
      std::string marker_message;
      if (!state_marker_policy_->AcknowledgeCurrent(&marker_message)) {
        guard_->ForceFault(
            "firmware state marker acknowledgement changed unexpectedly");
        marker_unlock_completed = false;
        disarm_message = marker_message;
      } else {
        message = marker_message;
      }
    }
    if (marker_can_be_acknowledged) {
      if (!message.empty()) {
        message += "; ";
      }
      message += disarm_message;
    }
    response.success = marker_change_pending ? marker_unlock_completed
                                             : disarm_request_accepted;
    commissioning_motion_active_ = false;
    commissioning_stop_reason_ = "explicit_disarm_or_cancel";
    const bool restore_classic_walk =
        response.success && clean_classic_walk_restore &&
        preserve_classic_walk_;
    bool classic_walk_restored = !restore_classic_walk;
    const bool daemon_stopped = BestEffortDaemonDisarm(
        restore_classic_walk, &classic_walk_restored);
    if (!daemon_stopped) {
      response.success = false;
      message += "; daemon did not acknowledge, independent watchdog must stop it";
    } else if (restore_classic_walk && !classic_walk_restored) {
      if (!message.empty()) {
        message += "; ";
      }
      message +=
          "chassis disarmed; ClassicWalk restore was not confirmed and will "
          "be retried before the next arm";
    }
    response.message = message;
  }

  void OnControlTimer() {
    const double now_sec = SteadyNowSec();
    CheckExternalOdometryInterlock(now_sec);
    if (!allow_motion_ || !motion_control_graph_initialized_) {
      return;
    }
    if (CommissioningSessionOpen() && !ExternalOdometryFresh(now_sec)) {
      FailClosedCommissioningStop("verified external odometry became stale");
      return;
    }
    std::string commissioning_limit_reason;
    if (CommissioningEnvelopeExceeded(now_sec,
                                      &commissioning_limit_reason)) {
      FailClosedCommissioningStop(commissioning_limit_reason);
      return;
    }
    const GuardDecision decision = guard_->Tick(now_sec, control_period_sec_);
    if (decision.state == GuardState::kFault) {
      commissioning_motion_active_ = false;
      commissioning_stop_reason_ = "safety_guard_fault: " + decision.reason;
      (void)BestEffortDaemonDisarm();
      return;
    }
    if (decision.state != GuardState::kArmed) {
      return;
    }

    std::string error;
    std::int32_t reply_code = 0;
    if (!daemon_armed_) {
      if (!ipc_->Connect(&error) ||
          !ipc_->Exchange(CommandOp::kArm, {}, &reply_code, &error)) {
        HandleIpcFault("cannot arm SDK daemon: " + error);
        return;
      }
      daemon_armed_ = true;
      // ARM emits no Move. Preserve the stopped-output invariant so the next
      // revalidated zero tick sends Ping instead of an unnecessary Stop RPC.
      last_output_stopped_ = true;
      // ARM can consume the full bounded 290 ms preflight budget. Never use
      // the state/command decision calculated before that wait to emit a
      // follow-on command.  The next timer tick must revalidate everything.
      return;
    }

    CommandOp operation = CommandOp::kPing;
    Velocity velocity;
    if (decision.action == GuardAction::kMove) {
      const bool first_motion_velocity =
          motion_profile_ == kFirstMotionProfile &&
          decision.velocity.vx >= 0.0 &&
          decision.velocity.vx <= kFirstMotionMaxVx &&
          decision.velocity.vy == kFirstMotionMaxVy &&
          decision.velocity.wz == kFirstMotionMaxWz;
      const bool second_motion_velocity =
          motion_profile_ == kSecondMotionProfile &&
          decision.velocity.vx >= 0.0 &&
          decision.velocity.vx <= kSecondMotionMaxVx &&
          decision.velocity.vy == kSecondMotionMaxVy &&
          decision.velocity.wz == kSecondMotionMaxWz;
      const bool staged_nav2_velocity =
          motion_profile_ == kStagedNav2Profile &&
          decision.velocity.vx >= 0.0 &&
          decision.velocity.vx <= kStagedNav2MaxVx &&
          decision.velocity.vy == kStagedNav2MaxVy &&
          std::fabs(decision.velocity.wz) <= kStagedNav2MaxWz;
      if (!Finite(decision.velocity.vx) || !Finite(decision.velocity.vy) ||
          !Finite(decision.velocity.wz) ||
          (!first_motion_velocity && !second_motion_velocity &&
           !staged_nav2_velocity)) {
        FailClosedCommissioningStop(
            "adapter generated velocity outside selected motion profile");
        return;
      }
      BeginCommissioningMotion(now_sec);
      if (!commissioning_motion_active_) {
        return;
      }
      if (CommissioningEnvelopeExceeded(now_sec,
                                        &commissioning_limit_reason)) {
        FailClosedCommissioningStop(commissioning_limit_reason);
        return;
      }
      operation = CommandOp::kMove;
      velocity = decision.velocity;
      last_output_stopped_ = false;
    } else if (!last_output_stopped_) {
      operation = CommandOp::kStop;
      last_output_stopped_ = true;
    }
    if (!ipc_->Exchange(operation, velocity, &reply_code, &error)) {
      HandleIpcFault("SDK daemon command failed: " + error);
    }
  }

  void HandleIpcFault(const std::string &reason) {
    commissioning_motion_active_ = false;
    commissioning_stop_reason_ = "ipc_disconnect_or_failure: " + reason;
    guard_->ForceFault(reason);
    (void)BestEffortDaemonDisarm();
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
  }

  bool BestEffortDaemonDisarm(bool restore_classic_walk = false,
                              bool *classic_walk_restored = nullptr) {
    if (classic_walk_restored != nullptr) {
      *classic_walk_restored = !restore_classic_walk;
    }
    if (!allow_motion_ || !motion_control_graph_initialized_) {
      daemon_armed_ = false;
      return true;
    }
    if (ipc_ == nullptr) {
      daemon_armed_ = false;
      return false;
    }
    if (!ipc_->connected()) {
      ipc_->Close();
      std::string connect_error;
      if (!ipc_->Connect(&connect_error)) {
        daemon_armed_ = false;
        RCLCPP_ERROR(get_logger(),
                     "Daemon disarm reconnect failed: %s",
                     connect_error.c_str());
        return false;
      }
    }
    std::string error;
    std::int32_t reply_code = 0;
    bool stop_success = true;
    if (daemon_armed_) {
      stop_success =
          ipc_->Exchange(CommandOp::kStop, {}, &reply_code, &error);
    }
    std::string disarm_error;
    const bool disarm_success =
        ipc_->Exchange(CommandOp::kDisarm, {}, &reply_code, &disarm_error);
    std::string classic_walk_error;
    bool classic_walk_success = true;
    if (stop_success && disarm_success && restore_classic_walk) {
      // Keep this as a separate IPC operation. StopMove and ClassicWalk are
      // both response-bearing SDK calls and cannot share the 190 ms deadline.
      classic_walk_success = ipc_->Exchange(
          CommandOp::kRestoreClassicWalk, {}, &reply_code,
          &classic_walk_error);
    }
    if (classic_walk_restored != nullptr) {
      *classic_walk_restored = classic_walk_success;
    }
    const bool success = stop_success && disarm_success;
    daemon_armed_ = false;
    last_output_stopped_ = true;
    ipc_->Close();
    if (!success) {
      RCLCPP_ERROR(get_logger(),
                   "Daemon stop/disarm acknowledgement failed; SDK watchdog "
                   "remains the stop fallback: stop=%s disarm=%s",
                   error.c_str(), disarm_error.c_str());
    } else if (!classic_walk_success) {
      RCLCPP_WARN(
          get_logger(),
          "Stop/disarm acknowledged, but ClassicWalk restoration was not "
          "acknowledged: %s",
          classic_walk_error.c_str());
    }
    return success;
  }

  void PublishDiagnostics() {
    const double now_sec = SteadyNowSec();
    // This timer exists in both passive/no-motion and motion profiles, so a
    // silent SportModeState or odometry outage always suppresses canonical
    // output. Motion profiles latch; passive profiles wait for independently
    // validated fresh state and odometry before starting a new epoch.
    CheckExternalOdometryInterlock(now_sec);
    const double state_age = last_state_receipt_sec_ > 0.0
                                 ? now_sec - last_state_receipt_sec_
                                 : -1.0;
    const double external_odom_age = last_external_odom_receipt_sec_ > 0.0
                                         ? now_sec - last_external_odom_receipt_sec_
                                         : -1.0;
    const bool external_odom_fault =
        odom_source_ == OdomSource::kExternalVerified &&
        !ExternalOdometryFresh(now_sec);
    const GuardState state = guard_->state();
    const bool source_stamp_fault = source_stamp_diagnostics_.active_fault();
    const bool state_marker_fault =
        state_marker_policy_->has_current_marker() &&
        (!state_marker_policy_->current_marker_allowed() ||
         state_marker_policy_->change_latched() ||
         state_marker_policy_->passive_recovery_pending());
    const bool opaque_marker_override =
        state_marker_policy_->has_current_marker() &&
        last_error_code_ != 0U && !state_marker_fault;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "go2_chassis_adapter";
    status.hardware_id = "unitree_go2";
    status.message = guard_->reason();
    if (source_stamp_diagnostics_.active_fault()) {
      status.message = "source timestamp rejected: " +
                       source_stamp_diagnostics_.latest_status();
    } else if (source_stamp_diagnostics_.fault_since_last_report()) {
      status.message =
          "source timestamp rejection observed since previous diagnostic: " +
          source_stamp_diagnostics_.last_fault_status();
    } else if (external_odom_fault) {
      status.message = "verified external odometry is invalid or stale: " +
                       external_odom_status_;
    } else if (state_marker_policy_->change_latched()) {
      status.message =
          "opaque firmware state marker changed; explicit disarm required";
    } else if (state_marker_policy_->passive_recovery_pending()) {
      status.message =
          "canonical odometry paused while an explicitly allowed firmware "
          "state marker stabilizes";
    } else if (state_marker_fault) {
      status.message =
          "non-zero firmware state marker is not explicitly allowed";
    } else if (opaque_marker_override) {
      status.message =
          "using explicitly allowed opaque firmware state marker";
    }
    if (state == GuardState::kFault || source_stamp_fault ||
        external_odom_fault ||
        state_marker_fault) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (state == GuardState::kDisabled ||
               state == GuardState::kDisarmed || !last_state_valid_ ||
               opaque_marker_override) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }
    status.values.push_back(
        DiagnosticValue("guard_state", GuardStateName(state)));
    status.values.push_back(DiagnosticValue(
        "allow_motion", allow_motion_ ? "true" : "false"));
    status.values.push_back(
        DiagnosticValue("motion_profile", motion_profile_));
    status.values.push_back(
        DiagnosticValue("odom_source", OdomSourceName(odom_source_)));
    status.values.push_back(DiagnosticValue(
        "external_odom_topic",
        odom_source_ == OdomSource::kExternalVerified ? external_odom_topic_
                                                       : "inactive"));
    status.values.push_back(DiagnosticValue(
        "external_odom_valid",
        ExternalOdometryFresh(now_sec) ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "external_odom_age_sec", std::to_string(external_odom_age)));
    status.values.push_back(DiagnosticValue(
        "external_odom_timeout_sec",
        std::to_string(external_odom_timeout_sec_)));
    status.values.push_back(
        DiagnosticValue("external_odom_status", external_odom_status_));
    status.values.push_back(DiagnosticValue(
        "external_odom_fault_latched",
        external_odom_fault_latch_.latched() ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "daemon_armed", daemon_armed_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "preserve_classic_walk",
        preserve_classic_walk_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "commissioning_permit_spent",
        commissioning_arm_spent_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "commissioning_motion_active",
        commissioning_motion_active_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "commissioning_stop_reason", commissioning_stop_reason_));
    status.values.push_back(DiagnosticValue(
        "commissioning_elapsed_sec",
        std::to_string(commissioning_elapsed_sec_)));
    status.values.push_back(DiagnosticValue(
        "commissioning_distance_m",
        std::to_string(commissioning_distance_m_)));
    status.values.push_back(DiagnosticValue(
        "commissioning_max_duration_sec",
        std::to_string(commissioning_max_duration_sec_)));
    status.values.push_back(DiagnosticValue(
        "commissioning_max_distance_m",
        std::to_string(commissioning_max_distance_m_)));
    status.values.push_back(DiagnosticValue(
        "commissioning_command_timeout_sec",
        std::to_string(command_timeout_sec_)));
    status.values.push_back(
        DiagnosticValue("sport_mode", std::to_string(last_mode_)));
    status.values.push_back(DiagnosticValue(
        "sport_error_code", std::to_string(last_error_code_)));
    status.values.push_back(DiagnosticValue(
        "opaque_state_marker_explicitly_allowed",
        state_marker_policy_->current_marker_allowed() ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "opaque_state_marker_change_latched",
        state_marker_policy_->change_latched() ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "passive_state_marker_transitions_enabled",
        allow_passive_state_marker_transitions_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "motion_state_marker_transitions_enabled",
        allow_motion_state_marker_transitions_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "passive_state_marker_recovery_pending",
        state_marker_policy_->passive_recovery_pending() ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "passive_state_marker_recovery_candidate",
        state_marker_policy_->has_passive_recovery_candidate()
            ? std::to_string(
                  state_marker_policy_->passive_recovery_candidate())
            : "none"));
    status.values.push_back(DiagnosticValue(
        "passive_state_marker_recovery_samples",
        std::to_string(state_marker_policy_->passive_recovery_samples())));
    status.values.push_back(DiagnosticValue(
        "passive_state_marker_recovery_elapsed_sec",
        std::to_string(
            state_marker_policy_->passive_recovery_elapsed_sec())));
    status.values.push_back(DiagnosticValue(
        "opaque_state_marker_bound",
        state_marker_policy_->has_bound_marker()
            ? std::to_string(state_marker_policy_->bound_marker())
            : "none"));
    status.values.push_back(
        DiagnosticValue("state_age_sec", std::to_string(state_age)));
    status.values.push_back(DiagnosticValue(
        "state_timeout_sec", std::to_string(state_timeout_sec_)));
    status.values.push_back(DiagnosticValue(
        "state_valid", last_state_valid_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "source_stamp_status", source_stamp_diagnostics_.latest_status()));
    status.values.push_back(DiagnosticValue(
        "source_stamp_last_fault",
        source_stamp_diagnostics_.last_fault_status()));
    status.values.push_back(DiagnosticValue(
        "source_stamp_rejection_count",
        std::to_string(source_stamp_diagnostics_.total_faults())));
    status.values.push_back(DiagnosticValue(
        "source_stamp_rejections_since_previous_diagnostic",
        std::to_string(source_stamp_diagnostics_.faults_since_report())));
    status.values.push_back(DiagnosticValue(
        "max_source_stamp_age_sec",
        std::to_string(max_source_stamp_age_sec_)));
    status.values.push_back(DiagnosticValue(
        "max_source_stamp_future_skew_sec",
        std::to_string(max_source_stamp_future_skew_sec_)));

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostics.status.push_back(status);
    diagnostics_publisher_->publish(diagnostics);
    source_stamp_diagnostics_.MarkReported();
    external_odom_stamp_diagnostics_.MarkReported();

    std_msgs::msg::String state_message;
    std::ostringstream stream;
    stream << GuardStateName(state) << ": " << guard_->reason()
           << "; motion_configured=" << (allow_motion_ ? "true" : "false")
           << "; motion_profile=" << motion_profile_
           << "; odom_source=" << OdomSourceName(odom_source_)
           << "; external_odom_age_sec=" << external_odom_age
           << "; external_odom_status=" << external_odom_status_
           << "; daemon_armed=" << (daemon_armed_ ? "true" : "false")
           << "; preserve_classic_walk="
           << (preserve_classic_walk_ ? "true" : "false")
           << "; commissioning_permit_spent="
           << (commissioning_arm_spent_ ? "true" : "false")
           << "; commissioning_motion_active="
           << (commissioning_motion_active_ ? "true" : "false")
           << "; commissioning_stop_reason=" << commissioning_stop_reason_
           << "; commissioning_elapsed_sec=" << commissioning_elapsed_sec_
           << "; commissioning_distance_m=" << commissioning_distance_m_
           << "; sport_mode=" << static_cast<unsigned int>(last_mode_)
           << "; opaque_state_marker=" << last_error_code_
           << "; marker_change_latched="
           << (state_marker_policy_->change_latched() ? "true" : "false")
           << "; passive_marker_transitions="
           << (allow_passive_state_marker_transitions_ ? "true" : "false")
           << "; motion_marker_transitions="
           << (allow_motion_state_marker_transitions_ ? "true" : "false")
           << "; passive_marker_recovery_pending="
           << (state_marker_policy_->passive_recovery_pending() ? "true"
                                                                : "false")
           << "; state_age_sec=" << state_age
           << "; source_stamp_status="
           << source_stamp_diagnostics_.latest_status();
    state_message.data = stream.str();
    status_publisher_->publish(state_message);
  }

  bool allow_motion_{false};
  bool preserve_classic_walk_{false};
  bool allow_passive_state_marker_transitions_{false};
  bool allow_motion_state_marker_transitions_{false};
  bool motion_control_graph_initialized_{false};
  bool daemon_armed_{false};
  bool last_output_stopped_{true};
  bool commissioning_arm_spent_{false};
  bool commissioning_motion_active_{false};
  bool commissioning_has_last_position_{false};
  bool last_state_valid_{false};
  bool last_state_measurements_valid_{false};
  bool has_previous_position_{false};
  bool last_external_odom_valid_{false};
  std::uint8_t last_mode_{0U};
  std::uint32_t last_error_code_{0U};
  int active_state_source_{0};
  SourceStampTracker primary_stamp_tracker_;
  SourceStampTracker fallback_stamp_tracker_;
  SourceStampTracker external_odom_stamp_tracker_;
  ExternalOdometryFaultLatch external_odom_fault_latch_;
  double last_state_receipt_sec_{0.0};
  double last_primary_state_receipt_sec_{0.0};
  double last_external_odom_receipt_sec_{0.0};
  double state_timeout_sec_{kMaximumStateTimeoutSec};
  double max_source_stamp_age_sec_{kMaximumSourceStampAgeSec};
  double max_source_stamp_future_skew_sec_{
      kMaximumSourceStampFutureSkewSec};
  double previous_x_{0.0};
  double previous_y_{0.0};
  double previous_position_receipt_sec_{0.0};
  double max_position_jump_m_{1.0};
  double previous_external_yaw_{0.0};
  double external_odom_timeout_sec_{kMaximumStateTimeoutSec};
  double max_external_odom_yaw_jump_rad_{1.0};
  double control_period_sec_{0.02};
  double command_timeout_sec_{0.25};
  double commissioning_motion_start_sec_{0.0};
  double commissioning_elapsed_sec_{0.0};
  double commissioning_distance_m_{0.0};
  double commissioning_last_x_{0.0};
  double commissioning_last_y_{0.0};
  double commissioning_max_duration_sec_{kFirstMotionMaxDurationSec};
  double commissioning_max_distance_m_{kFirstMotionMaxDistanceM};
  bool publish_odom_tf_{true};
  OdomSource odom_source_{OdomSource::kSportState};
  std::string sport_state_topic_;
  std::string sport_state_fallback_topic_;
  std::string cmd_vel_topic_;
  std::string motion_profile_;
  std::string odom_topic_;
  std::string external_odom_topic_;
  std::string imu_topic_;
  std::string status_topic_;
  std::string diagnostics_topic_;
  std::string arm_service_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string velocity_frame_;
  std::string commissioning_stop_reason_{"not_started"};
  std::string external_odom_status_{"not_received"};
  SourceStampDiagnosticTracker source_stamp_diagnostics_;
  SourceStampDiagnosticTracker external_odom_stamp_diagnostics_;
  std::unique_ptr<OpaqueFirmwareStateMarkerPolicy> state_marker_policy_;
  std::unique_ptr<PassiveStationaryPoseHold> stationary_pose_hold_;
  std::unique_ptr<SafetyGuard> guard_;
  std::unique_ptr<SeqpacketClient> ipc_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
      sport_state_subscription_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
      sport_state_fallback_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      external_odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace
}  // namespace go2_chassis

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_chassis::Go2ChassisAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
