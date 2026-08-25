#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "go2_motion_state_relay/freshness_contract.hpp"
#include "go2_motion_state_relay/gid_guard.hpp"
#include "go2_motion_state_relay/qos_contract.hpp"

namespace {

using namespace std::chrono_literals;
using go2_motion_state_relay::GidToHex;
using go2_motion_state_relay::GidDomain;
using go2_motion_state_relay::GidDomainToWire;
using go2_motion_state_relay::GraphObservation;
using go2_motion_state_relay::HeartbeatTimestampGuard;
using go2_motion_state_relay::IsSafeCorrectedStatePublisherQos;
using go2_motion_state_relay::PublisherGid;
using go2_motion_state_relay::PublisherGidGuard;

static_assert(RMW_GID_STORAGE_SIZE ==
                  go2_motion_state_relay::kPublisherGidBytes,
              "the audited endpoint identity contract requires a 24-byte GID");

constexpr char kInputTopic[] = "/robonix/time_corrected/raw/sportmodestate";
constexpr char kOutputTopic[] =
    "/robonix/time_corrected/motion/sportmodestate";
constexpr char kExpectedNodeName[] =
    "go2_workstation_nomotion_stamp_discipline";
constexpr char kExpectedNodeNamespace[] = "/";
constexpr char kExpectedTopicType[] = "unitree_go/msg/SportModeState";
constexpr char kNodeName[] = "go2_workstation_motion_state_relay";
constexpr char kRequiredRmwImplementation[] = "rmw_cyclonedds_cpp";
constexpr std::int64_t kHeartbeatTimeoutNs = 200'000'000LL;
constexpr std::size_t kHeartbeatHexCharacters = 16U;
constexpr std::size_t kMaximumHeartbeatBufferBytes = 1024U;
constexpr std::int64_t kInitialMessageTimeoutNs = 5'000'000'000LL;
constexpr std::int64_t kGraphFreshNs = 100'000'000LL;
constexpr std::size_t kMinimumReadySamples = 30U;

std::int64_t SteadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::int64_t SystemNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

PublisherGid CopyGid(const std::uint8_t *source) {
  PublisherGid result{};
  std::copy_n(source, result.size(), result.begin());
  return result;
}

std::string OptionalGidToWire(const std::optional<PublisherGid> &gid) {
  return gid.has_value() ? GidToHex(gid.value()) : "-";
}

std::string CurrentRmwIdentifier() {
  const char *const identifier = rmw_get_implementation_identifier();
  if (identifier == nullptr || *identifier == '\0') {
    throw std::runtime_error("current RMW implementation identifier is empty");
  }
  const std::string value(identifier);
  if (value.size() > 80U ||
      !std::all_of(value.begin(), value.end(), [](const char character) {
        return (character >= 'a' && character <= 'z') ||
               (character >= 'A' && character <= 'Z') ||
               (character >= '0' && character <= '9') || character == '_' ||
               character == '-' || character == '.';
      })) {
    throw std::runtime_error(
        "current RMW implementation identifier is not wire-safe");
  }
  if (value != kRequiredRmwImplementation) {
    throw std::runtime_error(
        "motion-state relay requires rmw_cyclonedds_cpp");
  }
  return value;
}

bool WriteWholeEvent(const int event_fd, const std::string &line) {
  const ssize_t written = ::write(event_fd, line.data(), line.size());
  return written == static_cast<ssize_t>(line.size());
}

int ParseFd(const std::string &value, const char *name) {
  if (value.empty()) {
    throw std::invalid_argument(std::string(name) + " is empty");
  }
  char *end = nullptr;
  errno = 0;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (errno != 0 || end == value.c_str() || *end != '\0' || parsed <= 2L ||
      parsed > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(std::string(name) + " is not a valid pipe fd");
  }
  const int fd = static_cast<int>(parsed);
  if (::fcntl(fd, F_GETFD) == -1) {
    throw std::invalid_argument(std::string(name) + " is not open");
  }
  const int flags = ::fcntl(fd, F_GETFL);
  if (flags == -1 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
    throw std::invalid_argument(std::string(name) + " cannot be nonblocking");
  }
  return fd;
}

struct PipeFds {
  int heartbeat{-1};
  int events{-1};
};

PipeFds ParseArguments(const int argc, char **argv) {
  std::optional<int> heartbeat;
  std::optional<int> events;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if ((argument == "--heartbeat-fd" || argument == "--event-fd") &&
        index + 1 < argc) {
      const int parsed = ParseFd(argv[++index], argument.c_str());
      if (argument == "--heartbeat-fd") {
        if (heartbeat.has_value()) {
          throw std::invalid_argument("duplicate --heartbeat-fd");
        }
        heartbeat = parsed;
      } else {
        if (events.has_value()) {
          throw std::invalid_argument("duplicate --event-fd");
        }
        events = parsed;
      }
    } else {
      throw std::invalid_argument("unexpected or incomplete worker argument");
    }
  }
  if (!heartbeat.has_value() || !events.has_value() ||
      heartbeat.value() == events.value()) {
    throw std::invalid_argument(
        "distinct --heartbeat-fd and --event-fd are required");
  }
  return PipeFds{heartbeat.value(), events.value()};
}

std::optional<std::int64_t> CorrectedStampNs(
    const unitree_go::msg::SportModeState &message) {
  constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
  if (message.stamp.sec <= 0 ||
      message.stamp.nanosec >=
          static_cast<std::uint32_t>(kNanosecondsPerSecond)) {
    return std::nullopt;
  }
  const auto seconds = static_cast<std::int64_t>(message.stamp.sec);
  if (seconds >
      (std::numeric_limits<std::int64_t>::max() -
       static_cast<std::int64_t>(message.stamp.nanosec)) /
          kNanosecondsPerSecond) {
    return std::nullopt;
  }
  return seconds * kNanosecondsPerSecond +
         static_cast<std::int64_t>(message.stamp.nanosec);
}

class MotionStateRelay final : public rclcpp::Node {
 public:
  explicit MotionStateRelay(const PipeFds pipes)
      : Node(kNodeName),
        heartbeat_fd_(pipes.heartbeat),
        event_fd_(pipes.events),
        started_steady_ns_(SteadyNowNs()),
        rmw_implementation_identifier_(CurrentRmwIdentifier()) {
    const auto output_qos = rclcpp::SensorDataQoS();
    const auto input_qos = go2_motion_state_relay::CorrectedStateInputQos();
    publisher_ = create_publisher<unitree_go::msg::SportModeState>(
        kOutputTopic, output_qos);
    subscription_ = create_subscription<unitree_go::msg::SportModeState>(
        kInputTopic, input_qos,
        [this](unitree_go::msg::SportModeState::ConstSharedPtr message,
               const rclcpp::MessageInfo &message_info) {
          ObserveMessage(std::move(message), message_info);
        });
    graph_timer_ = create_wall_timer(50ms, [this]() { PollGraphAndSupervisor(); });
  }

  bool faulted() const noexcept { return faulted_; }

 private:
  void Shutdown() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  bool HeartbeatFresh(const std::int64_t now_ns) const noexcept {
    return heartbeat_guard_.Fresh(now_ns, kHeartbeatTimeoutNs);
  }

  void DrainHeartbeat() {
    char buffer[256];
    while (!faulted_) {
      const ssize_t received = ::read(heartbeat_fd_, buffer, sizeof(buffer));
      if (received > 0) {
        heartbeat_buffer_.append(buffer, static_cast<std::size_t>(received));
        if (heartbeat_buffer_.size() > kMaximumHeartbeatBufferBytes) {
          LatchFault("supervisor_heartbeat_buffer_overflow");
          return;
        }
        ParseHeartbeatRecords();
        continue;
      }
      if (received == 0) {
        LatchFault("supervisor_heartbeat_eof");
        return;
      }
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      LatchFault("supervisor_heartbeat_read_error");
      return;
    }
  }

  void ParseHeartbeatRecords() {
    while (!faulted_) {
      const auto newline = heartbeat_buffer_.find('\n');
      if (newline == std::string::npos) {
        return;
      }
      const std::string record = heartbeat_buffer_.substr(0U, newline);
      heartbeat_buffer_.erase(0U, newline + 1U);
      if (record.size() != kHeartbeatHexCharacters ||
          !std::all_of(record.begin(), record.end(), [](const char character) {
            return (character >= '0' && character <= '9') ||
                   (character >= 'a' && character <= 'f');
          })) {
        LatchFault("supervisor_heartbeat_protocol_error");
        return;
      }
      std::uint64_t raw_stamp = 0U;
      const auto parsed =
          std::from_chars(record.data(), record.data() + record.size(),
                          raw_stamp, 16);
      if (parsed.ec != std::errc{} ||
          parsed.ptr != record.data() + record.size() || raw_stamp == 0U ||
          raw_stamp >
              static_cast<std::uint64_t>(
                  std::numeric_limits<std::int64_t>::max())) {
        LatchFault("supervisor_heartbeat_protocol_error");
        return;
      }
      const auto stamp = static_cast<std::int64_t>(raw_stamp);
      if (!heartbeat_guard_.Observe(stamp)) {
        LatchFault(heartbeat_guard_.reason());
        return;
      }
    }
  }

  void PollGraphAndSupervisor() {
    if (faulted_) {
      return;
    }
    DrainHeartbeat();
    if (faulted_) {
      return;
    }
    const std::int64_t now_ns = SteadyNowNs();
    if (!HeartbeatFresh(now_ns)) {
      if (now_ns < started_steady_ns_ ||
          now_ns - started_steady_ns_ > kHeartbeatTimeoutNs) {
        LatchFault("supervisor_heartbeat_stale");
      }
      return;
    }

    std::vector<rclcpp::TopicEndpointInfo> endpoints;
    try {
      endpoints = get_publishers_info_by_topic(kInputTopic);
    } catch (const std::exception &) {
      LatchFault("publisher_graph_query_failed");
      return;
    }

    GraphObservation observation;
    observation.publisher_count = endpoints.size();
    if (endpoints.empty() &&
        go2_motion_state_relay::MayDeferInitialPublisherAbsence(
            gid_guard_.candidate_seen(), started_steady_ns_, now_ns)) {
      return;
    }
    if (endpoints.size() == 1U) {
      const auto &endpoint = endpoints.front();
      observation.gid = endpoint.endpoint_gid();
      observation.identity_exact =
          endpoint.node_name() == kExpectedNodeName &&
          endpoint.node_namespace() == kExpectedNodeNamespace &&
          endpoint.topic_type() == kExpectedTopicType &&
          endpoint.endpoint_type() == rclcpp::EndpointType::Publisher;
      observation.qos_exact =
          IsSafeCorrectedStatePublisherQos(endpoint.qos_profile());
    }
    gid_guard_.ObserveGraph(observation);
    if (gid_guard_.faulted()) {
      LatchGuardFault();
      return;
    }
    if (gid_guard_.graph_bound()) {
      last_graph_verified_steady_ns_ = now_ns;
    }

    if (!last_receipt_steady_ns_.has_value()) {
      if (now_ns < started_steady_ns_ ||
          now_ns - started_steady_ns_ > kInitialMessageTimeoutNs) {
        LatchFault("corrected_state_initial_timeout");
      }
      return;
    }
    if (now_ns < last_receipt_steady_ns_.value()) {
      LatchFault("steady_receipt_clock_regressed");
    }
    // Missing corrected-state input deliberately produces no output.  The
    // downstream chassis owns the 200 ms motion watchdog and therefore stops
    // before a stale command can persist.  Do not make the relay process
    // permanently unrecoverable for the same absence: a short timestamp
    // recovery or scheduler wobble can resume through the already frozen
    // graph/message GIDs.  Publisher disappearance/change, heartbeat loss,
    // invalid/future/non-monotonic stamps and every identity fault remain
    // sticky.  An old sample is dropped below without refreshing liveness;
    // the downstream chassis therefore still stops on its 200 ms watchdog.
  }

  void ObserveMessage(
      unitree_go::msg::SportModeState::ConstSharedPtr message,
      const rclcpp::MessageInfo &message_info) {
    if (faulted_) {
      return;
    }
    // The corrected state can arrive before the first 50 ms graph timer tick.
    // Drain the private supervisor pipe in the callback as well so a heartbeat
    // already queued before worker launch is observed before freshness is
    // evaluated.  The sender timestamp is still used, so buffered/stalled
    // supervision cannot refresh an old heartbeat.
    DrainHeartbeat();
    if (faulted_) {
      return;
    }
    const auto &rmw_info = message_info.get_rmw_message_info();
    last_observed_gid_ = CopyGid(rmw_info.publisher_gid.data);
    const std::int64_t now_steady_ns = SteadyNowNs();
    if (!HeartbeatFresh(now_steady_ns)) {
      LatchFault("supervisor_heartbeat_stale", last_observed_gid_,
                 GidDomain::kMessage);
      return;
    }
    if (!gid_guard_.graph_bound()) {
      // A message-domain handle cannot establish graph-domain identity.
      return;
    }
    if (!last_graph_verified_steady_ns_.has_value() ||
        now_steady_ns < last_graph_verified_steady_ns_.value() ||
        now_steady_ns - last_graph_verified_steady_ns_.value() >
            kGraphFreshNs) {
      LatchFault("publisher_graph_verification_stale", last_observed_gid_,
                 GidDomain::kMessage);
      return;
    }

    const bool implementation_identifier_exact =
        rmw_info.publisher_gid.implementation_identifier != nullptr &&
        rmw_implementation_identifier_ ==
            rmw_info.publisher_gid.implementation_identifier;
    if (!gid_guard_.ObserveMessage(last_observed_gid_.value(),
                                   implementation_identifier_exact)) {
      if (gid_guard_.faulted()) {
        LatchGuardFault();
      }
      return;
    }

    const auto stamp_ns = CorrectedStampNs(*message);
    if (!stamp_ns.has_value()) {
      LatchFault("corrected_state_stamp_invalid", last_observed_gid_,
                 GidDomain::kMessage);
      return;
    }
    if (last_corrected_stamp_ns_.has_value() &&
        stamp_ns.value() <= last_corrected_stamp_ns_.value()) {
      LatchFault("corrected_state_stamp_nonmonotonic", last_observed_gid_,
                 GidDomain::kMessage);
      return;
    }
    const std::int64_t now_system_ns = SystemNowNs();
    if (now_system_ns <= 0 || now_steady_ns <= 0) {
      LatchFault("local_clock_invalid", last_observed_gid_,
                 GidDomain::kMessage);
      return;
    }
    const auto freshness =
        go2_motion_state_relay::EvaluateCorrectedStampFreshness(
            now_system_ns, stamp_ns.value());
    if (freshness ==
        go2_motion_state_relay::CorrectedStampFreshness::kTooFarFuture) {
      LatchFault("corrected_state_stamp_future", last_observed_gid_,
                 GidDomain::kMessage);
      return;
    }
    if (freshness ==
        go2_motion_state_relay::CorrectedStampFreshness::kTooOld) {
      // Wireless delivery and workstation scheduling may make one otherwise
      // valid corrected sample arrive outside the live window.  Never publish
      // it and never let it refresh the receipt clock.  Keeping the relay
      // process alive lets a disarmed startup recover on the next fresh
      // sample, while an armed chassis independently fails closed once its
      // existing 200 ms live-state deadline is crossed.
      return;
    }
    if (last_receipt_steady_ns_.has_value() &&
        now_steady_ns < last_receipt_steady_ns_.value()) {
      LatchFault("steady_receipt_clock_regressed", last_observed_gid_,
                 GidDomain::kMessage);
      return;
    }

    last_corrected_stamp_ns_ = stamp_ns;
    last_receipt_steady_ns_ = now_steady_ns;
    ++valid_samples_;
    if (!ready_emitted_) {
      if (valid_samples_ >= kMinimumReadySamples) {
        EmitReady();
      }
      // Qualification samples never enter the output topic.  The launcher
      // also does not start the sole intended adapter until Python commits the
      // worker READY event, so these first 30 samples cannot reach motion.
      return;
    }
    publisher_->publish(*message);
  }

  void EmitReady() {
    if (!gid_guard_.bound() || !gid_guard_.per_message_gid_verified() ||
        !gid_guard_.frozen_graph_gid().has_value() ||
        !gid_guard_.frozen_message_gid().has_value()) {
      LatchFault("internal_ready_without_frozen_gids");
      return;
    }
    const std::string line =
        "READY_V3\t" + GidToHex(gid_guard_.frozen_graph_gid().value()) +
        "\t" + GidToHex(gid_guard_.frozen_message_gid().value()) + "\t" +
        rmw_implementation_identifier_ + "\t" +
        std::to_string(valid_samples_) + "\t" +
        std::to_string(gid_guard_.last_publisher_count()) + "\t" +
        std::to_string(gid_guard_.stable_graph_polls()) + "\t" +
        std::to_string(gid_guard_.stable_message_samples()) + "\n";
    if (!WriteWholeEvent(event_fd_, line)) {
      faulted_ = true;
      std::cerr << "motion-state relay event pipe write failed\n";
      Shutdown();
      return;
    }
    ready_emitted_ = true;
  }

  void LatchGuardFault() {
    if (!gid_guard_.fault().has_value()) {
      LatchFault("internal_missing_gid_guard_fault");
      return;
    }
    const auto &fault = gid_guard_.fault().value();
    LatchFaultDetailed(fault.reason, fault.graph_gid, fault.message_gid,
                       fault.observed_gid, fault.observed_gid_domain,
                       fault.publisher_count);
  }

  void LatchFault(
      const std::string &reason,
      const std::optional<PublisherGid> &observed_gid = std::nullopt,
      const GidDomain observed_gid_domain = GidDomain::kNone,
      const std::optional<std::size_t> publisher_count_override =
          std::nullopt) {
    const GidDomain effective_observed_gid_domain =
        observed_gid.has_value() && observed_gid_domain == GidDomain::kNone
            ? GidDomain::kMessage
            : observed_gid_domain;
    LatchFaultDetailed(
        reason, gid_guard_.frozen_graph_gid(),
        gid_guard_.frozen_message_gid(), observed_gid,
        effective_observed_gid_domain,
        publisher_count_override.has_value()
            ? publisher_count_override.value()
            : gid_guard_.last_publisher_count());
  }

  void LatchFaultDetailed(
      const std::string &reason,
      const std::optional<PublisherGid> &graph_gid,
      const std::optional<PublisherGid> &message_gid,
      const std::optional<PublisherGid> &observed_gid,
      const GidDomain observed_gid_domain,
      const std::size_t publisher_count) {
    if (faulted_) {
      return;
    }
    faulted_ = true;
    const std::string line =
        "FAULT_V3\t" + reason + "\t" + OptionalGidToWire(graph_gid) +
        "\t" + OptionalGidToWire(message_gid) + "\t" +
        OptionalGidToWire(observed_gid) + "\t" +
        GidDomainToWire(observed_gid_domain) + "\t" +
        std::to_string(publisher_count) + "\t" +
        rmw_implementation_identifier_ + "\n";
    if (!WriteWholeEvent(event_fd_, line)) {
      std::cerr << "motion-state relay could not report fault: " << reason
                << '\n';
    }
    Shutdown();
  }

  int heartbeat_fd_;
  int event_fd_;
  const std::int64_t started_steady_ns_;
  const std::string rmw_implementation_identifier_;
  PublisherGidGuard gid_guard_;
  HeartbeatTimestampGuard heartbeat_guard_;
  std::optional<PublisherGid> last_observed_gid_;
  std::string heartbeat_buffer_;
  std::optional<std::int64_t> last_graph_verified_steady_ns_;
  std::optional<std::int64_t> last_receipt_steady_ns_;
  std::optional<std::int64_t> last_corrected_stamp_ns_;
  std::size_t valid_samples_{0U};
  bool ready_emitted_{false};
  bool faulted_{false};
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr publisher_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
};

}  // namespace

int main(int argc, char **argv) {
  std::signal(SIGPIPE, SIG_IGN);
  PipeFds pipes;
  try {
    pipes = ParseArguments(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << "motion-state relay argument error: " << error.what() << '\n';
    return 64;
  }

  int ros_argc = 1;
  char *ros_argv[] = {argv[0], nullptr};
  try {
    rclcpp::init(ros_argc, ros_argv);
    auto node = std::make_shared<MotionStateRelay>(pipes);
    rclcpp::spin(node);
    const bool faulted = node->faulted();
    node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    ::close(pipes.heartbeat);
    ::close(pipes.events);
    return faulted ? 70 : 0;
  } catch (const std::exception &error) {
    std::string rmw_identifier = "-";
    try {
      rmw_identifier = CurrentRmwIdentifier();
    } catch (const std::exception &) {
    }
    const std::string event =
        "FAULT_V3\tworker_exception\t-\t-\t-\t-\t0\t" +
        rmw_identifier + "\n";
    (void)WriteWholeEvent(pipes.events, event);
    std::cerr << "motion-state relay exception: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    ::close(pipes.heartbeat);
    ::close(pipes.events);
    return 70;
  }
}
