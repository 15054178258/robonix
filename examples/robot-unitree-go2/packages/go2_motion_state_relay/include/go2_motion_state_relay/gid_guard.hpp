#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

namespace go2_motion_state_relay {

inline constexpr std::size_t kPublisherGidBytes = 24U;
inline constexpr std::size_t kRequiredStableGraphPolls = 3U;
inline constexpr std::size_t kRequiredStableMessageSamples = 3U;
inline constexpr std::int64_t kInitialGraphDiscoveryTimeoutNs =
    1'000'000'000LL;

using PublisherGid = std::array<std::uint8_t, kPublisherGidBytes>;

inline bool MayDeferInitialPublisherAbsence(
    const bool candidate_seen, const std::int64_t started_ns,
    const std::int64_t now_ns) noexcept {
  return !candidate_seen && started_ns > 0 && now_ns >= started_ns &&
         now_ns - started_ns < kInitialGraphDiscoveryTimeoutNs;
}

// Heartbeats carry the supervisor's CLOCK_MONOTONIC timestamp.  Checking the
// sender timestamp, rather than the pipe-read time, prevents buffered old
// heartbeats from becoming fresh after a stalled worker resumes.
class HeartbeatTimestampGuard final {
 public:
  bool Observe(const std::int64_t sender_ns) noexcept {
    if (faulted_) {
      return false;
    }
    if (sender_ns <= 0) {
      faulted_ = true;
      reason_ = "supervisor_heartbeat_protocol_error";
      return false;
    }
    if (last_sender_ns_.has_value() &&
        sender_ns <= last_sender_ns_.value()) {
      faulted_ = true;
      reason_ = "supervisor_heartbeat_nonmonotonic";
      return false;
    }
    last_sender_ns_ = sender_ns;
    return true;
  }

  bool Fresh(const std::int64_t now_ns,
             const std::int64_t maximum_age_ns) const noexcept {
    return !faulted_ && maximum_age_ns >= 0 && last_sender_ns_.has_value() &&
           now_ns >= last_sender_ns_.value() &&
           now_ns - last_sender_ns_.value() <= maximum_age_ns;
  }

  bool faulted() const noexcept { return faulted_; }
  const std::string &reason() const noexcept { return reason_; }
  const std::optional<std::int64_t> &last_sender_ns() const noexcept {
    return last_sender_ns_;
  }

 private:
  std::optional<std::int64_t> last_sender_ns_;
  std::string reason_;
  bool faulted_{false};
};

inline bool IsNonzeroGid(const PublisherGid &gid) noexcept {
  for (const auto byte : gid) {
    if (byte != 0U) {
      return true;
    }
  }
  return false;
}

inline bool IsCycloneGraphGidShape(const PublisherGid &gid) noexcept {
  const bool writer_guid_nonzero = std::any_of(
      gid.begin(), gid.begin() + 16, [](const std::uint8_t byte) {
        return byte != 0U;
      });
  const bool storage_tail_zero = std::all_of(
      gid.begin() + 16, gid.end(), [](const std::uint8_t byte) {
        return byte == 0U;
      });
  return writer_guid_nonzero && storage_tail_zero;
}

inline bool IsCycloneMessageGidShape(const PublisherGid &gid) noexcept {
  const bool publication_handle_nonzero = std::any_of(
      gid.begin(), gid.begin() + 8, [](const std::uint8_t byte) {
        return byte != 0U;
      });
  const bool storage_tail_zero = std::all_of(
      gid.begin() + 8, gid.end(), [](const std::uint8_t byte) {
        return byte == 0U;
      });
  return publication_handle_nonzero && storage_tail_zero;
}

inline std::string GidToHex(const PublisherGid &gid) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto byte : gid) {
    output << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return output.str();
}

struct GraphObservation {
  std::size_t publisher_count{0U};
  bool identity_exact{false};
  bool qos_exact{false};
  PublisherGid gid{};
};

enum class GidDomain {
  kNone,
  kGraph,
  kMessage,
};

inline const char *GidDomainToWire(const GidDomain domain) noexcept {
  switch (domain) {
    case GidDomain::kGraph:
      return "graph";
    case GidDomain::kMessage:
      return "message";
    case GidDomain::kNone:
      return "-";
  }
  return "-";
}

struct GuardFault {
  std::string reason;
  std::optional<PublisherGid> graph_gid;
  std::optional<PublisherGid> message_gid;
  std::optional<PublisherGid> observed_gid;
  GidDomain observed_gid_domain{GidDomain::kNone};
  std::size_t publisher_count{0U};
};

// Pure, ROS-independent publisher identity state machine.  ROS 2 Humble does
// not require a graph endpoint GID and rmw_message_info.publisher_gid to use
// the same byte representation.  Freeze each domain independently and never
// compare their bytes.  A fault is sticky: neither domain can rebind after a
// later graph poll or message sample.
class PublisherGidGuard final {
 public:
  void ObserveGraph(const GraphObservation &observation) {
    if (fault_.has_value()) {
      return;
    }
    last_publisher_count_ = observation.publisher_count;
    if (observation.publisher_count != 1U) {
      LatchFault("publisher_count_not_one", CurrentGraphGid(),
                 CurrentMessageGid(), std::nullopt, GidDomain::kNone,
                 observation.publisher_count);
      return;
    }
    if (!observation.identity_exact) {
      LatchFault("publisher_identity_mismatch", CurrentGraphGid(),
                 CurrentMessageGid(), observation.gid, GidDomain::kGraph,
                 observation.publisher_count);
      return;
    }
    if (!observation.qos_exact) {
      LatchFault("publisher_qos_mismatch", CurrentGraphGid(),
                 CurrentMessageGid(), observation.gid, GidDomain::kGraph,
                 observation.publisher_count);
      return;
    }
    if (!IsNonzeroGid(observation.gid)) {
      LatchFault("graph_publisher_gid_zero", CurrentGraphGid(),
                 CurrentMessageGid(), std::nullopt, GidDomain::kGraph,
                 observation.publisher_count);
      return;
    }
    if (!IsCycloneGraphGidShape(observation.gid)) {
      LatchFault("graph_publisher_gid_shape_mismatch", CurrentGraphGid(),
                 CurrentMessageGid(), observation.gid, GidDomain::kGraph,
                 observation.publisher_count);
      return;
    }

    if (frozen_graph_gid_.has_value()) {
      if (observation.gid != frozen_graph_gid_.value()) {
        LatchFault("graph_publisher_gid_changed", frozen_graph_gid_,
                   CurrentMessageGid(), observation.gid, GidDomain::kGraph,
                   observation.publisher_count);
      }
      return;
    }

    if (!graph_candidate_gid_.has_value()) {
      graph_candidate_gid_ = observation.gid;
      stable_graph_polls_ = 1U;
    } else if (graph_candidate_gid_.value() != observation.gid) {
      LatchFault("graph_publisher_gid_changed_before_freeze",
                 graph_candidate_gid_, CurrentMessageGid(), observation.gid,
                 GidDomain::kGraph, observation.publisher_count);
      return;
    } else {
      ++stable_graph_polls_;
    }

    if (stable_graph_polls_ == kRequiredStableGraphPolls) {
      frozen_graph_gid_ = graph_candidate_gid_;
    }
  }

  // Before graph identity freezes, message metadata is deliberately discarded:
  // its GID belongs to a different RMW representation and cannot bind graph
  // identity.  After graph freeze, require a matching implementation identifier
  // plus three stable, nonzero message-domain GIDs before accepting a sample.
  bool ObserveMessage(const PublisherGid &observed_gid,
                      const bool implementation_identifier_exact) {
    if (fault_.has_value()) {
      return false;
    }
    if (!frozen_graph_gid_.has_value()) {
      return false;
    }
    if (!implementation_identifier_exact) {
      LatchFault("message_publisher_gid_implementation_mismatch",
                 frozen_graph_gid_, CurrentMessageGid(), observed_gid,
                 GidDomain::kMessage, last_publisher_count_);
      return false;
    }
    if (!IsNonzeroGid(observed_gid)) {
      LatchFault("message_publisher_gid_zero", frozen_graph_gid_,
                 CurrentMessageGid(), std::nullopt, GidDomain::kMessage,
                 last_publisher_count_);
      return false;
    }
    if (!IsCycloneMessageGidShape(observed_gid)) {
      LatchFault("message_publisher_gid_shape_mismatch", frozen_graph_gid_,
                 CurrentMessageGid(), observed_gid, GidDomain::kMessage,
                 last_publisher_count_);
      return false;
    }

    if (frozen_message_gid_.has_value()) {
      if (observed_gid != frozen_message_gid_.value()) {
        LatchFault("message_publisher_gid_changed", frozen_graph_gid_,
                   frozen_message_gid_, observed_gid, GidDomain::kMessage,
                   last_publisher_count_);
        return false;
      }
      per_message_gid_verified_ = true;
      return true;
    }

    if (!message_candidate_gid_.has_value()) {
      message_candidate_gid_ = observed_gid;
      stable_message_samples_ = 1U;
    } else if (message_candidate_gid_.value() != observed_gid) {
      LatchFault("message_publisher_gid_changed_before_freeze",
                 frozen_graph_gid_, message_candidate_gid_, observed_gid,
                 GidDomain::kMessage, last_publisher_count_);
      return false;
    } else {
      ++stable_message_samples_;
    }

    if (stable_message_samples_ < kRequiredStableMessageSamples) {
      return false;
    }
    frozen_message_gid_ = message_candidate_gid_;
    per_message_gid_verified_ = true;
    return true;
  }

  bool graph_bound() const noexcept { return frozen_graph_gid_.has_value(); }
  bool message_bound() const noexcept {
    return frozen_message_gid_.has_value();
  }
  bool bound() const noexcept { return graph_bound() && message_bound(); }
  bool faulted() const noexcept { return fault_.has_value(); }
  bool per_message_gid_verified() const noexcept {
    return per_message_gid_verified_;
  }
  bool candidate_seen() const noexcept {
    return graph_candidate_gid_.has_value();
  }
  std::size_t stable_graph_polls() const noexcept {
    return stable_graph_polls_;
  }
  std::size_t stable_message_samples() const noexcept {
    return stable_message_samples_;
  }
  std::size_t last_publisher_count() const noexcept {
    return last_publisher_count_;
  }
  const std::optional<PublisherGid> &frozen_graph_gid() const noexcept {
    return frozen_graph_gid_;
  }
  const std::optional<PublisherGid> &frozen_message_gid() const noexcept {
    return frozen_message_gid_;
  }
  const std::optional<GuardFault> &fault() const noexcept { return fault_; }

 private:
  std::optional<PublisherGid> CurrentGraphGid() const {
    if (frozen_graph_gid_.has_value()) {
      return frozen_graph_gid_;
    }
    return graph_candidate_gid_;
  }

  std::optional<PublisherGid> CurrentMessageGid() const {
    if (frozen_message_gid_.has_value()) {
      return frozen_message_gid_;
    }
    return message_candidate_gid_;
  }

  void LatchFault(const std::string &reason,
                  const std::optional<PublisherGid> &graph_gid,
                  const std::optional<PublisherGid> &message_gid,
                  const std::optional<PublisherGid> &observed_gid,
                  const GidDomain observed_gid_domain,
                  const std::size_t publisher_count) {
    if (!fault_.has_value()) {
      fault_ = GuardFault{reason, graph_gid, message_gid, observed_gid,
                          observed_gid_domain, publisher_count};
    }
  }

  std::optional<PublisherGid> graph_candidate_gid_;
  std::optional<PublisherGid> frozen_graph_gid_;
  std::optional<PublisherGid> message_candidate_gid_;
  std::optional<PublisherGid> frozen_message_gid_;
  std::optional<GuardFault> fault_;
  std::size_t stable_graph_polls_{0U};
  std::size_t stable_message_samples_{0U};
  std::size_t last_publisher_count_{0U};
  bool per_message_gid_verified_{false};
};

}  // namespace go2_motion_state_relay
