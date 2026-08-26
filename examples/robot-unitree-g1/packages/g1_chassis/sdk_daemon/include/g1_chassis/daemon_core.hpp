#pragma once

#include <cstdint>
#include <functional>
#include <mutex>

#include "g1_chassis/protocol.hpp"

namespace g1_chassis {

// Forward declaration — implemented in loco_client_adaptor.hpp
class ILocoClient;

struct DaemonConfig {
  bool allow_motion{false};
  bool repeatable_arm{false};
  uint64_t watchdog_ns{300'000'000ULL};   // 300 ms default
  float max_vx{1.0F};
  float max_vy{0.5F};
  float max_wz{2.0F};
  uint64_t max_motion_ns{0ULL};            // 0 = infinite session
};

/**
 * @brief Core safety engine: handles commands, watchdog, and SDK calls.
 *
 * Thread safety: all public methods are mutex-protected. The caller serializes
 * calls through the main poll loop.
 */
class DaemonCore {
 public:
  DaemonCore(const DaemonConfig &config, ILocoClient &client);

  // Process a command packet from the adapter. Returns a reply packet.
  ReplyPacket Handle(const CommandPacket &cmd, uint64_t now_ns);

  // Check watchdog: returns true if a stop should be issued.
  bool CheckWatchdog(uint64_t now_ns);

  // Called when the adapter disconnects. Issues a stop.
  void OnDisconnect();

  // Request a stop that hasn't been confirmed by the SDK yet.
  // Returns true if stop is still unconfirmed.
  bool stop_unconfirmed();

  // Status queries for the reply packet.
  bool armed() const { return armed_; }
  bool faulted() const { return faulted_; }

 private:
  void IssueStop();
  void IssueVelocity(float vx, float vy, float omega);

  const DaemonConfig &config_;
  ILocoClient &client_;

  std::mutex mutex_;
  bool armed_{false};
  bool faulted_{false};
  bool stop_pending_{false};

  // Last time a valid command was successfully processed.
  uint64_t last_arm_time_ns_{0};
  uint64_t last_motion_time_ns_{0};

  // Sequence counter for the daemon's own reply stream.
  uint8_t sequence_{0};
};

/**
 * @brief Construct a default ReplyPacket.
 */
inline ReplyPacket MakeReply(uint8_t sequence, ReplyCode code,
                              bool armed, bool faulted) {
  ReplyPacket reply{};
  reply.type = static_cast<uint8_t>(PacketType::kReply);
  reply.sequence = sequence;
  reply.code = static_cast<uint8_t>(code);
  reply.armed = armed ? 1 : 0;
  reply.faulted = faulted ? 1 : 0;
  return reply;
}

/**
 * @brief Check if a daemon can be deployed with motion safety.
 * @return true if the watchdog_ms is exactly 300 ms and allow_motion is true.
 */
bool MotionWatchdogDeploymentEligible(bool allow_motion, uint64_t watchdog_ms);

}  // namespace g1_chassis
