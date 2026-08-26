#pragma once

#include <cstdint>

namespace g1_chassis {

// IPC protocol — binary packet between adapter and daemon.
// Sized to fit in a single sendmsg() to avoid fragmentation.

// Packet types
enum class PacketType : uint8_t {
  kCmd = 1,
  kReply = 2,
};

// Reply codes returned by the daemon core.
enum class ReplyCode : uint8_t {
  kOk = 0,
  kDisabled = 1,      // motion disabled globally
  kDisarmed = 2,      // motion enabled but not yet armed (watchdog not yet
                       // received a valid cmd_vel)
  kFaulted = 3,       // watchdog faulted — stop has been issued
  kMalformed = 4,     // packet does not match expected format
  kSdkError = 5,      // SDK2 RPC failed
};

// ---- Command packet (adapter → daemon) ----
struct CommandPacket {
  uint8_t type;        // PacketType::kCmd
  uint8_t sequence;    // monotonically increasing, wraps at 255
  uint8_t reserved[6]; // reserved for alignment
  int32_t vx;          // scaled by 10000 (fixed-point)
  int32_t vy;          // scaled by 10000 (fixed-point)
  int32_t omega;       // scaled by 10000 (fixed-point)
};

static_assert(sizeof(CommandPacket) == 24,
              "CommandPacket must be exactly 24 bytes for IPC");

// ---- Reply packet (daemon → adapter) ----
struct ReplyPacket {
  uint8_t type;        // PacketType::kReply
  uint8_t sequence;    // echoes CommandPacket.sequence
  uint8_t code;        // ReplyCode
  uint8_t reserved[5]; // reserved
  uint8_t armed;       // 1 if motion is armed, 0 otherwise
  uint8_t faulted;     // 1 if in fault state, 0 otherwise
  uint8_t reserved2[2];
};

static_assert(sizeof(ReplyPacket) == 16,
              "ReplyPacket must be exactly 16 bytes for IPC");

}  // namespace g1_chassis
