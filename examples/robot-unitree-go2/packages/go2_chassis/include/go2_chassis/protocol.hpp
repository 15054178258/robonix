#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>

namespace go2_chassis {

constexpr std::uint32_t kProtocolMagic = 0x47324348U;  // "G2CH"
constexpr std::uint16_t kProtocolVersion = 1U;
constexpr std::uint64_t kMaximumPacketLifetimeNs = 1'000'000'000ULL;
constexpr std::uint64_t kMaximumClockLeadNs = 100'000'000ULL;

enum class CommandOp : std::uint8_t {
  kArm = 1,
  kDisarm = 2,
  kMove = 3,
  kStop = 4,
  kPing = 5,
  kRestoreClassicWalk = 6,
};

enum class ReplyCode : std::int32_t {
  kOk = 0,
  kMalformed = -1,
  kExpired = -2,
  kReplay = -3,
  kMotionDisabled = -4,
  kNotArmed = -5,
  kLimitExceeded = -6,
  kSdkError = -7,
  kPeerRejected = -8,
  kInternalError = -9,
  kFaultLatched = -10,
};

#pragma pack(push, 1)
struct CommandPacket {
  std::uint32_t magic{kProtocolMagic};
  std::uint16_t version{kProtocolVersion};
  std::uint16_t size{sizeof(CommandPacket)};
  std::uint64_t sequence{0};
  std::uint64_t sent_monotonic_ns{0};
  std::uint64_t deadline_monotonic_ns{0};
  std::uint8_t operation{static_cast<std::uint8_t>(CommandOp::kPing)};
  std::uint8_t reserved8{0};
  std::uint16_t reserved16{0};
  float vx{0.0F};
  float vy{0.0F};
  float wz{0.0F};
  std::uint32_t checksum{0};
};

struct ReplyPacket {
  std::uint32_t magic{kProtocolMagic};
  std::uint16_t version{kProtocolVersion};
  std::uint16_t size{sizeof(ReplyPacket)};
  std::uint64_t sequence{0};
  std::int32_t code{static_cast<std::int32_t>(ReplyCode::kInternalError)};
  std::uint8_t armed{0};
  std::uint8_t faulted{0};
  std::uint16_t reserved16{0};
  std::uint32_t checksum{0};
};
#pragma pack(pop)

static_assert(sizeof(CommandPacket) == 52U, "IPC command layout changed");
static_assert(sizeof(ReplyPacket) == 28U, "IPC reply layout changed");
static_assert(std::is_trivially_copyable<CommandPacket>::value,
              "IPC command must remain a wire-safe POD");
static_assert(std::is_trivially_copyable<ReplyPacket>::value,
              "IPC reply must remain a wire-safe POD");

inline std::uint32_t Fnv1a32(const void *data, std::size_t length) {
  const auto *bytes = static_cast<const std::uint8_t *>(data);
  std::uint32_t hash = 2166136261U;
  for (std::size_t index = 0; index < length; ++index) {
    hash ^= bytes[index];
    hash *= 16777619U;
  }
  return hash;
}

template <typename Packet>
inline std::uint32_t PacketChecksum(const Packet &packet) {
  Packet copy = packet;
  copy.checksum = 0U;
  return Fnv1a32(&copy, sizeof(copy));
}

inline void Seal(CommandPacket &packet) { packet.checksum = PacketChecksum(packet); }
inline void Seal(ReplyPacket &packet) { packet.checksum = PacketChecksum(packet); }

inline bool HasValidChecksum(const CommandPacket &packet) {
  return packet.checksum == PacketChecksum(packet);
}

inline bool HasValidChecksum(const ReplyPacket &packet) {
  return packet.checksum == PacketChecksum(packet);
}

inline bool IsKnownOperation(std::uint8_t value) {
  switch (static_cast<CommandOp>(value)) {
    case CommandOp::kArm:
    case CommandOp::kDisarm:
    case CommandOp::kMove:
    case CommandOp::kStop:
    case CommandOp::kPing:
    case CommandOp::kRestoreClassicWalk:
      return true;
  }
  return false;
}

inline bool IsZeroVelocity(const CommandPacket &packet, float epsilon = 1.0e-5F) {
  return std::fabs(packet.vx) <= epsilon && std::fabs(packet.vy) <= epsilon &&
         std::fabs(packet.wz) <= epsilon;
}

inline ReplyCode ValidateCommand(const CommandPacket &packet,
                                 std::uint64_t now_monotonic_ns,
                                 std::string *reason = nullptr) {
  const auto fail = [&](ReplyCode code, const char *message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return code;
  };

  if (packet.magic != kProtocolMagic || packet.version != kProtocolVersion ||
      packet.size != sizeof(CommandPacket) || !HasValidChecksum(packet) ||
      !IsKnownOperation(packet.operation)) {
    return fail(ReplyCode::kMalformed, "invalid header, operation, or checksum");
  }
  if (!std::isfinite(packet.vx) || !std::isfinite(packet.vy) ||
      !std::isfinite(packet.wz)) {
    return fail(ReplyCode::kMalformed, "velocity contains NaN or infinity");
  }
  if (packet.sent_monotonic_ns > now_monotonic_ns + kMaximumClockLeadNs) {
    return fail(ReplyCode::kMalformed, "packet timestamp is in the future");
  }
  if (packet.deadline_monotonic_ns < now_monotonic_ns) {
    return fail(ReplyCode::kExpired, "packet deadline has expired");
  }
  if (packet.deadline_monotonic_ns < packet.sent_monotonic_ns ||
      packet.deadline_monotonic_ns - packet.sent_monotonic_ns >
          kMaximumPacketLifetimeNs) {
    return fail(ReplyCode::kMalformed, "packet lifetime is invalid");
  }
  const auto operation = static_cast<CommandOp>(packet.operation);
  if (operation != CommandOp::kMove && !IsZeroVelocity(packet)) {
    return fail(ReplyCode::kMalformed, "non-move packet carries velocity");
  }
  return ReplyCode::kOk;
}

inline bool ValidateReply(const ReplyPacket &reply, std::uint64_t expected_sequence) {
  return reply.magic == kProtocolMagic && reply.version == kProtocolVersion &&
         reply.size == sizeof(ReplyPacket) && reply.sequence == expected_sequence &&
         HasValidChecksum(reply);
}

inline CommandPacket MakeCommand(CommandOp operation, std::uint64_t sequence,
                                 std::uint64_t now_monotonic_ns,
                                 std::uint64_t lifetime_ns, float vx = 0.0F,
                                 float vy = 0.0F, float wz = 0.0F) {
  CommandPacket packet;
  packet.sequence = sequence;
  packet.sent_monotonic_ns = now_monotonic_ns;
  packet.deadline_monotonic_ns = now_monotonic_ns + lifetime_ns;
  packet.operation = static_cast<std::uint8_t>(operation);
  packet.vx = vx;
  packet.vy = vy;
  packet.wz = wz;
  Seal(packet);
  return packet;
}

inline ReplyPacket MakeReply(std::uint64_t sequence, ReplyCode code, bool armed,
                             bool faulted) {
  ReplyPacket reply;
  reply.sequence = sequence;
  reply.code = static_cast<std::int32_t>(code);
  reply.armed = armed ? 1U : 0U;
  reply.faulted = faulted ? 1U : 0U;
  Seal(reply);
  return reply;
}

}  // namespace go2_chassis
