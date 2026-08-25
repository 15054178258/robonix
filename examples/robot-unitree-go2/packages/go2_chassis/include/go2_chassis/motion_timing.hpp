#pragma once

#include <cstdint>

#include "go2_chassis/protocol.hpp"

namespace go2_chassis {

// Cross-process timing contract for every motion-capable profile.
//
// ARM may perform one verified ClassicWalk mode RPC and retains a bounded
// evidence-settlement window. Move performs one one-way SDK write plus a request
// evidence window; Stop and RestoreClassicWalk each perform one
// response-bearing SDK call plus the request/response evidence window.  They
// remain separate IPC operations so two synchronous SDK calls are never
// serialized inside one command deadline. The adapter must wait long enough
// for each audited operation, but every wait remains below the independent
// daemon watchdog.
inline constexpr std::int32_t kSdkSynchronousCallTimeoutMs = 100;
inline constexpr std::int32_t kRpcEvidenceArrivalTimeoutMs = 50;
inline constexpr std::int32_t kRpcEvidenceSettlementMs = 25;
inline constexpr std::int32_t kRpcEvidenceMaximumWaitMs =
    kRpcEvidenceArrivalTimeoutMs + kRpcEvidenceSettlementMs;
inline constexpr std::int32_t kMotionArmIpcReplyTimeoutMs = 290;
inline constexpr std::int32_t kMotionCommandIpcReplyTimeoutMs = 190;
inline constexpr std::int32_t kMotionPingIpcReplyTimeoutMs = 20;
inline constexpr std::uint64_t kAuditedMotionWatchdogMs = 300U;

static_assert(kSdkSynchronousCallTimeoutMs + kRpcEvidenceMaximumWaitMs <
                  kMotionArmIpcReplyTimeoutMs,
              "ARM IPC timeout must cover StopMove and evidence wait");
static_assert(kSdkSynchronousCallTimeoutMs + kRpcEvidenceMaximumWaitMs <
                  kMotionCommandIpcReplyTimeoutMs,
              "command IPC timeout must cover one SDK call and evidence wait");
static_assert(kMotionArmIpcReplyTimeoutMs <
                  static_cast<std::int32_t>(kAuditedMotionWatchdogMs),
              "ARM IPC timeout must stay below the daemon watchdog");
static_assert(kMotionCommandIpcReplyTimeoutMs <
                  static_cast<std::int32_t>(kAuditedMotionWatchdogMs),
              "command IPC timeout must stay below the daemon watchdog");

inline constexpr std::int32_t MotionIpcReplyTimeoutMs(CommandOp operation) {
  switch (operation) {
    case CommandOp::kArm:
      return kMotionArmIpcReplyTimeoutMs;
    case CommandOp::kMove:
    case CommandOp::kStop:
    case CommandOp::kDisarm:
    case CommandOp::kRestoreClassicWalk:
      return kMotionCommandIpcReplyTimeoutMs;
    case CommandOp::kPing:
      return kMotionPingIpcReplyTimeoutMs;
  }
  return kMotionPingIpcReplyTimeoutMs;
}

}  // namespace go2_chassis
