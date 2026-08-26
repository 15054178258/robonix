#include "g1_chassis/daemon_core.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>

#include "g1_chassis/loco_client_interface.hpp"

namespace g1_chassis {

// ---- Deployment eligibility helper ----

bool MotionWatchdogDeploymentEligible(bool allow_motion, uint64_t watchdog_ms) {
  return allow_motion && watchdog_ms == 300U;
}

// ---- DaemonCore ----

DaemonCore::DaemonCore(const DaemonConfig &config, ILocoClient &client)
    : config_(config), client_(client) {}

void DaemonCore::IssueStop() {
  if (stop_pending_) return;
  stop_pending_ = true;
  int32_t result = client_.StopMove();
  if (result != 0) {
    std::cerr << "[g1-daemon] StopMove RPC returned " << result << "\n";
  } else {
    std::cerr << "[g1-daemon] StopMove acknowledged\n";
    stop_pending_ = false;
  }
}

void DaemonCore::IssueVelocity(float vx, float vy, float omega) {
  // Clamp to configured limits
  if (std::abs(vx) > config_.max_vx) {
    vx = std::copysign(config_.max_vx, vx);
  }
  if (std::abs(vy) > config_.max_vy) {
    vy = std::copysign(config_.max_vy, vy);
  }
  if (std::abs(omega) > config_.max_wz) {
    omega = std::copysign(config_.max_wz, omega);
  }
  // Send a continuous move (duration = infinity for streaming).
  client_.SetVelocity(vx, vy, omega, 86400.0F);
}

ReplyPacket DaemonCore::Handle(const CommandPacket &cmd, uint64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Always accept commands even when disarmed — the watchdog needs the stream.
  // But only act on them when armed.
  if (cmd.type != static_cast<uint8_t>(PacketType::kCmd)) {
    sequence_ = (sequence_ + 1) & 0xFF;
    return MakeReply(cmd.sequence, ReplyCode::kMalformed,
                     armed_, faulted_);
  }

  // Decode fixed-point velocity.
  double vx = cmd.vx / 10000.0;
  double vy = cmd.vy / 10000.0;
  double omega = cmd.omega / 10000.0;

  // Check if this is a zero-velocity command (adapter → daemon "stop" signal).
  bool is_zero = (std::abs(vx) < 0.0001F) && (std::abs(vy) < 0.0001F) &&
                 (std::abs(omega) < 0.0001F);

  if (config_.allow_motion) {
    // First valid command arms the daemon (zero-preamble safety).
    if (!armed_) {
      // Check SDK client is ready.
      int32_t arm_result = client_.PrepareArm();
      if (arm_result != 0) {
        sequence_ = (sequence_ + 1) & 0xFF;
        return MakeReply(cmd.sequence, ReplyCode::kSdkError, false, false);
      }
      // Issue BalanceStand to enter safe standing state.
      client_.BalanceStand();
      armed_ = true;
      last_arm_time_ns_ = now_ns;
      std::cerr << "[g1-daemon] ARMED (first valid cmd_vel)\n";
    }

    // Zero command: just update last-motion time, don't issue velocity.
    if (is_zero) {
      last_motion_time_ns_ = now_ns;
      sequence_ = (sequence_ + 1) & 0xFF;
      return MakeReply(cmd.sequence, ReplyCode::kOk, armed_, faulted_);
    }

    // Non-zero command: issue velocity.
    IssueVelocity(static_cast<float>(vx), static_cast<float>(vy),
                   static_cast<float>(omega));
    last_motion_time_ns_ = now_ns;
  }

  sequence_ = (sequence_ + 1) & 0xFF;
  return MakeReply(cmd.sequence, ReplyCode::kOk, armed_, faulted_);
}

bool DaemonCore::CheckWatchdog(uint64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!armed_ || faulted_) return false;

  // No watchdog when motion is disabled.
  if (!config_.allow_motion) return false;

  // If we're beyond the watchdog window without a valid command, fault.
  if (now_ns - last_motion_time_ns_ > config_.watchdog_ns) {
    if (!stop_pending_) {
      std::cerr << "[g1-daemon] WATCHDOG EXPIRED — issuing StopMove\n";
      IssueStop();
      faulted_ = true;
      return true;
    }
  }
  return false;
}

void DaemonCore::OnDisconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  armed_ = false;
  if (!stop_pending_) {
    std::cerr << "[g1-daemon] ADAPTER DISCONNECTED — issuing StopMove\n";
    client_.StopMove();
    stop_pending_ = true;
    faulted_ = true;
  }
}

bool DaemonCore::stop_unconfirmed() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_pending_) {
    return true;  // Still unconfirmed
  }
  // Retry stop if it was previously unconfirmed but client is ready.
  if (armed_ || stop_pending_) {
    client_.StopMove();
    return true;
  }
  return false;
}

}  // namespace g1_chassis
