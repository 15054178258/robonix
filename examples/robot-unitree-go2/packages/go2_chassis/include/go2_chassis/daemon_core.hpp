#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include "go2_chassis/motion_timing.hpp"
#include "go2_chassis/protocol.hpp"
#include "go2_chassis/sport_client_interface.hpp"

namespace go2_chassis {

// The SDK call timeouts and the adapter IPC timeout were reviewed against a
// 300 ms daemon watchdog.  A no-motion daemon may retain the wider diagnostic
// CLI range, but every motion-capable profile must use that exact value.
inline bool MotionWatchdogDeploymentEligible(bool allow_motion,
                                             std::uint64_t watchdog_ms) {
  return !allow_motion || watchdog_ms == kAuditedMotionWatchdogMs;
}

struct DaemonConfig {
  bool allow_motion{false};
  // Commissioning profiles consume one arm for the lifetime of the daemon.
  // The long-running Nav2 profile may re-arm only after an explicit disarm;
  // all velocity limits and watchdogs remain identical on every arm.
  bool repeatable_arm{false};
  // The persistent operator profile may request Unitree's official ClassicWalk
  // mode.  It stays opt-in so commissioning and no-motion profiles keep their
  // existing byte-for-byte command sequence.
  bool preserve_classic_walk{false};
  float max_vx{0.05F};
  float max_vy{0.0F};
  float max_wz{0.0F};
  std::uint64_t watchdog_ns{300'000'000ULL};
  std::uint64_t max_motion_ns{2'000'000'000ULL};
};

class DaemonCore {
 public:
  DaemonCore(DaemonConfig config, ISportClient &client)
      : config_(config), client_(client) {}

  bool armed() const { return armed_; }
  bool faulted() const { return faulted_; }
  bool moving() const { return moving_; }
  bool stop_unconfirmed() const { return stop_unconfirmed_; }

  ReplyPacket Handle(const CommandPacket &packet, std::uint64_t now_ns) {
    const ReplyCode validation = ValidateCommand(packet, now_ns);
    if (validation != ReplyCode::kOk) {
      return Reply(packet.sequence, validation);
    }
    if (packet.sequence <= last_sequence_) {
      return Reply(packet.sequence, ReplyCode::kReplay);
    }
    last_sequence_ = packet.sequence;
    last_packet_ns_ = now_ns;

    const auto operation = static_cast<CommandOp>(packet.operation);
    switch (operation) {
      case CommandOp::kArm:
        if (!config_.allow_motion) {
          return Reply(packet.sequence, ReplyCode::kMotionDisabled);
        }
        if (faulted_) {
          return Reply(packet.sequence, ReplyCode::kFaultLatched);
        }
        // Commissioning daemons remain one-shot.  Standard Nav2 is a
        // long-running service, so its explicitly selected profile may arm
        // again after the previous run completed an explicit stop/disarm.
        if (commissioning_arm_spent_ && !config_.repeatable_arm) {
          return Reply(packet.sequence, ReplyCode::kFaultLatched);
        }
        commissioning_arm_spent_ = true;
        if (client_.PrepareArm() != 0) {
          armed_ = false;
          moving_ = false;
          faulted_ = true;
          return Reply(packet.sequence, ReplyCode::kSdkError);
        }
        if (config_.preserve_classic_walk && client_.ClassicWalk(true) != 0) {
          armed_ = false;
          moving_ = false;
          faulted_ = true;
          return Reply(packet.sequence, ReplyCode::kSdkError);
        }
        armed_ = true;
        moving_ = false;
        motion_start_ns_ = 0U;
        return Reply(packet.sequence, ReplyCode::kOk);

      case CommandOp::kDisarm:
        return Reply(packet.sequence, ExplicitDisarm());

      case CommandOp::kStop:
        if (!config_.allow_motion || !armed_) {
          return Reply(packet.sequence, config_.allow_motion
                                            ? ReplyCode::kNotArmed
                                            : ReplyCode::kMotionDisabled);
        }
        return Reply(packet.sequence, Stop(false));

      case CommandOp::kPing:
        if (!config_.allow_motion || !armed_) {
          return Reply(packet.sequence, config_.allow_motion
                                            ? ReplyCode::kNotArmed
                                            : ReplyCode::kMotionDisabled);
        }
        return Reply(packet.sequence, ReplyCode::kOk);

      case CommandOp::kRestoreClassicWalk:
        if (!config_.allow_motion || !config_.preserve_classic_walk) {
          return Reply(packet.sequence, ReplyCode::kMotionDisabled);
        }
        // This mode command is intentionally a separate, post-disarm phase.
        // Never append it to StopMove or any fault/watchdog/disconnect path.
        if (armed_ || moving_ || faulted_ || stop_unconfirmed_) {
          return Reply(packet.sequence, ReplyCode::kFaultLatched);
        }
        if (client_.ClassicWalk(true) != 0) {
          return Reply(packet.sequence, ReplyCode::kSdkError);
        }
        return Reply(packet.sequence, ReplyCode::kOk);

      case CommandOp::kMove:
        if (!config_.allow_motion) {
          return Reply(packet.sequence, ReplyCode::kMotionDisabled);
        }
        if (!armed_ || faulted_) {
          return Reply(packet.sequence, ReplyCode::kNotArmed);
        }
        // Reverse motion is outside the initial audited envelope.  Reject it
        // here independently even if a compromised/buggy adapter bypasses its
        // own forward-only clamp.
        if (packet.vx < 0.0F || packet.vx > config_.max_vx ||
            std::fabs(packet.vy) > config_.max_vy ||
            std::fabs(packet.wz) > config_.max_wz) {
          FaultStop();
          return Reply(packet.sequence, ReplyCode::kLimitExceeded);
        }
        if (IsZeroVelocity(packet)) {
          return Reply(packet.sequence, Stop(false));
        }
        if (motion_start_ns_ == 0U) {
          motion_start_ns_ = now_ns;
        }
        if (now_ns < motion_start_ns_ ||
            (config_.max_motion_ns != 0U &&
             now_ns - motion_start_ns_ >= config_.max_motion_ns)) {
          FaultStop();
          return Reply(packet.sequence, ReplyCode::kLimitExceeded);
        }
        return Reply(packet.sequence, Move(packet));
    }
    return Reply(packet.sequence, ReplyCode::kMalformed);
  }

  bool CheckWatchdog(std::uint64_t now_ns) {
    // A failed StopMove acknowledgement is not equivalent to a stopped robot.
    // Keep retrying the stop while the daemon is alive, even though the local
    // arm latch has already been cleared.  The SDK call itself has a bounded
    // timeout, so this cannot keep a stale Move stream alive.
    bool stop_attempted = false;
    if (stop_unconfirmed_) {
      (void)StopAndDisarm();
      stop_attempted = true;
    }
    if (!armed_) {
      return stop_attempted;
    }
    if (motion_start_ns_ != 0U && config_.max_motion_ns != 0U &&
        (now_ns < motion_start_ns_ ||
         now_ns - motion_start_ns_ >= config_.max_motion_ns)) {
      FaultStop();
      return true;
    }
    if (last_packet_ns_ == 0U || now_ns < last_packet_ns_ ||
        now_ns - last_packet_ns_ <= config_.watchdog_ns) {
      return false;
    }
    FaultStop();
    return true;
  }

  void OnDisconnect() {
    (void)StopAndDisarm();
    // Give disconnect and process-shutdown cleanup one immediate retry.  If it
    // still fails, CheckWatchdog continues bounded-time retries for as long as
    // the daemon remains alive; local state always stays disarmed and faulted.
    if (stop_unconfirmed_) {
      (void)StopAndDisarm();
    }
    last_sequence_ = 0U;
    last_packet_ns_ = 0U;
    motion_start_ns_ = 0U;
  }

 private:
  ReplyPacket Reply(std::uint64_t sequence, ReplyCode code) const {
    return MakeReply(sequence, code, armed_, faulted_);
  }

  ReplyCode Move(const CommandPacket &packet) {
    const std::int32_t result = client_.Move(packet.vx, packet.vy, packet.wz);
    if (result != 0) {
      FaultStop();
      return ReplyCode::kSdkError;
    }
    moving_ = true;
    return ReplyCode::kOk;
  }

  ReplyCode Stop(bool disarm) {
    const std::int32_t result = client_.StopMove();
    moving_ = false;
    if (disarm) {
      armed_ = false;
    }
    if (result != 0) {
      armed_ = false;
      faulted_ = true;
      stop_unconfirmed_ = true;
      return ReplyCode::kSdkError;
    }
    stop_unconfirmed_ = false;
    return ReplyCode::kOk;
  }

  ReplyCode StopAndDisarm() {
    if (!config_.allow_motion) {
      armed_ = false;
      moving_ = false;
      motion_start_ns_ = 0U;
      stop_unconfirmed_ = false;
      return ReplyCode::kOk;
    }
    if (!armed_ && !moving_ && !faulted_ && !stop_unconfirmed_) {
      motion_start_ns_ = 0U;
      return ReplyCode::kOk;
    }
    return Stop(true);
  }

  ReplyCode ExplicitDisarm() {
    if (!config_.allow_motion) {
      armed_ = false;
      moving_ = false;
      faulted_ = false;
      motion_start_ns_ = 0U;
      return ReplyCode::kOk;
    }
    // A latched SDK/limit/watchdog fault may already have set armed=false.
    // Retry StopMove anyway before permitting the explicit disarm to clear it.
    if (armed_ || faulted_ || moving_ || stop_unconfirmed_) {
      const std::int32_t result = client_.StopMove();
      moving_ = false;
      armed_ = false;
      motion_start_ns_ = 0U;
      if (result != 0) {
        faulted_ = true;
        stop_unconfirmed_ = true;
        return ReplyCode::kSdkError;
      }
      stop_unconfirmed_ = false;
    }
    faulted_ = false;
    motion_start_ns_ = 0U;
    return ReplyCode::kOk;
  }

  void FaultStop() {
    if (config_.allow_motion && armed_) {
      stop_unconfirmed_ = client_.StopMove() != 0;
    }
    moving_ = false;
    armed_ = false;
    faulted_ = true;
  }

  DaemonConfig config_;
  ISportClient &client_;
  bool armed_{false};
  bool faulted_{false};
  bool moving_{false};
  bool stop_unconfirmed_{false};
  bool commissioning_arm_spent_{false};
  std::uint64_t last_sequence_{0};
  std::uint64_t last_packet_ns_{0};
  std::uint64_t motion_start_ns_{0};
};

}  // namespace go2_chassis
