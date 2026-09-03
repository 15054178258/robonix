#pragma once

#include <cstdint>

namespace go2_motion_state_relay {

constexpr std::int64_t kMaximumCorrectedAgeNs = 200'000'000LL;
constexpr std::int64_t kMaximumCorrectedFutureNs = 5'000'000LL;

enum class CorrectedStampFreshness {
  kFresh,
  kTooOld,
  kTooFarFuture,
};

constexpr CorrectedStampFreshness EvaluateCorrectedStampFreshness(
    const std::int64_t now_ns, const std::int64_t stamp_ns) noexcept {
  if (stamp_ns > now_ns &&
      stamp_ns - now_ns > kMaximumCorrectedFutureNs) {
    return CorrectedStampFreshness::kTooFarFuture;
  }
  if (stamp_ns <= now_ns &&
      now_ns - stamp_ns > kMaximumCorrectedAgeNs) {
    return CorrectedStampFreshness::kTooOld;
  }
  return CorrectedStampFreshness::kFresh;
}

}  // namespace go2_motion_state_relay
