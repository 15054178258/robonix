#include <cassert>
#include <cstdint>

#include "go2_motion_state_relay/freshness_contract.hpp"

int main() {
  using go2_motion_state_relay::CorrectedStampFreshness;
  using go2_motion_state_relay::EvaluateCorrectedStampFreshness;
  using go2_motion_state_relay::kMaximumCorrectedAgeNs;
  using go2_motion_state_relay::kMaximumCorrectedFutureNs;

  constexpr std::int64_t now_ns = 1'700'000'000'000'000'000LL;
  static_assert(kMaximumCorrectedAgeNs == 200'000'000LL);
  static_assert(kMaximumCorrectedFutureNs == 5'000'000LL);

  assert(EvaluateCorrectedStampFreshness(
             now_ns, now_ns - kMaximumCorrectedAgeNs) ==
         CorrectedStampFreshness::kFresh);
  assert(EvaluateCorrectedStampFreshness(
             now_ns, now_ns - kMaximumCorrectedAgeNs - 1LL) ==
         CorrectedStampFreshness::kTooOld);
  assert(EvaluateCorrectedStampFreshness(
             now_ns, now_ns + kMaximumCorrectedFutureNs) ==
         CorrectedStampFreshness::kFresh);
  assert(EvaluateCorrectedStampFreshness(
             now_ns, now_ns + kMaximumCorrectedFutureNs + 1LL) ==
         CorrectedStampFreshness::kTooFarFuture);
  return 0;
}
