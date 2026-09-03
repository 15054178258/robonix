#ifndef GO2_SENSORS__STAMP_GUARD_HPP_
#define GO2_SENSORS__STAMP_GUARD_HPP_

#include <cstdint>

namespace go2_sensors
{

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

enum class StampVerdict
{
  kFresh,
  kZero,
  kStale,
  kFuture,
  kInvalid,
};

struct StampPolicy
{
  std::int64_t max_age_ns{0};
  std::int64_t max_future_offset_ns{0};
};

struct StampEvaluation
{
  StampVerdict verdict{StampVerdict::kInvalid};
  // Positive means the sample is in the past; negative means it is in the future.
  std::int64_t age_ns{0};
  bool age_available{false};

  bool fresh() const noexcept
  {
    return verdict == StampVerdict::kFresh;
  }
};

inline bool stamp_policy_within_limits(
  const StampPolicy & policy, const StampPolicy & ceiling) noexcept
{
  return ceiling.max_age_ns > 0 && ceiling.max_future_offset_ns >= 0 &&
         policy.max_age_ns > 0 && policy.max_age_ns <= ceiling.max_age_ns &&
         policy.max_future_offset_ns >= 0 &&
         policy.max_future_offset_ns <= ceiling.max_future_offset_ns;
}

inline const char * stamp_verdict_name(const StampVerdict verdict) noexcept
{
  switch (verdict) {
    case StampVerdict::kFresh:
      return "fresh";
    case StampVerdict::kZero:
      return "zero";
    case StampVerdict::kStale:
      return "stale";
    case StampVerdict::kFuture:
      return "future";
    case StampVerdict::kInvalid:
      return "invalid";
  }
  return "invalid";
}

inline StampEvaluation evaluate_header_stamp(
  const std::int32_t stamp_sec, const std::uint32_t stamp_nanosec,
  const std::int64_t now_ns, const StampPolicy & policy) noexcept
{
  if (stamp_sec == 0 && stamp_nanosec == 0U) {
    return {StampVerdict::kZero, 0, false};
  }
  if (stamp_sec < 0 || stamp_nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond) ||
    now_ns <= 0 || policy.max_age_ns <= 0 || policy.max_future_offset_ns < 0)
  {
    return {StampVerdict::kInvalid, 0, false};
  }

  const std::int64_t stamp_ns =
    static_cast<std::int64_t>(stamp_sec) * kNanosecondsPerSecond +
    static_cast<std::int64_t>(stamp_nanosec);
  const std::int64_t age_ns = now_ns - stamp_ns;
  if (age_ns > policy.max_age_ns) {
    return {StampVerdict::kStale, age_ns, true};
  }
  if (age_ns < -policy.max_future_offset_ns) {
    return {StampVerdict::kFuture, age_ns, true};
  }
  return {StampVerdict::kFresh, age_ns, true};
}

}  // namespace go2_sensors

#endif  // GO2_SENSORS__STAMP_GUARD_HPP_
