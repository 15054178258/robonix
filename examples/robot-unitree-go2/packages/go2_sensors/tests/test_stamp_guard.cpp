#include <cstdint>
#include <iostream>
#include <string>

#include "go2_sensors/stamp_guard.hpp"

namespace
{

using go2_sensors::StampPolicy;
using go2_sensors::StampVerdict;
using go2_sensors::evaluate_header_stamp;
using go2_sensors::stamp_policy_within_limits;

bool expect(
  const std::string & name, const StampVerdict expected,
  const std::int32_t sec, const std::uint32_t nanosec,
  const std::int64_t now_ns, const StampPolicy & policy)
{
  const auto result = evaluate_header_stamp(sec, nanosec, now_ns, policy);
  if (result.verdict == expected) {
    return true;
  }
  std::cerr << name << ": expected " << go2_sensors::stamp_verdict_name(expected)
            << ", got " << go2_sensors::stamp_verdict_name(result.verdict) << '\n';
  return false;
}

}  // namespace

int main()
{
  constexpr std::int64_t second = go2_sensors::kNanosecondsPerSecond;
  const StampPolicy policy{500000000LL, 50000000LL};
  const std::int64_t now = 10LL * second;
  bool ok = true;

  ok = stamp_policy_within_limits(policy, policy) && ok;
  ok = stamp_policy_within_limits(
    StampPolicy{200000000LL, 10000000LL}, policy) && ok;
  ok = !stamp_policy_within_limits(StampPolicy{0, 0}, policy) && ok;
  ok = !stamp_policy_within_limits(
    StampPolicy{500000001LL, 50000000LL}, policy) && ok;
  ok = !stamp_policy_within_limits(
    StampPolicy{500000000LL, -1}, policy) && ok;
  ok = !stamp_policy_within_limits(
    StampPolicy{500000000LL, 50000001LL}, policy) && ok;

  ok = expect("current", StampVerdict::kFresh, 10, 0U, now, policy) && ok;
  ok = expect("old boundary", StampVerdict::kFresh, 9, 500000000U, now, policy) && ok;
  ok = expect("too old", StampVerdict::kStale, 9, 499999999U, now, policy) && ok;
  ok = expect("future boundary", StampVerdict::kFresh, 10, 50000000U, now, policy) && ok;
  ok = expect("too far future", StampVerdict::kFuture, 10, 50000001U, now, policy) && ok;
  ok = expect("zero", StampVerdict::kZero, 0, 0U, now, policy) && ok;
  ok = expect("invalid nanosecond", StampVerdict::kInvalid, 9, 1000000000U, now, policy) && ok;
  ok = expect("negative second", StampVerdict::kInvalid, -1, 1U, now, policy) && ok;
  ok = expect("invalid clock", StampVerdict::kInvalid, 1, 1U, 0, policy) && ok;
  ok = expect(
    "invalid policy", StampVerdict::kInvalid, 9, 900000000U, now,
    StampPolicy{0, 50000000LL}) && ok;

  if (!ok) {
    return 1;
  }
  std::cout << "sensor timestamp guard tests passed\n";
  return 0;
}
