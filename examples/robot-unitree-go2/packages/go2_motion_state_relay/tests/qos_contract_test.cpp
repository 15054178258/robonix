#include <cassert>
#include <iostream>

#include "go2_motion_state_relay/qos_contract.hpp"

namespace {

void TestCorrectedStateInputQosAndDiscoverablePublisherContract() {
  const auto expected = go2_motion_state_relay::CorrectedStateInputQos();
  const auto &profile = expected.get_rmw_qos_profile();
  assert(profile.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  assert(profile.depth == 1U);
  assert(profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  assert(profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE);
  assert(go2_motion_state_relay::IsSafeCorrectedStatePublisherQos(expected));

  // History and depth are explicitly not guaranteed by DDS graph discovery.
  const auto sensor_data = rclcpp::SensorDataQoS();
  assert(sensor_data.get_rmw_qos_profile().depth == 5U);
  assert(
      go2_motion_state_relay::IsSafeCorrectedStatePublisherQos(sensor_data));
  auto undiscovered_history = expected;
  undiscovered_history.get_rmw_qos_profile().history =
      RMW_QOS_POLICY_HISTORY_UNKNOWN;
  undiscovered_history.get_rmw_qos_profile().depth = 0U;
  assert(go2_motion_state_relay::IsSafeCorrectedStatePublisherQos(
      undiscovered_history));

  auto reliable = expected;
  reliable.reliable();
  assert(!go2_motion_state_relay::IsSafeCorrectedStatePublisherQos(reliable));
  auto transient_local = expected;
  transient_local.transient_local();
  assert(!go2_motion_state_relay::IsSafeCorrectedStatePublisherQos(
      transient_local));
  auto unknown = expected;
  unknown.get_rmw_qos_profile().reliability =
      RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
  assert(!go2_motion_state_relay::IsSafeCorrectedStatePublisherQos(unknown));
  unknown = expected;
  unknown.get_rmw_qos_profile().durability =
      RMW_QOS_POLICY_DURABILITY_UNKNOWN;
  assert(!go2_motion_state_relay::IsSafeCorrectedStatePublisherQos(unknown));
}

}  // namespace

int main() {
  TestCorrectedStateInputQosAndDiscoverablePublisherContract();
  std::cout << "qos_contract_test: PASS\n";
  return 0;
}
