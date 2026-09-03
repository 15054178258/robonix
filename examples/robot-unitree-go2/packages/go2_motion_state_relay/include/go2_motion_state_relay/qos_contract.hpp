#pragma once

#include "rclcpp/qos.hpp"

namespace go2_motion_state_relay {

// The Python timestamp discipline publishes corrected core state with this
// exact QoS profile.  Keep the local relay subscription on the same
// freshest-sample contract.
inline rclcpp::QoS CorrectedStateInputQos() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

// DDS discovery does not guarantee that publisher history or history depth is
// available (rmw_get_publishers_info_by_topic explicitly excludes those two
// policies).  Validate the graph-discoverable policies which affect endpoint
// matching and this relay's state-stream contract.  UNKNOWN is fail-closed
// because it cannot equal either required enum.
inline bool IsSafeCorrectedStatePublisherQos(const rclcpp::QoS &observed) {
  const auto &profile = observed.get_rmw_qos_profile();
  return profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
         profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE;
}

}  // namespace go2_motion_state_relay
