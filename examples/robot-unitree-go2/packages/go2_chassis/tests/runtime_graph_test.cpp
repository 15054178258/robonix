#include <cassert>

#include "go2_chassis/runtime_graph.hpp"

int main() {
  constexpr auto passive = go2_chassis::RuntimeGraphPlan::For(false);
  static_assert(passive.sport_state_subscriptions);
  static_assert(passive.canonical_state_outputs);
  static_assert(passive.diagnostics_timer);
  static_assert(!passive.seqpacket_client);
  static_assert(!passive.cmd_vel_subscription);
  static_assert(!passive.arm_service);
  static_assert(!passive.control_timer);
  static_assert(!passive.has_motion_control_entities());
  static_assert(!passive.has_complete_motion_control_graph());
  static_assert(passive.is_consistent());

  constexpr auto motion_enabled = go2_chassis::RuntimeGraphPlan::For(true);
  static_assert(motion_enabled.sport_state_subscriptions);
  static_assert(motion_enabled.canonical_state_outputs);
  static_assert(motion_enabled.diagnostics_timer);
  static_assert(motion_enabled.seqpacket_client);
  static_assert(motion_enabled.cmd_vel_subscription);
  static_assert(motion_enabled.arm_service);
  static_assert(motion_enabled.control_timer);
  static_assert(motion_enabled.has_motion_control_entities());
  static_assert(motion_enabled.has_complete_motion_control_graph());
  static_assert(motion_enabled.is_consistent());

  assert(!passive.has_motion_control_entities());
  assert(motion_enabled.has_complete_motion_control_graph());
  return 0;
}
