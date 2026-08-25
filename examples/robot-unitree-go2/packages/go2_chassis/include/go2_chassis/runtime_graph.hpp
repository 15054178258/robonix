#pragma once

namespace go2_chassis {

// ROS-independent description of the entities an adapter instance may create.
// The node consumes this plan before constructing any control-side object, so
// allow_motion=false is a passive graph property rather than only a callback
// check after control entities already exist.
struct RuntimeGraphPlan {
  bool sport_state_subscriptions;
  bool canonical_state_outputs;
  bool diagnostics_timer;
  bool seqpacket_client;
  bool cmd_vel_subscription;
  bool arm_service;
  bool control_timer;

  static constexpr RuntimeGraphPlan For(bool allow_motion) {
    return RuntimeGraphPlan{
        true,
        true,
        true,
        allow_motion,
        allow_motion,
        allow_motion,
        allow_motion,
    };
  }

  constexpr bool has_motion_control_entities() const {
    return seqpacket_client || cmd_vel_subscription || arm_service ||
           control_timer;
  }

  constexpr bool has_complete_motion_control_graph() const {
    return seqpacket_client && cmd_vel_subscription && arm_service &&
           control_timer;
  }

  constexpr bool is_consistent() const {
    return has_motion_control_entities() ==
           has_complete_motion_control_graph();
  }
};

}  // namespace go2_chassis
