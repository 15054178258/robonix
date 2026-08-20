#pragma once

#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "robonix_nav2_terminal/navigate_goal_epoch.hpp"

namespace robonix_nav2_terminal {

class PersistentRotateToGoalCritic : public dwb_core::TrajectoryCritic {
public:
  void onInit() override;
  void reset() override;
  bool prepare(const geometry_msgs::msg::Pose2D &pose,
               const nav_2d_msgs::msg::Twist2D &velocity,
               const geometry_msgs::msg::Pose2D &goal,
               const nav_2d_msgs::msg::Path2D &global_plan) override;
  double
  scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) override;

private:
  void startGoal(const geometry_msgs::msg::Pose2D &goal);
  bool goalChanged(const geometry_msgs::msg::Pose2D &goal) const;
  [[noreturn]] void reject(const std::string &reason) const;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
      goal_status_sub_;
  NavigateGoalEpoch goal_epoch_;
  uint64_t seen_goal_epoch_{0};
  geometry_msgs::msg::Pose2D goal_;
  bool have_goal_{false};
  bool in_window_{false};
  bool rotating_{false};
  bool yaw_reached_{false};
  bool have_last_yaw_{false};
  double last_yaw_{0.0};
  double accumulated_rotation_{0.0};
  double allowed_rotation_{0.0};
  double goal_yaw_{0.0};
  double current_yaw_{0.0};
  double current_xy_speed_sq_{0.0};
  double stopped_xy_speed_sq_{0.0};
  double xy_enter_{0.30};
  double xy_exit_{0.45};
  double yaw_tolerance_{0.12};
  double yaw_exit_{0.20};
  double slowing_factor_{3.0};
  double max_terminal_angular_velocity_{0.30};
  double lookahead_time_{0.5};
  double max_terminal_duration_{15.0};
  double no_progress_timeout_{3.0};
  double min_yaw_progress_{0.04};
  double rotation_margin_{0.50};
  double goal_change_xy_{0.02};
  double goal_change_yaw_{0.03};
  double best_yaw_error_{0.0};
  rclcpp::Time entered_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_progress_at_{0, 0, RCL_ROS_TIME};
};

} // namespace robonix_nav2_terminal
