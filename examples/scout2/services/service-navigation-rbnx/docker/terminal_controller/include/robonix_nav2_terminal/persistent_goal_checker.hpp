#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robonix_nav2_terminal/navigate_goal_epoch.hpp"

namespace robonix_nav2_terminal
{

class PersistentGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  bool goalChanged(const geometry_msgs::msg::Pose & goal_pose) const;
  void startGoal(const geometry_msgs::msg::Pose & goal_pose);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;
  NavigateGoalEpoch goal_epoch_;
  uint64_t seen_goal_epoch_{0};
  std::string plugin_name_;
  geometry_msgs::msg::Pose goal_;
  bool have_goal_{false};
  bool xy_latched_{false};
  bool yaw_latched_{false};
  double xy_enter_{0.30};
  double xy_exit_{0.45};
  double yaw_enter_{0.12};
  double yaw_exit_{0.20};
  double stopped_linear_{0.05};
  double stopped_angular_{0.05};
  double goal_change_xy_{0.02};
  double goal_change_yaw_{0.03};
};

}  // namespace robonix_nav2_terminal
