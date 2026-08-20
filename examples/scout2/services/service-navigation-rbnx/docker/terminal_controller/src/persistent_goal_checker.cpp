#include "robonix_nav2_terminal/persistent_goal_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace robonix_nav2_terminal {

void PersistentGoalChecker::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) {
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error(
        "PersistentGoalChecker cannot lock lifecycle node");
  }
  plugin_name_ = plugin_name;
  goal_status_sub_ =
      node_->create_subscription<action_msgs::msg::GoalStatusArray>(
          "/navigate_to_pose/_action/status", rclcpp::QoS(10),
          [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
            goal_epoch_.observe(*msg);
          });
  const auto declare = [this](const char *suffix, double value) {
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + "." + suffix, rclcpp::ParameterValue(value));
  };
  declare("xy_enter_tolerance", xy_enter_);
  declare("xy_exit_tolerance", xy_exit_);
  declare("yaw_enter_tolerance", yaw_enter_);
  declare("yaw_exit_tolerance", yaw_exit_);
  declare("stopped_linear_velocity", stopped_linear_);
  declare("stopped_angular_velocity", stopped_angular_);
  declare("goal_change_xy", goal_change_xy_);
  declare("goal_change_yaw", goal_change_yaw_);
  node_->get_parameter(plugin_name_ + ".xy_enter_tolerance", xy_enter_);
  node_->get_parameter(plugin_name_ + ".xy_exit_tolerance", xy_exit_);
  node_->get_parameter(plugin_name_ + ".yaw_enter_tolerance", yaw_enter_);
  node_->get_parameter(plugin_name_ + ".yaw_exit_tolerance", yaw_exit_);
  node_->get_parameter(plugin_name_ + ".stopped_linear_velocity",
                       stopped_linear_);
  node_->get_parameter(plugin_name_ + ".stopped_angular_velocity",
                       stopped_angular_);
  node_->get_parameter(plugin_name_ + ".goal_change_xy", goal_change_xy_);
  node_->get_parameter(plugin_name_ + ".goal_change_yaw", goal_change_yaw_);
  if (!(xy_enter_ > 0.0 && xy_exit_ >= xy_enter_ && yaw_enter_ > 0.0 &&
        yaw_exit_ >= yaw_enter_)) {
    throw std::runtime_error(
        "PersistentGoalChecker hysteresis tolerances are invalid");
  }
}

void PersistentGoalChecker::reset() {
  // ControllerServer calls reset for every 5 Hz replan. State is instead
  // reset when isGoalReached observes a genuinely different goal.
}

bool PersistentGoalChecker::goalChanged(
    const geometry_msgs::msg::Pose &goal_pose) const {
  if (!have_goal_) {
    return true;
  }
  const double dx = goal_pose.position.x - goal_.position.x;
  const double dy = goal_pose.position.y - goal_.position.y;
  const double dyaw = angles::shortest_angular_distance(
      tf2::getYaw(goal_.orientation), tf2::getYaw(goal_pose.orientation));
  return std::hypot(dx, dy) > goal_change_xy_ ||
         std::abs(dyaw) > goal_change_yaw_;
}

void PersistentGoalChecker::startGoal(
    const geometry_msgs::msg::Pose &goal_pose) {
  goal_ = goal_pose;
  have_goal_ = true;
  xy_latched_ = false;
  yaw_latched_ = false;
}

bool PersistentGoalChecker::isGoalReached(
    const geometry_msgs::msg::Pose &query_pose,
    const geometry_msgs::msg::Pose &goal_pose,
    const geometry_msgs::msg::Twist &velocity) {
  const auto epoch = goal_epoch_.value();
  // The action UUID is authoritative after its first status arrives. The goal
  // pose is expressed in the controller's local frame and therefore shifts
  // during map->odom localization corrections even for the same action.
  if (shouldStartGoal(have_goal_, epoch, seen_goal_epoch_,
                      goalChanged(goal_pose))) {
    startGoal(goal_pose);
  }
  // The first status callback can arrive well after control starts. Adopt its
  // epoch without clearing state for the geometrically identical goal.
  if (epoch != 0) {
    seen_goal_epoch_ = epoch;
  }
  const double distance =
      std::hypot(query_pose.position.x - goal_pose.position.x,
                 query_pose.position.y - goal_pose.position.y);
  if (!xy_latched_ && distance <= xy_enter_) {
    xy_latched_ = true;
    RCLCPP_INFO(node_->get_logger(), "terminal XY latched at %.3f m", distance);
  }
  if (!xy_latched_ || distance > xy_exit_) {
    return false;
  }

  const double yaw_error = std::abs(angles::shortest_angular_distance(
      tf2::getYaw(query_pose.orientation), tf2::getYaw(goal_pose.orientation)));
  if (!yaw_latched_ && yaw_error <= yaw_enter_) {
    yaw_latched_ = true;
    RCLCPP_INFO(node_->get_logger(), "terminal yaw latched at %.3f rad",
                yaw_error);
  }
  if (!yaw_latched_ || yaw_error > yaw_exit_) {
    return false;
  }

  const double linear_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  return linear_speed <= stopped_linear_ &&
         std::abs(velocity.angular.z) <= stopped_angular_;
}

bool PersistentGoalChecker::getTolerances(
    geometry_msgs::msg::Pose &pose_tolerance,
    geometry_msgs::msg::Twist &vel_tolerance) {
  const double invalid = std::numeric_limits<double>::lowest();
  pose_tolerance.position.x = xy_exit_;
  pose_tolerance.position.y = xy_exit_;
  pose_tolerance.position.z = invalid;
  pose_tolerance.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw_exit_);
  vel_tolerance.linear.x = stopped_linear_;
  vel_tolerance.linear.y = stopped_linear_;
  vel_tolerance.linear.z = invalid;
  vel_tolerance.angular.x = invalid;
  vel_tolerance.angular.y = invalid;
  vel_tolerance.angular.z = stopped_angular_;
  return true;
}

} // namespace robonix_nav2_terminal

PLUGINLIB_EXPORT_CLASS(robonix_nav2_terminal::PersistentGoalChecker,
                       nav2_core::GoalChecker)
