#include "robonix_nav2_terminal/persistent_rotate_to_goal_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "angles/angles.h"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robonix_nav2_terminal {

void PersistentRotateToGoalCritic::onInit() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PersistentRotateToGoalCritic cannot lock node");
  }
  clock_ = node->get_clock();
  goal_status_sub_ =
      node->create_subscription<action_msgs::msg::GoalStatusArray>(
          "/navigate_to_pose/_action/status", rclcpp::QoS(10),
          [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
            goal_epoch_.observe(*msg);
          });
  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  const auto get = [&node, &prefix](const char *key, double value) {
    return nav_2d_utils::searchAndGetParam(node, prefix + key, value);
  };
  xy_enter_ = get("xy_enter_tolerance", xy_enter_);
  xy_exit_ = get("xy_exit_tolerance", xy_exit_);
  yaw_tolerance_ = get("yaw_enter_tolerance", yaw_tolerance_);
  yaw_exit_ = get("yaw_exit_tolerance", yaw_exit_);
  const double stopped_xy = get("stopped_linear_velocity", 0.05);
  stopped_xy_speed_sq_ = stopped_xy * stopped_xy;
  slowing_factor_ = get("slowing_factor", slowing_factor_);
  max_terminal_angular_velocity_ =
      get("max_terminal_angular_velocity", max_terminal_angular_velocity_);
  lookahead_time_ = get("lookahead_time", lookahead_time_);
  max_terminal_duration_ = get("max_terminal_duration", max_terminal_duration_);
  no_progress_timeout_ = get("no_progress_timeout", no_progress_timeout_);
  min_yaw_progress_ = get("min_yaw_progress", min_yaw_progress_);
  rotation_margin_ = get("rotation_margin", rotation_margin_);
  goal_change_xy_ = get("goal_change_xy", goal_change_xy_);
  goal_change_yaw_ = get("goal_change_yaw", goal_change_yaw_);
  if (!(xy_enter_ > 0.0 && xy_exit_ >= xy_enter_ && yaw_tolerance_ > 0.0 &&
        yaw_exit_ >= yaw_tolerance_ && max_terminal_duration_ > 0.0 &&
        no_progress_timeout_ > 0.0 && max_terminal_angular_velocity_ > 0.0)) {
    throw std::runtime_error(
        "PersistentRotateToGoalCritic parameters are invalid");
  }
}

void PersistentRotateToGoalCritic::reset() {
  // DWB calls reset on every global replan. prepare() owns reset semantics and
  // only clears terminal state when the final goal actually changes.
}

bool PersistentRotateToGoalCritic::goalChanged(
    const geometry_msgs::msg::Pose2D &goal) const {
  if (!have_goal_) {
    return true;
  }
  return std::hypot(goal.x - goal_.x, goal.y - goal_.y) > goal_change_xy_ ||
         std::abs(angles::shortest_angular_distance(goal_.theta, goal.theta)) >
             goal_change_yaw_;
}

void PersistentRotateToGoalCritic::startGoal(
    const geometry_msgs::msg::Pose2D &goal) {
  goal_ = goal;
  have_goal_ = true;
  in_window_ = false;
  rotating_ = false;
  yaw_reached_ = false;
  have_last_yaw_ = false;
  accumulated_rotation_ = 0.0;
  allowed_rotation_ = 0.0;
}

[[noreturn]] void
PersistentRotateToGoalCritic::reject(const std::string &reason) const {
  throw dwb_core::IllegalTrajectoryException(name_, reason);
}

bool PersistentRotateToGoalCritic::prepare(
    const geometry_msgs::msg::Pose2D &pose,
    const nav_2d_msgs::msg::Twist2D &velocity,
    const geometry_msgs::msg::Pose2D &goal, const nav_2d_msgs::msg::Path2D &) {
  const auto epoch = goal_epoch_.value();
  // The action UUID is authoritative after its first status arrives. The goal
  // pose is expressed in the controller's local frame and therefore shifts
  // during map->odom localization corrections even for the same action.
  if (shouldStartGoal(have_goal_, epoch, seen_goal_epoch_, goalChanged(goal))) {
    startGoal(goal);
  }
  // The first status callback can arrive after the terminal window is already
  // latched. Adopt it without treating the same geometric goal as preemption.
  if (epoch != 0) {
    seen_goal_epoch_ = epoch;
  }
  const auto now = clock_->now();
  const double distance = std::hypot(pose.x - goal.x, pose.y - goal.y);
  const double yaw_error =
      std::abs(angles::shortest_angular_distance(pose.theta, goal.theta));

  if (!in_window_ && distance <= xy_enter_) {
    in_window_ = true;
    entered_at_ = now;
    last_progress_at_ = now;
    best_yaw_error_ = yaw_error;
    allowed_rotation_ = std::max(0.50, yaw_error + rotation_margin_);
    last_yaw_ = pose.theta;
    have_last_yaw_ = true;
  }
  if (!in_window_) {
    return true;
  }
  if (distance > xy_exit_) {
    reject("terminal XY drift exceeded exit tolerance");
  }
  if ((now - entered_at_).seconds() > max_terminal_duration_) {
    reject("terminal rotation time limit exceeded");
  }
  if (have_last_yaw_) {
    accumulated_rotation_ +=
        std::abs(angles::shortest_angular_distance(last_yaw_, pose.theta));
  }
  last_yaw_ = pose.theta;
  have_last_yaw_ = true;
  if (accumulated_rotation_ > allowed_rotation_) {
    reject("terminal cumulative rotation limit exceeded");
  }
  if (yaw_error + min_yaw_progress_ < best_yaw_error_) {
    best_yaw_error_ = yaw_error;
    last_progress_at_ = now;
  } else if (!yaw_reached_ &&
             (now - last_progress_at_).seconds() > no_progress_timeout_) {
    reject("terminal yaw made no progress");
  }
  if (!yaw_reached_ && yaw_error <= yaw_tolerance_) {
    yaw_reached_ = true;
  }
  if (yaw_reached_ && yaw_error > yaw_exit_) {
    reject("terminal yaw moved outside exit tolerance after latching");
  }

  current_xy_speed_sq_ = velocity.x * velocity.x + velocity.y * velocity.y;
  rotating_ = rotating_ || current_xy_speed_sq_ <= stopped_xy_speed_sq_;
  goal_yaw_ = goal.theta;
  current_yaw_ = pose.theta;
  return true;
}

double PersistentRotateToGoalCritic::scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D &trajectory) {
  if (!in_window_) {
    return 0.0;
  }
  const double linear_speed_sq = trajectory.velocity.x * trajectory.velocity.x +
                                 trajectory.velocity.y * trajectory.velocity.y;
  if (yaw_reached_) {
    if (linear_speed_sq > 0.0) {
      reject("translation forbidden after terminal yaw latch");
    }
    // A critic cannot publish a zero Twist. Rejecting every dynamically
    // reachable nonzero sample makes DWB throw, so the controller aborts before
    // odometry reaches the goal checker's stopped threshold. Keep rotation
    // samples legal and score speed directly; DWB then selects the slowest
    // reachable command each tick until its zero sample becomes reachable.
    const double angular_speed = std::abs(trajectory.velocity.theta);
    return slowing_factor_ * angular_speed * angular_speed;
  }
  if (std::abs(trajectory.velocity.theta) > max_terminal_angular_velocity_) {
    reject("terminal angular velocity exceeds configured limit");
  }
  const double signed_yaw_error =
      angles::shortest_angular_distance(current_yaw_, goal_yaw_);
  if (std::abs(signed_yaw_error) > yaw_tolerance_ &&
      trajectory.velocity.theta * signed_yaw_error < 0.0) {
    reject("terminal angular velocity points away from goal yaw");
  }
  if (!rotating_) {
    if (linear_speed_sq >= current_xy_speed_sq_) {
      reject("not slowing down in terminal XY window");
    }
  } else if (linear_speed_sq > 0.0) {
    reject("translation forbidden after terminal rotation latch");
  }
  if (trajectory.poses.empty()) {
    reject("empty terminal trajectory");
  }
  double end_yaw = trajectory.poses.back().theta;
  if (lookahead_time_ >= 0.0) {
    end_yaw = dwb_core::projectPose(trajectory, lookahead_time_).theta;
  }
  const double rotation_score =
      std::abs(angles::shortest_angular_distance(end_yaw, goal_yaw_));
  return linear_speed_sq * slowing_factor_ + rotation_score;
}

} // namespace robonix_nav2_terminal

PLUGINLIB_EXPORT_CLASS(robonix_nav2_terminal::PersistentRotateToGoalCritic,
                       dwb_core::TrajectoryCritic)
