#include "robonix_nav2_terminal/goal_aware_navfn_planner.hpp"
#include "robonix_nav2_terminal/unknown_path_budget.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robonix_nav2_terminal {

void GoalAwareNavfnPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error(
        "GoalAwareNavfnPlanner cannot lock lifecycle node");
  }
  name_ = std::move(name);
  costmap_ = costmap_ros->getCostmap();

  known_planner_ = std::make_unique<nav2_navfn_planner::NavfnPlanner>();
  unknown_planner_ = std::make_unique<nav2_navfn_planner::NavfnPlanner>();
  known_planner_->configure(parent, name_ + "_known", tf, costmap_ros);
  unknown_planner_->configure(parent, name_ + "_unknown", tf, costmap_ros);

  const auto declare = [this](const char *suffix, double value) {
    nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + suffix,
                                                 rclcpp::ParameterValue(value));
  };
  declare("max_unknown_ratio", max_unknown_ratio_);
  declare("max_unknown_length", max_unknown_length_);
  declare("max_unknown_run", max_unknown_run_);
  node_->get_parameter(name_ + ".max_unknown_ratio", max_unknown_ratio_);
  node_->get_parameter(name_ + ".max_unknown_length", max_unknown_length_);
  node_->get_parameter(name_ + ".max_unknown_run", max_unknown_run_);
}

void GoalAwareNavfnPlanner::cleanup() {
  known_planner_->cleanup();
  unknown_planner_->cleanup();
}

void GoalAwareNavfnPlanner::activate() {
  known_planner_->activate();
  unknown_planner_->activate();
}

void GoalAwareNavfnPlanner::deactivate() {
  known_planner_->deactivate();
  unknown_planner_->deactivate();
}

nav_msgs::msg::Path
GoalAwareNavfnPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                  const geometry_msgs::msg::PoseStamped &goal) {
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx,
                            my)) {
    RCLCPP_WARN(node_->get_logger(),
                "%s rejected goal (%.2f, %.2f): outside global costmap",
                name_.c_str(), goal.pose.position.x, goal.pose.position.y);
    return nav_msgs::msg::Path();
  }

  const auto goal_cost = costmap_->getCost(mx, my);
  const bool goal_is_unknown = goal_cost == nav2_costmap_2d::NO_INFORMATION;
  RCLCPP_INFO(node_->get_logger(),
              "%s goal cell cost=%u mode=%s at map cell (%u, %u)",
              name_.c_str(), static_cast<unsigned int>(goal_cost),
              goal_is_unknown ? "unknown-goal" : "known-only", mx, my);

  if (goal_is_unknown) {
    return unknown_planner_->createPlan(start, goal);
  }

  nav_msgs::msg::Path known_path;
  try {
    known_path = known_planner_->createPlan(start, goal);
  } catch (const std::exception &error) {
    RCLCPP_WARN(node_->get_logger(),
                "%s known-only candidate failed: %s; evaluating bounded "
                "unknown fallback",
                name_.c_str(), error.what());
  }
  if (!known_path.poses.empty()) {
    return known_path;
  }

  auto fallback_path = unknown_planner_->createPlan(start, goal);
  if (fallback_path.poses.empty()) {
    return fallback_path;
  }
  const auto exposure = measureUnknownExposure(*costmap_, fallback_path);
  const double ratio = exposure.path_length > 0.0
                           ? exposure.unknown_length / exposure.path_length
                           : 0.0;
  if (!withinUnknownBudget(exposure, max_unknown_ratio_, max_unknown_length_,
                           max_unknown_run_)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "%s rejected fallback path: unknown %.2f/%.2f m (%.1f%%), longest "
        "unknown run %.2f m; limits %.2f m, %.1f%%, %.2f m",
        name_.c_str(), exposure.unknown_length, exposure.path_length,
        ratio * 100.0, exposure.max_unknown_run, max_unknown_length_,
        max_unknown_ratio_ * 100.0, max_unknown_run_);
    return nav_msgs::msg::Path();
  }

  RCLCPP_WARN(node_->get_logger(),
              "%s accepted bounded fallback path: unknown %.2f/%.2f m "
              "(%.1f%%), longest unknown run %.2f m",
              name_.c_str(), exposure.unknown_length, exposure.path_length,
              ratio * 100.0, exposure.max_unknown_run);
  return fallback_path;
}

UnknownExposure
measureUnknownExposure(const nav2_costmap_2d::Costmap2D &costmap,
                       const nav_msgs::msg::Path &path) {
  UnknownExposure exposure;
  const double sample_step = std::max(0.01, costmap.getResolution() * 0.5);
  double current_unknown_run = 0.0;

  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto &a = path.poses[i - 1].pose.position;
    const auto &b = path.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double segment_length = std::hypot(dx, dy);
    if (segment_length <= 0.0) {
      continue;
    }
    const auto samples = static_cast<unsigned int>(
        std::max(1.0, std::ceil(segment_length / sample_step)));
    const double sample_length = segment_length / samples;
    exposure.path_length += segment_length;
    for (unsigned int sample = 0; sample < samples; ++sample) {
      const double t = (static_cast<double>(sample) + 0.5) / samples;
      unsigned int mx = 0;
      unsigned int my = 0;
      const bool in_bounds =
          costmap.worldToMap(a.x + t * dx, a.y + t * dy, mx, my);
      const bool unknown = !in_bounds || costmap.getCost(mx, my) ==
                                             nav2_costmap_2d::NO_INFORMATION;
      if (unknown) {
        exposure.unknown_length += sample_length;
        current_unknown_run += sample_length;
        exposure.max_unknown_run =
            std::max(exposure.max_unknown_run, current_unknown_run);
      } else {
        current_unknown_run = 0.0;
      }
    }
  }
  return exposure;
}

bool withinUnknownBudget(const UnknownExposure &exposure,
                         double max_unknown_ratio, double max_unknown_length,
                         double max_unknown_run) {
  const double ratio = exposure.path_length > 0.0
                           ? exposure.unknown_length / exposure.path_length
                           : 0.0;
  return exposure.unknown_length <= max_unknown_length &&
         exposure.max_unknown_run <= max_unknown_run &&
         ratio <= max_unknown_ratio;
}

} // namespace robonix_nav2_terminal

PLUGINLIB_EXPORT_CLASS(robonix_nav2_terminal::GoalAwareNavfnPlanner,
                       nav2_core::GlobalPlanner)
