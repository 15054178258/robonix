#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace robonix_nav2_terminal {

class GoalAwareNavfnPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D *costmap_{nullptr};
  std::string name_;
  std::unique_ptr<nav2_navfn_planner::NavfnPlanner> known_planner_;
  std::unique_ptr<nav2_navfn_planner::NavfnPlanner> unknown_planner_;
  double max_unknown_ratio_{0.05};
  double max_unknown_length_{0.75};
  double max_unknown_run_{0.40};
};

} // namespace robonix_nav2_terminal
