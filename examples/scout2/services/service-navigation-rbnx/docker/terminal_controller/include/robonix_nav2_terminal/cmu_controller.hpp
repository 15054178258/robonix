#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"

namespace robonix_nav2_terminal
{

class CmuController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct Point
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  using CandidatePath = std::vector<Point>;

  /** Load the original CMU pre-generated trajectories grouped by path id. */
  void loadCandidatePaths(const std::string & path_file);

  /** Keep only nearby obstacle samples, transformed into the robot base frame. */
  std::vector<Point> obstacleCloudInBaseFrame() const;

  /** Return the path point that guides the local selector along Nav2's plan. */
  Point globalPlanDirection(const geometry_msgs::msg::PoseStamped & pose) const;

  /** A path collides when an obstacle reaches the configured swept footprint. */
  bool isCollisionFree(const CandidatePath & path, const std::vector<Point> & obstacles) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::string base_frame_;
  std::string obstacle_cloud_topic_;
  rclcpp::Logger logger_{rclcpp::get_logger("CmuController")};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;

  mutable std::mutex mutex_;
  nav_msgs::msg::Path global_plan_;
  sensor_msgs::msg::PointCloud2::SharedPtr obstacle_cloud_;
  std::vector<CandidatePath> candidate_paths_;

  double vehicle_length_{0.93};
  double vehicle_width_{0.80};
  double footprint_padding_{0.08};
  double adjacent_range_{4.25};
  double min_obstacle_z_{-0.50};
  double max_obstacle_z_{0.25};
  double lookahead_distance_{0.50};
  double global_plan_lookahead_{2.0};
  double obstacle_voxel_size_{0.10};
  int max_obstacle_points_{1200};
  double self_filter_radius_{0.65};
  double obstacle_timeout_{0.50};
  double transform_timeout_{0.10};
  double max_linear_speed_{0.50};
  double max_angular_speed_{0.60};
  double heading_gain_{1.8};
  double speed_limit_{100.0};
  bool speed_limit_is_percentage_{true};
};

}  // namespace robonix_nav2_terminal
