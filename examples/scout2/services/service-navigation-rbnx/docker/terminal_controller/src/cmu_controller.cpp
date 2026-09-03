#include "robonix_nav2_terminal/cmu_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace robonix_nav2_terminal
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace

void CmuController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("CmuController cannot lock lifecycle node");
  }
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  base_frame_ = costmap_ros->getBaseFrameID();
  logger_ = node_->get_logger();

  const auto declare_double = [this](const char * suffix, double value) {
      nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + "." + suffix, rclcpp::ParameterValue(value));
    };
  const auto declare_string = [this](const char * suffix, const std::string & value) {
      nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + "." + suffix, rclcpp::ParameterValue(value));
    };

  declare_string("obstacle_cloud_topic", "/terrain_map");
  declare_string("candidate_paths_file", "");
  declare_double("vehicle_length", vehicle_length_);
  declare_double("vehicle_width", vehicle_width_);
  declare_double("footprint_padding", footprint_padding_);
  declare_double("adjacent_range", adjacent_range_);
  declare_double("min_obstacle_z", min_obstacle_z_);
  declare_double("max_obstacle_z", max_obstacle_z_);
  declare_double("lookahead_distance", lookahead_distance_);
  declare_double("global_plan_lookahead", global_plan_lookahead_);
  declare_double("obstacle_voxel_size", obstacle_voxel_size_);
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_obstacle_points", rclcpp::ParameterValue(max_obstacle_points_));
  declare_double("self_filter_radius", self_filter_radius_);
  declare_double("obstacle_timeout", obstacle_timeout_);
  declare_double("transform_timeout", transform_timeout_);
  declare_double("max_linear_speed", max_linear_speed_);
  declare_double("max_angular_speed", max_angular_speed_);
  declare_double("heading_gain", heading_gain_);

  std::string path_file;
  node_->get_parameter(plugin_name_ + ".obstacle_cloud_topic", obstacle_cloud_topic_);
  node_->get_parameter(plugin_name_ + ".candidate_paths_file", path_file);
  node_->get_parameter(plugin_name_ + ".vehicle_length", vehicle_length_);
  node_->get_parameter(plugin_name_ + ".vehicle_width", vehicle_width_);
  node_->get_parameter(plugin_name_ + ".footprint_padding", footprint_padding_);
  node_->get_parameter(plugin_name_ + ".adjacent_range", adjacent_range_);
  node_->get_parameter(plugin_name_ + ".min_obstacle_z", min_obstacle_z_);
  node_->get_parameter(plugin_name_ + ".max_obstacle_z", max_obstacle_z_);
  node_->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node_->get_parameter(plugin_name_ + ".global_plan_lookahead", global_plan_lookahead_);
  node_->get_parameter(plugin_name_ + ".obstacle_voxel_size", obstacle_voxel_size_);
  node_->get_parameter(plugin_name_ + ".max_obstacle_points", max_obstacle_points_);
  node_->get_parameter(plugin_name_ + ".self_filter_radius", self_filter_radius_);
  node_->get_parameter(plugin_name_ + ".obstacle_timeout", obstacle_timeout_);
  node_->get_parameter(plugin_name_ + ".transform_timeout", transform_timeout_);
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node_->get_parameter(plugin_name_ + ".heading_gain", heading_gain_);

  if (path_file.empty()) {
    path_file = ament_index_cpp::get_package_share_directory("robonix_nav2_terminal") +
      "/paths/paths.ply";
  }
  if (vehicle_length_ <= 0.0 || vehicle_width_ <= 0.0 || adjacent_range_ <= 0.0 ||
    obstacle_voxel_size_ <= 0.0 || max_obstacle_points_ <= 0 || self_filter_radius_ < 0.0 ||
    obstacle_timeout_ <= 0.0 ||
    transform_timeout_ < 0.0 || max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0)
  {
    throw std::runtime_error("CmuController received invalid vehicle or speed parameters");
  }
  loadCandidatePaths(path_file);

  obstacle_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    obstacle_cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      obstacle_cloud_ = std::move(message);
    });
  debug_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/local_path", rclcpp::QoS(1));
}

void CmuController::cleanup()
{
  obstacle_cloud_sub_.reset();
  debug_path_pub_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_cloud_.reset();
  global_plan_.poses.clear();
  candidate_paths_.clear();
}

void CmuController::activate() {}

void CmuController::deactivate() {}

void CmuController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = path;
}

void CmuController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

void CmuController::loadCandidatePaths(const std::string & path_file)
{
  std::ifstream input(path_file);
  if (!input.is_open()) {
    throw std::runtime_error("CmuController cannot open candidate paths: " + path_file);
  }

  std::string line;
  bool data_started = false;
  while (std::getline(input, line)) {
    if (line == "end_header") {
      data_started = true;
      break;
    }
  }
  if (!data_started) {
    throw std::runtime_error("CmuController candidate path file lacks PLY header terminator");
  }

  std::vector<CandidatePath> paths;
  while (std::getline(input, line)) {
    std::istringstream row(line);
    Point point;
    int path_id = -1;
    int group_id = -1;
    if (!(row >> point.x >> point.y >> point.z >> path_id >> group_id) || path_id < 0) {
      continue;
    }
    if (static_cast<size_t>(path_id) >= paths.size()) {
      paths.resize(static_cast<size_t>(path_id) + 1U);
    }
    paths[static_cast<size_t>(path_id)].push_back(point);
  }
  paths.erase(
    std::remove_if(paths.begin(), paths.end(), [](const CandidatePath & path) {
      return path.empty();
    }), paths.end());
  if (paths.empty()) {
    throw std::runtime_error("CmuController found no candidate paths in " + path_file);
  }
  candidate_paths_ = std::move(paths);
  RCLCPP_INFO(logger_, "CMU controller loaded %zu candidate paths from %s",
    candidate_paths_.size(), path_file.c_str());
}

std::vector<CmuController::Point> CmuController::obstacleCloudInBaseFrame() const
{
  sensor_msgs::msg::PointCloud2::SharedPtr obstacle_cloud;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_cloud = obstacle_cloud_;
  }
  if (!obstacle_cloud) {
    throw nav2_core::PlannerException("CMU obstacle point cloud has not arrived");
  }
  if (obstacle_cloud->header.frame_id.empty()) {
    throw nav2_core::PlannerException("CMU obstacle point cloud has no frame_id");
  }
  const rclcpp::Time cloud_stamp(
    obstacle_cloud->header.stamp, node_->get_clock()->get_clock_type());
  if (cloud_stamp.nanoseconds() != 0 &&
    (node_->now() - cloud_stamp).seconds() > obstacle_timeout_)
  {
    throw nav2_core::PlannerException("CMU obstacle point cloud is stale");
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      base_frame_, obstacle_cloud->header.frame_id, obstacle_cloud->header.stamp,
      rclcpp::Duration::from_seconds(transform_timeout_));
  } catch (const tf2::TransformException & error) {
    try {
      transform = tf_->lookupTransform(
        base_frame_, obstacle_cloud->header.frame_id, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException &) {
      throw nav2_core::PlannerException(
              "CMU cannot transform obstacle point cloud to " + base_frame_ + ": " + error.what());
    }
  }

  tf2::Transform tf_transform;
  tf2::fromMsg(transform.transform, tf_transform);
  std::vector<Point> obstacles;
  obstacles.reserve(static_cast<size_t>(max_obstacle_points_));
  std::unordered_set<std::uint64_t> occupied_voxels;
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*obstacle_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*obstacle_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*obstacle_cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const tf2::Vector3 local = tf_transform * tf2::Vector3(*iter_x, *iter_y, *iter_z);
      if (!std::isfinite(local.x()) || !std::isfinite(local.y()) || !std::isfinite(local.z())) {
        continue;
      }
      const double local_range = std::hypot(local.x(), local.y());
      if (local_range >= self_filter_radius_ && local_range <= adjacent_range_ &&
        local.z() >= min_obstacle_z_ && local.z() <= max_obstacle_z_)
      {
        const auto cell_x = static_cast<std::int32_t>(std::floor(local.x() / obstacle_voxel_size_));
        const auto cell_y = static_cast<std::int32_t>(std::floor(local.y() / obstacle_voxel_size_));
        const auto cell_z = static_cast<std::int32_t>(std::floor(local.z() / obstacle_voxel_size_));
        const std::uint64_t voxel_key =
          ((static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell_x)) & 0x1fffffU) << 42) |
          ((static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell_y)) & 0x1fffffU) << 21) |
          (static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell_z)) & 0x1fffffU);
        if (occupied_voxels.insert(voxel_key).second) {
          obstacles.push_back({local.x(), local.y(), local.z()});
          if (obstacles.size() >= static_cast<size_t>(max_obstacle_points_)) {
            break;
          }
        }
      }
    }
  } catch (const std::runtime_error & error) {
    throw nav2_core::PlannerException(
            "CMU cannot read obstacle PointCloud2: " + std::string(error.what()));
  }
  return obstacles;
}

CmuController::Point CmuController::globalPlanDirection(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  nav_msgs::msg::Path plan;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan = global_plan_;
  }
  if (plan.poses.empty()) {
    throw nav2_core::PlannerException("CMU controller has no global plan");
  }

  std::string plan_frame = plan.header.frame_id;
  if (plan_frame.empty()) {
    plan_frame = plan.poses.front().header.frame_id;
  }
  if (pose.header.frame_id.empty() || plan_frame.empty()) {
    throw nav2_core::PlannerException("CMU received a global plan or pose without a frame_id");
  }

  geometry_msgs::msg::TransformStamped plan_transform;
  const bool transform_plan = plan_frame != pose.header.frame_id;
  if (transform_plan) {
    try {
      plan_transform = tf_->lookupTransform(
        pose.header.frame_id, plan_frame, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & error) {
      throw nav2_core::PlannerException(
              "CMU cannot transform global plan to " + pose.header.frame_id + ": " + error.what());
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> local_plan;
  local_plan.reserve(plan.poses.size());
  for (const auto & plan_pose : plan.poses) {
    geometry_msgs::msg::PoseStamped local_pose = plan_pose;
    if (local_pose.header.frame_id.empty()) {
      local_pose.header.frame_id = plan_frame;
    }
    if (local_pose.header.frame_id != plan_frame) {
      throw nav2_core::PlannerException("CMU global plan contains mixed coordinate frames");
    }
    if (transform_plan) {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(local_pose, transformed_pose, plan_transform);
      local_pose = std::move(transformed_pose);
    }
    local_plan.push_back(std::move(local_pose));
  }

  const double robot_yaw = tf2::getYaw(pose.pose.orientation);
  size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < local_plan.size(); ++index) {
    const auto & point = local_plan[index].pose.position;
    const double distance = std::hypot(point.x - pose.pose.position.x, point.y - pose.pose.position.y);
    if (distance < nearest_distance) {
      nearest = index;
      nearest_distance = distance;
    }
  }

  const auto * target = &local_plan.back().pose.position;
  for (size_t index = nearest; index < local_plan.size(); ++index) {
    const auto & point = local_plan[index].pose.position;
    if (std::hypot(point.x - pose.pose.position.x, point.y - pose.pose.position.y) >= global_plan_lookahead_) {
      target = &point;
      break;
    }
  }
  const double dx = target->x - pose.pose.position.x;
  const double dy = target->y - pose.pose.position.y;
  return {
    std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy,
    -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy,
    0.0};
}

bool CmuController::isCollisionFree(
  const CandidatePath & path, const std::vector<Point> & obstacles) const
{
  const double radius = 0.5 * std::hypot(vehicle_length_, vehicle_width_) + footprint_padding_;
  constexpr size_t stride = 8;
  const double radius_squared = radius * radius;
  for (size_t path_index = 0; path_index < path.size(); path_index += stride) {
    const Point & sample = path[path_index];
    if (std::hypot(sample.x, sample.y) > adjacent_range_) {
      break;
    }
    for (const Point & obstacle : obstacles) {
      const double dx = sample.x - obstacle.x;
      const double dy = sample.y - obstacle.y;
      if (dx * dx + dy * dy <= radius_squared) {
        return false;
      }
    }
  }
  return true;
}

geometry_msgs::msg::TwistStamped CmuController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  const Point desired = globalPlanDirection(pose);
  const auto obstacles = obstacleCloudInBaseFrame();
  const double desired_heading = std::atan2(desired.y, desired.x);

  const CandidatePath * selected_path = nullptr;
  double selected_score = std::numeric_limits<double>::infinity();
  for (const CandidatePath & path : candidate_paths_) {
    if (path.empty() || !isCollisionFree(path, obstacles)) {
      continue;
    }
    const Point & endpoint = path.back();
    const double path_heading = std::atan2(endpoint.y, endpoint.x);
    const double heading_error = std::abs(normalizeAngle(path_heading - desired_heading));
    const double progress = std::hypot(endpoint.x, endpoint.y);
    const double score = heading_error - 0.02 * std::min(progress, adjacent_range_);
    if (score < selected_score) {
      selected_score = score;
      selected_path = &path;
    }
  }
  if (!selected_path) {
    throw nav2_core::PlannerException("CMU found no collision-free local trajectory");
  }

  const Point * lookahead = &selected_path->back();
  for (const Point & point : *selected_path) {
    if (std::hypot(point.x, point.y) >= lookahead_distance_) {
      lookahead = &point;
      break;
    }
  }
  const double heading_error = std::atan2(lookahead->y, lookahead->x);
  const double effective_limit = speed_limit_is_percentage_
    ? max_linear_speed_ * std::clamp(speed_limit_, 0.0, 100.0) / 100.0
    : std::min(max_linear_speed_, std::max(0.0, speed_limit_));

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_->now();
  command.header.frame_id = base_frame_;
  command.twist.angular.z = std::clamp(
    heading_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);
  const double alignment = std::max(0.0, std::cos(heading_error));
  command.twist.linear.x = effective_limit * alignment;

  if (debug_path_pub_) {
    nav_msgs::msg::Path debug_path;
    debug_path.header = command.header;
    debug_path.poses.reserve(selected_path->size());
    for (const Point & point : *selected_path) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = command.header;
      pose_msg.pose.position.x = point.x;
      pose_msg.pose.position.y = point.y;
      pose_msg.pose.orientation.w = 1.0;
      debug_path.poses.push_back(std::move(pose_msg));
    }
    debug_path_pub_->publish(debug_path);
  }
  return command;
}

}  // namespace robonix_nav2_terminal

PLUGINLIB_EXPORT_CLASS(robonix_nav2_terminal::CmuController, nav2_core::Controller)
