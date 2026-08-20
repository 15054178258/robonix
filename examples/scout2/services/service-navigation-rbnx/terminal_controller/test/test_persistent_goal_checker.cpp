#include <memory>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robonix_nav2_terminal/navigate_goal_epoch.hpp"
#include "robonix_nav2_terminal/persistent_goal_checker.hpp"
#include "robonix_nav2_terminal/persistent_rotate_to_goal_critic.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "gtest/gtest.h"

namespace {

geometry_msgs::msg::Pose pose(double x, double y, double yaw) {
  geometry_msgs::msg::Pose value;
  value.position.x = x;
  value.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  value.orientation = tf2::toMsg(q);
  return value;
}

action_msgs::msg::GoalStatus goalStatus(uint8_t id, int32_t stamp,
                                        int8_t state) {
  action_msgs::msg::GoalStatus value;
  value.goal_info.goal_id.uuid[0] = id;
  value.goal_info.stamp.sec = stamp;
  value.status = state;
  return value;
}

TEST(NavigateGoalEpoch, NewestActiveGoalWinsRegardlessOfArrayOrder) {
  robonix_nav2_terminal::NavigateGoalEpoch epoch;
  const auto old_canceling =
      goalStatus(1, 10, action_msgs::msg::GoalStatus::STATUS_CANCELING);
  const auto new_executing =
      goalStatus(2, 20, action_msgs::msg::GoalStatus::STATUS_EXECUTING);

  action_msgs::msg::GoalStatusArray newest_first;
  newest_first.status_list = {new_executing, old_canceling};
  epoch.observe(newest_first);
  EXPECT_EQ(epoch.value(), 1u);

  action_msgs::msg::GoalStatusArray oldest_first;
  oldest_first.status_list = {old_canceling, new_executing};
  epoch.observe(oldest_first);
  EXPECT_EQ(epoch.value(), 1u);
}

TEST(NavigateGoalIdentity, SameUuidIgnoresLocalFrameGoalMotion) {
  using robonix_nav2_terminal::shouldStartGoal;
  EXPECT_TRUE(shouldStartGoal(false, 0, 0, false));
  EXPECT_TRUE(shouldStartGoal(true, 0, 0, true));

  // Adopt the first action status without resetting an already-started goal.
  EXPECT_FALSE(shouldStartGoal(true, 1, 0, true));
  // RTAB-Map map->odom corrections must not reset a stable action UUID.
  EXPECT_FALSE(shouldStartGoal(true, 1, 1, true));
  EXPECT_TRUE(shouldStartGoal(true, 2, 1, false));
}

TEST(PersistentGoalChecker, SameGoalReplanDoesNotClearXyLatch) {
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_checker_test");
  robonix_nav2_terminal::PersistentGoalChecker checker;
  checker.initialize(node, "goal_checker", nullptr);
  const auto goal = pose(0.0, 0.0, 0.0);
  geometry_msgs::msg::Twist stopped;

  EXPECT_FALSE(checker.isGoalReached(pose(0.20, 0.0, 0.5), goal, stopped));
  checker.reset(); // Simulates ControllerServer receiving a 5 Hz replan.
  EXPECT_TRUE(checker.isGoalReached(pose(0.40, 0.0, 0.0), goal, stopped));
}

TEST(PersistentGoalChecker, NewGoalClearsLatch) {
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("new_goal_test");
  robonix_nav2_terminal::PersistentGoalChecker checker;
  checker.initialize(node, "goal_checker", nullptr);
  geometry_msgs::msg::Twist stopped;
  EXPECT_TRUE(
      checker.isGoalReached(pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0), stopped));
  EXPECT_FALSE(
      checker.isGoalReached(pose(0.0, 0.0, 0.0), pose(2.0, 0.0, 0.0), stopped));
}

TEST(PersistentRotateToGoalCritic, ReplanDoesNotReenableTranslation) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("critic_test");
  robonix_nav2_terminal::PersistentRotateToGoalCritic critic;
  critic.initialize(node, "PersistentRotateToGoal", "FollowPath", nullptr);

  geometry_msgs::msg::Pose2D current;
  current.x = 0.20;
  geometry_msgs::msg::Pose2D goal;
  goal.theta = 1.0;
  nav_2d_msgs::msg::Twist2D stopped;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(current, stopped, goal, path));

  critic.reset();   // DWB setPlan() during a same-goal replan.
  current.x = 0.40; // Wheel slip moved XY outside the enter window.
  current.theta = 0.10;
  ASSERT_TRUE(critic.prepare(current, stopped, goal, path));

  dwb_msgs::msg::Trajectory2D translating;
  translating.velocity.x = 0.10;
  translating.poses.push_back(current);
  EXPECT_THROW(critic.scoreTrajectory(translating),
               dwb_core::IllegalTrajectoryException);
}

TEST(PersistentRotateToGoalCritic,
     YawLatchSelectsBrakingInsteadOfRejectingAll) {
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("critic_braking_test");
  robonix_nav2_terminal::PersistentRotateToGoalCritic critic;
  critic.initialize(node, "PersistentRotateToGoal", "FollowPath", nullptr);

  geometry_msgs::msg::Pose2D current;
  current.x = 0.10;
  current.theta = 0.0;
  geometry_msgs::msg::Pose2D goal;
  goal.theta = 0.05;
  nav_2d_msgs::msg::Twist2D velocity;
  velocity.theta = 0.10;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(current, velocity, goal, path));

  dwb_msgs::msg::Trajectory2D slow;
  slow.velocity.theta = 0.02;
  slow.poses.push_back(current);
  dwb_msgs::msg::Trajectory2D fast = slow;
  fast.velocity.theta = 0.10;

  EXPECT_NO_THROW(critic.scoreTrajectory(slow));
  EXPECT_LT(critic.scoreTrajectory(slow), critic.scoreTrajectory(fast));

  dwb_msgs::msg::Trajectory2D translating = slow;
  translating.velocity.x = 0.01;
  EXPECT_THROW(critic.scoreTrajectory(translating),
               dwb_core::IllegalTrajectoryException);
}

TEST(PersistentRotateToGoalCritic, RejectsRotationAwayFromGoalYaw) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "critic_direction_test");
  node->declare_parameter<double>(
      "FollowPath.PersistentRotateToGoal.lookahead_time", -1.0);
  robonix_nav2_terminal::PersistentRotateToGoalCritic critic;
  critic.initialize(node, "PersistentRotateToGoal", "FollowPath", nullptr);

  geometry_msgs::msg::Pose2D current;
  current.x = 0.10;
  current.theta = 0.0;
  geometry_msgs::msg::Pose2D goal;
  goal.theta = 1.0;
  nav_2d_msgs::msg::Twist2D stopped;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(current, stopped, goal, path));

  dwb_msgs::msg::Trajectory2D toward_goal;
  toward_goal.velocity.theta = 0.20;
  toward_goal.poses.push_back(current);
  EXPECT_NO_THROW(critic.scoreTrajectory(toward_goal));

  dwb_msgs::msg::Trajectory2D away_from_goal = toward_goal;
  away_from_goal.velocity.theta = -0.20;
  EXPECT_THROW(critic.scoreTrajectory(away_from_goal),
               dwb_core::IllegalTrajectoryException);
}

TEST(PluginRegistration, LoadsBothTerminalPlugins) {
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_loader(
      "nav2_core", "nav2_core::GoalChecker");
  EXPECT_NO_THROW(goal_loader.createSharedInstance(
      "robonix_nav2_terminal::PersistentGoalChecker"));

  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> critic_loader(
      "dwb_core", "dwb_core::TrajectoryCritic");
  EXPECT_NO_THROW(critic_loader.createSharedInstance(
      "robonix_nav2_terminal::PersistentRotateToGoalCritic"));
}

TEST(PluginRegistration, LoadsSystemDwbControllerAlongsideTerminalOverlay) {
  pluginlib::ClassLoader<nav2_core::Controller> controller_loader(
      "nav2_core", "nav2_core::Controller");
  EXPECT_NO_THROW(
      controller_loader.createSharedInstance("dwb_core::DWBLocalPlanner"));
}

} // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
