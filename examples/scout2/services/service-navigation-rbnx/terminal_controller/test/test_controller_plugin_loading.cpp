#include "nav2_core/controller.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "gtest/gtest.h"

TEST(PluginLoadOrder, GoalCheckerDoesNotHideDwbControllerFactory) {
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_loader(
      "nav2_core", "nav2_core::GoalChecker");
  EXPECT_NO_THROW(goal_loader.createSharedInstance(
      "robonix_nav2_terminal::PersistentGoalChecker"));

  pluginlib::ClassLoader<nav2_core::Controller> controller_loader(
      "nav2_core", "nav2_core::Controller");
  EXPECT_NO_THROW(
      controller_loader.createSharedInstance("dwb_core::DWBLocalPlanner"));

  pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner_loader(
      "nav2_core", "nav2_core::GlobalPlanner");
  EXPECT_NO_THROW(planner_loader.createSharedInstance(
      "robonix_nav2_terminal::GoalAwareNavfnPlanner"));
}
