#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "robonix_nav2_terminal/unknown_path_budget.hpp"
#include "gtest/gtest.h"

namespace {

nav_msgs::msg::Path straightPath(double start_x, double end_x) {
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped start;
  start.pose.position.x = start_x;
  start.pose.position.y = 0.05;
  geometry_msgs::msg::PoseStamped end = start;
  end.pose.position.x = end_x;
  path.poses = {start, end};
  return path;
}

TEST(UnknownPathBudget, MeasuresSmallCoverageGap) {
  nav2_costmap_2d::Costmap2D costmap(20, 1, 0.1, 0.0, 0.0,
                                     nav2_costmap_2d::FREE_SPACE);
  costmap.setCost(5, 0, nav2_costmap_2d::NO_INFORMATION);
  costmap.setCost(6, 0, nav2_costmap_2d::NO_INFORMATION);

  const auto exposure = robonix_nav2_terminal::measureUnknownExposure(
      costmap, straightPath(0.05, 1.95));
  EXPECT_NEAR(exposure.path_length, 1.9, 1e-6);
  EXPECT_NEAR(exposure.unknown_length, 0.2, 0.03);
  EXPECT_NEAR(exposure.max_unknown_run, 0.2, 0.03);
  EXPECT_TRUE(
      robonix_nav2_terminal::withinUnknownBudget(exposure, 0.20, 0.25, 0.25));
}

TEST(UnknownPathBudget, RejectsHighUnknownRatio) {
  const robonix_nav2_terminal::UnknownExposure exposure{10.0, 0.6, 0.2};
  EXPECT_FALSE(
      robonix_nav2_terminal::withinUnknownBudget(exposure, 0.05, 0.75, 0.40));
}

TEST(UnknownPathBudget, RejectsLongContinuousGap) {
  const robonix_nav2_terminal::UnknownExposure exposure{20.0, 0.5, 0.5};
  EXPECT_FALSE(
      robonix_nav2_terminal::withinUnknownBudget(exposure, 0.05, 0.75, 0.40));
}

} // namespace
