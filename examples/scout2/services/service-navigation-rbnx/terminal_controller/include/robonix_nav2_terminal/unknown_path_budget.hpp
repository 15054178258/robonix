#pragma once

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robonix_nav2_terminal {

struct UnknownExposure {
  double path_length{0.0};
  double unknown_length{0.0};
  double max_unknown_run{0.0};
};

UnknownExposure
measureUnknownExposure(const nav2_costmap_2d::Costmap2D &costmap,
                       const nav_msgs::msg::Path &path);

bool withinUnknownBudget(const UnknownExposure &exposure,
                         double max_unknown_ratio, double max_unknown_length,
                         double max_unknown_run);

} // namespace robonix_nav2_terminal
