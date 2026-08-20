#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace robonix_nav2_terminal {

inline bool shouldStartGoal(bool have_goal, uint64_t observed_epoch,
                            uint64_t seen_epoch, bool geometry_changed) {
  if (!have_goal) {
    return true;
  }
  // Once an action UUID has been observed, it is the stable goal identity.
  // ControllerServer supplies the goal in the local costmap frame, so
  // map->odom localization corrections can move that pose while the actual
  // NavigateToPose goal remains unchanged.
  if (observed_epoch != 0) {
    return seen_epoch != 0 && observed_epoch != seen_epoch;
  }
  return geometry_changed;
}

class NavigateGoalEpoch {
public:
  void observe(const action_msgs::msg::GoalStatusArray &statuses) {
    const action_msgs::msg::GoalStatus *active = nullptr;
    for (const auto &status : statuses.status_list) {
      if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING) {
        if (active == nullptr || newerThan(status, *active)) {
          active = &status;
        }
      }
    }
    if (active == nullptr) {
      return;
    }
    std::string uuid;
    uuid.reserve(active->goal_info.goal_id.uuid.size() * 2);
    static constexpr char hex[] = "0123456789abcdef";
    for (const auto value : active->goal_info.goal_id.uuid) {
      uuid.push_back(hex[value >> 4]);
      uuid.push_back(hex[value & 0x0f]);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (uuid != active_uuid_) {
      active_uuid_ = std::move(uuid);
      epoch_.fetch_add(1, std::memory_order_release);
    }
  }

  uint64_t value() const { return epoch_.load(std::memory_order_acquire); }

private:
  static bool newerThan(const action_msgs::msg::GoalStatus &candidate,
                        const action_msgs::msg::GoalStatus &current) {
    const auto &a = candidate.goal_info.stamp;
    const auto &b = current.goal_info.stamp;
    if (a.sec != b.sec) {
      return a.sec > b.sec;
    }
    if (a.nanosec != b.nanosec) {
      return a.nanosec > b.nanosec;
    }
    const auto &auuid = candidate.goal_info.goal_id.uuid;
    const auto &buuid = current.goal_info.goal_id.uuid;
    return std::lexicographical_compare(buuid.begin(), buuid.end(),
                                        auuid.begin(), auuid.end());
  }

  mutable std::mutex mutex_;
  std::string active_uuid_;
  std::atomic<uint64_t> epoch_{0};
};

} // namespace robonix_nav2_terminal
