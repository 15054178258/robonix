#ifndef GO2_SENSORS__LATEST_FRAME_MAILBOX_HPP_
#define GO2_SENSORS__LATEST_FRAME_MAILBOX_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "go2_sensors/camera_ipc_protocol.hpp"

namespace go2_sensors
{

// A single-slot handoff between the IPC reader and the comparatively expensive
// JPEG/ROS publisher path.  Keeping the socket reader independent prevents a
// stream socket and its peer from becoming an implicit FIFO of stale frames.
// When processing cannot keep up, only the newest complete, protocol-validated
// record remains pending.
struct CameraFrameRecord
{
  camera_ipc::FrameHeader header;
  std::vector<std::uint8_t> jpeg;
  std::uint64_t connection_generation{0U};
};

class LatestFrameMailbox final
{
public:
  struct PutResult
  {
    bool accepted{false};
    bool replaced{false};
  };

  LatestFrameMailbox() = default;
  LatestFrameMailbox(const LatestFrameMailbox &) = delete;
  LatestFrameMailbox & operator=(const LatestFrameMailbox &) = delete;

  PutResult put(CameraFrameRecord record)
  {
    PutResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) {
        return result;
      }
      result.accepted = true;
      result.replaced = pending_.has_value();
      pending_.emplace(std::move(record));
    }
    condition_.notify_one();
    return result;
  }

  std::optional<CameraFrameRecord> wait_take()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() {return closed_ || pending_.has_value();});
    if (!pending_.has_value()) {
      return std::nullopt;
    }
    std::optional<CameraFrameRecord> result(std::move(pending_));
    pending_.reset();
    return result;
  }

  bool discard_generation(const std::uint64_t connection_generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.has_value() ||
      pending_->connection_generation != connection_generation)
    {
      return false;
    }
    pending_.reset();
    return true;
  }

  std::size_t depth() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.has_value() ? 1U : 0U;
  }

  bool close()
  {
    bool discarded = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
      discarded = pending_.has_value();
      pending_.reset();
    }
    condition_.notify_all();
    return discarded;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<CameraFrameRecord> pending_;
  bool closed_{false};
};

}  // namespace go2_sensors

#endif  // GO2_SENSORS__LATEST_FRAME_MAILBOX_HPP_
