#ifndef GO2_SENSORS__CAMERA_ERROR_WATERMARK_HPP_
#define GO2_SENSORS__CAMERA_ERROR_WATERMARK_HPP_

#include <cstdint>
#include <string>
#include <utility>

namespace go2_sensors
{

// Tracks whether an error belongs to the connection itself or to a particular
// IPC record.  A successfully published frame may clear only a stream-record
// error at or before its ordered (connection generation, record sequence)
// watermark; it must never erase a newer reader-side or connection error
// observed while decode/publication was in flight.
class CameraErrorWatermark final
{
public:
  enum class Scope
  {
    kNone,
    kConnection,
    kStreamRecord,
  };

  explicit CameraErrorWatermark(std::string initial_connection_error = {})
  : message_(std::move(initial_connection_error)),
    scope_(message_.empty() ? Scope::kNone : Scope::kConnection)
  {}

  void set_connection(std::string message)
  {
    message_ = std::move(message);
    scope_ = message_.empty() ? Scope::kNone : Scope::kConnection;
    connection_generation_ = 0U;
    record_sequence_ = 0U;
  }

  bool record_stream_error(
    std::string message, const std::uint64_t connection_generation,
    const std::uint64_t record_sequence)
  {
    if (message.empty() || scope_ == Scope::kConnection ||
      (scope_ == Scope::kStreamRecord &&
      (connection_generation < connection_generation_ ||
      (connection_generation == connection_generation_ &&
      record_sequence <= record_sequence_))))
    {
      return false;
    }
    message_ = std::move(message);
    scope_ = Scope::kStreamRecord;
    connection_generation_ = connection_generation;
    record_sequence_ = record_sequence;
    return true;
  }

  void clear()
  {
    message_.clear();
    scope_ = Scope::kNone;
    connection_generation_ = 0U;
    record_sequence_ = 0U;
  }

  bool clear_if_recovered_by(
    const std::uint64_t connection_generation, const std::uint64_t record_sequence)
  {
    if (message_.empty() || scope_ != Scope::kStreamRecord ||
      connection_generation < connection_generation_ ||
      (connection_generation == connection_generation_ &&
      record_sequence < record_sequence_))
    {
      return false;
    }
    clear();
    return true;
  }

  bool empty() const noexcept {return message_.empty();}
  const std::string & message() const noexcept {return message_;}

private:
  std::string message_;
  Scope scope_{Scope::kNone};
  std::uint64_t connection_generation_{0U};
  std::uint64_t record_sequence_{0U};
};

}  // namespace go2_sensors

#endif  // GO2_SENSORS__CAMERA_ERROR_WATERMARK_HPP_
