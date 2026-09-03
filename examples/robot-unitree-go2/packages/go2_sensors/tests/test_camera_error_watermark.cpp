#include <cassert>
#include <iostream>

#include "go2_sensors/camera_error_watermark.hpp"

namespace
{

void test_connection_errors_require_an_explicit_reconnect()
{
  go2_sensors::CameraErrorWatermark error("camera daemon not connected");
  assert(!error.empty());
  assert(!error.clear_if_recovered_by(1U, 10U));
  assert(error.message() == "camera daemon not connected");
  error.clear();
  assert(error.empty());
}

void test_same_or_newer_stream_watermark_recovers()
{
  go2_sensors::CameraErrorWatermark error;
  assert(error.record_stream_error("opaque API error", 3U, 20U));
  assert(!error.clear_if_recovered_by(2U, 21U));
  assert(!error.clear_if_recovered_by(3U, 19U));
  assert(error.clear_if_recovered_by(3U, 20U));
  assert(error.empty());

  assert(error.record_stream_error("old-generation stream error", 3U, 30U));
  assert(error.clear_if_recovered_by(4U, 1U));
  assert(error.empty());
}

void test_older_frame_cannot_erase_newer_reader_error()
{
  go2_sensors::CameraErrorWatermark error;
  assert(error.record_stream_error("newer reader error", 7U, 42U));
  assert(!error.record_stream_error("older processor error", 7U, 41U));
  assert(!error.clear_if_recovered_by(7U, 41U));
  assert(error.message() == "newer reader error");
  error.set_connection("socket disconnected");
  assert(!error.record_stream_error("late old-generation rejection", 7U, 100U));
  assert(!error.clear_if_recovered_by(7U, 100U));
  assert(error.message() == "socket disconnected");
}

}  // namespace

int main()
{
  test_connection_errors_require_an_explicit_reconnect();
  test_same_or_newer_stream_watermark_recovers();
  test_older_frame_cannot_erase_newer_reader_error();
  std::cout << "camera error watermark tests passed\n";
  return 0;
}
