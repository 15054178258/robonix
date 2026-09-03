#ifndef GO2_SENSORS__STRICT_JPEG_DECODER_HPP_
#define GO2_SENSORS__STRICT_JPEG_DECODER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace go2_sensors
{

struct StrictJpegImage
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  std::vector<std::uint8_t> bgr;
};

// Decode every scanline with libjpeg while treating decoder warnings as hard
// rejection. In particular, a payload that libjpeg reports as truncated,
// containing extraneous bytes, or having a marker mismatch is never returned.
bool decode_jpeg_strict(
  const std::uint8_t * data, std::size_t size,
  std::uint32_t max_width, std::uint32_t max_height,
  StrictJpegImage & output, std::string & error);

#ifdef GO2_SENSORS_STRICT_JPEG_TESTING
// Offline-only fault injection. These symbols do not exist in production
// builds and cannot affect the camera runtime.
void strict_jpeg_test_fail_after_allocation(bool enabled) noexcept;
std::size_t strict_jpeg_test_live_pixel_allocations() noexcept;
#endif

}  // namespace go2_sensors

#endif  // GO2_SENSORS__STRICT_JPEG_DECODER_HPP_
