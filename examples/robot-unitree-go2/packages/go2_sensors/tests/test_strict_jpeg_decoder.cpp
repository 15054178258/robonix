#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

extern "C"
{
#include <jpeglib.h>
}

#include "go2_sensors/strict_jpeg_decoder.hpp"

std::vector<std::uint8_t> make_jpeg()
{
  jpeg_compress_struct compressor{};
  jpeg_error_mgr errors{};
  compressor.err = jpeg_std_error(&errors);
  jpeg_create_compress(&compressor);
  unsigned char * encoded = nullptr;
  unsigned long encoded_size = 0UL;
  jpeg_mem_dest(&compressor, &encoded, &encoded_size);
  compressor.image_width = 4U;
  compressor.image_height = 4U;
  compressor.input_components = 3;
  compressor.in_color_space = JCS_RGB;
  jpeg_set_defaults(&compressor);
  jpeg_set_quality(&compressor, 85, TRUE);
  jpeg_start_compress(&compressor, TRUE);
  std::vector<std::uint8_t> pixels(4U * 4U * 3U, 127U);
  while (compressor.next_scanline < compressor.image_height) {
    JSAMPROW row = pixels.data() +
      static_cast<std::size_t>(compressor.next_scanline) * 4U * 3U;
    assert(jpeg_write_scanlines(&compressor, &row, 1U) == 1U);
  }
  jpeg_finish_compress(&compressor);
  std::vector<std::uint8_t> result(encoded, encoded + encoded_size);
  std::free(encoded);
  jpeg_destroy_compress(&compressor);
  return result;
}

int main()
{
  const auto valid = make_jpeg();
  go2_sensors::StrictJpegImage decoded;
  std::string error;
  assert(go2_sensors::decode_jpeg_strict(
    valid.data(), valid.size(), 64U, 64U, decoded, error));
  assert(decoded.width == 4U);
  assert(decoded.height == 4U);
  assert(decoded.bgr.size() == 4U * 4U * 3U);

#ifdef GO2_SENSORS_STRICT_JPEG_TESTING
  go2_sensors::strict_jpeg_test_fail_after_allocation(true);
  assert(!go2_sensors::decode_jpeg_strict(
    valid.data(), valid.size(), 64U, 64U, decoded, error));
  assert(!error.empty());
  assert(go2_sensors::strict_jpeg_test_live_pixel_allocations() == 0U);
  assert(decoded.width == 0U && decoded.height == 0U && decoded.bgr.empty());
#endif

  auto truncated = valid;
  assert(truncated.size() > 4U);
  truncated.erase(truncated.end() - 4, truncated.end() - 2);
  assert(!go2_sensors::decode_jpeg_strict(
    truncated.data(), truncated.size(), 64U, 64U, decoded, error));

  assert(!go2_sensors::decode_jpeg_strict(
    valid.data(), valid.size(), 2U, 2U, decoded, error));
#ifdef GO2_SENSORS_STRICT_JPEG_TESTING
  assert(go2_sensors::strict_jpeg_test_live_pixel_allocations() == 0U);
#endif
  std::cout << "strict JPEG decoder tests passed\n";
  return 0;
}
