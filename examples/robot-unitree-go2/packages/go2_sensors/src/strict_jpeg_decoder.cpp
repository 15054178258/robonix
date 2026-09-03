#include "go2_sensors/strict_jpeg_decoder.hpp"

#include <atomic>
#include <csetjmp>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>

extern "C"
{
#include <jerror.h>
#include <jpeglib.h>
}

#include "go2_sensors/camera_ipc_protocol.hpp"

namespace go2_sensors
{
namespace
{

struct JpegErrorContext
{
  jpeg_error_mgr manager{};
  std::jmp_buf jump_buffer{};
  int warning_count{0};
  char message[JMSG_LENGTH_MAX]{};
};

// Every value changed after setjmp lives in this heap object. The sole local
// pointer to it is assigned before setjmp and never changed, so the fatal
// longjmp path does not inspect an indeterminate automatic decoder or pixel
// pointer (C/C++ setjmp rule).
struct DecoderSession
{
  jpeg_decompress_struct decoder{};
  JpegErrorContext error{};
  unsigned char * pixels{nullptr};
  bool decoder_created{false};
};

#ifdef GO2_SENSORS_STRICT_JPEG_TESTING
std::atomic<bool> fail_after_allocation{false};
std::atomic<std::size_t> live_pixel_allocations{0U};
#endif

void format_decoder_message(j_common_ptr decoder, JpegErrorContext & context) noexcept
{
  if (decoder != nullptr && decoder->err != nullptr &&
    decoder->err->format_message != nullptr)
  {
    decoder->err->format_message(decoder, context.message);
  }
}

[[noreturn]] void decoder_error_exit(j_common_ptr decoder)
{
  auto * context = reinterpret_cast<JpegErrorContext *>(decoder->err);
  format_decoder_message(decoder, *context);
  std::longjmp(context->jump_buffer, 1);
}

void decoder_emit_message(j_common_ptr decoder, const int message_level)
{
  if (message_level < 0) {
    auto * context = reinterpret_cast<JpegErrorContext *>(decoder->err);
    context->warning_count += 1;
    format_decoder_message(decoder, *context);
  }
}

void release_pixels(DecoderSession * const session) noexcept
{
  if (session->pixels != nullptr) {
    std::free(session->pixels);
    session->pixels = nullptr;
#ifdef GO2_SENSORS_STRICT_JPEG_TESTING
    live_pixel_allocations.fetch_sub(1U);
#endif
  }
}

void destroy_session(DecoderSession * const session) noexcept
{
  if (session == nullptr) {
    return;
  }
  release_pixels(session);
  if (session->decoder_created) {
    // Clear first: if a broken decoder unexpectedly invokes error_exit during
    // destruction, the re-entered cleanup must not destroy it twice.
    session->decoder_created = false;
    jpeg_destroy_decompress(&session->decoder);
  }
  delete session;
}

bool reject(DecoderSession * const session, std::string & error, const char * const message)
{
  destroy_session(session);
  error = message;
  return false;
}

}  // namespace

#ifdef GO2_SENSORS_STRICT_JPEG_TESTING
void strict_jpeg_test_fail_after_allocation(const bool enabled) noexcept
{
  fail_after_allocation.store(enabled);
}

std::size_t strict_jpeg_test_live_pixel_allocations() noexcept
{
  return live_pixel_allocations.load();
}
#endif

bool decode_jpeg_strict(
  const std::uint8_t * const data, const std::size_t size,
  const std::uint32_t max_width, const std::uint32_t max_height,
  StrictJpegImage & output, std::string & error)
{
  output = StrictJpegImage{};
  error.clear();
  if (data == nullptr || size == 0U ||
    size > static_cast<std::size_t>(std::numeric_limits<unsigned long>::max()))
  {
    error = "JPEG input is empty or exceeds decoder size limits";
    return false;
  }
  std::uint16_t marker_width = 0U;
  std::uint16_t marker_height = 0U;
  if (!camera_ipc::jpeg_structure_is_valid(data, size, marker_width, marker_height)) {
    error = "JPEG marker structure is invalid";
    return false;
  }

  DecoderSession * const session = new (std::nothrow) DecoderSession();
  if (session == nullptr) {
    error = "could not allocate JPEG decoder session";
    return false;
  }
  session->decoder.err = jpeg_std_error(&session->error.manager);
  session->error.manager.error_exit = decoder_error_exit;
  session->error.manager.emit_message = decoder_emit_message;

  if (setjmp(session->error.jump_buffer) != 0) {
    char decoder_message[JMSG_LENGTH_MAX]{};
    std::memcpy(decoder_message, session->error.message, sizeof(decoder_message));
    destroy_session(session);
    error = decoder_message[0] == '\0' ?
      "JPEG decoder reported a fatal error" : decoder_message;
    return false;
  }

  jpeg_create_decompress(&session->decoder);
  session->decoder_created = true;
  jpeg_mem_src(
    &session->decoder, const_cast<unsigned char *>(data),
    static_cast<unsigned long>(size));
  if (jpeg_read_header(&session->decoder, TRUE) != JPEG_HEADER_OK) {
    return reject(session, error, "JPEG header was not accepted by the decoder");
  }
  if (session->decoder.image_width == 0U || session->decoder.image_height == 0U ||
    session->decoder.image_width > max_width || session->decoder.image_height > max_height ||
    session->decoder.image_width != marker_width || session->decoder.image_height != marker_height)
  {
    return reject(session, error, "JPEG dimensions are inconsistent or exceed configured limits");
  }

  session->decoder.out_color_space = JCS_RGB;
  if (jpeg_start_decompress(&session->decoder) == FALSE ||
    session->decoder.output_components != 3 ||
    session->decoder.output_width != session->decoder.image_width ||
    session->decoder.output_height != session->decoder.image_height)
  {
    return reject(session, error, "JPEG decoder produced an unsupported output layout");
  }
  const std::size_t width = static_cast<std::size_t>(session->decoder.output_width);
  const std::size_t height = static_cast<std::size_t>(session->decoder.output_height);
  if (width > std::numeric_limits<std::size_t>::max() / 3U ||
    height > std::numeric_limits<std::size_t>::max() / (width * 3U))
  {
    return reject(session, error, "decoded JPEG size overflows addressable memory");
  }
  const std::size_t row_bytes = width * 3U;
  const std::size_t pixel_bytes = row_bytes * height;
  session->pixels = static_cast<unsigned char *>(std::malloc(pixel_bytes));
  if (session->pixels == nullptr) {
    return reject(session, error, "could not allocate bounded JPEG decode buffer");
  }
#ifdef GO2_SENSORS_STRICT_JPEG_TESTING
  live_pixel_allocations.fetch_add(1U);
  if (fail_after_allocation.exchange(false)) {
    ERREXIT(&session->decoder, JERR_FILE_READ);
  }
#endif

  while (session->decoder.output_scanline < session->decoder.output_height) {
    JSAMPROW row = session->pixels +
      static_cast<std::size_t>(session->decoder.output_scanline) * row_bytes;
    if (jpeg_read_scanlines(&session->decoder, &row, 1U) != 1U) {
      return reject(session, error, "JPEG decoder did not return every scanline");
    }
  }
  if (jpeg_finish_decompress(&session->decoder) == FALSE) {
    return reject(session, error, "JPEG decoder did not finish the image");
  }
  session->decoder_created = false;
  jpeg_destroy_decompress(&session->decoder);

  if (session->error.warning_count != 0) {
    error = "JPEG decoder rejected warning(s)=" +
      std::to_string(session->error.warning_count);
    if (session->error.message[0] != '\0') {
      error += ": ";
      error += session->error.message;
    }
    destroy_session(session);
    return false;
  }

  try {
    output.bgr.resize(pixel_bytes);
  } catch (const std::bad_alloc &) {
    destroy_session(session);
    error = "could not allocate decoded camera image";
    return false;
  }
  for (std::size_t index = 0U; index < pixel_bytes; index += 3U) {
    output.bgr[index] = session->pixels[index + 2U];
    output.bgr[index + 1U] = session->pixels[index + 1U];
    output.bgr[index + 2U] = session->pixels[index];
  }
  destroy_session(session);
  output.width = static_cast<std::uint32_t>(width);
  output.height = static_cast<std::uint32_t>(height);
  return true;
}

}  // namespace go2_sensors
