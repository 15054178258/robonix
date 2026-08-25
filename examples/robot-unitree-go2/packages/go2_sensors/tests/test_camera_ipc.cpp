#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <unistd.h>

#include "go2_sensors/camera_ipc_protocol.hpp"

namespace ipc = go2_sensors::camera_ipc;

void test_header_round_trip()
{
  ipc::FrameHeader input;
  input.sequence = 42U;
  input.capture_realtime_ns = 1234567890000000000ULL;
  input.capture_monotonic_ns = 987654321U;
  input.payload_bytes = 12345U;
  input.width_hint = 1280U;
  input.height_hint = 720U;
  input.api_request_count = 8U;
  input.api_accepted_count = 6U;
  input.source_rejected_count = 1U;
  input.api_error_count = 1U;
  input.ipc_connection_count = 3U;
  input.ipc_disconnect_count = 2U;
  input.last_api_code = 3104;
  const auto encoded = ipc::encode_header(input);
  assert(encoded.size() == ipc::kHeaderBytes);

  ipc::FrameHeader output;
  assert(ipc::decode_header(encoded, ipc::kDefaultMaxJpegBytes, output) == ipc::HeaderError::kOk);
  assert(output.sequence == input.sequence);
  assert(output.capture_realtime_ns == input.capture_realtime_ns);
  assert(output.capture_monotonic_ns == input.capture_monotonic_ns);
  assert(output.payload_bytes == input.payload_bytes);
  assert(output.width_hint == input.width_hint);
  assert(output.height_hint == input.height_hint);
  assert(output.api_request_count == input.api_request_count);
  assert(output.api_accepted_count == input.api_accepted_count);
  assert(output.source_rejected_count == input.source_rejected_count);
  assert(output.api_error_count == input.api_error_count);
  assert(output.ipc_connection_count == input.ipc_connection_count);
  assert(output.ipc_disconnect_count == input.ipc_disconnect_count);
  assert(output.last_api_code == input.last_api_code);
}

void test_header_rejections()
{
  ipc::FrameHeader header;
  header.sequence = 1U;
  header.capture_realtime_ns = 1U;
  header.capture_monotonic_ns = 1U;
  header.payload_bytes = 64U;
  const auto valid = ipc::encode_header(header);
  ipc::FrameHeader output;

  auto modified = valid;
  modified[0] ^= 0xffU;
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kBadMagic);
  modified = valid;
  modified[4] = 3U;
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kBadVersion);
  modified = valid;
  ipc::put_u16(modified.data() + 6U, 47U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kBadHeaderSize);
  modified = valid;
  ipc::put_u32(modified.data() + 32U, 0U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kEmptyPayload);
  modified = valid;
  ipc::put_u32(modified.data() + 32U, 1025U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kPayloadTooLarge);
  modified = valid;
  ipc::put_u64(modified.data() + 16U, 0U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kMissingRealtimeStamp);
  modified = valid;
  ipc::put_u64(modified.data() + 24U, 0U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kMissingMonotonicStamp);
  modified = valid;
  ipc::put_u32(modified.data() + 36U, 1U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kUnexpectedPayload);
  modified = valid;
  ipc::put_u32(modified.data() + 36U, 2U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kReservedBitsSet);
  modified = valid;
  ipc::put_u32(modified.data() + 44U, 1U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kReservedBitsSet);

  modified = valid;
  ipc::put_u64(modified.data() + 48U, 1U);
  assert(ipc::decode_header(modified, 1024U, output) == ipc::HeaderError::kInconsistentCounters);
}

void test_status_record()
{
  ipc::FrameHeader input;
  input.sequence = 4U;
  input.capture_realtime_ns = 5U;
  input.capture_monotonic_ns = 6U;
  input.flags = ipc::kStatusOnly;
  input.api_request_count = 10U;
  input.api_accepted_count = 7U;
  input.source_rejected_count = 2U;
  input.api_error_count = 1U;
  input.last_api_code = 3104;
  const auto encoded = ipc::encode_header(input);
  ipc::FrameHeader output;
  assert(ipc::decode_header(encoded, 1024U, output) == ipc::HeaderError::kOk);
  assert(output.payload_bytes == 0U);
  assert(output.flags == ipc::kStatusOnly);
  assert(output.last_api_code == 3104);
}

void test_jpeg_boundaries()
{
  const std::array<std::uint8_t, 6> valid{0xffU, 0xd8U, 1U, 2U, 0xffU, 0xd9U};
  const std::array<std::uint8_t, 6> invalid{0xffU, 0xd8U, 1U, 2U, 0U, 0U};
  assert(ipc::looks_like_jpeg(valid.data(), valid.size()));
  assert(!ipc::looks_like_jpeg(invalid.data(), invalid.size()));
  assert(!ipc::looks_like_jpeg(nullptr, 0U));
}

void test_jpeg_dimensions()
{
  const std::array<std::uint8_t, 17> jpeg{
    0xffU, 0xd8U,
    0xffU, 0xc0U, 0x00U, 0x0bU, 0x08U, 0x02U, 0xd0U, 0x05U, 0x00U,
    0x01U, 0x01U, 0x11U, 0x00U,
    0xffU, 0xd9U};
  std::uint16_t width = 0U;
  std::uint16_t height = 0U;
  assert(ipc::jpeg_dimensions(jpeg.data(), jpeg.size(), width, height));
  assert(width == 1280U);
  assert(height == 720U);

  auto invalid = jpeg;
  invalid[5] = 0xffU;
  assert(!ipc::jpeg_dimensions(invalid.data(), invalid.size(), width, height));
}

void test_complete_jpeg_structure()
{
  const std::array<std::uint8_t, 28> jpeg{
    0xffU, 0xd8U,
    0xffU, 0xc0U, 0x00U, 0x0bU, 0x08U, 0x02U, 0xd0U, 0x05U, 0x00U,
    0x01U, 0x01U, 0x11U, 0x00U,
    0xffU, 0xdaU, 0x00U, 0x08U, 0x01U, 0x01U, 0x00U, 0x00U, 0x3fU, 0x00U,
    0x01U, 0xffU, 0xd9U};
  std::uint16_t width = 0U;
  std::uint16_t height = 0U;
  assert(ipc::jpeg_structure_is_valid(jpeg.data(), jpeg.size(), width, height));
  assert(width == 1280U);
  assert(height == 720U);
  auto invalid = jpeg;
  invalid[26] = 0x01U;
  assert(!ipc::jpeg_structure_is_valid(invalid.data(), invalid.size(), width, height));
}

void test_stream_framing()
{
  int sockets[2]{-1, -1};
  assert(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
  const std::vector<std::uint8_t> jpeg{0xffU, 0xd8U, 1U, 2U, 3U, 0xffU, 0xd9U};
  ipc::FrameHeader header;
  header.sequence = 7U;
  header.capture_realtime_ns = 123U;
  header.capture_monotonic_ns = 456U;
  header.payload_bytes = static_cast<std::uint32_t>(jpeg.size());
  const auto encoded = ipc::encode_header(header);

  std::thread writer([&]() {
    std::string error;
    if (!ipc::write_exact(sockets[0], encoded.data(), encoded.size(), 1000, error)) {
      std::cerr << "header write failed: " << error << "\n";
      std::abort();
    }
    if (!ipc::write_exact(sockets[0], jpeg.data(), jpeg.size(), 1000, error)) {
      std::cerr << "payload write failed: " << error << "\n";
      std::abort();
    }
  });
  std::array<std::uint8_t, ipc::kHeaderBytes> received_header{};
  std::string error;
  assert(ipc::read_exact(sockets[1], received_header.data(), received_header.size(), 1000, error));
  ipc::FrameHeader decoded;
  assert(ipc::decode_header(received_header, 1024U, decoded) == ipc::HeaderError::kOk);
  std::vector<std::uint8_t> received_payload(decoded.payload_bytes);
  assert(ipc::read_exact(sockets[1], received_payload.data(), received_payload.size(), 1000, error));
  assert(received_payload == jpeg);
  writer.join();
  ::close(sockets[0]);
  ::close(sockets[1]);
}

void test_read_timeout()
{
  int sockets[2]{-1, -1};
  assert(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
  std::uint8_t byte = 0U;
  std::string error;
  assert(!ipc::read_exact(sockets[0], &byte, 1U, 10, error));
  assert(error == "socket timeout");
  ::close(sockets[0]);
  ::close(sockets[1]);
}

int main()
{
  test_header_round_trip();
  test_header_rejections();
  test_status_record();
  test_jpeg_boundaries();
  test_jpeg_dimensions();
  test_complete_jpeg_structure();
  test_stream_framing();
  test_read_timeout();
  std::cout << "camera IPC protocol tests passed\n";
  return 0;
}
