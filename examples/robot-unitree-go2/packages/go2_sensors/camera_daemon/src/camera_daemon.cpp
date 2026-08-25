#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "go2_sensors/camera_ipc_protocol.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/go2/video/video_client.hpp"

namespace
{

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(const int /*signal*/) noexcept
{
  g_stop_requested = 1;
}

struct Options
{
  std::string interface;
  std::string socket_path{go2_sensors::camera_ipc::default_socket_path()};
  int domain_id{0};
  int api_timeout_ms{1000};
  int socket_timeout_ms{1500};
  double frames_per_second{10.0};
  std::uint32_t max_jpeg_bytes{go2_sensors::camera_ipc::kDefaultMaxJpegBytes};
};

struct DaemonCounters
{
  std::uint64_t api_requests{0U};
  std::uint64_t api_accepted{0U};
  std::uint64_t source_rejected{0U};
  std::uint64_t api_errors{0U};
  std::uint64_t ipc_connections{0U};
  std::uint64_t ipc_disconnects{0U};
  std::int32_t last_api_code{0};
};

class ChannelReleaseGuard final
{
public:
  ChannelReleaseGuard() = default;
  ChannelReleaseGuard(const ChannelReleaseGuard &) = delete;
  ChannelReleaseGuard & operator=(const ChannelReleaseGuard &) = delete;

  ~ChannelReleaseGuard()
  {
    unitree::robot::ChannelFactory::Instance()->Release();
  }
};

[[noreturn]] void usage_error(const std::string & message)
{
  throw std::invalid_argument(
          message +
          "\nUsage: go2_camera_daemon --interface IFACE [--socket PATH] "
          "[--domain-id 0] [--fps 10] [--api-timeout-ms 1000] "
          "[--socket-timeout-ms 1500] [--max-jpeg-bytes 4194304]");
}

std::string require_value(const int argc, char ** argv, int & index)
{
  if (index + 1 >= argc) {
    usage_error(std::string("missing value for ") + argv[index]);
  }
  ++index;
  return argv[index];
}

long long parse_integer(const std::string & text, const std::string & name)
{
  std::size_t consumed = 0U;
  long long value = 0;
  try {
    value = std::stoll(text, &consumed, 10);
  } catch (const std::exception &) {
    usage_error(name + " must be an integer");
  }
  if (consumed != text.size()) {
    usage_error(name + " must be an integer");
  }
  return value;
}

double parse_floating_point(const std::string & text, const std::string & name)
{
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception &) {
    usage_error(name + " must be numeric");
  }
  if (consumed != text.size() || !std::isfinite(value)) {
    usage_error(name + " must be a finite number");
  }
  return value;
}

Options parse_options(const int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--help" || option == "-h") {
      std::cout <<
        "READ-ONLY Go2 camera daemon\n"
        "Usage: go2_camera_daemon --interface IFACE [options]\n"
        "  --socket PATH             Private local Unix socket\n"
        "  --domain-id ID            DDS domain (default 0)\n"
        "  --fps RATE                Requested sample rate, 0.2..30\n"
        "  --api-timeout-ms MS       Video RPC timeout, 100..5000\n"
        "  --socket-timeout-ms MS    IPC write timeout, 100..60000\n"
        "  --max-jpeg-bytes BYTES    Frame cap, up to 16 MiB\n";
      std::exit(0);
    } else if (option == "--interface") {
      options.interface = require_value(argc, argv, index);
    } else if (option == "--socket") {
      options.socket_path = require_value(argc, argv, index);
    } else if (option == "--domain-id") {
      const auto value = parse_integer(require_value(argc, argv, index), option);
      if (value < 0 || value > 232) {
        usage_error("--domain-id must be in 0..232");
      }
      options.domain_id = static_cast<int>(value);
    } else if (option == "--fps") {
      options.frames_per_second = parse_floating_point(require_value(argc, argv, index), option);
      if (options.frames_per_second < 0.2 || options.frames_per_second > 30.0) {
        usage_error("--fps must be in 0.2..30");
      }
    } else if (option == "--api-timeout-ms") {
      const auto value = parse_integer(require_value(argc, argv, index), option);
      if (value < 100 || value > 5000) {
        usage_error("--api-timeout-ms must be in 100..5000");
      }
      options.api_timeout_ms = static_cast<int>(value);
    } else if (option == "--socket-timeout-ms") {
      const auto value = parse_integer(require_value(argc, argv, index), option);
      if (value < 100 || value > 60000) {
        usage_error("--socket-timeout-ms must be in 100..60000");
      }
      options.socket_timeout_ms = static_cast<int>(value);
    } else if (option == "--max-jpeg-bytes") {
      const auto value = parse_integer(require_value(argc, argv, index), option);
      if (value <= 0 || static_cast<unsigned long long>(value) >
        go2_sensors::camera_ipc::kAbsoluteMaxJpegBytes)
      {
        usage_error("--max-jpeg-bytes is outside its safe range");
      }
      options.max_jpeg_bytes = static_cast<std::uint32_t>(value);
    } else {
      usage_error("unknown option: " + option);
    }
  }
  if (options.interface.empty() || options.interface.find('/') != std::string::npos ||
    options.interface == "." || options.interface == ".." ||
    options.interface.size() >= IFNAMSIZ)
  {
    usage_error("--interface is required and must be a network interface name");
  }
  return options;
}

class SocketServer final
{
public:
  explicit SocketServer(std::string path)
  : path_(std::move(path))
  {
    open();
  }

  SocketServer(const SocketServer &) = delete;
  SocketServer & operator=(const SocketServer &) = delete;

  ~SocketServer()
  {
    close_and_unlink();
  }

  int accept_same_user(const int timeout_ms)
  {
    pollfd event{descriptor_, POLLIN, 0};
    int result = 0;
    do {
      result = ::poll(&event, 1, timeout_ms);
    } while (result < 0 && errno == EINTR && g_stop_requested == 0);
    if (result <= 0 || g_stop_requested != 0) {
      return -1;
    }
    if ((event.revents & POLLIN) == 0) {
      return -1;
    }
    const int peer = ::accept4(descriptor_, nullptr, nullptr, SOCK_CLOEXEC);
    if (peer < 0) {
      return -1;
    }
    ucred credentials{};
    socklen_t size = sizeof(credentials);
    if (::getsockopt(peer, SOL_SOCKET, SO_PEERCRED, &credentials, &size) != 0 ||
      size != sizeof(credentials) || credentials.uid != ::getuid())
    {
      ::close(peer);
      std::cerr << "Rejected camera IPC peer with mismatched credentials\n";
      return -1;
    }
    return peer;
  }

private:
  void close_and_unlink() noexcept
  {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
      descriptor_ = -1;
    }
    if (bound_) {
      ::unlink(path_.c_str());
      bound_ = false;
    }
  }
  void validate_parent(const std::filesystem::path & parent)
  {
    std::error_code error;
    const bool parent_exists = std::filesystem::exists(parent, error);
    if (error) {
      throw std::runtime_error("cannot inspect socket directory: " + error.message());
    }
    if (!parent_exists) {
      const auto ancestor = parent.parent_path();
      struct stat ancestor_status {};
      if (ancestor.empty() || ::lstat(ancestor.c_str(), &ancestor_status) != 0 ||
        !S_ISDIR(ancestor_status.st_mode) || ancestor_status.st_uid != ::getuid())
      {
        throw std::runtime_error(
                "socket parent does not exist and its immediate ancestor is not user-owned");
      }
      if (!std::filesystem::create_directory(parent, error) || error) {
        throw std::runtime_error("cannot create socket directory: " + error.message());
      }
      if (::chmod(parent.c_str(), S_IRWXU) != 0) {
        throw std::runtime_error(
                std::string("cannot secure new socket directory: ") + std::strerror(errno));
      }
    }
    struct stat status {};
    if (::lstat(parent.c_str(), &status) != 0 || !S_ISDIR(status.st_mode) ||
      status.st_uid != ::getuid() || (status.st_mode & (S_IRWXG | S_IRWXO)) != 0)
    {
      throw std::runtime_error("socket directory must be private and owned by the current user");
    }
  }

  void remove_owned_stale_socket()
  {
    struct stat status {};
    if (::lstat(path_.c_str(), &status) != 0) {
      if (errno == ENOENT) {
        return;
      }
      throw std::runtime_error(std::string("cannot inspect socket path: ") + std::strerror(errno));
    }
    if (!S_ISSOCK(status.st_mode) || status.st_uid != ::getuid()) {
      throw std::runtime_error("refusing to replace a non-socket or foreign-owned path");
    }
    if (::unlink(path_.c_str()) != 0) {
      throw std::runtime_error(std::string("cannot remove stale socket: ") + std::strerror(errno));
    }
  }

  void open()
  {
    sockaddr_un address{};
    if (path_.empty() || path_.front() != '/' || path_.size() >= sizeof(address.sun_path)) {
      throw std::invalid_argument("camera socket path must be absolute and fit sockaddr_un");
    }
    const std::filesystem::path filesystem_path(path_);
    if (!filesystem_path.has_parent_path()) {
      throw std::invalid_argument("camera socket path must have a parent directory");
    }
    validate_parent(filesystem_path.parent_path());
    remove_owned_stale_socket();

    const mode_t previous_mask = ::umask(S_IRWXG | S_IRWXO);
    descriptor_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (descriptor_ < 0) {
      ::umask(previous_mask);
      throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
    }
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path_.c_str(), path_.size() + 1U);
    const auto address_size = static_cast<socklen_t>(
      offsetof(sockaddr_un, sun_path) + path_.size() + 1U);
    if (::bind(descriptor_, reinterpret_cast<const sockaddr *>(&address), address_size) != 0) {
      const int saved_error = errno;
      ::umask(previous_mask);
      close_and_unlink();
      throw std::runtime_error(std::string("bind failed: ") + std::strerror(saved_error));
    }
    ::umask(previous_mask);
    bound_ = true;
    if (::chmod(path_.c_str(), S_IRUSR | S_IWUSR) != 0) {
      const int saved_error = errno;
      close_and_unlink();
      throw std::runtime_error(
              std::string("cannot set socket permissions: ") + std::strerror(saved_error));
    }
    if (::listen(descriptor_, 1) != 0) {
      const int saved_error = errno;
      close_and_unlink();
      throw std::runtime_error(std::string("listen failed: ") + std::strerror(saved_error));
    }
  }

  std::string path_;
  int descriptor_{-1};
  bool bound_{false};
};

void sleep_until_or_stop(const std::chrono::steady_clock::time_point deadline)
{
  using Clock = std::chrono::steady_clock;
  const auto quantum = std::chrono::duration_cast<Clock::duration>(
    std::chrono::milliseconds(100));
  while (g_stop_requested == 0) {
    const auto now = Clock::now();
    if (now >= deadline) {
      return;
    }
    std::this_thread::sleep_for(std::min(deadline - now, quantum));
  }
}

bool write_camera_record(
  const Options & options, const int peer, std::uint64_t & record_sequence,
  const DaemonCounters & counters, const std::vector<std::uint8_t> * const jpeg,
  const std::uint16_t width, const std::uint16_t height)
{
  if (record_sequence == std::numeric_limits<std::uint64_t>::max()) {
    std::cerr << "Camera IPC record sequence exhausted; stopping daemon\n";
    g_stop_requested = 1;
    return false;
  }
  go2_sensors::camera_ipc::FrameHeader header;
  header.sequence = record_sequence++;
  header.capture_realtime_ns = go2_sensors::camera_ipc::clock_nanoseconds(CLOCK_REALTIME);
  header.capture_monotonic_ns = go2_sensors::camera_ipc::clock_nanoseconds(CLOCK_MONOTONIC);
  header.payload_bytes = jpeg == nullptr ? 0U : static_cast<std::uint32_t>(jpeg->size());
  header.flags = jpeg == nullptr ? go2_sensors::camera_ipc::kStatusOnly : 0U;
  header.width_hint = width;
  header.height_hint = height;
  header.api_request_count = counters.api_requests;
  header.api_accepted_count = counters.api_accepted;
  header.source_rejected_count = counters.source_rejected;
  header.api_error_count = counters.api_errors;
  header.ipc_connection_count = counters.ipc_connections;
  header.ipc_disconnect_count = counters.ipc_disconnects;
  header.last_api_code = counters.last_api_code;
  if (header.capture_realtime_ns == 0U || header.capture_monotonic_ns == 0U) {
    std::cerr << "Clock read failed; camera quality record was not sent\n";
    return true;
  }

  const auto encoded_header = go2_sensors::camera_ipc::encode_header(header);
  std::string error;
  if (!go2_sensors::camera_ipc::write_exact(
      peer, encoded_header.data(), encoded_header.size(), options.socket_timeout_ms, error) ||
    (jpeg != nullptr && !go2_sensors::camera_ipc::write_exact(
      peer, jpeg->data(), jpeg->size(), options.socket_timeout_ms, error)))
  {
    std::cerr << "Camera IPC disconnected: " << error << "\n";
    return false;
  }
  return true;
}

bool stream_camera(
  unitree::robot::go2::VideoClient & video_client, const Options & options,
  const int peer, std::uint64_t & record_sequence, DaemonCounters & counters)
{
  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / options.frames_per_second));
  auto next_sample = Clock::now();
  while (g_stop_requested == 0) {
    std::vector<std::uint8_t> jpeg;
    const std::int32_t result = video_client.GetImageSample(jpeg);
    counters.api_requests += 1U;
    counters.last_api_code = result;
    if (result != 0) {
      counters.api_errors += 1U;
      std::cerr << "VideoClient read failed with code " << result << "\n";
      if (!write_camera_record(
          options, peer, record_sequence, counters, nullptr, 0U, 0U))
      {
        counters.ipc_disconnects += 1U;
        return false;
      }
      next_sample = Clock::now() + period;
      sleep_until_or_stop(next_sample);
      continue;
    }
    if (jpeg.empty() || jpeg.size() > options.max_jpeg_bytes ||
      !go2_sensors::camera_ipc::looks_like_jpeg(jpeg.data(), jpeg.size()))
    {
      counters.source_rejected += 1U;
      std::cerr << "Rejected invalid or oversized JPEG from camera API\n";
      if (!write_camera_record(
          options, peer, record_sequence, counters, nullptr, 0U, 0U))
      {
        counters.ipc_disconnects += 1U;
        return false;
      }
      next_sample = Clock::now() + period;
      sleep_until_or_stop(next_sample);
      continue;
    }

    std::uint16_t width = 0U;
    std::uint16_t height = 0U;
    if (!go2_sensors::camera_ipc::jpeg_structure_is_valid(
        jpeg.data(), jpeg.size(), width, height))
    {
      counters.source_rejected += 1U;
      std::cerr << "Rejected JPEG with malformed marker structure\n";
      if (!write_camera_record(
          options, peer, record_sequence, counters, nullptr, 0U, 0U))
      {
        counters.ipc_disconnects += 1U;
        return false;
      }
      next_sample = Clock::now() + period;
      sleep_until_or_stop(next_sample);
      continue;
    }
    counters.api_accepted += 1U;
    if (!write_camera_record(
        options, peer, record_sequence, counters, &jpeg, width, height))
    {
      counters.ipc_disconnects += 1U;
      return false;
    }

    next_sample += period;
    if (next_sample < Clock::now()) {
      next_sample = Clock::now();
    }
    sleep_until_or_stop(next_sample);
  }
  return true;
}

int run(const Options & options)
{
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  std::signal(SIGPIPE, SIG_IGN);

  std::cout <<
    "============================================================\n"
    " READ-ONLY GO2 CAMERA: image acquisition only; no control API\n"
    " Interface: " << options.interface << "\n"
    " Socket:    " << options.socket_path << "\n"
    "============================================================\n";

  if (::if_nametoindex(options.interface.c_str()) == 0U) {
    throw std::runtime_error("requested network interface does not exist: " + options.interface);
  }

  SocketServer server(options.socket_path);
  unitree::robot::ChannelFactory::Instance()->Init(options.domain_id, options.interface);
  ChannelReleaseGuard channel_release_guard;
  unitree::robot::go2::VideoClient video_client;
  video_client.SetTimeout(static_cast<float>(options.api_timeout_ms) / 1000.0F);
  video_client.Init();

  std::uint64_t record_sequence = 1U;
  DaemonCounters counters;
  while (g_stop_requested == 0) {
    const int peer = server.accept_same_user(500);
    if (peer < 0) {
      continue;
    }
    counters.ipc_connections += 1U;
    std::cout << "Accepted same-user ROS camera bridge\n";
    stream_camera(video_client, options, peer, record_sequence, counters);
    ::shutdown(peer, SHUT_RDWR);
    ::close(peer);
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(parse_options(argc, argv));
  } catch (const std::exception & error) {
    std::cerr << "go2_camera_daemon: " << error.what() << "\n";
    return 1;
  }
}
