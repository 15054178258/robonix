#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "go2_chassis/daemon_core.hpp"
#include "go2_chassis/protocol.hpp"
#include "go2_chassis/unitree_sport_client.hpp"

namespace {

constexpr const char *kRequiredMotionAcknowledgement =
    "GO2_PHYSICAL_MOTION_APPROVED";
constexpr float kCommissioningMaxVx = 0.05F;
constexpr float kCommissioningMaxVy = 0.0F;
constexpr float kCommissioningMaxWz = 0.0F;
constexpr std::uint64_t kCommissioningMaxMotionMs = 2'000U;
constexpr const char *kFirstMotionProfile =
    "workstation-first-motion-corrected-v1";
constexpr const char *kSecondMotionProfile =
    "workstation-second-motion-corrected-v1";
constexpr float kSecondMotionMaxVx = 0.30F;
constexpr float kSecondMotionMaxVy = 0.0F;
constexpr float kSecondMotionMaxWz = 0.0F;
constexpr std::uint64_t kSecondMotionMaxMotionMs = 1'500U;
constexpr const char *kStagedNav2Profile =
    "workstation-staged-nav2-corrected-v1";
constexpr float kStagedNav2MaxVx = 0.30F;
constexpr float kStagedNav2MaxVy = 0.0F;
constexpr float kStagedNav2MaxWz = 0.40F;
// Zero disables only the whole-navigation-session deadline.  The independent
// 300 ms daemon watchdog remains mandatory.
constexpr std::uint64_t kStagedNav2MaxMotionMs = 0U;
std::atomic<bool> g_shutdown_requested{false};

struct Options {
  std::string socket_path;
  std::string network_interface;
  bool allow_motion{false};
  bool preserve_classic_walk{false};
  std::string acknowledgement;
  std::string motion_profile{kFirstMotionProfile};
  std::uint64_t watchdog_ms{300U};
  float max_vx{0.0F};
  float max_vy{0.0F};
  float max_wz{0.0F};
  std::uint64_t max_motion_ms{0U};
  bool has_max_vx{false};
  bool has_max_vy{false};
  bool has_max_wz{false};
  bool has_max_motion_ms{false};
};

std::uint64_t MonotonicNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

void SignalHandler(int) { g_shutdown_requested.store(true); }

void PrintUsage(const char *program) {
  std::cerr
      << "READ CAREFULLY: this daemon is motion-disabled unless two explicit "
         "runtime gates are supplied.\n"
      << "Usage: " << program
      << " --socket ABSOLUTE_PATH [--watchdog-ms 100..1000]"
         " [--allow-motion --interface IFACE"
         " --motion-ack GO2_PHYSICAL_MOTION_APPROVED"
         " [--motion-profile workstation-second-motion-corrected-v1|"
         "workstation-staged-nav2-corrected-v1]"
         " [--preserve-classic-walk]"
         " --max-vx PROFILE_MAX --max-vy 0 --max-wz PROFILE_MAX"
         " --max-motion-ms PROFILE_MAX_MS]\n";
}

Options ParseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto require_value = [&](const char *name) -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return argv[++index];
    };
    const auto require_float = [&](const char *name) -> float {
      const std::string text = require_value(name);
      std::size_t consumed = 0U;
      const float value = std::stof(text, &consumed);
      if (consumed != text.size() || !std::isfinite(value)) {
        throw std::runtime_error(std::string("invalid value for ") + name);
      }
      return value;
    };
    const auto require_uint64 = [&](const char *name) -> std::uint64_t {
      const std::string text = require_value(name);
      std::size_t consumed = 0U;
      const std::uint64_t value = std::stoull(text, &consumed, 10);
      if (consumed != text.size() || text.empty() || text.front() == '-' ||
          text.front() == '+') {
        throw std::runtime_error(std::string("invalid value for ") + name);
      }
      return value;
    };
    if (argument == "--socket") {
      options.socket_path = require_value("--socket");
    } else if (argument == "--interface") {
      options.network_interface = require_value("--interface");
    } else if (argument == "--watchdog-ms") {
      options.watchdog_ms = require_uint64("--watchdog-ms");
    } else if (argument == "--max-vx") {
      options.max_vx = require_float("--max-vx");
      options.has_max_vx = true;
    } else if (argument == "--max-vy") {
      options.max_vy = require_float("--max-vy");
      options.has_max_vy = true;
    } else if (argument == "--max-wz") {
      options.max_wz = require_float("--max-wz");
      options.has_max_wz = true;
    } else if (argument == "--max-motion-ms") {
      options.max_motion_ms = require_uint64("--max-motion-ms");
      options.has_max_motion_ms = true;
    } else if (argument == "--allow-motion") {
      options.allow_motion = true;
    } else if (argument == "--preserve-classic-walk") {
      options.preserve_classic_walk = true;
    } else if (argument == "--motion-ack") {
      options.acknowledgement = require_value("--motion-ack");
    } else if (argument == "--motion-profile") {
      options.motion_profile = require_value("--motion-profile");
    } else if (argument == "--help" || argument == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (options.watchdog_ms < 100U || options.watchdog_ms > 1000U) {
    throw std::runtime_error("watchdog must be between 100 and 1000 ms");
  }
  if (options.socket_path.empty() ||
      !std::filesystem::path(options.socket_path).is_absolute() ||
      options.socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
    throw std::runtime_error("socket path must be absolute and fit sockaddr_un");
  }
  if (options.allow_motion && options.network_interface.empty()) {
    throw std::runtime_error("--interface is required with --allow-motion");
  }
  if (options.allow_motion &&
      options.acknowledgement != kRequiredMotionAcknowledgement) {
    throw std::runtime_error("exact --motion-ack value is required");
  }
  if (!go2_chassis::MotionWatchdogDeploymentEligible(
          options.allow_motion, options.watchdog_ms)) {
    throw std::runtime_error(
        "motion profiles require the exact audited 300 ms watchdog");
  }
  if (options.allow_motion &&
      (!options.has_max_vx || !options.has_max_vy || !options.has_max_wz ||
       !options.has_max_motion_ms)) {
    throw std::runtime_error(
        "motion requires every fixed commissioning envelope argument");
  }
  const bool first_motion_envelope =
      options.motion_profile == kFirstMotionProfile &&
      !(options.max_vx != kCommissioningMaxVx ||
        options.max_vy != kCommissioningMaxVy ||
        options.max_wz != kCommissioningMaxWz ||
        options.max_motion_ms != kCommissioningMaxMotionMs);
  const bool second_motion_envelope =
      options.motion_profile == kSecondMotionProfile &&
      options.max_vx == kSecondMotionMaxVx &&
      options.max_vy == kSecondMotionMaxVy &&
      options.max_wz == kSecondMotionMaxWz &&
      options.max_motion_ms == kSecondMotionMaxMotionMs;
  const bool staged_nav2_envelope =
      options.motion_profile == kStagedNav2Profile &&
      options.max_vx == kStagedNav2MaxVx &&
      options.max_vy == kStagedNav2MaxVy &&
      options.max_wz == kStagedNav2MaxWz &&
      options.max_motion_ms == kStagedNav2MaxMotionMs;
  if (options.allow_motion &&
      !first_motion_envelope && !second_motion_envelope &&
      !staged_nav2_envelope) {
    throw std::runtime_error(
        "motion envelope does not match the selected audited profile");
  }
  if (options.preserve_classic_walk &&
      (!options.allow_motion || options.motion_profile != kStagedNav2Profile)) {
    throw std::runtime_error(
        "--preserve-classic-walk requires the staged Nav2 motion profile");
  }
  return options;
}

void PrepareSocketDirectory(const std::filesystem::path &socket_path) {
  const std::filesystem::path parent =
      socket_path.has_parent_path() ? socket_path.parent_path() : ".";
  std::error_code error;
  std::filesystem::create_directories(parent, error);
  if (error) {
    throw std::runtime_error("cannot create socket directory: " +
                             error.message());
  }
  struct stat parent_stat {};
  if (::lstat(parent.c_str(), &parent_stat) != 0 ||
      !S_ISDIR(parent_stat.st_mode) || parent_stat.st_uid != ::geteuid()) {
    throw std::runtime_error("socket directory must be a real directory owned "
                             "by the current user");
  }
  if (::chmod(parent.c_str(), S_IRWXU) != 0) {
    throw std::runtime_error("cannot set socket directory mode 0700");
  }
}

void RemoveOwnedSocket(const std::filesystem::path &socket_path) {
  struct stat socket_stat {};
  if (::lstat(socket_path.c_str(), &socket_stat) != 0) {
    if (errno == ENOENT) {
      return;
    }
    throw std::runtime_error("cannot inspect existing socket path");
  }
  if (!S_ISSOCK(socket_stat.st_mode) || socket_stat.st_uid != ::geteuid()) {
    throw std::runtime_error("refusing to replace a non-socket or foreign path");
  }
  if (::unlink(socket_path.c_str()) != 0) {
    throw std::runtime_error("cannot remove stale owned socket");
  }
}

int OpenServerSocket(const std::filesystem::path &socket_path) {
  PrepareSocketDirectory(socket_path);
  RemoveOwnedSocket(socket_path);
  const int descriptor =
      ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
  if (descriptor < 0) {
    throw std::runtime_error("socket() failed: " +
                             std::string(std::strerror(errno)));
  }

  sockaddr_un address {};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, socket_path.c_str(), sizeof(address.sun_path) - 1U);
  const mode_t previous_umask = ::umask(0077);
  const int bind_result =
      ::bind(descriptor, reinterpret_cast<const sockaddr *>(&address),
             sizeof(address));
  ::umask(previous_umask);
  if (bind_result != 0 || ::chmod(socket_path.c_str(), S_IRUSR | S_IWUSR) != 0 ||
      ::listen(descriptor, 1) != 0) {
    const std::string error = std::strerror(errno);
    ::close(descriptor);
    (void)::unlink(socket_path.c_str());
    throw std::runtime_error("cannot bind/listen on IPC socket: " + error);
  }
  return descriptor;
}

bool PeerIsCurrentUser(int descriptor) {
  ucred credentials {};
  socklen_t length = sizeof(credentials);
  return ::getsockopt(descriptor, SOL_SOCKET, SO_PEERCRED, &credentials, &length) ==
             0 &&
         length == sizeof(credentials) && credentials.uid == ::geteuid();
}

int AcceptPeer(int server_descriptor) {
  const int peer = ::accept4(server_descriptor, nullptr, nullptr,
                             SOCK_CLOEXEC | SOCK_NONBLOCK);
  if (peer >= 0 && !PeerIsCurrentUser(peer)) {
    std::cerr << "Rejected IPC peer with a different UID\n";
    ::close(peer);
    return -1;
  }
  return peer;
}

void SendReply(int descriptor, const go2_chassis::ReplyPacket &reply) {
  const ssize_t sent =
      ::send(descriptor, &reply, sizeof(reply), MSG_DONTWAIT | MSG_NOSIGNAL);
  if (sent != static_cast<ssize_t>(sizeof(reply))) {
    throw std::runtime_error("failed to send complete IPC reply");
  }
}

int Run(const Options &options, go2_chassis::ISportClient &client) {
  go2_chassis::DaemonConfig daemon_config;
  daemon_config.allow_motion = options.allow_motion;
  daemon_config.repeatable_arm =
      options.motion_profile == kStagedNav2Profile;
  daemon_config.preserve_classic_walk = options.preserve_classic_walk;
  daemon_config.watchdog_ns = options.watchdog_ms * 1'000'000ULL;
  daemon_config.max_vx = options.max_vx;
  daemon_config.max_vy = options.max_vy;
  daemon_config.max_wz = options.max_wz;
  daemon_config.max_motion_ns = options.max_motion_ms * 1'000'000ULL;
  go2_chassis::DaemonCore core(daemon_config, client);

  const std::filesystem::path socket_path(options.socket_path);
  const int server = OpenServerSocket(socket_path);
  int peer = -1;
  std::cout << "Go2 SDK daemon ready at " << socket_path << "; motion="
            << (options.allow_motion ? "ENABLED (still disarmed)" : "DISABLED")
            << "\n";

  const auto cleanup = [&]() {
    core.OnDisconnect();
    // OnDisconnect performs the primary stop plus one immediate retry.  A
    // process shutdown must not silently turn a missing StopMove reply into a
    // successful stop, so make a few more bounded attempts before tearing down
    // the SDK process.  Each SDK call already has its own short timeout.
    constexpr int kShutdownStopRetries = 3;
    for (int attempt = 0;
         attempt < kShutdownStopRetries && core.stop_unconfirmed(); ++attempt) {
      (void)core.CheckWatchdog(MonotonicNowNs());
    }
    if (core.stop_unconfirmed()) {
      std::cerr << "SAFETY FAULT: StopMove acknowledgement remained "
                   "unconfirmed during bounded daemon shutdown; local "
                   "controller is disarmed\n";
    }
    if (peer >= 0) {
      ::close(peer);
      peer = -1;
    }
    ::close(server);
    (void)::unlink(socket_path.c_str());
  };

  try {
    while (!g_shutdown_requested.load()) {
    pollfd descriptors[2] = {
        {server, POLLIN, 0},
        {peer, static_cast<short>(POLLIN | POLLHUP | POLLERR), 0},
    };
    const nfds_t count = peer >= 0 ? 2U : 1U;
    const int poll_result = ::poll(descriptors, count, 20);
    if (poll_result < 0 && errno != EINTR) {
      throw std::runtime_error("poll() failed");
    }

    if ((descriptors[0].revents & POLLIN) != 0) {
      const int candidate = AcceptPeer(server);
      if (candidate >= 0) {
        if (peer >= 0) {
          std::cerr << "Rejected second IPC controller\n";
          ::close(candidate);
        } else {
          peer = candidate;
        }
      }
    }

    if (peer >= 0 &&
        (descriptors[1].revents & static_cast<short>(POLLHUP | POLLERR)) != 0) {
      core.OnDisconnect();
      ::close(peer);
      peer = -1;
    }

    if (peer >= 0 && (descriptors[1].revents & POLLIN) != 0) {
      go2_chassis::CommandPacket command {};
      const ssize_t received =
          ::recv(peer, &command, sizeof(command), MSG_DONTWAIT | MSG_TRUNC);
      if (received == 0) {
        core.OnDisconnect();
        ::close(peer);
        peer = -1;
      } else if (received > 0) {
        const auto reply =
            received == static_cast<ssize_t>(sizeof(command))
                ? core.Handle(command, MonotonicNowNs())
                : go2_chassis::MakeReply(command.sequence,
                                         go2_chassis::ReplyCode::kMalformed,
                                         core.armed(), core.faulted());
        SendReply(peer, reply);
      } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        core.OnDisconnect();
        ::close(peer);
        peer = -1;
      }
    }

      if (core.CheckWatchdog(MonotonicNowNs())) {
        std::cerr
            << "SDK watchdog expired: StopMove requested and daemon disarmed\n";
      }
    }
  } catch (...) {
    cleanup();
    throw;
  }

  cleanup();
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = ParseOptions(argc, argv);
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    go2_chassis::UnitreeSportClient client;
    if (options.allow_motion) {
      std::string error;
      if (!client.Initialize(options.network_interface, &error)) {
        throw std::runtime_error("Unitree SDK2 initialization failed: " + error);
      }
    } else {
      std::cout << "READ-ONLY/NO-MOTION MODE: SDK2 is not initialized and all "
                   "arm/move requests are rejected.\n";
    }
    return Run(options, client);
  } catch (const std::exception &exception) {
    std::cerr << "go2_sport_daemon: " << exception.what() << "\n";
    PrintUsage(argv[0]);
    return 2;
  }
}
