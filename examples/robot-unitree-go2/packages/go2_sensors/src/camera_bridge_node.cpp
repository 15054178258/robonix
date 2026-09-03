#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "cv_bridge/cv_bridge.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "go2_sensors/camera_error_watermark.hpp"
#include "go2_sensors/camera_ipc_protocol.hpp"
#include "go2_sensors/latest_frame_mailbox.hpp"
#include "go2_sensors/strict_jpeg_decoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace go2_sensors
{

using namespace std::chrono_literals;

diagnostic_msgs::msg::KeyValue camera_key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

struct CameraRuntimeState
{
  bool connected{false};
  std::uint64_t received{0U};
  std::uint64_t published{0U};
  std::uint64_t rejected{0U};
  std::uint64_t superseded{0U};
  std::uint64_t status_records{0U};
  std::uint64_t connection_attempts{0U};
  std::uint64_t connection_failures{0U};
  std::uint64_t connections{0U};
  std::uint64_t reconnects{0U};
  std::uint64_t disconnects{0U};
  std::uint64_t daemon_api_requests{0U};
  std::uint64_t daemon_api_accepted{0U};
  std::uint64_t daemon_source_rejected{0U};
  std::uint64_t daemon_api_errors{0U};
  std::uint64_t daemon_ipc_connections{0U};
  std::uint64_t daemon_ipc_disconnects{0U};
  std::uint64_t daemon_counter_resets{0U};
  std::uint64_t daemon_epoch{0U};
  std::int32_t daemon_last_api_code{0};
  double rate_hz{0.0};
  CameraErrorWatermark last_error{"camera daemon not connected"};
  std::chrono::steady_clock::time_point last_published{};
  std::chrono::steady_clock::time_point started{std::chrono::steady_clock::now()};
};

struct CameraQualitySample
{
  std::chrono::steady_clock::time_point stamp{};
  std::uint64_t daemon_api_requests{0U};
  std::uint64_t daemon_api_errors{0U};
  std::uint64_t daemon_source_rejected{0U};
  std::uint64_t bridge_rejected{0U};
  std::uint64_t published{0U};
  std::uint64_t disconnects{0U};
  std::uint64_t daemon_epoch{0U};
};

struct CameraQualityMetrics
{
  double elapsed_seconds{0.0};
  double valid_rate_hz{0.0};
  double error_ratio{0.0};
  std::uint64_t attempts{0U};
  std::uint64_t failures{0U};
  std::uint64_t disconnects{0U};
  std::size_t samples{0U};
  bool ready{false};
};

class CameraBridgeNode final : public rclcpp::Node
{
public:
  CameraBridgeNode()
  : Node("go2_camera_bridge")
  {
    socket_path_ = declare_parameter<std::string>(
      "camera.socket_path", camera_ipc::default_socket_path());
    frame_id_ = declare_parameter<std::string>("camera.frame_id", "front_camera");
    image_topic_ = declare_parameter<std::string>(
      "camera.image_topic", "/camera/color/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera.camera_info_topic", "/camera/color/camera_info");
    status_topic_ = declare_parameter<std::string>(
      "status.topic", "/go2/sensors/status");
    max_jpeg_bytes_ = checked_u32_parameter(
      "camera.max_jpeg_bytes", camera_ipc::kDefaultMaxJpegBytes,
      camera_ipc::kAbsoluteMaxJpegBytes);
    max_width_ = checked_positive_int_parameter("camera.max_width", 4096, 16384);
    max_height_ = checked_positive_int_parameter("camera.max_height", 4096, 16384);
    read_timeout_ms_ = checked_positive_int_parameter("camera.read_timeout_ms", 1500, 60000);
    reconnect_delay_ms_ = checked_positive_int_parameter(
      "camera.reconnect_delay_ms", 1000, 60000);
    max_frame_age_ms_ = checked_positive_int_parameter("camera.max_frame_age_ms", 2000, 60000);
    stale_timeout_seconds_ = declare_parameter<double>("status.stale_timeout_seconds", 2.0);
    startup_grace_seconds_ = declare_parameter<double>("status.startup_grace_seconds", 5.0);
    quality_window_seconds_ = declare_parameter<double>("status.quality_window_seconds", 10.0);
    min_valid_rate_hz_ = declare_parameter<double>("status.min_valid_rate_hz", 1.0);
    max_error_ratio_ = declare_parameter<double>("status.max_error_ratio", 0.20);
    min_quality_samples_ = checked_positive_int_parameter(
      "status.min_quality_samples", 5, 1000);
    const double status_period_seconds =
      declare_parameter<double>("status.publish_period_seconds", 1.0);
    if (!std::isfinite(stale_timeout_seconds_) || stale_timeout_seconds_ <= 0.0 ||
      !std::isfinite(startup_grace_seconds_) || startup_grace_seconds_ <= 0.0 ||
      !std::isfinite(quality_window_seconds_) || quality_window_seconds_ < 2.0 ||
      quality_window_seconds_ > 60.0 ||
      !std::isfinite(min_valid_rate_hz_) || min_valid_rate_hz_ < 0.1 ||
      min_valid_rate_hz_ > 30.0 ||
      !std::isfinite(max_error_ratio_) || max_error_ratio_ < 0.0 ||
      max_error_ratio_ > 1.0 ||
      !std::isfinite(status_period_seconds) || status_period_seconds <= 0.0)
    {
      throw std::invalid_argument("camera status timing parameters must be finite and positive");
    }
    validate_absolute_topic(image_topic_, "camera.image_topic");
    validate_absolute_topic(camera_info_topic_, "camera.camera_info_topic");
    validate_absolute_topic(status_topic_, "status.topic");
    if (frame_id_.empty() || frame_id_.front() == '/') {
      throw std::invalid_argument("camera.frame_id must be non-empty and must not start with '/'");
    }
    if (socket_path_.empty() || socket_path_.front() != '/') {
      throw std::invalid_argument("camera.socket_path must be absolute");
    }

    load_calibration();

    rclcpp::SensorDataQoS qos;
    qos.keep_last(1);
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    const auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    camera_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, camera_info_qos);
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(status_period_seconds)),
      [this]() {publish_status();});

    processor_ = std::thread([this]() {processor_loop();});
    try {
      worker_ = std::thread([this]() {worker_loop();});
    } catch (...) {
      running_.store(false);
      latest_frame_.close();
      processor_.join();
      throw;
    }
    RCLCPP_INFO(
      get_logger(), "READ-ONLY camera bridge waiting on local socket %s", socket_path_.c_str());
  }

  ~CameraBridgeNode() override
  {
    running_.store(false);
    active_connection_generation_.store(0U);
    reconnect_condition_.notify_all();
    if (latest_frame_.close()) {
      note_superseded();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    if (processor_.joinable()) {
      processor_.join();
    }
  }

private:
  static void validate_absolute_topic(const std::string & topic, const char * const parameter)
  {
    if (topic.empty() || topic.front() != '/') {
      throw std::invalid_argument(std::string(parameter) + " must be an absolute ROS topic name");
    }
  }

  int checked_positive_int_parameter(const std::string & name, const int default_value, const int limit)
  {
    const auto value = declare_parameter<int>(name, default_value);
    if (value <= 0 || value > limit) {
      throw std::invalid_argument(name + " is outside its safe range");
    }
    return static_cast<int>(value);
  }

  std::uint32_t checked_u32_parameter(
    const std::string & name, const std::uint32_t default_value, const std::uint32_t limit)
  {
    const auto value = declare_parameter<std::int64_t>(
      name, static_cast<std::int64_t>(default_value));
    if (value <= 0 || static_cast<std::uint64_t>(value) > limit) {
      throw std::invalid_argument(name + " is outside its safe range");
    }
    return static_cast<std::uint32_t>(value);
  }

  static std::array<double, 9> vector_to_nine(
    const std::vector<double> & source, const std::string & name)
  {
    if (source.size() != 9U) {
      throw std::invalid_argument(name + " must contain exactly 9 values");
    }
    std::array<double, 9> destination{};
    std::copy(source.begin(), source.end(), destination.begin());
    return destination;
  }

  static std::array<double, 12> vector_to_twelve(
    const std::vector<double> & source, const std::string & name)
  {
    if (source.size() != 12U) {
      throw std::invalid_argument(name + " must contain exactly 12 values");
    }
    std::array<double, 12> destination{};
    std::copy(source.begin(), source.end(), destination.begin());
    return destination;
  }

  void load_calibration()
  {
    distortion_model_ = declare_parameter<std::string>(
      "calibration.distortion_model", "plumb_bob");
    distortion_ = declare_parameter<std::vector<double>>(
      "calibration.d", std::vector<double>(5U, 0.0));
    intrinsic_ = vector_to_nine(
      declare_parameter<std::vector<double>>(
        "calibration.k", std::vector<double>(9U, 0.0)),
      "calibration.k");
    rectification_ = vector_to_nine(
      declare_parameter<std::vector<double>>(
        "calibration.r", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}),
      "calibration.r");
    projection_ = vector_to_twelve(
      declare_parameter<std::vector<double>>(
        "calibration.p", std::vector<double>(12U, 0.0)),
      "calibration.p");
    calibrated_ = intrinsic_[0] > 0.0 && intrinsic_[4] > 0.0;
    const auto finite = [](const auto & values) {
        return std::all_of(values.begin(), values.end(), [](const double value) {
            return std::isfinite(value);
          });
      };
    if (distortion_model_.empty() || !finite(distortion_) || !finite(intrinsic_) ||
      !finite(rectification_) || !finite(projection_))
    {
      throw std::invalid_argument("camera calibration values must be finite");
    }
  }

  bool socket_file_is_safe(std::string & error) const
  {
    const std::filesystem::path filesystem_path(socket_path_);
    const auto parent = filesystem_path.parent_path();
    struct stat parent_status {};
    if (parent.empty() || ::lstat(parent.c_str(), &parent_status) != 0 ||
      !S_ISDIR(parent_status.st_mode) || parent_status.st_uid != ::getuid() ||
      (parent_status.st_mode & (S_IRWXG | S_IRWXO)) != 0)
    {
      error = "camera socket directory is not private and user-owned";
      return false;
    }
    struct stat socket_status {};
    if (::lstat(socket_path_.c_str(), &socket_status) != 0) {
      error = std::string("camera socket unavailable: ") + std::strerror(errno);
      return false;
    }
    if (!S_ISSOCK(socket_status.st_mode)) {
      error = "camera socket path is not a Unix socket";
      return false;
    }
    if (socket_status.st_uid != ::getuid()) {
      error = "camera socket is not owned by the current user";
      return false;
    }
    if ((socket_status.st_mode & (S_IRWXG | S_IRWXO)) != 0 ||
      (socket_status.st_mode & (S_IRUSR | S_IWUSR)) != (S_IRUSR | S_IWUSR))
    {
      error = "camera socket permissions are not 0600-equivalent";
      return false;
    }
    return true;
  }

  int connect_daemon(std::string & error) const
  {
    sockaddr_un address{};
    if (socket_path_.empty() || socket_path_.size() >= sizeof(address.sun_path)) {
      error = "camera socket path is empty or too long";
      return -1;
    }
    if (!socket_file_is_safe(error)) {
      return -1;
    }
    const int descriptor = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (descriptor < 0) {
      error = std::string("socket failed: ") + std::strerror(errno);
      return -1;
    }
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, socket_path_.c_str(), socket_path_.size() + 1U);
    if (::connect(descriptor, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0) {
      error = std::string("connect failed: ") + std::strerror(errno);
      ::close(descriptor);
      return -1;
    }
    ucred credentials{};
    socklen_t credential_size = sizeof(credentials);
    if (::getsockopt(descriptor, SOL_SOCKET, SO_PEERCRED, &credentials, &credential_size) != 0 ||
      credential_size != sizeof(credentials) || credentials.uid != ::getuid())
    {
      error = "camera daemon peer credentials rejected";
      ::close(descriptor);
      return -1;
    }
    return descriptor;
  }

  bool wait_before_reconnect()
  {
    std::unique_lock<std::mutex> lock(reconnect_mutex_);
    reconnect_condition_.wait_for(
      lock, std::chrono::milliseconds(reconnect_delay_ms_),
      [this]() {return !running_.load();});
    return running_.load();
  }

  void set_connection_state(const bool connected, std::string error = {})
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.connected = connected;
    if (!error.empty()) {
      state_.last_error.set_connection(std::move(error));
    } else if (connected) {
      state_.last_error.clear();
    }
  }

  void reject_frame(const CameraFrameRecord & frame, std::string error)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.rejected += 1U;
    if (state_.connected &&
      active_connection_generation_.load() == frame.connection_generation)
    {
      state_.last_error.record_stream_error(
        std::move(error), frame.connection_generation, frame.header.sequence);
    }
  }

  void note_superseded()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.superseded += 1U;
  }

  static bool daemon_counters_regressed(
    const CameraRuntimeState & state, const camera_ipc::FrameHeader & header) noexcept
  {
    return header.api_request_count < state.daemon_api_requests ||
           header.api_accepted_count < state.daemon_api_accepted ||
           header.source_rejected_count < state.daemon_source_rejected ||
           header.api_error_count < state.daemon_api_errors ||
           header.ipc_connection_count < state.daemon_ipc_connections ||
           header.ipc_disconnect_count < state.daemon_ipc_disconnects;
  }

  void update_daemon_stats(
    const camera_ipc::FrameHeader & header, const std::uint64_t connection_generation)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool reset = daemon_counters_regressed(state_, header);
    const bool new_api_error = !reset && header.api_error_count > state_.daemon_api_errors;
    const bool new_source_rejection =
      !reset && header.source_rejected_count > state_.daemon_source_rejected;
    if (reset) {
      state_.daemon_counter_resets += 1U;
      state_.daemon_epoch += 1U;
      state_.last_error.record_stream_error(
        "camera daemon counters restarted", connection_generation, header.sequence);
    } else if (new_api_error) {
      state_.last_error.record_stream_error(
        "camera API error; opaque return code " + std::to_string(header.last_api_code),
        connection_generation, header.sequence);
    } else if (new_source_rejection) {
      state_.last_error.record_stream_error(
        "camera daemon rejected an invalid source JPEG", connection_generation,
        header.sequence);
    }
    state_.daemon_api_requests = header.api_request_count;
    state_.daemon_api_accepted = header.api_accepted_count;
    state_.daemon_source_rejected = header.source_rejected_count;
    state_.daemon_api_errors = header.api_error_count;
    state_.daemon_ipc_connections = header.ipc_connection_count;
    state_.daemon_ipc_disconnects = header.ipc_disconnect_count;
    state_.daemon_last_api_code = header.last_api_code;
    if ((header.flags & camera_ipc::kStatusOnly) != 0U) {
      state_.status_records += 1U;
    }
  }

  bool frame_timestamp_is_fresh(const camera_ipc::FrameHeader & header, std::string & error) const
  {
    const std::uint64_t now = camera_ipc::clock_nanoseconds(CLOCK_MONOTONIC);
    if (now == 0U || header.capture_monotonic_ns > now) {
      error = "invalid camera monotonic timestamp";
      return false;
    }
    const std::uint64_t age_ns = now - header.capture_monotonic_ns;
    const std::uint64_t max_age_ns = static_cast<std::uint64_t>(max_frame_age_ms_) * 1000000ULL;
    if (age_ns > max_age_ns) {
      error = "stale camera frame";
      return false;
    }
    constexpr std::uint64_t kNanosecondsPerSecond = 1000000000ULL;
    if ((header.capture_realtime_ns / kNanosecondsPerSecond) >
      static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
    {
      error = "camera timestamp exceeds ROS message range";
      return false;
    }
    return true;
  }

  bool frame_is_invalidated(const CameraFrameRecord & frame) const
  {
    return !running_.load() ||
           active_connection_generation_.load() != frame.connection_generation;
  }

  void publish_frame(const CameraFrameRecord & frame)
  {
    if (frame_is_invalidated(frame)) {
      note_superseded();
      return;
    }
    const auto & header = frame.header;
    const auto & jpeg = frame.jpeg;
    std::string error;
    if (!frame_timestamp_is_fresh(header, error)) {
      reject_frame(frame, std::move(error));
      return;
    }
    StrictJpegImage decoded;
    if (!decode_jpeg_strict(
        jpeg.data(), jpeg.size(), static_cast<std::uint32_t>(max_width_),
        static_cast<std::uint32_t>(max_height_), decoded, error))
    {
      reject_frame(frame, "strict JPEG rejection: " + error);
      return;
    }
    if ((header.width_hint != 0U && header.width_hint != decoded.width) ||
      (header.height_hint != 0U && header.height_hint != decoded.height))
    {
      reject_frame(frame, "decoded JPEG dimensions disagree with daemon hints");
      return;
    }
    // Decoding a full-resolution JPEG is intentionally outside the socket
    // reader.  The one-slot mailbox already bounds pending work; publishing
    // this valid in-flight frame avoids starvation when decode is consistently
    // slower than acquisition.  Connection changes still invalidate it, and
    // freshness is checked again after decode.
    if (frame_is_invalidated(frame)) {
      note_superseded();
      return;
    }
    if (!frame_timestamp_is_fresh(header, error)) {
      reject_frame(frame, std::move(error));
      return;
    }

    try {
      const cv::Mat decoded_view(
        static_cast<int>(decoded.height), static_cast<int>(decoded.width), CV_8UC3,
        decoded.bgr.data());
      std_msgs::msg::Header ros_header;
      const rclcpp::Time capture_time(
        static_cast<std::int64_t>(header.capture_realtime_ns), RCL_SYSTEM_TIME);
      ros_header.stamp = static_cast<builtin_interfaces::msg::Time>(capture_time);
      ros_header.frame_id = frame_id_;
      auto image = cv_bridge::CvImage(
        ros_header, sensor_msgs::image_encodings::BGR8, decoded_view).toImageMsg();

      sensor_msgs::msg::CameraInfo camera_info;
      camera_info.header = ros_header;
      camera_info.width = decoded.width;
      camera_info.height = decoded.height;
      camera_info.distortion_model = distortion_model_;
      camera_info.d = distortion_;
      camera_info.k = intrinsic_;
      camera_info.r = rectification_;
      camera_info.p = projection_;

      image_publisher_->publish(*image);
      camera_info_publisher_->publish(camera_info);
    } catch (const std::exception & exception) {
      reject_frame(
        frame, std::string("camera conversion/publish failed: ") + exception.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_.published += 1U;
      state_.last_published = std::chrono::steady_clock::now();
      if (state_.connected &&
        active_connection_generation_.load() == frame.connection_generation)
      {
        state_.last_error.clear_if_recovered_by(
          frame.connection_generation, header.sequence);
      }
    }
  }

  void processor_loop()
  {
    while (running_.load()) {
      auto frame = latest_frame_.wait_take();
      if (!frame.has_value()) {
        return;
      }
      publish_frame(*frame);
    }
  }

  void receive_frames(const int descriptor, const std::uint64_t connection_generation)
  {
    std::uint64_t previous_sequence = 0U;
    bool have_sequence = false;
    while (running_.load()) {
      std::array<std::uint8_t, camera_ipc::kHeaderBytes> encoded_header{};
      std::string error;
      if (!camera_ipc::read_exact(
          descriptor, encoded_header.data(), encoded_header.size(), read_timeout_ms_, error))
      {
        set_connection_state(false, std::move(error));
        return;
      }
      camera_ipc::FrameHeader header;
      const auto header_result = camera_ipc::decode_header(
        encoded_header, max_jpeg_bytes_, header);
      if (header_result != camera_ipc::HeaderError::kOk) {
        set_connection_state(
          false, std::string("camera protocol rejected: ") +
          camera_ipc::header_error_string(header_result));
        return;
      }
      if (have_sequence && header.sequence <= previous_sequence) {
        set_connection_state(false, "camera sequence did not increase");
        return;
      }
      previous_sequence = header.sequence;
      have_sequence = true;

      update_daemon_stats(header, connection_generation);
      if ((header.flags & camera_ipc::kStatusOnly) != 0U) {
        continue;
      }

      CameraFrameRecord frame;
      frame.header = header;
      frame.connection_generation = connection_generation;
      frame.jpeg.resize(header.payload_bytes);
      if (!camera_ipc::read_exact(
          descriptor, frame.jpeg.data(), frame.jpeg.size(), read_timeout_ms_, error))
      {
        set_connection_state(false, std::move(error));
        return;
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.received += 1U;
      }
      const auto put_result = latest_frame_.put(std::move(frame));
      if (!put_result.accepted) {
        return;
      }
      if (put_result.replaced) {
        note_superseded();
      }
    }
  }

  void worker_loop()
  {
    std::uint64_t next_connection_generation = 1U;
    while (running_.load()) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.connection_attempts += 1U;
      }
      std::string error;
      const int descriptor = connect_daemon(error);
      if (descriptor < 0) {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          state_.connection_failures += 1U;
          state_.connected = false;
          state_.last_error.set_connection(std::move(error));
        }
        if (!wait_before_reconnect()) {
          return;
        }
        continue;
      }
      if (next_connection_generation == std::numeric_limits<std::uint64_t>::max()) {
        set_connection_state(false, "camera connection generation exhausted");
        return;
      }
      const std::uint64_t connection_generation = next_connection_generation++;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_.connections > 0U) {
          state_.reconnects += 1U;
        }
        state_.connections += 1U;
        state_.connected = true;
        state_.last_error.clear();
      }
      active_connection_generation_.store(connection_generation);
      receive_frames(descriptor, connection_generation);
      std::uint64_t expected_generation = connection_generation;
      active_connection_generation_.compare_exchange_strong(expected_generation, 0U);
      if (latest_frame_.discard_generation(connection_generation)) {
        note_superseded();
      }
      ::shutdown(descriptor, SHUT_RDWR);
      ::close(descriptor);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.connected = false;
        state_.disconnects += 1U;
        if (state_.last_error.empty()) {
          state_.last_error.set_connection("camera IPC disconnected");
        }
      }
      if (running_.load() && !wait_before_reconnect()) {
        return;
      }
    }
  }

  void publish_status()
  {
    const auto now = std::chrono::steady_clock::now();
    CameraRuntimeState snapshot;
    CameraQualityMetrics quality;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      CameraQualitySample sample;
      sample.stamp = now;
      sample.daemon_api_requests = state_.daemon_api_requests;
      sample.daemon_api_errors = state_.daemon_api_errors;
      sample.daemon_source_rejected = state_.daemon_source_rejected;
      sample.bridge_rejected = state_.rejected;
      sample.published = state_.published;
      sample.disconnects = state_.disconnects;
      sample.daemon_epoch = state_.daemon_epoch;
      if (!quality_samples_.empty() &&
        quality_samples_.back().daemon_epoch != sample.daemon_epoch)
      {
        quality_samples_.clear();
      }
      quality_samples_.push_back(sample);
      const auto cutoff = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(quality_window_seconds_));
      while (quality_samples_.size() > 1U && quality_samples_[1U].stamp <= cutoff) {
        quality_samples_.pop_front();
      }
      const auto & baseline = quality_samples_.front();
      const auto delta = [](const std::uint64_t current, const std::uint64_t previous) {
          return current >= previous ? current - previous : 0U;
        };
      quality.elapsed_seconds = std::chrono::duration<double>(now - baseline.stamp).count();
      quality.attempts = delta(sample.daemon_api_requests, baseline.daemon_api_requests);
      const auto api_errors = delta(sample.daemon_api_errors, baseline.daemon_api_errors);
      const auto source_rejected =
        delta(sample.daemon_source_rejected, baseline.daemon_source_rejected);
      const auto bridge_rejected = delta(sample.bridge_rejected, baseline.bridge_rejected);
      quality.failures = api_errors + source_rejected + bridge_rejected;
      quality.disconnects = delta(sample.disconnects, baseline.disconnects);
      const auto published = delta(sample.published, baseline.published);
      if (quality.elapsed_seconds > 0.0) {
        quality.valid_rate_hz = static_cast<double>(published) / quality.elapsed_seconds;
      }
      if (quality.attempts > 0U) {
        quality.error_ratio = std::min(
          1.0, static_cast<double>(quality.failures) /
          static_cast<double>(quality.attempts));
      }
      quality.samples = quality_samples_.size();
      quality.ready = quality.elapsed_seconds >= 2.0 &&
        quality.attempts >= static_cast<std::uint64_t>(min_quality_samples_);
      state_.rate_hz = quality.valid_rate_hz;
      snapshot = state_;
    }

    const double startup_age = std::chrono::duration<double>(now - snapshot.started).count();
    double frame_age = std::numeric_limits<double>::infinity();
    if (snapshot.published > 0U) {
      frame_age = std::chrono::duration<double>(now - snapshot.last_published).count();
    }
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "go2_sensors/camera";
    status.hardware_id = "unitree_go2";
    if (startup_age < startup_grace_seconds_ &&
      (!snapshot.connected || snapshot.published == 0U || !quality.ready))
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "camera quality gate in startup grace period";
    } else if (!snapshot.connected) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = snapshot.last_error.empty() ?
        "camera daemon disconnected" : snapshot.last_error.message();
    } else if (snapshot.published == 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "no valid camera frame before startup deadline";
    } else if (frame_age > stale_timeout_seconds_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "camera stream stale";
    } else if (quality.ready && quality.valid_rate_hz < min_valid_rate_hz_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "valid camera rate is below required minimum";
    } else if (quality.ready && quality.error_ratio > max_error_ratio_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "camera rejection/API error ratio exceeds limit";
    } else if (!quality.ready) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "collecting bounded camera quality window";
    } else if (quality.failures > 0U || quality.disconnects > 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "camera is usable but quality errors occurred in the active window";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "camera quality gate passed";
    }
    const bool healthy = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.values.push_back(camera_key_value(
      "age_ms", std::isfinite(frame_age) ? std::to_string(frame_age * 1000.0) : "unknown"));
    status.values.push_back(camera_key_value("socket_path", socket_path_));
    status.values.push_back(camera_key_value("image_topic", image_topic_));
    status.values.push_back(camera_key_value("frame_id", frame_id_));
    status.values.push_back(camera_key_value("image_type", "sensor_msgs/msg/Image"));
    status.values.push_back(camera_key_value("camera_info_type", "sensor_msgs/msg/CameraInfo"));
    status.values.push_back(camera_key_value(
      "image_qos", "sensor_data (best_effort, volatile, keep_last(1))"));
    status.values.push_back(camera_key_value(
      "camera_info_qos", "reliable, transient_local, keep_last(1)"));
    status.values.push_back(camera_key_value("received_count", std::to_string(snapshot.received)));
    status.values.push_back(camera_key_value("published_count", std::to_string(snapshot.published)));
    status.values.push_back(camera_key_value("rejected_count", std::to_string(snapshot.rejected)));
    status.values.push_back(camera_key_value(
      "superseded_count", std::to_string(snapshot.superseded)));
    status.values.push_back(camera_key_value(
      "pending_frame_depth", std::to_string(latest_frame_.depth())));
    status.values.push_back(camera_key_value(
      "status_record_count", std::to_string(snapshot.status_records)));
    status.values.push_back(camera_key_value(
      "connection_attempt_count", std::to_string(snapshot.connection_attempts)));
    status.values.push_back(camera_key_value(
      "connection_failure_count", std::to_string(snapshot.connection_failures)));
    status.values.push_back(camera_key_value(
      "connection_count", std::to_string(snapshot.connections)));
    status.values.push_back(camera_key_value(
      "reconnect_count", std::to_string(snapshot.reconnects)));
    status.values.push_back(camera_key_value(
      "disconnect_count", std::to_string(snapshot.disconnects)));
    status.values.push_back(camera_key_value(
      "daemon_api_request_count", std::to_string(snapshot.daemon_api_requests)));
    status.values.push_back(camera_key_value(
      "daemon_api_accepted_count", std::to_string(snapshot.daemon_api_accepted)));
    status.values.push_back(camera_key_value(
      "daemon_source_rejected_count", std::to_string(snapshot.daemon_source_rejected)));
    status.values.push_back(camera_key_value(
      "daemon_api_error_count", std::to_string(snapshot.daemon_api_errors)));
    status.values.push_back(camera_key_value(
      "daemon_ipc_connection_count", std::to_string(snapshot.daemon_ipc_connections)));
    status.values.push_back(camera_key_value(
      "daemon_ipc_disconnect_count", std::to_string(snapshot.daemon_ipc_disconnects)));
    status.values.push_back(camera_key_value(
      "daemon_counter_reset_count", std::to_string(snapshot.daemon_counter_resets)));
    status.values.push_back(camera_key_value(
      "daemon_last_api_code", std::to_string(snapshot.daemon_last_api_code)));
    status.values.push_back(camera_key_value(
      "daemon_last_api_code_semantics", "opaque vendor return code; not interpreted"));
    status.values.push_back(camera_key_value("rate_hz", std::to_string(snapshot.rate_hz)));
    status.values.push_back(camera_key_value(
      "quality_window_seconds", std::to_string(quality.elapsed_seconds)));
    status.values.push_back(camera_key_value(
      "quality_window_attempts", std::to_string(quality.attempts)));
    status.values.push_back(camera_key_value(
      "quality_window_failures", std::to_string(quality.failures)));
    status.values.push_back(camera_key_value(
      "quality_window_disconnects", std::to_string(quality.disconnects)));
    status.values.push_back(camera_key_value(
      "quality_error_ratio", std::to_string(quality.error_ratio)));
    status.values.push_back(camera_key_value(
      "min_valid_rate_hz", std::to_string(min_valid_rate_hz_)));
    status.values.push_back(camera_key_value(
      "max_error_ratio", std::to_string(max_error_ratio_)));
    status.values.push_back(camera_key_value("quality_ready", quality.ready ? "true" : "false"));
    status.values.push_back(camera_key_value("healthy", healthy ? "true" : "false"));
    status.values.push_back(camera_key_value("last_error", snapshot.last_error.message()));
    status.values.push_back(camera_key_value("calibrated", calibrated_ ? "true" : "false"));

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now();
    diagnostics.status.push_back(std::move(status));
    diagnostics_publisher_->publish(diagnostics);
  }

  std::string socket_path_;
  std::string frame_id_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string status_topic_;
  std::uint32_t max_jpeg_bytes_{camera_ipc::kDefaultMaxJpegBytes};
  int max_width_{4096};
  int max_height_{4096};
  int read_timeout_ms_{1500};
  int reconnect_delay_ms_{1000};
  int max_frame_age_ms_{2000};
  double stale_timeout_seconds_{2.0};
  double startup_grace_seconds_{5.0};
  double quality_window_seconds_{10.0};
  double min_valid_rate_hz_{1.0};
  double max_error_ratio_{0.20};
  int min_quality_samples_{5};
  std::string distortion_model_;
  std::vector<double> distortion_;
  std::array<double, 9> intrinsic_{};
  std::array<double, 9> rectification_{};
  std::array<double, 12> projection_{};
  bool calibrated_{false};

  std::atomic<bool> running_{true};
  std::atomic<std::uint64_t> active_connection_generation_{0U};
  LatestFrameMailbox latest_frame_;
  std::thread worker_;
  std::thread processor_;
  std::mutex reconnect_mutex_;
  std::condition_variable reconnect_condition_;
  std::mutex state_mutex_;
  CameraRuntimeState state_;
  std::deque<CameraQualitySample> quality_samples_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace go2_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_sensors::CameraBridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_camera_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
