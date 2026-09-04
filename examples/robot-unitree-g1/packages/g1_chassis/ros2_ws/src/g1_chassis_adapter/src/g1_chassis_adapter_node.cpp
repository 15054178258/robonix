#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <sys/un.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "g1_chassis/protocol.hpp"
using g1_chassis::CommandPacket;
using g1_chassis::PacketType;

namespace g1_chassis_adapter {

class AdapterNode : public rclcpp::Node {
 public:
  AdapterNode() : Node("g1_chassis_adapter") {
    // Resolve IPC socket path from environment.
    const char *socket_env = std::getenv("G1_IPC_SOCKET");
    if (!socket_env) {
      RCLCPP_FATAL(this->get_logger(), "G1_IPC_SOCKET environment variable not set");
      std::exit(1);
    }
    socket_path_ = std::string(socket_env);
    RCLCPP_INFO(this->get_logger(), "IPC socket: %s", socket_path_.c_str());

    // Open a non-blocking UNIX socket to the daemon.
    socket_fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "socket() failed: %s", strerror(errno));
      std::exit(1);
    }

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path_.c_str(),
                 sizeof(address.sun_path) - 1U);

    // Connect in non-blocking mode; wait up to 2s for the daemon.
    int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    int ret = ::connect(socket_fd_, reinterpret_cast<const sockaddr *>(&address),
                        sizeof(address));
    if (ret < 0 && errno == EINPROGRESS) {
      // Wait for connection to complete.
      fd_set writefds;
      FD_ZERO(&writefds);
      FD_SET(socket_fd_, &writefds);
      timeval tv{};
      tv.tv_sec = 2;
      tv.tv_usec = 0;
      if (select(socket_fd_ + 1, nullptr, &writefds, nullptr, &tv) > 0) {
        int err = 0;
        socklen_t len = sizeof(err);
        getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err == 0) {
          ret = 0;
        } else {
          ret = -1;
        }
      } else {
        ret = -1;
      }
    }
    if (ret < 0) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to connect to daemon at %s: %s",
                   socket_path_.c_str(), strerror(errno));
      std::exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "Connected to daemon");

    // Parameters.
    twist_in_topic_ = declare_parameter("twist_in_topic", "/cmd_vel");
    odom_topic_ = declare_parameter("odom_topic", "/odom");
    joint_state_topic_ = declare_parameter("joint_state_topic", "/joint_states");

    // Twist subscriber.
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        twist_in_topic_, 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          OnTwist(msg);
        });

    // Publish a dummy odometry (the real odom comes from the chassis primitive
    // via topic declaration). We just need to confirm the graph.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);

    // Broadcast odom -> base_link TF so mapping and nav2 can resolve the chain.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Periodic timer to keep the ROS graph alive and forward stale-free cmd_vel.
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { TimerTick(); });

    sequence_ = 0;
  }

  ~AdapterNode() override {
    if (socket_fd_ >= 0) ::close(socket_fd_);
  }

 private:
  CommandPacket TwistToPacket(const geometry_msgs::msg::TwistStamped &msg) {
    CommandPacket pkt{};
    pkt.type = static_cast<uint8_t>(PacketType::kCmd);
    pkt.sequence = sequence_++;
    pkt.vx = static_cast<int32_t>(msg.twist.linear.x * 10000);
    pkt.vy = static_cast<int32_t>(msg.twist.linear.y * 10000);
    pkt.omega = static_cast<int32_t>(msg.twist.angular.z * 10000);
    return pkt;
  }

  void OnTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    last_twist_ = TwistToPacket(*msg);
  }

  void TimerTick() {
    // If a twist arrived since the last tick, send it.
    if (last_twist_received_) {
      last_twist_received_ = false;
      send_command(last_twist_);
    }

    // Publish a stationary odometry heartbeat so the ROS2 graph is visible.
    // The chassis primitive declares /odom with atlas; the real odometry
    // stream will be provided by a future integration.
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    const auto now = get_clock()->now();
    PublishNeutralJointState(now);

    odom->header.stamp = now;
    odom->header.frame_id = odom_frame_;
    odom->child_frame_id = base_frame_;
    odom->pose.pose.position.x = 0.0;
    odom->pose.pose.position.y = 0.0;
    odom->pose.pose.position.z = 0.0;
    odom->pose.pose.orientation.w = 1.0;
    odom->twist.twist.linear.x = 0.0;
    odom->twist.twist.linear.y = 0.0;
    odom->twist.twist.linear.z = 0.0;
    odom->twist.twist.angular.x = 0.0;
    odom->twist.twist.angular.y = 0.0;
    odom->twist.twist.angular.z = 0.0;
    odom_pub_->publish(std::move(odom));

    // Broadcast odom -> base_link TF transform.
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);
  }

  // Publish neutral waist joints so robot_state_publisher can connect
  // base_link to torso-mounted sensors when SDK joint states are absent.
  void PublishNeutralJointState(const rclcpp::Time &stamp) {
    auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state->header.stamp = stamp;
    joint_state->name = {
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
    };
    joint_state->position = {0.0, 0.0, 0.0};
    joint_state->velocity = {0.0, 0.0, 0.0};
    joint_state->effort = {0.0, 0.0, 0.0};
    joint_state_pub_->publish(std::move(joint_state));
  }

  void send_command(const CommandPacket &cmd) {
    // Non-blocking send — if EAGAIN, drop this tick and the next one will
    // retry with updated velocity.
    ssize_t sent = ::send(socket_fd_, &cmd, sizeof(cmd), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(this->get_logger(), "send() error: %s", strerror(errno));
    }
  }

  std::string socket_path_;
  int socket_fd_{-1};

  std::string twist_in_topic_;
  std::string odom_topic_;
  std::string joint_state_topic_;
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  CommandPacket last_twist_{};
  bool last_twist_received_{false};
  uint8_t sequence_{0};
};

}  // namespace g1_chassis_adapter

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_chassis_adapter::AdapterNode>());
  rclcpp::shutdown();
  return 0;
}
