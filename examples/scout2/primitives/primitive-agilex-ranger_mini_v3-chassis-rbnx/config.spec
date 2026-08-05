# Runtime config accepted by the Ranger Mini v3 chassis primitive.
#
# This file documents the mapping passed as the package's `config:` value in a
# robot deployment manifest. It is not loaded by the provider. Values below are
# runtime defaults. A normal Ranger Mini v3 deployment only overrides fields
# whose topic or frame ownership differs from these defaults.

config:
  # string, default: can_ranger.
  # Linux SocketCAN interface carrying Ranger chassis frames. The package
  # brings it up when it is currently down.
  port_name: can_ranger

  # integer (bit/s), default: 500000.
  # CAN bitrate used while bringing port_name up. It does not reconfigure an
  # interface that is already UP.
  can_bitrate: 500000

  # string, default: ranger_mini_v3.
  # Chassis driver selector.
  #   "ranger_mini_v3" — spawns ranger_ros2 / ranger_bringup (Ranger Mini v1/v2/v3)
  #   "scout"          — spawns scout_ros2 / scout_base (Scout / Scout 2)
  # Scout 2 shares the same CAN protocol as Scout V1 so "scout" works for both.
  # Use a different chassis provider rather than overriding this for an
  # incompatible model.
  chassis_model: ranger_mini_v3

  # string, default: odom.
  # Parent frame recorded in nav_msgs/Odometry and in the optional dynamic TF.
  odom_frame: odom

  # string, default: base_link.
  # Child frame recorded in nav_msgs/Odometry and in the optional dynamic TF.
  base_frame: base_link

  # integer (Hz), default: 50.
  # Chassis state and odometry publication rate requested from agilex_ros2.
  update_rate: 50

  # string, default: odom.
  # nav_msgs/Odometry output topic passed to agilex_ros2. Both `odom` and
  # `/odom` are accepted; the readiness check normalizes it to an absolute ROS
  # topic. Mapping and Navigation must consume the same topic.
  odom_topic_name: odom

  # boolean, default: false.
  # Ask agilex_ros2 to publish odom_frame -> base_frame. Enable exactly one
  # publisher for this dynamic TF edge in the complete robot stack.
  publish_odom_tf: false

  # string, default: /cmd_vel.
  # ROS geometry_msgs/Twist command topic consumed by the chassis driver.
  cmd_vel_topic: /cmd_vel

  # float (seconds), default: 30.0.
  # Maximum startup wait for the first Odometry message.
  sentinel_timeout_s: 30.0
