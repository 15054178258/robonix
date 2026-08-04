# scout2_chassis configuration parameters
# These fields are passed to the Driver CMD_INIT via `config:` in the
# robonix_manifest.yaml. They are for documentation purposes only;
# the driver code is responsible for actual validation.

# string, default: /odom
# ROS 2 topic name for odometry messages (nav_msgs/Odometry).
# The chassis driver subscribes to this topic and forwards readings
# through the robonix/primitive/chassis/odom capability.
odom_topic: /odom

# string, default: /cmd_vel
# ROS 2 topic name for velocity commands (geometry_msgs/Twist).
# The chassis driver subscribes to this topic and sends commands
# to the Scout2 motion controller via the robonix/primitive/chassis/twist_in
# capability.
cmd_vel_topic: /cmd_vel
