# scout2_lidar configuration parameters
# Passed to the Driver CMD_INIT via `config:` in robonix_manifest.yaml.

# string, default: /scan
# ROS 2 topic for 2D laser scan messages (sensor_msgs/LaserScan).
# The driver subscribes to this topic and exposes it through the
# robonix/primitive/lidar/lidar capability.
scan_topic: /scan

# string, default: /velodyne_points
# ROS 2 topic for 3D point cloud messages (sensor_msgs/PointCloud2).
# The driver subscribes to this topic and exposes it through the
# robonix/primitive/lidar/lidar3d capability.
pointcloud_topic: /velodyne_points
