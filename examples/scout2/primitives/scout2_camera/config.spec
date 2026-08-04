# scout2_camera configuration parameters
# Passed to the Driver CMD_INIT via `config:` in robonix_manifest.yaml.

# string, default: /camera/camera/color/image_raw
# ROS 2 topic for RGB image stream (sensor_msgs/Image).
rgb_topic: /camera/camera/color/image_raw

# string, default: /camera/camera/depth/image_rect_raw
# ROS 2 topic for rectified depth image (sensor_msgs/Image, 32FC1).
depth_topic: /camera/camera/depth/image_rect_raw

# string, default: /camera/camera/color/camera_info
# ROS 2 topic for camera intrinsics (sensor_msgs/CameraInfo).
camera_info_topic: /camera/camera/color/camera_info

# string, optional
# ROS 2 topic for depth-to-color extrinsics (geometry_msgs/TransformStamped).
# If omitted, extrinsics must come from TF/URDF instead.
extrinsics_topic: /camera/camera/extrinsics/depth_to_color

# string, default: base_link
# Frame ID of the robot base used as the parent frame for depth extrinsics.
base_frame: base_link

# The following fields are used as intrinsics fallback when no CameraInfo
# topic is found on the ROS graph. Real CameraInfo on camera_info_topic is
# always preferred at runtime.

# integer, default: 640
# Image width in pixels (used only as CameraInfo fallback).
width: 640

# integer, default: 480
# Image height in pixels (used only as CameraInfo fallback).
height: 480

# float, default: 605.77
# Focal length in pixels along x-axis (fx).
fx: 605.77

# float, default: 605.39
# Focal length in pixels along y-axis (fy).
fy: 605.39

# float, default: 329.25
# Principal point x coordinate in pixels (cx).
cx: 329.25

# float, default: 259.08
# Principal point y coordinate in pixels (cy).
cy: 259.08
