========================================
 Scout2 主机检查命令清单
========================================

请将以下命令逐条在 Scout2 主控机上执行。
每条命令执行后，将输出信息回复给部署助手。
----------------------------------------

【1】系统环境检查
----------------------------------------

uname -a
echo "ROS_DISTRO: $ROS_DISTRO"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
docker --version
docker compose version
which rbnx


【2】ROS2 话题检查（必须执行，source 后运行）
----------------------------------------

source /opt/ros/humble/setup.bash
ros2 topic list


【3】ROS2 话题类型检查
----------------------------------------

source /opt/ros/humble/setup.bash
ros2 topic type /odom
ros2 topic type /cmd_vel
ros2 topic type /scan
ros2 topic type /velodyne_points
ros2 topic type /camera/camera/color/image_raw
ros2 topic type /camera/camera/depth/image_rect_raw
ros2 topic type /camera/camera/color/camera_info


【4】Velodyne 3D 雷达信息检查
----------------------------------------

source /opt/ros/humble/setup.bash
ros2 topic info /velodyne_points

# 查看点云消息前几行
source /opt/ros/humble/setup.bash
ros2 topic echo /velodyne_points --once -r1 | head -20

# 查看运行中的 velodyne 相关节点
source /opt/ros/humble/setup.bash
ros2 node list | grep velodyne

# 检查 USB 设备
lsusb | grep -i velo

# 查看系统日志
dmesg | grep -i velo


【5】相机信息检查
----------------------------------------

source /opt/ros/humble/setup.bash
ros2 topic info /camera/camera/color/image_raw

# 查看相机驱动节点
source /opt/ros/humble/setup.bash
ros2 node list | grep realsense

# 查看 camera_info 内容
source /opt/ros/humble/setup.bash
ros2 topic echo /camera/camera/color/camera_info --once


【6】底盘驱动状态检查
----------------------------------------

source /opt/ros/humble/setup.bash
ros2 topic hz /odom

source /opt/ros/humble/setup.bash
ros2 topic type /odom

# 查看 odom 消息内容
source /opt/ros/humble/setup.bash
ros2 topic echo /odom --once -r1 | head -10

# 查看底盘相关节点
source /opt/ros/humble/setup.bash
ros2 node list | grep -i scout
ros2 service list


【7】TF 变换检查
----------------------------------------

source /opt/ros/humble/setup.bash
ros2 run tf2_tools view_frames.py -o /tmp/tf_output.pdf 2>&1

# 或者用命令检查
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

