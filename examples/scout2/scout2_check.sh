#!/bin/bash
########################################
# Scout2 主机环境检查脚本
# 使用方法：
#   1. 在 Scout2 主控机上运行 bash scout2_check.sh
#   2. 将输出结果回复给部署助手
#   3. 助手会帮你分析环境是否就绪
########################################

echo "=========================================="
echo " Scout2 主机环境检查 - $(date)"
echo "=========================================="

echo ""
echo "【1】系统环境"
echo "------------------------------------------"
uname -a
echo "ROS_DISTRO: $ROS_DISTRO"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
docker --version 2>/dev/null || echo "Docker: NOT INSTALLED"
docker compose version 2>/dev/null || echo "Docker Compose: NOT INSTALLED"
which rbnx 2>/dev/null || echo "rbnx CLI: NOT INSTALLED"

echo ""
echo "【2】ROS2 话题列表"
echo "------------------------------------------"
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "ERROR: Cannot source ROS2 humble setup.bash"; exit 1; }
ros2 topic list 2>/dev/null

echo ""
echo "【3】关键话题类型"
echo "------------------------------------------"
ros2 topic type /odom 2>/dev/null && echo " /odom: OK" || echo " /odom: MISSING"
ros2 topic type /cmd_vel 2>/dev/null && echo " /cmd_vel: OK" || echo " /cmd_vel: MISSING"
ros2 topic type /scan 2>/dev/null && echo " /scan: OK" || echo " /scan: MISSING"
ros2 topic type /velodyne_points 2>/dev/null && echo " /velodyne_points: OK" || echo " /velodyne_points: MISSING"
ros2 topic type /camera/camera/color/image_raw 2>/dev/null && echo " /camera/camera/color/image_raw: OK" || echo " /camera/.../image_raw: MISSING"
ros2 topic type /camera/camera/depth/image_rect_raw 2>/dev/null && echo " /camera/camera/depth/image_rect_raw: OK" || echo " /camera/.../depth: MISSING"

echo ""
echo "【4】Velodyne 雷达信息"
echo "------------------------------------------"
echo "--- /velodyne_points info ---"
ros2 topic info /velodyne_points 2>/dev/null || echo "  话题不存在"
echo ""
echo "--- ROS2 node list (velodyne) ---"
ros2 node list 2>/dev/null | grep -i velodyne || echo "  无 velodyne 相关节点"
echo ""
echo "--- USB 设备 (velodyne) ---"
lsusb 2>/dev/null | grep -i velo || echo "  无 velodyne 设备"
echo ""
echo "--- dmesg (velo)---"
dmesg 2>/dev/null | grep -i -E "velo|lidar" | tail -5 || echo "  无相关日志"

echo ""
echo "【5】相机信息"
echo "------------------------------------------"
echo "--- Realsense 节点 ---"
ros2 node list 2>/dev/null | grep -i realsense || echo "  无 realsense 节点"
echo ""
echo "--- camera_info 前3行 ---"
ros2 topic echo /camera/camera/color/camera_info --once 2>/dev/null | head -3 || echo "  camera_info 话题不可读"

echo ""
echo "【6】底盘（odom）状态"
echo "------------------------------------------"
echo "--- /odom 频率 ---"
ros2 topic hz /odom 2>/dev/null | head -2 || echo "  /odom 话题不存在"
echo ""
echo "--- /odom 消息前2行 ---"
ros2 topic echo /odom --once -r1 2>/dev/null | head -2 || echo "  /odom 消息不可读"
echo ""
echo "--- 底盘相关节点 ---"
ros2 node list 2>/dev/null | grep -i -E "scout|chassis|base" || echo "  无 scout/chassis 节点"
echo ""
echo "--- 可用服务 ---"
ros2 service list 2>/dev/null | head -10 || echo "  无服务"

echo ""
echo "【7】TF 变换摘要"
echo "------------------------------------------"
echo "--- TF 广播者 ---"
ros2 run tf2_ros tf2_echo base_link base_link 2>&1 | head -1
ros2 bag info 2>/dev/null
echo ""
echo "--- 所有话题总览完毕 ---"
echo ""
echo "=========================================="
echo " 检查完成。请将以上内容复制给部署助手。"
echo "=========================================="
