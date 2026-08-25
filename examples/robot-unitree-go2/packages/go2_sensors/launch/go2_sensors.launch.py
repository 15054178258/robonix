import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("go2_sensors"))
    default_config = str(package_share / "config" / "go2_sensors.yaml")

    config = LaunchConfiguration("config")
    enable_camera = LaunchConfiguration("enable_camera")
    socket_path = LaunchConfiguration("camera_socket")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument(
                "camera_socket",
                default_value=f"/run/user/{os.getuid()}/robonix-go2/camera.sock",
            ),
            Node(
                package="go2_sensors",
                executable="go2_sensor_relay",
                name="go2_sensor_relay",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="go2_sensors",
                executable="go2_camera_bridge",
                name="go2_camera_bridge",
                output="screen",
                condition=IfCondition(enable_camera),
                parameters=[config, {"camera.socket_path": socket_path}],
            ),
        ]
    )
