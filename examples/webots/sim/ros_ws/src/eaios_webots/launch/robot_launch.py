import os
from pathlib import Path

import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('eaios_webots')
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value=TextSubstitution(text='tiago_webots.urdf'),  # Default to the turtlebot_urdf file
        description='Path to the robot URDF file (relative to the package share directory)'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='office.wbt'),  # Default to the office.wbt file
        description='Path to the Webots world file (relative to the package share directory)'
    )
    
    robot_urdf_file = LaunchConfiguration('robot')
    world_wbt_file = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


    robot_description_path = PathJoinSubstitution([
        package_dir,
        'resource', # Assuming URDF files are in the 'resource' folder
        robot_urdf_file
    ])
    print(f"using robot_path:{robot_description_path}")

    def launch_webots(context):
        """Resolve the selected world before constructing WebotsLauncher.

        webots_ros2's Humble launcher copies a world and appends its
        Ros2Supervisor Robot. When handed a PathJoinSubstitution it creates
        that temporary copy but starts the original path instead, leaving the
        supervisor process disconnected. Supplying the resolved absolute path
        makes the launcher's temporary world authoritative.
        """
        world_name = world_wbt_file.perform(context)
        if not world_name or Path(world_name).name != world_name:
            raise RuntimeError(
                "world must be a .wbt filename inside the eaios_webots "
                "worlds directory"
            )
        world_path = Path(package_dir, "worlds", world_name)
        if world_path.suffix != ".wbt":
            raise RuntimeError(f"invalid Webots world selection: {world_name!r}")
        if not world_path.is_file():
            raise RuntimeError(f"Webots world does not exist: {world_path}")
        # With colcon --symlink-install each installed world file may resolve
        # back into the source tree while its containing ``worlds`` directory
        # remains under ``install/``. The basename-only check above already
        # prevents traversal, so do not reject that legitimate file symlink by
        # comparing resolved parents.
        resolved_world_path = world_path.resolve()
        print(f"using world_path:{resolved_world_path}")

        # WEBOTS_STREAM=1 enables Webots' built-in WebSocket stream on port
        # 1234. Port isolation is handled by the container network.
        webots = WebotsLauncher(
            world=str(resolved_world_path),
            mode="realtime",
            ros2_supervisor=True,
            stream=os.environ.get('WEBOTS_STREAM', '0') == '1',
        )
        return [
            webots,
            webots._supervisor,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.EmitEvent(
                            event=launch.events.Shutdown()
                        )
                    ],
                )
            ),
        ]
    
    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]
    

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )


    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/wheel_odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/wheel_odom')]
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    my_robot_driver = WebotsController(
        robot_name='my_robot', # Ensure this name matches the robot node name in the Webots world.
        parameters=[
            {'robot_description': robot_description_path, # This robot_description is typically for the Webots driver.
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True}, # Set to True if WebotsController should launch its own RSP.
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=my_robot_driver,
        nodes_to_start=ros_control_spawners
    )

    odom_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(package_dir, 'resource', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/odometry/filtered', '/odom')],
    )

    return LaunchDescription([
        robot_arg,
        world_arg,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=TextSubstitution(text='True'),
            description='Use simulation (Webots) clock if true'
        ),

        OpaqueFunction(function=launch_webots),
        robot_state_publisher, # Ensure robot_state_publisher starts before my_robot_driver if my_robot_driver depends on it.
        footprint_publisher,
        my_robot_driver,
        waiting_nodes,
        odom_filter,
    ])

