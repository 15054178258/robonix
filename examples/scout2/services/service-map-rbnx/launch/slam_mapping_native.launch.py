"""Native mapping launch. Uses /tmp/lio_resolved.yaml (written by atlas_bridge
after Atlas-discovered primitives) if present, else installed default."""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pc2grid = os.path.join(pkg_dir, 'scripts', 'pointcloud_to_grid.py')
    scan_pub = os.path.join(pkg_dir, 'scripts', 'scan_publisher.py')
    cloud_acc = os.path.join(pkg_dir, 'scripts', 'cloud_accumulator.py')

    ctx = LaunchContext()
    resolved = '/tmp/lio_resolved.yaml'
    if os.path.exists(resolved):
        lio_cfg = resolved
    else:
        lio_cfg = PathJoinSubstitution([FindPackageShare('fastlio2'),
                                         'config', 'lio.yaml']).perform(ctx)
    pgo_cfg = PathJoinSubstitution([FindPackageShare('pgo'),
                                     'config', 'pgo.yaml']).perform(ctx)

    return LaunchDescription([
        Node(package='fastlio2', namespace='fastlio2', executable='lio_node',
             name='lio_node', output='screen',
             parameters=[{'config_path': lio_cfg}]),
        Node(package='pgo', namespace='pgo', executable='pgo_node',
             name='pgo_node', output='screen',
             parameters=[{'config_path': pgo_cfg}]),
        Node(executable=pc2grid, name='pointcloud_to_grid', output='screen'),
        Node(executable=scan_pub, name='scan_publisher', output='screen'),
        Node(executable=cloud_acc, name='cloud_accumulator', output='screen'),
    ])
