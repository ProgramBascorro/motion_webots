from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_name = 'utrabot_vision'
    pkg_share_dir = get_package_share_directory(pkg_name)

    camera_cfg = PathJoinSubstitution([pkg_share_dir, 'config', 'camera.yaml'])

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_cfg': camera_cfg,
                'use_sim_time': True
            }],
        ),
        Node(
            package=pkg_name,
            executable='camera_processing',
            name='camera_processing',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])
