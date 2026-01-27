from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('op3_ball_localization')
    config_file = os.path.join(pkg_share, 'config', 'simple_ball_tracker.yaml')

    return LaunchDescription([
        Node(
            package='op3_ball_localization',
            executable='simple_ball_tracker',
            name='simple_ball_tracker',
            output='screen',
            parameters=[config_file]
        )
    ])
