from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('op3_field_map')
    default_params = os.path.join(pkg_share, 'config', 'field_kid.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the parameter YAML file'
        ),
        
        Node(
            package='op3_field_map',
            executable='field_map_publisher',
            name='field_map_publisher',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
