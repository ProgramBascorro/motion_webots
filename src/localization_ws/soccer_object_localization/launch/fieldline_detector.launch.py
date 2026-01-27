#!/usr/bin/env python3
"""
OP3 Field Line Detector Launch File
Simple version - no dynamic path configuration
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for field line detector"""
    
    # Get package directory at generation time
    pkg_dir = get_package_share_directory('soccer_object_localization')
    
    # Build config path (hardcoded for now)
    config_file = os.path.join(pkg_dir, 'config', 'op3_sim.yaml')
    
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation time'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='op3',
        description='Robot name'
    )
    
    # Field line detector node
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_ros',
        name='fieldline_detector',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim'),
                'robot_name': LaunchConfiguration('robot_name'),
            }
        ],
        remappings=[
            ('/camera/image_raw', '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        use_sim_arg,
        robot_name_arg,
        detector_node,
    ])