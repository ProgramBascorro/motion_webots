#!/usr/bin/env python3
"""
Launch field line detector in REAL ROBOT mode
Uses dynamic camera pose via TF tree (accurate, robust)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('soccer_object_localization')
    
    # Config file
    config_file = os.path.join(pkg_dir, 'config', 'op3_real.yaml')
    
    # Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )
    
    # Field line detector node
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_hybrid',
        name='detector_fieldline',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        camera_topic_arg,
        detector_node,
    ])