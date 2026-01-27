#!/usr/bin/env python3
"""
Launch field line detector in SIMULATOR mode
Uses static camera pose (fast, simple)
FIXED: Uses detector_fieldline_hybrid executable
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for simulator mode"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('soccer_object_localization')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'op3_sim.yaml')
    
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace (empty for single robot)'
    )
    
    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold',
        default_value='210',
        description='White detection threshold (0-255)'
    )
    
    spacing_arg = DeclareLaunchArgument(
        'spacing',
        default_value='30',
        description='Point cloud sampling spacing (pixels)'
    )
    
    # Field line detector node
    # FIXED: Use detector_fieldline_hybrid, not detector_fieldline_ros
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_hybrid',  # NEW HYBRID NODE!
        name='detector_fieldline',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            config_file,
            {
                # Override parameters from launch args
                'use_dynamic_tf': False,  # STATIC mode for simulator
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'point_cloud.spacing': LaunchConfiguration('spacing'),
            }
        ],
        remappings=[
            # Remap to OP3 Webots topics
            ('/camera/image_raw', '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        white_threshold_arg,
        spacing_arg,
        detector_node,
    ])