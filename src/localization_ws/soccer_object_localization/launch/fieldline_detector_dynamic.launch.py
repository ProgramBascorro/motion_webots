#!/usr/bin/env python3
"""
Launch field line detector in DYNAMIC mode
Uses TF transforms for accurate camera tracking
Works with Webots simulation (moving head/camera)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for dynamic TF mode"""
    
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
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='cam_link',
        description='Camera TF frame name (check with: ros2 topic echo /tf)'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base TF frame name'
    )
    
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='odom',
        description='World/fixed TF frame name'
    )
    
    # Field line detector node - DYNAMIC MODE
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_hybrid',
        name='detector_fieldline',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            config_file,
            {
                # DYNAMIC MODE!
                'use_dynamic_tf': True,
                
                # Override from launch args
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'point_cloud.spacing': LaunchConfiguration('spacing'),
                
                # Frame names
                'frames.camera': LaunchConfiguration('camera_frame'),
                'frames.base': LaunchConfiguration('base_frame'),
                'frames.world': LaunchConfiguration('world_frame'),
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
        camera_frame_arg,
        base_frame_arg,
        world_frame_arg,
        detector_node,
    ])