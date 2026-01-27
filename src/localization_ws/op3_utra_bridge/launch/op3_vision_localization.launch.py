#!/usr/bin/env python3
"""
Complete OP3 Vision-Based Localization Launch
Includes: Webots, TF tree, camera, field line detection

Usage:
  ros2 launch op3_utra_bridge op3_vision_localization.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package directories
    op3_webots_dir = get_package_share_directory('op3_webots_ros2')
    obj_loc_dir = get_package_share_directory('soccer_object_localization')
    
    # Arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use Webots simulation'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Show Webots GUI'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    # 1. Launch Webots with OP3
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(op3_webots_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
        }.items()
    )
    
    # 2. OP3 Static TF Tree
    tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='op3_tf_publisher',
        output='screen',
        parameters=[{
            'robot_name': 'op3',
            'camera_height': 0.475,
            'camera_tilt': -20.0,
        }]
    )
    
    # 3. Field Line Detector
    fieldline_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(obj_loc_dir, 'launch', 'fieldline_detector.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': 'op3',
            'camera_image_topic': '/robotis_op3/camera/image_raw',
            'camera_info_topic': '/robotis_op3/camera/camera_info',
        }.items()
    )
    # 4. RViz (optional)
    rviz_config = os.path.join(
        get_package_share_directory('op3_utra_bridge'),
        'rviz',
        'vision_localization.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=LaunchConfiguration('rviz')
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_arg,
        gui_arg,
        rviz_arg,
        
        # Nodes
        webots_launch,
        tf_publisher,
        fieldline_detector,
        # rviz_node,  # Uncomment when RViz config is ready
    ])