#!/usr/bin/env python3
"""
Full Soccer Demo for OP3
Complete integration: Localization + Vision + Behavior
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation or real robot'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Get package directories
    op3_utra_bridge_dir = get_package_share_directory('op3_utra_bridge')
    
    # Include localization with vision
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(op3_utra_bridge_dir, 'launch', 'localization_with_vision.launch.py')
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'use_camera': 'true'
        }.items()
    )
    
    # Object detection (ball, goal posts, etc.)
    object_detection_node = Node(
        package='soccer_object_detection',
        executable='object_detect_node',
        name='object_detection',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim'),
            'camera_topic': '/camera/image',
            'model_path': 'best.pt',  # YOLO model
            'confidence_threshold': 0.5,
            'classes': ['ball', 'goalpost', 'robot'],
        }]
    )
    
    # RViz for visualization
    rviz_config = os.path.join(
        op3_utra_bridge_dir,
        'rviz',
        'full_soccer_demo.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Path publisher for trajectory visualization
    path_publisher = Node(
        package='op3_utra_bridge',
        executable='odom_path_publisher',
        name='odom_path_publisher'
    )
    
    return LaunchDescription([
        use_sim_arg,
        use_rviz_arg,
        localization_launch,
        object_detection_node,
        rviz_node,
        path_publisher,
    ])