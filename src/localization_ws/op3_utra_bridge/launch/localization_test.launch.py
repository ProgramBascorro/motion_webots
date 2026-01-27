#!/usr/bin/env python3

"""
Test launch file for UTRA localization with OP3
Phase 1: Odometry-only localization (no vision)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch


def generate_launch_description():
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Webots) or real robot'
    )
    
    use_sim = LaunchConfiguration('use_sim')
    
    # 1. OP3 Bridges
    bridges_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('op3_utra_bridge'),
            'launch',
            'bridges.launch.py'
        ]),
        launch_arguments={'use_sim': use_sim}.items()
    )
    
    # 2. UTRA Soccer Localization
    localization_node = Node(
        package='soccer_localization',
        executable='soccer_localization',
        name='soccer_localization',
        output='screen',
        parameters=[{
            # Add any localization parameters here
            # e.g., 'distance_point_threshold': 5.0
        }]
    )
    
    # 3. RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('op3_utra_bridge'),
            'rviz',
            'localization.rviz'
        ])],
        condition=launch.conditions.IfCondition(
            LaunchConfiguration('use_rviz', default='true')
        )
    )
    
    return LaunchDescription([
        use_sim_arg,
        DeclareLaunchArgument('use_rviz', default_value='true'),
        bridges_launch,
        localization_node,
        # rviz_node  # Uncomment if RViz config is ready
    ])