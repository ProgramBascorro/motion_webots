#!/usr/bin/env python3
"""
Minimal debugging launch - Step by step testing
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation'
    )
    
    # Just the essentials for testing
    
    # 1. Odometry bridge (publishes /odom_combined)
    odom_bridge = Node(
        package='op3_utra_bridge',
        executable='odom_bridge_node',
        name='odom_bridge_debug',
        output='screen'
    )
    
    # 2. Static TF: world -> odom
    static_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'odom'],
        output='screen'
    )
    
    # 3. Odometry to TF (converts /odom_combined to TF)
    odom_to_tf = Node(
        package='op3_utra_bridge',
        executable='odom_to_tf.py',
        name='odom_to_tf_debug',
        parameters=[{
            'odom_topic': '/odom_combined',
            'parent_frame': 'odom',
            'child_frame': 'base'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_arg,
        odom_bridge,
        static_world_odom,
        odom_to_tf,
    ])