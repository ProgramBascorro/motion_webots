#!/usr/bin/env python3

"""
Launch file for OP3-UTRA bridges
Starts odometry, IMU, and robot state publishers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation topics (robotis_op3/*) instead of real robot (robotis/*)'
    )
    
    # Get launch configuration
    use_sim = LaunchConfiguration('use_sim')
    
    # Odometry Bridge Node (C++)
    odom_bridge_node = Node(
        package='op3_utra_bridge',
        executable='odom_bridge_node',
        name='odom_bridge',
        output='screen',
        parameters=[{
            'use_sim': use_sim
        }]
    )
    
    # Robot State Publisher (Python)
    robot_state_node = Node(
        package='op3_utra_bridge',
        executable='robot_state_publisher.py',
        name='robot_state_publisher',
        output='screen'
    )
    
    # IMU Bridge (Python)
    imu_bridge_node = Node(
        package='op3_utra_bridge',
        executable='imu_bridge.py',
        name='imu_bridge',
        output='screen',
        parameters=[{
            'use_sim': use_sim
        }]
    )
    
    return LaunchDescription([
        use_sim_arg,
        odom_bridge_node,
        robot_state_node,
        imu_bridge_node
    ])