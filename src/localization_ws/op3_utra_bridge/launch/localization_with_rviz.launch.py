#!/usr/bin/env python3
"""
Launch file for OP3-UTRA localization with RViz2 visualization
FINAL FIX VERSION:
1. Explicit joint_states_topic parameter for odom_bridge
2. Correct remapping for robot_state_publisher
3. All nodes with use_sim_time parameter
4. Added debug logging
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',  # ✅ Default true for Webots
        description='Use simulation (Webots) or real robot'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 for visualization'
    )
    
    # Get package directories
    op3_description_dir = get_package_share_directory('op3_description')
    
    # URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('op3_description'),
        'urdf',
        'robotis_op3.urdf.xacro'
    ])
    
    # Process xacro to URDF
    robot_description = Command(['xacro ', urdf_file])
    
    # RViz config
    rviz_config_file = os.path.join(op3_description_dir, 'rviz', 'op3.rviz')
    
    # ===== BRIDGE NODES =====
    
    # ✅ FIXED: Odometry bridge with EXPLICIT joint_states_topic parameter
    odom_bridge_node = Node(
        package='op3_utra_bridge',
        executable='odom_bridge_node',
        name='odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim'),
            'joint_states_topic': '/robotis/present_joint_states'  # ✅ EXPLICIT!
        }]
    )
    
    # IMU bridge
    imu_bridge_node = Node(
        package='op3_utra_bridge',
        executable='imu_bridge.py',
        name='imu_bridge',
        parameters=[{
            'use_sim': LaunchConfiguration('use_sim'),
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        output='screen'
    )
    
    # ===== TF PUBLISHERS =====
    
    # Static TF: world -> odom
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        output='screen'
    )
    
    # ✅ FIXED: Robot State Publisher with CORRECT remapping
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_urdf',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        remappings=[
            # ✅ For BOTH Webots simulation AND real robot
            ('/joint_states', '/robotis/present_joint_states')
        ],
        output='screen'
    )
    
    # ===== VISUALIZATION =====
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Path publisher
    path_publisher_node = Node(
        package='op3_utra_bridge',
        executable='odom_path_publisher.py',
        name='odom_path_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        output='screen'
    )
    
    # ✅ FIXED: Odometry to TF converter
    odom_to_tf_node = Node(
        package='op3_utra_bridge',
        executable='odom_to_tf.py',
        name='odom_to_tf_publisher',
        parameters=[{
            'odom_topic': '/odom_combined',
            'parent_frame': 'odom',
            'child_frame': 'base',  # ✅ Correct frame for OP3
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_arg,
        use_rviz_arg,
        LogInfo(msg='========================================'),
        LogInfo(msg='Starting OP3-UTRA Bridge with RViz'),
        LogInfo(msg='Joint states: /robotis/present_joint_states'),
        LogInfo(msg='Odometry: /odom_combined'),
        LogInfo(msg='TF: odom -> base'),
        LogInfo(msg='========================================'),
        odom_bridge_node,
        imu_bridge_node,
        static_tf_world_odom,
        robot_state_publisher_node,
        rviz_node,
        path_publisher_node,
        odom_to_tf_node,
    ])