#!/usr/bin/env python3
"""
kf_odom_only.launch.py — Fase 1 bersih

Stack:
  legged_odometry_kf_node → TF odom→base_link + /odom  (100Hz)
  map_server               → /map (visual RViz)
  pose_relay_node          → TF map→odom dari 2D Pose Estimate

TF tree yang terbentuk:
  map → odom → base_link → head_link → cam_link
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg  = get_package_share_directory('soccer_object_localization')
    map_file = os.path.join(pkg, 'maps', 'soccer_field.yaml')

    # ── Static TF (head_link, cam_link) ──────────────────────
    static_tf = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )

    # ── Map server ────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'frame_id': 'map'}]
    )
    lifecycle_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}]
    )

    # ── Legged Odometry KF (CORE) ─────────────────────────────
    kf_odom = Node(
        package='soccer_object_localization',
        executable='legged_odometry_kf_node',
        name='legged_odometry_kf',
        output='screen',
        parameters=[{
            'base_frame':       'base_link',
            'odom_frame':       'odom',
            'publish_rate':     100.0,
            'stance_l_min':     1.0,
            'stance_r_min':     1.0,
            'flat_ground_mode': True,
            'simulation_mode':  True,   # False untuk robot fisik
            # Process noise — tuning setelah verifikasi TF
            'q_pos':      0.001,
            'q_vel':      0.01,
            'q_foot':     0.001,
            'q_foot_sw':  10.0,
            # Measurement noise
            'r_contact':  0.001,
            'r_swing':    100.0,
        }]
    )

    # ── Pose Relay: RViz 2D Pose Estimate → TF map→odom ──────
    pose_relay = Node(
        package='soccer_object_localization',
        executable='pose_relay_node',
        name='pose_relay_node',
        output='screen',
        parameters=[{'tf_publish_rate': 20.0}]
    )

    return LaunchDescription([
        static_tf,
        map_server,
        TimerAction(period=1.0, actions=[lifecycle_map]),
        TimerAction(period=2.0, actions=[kf_odom, pose_relay]),
    ])