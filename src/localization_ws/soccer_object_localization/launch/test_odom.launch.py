#!/usr/bin/env python3
"""
test_jointodom.launch.py
Launch file khusus testing joint odometry — tanpa AMCL, tanpa detector.
Tujuan: verifikasi apakah joint odom bisa tracking posisi robot dengan benar.

Node yang berjalan:
  1. static_tf_publisher  — TF tree robot
  2. static_odom_baselink — TF odom → base_link (awal, identity)
  3. joint_odom_node      — odometri dari joint kaki

Topics yang dipantau di RViz:
  /odom       → Odometry display
  /tf         → TF display
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # ── Static TF robot (base_link → head_link → cam_link) ──
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )

    # ── Static TF odom → base_link (identity, sementara) ────
    static_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_baselink',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # ── Joint Odom Node ──────────────────────────────────────
    joint_odom_node = Node(
        package='soccer_object_localization',
        executable='joint_odom_node',
        name='joint_odom_node',
        output='screen',
        parameters=[{
            'base_frame':      'base_link',
            'odom_frame':      'odom',
            'use_imu_yaw':     True,
            'pose_cov_xy':     0.15,
            'pose_cov_yaw':    0.1,
            'smoothing_alpha': 0.5,
            'step_scale':      0.8,
            'dual_stance_avg': True,
        }]
    )

    delayed_jointodom = TimerAction(period=2.0, actions=[joint_odom_node])

    return LaunchDescription([
        static_tf_publisher,
        static_odom_baselink,
        delayed_jointodom,
    ])