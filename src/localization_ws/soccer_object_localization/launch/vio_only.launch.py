#!/usr/bin/env python3
"""
vio_only.launch.py - VIO Standalone tanpa AMCL
Untuk testing akurasi VIO secara terpisah.

Topics yang bisa di-monitor di RViz:
  /odom          - Odometry (tambahkan Odometry display)
  /vio_pose      - PoseWithCovariance (tambahkan PoseWithCovariance display)
  /vio_path      - Path trajektori robot (tambahkan Path display)
  /tf            - TF tree: map → odom → cam_link

TF chain:
  map (static) → odom → cam_link  [dari VIO]
               ↑
  base_link → head_link → cam_link [dari static_tf_publisher]
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_loc = get_package_share_directory('soccer_object_localization')

    # ── t=0s: Static TF ─────────────────────────────────────
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )

    # ── t=2s: VIO Node ──────────────────────────────────────
    vio_node = Node(
        package='soccer_object_localization',
        executable='vio_node_sim',
        name='vio_node',
        output='screen',
        parameters=[{
            # Frame
            'base_frame':   'cam_link',
            'odom_frame':   'odom',
            'map_frame':    'map',

            # Camera
            'focal_length':       900.0,
            'camera_height':      0.475,
            'camera_tilt':       -0.349,
            'orig_image_width':   1280,
            'orig_image_height':  720,

            # Smoothing — lebih responsif dari v3 (0.85 → 0.4)
            'smoothing_alpha':    0.4,
            'min_visual_motion':  0.008,
            'motion_threshold':   0.005,

            # VP yaw
            'use_vp_yaw':        True,
            'vp_every_n_frames': 5,

            # Covariance — lebih ketat karena standalone (tidak ada AMCL koreksi)
            'pose_cov_xy':   0.3,
            'pose_cov_yaw':  0.15,

            # Path publisher
            'publish_path':  True,
            'path_max_poses': 300,
        }],
        remappings=[
            ('/robotis_op3/camera/image_raw', '/robotis_op3/camera/image_raw'),
            ('/imu/data', '/imu/data'),
        ]
    )

    delayed_vio = TimerAction(period=2.0, actions=[vio_node])

    return LaunchDescription([
        static_tf_publisher,
        delayed_vio,
    ])