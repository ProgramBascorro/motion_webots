#!/usr/bin/env python3
"""
amcl_jointodom.launch.py v10 — Clean AMCL (Opsi C)

Perubahan dari v9:
  - HAPUS odom_to_amcl_node sepenuhnya
  - HAPUS set_initial_pose dari AMCL
  - joint_odom_node v5: publish /odom setiap frame (20Hz)
  - User set pose SEKALI via RViz 2D Pose Estimate
  - AMCL bekerja normal: odom=motion model, scan=sensor model

Alur kerja:
  1. Launch → AMCL aktif tapi partikel belum ter-set
  2. User klik 2D Pose Estimate di RViz → /initialpose → AMCL scatter partikel
  3. AMCL: terima /odom update 20Hz → gerakkan partikel sesuai motion model
  4. AMCL: terima /field_scan 20Hz → koreksi partikel sesuai peta
  5. Partikel konvergen → /amcl_pose akurat mengikuti robot

Parameter AMCL yang disesuaikan untuk lapangan sepakbola:
  - alpha kecil: odom cukup dipercaya (tidak ada roda, tapi IMU membantu)
  - laser_z_hit tinggi: scan field line cukup informatif
  - min/max_particles: cukup untuk lapangan yang punya pola berulang
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_loc     = get_package_share_directory('soccer_object_localization')
    config_file = os.path.join(pkg_loc, 'config', 'op3_sim.yaml')

    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold', default_value='165')

    # ════════════════════════════════════════════════════════
    # t = 0s
    # ════════════════════════════════════════════════════════

    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )

    # TIDAK ADA static_odom_baselink lagi
    # TF odom→base_link sekarang di-publish oleh joint_odom_node
    # Kalau ada dua publisher → TF conflict

    map_file = os.path.join(pkg_loc, 'maps', 'soccer_field.yaml')
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'topic_name':    'map',
            'frame_id':      'map'
        }]
    )

    # ════════════════════════════════════════════════════════
    # t = 1s: Lifecycle Map
    # ════════════════════════════════════════════════════════

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}]
    )
    delayed_lifecycle_map = TimerAction(period=1.0, actions=[lifecycle_manager_map])

    # ════════════════════════════════════════════════════════
    # t = 2s: Joint Odom v5
    # Publish /odom setiap frame (20Hz) — KUNCI untuk AMCL
    # ════════════════════════════════════════════════════════

    joint_odom_node = Node(
        package='soccer_object_localization',
        executable='joint_odom_node',
        name='joint_odom_node',
        output='screen',
        parameters=[{
            'base_frame':     'base_link',
            'odom_frame':     'odom',
            'use_imu_yaw':    True,
            'step_scale':      0.8,
            'stance_l_min':    1.0,
            'stance_r_min':    1.0,
            # Covariance — tuning untuk AMCL
            # Nilai ini mengontrol seberapa besar partikel menyebar
            # saat robot bergerak (motion noise model)
            'cov_xy':          0.05,
            'cov_yaw':         0.02,
            'publish_rate':    10.0,   # dikurangi dari 20Hz untuk hemat CPU
        }]
    )

    # ════════════════════════════════════════════════════════
    # t = 3s: Detector + PC2Scan + Particle converter
    # ════════════════════════════════════════════════════════

    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_enhanced2',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                'use_dynamic_tf':            False,
                'camera.height':             0.475,
                'camera.tilt':              -0.349,
                'camera.offset_x':           0.08,
                'camera.offset_y':           0.0,
                'camera.focal_length':       900.0,
                'camera.image_width':        1280,
                'camera.image_height':       720,
                'frames.camera':             'cam_link',
                'frames.base':               'cam_link',
                'frames.world':              'odom',
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'detection.use_enhanced':    True,
                'detection.roi_top_cut':     0.35,
                'detection.roi_bottom_cut':  0.08,
                'detection.min_line_length': 15,
                'detection.max_line_gap':    25,
                'detection.canny_low':       60,
                'detection.canny_high':      180,
                'detection.hough_threshold': 60,
                'detection.remove_grass':    True,
                'detection.grass_h_low':     35,
                'detection.grass_h_high':    85,
                'detection.grass_s_low':     40,
                'point_cloud.spacing':       12,
                'point_cloud.max_distance':  5.5,
                'point_cloud.min_points':    5,
                'publish.debug_image':       False,
                'publish.point_cloud':       True,
            }
        ],
        remappings=[
            ('/camera/image_raw',   '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )

    simple_pc2scan_node = Node(
        package='soccer_object_localization',
        executable='simple_pc2scan',
        name='simple_pc2scan',
        output='screen',
        parameters=[{
            # Kurangi resolusi scan untuk hemat CPU
            # angle_increment lebih besar = lebih sedikit beam
            'angle_min':       -3.14159,
            'angle_max':        3.14159,
            'angle_increment':  0.0349,  # 2° per beam (dari 1°) → 2x lebih sedikit
            'range_min':        0.2,
            'range_max':        4.0,
            'scan_height':      0.05,    # sedikit toleransi height
            'target_frame':     'odom',
        }]
    )

    particle_converter_node = Node(
        package='soccer_object_localization',
        executable='particle_converter',
        name='particle_converter',
        output='screen',
    )

    # ════════════════════════════════════════════════════════
    # t = 5s: AMCL — Clean Configuration
    # Tidak ada set_initial_pose → user yang set via RViz
    # ════════════════════════════════════════════════════════

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'odom_frame_id':   'odom',
            'base_frame_id':   'cam_link',
            'global_frame_id': 'map',
            'scan_topic':      'field_scan',

            # ── ANTI-CRASH: kurangi partikel dan beams ─────
            # 3000×60×20Hz = 3.6M ops/s → crash laptop
            # 500×30×10Hz  = 150K ops/s → jauh lebih ringan
            'min_particles': 200,
            'max_particles': 800,

            # Matikan recovery injection — penyebab scatter ulang
            # Saat scan noisy, AMCL recovery akan inject partikel baru
            # di seluruh peta → partikel hilang konvergensi
            # Dengan nilai 0: tidak ada recovery injection
            'recovery_alpha_slow': 0.0,
            'recovery_alpha_fast': 0.0,

            # Motion model
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.05,
            'alpha2': 0.05,
            'alpha3': 0.02,
            'alpha4': 0.02,
            'alpha5': 0.05,

            # Update threshold — lebih jarang update = lebih ringan
            'update_min_d':      0.03,   # 3cm (dari 1cm)
            'update_min_a':      0.05,   # ~3° (dari 0.5°)
            'resample_interval': 3,      # resample tiap 3 update (dari 2)

            # ── ANTI-NOISE: kurangi beams dan naikkan sigma ─
            'laser_model_type':          'likelihood_field',
            'laser_likelihood_max_dist':  0.5,
            'laser_max_range':            4.0,   # dari 5m
            'laser_min_range':            0.2,
            'laser_max_beams':            30,    # dari 60 → 2x lebih ringan
            'laser_z_hit':               0.9,   # sangat percaya titik yang cocok
            'laser_z_rand':              0.1,   # toleransi noise sangat kecil
            'laser_sigma_hit':           0.1,   # ketat → noise outlier diabaikan

            # Tidak set initial pose — user set via RViz
            'set_initial_pose':           False,

            'transform_tolerance':        1.5,
            'tf_broadcast':               True,
            'always_reset_initial_pose':  False,
            'first_map_only':             False,
            'save_pose_rate':             0.5,
        }]
    )

    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['amcl']}]
    )

    # ════════════════════════════════════════════════════════
    # Staggered startup
    # ════════════════════════════════════════════════════════

    delayed_jointodom  = TimerAction(period=2.0, actions=[joint_odom_node])
    delayed_perception = TimerAction(
        period=3.0,
        actions=[detector_node, simple_pc2scan_node, particle_converter_node]
    )
    delayed_amcl = TimerAction(
        period=5.0, actions=[amcl_node, lifecycle_manager_amcl]
    )

    return LaunchDescription([
        white_threshold_arg,
        static_tf_publisher,
        # static_odom_baselink DIHAPUS — joint_odom_node yang publish TF ini
        map_server_node,
        delayed_lifecycle_map,
        delayed_jointodom,
        delayed_perception,
        delayed_amcl,
        # odom_to_amcl_node DIHAPUS — tidak dibutuhkan di Opsi C
    ])