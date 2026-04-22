#!/usr/bin/env python3
"""
localization_kf_final.launch.py — Tuned + EKF + Anti-Symmetry Fix  [v9]
=========================================================================
BASIS: localization_kf_tuned.launch.py — program terstabil sejauh ini.

PENAMBAHAN:

  [EKF] robot_localization ekf_filter_node
        Fusi /odom (KF 100Hz) + /amcl_pose (~2Hz)
        → TF map→odom yang smooth, tidak loncat
        → AMCL tf_broadcast=False, EKF yang pegang TF map→odom
        → pose0_rejection_threshold=5.0: loncatan AMCL difilter EKF

  [A]   laser_likelihood_max_dist: 0.5 → 1.0
          AMCL lebih toleran mismatch peta vs realita
        transform_tolerance: 1.0 → 0.3
          TF yang dipakai lebih segar
        max_particles: 4000 → 6000
          Lebih banyak hipotesis untuk lapangan simetri

  [B]   Instruksi kalibrasi kamera ada di komentar detector node
        Prosedur: robot di (0,0,0°) → set 2D Pose → lihat overlay
        /field_line_cloud di RViz → sesuaikan camera.tilt jika melenceng

  [C]   initial_pose_covariance ketat: ±7cm, ±8°
        Default Nav2 ±30cm/±15° terlalu besar untuk lapangan simetri
        → partikel tidak menyebar ke posisi mirror

YANG DIPERTAHANKAN dari tuned (tidak diubah):
  base_frame_id = 'cam_link'
  update_min_d/a = 0.01  (TIDAK dinaikkan — terbukti optimal di tuned)
  laser_z_hit/rand/sigma = 0.7/0.3/0.25
  recovery_alpha_fast = 0.2
  scan_stabilizer: roll=5°, pitch=6°, hold=0.8s, stable=3
  lifecycle x2 terpisah, timer sequence identik

PERUBAHAN dari v8 (tuned_ekf):

  [SYM] Anti-simetri lapangan:
    roi_top_cut: 0.35 -> 0.15
      Gawang masuk ke ROI kamera → scan melihat gawang
      Gawang asimetrik (hanya di ujung lapangan) → AMCL bisa
      membedakan pose benar vs pose mirror 180°

    laser_max_beams: 60 → 180
      Lebih banyak beam → matching lebih kuat
      Dengan 60 beams, garis simetri sulit dibedakan

    max_particles: 6000 → 8000
      Cluster mirror dan cluster benar butuh partikel lebih banyak

  [EKF] pose0_rejection_threshold: 5.0 → 2.5
      EKF lebih agresif tolak loncatan AMCL saat konverge ke mirror

PREREQUISITE:
  ros2 pkg list | grep robot_localization
  # Jika belum:
  sudo apt install ros-$(echo $ROS_DISTRO)-robot-localization

  # Taruh ekf_soccer.yaml di:
  ~/ros2_ws/src/soccer_object_localization/config/ekf_soccer.yaml
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
    map_file    = os.path.join(pkg_loc, 'maps', 'soccer_field.yaml')
    ekf_config  = os.path.join(pkg_loc, 'config', 'ekf_soccer.yaml')

    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold',
        default_value='165',
        description='Threshold putih untuk deteksi garis lapangan'
    )

    # ── 1. Static TF ─────────────────────────────────────────────────
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )

    # ── 2. KF Odometry ───────────────────────────────────────────────
    kf_odom_node = Node(
        package='soccer_object_localization',
        executable='legged_odometry_kf_node',
        name='legged_odometry_kf',
        output='screen',
        parameters=[{
            'base_frame':      'base_link',
            'odom_frame':      'odom',
            'publish_rate':    100.0,
            'simulation_mode': True,
            'q_pos':  0.005,
            'q_vel':  0.05,
            'r_pos':  0.002,
        }]
    )

    # ── 3. Field Line Detector ───────────────────────────────────────
    # [B] INSTRUKSI KALIBRASI KAMERA:
    # 1. Jalankan sistem, set robot di (0,0) menghadap +X di Webots
    # 2. Klik 2D Pose Estimate di RViz → (0, 0, 0°)
    # 3. Aktifkan display /field_line_cloud di RViz (PointCloud2)
    # 4. Lihat apakah titik-titik cloud overlay tepat di garis peta:
    #    - Cloud terlalu JAUH  → naikkan |tilt|: -0.349 → -0.384 (-22°)
    #    - Cloud terlalu DEKAT → kurangi |tilt|: -0.349 → -0.314 (-18°)
    #    - Cloud melenceng KIRI/KANAN → sesuaikan camera.offset_y
    # Nilai saat ini: tilt=-0.349rad (-20°), height=0.475m
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_enhanced2',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                'use_dynamic_tf': False,

                'camera.height':       0.475,
                'camera.tilt':        -0.349,
                'camera.offset_x':     0.08,
                'camera.offset_y':     0.0,
                'camera.focal_length': 900.0,
                'camera.image_width':  1280,
                'camera.image_height': 720,

                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'detection.use_enhanced':    True,
                'detection.roi_top_cut':     0.15,   # was 0.35 — [SYM] gawang masuk ROI
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

                'point_cloud.spacing':      12,
                'point_cloud.max_distance':  4.5,
                'point_cloud.min_points':    5,

                'publish.debug_image':  True,
                'publish.point_cloud':  True,
            }
        ],
        remappings=[
            ('/camera/image_raw',   '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )

    # ── 4. PointCloud → LaserScan ────────────────────────────────────
    simple_pc2scan_node = Node(
        package='soccer_object_localization',
        executable='simple_pc2scan',
        name='simple_pc2scan',
        output='screen',
        parameters=[{
            'angle_min':       -3.14159,
            'angle_max':        3.14159,
            'angle_increment':  0.0174533,
            'range_min':        0.3,
            'range_max':        4.5,
            'scan_height':      0.0,
        }]
    )

    # ── 4b. Scan Stabilizer ──────────────────────────────────────────
    scan_stabilizer_node = Node(
        package='soccer_object_localization',
        executable='scan_stabilizer',
        name='scan_stabilizer',
        output='screen',
        parameters=[{
            'imu_topic':             '/robotis_op3/imu',
            'roll_threshold_deg':     5.0,
            'pitch_threshold_deg':    6.0,
            'max_hold_sec':           0.8,
            'min_stable_count':       3,
        }]
    )

    # ── 5. Map Server ─────────────────────────────────────────────────
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'topic_name':    'map',
            'frame_id':      'map',
        }]
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart':   True,
            'node_names': ['map_server'],
        }]
    )

    # ── 6. AMCL ──────────────────────────────────────────────────────
    # tf_broadcast=False → EKF yang publish TF map→odom
    # AMCL tetap bekerja normal, hanya tidak publish TF langsung
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'odom_frame_id':   'odom',
            'base_frame_id':   'cam_link',
            'global_frame_id': 'map',

            'scan_topic': 'field_scan_stable',

            'min_particles':       1000,
            'max_particles':       8000,          # [SYM] was 6000 — lebih banyak cluster
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.2,

            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.000001,
            'alpha2': 0.000001,
            'alpha3': 0.000001,
            'alpha4': 0.000001,
            'alpha5': 0.000001,

            'update_min_d':      0.01,            # dipertahankan dari tuned
            'update_min_a':      0.01,            # dipertahankan dari tuned
            'resample_interval': 1,

            'laser_model_type':          'likelihood_field',
            'laser_likelihood_max_dist':  1.0,    # [A] was 0.5
            'laser_max_range':            5.0,
            'laser_min_range':            0.3,
            'laser_max_beams':            180,   # was 60 — [SYM] matching lebih kuat
            'laser_z_hit':                0.7,
            'laser_z_rand':               0.3,
            'laser_sigma_hit':            0.25,

            'set_initial_pose':          True,
            'initial_pose.x':            0.0,
            'initial_pose.y':            0.0,
            'initial_pose.z':            0.0,
            'initial_pose.yaw':          0.0,

            # [C] Covariance ketat — partikel tidak menyebar ke posisi mirror
            # Default Nav2 (0.25, 0.25, 0.07) terlalu besar untuk lapangan simetri
            'initial_pose_covariance': [
                0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.02,
            ],

            'transform_tolerance':       0.3,     # [A] was 1.0
            'tf_broadcast':              False,   # [EKF] EKF yang publish TF
            'always_reset_initial_pose': False,
            'first_map_only':            False,

            'save_pose_rate': 2.0,
        }]
    )

    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'autostart':   True,
            'node_names': ['amcl'],
        }]
    )

    # ── 7. EKF Fusion ─────────────────────────────────────────────────
    # [EKF] Menggabungkan /odom (100Hz) + /amcl_pose (~2Hz)
    # Hasil: TF map→odom smooth + /odometry/filtered
    #
    # Mengapa lebih baik dari AMCL tf_broadcast saja:
    #   AMCL update TF secara discontinuous (loncat) saat scan update tiba
    #   EKF update TF secara continuous (smooth) berdasarkan weighted fusion
    #   pose0_rejection_threshold: tolak update AMCL yang loncat >5σ
    #   → yaw tidak loncat tiba-tiba, error posisi mengecil gradual
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('/set_pose',         '/initialpose'),
        ]
    )


    # ── 9. Goal Detector ──────────────────────────────────────────────
    # Deteksi gawang dari kamera → estimasi yaw + pose → koreksi AMCL
    # Mengatasi masalah simetri lapangan: gawang adalah satu-satunya
    # fitur yang asimetrik (jarak ke gawang kiri ≠ kanan)
    #
    # Mode koreksi:
    #   confidence >= 0.75 → FULL: publish x, y, yaw ke /initialpose
    #   confidence >= 0.60 → YAW-ONLY: publish hanya yaw (cov_x/y besar)
    #
    # max_yaw_correction_deg=30.0 mencegah false positive yang ekstrem
    goal_detector_node = Node(
        package='soccer_object_localization',
        executable='goal_detector',
        name='goal_detector',
        output='screen',
        parameters=[{
            'goal_width_m':             2.6,
            'goal_height_m':            1.2,
            'focal_length':             900.0,
            'image_width':              1280,
            'image_height':             720,
            'field_half_length':        4.5,
            'field_half_width':         3.0,
            'min_goal_width_px':        80,
            'min_confidence':           0.6,
            'correction_interval':      0.5,
            'yaw_only_threshold':       0.75,
            'max_yaw_correction_deg':   30.0,
            'white_threshold':          200,
        }],
        remappings=[
            ('/robotis_op3/camera/image_raw', '/robotis_op3/camera/image_raw'),
        ]
    )

    # ── 8. Particle Cloud Converter ───────────────────────────────────
    particle_converter_node = Node(
        package='soccer_object_localization',
        executable='particle_converter',
        name='particle_converter',
        output='screen',
    )

    # ── Launch sequence ───────────────────────────────────────────────
    # Timer identik dengan tuned — hanya ekf_node ditambahkan di t=0
    # EKF mulai bersamaan dengan node lain, menunggu data dari odom + amcl
    return LaunchDescription([
        white_threshold_arg,

        static_tf_publisher,
        kf_odom_node,
        map_server_node,
        ekf_node,          # [EKF] mulai dari t=0, langsung subscribe /odom

        TimerAction(period=1.0, actions=[lifecycle_manager_map]),

        detector_node,
        simple_pc2scan_node,
        scan_stabilizer_node,

        TimerAction(period=3.0, actions=[amcl_node]),
        TimerAction(period=4.0, actions=[lifecycle_manager_amcl]),
        TimerAction(period=5.0, actions=[particle_converter_node]),
        TimerAction(period=6.0, actions=[goal_detector_node]),  # setelah AMCL aktif
    ])