#!/usr/bin/env python3
"""
localization_kf.launch.py — Localization Stack dengan KF Odometry  [v2]
=========================================================================
Mengganti gt_odom_node + gt_odom_to_amcl dengan legged_odometry_kf_node.

TF chain yang terbentuk:
  map → odom → base_link → head_link → cam_link
  │            │
  │            └─ dari legged_odometry_kf_node  (TF odom→base_link)
  └─────────── dari AMCL                        (TF map→odom)

Aliran data:
  /robotis_op3/joint_states ──┐
  /robotis_op3/imu            ├─ KF node ──→ /odom  +  TF odom→base_link
                              ┘
  /robotis_op3/camera/image_raw ─→ detector ─→ /field_line_cloud
                                                 │
                                     simple_pc2scan ─→ /field_scan
                                                               │
                                                            AMCL ─→ TF map→odom

Perbaikan v2 (dari analisis RViz — false positive tiang gawang):
  [P1] roi_top_cut: 0.35 → 0.45  — potong lebih banyak bagian atas frame
       point_cloud.max_distance: 5.5 → 4.0  — buang titik sangat jauh
       simple_pc2scan range_max: 5.0 → 4.0  — konsisten dengan detector
       → Mengurangi false positive dari tiang gawang (tinggi & jauh)

  [P2] laser_z_hit: 0.5 → 0.85, laser_z_rand: 0.5 → 0.15
       laser_sigma_hit: 0.2 → 0.15
       → AMCL lebih tegas menolak scan yang tidak cocok peta
       → Mengurangi pengaruh noise gawang yang masih lolos

  [P3] max_particles: 3000 → 5000, recovery_alpha_fast: 0.1 → 0.3
       → Recovery lebih cepat jika partikel tersebar akibat false positive

TENTANG 2D POSE ESTIMATE (RViz):
  Saat klik 2D Pose Estimate, AMCL menerima /initialpose dan menggeser
  TF map→odom. Odom KF TIDAK di-reset — ini BENAR dan normal.
  x[0],x[1] KF terus berjalan dari 0, map→odom adjustment menjembataninya.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_loc    = get_package_share_directory('soccer_object_localization')
    config_file = os.path.join(pkg_loc, 'config', 'op3_sim.yaml')
    map_file    = os.path.join(pkg_loc, 'maps', 'soccer_field.yaml')

    # ── Launch argument ──────────────────────────────────────────────
    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold',
        default_value='165',
        description='Threshold putih untuk deteksi garis lapangan'
    )

    # ── 1. Static TF (base_link → head_link → cam_link) ─────────────
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )

    # ── 2. Legged Odometry KF — menggantikan gt_odom_node ───────────
    #    Publish: /odom (nav_msgs/Odometry) + TF odom→base_link
    kf_odom_node = Node(
        package='soccer_object_localization',
        executable='legged_odometry_kf_node',
        name='legged_odometry_kf',
        output='screen',
        parameters=[{
            'base_frame':      'base_link',
            'odom_frame':      'odom',
            'publish_rate':    100.0,
            'simulation_mode': True,    # ← False untuk robot fisik
            # Noise KF (dikalibrasi dari simulasi)
            'q_pos':  0.005,
            'q_vel':  0.05,
            'r_pos':  0.002,
        }]
    )

    # ── 3. Field Line Detector ───────────────────────────────────────
    # [P1] roi_top_cut dinaikkan 0.35→0.45: memotong lebih banyak area
    #      atas gambar dimana tiang gawang biasanya muncul (objek jauh+tinggi)
    # [P1] point_cloud.max_distance diturunkan 5.5→4.0: titik sangat jauh
    #      (ujung lapangan) cenderung noise bukan garis lapangan yang berguna
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_enhanced2',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                'use_dynamic_tf': False,

                # Kamera
                'camera.height':       0.475,
                'camera.tilt':        -0.349,
                'camera.offset_x':     0.08,
                'camera.offset_y':     0.0,
                'camera.focal_length': 900.0,
                'camera.image_width':  1280,
                'camera.image_height': 720,

                # Deteksi
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'detection.use_enhanced':    True,
                # [P1] Naikkan top_cut untuk buang area gawang di atas gambar
                'detection.roi_top_cut':     0.35,   # dikembalikan: garis jauh (penalti, goal line) ikut terpotong jika > 0.35
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

                # Point cloud
                'point_cloud.spacing':      12,
                # [P1] Kurangi jarak max: titik >4m cenderung dari gawang/noise
                'point_cloud.max_distance':  4.5,    # garis jauh tetap masuk, AMCL yang filter gawang
                'point_cloud.min_points':    5,

                # Publish
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
            # [P1] Sesuaikan dengan max_distance detector
            'range_max':        4.5,       # sesuai max_distance detector
            'scan_height':      0.0,
        }]
    )

    # ── 5. Map Server ────────────────────────────────────────────────
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

    # ── 6. AMCL ─────────────────────────────────────────────────────
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            # Frame IDs
            'odom_frame_id':   'odom',
            'base_frame_id':   'base_link',
            'global_frame_id': 'map',

            # Scan topic
            'scan_topic': 'field_scan',

            # Particle filter
            # [P3] max_particles dinaikkan untuk ruang recovery lebih baik
            'min_particles':       500,
            'max_particles':       5000,   # was 3000
            # [P3] recovery_alpha_fast lebih agresif agar konvergen ulang cepat
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.3,    # was 0.1

            # Motion model
            # Alpha sangat kecil karena KF odometry sudah sangat akurat (0.22%/m)
            # AMCL hampir sepenuhnya percaya odom untuk pergerakan partikel
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.000001,
            'alpha2': 0.000001,
            'alpha3': 0.000001,
            'alpha4': 0.000001,
            'alpha5': 0.000001,

            # Update threshold
            'update_min_d':      0.01,
            'update_min_a':      0.01,
            'resample_interval': 1,

            # Laser model
            # [P2] z_hit naik 0.5→0.85: lebih percaya scan yang cocok peta
            # [P2] z_rand turun 0.5→0.15: kurangi toleransi random noise
            # [P2] sigma_hit turun 0.2→0.15: lebih ketat pada matching
            # Efek gabungan: AMCL lebih tegas menolak titik gawang yg tidak ada di peta
            'laser_model_type':          'likelihood_field',
            'laser_likelihood_max_dist':  0.5,
            'laser_max_range':            4.5,   # sesuai range_max scan
            'laser_min_range':            0.3,
            'laser_max_beams':            60,
            'laser_z_hit':                0.85,  # was 0.5  [P2]
            'laser_z_rand':               0.15,  # was 0.5  [P2]
            'laser_sigma_hit':            0.15,  # was 0.2  [P2]

            # Pose awal
            'set_initial_pose':      True,
            'initial_pose.x':        0.0,
            'initial_pose.y':        0.0,
            'initial_pose.z':        0.0,
            'initial_pose.yaw':      0.0,

            # Transform
            'transform_tolerance':       1.0,
            'tf_broadcast':              True,
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

    # ── 7. Particle Cloud Converter ──────────────────────────────────
    particle_converter_node = Node(
        package='soccer_object_localization',
        executable='particle_converter',
        name='particle_converter',
        output='screen',
    )

    # ── Launch sequence ──────────────────────────────────────────────
    # Map server perlu ready sebelum AMCL start → gunakan TimerAction
    return LaunchDescription([
        white_threshold_arg,

        # TF statis: langsung
        static_tf_publisher,

        # KF odometry: langsung (tidak bergantung map)
        kf_odom_node,

        # Map server: langsung
        map_server_node,

        # Lifecycle map server: tunda 1s agar map server sempat init
        TimerAction(period=1.0, actions=[lifecycle_manager_map]),

        # Sensor nodes: langsung
        detector_node,
        simple_pc2scan_node,

        # AMCL + lifecycle: tunda 3s agar map + TF sudah tersedia
        TimerAction(period=3.0, actions=[
            amcl_node,
        ]),
        TimerAction(period=4.0, actions=[
            lifecycle_manager_amcl,
        ]),

        # Particle converter: tunda sampai AMCL ready
        TimerAction(period=5.0, actions=[
            particle_converter_node,
        ]),
    ])