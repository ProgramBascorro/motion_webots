#!/usr/bin/env python3
"""
amcl_jointodom.launch.py v9 — High Trust Odom
Strategi: joint odom diperlakukan seperti GT odom
  1. odom_to_amcl_node publish /initialpose terus-menerus (seperti gt_odom_to_amcl)
  2. AMCL alpha sangat kecil (0.0001) — odom dipercaya penuh
  3. Scan hanya untuk fine-tune, bukan untuk localize dari nol

Perbandingan dengan GT launch:
  GT:    gt_odom_to_amcl → alpha=0.000001, cov=0.01  (odom sempurna)
  Kita:  odom_to_amcl    → alpha=0.0001,   cov=0.05  (odom sangat baik, ada sedikit drift)
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

    static_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_baselink',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

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
    # t = 2s: Joint Odom + Odom-to-AMCL
    # Keduanya start bersamaan — analog dengan gt_odom_node + gt_odom_to_amcl
    # ════════════════════════════════════════════════════════

    joint_odom_node = Node(
        package='soccer_object_localization',
        executable='joint_odom_node',
        name='joint_odom_node',
        output='screen',
        parameters=[{
            'base_frame':      'base_link',
            'odom_frame':      'odom',
            'use_imu_yaw':     True,
            'pose_cov_xy':     0.1,    # kecil: odom dipercaya
            'pose_cov_yaw':    0.05,
            'smoothing_alpha': 0.5,
            'step_scale':      0.8,
            'dual_stance_avg': True,
            'stance_l_min':    1.0,
            'stance_r_min':    1.0,
        }]
    )

    odom_to_amcl_node = Node(
        package='soccer_object_localization',
        executable='odom_to_amcl_node',
        name='odom_to_amcl_node',
        output='screen',
        parameters=[{
            # high_trust: cov_xy=0.05, cov_yaw=0.03
            # Analog dengan gt_odom_to_amcl yang pakai cov ~0.01
            # Sedikit lebih besar karena joint odom ada drift
            'trust_level':    'high_trust',
            'publish_rate':    0.3,    # setiap 300ms — lebih sering dari v8
            'always_publish':  True,   # publish terus, tapi hanya setelah user set pose
            'min_move_dist':   0.01,
            'min_move_angle':  0.01,
        }]
    )

    # ════════════════════════════════════════════════════════
    # t = 3s: Detector + PC2Scan
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
            'angle_min':       -3.14159,
            'angle_max':        3.14159,
            'angle_increment':  0.0174533,
            'range_min':        0.3,
            'range_max':        5.0,
            'scan_height':      0.0,
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
    # t = 5s: AMCL — High Trust Odom Configuration
    # alpha sangat kecil = odom dipercaya, partikel tidak menyebar
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

            # Partikel lebih sedikit — initialpose sudah beri titik acuan
            'min_particles': 200,
            'max_particles': 1000,
            'recovery_alpha_slow': 0.0001,
            'recovery_alpha_fast': 0.01,

            # ── KUNCI UTAMA: alpha sangat kecil = odom dipercaya ──
            # GT pakai 0.000001, kita pakai 0.0001
            # Artinya: partikel hampir tidak menyebar saat bergerak
            # Posisi dari /initialpose yang dominan
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.0001,   # noise rotasi dari rotasi
            'alpha2': 0.0001,   # noise rotasi dari translasi
            'alpha3': 0.0001,   # noise translasi dari translasi
            'alpha4': 0.0001,   # noise translasi dari rotasi
            'alpha5': 0.0001,

            # Update sering agar pose AMCL mengikuti odom dengan cepat
            'update_min_d':      0.005,   # 0.5cm — hampir setiap langkah
            'update_min_a':      0.005,
            'resample_interval': 2,

            # Laser: balance — scan untuk fine-tune, bukan untuk localize
            # Sama dengan GT reference (z_hit=0.5, z_rand=0.5)
            'laser_model_type':          'likelihood_field',
            'laser_likelihood_max_dist':  0.5,
            'laser_max_range':            5.0,
            'laser_min_range':            0.3,
            'laser_max_beams':            60,
            'laser_z_hit':               0.5,
            'laser_z_rand':              0.5,
            'laser_sigma_hit':           0.2,

            # Initial pose TIDAK di-set otomatis
            # User harus klik 2D Pose Estimate di RViz
            # odom_to_amcl_node akan mulai track setelah itu
            'set_initial_pose':           False,

            'transform_tolerance':        3.0,
            'tf_broadcast':               True,
            'always_reset_initial_pose':  False,
            'first_map_only':             False,
            'save_pose_rate':             2.0,
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

    delayed_jointodom = TimerAction(
        period=2.0,
        actions=[joint_odom_node, odom_to_amcl_node]
    )
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
        static_odom_baselink,
        map_server_node,
        delayed_lifecycle_map,    # t=1s
        delayed_jointodom,        # t=2s
        delayed_perception,       # t=3s
        delayed_amcl,             # t=5s
    ])