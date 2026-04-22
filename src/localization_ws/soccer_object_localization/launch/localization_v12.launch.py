#!/usr/bin/env python3
"""
localization_v10.launch.py  [v10.6]
====================================
Basis: v10.5 (proven stable)
NEW v10.6:
  + crossing_detector (Fase 3 v2.4) — /field_line_segments → /detected_crossings
  + crossing_amcl_constraint (Paper 2 Eq.5) — /detected_crossings → /initialpose

Timing:
  t=0  : static_tf, kf_odom, odom_throttle, map_server, ekf, camera, rectify
  t=1  : lifecycle_manager_map
  t=2  : detector_fieldline, simple_pc2scan, scan_stabilizer,
          field_boundary, segment_classifier
  t=3  : amcl
  t=4  : lifecycle_manager_amcl
  t=5  : particle_converter
  t=6  : goal_yaw_corrector, cox_registration
  t=7  : crossing_detector, crossing_amcl_constraint  ← NEW
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_loc     = get_package_share_directory("soccer_object_localization")
    config_file = os.path.join(pkg_loc, "config", "op3_sim.yaml")
    map_file    = os.path.join(pkg_loc, "maps", "soccer_field.yaml")
    ekf_config  = os.path.join(pkg_loc, "config", "ekf_soccer.yaml")
    voronoi_lut = os.path.join(pkg_loc, "config", "voronoi_lut.npz")
    camera_yaml = os.path.join(pkg_loc, "config", "camera.yaml")

    white_threshold_arg = DeclareLaunchArgument(
        "white_threshold", default_value="165",
        description="Threshold putih untuk deteksi garis lapangan"
    )

    # ── t=0: Core nodes ──────────────────────────────────────────────────────
    static_tf_publisher = Node(
        package="op3_utra_bridge", executable="op3_static_transforms.py",
        name="static_tf_publisher", output="screen"
    )

    kf_odom_node = Node(
        package="soccer_object_localization", executable="legged_odometry_kf_node",
        name="legged_odometry_kf", output="screen",
        parameters=[{"base_frame":"base_link","odom_frame":"odom","publish_rate":100.0,
                     "simulation_mode":True,"q_pos":0.005,"q_vel":0.05,"r_pos":0.002}]
    )

    odom_throttle_node = Node(
        package="topic_tools", executable="throttle", name="odom_throttle",
        output="screen",
        arguments=["messages", "/odom", "20.0", "/odom_throttled"],
    )

    camera_info_publisher_node = Node(
        package="soccer_object_localization", executable="camera_info_publisher",
        name="camera_info_publisher", output="screen",
        parameters=[{"camera_yaml_path": camera_yaml}]
    )

    rectify_node = Node(
        package="image_proc", executable="rectify_node", name="rectify_node",
        output="screen",
        remappings=[
            ("image",       "/robotis_op3/camera/image_raw"),
            ("camera_info", "/robotis_op3/camera/camera_info"),
            ("image_rect",  "/robotis_op3/camera/image_rect"),
        ]
    )

    map_server_node = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"yaml_filename":map_file,"topic_name":"map","frame_id":"map"}]
    )

    ekf_node = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", output="screen",
        parameters=[ekf_config],
        remappings=[("odometry/filtered","/odometry/filtered"),("/set_pose","/initialpose")]
    )

    # ── t=1: Map lifecycle ───────────────────────────────────────────────────
    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen",
        parameters=[{"autostart":True,"node_names":["map_server"]}]
    )

    # ── t=2: Perception pipeline ─────────────────────────────────────────────
    detector_node = Node(
        package="soccer_object_localization", executable="detector_fieldline_enhanced2",
        name="detector_fieldline", output="screen",
        parameters=[config_file, {
            "use_dynamic_tf": False,
            "camera.height": 0.475, "camera.tilt": -0.349,
            "camera.offset_x": 0.08, "camera.offset_y": 0.0,
            "camera.focal_length": 793.3,
            "camera.image_width": 1280, "camera.image_height": 720,
            "detection.white_threshold": LaunchConfiguration("white_threshold"),
            "detection.use_enhanced": True,
            "detection.roi_top_cut": 0.15, "detection.roi_bottom_cut": 0.08,
            "detection.min_line_length": 15, "detection.max_line_gap": 25,
            "detection.canny_low": 60, "detection.canny_high": 180,
            "detection.hough_threshold": 60, "detection.remove_grass": True,
            "detection.grass_h_low": 35, "detection.grass_h_high": 85,
            "detection.grass_s_low": 40,
            "point_cloud.spacing": 12, "point_cloud.max_distance": 4.5,
            "point_cloud.min_points": 5,
            "publish.debug_image": True, "publish.point_cloud": True,
        }],
        remappings=[
            ("/camera/image_raw",   "/robotis_op3/camera/image_rect"),
            ("/camera/camera_info", "/robotis_op3/camera/camera_info"),
        ]
    )

    simple_pc2scan_node = Node(
        package="soccer_object_localization", executable="simple_pc2scan",
        name="simple_pc2scan", output="screen",
        parameters=[{"angle_min":-3.14159,"angle_max":3.14159,"angle_increment":0.0174533,
                     "range_min":0.3,"range_max":4.5,"scan_height":0.0}]
    )

    scan_stabilizer_node = Node(
        package="soccer_object_localization", executable="scan_stabilizer",
        name="scan_stabilizer", output="screen",
        parameters=[{"imu_topic":"/robotis_op3/imu","roll_threshold_deg":5.0,
                     "pitch_threshold_deg":6.0,"max_hold_sec":0.8,"min_stable_count":3}]
    )

    # field_boundary_detector v2.1 (sky-first approach)
    field_boundary_node = Node(
        package="soccer_object_localization", executable="field_boundary_detector",
        name="field_boundary_detector", output="screen",
        parameters=[{
            # Sky detection (sky-first approach v2.1)
            "sky_v_thresh":     55,      # V_HSV < 55 → background/sky Webots
            "t_sky":            0.35,    # min fraksi sky per blok
            # Green verification
            "green_h_low":      35,
            "green_h_high":     85,
            "green_s_low":      50,
            "green_v_low":      50,
            "t_green_verify":   0.30,
            # Goal post exclusion
            "goal_bright_v":    190,
            "goal_bright_frac": 0.10,
            "goal_top_ratio":   0.50,
            # Boundary constraints
            "subsample_x":      8,
            "subsample_y":      8,
            "boundary_margin_px": 8,
            "min_boundary_row": 0.04,
            "max_boundary_row": 0.85,
            "smooth_kernel":    9,
            "convex_iters":     8,
            "publish_debug":    True,
        }],
        remappings=[("/robotis_op3/camera/image_raw","/robotis_op3/camera/image_rect")]
    )

    segment_classifier_node = Node(
        package="soccer_object_localization", executable="segment_classifier_gw",
        name="segment_classifier_gw", output="screen",
        parameters=[{
            "max_angle_deg":40.0,"min_projection":0.15,"vote_threshold":0.45,"scan_half":4,
            "max_angle_deg_far":45.0,"min_projection_far":0.10,"vote_threshold_far":0.35,"scan_half_far":6,
            "hough_threshold":60,"hough_min_line":60,"hough_max_gap":15,
            "min_segment_len":30,"max_segment_len":600,
            "hough_threshold_far":20,"hough_min_line_far":15,"hough_max_gap_far":25,"min_segment_len_far":12,
            "far_zone_split":0.60,"nms_dist":15,"auto_calibrate_gw":True,
            "white_threshold":200,"use_boundary_roi":True,"publish_debug":True,
            "use_line_image":True,"roi_top_fallback":0.35,
        }]
    )

    # ── t=3-5: AMCL ──────────────────────────────────────────────────────────
    amcl_node = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[{
            "odom_frame_id":"odom","base_frame_id":"cam_link","global_frame_id":"map",
            "scan_topic":"field_scan_stable",
            "min_particles":1000,"max_particles":8000,
            "recovery_alpha_slow":0.001,"recovery_alpha_fast":0.2,
            "robot_model_type":"nav2_amcl::DifferentialMotionModel",
            "alpha1":0.000001,"alpha2":0.000001,"alpha3":0.000001,
            "alpha4":0.000001,"alpha5":0.000001,
            "update_min_d":0.01,"update_min_a":0.01,"resample_interval":1,
            "laser_model_type":"likelihood_field",
            "laser_likelihood_max_dist":1.0,"laser_max_range":5.0,"laser_min_range":0.3,
            "laser_max_beams":180,"laser_z_hit":0.7,"laser_z_rand":0.3,"laser_sigma_hit":0.25,
            "set_initial_pose":True,"initial_pose.x":0.0,"initial_pose.y":0.0,
            "initial_pose.z":0.0,"initial_pose.yaw":0.0,
            "initial_pose_covariance":[
                0.05,0.0,0.0,0.0,0.0,0.0, 0.0,0.05,0.0,0.0,0.0,0.0,
                0.0,0.0,0.01,0.0,0.0,0.0, 0.0,0.0,0.0,0.01,0.0,0.0,
                0.0,0.0,0.0,0.0,0.01,0.0, 0.0,0.0,0.0,0.0,0.0,0.02,
            ],
            "transform_tolerance":0.3,"tf_broadcast":False,
            "always_reset_initial_pose":False,"first_map_only":False,"save_pose_rate":2.0,
        }]
    )

    lifecycle_manager_amcl = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_amcl", output="screen",
        parameters=[{"autostart":True,"node_names":["amcl"]}]
    )

    particle_converter_node = Node(
        package="soccer_object_localization", executable="particle_converter",
        name="particle_converter", output="screen"
    )

    # ── t=6: Correctors ──────────────────────────────────────────────────────
    goal_yaw_corrector_node = Node(
        package="soccer_object_localization", executable="goal_yaw_corrector",
        name="goal_yaw_corrector", output="screen",
        parameters=[{
            "focal_length":793.3,"image_width":1280,"image_height":720,
            "roi_top":0.02,"roi_bottom":0.42,"white_threshold":200,
            "min_line_length_px":200,"max_line_angle_deg":25.0,
            "max_yaw_delta_deg":3.0,"min_confidence":0.7,"cooldown_sec":10.0,
            "cov_xy":9.0,"cov_yaw":0.015,
        }],
        remappings=[("/robotis_op3/camera/image_raw","/robotis_op3/camera/image_rect")]
    )

    cox_registration_node = Node(
        package="soccer_object_localization", executable="cox_registration",
        name="cox_registration", output="screen",
        parameters=[{
            "voronoi_lut_path":voronoi_lut,
            "image_width":1280,"image_height":720,"focal_length":793.3,
            "cam_pitch_deg":-20.0,"camera_height_m":0.475,
            "rate_hz":1.0,"max_delta_x":0.30,"max_delta_y":0.30,"max_delta_theta":20.0,
            "min_points":10,"min_confidence":0.30,"outlier_dist":0.5,
            "eta":0.01,"zeta":0.001,"cov_x":0.04,"cov_y":0.04,"cov_yaw":0.02,
            "field_half_len":4.5,"field_half_wid":3.0,
        }]
    )

    # ── t=7: Crossing detection + AMCL constraint [NEW v10.6] ────────────────
    # crossing_detector v2.4: /field_line_segments → /detected_crossings
    crossing_detector_node = Node(
        package="soccer_object_localization", executable="crossing_detector",
        name="crossing_detector", output="screen",
        parameters=[{
            # Kamera OP3
            "image_width":          1280,
            "image_height":          720,
            "focal_length":          793.3,
            "cam_pitch_deg":         -20.0,
            "camera_height_m":        0.475,
            "field_half_len":          4.5,
            "field_half_wid":          3.0,
            # Segmen input
            "min_seg_confidence":     0.20,
            "min_seg_length":         25.0,
            # Intersection & cluster
            "cluster_dist_px":         60,    # FIX2: 35→60 (kurangi cluster padat)
            "min_angle_diff_deg":      25.0,
            "near_endpoint_ratio":      0.30,
            # Goal post filter (FIX1)
            "goal_filter_angle_deg":   70.0,  # min angle vertikal (deg)
            "goal_filter_top_ratio":    0.30,  # ujung atas harus < 30% frame
            # World dedup (FIX2)
            "world_merge_dist_m":       0.25,  # merge crossing <0.25m di world
            # ROI
            "roi_top_ratio":            0.25,
            "roi_bottom_ratio":         0.97,
            # Output
            "min_confidence":           0.40,
            "max_crossings_per_frame":  6,
            "publish_debug":            True,
        }]
    )

    # crossing_amcl_constraint v1.0: /detected_crossings → /initialpose (Paper 2 Eq.5)
    crossing_constraint_node = Node(
        package="soccer_object_localization", executable="crossing_amcl_constraint",
        name="crossing_amcl_constraint", output="screen",
        parameters=[{
            # Dimensi lapangan
            "field_half_len":  4.5,
            "field_half_wid":  3.0,
            "penalty_x":       3.1,
            "penalty_y":       1.3,
            # Paper 2 Eq.5 noise model
            "sigma_d":         0.20,   # deviasi jarak dasar (m)
            "lambda_d":        0.05,   # faktor proporsional
            "sigma_beta":      0.15,   # deviasi bearing (rad)
            # Threshold integrasi
            "min_likelihood":  0.25,   # min score per crossing
            "min_confidence":  0.50,   # min confidence dari crossing_detector
            "max_match_dist_m": 1.50,  # max jarak match ke crossing peta (m)
            "cooldown_sec":    2.0,    # min interval antar koreksi
            # Covariance koreksi (besar = tidak agresif, hormati AMCL)
            "cov_x":           0.10,   # m²
            "cov_y":           0.10,   # m²
            "cov_yaw":         0.05,   # rad²
            "max_correction_m": 0.50,  # max koreksi per cycle (m)
        }]
    )

    # ── goal_detector (opsional, pertahankan dari v10.5) ─────────────────────
    goal_detector_node = Node(
        package="soccer_object_localization", executable="goal_detector",
        name="goal_detector", output="screen",
        parameters=[{
            "goal_width_m":2.6,"goal_height_m":1.2,"focal_length":793.3,
            "image_width":1280,"image_height":720,
            "field_half_length":4.5,"field_half_width":3.0,
            "min_goal_width_px":80,"min_confidence":0.6,"correction_interval":0.5,
            "yaw_only_threshold":0.75,"max_yaw_correction_deg":30.0,"white_threshold":200,
        }],
        remappings=[("/robotis_op3/camera/image_raw","/robotis_op3/camera/image_rect")]
    )

    particle_converter_node2 = Node(
        package="soccer_object_localization", executable="particle_converter",
        name="particle_converter", output="screen"
    )

    # ── Launch Description ────────────────────────────────────────────────────
    return LaunchDescription([
        white_threshold_arg,

        # t=0: langsung
        static_tf_publisher,
        kf_odom_node,
        odom_throttle_node,
        map_server_node,
        ekf_node,
        camera_info_publisher_node,
        rectify_node,

        # t=1: lifecycle map
        TimerAction(period=1.0, actions=[lifecycle_manager_map]),

        # t=2: perception pipeline
        TimerAction(period=2.0, actions=[
            detector_node,
            simple_pc2scan_node,
            scan_stabilizer_node,
            field_boundary_node,
            segment_classifier_node,
        ]),

        # t=3: AMCL
        TimerAction(period=3.0, actions=[amcl_node]),

        # t=4: AMCL lifecycle
        TimerAction(period=4.0, actions=[lifecycle_manager_amcl]),

        # t=5: particle converter
        TimerAction(period=5.0, actions=[particle_converter_node]),

        # t=6: correctors (goal yaw + cox registration)
        TimerAction(period=6.0, actions=[
            goal_yaw_corrector_node,
            cox_registration_node,
        ]),

        # t=7: crossing detection + AMCL constraint [NEW v10.6]
        # Tunggu t=7 agar /field_line_segments sudah stabil dari segment_classifier
        TimerAction(period=7.0, actions=[
            crossing_detector_node,
            crossing_constraint_node,
        ]),
    ])