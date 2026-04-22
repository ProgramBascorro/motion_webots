#!/usr/bin/env python3
"""
localization_v10.launch.py  [v10.5]
Timing IDENTIK v10.2 (proven) + odom_throttle (NEW) + parameter dari v10.3
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

    # [NEW v10.5] Throttle /odom 100Hz → /odom_throttled 20Hz
    # ekf_soccer.yaml: odom0: /odom_throttled
    # Prerequisite: sudo apt install ros-$ROS_DISTRO-topic-tools
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

    map_server_node = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"yaml_filename":map_file,"topic_name":"map","frame_id":"map"}]
    )

    # t=1s — IDENTIK dengan v10.2
    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen",
        parameters=[{"autostart":True,"node_names":["map_server"]}]
    )

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
            "laser_max_beams":360,"laser_z_hit":0.7,"laser_z_rand":0.3,"laser_sigma_hit":0.25,
            "set_initial_pose":True,"initial_pose.x":-0.3626850943317429,"initial_pose.y":-0.006978506726671451,
            "initial_pose.z":0.24689830513252942,"initial_pose.yaw":8.02837040365665e-07,
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

    ekf_node = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", output="screen",
        parameters=[ekf_config],
        remappings=[("odometry/filtered","/odometry/filtered"),("/set_pose","/initialpose")]
    )

    field_boundary_node = Node(
        package="soccer_object_localization", executable="field_boundary_detector",
        name="field_boundary_detector", output="screen",
        parameters=[{"green_h_low":35,"green_h_high":85,"green_s_low":60,"green_v_low":60,
                     "white_s_high":60,"white_v_low":160,"subsample_x":8,"subsample_y":8,
                     "t_pix":0.3,"t_row":0.25,"n_zero":4,"smooth_kernel":5,
                     "boundary_margin_px":10,"min_boundary_row":0.05,"max_boundary_row":0.85,
                     "publish_debug":True}],
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

    goal_yaw_corrector_node = Node(
        package="soccer_object_localization", executable="goal_yaw_corrector",
        name="goal_yaw_corrector", output="screen",
        parameters=[{
            "focal_length":793.3,"image_width":1280,"image_height":720,
            "roi_top":0.02,"roi_bottom":0.42,"white_threshold":200,
            "min_line_length_px":200,"max_line_angle_deg":25.0,
            "max_yaw_delta_deg":2.0,"min_confidence":0.7,"cooldown_sec":5.0,
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

    particle_converter_node = Node(
        package="soccer_object_localization", executable="particle_converter",
        name="particle_converter", output="screen"
    )

    # Timing IDENTIK dengan v10.2 + odom_throttle di t=0
    return LaunchDescription([
        white_threshold_arg,
        static_tf_publisher, kf_odom_node, odom_throttle_node,
        map_server_node, ekf_node, camera_info_publisher_node, rectify_node,
        TimerAction(period=1.0, actions=[lifecycle_manager_map]),
        TimerAction(period=2.0, actions=[
            detector_node, simple_pc2scan_node, scan_stabilizer_node,
            field_boundary_node, segment_classifier_node,
        ]),
        TimerAction(period=3.0, actions=[amcl_node]),
        TimerAction(period=4.0, actions=[lifecycle_manager_amcl]),
        TimerAction(period=5.0, actions=[particle_converter_node]),
        TimerAction(period=6.0, actions=[goal_yaw_corrector_node, cox_registration_node]),
    ])