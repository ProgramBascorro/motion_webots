from setuptools import setup
import os
from glob import glob

package_name = 'soccer_object_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
         # Map file
        (os.path.join("share", package_name, "maps"), 
         glob("maps/*.pgm") + glob("maps/*.yaml")),
         # npz files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.npz') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Soccer field line detection and localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_fieldline_ros = soccer_object_localization.detector_fieldline_ros:main',
            'detector_fieldline_hybrid = soccer_object_localization.detector_fieldline_hybrid:main',
            'simple_pc2scan = soccer_object_localization.simple_pc2scan:main',
            'particle_converter = soccer_object_localization.particle_converter:main',
            'detector_fieldline_enhanced = soccer_object_localization.detector_fieldline_hybrid_enhanced:main',
            'detector_fieldline_enhanced2 = soccer_object_localization.detector_fieldline_hybrid_enhanced2:main',
            'filtered_dynamic = soccer_object_localization.filtered_dynamic_camera_pose:main',
            'vio_node = soccer_object_localization.vio_node_sim:main',
            'vio_node_light = soccer_object_localization.vio_node_light:main',
            'joint_odom_node = soccer_object_localization.joint_odom_node:main',
            'global_relocalization = soccer_object_localization.global_relocalization:main',
            'odom_to_amcl_node = soccer_object_localization.odom_to_amcl:main',
            'odom_constraint_node = soccer_object_localization.odom_constraint_node:main',
            'pose_filter_node = soccer_object_localization.pose_filter_node:main',
            'pose_relay_node = soccer_object_localization.pose_relay_node:main',
            "legged_odometry_kf_node = soccer_object_localization.legged_odometry_kf_node:main",
            "scan_stabilizer = soccer_object_localization.scan_stabilizer:main",
            'initialpose_publisher = soccer_object_localization.initialpose_publisher:main',
            'goal_detector = soccer_object_localization.goal_detector:main',
            'field_boundary_detector = soccer_object_localization.field_boundary_detector:main',
            'segment_classifier_gw = soccer_object_localization.segment_classifier_gw:main',
            'goal_yaw_corrector = soccer_object_localization.goal_yaw_corrector:main',
            'cox_registration = soccer_object_localization.cox_registration:main',
            'camera_info_publisher = soccer_object_localization.camera_info_publisher:main',
            'crossing_detector = soccer_object_localization.crossing_detector:main',
            'crossing_amcl_constraint = soccer_object_localization.crossing_amcl_constraint:main',
            'scan_gate = soccer_object_localization.scan_gate:main',
            'goal_localizer = soccer_object_localization.goal_localizer:main'
        ],
    },
)