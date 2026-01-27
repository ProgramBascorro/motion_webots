#!/usr/bin/env python3
"""
Setup script for soccer_object_detection ROS2 package
Compatible with ROS2 Humble and ament_python build system
"""

import os
from glob import glob
from setuptools import setup, find_packages

package_name = "soccer_object_detection"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Resource marker for package discovery
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        
        # Package manifest
        ("share/" + package_name, ["package.xml"]),
        
        # Configuration files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        
        # Launch files (Python only, no .launch XML files)
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        
        # Model files (.pt YOLO weights)
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
        (os.path.join("share", package_name, "models"), ["models/README.md"]),
        
        # GUI configurations
        (os.path.join("share", package_name, "gui"), glob("gui/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anthony Pinson",
    maintainer_email="pinsonanthony@gmail.com",
    description="Soccer object detection using YOLO for ROS2 Humble - UTRA RoboSoccer",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Main object detection node
            "soccer_object_detection = soccer_object_detection.object_detect_node_ros:main",
            
            # Add other executables if needed
            # "camera_node = soccer_object_detection.camera.camera_calculations_ros:main",
        ],
    },
)