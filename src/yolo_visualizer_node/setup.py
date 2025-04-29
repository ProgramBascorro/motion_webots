from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_visualizer_node'

setup(
    name=package_name,
    version='0.0.1',
    # Find the package automatically within the src directory
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file for ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files if you create any later
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name', # CHANGE THIS
    maintainer_email='your@email.com', # CHANGE THIS
    description='ROS 2 node to visualize YOLO detections on images.', # CHANGE THIS
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Defines the executable name for ros2 run/launch
            'visualizer_node = yolo_visualizer_node.visualizer_node:main',
        ],
    },
)