from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_detector_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # Install resource files (like your custom model if you add one)
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.pt'))), # Install .pt files
        # Install launch files if you create any
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='YOLO object detection node using ultralytics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_detector_node.yolo_node:main',
        ],
    },
)