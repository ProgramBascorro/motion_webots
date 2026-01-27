from setuptools import setup
from glob import glob
import os

package_name = 'op3_utra_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # RViz config files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miftah',
    maintainer_email='your@email.com',
    description='OP3-UTRA Bridge Package for Soccer Robot Localization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'imu_bridge = op3_utra_bridge.imu_bridge:main',
        'odom_path_publisher = op3_utra_bridge.odom_path_publisher:main',
        'odom_to_tf = op3_utra_bridge.odom_to_tf:main',
        'robot_state_publisher = op3_utra_bridge.robot_state_publisher:main',
        'op3_static_transforms = op3_utra_bridge.op3_utra_br.op3_static_transforms:main',
        'odom_publisher_static = op3_utra_bridge.odom_publisher_static:main',  # NEW!
    ],
},
)