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
        ],
    },
)