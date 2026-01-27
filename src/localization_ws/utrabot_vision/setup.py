from setuptools import setup, find_packages
from glob import glob

package_name = 'utrabot_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Utrabot vision: camera interface and basic processing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = utrabot_vision.camera_node:main',
            'camera_processing = utrabot_vision.camera_processing:main',
            'view_image = utrabot_vision.view_image:main',
        ],
    },
)
