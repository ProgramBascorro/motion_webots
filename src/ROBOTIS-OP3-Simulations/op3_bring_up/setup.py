from setuptools import setup
import os
from glob import glob

package_name = 'op3_bring_up'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install semua launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Miftah',
    maintainer_email='your@email.com',
    description='Master bringup package untuk robot OP3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Tidak ada executable Python, hanya launch files
        ],
    },
)
