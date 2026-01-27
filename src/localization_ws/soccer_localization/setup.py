from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'soccer_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you have any
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files if you have any
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason',
    maintainer_email='jiashen.wang@mail.utoronto.ca',
    description='Localization under a known map using AMCL for walking robots',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'soccer_localization = soccer_localization.main:main',
            'test_localization = soccer_localization.test_localization:main',
        ],
    },
)
