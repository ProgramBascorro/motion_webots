from setuptools import setup

package_name = 'walking_imu_to_odometry'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Convert walking steps and IMU to nav_msgs/Odometry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walking_imu_to_odometry = walking_imu_to_odometry.walking_imu_to_odometry:main',
            'walking_step_planner    = walking_imu_to_odometry.walking_step_planner:main',
            'image_to_compressed = walking_imu_to_odometry.image_to_compressed:main', 
        ],
    },
)

