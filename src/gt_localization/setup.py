from setuptools import setup

package_name = 'gt_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mesmer',
    maintainer_email='mesmer@gmail.com',
    description='Ground truth localization node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gt_odom_node = gt_localization.gt_odom_node:main',
            'gt_odom_to_amcl = gt_localization.gt_odom_to_amcl:main',
            'noisy_odom_sim = gt_localization.noisy_odom_sim:main',
        ],
    },
)
