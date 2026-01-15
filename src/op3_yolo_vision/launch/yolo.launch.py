from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('op3_yolo_vision')
    default_params = os.path.join(pkg_share, 'config', 'yolo.yaml')
    opencv_prefix = os.environ.get(
        'OP3_OPENCV_PREFIX',
        '/home/farhan/Projects/Surgical_lokalisasi_bismillah/third_party/opencv-4.8.1/install'
    )
    opencv_lib = os.path.join(opencv_prefix, 'lib')
    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    ld_path = opencv_lib if not existing_ld else f"{opencv_lib}:{existing_ld}"

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_path),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the parameter YAML file'
        ),
        Node(
            package='op3_yolo_vision',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
