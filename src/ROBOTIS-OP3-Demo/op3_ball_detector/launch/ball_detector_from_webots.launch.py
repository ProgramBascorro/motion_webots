from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    op3_ball_detector_pkg_path = FindPackageShare('op3_ball_detector')
    ball_param_path = PathJoinSubstitution(
        [op3_ball_detector_pkg_path, 'config', 'ball_detector_params.yaml']
    )

    ball_detector_node = Node(
        package='op3_ball_detector',
        namespace='ball_detector_node',
        executable='ball_detector_node',
        output='screen',
        remappings=[
            ('/ball_detector_node/image_in',      '/robotis_op3/camera/image_raw/compressed'),
            ('/ball_detector_node/cameraInfo_in', '/robotis_op3/camera/camera_info'),
        ],
        parameters=[
            {"yaml_path": ball_param_path},
            ball_param_path,
        ],
    )

    return LaunchDescription([ball_detector_node])
