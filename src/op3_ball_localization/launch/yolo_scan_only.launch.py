from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch YOLO + Head Tracking Only.

    This launch file provides head tracking without walking or kicking.
    The robot will:
    - Track the ball by keeping it centered in camera view
    - NOT walk toward the ball
    - NOT kick the ball

    Prerequisites:
    - Launch manager + USB camera first (via script.sh)
    - This only launches YOLO vision and the head tracking node
    """

    # Get package directories
    op3_yolo_pkg = get_package_share_directory('op3_yolo_vision')
    op3_ball_loc_pkg = get_package_share_directory('op3_ball_localization')

    # Config files
    yolo_config = os.path.join(op3_yolo_pkg, 'config', 'yolo.yaml')
    head_tracking_config = os.path.join(op3_ball_loc_pkg, 'config', 'head_tracking.yaml')

    return LaunchDescription([
        # 1. YOLO Vision Detector
        Node(
            package='op3_yolo_vision',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[yolo_config]
        ),

        # 2. Head Tracking Only (NO walking or kicking)
        Node(
            package='op3_ball_localization',
            executable='head_tracking_node',
            name='head_tracking_node',
            output='screen',
            parameters=[head_tracking_config]
        ),
    ])
