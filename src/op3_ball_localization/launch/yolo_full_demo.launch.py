from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch YOLO + Full OP3 Demo (FULL DEMO mode)

    This launch file provides complete soccer behavior:
    - Scan for the ball with head movements
    - Track the ball by keeping it centered in camera view
    - Walk toward the ball
    - Kick when close enough

    Prerequisites:
    - Launch manager + USB camera first (via script.sh)
    - This only launches YOLO vision, bridge, and demo node
    """

    # Get package directory
    op3_yolo_pkg = get_package_share_directory('op3_yolo_vision')
    yolo_config = os.path.join(op3_yolo_pkg, 'config', 'yolo.yaml')

    return LaunchDescription([
        # 1. YOLO Vision Detector
        Node(
            package='op3_yolo_vision',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[yolo_config]
        ),

        # 2. YOLO to Demo Bridge (converts YOLO → CircleSetStamped)
        Node(
            package='op3_ball_localization',
            executable='yolo_to_demo_bridge',
            name='yolo_to_demo_bridge',
            output='screen'
        ),

        # 3. OP3 Demo Node (FULL behavior: tracking + following + kicking)
        Node(
            package='op3_demo',
            executable='op_demo_node',
            name='op_demo_node',
            output='screen',
            parameters=[{
                'grass_demo': False,
                'p_gain': 0.45,
                'd_gain': 0.045
            }]
        ),
    ])
