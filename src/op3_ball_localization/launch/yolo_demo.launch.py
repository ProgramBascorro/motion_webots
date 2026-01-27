from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch YOLO-based OP3 Demo
    Replaces HSV ball detector with YOLO + bridge
    """

    # Get package directories
    op3_yolo_pkg = get_package_share_directory('op3_yolo_vision')
    op3_demo_pkg = get_package_share_directory('op3_demo')
    op3_manager_pkg = get_package_share_directory('op3_manager')

    return LaunchDescription([
        # 1. OP3 Manager (robot control)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(op3_manager_pkg, 'launch', 'op3_manager.launch.py')
            )
        ),

        # 2. YOLO Vision (instead of HSV ball detector)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(op3_yolo_pkg, 'launch', 'yolo.launch.py')
            )
        ),

        # 3. YOLO to Demo Bridge (converts YOLO → CircleSetStamped)
        Node(
            package='op3_ball_localization',
            executable='yolo_to_demo_bridge',
            name='yolo_to_demo_bridge',
            output='screen'
        ),

        # 4. OP3 Demo Node (ball following behavior)
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
