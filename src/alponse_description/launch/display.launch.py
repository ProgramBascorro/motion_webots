from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("alponse_description"))
    urdf_path = package_share / "urdf" / "alponse.urdf"
    rviz_path = package_share / "rviz" / "alponse.rviz"

    robot_description = {
        "robot_description": urdf_path.read_text(encoding="utf-8")
    }

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start joint_state_publisher_gui for manual joint testing.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 with the Alponse robot model display.",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(rviz_path)],
            condition=IfCondition(rviz),
        ),
    ])
