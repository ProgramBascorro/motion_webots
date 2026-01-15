from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software = LaunchConfiguration("rviz_software")
    rviz_config = LaunchConfiguration("rviz_config")

    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    field_point_cloud_topic = LaunchConfiguration("field_point_cloud_topic")

    odom_topic = LaunchConfiguration("odom_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    field_marker_topic = LaunchConfiguration("field_marker_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_software", default_value="1"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("soccer_localization"), "rviz", "op3_localization.rviz"]
                ),
            ),
            DeclareLaunchArgument("image_topic", default_value="/robotis_op3/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/robotis_op3/camera/camera_info"),
            DeclareLaunchArgument("debug_image_topic", default_value="/vision/lines/debug"),
            DeclareLaunchArgument("field_point_cloud_topic", default_value="/field_point_cloud"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base"),
            DeclareLaunchArgument("camera_frame", default_value="cam_link"),
            DeclareLaunchArgument("field_marker_topic", default_value="/field_map/markers"),
            Node(
                package="soccer_object_localization",
                executable="detector_fieldline",
                name="fieldline_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "debug_image_topic": debug_image_topic,
                        "point_cloud_topic": field_point_cloud_topic,
                        "base_frame": base_frame,
                        "camera_frame": camera_frame,
                    }
                ],
            ),
            Node(
                package="soccer_localization",
                executable="field_lines_ukf",
                name="field_lines_ukf",
                output="screen",
                parameters=[
                    {
                        "odom_topic": odom_topic,
                        "field_point_cloud_topic": field_point_cloud_topic,
                        "map_frame": map_frame,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                    }
                ],
            ),
            Node(
                package="soccer_localization",
                executable="field_markers",
                name="field_markers",
                output="screen",
                parameters=[
                    {
                        "frame_id": map_frame,
                        "marker_topic": field_marker_topic,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                additional_env={"LIBGL_ALWAYS_SOFTWARE": rviz_software},
                condition=IfCondition(use_rviz),
            ),
        ]
    )
