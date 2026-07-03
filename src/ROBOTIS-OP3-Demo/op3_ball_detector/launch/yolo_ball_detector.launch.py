# Launch the YOLO ball detector together with a usb_cam camera node.
#
# The YOLO node is a drop-in replacement for the classic ball_detector_node:
# it runs under the "ball_detector_node" namespace and publishes the detected
# ball on /ball_detector_node/circle_set, so the existing op3_demo soccer
# pipeline keeps working unchanged.
#
# Examples:
#   ros2 launch op3_ball_detector yolo_ball_detector.launch.py
#   ros2 launch op3_ball_detector yolo_ball_detector.launch.py use_usb_cam:=false \
#        image_topic:=/usb_cam_node/image_raw use_compressed:=false
#   ros2 launch op3_ball_detector yolo_ball_detector.launch.py device:=cuda:0 \
#        model_path:=/abs/path/to/best.pt

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('op3_ball_detector')

    camera_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'camera_param.yaml'])
    yolo_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'yolo_ball_detector_params.yaml'])

    # The YAML (config/yolo_ball_detector_params.yaml) is the single source of
    # truth for the node parameters. CLI args override the YAML ONLY when given
    # a non-empty value, so editing the YAML actually takes effect.
    overrides = {}

    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path:
        overrides['model_path'] = model_path

    device = LaunchConfiguration('device').perform(context)
    if device:
        overrides['device'] = device

    use_compressed = LaunchConfiguration('use_compressed').perform(context)
    if use_compressed:
        overrides['use_compressed'] = use_compressed.lower() in ('1', 'true', 'yes')

    node_params = [yolo_param_path]
    if overrides:
        node_params.append(overrides)

    usb_cam_node = Node(
        package='usb_cam',
        namespace='usb_cam_node',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[camera_param_path],
        condition=IfCondition(LaunchConfiguration('use_usb_cam')),
    )

    yolo_node = Node(
        package='op3_ball_detector',
        namespace='ball_detector_node',
        executable='yolo_ball_detector.py',
        name='yolo_ball_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', LaunchConfiguration('image_topic'))],
    )

    return [usb_cam_node, yolo_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_usb_cam', default_value='true',
            description='Also start the usb_cam camera node.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/usb_cam_node/image_raw/compressed',
            description='Image topic the detector subscribes to (image_in).'),
        # The next three default to "" = use whatever the YAML says. Pass a
        # value only to override the YAML for this run.
        DeclareLaunchArgument(
            'use_compressed', default_value='',
            description='Override YAML: "true" for CompressedImage, "false" '
                        'for raw Image. Empty = use YAML.'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Override YAML model_path (abs path, or relative to '
                        'the package share dir). Empty = use YAML.'),
        DeclareLaunchArgument(
            'device', default_value='',
            description='Override YAML device, e.g. "cpu" or "cuda:0". '
                        'Empty = use YAML.'),
        OpaqueFunction(function=_launch_setup),
    ])
