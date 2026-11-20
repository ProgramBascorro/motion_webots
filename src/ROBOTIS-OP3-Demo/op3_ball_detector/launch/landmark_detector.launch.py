# Launch the standalone FIELD LANDMARK detector
# (scripts/landmark_detector.py), which runs the weights in
# model/landmark_deploy and knows five classes: L, T, X, goalpost,
# center_circle.
#
# It runs under the "landmark_detector_node" namespace and publishes
# /landmark_detector_node/detections (vision_msgs/Detection2DArray) plus
# /landmark_detector_node/fps (std_msgs/Float32).
#
# The BALL detector is a separate program with its own launch file:
#   ros2 launch op3_ball_detector ball_detector.launch.py
# Running both at once? Only ONE of them may start the camera -- give the
# second one use_usb_cam:=false.
#
# Examples:
#   ros2 launch op3_ball_detector landmark_detector.launch.py
#   # camera already running (e.g. the ball detector started it):
#   ros2 launch op3_ball_detector landmark_detector.launch.py use_usb_cam:=false

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
    detector_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'landmark_detector_params.yaml'])

    # The YAML is the single source of truth. CLI args override it ONLY when
    # given a non-empty value, so editing the YAML actually takes effect.
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

    detection_rate = LaunchConfiguration('detection_rate').perform(context)
    if detection_rate:
        overrides['detection_rate'] = float(detection_rate)

    fps_log_period = LaunchConfiguration('fps_log_period').perform(context)
    if fps_log_period:
        overrides['fps_log_period'] = float(fps_log_period)

    node_params = [detector_param_path]
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

    detector_node = Node(
        package='op3_ball_detector',
        namespace='landmark_detector_node',
        executable='landmark_detector.py',
        name='landmark_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', LaunchConfiguration('image_topic'))],
    )

    return [usb_cam_node, detector_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_usb_cam', default_value='true',
            description='Also start the usb_cam camera node. Set false when '
                        'something else (the demo, or the ball detector) '
                        'already owns the camera.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/usb_cam_node/image_raw/compressed',
            description='Image topic the detector subscribes to (image_in).'),
        # The rest default to "" = use whatever the YAML says. Pass a value
        # only to override the YAML for this run.
        DeclareLaunchArgument(
            'use_compressed', default_value='',
            description='Override YAML: "true" for CompressedImage, "false" '
                        'for raw Image. Empty = use YAML.'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Override YAML model_path (absolute, or relative to '
                        'the package share dir). Empty = auto-pick the best '
                        'weights in model/landmark_deploy.'),
        DeclareLaunchArgument(
            'device', default_value='',
            description='Override YAML device (cpu). Empty = use YAML.'),
        DeclareLaunchArgument(
            'detection_rate', default_value='',
            description='Override YAML detection_rate in Hz. Empty = use YAML.'),
        DeclareLaunchArgument(
            'fps_log_period', default_value='',
            description='Override how often the FPS line is logged, in '
                        'seconds. Empty = use YAML (5.0).'),
        OpaqueFunction(function=_launch_setup),
    ])
