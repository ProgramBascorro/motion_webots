# Launch the standalone BALL detector (scripts/ball_detector.py), which runs
# the weights in model/ball_model.
#
# It runs under the "ball_detector_node" namespace and publishes the ball on
# /ball_detector_node/circle_set, so it is a drop-in replacement for the
# classic C++ ball_detector_node and the op3_demo soccer pipeline keeps
# working unchanged.
#
# The FIELD LANDMARK detector is a separate program with its own launch file:
#   ros2 launch op3_ball_detector landmark_detector.launch.py
# Running both at once? Only ONE of them may start the camera -- give the
# second one use_usb_cam:=false.
#
# Examples:
#   ros2 launch op3_ball_detector ball_detector.launch.py
#   # camera already running, publishing a raw Image:
#   ros2 launch op3_ball_detector ball_detector.launch.py \
#        use_usb_cam:=false image_topic:=/usb_cam_node/image_raw use_compressed:=false

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
    TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('op3_ball_detector')

    camera_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'camera_param.yaml'])
    detector_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'ball_detector_yolo_params.yaml'])

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

    # "384,640" = height,width of a rectangular export. Lets you A/B two
    # exports from the command line without editing the YAML.
    imgsz_hw = LaunchConfiguration('imgsz_hw').perform(context).strip()
    if imgsz_hw:
        overrides['imgsz_hw'] = [int(v) for v in imgsz_hw.replace('x', ',').split(',')]

    # taskset only FORBIDS a process from leaving these cores -- it does not
    # reserve them. Whoever sits on the sibling hyperthread still steals from
    # the detector, which is what scripts/op3_cpu_thieves.py measures.
    # Empty = no taskset at all, so this can be turned off without editing.
    def _taskset(arg):
        cpus = LaunchConfiguration(arg).perform(context).strip()
        # One string, not a list: launch concatenates prefix substitutions
        # without separators, so a list would come out as "taskset-c0,2,4".
        return ('taskset -c %s' % cpus) if cpus else None

    det_prefix = _taskset('cpu_affinity')
    aux_prefix = _taskset('aux_cpu_affinity')

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
        prefix=aux_prefix,
        # This C922 drops off the USB bus on its own; usb_cam then throws
        # "Invalid v4l2 format" and dies, and the detector sits at FPS 0.0
        # until someone restarts the launch by hand. Respawning costs nothing
        # when the camera is healthy and recovers the run when it is not.
        # It does NOT fix the disconnect -- that is a cable/port/power
        # problem and still needs looking at.
        respawn=True,
        respawn_delay=2.0,
    )

    # The camera is allowed to stretch its own exposure and drop below the
    # framerate it was asked for -- that is exposure_dynamic_framerate=1, and
    # it is why a camera configured for 30 FPS can deliver 14. usb_cam has no
    # parameter for this control, so it has to be set with v4l2-ctl. It resets
    # to the default whenever the webcam is replugged, so it belongs here (run
    # on every launch) rather than in a one-off container start script.
    # || true: a camera without this control must not fail the launch.
    video_device = LaunchConfiguration('video_device').perform(context)

    def _unbrake(event, ctx):
        # A callable, NOT a fixed list of actions: usb_cam respawns, so this
        # handler fires again, and launch refuses to run the same
        # ExecuteProcess object twice ("executed more than once"). Building a
        # fresh action per start is what makes respawn and this handler able
        # to coexist. Re-running it matters -- the control goes back to its
        # default on every re-enumeration, which is exactly when we respawn.
        return [TimerAction(period=2.0, actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'v4l2-ctl -d %s -c exposure_dynamic_framerate=0 || true'
                 % video_device],
            output='screen')])]

    unbrake_camera = RegisterEventHandler(
        OnProcessStart(target_action=usb_cam_node, on_start=_unbrake),
        condition=IfCondition(LaunchConfiguration('use_usb_cam')),
    )

    detector_node = Node(
        package='op3_ball_detector',
        namespace='ball_detector_node',
        executable='ball_detector.py',
        name='ball_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', LaunchConfiguration('image_topic'))],
        prefix=det_prefix,
    )

    return [usb_cam_node, unbrake_camera, detector_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_usb_cam', default_value='true',
            description='Also start the usb_cam camera node. Set false when '
                        'something else (the demo, or the landmark detector) '
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
                        'weights in model/ball_model.'),
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
        DeclareLaunchArgument(
            'imgsz_hw', default_value='',
            description='Override YAML imgsz_hw as "height,width" for a '
                        'rectangular export (e.g. "384,640"). Empty = use YAML.'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='Which V4L2 device to unbrake with v4l2-ctl. Must '
                        'match video_device in config/camera_param.yaml.'),
        DeclareLaunchArgument(
            'cpu_affinity', default_value='0,2,4',
            description='taskset core list for the ball detector: 3 P-cores '
                        'whose HT siblings (1,3,5) are deliberately left '
                        'empty -- taskset forbids leaving, it does not '
                        'reserve, so what matters is who sits on the sibling '
                        '(scripts/op3_cpu_thieves.py measures exactly that). '
                        'Pass a single space to disable.'),
        DeclareLaunchArgument(
            'aux_cpu_affinity', default_value='12-15',
            description='taskset core list for usb_cam: E-cores, so decoding '
                        'MJPEG never lands on a P-core sibling of the '
                        'detector. Pass a single space to disable.'),
        OpaqueFunction(function=_launch_setup),
    ])
