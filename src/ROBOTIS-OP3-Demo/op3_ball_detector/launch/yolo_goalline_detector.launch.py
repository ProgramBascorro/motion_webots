# Launch the GOAL-LINE (garis gawang) detector.
#
# Runs scripts/yolo_goalline_detector.py under the "goalline_detector_node"
# namespace with the best_landmark_v8n model from model/landmark_deploy.zip, and
# publishes:
#
#   /goalline_detector_node/goal_line    geometry_msgs/PolygonStamped (pixels)
#   /goalline_detector_node/goal_center  geometry_msgs/PointStamped   (normalized)
#   /goalline_detector_node/goal_found   std_msgs/Bool
#   /goalline_detector_node/detections   vision_msgs/Detection2DArray
#   /goalline_detector_node/fps          std_msgs/Float32
#   /goalline_detector_node/image_out    sensor_msgs/Image (annotated)
#
# It does NOT touch the ball pipeline, so it is meant to run ALONGSIDE the ball
# detector — which usually already owns the camera, hence use_usb_cam:=false by
# default.
#
# Examples:
#   # alongside an already-running camera
#   ros2 launch op3_ball_detector yolo_goalline_detector.launch.py
#   # standalone (also start the camera)
#   ros2 launch op3_ball_detector yolo_goalline_detector.launch.py use_usb_cam:=true
#   # watch it: rqt_image_view /goalline_detector_node/image_out
#   #           ros2 topic echo /goalline_detector_node/fps

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _topic_has_publisher(topic, settle_sec=1.5):
    """Is anything already publishing on ``topic``?

    Launch itself has no view of the ROS graph, so ask with a throwaway node on
    its OWN rclpy context — initialising the global one would interfere with
    launch_ros. Discovery is not instant, hence the settle loop.

    Returns False on any failure: the caller then starts the camera, which is
    the recoverable direction to be wrong in (a second camera node fails loudly;
    a missing one leaves the detector silently starving).
    """
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node as RclpyNode
    except ImportError:
        return False

    ctx = Context()
    try:
        rclpy.init(context=ctx)
        # A node name may not start with "_", and must be unique on the graph.
        probe = RclpyNode('camera_probe_%d' % os.getpid(), context=ctx)
        deadline = time.time() + settle_sec
        count = 0
        while time.time() < deadline:
            count = probe.count_publishers(topic)
            if count:
                break
            time.sleep(0.1)
        probe.destroy_node()
        return count > 0
    except Exception:  # noqa: BLE001 - probing must never break the launch
        return False
    finally:
        try:
            rclpy.shutdown(context=ctx)
        except Exception:  # noqa: BLE001
            pass


def _decide_camera(use_usb_cam, image_topic):
    """Resolve use_usb_cam (true / false / auto) into a decision + a reason."""
    choice = (use_usb_cam or 'auto').strip().lower()
    if choice in ('1', 'true', 'yes', 'on'):
        return True, 'forced by use_usb_cam:=%s' % choice
    if choice in ('0', 'false', 'no', 'off'):
        return False, 'suppressed by use_usb_cam:=%s' % choice
    # "auto": start the camera only when nobody else is feeding that topic.
    # This is the default because the alternative bit us repeatedly — launching
    # with no camera anywhere leaves the detector waiting forever on a topic
    # that exists (its own subscription creates it) but carries nothing.
    if _topic_has_publisher(image_topic):
        return False, 'auto: "%s" already has a publisher' % image_topic
    return True, 'auto: nothing publishes "%s" yet' % image_topic


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('op3_ball_detector')

    camera_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'camera_param.yaml'])

    # The YAML profile is the single source of truth for the node parameters;
    # it carries model_path AND imgsz together, which matters because models
    # exported at different sizes cannot share one file.
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = 'yolo_goalline_detector_params.yaml'
    yolo_param_path = (
        params_file if os.path.isabs(params_file)
        else PathJoinSubstitution([pkg_share, 'config', params_file]))

    # CLI args override the YAML ONLY when given a non-empty value, so editing
    # the YAML actually takes effect.
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

    node_params = [yolo_param_path]
    if overrides:
        node_params.append(overrides)

    # Keep YOLO from monopolising the CPU so the walking controller stays
    # responsive. omp_threads caps the torch/numpy pre/post-processing threads;
    # cpu_affinity (taskset) pins ALL detector threads — including OpenVINO's
    # TBB pool — to a core subset, leaving the rest for walking.
    cpu_affinity = LaunchConfiguration('cpu_affinity').perform(context)
    omp_threads = LaunchConfiguration('omp_threads').perform(context)
    yolo_extra = {}
    if omp_threads:
        yolo_extra['additional_env'] = {
            'OMP_NUM_THREADS': omp_threads,
            'OPENBLAS_NUM_THREADS': omp_threads,
            # Never let ultralytics pip-install anything at load time: it drags
            # in NumPy 2 and breaks cv_bridge for every node in the workspace.
            'YOLO_AUTOINSTALL': 'false',
        }
    if cpu_affinity:
        yolo_extra['prefix'] = 'taskset -c %s' % cpu_affinity

    # Decide whether this launch must bring the camera up itself. Only ONE
    # process may hold /dev/video0, so we start it only when nothing is
    # already publishing the image topic.
    start_camera, camera_reason = _decide_camera(
        LaunchConfiguration('use_usb_cam').perform(context),
        LaunchConfiguration('image_topic').perform(context))

    usb_cam_node = Node(
        package='usb_cam',
        namespace='usb_cam_node',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[camera_param_path],
    )

    goalline_node = Node(
        package='op3_ball_detector',
        namespace='goalline_detector_node',
        executable='yolo_goalline_detector.py',
        name='yolo_goalline_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', LaunchConfiguration('image_topic'))],
        **yolo_extra,
    )

    actions = [LogInfo(msg='[camera] %s the usb_cam node (%s)'
                       % ('starting' if start_camera else 'NOT starting',
                          camera_reason))]
    if start_camera:
        actions.append(usb_cam_node)
    actions.append(goalline_node)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_usb_cam', default_value='auto',
            description='Start the usb_cam camera node too. "auto" (default) '
                        'starts it ONLY when nothing is publishing image_topic '
                        'yet, so running this launch on its own just works, '
                        'while running it beside the ball detector does not '
                        'fight over /dev/video0. Force with true/false.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/usb_cam_node/image_raw/compressed',
            description='Image topic the detector subscribes to (image_in).'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Parameter profile: a file name inside config/ or an '
                        'absolute path. Empty = '
                        'yolo_goalline_detector_params.yaml.'),
        # The rest default to "" = use whatever the YAML says. Pass a value only
        # to override the YAML for this one run.
        DeclareLaunchArgument(
            'use_compressed', default_value='',
            description='Override YAML: "true" for CompressedImage, "false" '
                        'for raw Image. Empty = use YAML.'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Override YAML model_path (abs path, or relative to the '
                        'package share dir, e.g. '
                        'model/best_landmark_v8n_openvino_model). '
                        'Empty = use YAML.'),
        DeclareLaunchArgument(
            'device', default_value='',
            description='Override YAML device (cpu). Empty = use YAML.'),
        DeclareLaunchArgument(
            'detection_rate', default_value='',
            description='Override YAML detection_rate in Hz. Empty = use YAML.'),
        DeclareLaunchArgument(
            'fps_log_period', default_value='',
            description='Override YAML fps_log_period (s between FPS log '
                        'lines). Empty = use YAML.'),
        DeclareLaunchArgument(
            'omp_threads', default_value='2',
            description='Cap torch/numpy (OpenMP/BLAS) threads for pre/post-'
                        'processing so YOLO leaves CPU for walking. Empty = no cap.'),
        DeclareLaunchArgument(
            'cpu_affinity', default_value='0-3',
            description='taskset core list to pin ALL detector threads, leaving '
                        'the rest for the controller. Default "0-3" measured ~2x '
                        'faster inference on this 4c/8t i5 (avoids HT contention). '
                        'Set empty to disable, or adjust for other CPUs.'),
        OpaqueFunction(function=_launch_setup),
    ])
