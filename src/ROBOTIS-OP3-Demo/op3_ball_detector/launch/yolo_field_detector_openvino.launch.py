# Launch the YOLO FIELD (lapangan) detector using an OpenVINO model.
#
# This is the multi-class sibling of yolo_ball_detector_openvino.launch.py:
# it runs yolo_field_detector.py under the "field_detector_node" namespace and
# publishes vision_msgs/Detection2DArray (bbox + class label + score) on
# /field_detector_node/detections. It does NOT touch the ball pipeline, so it is
# meant to run ALONGSIDE the ball detector.
#
# Because the ball detector (or usb_cam) usually already owns the camera, this
# launch does NOT start usb_cam by default (use_usb_cam:=false). It just
# subscribes to the existing camera stream. Set use_usb_cam:=true to also start
# the camera when running the field detector standalone.
#
# Needs the openvino package in the container (see requirements_yolo.txt) and an
# exported model dir:
#   yolo export model=model/v8n_lapangan_best.pt format=openvino imgsz=320
#
# Examples:
#   # alongside an already-running camera:
#   ros2 launch op3_ball_detector yolo_field_detector_openvino.launch.py
#   # standalone (also start the camera):
#   ros2 launch op3_ball_detector yolo_field_detector_openvino.launch.py \
#        use_usb_cam:=true

import os

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
    # Which parameter profile to load. Default = the lapangan model, exactly as
    # before. Pass params_file:=<file-in-config/ or abs path> to try a different
    # model profile (e.g. yolo_landmark_detector_openvino_params.yaml) without
    # editing anything. The profile carries model_path AND imgsz together, which
    # matters: models exported at different sizes cannot share one YAML.
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = 'yolo_field_detector_openvino_params.yaml'
    yolo_param_path = (
        params_file if os.path.isabs(params_file)
        else PathJoinSubstitution([pkg_share, 'config', params_file]))

    # The OpenVINO YAML is the single source of truth for the node parameters.
    # CLI args override it ONLY when given a non-empty value, so editing the
    # YAML actually takes effect.
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

    # Keep YOLO from monopolising the CPU so the walking controller stays
    # responsive. omp_threads caps the torch/numpy pre/post-processing threads;
    # cpu_affinity (taskset) optionally pins ALL detector threads - including
    # OpenVINO's TBB pool - to a core subset, leaving the rest for walking.
    cpu_affinity = LaunchConfiguration('cpu_affinity').perform(context)
    omp_threads = LaunchConfiguration('omp_threads').perform(context)
    yolo_extra = {}
    if omp_threads:
        yolo_extra['additional_env'] = {
            'OMP_NUM_THREADS': omp_threads,
            'OPENBLAS_NUM_THREADS': omp_threads,
        }
    if cpu_affinity:
        yolo_extra['prefix'] = 'taskset -c %s' % cpu_affinity

    usb_cam_node = Node(
        package='usb_cam',
        namespace='usb_cam_node',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[camera_param_path],
        condition=IfCondition(LaunchConfiguration('use_usb_cam')),
    )

    field_node = Node(
        package='op3_ball_detector',
        namespace='field_detector_node',
        executable='yolo_field_detector.py',
        name='yolo_field_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', LaunchConfiguration('image_topic'))],
        **yolo_extra,
    )

    return [usb_cam_node, field_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_usb_cam', default_value='false',
            description='Also start the usb_cam camera node. Default false: the '
                        'ball detector usually already owns the camera.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/usb_cam_node/image_raw/compressed',
            description='Image topic the detector subscribes to (image_in).'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Parameter profile: a file name inside config/ or an '
                        'absolute path. Empty = '
                        'yolo_field_detector_openvino_params.yaml (lapangan). '
                        'Use yolo_landmark_detector_openvino_params.yaml for '
                        'the best_landmark_v8n model.'),
        # The next three default to "" = use whatever the OpenVINO YAML says.
        # Pass a value only to override the YAML for this run.
        DeclareLaunchArgument(
            'use_compressed', default_value='',
            description='Override YAML: "true" for CompressedImage, "false" '
                        'for raw Image. Empty = use YAML.'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Override YAML model_path (abs path, or relative to '
                        'the package share dir, e.g. '
                        'model/v8n_lapangan_best_openvino_model). '
                        'Empty = use YAML.'),
        DeclareLaunchArgument(
            'device', default_value='',
            description='Override YAML device (cpu). Empty = use YAML.'),
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
