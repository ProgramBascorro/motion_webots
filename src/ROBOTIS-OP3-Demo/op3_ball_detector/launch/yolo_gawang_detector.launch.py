# Launch detektor GAWANG mandiri (OpenVINO INT8, ball_gawang_v3 288x480).
#
# Menjalankan scripts/yolo_gawang_detector.py di namespace
# "gawang_detector_node" dan menerbitkan:
#
#   /gawang_detector_node/detections     vision_msgs/Detection2DArray (piksel)
#   /gawang_detector_node/gawang_center  geometry_msgs/PointStamped [0,1], z = conf
#   /gawang_detector_node/fps            std_msgs/Float32
#   /gawang_detector_node/image_out      sensor_msgs/Image (beranotasi)
#
# TIDAK disambung ke demo mana pun dan tidak menyentuh jalur bola.
#
# Kamera: use_usb_cam:=auto (bawaan) hanya menyalakan usb_cam kalau belum ada
# yang menerbitkan image_topic -- jadi launch ini bisa jalan sendiri, dan juga
# bisa jalan di samping detektor bola tanpa berebut /dev/video0.
#
# Contoh:
#   ros2 launch op3_ball_detector yolo_gawang_detector.launch.py
#   ros2 launch op3_ball_detector yolo_gawang_detector.launch.py confidence_threshold:=0.45
#   ros2 run rqt_image_view rqt_image_view /gawang_detector_node/image_out

import os
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _topic_has_publisher(topic, settle_sec=1.5):
    """Sudah ada yang menerbitkan ``topic``?

    Pakai context rclpy SENDIRI -- menginisialisasi context global akan
    mengganggu launch_ros. Gagal apa pun = False: kamera kedua gagal dengan
    keras, kamera yang tidak ada membuat detektor kelaparan diam-diam.
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
    choice = (use_usb_cam or 'auto').strip().lower()
    if choice in ('1', 'true', 'yes', 'on'):
        return True, 'dipaksa use_usb_cam:=%s' % choice
    if choice in ('0', 'false', 'no', 'off'):
        return False, 'dimatikan use_usb_cam:=%s' % choice
    if _topic_has_publisher(image_topic):
        return False, 'auto: "%s" sudah ada penerbitnya' % image_topic
    return True, 'auto: belum ada yang menerbitkan "%s"' % image_topic


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('op3_ball_detector')
    camera_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'camera_param.yaml'])
    params_file = (LaunchConfiguration('params_file').perform(context)
                   or 'yolo_gawang_detector_params.yaml')
    yolo_param_path = (
        params_file if os.path.isabs(params_file)
        else PathJoinSubstitution([pkg_share, 'config', params_file]))

    # YAML = sumber kebenaran; argumen CLI hanya menimpa kalau diisi.
    overrides = {}
    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path:
        overrides['model_path'] = model_path
    use_compressed = LaunchConfiguration('use_compressed').perform(context)
    if use_compressed:
        overrides['use_compressed'] = use_compressed.lower() in ('1', 'true', 'yes')
    confidence = LaunchConfiguration('confidence_threshold').perform(context)
    if confidence:
        overrides['confidence_threshold'] = float(confidence)
    draw_other = LaunchConfiguration('draw_other_classes').perform(context)
    if draw_other:
        overrides['draw_other_classes'] = draw_other.lower() in ('1', 'true', 'yes')
    node_params = [yolo_param_path]
    if overrides:
        node_params.append(overrides)

    # --- pembagian inti CPU: sama dengan detektor bola ---------------------
    # i5-10210U 4 inti fisik / 8 utas, sibling 0<->4 1<->5 2<->6 3<->7. YOLO
    # di 0-2 dengan sibling 4,5,6 sengaja kosong; pendamping (usb_cam) di 3,7.
    # Sibling yang sibuk terukur 40 -> 84 ms per frame. Kalau detektor bola
    # juga hidup, keduanya berbagi 0-2 -- itu disengaja: mendaratkan YOLO
    # kedua di 4-6 justru menggerus YOLO pertama lewat unit eksekusi bersama.
    cpu_affinity = LaunchConfiguration('cpu_affinity').perform(context)
    aux_cpu_affinity = LaunchConfiguration('aux_cpu_affinity').perform(context)
    omp_threads = LaunchConfiguration('omp_threads').perform(context)
    yolo_extra = {'additional_env': {
        # Jangan biarkan ultralytics pip-install apa pun saat memuat model:
        # ia menyeret NumPy 2 dan merusak cv_bridge untuk seluruh workspace.
        'YOLO_AUTOINSTALL': 'false',
    }}
    if omp_threads:
        yolo_extra['additional_env'].update({
            'OMP_NUM_THREADS': omp_threads,
            'OPENBLAS_NUM_THREADS': omp_threads,
        })
    if cpu_affinity:
        yolo_extra['prefix'] = 'taskset -c %s' % cpu_affinity
    usb_cam_extra = {}
    if aux_cpu_affinity:
        usb_cam_extra['prefix'] = 'taskset -c %s' % aux_cpu_affinity

    start_camera, camera_reason = _decide_camera(
        LaunchConfiguration('use_usb_cam').perform(context),
        LaunchConfiguration('image_topic').perform(context))

    gawang_node = Node(
        package='op3_ball_detector',
        namespace='gawang_detector_node',
        executable='yolo_gawang_detector.py',
        name='yolo_gawang_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', LaunchConfiguration('image_topic'))],
        **yolo_extra,
    )

    actions = [LogInfo(msg='[kamera] usb_cam %s (%s)'
                       % ('DINYALAKAN' if start_camera else 'tidak dinyalakan',
                          camera_reason))]
    if start_camera:
        usb_cam_node = Node(
            package='usb_cam',
            namespace='usb_cam_node',
            executable='usb_cam_node_exe',
            output='screen',
            parameters=[camera_param_path],
            **usb_cam_extra,
        )
        # Lepas rem laju kamera: tanpa ini UVC boleh memperpanjang pajan di
        # tempat gelap dan FPS turun ke 1/pajan (terukur 15 FPS). Ini kontrol
        # perangkat yang hilang saat webcam dicabut, dan usb_cam tidak punya
        # parameternya -- maka v4l2-ctl, SESUDAH usb_cam membuka perangkat.
        # Lihat yolo_ball_detector_openvino.launch.py untuk angka lengkapnya.
        uncap_fps = RegisterEventHandler(OnProcessStart(
            target_action=usb_cam_node,
            on_start=[TimerAction(period=2.0, actions=[ExecuteProcess(
                cmd=['bash', '-c',
                     'v4l2-ctl -d %s -c exposure_dynamic_framerate=0 '
                     '&& echo "[kamera] rem laju dilepas '
                     '(exposure_dynamic_framerate=0)" || true'
                     % LaunchConfiguration('camera_device').perform(context)],
                output='screen',
            )])],
        ))
        actions += [usb_cam_node, uncap_fps]
    actions.append(gawang_node)

    # rqt_image_view ikut dinyalakan, di inti pendamping (3,7) supaya tidak
    # merebut YOLO. Menutup jendelanya = mematikan seluruh launch, jadi tidak
    # ada detektor yatim yang terus memakan CPU tanpa ada yang menonton.
    if LaunchConfiguration('rqt').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        rqt_cmd = ['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                   '/gawang_detector_node/image_out']
        if aux_cpu_affinity:
            rqt_cmd = ['taskset', '-c', aux_cpu_affinity] + rqt_cmd
        rqt = ExecuteProcess(cmd=rqt_cmd, output='screen')
        actions += [rqt, RegisterEventHandler(OnProcessExit(
            target_action=rqt,
            on_exit=[LogInfo(msg='[rqt] jendela ditutup (atau tampilan tidak '
                                 'bisa dibuka) -- detektor ikut dihentikan'),
                     EmitEvent(event=Shutdown(reason='rqt ditutup'))],
        ))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_usb_cam', default_value='auto',
            description='auto (bawaan) = nyalakan usb_cam hanya kalau belum '
                        'ada yang menerbitkan image_topic. true/false memaksa.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/usb_cam_node/image_raw/compressed',
            description='Topik gambar yang dilanggan (image_in).'),
        DeclareLaunchArgument(
            'camera_device', default_value='/dev/video0',
            description='Perangkat V4L2 untuk pelepas rem laju; harus sama '
                        'dengan video_device di camera_param.yaml.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Nama berkas di config/ atau path absolut. Kosong = '
                        'yolo_gawang_detector_params.yaml.'),
        DeclareLaunchArgument(
            'use_compressed', default_value='',
            description='Timpa YAML: "true" CompressedImage, "false" Image. '
                        'Kosong = pakai YAML.'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Timpa YAML model_path. Kosong = pakai YAML.'),
        DeclareLaunchArgument(
            'confidence_threshold', default_value='',
            description='Timpa YAML confidence_threshold. Kosong = pakai YAML.'),
        DeclareLaunchArgument(
            'draw_other_classes', default_value='',
            description='Timpa YAML: "true" = bola ikut DIGAMBAR di image_out '
                        '(tetap tidak diterbitkan). Kosong = pakai YAML.'),
        DeclareLaunchArgument(
            'omp_threads', default_value='2',
            description='Batas utas OpenMP/BLAS untuk pra/pasca-proses. '
                        'Kosong = tanpa batas.'),
        DeclareLaunchArgument(
            'cpu_affinity', default_value='0-2',
            description='taskset untuk SEMUA utas detektor (termasuk kolam TBB '
                        'OpenVINO). Kosong = tidak dipatok.'),
        DeclareLaunchArgument(
            'aux_cpu_affinity', default_value='3,7',
            description='taskset untuk usb_cam (kalau launch ini yang '
                        'menyalakannya) dan rqt. Kosong = tidak dipatok.'),
        DeclareLaunchArgument(
            'rqt', default_value='false',
            description='true = buka rqt_image_view pada image_out; menutup '
                        'jendelanya menghentikan seluruh launch.'),
        OpaqueFunction(function=_launch_setup),
    ])
