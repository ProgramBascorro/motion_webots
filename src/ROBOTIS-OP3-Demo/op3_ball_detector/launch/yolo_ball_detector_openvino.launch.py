# Launch the YOLO ball detector using an OpenVINO model (Intel-CPU runtime).
#
# Identical to yolo_ball_detector.launch.py except it loads the OpenVINO
# parameter file (config/yolo_ball_detector_openvino_params.yaml, model_path ->
# model/v8n_openvino_model). The node itself is the same drop-in replacement for
# ball_detector_node: it runs under the "ball_detector_node" namespace and
# publishes the detected ball on /ball_detector_node/circle_set, so the op3_demo
# soccer pipeline keeps working unchanged.
#
# Needs the openvino package in the container (see requirements_yolo.txt) and an
# exported model dir (yolo export model=model/v8n.pt format=openvino imgsz=320).
#
# Examples:
#   ros2 launch op3_ball_detector yolo_ball_detector_openvino.launch.py
#   # already-running camera publishing a raw Image:
#   ros2 launch op3_ball_detector yolo_ball_detector_openvino.launch.py \
#        use_usb_cam:=false image_topic:=/usb_cam_node/image_raw use_compressed:=false

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('op3_ball_detector')

    camera_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'camera_param.yaml'])
    yolo_param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'yolo_ball_detector_openvino_params.yaml'])

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

    # --- pembagian inti CPU ------------------------------------------------
    # i5-10210U ini 4 inti FISIK / 8 utas, dengan pasangan Hyper-Threading
    # 0<->4, 1<->5, 2<->6, 3<->7. Dua utas satu pasangan BERBAGI unit eksekusi,
    # jadi yang menentukan kecepatan YOLO bukan berapa utas yang ia pegang,
    # melainkan siapa yang memakai SIBLING-nya.
    #
    # Terukur di mesin ini (ball_yolov8n @640, OpenVINO LATENCY, 40 frame):
    #   YOLO di 0-3, inti 4-7 sepi ........  40,3 ms
    #   YOLO di 0-3, inti 4-7 sibuk .......  84,2 ms   <- 2,1x lebih lambat
    #
    # Karena itu default lama "0-3" TIDAK melindungi apa pun: ia cuma melarang
    # YOLO memakai 4-7, sementara proses lain tetap bebas mendarat di sana dan
    # merebut unit eksekusi yang sama. (Median 0-3 dan 0-7 praktis identik,
    # 40,7 vs 40,9 ms -- jadi "0-3 avoids HT contention" itu keliru.)
    #
    # Peta yang dipakai sekarang:
    #   inti fisik P0/P1/P2 (utas 0,1,2 + sibling 4,5,6) -> YOLO SAJA
    #   inti fisik P3       (utas 3,7)                   -> usb_cam, demo,
    #                                                       head_tracking, desktop
    # Sibling 4,5,6 sengaja DIKOSONGKAN, bukan terlupa.
    #
    # Terukur dengan pesaing terkurung di 3,7:
    #   median 63 -> 48 ms (15,8 -> 20,9 FPS), p95 114 -> 59 ms (8,7 -> 16,9 FPS)
    #
    # Desktop (Firefox/VS Code/GNOME) adalah pesaing TERBESAR -- lebih besar
    # dari node ROS mana pun. Kurung juga dengan: scripts/op3_docker.sh match-mode on
    #
    # omp_threads cuma membatasi utas torch/numpy untuk pra/pasca-proses; ia
    # TIDAK menyentuh kolam TBB milik OpenVINO (itu diatur oleh taskset).
    cpu_affinity = LaunchConfiguration('cpu_affinity').perform(context)
    aux_cpu_affinity = LaunchConfiguration('aux_cpu_affinity').perform(context)
    omp_threads = LaunchConfiguration('omp_threads').perform(context)
    yolo_extra = {}
    if omp_threads:
        yolo_extra['additional_env'] = {
            'OMP_NUM_THREADS': omp_threads,
            'OPENBLAS_NUM_THREADS': omp_threads,
        }
    if cpu_affinity:
        yolo_extra['prefix'] = 'taskset -c %s' % cpu_affinity

    # usb_cam ikut dikurung ke inti pendamping supaya ia tidak mendarat di
    # sibling milik YOLO. Ongkosnya sendiri kecil (~13% satu inti di 640x360).
    usb_cam_extra = {}
    if aux_cpu_affinity:
        usb_cam_extra['prefix'] = 'taskset -c %s' % aux_cpu_affinity

    usb_cam_node = Node(
        package='usb_cam',
        namespace='usb_cam_node',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[camera_param_path],
        condition=IfCondition(LaunchConfiguration('use_usb_cam')),
        **usb_cam_extra,
    )

    yolo_node = Node(
        package='op3_ball_detector',
        namespace='ball_detector_node',
        executable='yolo_ball_detector.py',
        name='yolo_ball_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', LaunchConfiguration('image_topic'))],
        **yolo_extra,
    )

    # --- lepas rem laju kamera ---------------------------------------------
    # Ini BUKAN soal CPU. Kamera UVC boleh MEMPERLAMBAT DIRI supaya sensornya
    # sempat memajan lebih lama di tempat gelap, dan tidak ada optimasi
    # komputasi apa pun yang bisa menambah frame yang memang tidak pernah
    # dikirim.
    #
    # Terukur di robot ini saat FPS "misterius" turun ke 14-15:
    #   exposure_time_absolute     = 666  (satuan 100 us) -> pajan 66,6 ms
    #                                     -> FPS MAKSIMUM FISIK = 1/0,0666 = 15,0
    #   exposure_dynamic_framerate = 1    (bawaannya 0!)
    #   gain                       = 255  (sudah mentok, tak ada cadangan lain)
    #
    # Rantainya: webcam diarahkan ke area lebih gelap -> auto-exposure
    # memperpanjang pajan -> exposure_dynamic_framerate=1 mengizinkan driver
    # menurunkan laju demi pajan itu -> sensor menyerahkan 15 frame/detik.
    #
    # Menyetelnya ke 0 TIDAK mematikan auto-exposure: kamera tetap menyesuaikan
    # cahaya, cuma tidak boleh lagi melanggar laju yang diminta, jadi pajan
    # terpotong di periode frame (41,7 ms pada 24 FPS). Harganya cahaya:
    # 66,6 -> 41,7 ms = 1,6x lebih sedikit (-0,68 EV), dan gain sudah mentok
    # jadi tidak bisa ditebus. Kalau conf deteksi ikut jatuh, turunkan
    # framerate di camera_param.yaml kembali ke 20.0 (pajan 50 ms).
    #
    # usb_cam TIDAK punya parameter untuk kontrol ini (ia cuma mengenal
    # autoexposure/exposure), jadi harus lewat v4l2-ctl.
    #
    # Dipasang di SINI, bukan di op3_docker.sh up: kontrol ini menempel pada
    # perangkat dan kembali ke bawaan begitu webcam dicabut-pasang. Di launch,
    # ia dipasang ulang tiap kali detektor dijalankan. Dijalankan SESUDAH
    # usb_cam membuka perangkat (OnProcessStart + jeda) supaya setelan kita
    # yang mendarat terakhir; `|| true` supaya kamera yang tidak punya kontrol
    # ini tidak menggagalkan launch.
    camera_device = LaunchConfiguration('camera_device').perform(context)
    uncap_fps = RegisterEventHandler(
        OnProcessStart(
            target_action=usb_cam_node,
            on_start=[TimerAction(period=2.0, actions=[ExecuteProcess(
                cmd=['bash', '-c',
                     'v4l2-ctl -d %s -c exposure_dynamic_framerate=0 '
                     '&& echo "[kamera] rem laju dilepas '
                     '(exposure_dynamic_framerate=0)" || true' % camera_device],
                output='screen',
            )])],
        ),
        condition=IfCondition(LaunchConfiguration('use_usb_cam')),
    )

    return [usb_cam_node, uncap_fps, yolo_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_usb_cam', default_value='true',
            description='Also start the usb_cam camera node.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/usb_cam_node/image_raw/compressed',
            description='Image topic the detector subscribes to (image_in).'),
        DeclareLaunchArgument(
            'camera_device', default_value='/dev/video0',
            description='V4L2 device the frame-rate uncap (v4l2-ctl '
                        'exposure_dynamic_framerate=0) is applied to. Must '
                        'match video_device in camera_param.yaml. Only used '
                        'when use_usb_cam is true.'),
        # The next three default to "" = use whatever the OpenVINO YAML says.
        # Pass a value only to override the YAML for this run.
        DeclareLaunchArgument(
            'use_compressed', default_value='',
            description='Override YAML: "true" for CompressedImage, "false" '
                        'for raw Image. Empty = use YAML.'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Override YAML model_path (abs path, or relative to '
                        'the package share dir, e.g. model/v8n_openvino_model). '
                        'Empty = use YAML.'),
        DeclareLaunchArgument(
            'device', default_value='',
            description='Override YAML device (cpu). Empty = use YAML.'),
        DeclareLaunchArgument(
            'omp_threads', default_value='2',
            description='Cap torch/numpy (OpenMP/BLAS) threads for pre/post-'
                        'processing so YOLO leaves CPU for walking. Empty = no cap.'),
        DeclareLaunchArgument(
            'cpu_affinity', default_value='0-2',
            description='taskset core list pinning ALL detector threads '
                        '(including the OpenVINO TBB pool). Default "0-2" = one '
                        'thread on each of physical cores P0/P1/P2; their HT '
                        'siblings 4,5,6 are deliberately left empty because a '
                        'busy sibling measured 40 -> 84 ms per frame. Set empty '
                        'to disable, or adjust for other CPUs.'),
        DeclareLaunchArgument(
            'aux_cpu_affinity', default_value='3,7',
            description='taskset core list for the companion nodes (usb_cam '
                        'here; demo/head_tracking get the same treatment in '
                        'demo.launch.xml). "3,7" = both threads of physical '
                        'core P3, keeping them off the cores YOLO owns. Empty '
                        'to disable.'),
        OpaqueFunction(function=_launch_setup),
    ])
