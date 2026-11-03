#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 ROBOTIS / Bascorro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""YOLO-based ball detector for ROBOTIS OP3.

This node is a drop-in replacement for the classic Hough-circle
``ball_detector_node`` (C++). It runs an Ultralytics YOLO model on the
camera stream and publishes the detected ball(s) as
``op3_ball_detector_msgs/CircleSetStamped`` on ``circle_set`` using the very
same convention as the original detector so the existing op3_demo
ball_tracker / ball_follower pipeline keeps working unchanged:

    circles[i].x : normalized x in [-1, 1]   (image left -1 ... right +1)
    circles[i].y : normalized y in [-1, 1]   (image top  -1 ... bottom +1)
    circles[i].z : ball radius in pixels

An annotated debug image is published on ``image_out`` and the node honors
the ``enable`` (std_msgs/Bool) topic exactly like the C++ detector.

FPS REPORTING — reported IN FPS, not as a bare millisecond latency:
  * at start-up the model is benchmarked and its ceiling logged
    ("model ceiling: NN.N FPS"),
  * every ``fps_log_period`` seconds a live line is logged and the rate is
    published on ``fps`` (std_msgs/Float32),
  * on shutdown (Ctrl-C included) a summary block reports the average, min/max
    and the pre/infer/post millisecond breakdown BEHIND that rate.
Milliseconds still appear — as the explanation for the frame rate, never as
the headline, because "43 ms" does not tell you whether the loop kept up.

Default model: <op3_ball_detector share>/model/ball_yolov8n_best.pt
(class 0 = "ball")

Run under the ``ball_detector_node`` namespace so it publishes on
``/ball_detector_node/circle_set`` (see yolo_ball_detector.launch.py).
"""

import math
import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, Float32, Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PointStamped
from op3_ball_detector_msgs.msg import CircleSetStamped

from cv_bridge import CvBridge

# detector_fps.py is installed into the SAME directory as this script
# (lib/op3_ball_detector), and sits next to it in scripts/ when run from source.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detector_fps import FpsMeter  # noqa: E402


class YoloBallDetector(Node):
    """Detects the ball with a YOLO model and republishes it as circles."""

    def __init__(self):
        super().__init__('yolo_ball_detector')

        # ----- parameters -------------------------------------------------
        self.model_path = str(self.declare_parameter('model_path', '').value)
        # Task for the model. .onnx files carry no task metadata, so ultralytics
        # otherwise warns and guesses; set it explicitly. Empty = let ultralytics
        # infer (only works for .pt).
        self.model_task = str(
            self.declare_parameter('model_task', 'detect').value).strip()
        self.use_compressed = bool(
            self.declare_parameter('use_compressed', True).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.5).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        # Kedua ambang di atas BOLEH diubah selagi node berjalan, lewat
        # /ball_detector_node/yolo_ball_detector/set_parameters (halaman Vision
        # di Bascorro Studio memakainya). Tanpa callback ini nilainya cuma
        # tersimpan di parameter dan tidak pernah sampai ke inferensi -- itu
        # keadaannya sampai 2026-09-08, dan slider di Studio memang tidak
        # berpengaruh apa pun.
        #
        # Yang lain SENGAJA tidak dibuka: model_path, imgsz dan device dipakai
        # saat memuat model, detection_rate saat membuat timer. Mengubahnya
        # lewat parameter tidak akan berpengaruh, jadi lebih baik ditolak
        # terang-terangan daripada diam-diam tidak berguna.
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.imgsz = int(self.declare_parameter('imgsz', 640).value)
        # --- masukan PERSEGI PANJANG ------------------------------------------
        # Kamera 640x360, tapi imgsz persegi memaksa Ultralytics mem-padding
        # jadi 640x640 sebelum inferensi: 230400 dari 409600 piksel yang
        # berguna, jadi 44% komputasi habis memproses bar abu-abu.
        #
        # imgsz_hw [tinggi, lebar] memakai bentuk yang mengikuti rasio kamera.
        # Skala letterbox tetap 1,0 di kedua kasus, jadi bola TIDAK diperkecil
        # sama sekali -- ini murni membuang padding, bukan menukar akurasi
        # dengan kecepatan.
        #
        # Modelnya harus DIEKSPOR pada bentuk itu; IR OpenVINO punya bentuk
        # masukan STATIS dan menolak yang lain (reshape() juga tidak bisa,
        # grid anchor-nya sudah dibakar jadi konstanta saat ekspor):
        #   yolo export model=model/ball_yolov8n_best.pt format=openvino imgsz=384,640
        #
        # [0, 0] = tidak dipakai, kembali ke imgsz persegi di atas. Bawaannya
        # harus dua angka, bukan senarai kosong, supaya rclpy menyimpulkan
        # tipenya sebagai INTEGER_ARRAY.
        imgsz_hw = list(self.declare_parameter('imgsz_hw', [0, 0]).value or [])
        if len(imgsz_hw) == 2 and imgsz_hw[0] > 0 and imgsz_hw[1] > 0:
            self.predict_imgsz = (int(imgsz_hw[0]), int(imgsz_hw[1]))
        else:
            self.predict_imgsz = self.imgsz
        self.device = str(self.declare_parameter('device', 'cpu').value)
        self.half = bool(self.declare_parameter('half', False).value)
        # Ultralytics deprecated the predict-time 'half' argument: passing it at
        # all (even =False) prints a "'half' is deprecated, use 'quantize'"
        # warning on every inference call. Forward an FP16 request only when half
        # is explicitly enabled; otherwise omit the kwarg so the default path
        # stays quiet. (OpenVINO CPU precision is fixed at export time anyway.)
        self._predict_precision = {'half': True} if self.half else {}
        self.max_detections = int(
            self.declare_parameter('max_detections', 10).value)
        # Class names (from the model) that should be treated as "the ball".
        self.target_classes = list(
            self.declare_parameter('target_classes', ['ball']).value)
        # --- gambar semua kelas, teruskan hanya bola ------------------------
        # Bobot ball_gawang_v3 punya kelas kedua, "gawang", yang HANYA untuk
        # ditampilkan di gambar debug -- perilaku demo tidak boleh ikut berubah.
        #
        # Masalahnya, sebelumnya `classes=target_class_ids` diteruskan ke
        # predict(), jadi kelas lain dibuang di NMS sebelum sempat digambar;
        # mengganti bobot saja tidak akan menampilkannya. Dengan
        # draw_all_classes=true, predict() dijalankan TANPA saringan kelas dan
        # saringannya pindah ke _extract_circles: hanya kotak berkelas target
        # yang jadi circle_set/ball_center. Semua yang di hilir (_select_target,
        # BallTracker, head_tracking, D58) tetap hanya melihat bola.
        #
        # Bawaan false supaya profil .pt/.onnx lama berperilaku persis seperti
        # dulu. Ongkos true: NMS memproses beberapa kotak ekstra -- terukur
        # sub-milidetik.
        self.draw_all_classes = bool(
            self.declare_parameter('draw_all_classes', False).value)
        # --- kelas AKTIF (hanya berlaku saat draw_all_classes=true) ----------
        # Bobot robot_torso punya 6 kelas: ball, goalpost, l, robot, t, x.
        # l/t/x (titik potong garis lapangan) tidak diinginkan di gambar debug,
        # jadi daftar ini membatasi kelas yang dideteksi + digambar. Kelas di
        # target_classes SELALU ikut, supaya bola tidak mungkin tersaring.
        # Saringan dilakukan di predict(classes=...), jadi kelas lain dibuang
        # sebelum NMS dan tidak memakan jatah max_detections.
        #
        # KOSONG = semua kelas (perilaku lama). Penjagaan dynamic_typing +
        # `or []` sama alasannya dengan display_class_names di bawah.
        self.active_classes = list(
            self.declare_parameter(
                'active_classes', [],
                ParameterDescriptor(dynamic_typing=True)).value or [])
        # Diisi sesudah model dimuat; None = semua kelas.
        self.active_class_ids = None
        # Nama yang DITULIS di gambar debug, kalau berbeda dari nama kelas di
        # model. Bentuknya "nama_di_model=Nama Tampilan"; kosong = pakai nama
        # asli model.
        #
        # Ada karena nama kelas berbeda antar bobot bola: v8n menamainya
        # "Bola-baru", ball_yolov8n_* menamainya "ball". Operator mengenal
        # tulisan yang pertama, jadi kalau suatu saat bobotnya kembali ke
        # ball_yolov8n, tulisannya bisa disamakan tanpa mengganti bobot:
        #   display_class_names: ["ball=Bola baru"]
        #
        # HANYA mempengaruhi gambar debug. Pemilihan kelas tetap memakai nama
        # asli dari model (lihat target_classes), jadi mengganti tulisan di sini
        # tidak bisa membuat deteksinya meleset.
        #
        # DUA penjagaan di sini, dan keduanya wajib. Bawaan daftar KOSONG
        # membuat rclpy harus menebak tipe elemennya, dan tebakannya menyakitkan
        # di dua arah:
        #
        #  1. `dynamic_typing=True` -- tanpa ini rclpy Humble menyimpulkan
        #     BYTE_ARRAY dari `[]`, lalu MENOLAK daftar string dari berkas
        #     parameter dan node MATI saat start:
        #       InvalidParameterTypeException: Trying to set parameter
        #       'display_class_names' to '['ball=Bola-baru']' of type
        #       'STRING_ARRAY', expecting type 'BYTE_ARRAY'
        #     Terbukti 2026-09-18 saat bobot pindah ke merged_ball_v2; gejalanya
        #     di lapangan adalah demo yang "tidak jalan" tanpa sebab jelas,
        #     karena galatnya cuma muncul di log node detektor.
        #
        #  2. `or []` -- saat berkas parameter benar-benar menuliskan
        #     `display_class_names: []`, `.value` bisa mengembalikan None, bukan
        #     []. Tanpa ini: "TypeError: 'NoneType' object is not iterable".
        self.display_class_names = list(
            self.declare_parameter(
                'display_class_names', [],
                ParameterDescriptor(dynamic_typing=True)).value or [])
        # Diisi sesudah model dimuat; None = pakai nama asli model.
        self.plot_names = None
        self.detection_rate = float(
            self.declare_parameter('detection_rate', 30.0).value)
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        self.detection_frame_id = str(
            self.declare_parameter('detection_frame_id', 'detector').value)
        # --- kunci SATU bola -------------------------------------------------
        # Begitu satu bola terpilih, bola lain DIABAIKAN sampai yang terkunci
        # benar-benar hilang. Tanpa ini, tiap frame dipilih bola yang paling
        # BESAR, jadi dua bola dengan ukuran mirip membuat pilihan bertukar
        # bolak-balik dan kepala ikut melompat di antara keduanya.
        self.single_target = bool(
            self.declare_parameter('single_target', True).value)
        # Seberapa jauh bola boleh berpindah antar frame dan masih dianggap
        # bola YANG SAMA. Satuan koordinat ternormalisasi [-1, 1], jadi lebar
        # bingkai penuh = 2,0. Kalau kepala sedang menyapu cepat, bola bisa
        # bergeser cukup jauh dalam satu frame -- itu sebabnya nilainya tidak
        # kecil. Turunkan kalau dua bola berdekatan masih tertukar.
        self.target_lock_radius = float(
            self.declare_parameter('target_lock_radius', 0.40).value)
        # Berapa lama bola yang terkunci boleh tidak terlihat sebelum node ini
        # boleh mengunci bola lain. Sengaja sedikit LEBIH PENDEK dari
        # lock_hold_sec di head_tracking.yaml (2,0 s) supaya detektor sudah
        # mengunci ulang sebelum kepala menyerah dan mulai menyapu lagi.
        self.target_lost_sec = float(
            self.declare_parameter('target_lost_sec', 1.5).value)
        self._target_xy = None
        self._target_seen = 0.0
        self.enabled = bool(
            self.declare_parameter('enable_at_start', True).value)
        # Seconds between the live FPS lines. The same rate is published
        # continuously on the "fps" topic regardless of this.
        self.fps_log_period = float(
            self.declare_parameter('fps_log_period', 5.0).value)
        # Length of the sliding window the live FPS is measured over (s).
        self.fps_window_sec = float(
            self.declare_parameter('fps_window_sec', 2.0).value)
        # Frames used at start-up to measure the model's FPS ceiling on this
        # machine (logged once, before any camera frame arrives). 0 disables it.
        self.startup_benchmark_frames = int(
            self.declare_parameter('startup_benchmark_frames', 5).value)
        # Head-tracking feed. Also publish the best ball's center as a
        # geometry_msgs/PointStamped in [0, 1] image coordinates (0 = left/top,
        # 1 = right/bottom) on this topic, matching op3_yolo_vision so that
        # op3_ball_localization/head_tracking_node can drive the head scan/track.
        # Only published when a ball is seen (absence -> the tracker scans).
        # Empty string disables this extra publisher.
        self.ball_center_topic = str(
            self.declare_parameter(
                'ball_center_topic', '/vision/yolo/ball_center').value).strip()

        if self.detection_rate <= 0.0:
            self.get_logger().warn('detection_rate <= 0, defaulting to 30 Hz')
            self.detection_rate = 30.0

        # ----- FPS accounting ---------------------------------------------
        self.fps = FpsMeter(window_sec=self.fps_window_sec, name='ball',
                            hit_label='ball seen')
        self._summary_logged = False
        self._last_fps_log = time.monotonic()

        # ----- load the YOLO model ---------------------------------------
        self.model = self._load_model()

        # ----- ROS interface ---------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None        # (cv_bgr, header)
        self._processed_stamp = None     # last header processed (dedupe)

        self.circles_pub = self.create_publisher(
            CircleSetStamped, 'circle_set', 10)
        self.fps_pub = self.create_publisher(Float32, 'fps', 10)
        # Statistik kecerahan kamera, untuk memantau AE firmware di lapangan
        # (`ros2 topic echo /ball_detector_node/frame_stats`) dan untuk
        # scripts/sweep_camera_fps.py. Luma lapangan dan luma bola itulah yang
        # dilihat model; laju topik tidak mengatakan apa-apa soal itu.
        # Susunan data, semuanya 0..255 kecuali disebut lain:
        #   [0] luma rata-rata 2/3 BAWAH frame (lapangan; atap/lampu tak ikut)
        #   [1] fraksi piksel terbakar (>250) di seluruh frame, 0..1
        #   [2] 1.0 kalau bola terlihat di frame ini, 0.0 kalau tidak
        #   [3] luma rata-rata kotak bola (-1 kalau tidak ada bola)
        #   [4] luma rata-rata seluruh frame
        # Dihitung dari subsampel [::8, ::8] frame yang SUDAH di-decode,
        # jadi ongkosnya praktis nol. Dibatasi ~frame_stats_rate Hz.
        self.frame_stats_rate = float(
            self.declare_parameter('frame_stats_rate', 5.0).value)
        self.frame_stats_pub = self.create_publisher(
            Float32MultiArray, 'frame_stats', 10)
        self._last_stats_pub = 0.0
        # Absolute topic (leading '/') stays global regardless of the node's
        # namespace, so head_tracking_node finds it at /vision/yolo/ball_center.
        if self.ball_center_topic:
            self.ball_center_pub = self.create_publisher(
                PointStamped, self.ball_center_topic, 10)
        else:
            self.ball_center_pub = None
        if self.publish_image:
            self.image_pub = self.create_publisher(Image, 'image_out', 1)
        else:
            self.image_pub = None

        # Antrean gambar KEDALAMAN 1, bukan 5 (qos_profile_sensor_data).
        # Node ini cuma menyimpan gambar TERBARU (_latest_image ditimpa tiap
        # callback), jadi antrean yang dalam tidak menambah informasi -- ia
        # cuma menyimpan gambar BASI. Dan executor rclpy itu satu utas: selama
        # inferensi (~80 ms) tidak ada callback yang jalan, jadi antrean 5 itu
        # benar-benar terisi dan callback berikutnya mengolahnya satu per satu
        # dari yang PALING LAMA. Terukur menyumbang ~0,3 detik umur deteksi.
        # Kedalaman 1 + best effort = frame lama dibuang di lapisan DDS.
        image_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, 'image_in',
                self._compressed_image_callback, image_qos)
        else:
            self.image_sub = self.create_subscription(
                Image, 'image_in', self._image_callback, image_qos)

        self.enable_sub = self.create_subscription(
            Bool, 'enable', self._enable_callback, 1)

        # ----- image watchdog ------------------------------------------------
        # A detector that receives nothing must SAY so. Without this the node
        # sits perfectly silent when no camera is publishing, which on screen is
        # indistinguishable from "the model found nothing".
        self._frames_in = 0
        self._last_frame_time = None
        self._start_time = time.monotonic()
        self.watchdog = self.create_timer(1.0, self._check_image_flow)

        # Run inference on the freshest frame at a fixed rate. Decoupling the
        # camera rate from inference avoids queue build-up when YOLO is slow.
        self.timer = self.create_timer(
            1.0 / self.detection_rate, self._process_latest)

        self.get_logger().info(
            'Listening for images on "%s"'
            % self.image_sub.topic_name)
        self.get_logger().info(
            'YOLO ball detector ready '
            f'(compressed={self.use_compressed}, conf={self.confidence_threshold}, '
            f'device={self.device}, rate={self.detection_rate:.1f}Hz, '
            f'enabled={self.enabled})')

    # ---------------------------------------------------------------------
    # setup helpers
    # ---------------------------------------------------------------------
    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().fatal(
                'Failed to import ultralytics. Install it with '
                '"pip3 install ultralytics". Error: %s' % exc)
            raise

        share_dir = get_package_share_directory('op3_ball_detector')
        model_path = self.model_path.strip()
        if not model_path:
            # No model configured: use the package default.
            model_path = os.path.join(share_dir, 'model', 'ball_yolov8n_best.pt')
        elif not os.path.isabs(model_path) and not os.path.exists(model_path):
            # A relative path like "model/v8s.pt" (a .pt/.onnx file) or
            # "model/v8n_openvino_model" (an OpenVINO directory) is resolved
            # against the package share dir (where install(DIRECTORY ... model)
            # puts it), so editing model_path in the YAML "just works".
            model_path = os.path.join(share_dir, model_path)

        # An OpenVINO model is a directory (.xml/.bin); .pt/.onnx are files.
        if not os.path.exists(model_path):
            self.get_logger().fatal('Model path not found: %s' % model_path)
            raise FileNotFoundError(model_path)

        self.get_logger().info('Loading YOLO model: %s' % model_path)
        model = (YOLO(model_path, task=self.model_task)
                 if self.model_task else YOLO(model_path))

        # Warm-up DULU, baru baca nama kelas. Urutannya penting, bukan selera.
        #
        # Untuk format ekspor seperti OpenVINO, YOLO(path) tidak memuat apa pun --
        # cuma menyimpan path (engine/model.py:286-289). Pemuatan sebenarnya baru
        # terjadi di predictor.setup_model(). Masalahnya, membaca `model.names`
        # sebelum predictor ada membuat Ultralytics MEMBANGUN PREDICTOR SEKALI
        # PAKAI hanya untuk mengambil daftar kelas, lalu membuangnya
        # (engine/model.py:924-927). Jadi urutan lama -- names dulu, warm-up
        # kemudian -- memuat model OpenVINO DUA KALI saat start, dan itu terlihat
        # di log sebagai dua pasang "Loading ... for OpenVINO inference..." +
        # "Using OpenVINO LATENCY mode ...".
        #
        # Dengan warm-up dijalankan lebih dulu, predictor asli sudah ada, sehingga
        # `model.names` tinggal membacanya (cabang terakhir properti itu) tanpa
        # memuat ulang. Kalau warm-up gagal, perilakunya kembali ke jalur lama
        # yang tetap benar -- cuma memuat dua kali seperti sebelumnya.
        #
        # The warm-up doubles as the FPS-ceiling measurement: the VERY first
        # inference pays for lazy graph/IR compilation (~200 ms on OpenVINO
        # against ~20 ms steady state), so it is thrown away and only the
        # frames after it are timed. What comes out is an upper bound — the
        # loop can never beat it, and a detection_rate above it is wishful
        # thinking.
        try:
            import numpy as np
            dummy_h, dummy_w = (
                self.predict_imgsz if isinstance(self.predict_imgsz, tuple)
                else (self.predict_imgsz, self.predict_imgsz))
            dummy = np.zeros((dummy_h, dummy_w, 3), dtype=np.uint8)
            model.predict(dummy, imgsz=self.predict_imgsz, device=self.device,
                          verbose=False, **self._predict_precision)

            frames = max(0, self.startup_benchmark_frames)
            if frames:
                started = time.monotonic()
                for _ in range(frames):
                    model.predict(dummy, imgsz=self.predict_imgsz, device=self.device,
                                  verbose=False, **self._predict_precision)
                elapsed = time.monotonic() - started
                if elapsed > 0.0:
                    ceiling = frames / elapsed
                    note = ''
                    if ceiling < self.detection_rate:
                        note = (' — BELOW the requested detection_rate of '
                                '%.1f Hz, so the loop will run at about '
                                '%.1f FPS' % (self.detection_rate, ceiling))
                    self.get_logger().info(
                        'Model ceiling on this machine: %.1f FPS (%.1f ms/frame '
                        'over %d warm frames)%s'
                        % (ceiling, elapsed / frames * 1000.0, frames, note))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('Model warm-up skipped: %s' % exc)

        # Resolve target class names -> class indices for the model.
        self.class_names = dict(model.names)
        self.target_class_ids = self._resolve_target_class_ids()
        self.active_class_ids = self._resolve_active_class_ids()
        self.plot_names = self._resolve_display_names()
        self.get_logger().info(
            'Model classes: %s | targeting ids: %s'
            % (self.class_names, self.target_class_ids))
        if self.draw_all_classes:
            # Ditulis eksplisit supaya salah konfigurasi terlihat di log,
            # bukan baru ketahuan di lapangan saat gawang "jadi bola".
            passed = ([self.class_names.get(i, '?') for i in self.target_class_ids]
                      if self.target_class_ids is not None else 'SEMUA')
            drawn = ([self.class_names.get(i, '?') for i in self.active_class_ids]
                     if self.active_class_ids is not None else 'SEMUA')
            self.get_logger().info(
                'draw_all_classes: menggambar kelas %s di image_out, '
                'meneruskan ke circle_set/ball_center HANYA: %s'
                % (drawn, passed))
        if self.plot_names is not None:
            self.get_logger().info(
                'Nama di gambar debug diganti jadi: %s' % self.plot_names)

        return model

    # rclpy memanggil callback ini SEBELUM nilai barunya disimpan, jadi angkanya
    # diambil dari daftar yang masuk. Membaca get_parameter() di sini akan
    # mengembalikan nilai LAMA -- jebakan yang sama dengan on_set di rclcpp.
    def _on_set_parameters(self, parameters):
        AMBANG = {'confidence_threshold', 'iou_threshold'}
        for parameter in parameters:
            if parameter.name not in AMBANG:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name}: harus angka')
            if not 0.0 < value < 1.0:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name}: harus di antara 0 dan 1 (eksklusif)')
        for parameter in parameters:
            if parameter.name == 'confidence_threshold':
                self.confidence_threshold = float(parameter.value)
                self.get_logger().warn(
                    f'confidence_threshold diubah jadi {self.confidence_threshold:.2f}')
            elif parameter.name == 'iou_threshold':
                self.iou_threshold = float(parameter.value)
                self.get_logger().warn(
                    f'iou_threshold diubah jadi {self.iou_threshold:.2f}')
        return SetParametersResult(successful=True)

    def _resolve_display_names(self):
        """Peta {id_kelas: nama tampilan} untuk gambar debug.

        None berarti pakai nama asli model apa adanya.
        """
        if not self.display_class_names:
            return None

        name_to_id = {str(name).lower(): idx
                      for idx, name in self.class_names.items()}
        plot_names = dict(self.class_names)
        changed = False
        for entry in self.display_class_names:
            text = str(entry)
            if '=' not in text:
                self.get_logger().warn(
                    'display_class_names: "%s" tidak berbentuk '
                    '"nama_model=Nama Tampilan"; dilewati' % text)
                continue
            source, _, shown = text.partition('=')
            key = source.strip().lower()
            shown = shown.strip()
            if key not in name_to_id:
                self.get_logger().warn(
                    'display_class_names: kelas "%s" tidak ada di model; '
                    'dilewati' % source.strip())
                continue
            if not shown:
                self.get_logger().warn(
                    'display_class_names: nama tampilan untuk "%s" kosong; '
                    'dilewati' % source.strip())
                continue
            plot_names[name_to_id[key]] = shown
            changed = True

        return plot_names if changed else None

    def _resolve_active_class_ids(self):
        """Indeks kelas yang dideteksi saat draw_all_classes=true.

        None = semua kelas. Kelas target selalu disertakan.
        """
        if not self.active_classes:
            return None
        name_to_id = {str(name).lower(): idx
                      for idx, name in self.class_names.items()}
        ids = []
        for wanted in self.active_classes:
            key = str(wanted).lower()
            if key in name_to_id:
                ids.append(name_to_id[key])
            else:
                self.get_logger().warn(
                    'active_classes: kelas "%s" tidak ada di model; '
                    'dilewati' % wanted)
        for idx in (self.target_class_ids or []):
            if idx not in ids:
                ids.append(idx)
        if not ids:
            self.get_logger().warn(
                'active_classes: tidak ada yang cocok dengan model; '
                'memakai SEMUA kelas.')
            return None
        return sorted(ids)

    def _resolve_target_class_ids(self):
        """Map configured target class names to model class indices.

        Returns None to mean "detect every class" when no target name matches.
        """
        if not self.target_classes:
            return None
        name_to_id = {str(name).lower(): idx
                      for idx, name in self.class_names.items()}
        ids = []
        for wanted in self.target_classes:
            key = str(wanted).lower()
            if key in name_to_id:
                ids.append(name_to_id[key])
            else:
                self.get_logger().warn(
                    'target class "%s" not in model; ignoring' % wanted)
        if not ids:
            self.get_logger().warn(
                'No configured target class matched the model classes; '
                'falling back to detecting ALL classes as ball.')
            return None
        return ids

    # ---------------------------------------------------------------------
    # subscriptions
    # ---------------------------------------------------------------------
    def _enable_callback(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(
                'Detection %s' % ('ENABLED' if msg.data else 'DISABLED'))
        self.enabled = msg.data

    def _compressed_image_callback(self, msg: CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('compressed cv_bridge failed: %s' % exc)
            return
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)
        self._note_incoming_frame(cv_img)

    def _image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc)
            return
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)
        self._note_incoming_frame(cv_img)

    def _note_incoming_frame(self, cv_img):
        """Book-keeping for the watchdog, called from the image callbacks."""
        self._frames_in += 1
        self._last_frame_time = time.monotonic()
        if self._frames_in == 1:
            height, width = cv_img.shape[:2]
            self.get_logger().info(
                'First image received on "%s": %dx%d (mean level %.1f/255)'
                % (self.image_sub.topic_name, width, height,
                   self._frame_level(cv_img)))
            return

        # A frame that is essentially black — lens covered, lights off, head
        # pointed into a shadow — yields zero detections and on the console
        # looks EXACTLY like a model that cannot see anything. Say which of the
        # two it is. Sampled every 8th pixel, so this costs nothing.
        level = self._frame_level(cv_img)
        if level < 16.0:
            self.get_logger().warn(
                'Image on "%s" is almost BLACK (mean %.1f/255): there is '
                'nothing in it for the model to find. Check the lens cap, the '
                'room lights, and where the head is pointing.'
                % (self.image_sub.topic_name, level),
                throttle_duration_sec=10.0)
        elif level > 245.0:
            self.get_logger().warn(
                'Image on "%s" is BLOWN OUT (mean %.1f/255): overexposed, so '
                'the model has no contrast to work with.'
                % (self.image_sub.topic_name, level),
                throttle_duration_sec=10.0)

    @staticmethod
    def _frame_level(cv_img):
        """Mean brightness over a cheap subsample of the frame (0..255)."""
        return float(cv_img[::8, ::8].mean())

    def _maybe_publish_frame_stats(self, cv_img, circles):
        """Terbitkan frame_stats (lihat susunannya di __init__), dibatasi laju."""
        if self.frame_stats_rate <= 0.0:
            return
        now = time.monotonic()
        if (now - self._last_stats_pub) < (1.0 / self.frame_stats_rate):
            return
        self._last_stats_pub = now

        sub = cv_img[::8, ::8].astype('float32')
        # BGR -> luma (BT.601), sama dengan yang dilihat sensor sebagai "terang".
        luma = sub[..., 0] * 0.114 + sub[..., 1] * 0.587 + sub[..., 2] * 0.299
        rows = luma.shape[0]
        field = luma[rows // 3:, :]
        full_mean = float(luma.mean())
        field_mean = float(field.mean()) if field.size else full_mean
        clipped = float((luma > 250.0).mean())

        ball_seen = 1.0 if circles else 0.0
        ball_luma = -1.0
        if circles:
            height, width = cv_img.shape[:2]
            nx, ny, radius = circles[0][0], circles[0][1], circles[0][2]
            cx = (nx + 1.0) * 0.5 * width
            cy = (ny + 1.0) * 0.5 * height
            x1 = max(0, int(cx - radius))
            x2 = min(width, int(cx + radius) + 1)
            y1 = max(0, int(cy - radius))
            y2 = min(height, int(cy + radius) + 1)
            if x2 > x1 and y2 > y1:
                step = max(1, int(radius) // 8)
                box = cv_img[y1:y2:step, x1:x2:step].astype('float32')
                ball_luma = float((box[..., 0] * 0.114 + box[..., 1] * 0.587
                                   + box[..., 2] * 0.299).mean())

        self.frame_stats_pub.publish(Float32MultiArray(
            data=[field_mean, clipped, ball_seen, ball_luma, full_mean]))

    def _check_image_flow(self):
        """Say out loud when nothing is arriving on image_in."""
        now = time.monotonic()
        topic = self.image_sub.topic_name
        if self._frames_in == 0:
            # Give the camera a moment to come up before complaining.
            if (now - self._start_time) < 3.0:
                return
            self.get_logger().warn(
                'NO IMAGE on "%s" after %.0f s — nothing is publishing there, '
                'so this detector has nothing to look at. Either start the '
                'camera together with it (use_usb_cam:=true), or point '
                'image_topic:= at a topic that has a publisher. Check with: '
                'ros2 topic hz %s'
                % (topic, now - self._start_time, topic),
                throttle_duration_sec=5.0)
            return
        gap = now - self._last_frame_time
        if gap > 3.0:
            self.get_logger().warn(
                'Image flow STOPPED on "%s": nothing for %.1f s after %d '
                'frames. Did the camera node die?'
                % (topic, gap, self._frames_in),
                throttle_duration_sec=5.0)

    # ---------------------------------------------------------------------
    # main detection loop
    # ---------------------------------------------------------------------
    def _process_latest(self):
        # Ctrl-C tears the context down while this callback may be halfway
        # through an inference; every publish() after that raises RCLError
        # ("publisher's context is invalid"). That is a clean shutdown, not a
        # failure, so swallow it instead of dumping a traceback over the FPS
        # summary. rclpy Humble exposes no importable RCLError class, hence the
        # rclpy.ok() test rather than a narrower "except".
        if not rclpy.ok():
            return
        try:
            self._process_frame()
        except Exception:  # noqa: BLE001
            if not rclpy.ok():
                return
            raise

    def _process_frame(self):
        with self._frame_lock:
            data = self._latest_image
            self._latest_image = None
        if data is None:
            return
        cv_img, header = data
        loop_start = time.monotonic()

        if not self.enabled:
            # Mirror the C++ detector: do not publish circles while disabled,
            # but still stream the (raw) image for debugging.
            self._publish_image(cv_img, header)
            # Kamera tetap perlu diatur pajannya walau deteksi dimatikan.
            self._maybe_publish_frame_stats(cv_img, [])
            return

        try:
            results = self.model.predict(
                cv_img,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.predict_imgsz,
                device=self.device,
                max_det=self.max_detections,
                classes=(self.active_class_ids if self.draw_all_classes
                         else self.target_class_ids),
                verbose=False,
                **self._predict_precision)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('YOLO inference failed: %s' % exc,
                                    throttle_duration_sec=2.0)
            return

        height, width = cv_img.shape[:2]
        circles = self._extract_circles(results, width, height)
        circles = self._select_target(circles)
        self._publish_circles(circles, header)
        self._publish_annotated(results, cv_img, header)
        self._maybe_publish_frame_stats(cv_img, circles)

        # Periodic FRAME-RATE readout. Ultralytics only reports per-stage
        # milliseconds (results[].speed); those say how expensive one frame is
        # but not how many frames per second the loop actually delivers — the
        # two part company as soon as the camera, detection_rate or a dropped
        # frame gets involved. So measure real arrivals and lead with FPS,
        # keeping the milliseconds behind it as the explanation.
        speed = getattr(results[0], 'speed', None) if results else {}
        speed = speed or {}
        loop_ms = (time.monotonic() - loop_start) * 1000.0
        fps_now = self.fps.tick(now=loop_start,
                                infer_ms=speed.get('inference'),
                                pre_ms=speed.get('preprocess'),
                                post_ms=speed.get('postprocess'),
                                loop_ms=loop_ms, hit=bool(circles))
        self.fps_pub.publish(Float32(data=float(fps_now)))

        now = time.monotonic()
        if (now - self._last_fps_log) >= max(0.5, self.fps_log_period):
            self._last_fps_log = now
            self.get_logger().info(
                '%s | %s' % (self.fps.live_line(),
                             'ball x%d' % len(circles) if circles
                             else 'no ball'))

    def _extract_circles(self, results, width, height):
        """Convert YOLO boxes to (norm_x, norm_y, radius_px, conf) tuples."""
        circles = []
        if not results:
            return circles
        result = results[0]
        boxes = getattr(result, 'boxes', None)
        if boxes is None or boxes.xyxy is None:
            return circles

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy() if boxes.conf is not None else None
        # Saringan kelas di sini adalah SATU-SATUNYA yang menjaga agar kelas
        # non-bola (mis. gawang) tidak pernah bocor ke circle_set/ball_center
        # saat draw_all_classes=true. Jangan dihapus walau predict() sudah
        # menyaring -- biarkan keduanya, biayanya nol.
        cls_ids = None
        if self.target_class_ids is not None and boxes.cls is not None:
            cls_ids = boxes.cls.cpu().numpy().astype(int)
        for idx, (x1, y1, x2, y2) in enumerate(xyxy):
            if cls_ids is not None and int(cls_ids[idx]) not in self.target_class_ids:
                continue
            cx = (x1 + x2) * 0.5
            cy = (y1 + y2) * 0.5
            radius = ((x2 - x1) + (y2 - y1)) * 0.25  # mean half-side, in px
            norm_x = (cx / width) * 2.0 - 1.0
            norm_y = (cy / height) * 2.0 - 1.0
            conf = float(confs[idx]) if confs is not None else 1.0
            circles.append((norm_x, norm_y, float(radius), conf))

        # Largest ball first; downstream picks the circle with the biggest z.
        circles.sort(key=lambda c: c[2], reverse=True)
        return circles

    def _select_target(self, circles):
        """Kunci satu bola dan abaikan sisanya.

        Mengembalikan daftar berisi 0 atau 1 bola. Yang dikembalikan inilah
        yang diterbitkan ke circle_set MAUPUN ball_center, jadi pelacak kepala
        (python) dan BallTracker (C++, memilih lingkaran ber-z terbesar)
        sama-sama melihat bola yang sama -- tidak bisa berbeda pendapat.

        Kalau bola yang terkunci tidak terlihat di suatu frame, frame itu
        dilaporkan KOSONG, bukan diganti bola lain. Kepala lalu menahan posisi
        lewat lock_hold_sec, bukan melompat ke bola lain.
        """
        if not self.single_target:
            return circles

        now = time.monotonic()

        # target lama sudah terlalu lama hilang -> boleh mengunci yang baru
        if (self._target_xy is not None
                and (now - self._target_seen) > self.target_lost_sec):
            self.get_logger().info(
                'kunci bola dilepas (tidak terlihat %.1f s)'
                % (now - self._target_seen))
            self._target_xy = None

        if not circles:
            return []

        if self._target_xy is None:
            # circles sudah terurut dari yang paling besar = paling dekat
            best = circles[0]
            self._target_xy = (best[0], best[1])
            self._target_seen = now
            self.get_logger().info(
                'kunci bola BARU di (%.2f, %.2f), %d kandidat'
                % (best[0], best[1], len(circles)))
            return [best]

        tx, ty = self._target_xy
        nearest = min(circles, key=lambda c: math.hypot(c[0] - tx, c[1] - ty))
        if math.hypot(nearest[0] - tx, nearest[1] - ty) <= self.target_lock_radius:
            self._target_xy = (nearest[0], nearest[1])
            self._target_seen = now
            return [nearest]

        # ada bola, tapi tidak ada yang cukup dekat dengan yang terkunci:
        # itu bola LAIN. Laporkan kosong dan biarkan target kedaluwarsa sendiri.
        return []

    def _publish_circles(self, circles, header):
        msg = CircleSetStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.detection_frame_id
        msg.circles = [Point(x=c[0], y=c[1], z=c[2]) for c in circles]
        self.circles_pub.publish(msg)

        if circles:
            best = circles[0]
            if self.ball_center_pub is not None:
                # circles carry x,y in [-1, 1]; head_tracking_node wants [0, 1].
                center = PointStamped()
                center.header.stamp = header.stamp
                center.header.frame_id = self.detection_frame_id
                center.point.x = (best[0] + 1.0) / 2.0
                center.point.y = (best[1] + 1.0) / 2.0
                center.point.z = best[3]   # confidence
                self.ball_center_pub.publish(center)
            self.get_logger().info(
                'ball: center=(%.2f, %.2f) radius=%.1fpx conf=%.2f (%d total)'
                % (best[0], best[1], best[2], best[3], len(circles)),
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info('no ball detected',
                                   throttle_duration_sec=3.0)

    def _publish_annotated(self, results, cv_img, header):
        # Skip the costly plot()/encode/publish entirely when nobody subscribes
        # to image_out, so the debug overlay is free during real operation yet
        # still available on demand (open it in rqt/foxglove to resume).
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            if results:
                # Dipasang DI SINI, sesudah lingkaran diambil dan diterbitkan,
                # supaya penggantian nama tidak bisa mempengaruhi deteksinya --
                # yang berubah cuma tulisan yang digambar plot().
                if self.plot_names is not None:
                    results[0].names = self.plot_names
                annotated = results[0].plot()
            else:
                annotated = cv_img
        except Exception:  # noqa: BLE001
            annotated = cv_img
        self._publish_image(annotated, header)

    def _publish_image(self, cv_img, header):
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            out = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
            out.header.stamp = header.stamp
            out.header.frame_id = self.detection_frame_id
            self.image_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('failed to publish image_out: %s' % exc,
                                   throttle_duration_sec=2.0)

    # ---------------------------------------------------------------------
    # shutdown report
    # ---------------------------------------------------------------------
    def log_fps_summary(self):
        """Log the end-of-run FPS block. Safe to call more than once."""
        if self._summary_logged:
            return
        self._summary_logged = True

        # After Ctrl-C rclpy's context is already down, and every logger call
        # then also prints "Failed to publish log message to rosout" — which
        # would bury the summary in noise. Fall back to plain stdout there.
        emit = self.get_logger().info if rclpy.ok() else print

        if self.fps.frame_count == 0:
            emit('FPS SUMMARY: no frame was processed — was anything '
                 'publishing on image_in?')
            return
        for line in self.fps.summary_lines(title='ball detector',
                                           target_fps=self.detection_rate):
            emit(line)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBallDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C under `ros2 launch` delivers SIGINT more than once: the
        # terminal signals the whole foreground process group, then launch
        # forwards one of its own. Those extras used to land in the middle of
        # the teardown and end the run with a traceback instead of the summary.
        # Deafen SIGINT for the entire teardown; SIGTERM/SIGKILL still work.
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except ValueError:
            pass

        # Report the achieved FPS on EVERY exit, Ctrl-C included.
        node.log_fps_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
