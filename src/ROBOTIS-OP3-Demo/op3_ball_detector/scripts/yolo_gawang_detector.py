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

"""Detektor GAWANG untuk ROBOTIS OP3, model OpenVINO INT8 ball_gawang_v3.

Node MANDIRI: tidak dipakai demo mana pun dan tidak menyentuh jalur bola.
Bobotnya sama dengan detektor bola (YOLOv8n, kelas ``{0: ball, 1: gawang}``),
tapi node ini HANYA meneruskan kelas ``gawang``. Satu kotak = satu gawang utuh
(begitu dataset-nya dilabeli); dua gawang terlihat = dua kotak.

Topik keluaran (relatif ke namespace, bawaan ``gawang_detector_node``):

    detections     vision_msgs/Detection2DArray  SEMUA gawang, PIKSEL gambar asli:
                       .results[0].hypothesis.class_id = "gawang"
                       .results[0].hypothesis.score    = conf
                       .bbox.center.position.x / .y    = pusat (px)
                       .bbox.size_x / .size_y          = ukuran (px)
                   Terbit tiap frame yang diproses, KOSONG kalau tidak ada
                   gawang -- jadi "tidak terlihat" bisa dibedakan dari
                   "detektor mati" (yang terakhir: topik ini berhenti).
    gawang_center  geometry_msgs/PointStamped    gawang ber-conf TERTINGGI,
                   x,y dalam [0, 1] (0 = kiri/atas), z = conf. Konvensinya
                   sama dengan /vision/yolo/ball_center. Hanya terbit saat
                   gawang terlihat.
    fps            std_msgs/Float32              laju proses nyata
    image_out      sensor_msgs/Image             gambar beranotasi, HANYA
                   dihitung kalau ada pelanggan

Masukan: ``image_in`` (CompressedImage atau Image, lihat use_compressed) dan
``enable`` (std_msgs/Bool).

Optimasi yang dipakai sama dengan detektor bola (PANDUAN_OPTIMASI_FPS_DETEKSI.md):
IR OpenVINO INT8 dengan masukan PERSEGI PANJANG 288x480, antrean gambar
kedalaman 1 best-effort, timer detection_rate jauh di atas laju kamera, dan
plot()/encode dilewati kalau tidak ada yang menonton image_out. Pematokan inti
CPU dan pelepasan rem laju kamera ada di launch-nya.
"""

import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from cv_bridge import CvBridge

# detector_fps.py dan goalline_core.py dipasang di direktori yang SAMA dengan
# skrip ini (lib/op3_ball_detector), dan bersebelahan di scripts/ dari sumber.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detector_fps import FpsMeter  # noqa: E402
from goalline_core import draw_hud, extract_detections, speed_of  # noqa: E402


class YoloGawangDetector(Node):
    """Mendeteksi gawang dengan model YOLO OpenVINO."""

    def __init__(self):
        super().__init__('yolo_gawang_detector')

        # ----- model / inferensi ----------------------------------------------
        self.model_path = str(self.declare_parameter(
            'model_path',
            'model/ball_gawang_v3_288x480_int8_calgawang_openvino_model').value)
        self.model_task = str(
            self.declare_parameter('model_task', 'detect').value).strip()
        self.use_compressed = bool(
            self.declare_parameter('use_compressed', True).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.35).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        # Kedua ambang di atas boleh diubah selagi node hidup (ros2 param set);
        # yang lain dipakai saat memuat model / membuat timer, jadi ditolak.
        self.add_on_set_parameters_callback(self._on_set_parameters)
        # [tinggi, lebar] WAJIB sama dengan bentuk ekspor IR -- masukan IR
        # OpenVINO statis, reshape() juga ditolak (grid anchor sudah dibakar).
        # Bawaannya dua angka, bukan [], supaya rclpy menyimpulkan INTEGER_ARRAY.
        imgsz_hw = list(self.declare_parameter('imgsz_hw', [288, 480]).value or [])
        if len(imgsz_hw) != 2 or imgsz_hw[0] <= 0 or imgsz_hw[1] <= 0:
            raise ValueError('imgsz_hw harus [tinggi, lebar], dapat %s' % imgsz_hw)
        self.predict_imgsz = (int(imgsz_hw[0]), int(imgsz_hw[1]))
        self.device = str(self.declare_parameter('device', 'cpu').value)
        self.max_detections = int(
            self.declare_parameter('max_detections', 5).value)
        # Nama kelas DI DALAM MODEL. Kalau tidak ada, node MENOLAK start --
        # sengaja tidak jatuh ke "semua kelas" seperti detektor bola, karena
        # itu berarti bola diterbitkan sebagai gawang tanpa ada yang sadar.
        self.gawang_class = str(
            self.declare_parameter('gawang_class', 'gawang').value).strip()
        # true = kelas lain (bola) ikut DIGAMBAR di image_out, tetap tidak
        # pernah diterbitkan. Ongkosnya cuma beberapa kotak ekstra di NMS.
        self.draw_other_classes = bool(
            self.declare_parameter('draw_other_classes', False).value)
        self.detection_rate = float(
            self.declare_parameter('detection_rate', 100.0).value)

        # ----- keluaran / laporan ---------------------------------------------
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        self.detection_frame_id = str(
            self.declare_parameter('detection_frame_id', 'detector').value)
        self.enabled = bool(
            self.declare_parameter('enable_at_start', True).value)
        self.fps_log_period = float(
            self.declare_parameter('fps_log_period', 5.0).value)
        self.fps_window_sec = float(
            self.declare_parameter('fps_window_sec', 2.0).value)
        self.startup_benchmark_frames = int(
            self.declare_parameter('startup_benchmark_frames', 5).value)

        if self.detection_rate <= 0.0:
            self.get_logger().warn('detection_rate <= 0, dipakai 100 Hz')
            self.detection_rate = 100.0

        # ----- status ---------------------------------------------------------
        self.fps = FpsMeter(window_sec=self.fps_window_sec, name='gawang',
                            hit_label='gawang seen')
        self._summary_logged = False
        self._last_fps_log = time.monotonic()
        # Umur frame (sekarang - header.stamp) saat hasilnya terbit, dirangkum
        # per baris log. Laju topik saja menipu: 20 FPS bisa berisi frame
        # berumur 1 detik kalau ada antrean di depan.
        self._age_sum = 0.0
        self._age_max = 0.0
        self._age_n = 0

        self.model = self._load_model()

        # ----- antarmuka ROS ----------------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None        # (cv_bgr, header)

        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        self.center_pub = self.create_publisher(
            PointStamped, 'gawang_center', 10)
        self.fps_pub = self.create_publisher(Float32, 'fps', 10)
        self.image_pub = (self.create_publisher(Image, 'image_out', 1)
                          if self.publish_image else None)

        # Kedalaman 1 + best effort: node cuma memakai frame TERBARU, jadi
        # antrean yang lebih dalam hanya menyimpan frame BASI yang lalu diproses
        # satu per satu dari yang paling lama (terukur ~0,3 s umur tambahan
        # di detektor bola dengan qos_profile_sensor_data kedalaman 5).
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

        self._frames_in = 0
        self._last_frame_time = None
        self._start_time = time.monotonic()
        self.watchdog = self.create_timer(1.0, self._check_image_flow)

        # Timer JAUH di atas laju kamera, bukan target FPS. Executor rclpy satu
        # utas: kalau lajunya dekat laju kamera, timer dan kamera beradu-layang
        # dan sepertiga frame hangus (terukur 25 Hz -> 13,4 FPS, 60 Hz -> 19,4
        # FPS pada kamera 20 FPS). Tik tanpa frame baru cuma satu lock + tes None.
        self.timer = self.create_timer(
            1.0 / self.detection_rate, self._process_latest)

        self.get_logger().info(
            'Listening for images on "%s"' % self.image_sub.topic_name)
        self.get_logger().info(
            'YOLO gawang detector ready (compressed=%s, conf=%.2f, imgsz=%dx%d, '
            'device=%s, rate=%.0fHz, enabled=%s)'
            % (self.use_compressed, self.confidence_threshold,
               self.predict_imgsz[0], self.predict_imgsz[1], self.device,
               self.detection_rate, self.enabled))

    # ---------------------------------------------------------------------
    # setup
    # ---------------------------------------------------------------------
    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().fatal(
                'Failed to import ultralytics. Install it with '
                '"pip3 install ultralytics". Error: %s' % exc)
            raise

        model_path = self.model_path.strip()
        if not model_path:
            raise ValueError('model_path kosong')
        if not os.path.isabs(model_path) and not os.path.exists(model_path):
            # Relatif terhadap share dir paket, tempat install(DIRECTORY model).
            model_path = os.path.join(
                get_package_share_directory('op3_ball_detector'), model_path)
        if not os.path.exists(model_path):
            self.get_logger().fatal(
                'Model path not found: %s -- folder model/ di-.gitignore, jadi '
                'IR-nya harus dibuat di mesin ini (lihat README_GAWANG.md).'
                % model_path)
            raise FileNotFoundError(model_path)
        if os.path.isdir(model_path):
            have = os.listdir(model_path)
            for suffix in ('.xml', '.bin'):
                if not any(f.endswith(suffix) for f in have):
                    self.get_logger().fatal(
                        'OpenVINO dir %s tidak punya berkas %s (isinya: %s). '
                        'IR butuh .xml DAN .bin.'
                        % (model_path, suffix, sorted(have)))
                    raise FileNotFoundError(os.path.join(model_path, '*' + suffix))

        self.get_logger().info('Loading YOLO model: %s' % model_path)
        model = (YOLO(model_path, task=self.model_task)
                 if self.model_task else YOLO(model_path))
        # Pemanasan DULU, baru baca model.names: untuk IR OpenVINO, membaca
        # names lebih dulu membuat Ultralytics membangun predictor sekali pakai
        # dan memuat modelnya DUA KALI saat start.
        self._benchmark_model(model)

        self.class_names = dict(model.names)
        lookup = {str(name).lower(): idx for idx, name in self.class_names.items()}
        if self.gawang_class.lower() not in lookup:
            self.get_logger().fatal(
                'Kelas "%s" tidak ada di model (kelasnya: %s). Bobot salah, '
                'atau gawang_class tidak sama persis dengan metadata.yaml.'
                % (self.gawang_class, self.class_names))
            raise ValueError('gawang_class tidak ada di model')
        self.gawang_id = lookup[self.gawang_class.lower()]
        self.get_logger().info(
            'Model classes: %s | diterbitkan: HANYA "%s" (id %d)%s'
            % (self.class_names, self.class_names[self.gawang_id],
               self.gawang_id,
               ' | kelas lain ikut digambar' if self.draw_other_classes else ''))
        return model

    def _benchmark_model(self, model):
        """Panaskan model sekaligus laporkan plafon FPS-nya di mesin ini.

        Inferensi pertama membayar kompilasi IR (~200 ms vs ~12 ms stabil),
        jadi dibuang; frame sesudahnya yang diukur.
        """
        import numpy as np
        height, width = self.predict_imgsz
        dummy = np.zeros((height, width, 3), dtype=np.uint8)
        try:
            model.predict(dummy, imgsz=self.predict_imgsz, device=self.device,
                          verbose=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('Model warm-up skipped: %s' % exc)
            return
        frames = max(0, self.startup_benchmark_frames)
        if not frames:
            return
        started = time.monotonic()
        for _ in range(frames):
            model.predict(dummy, imgsz=self.predict_imgsz, device=self.device,
                          verbose=False)
        elapsed = time.monotonic() - started
        if elapsed <= 0.0:
            return
        self.get_logger().info(
            'Model ceiling on this machine: %.1f FPS (%.1f ms/frame over %d '
            'warm frames)' % (frames / elapsed, elapsed / frames * 1000.0, frames))

    # rclpy memanggil ini SEBELUM nilai baru disimpan; ambil dari daftar masuk.
    def _on_set_parameters(self, parameters):
        thresholds = {'confidence_threshold', 'iou_threshold'}
        for parameter in parameters:
            if parameter.name not in thresholds:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False, reason='%s: harus angka' % parameter.name)
            if not 0.0 < value < 1.0:
                return SetParametersResult(
                    successful=False,
                    reason='%s: harus di antara 0 dan 1 (eksklusif)' % parameter.name)
        for parameter in parameters:
            if parameter.name == 'confidence_threshold':
                self.confidence_threshold = float(parameter.value)
                self.get_logger().warn(
                    'confidence_threshold diubah jadi %.2f' % self.confidence_threshold)
            elif parameter.name == 'iou_threshold':
                self.iou_threshold = float(parameter.value)
                self.get_logger().warn(
                    'iou_threshold diubah jadi %.2f' % self.iou_threshold)
        return SetParametersResult(successful=True)

    # ---------------------------------------------------------------------
    # langganan
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
        self._frames_in += 1
        self._last_frame_time = time.monotonic()
        level = float(cv_img[::8, ::8].mean())
        if self._frames_in == 1:
            height, width = cv_img.shape[:2]
            self.get_logger().info(
                'First image received on "%s": %dx%d (mean level %.1f/255)'
                % (self.image_sub.topic_name, width, height, level))
            return
        # Frame hitam / terlalu terang = nol deteksi yang di konsol tampak
        # persis seperti model yang buta. Katakan yang mana.
        if level < 16.0:
            self.get_logger().warn(
                'Image on "%s" is almost BLACK (mean %.1f/255): lens cap, '
                'room lights, or the head pointing into a shadow.'
                % (self.image_sub.topic_name, level),
                throttle_duration_sec=10.0)
        elif level > 245.0:
            self.get_logger().warn(
                'Image on "%s" is BLOWN OUT (mean %.1f/255): overexposed.'
                % (self.image_sub.topic_name, level),
                throttle_duration_sec=10.0)

    def _check_image_flow(self):
        now = time.monotonic()
        topic = self.image_sub.topic_name
        if self._frames_in == 0:
            if (now - self._start_time) < 3.0:
                return
            # Penyebab tersering di robot ini bukan kamera mati, tapi node ini
            # beda ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY dengan kamera -- keduanya
            # lalu tidak saling menemukan. Sebutkan milik sendiri supaya bisa
            # langsung dibandingkan.
            self.get_logger().warn(
                'NO IMAGE on "%s" after %.0f s. Node ini ROS_DOMAIN_ID=%s '
                'ROS_LOCALHOST_ONLY=%s -- KEDUANYA harus sama dengan kamera. '
                'Kalau sama: nyalakan kamera (use_usb_cam:=true) atau arahkan '
                'image_topic:=. Cek: ros2 topic hz %s'
                % (topic, now - self._start_time,
                   os.environ.get('ROS_DOMAIN_ID', '0 (bawaan)'),
                   os.environ.get('ROS_LOCALHOST_ONLY', '0 (bawaan)'), topic),
                throttle_duration_sec=5.0)
            return
        gap = now - self._last_frame_time
        if gap > 3.0:
            self.get_logger().warn(
                'Image flow STOPPED on "%s": nothing for %.1f s after %d '
                'frames. Did the camera node die?' % (topic, gap, self._frames_in),
                throttle_duration_sec=5.0)

    # ---------------------------------------------------------------------
    # loop deteksi
    # ---------------------------------------------------------------------
    def _process_latest(self):
        # Ctrl-C bisa meruntuhkan context di tengah inferensi; publish() sesudah
        # itu melempar galat. Itu shutdown yang bersih, bukan kegagalan.
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
            self._publish_image(cv_img, header)
            return

        try:
            results = self.model.predict(
                cv_img,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.predict_imgsz,
                device=self.device,
                max_det=self.max_detections,
                classes=None if self.draw_other_classes else [self.gawang_id],
                verbose=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('YOLO inference failed: %s' % exc,
                                    throttle_duration_sec=2.0)
            return

        # Saringan kelas di SINI yang menjaga bola tidak pernah terbit sebagai
        # gawang saat draw_other_classes=true. Biarkan walau predict() juga
        # sudah menyaring -- ongkosnya nol.
        gawang = [d for d in extract_detections(results, self.class_names)
                  if d[4] == self.class_names[self.gawang_id]]
        gawang.sort(key=lambda d: d[5], reverse=True)
        height, width = cv_img.shape[:2]
        self._publish_detections(gawang, header, width, height)

        pre_ms, infer_ms, post_ms = speed_of(results)
        loop_ms = (time.monotonic() - loop_start) * 1000.0
        fps_now = self.fps.tick(now=loop_start, infer_ms=infer_ms,
                                pre_ms=pre_ms, post_ms=post_ms,
                                loop_ms=loop_ms, hit=bool(gawang))
        self.fps_pub.publish(Float32(data=float(fps_now)))
        self._log_periodically(gawang)

        self._publish_annotated(results, cv_img, header, fps_now, gawang)

    def _note_age(self, header):
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - stamp
        if 0.0 <= age < 60.0:
            self._age_sum += age
            self._age_max = max(self._age_max, age)
            self._age_n += 1

    def _log_periodically(self, gawang):
        now = time.monotonic()
        if (now - self._last_fps_log) < max(0.5, self.fps_log_period):
            return
        self._last_fps_log = now
        if gawang:
            cx, cy, w, h, _name, conf = gawang[0]
            seen = 'gawang x%d, terbaik conf %.2f pusat (%.0f, %.0f) %.0fx%.0f px' % (
                len(gawang), conf, cx, cy, w, h)
        else:
            seen = 'tidak ada gawang'
        age = ''
        if self._age_n:
            age = ' | umur frame %.0f ms (maks %.0f)' % (
                self._age_sum / self._age_n * 1000.0, self._age_max * 1000.0)
            self._age_sum = self._age_max = 0.0
            self._age_n = 0
        self.get_logger().info('%s | %s%s' % (self.fps.live_line(), seen, age))

    # ---------------------------------------------------------------------
    # penerbitan
    # ---------------------------------------------------------------------
    def _publish_detections(self, gawang, header, width, height):
        msg = Detection2DArray()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.detection_frame_id
        for cx, cy, w, h, name, conf in gawang:
            det = Detection2D()
            det.header = msg.header
            det.id = name
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            msg.detections.append(det)
        self.detections_pub.publish(msg)

        if gawang:
            cx, cy, _w, _h, _name, conf = gawang[0]
            center = PointStamped()
            center.header = msg.header
            center.point.x = cx / float(width)
            center.point.y = cy / float(height)
            center.point.z = conf
            self.center_pub.publish(center)
        self._note_age(header)

    def _publish_annotated(self, results, cv_img, header, fps_now, gawang):
        # plot()/encode dilewati kalau tidak ada yang menonton image_out.
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            annotated = results[0].plot() if results else cv_img.copy()
        except Exception:  # noqa: BLE001
            annotated = cv_img.copy()
        draw_hud(annotated, [
            'FPS %.1f (avg %.1f)' % (fps_now, self.fps.average_fps),
            ('gawang x%d  conf %.2f' % (len(gawang), gawang[0][5]))
            if gawang else 'tidak ada gawang'])
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
    # laporan saat berhenti
    # ---------------------------------------------------------------------
    def log_fps_summary(self):
        """Blok ringkasan FPS di akhir run. Aman dipanggil berkali-kali."""
        if self._summary_logged:
            return
        self._summary_logged = True
        # Sesudah Ctrl-C context rclpy sudah mati dan logger ikut mengeluh
        # "Failed to publish log message to rosout"; pakai stdout saja.
        emit = self.get_logger().info if rclpy.ok() else print
        if self.fps.frame_count == 0:
            emit('FPS SUMMARY: no frame was processed -- was anything '
                 'publishing on image_in?')
            return
        for line in self.fps.summary_lines(title='gawang detector',
                                           target_fps=self.detection_rate):
            emit(line)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloGawangDetector()
    except KeyboardInterrupt:
        # SIGINT saat model masih dimuat (mis. rqt:=true tapi rqt gagal
        # membuka layar): berhenti tenang, bukan traceback dari dalam import.
        print('[gawang] dihentikan saat model masih dimuat')
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C di bawah `ros2 launch` mengirim SIGINT lebih dari sekali; yang
        # ekstra jangan sampai memotong ringkasan dengan traceback.
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except ValueError:
            pass
        node.log_fps_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
