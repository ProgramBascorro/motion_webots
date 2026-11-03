#!/usr/bin/env python3
"""
Head-only ball tracking node for the OpenVINO YOLO stack.

Uses /vision/yolo/ball_center directly and follows the original OP3 head
tracking convention: the node publishes head joint offsets that drive the ball
toward the center of the camera image.
"""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robotis_controller_msgs.msg import JointCtrlModule

from op3_ball_localization.head_scan_pattern import (
    build_scan_positions,
    resolve_scan_tilt_upper,
)

# Servo 19 (head_pan): persen 0 = paling KIRI (+pan_left_limit), 100 = paling
# KANAN (pan_right_limit). Servo 20 (head_tilt): derajat servo = 180 + sudut
# sendi (POSITIF = MENUNDUK di robot ini). Konvensi yang sama dengan mode D58/D60.
SERVO_CENTER_DEG = 180.0


class HeadTrackingNode(Node):
    """Head-only ball tracker - no walking or kicking."""

    def __init__(self) -> None:
        super().__init__("head_tracking_node")

        self._ball_center_topic = str(
            self.declare_parameter("ball_center_topic", "/vision/yolo/ball_center").value
        )

        # PID gains copied from the original OP3 ball tracker defaults.
        self._p_gain = float(self.declare_parameter("p_gain", 0.30).value)
        self._i_gain = float(self.declare_parameter("i_gain", 0.0).value)
        self._d_gain = float(self.declare_parameter("d_gain", 0.045).value)

        # Deadzone around image center — don't react to YOLO jitter when the
        # ball is essentially centered already.
        self._center_deadzone = float(
            self.declare_parameter("center_deadzone", 0.10).value
        )
        # EMA smoothing on ball center. 1.0 = no filtering.
        self._input_smoothing = float(
            self.declare_parameter("input_smoothing", 0.4).value
        )

        self._control_rate = float(self.declare_parameter("control_rate", 20.0).value)
        self._lost_timeout = float(self.declare_parameter("lost_timeout", 0.8).value)
        # Sesudah bola pernah terkunci, jangan buru-buru menyapu lagi kalau
        # deteksinya cuma putus sekejap. Diukur dari deteksi TERAKHIR, sama
        # seperti lost_timeout, jadi nilainya harus lebih besar dari itu:
        # di bawah lost_timeout bolanya masih dilacak seperti biasa, di antara
        # keduanya kepala DITAHAN diam, di atasnya baru boleh menyapu lagi.
        self._lock_hold_sec = float(
            self.declare_parameter("lock_hold_sec", 2.0).value
        )
        self._scan_enabled = bool(self.declare_parameter("scan_enabled", True).value)
        self._scan_period_sec = float(self.declare_parameter("scan_period_sec", 2.5).value)
        self._scan_pan_rad = float(self.declare_parameter("scan_pan_rad", 0.7).value)
        self._scan_tilt_down_rad = float(
            self.declare_parameter("scan_tilt_down_rad", -0.35).value
        )
        # Lebar baris BAWAH sebagai pecahan scan_pan_rad. 1,0 = sama lebar dengan
        # baris depan; 0,7 (nilai lama) mempersempitnya supaya kamera melihat di
        # antara kedua kaki.
        self._scan_pan_down_ratio = float(
            self.declare_parameter("scan_pan_down_ratio", 1.0).value
        )
        # BATAS ATAS sapuan (paling mendongak), dipakai keempat titik ujung pan.
        # head_tilt POSITIF = MENUNDUK di robot ini, jadi kepala melihat lebih
        # tinggi kalau angka ini DIKURANGI. Menggantikan scan_tilt_forward_rad
        # dan scan_tilt_down_side_rad (2026-09-15), yang dulu harus selalu
        # diisi sama. Dijepit ke rentang terverifikasi di head_scan_pattern.py.
        self._scan_tilt_upper_rad = float(
            self.declare_parameter("scan_tilt_upper_rad", -0.12).value
        )
        # Sudut tilt pose istirahat sesudah perintah "stop". Dipisah dari
        # sudut sapuan supaya mengubah sapuan tidak ikut memindahkan pose
        # istirahat kepala.
        self._scan_neutral_tilt_rad = float(
            self.declare_parameter("scan_neutral_tilt_rad", -0.12).value
        )
        # POSE PARKIR sesudah "stop" = posisi awal kepala setiap halaman action
        # (tendang/bangun) dimulai: servo 19 di 0 deg sendi (50 %), servo 20 di
        # 200 deg. Ketiga demo (D56/D58/D60) memarkir lewat perintah "stop" ini
        # SEBELUM menyerahkan kepala ke action_module.
        self._park_pan_deg = float(self.declare_parameter("park_pan_deg", 0.0).value)
        self._park_tilt_servo_deg = float(
            self.declare_parameter("park_tilt_servo_deg", 200.0).value
        )
        # SAPUAN BARU (17 Sep 2026): siklus 7 tahap berbasis waktu, kedua servo
        # diinterpolasi bersama, tanpa jeda diam antar tahap. Dinyatakan dalam
        # persen servo 19 dan derajat servo 20; diawali dari titik awal.
        self._scan_pan_left_limit_deg = float(
            self.declare_parameter("scan_pan_left_limit_deg", 85.0).value
        )
        self._scan_pan_right_limit_deg = float(
            self.declare_parameter("scan_pan_right_limit_deg", -85.0).value
        )
        self._scan_start_pan_percent = float(
            self.declare_parameter("scan_start_pan_percent", 50.0).value
        )
        self._scan_start_tilt_servo_deg = float(
            self.declare_parameter("scan_start_tilt_servo_deg", 207.0).value
        )
        # Titik PRA-AWAL: saat sapuan mulai (pertama kali / sesudah halaman
        # action) kepala dibawa dulu ke (50 %, 180), baru perlahan ke titik awal
        # (50 %, 207) dengan laju yang sama, baru siklus tahap 1..7.
        self._scan_prestart_tilt_servo_deg = float(
            self.declare_parameter("scan_prestart_tilt_servo_deg", 180.0).value
        )
        # Daftar datar [pan_persen, tilt_servo_deg, ...] tahap 1..N, diulang.
        self._scan_stages_raw = [
            float(v)
            for v in self.declare_parameter(
                "scan_stages_percent_servo",
                [70.0, 207.0, 100.0, 180.0, 100.0, 135.0, 0.0, 135.0,
                 0.0, 180.0, 30.0, 207.0, 50.0, 207.0],
            ).value
        ]
        # Laju sapuan, deg/s, sumbu yang bergerak paling jauh; sumbu lain
        # mengikuti proporsional supaya keduanya sampai bersamaan.
        self._scan_speed_deg_s = float(
            self.declare_parameter("scan_speed_deg_s", 30.0).value
        )
        self._scan_min_stage_sec = float(
            self.declare_parameter("scan_min_stage_sec", 0.2).value
        )
        # Mode sapuan: "legacy" = titik sapuan lama (scan_pan_rad / scan_tilt_*
        # / scan_period_sec, dipakai D56 dan D58), "stages" = siklus 7 tahap di
        # atas (HANYA D60). Dipilih lewat /ball_tracker/command "scan_stages" /
        # "scan_legacy"; D60 mengirim scan_stages sebelum start dan scan_legacy
        # saat berhenti, jadi demo lain tidak terpengaruh.
        self._scan_mode = str(self.declare_parameter("scan_mode", "legacy").value)
        # Mode "walk" (D60 SEARCH_BALL: bola tidak terdeteksi + badan MENCARI CELAH):
        # servo 20 tetap, servo 19 bolak-balik antar titik di daftar ini
        # [pan_persen, tilt_servo_deg, ...], laju sama (scan_speed_deg_s).
        self._scan_walk_raw = [
            float(v)
            for v in self.declare_parameter(
                "scan_walk_percent_servo", [0.0, 169.0, 100.0, 169.0]
            ).value
        ]
        # Mode "hold" (D60, diam PERTAMA sesudah D60 ditekan): kepala ditahan di
        # titik ini [pan_persen, tilt_servo_deg], tidak menyapu; bola terdeteksi
        # tetap langsung dilacak seperti mode lain.
        self._scan_hold_raw = [
            float(v)
            for v in self.declare_parameter(
                "scan_hold_percent_servo", [50.0, 135.0]
            ).value
        ]
        self._head_enable_retry_sec = float(
            self.declare_parameter("head_enable_retry_sec", 1.5).value
        )
        self._head_enable_boot_sec = float(
            self.declare_parameter("head_enable_boot_sec", 5.0).value
        )
        # Berapa lama kepala boleh dipegang modul LAIN sebelum node ini
        # merebutnya kembali. Lihat _ctrl_modules_callback.
        self._module_reclaim_sec = float(
            self.declare_parameter("module_reclaim_sec", 2.0).value
        )

        # Original OP3 tracker FOV values.
        self._fov_width_rad = math.radians(
            float(self.declare_parameter("fov_width_deg", 35.2).value)
        )
        self._fov_height_rad = math.radians(
            float(self.declare_parameter("fov_height_deg", 21.6).value)
        )
        self._min_command_rad = math.radians(
            float(self.declare_parameter("min_command_deg", 1.0).value)
        )
        self._max_offset_pan_rad = abs(
            float(self.declare_parameter("max_offset_pan_rad", 0.25).value)
        )
        self._max_offset_tilt_rad = abs(
            float(self.declare_parameter("max_offset_tilt_rad", 0.20).value)
        )

        self._pan_sign = float(self.declare_parameter("pan_sign", 1.0).value)
        self._tilt_sign = float(self.declare_parameter("tilt_sign", -1.0).value)

        # When False the node starts paused; a "start" on /ball_tracker/command
        # activates it. Used in Demo mode so the head only moves after the user
        # presses the button to start soccer mode.
        self._active = bool(self.declare_parameter("auto_start", True).value)

        self._head_pan_joint = str(
            self.declare_parameter("head_pan_joint", "head_pan").value
        )
        self._head_tilt_joint = str(
            self.declare_parameter("head_tilt_joint", "head_tilt").value
        )
        self._head_module_name = str(
            self.declare_parameter("head_module_name", "head_control_module").value
        )

        self._head_offset_pub = self.create_publisher(
            JointState, "/robotis/head_control/set_joint_states_offset", 10
        )
        self._head_abs_pub = self.create_publisher(
            JointState, "/robotis/head_control/set_joint_states", 10
        )
        self._enable_pub = self.create_publisher(
            String, "/robotis/enable_ctrl_module", 10
        )
        self._joint_ctrl_pub = self.create_publisher(
            JointCtrlModule, "/robotis/set_joint_ctrl_modules", 10
        )
        # Status EKSPLISIT node ini, tiap tick kendali: INACTIVE, SCANNING,
        # HOLD (bola sempat terkunci lalu hilang sesaat, kepala ditahan), atau
        # TRACKING. Hanya melaporkan; tidak mengubah sapuan maupun pelacakan.
        # Dipakai mode otomatis D60 supaya "kepala sedang menyapu" tidak pernah
        # tertukar dengan "kepala sedang mengunci bola" -- sudut servo saja
        # tidak bisa membedakan keduanya.
        self._status_pub = self.create_publisher(String, "/ball_tracker/status", 10)

        self.create_subscription(
            PointStamped,
            self._ball_center_topic,
            self._ball_center_callback,
            10,
        )
        self.create_subscription(
            String,
            "/ball_tracker/command",
            self._tracker_command_callback,
            10,
        )
        # Siapa yang sedang memegang sendi kepala. Diterbitkan controller
        # SETIAP KALI kepemilikan modul berubah.
        self.create_subscription(
            JointCtrlModule,
            "/robotis/present_joint_ctrl_modules",
            self._ctrl_modules_callback,
            10,
        )

        # Goal kepala dari controller (rad), untuk offset sapuan = setpoint - goal.
        self.create_subscription(
            JointState, "/robotis/goal_joint_states", self._goal_joints_callback, 10
        )
        self._goal_pan_rad: Optional[float] = None
        self._goal_tilt_rad: Optional[float] = None
        # Segmen sapuan yang sedang berjalan: (pan0, tilt0) -> (pan1, tilt1) rad,
        # mulai t0, lama dur. Indeks -1 = segmen menuju titik awal.
        self._scan_seg_from = (0.0, 0.0)
        self._scan_seg_to = (0.0, 0.0)
        self._scan_seg_t0 = 0.0
        self._scan_seg_dur = 0.0
        self._scan_stage_idx = -1

        self._last_ball_time: Optional[float] = None
        self._last_ball_x: Optional[float] = None
        self._last_ball_y: Optional[float] = None
        self._last_x_norm: float = 0.0
        self._last_y_norm: float = 0.0
        self._prev_pan_error = 0.0
        self._prev_tilt_error = 0.0
        self._pan_error_sum = 0.0
        self._tilt_error_sum = 0.0
        self._last_control_time = time.monotonic()
        self._last_head_enable_time = 0.0
        self._joint_ctrl_modules_sent = False
        self._scan_active = False
        self._last_scan_cmd_time = 0.0
        self._scan_idx = 0
        # True sejak bola benar-benar dilacak. Dipakai untuk menahan sapuan
        # selama lock_hold_sec sesudah deteksinya hilang.
        self._locked = False
        # True hanya selama ada deteksi yang BELUM dipakai jadi perintah.
        self._ball_unused = False
        # False selama belum ada galat sebelumnya yang boleh dipakai suku D.
        self._prev_error_valid = False
        # Pemilik sendi kepala menurut controller, dan sejak kapan ia bukan
        # head_control_module.
        self._head_owner: Optional[str] = None
        self._head_lost_since: Optional[float] = None

        # Pola sapuan, searah keliling dan DIAWALI MENUNDUK KE DEPAN (pan 0).
        # Titik depan itu satu-satunya yang memakai batas BAWAH: di sana tidak
        # ada bahu yang menghalangi. Begitu kepala menoleh ke ujung, ia naik ke
        # batas ATAS.
        #
        #   1. (    0, batas bawah)  <- awal, melihat lantai tepat di depan
        #   2. (-pan , batas atas)
        #   3. (-pan , batas atas)   <- sasaran sama: kepala diam
        #   4. (+pan , batas atas)
        #   5. (+pan , batas atas)   <- sasaran sama -> balik ke 1
        #
        # Urutannya BUKAN sekadar selera: sasaran pan ujung harus bertahan dua
        # langkah berturut-turut, kalau tidak kepala diperintah pergi sebelum
        # sampai dan sapuannya tidak pernah selebar yang tertulis. Lihat tabel
        # waktu di head_tracking.yaml.
        self._scan_tilt_upper_rad, upper_problem = resolve_scan_tilt_upper(
            self._scan_tilt_upper_rad, self._scan_tilt_down_rad
        )
        if upper_problem:
            self.get_logger().error(upper_problem)
        self._scan_positions = build_scan_positions(
            self._scan_pan_rad,
            self._scan_pan_down_ratio,
            self._scan_tilt_upper_rad,
            self._scan_tilt_down_rad,
        )

        self.create_timer(1.0 / self._control_rate, self._control_loop)
        if self._active:
            self._ensure_head_module_enabled(force=True)

        self.get_logger().info(
            f"Head Tracking Node initialized (auto_start={self._active})"
        )
        self.get_logger().info(f"  Ball center topic: {self._ball_center_topic}")
        self.get_logger().info(
            f"  PID gains: P={self._p_gain:.3f}, I={self._i_gain:.3f}, D={self._d_gain:.3f}"
        )
        self.get_logger().info(f"  Scan enabled: {self._scan_enabled}")
        self.get_logger().info(
            "  Sapuan: pan +-%.1f deg (total %.0f deg, baris bawah %.0f%%), "
            "tilt batas ATAS %+.1f deg (servo 20 %.1f) .. batas BAWAH %+.1f deg "
            "(servo 20 %.1f), istirahat %+.1f deg, periode %.1f s, satu putaran %.1f s"
            % (math.degrees(self._scan_pan_rad), 2 * math.degrees(self._scan_pan_rad),
               self._scan_pan_down_ratio * 100.0,
               math.degrees(self._scan_tilt_upper_rad),
               180.0 + math.degrees(self._scan_tilt_upper_rad),
               math.degrees(self._scan_tilt_down_rad),
               180.0 + math.degrees(self._scan_tilt_down_rad),
               math.degrees(self._scan_neutral_tilt_rad),
               self._scan_period_sec,
               len(self._scan_positions) * self._scan_period_sec)
        )
        stages = self._scan_stages()
        self.get_logger().info(
            "  SAPUAN 7 TAHAP [mode D60, aktif lewat perintah scan_stages; sekarang: %s] (persen servo 19 / "
            "servo 20 deg), awal (%.0f%%, %.0f) lalu: %s | laju %.0f deg/s, tanpa jeda antar tahap | "
            "parkir (\"stop\") pan %+.0f deg, servo 20 %.0f"
            % (self._scan_mode, self._scan_start_pan_percent, self._scan_start_tilt_servo_deg,
               " -> ".join("(%.0f%%, %.0f)" % (p, t) for p, t in stages),
               self._scan_speed_deg_s, self._park_pan_deg, self._park_tilt_servo_deg)
        )
        self.get_logger().info(
            "  dalam sudut sendi: %s"
            % " -> ".join("(%+.1f, %+.1f)" % (math.degrees(a), math.degrees(b))
                          for a, b in (self._percent_servo_to_rad(p, t) for p, t in stages))
        )

    def _tracker_command_callback(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd in ("scan_stages", "scan_legacy", "scan_walk", "scan_hold"):
            mode = {"scan_stages": "stages", "scan_legacy": "legacy", "scan_walk": "walk",
                    "scan_hold": "hold"}[cmd]
            if mode != self._scan_mode:
                self._scan_mode = mode
                self._scan_active = False
                self._last_scan_cmd_time = 0.0
                self._scan_stage_idx = -1
                self._scan_idx = 0
                self.get_logger().info(f"[CMD] mode sapuan -> {mode}")
            return
        if cmd == "start" and not self._active:
            self._active = True
            self._reset_pid()
            self._last_ball_x = None
            self._last_ball_y = None
            self._last_ball_time = None
            self._locked = False
            self._ball_unused = False
            self._last_head_enable_time = 0.0
            self._joint_ctrl_modules_sent = False
            self._scan_stage_idx = -1
            self._ensure_head_module_enabled(force=True)
            self.get_logger().info("[CMD] activated by start command")
        elif (cmd == "stop" or cmd.startswith("stop:")) and self._active:
            # "stop:<servo20>" = sama dengan stop, tetapi servo 20 diparkir ke sudut
            # itu (D60 OFF: 170). "stop" biasa tetap park_tilt_servo_deg (200,
            # pose awal halaman action).
            park_tilt_servo = self._park_tilt_servo_deg
            if cmd.startswith("stop:"):
                try:
                    park_tilt_servo = float(cmd.split(":", 1)[1])
                except ValueError:
                    self.get_logger().warn(f"[CMD] '{cmd}' tidak sah; parkir servo 20 {park_tilt_servo:.0f}")
            self._active = False
            self._reset_pid()
            self._scan_active = False
            self._last_scan_cmd_time = 0.0
            self._locked = False
            self._ball_unused = False
            self._scan_stage_idx = -1
            # PARKIR: pose awal setiap halaman action (pan 0 deg, servo 20 200).
            # Dikirim absolut, jadi sapuan/tracking yang masih berjalan diganti.
            park_pan, park_tilt = self._percent_servo_to_rad(
                self._pan_deg_to_percent(self._park_pan_deg), park_tilt_servo
            )
            neutral = JointState()
            neutral.name = [self._head_pan_joint, self._head_tilt_joint]
            neutral.position = [park_pan, park_tilt]
            neutral.header.stamp = self.get_clock().now().to_msg()
            self._head_abs_pub.publish(neutral)
            self.get_logger().info(
                "[CMD] deactivated by stop command; kepala diparkir ke pan %+.0f deg, servo 20 %.0f"
                % (self._park_pan_deg, park_tilt_servo)
            )

    def _ball_center_callback(self, msg: PointStamped) -> None:
        """Track the latest normalized YOLO center point."""
        x_norm = float(msg.point.x)
        y_norm = float(msg.point.y)

        if math.isnan(x_norm) or math.isnan(y_norm):
            return

        # /vision/yolo/ball_center is normalized to [0, 1]. Convert to the same
        # [-1, 1] image coordinate convention used by the original OP3 tracker:
        # (-1, -1) = top-left, (+1, +1) = bottom-right.
        self._last_x_norm = x_norm
        self._last_y_norm = y_norm
        raw_x = (x_norm * 2.0) - 1.0
        raw_y = (y_norm * 2.0) - 1.0

        a = max(0.0, min(1.0, self._input_smoothing))
        if self._last_ball_x is None or self._last_ball_y is None:
            self._last_ball_x = raw_x
            self._last_ball_y = raw_y
        else:
            self._last_ball_x = a * raw_x + (1.0 - a) * self._last_ball_x
            self._last_ball_y = a * raw_y + (1.0 - a) * self._last_ball_y
        self._last_ball_time = time.monotonic()
        self._ball_unused = True

    def _publish_status(self, status: str) -> None:
        self._status_pub.publish(String(data=status))

    def _control_loop(self) -> None:
        if not self._active:
            self._publish_status("INACTIVE")
            return

        now = time.monotonic()
        self._reclaim_head_if_lost(now)

        ball_visible = (
            self._last_ball_time is not None
            and self._last_ball_x is not None
            and self._last_ball_y is not None
            and (now - self._last_ball_time) < self._lost_timeout
        )

        if ball_visible:
            self._publish_status("TRACKING")
            self._track_ball(now)
        elif (
            self._locked
            and self._last_ball_time is not None
            and (now - self._last_ball_time) < self._lock_hold_sec
        ):
            # Bola sempat terkunci lalu deteksinya putus sebentar -- tertutup
            # kaki, satu frame meleset, bola diam di pinggir bingkai. Kepala
            # DITAHAN di tempat, tidak menyapu: sekali menyapu kuncinya lepas
            # dan bolanya baru ketemu lagi satu putaran kemudian.
            self._publish_status("HOLD")
            self._reset_pid()
            self._ensure_head_module_enabled(now)
        else:
            self._publish_status("SCANNING" if self._scan_enabled else "IDLE")
            self._locked = False
            self._reset_pid()
            self._ensure_head_module_enabled(now)  # retry only when idle
            if self._scan_enabled:
                self._ensure_scan_mode(now)

    def _track_ball(self, now: float) -> None:
        if self._last_ball_x is None or self._last_ball_y is None:
            return

        if self._scan_active:
            self._scan_active = False
            self._last_scan_cmd_time = 0.0
            self._scan_stage_idx = -1
            # Sapuan meninggalkan trajektori ABSOLUT yang MASIH BERJALAN di
            # head_control_module -- satu segmen bisa 2,7 detik. Koreksi di
            # bawah belum tentu jadi diterbitkan (bola sudah di dalam
            # center_deadzone, atau koreksinya lebih kecil dari min_command),
            # dan kalau itu terjadi kepala MENERUSKAN sapuannya lalu melewati
            # bolanya. Offset nol menyetel target = goal sekarang dengan
            # moving_time 0,1 s, jadi kepala berhenti di tempat begitu bola
            # terlihat, sebelum PID mulai membetulkan posisinya.
            self._publish_offset(0.0, 0.0)
        self._locked = True

        # SATU deteksi = SATU evaluasi PID. Dulu PID dihitung tiap tick (20 Hz)
        # sedangkan galatnya cuma berubah tiap deteksi (terukur 8 Hz di robot),
        # jadi delta_time suku D memakai 0,05 s padahal galatnya butuh 0,125 s
        # -- suku D jadi 2,5x terlalu besar. Digabung dengan tendangan turunan
        # di bawah, perintah pertama terukur -12,6 deg untuk galat yang cuma
        # -10,4 deg, dan kepala mengayun sampai -21,6 deg: LEWAT dari bolanya,
        # bukan mengunci. Sekarang seluruh PID maju per deteksi, jadi
        # delta_time-nya memang jarak antar deteksi yang sebenarnya.
        if not self._ball_unused:
            return
        self._ball_unused = False

        # Deadzone: ball is within the inner box around image center, hold still.
        # Stops YOLO detection jitter from turning into head oscillation when the
        # ball is stationary.
        if (
            abs(self._last_ball_x) < self._center_deadzone
            and abs(self._last_ball_y) < self._center_deadzone
        ):
            # Jangan simpan galat nol sebagai riwayat: begitu bola keluar
            # deadzone lagi, selisihnya akan terbaca sebagai lonjakan dan suku
            # D menendang. Cukup tandai riwayatnya tidak sahih.
            self._prev_error_valid = False
            return

        # Preserve the original OP3 sign convention.
        pan_error = -math.atan(self._last_ball_x * math.tan(self._fov_width_rad))
        tilt_error = -math.atan(self._last_ball_y * math.tan(self._fov_height_rad))

        delta_time = max(now - self._last_control_time, 1.0 / 240.0)
        self._last_control_time = now

        # Tendangan turunan: saat bola BARU ketemu, _prev_*_error masih nol
        # sedangkan galatnya sudah sebesar simpangan bola dari tengah. Selisih
        # itu bukan gerakan bola, cuma awal pengukuran -- terukur menghasilkan
        # suku D -9,4 deg melawan suku P yang cuma -3,1 deg. Deteksi pertama
        # sekarang dijalankan tanpa suku D sama sekali.
        if self._prev_error_valid:
            pan_error_diff = (pan_error - self._prev_pan_error) / delta_time
            tilt_error_diff = (tilt_error - self._prev_tilt_error) / delta_time
        else:
            pan_error_diff = 0.0
            tilt_error_diff = 0.0

        self._pan_error_sum += pan_error
        self._tilt_error_sum += tilt_error

        pan_cmd = (
            pan_error * self._p_gain
            + pan_error_diff * self._d_gain
            + self._pan_error_sum * self._i_gain
        )
        tilt_cmd = (
            tilt_error * self._p_gain
            + tilt_error_diff * self._d_gain
            + self._tilt_error_sum * self._i_gain
        )

        self._prev_pan_error = pan_error
        self._prev_tilt_error = tilt_error
        self._prev_error_valid = True

        pan_cmd *= self._pan_sign
        tilt_cmd *= self._tilt_sign

        pan_cmd = max(-self._max_offset_pan_rad, min(self._max_offset_pan_rad, pan_cmd))
        tilt_cmd = max(-self._max_offset_tilt_rad, min(self._max_offset_tilt_rad, tilt_cmd))

        if abs(pan_cmd) < self._min_command_rad and abs(tilt_cmd) < self._min_command_rad:
            return

        self._publish_offset(pan_cmd, tilt_cmd)

        self.get_logger().info(
            f"[TRACK] norm=({self._last_x_norm:.2f},{self._last_y_norm:.2f}) "
            f"xy=({self._last_ball_x:.2f},{self._last_ball_y:.2f}) "
            f"offset=({pan_cmd:.3f},{tilt_cmd:.3f})",
            throttle_duration_sec=0.5,
        )

    def _ctrl_modules_callback(self, msg: JointCtrlModule) -> None:
        """Catat modul mana yang sedang memegang sendi kepala."""
        owner = None
        for joint, module in zip(msg.joint_name, msg.module_name):
            if joint in (self._head_pan_joint, self._head_tilt_joint):
                owner = module
                break
        if owner is None:
            return

        if owner != self._head_owner:
            self.get_logger().info(f"[MODUL] sendi kepala kini dipegang: {owner}")
        self._head_owner = owner

        if owner == self._head_module_name:
            self._head_lost_since = None
        elif self._head_lost_since is None:
            self._head_lost_since = time.monotonic()

    def _reclaim_head_if_lost(self, now: float) -> None:
        """Rebut kembali kepala kalau ada yang menyapunya secara tidak sengaja.

        Latar belakangnya nyata dan terekam di log robot: satu pesan "ini_pose"
        ke /robotis/base/ini_pose membuat base_module meminta SELURUH 20 sendi
        (base_module memang memiliki semuanya), jadi kepala ikut terbawa
        walaupun yang dimaksud cuma init pose. Sesudah itu head_control_module
        mati dan SEMUA perintah node ini dibuang diam-diam -- di log manager
        muncul "Head module is not enable." berulang-ulang. Node ini dulu
        berhenti mencoba enable sesudah head_enable_boot_sec (5 detik), jadi
        kepala TIDAK PERNAH kembali sampai operator menekan START lagi.
        Itulah gejala "di tengah demo tiba-tiba parameternya lain".

        Sengaja TIDAK merebut dari action_module atau walking_module: keduanya
        memang mengambil kepala dengan sengaja saat menendang atau bangun
        (SoccerDemo::setModuleToDemo("action_module")), dan SoccerDemo
        mengembalikannya sendiri sesudah selesai. Yang direbut hanya kalau
        kepala nyangkut di base_module/none, yang tidak ada yang memulihkan.
        """
        if self._head_lost_since is None or self._head_owner is None:
            return
        if self._head_owner not in ("base_module", "none", ""):
            return
        if (now - self._head_lost_since) < self._module_reclaim_sec:
            return

        self.get_logger().warn(
            f"[MODUL] kepala nyangkut di '{self._head_owner}' selama "
            f"{now - self._head_lost_since:.1f} s -- direbut kembali"
        )
        self._ensure_head_module_enabled(force=True)
        # beri jeda sebelum mencoba lagi kalau rebutannya gagal
        self._head_lost_since = now

    def _publish_offset(self, pan: float, tilt: float) -> None:
        """Terbitkan koreksi RELATIF terhadap goal kepala saat ini."""
        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan, tilt]
        msg.header.stamp = self.get_clock().now().to_msg()
        self._head_offset_pub.publish(msg)

    # ---- konversi persen / derajat servo <-> sudut sendi -------------------
    def _pan_percent_to_rad(self, percent: float) -> float:
        left = self._scan_pan_left_limit_deg
        right = self._scan_pan_right_limit_deg
        percent = max(0.0, min(100.0, percent))
        return math.radians(left - percent / 100.0 * (left - right))

    def _pan_deg_to_percent(self, pan_deg: float) -> float:
        left = self._scan_pan_left_limit_deg
        right = self._scan_pan_right_limit_deg
        return (left - pan_deg) / (left - right) * 100.0

    def _percent_servo_to_rad(self, pan_percent: float, tilt_servo_deg: float):
        return (self._pan_percent_to_rad(pan_percent), math.radians(tilt_servo_deg - SERVO_CENTER_DEG))

    def _scan_stages(self):
        raw = self._scan_stages_raw
        stages = [(raw[i], raw[i + 1]) for i in range(0, len(raw) - 1, 2)]
        if not stages:
            stages = [(self._scan_start_pan_percent, self._scan_start_tilt_servo_deg)]
        return stages

    def _goal_joints_callback(self, msg: JointState) -> None:
        n = min(len(msg.name), len(msg.position))
        for i in range(n):
            if msg.name[i] == self._head_pan_joint:
                self._goal_pan_rad = float(msg.position[i])
            elif msg.name[i] == self._head_tilt_joint:
                self._goal_tilt_rad = float(msg.position[i])

    # ---- sapuan berbasis waktu ------------------------------------------------
    def _begin_scan_segment(self, now: float, target, stage_idx: int) -> None:
        """Mulai segmen menuju target (rad) dari setpoint/goal saat ini."""
        if self._scan_active and self._scan_seg_dur > 0.0:
            origin = self._scan_seg_to
        elif self._goal_pan_rad is not None and self._goal_tilt_rad is not None:
            origin = (self._goal_pan_rad, self._goal_tilt_rad)
        else:
            origin = target
        d_pan = abs(math.degrees(target[0] - origin[0]))
        d_tilt = abs(math.degrees(target[1] - origin[1]))
        speed = max(1.0, self._scan_speed_deg_s)
        # Kedua sumbu sampai BERSAMAAN: lamanya dari sumbu yang paling jauh.
        dur = max(self._scan_min_stage_sec, max(d_pan, d_tilt) / speed)
        self._scan_seg_from = origin
        self._scan_seg_to = target
        self._scan_seg_t0 = now
        self._scan_seg_dur = dur
        self._scan_stage_idx = stage_idx
        self._scan_active = True
        self.get_logger().info(
            "[SCAN] tahap %s -> pan %+.1f deg (%.0f%%), servo 20 %.1f, %.2f s"
            % ("pra-awal" if stage_idx == -2 else "awal" if stage_idx < 0 else str(stage_idx + 1),
               math.degrees(target[0]), self._pan_deg_to_percent(math.degrees(target[0])),
               SERVO_CENTER_DEG + math.degrees(target[1]), dur)
        )

    def _ensure_scan_mode(self, now: Optional[float] = None) -> None:
        if self._scan_mode == "stages":
            self._ensure_scan_stages(now)
        elif self._scan_mode == "walk":
            self._ensure_scan_walk(now)
        elif self._scan_mode == "hold":
            self._ensure_scan_walk(now, self._scan_hold_raw)
        else:
            self._ensure_scan_legacy(now)

    def _ensure_scan_legacy(self, now: Optional[float] = None) -> None:
        """Sapuan LAMA (D56/D58): titik absolut tiap scan_period_sec."""
        if now is None:
            now = time.monotonic()
        if (now - self._last_scan_cmd_time) < self._scan_period_sec:
            return
        pan, tilt = self._scan_positions[self._scan_idx]
        self._scan_idx = (self._scan_idx + 1) % len(self._scan_positions)

        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan, tilt]
        msg.header.stamp = self.get_clock().now().to_msg()
        self._head_abs_pub.publish(msg)
        self._last_scan_cmd_time = now
        self._scan_active = True
        self.get_logger().info(
            f"[SCAN] move to ({math.degrees(pan):+.0f}°, {math.degrees(tilt):+.0f}°)",
            throttle_duration_sec=1.0,
        )

    def _ensure_scan_walk(self, now: Optional[float] = None, raw=None) -> None:
        """Sapuan mode walk (D60 SEARCH_BALL): servo 19 bolak-balik 0 <-> 100 %,
        servo 20 tetap 169, laju scan_speed_deg_s, tanpa jeda; dikirim sebagai
        offset seperti mode stages. Dimulai dari ujung yang lebih dekat."""
        if now is None:
            now = time.monotonic()
        if raw is None:
            raw = self._scan_walk_raw
        points = [(raw[i], raw[i + 1]) for i in range(0, len(raw) - 1, 2)] or [(50.0, 169.0)]
        if not self._scan_active:
            start = 0
            if self._goal_pan_rad is not None and len(points) > 1:
                start = min(range(len(points)),
                            key=lambda k: abs(self._pan_percent_to_rad(points[k][0]) - self._goal_pan_rad))
            self._begin_scan_segment(now, self._percent_servo_to_rad(*points[start]), start)
        guard = 0
        while now - self._scan_seg_t0 >= self._scan_seg_dur and guard < len(points) + 1:
            leftover = now - self._scan_seg_t0 - self._scan_seg_dur
            next_idx = (max(self._scan_stage_idx, 0) + 1) % len(points)
            self._begin_scan_segment(now - leftover, self._percent_servo_to_rad(*points[next_idx]), next_idx)
            guard += 1
        u = 0.0 if self._scan_seg_dur <= 0.0 else min(1.0, (now - self._scan_seg_t0) / self._scan_seg_dur)
        sp_pan = self._scan_seg_from[0] + u * (self._scan_seg_to[0] - self._scan_seg_from[0])
        sp_tilt = self._scan_seg_from[1] + u * (self._scan_seg_to[1] - self._scan_seg_from[1])
        if self._goal_pan_rad is None or self._goal_tilt_rad is None:
            if (now - self._last_scan_cmd_time) >= 0.5:
                msg = JointState()
                msg.name = [self._head_pan_joint, self._head_tilt_joint]
                msg.position = [sp_pan, sp_tilt]
                msg.header.stamp = self.get_clock().now().to_msg()
                self._head_abs_pub.publish(msg)
                self._last_scan_cmd_time = now
            return
        self._publish_offset(sp_pan - self._goal_pan_rad, sp_tilt - self._goal_tilt_rad)
        self._last_scan_cmd_time = now

    def _ensure_scan_stages(self, now: Optional[float] = None) -> None:
        """Sapuan 7 tahap berbasis waktu, non-blocking, dijalankan tiap tick (D60).

        Setpoint diinterpolasi linier di dalam tiap segmen (kedua servo
        bersamaan), lalu dikirim sebagai OFFSET = setpoint - goal kepala saat
        ini, sehingga head_control_module (moving_time 0,1 s) mengikuti dengan
        halus dan tanpa jeda diam antar tahap. Tahap terakhir langsung
        disambung ke tahap 1 (siklus). Batas menunduk mengikuti pan dijaga
        head_control_module (amplop), jadi target di luar amplop dijepit di sana.
        """
        if now is None:
            now = time.monotonic()
        stages = self._scan_stages()
        if not self._scan_active:
            # Mulai: ke PRA-AWAL (50 %, 180) dulu dari mana pun kepala berada,
            # lalu (segmen berikutnya) perlahan ke titik awal (50 %, 207).
            self._begin_scan_segment(
                now,
                self._percent_servo_to_rad(self._scan_start_pan_percent, self._scan_prestart_tilt_servo_deg),
                -2,
            )

        # Segmen selesai -> langsung tahap berikutnya (tanpa jeda).
        # Urutan: -2 (pra-awal 180) -> -1 (awal 207) -> 0..N-1 -> 0 ...
        guard = 0
        while now - self._scan_seg_t0 >= self._scan_seg_dur and guard < len(stages) + 2:
            leftover = now - self._scan_seg_t0 - self._scan_seg_dur
            if self._scan_stage_idx == -2:
                next_idx = -1
                p, t = self._scan_start_pan_percent, self._scan_start_tilt_servo_deg
            else:
                next_idx = (self._scan_stage_idx + 1) % len(stages)
                p, t = stages[next_idx]
            self._begin_scan_segment(now - leftover, self._percent_servo_to_rad(p, t), next_idx)
            guard += 1

        u = 0.0 if self._scan_seg_dur <= 0.0 else min(1.0, (now - self._scan_seg_t0) / self._scan_seg_dur)
        sp_pan = self._scan_seg_from[0] + u * (self._scan_seg_to[0] - self._scan_seg_from[0])
        sp_tilt = self._scan_seg_from[1] + u * (self._scan_seg_to[1] - self._scan_seg_from[1])

        if self._goal_pan_rad is None or self._goal_tilt_rad is None:
            # Goal kepala belum diterima: kirim absolut sekali supaya tetap bergerak.
            if (now - self._last_scan_cmd_time) >= 0.5:
                msg = JointState()
                msg.name = [self._head_pan_joint, self._head_tilt_joint]
                msg.position = [sp_pan, sp_tilt]
                msg.header.stamp = self.get_clock().now().to_msg()
                self._head_abs_pub.publish(msg)
                self._last_scan_cmd_time = now
            return

        self._publish_offset(sp_pan - self._goal_pan_rad, sp_tilt - self._goal_tilt_rad)
        self._last_scan_cmd_time = now

    def _reset_pid(self) -> None:
        self._prev_pan_error = 0.0
        self._prev_tilt_error = 0.0
        self._prev_error_valid = False
        self._pan_error_sum = 0.0
        self._tilt_error_sum = 0.0
        self._last_control_time = time.monotonic()

    def _ensure_head_module_enabled(
        self, now: Optional[float] = None, force: bool = False
    ) -> None:
        if now is None:
            now = time.monotonic()

        if not force and (now - self._last_head_enable_time) < self._head_enable_retry_sec:
            return

        # Always safe to re-publish enable_ctrl_module: MotionModule::setModuleEnable
        # is guarded (`if this->enable_ == enable return;`), so repeat enables are
        # no-ops and do NOT reset scan_state_. If op3_manager came up slowly and the
        # first message was lost, subsequent tries will land.
        self._enable_pub.publish(String(data=self._head_module_name))

        # set_joint_ctrl_modules re-runs sync_write setup and can disrupt an
        # in-progress scan, so only publish it at startup (or explicit force).
        if force or not self._joint_ctrl_modules_sent:
            ctrl_msg = JointCtrlModule()
            ctrl_msg.joint_name = [self._head_pan_joint, self._head_tilt_joint]
            ctrl_msg.module_name = [self._head_module_name, self._head_module_name]
            self._joint_ctrl_pub.publish(ctrl_msg)
            self._joint_ctrl_modules_sent = True

        self._last_head_enable_time = now


def main() -> None:
    rclpy.init()
    node = HeadTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
