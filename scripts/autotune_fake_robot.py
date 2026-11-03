#!/usr/bin/env python3
"""ROBOT TIRUAN untuk menguji scripts/autotune_walking.py TANPA robot.

Meniru antarmuka op3_manager yang dipakai auto-tune (service modul sendi,
get_params walking, is_running action, topik perintah) dan menerbitkan IMU +
present_joint_states sintetis. Goyangan badan bergantung parameter lewat
"lanskap" buatan yang punya titik optimum dan bisa membuat robot "jatuh", jadi
optimizer dan seluruh alur percobaan teruji -- bukan model fisika.

WAJIB di domain ROS terisolasi supaya tidak pernah bertemu manager sungguhan:
  ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 scripts/autotune_fake_robot.py

Training page Action Editor (autotune_action.py) juga bisa diuji: beri
AUTOTUNE_FAKE_BIN (SALINAN .bin, sama dengan BASCORRO_TRAINING_BIN pelatih) dan
AUTOTUNE_FAKE_SOURCE_PAGE. Page kerja yang diputar dibandingkan dengan page asal
untuk membaca kenopnya; lanskap buatan optimum di pitch_ankle +1, roll_ankle -0,5.
AUTOTUNE_FAKE_LYING=1 = robot "berbaring" (profil bangun): berhasil tegak kalau
|pitch_hip| < 2 deg, lalu dibaringkan lagi 2 s sesudah gerakan selesai.
AUTOTUNE_FAKE_SPEED = pengali durasi page (0,25 = uji cepat).
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int32, String
from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam
from op3_action_module_msgs.srv import IsRunning
from robotis_controller_msgs.msg import StatusMsg, SyncWriteItem
from robotis_controller_msgs.srv import GetJointModule

import training_sources as src
from autotune_action_core import RAW_PER_DEG

# AUTOTUNE_FAKE_FALL=1: selalu jatuh 2 s sesudah start (menguji jalur jatuh).
FORCE_FALL = os.environ.get("AUTOTUNE_FAKE_FALL") == "1"
# init_pitch_offset di bawah ini = pose BERDIRI terguling 1000 deg/rad (menguji gerbang pose).
TIP_BELOW = float(os.environ.get("AUTOTUNE_FAKE_TIP_BELOW", "-0.04"))
FAKE_BIN = os.environ.get("AUTOTUNE_FAKE_BIN")
SOURCE_PAGE = int(os.environ.get("AUTOTUNE_FAKE_SOURCE_PAGE", "0"))
LYING = os.environ.get("AUTOTUNE_FAKE_LYING") == "1"
SPEED = float(os.environ.get("AUTOTUNE_FAKE_SPEED", "1.0"))
LEGS = ["r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
        "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll"]


def rocking_deg(p):
    """Lanskap buatan: optimum di dsp 0.30, y_swap 0.012, z_move 0.04."""
    return (2.0 + 300 * (p.dsp_ratio - 0.30) ** 2 + 4000 * (p.y_swap_amplitude - 0.012) ** 2
            + 2000 * (p.z_move_amplitude - 0.04) ** 2)


class FakeRobot(Node):
    def __init__(self):
        super().__init__("autotune_fake_robot")
        self.owner = "none"
        self.params = WalkingParam(period_time=0.65, soft_start_time=0.65, dsp_ratio=0.18, z_move_amplitude=0.05)
        self.action_until = 0.0
        self.action_since = 0.0
        self.knobs = None           # kenop page kerja yang sedang diputar (None = page biasa)
        self.lying = LYING
        self.walking = False
        self.walk_since = 0.0
        self.writes = 0
        self.imu_pub = self.create_publisher(Imu, "/robotis/open_cr/imu", 50)
        self.js_pub = self.create_publisher(JointState, "/robotis/present_joint_states", 20)
        self.status_pub = self.create_publisher(StatusMsg, "/robotis/status", 20)
        self.create_subscription(String, "/robotis/enable_ctrl_module", self.on_module, 10)
        self.create_subscription(Int32, "/robotis/action/page_num", self.on_page, 10)
        self.create_subscription(WalkingParam, "/robotis/walking/set_params", self.on_params, 10)
        self.create_subscription(String, "/robotis/walking/command", self.on_command, 10)
        self.create_subscription(SyncWriteItem, "/robotis/sync_write_item", self.on_sync, 10)
        self.create_service(GetJointModule, "/robotis/get_present_joint_ctrl_modules", self.srv_modules)
        self.create_service(GetWalkingParam, "/robotis/walking/get_params", self.srv_params)
        self.create_service(IsRunning, "/robotis/action/is_running", self.srv_running)
        self.create_timer(0.008, self.tick_imu)
        self.create_timer(0.02, self.tick_joints)
        self.create_timer(2.0, self.tick_volt)

    def on_module(self, msg):
        self.owner = msg.data
        if msg.data != "walking_module":
            self.walking = False

    def on_page(self, msg):
        if self.owner != "action_module":
            return
        now = time.monotonic()
        if msg.data == -2:                                   # brake
            self.action_until = now
            return
        self.knobs = None
        duration = 1.2
        if msg.data == src.TRAINING_SCRATCH_PAGE and FAKE_BIN:
            data = src.read_bin(FAKE_BIN)
            scratch, source = src.decode_page(data, msg.data), src.decode_page(data, SOURCE_PAGE)
            if scratch["stepnum"] == 0 or not scratch["checksum_ok"]:
                return                                       # = "Failed to start page"
            chain, _ = src.page_chain(data, msg.data)
            duration = SPEED * src.chain_seconds(data, chain)
            self.knobs = self.read_knobs(scratch, source)
        self.action_since, self.action_until = now, now + duration

    @staticmethod
    def read_knobs(scratch, source):
        def diff(joint):
            i = src.JOINT_IDS[joint]
            return (scratch["steps"][0]["positions"][i] - source["steps"][0]["positions"][i]) / RAW_PER_DEG
        return {"pitch_ankle_deg": diff("r_ank_pitch"), "roll_ankle_deg": diff("r_ank_roll"),
                "pitch_hip_deg": -diff("r_hip_pitch")}

    def on_params(self, msg):
        self.params = msg

    def on_sync(self, msg):
        self.writes += 1

    def on_command(self, msg):
        if msg.data == "start" and self.owner == "walking_module":
            self.walking, self.walk_since = True, time.monotonic()
            self.status_pub.publish(StatusMsg(status_msg="Start walking"))
        elif msg.data == "stop":
            self.walking = False

    def srv_modules(self, request, response):
        response.joint_name = list(request.joint_name)
        response.module_name = [self.owner] * len(request.joint_name)
        return response

    def srv_params(self, request, response):
        response.parameters = self.params
        return response

    def srv_running(self, request, response):
        response.is_running = time.monotonic() < self.action_until
        return response

    def tick_imu(self):
        t = time.monotonic()
        roll = pitch = gx = gz = 0.0
        if self.walking:
            w = 2 * math.pi / max(self.params.period_time, 0.1)
            amp = rocking_deg(self.params)
            roll = amp * math.sin(w * t)
            gx = math.radians(amp) * w * math.cos(w * t)
            gz = math.radians(1.0 - 80 * self.params.angle_move_amplitude)   # belok, optimum 0.0125
            if (amp > 12 or FORCE_FALL) and t - self.walk_since > 2.0:         # "jatuh"
                pitch = min(80.0, (t - self.walk_since - 2.0) * 160)
        if self.knobs is not None or self.lying:
            roll, pitch, gx = self.action_imu(t)
        if self.params.init_pitch_offset < TIP_BELOW:                       # pose berdiri terguling
            pitch = max(pitch, 1000 * (TIP_BELOW - self.params.init_pitch_offset))
        gz += 0.002                                                          # bias gyro
        r, p = math.radians(roll) / 2, math.radians(pitch) / 2
        q = Quaternion(w=math.cos(r) * math.cos(p), x=math.sin(r) * math.cos(p),
                       y=math.cos(r) * math.sin(p), z=-math.sin(r) * math.sin(p))
        msg = Imu(orientation=q)
        msg.angular_velocity.x, msg.angular_velocity.z = gx, gz
        self.imu_pub.publish(msg)

    def action_imu(self, t):
        """Lanskap page kerja: goyang naik kalau kenop jauh dari optimum; >12 deg = jatuh."""
        running = t < self.action_until
        knobs = self.knobs or {}
        if self.lying:
            if running and knobs:
                frac = (t - self.action_since) / max(self.action_until - self.action_since, 0.1)
                goal = 3.0 if abs(knobs["pitch_hip_deg"]) < 2.0 else 60.0
                return 0.0, 80.0 + (goal - 80.0) * min(1.0, frac), 0.0
            if knobs and t < self.action_until + 2.0:          # selesai: tetap di posisi akhir 2 s
                return 0.0, 3.0 if abs(knobs["pitch_hip_deg"]) < 2.0 else 60.0, 0.0
            return 0.0, 80.0, 0.0                               # dibaringkan (lagi) oleh "operator"
        if not running or not knobs:
            return 0.0, 0.0, 0.0
        sway = 2.0 + 3.0 * (knobs["pitch_ankle_deg"] - 1.0) ** 2 + 4.0 * (knobs["roll_ankle_deg"] + 0.5) ** 2
        w = 2 * math.pi
        pitch = min(80.0, (t - self.action_since - 0.5) * 160) if sway > 12 and t - self.action_since > 0.5 else 0.0
        return sway * math.sin(w * t), pitch, math.radians(sway) * w * math.cos(w * t)

    def tick_joints(self):
        swing = 0.15 * math.sin(2 * math.pi * time.monotonic() / max(self.params.period_time, 0.1)) if self.walking else 0
        self.js_pub.publish(JointState(name=["r_hip_pitch"], position=[0.79 + swing]))

    def tick_volt(self):
        self.status_pub.publish(StatusMsg(status_msg="Present Volt : 12.2V"))


def main():
    rclpy.init()
    rclpy.spin(FakeRobot())


if __name__ == "__main__":
    main()
