#!/usr/bin/env python3

import json
import math
import os
import re
import signal
import socket
import subprocess
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from robotis_controller_msgs.msg import StatusMsg
from sensor_msgs.msg import Image, Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def read_cpu_times() -> Optional[Tuple[int, int]]:
    try:
        with open("/proc/stat", "r", encoding="utf-8") as handle:
            parts = handle.readline().split()
    except OSError:
        return None
    if not parts or parts[0] != "cpu":
        return None
    values = [int(v) for v in parts[1:]]
    if len(values) < 4:
        return None
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    total = sum(values)
    return total, idle


def read_mem_info() -> Tuple[Optional[int], Optional[int]]:
    total_kb = None
    avail_kb = None
    try:
        with open("/proc/meminfo", "r", encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("MemTotal:"):
                    total_kb = int(line.split()[1])
                elif line.startswith("MemAvailable:"):
                    avail_kb = int(line.split()[1])
    except OSError:
        return None, None
    return total_kb, avail_kb


def read_net_dev() -> Dict[str, Tuple[int, int]]:
    stats = {}
    try:
        with open("/proc/net/dev", "r", encoding="utf-8") as handle:
            lines = handle.readlines()[2:]
    except OSError:
        return stats
    for line in lines:
        if ":" not in line:
            continue
        iface, data = line.split(":", 1)
        parts = data.split()
        if len(parts) < 9:
            continue
        rx_bytes = int(parts[0])
        tx_bytes = int(parts[8])
        stats[iface.strip()] = (rx_bytes, tx_bytes)
    return stats


def probe_latency(host: str, port: int, timeout: float) -> Optional[float]:
    if not host or port <= 0:
        return None
    start = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout):
            pass
    except OSError:
        return None
    return (time.monotonic() - start) * 1000.0


class StudioAgent(Node):
    def __init__(self) -> None:
        super().__init__("bascorro_studio_agent")

        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("battery_min_voltage", 10.5)
        self.declare_parameter("battery_max_voltage", 12.6)
        self.declare_parameter("battery_warn_voltage", 11.1)
        self.declare_parameter("battery_match_voltage", 11.4)
        self.declare_parameter("fall_pitch_front_deg", 60.0)
        self.declare_parameter("fall_pitch_back_deg", 60.0)
        self.declare_parameter("fall_roll_deg", 55.0)
        self.declare_parameter("fall_hold_sec", 0.3)
        self.declare_parameter("fall_cooldown_sec", 2.0)
        self.declare_parameter(
            "heartbeat_topics",
            [
                "/robotis/open_cr/imu",
                "/robotis/present_joint_states",
                "/robotis/status",
                "/vision/yolo/debug",
            ],
        )
        self.declare_parameter("heartbeat_timeout_sec", 1.0)
        self.declare_parameter("snapshot_dir", "/tmp/bascorro_studio/snapshots")
        self.declare_parameter("snapshot_image_topic", "/vision/yolo/debug")
        self.declare_parameter("bag_dir", "/tmp/bascorro_studio/bags")
        self.declare_parameter(
            "bag_topics",
            [
                "/robotis/open_cr/imu",
                "/robotis/present_joint_states",
                "/robotis/status",
                "/vision/yolo/debug",
            ],
        )
        self.declare_parameter("bag_prefix", "op3")
        self.declare_parameter("network_interface", "")
        self.declare_parameter("network_probe_host", "")
        self.declare_parameter("network_probe_port", 0)
        self.declare_parameter("network_probe_timeout_sec", 0.3)
        self.declare_parameter("network_probe_interval_sec", 5.0)
        self.declare_parameter("max_events", 200)

        self._battery_voltage = None
        self._last_roll = None
        self._last_pitch = None
        self._last_yaw = None
        self._last_imu_time = None
        self._joint_names: List[str] = []
        self._joint_effort: Dict[str, float] = {}
        self._fall_state = "upright"
        self._fall_candidate = None
        self._fall_candidate_since = 0.0
        self._last_fall_time = 0.0
        self._last_fall_reason = None
        self._battery_low = False
        self._events: List[Dict[str, object]] = []
        self._event_seq = 0
        self._heartbeat: Dict[str, float] = {}
        self._stale_topics: set = set()

        self._last_cpu = read_cpu_times()
        self._last_cpu_time = time.monotonic()
        self._last_net = {}
        self._last_net_time = time.monotonic()
        self._last_latency_ms = None
        self._last_latency_time = 0.0
        self._bag_proc: Optional[subprocess.Popen] = None
        self._bag_log_path: Optional[str] = None

        self._last_image: Optional[Image] = None
        self.metrics_pub = self.create_publisher(String, "/bascorro_studio/metrics", 10)
        self.events_pub = self.create_publisher(String, "/bascorro_studio/events", 10)

        self.create_subscription(Imu, "/robotis/open_cr/imu", self._imu_cb, 10)
        self.create_subscription(StatusMsg, "/robotis/status", self._status_cb, 10)
        self.create_subscription(
            JointState, "/robotis/present_joint_states", self._joint_cb, 10
        )

        snapshot_topic = self.get_parameter("snapshot_image_topic").value
        if snapshot_topic:
            self.create_subscription(Image, snapshot_topic, self._image_cb, 10)

        self.create_service(Trigger, "/bascorro_studio/snapshot", self._handle_snapshot)
        self.create_service(Trigger, "/bascorro_studio/bag_start", self._handle_bag_start)
        self.create_service(Trigger, "/bascorro_studio/bag_stop", self._handle_bag_stop)

        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            publish_rate = 5.0
        self.create_timer(1.0 / publish_rate, self._publish_metrics)

    def _mark_heartbeat(self, topic: str) -> None:
        self._heartbeat[topic] = time.monotonic()

    def _imu_cb(self, msg: Imu) -> None:
        self._mark_heartbeat("/robotis/open_cr/imu")
        self._last_imu_time = time.monotonic()
        roll, pitch, yaw = quat_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self._last_roll = roll
        self._last_pitch = pitch
        self._last_yaw = yaw
        self._check_fall()

    def _status_cb(self, msg: StatusMsg) -> None:
        self._mark_heartbeat("/robotis/status")
        match = re.search(r"Present Volt\\s*:\\s*([0-9.]+)", msg.status_msg)
        if not match:
            return
        try:
            self._battery_voltage = float(match.group(1))
        except ValueError:
            return

    def _joint_cb(self, msg: JointState) -> None:
        self._mark_heartbeat("/robotis/present_joint_states")
        self._joint_names = list(msg.name)
        self._joint_effort = {
            name: float(msg.effort[idx]) if idx < len(msg.effort) else 0.0
            for idx, name in enumerate(msg.name)
        }

    def _image_cb(self, msg: Image) -> None:
        topic = str(self.get_parameter("snapshot_image_topic").value)
        if topic:
            self._mark_heartbeat(topic)
        self._last_image = msg

    def _check_fall(self) -> None:
        if self._last_pitch is None or self._last_roll is None:
            return
        now = time.monotonic()
        pitch_deg = math.degrees(self._last_pitch)
        roll_deg = math.degrees(self._last_roll)
        front_thresh = float(self.get_parameter("fall_pitch_front_deg").value)
        back_thresh = float(self.get_parameter("fall_pitch_back_deg").value)
        roll_thresh = float(self.get_parameter("fall_roll_deg").value)

        candidate = None
        if pitch_deg > front_thresh:
            candidate = "front"
        elif pitch_deg < -back_thresh:
            candidate = "back"
        elif abs(roll_deg) > roll_thresh:
            candidate = "side"

        if candidate != self._fall_candidate:
            self._fall_candidate = candidate
            self._fall_candidate_since = now

        if candidate is None:
            if self._fall_state != "upright":
                self._fall_state = "upright"
                self._record_event("recover", "Recovered upright")
            return

        hold_sec = float(self.get_parameter("fall_hold_sec").value)
        cooldown = float(self.get_parameter("fall_cooldown_sec").value)
        if now - self._fall_candidate_since < hold_sec:
            return
        if now - self._last_fall_time < cooldown:
            return

        self._fall_state = candidate
        self._last_fall_time = now
        self._last_fall_reason = candidate
        self._record_event("fall", f"Fall detected: {candidate}")

    def _record_event(self, event_type: str, message: str) -> None:
        self._event_seq += 1
        event = {
            "id": self._event_seq,
            "ts": time.time(),
            "type": event_type,
            "message": message,
        }
        self._events.append(event)
        max_events = int(self.get_parameter("max_events").value)
        if len(self._events) > max_events:
            self._events = self._events[-max_events:]
        self._publish_events()

    def _publish_events(self) -> None:
        payload = {"events": self._events}
        msg = String()
        msg.data = json.dumps(payload)
        self.events_pub.publish(msg)

    def _build_system_metrics(self) -> Dict[str, Optional[float]]:
        cpu_percent = None
        load_avg = None
        cpu_times = read_cpu_times()
        now = time.monotonic()
        if cpu_times and self._last_cpu:
            total, idle = cpu_times
            last_total, last_idle = self._last_cpu
            total_delta = total - last_total
            idle_delta = idle - last_idle
            if total_delta > 0:
                cpu_percent = (1.0 - (idle_delta / total_delta)) * 100.0
            self._last_cpu = cpu_times
            self._last_cpu_time = now
        else:
            self._last_cpu = cpu_times
            self._last_cpu_time = now

        try:
            load_avg = os.getloadavg()[0]
        except OSError:
            load_avg = None

        total_kb, avail_kb = read_mem_info()
        mem_total_mb = total_kb / 1024.0 if total_kb else None
        mem_used_mb = None
        mem_percent = None
        if total_kb and avail_kb is not None:
            mem_used_mb = (total_kb - avail_kb) / 1024.0
            mem_percent = ((total_kb - avail_kb) / total_kb) * 100.0

        disk_total_gb = None
        disk_used_gb = None
        disk_percent = None
        try:
            disk_path = str(self.get_parameter("bag_dir").value)
            stat = os.statvfs(disk_path)
            total = stat.f_blocks * stat.f_frsize
            free = stat.f_bfree * stat.f_frsize
            used = total - free
            disk_total_gb = total / (1024.0 ** 3)
            disk_used_gb = used / (1024.0 ** 3)
            disk_percent = (used / total) * 100.0 if total > 0 else None
        except OSError:
            pass

        return {
            "cpu_percent": cpu_percent,
            "load": load_avg,
            "mem_total_mb": mem_total_mb,
            "mem_used_mb": mem_used_mb,
            "mem_percent": mem_percent,
            "disk_total_gb": disk_total_gb,
            "disk_used_gb": disk_used_gb,
            "disk_percent": disk_percent,
        }

    def _build_network_metrics(self) -> Dict[str, Optional[float]]:
        interface = str(self.get_parameter("network_interface").value).strip()
        stats = read_net_dev()
        if not interface:
            for name in stats.keys():
                if name != "lo":
                    interface = name
                    break
        if interface not in stats:
            interface = next(iter(stats.keys()), "")

        rx_kbps = None
        tx_kbps = None
        now = time.monotonic()
        if interface and interface in stats:
            rx_bytes, tx_bytes = stats[interface]
            last = self._last_net.get(interface)
            dt = now - self._last_net_time
            if last and dt > 0:
                rx_kbps = (rx_bytes - last[0]) / 1024.0 / dt
                tx_kbps = (tx_bytes - last[1]) / 1024.0 / dt
            self._last_net[interface] = (rx_bytes, tx_bytes)
            self._last_net_time = now

        probe_host = str(self.get_parameter("network_probe_host").value).strip()
        probe_port = int(self.get_parameter("network_probe_port").value)
        probe_timeout = float(self.get_parameter("network_probe_timeout_sec").value)
        probe_interval = float(self.get_parameter("network_probe_interval_sec").value)
        if probe_host and probe_port > 0:
            if now - self._last_latency_time >= probe_interval:
                self._last_latency_time = now
                self._last_latency_ms = probe_latency(probe_host, probe_port, probe_timeout)

        return {
            "interface": interface,
            "rx_kbps": rx_kbps,
            "tx_kbps": tx_kbps,
            "latency_ms": self._last_latency_ms,
        }

    def _build_battery_metrics(self) -> Dict[str, Optional[float]]:
        min_v = float(self.get_parameter("battery_min_voltage").value)
        max_v = float(self.get_parameter("battery_max_voltage").value)
        warn_v = float(self.get_parameter("battery_warn_voltage").value)
        match_v = float(self.get_parameter("battery_match_voltage").value)
        percent = None
        if self._battery_voltage is not None:
            percent = clamp(
                (self._battery_voltage - min_v) / (max_v - min_v) * 100.0,
                0.0,
                100.0,
            )
            if self._battery_voltage <= warn_v and not self._battery_low:
                self._battery_low = True
                self._record_event(
                    "battery_low",
                    f"Battery low: {self._battery_voltage:.2f}V",
                )
            if self._battery_low and self._battery_voltage > warn_v + 0.2:
                self._battery_low = False
        return {
            "voltage": self._battery_voltage,
            "percent": percent,
            "warn_voltage": warn_v,
            "match_voltage": match_v,
        }

    def _build_torque_metrics(self) -> Dict[str, object]:
        values = list(self._joint_effort.values())
        max_val = max((abs(v) for v in values), default=None)
        avg_val = sum(abs(v) for v in values) / len(values) if values else None
        return {
            "max": max_val,
            "avg": avg_val,
            "joints": self._joint_effort,
        }

    def _build_heartbeat(self) -> Dict[str, object]:
        now = time.monotonic()
        topics = {}
        stale = []
        timeout = float(self.get_parameter("heartbeat_timeout_sec").value)
        for topic in self.get_parameter("heartbeat_topics").value:
            last = self._heartbeat.get(topic)
            age = None
            if last is not None:
                age = now - last
            topics[topic] = age
            if age is not None and age > timeout:
                stale.append(topic)
        for topic in stale:
            if topic not in self._stale_topics:
                self._stale_topics.add(topic)
                self._record_event("stale", f"Topic stale: {topic}")
        for topic in list(self._stale_topics):
            if topic not in stale:
                self._stale_topics.discard(topic)
        return {"topics": topics, "stale": stale}

    def _publish_metrics(self) -> None:
        battery = self._build_battery_metrics()
        system = self._build_system_metrics()
        network = self._build_network_metrics()
        torque = self._build_torque_metrics()
        heartbeat = self._build_heartbeat()

        imu = None
        if self._last_roll is not None and self._last_pitch is not None:
            imu = {
                "roll": math.degrees(self._last_roll),
                "pitch": math.degrees(self._last_pitch),
                "yaw": math.degrees(self._last_yaw)
                if self._last_yaw is not None
                else None,
            }

        payload = {
            "timestamp": time.time(),
            "battery": battery,
            "imu": imu,
            "fall": {
                "state": self._fall_state,
                "last_reason": self._last_fall_reason,
                "last_time": self._last_fall_time,
            },
            "torque": torque,
            "system": system,
            "network": network,
            "heartbeat": heartbeat,
            "joint_names": self._joint_names,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.metrics_pub.publish(msg)

    def _handle_snapshot(self, request, response):  # noqa: ARG002
        if self._last_image is None:
            response.success = False
            response.message = "No image available"
            return response

        snapshot_dir = str(self.get_parameter("snapshot_dir").value)
        os.makedirs(snapshot_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(snapshot_dir, f"snapshot_{stamp}")
        image_path = self._write_image(self._last_image, base)
        meta_path = base + ".json"

        meta = {
            "timestamp": time.time(),
            "image": os.path.basename(image_path) if image_path else None,
            "metrics": {
                "battery_voltage": self._battery_voltage,
                "fall_state": self._fall_state,
            },
        }
        with open(meta_path, "w", encoding="utf-8") as handle:
            json.dump(meta, handle, indent=2)

        self._record_event("snapshot", f"Snapshot saved: {base}")
        response.success = True
        response.message = base
        return response

    def _write_image(self, msg: Image, base: str) -> Optional[str]:
        encoding = msg.encoding.lower()
        width = int(msg.width)
        height = int(msg.height)
        data = bytes(msg.data)
        if encoding in ("rgb8", "bgr8"):
            if encoding == "bgr8":
                rgb = bytearray(len(data))
                for i in range(0, len(data), 3):
                    rgb[i] = data[i + 2]
                    rgb[i + 1] = data[i + 1]
                    rgb[i + 2] = data[i]
                data = bytes(rgb)
            path = base + ".ppm"
            header = f"P6\n{width} {height}\n255\n".encode("ascii")
            with open(path, "wb") as handle:
                handle.write(header)
                handle.write(data)
            return path
        if encoding == "mono8":
            path = base + ".pgm"
            header = f"P5\n{width} {height}\n255\n".encode("ascii")
            with open(path, "wb") as handle:
                handle.write(header)
                handle.write(data)
            return path
        path = base + ".bin"
        with open(path, "wb") as handle:
            handle.write(data)
        return path

    def _handle_bag_start(self, request, response):  # noqa: ARG002
        if self._bag_proc and self._bag_proc.poll() is None:
            response.success = False
            response.message = "Bag already running"
            return response

        bag_dir = str(self.get_parameter("bag_dir").value)
        os.makedirs(bag_dir, exist_ok=True)
        prefix = str(self.get_parameter("bag_prefix").value)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        output = os.path.join(bag_dir, f"{prefix}_{stamp}")
        topics = list(self.get_parameter("bag_topics").value)
        if not topics:
            response.success = False
            response.message = "No bag topics configured"
            return response

        log_path = output + ".log"
        try:
            with open(log_path, "wb") as log_handle:
                self._bag_proc = subprocess.Popen(
                    ["ros2", "bag", "record", "--output", output, *topics],
                    stdout=log_handle,
                    stderr=log_handle,
                )
            self._bag_log_path = log_path
        except OSError as exc:
            response.success = False
            response.message = f"Bag start failed: {exc}"
            return response

        self._record_event("bag_start", f"Bag recording: {output}")
        response.success = True
        response.message = output
        return response

    def _handle_bag_stop(self, request, response):  # noqa: ARG002
        if not self._bag_proc or self._bag_proc.poll() is not None:
            response.success = False
            response.message = "Bag not running"
            return response
        self._bag_proc.send_signal(signal.SIGINT)
        try:
            self._bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._bag_proc.kill()
        self._bag_proc = None
        self._record_event("bag_stop", "Bag recording stopped")
        response.success = True
        response.message = "Bag stopped"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StudioAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._bag_proc and node._bag_proc.poll() is None:
            node._bag_proc.send_signal(signal.SIGINT)
            try:
                node._bag_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                node._bag_proc.kill()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
