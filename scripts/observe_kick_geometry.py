#!/usr/bin/env python3
"""Record camera and robot telemetry for kick calibration without motion commands."""

import argparse
from bisect import bisect_left
import copy
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import time


PARAMETER_NAMES = (
    "kick_camera_height", "kick_distance", "kick_ball_real_radius_m",
    "kick_camera_pitch_offset_deg",
    "kick_head_tilt_sign", "kick_min_depression_deg",
    "head_pan_servo_center_deg", "head_pan_servo_dir",
    "kick_pan_window_from_bearing", "kick_pan_right_min_deg",
    "kick_pan_right_max_deg", "kick_pan_left_min_deg", "kick_pan_left_max_deg",
    "kick_yaw_target_deg", "kick_yaw_tolerance_deg", "kick_pan_max_distance",
    "kick_distance_from_radius", "kick_radius_ready_enable",
)


def ball_geometry(circle, head, hip_pitch, parameters, camera_height=None):
    normalized_x, normalized_y, radius_px = circle
    pan, tilt = head["pan"], head["tilt"]
    if not all(math.isfinite(value) for value in
               (normalized_x, normalized_y, radius_px, pan, tilt, hip_pitch)):
        return None
    if radius_px <= 0.0:
        return None
    image_left = -normalized_x * math.tan(math.radians(35.2))
    image_up = -normalized_y * math.tan(math.radians(21.6))
    camera_pitch_offset = parameters.get("kick_camera_pitch_offset_deg", 0.0)
    if not math.isfinite(camera_pitch_offset) or abs(camera_pitch_offset) > 60.0:
        return None
    camera_depression = (hip_pitch - parameters["kick_head_tilt_sign"] * tilt +
                         math.radians(camera_pitch_offset))
    pitched_forward = math.cos(camera_depression) + image_up * math.sin(camera_depression)
    pitched_up = -math.sin(camera_depression) + image_up * math.cos(camera_depression)
    body_forward = pitched_forward * math.cos(pan) - image_left * math.sin(pan)
    body_left = pitched_forward * math.sin(pan) + image_left * math.cos(pan)
    bearing = math.atan2(body_left, body_forward)
    depression = -math.atan2(pitched_up, math.hypot(body_forward, body_left))
    height = parameters["kick_camera_height"] if camera_height is None else camera_height
    ball_radius = parameters["kick_ball_real_radius_m"]
    valid = (math.isfinite(height) and math.isfinite(ball_radius) and
             height > ball_radius > 0.0 and
             parameters["kick_min_depression_deg"] <= math.degrees(depression) < 89.0)
    distance = (height - ball_radius) / math.tan(depression) if valid else None
    pan_for_window = bearing if parameters["kick_pan_window_from_bearing"] > 0.5 else pan
    servo = parameters["head_pan_servo_center_deg"] + (
        parameters["head_pan_servo_dir"] * math.degrees(pan_for_window))
    foot = "outside"
    for side in ("right", "left"):
        if parameters[f"kick_pan_{side}_min_deg"] <= servo <= parameters[f"kick_pan_{side}_max_deg"]:
            foot = side
            break
    return {"distance_m": distance, "depression_deg": math.degrees(depression),
            "bearing_deg": math.degrees(bearing), "servo_window_deg": servo,
            "foot_window": foot}


def describe(values):
    ordered = sorted(value for value in values if value is not None and math.isfinite(value))
    if not ordered:
        return {"count": 0}
    return {"count": len(ordered), "median": statistics.median(ordered),
            "min": ordered[0], "max": ordered[-1],
            "p10": ordered[int((len(ordered) - 1) * 0.1)],
            "p90": ordered[int((len(ordered) - 1) * 0.9)]}


def nearest_head(records, stamps, target, max_skew_sec=0.1):
    insertion = bisect_left(stamps, target)
    candidates = records[max(0, insertion - 1):insertion + 1]
    if not candidates:
        return None
    closest = min(candidates, key=lambda record: abs(record["stamp"] - target))
    return closest if abs(closest["stamp"] - target) <= max_skew_sec else None


def summarize(recording):
    parameters = recording["parameters"]
    hip_pitch = recording["walking"]["hip_pitch_offset"]
    frames = recording["frames"]
    summary = {"frame_count": len(frames),
               "frames_with_ball": sum(bool(frame["circles"]) for frame in frames),
               "detector_age_ms": describe(1000.0 * (frame["received"] - frame["stamp"])
                                            for frame in frames)}
    for source in ("goal", "present"):
        heads = sorted(recording[source], key=lambda record: record["stamp"])
        stamps = [head["stamp"] for head in heads]
        observations = []
        for frame in frames:
            head = nearest_head(heads, stamps, frame["stamp"])
            if not head or not frame["circles"]:
                continue
            circle = max(frame["circles"], key=lambda candidate: candidate[2])
            geometry = ball_geometry(circle, head, hip_pitch, parameters)
            if geometry is not None:
                observations.append({"stamp": frame["stamp"], "head_pan_deg": math.degrees(head["pan"]),
                                     "head_tilt_deg": math.degrees(head["tilt"]),
                                     "radius_px": circle[2], **geometry})
        recording[f"{source}_observations"] = observations
        summary[source] = {field: describe(observation[field] for observation in observations)
                           for field in ("distance_m", "depression_deg", "bearing_deg",
                                         "servo_window_deg", "head_pan_deg", "head_tilt_deg", "radius_px")}
        summary[source]["foot_windows"] = {
            side: sum(observation["foot_window"] == side for observation in observations)
            for side in ("right", "left", "outside")}
    summary["imu_yaw_deg"] = describe(sample["yaw_deg"] for sample in recording["imu"])
    return summary


def fit_camera_pitch(circle, head, hip_pitch, parameters, distance):
    normalized_x, normalized_y, radius_px = circle
    values = (normalized_x, normalized_y, radius_px, head["pan"], head["tilt"],
              hip_pitch, distance, parameters["kick_head_tilt_sign"])
    if not all(math.isfinite(value) for value in values) or radius_px <= 0.0 or distance <= 0.0:
        return None
    image_left = -normalized_x * math.tan(math.radians(35.2))
    image_up = -normalized_y * math.tan(math.radians(21.6))
    drop = parameters["kick_camera_height"] - parameters["kick_ball_real_radius_m"]
    target_depression = math.atan2(drop, distance)
    if not parameters["kick_min_depression_deg"] <= math.degrees(target_depression) < 89.0:
        return None
    sine = math.sqrt(1.0 + image_left ** 2 + image_up ** 2) / math.hypot(1.0, image_up)
    sine *= math.sin(target_depression)
    if not -1.0 <= sine <= 1.0:
        return None
    pitch = math.asin(sine) + math.atan(image_up)
    offset = math.degrees(pitch - hip_pitch + parameters["kick_head_tilt_sign"] * head["tilt"])
    return offset if math.isfinite(offset) and abs(offset) <= 60.0 else None


def calibrate_recording(recording, camera_height, ball_diameter, floor_contact_slant):
    """Fit an effective pitch offset to a stationary, manually measured ball position."""
    measurements = (camera_height, ball_diameter, floor_contact_slant)
    if not all(math.isfinite(value) and value > 0.0 for value in measurements):
        raise ValueError("Manual measurements must be finite and positive, in meters")
    ball_radius = ball_diameter / 2.0
    if camera_height <= ball_radius or floor_contact_slant <= camera_height:
        raise ValueError("Height must exceed ball radius; floor-contact slant must exceed height")
    distance = math.sqrt((floor_contact_slant - camera_height) *
                         (floor_contact_slant + camera_height))
    calibrated = copy.deepcopy(recording)
    parameters = calibrated["parameters"]
    parameters.update(kick_camera_height=camera_height, kick_ball_real_radius_m=ball_radius)
    hip_pitch = calibrated["walking"]["hip_pitch_offset"]
    heads = sorted(calibrated["goal"], key=lambda record: record["stamp"])
    stamps = [head["stamp"] for head in heads]
    offsets = []
    for frame in calibrated["frames"]:
        head = nearest_head(heads, stamps, frame["stamp"])
        if head is None or not frame["circles"]:
            continue
        circle = max(frame["circles"], key=lambda candidate: candidate[2])
        offset = fit_camera_pitch(circle, head, hip_pitch, parameters, distance)
        if offset is not None:
            offsets.append(offset)
    if len(offsets) < 3:
        raise ValueError("At least three valid ball/head pairs are required for calibration")
    pitch_offset = round(statistics.median(offsets), 2)
    parameters["kick_camera_pitch_offset_deg"] = pitch_offset
    return {
        "reference": {
            "camera_height_m": camera_height, "ball_diameter_m": ball_diameter,
            "camera_to_floor_contact_m": floor_contact_slant,
            "horizontal_distance_m": distance,
            "predicted_camera_to_nearest_surface_m": math.hypot(camera_height - ball_radius, distance) - ball_radius,
        },
        "geometry_parameters": {name: parameters[name] for name in
                                ("kick_camera_height", "kick_ball_real_radius_m", "kick_camera_pitch_offset_deg")},
        "pitch_offset_fit_deg": describe(offsets),
        "replay_summary": summarize(calibrated),
    }


def capture(seconds, output_parent):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState
    from std_msgs.msg import UInt8
    from op3_ball_detector_msgs.msg import CircleSetStamped
    from op3_walking_module_msgs.srv import GetWalkingParam

    class Observer(Node):
        def __init__(self):
            super().__init__("observe_kick_geometry", start_parameter_services=False, enable_rosout=False)
            self.recording = {"goal": [], "present": [], "frames": [], "imu": [], "mag_status": []}
            self.active = False
            self.images = {}
            self.image_count = 0
            self.image_stamps = []
            self.create_subscription(JointState, "/robotis/goal_joint_states",
                                     lambda message: self.on_head("goal", message), qos_profile_sensor_data)
            self.create_subscription(JointState, "/robotis/present_joint_states",
                                     lambda message: self.on_head("present", message), qos_profile_sensor_data)
            self.create_subscription(CircleSetStamped, "/ball_detector_node/circle_set",
                                     self.on_ball, qos_profile_sensor_data)
            self.create_subscription(CompressedImage, "/usb_cam_node/image_raw/compressed",
                                     self.on_image, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, "/usb_cam_node/camera_info",
                                     self.on_camera_info, qos_profile_sensor_data)
            self.create_subscription(Imu, "/robotis/open_cr/imu", self.on_imu, qos_profile_sensor_data)
            self.create_subscription(UInt8, "/robotis/open_cr/mag_status",
                                     lambda message: self.recording["mag_status"].append(message.data),
                                     qos_profile_sensor_data)

        def times(self, message):
            received = self.get_clock().now().nanoseconds / 1e9
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            return {"stamp": stamp or received, "received": received, "header_stamp_valid": stamp > 0.0}

        def on_head(self, source, message):
            positions = dict(zip(message.name, message.position))
            if "head_pan" in positions and "head_tilt" in positions:
                self.recording[source].append({**self.times(message), "pan": positions["head_pan"],
                                               "tilt": positions["head_tilt"],
                                               "unique_joint_values": len(set(positions.values()))})

        def on_ball(self, message):
            if self.active:
                circles = [[circle.x, circle.y, circle.z] for circle in message.circles
                           if all(math.isfinite(value) for value in (circle.x, circle.y, circle.z))
                           and circle.z > 0.0]
                self.recording["frames"].append({**self.times(message), "circles": circles})

        def on_image(self, message):
            if not self.active:
                return
            self.image_count += 1
            self.image_stamps.append(self.times(message))
            data = bytes(message.data)
            self.images.setdefault("first", (data, self.times(message)))
            if time.monotonic() >= self.midpoint and "middle" not in self.images:
                self.images["middle"] = (data, self.times(message))
            self.images["last"] = (data, self.times(message))

        def on_camera_info(self, message):
            self.recording["camera_info"] = dict(message_to_ordereddict(message))

        def on_imu(self, message):
            orientation = message.orientation
            yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                             1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
            self.recording["imu"].append({**self.times(message), "yaw_deg": math.degrees(yaw),
                                          "gyro_z": message.angular_velocity.z})

        def read_settings(self):
            parameter_client = self.create_client(GetParameters, "/demo_node/get_parameters")
            walking_client = self.create_client(GetWalkingParam, "/robotis/walking/get_params")
            for client in (parameter_client, walking_client):
                if not client.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError(f"Read-only service unavailable: {client.srv_name}")
            parameter_future = parameter_client.call_async(GetParameters.Request(names=list(PARAMETER_NAMES)))
            walking_future = walking_client.call_async(GetWalkingParam.Request(get_param=True))
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and not (parameter_future.done() and walking_future.done()):
                rclpy.spin_once(self, timeout_sec=0.05)
            if not (parameter_future.done() and walking_future.done()):
                raise RuntimeError("Read-only parameter request timed out")
            parameters = {}
            for name, value in zip(PARAMETER_NAMES, parameter_future.result().values):
                if name == "kick_camera_pitch_offset_deg" and value.type == 0:
                    self.recording.setdefault("unavailable_parameters", []).append(name)
                    continue
                if value.type not in (2, 3):
                    raise RuntimeError(f"Numeric runtime parameter unavailable: {name}")
                parameters[name] = value.double_value if value.type == 3 else value.integer_value
            self.recording["parameters"] = parameters
            self.recording["walking"] = dict(message_to_ordereddict(walking_future.result().parameters))

    rclpy.init()
    node = Observer()
    try:
        node.read_settings()
        node.midpoint = time.monotonic() + seconds / 2.0
        deadline = time.monotonic() + seconds
        node.active = True
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.active = False
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.05)
        output_parent.mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(prefix="capture-", dir=str(output_parent)))
        node.recording["image_count"] = node.image_count
        node.recording["image_timestamps"] = node.image_stamps
        node.recording["images"] = {}
        for label, (data, timing) in node.images.items():
            extension = ".png" if data.startswith(b"\x89PNG") else ".jpg"
            filename = label + extension
            (output / filename).write_bytes(data)
            node.recording["images"][filename] = timing
        node.recording["summary"] = summarize(node.recording)
        (output / "measurement.json").write_text(json.dumps(node.recording, indent=2), encoding="utf-8")
        if os.geteuid() == 0:
            owner = Path(__file__).resolve().parents[1].stat()
            for artifact in [output, *output.iterdir()]:
                os.chown(artifact, owner.st_uid, owner.st_gid)
        print(json.dumps({"output": str(output), "parameters": node.recording["parameters"],
                          "summary": node.recording["summary"]}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--output-parent", type=Path, default=Path("log/kick_geometry"))
    parser.add_argument("--replay", type=Path, help="Fit a recorded JSON file offline, without ROS or robot commands")
    parser.add_argument("--camera-height", type=float, help="Vertical optical-center height above floor, meters")
    parser.add_argument("--ball-diameter", type=float, help="Physical ball diameter, meters")
    parser.add_argument("--floor-contact-slant", type=float, help="Straight-line camera-to-ball/floor contact distance, meters")
    args = parser.parse_args()
    measurements = (args.camera_height, args.ball_diameter, args.floor_contact_slant)
    if args.replay is not None:
        if any(value is None for value in measurements):
            parser.error("--replay requires --camera-height, --ball-diameter and --floor-contact-slant")
        try:
            recording = json.loads(args.replay.read_text(encoding="utf-8"))
            result = calibrate_recording(recording, *measurements)
        except (OSError, ValueError, KeyError) as error:
            parser.error(str(error))
        print(json.dumps(result, indent=2, allow_nan=False))
        return
    if any(value is not None for value in measurements):
        parser.error("Manual measurements are only accepted with --replay")
    if not math.isfinite(args.seconds) or not 1.0 <= args.seconds <= 60.0:
        parser.error("--seconds must be between 1 and 60")
    capture(args.seconds, args.output_parent)


if __name__ == "__main__":
    main()
