"""Layer 7 — Localization Node.

Fuses /odom + /robotis_op3/imu + /perception/landmarks into a field-frame
pose estimate using a lightweight complementary filter.

Outputs:
  /localization/pose        (PoseWithCovarianceStamped) — pose in field frame
  /localization/confidence  (Float32) — 0.0–1.0

Architecture note: a full UKF requires scipy which may not be installed.
This implementation uses dead-reckoning from odometry (reset-on-startup)
with IMU heading correction and optional landmark bearing correction.
Confidence decays over time without landmark updates.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

from .config_loader import load_rules


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class LocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_node")

        rules_file = self.declare_parameter("rules_file", "").value
        rules = load_rules(rules_file)

        self._publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)
        self._landmark_weight = float(
            self.declare_parameter("landmark_correction_weight", 0.15).value
        )
        self._confidence_decay_rate = float(
            self.declare_parameter(
                "confidence_decay_rate",
                rules.get("confidence_decay_rate", 0.01)).value
        )
        self._confidence_landmark_boost = float(
            self.declare_parameter(
                "confidence_landmark_boost",
                rules.get("confidence_landmark_boost", 0.30)).value
        )

        # Known goal post positions on the field (RoboCup KidSize 9×6m)
        # Left goal: x=-4.5, Right goal: x=+4.5
        self._goal_posts = [
            (-4.5,  1.3),   # left goal, top post
            (-4.5, -1.3),   # left goal, bottom post
            ( 4.5,  1.3),   # right goal, top post
            ( 4.5, -1.3),   # right goal, bottom post
        ]

        # State: pose in field frame
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._confidence = 0.0

        # Odometry tracking
        self._odom_x0: float | None = None
        self._odom_y0: float | None = None
        self._odom_yaw0: float | None = None
        self._prev_odom_x: float | None = None
        self._prev_odom_y: float | None = None
        self._prev_odom_yaw: float | None = None

        self._last_landmark_time = 0.0

        # IMU heading
        self._imu_yaw = 0.0
        self._imu_yaw0: float | None = None

        # Subscriptions
        self.create_subscription(Odometry, "/odom", self._cb_odom, 10)
        self.create_subscription(Imu, "/robotis_op3/imu", self._cb_imu, 10)
        self.create_subscription(String, "/perception/landmarks", self._cb_landmarks, 10)

        # Publishers
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/pose", 10
        )
        self._conf_pub = self.create_publisher(Float32, "/localization/confidence", 10)

        self.create_timer(1.0 / self._publish_rate, self._tick)
        self.get_logger().info("LocalizationNode ready (odom + IMU + landmark correction)")

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _cb_odom(self, msg: Odometry) -> None:
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        oy_angle = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if self._odom_x0 is None:
            self._odom_x0 = ox
            self._odom_y0 = oy
            self._odom_yaw0 = oy_angle
            self._prev_odom_x = ox
            self._prev_odom_y = oy
            self._prev_odom_yaw = oy_angle
            return

        # Dead-reckoning delta in field frame
        dx_odom = ox - self._prev_odom_x
        dy_odom = oy - self._prev_odom_y
        dyaw_odom = _wrap(oy_angle - self._prev_odom_yaw)

        # Rotate delta from odom frame to field frame
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        self._x += dx_odom * cos_yaw - dy_odom * sin_yaw
        self._y += dx_odom * sin_yaw + dy_odom * cos_yaw
        self._yaw = _wrap(self._yaw + dyaw_odom)

        self._prev_odom_x = ox
        self._prev_odom_y = oy
        self._prev_odom_yaw = oy_angle

        # Confidence increases with odometry updates (we're tracking)
        self._confidence = min(1.0, self._confidence + 0.01)

    def _cb_imu(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if self._imu_yaw0 is None:
            self._imu_yaw0 = yaw
        self._imu_yaw = _wrap(yaw - self._imu_yaw0)

        # Blend IMU heading into yaw (low-pass)
        yaw_err = _wrap(self._imu_yaw - self._yaw)
        self._yaw = _wrap(self._yaw + 0.1 * yaw_err)

    def _cb_landmarks(self, msg: String) -> None:
        try:
            landmarks = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        for lm in landmarks:
            if lm.get("type") != "goal_post":
                continue
            dist = float(lm.get("dist", 0.0))
            bearing = float(lm.get("bearing", 0.0))
            conf = float(lm.get("confidence", 0.0))
            if dist < 0.3 or conf < 0.3:
                continue

            # Find nearest known goal post consistent with observation
            obs_angle = _wrap(self._yaw + bearing)
            obs_x = self._x + dist * math.cos(obs_angle)
            obs_y = self._y + dist * math.sin(obs_angle)

            best_post = min(
                self._goal_posts,
                key=lambda p: math.hypot(p[0] - obs_x, p[1] - obs_y),
            )
            post_err = math.hypot(best_post[0] - obs_x, best_post[1] - obs_y)

            if post_err > 1.5:
                continue  # observation inconsistent with map

            # Correction: nudge pose toward what observation implies
            w = self._landmark_weight * conf
            self._x += w * (best_post[0] - obs_x)
            self._y += w * (best_post[1] - obs_y)

            self._confidence = min(1.0, self._confidence + self._confidence_landmark_boost)
            self._last_landmark_time = time.monotonic()

    # ------------------------------------------------------------------ #
    #  Publish                                                             #
    # ------------------------------------------------------------------ #

    def _tick(self) -> None:
        # Decay confidence when no landmark update for 2s.
        # Rate is per-tick (10Hz), so effective decay = rate * 10 per second.
        time_since_lm = time.monotonic() - self._last_landmark_time
        if time_since_lm > 2.0:
            self._confidence = max(0.0, self._confidence - self._confidence_decay_rate)

        # Publish pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "field"
        pose_msg.pose.pose.position.x = self._x
        pose_msg.pose.pose.position.y = self._y
        pose_msg.pose.pose.position.z = 0.0

        # Yaw → quaternion
        half_yaw = self._yaw / 2.0
        pose_msg.pose.pose.orientation.w = math.cos(half_yaw)
        pose_msg.pose.pose.orientation.z = math.sin(half_yaw)

        # Covariance diagonal ~ inverse confidence
        uncertainty = max(0.01, 1.0 - self._confidence) * 5.0
        for i in range(3):
            pose_msg.pose.covariance[i * 7] = uncertainty

        self._pose_pub.publish(pose_msg)
        self._conf_pub.publish(Float32(data=float(self._confidence)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
