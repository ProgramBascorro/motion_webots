#!/usr/bin/env python3
"""Ball estimator — converts YOLO detections to /ball/estimate Vector3Stamped.

Fix: distance AND bearing are now derived inside the same _detections_cb so they
always come from the same detection frame.  The separate ball_center PointStamped
callback is kept as an optional refinement but is not required for a valid estimate.
"""
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped
from soccer_msgs.msg import BoundingBoxes


class BallEstimator(Node):
    def __init__(self) -> None:
        super().__init__("op3_ball_estimator")

        self._detections_topic = str(
            self.declare_parameter("detections_topic", "/vision/yolo/detections").value)
        self._ball_center_topic = str(
            self.declare_parameter("ball_center_topic", "/vision/yolo/ball_center").value)
        self._output_topic = str(
            self.declare_parameter("output_topic", "/ball/estimate").value)
        self._publish_rate           = float(self.declare_parameter("publish_rate", 10.0).value)
        self._ball_real_diameter_m   = float(self.declare_parameter("ball_real_diameter_m", 0.065).value)
        self._camera_focal_length_px = float(self.declare_parameter("camera_focal_length_px", 1109.0).value)
        self._min_bbox_area          = float(self.declare_parameter("min_bbox_area", 100.0).value)
        self._lost_timeout           = float(self.declare_parameter("lost_timeout", 0.8).value)
        self._fov_deg                = float(self.declare_parameter("camera_fov_deg", 60.0).value)
        self._img_w                  = int(self.declare_parameter("image_width", 1280).value)

        self._last_ball_time: Optional[float] = None
        self._last_ball_distance: Optional[float] = None
        self._last_ball_bearing: Optional[float] = None

        self.create_subscription(
            BoundingBoxes, self._detections_topic, self._detections_cb, 10)
        # Optional: ball_center refines bearing but detections alone are sufficient
        self.create_subscription(
            PointStamped, self._ball_center_topic, self._ball_center_cb, 10)

        self._pub = self.create_publisher(Vector3Stamped, self._output_topic, 10)
        self.create_timer(1.0 / self._publish_rate, self._publish)

        self.get_logger().info("Ball estimator ready")

    def _detections_cb(self, msg: BoundingBoxes) -> None:
        best_ball = None
        best_area = 0.0
        for bbox in msg.bounding_boxes:
            if bbox.class_id != "ball":
                continue
            area = float((bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin))
            if area < self._min_bbox_area:
                continue
            if area > best_area:
                best_area = area
                best_ball = bbox

        if best_ball is None:
            return

        bbox_height = float(best_ball.ymax - best_ball.ymin)
        if bbox_height <= 0.0:
            return

        # Distance from pinhole model
        distance = (self._ball_real_diameter_m * self._camera_focal_length_px) / bbox_height
        distance = max(0.1, min(5.0, distance))

        # Bearing from bbox centre — same detection frame as distance (no mismatch)
        x_center_px = (best_ball.xmin + best_ball.xmax) / 2.0
        bearing = (x_center_px / self._img_w - 0.5) * math.radians(self._fov_deg)

        self._last_ball_distance = distance
        self._last_ball_bearing  = bearing
        self._last_ball_time     = time.monotonic()

    def _ball_center_cb(self, msg: PointStamped) -> None:
        """Optional: refine bearing from YOLO sub-pixel centre if available."""
        x_norm = msg.point.x
        if math.isnan(x_norm):
            return
        # Only update if detections-derived estimate is fresh
        if (self._last_ball_time is not None
                and (time.monotonic() - self._last_ball_time) < self._lost_timeout):
            self._last_ball_bearing = (x_norm - 0.5) * math.radians(self._fov_deg)

    def _publish(self) -> None:
        now = time.monotonic()
        visible = (
            self._last_ball_time is not None
            and (now - self._last_ball_time) < self._lost_timeout
            and self._last_ball_distance is not None
            and self._last_ball_bearing is not None
        )
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if visible:
            msg.vector.x = float(self._last_ball_distance)
            msg.vector.y = float(self._last_ball_bearing)
            msg.vector.z = 1.0
        else:
            msg.vector.x = 0.0
            msg.vector.y = 0.0
            msg.vector.z = 0.0
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = BallEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
