"""Layer 7 — Perception Node.

Converts YOLO BoundingBoxes → obstacle/landmark estimates in robot frame.
  /vision/yolo/detections → /perception/obstacles + /perception/landmarks

Obstacle bearing:  bearing = (x_center_norm - 0.5) * FOV_rad
Obstacle distance: estimated from bounding box height via pinhole model.
"""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from soccer_msgs.msg import BoundingBoxes

from .config_loader import load_rules

_ROBOT_HEIGHT_M = 0.50   # approximate standing height of OP3
_GOAL_HEIGHT_M = 1.20    # RoboCup KidSize goal height


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")

        rules_file = self.declare_parameter("rules_file", "").value
        rules = load_rules(rules_file)

        self._focal_px = float(
            self.declare_parameter("camera_focal_length_px",
                                   rules.get("camera_focal_length_px", 1109.0)).value
        )
        self._fov_deg = float(self.declare_parameter(
            "camera_fov_deg", rules.get("camera_fov_deg", 60.0)).value)
        self._fov_rad = math.radians(self._fov_deg)
        self._img_w = int(self.declare_parameter("image_width", 1280).value)
        self._img_h = int(self.declare_parameter("image_height", 720).value)
        self._min_bbox_area = float(self.declare_parameter(
            "min_bbox_area", rules.get("min_bbox_area", 100.0)).value)
        self._max_dist_m = float(self.declare_parameter("max_obstacle_dist_m", 4.0).value)

        self.create_subscription(
            BoundingBoxes, "/vision/yolo/detections", self._cb_detections, 10
        )
        self._obs_pub = self.create_publisher(String, "/perception/obstacles", 10)
        self._lm_pub = self.create_publisher(String, "/perception/landmarks", 10)

        self.get_logger().info(
            f"PerceptionNode ready (focal={self._focal_px:.0f}px, FOV={self._fov_deg}°)"
        )

    def _cb_detections(self, msg: BoundingBoxes) -> None:
        obstacles = []
        landmarks = []

        for bbox in msg.bounding_boxes:
            area = (bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin)
            if area < self._min_bbox_area:
                continue

            x_center = (bbox.xmin + bbox.xmax) / 2.0
            bbox_h = float(bbox.ymax - bbox.ymin)
            bearing = (x_center / self._img_w - 0.5) * self._fov_rad
            class_id = str(bbox.class_id).lower()

            if class_id == "robot":
                dist = self._dist_from_height(_ROBOT_HEIGHT_M, bbox_h)
                if dist <= self._max_dist_m:
                    ox = dist * math.cos(bearing)
                    oy = dist * math.sin(bearing)
                    obstacles.append({
                        "x": round(ox, 3),
                        "y": round(oy, 3),
                        "dist": round(dist, 3),
                        "bearing": round(bearing, 4),
                        "confidence": round(float(bbox.probability), 3),
                    })

            elif class_id in ("goal post", "goalpost"):
                dist = self._dist_from_height(_GOAL_HEIGHT_M, bbox_h)
                landmarks.append({
                    "type": "goal_post",
                    "dist": round(dist, 3),
                    "bearing": round(bearing, 4),
                    "confidence": round(float(bbox.probability), 3),
                })

            elif class_id in ("l-intersection", "t-intersection", "x-intersection"):
                landmarks.append({
                    "type": class_id.replace("-", "_"),
                    "bearing": round(bearing, 4),
                    "confidence": round(float(bbox.probability), 3),
                })

        self._obs_pub.publish(String(data=json.dumps(obstacles)))
        if landmarks:
            self._lm_pub.publish(String(data=json.dumps(landmarks)))

    def _dist_from_height(self, real_height_m: float, bbox_h_px: float) -> float:
        if bbox_h_px < 1.0:
            return self._max_dist_m
        return _clamp(
            (real_height_m * self._focal_px) / bbox_h_px,
            0.1, self._max_dist_m,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
