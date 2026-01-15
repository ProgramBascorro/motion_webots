import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from soccer_localization.field import Field, Line, Circle


class FieldMarkersNode(Node):
    def __init__(self):
        super().__init__("field_markers")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_topic", "/field_map/markers")
        self.declare_parameter("line_width", 0.05)
        self.declare_parameter("circle_segments", 64)

        frame_id = self.get_parameter("frame_id").value
        marker_topic = self.get_parameter("marker_topic").value

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(MarkerArray, marker_topic, qos)

        self.frame_id = frame_id
        self.field = Field()

        self.timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()

        lines_marker = Marker()
        lines_marker.header.frame_id = self.frame_id
        lines_marker.header.stamp = self.get_clock().now().to_msg()
        lines_marker.ns = "field_lines"
        lines_marker.id = 0
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.scale.x = float(self.get_parameter("line_width").value)
        lines_marker.color.r = 1.0
        lines_marker.color.g = 1.0
        lines_marker.color.b = 1.0
        lines_marker.color.a = 1.0

        circle_marker = Marker()
        circle_marker.header.frame_id = self.frame_id
        circle_marker.header.stamp = lines_marker.header.stamp
        circle_marker.ns = "field_circle"
        circle_marker.id = 1
        circle_marker.type = Marker.LINE_STRIP
        circle_marker.action = Marker.ADD
        circle_marker.scale.x = lines_marker.scale.x
        circle_marker.color = lines_marker.color

        for element in self.field.lines:
            if isinstance(element, Line):
                p1 = Point(x=float(element.p1.x), y=float(element.p1.y), z=0.0)
                p2 = Point(x=float(element.p2.x), y=float(element.p2.y), z=0.0)
                lines_marker.points.append(p1)
                lines_marker.points.append(p2)
            elif isinstance(element, Circle):
                segments = int(self.get_parameter("circle_segments").value)
                for i in range(segments + 1):
                    angle = 2.0 * math.pi * i / segments
                    circle_marker.points.append(
                        Point(
                            x=float(element.center.x + math.cos(angle) * element.radius),
                            y=float(element.center.y + math.sin(angle) * element.radius),
                            z=0.0,
                        )
                    )

        marker_array.markers.append(lines_marker)
        marker_array.markers.append(circle_marker)

        self.publisher.publish(marker_array)


def main():
    rclpy.init()
    node = FieldMarkersNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
