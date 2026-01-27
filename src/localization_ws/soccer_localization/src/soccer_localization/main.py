#!/usr/bin/env python3
import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import rclpy
from soccer_localization.field_lines_ukf_ros import FieldLinesUKFROS


def main(args=None):
    rclpy.init(args=args)
    
    node = FieldLinesUKFROS()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()