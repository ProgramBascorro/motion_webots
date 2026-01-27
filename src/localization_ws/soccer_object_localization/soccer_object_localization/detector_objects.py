# #!/usr/bin/env python3
# """
# Object Detector for Soccer - Migrated to ROS2 Humble
# Detects balls and obstacles from bounding boxes
# Note: This is currently not used as a standalone node, but as a helper class
# """

# import os
# import time
# import numpy as np

# np.set_printoptions(precision=3)

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose2D
# from sensor_msgs.msg import JointState

# # Import custom modules
# from soccer_object_localization.detector import Detector
# from soccer_common.transformation import Transformation
# from soccer_msgs.msg import BoundingBox, BoundingBoxes


# class DetectorObjects(Detector):
#     """
#     Detector for balls and obstacles from object detection bounding boxes
#     This class processes bounding boxes and calculates object positions
#     """
    
#     def __init__(self, node=None):
#         """
#         Initialize object detector
        
#         Args:
#             node: ROS2 node instance for logging (optional)
#         """
#         super().__init__()
        
#         self.node = node
        
#         # Ball tracking state
#         self.last_ball_pose = None
#         self.last_ball_pose_counter = 0
#         self.candidate_ball_counter = 0
#         self.max_detection_size = 0
#         self.final_camera_to_ball: Transformation = None
        
#         # Detection thresholds (can be parameters in node)
#         self.first_ball_confidence_threshold = 0.78
#         self.robot_confidence_threshold = 0.78
#         self.max_ball_distance_threshold = 3.0  # meters
        
#         if self.node:
#             self.node.get_logger().info("Object detector initialized")
    
#     def log_info(self, msg):
#         """Helper for logging"""
#         if self.node:
#             self.node.get_logger().info(msg)
#         else:
#             print(f"[INFO] {msg}")
    
#     def log_warn(self, msg):
#         """Helper for logging warnings"""
#         if self.node:
#             self.node.get_logger().warn(msg, throttle_duration_sec=1.0)
#         else:
#             print(f"[WARN] {msg}")
    
#     def detect_ball(self, box: BoundingBox) -> bool:
#         """
#         Process a ball detection from bounding box
        
#         Args:
#             box: BoundingBox message containing ball detection
            
#         Returns:
#             True if detection should be excluded, False if valid
#         """
#         # Exclude weirdly shaped balls
#         width = box.xmax - box.xmin
#         height = box.ymax - box.ymin
        
#         if width <= 0 or height <= 0:
#             return True
        
#         ratio = height / width
#         if ratio > 2.0 or ratio < 0.5:
#             self.log_warn(
#                 f"Excluding weirdly shaped ball: {height:.1f}x{width:.1f}, ratio={ratio:.2f}"
#             )
#             return True
        
#         # Calculate ball position from bounding box
#         boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
        
#         try:
#             # Assuming ball diameter of 0.07m (regulation soccer ball is ~0.22m diameter)
#             # This might need adjustment based on robot league rules
#             ball_pose = self.camera.calculate_ball_from_bounding_boxes(
#                 ball_diameter=0.07,
#                 bounding_boxes=boundingBoxes
#             )
#         except Exception as e:
#             self.log_warn(f"Failed to calculate ball position: {e}")
#             return True
        
#         # Calculate camera-relative position
#         try:
#             camera_to_ball = np.linalg.inv(self.camera.pose.matrix) @ ball_pose.matrix
#             camera_to_ball_transform = Transformation(matrix=camera_to_ball)
#         except Exception as e:
#             self.log_warn(f"Failed to calculate camera transform: {e}")
#             return True
        
#         detection_size = height * width
        
#         self.candidate_ball_counter += 1
        
#         # Exclude balls outside the field boundaries
#         # Field dimensions: ~5.2m x 3.5m (approximate, adjust as needed)
#         if abs(ball_pose.position[0]) > 5.2 or abs(ball_pose.position[1]) > 3.5:
#             self.log_warn(
#                 f"Ball outside field: [{ball_pose.position[0]:.2f}, {ball_pose.position[1]:.2f}]"
#             )
#             return True
        
#         # First detection requires high confidence
#         if self.last_ball_pose is None:
#             if box.probability < self.first_ball_confidence_threshold:
#                 self.log_warn(
#                     f"First ball detection low confidence: {box.probability:.2f} < "
#                     f"{self.first_ball_confidence_threshold:.2f}"
#                 )
#                 return True
        
#         # Check consistency with previous detection
#         if self.last_ball_pose is not None:
#             # Allow balls near start position
#             if np.linalg.norm(ball_pose.position[0:2]) < 0.1:
#                 pass
#             else:
#                 # Check distance from previous detection
#                 distance = np.linalg.norm(
#                     ball_pose.position[0:2] - self.last_ball_pose.position[0:2]
#                 )
                
#                 if distance > self.max_ball_distance_threshold:
#                     self.log_warn(
#                         f"Ball too far from previous position ({self.last_ball_pose_counter}): "
#                         f"last={self.last_ball_pose.position[0:2]}, "
#                         f"current={ball_pose.position[0:2]}, "
#                         f"distance={distance:.2f}m"
#                     )
                    
#                     self.last_ball_pose_counter += 1
                    
#                     # Reset after threshold to allow for ball movement
#                     if self.last_ball_pose_counter > 5:
#                         self.last_ball_pose_counter = 0
#                         self.last_ball_pose = None
#                         self.log_info("Resetting ball tracking - ball may have moved")
                    
#                     return True
        
#         # Accept detection if it's the largest one in this frame
#         if detection_size > self.max_detection_size:
#             self.final_camera_to_ball = camera_to_ball_transform
            
#             # Calculate pixel center for publishing
#             final_ball_pixel = Pose2D()
#             final_ball_pixel.x = (box.xmax + box.xmin) * 0.5
#             final_ball_pixel.y = (box.ymax + box.ymin) * 0.5
            
#             # Update tracking state
#             self.last_ball_pose = ball_pose
#             self.last_ball_pose_counter = 0
#             self.max_detection_size = detection_size
        
#         return False
    
#     def detect_obstacle(self, box: BoundingBox, obstacle_counter: int):
#         """
#         Process an obstacle (robot) detection
        
#         Args:
#             box: BoundingBox message containing robot detection
#             obstacle_counter: Current obstacle count
            
#         Returns:
#             Transformation to obstacle if detected, None otherwise
#         """
#         if box.probability < self.robot_confidence_threshold:
#             return None
        
#         # Check if obstacle detection includes base position
#         if not hasattr(box, 'xbase') or not hasattr(box, 'ybase'):
#             self.log_warn("Obstacle bounding box missing base position")
#             return None
        
#         try:
#             # Get floor position from base of bounding box
#             pos = [box.xbase, box.ybase]
#             floor_coordinate_robot = self.camera.find_floor_coordinate(pos)
            
#             # Create transformation to obstacle
#             world_to_obstacle = Transformation(position=floor_coordinate_robot)
#             camera_to_obstacle = (
#                 np.linalg.inv(self.camera.pose.matrix) @ world_to_obstacle.matrix
#             )
            
#             camera_to_obstacle_transform = Transformation(matrix=camera_to_obstacle)
            
#             self.log_info(
#                 f"Obstacle {obstacle_counter} detected at pixel [{pos}], "
#                 f"floor coord {floor_coordinate_robot}, "
#                 f"camera relative {camera_to_obstacle_transform.position}"
#             )
            
#             return camera_to_obstacle_transform
            
#         except Exception as e:
#             self.log_warn(f"Failed to calculate obstacle position: {e}")
#             return None
    
#     def process_detections(self, msg: BoundingBoxes):
#         """
#         Process all detections in a BoundingBoxes message
        
#         Args:
#             msg: BoundingBoxes message containing all detections
            
#         Returns:
#             tuple: (ball_pixel, camera_to_obstacle, obstacle_count)
#         """
#         # Reset detection state
#         self.max_detection_size = 0
#         self.final_camera_to_ball = None
#         final_ball_pixel = None
#         self.candidate_ball_counter = 0
#         obstacle_counter = 0
#         camera_to_obstacle = None
        
#         # Update camera position
#         try:
#             self.camera.reset_position(
#                 timestamp=msg.header.stamp,
#                 skip_if_not_found=True
#             )
#         except Exception as e:
#             self.log_warn(f"Failed to reset camera position: {e}")
#             return final_ball_pixel, camera_to_obstacle, obstacle_counter
        
#         # Process each detection
#         for box in msg.bounding_boxes:
#             # Ball detection (class "0")
#             if box.data == "0":
#                 if not self.detect_ball(box):
#                     # Ball was valid and processed
#                     final_ball_pixel = Pose2D()
#                     final_ball_pixel.x = (box.xmax + box.xmin) * 0.5
#                     final_ball_pixel.y = (box.ymax + box.ymin) * 0.5
            
#             # Robot/obstacle detection (class "2")
#             elif box.data == "2":
#                 obstacle_transform = self.detect_obstacle(box, obstacle_counter)
#                 if obstacle_transform is not None:
#                     camera_to_obstacle = obstacle_transform
#                     obstacle_counter += 1
        
#         # Log final ball detection
#         if self.final_camera_to_ball is not None:
#             self.log_info(
#                 f"\033[1m\033[34mBall detected at "
#                 f"[{self.last_ball_pose.position[0]:.3f}, "
#                 f"{self.last_ball_pose.position[1]:.3f}]\033[0m"
#             )
        
#         return final_ball_pixel, camera_to_obstacle, obstacle_counter


# # ROS2 Node wrapper (if needed as standalone node)
# class DetectorObjectsRos(DetectorObjects, Node):
#     """
#     ROS2 node wrapper for object detection
#     """
    
#     def __init__(self):
#         Node.__init__(self, "detector_objects")
#         DetectorObjects.__init__(self, node=self)
        
#         # Declare parameters
#         self.declare_parameter("first_ball_confidence_threshold", 0.78)
#         self.declare_parameter("robot_confidence_threshold", 0.78)
#         self.declare_parameter("max_ball_distance_threshold", 3.0)
        
#         # Get parameters
#         self.first_ball_confidence_threshold = self.get_parameter(
#             "first_ball_confidence_threshold"
#         ).value
#         self.robot_confidence_threshold = self.get_parameter(
#             "robot_confidence_threshold"
#         ).value
#         self.max_ball_distance_threshold = self.get_parameter(
#             "max_ball_distance_threshold"
#         ).value
        
#         # Subscriber
#         self.bounding_boxes_sub = self.create_subscription(
#             BoundingBoxes,
#             "object_bounding_boxes",
#             self.bounding_boxes_callback,
#             10
#         )
        
#         # Publisher
#         self.ball_pixel_pub = self.create_publisher(
#             Pose2D,
#             "ball_pixel",
#             10
#         )
        
#         self.get_logger().info("Object detector node initialized")
    
#     def bounding_boxes_callback(self, msg: BoundingBoxes):
#         """Callback for bounding boxes"""
#         ball_pixel, obstacle, obstacle_count = self.process_detections(msg)
        
#         if ball_pixel is not None:
#             self.ball_pixel_pub.publish(ball_pixel)


# def main(args=None):
#     """Main entry point for the node"""
#     rclpy.init(args=args)
    
#     try:
#         node = DetectorObjectsRos()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("\nShutting down object detector node...")
#     except Exception as e:
#         print(f"Error running node: {e}")
#     finally:
#         if rclpy.ok():
#             node.destroy_node()
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()