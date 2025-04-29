#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from ultralytics import YOLO # Import YOLO from ultralytics
import os
from ament_index_python.packages import get_package_share_directory

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # --- Parameters ---
        self.declare_parameter('model_name', 'yolov8n.pt') # Standard model name or path to custom .pt file
        self.declare_parameter('conf_threshold', 0.4)      # Confidence threshold
        self.declare_parameter('iou_threshold', 0.5)       # NMS IoU threshold
        self.declare_parameter('device', '')               # Device: 'cpu', 'cuda:0' etc. '' means auto-detect
        self.declare_parameter('input_topic', '/robotis_op3/camera/image_raw')
        self.declare_parameter('output_topic', '/yolo/detections')

        # Get parameter values
        model_param = self.get_parameter('model_name').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # --- Model Loading ---
        # If the model_param doesn't contain a path separator, assume it's a standard
        # ultralytics model name. Otherwise, treat it as a path relative to package share.
        if os.path.sep not in model_param:
             self.model_path = model_param # e.g., 'yolov8n.pt', ultralytics will handle download
             self.get_logger().info(f"Using standard YOLO model: {self.model_path}")
        else:
            # Construct full path if it's a relative path within the package
            package_share_directory = get_package_share_directory('yolo_detector_node')
            # Assuming model placed in 'resource' subdir
            self.model_path = os.path.join(package_share_directory, 'resource', model_param)
            self.get_logger().info(f"Looking for custom model at: {self.model_path}")


        # Auto-detect device if not specified
        if not self.device:
             self.device = "cuda" if torch.cuda.is_available() else "cpu"

        try:
            self.model = YOLO(self.model_path) # Load the model
            self.model.to(self.device) # Move model to specified device
            # Optional: Warmup - Run inference once on a dummy image
            # self.model.predict(source=np.zeros((640, 480, 3)), device=self.device, verbose=False)
            self.get_logger().info(f"Successfully loaded YOLO model '{self.model_path}' on device '{self.device}'")
        except Exception as e:
            self.get_logger().error(f"Error loading YOLO model: {e}")
            raise SystemExit(f"Error loading YOLO model: {e}")

        # --- ROS Setup ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            RosImage,
            input_topic,
            self.image_callback,
            10) # QoS profile depth
        self.publisher_ = self.create_publisher(
            Detection2DArray,
            output_topic,
            10) # QoS profile depth
        self.get_logger().info(f"Subscribing to '{input_topic}', Publishing to '{output_topic}'")


    def image_callback(self, msg: RosImage):
        # self.get_logger().debug('Received image')
        try:
            # Convert ROS Image message to OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # --- Inference ---
        # Ultralytics handles pre/post-processing (resizing, normalization, NMS etc.)
        try:
            results = self.model.predict(
                source=cv_image,            # Pass the OpenCV BGR image directly
                device=self.device,         # Specify device
                conf=self.conf_threshold,   # Confidence threshold
                iou=self.iou_threshold,     # NMS IoU threshold
                verbose=False               # Suppress ultralytics console output
            )
        except Exception as e:
             self.get_logger().error(f"Error during YOLO prediction: {e}")
             return

        # --- Process Results & Publish ---
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header # Use the same timestamp and frame_id

        # Check if results contain detections
        if results and len(results) > 0:
             # Process results for the first image (index 0)
             result = results[0] # Ultralytics result object for the image
             boxes = result.boxes  # Access detected boxes and their properties

             for i in range(len(boxes)):
                box_data = boxes[i] # Get data for one box

                detection = Detection2D()
                detection.header = msg.header # Consistent header

                # Bounding Box (convert xyxy format from ultralytics to center/size)
                xyxy = box_data.xyxy.cpu().numpy().squeeze() # Get tensor, move to CPU, convert to numpy, remove extra dims
                if xyxy.shape == (4,): # Ensure it's a valid box
                    x1, y1, x2, y2 = xyxy
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                    width = x2 - x1
                    height = y2 - y1

                    if width <= 0 or height <= 0: continue # Skip invalid boxes

                    detection.bbox.center.position.x = float(center_x)
                    detection.bbox.center.position.y = float(center_y)
                    detection.bbox.size_x = float(width)
                    detection.bbox.size_y = float(height)
                    # detection.bbox.center.theta = 0.0 # Assuming non-rotated boxes
                else:
                     self.get_logger().warn(f"Unexpected box format: {xyxy}")
                     continue


                # Hypothesis (Score and Class ID)
                hypothesis = ObjectHypothesisWithPose()
                cls_id_tensor = box_data.cls.cpu() # Get class ID tensor, move to CPU
                conf_tensor = box_data.conf.cpu() # Get confidence tensor, move to CPU

                if cls_id_tensor.numel() > 0 and conf_tensor.numel() > 0:
                     class_id = int(cls_id_tensor.item())   # Convert tensor to integer Python scalar
                     score = float(conf_tensor.item())    # Convert tensor to float Python scalar

                     # Get class name if available (optional)
                     class_name = self.model.names[class_id] if hasattr(self.model, 'names') else str(class_id)
                     hypothesis.hypothesis.class_id = class_name # Use name or ID
                     hypothesis.hypothesis.score = score
                     detection.results.append(hypothesis)
                else:
                     self.get_logger().warn(f"Missing class or score for a detection.")
                     continue # Skip if missing essential info

                detections_msg.detections.append(detection)


        # Publish the array, even if it's empty (indicates no detections)
        self.publisher_.publish(detections_msg)
        # self.get_logger().debug(f'Published {len(detections_msg.detections)} detections.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloNode()
        rclpy.spin(node)
    except SystemExit as e:
         print(f"Node initialization failed: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if node and hasattr(node, 'destroy_node'): node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
