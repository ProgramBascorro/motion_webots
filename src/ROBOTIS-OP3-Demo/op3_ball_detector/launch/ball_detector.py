from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('op3_ball_detector')
    config_dir = os.path.join(pkg_dir, 'config')
    camera_param_path = os.path.join(config_dir, 'camera_param.yaml')
    model_path = os.path.join(pkg_dir, 'models', 'best_yolov8.onnx')
    
    ld = LaunchDescription()
    
    # USB Camera Node - with namespace 'camera'
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',  # ← ADD THIS: Creates /camera/image_raw
        output='screen',
        parameters=[camera_param_path]
    )
    
    # Ball Detector Node (C++ with ONNX)
    ball_detector_node = Node(
        package='op3_ball_detector',
        namespace='ball_detector',
        executable='ball_detector_node',
        name='ball_detector',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'camera_topic': '/camera/image_raw',  # ← Matches namespace
            'confidence_threshold': 0.6,
            'nms_threshold': 0.4,
            'ball_class_id': 0,
            'input_width': 320,
            'input_height': 320,
            'use_gpu': False,
            'publish_debug_image': True,
            'process_every_n_frames': 1,
        }]
    )
    
    ld.add_action(usb_cam_node)
    ld.add_action(ball_detector_node)
    
    return ld