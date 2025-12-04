import argparse
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

# Import camera config
sys.path.insert(0, str(Path(__file__).parent))
from camera_config import CameraConfig

USB_CAM_DIR = get_package_share_directory('usb_cam')

# Define cameras with EXPLICIT remappings
CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='camera',
        param_path=Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
        namespace='',  # No namespace
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('image_raw/compressed', '/camera/image_raw/compressed'),
            ('image_raw/compressedDepth', '/camera/image_raw/compressedDepth'),
            ('image_raw/theora', '/camera/image_raw/theora'),
            ('camera_info', '/camera/camera_info'),
        ]
    )
)


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='usb_cam')

    camera_nodes = [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            name=camera.name if camera.name else 'usb_cam',
            namespace=camera.namespace if camera.namespace else '',
            parameters=[str(camera.param_path)],
            remappings=camera.remappings if camera.remappings else []
        )
        for camera in CAMERAS
    ]

    camera_group = GroupAction(camera_nodes)
    ld.add_action(camera_group)

    return ld