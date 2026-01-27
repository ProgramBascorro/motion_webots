import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Webots) or real robot'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='op3',
        description='Robot name: op3, bez1, bez2'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='half_5.pt',
        description='YOLOv5 model file'
    )
    
    # Get config path
    use_sim = LaunchConfiguration('use_sim')
    robot_name = LaunchConfiguration('robot_name')
    
    pkg_dir = get_package_share_directory('soccer_object_detection')
    
    # Config file based on robot and sim/real
    config_file = os.path.join(
        pkg_dir, 'config', 'op3_sim.yaml'  # We just created this!
    )
    
    # Model path
    model_path = os.path.join(
        pkg_dir, 'models', LaunchConfiguration('model')
    )
    
    # Object detection node
    object_detection_node = Node(
        package='soccer_object_detection',
        executable='soccer_object_detection',
        name='object_detector',
        output='screen',
        parameters=[config_file],
        arguments=['--model', model_path],
        remappings=[
            ('/camera/image_raw', '/robotis_op3/camera/image_raw'),  # OP3 topic
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        use_sim_arg,
        robot_name_arg,
        model_arg,
        object_detection_node,
    ])