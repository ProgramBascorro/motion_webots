import os
import launch
from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, ExecuteProcess # Remove ExecuteProcess if not used elsewhere
from launch.actions import IncludeLaunchDescription # Keep if needed
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# from webots_ros2_driver.webots_launcher import WebotsLauncher # Still not needed

def generate_launch_description():

    ld = LaunchDescription()

    package_dir = get_package_share_directory('op3_webots_ros2')
    gain_file_path_default = package_dir + '/resource/op3_pid_gain_default.yaml'

    # Define the path to the Webots world file for the OP3
    # We still might need this path to tell the user which world to open manually
    webots_world_path_info = os.path.join(package_dir, 'worlds', 'robotis_op3_extern.wbt')
    print(f"--- Please manually open this world file in Webots on Windows: {webots_world_path_info} ---")
    print(f"--- Ensure the robot controller is set to <extern> in Webots ---")


    # ############################################################
    # ## REMOVE OR COMMENT OUT THESE LINES ##
    # # Define the path to your Webots executable in WSL
    # webots_executable_path = '/usr/local/webots/webots' # No longer needed here
    # # Use ExecuteProcess to launch Webots directly
    # webots = ExecuteProcess(
    #     cmd=[webots_executable_path, webots_world_path_info], # Use the world path variable
    #     output='screen',
    #     # Add this event handler so that closing Webots stops the launch file
    #     on_exit=launch.actions.EmitEvent(event=launch.events.Shutdown()),
    # )
    # ############################################################


    # This Node launches the external controller for the OP3 that communicates with Webots
    op3_controller_node = Node(
        package='op3_webots_ros2',
        executable='op3_extern_controller',
        output='screen',
        parameters=[{'gain_file_path': gain_file_path_default}]
        # IMPORTANT: Check if op3_extern_controller needs parameters
        #            to know WHERE Webots is running (e.g., IP address/port).
        #            It might default to 127.0.0.1:1234 which *should* work
        #            for connecting from WSL2 to Windows host.
    )

    get_up_node = Node(
        package='op3_get_up_behavior', # The new package name
        executable='get_up_node',      # The executable name from setup.py
        name='op3_get_up_node',        # Optional instance name
        output='screen',
        parameters=[
        {'imu_topic': '/robotis_op3/imu'}, # <<< ADD THIS LINE
        # You can also override other params here if needed:
        # {'fallen_back_pitch_threshold_rad': 1.2}
        ]
    )

    yolo_detector_node = Node(
        package='yolo_detector_node',
        executable='yolo_node', # Matches entry_point in setup.py
        name='yolo_detector_node',
        output='screen',
        parameters=[{
            # Specify model (e.g., standard yolov8 nano)
            # Or if using custom: 'model_name': 'resource/your_custom_model.pt'
            'model_name': 'yolov8n.pt',
            'input_topic': '/robotis_op3/camera/image_raw', # Verify camera topic
            'output_topic': '/yolo/detections',
            'device': '', # Auto-detect ('cpu' or 'cuda')
            'conf_threshold': 0.4,
            'iou_threshold': 0.5,
        }]
    )

    yolo_visualizer_node = Node(
        package='yolo_visualizer_node',
        executable='visualizer_node', # Name from its setup.py
        name='yolo_visualizer_node',
        output='screen',
        # Remap input topics if they differ from defaults in the node
        # remappings=[
        #     ('input_image_topic', '/robotis_op3/camera/image_raw'),
        #     ('input_detections_topic', '/yolo/detections'),
        # ]
    )

    # Add the actions to the launch description
    # ld.add_action(webots) # REMOVE OR COMMENT OUT THIS LINE
    ld.add_action(op3_controller_node)
    ld.add_action(yolo_detector_node) # Add the YOLO node
    ld.add_action(yolo_visualizer_node)
    # ld.add_action(get_up_node) # Add the new get_up_node

    return ld
