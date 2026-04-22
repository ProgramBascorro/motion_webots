from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Deklarasi argumen launch
    control_cycle_arg = DeclareLaunchArgument(
        'control_cycle',
        default_value='8',
        description='Control cycle in milliseconds'
    )

    use_dummy_data_arg = DeclareLaunchArgument(
        'use_dummy_data',
        default_value='false',
        description='Use dummy data for testing without real hardware'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'
    )
    
    reader_only_arg = DeclareLaunchArgument(
        'reader_only',
        default_value='false',
        description='Launch only the reader node without the main OpenCR node'
    )
    
    # Get launch configurations
    control_cycle = LaunchConfiguration('control_cycle')
    use_dummy_data = LaunchConfiguration('use_dummy_data')
    log_level = LaunchConfiguration('log_level')
    reader_only = LaunchConfiguration('reader_only')
    
    # Define the OpenCR module node
    open_cr_node = Node(
        package='open_cr_module',
        executable='open_cr_node',
        name='open_cr_module',
        output='screen',
        parameters=[{
            'control_cycle': control_cycle,
            'use_dummy_data': use_dummy_data
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=UnlessCondition(reader_only)
    )
    
    # Define the OpenCR reader node
    open_cr_reader = Node(
        package='open_cr_module',
        executable='open_cr_reader',
        name='open_cr_reader',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    return LaunchDescription([
        control_cycle_arg,
        use_dummy_data_arg,
        log_level_arg,
        reader_only_arg,
        open_cr_node,
        open_cr_reader
    ])