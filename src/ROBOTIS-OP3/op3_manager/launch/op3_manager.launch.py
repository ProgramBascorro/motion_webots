from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gazebo_default = False
    gazebo_robot_name_default = 'robotis_op3'
    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    action_file_path_default = get_package_share_directory('op3_action_module') + '/data/ALPHONSE.bin'
    # Dua port, dua peran -- op3_manager memakai keduanya untuk hal berbeda:
    #   device_name           -> bus servo (U2D2). Dipakai buttonHandlerCallback
    #                            untuk membaca torque ID 1 saat tombol ditekan lama.
    #   sub_controller_device -> OpenCR (ID 200). Dipakai untuk power-on DXL + LED.
    # Antarmuka bus TTL OpenCR di board ini mati, jadi ID 200 hanya menjawab lewat
    # micro-USB-nya. Menyatukan keduanya ke satu port membuat salah satu pasti
    # gagal: itulah asal "Torque on DXLs! [RxPacketError]" + "Fail to control LED".
    device_name_default = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0'
    sub_controller_device_default = '/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00'

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_name',
            default_value=device_name_default,
            description='Serial device path for the Dynamixel servo bus (U2D2)'
        ),
        DeclareLaunchArgument(
            'sub_controller_device',
            default_value=sub_controller_device_default,
            description='Serial device path for the OpenCR sub-controller (ID 200)'
        ),
        SetEnvironmentVariable('OP3_ACTION_FILE', action_file_path_default),
        Node(
            package='op3_manager',
            executable='op3_manager',
            # name='op3_manager',
            output='screen',
            parameters=[{
                # Batas laju kepala, deg/s. Dipakai head_control_module untuk
                # menghitung moving_time = MAX(1,0 ; |delta| / angle_unit) pada
                # perintah absolut (sapuan), dan angle_unit * 1,5 pada perintah
                # offset (pelacakan bola). Dinaikkan 30 -> 35 pada 2026-08-25
                # supaya kepala lebih gesit; 35 juga nilai bawaan modulnya
                # sendiri. Harus seiring dengan scan_period_sec di
                # head_tracking.yaml -- alasannya ada di komentar sana.
                'angle_unit': 35.0,
                'gazebo': gazebo_default,
                'gazebo_robot_name': gazebo_robot_name_default,
                'offset_file_path': offset_file_path_default,
                'robot_file_path': robot_file_path_default,
                'init_file_path': init_file_path_default,
                'device_name': LaunchConfiguration('device_name'),
                'sub_controller_device': LaunchConfiguration('sub_controller_device')
            }]
        )
        # Node(
        #     package='op3_localization',
        #     executable='op3_localization',
        #     name='op3_localization',
        #     output='screen'
        # )
    ])
