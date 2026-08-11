[ control info ]
control_cycle = 8    milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
# Servo 1-20 lewat U2D2 (bus Dynamixel TTL).
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 2000000   | r_sho_pitch
# OpenCR (ID 200) lewat micro-USB, BUKAN lewat bus: antarmuka TTL bus di board ini
# mati (ping ID 200 diam di semua baud 9600-4M, sementara 20 servo menjawab bersih),
# tapi MCU/firmware/IMU sehat -- lewat USB dia menjawab err=0, model 0x7400,
# acc_z ~17616 (1g), 12.5V. Baudrate diabaikan oleh USB CDC, diisi sama demi konsisten.
/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00 | 2000000   | open-cr

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 1   | XM430-W350     | 2.0      | r_sho_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 2   | XM430-W350     | 2.0      | l_sho_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 3   | XM430-W350     | 2.0      | r_sho_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 4   | XM430-W350     | 2.0      | l_sho_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 5   | XM430-W350     | 2.0      | r_el           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 6   | XM430-W350     | 2.0      | l_el           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 7   | XM430-W350     | 2.0      | r_hip_yaw      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 8   | XM430-W350     | 2.0      | l_hip_yaw      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 9   | XM430-W350     | 2.0      | r_hip_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 10  | XM430-W350     | 2.0      | l_hip_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 11  | XM430-W350     | 2.0      | r_hip_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 12  | XM430-W350     | 2.0      | l_hip_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 13  | XM430-W350     | 2.0      | r_knee         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 14  | XM430-W350     | 2.0      | l_knee         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 15  | XM430-W350     | 2.0      | r_ank_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 16  | XM430-W350     | 2.0      | l_ank_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 17  | XM430-W350     | 2.0      | r_ank_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 18  | XM430-W350     | 2.0      | l_ank_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 19  | XM430-W350     | 2.0      | head_pan       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 20  | XM430-W350     | 2.0      | head_tilt      | present_position, position_p_gain, position_i_gain, position_d_gain
sensor    | /dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00 | 200 | OPEN-CR        | 2.0      | open-cr        | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw
