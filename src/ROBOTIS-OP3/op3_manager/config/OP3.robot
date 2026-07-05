[ control info ]
control_cycle = 8    milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyOP3 | 2000000   | r_sho_pitch

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/ttyOP3 | 1   | XM430-W350     | 2.0      | r_sho_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 2   | XM430-W350     | 2.0      | l_sho_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 3   | XM430-W350     | 2.0      | r_sho_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 4   | XM430-W350     | 2.0      | l_sho_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 5   | XM430-W350     | 2.0      | r_el           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 6   | XM430-W350     | 2.0      | l_el           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 7   | XM430-W350     | 2.0      | r_hip_yaw      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 8   | XM430-W350     | 2.0      | l_hip_yaw      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 9   | XM430-W350     | 2.0      | r_hip_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 10  | XM430-W350     | 2.0      | l_hip_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 11  | XM430-W350     | 2.0      | r_hip_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 12  | XM430-W350     | 2.0      | l_hip_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 13  | XM430-W350     | 2.0      | r_knee         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 14  | XM430-W350     | 2.0      | l_knee         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 15  | XM430-W350     | 2.0      | r_ank_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 16  | XM430-W350     | 2.0      | l_ank_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 17  | XM430-W350     | 2.0      | r_ank_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 18  | XM430-W350     | 2.0      | l_ank_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 19  | XM430-W350     | 2.0      | head_pan       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyOP3 | 20  | XM430-W350     | 2.0      | head_tilt      | present_position, position_p_gain, position_i_gain, position_d_gain
sensor    | /dev/ttyOP3 | 200 | OPEN-CR        | 2.0      | open-cr        | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw
