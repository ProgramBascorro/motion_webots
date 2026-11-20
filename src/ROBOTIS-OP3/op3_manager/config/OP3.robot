[ control info ]
control_cycle = 8    milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
#
# DUA jalur terpisah, sesuai firmware "opencr_op3 edited by Fayyadh Raja Gula".
# Firmware itu memindahkan DXL_PORT dari Serial3 ke Serial (dxl_def.h), jadi
# sub controller ID 200 tidak lagi menjawab di bus TTL -- ia menjawab di micro-USB
# OpenCR sebagai USB CDC. Akibatnya:
#
#   /dev/ttyOP3     = U2D2 (FTDI 0403:6014)  -> HANYA 20 servo
#   /dev/ttyOPENCR  = OpenCR CDC (0483:5740) -> HANYA sensor ID 200 (IMU+magnetometer)
#
# Inilah tujuan pemisahannya: bulk read servo (20 device, tiap siklus 8 ms) tidak
# lagi berbagi bus dengan bulk read IMU, jadi keduanya tidak saling menunggu dan
# tidak saling merusak paket. robotis_controller memang menyimpan satu
# GroupBulkRead per port (port_to_bulk_read_), jadi kedua port berjalan sendiri.
#
# Baudrate CDC diabaikan perangkat kerasnya; 2000000 hanya menyamakan tulisan.
# Kalau board dikembalikan ke firmware bawaan (ID 200 di Serial3), hapus baris
# ttyOPENCR dan kembalikan sensor open-cr ke /dev/ttyOP3.
/dev/ttyOP3     | 2000000   | r_sho_pitch
/dev/ttyOPENCR  | 2000000   | open-cr

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
sensor    | /dev/ttyOPENCR | 200 | OPEN-CR     | 2.0      | open-cr        | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw, mag_x, mag_y, mag_z, mag_heading, yaw_gyro, mag_status
