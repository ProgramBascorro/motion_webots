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
# BULK READ ITEMS servo sengaja HANYA present_position.
#
# position_p/i/d_gain pernah ada di daftar ini, dan itu murni beban mati:
# GroupBulkRead memakai alamat item PERTAMA sebagai alamat awal (132,
# present_position), jadi isAvailable() untuk gain -- alamat 84/82/80, di BAWAH
# 132 -- selalu false dan nilainya TIDAK PERNAH terbaca. Yang ia lakukan cuma
# memanjangkan balasan tiap servo dari 4 byte jadi 10.
#
# Bedanya nyata di kabel: 20 x (11 + 10) = 420 byte jadi 20 x (11 + 4) = 300
# byte; round-trip terukur 3,79 ms -> 2,81 ms. Di robot ini itu menentukan,
# karena OpenCR full-speed 12 Mbit menempel di hub USB yang sama dengan U2D2
# dan menggerus jendela balasan servo (lihat robotis_controller.cpp).
#
# Gain tetap dibaca sekali saat init lewat pembacaan per-item biasa, jadi tidak
# ada yang hilang. Ini juga TIDAK mengubah peta indirect address, jadi tidak
# perlu torque off.
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 1   | XM430-W350     | 2.0      | r_sho_pitch    | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 2   | XM430-W350     | 2.0      | l_sho_pitch    | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 3   | XM430-W350     | 2.0      | r_sho_roll     | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 4   | XM430-W350     | 2.0      | l_sho_roll     | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 5   | XM430-W350     | 2.0      | r_el           | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 6   | XM430-W350     | 2.0      | l_el           | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 7   | XM430-W350     | 2.0      | r_hip_yaw      | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 8   | XM430-W350     | 2.0      | l_hip_yaw      | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 9   | XM430-W350     | 2.0      | r_hip_roll     | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 10  | XM430-W350     | 2.0      | l_hip_roll     | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 11  | XM430-W350     | 2.0      | r_hip_pitch    | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 12  | XM430-W350     | 2.0      | l_hip_pitch    | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 13  | XM430-W350     | 2.0      | r_knee         | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 14  | XM430-W350     | 2.0      | l_knee         | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 15  | XM430-W350     | 2.0      | r_ank_pitch    | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 16  | XM430-W350     | 2.0      | l_ank_pitch    | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 17  | XM430-W350     | 2.0      | r_ank_roll     | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 18  | XM430-W350     | 2.0      | l_ank_roll     | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 19  | XM430-W350     | 2.0      | head_pan       | present_position
dynamixel | /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0 | 20  | XM430-W350     | 2.0      | head_tilt      | present_position
sensor    | /dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00 | 200 | OPEN-CR        | 2.0      | open-cr        | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw, mag_status
