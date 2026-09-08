####### walking init pose #######
float32 init_x_offset
float32 init_y_offset
float32 init_z_offset
float32 init_roll_offset
float32 init_pitch_offset
float32 init_yaw_offset

####### time parameter #####
float32 period_time
float32 dsp_ratio
float32 step_fb_ratio
# Lama (detik) menaikkan amplitudo Movement dan tempo dari kecil ke nilai penuh
# sesudah "start". 0 = mati (langsung penuh, perilaku lama).
float32 soft_start_time

########## walking parameter ########
float32 x_move_amplitude
float32 y_move_amplitude
float32 z_move_amplitude
float32 angle_move_amplitude
# Putaran yaw "putar dalam" (turn in). Terpisah dari angle_move_amplitude di atas,
# yang kembali persis seperti aslinya. Dipakai untuk hip yaw servo 7 + 8 memutar
# ke arah yang SAMA, dan kaki yang sedang menapak melakukan seluruh putarannya.
# Tandanya memilih kaki mana yang berputar saat menapak sekaligus arah putaran
# badan: > 0 kaki kiri / searah jarum jam, < 0 kaki kanan / berlawanan arah
# jarum jam. Kedua sisi benar-benar bercermin -- sifat "putar ke dalam" sama.
float32 angle_move_amplitude_turn_in
bool move_aim_on

########## balance parameter ##########
bool balance_enable
float32 balance_hip_roll_gain
float32 balance_knee_gain
float32 balance_ankle_roll_gain
float32 balance_ankle_pitch_gain
float32 y_swap_amplitude
float32 z_swap_amplitude
float32 arm_swing_gain
float32 pelvis_offset
float32 hip_pitch_offset

########## gain parameter ##########
int32 p_gain
int32 i_gain
int32 d_gain