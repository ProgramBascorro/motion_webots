"""Training di robot asli untuk Bascorro Studio (tab RL Training dan Action Training).

Diadaptasi dari branch ALPHONSE_NEW_V3 (scripts/training_action_web.py,
autotune_action_core.py, autotune_walking_core.py, training_sources.py) untuk
CHRONUS. Hanya bagian yang dipakai Studio yang diambil; menu Training terminal
(CMA-ES, feedback teks) tidak.

Modul:
  action_bin      format .bin action OP3 (baca, tulis di tempat, validasi)
  action_knobs    variabel "<sendi>@a-b" / "time_scale@a-b" -> page baru, fase page
  imu_metrics     metrik IMU satu percobaan (kemiringan tertapis, detektor jatuh)
  action_trainer  layanan tab Action Training (dipanggil apply_node, aksi act_*)
  trial_store     penyimpanan percobaan yang sudah dinilai operator (JSON)

Semuanya MURNI (tanpa rclpy), jadi bisa diuji tanpa robot: test/test_training_*.py.
"""
