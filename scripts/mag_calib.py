#!/usr/bin/env python3
"""Kalibrasi yaw kompas OpenCR: ukur dulu, baru koreksi.

Kenapa berkas ini ada
=====================
Gejala yang membawa kita ke sini: yaw benar di 0 derajat, meleset di 180.
Itu BUKAN gejala acak -- itu tanda tangan galat hard-iron, dan cuma satu
penyebab yang menghasilkan pola persis begitu.

Magnetometer mengukur medan bumi DITAMBAH medan tetap yang ikut menempel di
robot (baut, motor, kabel arus). Medan tambahan itu (hard-iron) menggeser pusat
lingkaran pembacaan dari titik nol. Kalau pusatnya bergeser sejauh b sementara
medan bumi sebesar H, arah yang terbaca meleset sebesar

    galat(psi) ~= -(by*cos(psi) + bx*sin(psi)) / H          [radian]

Sekarang perhatikan apa yang terjadi setelah yaw di-nol-kan di satu arah
(firmware melakukannya saat OpenCR menyala, atau lewat perintah zero_yaw):

    yaw_terbaca(phi) = phi + galat(ref + phi) - galat(ref)

Di phi = 0 galatnya SELALU nol -- bukan karena kompasnya benar, tapi karena
kita baru saja mendefinisikan titik itu sebagai nol. Di phi = 180, cos dan sin
keduanya berbalik tanda, jadi selisihnya menjadi DUA KALI galat di titik nol:

    yaw_terbaca(180) = 180 + 2 * galat(ref)

Jadi "0 oke, 180 meleset" adalah pola yang paling khas dari hard-iron yang
belum dikoreksi, dan besar melesetnya di 180 langsung memberi tahu seberapa
besar biasnya: meleset 30 derajat berarti b/H sekitar 0.26.

Dua kandidat lain bisa dicoret dengan alasan yang sama tegasnya:
  - Soft-iron (lingkaran jadi elips) menghasilkan galat DUA siklus per putaran,
    yang nilainya sama di 0 dan di 180 -- setelah dinolkan di 0, galatnya di 180
    justru hilang. Bukan ini.
  - Salah tanda/arah putar akan salah sejak awal, termasuk di dekat 0. Bukan ini.

Karena itu alat ini tidak langsung menulis angka. Urutannya: UKUR (check),
lihat bentuk galatnya, baru KOREKSI, lalu UKUR LAGI.

Urutan pemakaian
================
    scripts/mag_calib.py monitor              # apakah datanya hidup dan waras
    scripts/mag_calib.py check                # ukur galat di 0/90/180/270
    scripts/mag_calib.py calibrate            # kalibrasi hard-iron oleh firmware
    scripts/mag_calib.py check                # buktikan sudah membaik

Kalau yaw benar selama servo mati tapi meleset begitu servo hidup, itu gejala
yang lain dan ada perintahnya sendiri:

    scripts/mag_calib.py torque               # ukur pergeserannya, cari sebabnya

Untuk hampir semua kasus, sampai di sini sudah selesai. Empat perintah di atas
tidak menyentuh apa pun selain register yang memang disediakan firmware, dan
`calibrate` berlaku seketika tanpa reset.

Kalau setelah itu galatnya masih besar, barulah jalur teliti dipakai. Ia perlu
tahu offset yang SEDANG berlaku -- dan firmware tidak menerbitkan angka itu ke
ROS -- jadi caranya dibuat nol dulu supaya tidak ada yang ditebak:

    scripts/mag_calib.py write --x 0 --y 0 --z 0   # kosongkan
    (reset OpenCR)
    scripts/mag_calib.py record                    # fit seluruh sampel
    scripts/mag_calib.py write --from-record       # basis 0, jadi ini mutlak
    (reset OpenCR)
    scripts/mag_calib.py check

Dijalankan DI DALAM container, lewat entrypoint supaya ROS ada di PATH:

    docker exec -it op3 /docker-entrypoint.sh bash
    ROS_DOMAIN_ID=1 python3 /ros2_ws/scripts/mag_calib.py check

Syarat yang tidak boleh dilanggar saat mengukur maupun mengkalibrasi
===================================================================
Hard-iron itu medan yang IKUT BERPUTAR bersama robot. Supaya yang terukur
memang medan itu dan bukan yang lain:

  - Robot dalam keadaan operasional: torque ON, semua kabel terpasang seperti
    biasa. Kalibrasi dengan servo mati lalu dipakai dengan servo hidup berarti
    mengkalibrasi robot yang berbeda.
  - Jauh dari meja besi, tulangan beton, speaker, laptop, dan lantai berbaja.
    Berdiri di tengah ruangan jauh lebih baik daripada di pinggir.
  - Robot tegak dan diputar pada sumbu tegak. Kompensasi kemiringan di firmware
    dihitung dari roll/pitch fusi IMU; memiringkan robot saat kalibrasi
    memasukkan galat yang tidak ada hubungannya dengan hard-iron.
  - Putar PELAN. Filter arah di firmware (MAG_LPF_ALPHA 0.05) sengaja lambat;
    memutar cepat membuat angka yang terbaca tertinggal dari posisi sebenarnya.

Yaw bergeser saat torque dinyalakan
===================================
Gejala: yaw benar selama servo mati, lalu melompat belasan sampai puluhan
derajat begitu torque ON, dan kembali lagi ketika torque OFF. Karena datang dan
perginya mengikuti torque, yang salah bukan kalibrasinya -- kalibrasinya benar,
tapi benar untuk robot dalam keadaan yang lain.

Tiga hal berubah saat torque menyala, dan ketiganya menghasilkan gejala yang
persis sama:

  1. ARUS. Belasan servo menahan pose menarik arus lewat kabel daya yang
     melintas dekat OpenCR. Arus 2 A pada jarak 2 cm menghasilkan sekitar 20 uT,
     sementara medan mendatar bumi di sini cuma sekitar 40 uT -- cukup untuk
     memutar arah yang terbaca sekitar 20 derajat. Medan ini IKUT BERPUTAR
     bersama robot, jadi bentuknya sama persis dengan hard-iron: bias tetap di
     kerangka badan. Bedanya, besarnya berubah mengikuti beban.
  2. POSE. Tiap servo punya magnet permanen di rotor dan di enkodernya. Begitu
     torque menyala robot pindah dari pose lemas ke pose berdiri, dan magnet-
     magnet itu ikut pindah. Hasilnya juga bias tetap di kerangka badan -- yang
     ini berubah mengikuti pose, bukan beban.
  3. KEMIRINGAN. Kalau tegaknya badan berubah saat torque menyala, kompensasi
     kemiringan mulai memakai sumbu Z. Offset Z yang keliru (lihat bagian
     berikutnya) baru terasa persis di saat itu.

Nomor 1 dan 2 obatnya sama: kalibrasi ulang dalam keadaan operasional, karena
keduanya berupa bias tetap di kerangka badan dan bias tetap memang persis yang
bisa disimpan firmware. Nomor 3 tidak -- ia butuh offset Z yang benar, dan
kalibrasi ulang di bidang mendatar justru tidak menyentuhnya.

Bedanya juga menentukan seberapa jauh perbaikannya akan bertahan:

  - arus  -> offset yang ditulis cuma benar pada satu titik beban. Berdiri diam
             dan berjalan menarik arus yang berbeda, jadi sisa galatnya ikut
             berubah-ubah. Ukur dulu seberapa besar variasinya sebelum percaya.
  - pose  -> offset yang ditulis benar untuk pose itu, dan pose berdiri robot
             berulang dengan rapi. Yaw tetap boleh dipercaya selama posenya
             tidak berubah jauh (lengan mengayun saat berjalan tetap menggeser).

Perintah `torque` memisahkan ketiganya dengan mengukur, bukan menebak: ia
membandingkan medan yang terbaca pada empat keadaan yang cuma berbeda pada
torque dan bebannya. Selama pengukuran robot TIDAK BOLEH DIPUTAR -- yang
diukur adalah medan tambahannya, dan memutar robot mengubah medan bumi yang
terbaca sehingga keduanya tercampur.

Catatan penting soal kalibrasi bawaan firmware
==============================================
Perintah `calibrate` memakai kalibrasi min/maks milik firmware. Itu cukup untuk
sumbu X dan Y, tapi ia JUGA menulis ulang offset sumbu Z dari data putaran
mendatar -- padahal saat robot berputar tegak, nilai Z hampir tidak berubah,
jadi min dan maks Z nyaris sama dan titik tengahnya jatuh di nilai Z saat itu.
Akibatnya offset Z menjadi kira-kira sebesar pembacaan Z itu sendiri.

Selama robot tegak, Z tidak dipakai (kompensasi kemiringan meniadakannya), jadi
tidak terasa. Begitu robot berjalan dan badannya miring, offset Z yang keliru
mulai merusak yaw. Karena itu `write` selalu menulis offset Z secara eksplisit
(bawaannya 0), bukan meninggalkan hasil min/maks.
"""

import argparse
import math
import os
import statistics
import sys
import threading
import time

GREEN, RED, YELLOW, DIM, BOLD, RST = (
    "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[1m", "\033[0m")

# 0.15 uT per cacahan, AK8963 mode 16 bit. Dipakai hanya untuk melaporkan
# kekuatan medan dalam satuan yang bisa dibandingkan dengan peta magnet bumi.
LSB_TO_UT = 0.15

# Berkas perantara antara `record` dan `write`, supaya dua langkah itu tidak
# harus dijalankan dalam satu sesi terminal.
FIT_FILE = os.environ.get("OP3_MAG_FIT_FILE", "/tmp/op3_mag_fit.txt")

STATUS_NAME = {
    0: "NO_SENSOR (magnetometer tidak terbaca)",
    1: "SETTLING (firmware masih mengunci referensi 0 derajat)",
    2: "OK",
    3: "CALIBRATING (kalibrasi hard-iron sedang berjalan)",
}


# ---------------------------------------------------------------------------
# Bantuan sudut
# ---------------------------------------------------------------------------

def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def wrap360(deg):
    return deg % 360.0


def roll_pitch_dari_acc(ax, ay, az):
    """Kemiringan badan dari accelerometer, rumus yang sama dengan modulnya.

    Sengaja meniru publishIMU() di open_cr_module.cpp, bukan memakai register
    roll/pitch firmware: yang dipakai balance control adalah angka accel ini.
    Rumusnya tidak bergantung satuan, jadi m/s2 dari topik memberi hasil yang
    sama dengan cacahan mentah di modul.

    Penamaan sumbunya mengikuti modul apa adanya, dan modul itu sendiri
    mencatat bahwa board terpasang terputar 90 derajat sehingga "roll" dan
    "pitch" di sini belum tentu roll dan pitch badan. Untuk keperluan 'torque'
    itu tidak jadi soal: yang dipakai cuma SELISIH antar keadaan dan besar
    perubahannya, dan keduanya kebal terhadap tukar-menukar sumbu.
    """
    mui = 0.01
    tanda = math.copysign(1.0, az)
    roll = math.atan2(ay, tanda * math.sqrt(az * az + mui * ax * ax))
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return math.degrees(roll), math.degrees(pitch)


def rata_sudut(daftar):
    """Rata-rata lewat sin/cos, supaya lompatan +179 / -179 tidak jadi 0."""
    s_ = sum(math.sin(math.radians(v)) for v in daftar) / len(daftar)
    c_ = sum(math.cos(math.radians(v)) for v in daftar) / len(daftar)
    return math.degrees(math.atan2(s_, c_))


# ---------------------------------------------------------------------------
# Aljabar linear seadanya
#
# Sengaja tidak memakai numpy. Alasannya sama dengan op3_ping.py dan dxl_regs.py:
# alat diagnosa harus tetap jalan justru ketika lingkungannya bermasalah, dan
# numpy di container ini pernah hilang/berganti versi (lihat catatan cv_bridge).
# Yang dibutuhkan cuma penyelesaian sistem 5x5, itu muat dalam 20 baris.
# ---------------------------------------------------------------------------

def solve(matrix, rhs):
    """Eliminasi Gauss dengan pivot parsial. Mengembalikan None kalau singular."""
    n = len(rhs)
    m = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]

    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            return None
        m[col], m[pivot] = m[pivot], m[col]

        for row in range(n):
            if row == col:
                continue
            factor = m[row][col] / m[col][col]
            for k in range(col, n + 1):
                m[row][k] -= factor * m[col][k]

    return [m[i][n] / m[i][i] for i in range(n)]


def normal_equations(rows, rhs):
    """Bentuk A^T A dan A^T b, lalu selesaikan. rows = daftar baris desain."""
    n = len(rows[0])
    ata = [[0.0] * n for _ in range(n)]
    atb = [0.0] * n

    for row, b in zip(rows, rhs):
        for i in range(n):
            atb[i] += row[i] * b
            for j in range(n):
                ata[i][j] += row[i] * row[j]

    return solve(ata, atb)


# ---------------------------------------------------------------------------
# Pencocokan lingkaran dan elips
# ---------------------------------------------------------------------------

def fit_circle(points):
    """Kasa fit: x^2+y^2 = 2*cx*x + 2*cy*y + c. Mengembalikan (cx, cy, r)."""
    rows = [[2.0 * x, 2.0 * y, 1.0] for x, y in points]
    rhs = [x * x + y * y for x, y in points]

    sol = normal_equations(rows, rhs)
    if sol is None:
        return None

    cx, cy, c = sol
    radikal = c + cx * cx + cy * cy
    if radikal <= 0.0:
        return None

    return cx, cy, math.sqrt(radikal)


def fit_ellipse(points):
    """Fit kerucut umum a x^2 + b xy + c y^2 + d x + e y - 1 = 0.

    Dipakai HANYA sebagai diagnosa soft-iron: yang dilaporkan adalah rasio
    sumbu panjang terhadap sumbu pendek. Firmware tidak punya tempat untuk
    menyimpan koreksi soft-iron, jadi angka ini tidak dituliskan ke mana pun --
    ia cuma memberi tahu apakah sisa galat setelah koreksi hard-iron memang
    masih bisa diperbaiki atau sudah mentok.
    """
    rows = [[x * x, x * y, y * y, x, y] for x, y in points]
    rhs = [1.0] * len(points)

    sol = normal_equations(rows, rhs)
    if sol is None:
        return None

    a, b, c, d, e = sol
    f = -1.0

    denom = b * b - 4.0 * a * c
    if abs(denom) < 1e-12:
        return None

    x0 = (2.0 * c * d - b * e) / denom
    y0 = (2.0 * a * e - b * d) / denom

    konstanta = -(a * x0 * x0 + b * x0 * y0 + c * y0 * y0 + d * x0 + e * y0 + f)
    if konstanta <= 0.0:
        return None

    # Nilai eigen matriks [[a, b/2], [b/2, c]], bentuk tertutup untuk 2x2 simetris.
    tengah = (a + c) / 2.0
    beda = math.sqrt(((a - c) / 2.0) ** 2 + (b / 2.0) ** 2)
    lam_besar, lam_kecil = tengah + beda, tengah - beda

    if lam_kecil <= 0.0 or lam_besar <= 0.0:
        return None

    # Sumbu = sqrt(konstanta / lambda): lambda besar -> sumbu pendek.
    sumbu_pendek = math.sqrt(konstanta / lam_besar)
    sumbu_panjang = math.sqrt(konstanta / lam_kecil)

    return x0, y0, sumbu_panjang, sumbu_pendek


def heading_from(mx, my):
    """Arah kompas dari komponen mendatar, meniru persis rumus firmware.

    Firmware memakai atan2(-yh, xh) dengan xh/yh hasil kompensasi kemiringan.
    Selama robot tegak, xh = mx dan yh = my, jadi rumus ini setara -- dan itu
    memang syarat pengukuran di sini (lihat docstring di atas).
    """
    return wrap360(math.degrees(math.atan2(-my, mx)))


# ---------------------------------------------------------------------------
# Simpul ROS
# ---------------------------------------------------------------------------

class MagNode:
    """Pembungkus tipis di atas rclpy supaya perintah-perintah di bawah pendek."""

    def __init__(self, domain=None):
        if domain is not None:
            os.environ["ROS_DOMAIN_ID"] = str(domain)

        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray, String
        from sensor_msgs.msg import Imu
        from robotis_controller_msgs.msg import SyncWriteItem

        self._rclpy = rclpy
        self._SyncWriteItem = SyncWriteItem

        rclpy.init(args=None)
        self.node = Node("mag_calib")

        self.terakhir = None
        self.waktu_terakhir = None
        self.terakhir_imu = None
        self._kunci = threading.Lock()

        self.node.create_subscription(
            Float64MultiArray, "/robotis/open_cr/mag_status", self._cb, 10)
        # Kemiringan dibutuhkan cuma oleh 'torque', untuk mencoret (atau
        # menetapkan) kemiringan sebagai penyebab pergeseran yaw. Perintah lain
        # mengabaikannya, dan langganan yang menganggur tidak berbiaya apa-apa.
        self.node.create_subscription(Imu, "/robotis/open_cr/imu", self._cb_imu, 10)
        self.imu_cmd = self.node.create_publisher(
            String, "/robotis/open_cr/imu_command", 1)
        self.sync_write = self.node.create_publisher(
            SyncWriteItem, "/robotis/sync_write_item", 1)
        self._String = String

        self._exec = threading.Thread(target=self._spin, daemon=True)
        self._exec.start()

    def _spin(self):
        try:
            self._rclpy.spin(self.node)
        except Exception:
            pass

    def _cb(self, msg):
        data = list(msg.data)
        if len(data) < 7:
            return

        with self._kunci:
            self.terakhir = data
            self.waktu_terakhir = time.monotonic()

    def _cb_imu(self, msg):
        a = msg.linear_acceleration
        with self._kunci:
            self.terakhir_imu = (a.x, a.y, a.z)

    def baca(self, timeout=5.0):
        """Cuplikan terakhir sebagai dict, atau None kalau topiknya diam."""
        batas = time.monotonic() + timeout
        while time.monotonic() < batas:
            with self._kunci:
                data = self.terakhir
            if data is not None:
                return self._sebagai_dict(data)
            time.sleep(0.02)
        return None

    def baca_imu(self):
        """Percepatan terakhir (ax, ay, az), atau None kalau topiknya diam."""
        with self._kunci:
            return self.terakhir_imu

    @staticmethod
    def _sebagai_dict(data):
        hasil = {
            "status": int(data[0]),
            "heading": data[1],
            "yaw": data[2],
            "yaw_gyro": data[3],
            "mag_x": data[4],
            "mag_y": data[5],
            "mag_z": data[6],
        }
        return hasil

    def kirim_perintah(self, teks):
        msg = self._String()
        msg.data = teks
        self.imu_cmd.publish(msg)

    def tulis_item(self, nama_item, nilai):
        """Tulis satu item ke sub controller.

        SATU item per panggilan, dengan jeda di pemanggil. robotis_controller
        menyimpan satu GroupSyncWrite per port per siklus, jadi item kedua dan
        seterusnya yang datang dalam siklus yang sama akan hilang diam-diam.
        """
        msg = self._SyncWriteItem()
        msg.item_name = nama_item
        msg.joint_name.append("open-cr")
        msg.value.append(int(nilai) & 0xFFFF)
        self.sync_write.publish(msg)

    def tutup(self):
        try:
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


def butuh_data(mag):
    contoh = mag.baca()
    if contoh is None:
        domain = os.environ.get("ROS_DOMAIN_ID", "(kosong)")
        print(f"{RED}Tidak ada pesan di /robotis/open_cr/mag_status.{RST}")
        print(f"{DIM}  ROS_DOMAIN_ID sekarang = {domain}. Stack robot ini jalan di"
              f" domain 1 (profil real_robot),{RST}")
        print(f"{DIM}  jadi coba: ROS_DOMAIN_ID=1 python3 scripts/mag_calib.py ...{RST}")
        print(f"{DIM}  Kalau tetap kosong, op3_manager belum jalan atau masih memakai"
              f" build lama{RST}")
        print(f"{DIM}  (topik /robotis/open_cr/mag baru ada setelah open_cr_module"
              f" dibangun ulang).{RST}")
        return None
    return contoh


def peringatan_status(contoh):
    status = contoh["status"]
    if status == 2:
        return True

    print(f"{YELLOW}mag_status = {status} : {STATUS_NAME.get(status, '?')}{RST}")
    if status == 0:
        print(f"{DIM}  Yaw kompas tidak akan bergerak sama sekali. Periksa apakah board"
              f" memang MPU9250{RST}")
        print(f"{DIM}  (varian dengan ICM20648 tidak punya magnetometer).{RST}")
    elif status == 1:
        print(f"{DIM}  Tunggu beberapa detik; firmware sedang merata-rata arah awal.{RST}")
    elif status == 3:
        print(f"{DIM}  Kalibrasi sedang berjalan, tunggu sampai selesai.{RST}")
    return False


# ---------------------------------------------------------------------------
# Perintah
# ---------------------------------------------------------------------------

def cmd_monitor(mag, args):
    contoh = butuh_data(mag)
    if contoh is None:
        return 1

    print(f"{BOLD}Pantau magnetometer{RST} -- Ctrl-C untuk berhenti\n")
    print(f"{DIM}heading = arah kompas mutlak, yaw = relatif terhadap referensi"
          f" (arah saat OpenCR menyala{RST}")
    print(f"{DIM}atau saat zero_yaw terakhir). norm = kekuatan medan; kalau ia"
          f" berubah banyak saat robot{RST}")
    print(f"{DIM}diputar, berarti masih ada hard-iron atau ada benda besi di"
          f" dekatnya.{RST}\n")

    # Offset hard-iron ada di EEPROM OpenCR alamat 6..12, tapi tidak ikut
    # diterbitkan ke ROS: yang dibaca tiap siklus cuma daftar bulk read di
    # OP3.robot, dan register itu tidak ada di sana. Jadi "sudah dikalibrasi
    # atau belum" tidak bisa dijawab dengan membaca angka -- dijawab dengan
    # melihat BENTUK galatnya lewat 'check'. Galat satu siklus per putaran yang
    # besar berarti hard-iron belum terkoreksi.
    print(f"{DIM}  Offset hard-iron tersimpan di EEPROM OpenCR dan tidak diterbitkan"
          f" ke ROS.{RST}")
    print(f"{DIM}  Untuk tahu sudah dikalibrasi atau belum, jalankan 'check' dan lihat"
          f" komponen{RST}")
    print(f"{DIM}  satu siklusnya.{RST}\n")

    header = (f"  {'status':<14}{'heading':>9}{'yaw':>9}{'yaw_gyro':>10}"
              f"{'mag_x':>8}{'mag_y':>8}{'mag_z':>8}{'norm':>9}{'uT':>8}")
    print(header)
    print("  " + "-" * (len(header) - 2))

    try:
        while True:
            c = mag.baca(timeout=2.0)
            if c is None:
                print(f"\r{RED}  data berhenti{RST}", end="", flush=True)
                time.sleep(0.2)
                continue

            norm = math.sqrt(c["mag_x"] ** 2 + c["mag_y"] ** 2 + c["mag_z"] ** 2)
            warna = GREEN if c["status"] == 2 else YELLOW
            nama = STATUS_NAME.get(c["status"], "?").split(" ")[0]

            print(f"\r  {warna}{nama:<14}{RST}{c['heading']:>9.1f}{c['yaw']:>9.1f}"
                  f"{c['yaw_gyro']:>10.1f}{c['mag_x']:>8.0f}{c['mag_y']:>8.0f}"
                  f"{c['mag_z']:>8.0f}{norm:>9.1f}{norm * LSB_TO_UT:>8.1f}",
                  end="", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n")
    return 0


def cmd_zero(mag, args):
    if butuh_data(mag) is None:
        return 1

    mag.kirim_perintah("zero_yaw")
    time.sleep(1.0)

    c = mag.baca()
    print(f"{GREEN}Yaw dinolkan di arah sekarang.{RST}")
    if c:
        print(f"  heading {c['heading']:.1f} derajat sekarang berarti yaw"
              f" {c['yaw']:.1f} derajat")
    return 0


def sektor_bar(terisi, jumlah=24):
    return "".join(f"{GREEN}#{RST}" if i in terisi else f"{DIM}.{RST}"
                   for i in range(jumlah))


def kumpulkan_putaran(mag, detik, judul):
    """Rekam sambil operator memutar robot; laporkan cakupan arah secara langsung.

    Cakupan itu yang paling sering jadi biang gagalnya kalibrasi kompas: kalau
    robot cuma diputar 200 derajat, sisi lingkaran yang lain tidak pernah
    terlihat, dan titik tengah yang dihitung dari data separuh lingkaran meleset
    jauh. Karena itu progresnya ditampilkan per sektor 15 derajat, bukan cuma
    hitungan mundur waktu.
    """
    titik = []
    terisi = set()
    mulai = time.monotonic()
    terakhir_gambar = 0.0

    print(f"\n{BOLD}{judul}{RST}")
    print(f"{DIM}  Putar robot PELAN pada sumbu tegak, minimal satu putaran penuh."
          f" Tiap '#'{RST}")
    print(f"{DIM}  adalah sektor 15 derajat yang sudah terekam.{RST}\n")

    while time.monotonic() - mulai < detik:
        c = mag.baca(timeout=1.0)
        if c is None:
            time.sleep(0.05)
            continue

        mx, my = c["mag_x"], c["mag_y"]
        titik.append((mx, my, c["mag_z"], c["heading"], c["yaw"]))
        terisi.add(int(heading_from(mx, my) // 15) % 24)

        sekarang = time.monotonic()
        if sekarang - terakhir_gambar > 0.15:
            sisa = detik - (sekarang - mulai)
            print(f"\r  [{sektor_bar(terisi)}] {len(terisi):2d}/24 sektor"
                  f"   sisa {sisa:4.1f}s  ", end="", flush=True)
            terakhir_gambar = sekarang

        time.sleep(0.02)

    print(f"\r  [{sektor_bar(terisi)}] {len(terisi):2d}/24 sektor   selesai      ")
    return titik, terisi


def laporkan_fit(titik, terisi):
    """Cocokkan lingkaran, lalu terjemahkan hasilnya jadi ramalan galat yaw."""
    if len(terisi) < 16:
        print(f"\n{YELLOW}Cakupan cuma {len(terisi)}/24 sektor.{RST}"
              f" Titik tengah dari data setengah lingkaran tidak bisa dipercaya --")
        print(f"{DIM}  ulangi sambil memutar robot lebih jauh.{RST}")

    xy = [(t[0], t[1]) for t in titik]
    lingkaran = fit_circle(xy)
    if lingkaran is None:
        print(f"{RED}Pencocokan lingkaran gagal -- datanya tidak membentuk busur.{RST}")
        return None

    cx, cy, r = lingkaran

    sisa = [math.hypot(x - cx, y - cy) - r for x, y in xy]
    rms = math.sqrt(sum(s * s for s in sisa) / len(sisa))

    print(f"\n{BOLD}Hasil pencocokan lingkaran{RST}")
    print(f"  jumlah sampel        : {len(xy)}")
    print(f"  pusat (bias hard-iron): x {cx:+8.1f}   y {cy:+8.1f}  cacahan")
    print(f"  jari-jari (medan bumi): {r:8.1f} cacahan = {r * LSB_TO_UT:.1f} uT")
    print(f"  sisa RMS              : {rms:8.1f} cacahan")

    if r < 1e-6:
        return None

    rasio = math.hypot(cx, cy) / r
    print(f"  bias / medan          : {rasio:8.3f}")

    elips = fit_ellipse(xy)
    if elips is not None:
        _, _, panjang, pendek = elips
        if pendek > 1e-6:
            print(f"  rasio sumbu elips     : {panjang / pendek:8.3f}"
                  f"   {DIM}(1.00 = bulat sempurna; soft-iron){RST}")

    # Ramalan galat: bandingkan arah yang dihitung dari data mentah (yang dipakai
    # firmware sekarang) dengan arah dari data yang sudah dipindah ke pusat.
    galat = []
    for x, y in xy:
        benar = heading_from(x - cx, y - cy)
        mentah = heading_from(x, y)
        galat.append((benar, wrap180(mentah - benar)))

    print(f"\n{BOLD}Ramalan yaw yang akan terbaca, kalau offset ini TIDAK dipakai{RST}")
    print(f"{DIM}  Titik 0 diambil dari referensi yaw yang berlaku sekarang."
          f" Bandingkan baris 180{RST}")
    print(f"{DIM}  dengan apa yang Anda lihat di robot -- kalau cocok, penyebabnya"
          f" memang hard-iron.{RST}\n")

    # Referensi yang sedang dipakai firmware: heading dikurangi yaw.
    ref = wrap360(titik[-1][3] - titik[-1][4])
    galat_ref = galat_di(galat, ref)

    if galat_ref is None:
        print(f"{YELLOW}  Arah referensi tidak ada dalam rekaman -- lewati ramalan.{RST}")
    else:
        print(f"  {'putaran sebenarnya':>19}  {'yaw akan terbaca':>17}  {'meleset':>9}")
        terburuk = 0.0
        for phi in range(0, 360, 45):
            g = galat_di(galat, wrap360(ref + phi))
            if g is None:
                continue
            terbaca = phi + wrap180(g - galat_ref)
            meleset = wrap180(terbaca - phi)
            terburuk = max(terburuk, abs(meleset))
            warna = GREEN if abs(meleset) < 5 else (YELLOW if abs(meleset) < 15 else RED)
            print(f"  {phi:>16d} deg  {wrap180(terbaca):>14.1f} deg"
                  f"  {warna}{meleset:>+8.1f}{RST}")
        print(f"\n  meleset terparah sekeliling lingkaran : "
              f"{RED if terburuk > 15 else YELLOW}{terburuk:.1f} derajat{RST}")

    return cx, cy, r, rms


def galat_di(galat, arah, toleransi=12.0):
    """Galat pada arah tertentu: rata-rata sampel yang arahnya paling dekat."""
    dekat = [g for a, g in galat if abs(wrap180(a - arah)) <= toleransi]
    if not dekat:
        return None
    # Rata-rata lewat sin/cos supaya lompatan +179 / -179 tidak merusak.
    s = sum(math.sin(math.radians(g)) for g in dekat) / len(dekat)
    c = sum(math.cos(math.radians(g)) for g in dekat) / len(dekat)
    return math.degrees(math.atan2(s, c))


def cmd_record(mag, args):
    contoh = butuh_data(mag)
    if contoh is None:
        return 1
    peringatan_status(contoh)

    print(f"\n{BOLD}Rekam satu putaran penuh{RST}")
    print(f"{DIM}  Robot tegak, torque ON, jauh dari besi. Tekan Enter lalu mulai"
          f" memutar.{RST}")
    input("  Enter untuk mulai... ")

    titik, terisi = kumpulkan_putaran(mag, args.seconds, "Merekam")
    if len(titik) < 30:
        print(f"{RED}Sampel terlalu sedikit ({len(titik)}).{RST}")
        return 1

    hasil = laporkan_fit(titik, terisi)
    if hasil is None:
        return 1

    cx, cy, r, rms = hasil

    # Terjemahkan pusat di kerangka badan menjadi offset di sumbu sensor mentah.
    #
    # Firmware:  mx_c = magADC[0] - off[0]
    #            my_c = magADC[1] - off[1]
    #   lalu     Bx   = my_c      (MAG_BODY_X)
    #            By   = mx_c      (MAG_BODY_Y)
    #
    # Supaya Bx berkurang cx dan By berkurang cy, yang harus bertambah adalah
    # off[1] sebesar cx dan off[0] sebesar cy -- sumbunya BERTUKAR, dan itu
    # bukan salah ketik.
    delta_off_x = cy
    delta_off_y = cx

    with open(FIT_FILE, "w") as f:
        f.write(f"delta_off_x {delta_off_x:.1f}\n")
        f.write(f"delta_off_y {delta_off_y:.1f}\n")
        f.write(f"center_bx {cx:.1f}\n")
        f.write(f"center_by {cy:.1f}\n")
        f.write(f"radius {r:.1f}\n")
        f.write(f"rms {rms:.1f}\n")
        f.write(f"waktu {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    print(f"\n{BOLD}Koreksi yang perlu ditambahkan ke offset sekarang{RST}")
    print(f"  mag_x_offset += {delta_off_x:+8.1f}")
    print(f"  mag_y_offset += {delta_off_y:+8.1f}")
    print(f"{DIM}  (sumbu bertukar karena firmware memetakan Bx dari magADC[1]"
          f" dan By dari magADC[0]){RST}")
    print(f"\n  Tersimpan di {FIT_FILE}. Lanjutkan dengan:")
    print(f"    scripts/mag_calib.py write --from-record")
    return 0


def cmd_calibrate(mag, args):
    contoh = butuh_data(mag)
    if contoh is None:
        return 1

    print(f"{BOLD}Kalibrasi hard-iron oleh firmware{RST}")
    print(f"{DIM}  Firmware mencatat nilai terkecil dan terbesar tiap sumbu selama"
          f" 20 detik, lalu{RST}")
    print(f"{DIM}  menyimpan titik tengahnya sebagai offset. Karena cuma dua sampel"
          f" ekstrem per sumbu{RST}")
    print(f"{DIM}  yang menentukan hasilnya, satu lonjakan derau bisa merusak"
          f" seluruh kalibrasi --{RST}")
    print(f"{DIM}  kalau hasilnya meragukan, pakai 'record' + 'write' yang mencocokkan"
          f" seluruh sampel.{RST}\n")
    print(f"{YELLOW}  Offset sumbu Z juga ikut ditulis ulang dari putaran mendatar ini,"
          f" dan itu keliru{RST}")
    print(f"{YELLOW}  (lihat catatan di kepala berkas). Perbaiki dengan"
          f" 'write --z 0' sesudahnya.{RST}\n")

    input("  Robot tegak dan siap diputar. Enter untuk mulai... ")

    mag.kirim_perintah("calibrate_mag")
    time.sleep(0.3)

    titik, terisi = kumpulkan_putaran(mag, 21.0, "Kalibrasi berjalan (20 detik)")

    time.sleep(1.5)

    if len(terisi) < 16:
        print(f"\n{RED}Cakupan cuma {len(terisi)}/24 sektor -- putaran belum penuh.{RST}")
        print(f"{DIM}  Offset yang baru saja tersimpan dihitung dari data yang tidak"
              f" lengkap. Ulangi.{RST}")

    print(f"\n{GREEN}Selesai. Firmware sudah menolkan yaw di arah sekarang.{RST}")
    print(f"  Buktikan dengan: scripts/mag_calib.py check")
    return 0


def cmd_check(mag, args):
    """Ukur galat pada arah fisik yang sudah diketahui, lalu uraikan bentuknya."""
    contoh = butuh_data(mag)
    if contoh is None:
        return 1
    if not peringatan_status(contoh):
        print(f"{DIM}  Pengukuran diteruskan, tapi angkanya belum tentu berarti.{RST}\n")

    n = args.points
    langkah = 360.0 / n

    print(f"{BOLD}Ukur galat yaw di {n} arah{RST}")
    print(f"{DIM}  Butuh acuan arah fisik. Yang paling gampang: tempel busur derajat"
          f" atau tandai{RST}")
    print(f"{DIM}  lantai dengan selotip tiap {langkah:.0f} derajat. Ketelitian"
          f" acuan menentukan{RST}")
    print(f"{DIM}  ketelitian hasil -- kalau tandanya meleset 10 derajat, hasilnya"
          f" ikut meleset.{RST}")
    print(f"{DIM}  Robot diputar SEARAH dengan arah yang menaikkan yaw. Kalau di"
          f" langkah kedua{RST}")
    print(f"{DIM}  yaw malah mengecil, hentikan (Ctrl-C) dan putar ke arah"
          f" sebaliknya.{RST}\n")

    print(f"  Hadapkan robot ke arah acuan 0 derajat.")
    input("  Enter kalau sudah... ")

    mag.kirim_perintah("zero_yaw")
    time.sleep(1.2)

    ukuran = []
    for i in range(n):
        sudut = i * langkah
        if i > 0:
            print(f"\n  Putar robot ke {sudut:.0f} derajat.")
            input("  Enter kalau sudah... ")

        # Rata-rata sebentar: filter firmware lambat, satu cuplikan bisa
        # tertinggal beberapa derajat dari posisi sebenarnya.
        contoh_yaw = []
        batas = time.monotonic() + 1.5
        while time.monotonic() < batas:
            c = mag.baca(timeout=1.0)
            if c:
                contoh_yaw.append(c["yaw"])
            time.sleep(0.05)

        if not contoh_yaw:
            print(f"{RED}  Tidak ada data.{RST}")
            return 1

        s = sum(math.sin(math.radians(v)) for v in contoh_yaw) / len(contoh_yaw)
        cc = sum(math.cos(math.radians(v)) for v in contoh_yaw) / len(contoh_yaw)
        yaw = math.degrees(math.atan2(s, cc))

        sebaran = statistics.pstdev(contoh_yaw) if len(contoh_yaw) > 1 else 0.0
        meleset = wrap180(yaw - sudut)
        ukuran.append((sudut, yaw, meleset))

        warna = GREEN if abs(meleset) < 5 else (YELLOW if abs(meleset) < 15 else RED)
        catatan = f"  {DIM}(sebaran {sebaran:.1f}){RST}" if sebaran > 3 else ""
        print(f"    acuan {sudut:6.1f}  ->  yaw {yaw:+7.1f}"
              f"   meleset {warna}{meleset:+6.1f}{RST}{catatan}")

    laporkan_galat(ukuran)
    return 0


def laporkan_galat(ukuran):
    """Uraikan galat jadi komponen tetap, satu siklus, dan dua siklus per putaran.

    Pembagian ini yang menentukan tindakan berikutnya, karena tiap bentuk galat
    punya penyebab yang berbeda dan obat yang berbeda:

      satu siklus  -> hard-iron. Bisa dihilangkan: kalibrasi atau tulis offset.
      dua siklus   -> soft-iron. TIDAK bisa disimpan di firmware ini; obatnya
                      memindahkan sumber besi/menjauhkan magnetometer.
      tetap        -> acuan fisiknya sendiri yang bergeser, bukan kompasnya.
    """
    n = len(ukuran)
    terburuk = max(abs(m) for _, _, m in ukuran)

    print(f"\n{BOLD}Ringkasan{RST}")
    print(f"  meleset terparah : "
          f"{GREEN if terburuk < 5 else (YELLOW if terburuk < 15 else RED)}"
          f"{terburuk:.1f} derajat{RST}")

    if n < 4:
        return

    pakai_harmonik_2 = n >= 6
    rows, rhs = [], []
    for sudut, _, meleset in ukuran:
        r = math.radians(sudut)
        baris = [1.0, math.cos(r), math.sin(r)]
        if pakai_harmonik_2:
            baris += [math.cos(2 * r), math.sin(2 * r)]
        rows.append(baris)
        rhs.append(meleset)

    sol = normal_equations(rows, rhs)
    if sol is None:
        return

    tetap = sol[0]
    amp1 = math.hypot(sol[1], sol[2])
    amp2 = math.hypot(sol[3], sol[4]) if pakai_harmonik_2 else None

    print(f"  komponen tetap        : {tetap:+6.1f} derajat"
          f"   {DIM}(acuan fisik bergeser, bukan salah kompas){RST}")
    print(f"  komponen satu siklus  : {amp1:6.1f} derajat"
          f"   {DIM}(hard-iron -- bisa dikoreksi){RST}")
    if amp2 is not None:
        print(f"  komponen dua siklus   : {amp2:6.1f} derajat"
              f"   {DIM}(soft-iron -- tidak bisa disimpan di firmware ini){RST}")
    else:
        print(f"{DIM}  komponen dua siklus   : butuh minimal 6 titik"
              f" (--points 8){RST}")

    print()
    if terburuk < 5:
        print(f"{GREEN}  Sudah bagus. Tidak perlu tindakan lain.{RST}")
    elif amp1 > (amp2 or 0.0) and amp1 > 4.0:
        print(f"  Hard-iron masih dominan. Jalankan:")
        print(f"    scripts/mag_calib.py calibrate      {DIM}# cepat{RST}")
        print(f"    scripts/mag_calib.py record         {DIM}# lebih teliti{RST}")
    elif amp2 is not None and amp2 > 4.0:
        print(f"{YELLOW}  Soft-iron yang dominan, dan firmware tidak punya tempat"
              f" menyimpan koreksinya.{RST}")
        print(f"{DIM}  Yang berpengaruh: jarak magnetometer ke motor dan ke rangka"
              f" besi. Ukur ulang{RST}")
        print(f"{DIM}  dengan robot diangkat dari meja dulu -- kalau angkanya"
              f" langsung membaik, sumbernya{RST}")
        print(f"{DIM}  meja, bukan robot.{RST}")
    elif abs(tetap) > 5.0:
        print(f"{YELLOW}  Yang bergeser cuma titik nolnya. Jalankan 'zero' sambil"
              f" robot menghadap acuan 0.{RST}")


def cmd_steps(mag, args):
    """Kalibrasi berhenti-per-berhenti: putar 90 derajat, berhenti, ukur, ulangi.

    Kenapa cara ini lebih kuat daripada memutar menerus
    ===================================================
    Empat titik berjarak 90 derajat membentuk DUA PASANG BERLAWANAN: 0 dengan
    180, dan 90 dengan 270. Baik lingkaran maupun elips sama-sama simetris
    terhadap pusatnya, jadi rata-rata sepasang titik yang berlawanan jatuh
    TEPAT di pusat -- berapa pun jari-jarinya, berapa pun kelonjongannya, dan
    dari arah awal mana pun mulainya:

        (P(psi) + P(psi + 180)) / 2  =  pusat        (persis, bukan hampiran)

    Tiga akibat yang tidak dimiliki kalibrasi min/maks bawaan firmware:

      - Tidak ada pencocokan kurva sama sekali, jadi tidak ada yang bisa gagal
        karena cakupan putaran kurang. Yang dibutuhkan cuma titik-titiknya
        berpasangan berlawanan.
      - Kebal soft-iron. Kelonjongan elips menggeser jari-jari, tapi tidak
        menggeser pusat, jadi bias yang terhitung tetap benar.
      - Karena robot DIAM saat diukur, tiap titik bisa dirata-rata dari ratusan
        cuplikan. Derau magnetometer di robot ini kurang lebih +/-13 cacahan
        pada medan mendatar ~110 cacahan; merata-rata 2 detik menekannya sampai
        hampir hilang. Min/maks justru sebaliknya: hasilnya ditentukan oleh DUA
        cuplikan paling ekstrem per sumbu, jadi satu lonjakan derau langsung
        masuk ke offset.

    Yang harus akurat adalah PERLAWANANNYA (0 lawan 180), bukan 90 derajatnya.
    Itu kabar baik, karena membalik robot tepat berlawanan jauh lebih gampang
    dilakukan dengan tepat -- sejajarkan ke tepi lurus, lalu putar balik --
    daripada menakar 90 derajat dengan mata.

    Sekali jalan, dua jawaban: yaw yang terbaca di tiap titik memberi ukuran
    galat sekarang (seperti 'check'), sementara komponen magnetometernya memberi
    koreksinya.
    """
    contoh = butuh_data(mag)
    if contoh is None:
        return 1
    if not peringatan_status(contoh):
        print(f"{DIM}  Pengukuran diteruskan, tapi angkanya belum tentu berarti.{RST}\n")

    n = args.points
    if n % 2 != 0:
        print(f"{RED}Jumlah titik harus genap{RST} -- inti cara ini adalah pasangan"
              f" berlawanan, dan titik ganjil tidak punya pasangan.")
        return 1

    langkah = 360.0 / n

    print(f"{BOLD}Kalibrasi berhenti-per-berhenti, {n} titik tiap"
          f" {langkah:.0f} derajat{RST}")
    print(f"{DIM}  Robot tegak, torque ON, jauh dari besi. Di tiap titik robot DIAM"
          f" {args.dwell:.0f} detik{RST}")
    print(f"{DIM}  sementara alat ini merata-rata pembacaannya.{RST}\n")
    print(f"{DIM}  Yang menentukan ketelitian hasil adalah seberapa tepat titik ke-i"
          f" berlawanan dengan{RST}")
    print(f"{DIM}  titik ke-{n // 2 + 1} (0 lawan 180). Selisih 90 derajatnya sendiri"
          f" boleh meleset sedikit.{RST}\n")

    print(f"  Hadapkan robot ke arah acuan 0 derajat.")
    input("  Enter kalau sudah... ")

    mag.kirim_perintah("zero_yaw")
    time.sleep(1.2)

    titik = []
    for i in range(n):
        sudut = i * langkah
        if i > 0:
            print(f"\n  Putar robot ke {sudut:.0f} derajat, lalu diamkan.")
            input("  Enter kalau sudah... ")

        xs, ys, zs, yaws = [], [], [], []
        batas = time.monotonic() + args.dwell
        while time.monotonic() < batas:
            c = mag.baca(timeout=1.0)
            if c:
                xs.append(c["mag_x"])
                ys.append(c["mag_y"])
                zs.append(c["mag_z"])
                yaws.append(c["yaw"])
            time.sleep(0.02)

        if len(xs) < 5:
            print(f"{RED}  Cuplikan terlalu sedikit ({len(xs)}).{RST}")
            return 1

        # Komponen magnetometer yang diterbitkan firmware TIDAK disaring
        # (register Mag_X = mag_body_raw, filter sin/cos cuma dipakai untuk
        # heading), jadi rata-rata biasa memang yang benar di sini.
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        mz = sum(zs) / len(zs)

        # Yaw dirata-rata lewat sin/cos: di dekat titik 180 nilainya melompat
        # antara +180 dan -180, dan rata-rata biasa akan menghasilkan 0.
        sy = sum(math.sin(math.radians(v)) for v in yaws) / len(yaws)
        cy_ = sum(math.cos(math.radians(v)) for v in yaws) / len(yaws)
        yaw = math.degrees(math.atan2(sy, cy_))

        sebaran = statistics.pstdev(xs) if len(xs) > 1 else 0.0
        titik.append({"sudut": sudut, "mx": mx, "my": my, "mz": mz, "yaw": yaw})

        meleset = wrap180(yaw - sudut)
        warna = GREEN if abs(meleset) < 5 else (YELLOW if abs(meleset) < 15 else RED)
        print(f"    acuan {sudut:6.1f}  ->  yaw {yaw:+7.1f}  meleset"
              f" {warna}{meleset:+6.1f}{RST}   mag ({mx:+6.1f},{my:+6.1f})"
              f"  {DIM}{len(xs)} cuplikan, sebaran {sebaran:.1f}{RST}")

    return laporkan_steps(titik, n, args)


def laporkan_steps(titik, n, args):
    separuh = n // 2

    # Pusat dari tiap pasangan berlawanan. Dihitung terpisah dulu, baru
    # dirata-rata, supaya SELISIH antar pasangan bisa dilaporkan: kalau kedua
    # pasangan tidak sepakat, yang salah bukan hitungannya melainkan
    # pengukurannya -- tanda acuan miring, robot bergeser, atau ada besi yang
    # ikut pindah di tengah jalan.
    pusat = []
    for i in range(separuh):
        a, b = titik[i], titik[i + separuh]
        pusat.append(((a["mx"] + b["mx"]) / 2.0, (a["my"] + b["my"]) / 2.0))

    cx = sum(p[0] for p in pusat) / len(pusat)
    cy = sum(p[1] for p in pusat) / len(pusat)
    ketaksepakatan = max(math.hypot(p[0] - cx, p[1] - cy) for p in pusat) * 2.0

    jari = [math.hypot(t["mx"] - cx, t["my"] - cy) for t in titik]
    r = sum(jari) / len(jari)

    print(f"\n{BOLD}Pusat lingkaran = bias hard-iron{RST}")
    for i, p in enumerate(pusat):
        print(f"  dari pasangan {i * (360 // n):3d}/{i * (360 // n) + 180:3d}"
              f" : x {p[0]:+8.1f}   y {p[1]:+8.1f}")
    print(f"  {BOLD}gabungan{RST}              : x {cx:+8.1f}   y {cy:+8.1f}  cacahan")

    if len(pusat) > 1:
        warna = GREEN if ketaksepakatan < 0.15 * r else (
            YELLOW if ketaksepakatan < 0.35 * r else RED)
        print(f"  ketaksepakatan antar pasangan : {warna}{ketaksepakatan:.1f}"
              f" cacahan{RST}  {DIM}({ketaksepakatan / r * 100:.0f}% dari"
              f" jari-jari){RST}")
        if ketaksepakatan > 0.35 * r:
            print(f"{DIM}    Dua pasangan seharusnya menunjuk pusat yang sama."
                  f" Kalau tidak, biasanya{RST}")
            print(f"{DIM}    titik yang katanya berlawanan sebenarnya tidak"
                  f" berlawanan. Ulangi dengan{RST}")
            print(f"{DIM}    acuan yang lebih rapi sebelum menulis offset ini.{RST}")

    print(f"  jari-jari (medan bumi)        : {r:8.1f} cacahan"
          f" = {r * LSB_TO_UT:.1f} uT")
    print(f"  bias / medan                  : {math.hypot(cx, cy) / r:8.3f}")

    # Dua "garis tengah" yang saling tegak lurus. Rasionya menunjukkan
    # kelonjongan elips (soft-iron) tanpa perlu pencocokan apa pun -- dan
    # kelonjongan itu TIDAK merusak pusat yang dihitung di atas.
    if separuh >= 2:
        d = [math.hypot(titik[i]["mx"] - titik[i + separuh]["mx"],
                        titik[i]["my"] - titik[i + separuh]["my"]) / 2.0
             for i in range(separuh)]
        rasio = max(d) / min(d) if min(d) > 1e-6 else float("inf")
        warna = GREEN if rasio < 1.1 else (YELLOW if rasio < 1.3 else RED)
        print(f"  rasio garis tengah            : {warna}{rasio:8.3f}{RST}"
              f"   {DIM}(1.00 = bulat; di atas ~1.3 soft-iron mulai terasa){RST}")

    # Ramalan vs kenyataan. Kalau keduanya cocok, hard-iron memang seluruh
    # ceritanya dan koreksi di bawah akan menyelesaikannya. Kalau ramalan jauh
    # lebih kecil daripada yang terukur, ada penyebab lain yang belum tertangkap.
    galat = []
    for t in titik:
        benar = heading_from(t["mx"] - cx, t["my"] - cy)
        mentah = heading_from(t["mx"], t["my"])
        galat.append(wrap180(mentah - benar))

    print(f"\n{BOLD}Ramalan hard-iron dibanding yaw yang benar-benar terbaca{RST}")
    print(f"  {'acuan':>7}  {'yaw terukur':>12}  {'ramalan':>9}  {'selisih':>9}"
          f"  {'putaran sejati':>15}")
    for i, t in enumerate(titik):
        ramal = wrap180(t["sudut"] + wrap180(galat[i] - galat[0]))
        selisih = wrap180(t["yaw"] - ramal)
        sejati = wrap180(heading_from(titik[i]["mx"] - cx, titik[i]["my"] - cy)
                         - heading_from(titik[0]["mx"] - cx, titik[0]["my"] - cy))
        warna = GREEN if abs(selisih) < 5 else YELLOW
        print(f"  {t['sudut']:>6.0f}   {t['yaw']:>+11.1f}   {ramal:>+8.1f}"
              f"   {warna}{selisih:>+8.1f}{RST}   {sejati:>+14.1f}")

    print(f"{DIM}  'putaran sejati' dihitung dari magnetometer yang sudah dipindah"
          f" ke pusat. Kalau ia{RST}")
    print(f"{DIM}  jauh dari kolom acuan, tanda acuan di lantai yang meleset, bukan"
          f" kompasnya.{RST}")

    terburuk = max(abs(wrap180(t["yaw"] - t["sudut"])) for t in titik)
    print(f"\n  meleset terparah sekarang : "
          f"{GREEN if terburuk < 5 else (YELLOW if terburuk < 15 else RED)}"
          f"{terburuk:.1f} derajat{RST}")

    # Pemetaan sumbu: firmware mengurangi off[1] dari magADC[1] lalu memakainya
    # sebagai Bx, dan off[0] dari magADC[0] sebagai By. Jadi koreksi untuk sumbu
    # X badan masuk ke mag_y_offset, dan sebaliknya. Bukan salah ketik.
    delta_off_x = cy
    delta_off_y = cx

    with open(FIT_FILE, "w") as f:
        f.write(f"delta_off_x {delta_off_x:.1f}\n")
        f.write(f"delta_off_y {delta_off_y:.1f}\n")
        f.write(f"center_bx {cx:.1f}\n")
        f.write(f"center_by {cy:.1f}\n")
        f.write(f"radius {r:.1f}\n")
        f.write(f"rms 0.0\n")
        f.write(f"waktu {time.strftime('%Y-%m-%d %H:%M:%S')} (steps {n} titik)\n")

    print(f"\n{BOLD}Koreksi yang perlu ditambahkan ke offset sekarang{RST}")
    print(f"  mag_x_offset += {delta_off_x:+8.1f}")
    print(f"  mag_y_offset += {delta_off_y:+8.1f}")
    print(f"  Tersimpan di {FIT_FILE}.")

    if terburuk < 5:
        print(f"\n{GREEN}  Yaw sudah cukup baik -- tidak perlu menulis apa pun.{RST}")
    else:
        print(f"\n  Menuliskannya (offset di EEPROM sekarang harus 0,0,0 --"
              f" lihat kepala berkas):")
        print(f"    scripts/mag_calib.py write --from-record")
    return 0


def ukur_keadaan(mag, detik):
    """Rata-ratakan satu keadaan robot yang DIAM: magnetometer plus kemiringan.

    Dirata-rata dari ratusan cuplikan karena yang dicari perintah 'torque'
    adalah selisih belasan cacahan antara dua keadaan, sementara derau
    magnetometer sendiri kurang lebih +/-13 cacahan. Satu cuplikan tidak akan
    pernah bisa memisahkan keduanya.
    """
    xs, ys, zs, yaws, headings, rolls, pitches = [], [], [], [], [], [], []

    batas = time.monotonic() + detik
    while time.monotonic() < batas:
        c = mag.baca(timeout=1.0)
        if c:
            xs.append(c["mag_x"])
            ys.append(c["mag_y"])
            zs.append(c["mag_z"])
            yaws.append(c["yaw"])
            headings.append(c["heading"])

        acc = mag.baca_imu()
        if acc:
            r, p = roll_pitch_dari_acc(*acc)
            rolls.append(r)
            pitches.append(p)

        sisa = batas - time.monotonic()
        print(f"\r    merekam... {max(sisa, 0.0):4.1f}s  ", end="", flush=True)
        time.sleep(0.02)

    if len(xs) < 5:
        print(f"\r    {RED}cuplikan terlalu sedikit ({len(xs)}){RST}          ")
        return None

    hasil = {
        "mx": sum(xs) / len(xs),
        "my": sum(ys) / len(ys),
        "mz": sum(zs) / len(zs),
        "yaw": rata_sudut(yaws),
        "heading": rata_sudut(headings),
        "roll": sum(rolls) / len(rolls) if rolls else None,
        "pitch": sum(pitches) / len(pitches) if pitches else None,
        "n": len(xs),
        "sebaran": statistics.pstdev(xs) if len(xs) > 1 else 0.0,
    }
    print(f"\r    {len(xs)} cuplikan, sebaran {hasil['sebaran']:.1f} cacahan"
          f"          ")
    return hasil


def cmd_torque(mag, args):
    """Ukur pergeseran yaw antara servo mati dan servo hidup, lalu cari sebabnya.

    Empat keadaan direkam berturut-turut tanpa memutar robot sedikit pun, dan
    tiap pasangan menjawab satu pertanyaan yang berbeda:

        A  torque OFF                 <- acuan
        B  torque ON, pose berdiri    B-A = seluruh pergeserannya
        C  torque ON, ditekan         C-B = bagian yang mengikuti ARUS
        D  torque OFF lagi            D-A = apakah semuanya memang berulang

    C yang membuat perintah ini bisa menjawab lebih dari sekadar "ada
    pergeseran". Antara B dan C posenya sama persis dan yang bertambah cuma
    arus penahan, jadi apa pun yang berubah di situ pasti datang dari arus.
    Sisanya -- yang muncul di B-A tapi tidak di C-B -- datang dari pose.

    D bukan basa-basi. Kalau robot tergeser sedikit saja oleh tangan operator
    di tengah jalan, seluruh selisih di atas jadi campuran medan tambahan
    dengan medan bumi, dan tidak ada cara memisahkannya lagi setelah itu.
    D yang kembali ke A membuktikan hal itu tidak terjadi.
    """
    contoh = butuh_data(mag)
    if contoh is None:
        return 1
    if not peringatan_status(contoh):
        print(f"{DIM}  Pengukuran diteruskan, tapi angkanya belum tentu berarti.{RST}\n")

    punya_imu = mag.baca_imu() is not None
    if not punya_imu:
        print(f"{YELLOW}Topik /robotis/open_cr/imu diam -- kemiringan tidak terukur.{RST}")
        print(f"{DIM}  Arus dan pose tetap bisa dipisahkan, tapi 'kemiringan' tidak"
              f" akan bisa dicoret{RST}")
        print(f"{DIM}  sebagai penyebab. Pengukuran diteruskan.{RST}\n")

    print(f"{BOLD}Pergeseran yaw antara servo mati dan servo hidup{RST}")
    print(f"{DIM}  Yang diukur adalah medan tambahan yang muncul bersama torque."
          f" Supaya medan itu{RST}")
    print(f"{DIM}  tidak tercampur dengan medan bumi, robot harus TETAP MENGHADAP"
          f" ARAH YANG SAMA{RST}")
    print(f"{DIM}  dari awal sampai akhir -- boleh disentuh, tidak boleh diputar"
          f" atau digeser.{RST}")
    print(f"{DIM}  Jauhkan juga tangan, laptop, dan ponsel dari badan robot selama"
          f" merekam.{RST}\n")

    fase = []

    print(f"{BOLD}A. Torque MATI{RST}")
    print(f"{DIM}   Biarkan robot seperti apa adanya sekarang.{RST}")
    input("   Enter kalau siap... ")
    a = ukur_keadaan(mag, args.dwell)
    if a is None:
        return 1
    fase.append(("A  torque OFF", a))

    print(f"\n{BOLD}B. Nyalakan torque{RST}")
    print(f"{DIM}   Pakai cara biasa Anda. Biarkan robot masuk ke pose berdirinya"
          f" dan diam dulu{RST}")
    print(f"{DIM}   sebentar sebelum Enter -- getaran sisa gerakan ikut terekam"
          f" kalau terburu-buru.{RST}")
    input("   Enter kalau torque sudah hidup dan robot sudah tenang... ")
    b = ukur_keadaan(mag, args.dwell)
    if b is None:
        return 1
    fase.append(("B  torque ON", b))

    c = None
    if not args.tanpa_beban:
        print(f"\n{BOLD}C. Torque tetap hidup, tambah beban{RST}")
        print(f"{DIM}   Tekan kedua bahu robot ke bawah PELAN dan tahan, secukupnya"
              f" saja sampai{RST}")
        print(f"{DIM}   terasa servo melawan. Posenya harus tetap sama -- yang"
              f" ditambah cuma arus{RST}")
        print(f"{DIM}   penahannya. Pegangi robot supaya tidak jatuh, dan jangan"
              f" sampai kakinya{RST}")
        print(f"{DIM}   bergeser di lantai.{RST}")
        print(f"{DIM}   (lewati fase ini dengan --tanpa-beban kalau robot belum"
              f" aman ditekan){RST}")
        input("   Enter sambil terus menekan... ")
        c = ukur_keadaan(mag, args.dwell)
        if c is None:
            return 1
        fase.append(("C  ON + ditekan", c))

    print(f"\n{BOLD}D. Matikan torque lagi{RST}")
    print(f"{DIM}   Jangan putar atau geser robot. Fase ini yang membuktikan"
          f" pengukuran di atas sah.{RST}")
    input("   Enter kalau torque sudah mati... ")
    d = ukur_keadaan(mag, args.dwell)
    if d is None:
        return 1
    fase.append(("D  torque OFF", d))

    return laporkan_torque(fase, a, b, c, d, punya_imu)


def laporkan_torque(fase, a, b, c, d, punya_imu):
    def kolom_kemiringan(p):
        if p["roll"] is None:
            return f"{'-':>8}{'-':>8}"
        return f"{p['roll']:>+8.1f}{p['pitch']:>+8.1f}"

    print(f"\n{BOLD}Yang terekam{RST}")
    print(f"  {'fase':<16}{'mag_x':>8}{'mag_y':>8}{'mag_z':>8}{'|mendatar|':>12}"
          f"{'heading':>9}{'yaw':>8}{'kemiringan':>16}")
    for nama, p in fase:
        rh = math.hypot(p["mx"], p["my"])
        print(f"  {nama:<16}{p['mx']:>+8.1f}{p['my']:>+8.1f}{p['mz']:>+8.1f}"
              f"{rh:>12.1f}{p['heading']:>9.1f}{p['yaw']:>+8.1f}"
              f"{kolom_kemiringan(p)}")

    rh_bumi = math.hypot(a["mx"], a["my"])

    # --- B - A : seluruh pergeserannya ------------------------------------
    dbx, dby, dbz = b["mx"] - a["mx"], b["my"] - a["my"], b["mz"] - a["mz"]
    dbh = math.hypot(dbx, dby)

    # Robot tidak diputar, jadi seluruh perubahan arah mendatar yang terbaca
    # pasti datang dari medan yang bertambah -- bukan dari robotnya. Angka ini
    # yang dibandingkan dengan yaw firmware: kalau keduanya sama, medan mendatar
    # menjelaskan segalanya dan kemiringan tidak ikut bermain.
    dyaw_ramal = wrap180(heading_from(b["mx"], b["my"]) - heading_from(a["mx"], a["my"]))
    dyaw_ukur = wrap180(b["yaw"] - a["yaw"])
    sisa = wrap180(dyaw_ukur - dyaw_ramal)

    print(f"\n{BOLD}Pergeseran saat torque dinyalakan (B - A){RST}")
    print(f"  medan tambahan, kerangka badan : x {dbx:+7.1f}  y {dby:+7.1f}"
          f"  z {dbz:+7.1f}  cacahan")
    banding = (f"   {DIM}({dbh / rh_bumi * 100:.0f}% dari medan bumi){RST}"
               if rh_bumi > 1e-6 else "")
    print(f"  besarnya di bidang mendatar    : {dbh:7.1f} cacahan"
          f" = {dbh * LSB_TO_UT:.1f} uT{banding}")
    print(f"  yaw yang terbaca bergeser      : {dyaw_ukur:+7.1f} derajat")
    print(f"  dijelaskan oleh medan mendatar : {dyaw_ramal:+7.1f} derajat")
    print(f"  sisa yang tidak terjelaskan    : {sisa:+7.1f} derajat")

    if punya_imu:
        droll = b["roll"] - a["roll"]
        dpitch = b["pitch"] - a["pitch"]
        dtilt = math.hypot(droll, dpitch)
        print(f"  kemiringan badan berubah       : {dtilt:7.1f} derajat"
              f"   {DIM}(komponen {droll:+.1f} / {dpitch:+.1f}){RST}")
    else:
        droll = dpitch = dtilt = None

    # --- C - B : bagian yang mengikuti arus -------------------------------
    if c is not None:
        dcx, dcy = c["mx"] - b["mx"], c["my"] - b["my"]
        dch = math.hypot(dcx, dcy)
        dyaw_beban = wrap180(c["yaw"] - b["yaw"])
        bagian_arus = dch / dbh if dbh > 1e-6 else 0.0

        print(f"\n{BOLD}Tambahan beban, pose tetap sama (C - B){RST}")
        print(f"{DIM}  Posenya tidak berubah di sini, jadi apa pun yang bergeser"
              f" datang dari arus.{RST}")
        print(f"  medan bertambah lagi           : {dch:7.1f} cacahan"
              f" = {dch * LSB_TO_UT:.1f} uT")
        print(f"  yaw bergeser lagi              : {dyaw_beban:+7.1f} derajat")
        if dbh > 8.0:
            print(f"  dibanding pergeseran B - A     : {bagian_arus * 100:7.0f}%")
        else:
            # Pembaginya sendiri masih sebesar derau; 1 dari 3 cacahan tetap
            # menghasilkan "33%" dan angka itu cuma menyesatkan.
            print(f"  dibanding pergeseran B - A     : {DIM}pergeseran dasarnya"
                  f" terlalu kecil untuk dibandingkan{RST}")
            bagian_arus = None
    else:
        dch = dyaw_beban = bagian_arus = None

    # --- D - A : apakah semuanya berulang ---------------------------------
    kembali = math.hypot(d["mx"] - a["mx"], d["my"] - a["my"])
    dyaw_kembali = wrap180(d["yaw"] - a["yaw"])
    print(f"\n{BOLD}Kembali ke keadaan awal (D - A){RST}")
    print(f"  medan meleset dari titik awal  : {kembali:7.1f} cacahan")
    print(f"  yaw meleset dari titik awal    : {dyaw_kembali:+7.1f} derajat")

    sah = abs(dyaw_kembali) <= max(3.0, 0.2 * abs(dyaw_ukur))
    if not sah:
        print(f"\n{RED}  D tidak kembali ke A.{RST} Selama itu belum beres, angka"
              f" di atas tidak bisa dipakai:")
        print(f"{DIM}    - robot tergeser atau terputar di tengah pengukuran, atau{RST}")
        print(f"{DIM}    - ada besi/ponsel/laptop yang ikut pindah di dekatnya, atau{RST}")
        print(f"{DIM}    - servo masih panas dan arus diamnya belum turun -- tunggu,"
              f" lalu ulangi.{RST}")
        return 1
    print(f"{GREEN}  Berulang dengan rapi -- pergeserannya memang datang dari torque.{RST}")

    # --- Kesimpulan -------------------------------------------------------
    print(f"\n{BOLD}Kesimpulan{RST}")

    if abs(dyaw_ukur) < 3.0:
        print(f"{GREEN}  Yaw praktis tidak bergeser ({dyaw_ukur:+.1f} derajat)."
              f" Tidak ada yang perlu dibetulkan.{RST}")
        return 0

    # Ambang 30% bukan angka kira-kira: derau magnetometer sekitar 13 cacahan
    # menyisakan beberapa derajat ketidakpastian pada tiap fase, jadi selisih
    # yang lebih kecil dari itu tidak bisa dibedakan dari deraunya sendiri.
    medan_menjelaskan = abs(sisa) <= max(3.0, 0.3 * abs(dyaw_ukur))

    if not medan_menjelaskan:
        print(f"{YELLOW}  Medan mendatar cuma menjelaskan {dyaw_ramal:+.1f} dari"
              f" {dyaw_ukur:+.1f} derajat.{RST}")
        if dtilt is not None and dtilt > 3.0:
            print(f"  Badan ikut berubah tegaknya {dtilt:.1f} derajat, dan mag_z"
                  f" sedang {b['mz']:+.0f} cacahan.")
            print(f"  Ini pola nomor 3: {BOLD}kemiringan bertemu offset Z yang"
                  f" keliru{RST}.")
            print(f"{DIM}    Kalibrasi ulang di bidang mendatar TIDAK akan"
                  f" memperbaikinya -- ia tidak{RST}")
            print(f"{DIM}    menyentuh sumbu Z sama sekali. Yang harus dibetulkan"
                  f" offset Z-nya:{RST}")
            print(f"      scripts/mag_calib.py write --x <x sekarang> --y <y sekarang> --z 0")
            print(f"{DIM}    lalu reset OpenCR dan jalankan 'torque' lagi. Kalau"
                  f" sisanya menyusut,{RST}")
            print(f"{DIM}    offset Z memang biangnya.{RST}")
        else:
            print(f"  Kemiringan tidak ikut berubah berarti, jadi bukan nomor 3"
                  f" juga.")
            print(f"{DIM}    Yang paling sering: robot tergeser sedikit, atau"
                  f" magnetometer sempat{RST}")
            print(f"{DIM}    jenuh. Ulangi sekali lagi sebelum mengambil"
                  f" kesimpulan.{RST}")
        return 0

    print(f"  Seluruh pergeseran dijelaskan oleh medan tambahan yang tetap di"
          f" kerangka badan.")
    print(f"  Bentuknya sama dengan hard-iron, jadi {BOLD}bisa disimpan sebagai"
          f" offset{RST}.")

    if bagian_arus is None:
        print(f"\n{DIM}  Fase beban dilewati, jadi arus dan pose belum terpisah."
              f" Jalankan lagi tanpa{RST}")
        print(f"{DIM}  --tanpa-beban untuk tahu seberapa jauh perbaikannya akan"
              f" bertahan.{RST}")
    elif bagian_arus > 0.25:
        print(f"\n{YELLOW}  Tapi {bagian_arus * 100:.0f}% dari pergeseran itu"
              f" mengikuti ARUS, bukan pose.{RST}")
        print(f"  Sekali ditekan saja yaw sudah bergeser lagi {dyaw_beban:+.1f}"
              f" derajat, padahal")
        print(f"  robot tidak berpindah pose. Berjalan menarik arus yang berubah"
              f" terus, jadi")
        print(f"  offset yang ditulis cuma benar pada satu titik beban dan sisa"
              f" galat sebesar")
        print(f"  {abs(dyaw_beban):.0f} derajat akan tetap ada saat robot"
              f" bergerak.")
        print(f"{DIM}    Menulis offset tetap masih sepadan -- {abs(dyaw_ukur):.0f}"
              f" derajat turun jadi sekitar{RST}")
        print(f"{DIM}    {abs(dyaw_beban):.0f} derajat. Kalau itu masih terlalu"
              f" besar, yang tersisa bukan pekerjaan{RST}")
        print(f"{DIM}    kalibrasi lagi melainkan kabel: pilin kabel daya DXL"
              f" berpasangan (pergi dan{RST}")
        print(f"{DIM}    pulang saling meniadakan medannya) dan jauhkan"
              f" lintasannya dari OpenCR.{RST}")
    elif bagian_arus < 0.10:
        print(f"\n{GREEN}  Arus hampir tidak berperan ({bagian_arus * 100:.0f}%)"
              f" -- yang menggeser adalah POSE.{RST}")
        print(f"  Pose berdiri robot berulang dengan rapi, jadi offset yang"
              f" ditulis akan bertahan")
        print(f"  selama posenya tidak berubah jauh.")
        print(f"{DIM}    Yang tetap akan menggeser: lengan mengayun saat berjalan,"
              f" dan kepala menoleh{RST}")
        print(f"{DIM}    saat memindai bola. Ukur ulang dengan 'torque' pada pose"
              f" berjalan kalau yaw{RST}")
        print(f"{DIM}    dipakai untuk menavigasi lapangan.{RST}")
    else:
        print(f"\n  Arus dan pose berbagi kira-kira rata"
              f" ({bagian_arus * 100:.0f}% arus).")
        print(f"  Offset akan memperbaiki sebagian besarnya; sisa sekitar"
              f" {abs(dyaw_beban):.0f} derajat")
        print(f"  ikut berubah mengikuti beban.")

    # --- Koreksi yang perlu ditulis ---------------------------------------
    #
    # Pemetaan sumbunya sama dengan 'record' dan 'steps', dan alasannya sama:
    # firmware mengurangi off[1] dari magADC[1] lalu memakainya sebagai Bx, dan
    # off[0] dari magADC[0] sebagai By. Jadi medan tambahan di sumbu X badan
    # dikoreksi lewat mag_y_offset, dan sebaliknya. Bukan salah ketik.
    delta_off_x = dby
    delta_off_y = dbx

    with open(FIT_FILE, "w") as f:
        f.write(f"delta_off_x {delta_off_x:.1f}\n")
        f.write(f"delta_off_y {delta_off_y:.1f}\n")
        f.write(f"center_bx {dbx:.1f}\n")
        f.write(f"center_by {dby:.1f}\n")
        f.write(f"radius {rh_bumi:.1f}\n")
        f.write(f"rms 0.0\n")
        f.write(f"waktu {time.strftime('%Y-%m-%d %H:%M:%S')} (torque ON - OFF)\n")

    print(f"\n{BOLD}Koreksi yang perlu ditambahkan ke offset yang berlaku sekarang{RST}")
    print(f"  mag_x_offset += {delta_off_x:+8.1f}")
    print(f"  mag_y_offset += {delta_off_y:+8.1f}")
    print(f"  mag_z_offset += {dbz:+8.1f}   {DIM}(cuma terasa saat badan miring){RST}")
    print(f"  Tersimpan di {FIT_FILE}.")

    # FIT_FILE tidak menyimpan koreksi Z: 'write' sengaja selalu menulis Z
    # secara eksplisit supaya hasil min/maks firmware yang keliru tidak pernah
    # ikut terbawa diam-diam. Jadi kalau Z memang bergeser, ia harus disebut
    # sendiri di baris perintahnya.
    if abs(dbz) > 8.0:
        print(f"\n{YELLOW}  Sumbu Z ikut bergeser {dbz:+.0f} cacahan.{RST}"
              f" 'write' menulis z = 0 kalau tidak diberi tahu,")
        print(f"  jadi tambahkan {BOLD}--z {int(round(dbz))}{RST} pada perintah di"
              f" bawah -- benar selama offset Z")
        print(f"  yang sekarang memang 0, dan itu yang dipakai 'write' sejak"
              f" awal.")

    print(f"\n{BOLD}Cara memakainya{RST}")
    print(f"  Koreksi di atas SELISIH, bukan nilai mutlak, jadi ia harus"
          f" ditambahkan ke offset")
    print(f"  yang sedang berlaku -- yang membuat yaw benar saat servo mati."
          f" Firmware tidak")
    print(f"  menerbitkan angka itu ke ROS, jadi hanya ada dua jalan:")
    print(f"\n  {BOLD}Kalau Anda tahu offset yang sekarang{RST} (misalnya karena"
          f" Anda sendiri yang menulisnya):")
    print(f"    scripts/mag_calib.py write --from-record --base-x <x> --base-y <y>")
    print(f"    (reset OpenCR, lalu 'check' dengan torque HIDUP)")
    print(f"\n  {BOLD}Kalau tidak tahu{RST}, jangan menebak -- ulangi seluruh"
          f" kalibrasi dalam keadaan")
    print(f"  operasional, dan hasilnya jadi mutlak:")
    print(f"    scripts/mag_calib.py write --x 0 --y 0 --z 0")
    print(f"    (reset OpenCR)")
    print(f"    {BOLD}nyalakan torque{RST}, biarkan robot berdiri")
    print(f"    scripts/mag_calib.py steps")
    print(f"    scripts/mag_calib.py write --from-record")
    print(f"    (reset OpenCR, nyalakan torque lagi)")
    print(f"    scripts/mag_calib.py zero    {DIM}# titik nol ikut bergeser,"
          f" nolkan ulang{RST}")
    print(f"    scripts/mag_calib.py check")
    print(f"\n{YELLOW}  Sesudahnya yaw akan benar saat torque HIDUP dan meleset"
          f" sekitar {abs(dyaw_ukur):.0f} derajat{RST}")
    print(f"{YELLOW}  saat torque mati. Itu memang yang diinginkan: robot"
          f" bekerja dengan torque hidup.{RST}")
    return 0


def cmd_write(mag, args):
    contoh = butuh_data(mag)
    if contoh is None:
        return 1

    if args.from_record:
        if not os.path.exists(FIT_FILE):
            print(f"{RED}Belum ada hasil rekaman di {FIT_FILE}.{RST}"
                  f" Jalankan 'record' dulu.")
            return 1
        nilai = {}
        with open(FIT_FILE) as f:
            for baris in f:
                bagian = baris.split()
                if len(bagian) == 2:
                    nilai[bagian[0]] = bagian[1]

        # 'record' menghasilkan KOREKSI, bukan nilai mutlak: yang diukur adalah
        # pergeseran pusat lingkaran pada data yang SUDAH dikurangi offset yang
        # sedang berlaku di firmware. Offset itu tidak diterbitkan ke ROS, jadi
        # nilainya harus datang dari operator.
        #
        # Cara paling pasti membuatnya nol, tanpa menebak:
        #   1. mag_calib.py write --x 0 --y 0 --z 0
        #   2. reset OpenCR (offset baru dibaca saat init)
        #   3. mag_calib.py record
        #   4. mag_calib.py write --from-record     <- basis 0 sekarang benar
        #   5. reset OpenCR lagi, lalu check
        off_x = int(round(args.base_x + float(nilai["delta_off_x"])))
        off_y = int(round(args.base_y + float(nilai["delta_off_y"])))
        off_z = args.z if args.z is not None else 0

        print(f"{DIM}Dari rekaman {nilai.get('waktu', '?')}:"
              f" koreksi x {float(nilai['delta_off_x']):+.1f},"
              f" y {float(nilai['delta_off_y']):+.1f}{RST}")
        print(f"{DIM}Offset dasar yang diasumsikan sedang berlaku:"
              f" x={args.base_x}, y={args.base_y}"
              f"{'  (bawaan)' if args.base_x == 0 and args.base_y == 0 else ''}{RST}")
        if args.base_x == 0 and args.base_y == 0:
            print(f"{YELLOW}  Asumsi ini benar HANYA kalau offset di EEPROM memang"
                  f" masih 0,0,0 --{RST}")
            print(f"{DIM}  yaitu kalau 'calibrate' belum pernah dijalankan, atau"
                  f" Anda baru saja menulis{RST}")
            print(f"{DIM}  nol lalu me-reset OpenCR. Kalau tidak, beri --base-x /"
                  f" --base-y yang sesuai,{RST}")
            print(f"{DIM}  atau tempuh urutan tulis-nol -> reset -> record ->"
                  f" write.{RST}")
    else:
        if args.x is None or args.y is None:
            print(f"{RED}Beri --x dan --y, atau pakai --from-record.{RST}")
            return 1
        off_x, off_y = args.x, args.y
        off_z = args.z if args.z is not None else 0

    for v, nama in ((off_x, "x"), (off_y, "y"), (off_z, "z")):
        if not -32768 <= v <= 32767:
            print(f"{RED}Offset {nama} di luar jangkauan int16: {v}{RST}")
            return 1

    print(f"\n{BOLD}Akan ditulis ke EEPROM OpenCR{RST}")
    print(f"  mag_x_offset  = {off_x:+6d}")
    print(f"  mag_y_offset  = {off_y:+6d}")
    print(f"  mag_z_offset  = {off_z:+6d}")
    print(f"  mag_cali_valid = 0x5A")

    if not args.yes:
        jawab = input(f"\n  Tulis sekarang? [y/N] ").strip().lower()
        if jawab not in ("y", "ya", "yes"):
            print("  Dibatalkan.")
            return 0

    # Satu item per siklus kontrol. robotis_controller menyimpan satu
    # GroupSyncWrite per port; item kedua yang datang dalam siklus yang sama
    # menimpa yang pertama dan hilang tanpa pesan error.
    # Tunggu robotis_controller benar-benar tersambung ke penerbit ini dulu.
    # publish() sebelum discovery selesai tidak menghasilkan error apa pun --
    # pesannya cuma lenyap, dan offset yang kita kira sudah tersimpan sebenarnya
    # tidak pernah sampai. Itu baru ketahuan jauh kemudian, setelah OpenCR
    # di-reset dan yaw ternyata tidak berubah sama sekali.
    batas = time.time() + 5.0
    while mag.sync_write.get_subscription_count() == 0 and time.time() < batas:
        time.sleep(0.05)
    if mag.sync_write.get_subscription_count() == 0:
        print(f"{RED}robotis_controller tidak berlangganan"
              f" /robotis/sync_write_item -- tulisan tidak akan sampai.{RST}")
        return 1

    daftar = (("mag_x_offset", off_x),
              ("mag_y_offset", off_y),
              ("mag_z_offset", off_z),
              ("mag_cali_valid", 0x5A))

    # Dua lintasan dengan nilai yang sama. Menulis ulang nilai identik tidak
    # berefek samping, dan itu murah dibanding satu item yang hilang karena
    # tabrakan siklus -- offset yang separuh tertulis menghasilkan yaw yang
    # salah dengan cara yang sulit dilacak.
    for lintasan in range(2):
        for nama, nilai in daftar:
            mag.tulis_item(nama, nilai)
            if lintasan == 0:
                print(f"  ditulis {nama} = {nilai}")
            time.sleep(0.4)

    print(f"\n{YELLOW}OpenCR harus di-reset supaya offset ini dipakai.{RST}")
    print(f"{DIM}  Firmware cuma membaca EEPROM sekali, saat init"
          f" (dxl_node_op3_init). Selama board{RST}")
    print(f"{DIM}  belum di-reset, yaw masih memakai offset yang lama walau EEPROM"
          f" sudah berubah.{RST}")
    print(f"  Tekan tombol reset di OpenCR (atau matikan-hidupkan robot), lalu:")
    print(f"    scripts/mag_calib.py check")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Kalibrasi yaw kompas OpenCR (ukur -> koreksi -> ukur lagi)")
    ap.add_argument("--domain", type=int, default=None,
                    help="ROS_DOMAIN_ID (stack robot ini memakai 1)")
    sub = ap.add_subparsers(dest="perintah")

    sub.add_parser("monitor", help="tampilkan data magnetometer terus-menerus")

    p = sub.add_parser("check", help="ukur galat yaw di arah fisik yang diketahui")
    p.add_argument("--points", type=int, default=4,
                   help="jumlah arah acuan (4 atau 8; 8 bisa memisahkan soft-iron)")

    p = sub.add_parser("record", help="rekam satu putaran, cocokkan lingkaran")
    p.add_argument("--seconds", type=float, default=25.0)

    p = sub.add_parser("steps",
                       help="putar-berhenti tiap 90 derajat (paling teliti)")
    p.add_argument("--points", type=int, default=4,
                   help="jumlah titik berhenti, harus genap (4 = tiap 90 derajat)")
    p.add_argument("--dwell", type=float, default=2.0,
                   help="lama diam di tiap titik, detik")

    p = sub.add_parser("torque",
                       help="ukur pergeseran yaw antara servo mati dan servo hidup")
    p.add_argument("--dwell", type=float, default=3.0,
                   help="lama merekam tiap fase, detik")
    p.add_argument("--tanpa-beban", dest="tanpa_beban", action="store_true",
                   help="lewati fase 'tekan bahu'; arus dan pose jadi tidak terpisah")

    sub.add_parser("calibrate", help="kalibrasi hard-iron oleh firmware (20 detik)")
    sub.add_parser("zero", help="jadikan arah sekarang sebagai yaw 0")

    p = sub.add_parser("write", help="tulis offset hard-iron ke EEPROM OpenCR")
    p.add_argument("--from-record", action="store_true",
                   help="pakai hasil 'record' terakhir")
    p.add_argument("--x", type=int, default=None)
    p.add_argument("--y", type=int, default=None)
    p.add_argument("--z", type=int, default=None,
                   help="bawaan 0 -- lihat catatan sumbu Z di kepala berkas")
    p.add_argument("--base-x", type=int, default=0,
                   help="offset yang sedang berlaku di firmware (bawaan 0);"
                        " 'record' cuma menghasilkan selisih terhadapnya")
    p.add_argument("--base-y", type=int, default=0)
    p.add_argument("--yes", action="store_true", help="jangan tanya konfirmasi")

    args = ap.parse_args()
    if not args.perintah:
        ap.print_help()
        return 1

    try:
        mag = MagNode(domain=args.domain)
    except ImportError as e:
        print(f"{RED}{e}{RST}")
        print(f"{DIM}Jalankan lewat entrypoint container supaya ROS ada di PATH:{RST}")
        print(f"  docker exec -it op3 /docker-entrypoint.sh bash")
        return 1

    fungsi = {
        "monitor": cmd_monitor,
        "check": cmd_check,
        "record": cmd_record,
        "steps": cmd_steps,
        "torque": cmd_torque,
        "calibrate": cmd_calibrate,
        "zero": cmd_zero,
        "write": cmd_write,
    }[args.perintah]

    try:
        return fungsi(mag, args)
    except KeyboardInterrupt:
        print("\nDihentikan.")
        return 130
    finally:
        mag.tutup()


if __name__ == "__main__":
    sys.exit(main())
