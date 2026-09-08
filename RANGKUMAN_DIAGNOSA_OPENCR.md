# Rangkuman Lengkap: Diagnosa & Recovery OpenCR (Robot OP3)

> Dokumen ini merangkum seluruh proses dari awal sampai akhir, dengan bahasa
> sederhana. Tujuannya: siapa pun bisa membaca dan mengerti apa yang rusak,
> bagaimana kami tahu, dan apa solusinya.
>
> Tanggal: 8–9 Juli 2026

---

## 1. Kesimpulan Singkat (baca ini dulu)

- **OpenCR lama RUSAK di bagian antarmuka bus Dynamixel (jalur data ke servo).**
- **Bagian OpenCR lainnya SEHAT:** otak (chip STM32), USB, firmware, dan sensor
  gerak (IMU) semuanya masih hidup dan bekerja normal.
- Kerusakannya **dua arah**: OpenCR tidak bisa **mengirim** dan tidak bisa
  **menerima** data lewat bus Dynamixel.
- **Recovery software sudah TUNTAS** — masalahnya murni hardware. Tidak perlu
  flash/instal ulang lagi.
- **Solusi untuk robot jalan sekarang: pakai OpenCR baru** (sudah terbukti
  bekerja 100%).
- **Solusi untuk menyelamatkan board lama: ganti chip buffer/transceiver di
  jalur bus Dynamixel** (perbaikan tingkat komponen).

---

## 2. Latar Belakang: Apa Itu OpenCR & Bagaimana Robot Terhubung

**OpenCR** adalah papan kontrol kecil di robot (memakai chip STM32F746). Di robot
OP3 ini, OpenCR berperan sebagai salah satu "anggota" di jaringan servo, dengan
alamat **ID 200**. Tugasnya:

- Membaca **sensor gerak (IMU)**: gyro, akselerometer, sudut roll/pitch/yaw.
- Membaca **tombol**.
- Mengatur **buzzer/LED** dan **power** ke servo.

**Jalur komunikasi (topologi) robot:**

```
NUC (komputer)
   │  (kabel USB)
   ▼
U2D2  ← adapter USB↔Dynamixel (chip FTDI, terbaca /dev/ttyUSB0)
   │  (bus Dynamixel — TTL, Protocol 2.0, kecepatan 2 Mbps)
   ▼
┌──────────────────────────────────────────┐
│  Servo 1–18   +   OpenCR (sebagai ID 200) │
└──────────────────────────────────────────┘
```

**Poin penting yang sempat membingungkan:**

- **U2D2 tetap WAJIB.** U2D2 adalah pintu masuk data dari komputer ke SEMUA
  perangkat Dynamixel, termasuk OpenCR (ID 200). Tanpa U2D2, tidak ada yang bisa
  bicara ke bus.
- **Power OpenCR datang dari jack DC**, bukan dari micro-USB. Micro-USB OpenCR
  hanya untuk 5V logika, flashing firmware, dan diagnosa.

---

## 3. Gejala Masalah

Saat menjalankan robot / action editor, muncul error berulang:

```
Failed to turn on Power (attempt 1/5) ... 5/5
Failed to turn on the Power of DXLs after 5 attempts!
```

Artinya: perintah dikirim ke OpenCR (ID 200), tapi **OpenCR tidak pernah
menjawab**. Servo 1–18 di bus yang sama tetap normal — jadi masalahnya khusus di
OpenCR, bukan di bus atau U2D2.

> Catatan: pesan `[RxPacketError] Unknown error code` di kode itu menyesatkan.
> Arti sebenarnya adalah **timeout** (tidak ada jawaban), bukan error paket.

---

## 4. Langkah-Langkah yang Dikerjakan (Kronologis)

### Langkah 1 — Pasang Arduino IDE di NUC
Untuk bisa flash ulang firmware OpenCR, kami pasang Arduino IDE beserta:
- `libfuse2` (agar AppImage bisa jalan),
- user dimasukkan ke grup `dialout` + `plugdev` (agar bisa akses port serial),
- core board **OpenCR versi 1.5.3** terpasang.

### Langkah 2 — Pulihkan Bootloader lewat DFU (mode BOOT0)
Kami curiga awalnya bootloader rusak. Solusinya: masuk **mode DFU** (tahan tombol
BOOT + tekan Reset), lalu tulis ulang bootloader pakai `dfu-util`:

```bash
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D ./opencr_boot.bin
```

Hasil: **"File downloaded successfully"**. Setelah keluar DFU, muncul port
`/dev/ttyACM0` — tandanya bootloader OpenCR kembali sehat.

> DFU aman: tidak bisa membuat board "mati total", karena ROM bootloader STM32
> ada di memori sistem yang tidak bisa terhapus.

### Langkah 3 — Flash Firmware OP3
Kami unggah firmware OP3 (`opencr_op3` dari core 1.5.3) lewat Arduino IDE.
Hasil: **"Done uploading"** (bar hijau). Firmware ini yang membuat OpenCR menjadi
ID 200 di bus, dengan baud 2 Mbps.

**Tapi hasilnya sama saja** — robot tetap error "Failed to turn on Power".
Berarti masalahnya BUKAN di firmware. Kami lanjut menyelidiki.

### Langkah 4 — Buktikan Otak & Sensor OpenCR Masih Hidup (Konsol Debug USB)
Firmware OP3 punya **konsol debug** lewat USB. Kami kirim perintah:
- `m` → tampilkan menu,
- `l` → tampilkan isi tabel kontrol + data IMU.

Hasil bacaan langsung dari OpenCR:

| Yang dibaca         | Nilai                          | Artinya            |
|---------------------|--------------------------------|--------------------|
| Model               | 0x7400                         | Firmware OP3 benar |
| **ID**              | **200**                        | Alamat benar       |
| **Baud**            | **4 (= 2 Mbps)**               | Kecepatan benar    |
| Status Return Level | 2                              | Setting benar      |
| **IMU Acc_Z**       | ≈17616 (≈1g gravitasi)         | **Sensor HIDUP**   |
| Roll/Pitch          | bergerak saat board digerakkan | **Sensor HIDUP**   |

**Kesimpulan langkah ini:** MCU (otak), USB, firmware, dan IMU **semua SEHAT dan
berjalan**. Jadi kerusakan bukan di sana.

### Langkah 5 — Ping Bus Dynamixel (dari sisi U2D2)
Kami kirim "ping" ke ID 200 lewat U2D2 (jalur bus normal robot), pakai skrip
Protocol 2.0. Kami sapu **semua kecepatan** dari 9600 sampai 4 Mbps.

Hasil:
- **Servo 1–18: menjawab bersih** di 2 Mbps (bus & kabel terbukti tercolok).
- **OpenCR (ID 200): DIAM di SEMUA kecepatan.** Tidak menjawab sama sekali.

### Langkah 6 — Tes Lampu LED (Flood Test)
Ini tes penentu untuk tahu **arah kerusakan** (kirim saja, terima saja, atau
keduanya). Cara kerjanya:

- Firmware OpenCR menyalakan **LED RX** setiap kali menerima 1 byte dari bus.
- Kami "banjiri" OpenCR dengan ~13.000 ping ke ID 200 pada 2 Mbps.
- Kalau OpenCR menerima data, LED RX-nya PASTI berkedip.

Hasil: **LED OpenCR diam total** (yang menyala hanya LED di U2D2). Bahkan tidak
satu byte pun sampai ke OpenCR.

**Ini bukti kuat:** jalur **terima (RX)** OpenCR putus. Digabung dengan hasil
ping (jalur **kirim/TX** juga diam), maka **RX dan TX dua-duanya mati**.

---

## 5. Vonis Akhir: Apa yang Rusak

**Antarmuka bus Dynamixel (TTL) di OpenCR rusak dua arah (RX + TX).**

Bagian ini bertugas menghubungkan jalur data bus (yang dipakai bersama untuk
kirim & terima secara bergantian) ke chip STM32. Bagian inilah yang mati.
Semua bagian lain OpenCR (otak, USB, firmware, IMU) sehat total.

Ini juga sesuai dengan **tes tukar (swap test)** sebelumnya:
- OpenCR **LAMA** → ID 200 menjawab **0 dari 30** percobaan.
- OpenCR **BARU** → ID 200 menjawab **30 dari 30** percobaan ✅
  (dengan U2D2, bus, kabel, config, dan kode yang persis sama).

---

## 6. Cara Membedakan Jenis Kerusakan (Panduan Teknik)

Kalau ingin memperbaiki board lama, perlu tahu penyebab pastinya. Dua kemungkinan:

- **(A) Chip buffer/transceiver-nya mati total.**
- **(B) Sambungan dari bus ke chip putus / salah colok port.**

**Cara membedakan A vs B:**

1. **Multimeter — ukur tegangan diam (idle) di pin DATA:**
   - Bandingkan pin DATA konektor OpenCR vs pin DATA di servo.
   - **Sama** → jalur nyambung → kemungkinan **(A) chip mati**.
   - **Beda** → jalur putus → kemungkinan **(B) sambungan putus**.
2. **Cek tegangan suplai chip** (mis. 3.3V) — kalau tidak ada, chip memang tak
   bekerja.
3. **Cek continuity** (sambungan) dari pin konektor ke kaki chip pakai multimeter
   mode "beep".
4. **Osiloskop** (kalau ada) — lihat apakah ada sinyal data di konektor tapi
   hilang di jalur menuju STM32.

**Perbaikan:** ganti chip buffer/transceiver di jalur bus, dan periksa
konektor + jalur pin arah (DIR).

---

## 7. Rekomendasi

| Kebutuhan                    | Solusi                                                    |
|------------------------------|-----------------------------------------------------------|
| **Robot harus jalan sekarang** | Pakai **OpenCR baru** (sudah terbukti bekerja 30/30, IMU aktif). |
| **Selamatkan board lama**    | Perbaikan komponen: ganti chip buffer/transceiver bus TTL + cek jalur. |

Setelah pakai OpenCR baru: pastikan port di `OP3.robot` / executor menunjuk ke
U2D2 yang tercolok (alias stabil `/dev/ttyOP3`, atau `FT3WKHM6` / `/dev/ttyUSB0`).

---

## 8. Catatan Teknik yang Berguna (untuk lain waktu)

- **Baca kondisi OpenCR langsung lewat USB:** buka port CDC OpenCR, kirim `m`
  (menu) atau `l` (tabel kontrol + IMU). Cara ini melewati bus, jadi bagus untuk
  memastikan otak/firmware/IMU hidup meski bus mati.
- **Selalu tutup Dynamixel Wizard sebelum menjalankan robot.** Kalau Wizard
  terbuka, ia memegang port dan membuat pembacaan bus jadi kacau (servo cuma
  terbaca sebagian).
- **Device yang dicolok setelah container start tidak otomatis terlihat di
  dalam container.** Micro-USB OpenCR yang dicolok belakangan harus dibaca dari
  **host**, sedangkan ping bus dijalankan **di container** (karena
  `/dev/ttyUSB0` sudah di-map ke sana).
- **DFU recovery itu aman** dan tidak bisa membuat board mati permanen.

---

## 9. Ringkasan Satu Kalimat

> Otak OpenCR lama masih hidup, tapi "mulut dan telinganya" ke bus servo sudah
> mati total — jadi untuk robot jalan pakai OpenCR baru, dan board lama hanya
> bisa diselamatkan dengan mengganti chip di jalur bus Dynamixel.
