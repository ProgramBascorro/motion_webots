# Troubleshoot: OpenCR "Failed to turn on the Power of DXLs" Error

## Ringkasan Masalah

Ketika menjalankan `op3_action_editor`, muncul error:
```
[ERROR] Failed to turn on the Power of DXLs!
```
atau
```
[ERROR] Error Set port
```

Ini berarti aplikasi tidak bisa berkomunikasi dengan OpenCR board (sub-controller yang mengontrol servo motor).

---

## Penyebab Umum

### 1. **OpenCR Board Lama vs Baru**
- **Board Lama**: Firmware tidak responsive/incompatible dengan protokol ROS2 terbaru
- **Board Baru**: Sudah support protokol terbaru dengan baik

**Cara Cek**: Test dengan Dynamixel Wizard
- Jika Wizard bisa detect servo tapi ROS action_editor gagal → Board lama incompatible
- Jika Wizard juga gagal → Hardware rusak/tidak terhubung

---

### 2. **Device Port Tidak Konsisten**
- OpenCR bisa muncul di `/dev/ttyUSB0`, `/dev/ttyUSB1`, bahkan `/dev/ttyUSB9` tergantung kondisi hardware
- Code action_editor mungkin hardcoded ke `/dev/ttyUSB0` padahal device sebenarnya di port lain

**Cara Cek**:
```bash
ls -la /dev/ttyUSB*
```

---

### 3. **No Retry Logic**
- `action_editor` coba power on **hanya 1 kali**
- Jika komunikasi unstable/timeout, langsung error
- `op3_manager` punya **retry logic 5 kali**, jadi lebih robust

---

## Solusi

### **SOLUSI 1: Upgrade ke OpenCR Board Baru** ✅ RECOMMENDED
**Pro**: Paling reliable, sudah tested dan kompatibel
**Cons**: Harus ganti hardware

```
Jika OpenCR lama tidak bisa dihidupkan → discard dan gunakan board baru
```

---

### **SOLUSI 2: Add Retry Logic ke Action Editor** (Jika masih pakai board lama)

#### Step 1: Update `turnOnDynamixelPower()` function

Edit file: `src/ROBOTIS-OP3-Tools/op3_action_editor/src/main.cpp`

**Before:**
```cpp
bool turnOnDynamixelPower(rclcpp::Node::SharedPtr node, const std::string &device_name, const int &baud_rate)
{
  // ... setup port ...
  
  int _return = _packet_h->write1ByteTxRx(_port_h, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);
  
  if(_return != COMM_SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to turn on the Power of DXLs!");
    return false;  // ← Exit langsung kalau error
  }
  
  RCLCPP_INFO(node->get_logger(), "Power on DXLs!");
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  return true;
}
```

**After (dengan retry logic):**
```cpp
bool turnOnDynamixelPower(rclcpp::Node::SharedPtr node, const std::string &device_name, const int &baud_rate)
{
  // ... setup port ...
  
  int torque_on_count = 0;
  while (torque_on_count < 5)  // ← Coba 5 kali
  {
    int _return = _packet_h->write1ByteTxRx(_port_h, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);
    
    if(_return != 0)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to turn on Power (attempt %d/5) [%s]", 
                   torque_on_count + 1, _packet_h->getRxPacketError(_return));
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "Power on DXLs!");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      return true;  // ← Success, exit langsung
    }
    
    torque_on_count++;  // ← Increment counter untuk retry
  }
  
  RCLCPP_ERROR(node->get_logger(), "Failed to turn on the Power of DXLs after 5 attempts!");
  return false;
}
```

**Penjelasan**:
- Coba send power command **5 kali** (bukan 1 kali)
- Jika salah satu attempt berhasil → langsung return true
- Jika semua gagal → return false dengan error message
- Sama logic seperti `op3_manager` yang sudah proven working

---

### **SOLUSI 3: Auto-Detect Device Port**

Jika device tidak konsisten di `/dev/ttyUSB0`, tambah auto-detection:

#### Step 1: Tambah Function di main.cpp

```cpp
#include <cstdio>  // ← Tambah ini di atas

std::string findOpenCRDevice()
{
  for (int i = 0; i < 10; i++)
  {
    std::string device = "/dev/ttyUSB" + std::to_string(i);
    FILE *fp = fopen(device.c_str(), "r");
    if (fp != NULL)
    {
      fclose(fp);
      RCLCPP_INFO(..., "Found OpenCR at: %s", device.c_str());
      return device;
    }
  }
  return SUB_CONTROLLER_DEVICE;  // fallback ke default
}
```

#### Step 2: Gunakan function di main()

```cpp
// Sebelum turnOnDynamixelPower() dipanggil
std::string detected_device = findOpenCRDevice();
if (detected_device != SUB_CONTROLLER_DEVICE)
{
  _device_name = detected_device;
}

if(turnOnDynamixelPower(editor, _device_name, _baud_rate) == false)
  return 0;
```

**Penjelasan**:
- Cari `/dev/ttyUSB0` sampai `/dev/ttyUSB9`
- Gunakan device pertama yang ditemukan
- Tidak perlu hardcode satu port tertentu

---

## Langkah Testing Setelah Perubahan

```bash
# 1. Rebuild
cd /ros2_ws
colcon build --packages-select op3_action_editor

# 2. Source setup
source install/setup.bash

# 3. Test dengan verbose
ros2 run op3_action_editor executor.py

# 4. Cek log - seharusnya melihat:
# [INFO] Found OpenCR at: /dev/ttyUSB1
# [INFO] Power on DXLs!
```

---

## Checklist Debugging

Jika masih error, check ini dalam order:

- [ ] **Device ada?**
  ```bash
  ls -la /dev/ttyUSB*
  ```

- [ ] **User punya akses?**
  ```bash
  id | grep dialout
  # Harus ada "dialout" di output
  ```

- [ ] **Port tidak dipakai proses lain?**
  ```bash
  lsof /dev/ttyUSB1
  # Seharusnya "Port not in use" atau kosong
  ```

- [ ] **Board terhubung dengan benar?**
  ```bash
  # Test dengan Dynamixel Wizard atau Arduino IDE
  # Jika ini jalan, hardware OK
  ```

- [ ] **Code sudah di-rebuild?**
  ```bash
  colcon build --packages-select op3_action_editor
  source install/setup.bash
  ```

---

## Quick Reference Command

```bash
# Cek device
ls -la /dev/ttyUSB*

# Build
colcon build --packages-select op3_action_editor

# Test dengan explicit device (bypass auto-detect)
ros2 run op3_action_editor executor.py --ros-args -p device_name:=/dev/ttyUSB1

# Check permission
id | grep dialout

# Monitor port
lsof /dev/ttyUSB1
```

---

## Recommended Approach untuk Robot Baru

1. **Test Board terlebih dahulu** dengan Dynamixel Wizard sebelum pakai ROS
2. **Combine Solusi 2 + Solusi 3**: 
   - Add retry logic (lebih robust)
   - Add auto-detect (lebih flexible)
3. **Gunakan OpenCR board baru** (recommended untuk minimalize compatibility issues)
4. **Document device mapping** untuk project kamu (simpan note device mana yang connected)

---

## Referensi Code

- `op3_action_editor/src/main.cpp` - Yang dimodifikasi
- `op3_manager/src/op3_manager.cpp` - Reference untuk retry logic (line 258-274)
- Memory note: `reference_op3_motion_format.md`, `reference_container_serial_stale_node.md`
