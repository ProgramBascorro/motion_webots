# Repository Guidelines

## Start here (baca sebelum menjalankan apa pun)

Repo ini dipakai di robot OP3 nyata, di dalam **container Docker**. Tiga hal yang
membatalkan sebagian besar asumsi ROS 2 biasa:

1. **Workspace hanya jalan di dalam container.** Host tidak punya root workspace-nya.
   `install/` berisi symlink per-berkas yang menunjuk ke sana, jadi di host
   `source install/setup.bash` tetap keluar dengan kode 0 tapi diam-diam melewatkan
   hampir semua paket, dan `ros2 run` akan bilang "package not found".
2. **Periksa dulu, jangan langsung build:** `bash scripts/op3_docker.sh doctor`
   memeriksa perangkat serial, kestabilan colokan, port yang diminta `OP3.robot`,
   akses dari dalam container, ROS di PATH, dan kebasian ABI.
3. **Pakai `--packages-above`, bukan `--packages-select`,** kalau header ikut berubah.
   `--packages-select` meninggalkan pemakainya memakai layout kelas lama, dan
   gejalanya kejam: node hang di 100% CPU **tanpa pesan error sama sekali**.

Robot ini bergerak. Perubahan pada modul motion bisa membuatnya jatuh atau
memanaskan servo — mintalah robot digantung/dipegang sebelum uji pertama.

### Dokumen yang harus dibaca sesuai tugasnya

| Tugas | Dokumen |
|---|---|
| **Apa pun — baca paling awal** | `CLAUDE_ROBOT_HANDOFF.md` |
| Parameter walking, tab Walking di Bascorro Studio | `PANDUAN_PARAMETER_WALKING.md` |
| Penurunan FK `x/y/z_offset` dari page 2 | `CLAUDE_ROBOT_HANDOFF.md` §1 dan §10 |

Pola kerja di kedua dokumen itu sama dan **wajib diikuti**: grep penanda dulu,
terapkan hanya yang belum ada, dan jangan menyalin nilai per-robot dari robot lain.

Tabel ini menyebut berkas yang ada **di branch ini**. Branch robot lain bisa punya
sebagian saja — periksa dengan `ls *.md` sebelum menyimpulkan hilang.

Bagian di bawah ini adalah konvensi umum ROS 2 dan tetap berlaku untuk hal-hal
yang tidak menyentuh hardware.

## Project Structure & Module Organization
The workspace follows a standard ROS 2 layout: `src/` houses all packages, `build/`, `install/`, and `log/` are colcon artifacts. Key domains include `src/ROBOTIS-OP3*` for motion, hardware, and demo nodes, `src/ROBOTIS-OP3-Simulations` for Webots/Gazebo launch files, `src/humanoid_navigation` for localization and footstep planners, and `src/walking_imu_to_odometry` for IMU-based walking control. Keep robotics-specific assets (meshes, worlds, configs) inside their package to preserve `rosdep` compatibility.

## Build, Test, and Development Commands
- `source /opt/ros/humble/setup.bash && colcon build --continue-on-error`: full build — **di dalam container**, lihat "Start here". Untuk build sebagian pakai `--packages-above`. Keep warnings clean because downstream launch files assume every package installs.
- `colcon test --packages-select humanoid_navigation walking_imu_to_odometry`: run unit and launch tests for targeted stacks before opening a PR.
- `ros2 launch op3_webots_ros2 robot_launch.py world:=worlds/op3_env.wbt`: start the Webots simulation to validate motion controllers.
After building, always `source install/setup.bash` in any new terminal before invoking ROS 2 CLI tools — **di dalam container**. Di host perintah itu berhasil tanpa keluhan tapi hampir tidak memuat apa pun, karena symlink-nya menunjuk ke root workspace container yang tidak ada di host.

## Coding Style & Naming Conventions
C++ nodes follow `ament_cmake` defaults: 2-space indentation, UpperCamelCase class names, and snake_case topics/services. Python uses PEP 8 with 4-space indents, type hints where practical, and lowercase_with_underscores modules (see `walking_step_planner.py`). Run `ament_uncrustify` or `ament_clang_format` for C++ and `ament_flake8` for Python packages via `colcon test --ctest-args -R lint`.

## Testing Guidelines
Prefer ROS 2 launch tests for integration-heavy packages; place them under each package’s `test/` directory and name files `<feature>_test.cpp` or `<feature>_test.py`. Keep sensor and gait logic covered with deterministic inputs (mock tf streams, recorded bagfiles). Use `colcon test` regularly and review `log/latest_test` for regressions; do not merge if failures remain.

## Commit & Pull Request Guidelines
Follow the existing Git history: short, imperative subjects (e.g., `add walking step planner`, `fix odom cov`). Reference the affected stack in the first word when feasible. Each PR should include: summary, testing evidence (`colcon test` output or simulation screenshots), linked tracking issue, and any required configuration changes. Tag reviewers owning the touched packages and note any ABI-impacting changes.

## Simulation & Configuration Tips
Store Webots worlds and controller configs inside `src/ROBOTIS-OP3-Simulations/op3_webots_ros2`. Document new parameters in the matching launch file and update `README.md` when a new simulation flow is introduced.
