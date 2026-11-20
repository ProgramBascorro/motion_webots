#!/usr/bin/env python3

import datetime
import json
import os
import shutil
import subprocess
import tempfile
import time
from typing import Dict, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from bascorro_studio import yaw_calibration as yawcal
from bascorro_studio.training.trial_store import TrialStore


def resolve_action_file() -> str:
    env_path = os.environ.get("OP3_ACTION_FILE", "").strip()
    if env_path:
        return env_path
    return get_package_share_directory("op3_action_module") + "/data/motion_4095_CHRONUS.bin"


def resolve_seed_file() -> str:
    env_path = os.environ.get("OP3_ACTION_FILE_SEED", "").strip()
    if env_path:
        return env_path
    return get_package_share_directory("op3_action_module") + "/data/motion_4095_CHRONUS.bin"


def resolve_robot_file() -> str:
    return get_package_share_directory("op3_manager") + "/config/OP3.robot"


def ensure_action_file(action_file: str, seed_file: str, logger) -> None:
    if os.path.isfile(action_file):
        return
    if not os.path.isfile(seed_file):
        logger.warning(f"Action seed file not found: {seed_file}")
        return
    os.makedirs(os.path.dirname(action_file), exist_ok=True)
    shutil.copyfile(seed_file, action_file)
    logger.info(f"Initialized action file: {action_file}")


def run_action_yaml(action_file: str, robot_file: str, args: list) -> Tuple[int, str, str]:
    cmd = [
        "ros2",
        "run",
        "op3_action_editor",
        "action_yaml.py",
        "--action-file",
        action_file,
        "--robot-file",
        robot_file,
    ] + args
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout, proc.stderr


class ActionWebNode(Node):
    def __init__(self) -> None:
        super().__init__("bascorro_studio_apply")

        self.action_file = resolve_action_file()
        self.seed_file = resolve_seed_file()
        self.robot_file = resolve_robot_file()
        # Directory that holds the action .bin files. The web "Bin File" panel can
        # list/select/delete bins here; self.action_file is the launch-active one
        # (the bin the running op3_action_module actually loaded) and is protected.
        self.data_dir = os.path.dirname(self.action_file)
        raw_backup = self.declare_parameter("create_backup", False).value
        if isinstance(raw_backup, bool):
            self.create_backup = raw_backup
        else:
            self.create_backup = str(raw_backup).strip().lower() in ("1", "true", "yes")

        ensure_action_file(self.action_file, self.seed_file, self.get_logger())

        self.result_pub = self.create_publisher(
            String, "/bascorro_studio/result", 10
        )
        self.legacy_result_pub = self.create_publisher(
            String, "/op3_action_web/result", 10
        )
        self.apply_sub = self.create_subscription(
            String,
            "/bascorro_studio/apply_yaml",
            self.handle_apply_yaml,
            10,
        )
        self.request_sub = self.create_subscription(
            String,
            "/bascorro_studio/request",
            self.handle_request,
            10,
        )
        self.create_subscription(
            String,
            "/op3_action_web/apply_yaml",
            self.handle_apply_yaml,
            10,
        )
        self.create_subscription(
            String,
            "/op3_action_web/request",
            self.handle_request,
            10,
        )

        #-- Kalibrasi yaw ---------------------------------------------------
        # Sampel mag_status disimpan dalam jendela bergulir supaya "rekam" bisa
        # langsung memakai beberapa detik terakhir: operator menahan robot diam
        # lalu menekan tombol, tanpa harus menunggu perekaman berjalan.
        self._mag_samples = []          # (waktu, heading, gyro, yaw_tampil)
        self._yaw_cal_active = False
        self.create_subscription(
            Float64MultiArray, "/robotis/open_cr/mag_status", self.handle_mag_status, 20
        )
        self.imu_command_pub = self.create_publisher(
            String, "/robotis/open_cr/imu_command", 10
        )

        self._action_trainer = None   # tab Action Training, dibuat saat pertama dipakai
        self._busy = False
        self.get_logger().info("Studio action node ready")
        self.get_logger().info(f"Action file: {self.action_file}")

    def handle_apply_yaml(self, msg: String) -> None:
        payload = {"action": "apply", "yaml": msg.data}
        self.handle_payload(payload)

    def handle_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"action": "apply", "yaml": msg.data}
        if not isinstance(payload, dict):
            payload = {"action": "apply", "yaml": msg.data}
        elif "action" not in payload:
            payload["action"] = "apply"
        self.handle_payload(payload)

    def handle_payload(self, payload: Dict[str, object]) -> None:
        if self._busy:
            self.publish_result(
                {
                    "ok": False,
                    "action": payload.get("action", "apply"),
                    "message": "Busy; try again shortly",
                }
            )
            return

        self._busy = True
        try:
            action = str(payload.get("action", "apply"))
            request_id = payload.get("request_id")
            target = payload.get("target")
            if action == "apply":
                self.apply_yaml(str(payload.get("yaml", "")), request_id, target)
            elif action == "export":
                pages = str(payload.get("pages", "used"))
                self.export_yaml(pages, request_id, target)
            elif action == "stat":
                # cheap query: which bin will be written and when it last changed,
                # so the UI can warn before overwriting it with a stale draft.
                path, err = self.resolve_target(target, must_exist=True)
                if err:
                    self.publish_result({"ok": False, "action": "stat", "message": err, "request_id": request_id})
                else:
                    self.publish_result(
                        {"ok": True, "action": "stat", "request_id": request_id, **self.file_info(path)}
                    )
            elif action == "list":
                result = self.list_bins()
                result["action"] = "list"
                result["request_id"] = request_id
                self.publish_result(result)
            elif action == "delete":
                self.delete_bin(target, request_id)
            elif action == "walking_versions_get":
                self.walking_versions_get(request_id)
            elif action == "walking_versions_put":
                self.walking_versions_put(payload.get("versions"), request_id)
            elif action == "yaw_calib_record":
                self.yaw_calib_record(payload.get("target"), payload.get("verify", False),
                                      request_id)
            elif action == "yaw_calib_get":
                self.yaw_calib_get(request_id)
            elif action == "yaw_calib_clear":
                self.yaw_calib_clear(payload.get("target"), payload.get("verify", False),
                                     request_id)
            elif action == "yaw_calib_apply":
                self.yaw_calib_apply(payload.get("source"), request_id)
            elif action == "yaw_calib_disable":
                self.yaw_calib_disable(request_id)
            elif action in ("rl_get", "rl_add_trial", "rl_delete_trial",
                            "act_get", "act_add_trial", "act_delete_trial"):
                self.trial_request(action, payload, request_id)
            elif action.startswith("act_"):
                self.action_train(action, payload, request_id)
            else:
                self.publish_result(
                    {
                        "ok": False,
                        "action": action,
                        "message": f"Unknown action: {action}",
                        "request_id": request_id,
                    }
                )
        finally:
            self._busy = False

    def apply_yaml(self, yaml_text: str, request_id=None, target=None) -> None:
        if not yaml_text.strip():
            self.publish_result(
                {
                    "ok": False,
                    "action": "apply",
                    "message": "YAML is empty",
                    "request_id": request_id,
                }
            )
            return

        action_file, err = self.resolve_target(target, must_exist=True)
        if err:
            self.publish_result(
                {"ok": False, "action": "apply", "message": err, "request_id": request_id}
            )
            return

        tmp_path = ""
        try:
            with tempfile.NamedTemporaryFile("w", delete=False, suffix=".yaml") as handle:
                handle.write(yaml_text)
                tmp_path = handle.name

            args = ["import", "--in", tmp_path]
            if not self.create_backup:
                args.append("--no-backup")

            code, stdout, stderr = run_action_yaml(action_file, self.robot_file, args)
            ok = code == 0
            if not ok:
                err_text = stderr.strip() or stdout.strip()
                if err_text:
                    self.get_logger().error(f"action_yaml import failed: {err_text}")
            self.publish_result(
                {
                    "ok": ok,
                    "action": "apply",
                    "message": "Applied YAML" if ok else "Failed to apply YAML",
                    "stdout": stdout.strip(),
                    "stderr": stderr.strip(),
                    "request_id": request_id,
                    **self.file_info(action_file),
                }
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.publish_result(
                {
                    "ok": False,
                    "action": "apply",
                    "message": f"Apply error: {exc}",
                    "request_id": request_id,
                }
            )
        finally:
            if tmp_path and os.path.isfile(tmp_path):
                os.unlink(tmp_path)

    def export_yaml(self, pages: str, request_id=None, target=None) -> None:
        action_file, err = self.resolve_target(target, must_exist=True)
        if err:
            self.publish_result(
                {"ok": False, "action": "export", "message": err, "request_id": request_id}
            )
            return

        tmp_path = ""
        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".yaml") as handle:
                tmp_path = handle.name

            args = ["export", "--out", tmp_path, "--pages", pages]
            code, stdout, stderr = run_action_yaml(action_file, self.robot_file, args)
            ok = code == 0
            if not ok:
                err_text = stderr.strip() or stdout.strip()
                if err_text:
                    self.get_logger().error(f"action_yaml export failed: {err_text}")
            yaml_text = ""
            if ok and os.path.isfile(tmp_path):
                with open(tmp_path, "r", encoding="utf-8") as handle:
                    yaml_text = handle.read()

            self.publish_result(
                {
                    "ok": ok,
                    "action": "export",
                    "message": "Exported YAML" if ok else "Failed to export YAML",
                    "stdout": stdout.strip(),
                    "stderr": stderr.strip(),
                    "yaml": yaml_text,
                    "request_id": request_id,
                    **self.file_info(action_file),
                }
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.publish_result(
                {
                    "ok": False,
                    "action": "export",
                    "message": f"Export error: {exc}",
                    "request_id": request_id,
                }
            )
        finally:
            if tmp_path and os.path.isfile(tmp_path):
                os.unlink(tmp_path)

    def resolve_target(self, target, must_exist=False) -> Tuple[str, str]:
        """Resolve a user-supplied bin name to a full path inside the data dir.

        Only a bare ``*.bin`` filename is allowed (no path separators, no ``..``),
        so the web cannot read/write/delete outside the action data directory.
        Returns ``(path, error)`` -- error is "" on success. When target is empty
        the launch-active action file is used.
        """
        if not target:
            return self.action_file, ""
        name = str(target)
        base = os.path.basename(name)
        if base != name or base in ("", ".", ".."):
            return "", "Invalid target name"
        if not base.endswith(".bin"):
            return "", "Target must be a .bin file"
        path = os.path.join(self.data_dir, base)
        if must_exist and not os.path.isfile(path):
            return "", f"Bin not found: {base}"
        return path, ""

    def list_bins(self) -> Dict[str, object]:
        """List the *.bin files in the action data dir for the Bin File picker.
        Marks the launch-active bin so the UI can protect it from deletion."""
        active = os.path.basename(self.action_file)
        bins = []
        try:
            for name in sorted(os.listdir(self.data_dir)):
                if not name.endswith(".bin"):
                    continue
                path = os.path.join(self.data_dir, name)
                if not os.path.isfile(path):
                    continue
                try:
                    mtime = os.path.getmtime(path)
                    mtime_iso = datetime.datetime.fromtimestamp(mtime).isoformat(timespec="seconds")
                    size = os.path.getsize(path)
                except OSError:
                    mtime, mtime_iso, size = None, None, None
                bins.append(
                    {
                        "name": name,
                        "mtime": mtime,
                        "mtime_iso": mtime_iso,
                        "size": size,
                        "active": name == active,
                    }
                )
            return {"ok": True, "bins": bins, "active": active}
        except OSError as exc:
            return {"ok": False, "bins": [], "active": active, "message": f"List error: {exc}"}

    # --- Versi walking -------------------------------------------------------
    # Dulu daftar versi walking hanya hidup di localStorage peramban. Halaman ini
    # disajikan vite dengan --host 0.0.0.0, jadi ia bisa dibuka lewat
    # http://localhost:5173 MAUPUN http://<ip-robot>:5173 -- dan localStorage
    # dipisahkan per origin. Membuka lewat alamat yang berbeda, dari perangkat
    # lain, atau dari jendela penyamaran, membuat semua versi tersimpan seolah
    # hilang. Karena itu robot yang menyimpannya sekarang: satu berkas, sama dari
    # peramban mana pun.
    #
    # Disimpan di data_dir (tempat .bin action) karena direktori itu ter-bind-mount
    # ke repo, jadi ia bertahan walau kontainer dibuat ulang. list_bins() menyaring
    # "*.bin" saja, jadi berkas .json ini tidak akan muncul di picker Bin File.
    #== Kalibrasi yaw =======================================================
    # Perekaman dan perhitungan sengaja dipisah, persis seperti diminta operator:
    # titik direkam satu per satu dulu (yaw_calib_record), pemetaan baru dihitung
    # setelah kelima titik lengkap dan operator menekan Terapkan
    # (yaw_calib_apply). Tidak ada nilai yang diandaikan atau diubah di antara
    # keduanya -- yang disimpan adalah apa yang benar-benar dilaporkan sensor.

    MAG_WINDOW_SEC = 3.0        # jendela sampel bergulir yang dipakai "rekam"
    RECORD_MIN_SAMPLES = 15     # di bawah ini rekaman ditolak, bukan dipaksakan

    def handle_mag_status(self, msg: Float64MultiArray) -> None:
        # Urutan dari open_cr_module::publishMagStatus():
        #   0 status, 1 heading_deg, 2 yaw_mag_deg, 3 yaw_gyro_deg,
        #   4..6 mag_xyz, 7 yaw_gyro_rel_deg, 8 yaw_cal_active
        if len(msg.data) < 4:
            return
        now = time.monotonic()
        # Elemen 9 = yaw gyro TANPA hanyutan (open_cr_module mengestimasi bias
        # gyro). Kalau ada, itu yang direkam: rekaman 5 titik makan waktu menit,
        # dan hanyutan +4,3 deg/menit terukur mencemari bentuk kurvanya.
        gyro = msg.data[9] if len(msg.data) >= 10 else msg.data[3]
        self._mag_samples.append((now, msg.data[1], gyro, msg.data[2]))
        if len(msg.data) >= 9:
            self._yaw_cal_active = msg.data[8] >= 0.5
        cutoff = now - self.MAG_WINDOW_SEC
        while self._mag_samples and self._mag_samples[0][0] < cutoff:
            self._mag_samples.pop(0)

    def yaw_calib_path(self) -> str:
        # Bersebelahan dengan walking_versions.json: data_dir ter-bind-mount ke
        # repo, jadi rekaman bertahan walau kontainer dibuat ulang.
        return os.path.join(self.data_dir, "yaw_calibration_points.json")

    def yaw_calib_load(self) -> Dict[str, dict]:
        path = self.yaw_calib_path()
        if not os.path.exists(path):
            return {"points": {}, "verify": {}, "source": yawcal.DEFAULT_SOURCE}
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return {"points": {}, "verify": {}, "source": yawcal.DEFAULT_SOURCE}
        if not isinstance(data, dict):
            return {"points": {}, "verify": {}, "source": yawcal.DEFAULT_SOURCE}
        data.setdefault("points", {})
        data.setdefault("verify", {})
        data.setdefault("source", yawcal.DEFAULT_SOURCE)
        return data

    def yaw_calib_save(self, data: Dict[str, dict]) -> None:
        path = self.yaw_calib_path()
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
        os.replace(tmp, path)

    def yaw_calib_state(self, request_id=None, action="yaw_calib_get", extra=None) -> dict:
        """Kumpulkan keadaan lengkap: rekaman, pemetaan kedua sumber, tabel."""
        data = self.yaw_calib_load()
        points = {int(k): v for k, v in data["points"].items()}
        verify = {int(k): v for k, v in data["verify"].items()}

        # Pemetaan dihitung untuk KEDUA sumber supaya operator bisa memilih
        # dengan angka, bukan dengan anggapan. Yang gagal dilaporkan alasannya.
        mappings, errors = {}, {}
        for src in yawcal.SOURCES:
            try:
                mappings[src] = yawcal.compute_mapping(yawcal.raw_values(points, src), src)
            except yawcal.CalibrationError as exc:
                errors[src] = str(exc)

        chosen = data.get("source", yawcal.DEFAULT_SOURCE)
        cal = mappings.get(chosen)
        result = {
            "ok": True,
            "action": action,
            "request_id": request_id,
            "points": {str(k): v for k, v in points.items()},
            "verify": {str(k): v for k, v in verify.items()},
            "recorded": sorted(points.keys()),
            "missing": [t for t in yawcal.TARGETS if t not in points],
            "source": chosen,
            "sources": list(yawcal.SOURCES),
            "mapping_ok": {s: (s in mappings) for s in yawcal.SOURCES},
            "mapping_error": errors,
            "back_gap_deg": {s: m["back_gap_deg"] for s, m in mappings.items()},
            "table": yawcal.build_table(points, cal, verify),
            "applied": self._yaw_cal_active,
            "file": self.yaw_calib_path(),
        }
        if extra:
            result.update(extra)
        return result

    def yaw_calib_get(self, request_id=None) -> None:
        # Dulu hanya dipanggil dari handle_payload tanpa pernah didefinisikan:
        # Studio memintanya setiap kali tersambung, AttributeError mematikan
        # apply_node, dan semua Save versi walking sesudahnya tidak ada yang
        # menerima -- nilai "tersimpan" di layar, hilang saat Studio dibuka lagi.
        self.publish_result(self.yaw_calib_state(request_id, "yaw_calib_get"))

    def yaw_calib_record(self, target, verify=False, request_id=None) -> None:
        action = "yaw_calib_record"
        try:
            target = int(target)
        except (TypeError, ValueError):
            target = None
        if target not in yawcal.TARGETS:
            self.publish_result({"ok": False, "action": action, "request_id": request_id,
                                 "message": f"Target tidak dikenal: {target}"})
            return

        samples = list(self._mag_samples)
        if len(samples) < self.RECORD_MIN_SAMPLES:
            self.publish_result({
                "ok": False, "action": action, "request_id": request_id,
                "message": (f"Hanya {len(samples)} sampel dalam {self.MAG_WINDOW_SEC:.0f} "
                            "detik terakhir. Pastikan op3_manager jalan dan "
                            "/robotis/open_cr/mag_status mengalir.")})
            return

        # Rerata MELINGKAR: heading kompas sering berada tepat di sambungan
        # 0/360, dan rerata biasa akan memberi angka di seberang lingkaran.
        heading, spread_h = yawcal.circular_mean_deg([s[1] for s in samples])
        gyro, spread_g = yawcal.circular_mean_deg([s[2] for s in samples])
        yaw_now, _ = yawcal.circular_mean_deg([s[3] for s in samples])

        record = {
            "heading": heading, "spread_heading": spread_h,
            "gyro": gyro, "spread_gyro": spread_g,
            "yaw_old": yawcal.wrap_pm180(yaw_now),
            "samples": len(samples),
            "recorded_at": datetime.datetime.now().isoformat(timespec="seconds"),
        }
        if verify:
            record["yaw_new"] = record.pop("yaw_old")

        data = self.yaw_calib_load()
        data["verify" if verify else "points"][str(target)] = record
        if not verify:
            # Rekaman baru membuat verifikasi lama tidak relevan.
            data["verify"] = {}
        self.yaw_calib_save(data)

        warn = []
        if spread_h > yawcal.SPREAD_WARN_DEG:
            warn.append(f"sebaran kompas {spread_h:.1f} deg")
        if spread_g > yawcal.SPREAD_WARN_DEG:
            warn.append(f"sebaran gyro {spread_g:.1f} deg")
        extra = {"recorded_target": target, "record": record, "verify_mode": verify}
        if warn:
            extra["message"] = ("Direkam apa adanya, tapi robot tampak bergerak: "
                                + ", ".join(warn))
        self.publish_result(self.yaw_calib_state(request_id, action, extra))

    def yaw_calib_clear(self, target=None, verify=False, request_id=None) -> None:
        data = self.yaw_calib_load()
        key = "verify" if verify else "points"
        if target is None:
            data[key] = {}
            if not verify:
                data["verify"] = {}
        else:
            data[key].pop(str(int(target)), None)
        self.yaw_calib_save(data)
        self.publish_result(self.yaw_calib_state(request_id, "yaw_calib_clear"))

    def yaw_calib_apply(self, source=None, request_id=None) -> None:
        action = "yaw_calib_apply"
        data = self.yaw_calib_load()
        source = source if source in yawcal.SOURCES else data.get("source", yawcal.DEFAULT_SOURCE)
        points = {int(k): v for k, v in data["points"].items()}
        try:
            cal = yawcal.compute_mapping(yawcal.raw_values(points, source), source)
        except yawcal.CalibrationError as exc:
            self.publish_result(self.yaw_calib_state(
                request_id, action, {"ok": False, "message": str(exc)}))
            return

        try:
            yawcal.write_calibration_txt(cal, yawcal.calibration_txt_path())
        except OSError as exc:
            self.publish_result(self.yaw_calib_state(
                request_id, action,
                {"ok": False, "message": f"Gagal menulis berkas kalibrasi: {exc}"}))
            return

        data["source"] = source
        data["verify"] = {}     # pemetaan berubah -> verifikasi lama basi
        self.yaw_calib_save(data)
        self.imu_command_pub.publish(String(data="reload_yaw_calibration"))
        self.publish_result(self.yaw_calib_state(request_id, action, {
            "message": (f"Kalibrasi diterapkan (sumber {source}). Putar robot ke tiap "
                        "titik lalu tekan Verifikasi untuk melihat galat sebenarnya."),
            "mapping": cal,
        }))

    def yaw_calib_disable(self, request_id=None) -> None:
        self.imu_command_pub.publish(String(data="clear_yaw_calibration"))
        self.publish_result(self.yaw_calib_state(
            request_id, "yaw_calib_disable",
            {"message": "Pemetaan dimatikan sementara; berkas dan rekaman tidak dihapus."}))

    def walking_versions_path(self) -> str:
        return os.path.join(self.data_dir, "walking_versions.json")

    def walking_versions_get(self, request_id=None) -> None:
        path = self.walking_versions_path()
        if not os.path.exists(path):
            self.publish_result(
                {"ok": True, "action": "walking_versions_get", "versions": [], "request_id": request_id}
            )
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                versions = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            self.publish_result(
                {
                    "ok": False,
                    "action": "walking_versions_get",
                    "message": f"Read error: {exc}",
                    "request_id": request_id,
                }
            )
            return
        if not isinstance(versions, list):
            versions = []
        self.publish_result(
            {"ok": True, "action": "walking_versions_get", "versions": versions, "request_id": request_id}
        )

    def walking_versions_put(self, versions, request_id=None) -> None:
        if not isinstance(versions, list):
            self.publish_result(
                {
                    "ok": False,
                    "action": "walking_versions_put",
                    "message": "versions harus berupa list",
                    "request_id": request_id,
                }
            )
            return
        path = self.walking_versions_path()
        # Tulis ke berkas sementara lalu ganti nama: kalau proses mati di tengah
        # tulis, daftar lama tetap utuh alih-alih jadi JSON terpotong yang
        # membuat SEMUA versi hilang -- persis kegagalan yang mau dihindari.
        tmp = path + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump(versions, handle, indent=2)
            os.replace(tmp, path)
        except OSError as exc:
            try:
                os.remove(tmp)
            except OSError:
                pass
            self.publish_result(
                {
                    "ok": False,
                    "action": "walking_versions_put",
                    "message": f"Write error: {exc}",
                    "request_id": request_id,
                }
            )
            return
        self.publish_result(
            {
                "ok": True,
                "action": "walking_versions_put",
                "count": len(versions),
                "request_id": request_id,
            }
        )

    #-- Training: tab "RL Training" dan "Action Training" di Studio ------------
    # Percobaan yang sudah dinilai operator disimpan di robot lewat
    # training/trial_store.py (bukan localStorage: dipisah per origin, lihat
    # walking_versions): rl_* -> rl_runs/studio, act_* -> rl_runs/action.
    # Semua yang menyentuh .bin untuk Action Training ada di
    # training/action_trainer.py; robotnya dijalankan dari browser.
    TRIAL_STORES = {"rl": ("studio", "params"), "act": ("action", "values")}

    def training_dir(self, store: str) -> str:
        return os.path.join(os.environ.get("OP3_WS", "/ros2_ws"), "rl_runs", store)

    def trial_request(self, action: str, payload: Dict[str, object], request_id=None) -> None:
        prefix, _, verb = action.partition("_")
        folder, field = self.TRIAL_STORES[prefix]
        store = TrialStore(os.path.join(self.training_dir(folder), "trials.json"), field)
        try:
            if verb == "get":
                result = {"trials": store.load(), "path": store.path}
            elif verb == "add_trial":
                trial, trials = store.add(payload.get("trial"))
                result = {"trial": trial, "trials": trials}
            else:
                result = {"trials": store.delete(payload.get("id"))}
        except (ValueError, LookupError) as exc:
            self.publish_result({"ok": False, "action": action, "message": str(exc), "request_id": request_id})
            return
        except OSError as exc:
            self.publish_result({"ok": False, "action": action, "message": f"Write error: {exc}",
                                 "request_id": request_id})
            return
        self.publish_result({"ok": True, "action": action, "request_id": request_id, **result})

    def action_trainer(self):
        # Diimpor saat pertama dipakai: numpy tidak perlu dimuat kalau tab ini tidak dibuka.
        if self._action_trainer is None:
            from bascorro_studio.training.action_trainer import ActionTrainer
            ws = os.environ.get("OP3_WS", "/ros2_ws")
            self._action_trainer = ActionTrainer(
                self.action_file,
                op_demo_h=os.path.join(ws, "src/ROBOTIS-OP3-Demo/op3_demo/include/op3_demo/op_demo.h"),
                backup_dir=os.path.join(self.training_dir("action"), "backup"),
            )
        return self._action_trainer

    def action_train(self, action: str, payload: Dict[str, object], request_id=None) -> None:
        try:
            trainer = self.action_trainer()
            if action == "act_pages":
                result = trainer.page_list()
            elif action == "act_page":
                result = trainer.page_detail(payload.get("page"))
            elif action == "act_load":
                result = trainer.load(payload.get("page"), payload.get("sha"), payload.get("values") or {})
            elif action == "act_restore":
                result = trainer.restore()
            elif action == "act_metrics":
                result = trainer.trial_metrics(payload.get("samples") or [], payload.get("marks") or {},
                                               str(payload.get("profile") or "kick"))
            elif action == "act_deploy":
                result = trainer.deploy(payload.get("page"), payload.get("sha"), payload.get("values") or {},
                                        payload.get("target"), payload.get("name") or None,
                                        bool(payload.get("overwrite")))
            else:
                raise ValueError(f"Unknown action: {action}")
        except Exception as exc:   # satu permintaan buruk tidak boleh mematikan node
            self.get_logger().error(f"{action}: {exc}")
            self.publish_result({"ok": False, "action": action, "message": str(exc), "request_id": request_id})
            return
        self.publish_result({"ok": True, "action": action, "request_id": request_id, **result})

    def delete_bin(self, target, request_id=None) -> None:
        path, err = self.resolve_target(target, must_exist=True)
        if err:
            self.publish_result({"ok": False, "action": "delete", "message": err, "request_id": request_id})
            return
        if os.path.abspath(path) == os.path.abspath(self.action_file):
            self.publish_result(
                {
                    "ok": False,
                    "action": "delete",
                    "message": "Refusing to delete the active action file",
                    "request_id": request_id,
                }
            )
            return
        name = os.path.basename(path)
        try:
            os.remove(path)
            ok, message = True, f"Deleted {name}"
        except OSError as exc:
            ok, message = False, f"Delete error: {exc}"
        result = {"ok": ok, "action": "delete", "message": message, "request_id": request_id}
        # include a refreshed bin list so the dropdown updates immediately
        listing = self.list_bins()
        result["bins"] = listing.get("bins", [])
        result["active"] = listing.get("active")
        self.publish_result(result)

    def file_info(self, path=None) -> Dict[str, object]:
        """Target bin path + when it was last modified on disk, so the UI can show
        'this bin was last updated at X' and warn if a draft is older than that."""
        if not path:
            path = self.action_file
        info: Dict[str, object] = {
            "action_file": path,
            "action_file_name": os.path.basename(path),
        }
        try:
            mtime = os.path.getmtime(path)
            info["mtime"] = mtime  # epoch seconds
            info["mtime_iso"] = datetime.datetime.fromtimestamp(mtime).isoformat(timespec="seconds")
        except OSError:
            info["mtime"] = None
            info["mtime_iso"] = None
        return info

    def publish_result(self, payload: Dict[str, object]) -> None:
        # always attach current bin file info so the UI knows the on-disk state
        if "action_file" not in payload:
            payload = {**payload, **self.file_info()}
        msg = String()
        msg.data = json.dumps(payload)
        self.result_pub.publish(msg)
        self.legacy_result_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActionWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
