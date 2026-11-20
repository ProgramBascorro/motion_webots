"""Aksi rl_* / act_* apply_node tanpa ROS: rclpy dan pesan diganti tiruan.

Memastikan bentuk balasan rl_* tetap sama dengan sebelum penyimpanan dipindah ke
training/trial_store.py (tab RL Training bergantung padanya), dan act_* sampai ke
ActionTrainer. Jalankan dari src/bascorro_studio:
  python3 -m pytest test/test_apply_node_training.py -q
"""

import json
import os
import shutil
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _stub_ros():
    if "rclpy" in sys.modules and not getattr(sys.modules["rclpy"], "__stub__", False):
        return
    rclpy = types.ModuleType("rclpy")
    rclpy.__stub__ = True
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {})
    rclpy.node = node
    ament = types.ModuleType("ament_index_python")
    packages = types.ModuleType("ament_index_python.packages")
    packages.get_package_share_directory = lambda name: "/nonexistent/" + name
    ament.packages = packages
    std_msgs = types.ModuleType("std_msgs")
    msg = types.ModuleType("std_msgs.msg")
    msg.String = msg.Float64MultiArray = type("Msg", (), {})
    std_msgs.msg = msg
    sys.modules.update({"rclpy": rclpy, "rclpy.node": node, "ament_index_python": ament,
                        "ament_index_python.packages": packages, "std_msgs": std_msgs, "std_msgs.msg": msg})


_stub_ros()
from bascorro_studio import apply_node  # noqa: E402

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CHRONUS_BIN = os.path.join(WS, "src/ROBOTIS-OP3/op3_action_module/data/motion_4095_CHRONUS.bin")


class _Logger:
    def error(self, text):
        pass

    info = warning = error


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv("OP3_WS", str(tmp_path))
    n = apply_node.ActionWebNode.__new__(apply_node.ActionWebNode)
    n.results = []
    n.publish_result = n.results.append
    n.get_logger = lambda: _Logger()
    n._busy = False
    n._action_trainer = None
    n.action_file = str(tmp_path / "chronus.bin")
    if os.path.isfile(CHRONUS_BIN):
        shutil.copyfile(CHRONUS_BIN, n.action_file)
    return n


def test_rl_balasan_sama_seperti_sebelumnya(node, tmp_path):
    node.handle_payload({"action": "rl_add_trial", "request_id": "r1",
                         "trial": {"id": "t1", "session": "s", "params": {"period_time": 0.7}}})
    added = node.results[-1]
    assert added["ok"] and added["action"] == "rl_add_trial" and added["request_id"] == "r1"
    assert added["trial"]["no"] == 1 and len(added["trials"]) == 1
    stored = json.loads((tmp_path / "rl_runs" / "studio" / "trials.json").read_text())
    assert stored["trials"][0]["id"] == "t1"

    node.handle_payload({"action": "rl_get"})
    got = node.results[-1]
    assert got["ok"] and got["path"].endswith("rl_runs/studio/trials.json") and len(got["trials"]) == 1

    node.handle_payload({"action": "rl_add_trial", "trial": {"id": "x"}})
    assert node.results[-1] == {"ok": False, "action": "rl_add_trial", "message": "trial harus berisi params",
                                "request_id": None}
    node.handle_payload({"action": "rl_delete_trial", "id": "nope"})
    assert node.results[-1]["message"] == "percobaan tidak ditemukan"
    node.handle_payload({"action": "rl_delete_trial", "id": "t1"})
    assert node.results[-1]["ok"] and node.results[-1]["trials"] == []


def test_act_terpisah_dari_rl(node, tmp_path):
    node.handle_payload({"action": "act_add_trial", "trial": {"id": "a", "session": "s", "values": {}}})
    assert node.results[-1]["ok"]
    assert (tmp_path / "rl_runs" / "action" / "trials.json").is_file()
    assert not (tmp_path / "rl_runs" / "studio" / "trials.json").exists()


def test_act_pages_dan_galat_tidak_mematikan_node(node):
    if not os.path.isfile(CHRONUS_BIN):
        pytest.skip("bin CHRONUS tidak ada")
    node.handle_payload({"action": "act_pages", "request_id": "p"})
    pages = node.results[-1]
    assert pages["ok"] and pages["request_id"] == "p" and pages["pages"][0]["name"] == "TEND_KANAN"
    node.handle_payload({"action": "act_page", "page": 999})
    assert node.results[-1]["ok"] is False and "di luar" in node.results[-1]["message"]
    node.handle_payload({"action": "act_bogus"})
    assert node.results[-1]["ok"] is False and node._busy is False
