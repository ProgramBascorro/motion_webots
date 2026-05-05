"""Utility: load rc_hl_kidsize.yaml flat rules into a dict.

Usage in a ROS2 node:
    rules_file = self.get_parameter("rules_file").value
    rules = load_rules(rules_file)
    self._max_hold = float(rules.get("max_ball_hold_sec", 5.0))
"""
import os
from typing import Any, Dict


def load_rules(path: str) -> Dict[str, Any]:
    """Load rc_hl_kidsize.yaml and return a flat dict of parameter values.

    The YAML has a single top-level namespace key (e.g. 'rc_hl_kidsize');
    this function strips that wrapper and returns the inner dict.
    Returns an empty dict on any error so callers can safely fall back to
    declare_parameter defaults.
    """
    if not path or not os.path.isfile(path):
        return {}
    try:
        import yaml  # type: ignore
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            return {}
        # Strip the namespace wrapper if present
        for val in data.values():
            if isinstance(val, dict):
                return val
        return data
    except Exception:  # noqa: BLE001
        return {}
