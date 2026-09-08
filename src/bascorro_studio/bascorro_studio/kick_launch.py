import math
import re
import xml.etree.ElementTree as ElementTree


KICK_LAUNCH_KEYS = (
    "kick_distance", "kick_pan_max_distance", "kick_camera_height",
    "kick_camera_pitch_offset_deg",
    "kick_ball_real_radius_m", "kick_fov_width_deg", "kick_image_width_px",
    "kick_head_tilt_sign", "kick_distance_from_radius", "kick_min_depression_deg",
    "kick_distance_filter_n", "kick_ready_count", "kick_ball_radius_px",
    "kick_ball_radius_margin", "kick_radius_ready_enable",
)


def validate_kick_values(values):
    for name in KICK_LAUNCH_KEYS:
        value = values.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name}: nilai numerik berhingga tidak tersedia")
    for name in ("kick_distance", "kick_pan_max_distance", "kick_ball_real_radius_m"):
        if values[name] <= 0:
            raise ValueError(f"{name}: harus lebih besar dari 0")
    if values["kick_camera_height"] <= values["kick_ball_real_radius_m"]:
        raise ValueError("tinggi kamera harus lebih besar dari jari-jari bola")
    if abs(values["kick_camera_pitch_offset_deg"]) > 60:
        raise ValueError("koreksi sudut kamera harus dalam -60..60 derajat")
    if not 0 < values["kick_fov_width_deg"] < 90:
        raise ValueError("setengah bukaan kamera harus antara 0 dan 90 derajat")
    if values["kick_head_tilt_sign"] not in (-1, 1):
        raise ValueError("tanda head tilt hanya -1 atau +1")
    if not 0 < values["kick_min_depression_deg"] < 89:
        raise ValueError("sudut tunduk minimum harus lebih dari 0 dan kurang dari 89")
    for name in ("kick_image_width_px", "kick_distance_filter_n", "kick_ready_count"):
        value = values[name]
        minimum = 0 if name == "kick_ready_count" else 1
        if not minimum <= value <= 2147483647 or int(value) != value:
            raise ValueError(f"{name}: bilangan bulat di luar rentang yang sah")
    if values["kick_ball_radius_px"] < 0 or not 0 <= values["kick_ball_radius_margin"] <= 0.9:
        raise ValueError("ambang atau margin radius tidak sah")
    for name in ("kick_distance_from_radius", "kick_radius_ready_enable"):
        if values[name] not in (0, 1):
            raise ValueError(f"{name}: hanya 0 atau 1")


def read_kick_launch_defaults(text):
    root = ElementTree.fromstring(text)
    values = {}
    for name in KICK_LAUNCH_KEYS:
        matches = [element for element in root.findall("arg") if element.get("name") == name]
        if len(matches) != 1:
            raise ValueError(f"arg {name} ditemukan {len(matches)} kali")
        values[name] = float(matches[0].attrib["default"])
    validate_kick_values(values)
    return values


def format_launch_float(value):
    if not math.isfinite(value):
        raise ValueError("nilai launch harus berhingga")
    text = f"{value:.10g}"
    mantissa, separator, exponent = text.partition("e")
    if "." not in mantissa:
        mantissa += ".0"
    return mantissa + separator + exponent


def rewrite_kick_launch_defaults(text, values):
    try:
        validate_kick_values(values)
        read_kick_launch_defaults(text)
    except (ValueError, KeyError, ElementTree.ParseError) as error:
        return text, [], str(error)
    updated = text
    changed = []
    for name in KICK_LAUNCH_KEYS:
        pattern = re.compile(r'(<arg\s+name="' + re.escape(name) + r'"\s+default=")([^"]*)(")')
        matches = list(pattern.finditer(updated))
        if len(matches) != 1:
            return text, [], f"arg {name}: format ambigu; tidak ada yang diubah"
        old_value = matches[0].group(2)
        new_value = format_launch_float(values[name])
        if float(old_value) == float(new_value):
            continue
        changed.append(f"{name} {old_value} -> {new_value}")
        updated = pattern.sub(lambda match: match.group(1) + new_value + match.group(3), updated, count=1)
    try:
        read_kick_launch_defaults(updated)
    except (ValueError, KeyError, ElementTree.ParseError) as error:
        return text, [], str(error)
    return updated, changed, ""
