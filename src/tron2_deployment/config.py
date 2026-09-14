"""Explicit, portable calibration and deployment profile loading."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path

import numpy as np


def finite(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name} must be finite with shape {shape}")
    return array


def rigid(value, name):
    matrix = finite(value, (4, 4), name)
    if not np.allclose(matrix[3], [0, 0, 0, 1]) or not np.allclose(
            matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(
                np.linalg.det(matrix[:3, :3]), 1):
        raise ValueError(f"{name} must be a rigid transform")
    return matrix


def validate_profile(profile):
    if not isinstance(profile, dict) or profile.get("schema_version") != 1:
        raise ValueError("profile schema_version must be 1")
    if not profile.get("profile_id"):
        raise ValueError("profile_id is required")
    robot = profile["robot"]
    names = robot["arm_joint_names"]["left"] + robot["arm_joint_names"]["right"]
    if len(names) != 14 or len(set(names)) != 14:
        raise ValueError("robot needs 14 distinct arm joint names")
    head = robot["head_joint_names"]
    if len(head) != 2 or len(set(names + head)) != 16:
        raise ValueError("robot needs two distinct head joint names")
    lower = finite(robot["joint_lower"], (14,), "joint_lower")
    upper = finite(robot["joint_upper"], (14,), "joint_upper")
    if np.any(lower >= upper):
        raise ValueError("joint lower limits must be below upper limits")
    for key in ("velocity_limits", "acceleration_limits"):
        if np.any(finite(robot[key], (14,), key) <= 0):
            raise ValueError(f"{key} must be positive")
    for side in ("left", "right"):
        if not robot["wrist_bodies"].get(side):
            raise ValueError("both wrist bodies must be configured")
    calibration = profile["calibration"]
    if calibration.get("source") not in ("mock", "real"):
        raise ValueError("calibration source must be mock or real")
    if not isinstance(calibration.get("verified"), bool):
        raise ValueError("calibration.verified must be a boolean")
    rigid(calibration["camera_to_base"], "camera_to_base")
    finite(calibration["head_q2"], (2,), "calibrated head state")
    camera = profile["camera"]
    k = finite(camera["intrinsics"], (3, 3), "camera intrinsics")
    if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
        raise ValueError("camera intrinsics have invalid focal length or bottom row")
    if any(not isinstance(camera[key], int) or camera[key] <= 0 for key in ("width", "height")):
        raise ValueError("camera image dimensions must be positive integers")
    if camera.get("backend") not in ("mock", "ros", "bridge"):
        raise ValueError("camera backend must be mock, ros or bridge")
    if camera["backend"] != "mock":
        if (camera["width"], camera["height"]) != (640, 480):
            raise ValueError("TRON2 real RGB-D capture currently requires 640x480 color and depth")
        kd = finite(camera["depth_intrinsics"], (3, 3), "depth intrinsics")
        if kd[0, 0] <= 0 or kd[1, 1] <= 0 or not np.allclose(kd[2], [0, 0, 1]):
            raise ValueError("depth intrinsics have invalid focal length or bottom row")
        rigid(camera["depth_to_color"], "depth_to_color")
        if not camera.get("identity"):
            raise ValueError("camera physical identity is required")
    distortion = np.asarray(camera.get("distortion", [0]*5), dtype=float)
    if distortion.shape not in ((4,), (5,), (8,), (12,), (14,)) or not np.isfinite(distortion).all():
        raise ValueError("camera.distortion must contain 4, 5, 8, 12, or 14 finite coefficients")
    max_frame_age = float(camera.get("max_frame_age_s", 0.75))
    if not np.isfinite(max_frame_age) or max_frame_age <= 0:
        raise ValueError("camera.max_frame_age_s must be positive and finite")
    for key in ("table_z_m", "clearance_m", "object_radius_m"):
        number = profile["scene"][key]
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not np.isfinite(number):
            raise ValueError(f"scene.{key} must be finite")
    if profile["scene"]["clearance_m"] <= 0 or profile["scene"]["object_radius_m"] <= 0:
        raise ValueError("scene clearance and object radius must be positive")
    json.dumps(profile, allow_nan=False)
    return profile


def load_profile(path):
    path = Path(path).expanduser().resolve()
    profile = json.loads(path.read_text())
    for key in ("model_xml", "urdf"):
        value = profile.get("robot", {}).get(key)
        if value:
            resolved = Path(os.path.expandvars(value)).expanduser()
            if not resolved.is_absolute():
                resolved = path.parent / resolved
            profile["robot"][key] = str(resolved.resolve())
    profile["_profile_path"] = str(path)
    return validate_profile(profile)


def profile_fingerprint(profile):
    value = deepcopy(profile)
    value.pop("_profile_path", None)
    for key in ("model_xml", "urdf"):
        path = value.get("robot", {}).get(key)
        if path:
            value["robot"][key] = {"sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest()}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    return path
