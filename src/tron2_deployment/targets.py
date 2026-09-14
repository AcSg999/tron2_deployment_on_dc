"""Choose object-aware standoff targets. No contact or gripper commands."""
from __future__ import annotations

import numpy as np

from .geometry import matrix_pose, pose_matrix, unit, vector


def build_targets(profile, observation, current_wrist_poses, sides=("left", "right")):
    if observation.get("reference_frame") != "base_Link":
        raise ValueError("observation must be calibrated into base_Link")
    object_transform = pose_matrix(observation["pose7"])
    selected = tuple(sides)
    if not selected or len(set(selected)) != len(selected) or any(s not in ("left", "right") for s in selected):
        raise ValueError("select left, right, or both once")
    result = {}
    for side in selected:
        settings = profile["pregrasp"][side]
        mount = pose_matrix(settings["wrist_to_tcp_pose7"])
        current = pose_matrix(current_wrist_poses[side]) @ mount
        standoff = float(settings["standoff_m"])
        lift = float(settings.get("lift_clearance_m", 0.05))
        if not np.isfinite([standoff, lift]).all() or standoff <= 0 or lift < 0:
            raise ValueError("standoff must be positive and lift clearance nonnegative")
        up = object_transform[:3, :3] @ unit(settings.get("up_axis_object", [0, 0, 1]), "up_axis_object")
        symmetry = settings.get("symmetry", "none")
        if symmetry == "axial":
            radial = vector(settings.get("azimuth_hint_base", current[:3, 3] - object_transform[:3, 3]), 3, "azimuth_hint_base")
            radial = unit(radial - up * np.dot(radial, up), "projected axial approach")
            radius = float(settings["radius_m"])
            height = float(settings["axial_offset_m"])
            if not np.isfinite([radius, height]).all() or radius < 0:
                raise ValueError("axial radius must be nonnegative and offset finite")
            outward = radial
            anchor = object_transform[:3, 3] + radial * radius + up * height
        elif symmetry == "none":
            anchor = object_transform[:3, 3] + object_transform[:3, :3] @ vector(settings["anchor_object"], 3, "anchor_object")
            outward = object_transform[:3, :3] @ unit(settings["outward_axis_object"], "outward_axis_object")
        else:
            raise ValueError("symmetry must be none or axial")
        # TCP +Z aims inward; roll is chosen using object up. For a pole approach,
        # preserve projected measured TCP +Y so an arbitrary object yaw cannot spin it.
        forward = -outward
        vertical = up - forward * np.dot(up, forward)
        if np.linalg.norm(vertical) < 1e-7:
            vertical = current[:3, 1] - forward * np.dot(current[:3, 1], forward)
        if np.linalg.norm(vertical) < 1e-7:
            vertical = current[:3, 0] - forward * np.dot(current[:3, 0], forward)
        vertical = unit(vertical, "TCP roll reference")
        horizontal = unit(np.cross(vertical, forward), "TCP horizontal axis")
        tcp = np.eye(4)
        tcp[:3, :3] = np.column_stack((horizontal, np.cross(forward, horizontal), forward))
        tcp[:3, 3] = anchor + standoff * outward
        wrist = tcp @ np.linalg.inv(mount)
        result[side] = {
            "tcp_pregrasp_pose7_base": matrix_pose(tcp),
            "wrist_pregrasp_pose7_base": matrix_pose(wrist),
            "anchor_base": anchor.tolist(), "outward_axis_base": outward.tolist(),
            "up_axis_base": up.tolist(), "symmetry": symmetry,
            "standoff_m": standoff, "lift_clearance_m": lift,
        }
    return result
