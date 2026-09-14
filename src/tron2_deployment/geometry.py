"""Explicit, finite SE(3) conversions; poses use xyz + scalar-first quaternion."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def unit(value, name: str) -> np.ndarray:
    result = vector(value, 3, name)
    length = np.linalg.norm(result)
    if length < 1e-9:
        raise ValueError(f"{name} must be nonzero")
    return result / length


def pose_matrix(pose) -> np.ndarray:
    pose = vector(pose, 7, "pose7")
    if abs(np.linalg.norm(pose[3:]) - 1) > 1e-4:
        raise ValueError("pose7 quaternion must be normalized")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def matrix_pose(transform) -> list[float]:
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("transform must be a finite 4x4 matrix")
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    if quaternion[3] < 0:
        quaternion *= -1
    return np.r_[transform[:3, 3], quaternion[[3, 0, 1, 2]]].tolist()


def rotation_error(target, actual) -> np.ndarray:
    return Rotation.from_matrix(np.asarray(target) @ np.asarray(actual).T).as_rotvec()


def interpolate_transform(start, end, fraction: float) -> np.ndarray:
    result = np.eye(4)
    result[:3, 3] = start[:3, 3] * (1 - fraction) + end[:3, 3] * fraction
    delta = Rotation.from_matrix(end[:3, :3] @ start[:3, :3].T).as_rotvec()
    result[:3, :3] = Rotation.from_rotvec(delta * fraction).as_matrix() @ start[:3, :3]
    return result
