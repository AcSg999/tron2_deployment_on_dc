"""Small WXYZ conversion helpers shared with the vision wire protocol."""
import numpy as np
from scipy.spatial.transform import Rotation


def quat_to_rotmat(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-3:
        raise ValueError("expected a unit WXYZ quaternion")
    return Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()


def rot_to_quat(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or not np.allclose(
            matrix.T @ matrix, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(matrix), 1):
        raise ValueError("expected a proper rotation matrix")
    return Rotation.from_matrix(matrix).as_quat()[[3, 0, 1, 2]]
