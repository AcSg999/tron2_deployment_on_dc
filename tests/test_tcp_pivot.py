import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scripts.solve_tcp_pivot import fit_pivot


def test_pivot_recovers_tip_from_distinct_wrist_orientations():
    tip = np.array([0.03, -0.02, 0.12])
    point = np.array([0.4, -0.1, 0.3])
    wrists = []
    for angles in ([0, 0, 0], [0.4, 0, 0], [0, -0.5, 0],
                   [0.1, 0.2, 0.6], [-0.3, 0.1, -0.4]):
        wrist = np.eye(4)
        wrist[:3, :3] = Rotation.from_euler("xyz", angles).as_matrix()
        wrist[:3, 3] = point - wrist[:3, :3] @ tip
        wrists.append(wrist)
    found_tip, found_point, errors, condition = fit_pivot(wrists)
    np.testing.assert_allclose(found_tip, tip, atol=1e-12)
    np.testing.assert_allclose(found_point, point, atol=1e-12)
    assert np.max(errors) < 1e-12
    assert condition < 1000


def test_pivot_rejects_unobservable_identical_orientations():
    with pytest.raises(ValueError, match="orientations are too similar"):
        fit_pivot([np.eye(4) for _ in range(4)])
