import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tron2_deployment.geometry import matrix_pose, pose_matrix
from tron2_deployment.targets import build_targets


def setup_targets():
    settings = {"symmetry": "axial", "radius_m": 0.08, "axial_offset_m": 0,
        "standoff_m": 0.12, "lift_clearance_m": 0.05,
        "wrist_to_tcp_pose7": [0, 0, 0, 1, 0, 0, 0]}
    profile = {"pregrasp": {"left": copy.deepcopy(settings), "right": copy.deepcopy(settings)}}
    observation = {"pose7": [0, 0, 0.5, 1, 0, 0, 0], "reference_frame": "base_Link"}
    wrists = {"left": [-0.6, 0, 0.5, 1, 0, 0, 0], "right": [0.6, 0, 0.5, 1, 0, 0, 0]}
    return profile, observation, wrists


def test_axial_yaw_does_not_force_wrist_rotation():
    profile, observation, wrists = setup_targets()
    original = build_targets(profile, observation, wrists)
    rotation = np.eye(4)
    rotation[:3, :3] = Rotation.from_euler("z", 1.7).as_matrix()
    rotation[:3, 3] = [0, 0, 0.5]
    observation["pose7"] = matrix_pose(rotation)
    yawed = build_targets(profile, observation, wrists)
    for side in ("left", "right"):
        np.testing.assert_allclose(original[side]["wrist_pregrasp_pose7_base"], yawed[side]["wrist_pregrasp_pose7_base"], atol=1e-12)
    np.testing.assert_allclose(original["left"]["tcp_pregrasp_pose7_base"][:3], [-0.2, 0, 0.5])
    assert original["left"]["wrist_pregrasp_pose7_base"] != observation["pose7"]


def test_tilted_object_and_mount_transform_obey_approach_constraints():
    profile, observation, wrists = setup_targets()
    obj = np.eye(4)
    obj[:3, :3] = Rotation.from_euler("xyz", [0.3, 0.4, -0.2]).as_matrix()
    obj[:3, 3] = [0.2, 0.1, 0.5]
    observation["pose7"] = matrix_pose(obj)
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler("y", 0.4).as_matrix()
    mount[:3, 3] = [0.01, -0.02, 0.08]
    settings = profile["pregrasp"]["left"]
    settings.update(symmetry="none", anchor_object=[-0.08, 0, 0.02], outward_axis_object=[-1, 0, 0], wrist_to_tcp_pose7=matrix_pose(mount))
    target = build_targets(profile, observation, wrists, ("left",))["left"]
    tcp = pose_matrix(target["tcp_pregrasp_pose7_base"])
    wrist = pose_matrix(target["wrist_pregrasp_pose7_base"])
    np.testing.assert_allclose(wrist @ mount, tcp, atol=1e-12)
    np.testing.assert_allclose(tcp[:3, 3], (obj @ np.array([-0.2, 0, 0.02, 1]))[:3])
    np.testing.assert_allclose(tcp[:3, 2], obj[:3, 0])
    np.testing.assert_allclose(tcp[:3, 1], obj[:3, 2])


@pytest.mark.parametrize("change", [
    {"standoff_m": -0.01}, {"standoff_m": float("nan")}, {"radius_m": -1},
    {"wrist_to_tcp_pose7": [0, 0, 0, 2, 0, 0, 0]}, {"symmetry": "unknown"},
    {"azimuth_hint_base": [0, 0, 1]},
])
def test_invalid_task_geometry_fails(change):
    profile, observation, wrists = setup_targets()
    profile["pregrasp"]["left"].update(change)
    with pytest.raises(ValueError):
        build_targets(profile, observation, wrists)


def test_frame_and_empty_selection_rejected():
    profile, observation, wrists = setup_targets()
    with pytest.raises(ValueError):
        build_targets(profile, observation, wrists, ())
    observation["reference_frame"] = "camera"
    with pytest.raises(ValueError):
        build_targets(profile, observation, wrists)
