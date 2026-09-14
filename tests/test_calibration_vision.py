from copy import deepcopy
from pathlib import Path
import shutil

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tron2_deployment.camera import capture
from tron2_deployment.config import load_profile, profile_fingerprint, validate_profile
from tron2_deployment.calibration import solve_samples, validate_extrinsics
from tron2_deployment.vision import segment, estimate


def profile():
    return load_profile(Path(__file__).parents[1]/"configs/demo.json")


def test_mock_perception_binds_pose_to_current_frame_and_calibration():
    p = profile()
    frame = capture(p, mock=True)
    mask = segment(p, frame, {"type":"box", "xyxy":[100,100,300,300], "coordinates":"pixels"}, mock=True)
    observation = estimate(p, frame, mask, p["vision"]["mesh_id"], mock=True)
    assert mask["area_px"] == 40000
    assert observation["frame_ref"] == frame["frame_ref"]
    assert observation["mask_ref"] == mask["mask_ref"]
    assert observation["source"] == "mock"
    assert observation["profile_hash"] == profile_fingerprint(p)
    assert observation["pose7"] == p["demo"]["object_pose7_base"]
    with pytest.raises(ValueError, match="different capture"):
        estimate(p, capture(p, mock=True), mask, p["vision"]["mesh_id"], mock=True)
    with pytest.raises(ValueError, match="requires --mock"):
        capture(p)
    with pytest.raises(ValueError, match="source"):
        segment(p, frame, {"type":"box", "xyxy":[0,0,1,1]}, mock=False)
    with pytest.raises(ValueError, match="mesh_id"):
        estimate(p, frame, mask, "different-object", mock=True)


def test_profile_hash_portable_and_geometry_bound(tmp_path):
    p = profile()
    other = deepcopy(p)
    target = tmp_path/"same-model.xml"
    shutil.copyfile(p["robot"]["model_xml"], target)
    other["robot"]["model_xml"] = str(target)
    other["_profile_path"] = "different-machine"
    assert profile_fingerprint(p) == profile_fingerprint(other)
    other["calibration"]["camera_to_base"][0][3] = .01
    assert profile_fingerprint(p) != profile_fingerprint(other)
    other = deepcopy(p)
    other["camera"]["intrinsics"][0][0] = -1
    with pytest.raises(ValueError, match="focal"):
        validate_profile(other)


def test_handeye_recovers_synthetic_camera_and_rejects_moving_head():
    rng = np.random.default_rng(4)
    camera = np.eye(4)
    camera[:3,:3] = Rotation.from_euler("xyz", [.3,-.4,.1]).as_matrix()
    camera[:3,3] = [.2,-.1,.6]
    board_mount = np.eye(4)
    board_mount[:3,3] = [.02,.03,.1]
    samples=[]
    for _ in range(9):
        wrist=np.eye(4)
        wrist[:3,:3] = Rotation.from_rotvec(rng.normal(size=3)*.5).as_matrix()
        wrist[:3,3] = rng.uniform(-.2,.2,3)
        board=np.linalg.inv(camera) @ wrist @ board_mount
        samples.append({"side":"left", "source":"mock", "pattern":[9,6], "square_m":.025,
            "frame":{"camera_id":"synthetic"}, "head_q2":[0,0],
            "robot_gripper_to_base":wrist.tolist(), "target_to_camera":board.tolist()})
    result = solve_samples(samples)
    np.testing.assert_allclose(result["camera_to_base"], camera, atol=1e-5)
    assert result["verified"] is False
    samples[-1]["head_q2"] = [.1,0]
    with pytest.raises(ValueError, match="stationary head"):
        solve_samples(samples)


def test_independent_calibration_points_detect_bad_transform():
    points = np.array([[0,0,1],[.1,0,1],[0,.1,1],[.1,.1,1]])
    good = np.eye(4)
    assert validate_extrinsics(good, points, points, .001)["passed"]
    good[0,3] = .01
    assert not validate_extrinsics(good, points, points, .001)["passed"]
    with pytest.raises(ValueError, match="non-collinear"):
        validate_extrinsics(np.eye(4), np.ones((3,3)), np.ones((3,3)), .001)
