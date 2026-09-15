from copy import deepcopy
import json

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tron2_deployment.calibration import board_points
from tron2_deployment.calibration_diagnostics import (
    handeye_diagnostics, heldout_diagnostics, intrinsic_diagnostics,
)


def intrinsic():
    return {"K": [[600, 0, 320], [0, 610, 240], [0, 0, 1]], "dist": [0] * 5,
            "width": 640, "height": 480, "pattern": [9, 6], "square_m": .025,
            "rms_px": .2, "source": "mock", "verified": False}


def board_image(calibration):
    """Render a perspective checkerboard from independently specified geometry."""
    pattern = calibration["pattern"]
    texture = np.full(((pattern[1] + 1) * 80, (pattern[0] + 1) * 80), 255, np.uint8)
    for row in range(pattern[1] + 1):
        for col in range(pattern[0] + 1):
            texture[row * 80:(row + 1) * 80, col * 80:(col + 1) * 80] = 255 * ((row + col) % 2)
    h, w = texture.shape
    outer = np.array([[-1, -1, 0], [pattern[0], -1, 0],
                      [pattern[0], pattern[1], 0], [-1, pattern[1], 0]], np.float32) * calibration["square_m"]
    projected = cv2.projectPoints(outer, np.array([.22, -.12, .06]), np.array([-.105, -.07, .65]),
                                  np.array(calibration["K"], float), np.zeros(5))[0].reshape(4, 2)
    homography = cv2.getPerspectiveTransform(np.array([[0, 0], [w, 0], [w, h], [0, h]], np.float32), projected)
    return cv2.warpPerspective(texture, homography, (640, 480), borderValue=255)


def handeye_fixture():
    rng = np.random.default_rng(129)
    camera = np.eye(4)
    camera[:3, :3] = Rotation.from_euler("xyz", [.2, -.4, .12]).as_matrix()
    camera[:3, 3] = [.08, -.06, .9]
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler("xyz", [.12, -.04, .21]).as_matrix()
    mount[:3, 3] = [.03, .01, .06]
    samples = []
    for _ in range(9):
        wrist = np.eye(4)
        wrist[:3, :3] = Rotation.from_rotvec(rng.normal(size=3) * .55).as_matrix()
        wrist[:3, 3] = rng.uniform(-.2, .2, size=3)
        sample = {"source": "mock", "side": "left", "head_q2": [0, -.3],
                  "frame": {"camera_id": "synthetic", "source": "mock", "head_q2": [0, -.3]},
                  "pattern": [9, 6], "square_m": .025, "robot_gripper_to_base": wrist.tolist(),
                  "target_to_camera": (np.linalg.inv(camera) @ wrist @ mount).tolist()}
        samples.append(sample)
    solution = {"mode": "eye_to_hand", "reference_frame": "base", "source": "mock", "side": "left",
                "camera_id": "synthetic", "head_q2": [0, -.3], "camera_to_base": camera.tolist(),
                "samples": len(samples), "verified": False}
    return solution, samples, mount


def test_original_board_detection_reprojection_and_rejected_inputs_are_reported(tmp_path):
    calibration = intrinsic()
    board = tmp_path / "board.png"
    cv2.imwrite(str(board), board_image(calibration))
    blank = tmp_path / "blank.png"
    cv2.imwrite(str(blank), np.full((480, 640), 127, np.uint8))
    small = tmp_path / "small.png"
    cv2.imwrite(str(small), np.zeros((100, 100), np.uint8))
    broken = tmp_path / "invalid.png"
    broken.write_text("not an image")
    calibration["images"] = [str(board)]
    original = deepcopy(calibration)
    result = intrinsic_diagnostics(calibration, [board, blank, small, broken, tmp_path / "missing.png"])
    assert result["accepted_count"] == 1
    assert result["rejected_count"] == 4
    assert result["rms_px"] < .8
    assert result["max_error_px"] < 1.5
    view = result["views"][0]
    assert view["included_in_fit"] is True
    assert len(view["observed_xy_px"]) == 54
    assert len(view["image_sha256"]) == 64
    assert all(view["reason"] for view in result["views"][1:])
    assert 0 < result["coverage"]["convex_hull_fraction"] < .5
    assert 0 < result["coverage"]["occupied_cells"] < result["coverage"]["total_cells"]
    assert calibration == original
    json.dumps(result, allow_nan=False)


def test_intrinsic_evaluation_keeps_supplied_intrinsics_fixed(tmp_path, monkeypatch):
    calibration = intrinsic()
    calibration["dist"] = [.15, -.08, .004, -.005, .02]
    path = tmp_path / "geometry.png"
    cv2.imwrite(str(path), np.full((480, 640), 127, np.uint8))
    observed = cv2.projectPoints(board_points(), np.array([.6, -.5, .15]), np.array([-.09, -.03, .45]),
                                np.array(calibration["K"], float), np.array(calibration["dist"]))[0]
    monkeypatch.setattr("tron2_deployment.calibration_diagnostics.corners", lambda *_: observed)
    monkeypatch.setattr(cv2, "calibrateCamera", lambda *_args, **_kwargs: pytest.fail("intrinsics must not be refitted"))
    good = intrinsic_diagnostics(calibration, [path])
    assert good["rms_px"] < 1e-3
    wrong = deepcopy(calibration)
    wrong["K"][0][0] = 950
    wrong["K"][1][1] = 400
    bad = intrinsic_diagnostics(wrong, [path])
    assert bad["rms_px"] > .2
    assert bad["calibration"]["K"] == wrong["K"]
    predicted, seen = np.array(bad["views"][0]["predicted_xy_px"]), np.array(bad["views"][0]["observed_xy_px"])
    np.testing.assert_allclose(bad["views"][0]["residual_xy_px"], predicted - seen)


@pytest.mark.parametrize("field,value", [("K", [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                                         ("dist", [float("nan")] * 5), ("width", 0),
                                         ("pattern", [9, 0]), ("square_m", -1)])
def test_intrinsic_invalid_calibration_is_rejected(field, value):
    calibration = intrinsic()
    calibration[field] = value
    with pytest.raises(ValueError):
        intrinsic_diagnostics(calibration, [])


def test_no_accepted_images_has_no_fabricated_success(tmp_path):
    result = intrinsic_diagnostics(intrinsic(), [tmp_path / "missing.png"])
    assert result["accepted_count"] == 0
    assert result["rms_px"] is None and result["max_error_px"] is None
    assert result["coverage"]["occupied_fraction"] == 0
    json.dumps(result, allow_nan=False)


def test_handeye_recovers_fixed_board_mount_without_mutating_or_refitting():
    solution, samples, mount = handeye_fixture()
    original = deepcopy((solution, samples))
    result = handeye_diagnostics(solution, samples, [f"pose {i}" for i in range(len(samples))])
    np.testing.assert_allclose(result["mean_target_to_wrist"], mount, atol=1e-12)
    assert result["translation_rms_mm"] < 1e-10
    assert result["rotation_rms_deg"] < 1e-10
    assert result["wrist_spread"]["max_pairwise_rotation_deg"] > 15
    assert result["wrist_spread"]["rotation_spread_at_least_15_deg"] is True
    assert result["samples"][0]["label"] == "pose 0"
    assert (solution, samples) == original
    json.dumps(result, allow_nan=False)


def test_bad_camera_transform_produces_separate_metric_and_angular_errors():
    solution, samples, _ = handeye_fixture()
    wrong = np.array(solution["camera_to_base"])
    wrong[:3, :3] = Rotation.from_euler("xyz", [.05, 0, 0]).as_matrix() @ wrong[:3, :3]
    wrong[:3, 3] += [.02, -.01, .01]
    solution["camera_to_base"] = wrong.tolist()
    result = handeye_diagnostics(solution, samples)
    assert result["translation_rms_mm"] > 5
    assert result["rotation_rms_deg"] > .2
    expected = np.sqrt(np.mean([sample["translation_error_mm"] ** 2 for sample in result["samples"]]))
    assert result["translation_rms_mm"] == pytest.approx(expected)


@pytest.mark.parametrize("mutation,match", [
    (lambda s, v: v[0].update(source="real"), "source"),
    (lambda s, v: v[0].update(side="right"), "side"),
    (lambda s, v: v[0]["frame"].update(camera_id="other"), "camera_id"),
    (lambda s, v: v[0]["frame"].update(source="real"), "source"),
    (lambda s, v: v[0].update(head_q2=[0, 0]), "stationary head"),
    (lambda s, v: v[0].update(square_m=.05), "square_m"),
    (lambda s, v: v[0].update(pattern=[8, 6]), "pattern"),
    (lambda s, v: s.update(mode="eye_in_hand"), "eye_to_hand"),
    (lambda s, v: v[0]["robot_gripper_to_base"][0].__setitem__(0, 2), "rigid"),
    (lambda s, v: s.update(camera_to_reference=np.eye(4).tolist()), "aliases"),
    (lambda s, v: s.update(base_to_camera=np.eye(4).tolist()), "inverse aliases"),
    (lambda s, v: v[0]["frame"].update(width=640), "provenance is incomplete"),
])
def test_handeye_rejects_incompatible_provenance_or_transforms(mutation, match):
    solution, samples, _ = handeye_fixture()
    mutation(solution, samples)
    with pytest.raises(ValueError, match=match):
        handeye_diagnostics(solution, samples)


def test_handeye_transform_only_and_insufficient_excitation_are_explicit():
    solution, samples, _ = handeye_fixture()
    summary = handeye_diagnostics(solution, [], [])
    assert summary["sample_count"] == 0
    assert summary["translation_rms_mm"] is None
    assert summary["wrist_spread"] is None
    assert any("No hand-eye sample" in text for text in summary["notes"])
    json.dumps(summary, allow_nan=False)
    result = handeye_diagnostics(solution, [samples[0]] * 3)
    assert result["wrist_spread"]["rotation_spread_at_least_15_deg"] is False
    assert any("below 15 degrees" in text for text in result["notes"])
    with pytest.raises(ValueError, match="at least three"):
        handeye_diagnostics(solution, samples[:2])


def test_handeye_rejects_changed_intrinsics_between_sample_pnp_poses():
    solution, samples, _ = handeye_fixture()
    for sample in samples:
        sample["frame"]["intrinsics"] = intrinsic()["K"]
    samples[-1]["frame"]["intrinsics"][0][0] += 20
    with pytest.raises(ValueError, match="intrinsics must be consistent"):
        handeye_diagnostics(solution, samples)


def test_rotation_mean_stays_proper_for_rotations_with_negative_arithmetic_mean_determinant():
    solution, samples, _ = handeye_fixture()
    camera = np.array(solution["camera_to_base"])
    rotations = Rotation.from_euler("x", [0, 180, 180], degrees=True).as_matrix()
    rotations[2] = Rotation.from_euler("y", 180, degrees=True).as_matrix()
    assert np.linalg.det(rotations.mean(axis=0)) < 0
    for sample, rotation in zip(samples, rotations):
        target = np.eye(4)
        target[:3, :3] = rotation
        wrist = np.array(sample["robot_gripper_to_base"])
        sample["target_to_camera"] = (np.linalg.inv(camera) @ wrist @ target).tolist()
    result = handeye_diagnostics(solution, samples[:3])
    mean_rotation = np.array(result["mean_target_to_wrist"])[:3, :3]
    np.testing.assert_allclose(mean_rotation.T @ mean_rotation, np.eye(3), atol=1e-12)
    assert np.linalg.det(mean_rotation) == pytest.approx(1)


def test_heldout_exact_geometry_and_perturbed_transform_report_signed_mm():
    camera = np.eye(4)
    camera[:3, :3] = Rotation.from_euler("xyz", [.2, .1, -.4]).as_matrix()
    camera[:3, 3] = [.3, .2, -.1]
    points = np.array([[0, 0, 1], [.1, 0, 1], [0, .1, 1], [.1, .1, 1]])
    expected = points @ camera[:3, :3].T + camera[:3, 3]
    result = heldout_diagnostics(camera, points, expected, .005)
    assert result["passed"] is True
    assert result["rms_mm"] < 1e-10
    wrong = camera.copy()
    wrong[0, 3] += .012
    result = heldout_diagnostics(wrong, points, expected, .005)
    assert result["passed"] is False
    np.testing.assert_allclose(result["residual_base_mm"], np.tile([12, 0, 0], (4, 1)), atol=1e-10)
    np.testing.assert_allclose(result["error_mm"], [12] * 4)
    assert result["max_error_mm"] == pytest.approx(12)
    assert "verified" not in result and "allow_real" not in result
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("points,reference,tolerance", [
    (np.ones((3, 3)), np.ones((3, 3)), .01),
    (np.zeros((2, 3)), np.zeros((2, 3)), .01),
    (np.eye(3), np.eye(3), 0),
    (np.eye(3), np.eye(3), True),
    (np.eye(3), np.eye(3) * float("nan"), .01),
    (np.eye(3), np.ones((3, 3)), .01),
])
def test_heldout_rejects_invalid_or_degenerate_input(points, reference, tolerance):
    with pytest.raises(ValueError):
        heldout_diagnostics(np.eye(4), points, reference, tolerance)
