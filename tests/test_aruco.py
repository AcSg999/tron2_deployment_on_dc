import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tron2_deployment import aruco

K = np.array([[600.0, 0, 320], [0, 610, 240], [0, 0, 1]])
SIZE = (480, 640)


def marker_spec(**overrides):
    value = {"schema_version": 1, "kind": "marker", "dictionary": "DICT_6X6_250",
             "marker_length_m": 0.05, "marker_id": 23}
    value.update(overrides)
    return aruco.validate_target_spec(value)


def board_spec(**overrides):
    value = {"schema_version": 1, "kind": "board", "dictionary": "DICT_6X6_250",
             "marker_length_m": 0.04, "marker_separation_m": 0.012,
             "markers_x": 2, "markers_y": 3, "first_marker_id": 0}
    if "marker_ids" in overrides:
        value.pop("first_marker_id")
    value.update(overrides)
    return aruco.validate_target_spec(value)


def render_and_solve(spec, rotation, translation):
    image = aruco.render_target(spec, SIZE, rotation, translation, K)
    pose, detail = aruco.estimate_pose(image, spec, K, np.zeros(5))
    return pose, detail


def pose_error(pose, rotation, translation):
    translation_error = float(np.linalg.norm(pose[:3, 3] - np.asarray(translation)))
    rotation_error = float(np.degrees(
        Rotation.from_matrix(np.asarray(rotation).T @ pose[:3, :3]).magnitude()))
    return translation_error, rotation_error


def test_single_marker_pose_round_trip():
    spec = marker_spec()
    for translation in ([0.0, 0.0, 0.35], [0.06, -0.04, 0.5]):
        for tilt in ([25, -18, 8], [35, 12, -20]):
            rotation = Rotation.from_euler("xyz", tilt, degrees=True).as_matrix()
            pose, detail = render_and_solve(spec, rotation, translation)
            translation_error, rotation_error = pose_error(pose, rotation, translation)
            assert detail["visible_markers"] == 1
            assert detail["marker_ids"] == [23]
            assert detail["reprojection_rms_px"] < 1
            assert translation_error < 0.005
            assert rotation_error < 2


def test_board_pose_round_trip():
    spec = board_spec()
    for translation in ([0.0, 0.0, 0.35], [0.05, 0.02, 0.5]):
        for tilt in ([25, -18, 8], [12, 22, -14]):
            rotation = Rotation.from_euler("xyz", tilt, degrees=True).as_matrix()
            pose, detail = render_and_solve(spec, rotation, translation)
            translation_error, rotation_error = pose_error(pose, rotation, translation)
            assert detail["visible_markers"] >= 2
            assert detail["reprojection_rms_px"] < 1.5
            assert translation_error < 0.01
            assert rotation_error < 1.5


def test_board_uses_one_rigid_fit_not_per_marker_average():
    """A single marker elsewhere on the board must not be able to bias the fit."""
    spec = board_spec()
    rotation = Rotation.from_euler("xyz", [30, -20, 12], degrees=True).as_matrix()
    translation = np.array([0.0, 0.0, 0.4])
    image = aruco.render_target(spec, SIZE, rotation, translation, K)
    pose, detail = render_and_solve(spec, rotation, translation)
    assert detail["visible_markers"] == 6
    translation_error, rotation_error = pose_error(pose, rotation, translation)
    assert translation_error < 0.005 and rotation_error < 1
    assert len(detail["marker_ids"]) == 6
    assert sorted(detail["marker_ids"]) == spec["marker_ids"]
    assert image.shape == (480, 640, 3)


def test_frame_marker_choice_only_moves_the_documented_origin():
    board = board_spec()
    shifted = board_spec(frame_marker_id=5)
    rotation = Rotation.from_euler("xyz", [26, -16, 9], degrees=True).as_matrix()
    translation = np.array([0.0, 0.0, 0.4])
    image = aruco.render_target(board, SIZE, rotation, translation, K)
    pose_board, _ = aruco.estimate_pose(image, board, K, np.zeros(5))
    pose_shifted, _ = aruco.estimate_pose(image, shifted, K, np.zeros(5))
    pitch = shifted["marker_length_m"] + shifted["marker_separation_m"]
    expected = np.eye(4)
    # Marker 5 sits at column 1, row 2 of the layout, so the shifted target
    # origin becomes (pitch, 2*pitch) in the board target frame.
    expected[:3, 3] = [pitch, 2 * pitch, 0.0]
    np.testing.assert_allclose(pose_shifted, pose_board @ expected, atol=2e-3)


def test_ambiguous_planar_view_is_rejected():
    spec = marker_spec()
    rotation = Rotation.from_euler("xyz", [3, -2, 4], degrees=True).as_matrix()
    image = aruco.render_target(spec, SIZE, rotation, np.array([0.0, 0.0, 1.2]), K)
    with pytest.raises(ValueError, match="ambiguous"):
        aruco.estimate_pose(image, spec, K, np.zeros(5))


def test_stricter_solution_ratio_rejects_more_views():
    rotation = Rotation.from_euler("xyz", [10, -6, 5], degrees=True).as_matrix()
    translation = np.array([0.0, 0.0, 0.6])
    strict = marker_spec(min_solution_ratio=8.0)
    image = aruco.render_target(strict, SIZE, rotation, translation, K)
    with pytest.raises(ValueError, match="ambiguous"):
        aruco.estimate_pose(image, strict, K, np.zeros(5))


def test_min_visible_markers_is_enforced():
    spec = board_spec(min_visible_markers=2)
    single = marker_spec(marker_id=0)
    image = aruco.render_target(single, SIZE, np.eye(3), np.array([0.0, 0.0, 0.4]), K)
    found, order = aruco.detect_markers(image, spec)
    assert order == [0]
    with pytest.raises(ValueError, match="needs 2 visible"):
        aruco.estimate_pose(image, spec, K, np.zeros(5))


def test_unknown_markers_are_ignored():
    spec = marker_spec(marker_id=23)
    other = marker_spec(marker_id=7)
    image = aruco.render_target(other, SIZE, np.eye(3), np.array([0.0, 0.0, 0.4]), K)
    found, order = aruco.detect_markers(image, spec)
    assert found == {} and order == []
    with pytest.raises(ValueError, match="needs 1 visible"):
        aruco.estimate_pose(image, spec, K, np.zeros(5))


def test_empty_image_reports_no_detection():
    image = np.full((480, 640, 3), 200, np.uint8)
    assert aruco.detect_markers(image, board_spec()) == ({}, [])
    with pytest.raises(ValueError, match="visible"):
        aruco.estimate_pose(image, board_spec(), K, np.zeros(5))


def test_single_marker_spec_matches_one_cell_board():
    marker = marker_spec()
    one_cell = board_spec(marker_length_m=0.05, markers_x=1, markers_y=1,
                          marker_ids=[23], marker_separation_m=0.0)
    np.testing.assert_allclose(aruco.marker_object_points(marker, 23),
                               aruco.marker_object_points(one_cell, 23), atol=1e-7)
    assert one_cell["min_visible_markers"] == 1
    assert aruco.layout_extent(one_cell) == (0.05, 0.05)


def test_layout_geometry_is_row_major():
    spec = board_spec(markers_x=3, markers_y=2, marker_ids=[4, 5, 6, 7, 8, 9],
                      frame_marker_id=4)
    assert aruco.layout(spec) == {4: (0, 0), 5: (1, 0), 6: (2, 0),
                                  7: (0, 1), 8: (1, 1), 9: (2, 1)}
    pitch = spec["marker_length_m"] + spec["marker_separation_m"]
    np.testing.assert_allclose(aruco.marker_object_points(spec, 9)[0], [2 * pitch, pitch, 0])
    assert aruco.layout_extent(spec) == pytest.approx((2 * pitch + 0.04, pitch + 0.04))


def test_load_target_spec_records_provenance(tmp_path):
    path = tmp_path / "target.json"
    path.write_text(json.dumps({"schema_version": 1, "kind": "marker",
                                "dictionary": "DICT_4X4_50", "marker_length_m": 0.05,
                                "marker_id": 3}))
    spec = aruco.load_target_spec(path)
    assert spec["spec_file"] == str(path.resolve())
    assert spec["spec_sha256"] == aruco.fingerprint(spec)
    assert aruco.target_record(spec)["marker_ids"] == [3]
    moved = tmp_path / "copy.json"
    moved.write_text(path.read_text())
    assert aruco.fingerprint(aruco.load_target_spec(moved)) == spec["spec_sha256"]


def test_defaults_fill_in_for_a_board():
    spec = board_spec()
    assert spec["min_visible_markers"] == 2
    assert spec["frame_marker_id"] == 0
    assert spec["marker_ids"] == [0, 1, 2, 3, 4, 5]
    assert spec["min_solution_ratio"] == aruco.DEFAULT_MIN_SOLUTION_RATIO
    assert spec["marker_separation_m"] == 0.012


@pytest.mark.parametrize("value, match", [
    ({"schema_version": 2, "kind": "marker", "dictionary": "DICT_4X4_50",
      "marker_length_m": .05, "marker_id": 0}, "schema_version"),
    ({"schema_version": 1, "kind": "square", "dictionary": "DICT_4X4_50",
      "marker_length_m": .05}, "kind"),
    ({"schema_version": 1, "kind": "marker", "dictionary": "DICT_9X9_9",
      "marker_length_m": .05, "marker_id": 0}, "unknown ArUco dictionary"),
    ({"schema_version": 1, "kind": "marker", "dictionary": "QRCODE",
      "marker_length_m": .05, "marker_id": 0}, "DICT_\\* name"),
    ({"schema_version": 1, "kind": "marker", "dictionary": "DICT_4X4_50",
      "marker_length_m": 0, "marker_id": 0}, "marker_length_m"),
    ({"schema_version": 1, "kind": "marker", "dictionary": "DICT_4X4_50",
      "marker_length_m": .05, "marker_id": 50}, "marker_id"),
    ({"schema_version": 1, "kind": "marker", "dictionary": "DICT_4X4_50",
      "marker_length_m": .05, "marker_id": 0, "min_solution_ratio": 1.0}, "greater than 1"),
    ({"schema_version": 1, "kind": "marker", "dictionary": "DICT_4X4_50",
      "marker_length_m": .05, "marker_id": 0, "extra": 1}, "unknown fields"),
    ({"schema_version": 1, "kind": "board", "dictionary": "DICT_4X4_50",
      "marker_length_m": .04, "marker_separation_m": .01, "markers_x": 2,
      "markers_y": 2, "marker_ids": [0, 1, 2]}, "row-major"),
    ({"schema_version": 1, "kind": "board", "dictionary": "DICT_4X4_50",
      "marker_length_m": .04, "marker_separation_m": .01, "markers_x": 2,
      "markers_y": 2, "marker_ids": [0, 1, 1, 2]}, "unique"),
    ({"schema_version": 1, "kind": "board", "dictionary": "DICT_4X4_50",
      "marker_length_m": .04, "marker_separation_m": .01, "markers_x": 2,
      "markers_y": 2, "marker_ids": [0, 1, 2, 3], "first_marker_id": 0}, "either"),
    ({"schema_version": 1, "kind": "board", "dictionary": "DICT_4X4_50",
      "marker_length_m": .04, "marker_separation_m": -.01, "markers_x": 2,
      "markers_y": 2}, "marker_separation_m"),
    ({"schema_version": 1, "kind": "board", "dictionary": "DICT_4X4_50",
      "marker_length_m": .04, "marker_separation_m": .01, "markers_x": 2,
      "markers_y": 2, "frame_marker_id": 9}, "frame_marker_id"),
    ({"schema_version": 1, "kind": "board", "dictionary": "DICT_4X4_50",
      "marker_length_m": .04, "marker_separation_m": .01, "markers_x": 2,
      "markers_y": 2, "min_visible_markers": 5}, "min_visible_markers"),
    ({"schema_version": 1, "kind": "board", "dictionary": "DICT_4X4_50",
      "marker_length_m": .04, "marker_separation_m": .01, "markers_x": 0,
      "markers_y": 2}, "markers_x"),
    ({"schema_version": 1, "kind": "board", "dictionary": "DICT_4X4_50",
      "marker_length_m": .04, "marker_separation_m": .01, "markers_x": 3,
      "markers_y": 30}, "at most"),
])
def test_invalid_specs_are_rejected(value, match):
    with pytest.raises(ValueError, match=match):
        aruco.validate_target_spec(value)


def test_load_target_spec_rejects_missing_and_broken_files(tmp_path):
    with pytest.raises(ValueError, match="does not exist"):
        aruco.load_target_spec(tmp_path / "absent.json")
    broken = tmp_path / "broken.json"
    broken.write_text("{not json")
    with pytest.raises(ValueError, match="valid JSON"):
        aruco.load_target_spec(broken)
    empty = tmp_path / "list.json"
    empty.write_text("[]")
    with pytest.raises(ValueError, match="JSON object"):
        aruco.load_target_spec(empty)
