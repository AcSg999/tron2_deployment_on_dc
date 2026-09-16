from copy import deepcopy
import json
from pathlib import Path
import time

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tron2_deployment import aruco, calibration, camera, kinematics, robot
from tron2_deployment.calibration_guide import CalibrationGuide
from tron2_deployment.config import load_profile


def write_target_spec(path, marker=False, **overrides):
    if marker:
        value = {"schema_version": 1, "kind": "marker", "dictionary": "DICT_6X6_250",
                 "marker_length_m": 0.05, "marker_id": 23}
    else:
        value = {"schema_version": 1, "kind": "board", "dictionary": "DICT_6X6_250",
                 "marker_length_m": 0.04, "marker_separation_m": 0.012,
                 "markers_x": 2, "markers_y": 3, "first_marker_id": 0}
    value.update(overrides)
    path.write_text(json.dumps(value))
    return aruco.load_target_spec(path)


@pytest.fixture
def profile():
    return load_profile(Path(__file__).parents[1] / "configs/demo.json")


def test_constructor_and_status_do_not_connect(profile, tmp_path, monkeypatch):
    def unexpected(*args, **kwargs):
        pytest.fail("startup must not connect to hardware")
    monkeypatch.setattr(camera, "capture", unexpected)
    monkeypatch.setattr(calibration, "record_sample", unexpected)
    guide = CalibrationGuide(profile, tmp_path / "session")
    status = guide.status()
    assert status["saved_count"] == 0 and not status["can_solve"]
    assert status["preview_image"] is None and status["result"] is None
    assert set(status["next"]) == {"en", "zh"}
    assert not status["hardware_commanded"] and not status["calibration_applied"]


@pytest.mark.parametrize("kind", ["occupied", "file", "symlink"])
def test_session_must_be_new_or_empty(profile, tmp_path, kind):
    path = tmp_path / "session"
    if kind == "file":
        path.write_text("existing")
    elif kind == "symlink":
        target = tmp_path / "target"
        target.mkdir()
        path.symlink_to(target, target_is_directory=True)
    else:
        path.mkdir()
        (path / "old.json").write_text("{}")
    with pytest.raises(ValueError, match="new or empty"):
        CalibrationGuide(profile, path)


@pytest.mark.parametrize("operation", ["preview", "save", "solve"])
def test_profile_change_blocks_operations(profile, tmp_path, operation):
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(profile))
    guide = CalibrationGuide(profile, tmp_path / "session", mock=True, profile_path=path)
    path.write_text("{}")
    with pytest.raises(ValueError, match="profile changed"):
        getattr(guide, operation)()
    assert guide.status()["saved_count"] == 0


def test_save_uses_fresh_raw_frame_not_the_preview(profile, tmp_path, monkeypatch):
    guide = CalibrationGuide(profile, tmp_path / "session")
    first, _ = guide._mock_observation(0)
    second, _ = guide._mock_observation(1)
    frames = iter([first, second])
    calls = []
    def capture(p, **kwargs):
        calls.append((p, kwargs))
        return next(frames)
    monkeypatch.setattr(camera, "capture", capture)
    assert guide.preview()["board_detected"]
    status = guide.save()
    assert status["saved_count"] == 1
    assert all(kwargs == {"mock": False, "undistort": False} for _, kwargs in calls)
    stored = cv2.imread(str(tmp_path / "session/view-001/color.png"))
    np.testing.assert_array_equal(stored, camera.decode_image(second["image"]))
    assert not np.array_equal(stored, camera.decode_image(first["image"]))
    found = calibration.corners(stored)
    cv2.drawChessboardCorners(stored, (9, 6), found, True)
    np.testing.assert_array_equal(camera.decode_image(status["preview_image"]), stored)
    saved_frame = json.loads((tmp_path / "session/view-001/frame.json").read_text())
    assert saved_frame == second
    assert not list((tmp_path / "session").glob(".capture-*"))


def test_model_change_blocks_new_observations(profile, tmp_path):
    model = tmp_path / "model.xml"
    model.write_bytes(Path(profile["robot"]["model_xml"]).read_bytes())
    profile["robot"]["model_xml"] = str(model)
    guide = CalibrationGuide(profile, tmp_path / "session", mock=True)
    model.write_text("changed")
    with pytest.raises(ValueError, match="robot model changed"):
        guide.preview()


def test_missing_board_stays_visible_without_saving(profile, tmp_path, monkeypatch):
    guide = CalibrationGuide(profile, tmp_path / "session")
    frame, _ = guide._mock_observation(0)
    frame["image"] = camera.encode_image(np.full((480, 640, 3), 200, np.uint8))
    monkeypatch.setattr(camera, "capture", lambda *a, **k: frame)
    state = guide.save()
    assert state["saved_count"] == 0 and state["board_detected"] is False
    assert state["preview_image"] == frame["image"]
    assert "entire board" in state["next"]["en"]
    assert "完整棋盘" in state["next"]["zh"]
    assert [p.name for p in (tmp_path / "session").iterdir()] == ["session.json"]


@pytest.mark.parametrize("stage", ["intrinsics", "handeye"])
def test_mock_walkthrough_solve_and_invalidate(profile, tmp_path, stage, monkeypatch):
    def unexpected(*args, **kwargs):
        pytest.fail("mock mode must not touch a real camera or robot")
    monkeypatch.setattr(camera, "capture", unexpected)
    monkeypatch.setattr(robot, "WebsocketRobot", unexpected)
    original = deepcopy(profile)
    guide = CalibrationGuide(profile, tmp_path / "session", stage=stage, side="right", mock=True)
    for index in range(9):
        state = guide.save()
        assert state["saved_count"] == index + 1
        assert state["board_detected"]
        assert state["can_solve"] == (index >= 4)
    assert len(state["coverage_regions"]) == 9
    result = guide.solve()["result"]
    assert result["status"] == "unverified"
    path = Path(result["path"])
    assert path.name == ("intrinsics.json" if stage == "intrinsics" else "handeye-right.json")
    solution = json.loads(path.read_text())
    assert solution["verified"] is False and solution["source"] == "mock"
    if stage == "intrinsics":
        assert solution["rms_px"] < .5
        assert len(solution["images"]) == 9
    else:
        assert len(list((tmp_path / "session/samples").glob("*.json"))) == 9
        assert "samples" not in path.parts
        expected = np.eye(4)
        expected[:3, :3] = Rotation.from_euler("xyz", [.2, -.3, .1]).as_matrix()
        expected[:3, 3] = [.2, -.1, .5]
        np.testing.assert_allclose(solution["camera_to_base"], expected, atol=1e-6)
    state = guide.save()
    assert state["saved_count"] == 10 and state["result"] is None
    assert not path.exists()
    assert profile == original


def test_duplicate_rejection_preserves_previous_solution(profile, tmp_path, monkeypatch):
    guide = CalibrationGuide(profile, tmp_path / "session", mock=True)
    for _ in range(5):
        guide.save()
    result = guide.solve()["result"]
    monkeypatch.setattr(guide, "_fresh_frame", lambda: guide._mock_observation(0))
    state = guide.save()
    assert state["saved_count"] == 5 and "similar" in state["notice"]["en"]
    assert state["result"] == result and Path(result["path"]).is_file()


@pytest.mark.parametrize("operation", ["save", "solve"])
def test_external_solution_edits_are_preserved(profile, tmp_path, operation):
    guide = CalibrationGuide(profile, tmp_path / "session", mock=True)
    for _ in range(5):
        guide.save()
    result = guide.solve()["result"]
    path = Path(result["path"])
    path.write_text("user changed this result")
    state = getattr(guide, operation)()
    assert state["saved_count"] == 5
    assert "solution changed" in state["notice"]["en"]
    assert state["notice"]["level"] == "error"
    assert path.read_text() == "user changed this result"


def test_changed_sample_blocks_solve(profile, tmp_path):
    guide = CalibrationGuide(profile, tmp_path / "session", mock=True)
    for _ in range(5):
        guide.save()
    (tmp_path / "session/view-001/color.png").write_bytes(b"changed")
    state = guide.solve()
    assert "changed" in state["notice"]["en"]
    assert state["result"] is None and not (tmp_path / "session/intrinsics.json").exists()


def test_handeye_rotation_readiness_matches_solver_gate(profile, tmp_path, monkeypatch):
    guide = CalibrationGuide(profile, tmp_path / "session", stage="handeye", side="left", mock=True)
    original = guide._mock_sample
    def unexcited(frame, target):
        sample = original(frame, target)
        wrist = np.array(sample["robot_gripper_to_base"])
        wrist[:3, :3] = np.eye(3)
        sample["robot_gripper_to_base"] = wrist.tolist()
        return sample
    monkeypatch.setattr(guide, "_mock_sample", unexcited)
    for _ in range(5):
        state = guide.save()
    assert state["saved_count"] == 5 and not state["can_solve"]
    assert state["rotation_spread_deg"] == 0
    assert guide.solve()["result"] is None


@pytest.mark.parametrize("failure", [None, "stale", "moving", "board"])
def test_real_handeye_uses_existing_stationary_capture_gates(profile, tmp_path, monkeypatch, failure):
    guide = CalibrationGuide(profile, tmp_path / "session", stage="handeye", side="left")
    calls = []
    class Adapter:
        def __init__(self, p):
            self.count = 0
        def read_state(self):
            self.count += 1
            calls.append("read")
            return {"timestamp_s": time.time() - (10 if failure == "stale" else 0),
                    "arm_q14": [(.1 if failure == "moving" and self.count == 2 else 0)] * 14,
                    "head_q2": profile["calibration"]["head_q2"]}
        def close(self):
            calls.append("close")
    class Model:
        def __init__(self, p):
            pass
        def set_state(self, *args):
            pass
        def wrist_poses(self):
            return {"left": [0, 0, 0, 1, 0, 0, 0]}
    def capture(p, **kwargs):
        assert kwargs == {"mock": False, "undistort": False}
        calls.append("capture")
        frame, _ = guide._mock_observation(0)
        frame["source"] = "real"
        if failure == "board":
            frame["image"] = camera.encode_image(np.full((480, 640, 3), 200, np.uint8))
        return frame
    monkeypatch.setattr(robot, "WebsocketRobot", Adapter)
    monkeypatch.setattr(kinematics, "RobotModel", Model)
    monkeypatch.setattr(calibration, "capture", capture)
    state = guide.save()
    assert calls == ["read", "capture", "read", "close"]
    assert state["saved_count"] == (0 if failure else 1)
    if failure in ("stale", "moving"):
        assert state["preview_image"] is None
    elif failure == "board":
        assert state["preview_image"] and state["board_detected"] is False
        assert "完整棋盘" in state["next"]["zh"]
    else:
        assert state["board_detected"] is True
        path = next((tmp_path / "session/samples").glob("*.json"))
        sample = json.loads(path.read_text())
        assert sample["source"] == "real" and sample["side"] == "left"
        assert (path.parent / sample["image"]).is_file()


def test_capture_failures_clear_old_preview_and_preserve_samples(profile, tmp_path, monkeypatch):
    guide = CalibrationGuide(profile, tmp_path / "session", mock=True)
    assert guide.save()["saved_count"] == 1
    def failed():
        raise OSError("camera unavailable")
    monkeypatch.setattr(guide, "_fresh_frame", failed)
    state = guide.preview()
    assert state["saved_count"] == 1 and state["preview_image"] is None
    assert "camera unavailable" in state["notice"]["en"]


def test_programming_errors_are_not_silenced(profile, tmp_path, monkeypatch):
    guide = CalibrationGuide(profile, tmp_path / "session", mock=True)
    def failed():
        raise TypeError("programmer error")
    monkeypatch.setattr(guide, "_fresh_frame", failed)
    with pytest.raises(TypeError, match="programmer error"):
        guide.save()
    assert not list((tmp_path / "session").glob(".capture-*"))


@pytest.mark.parametrize("marker_kind", ["board", "marker"])
def test_mock_aruco_walkthrough_solves_without_hardware(profile, tmp_path, marker_kind, monkeypatch):
    def unexpected(*args, **kwargs):
        pytest.fail("mock mode must not touch a real camera or robot")
    monkeypatch.setattr(camera, "capture", unexpected)
    monkeypatch.setattr(calibration, "record_sample", unexpected)
    overrides = {"marker_length_m": 0.05} if marker_kind == "marker" else {}
    path = tmp_path / "target.json"
    target = write_target_spec(path, marker=(marker_kind == "marker"), **overrides)
    guide = CalibrationGuide(profile, tmp_path / "session", stage="handeye", side="left",
                             target=target, mock=True)
    status = guide.status()
    assert status["target"]["kind"] == marker_kind
    assert status["pattern"] is None and status["square_m"] is None
    assert status["detected_marker_ids"] == []
    for index in range(9):
        state = guide.save()
        assert state["saved_count"] == index + 1
        assert state["board_detected"]
        assert state["detected_marker_ids"] == target["marker_ids"]
    assert state["can_solve"]
    result = guide.solve()["result"]
    assert result["status"] == "unverified"
    solution = json.loads(Path(result["path"]).read_text())
    assert solution["verified"] is False and solution["source"] == "mock"
    assert solution["target"]["kind"] == marker_kind
    assert solution["target"]["marker_length_m"] == target["marker_length_m"]
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler("xyz", [.2, -.3, .1]).as_matrix()
    expected[:3, 3] = [.2, -.1, .5]
    np.testing.assert_allclose(solution["camera_to_base"], expected, atol=1e-6)
    sample = json.loads(next((tmp_path / "session/samples").glob("*.json")).read_text())
    assert sample["target"]["kind"] == marker_kind
    assert "pattern" not in sample and "square_m" not in sample


def test_target_spec_change_blocks_collection(profile, tmp_path):
    path = tmp_path / "target.json"
    target = write_target_spec(path)
    guide = CalibrationGuide(profile, tmp_path / "session", stage="handeye", side="left",
                             target=target, mock=True)
    write_target_spec(path, marker_length_m=0.041)
    with pytest.raises(ValueError, match="target spec changed"):
        guide.preview()
    assert guide.status()["saved_count"] == 0


def test_aruco_target_is_handeye_only(profile, tmp_path):
    target = write_target_spec(tmp_path / "target.json")
    with pytest.raises(ValueError, match="hand-eye collection only"):
        CalibrationGuide(profile, tmp_path / "session", target=target, mock=True)


def test_cli_target_loader_requires_and_validates_spec(tmp_path):
    from argparse import Namespace
    from tron2_deployment.cli import calibration_target
    with pytest.raises(ValueError, match="requires --target-spec"):
        calibration_target(Namespace(target="aruco", target_spec=None, stage="handeye"))
    with pytest.raises(ValueError, match="only to --target aruco"):
        calibration_target(Namespace(target="chessboard", target_spec="x.json", stage="handeye"))
    spec = write_target_spec(tmp_path / "target.json")
    with pytest.raises(ValueError, match="hand-eye collection only"):
        calibration_target(Namespace(target="aruco", target_spec=spec["spec_file"], stage="intrinsics"))
    assert calibration_target(Namespace(target="chessboard", target_spec=None, stage="intrinsics")) is None
