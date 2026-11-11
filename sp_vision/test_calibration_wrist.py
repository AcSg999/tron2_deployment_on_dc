from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import calibration_wrist as wrist


def _config():
    return wrist.load_config(Path(__file__).parent / "configs" / "wrist_config.example.json")


def test_camera_is_fixed_to_roll_link_but_moves_relative_to_pitch_link():
    config = _config()
    model = wrist.common.UrdfKinematics(wrist.common.config_path(config, config["robot"]["urdf"]))
    nominal = wrist.nominal_matrix(config, model)
    assert wrist.check_model_consistency(config, model)["passed"]
    zero = model.matrix("wrist_pitch_R_Link", "wrist_roll_R_Link", {"wrist_roll_R_Joint": 0}) @ nominal
    rolled = model.matrix("wrist_pitch_R_Link", "wrist_roll_R_Link", {"wrist_roll_R_Joint": .4}) @ nominal
    assert not np.allclose(zero[:3, :3], rolled[:3, :3])
    assert np.allclose(nominal, model.matrix("wrist_roll_R_Link", config["robot"]["camera_frame"], {}))


def test_capture_state_requires_named_joint_values_and_synchronized_stamps():
    config = _config()
    source = config["robot"]["ros2_joint_names"]["left"] + config["robot"]["ros2_joint_names"]["right"]
    joints = dict(zip(source, np.linspace(-.3, .3, 14)))
    frame = {"camera_frame_id": "right_color_optical_frame", "arm_q14": wrist.arm_state(config, joints),
             "sensor_sync": {"image_stamp_ns": 1_000_000_000, "joint_stamp_ns": 1_050_000_000,
                             "joints": joints}}
    assert np.allclose(wrist.check_frame(config, frame), frame["arm_q14"])
    frame["sensor_sync"]["joint_stamp_ns"] = 1_200_000_000
    with pytest.raises(ValueError, match="synchronization"):
        wrist.check_frame(config, frame)
    frame["sensor_sync"]["joint_stamp_ns"] = 1_050_000_000
    del joints[source[7]]
    with pytest.raises(ValueError, match="lacks"):
        wrist.check_frame(config, frame)


def test_probe_compares_ros2_order_with_head_validation_controller_state(tmp_path, monkeypatch):
    config = _config()
    names = config["robot"]["ros2_joint_names"]["left"] + config["robot"]["ros2_joint_names"]["right"]
    values = np.linspace(-.7, .7, 14)
    sample = {"frame_id": "right_color_optical_frame", "image_stamp_ns": 1_000_000_000,
              "joint_stamp_ns": 1_002_000_000, "joints": dict(zip(names, values))}
    monkeypatch.setattr(wrist, "capture_pair", lambda _: (np.zeros((4, 5, 3), dtype=np.uint8), sample))
    monkeypatch.setattr(wrist, "vendor_arm_state", lambda _: (values + .0001).tolist())
    output = tmp_path / "probe.json"
    wrist.probe_command(config, output)
    report = wrist.common.read_json(output)
    assert report["mapping_check"]["passed"]
    assert report["mapping_check"]["max_ros2_vendor_difference_rad"] < .005
    assert report["state_skew_ms"] == 2


def test_bridge_capture_uses_configured_profile_and_preserves_provenance(tmp_path, monkeypatch):
    from tron2_deployment import camera_bridge
    config = _config()
    profile = tmp_path / "profile.json"
    wrist.common.write_json(profile, {"camera": {"backend": "bridge", "bridge_host": "wss://camera.example"}})
    config["capture"].update(backend="bridge", bridge_profile=str(profile))
    names = config["robot"]["ros2_joint_names"]["left"] + config["robot"]["ros2_joint_names"]["right"]
    sample = {"frame_id": config["robot"]["camera_frame"], "source": "limx_studio_bridge",
              "frame_id_source": "configured_camera_topic", "timestamp_source": "bridge_forwarding_ms",
              "sensor_header_timestamps_available": False, "color_topic": config["capture"]["color_topic"],
              "image_stamp_ns": 1_000_000_000, "joint_stamp_ns": 1_023_000_000,
              "joints": dict(zip(names, np.linspace(-.3, .3, 14)))}

    def capture(camera, **options):
        assert camera["bridge_host"] == "wss://camera.example"
        assert options["required_names"] == names
        return np.zeros((480, 640, 3), np.uint8), sample

    monkeypatch.setattr(camera_bridge, "capture_pair", capture)
    monkeypatch.setattr(wrist.subprocess, "run", lambda *a, **k: pytest.fail("Bridge capture must not invoke SSH"))
    image, saved = wrist.capture_pair(config)
    frame = {"camera_frame_id": saved["frame_id"], "arm_q14": wrist.arm_state(config, saved["joints"]),
             "sensor_sync": saved}
    assert image.shape == (480, 640, 3)
    assert wrist.check_frame(config, frame) == frame["arm_q14"]
    # A real bridge capture must reach the intrinsic fit, then retain its
    # configured camera identity for the subsequent extrinsic solve.
    session = tmp_path / "session"
    wrist.common.write_json(session / "view-001" / "frame.json", frame)
    assert wrist.cv2.imwrite(str(session / "view-001" / "color.png"), image)
    output = session / "wrist_intrinsics.json"

    def intrinsic_fit(_config, _session, destination):
        wrist.common.write_json(destination, {"passed": True})
        return True

    monkeypatch.setattr(wrist.common, "intrinsics_command", intrinsic_fit)
    assert wrist.intrinsics_command(config, session, output)
    intrinsic = wrist.common.read_json(output)
    assert intrinsic["camera_frame"] == config["robot"]["camera_frame"]
    assert intrinsic["color_topic"] == config["capture"]["color_topic"]
    wrong_frame = {**frame, "camera_frame_id": "head_camera_color_optical_frame"}
    wrist.common.write_json(session / "view-001" / "frame.json", wrong_frame)
    with pytest.raises(ValueError, match="unexpected camera frame"):
        wrist.intrinsics_command(config, session, output)
    saved["timestamp_source"] = "ros2_header_ns"
    with pytest.raises(ValueError, match="provenance"):
        wrist.check_frame(config, frame)


def test_bridge_failure_does_not_fall_back_to_ssh(tmp_path, monkeypatch):
    from tron2_deployment import camera_bridge
    config = _config()
    profile = tmp_path / "profile.json"
    wrist.common.write_json(profile, {"camera": {"backend": "bridge"}})
    config["capture"].update(backend="bridge", bridge_profile=str(profile))

    def unavailable(*args, **kwargs):
        raise TimeoutError("camera unavailable")

    monkeypatch.setattr(camera_bridge, "capture_pair", unavailable)
    monkeypatch.setattr(wrist.subprocess, "run", lambda *a, **k: pytest.fail("Bridge capture must not invoke SSH"))
    with pytest.raises(TimeoutError, match="camera unavailable"):
        wrist.capture_pair(config)


def test_touch_validation_rejects_tcp_from_other_arm(tmp_path):
    config = _config()
    selection = tmp_path / "selection.json"
    pivot = tmp_path / "pivot.json"
    wrist.common.write_json(selection, {"kind": "sp_vision_wrist_touch_selection",
                                        "corners": [{}, {}, {}]})
    wrist.common.write_json(pivot, {"kind": "sp_vision_tcp_pivot", "side": "right",
                                    "passed": True, "tip_in_wrist_m": [0, 0, 0]})
    with pytest.raises(ValueError, match="selected touch arm"):
        wrist.validate_command(config, selection, "left", [tmp_path] * 3, pivot,
                               tmp_path / "report.json")


def test_capture_replaces_previous_view_only_after_complete_save(tmp_path, monkeypatch):
    config = _config()
    session = tmp_path / "validation"
    previous = session / "view-001"
    previous.mkdir(parents=True)
    (previous / "color.png").write_bytes(b"previous")
    (session / "selection.json").write_text("previous selection")
    names = config["robot"]["ros2_joint_names"]["left"] + config["robot"]["ros2_joint_names"]["right"]
    sample = {"frame_id": "right_color_optical_frame", "image_stamp_ns": 1,
              "joint_stamp_ns": 1, "joints": dict.fromkeys(names, 0.0)}
    monkeypatch.setattr(wrist, "capture_pair", lambda _config: (np.zeros((4, 5, 3), dtype=np.uint8), sample))
    monkeypatch.setattr(wrist.common, "detect_board", lambda *_args: (
        np.zeros((70, 2), dtype=np.float32), {"accepted": True}))
    monkeypatch.setattr(wrist.common, "draw_detection", lambda image, *_args: image)
    monkeypatch.setattr(wrist.cv2, "imshow", lambda *_args: None)
    monkeypatch.setattr(wrist.cv2, "destroyAllWindows", lambda: None)
    monkeypatch.setattr(wrist.cv2, "imwrite", lambda path, _image: Path(path).write_bytes(b"new") or True)

    monkeypatch.setattr(wrist.cv2, "waitKey", lambda _delay: ord("q"))
    wrist.capture_command(config, session, 1, False)
    assert (previous / "color.png").read_bytes() == b"previous"

    monkeypatch.setattr(wrist.cv2, "waitKey", lambda _delay: ord("s"))
    wrist.capture_command(config, session, 1, False)
    assert (previous / "color.png").read_bytes() == b"new"
    assert (session / "selection.json").read_text() == "previous selection"
