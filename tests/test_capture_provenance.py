"""RGB-D geometry and calibration timing checks, using synthetic local inputs."""

from copy import deepcopy
from pathlib import Path
import time

import numpy as np
import pytest

from tron2_deployment import calibration, camera, config, rgbd


def demo_profile():
    return config.load_profile(Path(config.__file__).with_name("assets") / "demo.json")


def real_camera_profile():
    profile = demo_profile()
    profile["camera"].update(backend="bridge", depth_intrinsics=profile["camera"]["intrinsics"],
        depth_to_color=np.eye(4).tolist(), color_topic="/color", depth_topic="/depth",
        max_frame_age_s=0.75)
    return profile


def depth_config(translation):
    k = (500, 0, 320, 0, 500, 240, 0, 0, 1)
    return rgbd.Tron2HighRgbdConfig(color_k=k, color_dist=(0,)*5, depth_k=k,
        depth_to_color_r=tuple(np.eye(3).ravel()), depth_to_color_t_m=translation)


def test_aligned_depth_contains_color_camera_metric_z():
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[240, 320] = 1000
    aligned = rgbd.align_depth_to_color(depth, depth_config((0, 0, 0.1)))
    assert aligned[240, 320] == 1100
    assert np.count_nonzero(aligned) == 1


def test_aligned_depth_zbuffer_keeps_nearest_color_depth():
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[240, 320] = 1000
    depth[240, 321] = 500
    aligned = rgbd.align_depth_to_color(depth, depth_config((0, 0, 1.0)))
    assert aligned[240, 320] == 1500


def test_real_camera_resolution_is_explicit():
    profile = real_camera_profile()
    config.validate_profile(profile)
    profile["camera"]["width"] = 1280
    with pytest.raises(ValueError, match="640x480"):
        config.validate_profile(profile)


def _fake_capture_driver(monkeypatch, *, stamp, head=(0, 0), received_stamp=None):
    class Driver:
        closed = False

        def __init__(self, config):
            self.config = config

        def capture(self, **kwargs):
            sync = {"head_pitch_yaw": list(head), "color_timestamp_ms": stamp}
            if received_stamp is not None:
                sync["color_received_timestamp_ms"] = received_stamp
            return (np.zeros((480, 640, 3), dtype=np.uint8),
                    np.full((480, 640), 1000, dtype=np.uint16),
                    sync)

        def close(self):
            self.closed = True

    monkeypatch.setattr(rgbd, "Tron2HighRgbdCapture", Driver)


def test_capture_retains_acquisition_age_instead_of_refreshing_it(monkeypatch):
    monkeypatch.setattr(camera.time, "time", lambda: 100.0)
    _fake_capture_driver(monkeypatch, stamp=99_500)
    frame = camera.capture(real_camera_profile())
    assert frame["capture_timestamp_s"] == 99.5
    assert frame["timestamp_s"] == 100.0


def test_bridge_capture_uses_local_receipt_time_for_freshness(monkeypatch):
    monkeypatch.setattr(camera.time, "time", lambda: 100.0)
    _fake_capture_driver(
        monkeypatch, stamp=87_000, received_stamp=99_900)
    frame = camera.capture(real_camera_profile())
    assert frame["capture_timestamp_s"] == 99.9
    assert frame["sensor_sync"]["color_timestamp_ms"] == 87_000


@pytest.mark.parametrize("stamp", [0, 98_000, 101_000])
def test_capture_rejects_old_or_unsynchronized_sensor_clock(monkeypatch, stamp):
    monkeypatch.setattr(camera.time, "time", lambda: 100.0)
    _fake_capture_driver(monkeypatch, stamp=stamp)
    with pytest.raises(ValueError, match="stale|synchronized"):
        camera.capture(real_camera_profile())


def test_camera_rejects_head_moved_from_calibration(monkeypatch):
    monkeypatch.setattr(camera.time, "time", lambda: 100.0)
    _fake_capture_driver(monkeypatch, stamp=100_000, head=(0.1, 0))
    with pytest.raises(ValueError, match="head moved"):
        camera.capture(real_camera_profile())


def _sample_fixture(monkeypatch, *, before_stamp=99.95, after_stamp=100.0, captured=99.975, sync=None):
    from tron2_deployment import robot, kinematics

    class Adapter:
        def __init__(self, profile):
            self.count = 0

        def read_state(self):
            self.count += 1
            return {"arm_q14": [0.0]*14, "head_q2": [0.0, 0.0], "source": "real",
                    "timestamp_s": before_stamp if self.count == 1 else after_stamp}

        def close(self):
            pass

    monkeypatch.setattr(robot, "WebsocketRobot", Adapter)
    monkeypatch.setattr(calibration.time, "time", lambda: 100.0)
    monkeypatch.setattr(calibration, "capture", lambda *a, **kw: {
        "source": "real", "capture_timestamp_s": captured, "head_q2": [0, 0],
        "sensor_sync": sync or {}, "camera_id": "test", "image": "ignored"})
    monkeypatch.setattr(calibration, "decode_image", lambda *a, **kw: np.zeros((480, 640, 3), dtype=np.uint8))
    monkeypatch.setattr(calibration, "corners", lambda *a, **kw: np.zeros((54, 1, 2), dtype=np.float32))
    monkeypatch.setattr(calibration.cv2, "solvePnP", lambda *a, **kw: (True, np.zeros(3), np.array([0, 0, 1.0])))
    used_sides = []

    class Model:
        def __init__(self, profile):
            pass

        def set_state(self, arms, head):
            pass

        def wrist_poses(self):
            class Poses(dict):
                def __getitem__(self, key):
                    used_sides.append(key)
                    return [0, 0, 0, 1, 0, 0, 0]
            return Poses()

    monkeypatch.setattr(kinematics, "RobotModel", Model)
    return used_sides


def test_handeye_sample_selects_requested_arm_with_valid_timing(monkeypatch, tmp_path):
    used = _sample_fixture(monkeypatch)
    path = calibration.record_sample(real_camera_profile(), "right", tmp_path)
    assert path.exists() and used == ["right"]


def test_handeye_brackets_frame_during_slow_bridge_startup(monkeypatch, tmp_path):
    from tron2_deployment import robot

    _sample_fixture(monkeypatch)
    started = time.monotonic()
    clock = lambda: 100.0 + 10.0 * (time.monotonic() - started)
    monkeypatch.setattr(calibration.time, "time", clock)

    class Adapter:
        def __init__(self, profile):
            pass

        def read_state(self):
            return {"arm_q14": [0.0]*14, "head_q2": [0.0, 0.0],
                    "timestamp_s": clock(), "source": "real"}

        def close(self):
            pass

    def slow_capture(*args, **kwargs):
        time.sleep(0.2)  # Two virtual seconds before the RGB-D frame arrives.
        captured = clock()
        time.sleep(0.2)  # Bridge decoding can also finish after frame receipt.
        return {"source": "real", "capture_timestamp_s": captured,
                "head_q2": [0, 0], "sensor_sync": {}, "camera_id": "test",
                "image": "ignored"}

    monkeypatch.setattr(robot, "WebsocketRobot", Adapter)
    monkeypatch.setattr(calibration, "capture", slow_capture)
    path = calibration.record_sample(real_camera_profile(), "right", tmp_path)
    assert path.exists()


@pytest.mark.parametrize("settings,error", [
    ({"before_stamp": 99.0}, "stale"),
    ({"after_stamp": 99.0}, "stale"),
    ({"captured": 99.0}, "bracketed"),
    ({"sync": {"state_alignment": "stable_head_receipt_fallback"}}, "synchronized"),
    ({"sync": {"state_header_skew_exceeded": True}}, "synchronized"),
    ({"sync": {"state_skew_ms": 200}}, "synchronized"),
])
def test_handeye_rejects_unsynchronized_arm_image_samples(monkeypatch, tmp_path, settings, error):
    _sample_fixture(monkeypatch, **settings)
    with pytest.raises(ValueError, match=error):
        calibration.record_sample(real_camera_profile(), "left", tmp_path)
    assert list(tmp_path.iterdir()) == []
