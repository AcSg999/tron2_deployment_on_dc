import asyncio
import json
import struct
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
import pytest
import websockets

from tron2_deployment import camera_bridge as bridge


NAMES = [f"arm_{index}" for index in range(14)]
CAMERA = {"backend": "bridge", "bridge_host": "ws://camera.example",
          "color_topic": "/camera/top/color/image_raw/compressed"}
WRIST_TOPIC = "/camera/right/color/image_resized/compressed"


def _image(shape=(480, 640, 3)):
    ok, data = cv2.imencode(".png", np.zeros(shape, np.uint8))
    assert ok
    mime = b"image/png"
    return b"BRDG" + bytes([1, len(mime)]) + mime + struct.pack("<Q", 1000) + data.tobytes()


def _joint(**changes):
    value = {"type": "joint_state", "timestamp": 1023,
             "data": {"names": list(reversed(NAMES)), "positions": list(reversed(range(14)))}}
    value.update(changes)
    return json.dumps(value)


def _mock_sockets(monkeypatch, *, image=None, joint=None):
    opened, closed = [], []

    class Socket:
        def __init__(self, url):
            self.kind = parse_qs(urlsplit(url).query)["kind"][0]

        async def __aenter__(self):
            return self

        async def __aexit__(self, *args):
            closed.append(self.kind)

        async def __aiter__(self):
            yield (_image() if image is None else image) if self.kind == "image" else (_joint() if joint is None else joint)
            await asyncio.Future()

    def connect(url, **options):
        opened.append((url, options))
        return Socket(url)

    monkeypatch.setattr(websockets, "connect", connect)
    return opened, closed


def _capture(**changes):
    options = {"color_topic": WRIST_TOPIC, "joint_topic": "/joint_states", "domain_id": 0,
               "required_names": NAMES, "camera_frame": "right_wrist_camera_color_optical_frame",
               "timeout_s": 1, "max_state_skew_ms": 100}
    options.update(changes)
    return bridge.capture_pair(CAMERA, **options)


def test_capture_subscribes_to_wrist_and_named_feedback_without_sending_commands(monkeypatch):
    opened, closed = _mock_sockets(monkeypatch)
    image, sample = _capture()
    assert image.shape == (480, 640, 3)
    queries = [parse_qs(urlsplit(url).query) for url, _ in opened]
    assert queries == [{"topic": [WRIST_TOPIC], "kind": ["image"], "domain_id": ["0"], "max_fps": ["30"]},
                       {"topic": ["/joint_states"], "kind": ["joint"], "domain_id": ["0"]}]
    assert sample["joints"] == dict(zip(NAMES, range(14)))
    assert sample["joint_stamp_ns"] - sample["image_stamp_ns"] == 23_000_000
    assert sample["timestamp_source"] == "bridge_forwarding_ms"
    assert sample["sensor_header_timestamps_available"] is False
    assert sample["frame_id_source"] == "configured_camera_topic"
    assert sorted(closed) == ["image", "joint"]


@pytest.mark.parametrize("raw", [
    _joint(timestamp=None),
    _joint(data={"positions": list(range(14))}),
    _joint(data={"names": ["arm_0"] * 14, "positions": list(range(14))}),
    _joint(data={"names": NAMES, "positions": [float("nan")] * 14}),
    _joint(data={"names": NAMES[:-1], "positions": list(range(13))}),
])
def test_bridge_feedback_requires_real_names_values_and_explicit_timestamp(raw):
    with pytest.raises(ValueError):
        bridge._joint_sample(raw, NAMES, 1.)


def test_pairing_rejects_stale_feedback_and_large_skew():
    image = bridge._Sample(1000, 5., "image")
    near = bridge._Sample(1023, 5., "near")
    far = bridge._Sample(1101, 5., "far")
    assert bridge._select_pair([image], [far, near], 5.1, 750, 100) == (image, near)
    assert bridge._select_pair([image], [far], 5.1, 750, 100) is None
    assert bridge._select_pair([image], [near], 6., 750, 100) is None
    assert bridge._select_pair([image], [bridge._Sample(1023, 4., "stale")], 5.1, 750, 100) is None


def test_capture_rejects_other_image_dimensions_and_closes_subscriptions(monkeypatch):
    _, closed = _mock_sockets(monkeypatch, image=_image((240, 320, 3)))
    with pytest.raises(ValueError, match="640x480"):
        _capture()
    assert sorted(closed) == ["image", "joint"]


def test_capture_reports_gateway_errors(monkeypatch):
    _mock_sockets(monkeypatch, joint=json.dumps({"type": "error", "message": "topic unavailable"}))
    with pytest.raises(RuntimeError, match="topic unavailable"):
        _capture()


def test_capture_rejects_unsynchronized_streams_instead_of_reusing_old_state(monkeypatch):
    _mock_sockets(monkeypatch, joint=_joint(timestamp=1500))
    with pytest.raises(TimeoutError, match="No fresh synchronized"):
        _capture(timeout_s=.05)
