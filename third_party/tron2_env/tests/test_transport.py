"""Tests for WebsocketTransport — uses a fake websocket; no robot needed."""

from __future__ import annotations

import json
import sys
import threading
import time
import types

import numpy as np
import pytest

# Build the fake module first; we'll inject it both into sys.modules (covers
# fresh imports) and onto the already-imported transport module (covers cases
# where another test pulled the real ``websocket`` in earlier).
_fake_ws_module = types.ModuleType("websocket")


class _FakeApp:
    """Recorded sends + can inject incoming messages via .feed_message()."""

    instances: list = []

    def __init__(self, url, on_open=None, on_message=None, on_close=None, on_error=None):
        self.url = url
        self.on_open = on_open
        self.on_message = on_message
        self.on_close = on_close
        self.on_error = on_error
        self.sent: list = []
        self.closed = False
        _FakeApp.instances.append(self)

    def run_forever(self):
        # Simulate "connection open" then sleep until close.
        if self.on_open:
            self.on_open(self)
        while not self.closed:
            time.sleep(0.005)

    def send(self, message: str):
        self.sent.append(message)

    def close(self):
        self.closed = True
        if self.on_close:
            self.on_close(self, 0, "fake close")

    # helper for tests
    def feed_message(self, payload: dict):
        if self.on_message:
            self.on_message(self, json.dumps(payload))


_fake_ws_module.WebSocketApp = _FakeApp
sys.modules["websocket"] = _fake_ws_module

# Force tron2_env.transport.websocket to bind to our fake even if it was
# already imported by another test (in which case sys.modules patch alone
# is ignored — the module already has its own ``websocket`` reference).
import tron2_env.transport.websocket as _ws_mod  # noqa: E402

_ws_mod.websocket = _fake_ws_module

from tron2_env.config import Tron2Config  # noqa: E402
from tron2_env.errors import CommandError, StateError  # noqa: E402
from tron2_env.joints import JointIndex  # noqa: E402
from tron2_env.transport import WebsocketTransport  # noqa: E402


@pytest.fixture
def transport():
    _FakeApp.instances.clear()
    config = Tron2Config(robot_ip="127.0.0.1", polling_rate=1000.0)
    t = WebsocketTransport(config)
    # wait until on_open has fired
    deadline = time.time() + 1.0
    while not t.is_connected() and time.time() < deadline:
        time.sleep(0.01)
    # The polling thread spams ws messages — start each test from a clean send log.
    for app in _FakeApp.instances:
        app.sent.clear()
    yield t
    t.disconnect()


def _ws_app() -> _FakeApp:
    assert _FakeApp.instances, "no fake ws created"
    return _FakeApp.instances[0]


def test_send_joint_cmd_emits_servoj_message(transport):
    transport.send_joint_cmd(np.linspace(0.1, 1.6, 16))
    sent = [json.loads(m) for m in _ws_app().sent if json.loads(m).get("title") == "request_servoj"]
    assert sent, "no servoj message was sent"
    payload = sent[-1]["data"]
    assert len(payload["q"]) == 16
    assert payload["filter_ratio"] == 1.0


def test_send_joint_cmd_rejects_wrong_shape(transport):
    with pytest.raises(CommandError):
        transport.send_joint_cmd(np.zeros(14))


def test_get_joint_state_commits_when_both_frames_arrive(transport):
    # Inject one joint state frame — should NOT commit by itself.
    joint_q = list(np.arange(16) * 0.01)
    before_client_timestamp = int(time.time() * 1000)
    _ws_app().feed_message({
        "title": "response_get_joint_state",
        "timestamp": 1234567,
        "data": {"q": joint_q},
    })
    with pytest.raises(StateError):
        transport.get_joint_state(timeout=0.05)

    # Inject gripper frame — now commit fires.
    _ws_app().feed_message({
        "title": "response_get_limx_2fclaw_state",
        "timestamp": 1234568,
        "data": {"left_opening": 40, "right_opening": 60},
    })
    state = transport.get_joint_state(timeout=0.5)
    assert state["timestamp"] >= before_client_timestamp
    assert state["timestamp"] <= int(time.time() * 1000)
    assert state["robot_timestamp"] == 1234567
    assert len(state["states"]) == JointIndex.STATE_DIM
    np.testing.assert_allclose(state["states"][JointIndex.LEFT_ARM], joint_q[:7])
    np.testing.assert_allclose(state["states"][JointIndex.RIGHT_ARM], joint_q[7:14])
    assert state["states"][JointIndex.LEFT_GRIPPER] == pytest.approx(0.4)
    assert state["states"][JointIndex.RIGHT_GRIPPER] == pytest.approx(0.6)
    assert state["states"][JointIndex.HEAD_PITCH] == pytest.approx(joint_q[14])
    assert state["states"][JointIndex.HEAD_YAW] == pytest.approx(joint_q[15])


def test_get_head_position_returns_latest(transport):
    joint_q = list(np.zeros(16))
    joint_q[14] = 1.0
    joint_q[15] = -0.5
    _ws_app().feed_message({
        "title": "response_get_joint_state",
        "timestamp": 1,
        "data": {"q": joint_q},
    })
    # No need to commit; get_head_position reads from the snapshot directly.
    np.testing.assert_allclose(transport.get_head_position(), [1.0, -0.5])


def test_set_gripper_emits_message(transport):
    transport.set_gripper(left_opening=33, right_opening=66)
    sent = [json.loads(m) for m in _ws_app().sent if json.loads(m).get("title") == "request_set_limx_2fclaw_cmd"]
    assert sent
    payload = sent[-1]["data"]
    assert payload["left_opening"] == 33
    assert payload["right_opening"] == 66


@pytest.mark.parametrize("result, should_raise", [("success", False), ("fail_invalid_cmd", True)])
def test_movej_requires_matching_controller_response(transport, result, should_raise):
    outcome: list[Exception | None] = []

    def request() -> None:
        try:
            transport.movej(np.zeros(JointIndex.MOVEJ_DIM), response_timeout=0.5)
        except Exception as exc:  # assertion runs in the parent thread
            outcome.append(exc)
        else:
            outcome.append(None)

    thread = threading.Thread(target=request)
    thread.start()
    deadline = time.time() + 0.5
    request_message = None
    while time.time() < deadline:
        sent = [json.loads(message) for message in _ws_app().sent]
        matches = [message for message in sent if message.get("title") == "request_movej"]
        if matches:
            request_message = matches[-1]
            break
        time.sleep(0.005)
    assert request_message is not None, "no MoveJ request was sent"
    _ws_app().feed_message({
        "title": "response_movej",
        "guid": request_message["guid"],
        "data": {"result": result},
    })
    thread.join(timeout=1.0)
    assert len(outcome) == 1
    if should_raise:
        assert isinstance(outcome[0], CommandError)
    else:
        assert outcome == [None]


@pytest.mark.parametrize("result, expected", [("success", True), ("fail_motor", False)])
def test_emergency_stop_requires_matching_success_response(transport, result, expected):
    outcome: list[bool] = []
    thread = threading.Thread(
        target=lambda: outcome.append(transport.emergency_stop(timeout=0.5)))
    thread.start()
    deadline = time.time() + 0.5
    request = None
    while time.time() < deadline:
        sent = [json.loads(message) for message in _ws_app().sent]
        matches = [message for message in sent
                   if message.get("title") == "request_emgy_stop"]
        if matches:
            request = matches[-1]
            break
        time.sleep(0.005)
    assert request is not None, "no emergency-stop request was sent"
    _ws_app().feed_message({
        "title": "response_emgy_stop",
        "guid": request["guid"],
        "data": {"result": result},
    })
    thread.join(timeout=1.0)
    assert outcome == [expected]


def test_disconnect_marks_disconnected(transport):
    transport.disconnect()
    assert not transport.is_connected()


def test_official_restart_routes_via_second_joint_for_low_ee_z(monkeypatch):
    t = WebsocketTransport.__new__(WebsocketTransport)
    init = [0.2] * JointIndex.MOVEJ_DIM
    second = [0.1] * JointIndex.MOVEJ_DIM
    t.config = Tron2Config(init_joints=init, init_ee_z_min=0.25)
    t._second_joint = second
    t.logger = _ws_mod.logging.getLogger("test-official-restart")
    t.get_joint_state = lambda timeout=1.0: {"states": [0.5] * JointIndex.STATE_DIM}
    t.get_ee_poses = lambda timeout=1.0: {
        "left_position": [0.0, 0.0, 0.2],
        "right_position": [0.0, 0.0, 0.4],
    }
    moves = []
    t.movej = lambda joints, move_time=2.0: moves.append(list(joints))
    t.set_gripper = lambda **kwargs: None
    monkeypatch.setattr(_ws_mod.time, "sleep", lambda _seconds: None)
    t._move_to_init_pose()
    assert moves == [second, init]
