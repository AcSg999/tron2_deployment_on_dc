"""Tests for MotionController — uses a FakeTransport, no robot needed."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from tron2_env.errors import StateError
from tron2_env.interpolation import LinearInterpolator
from tron2_env.joints import JointIndex
from tron2_env.config import Tron2Config
from tron2_env.motion import MotionController, create_motion_controller


class FakeTransport:
    """Minimal transport that records every send_joint_cmd call."""

    def __init__(self, initial_state=None, fail_state: bool = False):
        if initial_state is None:
            initial_state = list(np.zeros(JointIndex.STATE_DIM, dtype=np.float64))
        self._state = list(initial_state)
        self._fail_state = fail_state
        self._sends: list = []  # list[(timestamp, q)]
        self._lock = threading.Lock()
        self.connected = True
        self.gripper_calls: list = []
        self.disconnected = False

    # RobotTransport API

    def send_joint_cmd(self, q):
        with self._lock:
            self._sends.append((time.perf_counter(), np.asarray(q, dtype=np.float64).copy()))

    def get_joint_state(self, timeout: float = 1.0):
        if self._fail_state:
            raise StateError("fake transport configured to fail")
        return {"timestamp": int(time.time() * 1000), "states": list(self._state)}

    def get_head_position(self):
        return np.asarray(self._state[JointIndex.HEAD], dtype=np.float64)

    def set_gripper(self, left, right):
        self.gripper_calls.append((left, right))

    def wait_until_reached(self, target, tolerance=0.05, timeout=10.0):
        return True

    def disconnect(self):
        self.disconnected = True

    def is_connected(self):
        return self.connected

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    # introspection

    def sends(self):
        with self._lock:
            return list(self._sends)


def test_websocket_factory_rejects_unsafe_servoj_rate_before_connecting():
    with pytest.raises(ValueError, match="requires publish_rate >= 500"):
        create_motion_controller(Tron2Config(robot_ip="unused"), publish_rate=499.0)


def test_start_seeds_interpolator_from_transport_state():
    """First publish should match the measured q, not zeros."""
    state18 = np.zeros(JointIndex.STATE_DIM)
    state18[JointIndex.LEFT_ARM] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    state18[JointIndex.RIGHT_ARM] = [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7]
    state18[JointIndex.HEAD] = [0.8, -0.8]
    fake = FakeTransport(initial_state=state18)

    mc = MotionController(transport=fake, interpolator=LinearInterpolator(), publish_rate=200.0)
    mc.start()
    time.sleep(0.05)  # let a few publish cycles run

    try:
        sends = fake.sends()
        assert sends, "publish loop should have sent at least one frame"
        first_q = sends[0][1]
        expected = np.concatenate([state18[JointIndex.LEFT_ARM], state18[JointIndex.RIGHT_ARM], state18[JointIndex.HEAD]])
        np.testing.assert_allclose(first_q, expected, atol=1e-9)
    finally:
        mc.disconnect()


def test_publish_rate_close_to_requested():
    fake = FakeTransport()
    mc = MotionController(transport=fake, publish_rate=200.0)
    mc.start()
    try:
        time.sleep(0.6)
        sends = fake.sends()
    finally:
        mc.disconnect()

    assert len(sends) >= 50, f"expected ~120 sends in 0.6s @ 200Hz, got {len(sends)}"
    # rough rate sanity — within 25% of requested
    duration = sends[-1][0] - sends[0][0]
    measured_rate = (len(sends) - 1) / max(duration, 1e-6)
    assert 150.0 <= measured_rate <= 260.0, f"measured rate {measured_rate:.1f} far from 200"


def test_command_joints_non_blocking_and_retargets():
    fake = FakeTransport()
    mc = MotionController(transport=fake, publish_rate=200.0, eta_default=0.05)
    mc.start()
    try:
        # Initially publishes zeros. After command_joints, the published q
        # should converge toward the new target within eta.
        target = np.ones(JointIndex.SERVOJ_DIM) * 0.5

        t_before = time.perf_counter()
        mc.command_joints(target)
        elapsed = time.perf_counter() - t_before
        assert elapsed < 0.01, f"command_joints should be non-blocking, took {elapsed*1000:.1f}ms"

        time.sleep(0.15)  # well past eta
        sends = fake.sends()
    finally:
        mc.disconnect()

    last_q = sends[-1][1]
    np.testing.assert_allclose(last_q, target, atol=1e-6)


def test_disconnect_stops_publish_thread():
    fake = FakeTransport()
    mc = MotionController(transport=fake, publish_rate=200.0)
    mc.start()
    time.sleep(0.05)
    mc.disconnect()

    assert fake.disconnected
    # no new sends after disconnect
    snapshot = len(fake.sends())
    time.sleep(0.05)
    assert len(fake.sends()) == snapshot


def test_start_fails_when_transport_has_no_state():
    fake = FakeTransport(fail_state=True)
    mc = MotionController(transport=fake, publish_rate=200.0)
    with pytest.raises(RuntimeError, match="initial joint state"):
        mc.start()


def test_set_gripper_delegates():
    fake = FakeTransport()
    mc = MotionController(transport=fake, publish_rate=200.0)
    mc.start()
    try:
        mc.set_gripper(75.0, 25.0)
        assert fake.gripper_calls == [(75.0, 25.0)]
    finally:
        mc.disconnect()


def test_get_head_position_delegates():
    state18 = np.zeros(JointIndex.STATE_DIM)
    state18[JointIndex.HEAD] = [1.2, -0.3]
    fake = FakeTransport(initial_state=state18)
    mc = MotionController(transport=fake, publish_rate=200.0)
    mc.start()
    try:
        np.testing.assert_allclose(mc.get_head_position(), [1.2, -0.3])
    finally:
        mc.disconnect()
