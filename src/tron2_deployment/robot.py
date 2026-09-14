"""Lazy robot adapters. Importing or constructing an adapter never moves hardware."""

from __future__ import annotations

import math
import time
from copy import deepcopy
from typing import Any, Callable

import numpy as np


def _vector(values: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


class MockRobot:
    """Deterministic feedback adapter; it never imports the hardware transport."""

    source = "mock"

    def __init__(self, arm_q14=None, head_q2=None, *, clock: Callable = time.time):
        if isinstance(arm_q14, dict):
            profile = arm_q14
            demo = profile.get("demo", {})
            if "arm_q14" not in demo:
                raise ValueError("mock profile requires demo.arm_q14")
            arm_q14 = demo["arm_q14"]
            if head_q2 is None:
                head_q2 = demo.get("head_q2", profile.get("calibration", {}).get("head_q2"))
            if head_q2 is None:
                raise ValueError("mock profile requires demo.head_q2 or calibration.head_q2")
        self.arm_q14 = _vector([0.0] * 14 if arm_q14 is None else arm_q14, 14, "arm_q14")
        self.head_q2 = _vector([0.0] * 2 if head_q2 is None else head_q2, 2, "head_q2")
        self.clock = clock
        self.commands: list[dict] = []
        self.closed = False

    def read_state(self) -> dict:
        if self.closed:
            raise RuntimeError("robot adapter is closed")
        return {"arm_q14": self.arm_q14.tolist(), "head_q2": self.head_q2.tolist(),
                "timestamp_s": self.clock(), "source": self.source}

    def command(self, arm_q14, head_q2) -> None:
        if self.closed:
            raise RuntimeError("robot adapter is closed")
        self.arm_q14 = _vector(arm_q14, 14, "arm_q14").copy()
        self.head_q2 = _vector(head_q2, 2, "head_q2").copy()
        self.commands.append(self.read_state())

    def close(self) -> None:
        self.closed = True


def _new_transport(robot_config: dict):
    """Import the pinned runtime only on an explicit real feedback request."""
    from tron2_env.config import Tron2Config
    from tron2_env.transport.websocket import WebsocketTransport

    class ArmFeedbackTransport(WebsocketTransport):
        def _poll_feedback(self):
            # The upstream combined queue waits for gripper feedback. This
            # application only needs arm/head feedback and sends no claw requests.
            period = 1.0 / self.config.polling_rate
            while not self.should_exit:
                started = time.monotonic()
                self._send_request("request_get_joint_state")
                time.sleep(max(0.0, period - (time.monotonic() - started)))

    return ArmFeedbackTransport(Tron2Config(
        robot_ip=robot_config["host"], port=int(robot_config.get("port", 5000)),
        init_joints=None, init_head=None, init_ee_z_min=None,
        polling_rate=float(robot_config.get("feedback_hz", 200.0)),
        connection_timeout=float(robot_config.get("connection_timeout_s", 5.0)),
    ))


class WebsocketRobot:
    """Read-only until the executor authorizes one validated plan.

    A socket write is only transport success. Arrival and feedback freshness
    are verified by the executor, independently of command submission.
    """

    source = "real"

    def __init__(self, profile: dict, *, transport_factory: Callable | None = None):
        self.profile = deepcopy(profile)
        self._factory = transport_factory or _new_transport
        self._transport = None
        self._authorized_plan_id = None
        self._closed = False
        self._received_state = False

    def _connect(self):
        if self._closed:
            raise RuntimeError("robot adapter is closed")
        if self._transport is None:
            cfg = self.profile.get("robot", {})
            if not isinstance(cfg.get("host"), str) or not cfg["host"].strip():
                raise ValueError("robot.host must explicitly identify the controller")
            for key, default in (("feedback_hz", 200.0), ("connection_timeout_s", 5.0)):
                value = float(cfg.get(key, default))
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"robot.{key} must be positive and finite")
            self._transport = self._factory(cfg)
        return self._transport

    def read_state(self) -> dict:
        transport = self._connect()
        timeout = float(self.profile["robot"].get("connection_timeout_s", 5.0))
        deadline = time.monotonic() + timeout
        while True:
            # Snapshot the arm response directly: the vendor queue otherwise
            # requires a gripper message and can retain earlier samples.
            with transport._state_lock:
                raw = deepcopy(transport.joint_states)
            stamp = float(raw.get("timestamp", -1)) / 1000.0
            if transport.is_connected() and stamp > 0:
                state18 = _vector(raw.get("states"), 18, "TRON2 state")
                self._received_state = True
                return {"arm_q14": np.r_[state18[:7], state18[8:15]].tolist(),
                        "head_q2": state18[16:18].tolist(), "timestamp_s": stamp,
                        "robot_timestamp": raw.get("robot_timestamp"), "source": self.source}
            if self._received_state and not transport.is_connected():
                raise ConnectionError("TRON2 disconnected while reading arm/head feedback")
            if time.monotonic() >= deadline:
                raise TimeoutError("fresh TRON2 arm/head feedback was not received")
            time.sleep(0.001)

    def _authorize_execution(self, report: dict) -> None:
        if self._authorized_plan_id is not None:
            raise RuntimeError("an execution is already authorized on this adapter")
        if report.get("ok") is not True or report.get("real") is not True:
            raise ValueError("real commands require a successful real preflight")
        if not report.get("plan_id"):
            raise ValueError("preflight is missing the reviewed plan ID")
        self._authorized_plan_id = report["plan_id"]

    def _revoke_execution(self) -> None:
        self._authorized_plan_id = None

    def command(self, arm_q14, head_q2) -> None:
        if not self._authorized_plan_id:
            raise RuntimeError("adapter is read-only outside validated execution")
        arms = _vector(arm_q14, 14, "arm_q14")
        head = _vector(head_q2, 2, "head_q2")
        transport = self._connect()
        if not transport.is_connected():
            raise RuntimeError("TRON2 disconnected before ServoJ")
        # Patched send_joint_cmd raises when _send_request returns false; it
        # returns None after a successful socket write, never an arrival ACK.
        result = transport.send_joint_cmd(np.r_[arms, head])
        if result is False:
            raise RuntimeError("TRON2 rejected the ServoJ write")

    def close(self) -> None:
        self._revoke_execution()
        self._closed = True
        if self._transport is not None:
            self._transport.disconnect()
