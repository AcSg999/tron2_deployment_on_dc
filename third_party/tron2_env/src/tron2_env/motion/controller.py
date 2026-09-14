"""MotionController — orchestrates transport + interpolator + publish loop.

This is the concrete object the environment talks to. It owns:

  * a :class:`RobotTransport`     — sends single frames to the robot
  * a :class:`JointInterpolator`  — produces smooth ``current(t)`` between waypoints
  * a daemon publish thread       — drives the transport at ``publish_rate`` Hz

The controller is transport-agnostic, but the public runtime currently exposes
the WebSocket transport only.

Call ``command_joints(target)`` to retarget (non-blocking). The publish
thread keeps the robot at the latest target indefinitely thanks to PD on
the robot side.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import numpy as np

from tron2_env.config import Tron2Config
from tron2_env.errors import StateError
from tron2_env.interpolation import JointInterpolator, LinearInterpolator
from tron2_env.joints import JointIndex
from tron2_env.transport import RobotTransport, WebsocketTransport
from tron2_env.util import RateLimiter

logger = logging.getLogger(__name__)

# The TRON2 high-level SDK explicitly requires ServoJ at >=500 Hz.  ``MoveJ``
# is a separate, controller-interpolated command and is not governed by this
# setting.  Keep the guard at the WebSocket factory boundary so generic/mock
# MotionController tests can still use slower rates without implying hardware
# safety.
MIN_WEBSOCKET_SERVOJ_RATE_HZ = 500.0


class MotionController:
    """Owns a transport + interpolator + publish loop."""

    def __init__(
        self,
        transport: RobotTransport,
        interpolator: Optional[JointInterpolator] = None,
        publish_rate: float = 500.0,
        eta_default: float = 1.0 / 30.0,
    ) -> None:
        required_rate = float(getattr(transport, "min_servoj_rate_hz", 0.0))
        if publish_rate < required_rate:
            raise ValueError(
                f"this transport requires publish_rate >= {required_rate:.0f} Hz")
        self._transport = transport
        self._interpolator = interpolator or LinearInterpolator()
        self._publish_rate = publish_rate
        self._eta_default = eta_default

        self._shutdown = threading.Event()
        self._publish_thread: Optional[threading.Thread] = None
        self._started = False

    # ------------------------------------------------------------- lifecycle

    def start(self) -> None:
        """Seed the interpolator with the current measured q, then launch the publish loop.

        Reading the first state before publishing is mandatory: otherwise the
        publish loop would broadcast a default-initialised q and yank the robot.
        """
        if self._started:
            return
        try:
            state = self._transport.get_joint_state(timeout=2.0)
        except StateError as exc:
            raise RuntimeError(
                "MotionController.start: could not read initial joint state — "
                "is the transport connected and producing state?"
            ) from exc

        q_now_18 = np.asarray(state["states"], dtype=np.float64)
        q_now_16 = self._state_to_servoj(q_now_18)
        self._interpolator.reset(q_now_16)

        self._publish_thread = threading.Thread(
            target=self._publish_loop,
            daemon=True,
            name="MotionController-publish",
        )
        self._publish_thread.start()
        self._started = True
        logger.info(
            "MotionController started: publish_rate=%.1fHz eta_default=%.1fms",
            self._publish_rate,
            self._eta_default * 1000.0,
        )

    def disconnect(self) -> None:
        self._shutdown.set()
        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=1.0)
        self._transport.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    # ------------------------------------------------------------ public API

    def command_joints(
        self,
        target: np.ndarray,
        eta: Optional[float] = None,
    ) -> None:
        """Retarget. Non-blocking: just updates the interpolator destination."""
        self._interpolator.set_destination(
            np.asarray(target, dtype=np.float64),
            eta if eta is not None else self._eta_default,
        )

    def set_gripper(self, left_opening: float, right_opening: float) -> None:
        self._transport.set_gripper(left_opening, right_opening)

    def get_joint_states(self, timeout: float = 1.0) -> dict:
        return self._transport.get_joint_state(timeout)

    def find_nearest_state(self, target_timestamp_s: float):
        """Find the queued state closest to target_timestamp_s (non-destructive).

        Returns ``None`` if the underlying transport doesn't support search or
        if the queue is empty. Useful for image/state alignment in legacy obs.
        """
        finder = getattr(self._transport, "find_nearest_state", None)
        if finder is None:
            return None
        return finder(target_timestamp_s)

    def get_head_position(self) -> np.ndarray:
        return self._transport.get_head_position()

    def wait_until_reached(self, *args, **kwargs) -> bool:
        return self._transport.wait_until_reached(*args, **kwargs)

    def is_connected(self) -> bool:
        return self._transport.is_connected()

    @property
    def transport(self) -> RobotTransport:
        """Exposes the underlying transport for backend-specific operations (e.g. movej during reset)."""
        return self._transport

    # ----------------------------------------------------------------- loop

    def _publish_loop(self) -> None:
        rate = RateLimiter(self._publish_rate)
        while not self._shutdown.is_set():
            try:
                q = self._interpolator.current()
                self._transport.send_joint_cmd(q)
            except Exception as exc:  # noqa: BLE001
                # Don't let one transport hiccup kill the loop; the interpolator
                # still holds the latest target, so we'll retry next tick.
                logger.warning("publish loop: %s", exc)
            rate.sleep()

    # ---------------------------------------------------------------- helpers

    @staticmethod
    def _state_to_servoj(state18: np.ndarray) -> np.ndarray:
        """Convert an 18-dim ``[L_arm, L_grip, R_arm, R_grip, head]`` state
        into the 16-dim ``[L_arm, R_arm, head]`` setpoint vector."""
        return np.concatenate(
            [
                state18[JointIndex.LEFT_ARM],
                state18[JointIndex.RIGHT_ARM],
                state18[JointIndex.HEAD],
            ]
        )


# ---------------------------------------------------------------------- factory


def create_motion_controller(
    config: Tron2Config,
    backend: str = "websocket",
    publish_rate: float = MIN_WEBSOCKET_SERVOJ_RATE_HZ,
    eta_default: float = 1.0 / 30.0,
    interpolator: Optional[JointInterpolator] = None,
) -> MotionController:
    """Build a started MotionController with the requested backend.

    Args:
        config: Robot connection + bring-up params.
        backend: ``"websocket"``.
        publish_rate: Hz at which the publish loop drives the transport.
        eta_default: default time-to-target for ``command_joints`` (= 1/fps).
        interpolator: override the default ``LinearInterpolator``.
    """
    if backend != "websocket":
        raise ValueError(f"unknown control backend: {backend!r} (expected 'websocket')")
    if publish_rate < MIN_WEBSOCKET_SERVOJ_RATE_HZ:
        raise ValueError(
            "TRON2 WebSocket ServoJ requires publish_rate >= "
            f"{MIN_WEBSOCKET_SERVOJ_RATE_HZ:.0f} Hz (vendor safety requirement)")
    transport: RobotTransport = WebsocketTransport(config)

    mc = MotionController(
        transport=transport,
        interpolator=interpolator,
        publish_rate=publish_rate,
        eta_default=eta_default,
    )
    mc.start()
    return mc
