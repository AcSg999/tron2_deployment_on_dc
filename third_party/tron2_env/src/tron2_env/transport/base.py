"""RobotTransport abstraction — the bottom layer of the control stack.

A transport owns the connection to the robot and exposes ONLY single-shot
primitives:

  * ``send_joint_cmd(q[16])``                — send one joint setpoint
  * ``get_joint_state(timeout)``             — read latest 18-dim state
  * ``get_head_position()``                  — read latest head [pitch, yaw]
  * ``set_gripper(left, right)``             — drive grippers
  * ``wait_until_reached(target, tol, t)``   — blocking polling helper
  * ``disconnect`` / ``is_connected``        — lifecycle

No rate limiting, no interpolation, no publish loop — those live in
``tron2_env.motion.MotionController``. This keeps transports trivially
testable (mock socket / fake SDK) and lets the same publish loop drive any
backend.

Concrete implementation:
  * ``tron2_env.transport.websocket.WebsocketTransport`` — JSON over ws://...
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class RobotTransport(Protocol):
    """Single-shot send/recv primitives. No internal pacing."""

    def send_joint_cmd(self, q: np.ndarray) -> None:
        """Send a 16-dim joint setpoint [L_arm(7), R_arm(7), head(2)].

        Non-blocking; the actual on-the-wire send may be async. Raises
        ``CommandError`` if the transport is disconnected or the array is
        the wrong shape.
        """

    def get_joint_state(self, timeout: float = 1.0) -> dict:
        """Return the latest 18-dim state.

        Shape::

            {
                "timestamp": int (ms),
                "states": list[float] (18),  # [L_arm(7), L_grip, R_arm(7), R_grip, head(2)]
                ...
            }

        Raises ``StateError`` on timeout.
        """

    def get_head_position(self) -> np.ndarray:
        """Return the latest 2-dim head [pitch, yaw] without dequeuing state."""

    def set_gripper(self, left_opening: float, right_opening: float) -> None:
        """Set gripper opening 0..100 for left/right."""

    def wait_until_reached(
        self,
        target_joints,
        tolerance: float = 0.05,
        timeout: float = 10.0,
    ) -> bool:
        """Block until arm joints are within ``tolerance`` rad of ``target_joints`` (14-dim)."""

    def disconnect(self) -> None:
        """Close the connection and stop any internal threads."""

    def is_connected(self) -> bool: ...

    def __enter__(self): ...

    def __exit__(self, exc_type, exc_val, exc_tb): ...
