"""JointInterpolator — pure math primitive between waypoints.

Owned by :class:`tron2_env.motion.MotionController`. The controller asks
``current(t)`` once per publish tick to drive the transport; callers update
the destination via ``set_destination(target, eta)``.

Implementations must be thread-safe (publish loop reads, env.step writes).
"""

from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class JointInterpolator(Protocol):
    """Smoothly interpolate joint targets between waypoints over time."""

    def reset(self, q: np.ndarray) -> None:
        """Seed both start and destination to ``q`` — ``current()`` will return ``q`` until first set_destination."""

    def set_destination(
        self,
        target: np.ndarray,
        eta: float,
        *,
        now: Optional[float] = None,
    ) -> None:
        """Pre-emptively retarget.

        ``q_start = current(now)``, ``q_end = target``, ``t_start = now``,
        ``t_end = now + eta``. Calling this in the middle of an in-flight
        interpolation produces a kink-free curve because q_start is captured
        from the current trajectory value, not the last destination.
        """

    def current(self, t: Optional[float] = None) -> np.ndarray:
        """Return q at wall-time ``t`` (perf_counter; default = now)."""

    def at_destination(self, t: Optional[float] = None) -> bool:
        """True once t >= t_end (motion has reached its target)."""
