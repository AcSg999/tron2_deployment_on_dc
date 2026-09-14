"""Linear interpolation between joint waypoints."""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

_EPS = 1e-9


class LinearInterpolator:
    """First-order linear interpolation between consecutive waypoints.

    Suitable for short eta (~1/fps) where higher-order curves wouldn't have
    room to differentiate. Behaviour mirrors what the env used to do with
    ``np.linspace + servoj`` but at the publish_rate granularity instead of
    a fixed ``trajectory_points`` count.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._t_start = 0.0
        self._t_end = 0.0
        self._q_start: Optional[np.ndarray] = None
        self._q_end: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ public

    def reset(self, q: np.ndarray) -> None:
        q_arr = np.asarray(q, dtype=np.float64).copy()
        now = time.perf_counter()
        with self._lock:
            self._t_start = now
            self._t_end = now
            self._q_start = q_arr
            self._q_end = q_arr.copy()

    def set_destination(
        self,
        target: np.ndarray,
        eta: float,
        *,
        now: Optional[float] = None,
    ) -> None:
        target_arr = np.asarray(target, dtype=np.float64).copy()
        if now is None:
            now = time.perf_counter()
        with self._lock:
            if self._q_end is None:
                # First set_destination before reset — initialise from target.
                self._q_start = target_arr.copy()
                self._q_end = target_arr.copy()
                self._t_start = now
                self._t_end = now
                return
            if target_arr.shape != self._q_end.shape:
                raise ValueError(
                    f"target shape {target_arr.shape} != current {self._q_end.shape}"
                )
            self._q_start = self._current_unlocked(now).copy()
            self._q_end = target_arr
            self._t_start = now
            self._t_end = now + max(eta, 0.0)

    def current(self, t: Optional[float] = None) -> np.ndarray:
        with self._lock:
            return self._current_unlocked(t)

    def at_destination(self, t: Optional[float] = None) -> bool:
        if t is None:
            t = time.perf_counter()
        with self._lock:
            return t >= self._t_end

    # ----------------------------------------------------------------- private

    def _current_unlocked(self, t: Optional[float]) -> np.ndarray:
        if self._q_end is None or self._q_start is None:
            raise RuntimeError("interpolator not initialised — call reset() or set_destination()")
        if t is None:
            t = time.perf_counter()
        span = self._t_end - self._t_start
        if span <= _EPS or t >= self._t_end:
            return self._q_end.copy()
        if t <= self._t_start:
            return self._q_start.copy()
        alpha = (t - self._t_start) / span
        return self._q_start + alpha * (self._q_end - self._q_start)
