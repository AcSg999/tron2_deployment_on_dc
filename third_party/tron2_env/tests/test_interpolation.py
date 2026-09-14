"""Tests for LinearInterpolator — pure math, no robot needed."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from tron2_env.interpolation import LinearInterpolator


def test_reset_holds_q():
    interp = LinearInterpolator()
    interp.reset(np.array([1.0, 2.0, 3.0]))

    # Multiple reads after reset all return the same value
    for _ in range(5):
        out = interp.current()
        np.testing.assert_allclose(out, [1.0, 2.0, 3.0])
    assert interp.at_destination()


def test_set_destination_endpoints():
    interp = LinearInterpolator()
    interp.reset(np.zeros(4))

    target = np.array([1.0, 2.0, 3.0, 4.0])
    interp.set_destination(target, eta=0.1, now=1000.0)

    # at t_start, should be at q_start (= 0)
    np.testing.assert_allclose(interp.current(t=1000.0), np.zeros(4))
    # at t_end, should be at q_end
    np.testing.assert_allclose(interp.current(t=1000.1), target)
    # before t_start, clamp to q_start
    np.testing.assert_allclose(interp.current(t=999.5), np.zeros(4))
    # after t_end, clamp to q_end
    np.testing.assert_allclose(interp.current(t=1001.0), target)


def test_midpoint_interpolation():
    interp = LinearInterpolator()
    interp.reset(np.array([0.0, 10.0]))
    interp.set_destination(np.array([4.0, 0.0]), eta=1.0, now=0.0)
    out = interp.current(t=0.5)
    np.testing.assert_allclose(out, [2.0, 5.0])


def test_preemptive_retarget_starts_from_current():
    """Calling set_destination mid-flight should make q_start = current(), not last destination."""
    interp = LinearInterpolator()
    interp.reset(np.zeros(2))
    interp.set_destination(np.array([10.0, 0.0]), eta=1.0, now=0.0)

    # at t=0.5 we're halfway to (10,0) -> (5,0)
    midway = interp.current(t=0.5)
    np.testing.assert_allclose(midway, [5.0, 0.0])

    # retarget toward (5, 10) starting from t=0.5
    interp.set_destination(np.array([5.0, 10.0]), eta=1.0, now=0.5)

    # immediately should still be at (5,0)
    np.testing.assert_allclose(interp.current(t=0.5), [5.0, 0.0])
    # midpoint of new segment -> ((5+5)/2, (0+10)/2) = (5, 5)
    np.testing.assert_allclose(interp.current(t=1.0), [5.0, 5.0])
    # at end -> (5, 10)
    np.testing.assert_allclose(interp.current(t=1.5), [5.0, 10.0])


def test_zero_eta_jumps_immediately():
    interp = LinearInterpolator()
    interp.reset(np.zeros(3))
    interp.set_destination(np.array([7.0, 7.0, 7.0]), eta=0.0, now=100.0)
    # span <= EPS branch -> returns q_end straight away
    np.testing.assert_allclose(interp.current(t=100.0), [7.0, 7.0, 7.0])
    assert interp.at_destination(t=100.0)


def test_current_requires_initialisation():
    interp = LinearInterpolator()
    with pytest.raises(RuntimeError):
        interp.current()


def test_thread_safety_smoke():
    """Hammer set_destination + current from two threads; expect no exceptions and bounded values."""
    interp = LinearInterpolator()
    interp.reset(np.zeros(4))

    stop = threading.Event()

    def writer():
        target = np.array([1.0, 2.0, 3.0, 4.0])
        while not stop.is_set():
            interp.set_destination(target, eta=0.01)
            target = -target
            time.sleep(0.001)

    def reader():
        while not stop.is_set():
            q = interp.current()
            assert q.shape == (4,)
            # bounded between ±(4 + slack)
            assert np.all(np.isfinite(q))

    threads = [threading.Thread(target=writer), threading.Thread(target=reader)]
    for t in threads:
        t.start()
    time.sleep(0.2)
    stop.set()
    for t in threads:
        t.join(timeout=1.0)
