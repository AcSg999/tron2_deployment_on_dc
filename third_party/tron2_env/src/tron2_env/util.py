"""Small utilities shared across the package."""

from __future__ import annotations

import time


class RateLimiter:
    """Fixed-frequency loop pacer based on absolute monotonic clock.

    Use case: ``while running: do_work(); rate.sleep()``. Catches up after a
    missed deadline by resetting next_tick to *now + period*, so a single
    overrun doesn't accumulate phase error.
    """

    def __init__(self, rate_hz: float) -> None:
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.next_tick = time.monotonic()

    def sleep(self) -> None:
        now = time.monotonic()
        delay = self.next_tick - now
        if delay > 0:
            time.sleep(delay)
        self.next_tick += self.period
        if self.next_tick < time.monotonic():
            self.next_tick = time.monotonic() + self.period

    def reset(self) -> None:
        self.next_tick = time.monotonic()
