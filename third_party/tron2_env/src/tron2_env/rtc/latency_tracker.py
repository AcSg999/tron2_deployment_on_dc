"""Latency tracking utilities for Real-Time Chunking (RTC)."""

import logging
from collections import deque

import numpy as np

logger = logging.getLogger(__name__)


class LatencyTracker:
    """Tracks recent latencies and provides max/percentile queries.

    Args:
        maxlen: Sliding window size. If provided, only the most recent
            ``maxlen`` latencies are kept. If None, keeps all.
        max_cap_s: Per-sample upper bound (seconds) used when computing
            percentile/p95 to suppress outliers such as JAX JIT recompilation
            spikes. Raw values are still recorded; only the percentile query
            clamps each sample to this cap. Set to None or <=0 to disable.
    """

    def __init__(self, maxlen: int = 100, max_cap_s: float | None = 0.6):
        self._values = deque(maxlen=maxlen)
        self._max_cap_s = max_cap_s if (max_cap_s is None or max_cap_s > 0) else None
        self.reset()

    def reset(self) -> None:
        """Clear all recorded latencies."""
        self._values.clear()
        self.max_latency = 0.0

    def add(self, latency: float) -> None:
        """Add a latency sample (seconds)."""
        val = float(latency)
        if val < 0:
            return
        self._values.append(val)
        self.max_latency = max(self.max_latency, val)
        logger.debug("LatencyTracker: added %.1fms, max=%.1fms, count=%d",
                     val * 1000, self.max_latency * 1000, len(self._values))

    def __len__(self) -> int:
        return len(self._values)

    def max(self) -> float | None:
        """Return the maximum latency or None if empty."""
        return self.max_latency

    def percentile(self, q: float) -> float | None:
        """Return the q-quantile (q in [0,1]) of recorded latencies or None if empty.

        Samples are individually clamped to ``max_cap_s`` before the quantile is
        taken, so a single cold-start or JIT-recompile spike cannot drag the
        steady-state estimate up. The raw ``max_latency`` is unaffected.
        """
        if not self._values:
            return 0.0
        q = float(q)
        vals = np.array(list(self._values), dtype=np.float32)
        if self._max_cap_s is not None:
            vals = np.minimum(vals, self._max_cap_s)
        if q <= 0.0:
            return float(vals.min())
        if q >= 1.0:
            return float(vals.max())
        return float(np.quantile(vals, q))

    def p95(self) -> float | None:
        """Return the 95th percentile latency or None if empty."""
        return self.percentile(0.95)

    def summary(self) -> str:
        """Return a human-readable summary of tracked latencies."""
        if not self._values:
            return "LatencyTracker: no data"
        vals = np.array(list(self._values), dtype=np.float32)
        capped_note = ""
        if self._max_cap_s is not None:
            capped = np.minimum(vals, self._max_cap_s)
            capped_note = (
                f", p95_capped({self._max_cap_s*1000:.0f}ms)={np.quantile(capped, 0.95)*1000:.1f}ms"
            )
        return (
            f"LatencyTracker: n={len(vals)}, "
            f"mean={vals.mean()*1000:.1f}ms, "
            f"p50={np.quantile(vals, 0.5)*1000:.1f}ms, "
            f"p95={np.quantile(vals, 0.95)*1000:.1f}ms{capped_note}, "
            f"max={self.max_latency*1000:.1f}ms"
        )
