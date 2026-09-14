# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 LimX Dynamics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by LimX Dynamics in 2026 from Hugging Face LeRobot commit
# ca87ccd9413c59c30f524967222d2e3f1b7bb549. The modifications port queue
# storage from PyTorch tensors to NumPy arrays, replace RTCConfig coupling with
# runtime arguments, and add atomic snapshots, delay handling, and merge-boundary
# diagnostics for the TRON2 runtime.

"""Action queue management for Real-Time Chunking (RTC).

Adapted from LeRobot's ActionQueue (PyTorch) to use numpy.
"""

import logging
from threading import Lock

import numpy as np

logger = logging.getLogger(__name__)

PROCESSED_ARM_INDICES = tuple(range(7)) + tuple(range(8, 15))
PROCESSED_GRIPPER_INDICES = (7, 15)


def _safe_dim(value: float, default: int = -1) -> int:
    if np.isnan(value):
        return default
    return int(value)


class ActionQueue:
    """Thread-safe queue for managing action chunks in real-time control.

    Two types of action sequences:
    - Original actions: Used for RTC to compute leftovers from previous chunks
    - Processed actions: Post-processed actions ready for robot execution

    Two modes:
    1. RTC-enabled: Replaces the entire queue with new actions, accounting for inference delay
    2. RTC-disabled: Appends new actions to the queue, maintaining continuity
    """

    def __init__(
        self,
        rtc_enabled: bool = False,
    ):
        self.queue: np.ndarray | None = None  # Processed actions (T, action_dim)
        self.original_queue: np.ndarray | None = None  # Original actions for RTC (T, action_dim)
        self.lock = Lock()
        self.last_index = 0
        self.rtc_enabled = rtc_enabled
        self._merge_count = 0
        self._last_merge_diagnostics: dict[str, float] = {}

    def get(self) -> np.ndarray | None:
        """Get the next action from the queue."""
        with self.lock:
            if self.queue is None or self.last_index >= len(self.queue):
                return None
            action = self.queue[self.last_index].copy()
            self.last_index += 1
            return action

    def clear(self) -> None:
        """Clear queued actions and reset consumption index."""
        with self.lock:
            self.queue = None
            self.original_queue = None
            self.last_index = 0

    def qsize(self) -> int:
        """Get the number of remaining actions in the queue."""
        with self.lock:
            if self.queue is None:
                return 0
            return len(self.queue) - self.last_index

    def empty(self) -> bool:
        """Check if the queue is empty."""
        if self.queue is None:
            return True
        return len(self.queue) - self.last_index <= 0

    def get_action_index(self) -> int:
        """Get the current action consumption index."""
        with self.lock:
            return self.last_index

    def snapshot_left_over(self) -> tuple[int, np.ndarray | None, int]:
        """Atomically snapshot the current index and original-action leftover.

        RTC needs ``s`` and ``A_cur[s:]`` from the same instant. Calling
        get_action_index() and get_left_over() separately can race with the
        consumer thread and produce an inconsistent snapshot.
        """
        with self.lock:
            index = self.last_index
            qsize = 0 if self.queue is None else len(self.queue) - index
            if self.original_queue is None:
                return index, None, qsize
            return index, self.original_queue[index:].copy(), qsize

    def get_left_over(self) -> np.ndarray | None:
        """Get leftover original actions for RTC prev_chunk_left_over.

        These are the unconsumed actions from the current chunk in normalized
        (model) space, which will be used by RTC to compute corrections.
        """
        with self.lock:
            if self.original_queue is None:
                return None
            return self.original_queue[self.last_index :].copy()

    def get_processed_left_over(self) -> np.ndarray | None:
        """Get leftover processed actions (the actions currently being executed)."""
        with self.lock:
            if self.queue is None:
                return None
            return self.queue[self.last_index :].copy()

    def get_last_merge_diagnostics(self) -> dict[str, float]:
        """Return diagnostics from the most recent merge."""
        with self.lock:
            return dict(self._last_merge_diagnostics)

    def merge(
        self,
        original_actions: np.ndarray,
        processed_actions: np.ndarray,
        real_delay: int,
        action_index_before_inference: int | None = None,
        extra_delay: int = 0,
    ) -> int:
        """Merge new actions into the queue.

        Args:
            original_actions: Unprocessed actions from policy (T, action_dim) in normalized space.
            processed_actions: Post-processed actions for robot (T, action_dim) in real space.
            real_delay: Number of time steps of inference delay.
            action_index_before_inference: Index before inference started, for validation.
            extra_delay: Additional frames to discard for observation staleness.
        """
        with self.lock:
            delay = self._check_and_resolve_delays(real_delay, action_index_before_inference, extra_delay)

            # Debug: pre-merge state
            pre_qsize = len(self.queue) - self.last_index if self.queue is not None else 0
            pre_last_idx = self.last_index
            diagnostics = self._compute_boundary_diagnostics(original_actions, processed_actions, delay)

            if self.rtc_enabled:
                self._replace_actions_queue(original_actions, processed_actions, delay)
            else:
                self._append_actions_queue(original_actions, processed_actions)

            self._merge_count += 1
            diagnostics.update({
                "merge_count": float(self._merge_count),
                "merge_used_delay": float(delay),
                "merge_extra_delay": float(extra_delay),
                "merge_pre_qsize": float(pre_qsize),
                "merge_pre_index": float(pre_last_idx),
            })
            self._last_merge_diagnostics = diagnostics

            # Debug: post-merge state
            post_qsize = len(self.queue) - self.last_index if self.queue is not None else 0
            logger.debug(
                "merge #%d: pre_qsize=%d, post_qsize=%d, pre_idx=%d, post_idx=%d, "
                "delay=%d, extra_delay=%d, rtc=%s, orig=%s, proc=%s",
                self._merge_count, pre_qsize, post_qsize, pre_last_idx, self.last_index,
                delay, extra_delay, self.rtc_enabled,
                original_actions.shape, processed_actions.shape,
            )
            if diagnostics:
                logger.info(
                    "boundary #%d: delay=%d extra=%d proc_plan=max %.4f@%d d=%.4f mae %.4f "
                    "proc_exec=max %.4f@%d d=%.4f mae %.4f "
                    "arm_plan=max %.4f@%d d=%.4f mae %.4f arm_exec=max %.4f@%d d=%.4f mae %.4f "
                    "grip_plan=max %.4f@%d d=%.4f mae %.4f raw_plan=max %.4f@%d d=%.4f mae %.4f",
                    self._merge_count,
                    delay,
                    extra_delay,
                    diagnostics.get("boundary_proc_plan_max", np.nan),
                    _safe_dim(diagnostics.get("boundary_proc_plan_max_dim", np.nan)),
                    diagnostics.get("boundary_proc_plan_max_delta", np.nan),
                    diagnostics.get("boundary_proc_plan_mae", np.nan),
                    diagnostics.get("boundary_proc_exec_max", np.nan),
                    _safe_dim(diagnostics.get("boundary_proc_exec_max_dim", np.nan)),
                    diagnostics.get("boundary_proc_exec_max_delta", np.nan),
                    diagnostics.get("boundary_proc_exec_mae", np.nan),
                    diagnostics.get("boundary_proc_plan_arm_max", np.nan),
                    _safe_dim(diagnostics.get("boundary_proc_plan_arm_max_dim", np.nan)),
                    diagnostics.get("boundary_proc_plan_arm_max_delta", np.nan),
                    diagnostics.get("boundary_proc_plan_arm_mae", np.nan),
                    diagnostics.get("boundary_proc_exec_arm_max", np.nan),
                    _safe_dim(diagnostics.get("boundary_proc_exec_arm_max_dim", np.nan)),
                    diagnostics.get("boundary_proc_exec_arm_max_delta", np.nan),
                    diagnostics.get("boundary_proc_exec_arm_mae", np.nan),
                    diagnostics.get("boundary_proc_plan_gripper_max", np.nan),
                    _safe_dim(diagnostics.get("boundary_proc_plan_gripper_max_dim", np.nan)),
                    diagnostics.get("boundary_proc_plan_gripper_max_delta", np.nan),
                    diagnostics.get("boundary_proc_plan_gripper_mae", np.nan),
                    diagnostics.get("boundary_raw_plan_max", np.nan),
                    _safe_dim(diagnostics.get("boundary_raw_plan_max_dim", np.nan)),
                    diagnostics.get("boundary_raw_plan_max_delta", np.nan),
                    diagnostics.get("boundary_raw_plan_mae", np.nan),
                )
            return delay

    def _compute_boundary_diagnostics(
        self,
        original_actions: np.ndarray,
        processed_actions: np.ndarray,
        delay: int,
    ) -> dict[str, float]:
        """Measure old/new chunk continuity at the replacement boundary."""
        diagnostics: dict[str, float] = {}
        clamped_delay = max(0, min(delay, len(original_actions), len(processed_actions)))

        new_proc = processed_actions[clamped_delay] if clamped_delay < len(processed_actions) else None
        new_raw = original_actions[clamped_delay] if clamped_delay < len(original_actions) else None
        old_next_proc = (
            self.queue[self.last_index]
            if self.queue is not None and self.last_index < len(self.queue)
            else None
        )
        old_prev_proc = (
            self.queue[self.last_index - 1]
            if self.queue is not None and self.last_index > 0 and self.last_index - 1 < len(self.queue)
            else None
        )
        old_next_raw = (
            self.original_queue[self.last_index]
            if self.original_queue is not None and self.last_index < len(self.original_queue)
            else None
        )

        diagnostics.update(self._delta_metrics("boundary_proc_plan", old_next_proc, new_proc))
        diagnostics.update(self._delta_metrics("boundary_proc_exec", old_prev_proc, new_proc))
        diagnostics.update(self._delta_metrics("boundary_raw_plan", old_next_raw, new_raw))
        diagnostics.update(self._delta_metrics("boundary_proc_plan_arm", old_next_proc, new_proc, PROCESSED_ARM_INDICES))
        diagnostics.update(self._delta_metrics("boundary_proc_exec_arm", old_prev_proc, new_proc, PROCESSED_ARM_INDICES))
        diagnostics.update(
            self._delta_metrics("boundary_proc_plan_gripper", old_next_proc, new_proc, PROCESSED_GRIPPER_INDICES)
        )
        diagnostics.update(
            self._delta_metrics("boundary_proc_exec_gripper", old_prev_proc, new_proc, PROCESSED_GRIPPER_INDICES)
        )
        diagnostics["boundary_new_index"] = float(clamped_delay)
        return diagnostics

    @staticmethod
    def _delta_metrics(
        prefix: str,
        old: np.ndarray | None,
        new: np.ndarray | None,
        indices: tuple[int, ...] | None = None,
    ) -> dict[str, float]:
        if old is None or new is None:
            return {
                f"{prefix}_mae": float("nan"),
                f"{prefix}_max": float("nan"),
                f"{prefix}_max_dim": float("nan"),
                f"{prefix}_max_delta": float("nan"),
            }

        old_arr = np.asarray(old, dtype=np.float64).reshape(-1)
        new_arr = np.asarray(new, dtype=np.float64).reshape(-1)
        dim = min(old_arr.shape[0], new_arr.shape[0])
        if indices is not None:
            valid_indices = [idx for idx in indices if idx < dim]
            if not valid_indices:
                return {
                    f"{prefix}_mae": float("nan"),
                    f"{prefix}_max": float("nan"),
                    f"{prefix}_max_dim": float("nan"),
                    f"{prefix}_max_delta": float("nan"),
                }
            old_arr = old_arr[valid_indices]
            new_arr = new_arr[valid_indices]
            dim_indices = np.asarray(valid_indices)
            dim = len(valid_indices)
        else:
            dim_indices = np.arange(dim)
        if dim == 0:
            return {
                f"{prefix}_mae": float("nan"),
                f"{prefix}_max": float("nan"),
                f"{prefix}_max_dim": float("nan"),
                f"{prefix}_max_delta": float("nan"),
            }
        delta = new_arr[:dim] - old_arr[:dim]
        abs_delta = np.abs(delta)
        max_pos = int(np.argmax(abs_delta))
        max_dim = int(dim_indices[max_pos])
        return {
            f"{prefix}_mae": float(np.mean(abs_delta)),
            f"{prefix}_max": float(abs_delta[max_pos]),
            f"{prefix}_max_dim": float(max_dim),
            f"{prefix}_max_delta": float(delta[max_pos]),
        }

    def _replace_actions_queue(self, original_actions: np.ndarray, processed_actions: np.ndarray, real_delay: int):
        """Replace the queue with new actions (RTC mode)."""
        clamped_delay = max(0, min(real_delay, len(original_actions), len(processed_actions)))
        self.original_queue = original_actions[clamped_delay:].copy()
        self.queue = processed_actions[clamped_delay:].copy()
        self.last_index = 0

        logger.debug(
            "replace: orig=%s, proc=%s, real_delay=%d, clamped=%d, "
            "remaining_orig=%s, remaining_proc=%s",
            original_actions.shape, processed_actions.shape,
            real_delay, clamped_delay,
            self.original_queue.shape, self.queue.shape,
        )

    def _append_actions_queue(self, original_actions: np.ndarray, processed_actions: np.ndarray):
        """Append new actions to the queue (non-RTC mode)."""
        if self.queue is None:
            self.original_queue = original_actions.copy()
            self.queue = processed_actions.copy()
            return

        self.original_queue = np.concatenate([self.original_queue, original_actions.copy()])
        self.original_queue = self.original_queue[self.last_index :]

        self.queue = np.concatenate([self.queue, processed_actions.copy()])
        self.queue = self.queue[self.last_index :]

        self.last_index = 0

    def _check_and_resolve_delays(
        self,
        real_delay: int,
        action_index_before_inference: int | None = None,
        extra_delay: int = 0,
    ) -> int:
        """Validate that computed delays match expectations and return the
        delay that should actually drive the merge.

        ``real_delay`` is the wall-clock delay estimate. ``indexes_diff`` is
        how many frames the consumer actually advanced while inference was in
        flight, matching the RTC paper's observed delay ``t - s``.

        Resolution rule:
          - if action_index_before_inference is available, use the observed
            index delta because it is the same clock that drives queue reads,
            then add any explicit observation-staleness compensation.
          - otherwise fall back to real_delay (clamped to >= 0).
        """
        extra_delay = max(0, extra_delay)
        effective_delay = max(0, real_delay) + extra_delay

        if action_index_before_inference is not None:
            indexes_diff = max(0, self.last_index - action_index_before_inference)
            resolved = indexes_diff + extra_delay
            if indexes_diff != real_delay:
                log = logger.debug if abs(indexes_diff - real_delay) <= 1 else logger.warning
                log(
                    "Indexes diff != real delay. indexes_diff=%d, real_delay=%d, "
                    "extra_delay=%d, using=%d",
                    indexes_diff, real_delay, extra_delay, resolved,
                )
                return resolved
            return resolved

        return effective_delay
