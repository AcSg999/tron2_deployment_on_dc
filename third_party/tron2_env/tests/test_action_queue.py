import numpy as np

from tron2_env.rtc.action_queue import ActionQueue


def test_rtc_merge_uses_actual_consumed_index_when_latency_underestimates():
    queue = ActionQueue(rtc_enabled=True)
    old = np.arange(50, dtype=np.float32)[:, None]
    queue.merge(old, old, real_delay=0)

    action_index_before = queue.get_action_index()
    for _ in range(5):
        queue.get()

    new = np.arange(1000, 1050, dtype=np.float32)[:, None]
    queue.merge(new, new, real_delay=4, action_index_before_inference=action_index_before)

    np.testing.assert_allclose(queue.get(), [1005.0])


def test_rtc_merge_uses_actual_consumed_index_when_latency_overestimates():
    queue = ActionQueue(rtc_enabled=True)
    old = np.arange(50, dtype=np.float32)[:, None]
    queue.merge(old, old, real_delay=0)

    action_index_before = queue.get_action_index()
    for _ in range(4):
        queue.get()

    new = np.arange(1000, 1050, dtype=np.float32)[:, None]
    queue.merge(new, new, real_delay=5, action_index_before_inference=action_index_before)

    np.testing.assert_allclose(queue.get(), [1004.0])
