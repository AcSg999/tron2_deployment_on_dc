import asyncio

from tron2_deployment.rgbd import (
    HIGH_COLOR_TOPIC,
    Tron2HighRgbdCapture,
    Tron2HighRgbdConfig,
    _BridgeFrame,
    _JointFrame,
)


def _config():
    return Tron2HighRgbdConfig(
        color_k=(1, 0, 0, 0, 1, 0, 0, 0, 1),
        color_dist=(0, 0, 0, 0, 0),
        depth_k=(1, 0, 0, 0, 1, 0, 0, 0, 1),
        depth_to_color_r=(1, 0, 0, 0, 1, 0, 0, 0, 1),
        depth_to_color_t_m=(0, 0, 0),
        timeout_s=1,
        max_skew_ms=100,
        max_state_skew_ms=100,
        sample_count=3,
    )


class _DelayedDepthCapture(Tron2HighRgbdCapture):
    async def _frames(self, topic, *, updated=None, keep_streaming=False, sink=None):
        del keep_streaming
        frames = sink if sink is not None else []
        if topic == HIGH_COLOR_TOPIC:
            for timestamp in (1_000, 1_040, 1_080, 1_120):
                frames.append(_BridgeFrame(timestamp, "image/jpeg", b""))
                if len(frames) > self.config.sample_count:
                    del frames[:-self.config.sample_count]
                updated.set()
                await asyncio.sleep(0.01)
        else:
            await asyncio.sleep(0.06)
            frames.append(_BridgeFrame(1_125, "application/x-ros-image", b""))
            updated.set()
        await asyncio.Future()

    async def _joint_frames(self, *, updated=None, keep_streaming=False, sink=None):
        del keep_streaming
        frames = sink if sink is not None else []
        await asyncio.sleep(0.07)
        frames.append(_JointFrame(1_127, tuple(range(16))))
        updated.set()
        await asyncio.Future()


def test_bridge_waits_for_delayed_depth_and_joint_streams():
    capture = _DelayedDepthCapture(_config())
    color, depth, joint, stable_fallback = asyncio.run(
        capture._capture_async(include_joint_state=True))

    assert color.timestamp_ms == 1_120
    assert depth.timestamp_ms == 1_125
    assert joint is not None
    assert joint.timestamp_ms == 1_127
    assert stable_fallback is False
