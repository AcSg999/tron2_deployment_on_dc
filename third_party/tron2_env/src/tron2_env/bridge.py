"""Bridge WebSocket 观测提供者

通过 ROS bridge WebSocket 订阅图像和关节状态话题，
后台线程运行 asyncio 事件循环，通过线程安全队列向主线程提供对齐全的观测数据。

控制命令仍走 Tron2 WebSocket 直连机器人，仅观测从 bridge 获取。
"""

from __future__ import annotations

import asyncio
import collections
import io
import json
import logging
import queue
import ssl
import struct
import threading
import time
import urllib.parse
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Optional

import numpy as np
from PIL import Image


logger = logging.getLogger(__name__)


# ============================================================================
# 默认话题配置
# ============================================================================

DEFAULT_HOST = "wss://BRIDGE_HOST"
DEFAULT_HTTP_HOST = "https://BRIDGE_HOST"
DEFAULT_WS_PATH = "/bridge/ws"

DEFAULT_IMAGE_TOPICS = {
    "camera_left": "/camera/left/color/image_resized/compressed",
    "camera_right": "/camera/right/color/image_resized/compressed",
    "camera_top": "/camera/top/color/image_raw/compressed",
}
DEFAULT_JOINT_TOPICS = {
    "joint_states": "/joint_states",
    "gripper": "/gripper_state",
}


# ============================================================================
# Bridge 配置
# ============================================================================

@dataclass
class BridgeConfig:
    """Bridge WebSocket 观测配置"""
    host: str = DEFAULT_HOST
    ws_path: str = DEFAULT_WS_PATH
    image_max_fps: int = 0
    align_max_delay_ms: int = 200
    verify_tls: bool = False
    image_topics: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_IMAGE_TOPICS))
    joint_topics: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_JOINT_TOPICS))
    save_debug_images: bool = True
    debug_image_dir: str = "./debug_images"


# ============================================================================
# 数据类
# ============================================================================

@dataclass
class BrdgFrame:
    mime: str
    timestamp_ms: int
    payload: bytes


@dataclass
class ImageFrame:
    key: str
    topic: str
    timestamp_ms: int
    image: np.ndarray


@dataclass
class JointFrame:
    key: str
    topic: str
    timestamp_ms: int
    names: list[str]
    positions: list[float]
    velocities: list[float]
    efforts: list[float]


# ============================================================================
# 协议解析
# ============================================================================

def make_ssl_ctx(insecure: bool = True) -> ssl.SSLContext:
    """构建 SSL 上下文，bridge 通常使用自签名证书"""
    ctx = ssl.create_default_context()
    if insecure:
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
    return ctx


def parse_brdg_frame(data: bytes) -> Optional[BrdgFrame]:
    """解析 BRDG 二进制 WebSocket 消息"""
    if len(data) < 14 or data[:4] != b"BRDG":
        return None
    version = data[4]
    if version != 1:
        return None
    mime_len = data[5]
    mime_start = 6
    mime_end = mime_start + mime_len
    ts_end = mime_end + 8
    if len(data) < ts_end:
        return None
    mime = data[mime_start:mime_end].decode("utf-8", errors="replace")
    timestamp_ms = struct.unpack_from("<Q", data, mime_end)[0]
    return BrdgFrame(mime=mime, timestamp_ms=timestamp_ms, payload=data[ts_end:])


def decode_image(frame: BrdgFrame) -> Optional[np.ndarray]:
    """将 BRDG 帧解码为 RGB uint8 图像数组"""
    if frame.mime in ("image/jpeg", "image/png"):
        image = Image.open(io.BytesIO(frame.payload)).convert("RGB")
        return np.asarray(image, dtype=np.uint8)

    if frame.mime != "application/x-ros-image":
        return None

    payload = frame.payload
    if len(payload) < 18 or payload[:4] != b"RIMG":
        return None

    width = struct.unpack_from("<I", payload, 4)[0]
    height = struct.unpack_from("<I", payload, 8)[0]
    step = struct.unpack_from("<I", payload, 12)[0]
    enc_len = payload[17]
    enc_start = 18
    enc_end = enc_start + enc_len
    if len(payload) < enc_end:
        return None

    encoding = payload[enc_start:enc_end].decode("utf-8", errors="replace")
    raw = payload[enc_end:]
    if step <= 0:
        return None

    if encoding in ("rgb8", "bgr8"):
        expected = step * height
        if len(raw) < expected:
            return None
        rows = np.frombuffer(raw[:expected], dtype=np.uint8).reshape(height, step)
        arr = rows[:, : width * 3].reshape(height, width, 3)
        if encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return arr.copy()

    if encoding == "mono8":
        expected = step * height
        if len(raw) < expected:
            return None
        rows = np.frombuffer(raw[:expected], dtype=np.uint8).reshape(height, step)
        arr = rows[:, :width]
        return np.stack([arr, arr, arr], axis=-1).copy()

    return None


def parse_joint_message(raw: str, key: str, default_topic: str) -> Optional[JointFrame]:
    """解析 bridge JSON 文本帧为 JointFrame，忽略状态消息"""
    try:
        msg = json.loads(raw)
    except json.JSONDecodeError:
        return None

    if msg.get("type") != "joint_state":
        return None

    data = msg.get("data", {})
    return JointFrame(
        key=key,
        topic=msg.get("topic", default_topic),
        timestamp_ms=int(msg.get("timestamp", int(time.time() * 1000))),
        names=list(data.get("names", [])),
        positions=list(data.get("positions", [])),
        velocities=list(data.get("velocities", [])),
        efforts=list(data.get("efforts", [])),
    )


def ws_url(host: str, path: str, topic: str, kind: str, max_fps: Optional[int] = None) -> str:
    """构造 bridge WebSocket 订阅 URL"""
    query_params = {"topic": topic, "kind": kind}
    if max_fps is not None:
        query_params["max_fps"] = str(max_fps)
    return f"{host.rstrip('/')}{path}?{urllib.parse.urlencode(query_params)}"


# ============================================================================
# 时间对齐器
# ============================================================================

class TopicAligner:
    """ApproximateTime 对齐器：以最早图像时间戳为参考，匹配最接近的关节数据"""

    def __init__(
        self,
        image_keys: Iterable[str],
        joint_keys: Iterable[str],
        max_delay_ms: int = 200,
        joint_buffer_size: int = 100,
    ):
        self.image_keys = list(image_keys)
        self.joint_keys = list(joint_keys)
        self.max_delay_ms = int(max_delay_ms)
        self._latest_image: Dict[str, ImageFrame] = {}
        self._joint_buffer: Dict[str, collections.deque[JointFrame]] = {
            key: collections.deque(maxlen=joint_buffer_size) for key in self.joint_keys
        }
        self._last_emitted_ref_ts: Optional[int] = None
        self._last_emit_wall_s: float = time.time()
        self._last_stall_log_s: float = 0.0
        self._stall_warn_ms: float = 500.0

    def _log_stall(self, reason: str) -> None:
        """Throttle diagnostics when the aligner has not emitted recently."""
        now = time.time()
        stalled_ms = (now - self._last_emit_wall_s) * 1000.0
        if stalled_ms < self._stall_warn_ms:
            return
        if now - self._last_stall_log_s >= 1.0:
            self._last_stall_log_s = now
            logger.warning("[bridge:align] no aligned observation for %.0fms: %s", stalled_ms, reason)

    def push_image(self, frame: ImageFrame):
        self._latest_image[frame.key] = frame

    def push_joint(self, frame: JointFrame):
        if frame.key in self._joint_buffer:
            self._joint_buffer[frame.key].append(frame)

    def try_align(self) -> Optional[Dict[str, Any]]:
        missing = [key for key in self.image_keys if key not in self._latest_image]
        if missing:
            self._log_stall("waiting for first image frame: " + ", ".join(missing))
            return None

        img_ts = {key: self._latest_image[key].timestamp_ms for key in self.image_keys}
        ref_ts = min(img_ts.values())
        if self._last_emitted_ref_ts == ref_ts:
            newest = max(img_ts.values())
            lagging = ", ".join(
                f"{key} lags {newest - ts}ms" for key, ts in img_ts.items() if newest - ts > 0
            ) or "image timestamps are equal"
            self._log_stall(f"image reference timestamp has not advanced: ref_ts={ref_ts}ms; {lagging}")
            return None

        matched: Dict[str, JointFrame] = {}
        for key in self.joint_keys:
            buffer = self._joint_buffer[key]
            if not buffer:
                self._log_stall(f"joint topic {key} has no buffered frames")
                return None
            best = min(buffer, key=lambda f: abs(f.timestamp_ms - ref_ts))
            dt = abs(best.timestamp_ms - ref_ts)
            if dt > self.max_delay_ms:
                self._log_stall(
                    f"joint topic {key} is {dt}ms from image ref {ref_ts}ms "
                    f"(limit {self.max_delay_ms}ms, buffer={len(buffer)}, best={best.timestamp_ms}ms)"
                )
                return None
            matched[key] = best

        # 清理过期的关节数据
        for key, buffer in self._joint_buffer.items():
            while len(buffer) > 1 and buffer[0].timestamp_ms < ref_ts - self.max_delay_ms:
                buffer.popleft()

        self._last_emitted_ref_ts = ref_ts
        self._last_emit_wall_s = time.time()
        return {
            "images": {key: self._latest_image[key].image for key in self.image_keys},
            "image_frames": dict(self._latest_image),
            "joint_states": matched,
            "timestamp_ms": ref_ts,
        }


# ============================================================================
# 观测格式转换
# ============================================================================

def build_openpi_observation(obs: Dict[str, Any]) -> Dict[str, Any]:
    """将 bridge 对齐全的观测转换为 TRON2/OpenPI 惯例格式

    TRON2 惯例: [左臂7, 左夹爪1, 右臂7, 右夹爪1, 头部2] = 18维
    Bridge /joint_states: [左臂7, 右臂7, 头部2] = 16维
    Bridge /gripper_state: [左, 右] = 0-100
    """
    images = obs["images"]
    joints = obs["joint_states"].get("joint_states")
    gripper = obs["joint_states"].get("gripper")

    openpi_images = {
        "cam_high": images.get("camera_top"),
        "cam_left_wrist": images.get("camera_left"),
        "cam_right_wrist": images.get("camera_right"),
    }
    openpi_images = {k: v for k, v in openpi_images.items() if v is not None}

    image_frame_map = {
        "cam_high": obs.get("image_frames", {}).get("camera_top"),
        "cam_left_wrist": obs.get("image_frames", {}).get("camera_left"),
        "cam_right_wrist": obs.get("image_frames", {}).get("camera_right"),
    }
    image_timestamps_ms = {
        key: frame.timestamp_ms
        for key, frame in image_frame_map.items()
        if frame is not None
    }

    joint_pos = np.asarray(joints.positions if joints else [], dtype=np.float32)
    gripper_pos = np.asarray(gripper.positions if gripper else [], dtype=np.float32) / 100.0

    if joint_pos.shape[0] >= 16 and gripper_pos.shape[0] >= 2:
        state = np.concatenate([
            joint_pos[:7],       # 左臂
            gripper_pos[:1],     # 左夹爪
            joint_pos[7:14],     # 右臂
            gripper_pos[1:2],    # 右夹爪
            joint_pos[14:16],    # 头部
        ])
    else:
        state = np.concatenate([joint_pos, gripper_pos])

    bridge_ref_timestamp_ms = obs.get("timestamp_ms")
    image_timestamp_ms = (
        min(image_timestamps_ms.values()) if image_timestamps_ms else bridge_ref_timestamp_ms
    )
    metadata = {
        "observation_source": "bridge",
        "bridge_ref_timestamp_ms": bridge_ref_timestamp_ms, # 本次 observation 以哪个图像时间为对齐基准 
        "joint_timestamp_ms": joints.timestamp_ms if joints else None, # 被对齐的关节状态时间
        "gripper_timestamp_ms": gripper.timestamp_ms if gripper else None,
        "image_timestamp_ms": image_timestamp_ms, #当前图像组的最早图像时间，基本等于 ref
        "image_timestamps_ms": image_timestamps_ms, #每个相机各自是什么时间的图像
    }

    return {
        "images": openpi_images,
        "state": state.astype(np.float32, copy=False),
        "metadata": metadata,
    }


# ============================================================================
# Bridge 观测提供者
# ============================================================================

class BridgeObservationProvider:
    """后台线程从 bridge WebSocket 获取对齐全的观测数据

    控制命令不经过此类，仍由 Tron2 WebSocket 直连机器人。
    """

    def __init__(self, config: BridgeConfig):
        self.config = config
        self._obs_queue: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=10)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        """启动后台订阅线程"""
        if self._running:
            return
        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("Bridge 观测提供者已启动 (host=%s, fps=%d)",
                     self.config.host, self.config.image_max_fps)

    def stop(self):
        """停止后台订阅线程"""
        if not self._running:
            return
        self._running = False
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        logger.info("Bridge 观测提供者已停止")

    def get_obs(self, timeout: float = 1.0) -> Dict[str, Any]:
        """阻塞获取对齐全的观测数据

        Args:
            timeout: 超时秒数

        Returns:
            最新观测字典 {"images": {...}, "state": np.ndarray(18,)}

        Raises:
            TimeoutError: 超时未获取到观测
        """
        try:
            obs = self._obs_queue.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(f"Bridge 观测获取超时 ({timeout}s),请检查相机是否连接算力模块")

        # Producer may run much slower than the bridge subscription rate. Drain
        # queued observations so policy inference always uses the freshest frame.
        while True:
            try:
                obs = self._obs_queue.get_nowait()
            except queue.Empty:
                return obs

    # ------------------------------------------------------------------
    # 后台线程
    # ------------------------------------------------------------------

    def _run_loop(self):
        """后台线程入口：运行 asyncio 事件循环"""
        try:
            asyncio.run(self._async_main())
        except Exception as e:
            logger.error("Bridge 事件循环异常退出: %s", e)

    async def _async_main(self):
        """异步主逻辑：订阅所有话题，等待停止信号"""
        cfg = self.config
        ssl_ctx = make_ssl_ctx(insecure=not cfg.verify_tls)
        aligner = TopicAligner(
            image_keys=cfg.image_topics.keys(),
            joint_keys=cfg.joint_topics.keys(),
            max_delay_ms=cfg.align_max_delay_ms,
        )

        tasks = []
        for key, topic in cfg.image_topics.items():
            tasks.append(asyncio.create_task(
                self._subscribe_image(key, topic, aligner, ssl_ctx)
            ))
        for key, topic in cfg.joint_topics.items():
            tasks.append(asyncio.create_task(
                self._subscribe_joint(key, topic, aligner, ssl_ctx)
            ))
        # 对齐分发任务
        tasks.append(asyncio.create_task(
            self._align_dispatcher(aligner)
        ))

        try:
            # threading.Event cannot be awaited. Poll it and surface task failures
            # so a dead subscription does not look like a silent alignment stall.
            while not self._stop_event.is_set():
                for task in tasks:
                    if task.done():
                        exc = task.exception()
                        if exc is not None:
                            logger.error("[bridge] background task %s exited: %r", task.get_name(), exc)
                await asyncio.sleep(0.1)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _subscribe_image(
        self, key: str, topic: str, aligner: TopicAligner, ssl_ctx: ssl.SSLContext
    ):
        """订阅图像话题"""
        import websockets

        cfg = self.config
        url = ws_url(cfg.host, cfg.ws_path, topic, "image", cfg.image_max_fps)
        while not self._stop_event.is_set():
            try:
                async with websockets.connect(url, ssl=ssl_ctx) as socket:
                    async for msg in socket:
                        if self._stop_event.is_set():
                            return
                        if isinstance(msg, str):
                            continue
                        if not isinstance(msg, bytes):
                            continue

                        frame = parse_brdg_frame(msg)
                        if frame is None:
                            continue
                        image = decode_image(frame)
                        if image is None:
                            continue

                        aligner.push_image(ImageFrame(key, topic, frame.timestamp_ms, image))
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning("[bridge:image] %s 断线: %s, 3秒后重连", key, exc)
                await asyncio.sleep(3)

    async def _subscribe_joint(
        self, key: str, topic: str, aligner: TopicAligner, ssl_ctx: ssl.SSLContext
    ):
        """订阅关节状态话题"""
        import websockets

        cfg = self.config
        url = ws_url(cfg.host, cfg.ws_path, topic, "joint")
        while not self._stop_event.is_set():
            try:
                async with websockets.connect(url, ssl=ssl_ctx) as socket:
                    async for msg in socket:
                        if self._stop_event.is_set():
                            return
                        if not isinstance(msg, str):
                            continue
                        frame = parse_joint_message(msg, key, topic)
                        if frame is None:
                            continue
                        aligner.push_joint(frame)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning("[bridge:joint] %s 断线: %s, 3秒后重连", key, exc)
                await asyncio.sleep(3)

    async def _align_dispatcher(self, aligner: TopicAligner):
        """定期轮询对齐器，将结果放入线程安全队列"""
        while not self._stop_event.is_set():
            obs = aligner.try_align()
            if obs is not None:
                openpi_obs = build_openpi_observation(obs)
                try:
                    self._obs_queue.put_nowait(openpi_obs)
                except queue.Full:
                    # 队列满时丢弃最旧的观测
                    try:
                        self._obs_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self._obs_queue.put_nowait(openpi_obs)
                    except queue.Full:
                        pass
            await asyncio.sleep(0.001)  # 1ms 轮询间隔

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


# ============================================================================
# 测试辅助函数
# ============================================================================

def make_brdg(mime: str, timestamp_ms: int, payload: bytes) -> bytes:
    """构造 BRDG 二进制帧（测试用）"""
    mime_bytes = mime.encode("utf-8")
    return b"BRDG" + bytes([1, len(mime_bytes)]) + mime_bytes + struct.pack("<Q", timestamp_ms) + payload


def make_rimg(width: int, height: int, encoding: str, raw: bytes, step: Optional[int] = None) -> bytes:
    """构造 RIMG 原始图像帧（测试用）"""
    enc = encoding.encode("utf-8")
    step = step if step is not None else width * (3 if encoding in ("rgb8", "bgr8") else 1)
    header = b"RIMG" + struct.pack("<III", width, height, step) + bytes([0, len(enc)]) + enc
    return header + raw
