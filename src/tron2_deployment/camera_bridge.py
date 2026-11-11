"""Read one color image and named joint state through LimX Studio's bridge."""
from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
import inspect
import json
import math
import os
import ssl
import time
from urllib.parse import urlencode, urlsplit

import cv2
import numpy as np

from .rgbd import _decode_brdg


@dataclass(frozen=True)
class _Sample:
    stamp_ms: int
    received: float
    value: object


def _connection(camera, color_topic, joint_topic, domain_id, timeout):
    if camera.get("backend") != "bridge":
        raise ValueError("LimX Studio capture requires a camera.backend=bridge profile")
    host = camera["bridge_host"].rstrip("/")
    if "://" not in host:
        host = "wss://" + host
    parsed = urlsplit(host)
    if (parsed.scheme not in ("ws", "wss") or not parsed.hostname or parsed.username
            or parsed.password or parsed.query or parsed.fragment or parsed.path not in ("", "/")):
        raise ValueError("bridge_host must be a ws/wss host without credentials, query or path")
    path = camera.get("bridge_path", "/bridge/ws")
    if not path.startswith("/") or "?" in path or "#" in path:
        raise ValueError("invalid bridge_path")
    context = None
    if parsed.scheme == "wss":
        context = ssl.create_default_context()
        if not camera.get("verify_tls", True):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
    try:
        import websockets
    except ImportError as error:
        raise RuntimeError("LimX Studio capture requires the tron2-deployment bridge extra") from error
    options = {"ssl": context, "open_timeout": timeout, "close_timeout": .25,
               "max_size": 8 * 1024 * 1024, "max_queue": 4}
    parameters = inspect.signature(websockets.connect).parameters
    if "proxy" in parameters:
        options["proxy"] = None
    token = os.environ.get(camera.get("token_env") or "TRON2_BRIDGE_TOKEN")
    if token:
        key = "additional_headers" if "additional_headers" in parameters else "extra_headers"
        options[key] = {"X-Internal-Token": token}

    def url(topic, kind):
        query = {"topic": topic, "kind": kind, "domain_id": domain_id}
        if kind == "image":
            query["max_fps"] = 30
        return host + path + "?" + urlencode(query)

    return url(color_topic, "image"), url(joint_topic, "joint"), options


def _joint_sample(raw, required_names, now):
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("Bridge message must be a JSON object")
    if value.get("type") == "error":
        raise RuntimeError("Bridge subscription error: " + str(value.get("message") or value.get("error")))
    if value.get("type") != "joint_state":
        return None
    stamp, data = value.get("timestamp"), value.get("data", {})
    if not isinstance(data, dict):
        raise ValueError("Bridge joint data must be a JSON object")
    names, positions = data.get("names"), data.get("positions")
    if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
            or not math.isfinite(stamp) or stamp <= 0 or not isinstance(names, list)
            or not names or not all(isinstance(name, str) and name for name in names)
            or len(names) != len(set(names)) or not isinstance(positions, list)
            or len(names) != len(positions)):
        raise ValueError("Bridge feedback requires an explicit timestamp and matching unique joint names")
    values = np.asarray(positions, dtype=float)
    if values.shape != (len(names),) or not np.isfinite(values).all():
        raise ValueError("Bridge joint positions must be finite")
    joints = dict(zip(names, values.tolist()))
    missing = [name for name in required_names if name not in joints]
    if missing:
        raise ValueError(f"Bridge joint state lacks {missing}")
    return _Sample(int(stamp), now, joints)


def _select_pair(images, states, now, max_age_ms, max_skew_ms):
    for image in reversed(images):
        if not 0 <= (now - image.received) * 1000 <= max_age_ms:
            continue
        eligible = [state for state in states
                    if 0 <= (now - state.received) * 1000 <= max_age_ms
                    and abs(image.stamp_ms - state.stamp_ms) <= max_skew_ms]
        if eligible:
            return image, min(eligible, key=lambda state: abs(image.stamp_ms - state.stamp_ms))
    return None


async def _read(url, sink, options, required_names, image_size=None):
    import websockets
    async with websockets.connect(url, **options) as socket:
        async for raw in socket:
            now, sample = time.monotonic(), None
            if isinstance(raw, str):
                joint = _joint_sample(raw, required_names, now)
                if image_size is None:
                    sample = joint
            elif image_size is not None:
                frame = _decode_brdg(raw)
                if frame is not None:
                    if frame.timestamp_ms <= 0 or frame.mime not in ("image/jpeg", "image/png"):
                        raise ValueError("Bridge RGB lacks a valid timestamp or compressed color payload")
                    image = cv2.imdecode(np.frombuffer(frame.payload, np.uint8), cv2.IMREAD_COLOR)
                    if image is None or (image.shape[1], image.shape[0]) != image_size:
                        raise ValueError(f"Bridge RGB must have size {image_size[0]}x{image_size[1]}")
                    sample = _Sample(frame.timestamp_ms, now, image)
            if sample is not None and (not sink or sample.stamp_ms > sink[-1].stamp_ms):
                sink.append(sample)
        raise RuntimeError("Bridge subscription closed before a synchronized pair was captured")


async def _capture(urls, options, required_names, image_size, timeout, max_age_ms, max_skew_ms):
    images, states = deque(maxlen=30), deque(maxlen=200)
    tasks = [asyncio.create_task(_read(urls[0], images, options, required_names, image_size)),
             asyncio.create_task(_read(urls[1], states, options, required_names))]
    deadline = time.monotonic() + timeout
    try:
        while True:
            for task in tasks:
                if task.done():
                    task.result()
            now = time.monotonic()
            pair = _select_pair(images, states, now, max_age_ms, max_skew_ms)
            if pair is not None:
                return pair, now
            if now >= deadline:
                raise TimeoutError("No fresh synchronized wrist RGB and named joint state through LimX Studio "
                                   f"(RGB={len(images)}, joints={len(states)})")
            await asyncio.sleep(.02)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


def capture_pair(camera, *, color_topic, joint_topic, domain_id, required_names,
                 camera_frame, timeout_s=8, max_state_skew_ms=100):
    """Explicitly subscribe, return one fresh pair, and close both subscriptions."""
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 60:
        raise ValueError("capture.timeout_s must be in (0, 60]")
    if not math.isfinite(max_state_skew_ms) or max_state_skew_ms < 0:
        raise ValueError("capture.max_state_skew_ms must be finite and nonnegative")
    image_url, joint_url, options = _connection(camera, color_topic, joint_topic, domain_id, timeout_s)
    (image, state), now = asyncio.run(_capture(
        (image_url, joint_url), options, required_names, (640, 480), timeout_s, 750, max_state_skew_ms))
    return image.value, {
        "source": "limx_studio_bridge", "timestamp_source": "bridge_forwarding_ms",
        "sensor_header_timestamps_available": False,
        "frame_id": camera_frame, "frame_id_source": "configured_camera_topic",
        "color_topic": color_topic, "joint_topic": joint_topic,
        "image_stamp_ns": image.stamp_ms * 1_000_000,
        "joint_stamp_ns": state.stamp_ms * 1_000_000,
        "image_receipt_age_ms": (now - image.received) * 1000,
        "joint_receipt_age_ms": (now - state.received) * 1000,
        "joints": state.value,
    }
