"""Multipart RPC protocol for prompt-driven object segmentation."""
from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

PROTOCOL_VERSION = 1


@dataclass(frozen=True)
class MaskEstimate:
    mask: np.ndarray
    score: float
    inference_ms: float
    timestamp_s: float
    request_id: str

    def validate(self) -> None:
        mask = np.asarray(self.mask)
        if mask.ndim != 2 or mask.dtype != np.bool_:
            raise ValueError("mask must be a bool shape-(H, W) array")
        if not np.any(mask):
            raise ValueError("mask must contain at least one foreground pixel")
        if not 0.0 <= self.score <= 1.0:
            raise ValueError("score must be in [0, 1]")


def validate_prompt(prompt: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(prompt, dict):
        raise ValueError("prompt must be an object")
    prompt_type = prompt.get("type")
    if prompt_type == "box":
        box = np.asarray(prompt.get("xyxy"), dtype=np.float64)
        if box.shape != (4,) or not np.isfinite(box).all():
            raise ValueError("box prompt xyxy must contain four finite values")
        if np.any(box < 0) or np.any(box > 1):
            raise ValueError("box prompt coordinates must be normalized to [0, 1]")
        if box[0] >= box[2] or box[1] >= box[3]:
            raise ValueError("box prompt must have positive width and height")
        return {"type": "box", "xyxy": box.tolist()}
    if prompt_type == "points":
        points = np.asarray(prompt.get("points"), dtype=np.float64)
        labels = np.asarray(prompt.get("labels"), dtype=np.int64)
        if points.ndim != 2 or points.shape[1:] != (2,) or not 1 <= len(points) <= 32:
            raise ValueError("point prompt must contain 1 to 32 XY points")
        if labels.shape != (len(points),) or not np.isin(labels, (0, 1)).all():
            raise ValueError("point prompt labels must match points and be 0 or 1")
        if not np.isfinite(points).all() or np.any(points < 0) or np.any(points > 1):
            raise ValueError("point prompt coordinates must be normalized to [0, 1]")
        return {
            "type": "points",
            "points": points.tolist(),
            "labels": labels.tolist(),
        }
    raise ValueError("prompt type must be box or points")


def encode_request(
    rgb: np.ndarray,
    prompt: dict[str, Any],
    *,
    request_id: str | None = None,
    timestamp_s: float | None = None,
) -> list[bytes]:
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("rgb must be uint8 shape-(H, W, 3)")
    prompt = validate_prompt(prompt)
    ok, encoded = cv2.imencode(
        ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError("failed to encode RGB frame")
    metadata = {
        "version": PROTOCOL_VERSION,
        "request_id": request_id or uuid.uuid4().hex,
        "timestamp_s": time.time() if timestamp_s is None else timestamp_s,
        "prompt": prompt,
        "rgb_shape": list(rgb.shape),
    }
    return [json.dumps(metadata).encode("utf-8"), encoded.tobytes()]


def decode_request(frames: list[bytes]) -> dict[str, Any]:
    if len(frames) != 2:
        raise ValueError("mask request must contain metadata and one RGB frame")
    metadata = json.loads(frames[0].decode("utf-8"))
    if metadata.get("version") != PROTOCOL_VERSION:
        raise ValueError("unsupported mask protocol version")
    encoded = np.frombuffer(frames[1], dtype=np.uint8)
    bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("invalid JPEG frame")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return {**metadata, "prompt": validate_prompt(metadata.get("prompt")), "rgb": rgb}


def encode_response(
    *,
    request_id: str,
    mask: np.ndarray | None = None,
    score: float = 0.0,
    inference_ms: float = 0.0,
    error: str | None = None,
) -> list[bytes]:
    metadata = {
        "version": PROTOCOL_VERSION,
        "request_id": request_id,
        "ok": error is None,
        "score": float(score),
        "inference_ms": float(inference_ms),
        "timestamp_s": time.time(),
    }
    if error is not None:
        metadata["error"] = str(error)
        return [json.dumps(metadata).encode("utf-8")]
    mask = np.asarray(mask)
    if mask.ndim != 2:
        raise ValueError("mask must have shape (H, W)")
    mask_u8 = np.where(mask, 255, 0).astype(np.uint8)
    ok, encoded = cv2.imencode(".png", mask_u8)
    if not ok:
        raise RuntimeError("failed to encode mask")
    metadata["mask_shape"] = list(mask.shape)
    return [json.dumps(metadata).encode("utf-8"), encoded.tobytes()]


def decode_response(frames: list[bytes]) -> MaskEstimate:
    if not frames:
        raise ValueError("empty mask response")
    metadata = json.loads(frames[0].decode("utf-8"))
    if metadata.get("version") != PROTOCOL_VERSION:
        raise ValueError("unsupported mask response version")
    if not metadata.get("ok"):
        raise RuntimeError(f"mask server error: {metadata.get('error')}")
    if len(frames) != 2:
        raise ValueError("successful mask response must include a PNG frame")
    encoded = np.frombuffer(frames[1], dtype=np.uint8)
    mask = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
    if mask is None or list(mask.shape) != metadata.get("mask_shape"):
        raise ValueError("decoded mask shape does not match response metadata")
    estimate = MaskEstimate(
        mask=mask > 0,
        score=float(metadata["score"]),
        inference_ms=float(metadata["inference_ms"]),
        timestamp_s=float(metadata["timestamp_s"]),
        request_id=str(metadata["request_id"]),
    )
    estimate.validate()
    return estimate
