"""Versioned multipart protocol for remote FoundationPose inference."""
from __future__ import annotations

import io
import json
import time
import uuid
from dataclasses import dataclass

import cv2
import numpy as np

PROTOCOL_VERSION = 2
SUPPORTED_PROTOCOL_VERSIONS = (1, 2)


@dataclass(frozen=True)
class PoseEstimate:
    pose7: np.ndarray
    confidence: float
    inference_ms: float
    timestamp_s: float
    request_id: str

    def validate(self) -> None:
        pose = np.asarray(self.pose7, dtype=np.float64)
        if pose.shape != (7,) or not np.isfinite(pose).all():
            raise ValueError("pose7 must be a finite shape-(7,) vector")
        norm = np.linalg.norm(pose[3:])
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("pose7 quaternion must be normalized")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")


def encode_request(
    rgb: np.ndarray,
    intrinsics: np.ndarray,
    mesh_id: str,
    *,
    depth: np.ndarray | None = None,
    mask: np.ndarray | None = None,
    mode: str = "estimate",
    request_id: str | None = None,
    timestamp_s: float | None = None,
    protocol_version: int = PROTOCOL_VERSION,
) -> list[bytes]:
    rgb = np.asarray(rgb)
    intrinsics = np.asarray(intrinsics, dtype=np.float64)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("rgb must be uint8 shape-(H, W, 3)")
    if intrinsics.shape != (3, 3) or not np.isfinite(intrinsics).all():
        raise ValueError("intrinsics must be a finite 3x3 matrix")
    if not mesh_id or mode not in ("estimate", "track"):
        raise ValueError("mesh_id and mode are invalid")
    if protocol_version not in SUPPORTED_PROTOCOL_VERSIONS:
        raise ValueError("unsupported FoundationPose protocol version")
    if protocol_version == 1 and mask is not None:
        raise ValueError("FoundationPose protocol v1 does not support masks")
    ok, encoded_rgb = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError("failed to encode RGB frame")
    metadata = {
        "version": protocol_version,
        "request_id": request_id or uuid.uuid4().hex,
        "timestamp_s": time.time() if timestamp_s is None else timestamp_s,
        "mesh_id": mesh_id,
        "mode": mode,
        "intrinsics": intrinsics.tolist(),
        "rgb_shape": list(rgb.shape),
        "has_depth": depth is not None,
    }
    if protocol_version >= 2:
        metadata["has_mask"] = mask is not None
    frames = [json.dumps(metadata).encode("utf-8"), encoded_rgb.tobytes()]
    if depth is not None:
        depth = np.asarray(depth)
        if depth.shape != rgb.shape[:2] or not np.isfinite(depth).all():
            raise ValueError("depth must be finite and match RGB height/width")
        buffer = io.BytesIO()
        np.save(buffer, depth.astype(np.float32), allow_pickle=False)
        frames.append(buffer.getvalue())
    if mask is not None:
        mask = np.asarray(mask)
        if mask.shape != rgb.shape[:2]:
            raise ValueError("mask must match RGB height/width")
        mask_u8 = np.where(mask > 0, 255, 0).astype(np.uint8)
        ok, encoded_mask = cv2.imencode(".png", mask_u8)
        if not ok:
            raise RuntimeError("failed to encode object mask")
        frames.append(encoded_mask.tobytes())
    return frames


def decode_request(frames: list[bytes]) -> dict:
    if len(frames) not in (2, 3, 4):
        raise ValueError("FoundationPose request must contain 2 to 4 frames")
    metadata = json.loads(frames[0].decode("utf-8"))
    version = metadata.get("version")
    if version not in SUPPORTED_PROTOCOL_VERSIONS:
        raise ValueError("unsupported FoundationPose protocol version")
    has_depth = bool(metadata.get("has_depth"))
    has_mask = bool(metadata.get("has_mask", False))
    if version == 1 and has_mask:
        raise ValueError("FoundationPose protocol v1 does not support masks")
    expected_frames = 2 + int(has_depth) + int(has_mask)
    if len(frames) != expected_frames:
        raise ValueError("request metadata does not match multipart frames")
    encoded = np.frombuffer(frames[1], dtype=np.uint8)
    bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("invalid JPEG frame")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    depth = None
    frame_index = 2
    if has_depth:
        depth = np.load(io.BytesIO(frames[frame_index]), allow_pickle=False)
        frame_index += 1
        if depth.shape != rgb.shape[:2]:
            raise ValueError("decoded depth shape does not match RGB")
        if not np.isfinite(depth).all():
            raise ValueError("decoded depth must be finite")
    mask = None
    if has_mask:
        encoded_mask = np.frombuffer(frames[frame_index], dtype=np.uint8)
        mask = cv2.imdecode(encoded_mask, cv2.IMREAD_GRAYSCALE)
        if mask is None or mask.shape != rgb.shape[:2]:
            raise ValueError("decoded mask shape does not match RGB")
        mask = mask > 0
    intrinsics = np.asarray(metadata["intrinsics"], dtype=np.float64)
    if intrinsics.shape != (3, 3):
        raise ValueError("decoded intrinsics must have shape (3, 3)")
    return {**metadata, "rgb": rgb, "depth": depth, "mask": mask,
            "intrinsics": intrinsics}


def encode_response(*, request_id: str, pose7: np.ndarray | None = None,
                    confidence: float = 0.0, inference_ms: float = 0.0,
                    error: str | None = None,
                    protocol_version: int = PROTOCOL_VERSION) -> bytes:
    if protocol_version not in SUPPORTED_PROTOCOL_VERSIONS:
        raise ValueError("unsupported FoundationPose protocol version")
    data = {
        "version": protocol_version,
        "request_id": request_id,
        "ok": error is None,
        "confidence": confidence,
        "inference_ms": inference_ms,
        "timestamp_s": time.time(),
    }
    if error is None:
        pose = np.asarray(pose7, dtype=np.float64)
        estimate = PoseEstimate(
            pose7=pose, confidence=float(confidence),
            inference_ms=float(inference_ms), timestamp_s=data["timestamp_s"],
            request_id=request_id,
        )
        estimate.validate()
        data["pose7"] = pose.tolist()
    else:
        data["error"] = str(error)
    return json.dumps(data).encode("utf-8")


def decode_response(payload: bytes) -> PoseEstimate:
    data = json.loads(payload.decode("utf-8"))
    if data.get("version") not in SUPPORTED_PROTOCOL_VERSIONS:
        raise ValueError("unsupported FoundationPose response version")
    if not data.get("ok"):
        raise RuntimeError(f"FoundationPose server error: {data.get('error')}")
    estimate = PoseEstimate(
        pose7=np.asarray(data["pose7"], dtype=np.float64),
        confidence=float(data["confidence"]),
        inference_ms=float(data["inference_ms"]),
        timestamp_s=float(data["timestamp_s"]),
        request_id=str(data["request_id"]),
    )
    estimate.validate()
    return estimate
