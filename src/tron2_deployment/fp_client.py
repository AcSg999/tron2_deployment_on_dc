"""Control-host client and rigid single-shot re-anchoring utilities."""
from __future__ import annotations

import argparse
import json
import time
import uuid
from pathlib import Path

import cv2
import numpy as np
import zmq

from tron2_deployment.fp_protocol import (
    PROTOCOL_VERSION, PoseEstimate, decode_response, encode_request,
)
from tron2_deployment.rotations import quat_to_rotmat, rot_to_quat


def pose7_to_matrix(pose7: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose7, dtype=np.float64)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError("pose7 must be a finite shape-(7,) vector")
    transform = np.eye(4)
    transform[:3, :3] = quat_to_rotmat(pose[3:])
    transform[:3, 3] = pose[:3]
    return transform


def matrix_to_pose7(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("pose transform must be a finite 4x4 matrix")
    return np.concatenate([transform[:3, 3], rot_to_quat(transform[:3, :3])])


def transform_pose7(frame_transform: np.ndarray, pose7: np.ndarray) -> np.ndarray:
    return matrix_to_pose7(np.asarray(frame_transform) @ pose7_to_matrix(pose7))


def alignment_from_object_poses(obj_ref0: np.ndarray,
                                obj_measured0: np.ndarray) -> np.ndarray:
    """Return delta that maps reference-scene poses into the measured scene."""
    return pose7_to_matrix(obj_measured0) @ np.linalg.inv(pose7_to_matrix(obj_ref0))


def apply_alignment(poses: np.ndarray, alignment: np.ndarray) -> np.ndarray:
    poses = np.asarray(poses, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1] != 7:
        raise ValueError("pose trajectory must have shape (T, 7)")
    return np.asarray([transform_pose7(alignment, pose) for pose in poses])


def reanchor_reference(eef_ref: np.ndarray, obj_ref: np.ndarray,
                       obj_measured0: np.ndarray
                       ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    alignment = alignment_from_object_poses(obj_ref[0], obj_measured0)
    return (apply_alignment(eef_ref, alignment),
            apply_alignment(obj_ref, alignment), alignment)


class FoundationPoseClient:
    def __init__(self, endpoint: str, *, timeout_ms: int = 1000,
                 protocol_version: int | None = None,
                 context: zmq.Context | None = None) -> None:
        if timeout_ms <= 0:
            raise ValueError("timeout_ms must be positive")
        self.endpoint = endpoint
        self.timeout_ms = int(timeout_ms)
        self.protocol_version = (PROTOCOL_VERSION if protocol_version is None
                                 else int(protocol_version))
        self.context = context or zmq.Context.instance()
        self.socket: zmq.Socket | None = None
        self._connect()

    def _connect(self) -> None:
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(self.endpoint)

    def close(self) -> None:
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def estimate(self, rgb: np.ndarray, intrinsics: np.ndarray, mesh_id: str,
                 *, depth: np.ndarray | None = None,
                 mask: np.ndarray | None = None,
                 mode: str = "estimate") -> PoseEstimate:
        request_id = uuid.uuid4().hex
        frames = encode_request(
            rgb, intrinsics, mesh_id, depth=depth, mode=mode,
            mask=mask, request_id=request_id, timestamp_s=time.time(),
            protocol_version=self.protocol_version)
        assert self.socket is not None
        self.socket.send_multipart(frames)
        if not self.socket.poll(self.timeout_ms, zmq.POLLIN):
            self.close()
            self._connect()
            raise TimeoutError(
                f"FoundationPose RPC timed out after {self.timeout_ms} ms")
        estimate = decode_response(self.socket.recv())
        if estimate.request_id != request_id:
            raise RuntimeError("FoundationPose response request_id mismatch")
        return estimate

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def _load_intrinsics(path: Path) -> np.ndarray:
    data = json.loads(path.read_text())
    if isinstance(data, dict) and "intrinsics" in data:
        data = data["intrinsics"]
    return np.asarray(data, dtype=np.float64)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("intrinsics", type=Path)
    parser.add_argument("mesh_id")
    parser.add_argument("--depth", type=Path)
    parser.add_argument("--mask", type=Path)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--mode", choices=("estimate", "track"),
                        default="estimate")
    parser.add_argument("--camera-to-output", type=Path,
                        help="JSON 4x4 transform for camera pose conversion")
    args = parser.parse_args(argv)
    bgr = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if bgr is None:
        parser.error(f"could not read image: {args.image}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    depth = np.load(args.depth) if args.depth else None
    mask = None
    if args.mask:
        mask = cv2.imread(str(args.mask), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            parser.error(f"could not read mask: {args.mask}")
    with FoundationPoseClient(args.endpoint, timeout_ms=args.timeout_ms) as client:
        estimate = client.estimate(
            rgb, _load_intrinsics(args.intrinsics), args.mesh_id,
            depth=depth, mask=mask, mode=args.mode)
    pose = estimate.pose7
    if args.camera_to_output:
        transform = np.asarray(
            json.loads(args.camera_to_output.read_text()), dtype=np.float64)
        pose = transform_pose7(transform, pose)
    print(json.dumps({
        "pose7": pose.tolist(), "confidence": estimate.confidence,
        "inference_ms": estimate.inference_ms,
        "timestamp_s": estimate.timestamp_s,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
