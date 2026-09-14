"""FoundationPose/SAM clients with frame-bound provenance and explicit mock mode."""
import time
import uuid

import cv2
import numpy as np

from .camera import decode_image, encode_image
from .config import profile_fingerprint
from .fp_client import FoundationPoseClient, transform_pose7
from .mask_client import MaskClient
from .mask_protocol import validate_prompt


def segment(profile, frame, prompt, mock=False):
    if (frame["source"] == "mock") != bool(mock):
        raise ValueError("frame source does not match vision mode")
    prompt = dict(prompt)
    if prompt.pop("coordinates", None) == "pixels":
        if prompt.get("type") != "box":
            raise ValueError("pixel coordinates require a bounding box")
        prompt["xyxy"] = (np.asarray(prompt["xyxy"], dtype=float) /
                          [frame["width"], frame["height"], frame["width"], frame["height"]]).tolist()
    prompt = validate_prompt(prompt)
    image = cv2.cvtColor(decode_image(frame["image"], cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    if mock:
        if prompt["type"] != "box":
            raise ValueError("mock segmentation supports bounding boxes")
        x1, y1, x2, y2 = np.array(prompt["xyxy"]) * [frame["width"], frame["height"], frame["width"], frame["height"]]
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        mask[int(y1):int(np.ceil(y2)), int(x1):int(np.ceil(x2))] = 255
        score = 1.0
    else:
        with MaskClient(profile["vision"]["mask_endpoint"],
                        timeout_ms=profile["vision"].get("timeout_ms", 120000)) as client:
            result = client.segment(image, prompt)
        mask = result.mask.astype(np.uint8)*255
        score = result.score
    if mask.shape != image.shape[:2] or not np.any(mask):
        raise ValueError("mask is empty or does not match the captured frame")
    return {"mask": encode_image(mask), "score": score, "area_px": int(np.count_nonzero(mask)),
            "frame_ref": frame["frame_ref"], "mask_ref": uuid.uuid4().hex,
            "width": frame["width"], "height": frame["height"]}


def estimate(profile, frame, mask, mesh_id, mock=False):
    if not isinstance(mesh_id, str) or not mesh_id.strip():
        raise ValueError("mesh_id must identify the registered object mesh")
    if mask["frame_ref"] != frame["frame_ref"]:
        raise ValueError("mask belongs to a different capture")
    if (frame["source"] == "mock") != bool(mock):
        raise ValueError("frame source does not match estimator mode")
    if mesh_id != profile["vision"]["mesh_id"]:
        raise ValueError("mesh_id must match the object geometry in the deployment profile")
    if mock:
        pose = np.asarray(profile["demo"]["object_pose7_base"])
        confidence = 1.0
    else:
        rgb = cv2.cvtColor(decode_image(frame["image"], cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        depth = decode_image(frame["depth"]).astype(np.float32)*frame["depth_scale"]
        object_mask = decode_image(mask["mask"], cv2.IMREAD_GRAYSCALE) > 0
        if depth.shape != object_mask.shape or not np.any(depth[object_mask] > 0):
            raise ValueError("object mask lacks valid metric depth")
        with FoundationPoseClient(profile["vision"]["pose_endpoint"],
                                  timeout_ms=profile["vision"].get("timeout_ms", 120000)) as client:
            result = client.estimate(rgb, frame["intrinsics"], mesh_id,
                                     depth=depth, mask=object_mask, mode="estimate")
        pose = transform_pose7(frame["camera_to_base"], result.pose7)
        confidence = result.confidence
    threshold = float(profile["vision"].get("min_confidence", 0.5))
    if not np.isfinite(threshold) or not 0 <= threshold <= 1:
        raise ValueError("vision.min_confidence must be within [0, 1]")
    if not np.isfinite(confidence) or not threshold <= confidence <= 1:
        raise ValueError("object pose confidence does not meet the configured threshold")
    return {"pose7": pose.tolist(), "reference_frame": "base_Link",
            "confidence": float(confidence), "timestamp_s": time.time(),
            "capture_timestamp_s": frame["capture_timestamp_s"],
            "observation_id": uuid.uuid4().hex, "frame_ref": frame["frame_ref"],
            "mask_ref": mask["mask_ref"], "mesh_id": mesh_id, "source": frame["source"],
            "head_q2": frame["head_q2"], "camera_id": frame["camera_id"],
            "calibration_id": frame["calibration_id"], "profile_hash": profile_fingerprint(profile)}
