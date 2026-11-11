#!/usr/bin/env python3
"""Read-only wrist camera position and optical axes on raw head RGB."""
from __future__ import annotations

import argparse
import asyncio
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import ssl
import sys
import time
from urllib.parse import parse_qsl, urlencode, urlsplit

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "sp_vision"))
import calibration as common
import calibration_wrist as wrist
from tron2_deployment.config import finite, rigid
from tron2_deployment.rgbd import _decode_brdg

HEAD_TOPIC = "/camera/top/color/image_raw/compressed"
DEFAULT_CALIBRATION_SUMMARY = ROOT / "configs/local-camera-calibration-latest.json"
WINDOW = "Wrist camera position and axes in head RGB (read-only)"
AXIS_COLORS = {"X": (0, 0, 255), "Y": (0, 255, 0), "Z": (255, 0, 0)}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fk_chain_signature(model, base_link, mount_link):
    """Pin only FK joints used before the calibrated camera mounting transform."""
    chain, seen = [], set()
    current = mount_link
    while current != base_link:
        if current in seen or current not in model.by_child:
            raise ValueError(f"invalid URDF chain from {base_link} to {mount_link}")
        seen.add(current)
        joint = model.by_child[current]
        chain.append({key: joint[key] for key in ("name", "type", "parent", "child")})
        chain[-1].update(origin=joint["origin"].tolist(), axis=joint["axis"].tolist())
        current = joint["parent"]
    encoded = json.dumps(list(reversed(chain)), sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(encoded.encode()).hexdigest()


def project_center(base_wrist_camera, base_head_camera, k, distortion, size):
    """Transform the optical origin; preserve off-screen coordinates, reject Z<=0."""
    point_base = base_wrist_camera[:, 3]
    point_head = common.invert(base_head_camera) @ point_base
    result = {"wrist_position_base_m": point_base[:3].tolist(),
              "wrist_position_head_m": point_head[:3].tolist(),
              "pixel_uv": None, "visible": False, "status": "behind_camera"}
    if point_head[2] <= 1e-9:
        return result
    pixels = cv2.projectPoints(point_head[:3].reshape(1, 3), np.zeros(3),
                               np.zeros(3), k, distortion)[0].reshape(2)
    if not np.isfinite(pixels).all():
        result["status"] = "invalid_projection"
        return result
    width, height = size
    result.update(pixel_uv=pixels.tolist(),
                  visible=bool(0 <= pixels[0] < width and 0 <= pixels[1] < height))
    result["status"] = "projected" if result["visible"] else "outside_image"
    return result


def project_axes(base_wrist_camera, base_head_camera, k, distortion, size, length_m):
    """Project positive optical axes using the full relative camera rotation."""
    if not math.isfinite(length_m) or length_m <= 0:
        raise ValueError("axis length must be positive and finite")
    relative = common.invert(base_head_camera) @ base_wrist_camera
    origin = project_center(base_wrist_camera, base_head_camera, k, distortion, size)
    endpoints = np.eye(3) * length_m
    points_head = endpoints @ relative[:3, :3].T + relative[:3, 3]
    axes = {}
    width, height = size
    for index, name in enumerate(AXIS_COLORS):
        point = points_head[index]
        axis = {"endpoint_wrist_m": endpoints[index].tolist(),
                "endpoint_head_m": point.tolist(), "pixel_uv": None,
                "endpoint_in_image": False, "drawable": False,
                "clipped": False, "segment_uv": None, "status": "behind_camera"}
        if point[2] > 1e-9:
            pixel = cv2.projectPoints(point.reshape(1, 3), np.zeros(3), np.zeros(3),
                                      k, distortion)[0].reshape(2)
            if np.isfinite(pixel).all():
                inside = bool(0 <= pixel[0] < width and 0 <= pixel[1] < height)
                axis.update(pixel_uv=pixel.tolist(), endpoint_in_image=inside,
                            status="endpoint_projected" if inside else "endpoint_outside_image")
                if origin["visible"]:
                    if np.max(np.abs(pixel)) > 1e8:
                        axis["status"] = "projection_too_large"
                    else:
                        start = tuple(map(int, np.rint(origin["pixel_uv"])))
                        end = tuple(map(int, np.rint(pixel)))
                        visible, p0, p1 = cv2.clipLine((0, 0, width, height), start, end)
                        axis.update(drawable=bool(visible), clipped=not inside,
                                    segment_uv=[list(p0), list(p1)] if visible else None)
            else:
                axis["status"] = "invalid_projection"
        if not origin["visible"]:
            axis["status"] = "origin_not_visible"
        axes[name] = axis
    return {"axis_length_m": float(length_m), "axes": axes,
            "T_head_wrist_camera": relative.tolist(),
            "R_head_wrist_camera": relative[:3, :3].tolist(),
            "orientation_head_quaternion_xyzw": Rotation.from_matrix(relative[:3, :3]).as_quat().tolist()}


class Calibration:
    """Reuse the standalone calibration URDF reader and ROS-to-URDF mapping."""

    def __init__(self, summary_path, wrist_config, axis_length_m=.08):
        if not math.isfinite(axis_length_m) or axis_length_m <= 0:
            raise ValueError("axis length must be positive and finite")
        self.axis_length_m = axis_length_m
        self.summary_path = Path(summary_path).resolve()
        if not self.summary_path.is_file():
            raise ValueError(f"calibration manifest missing: {self.summary_path}; provide --calibration-summary PATH")
        summary = common.read_json(self.summary_path)
        if summary.get("schema_version") != 1:
            raise ValueError("unsupported calibration summary schema")
        self.calibration_id = summary.get("calibration_id", self.summary_path.stem)
        if not isinstance(self.calibration_id, str) or not self.calibration_id:
            raise ValueError("calibration_id must be a nonempty string")
        self.config = wrist.load_config(Path(wrist_config))
        sources, self.provenance = {}, {}
        for camera in ("head", "right_wrist"):
            record = summary["cameras"][camera]
            for kind in ("intrinsics", "extrinsics"):
                path = Path(record["source_files"][kind])
                if not path.is_absolute():
                    path = self.summary_path.parent / path
                digest = sha256(path)
                if digest != record["source_sha256"][kind]:
                    raise ValueError(f"source hash changed; refresh calibration summary: {path}")
                value = common.read_json(path)
                if value.get("passed") is not True:
                    raise ValueError(f"calibration result has not passed: {path}")
                sources[camera, kind] = value
                self.provenance[f"{camera}_{kind}"] = {"path": str(path.resolve()),
                                                        "sha256": digest}
        for camera in ("head", "right_wrist"):
            expected = (sources[camera, "extrinsics"].get("intrinsics_sha256") or
                        summary["cameras"][camera].get("intrinsics_sha256_for_extrinsics"))
            if expected and expected != self.provenance[f"{camera}_intrinsics"]["sha256"]:
                raise ValueError(f"{camera} extrinsics were solved with different intrinsics")
        head = sources["head", "intrinsics"]
        self.k = finite(head["camera_matrix"], (3, 3), "head K")
        if (self.k[0, 0] <= 0 or self.k[1, 1] <= 0 or
                not np.allclose(self.k[2], [0, 0, 1])):
            raise ValueError("invalid head intrinsics")
        self.distortion = np.asarray(head["distortion"], float)
        if self.distortion.shape not in ((4,), (5,), (8,), (12,), (14,)) or not np.isfinite(self.distortion).all():
            raise ValueError("invalid head distortion coefficients")
        self.size = tuple(head["image_size"])
        if len(self.size) != 2 or any(type(v) is not int or v <= 0 for v in self.size):
            raise ValueError("invalid calibrated image dimensions")
        wrist_intrinsic = sources["right_wrist", "intrinsics"]
        self.wrist_k = finite(wrist_intrinsic["camera_matrix"], (3, 3), "wrist K")
        self.wrist_distortion = np.asarray(wrist_intrinsic["distortion"], float)
        self.wrist_size = tuple(wrist_intrinsic["image_size"])
        if (self.wrist_k[0, 0] <= 0 or self.wrist_k[1, 1] <= 0 or
                not np.allclose(self.wrist_k[2], [0, 0, 1]) or
                self.wrist_distortion.shape not in ((4,), (5,), (8,), (12,), (14,)) or
                not np.isfinite(self.wrist_distortion).all() or len(self.wrist_size) != 2 or
                any(type(v) is not int or v <= 0 for v in self.wrist_size)):
            raise ValueError("invalid wrist camera geometry")
        head_ext, wrist_ext = sources["head", "extrinsics"], sources["right_wrist", "extrinsics"]
        robot = self.config["robot"]
        if (head_ext.get("kind") != "sp_vision_pitch_camera_extrinsics" or
                head_ext.get("pitch_link") != "head_pitch_Link" or
                wrist_ext.get("kind") != "sp_vision_right_wrist_extrinsics" or
                wrist_ext.get("mount_link") != robot["mount_link"] or
                wrist_ext.get("camera_frame") != robot["camera_frame"] or
                robot["base_link"] != "base_Link"):
            raise ValueError("unexpected calibrated camera frames")
        self.head_names = head_ext["head_joint_names"]
        self.arm_names = robot["ros2_joint_names"]["left"] + robot["ros2_joint_names"]["right"]
        if len(self.head_names) != 2 or len(set(self.arm_names + self.head_names)) != 16:
            raise ValueError("require 14 named arm joints and 2 named head joints")
        self.head_mount = rigid(head_ext["T_pitch_camera"], "T_pitch_camera")
        self.wrist_mount = rigid(wrist_ext["T_wrist_roll_camera"], "T_wrist_roll_camera")
        self.urdf = common.config_path(self.config, robot["urdf"])
        self.model = common.robot_kinematics(self.config)
        # The capture manifest, when available, binds the original solve's FK model.
        for camera, ext in (("head", head_ext), ("right_wrist", wrist_ext)):
            chain = (ext.get("kinematic_chain") or summary["cameras"][camera].get("kinematic_chain"))
            if chain is not None:
                mount = "head_pitch_Link" if camera == "head" else robot["mount_link"]
                if chain.get("base_link") != robot["base_link"] or chain.get("mount_link") != mount:
                    raise ValueError(f"unexpected {camera} FK chain frames")
                actual = fk_chain_signature(self.model, robot["base_link"], mount)
                if actual != chain.get("signature_sha256"):
                    raise ValueError(f"FK chain differs from the {camera} calibration")
                # Unrelated visual or nominal camera joints do not participate
                # in FK once the calibrated mounting transform is applied.
                continue
            manifest_path = ext.get("input_manifest")
            if manifest_path and Path(manifest_path).is_file():
                manifest = common.read_json(Path(manifest_path))
                expected = manifest.get("solver_files", {}).get("urdf", {}).get("sha256")
                if expected and sha256(self.urdf) != expected:
                    raise ValueError(f"URDF differs from the {camera} solve; use its original --wrist-config")
        self.provenance["urdf"] = {"path": str(self.urdf), "sha256": sha256(self.urdf)}

    def positions(self, joints):
        arm = wrist.arm_state(self.config, joints)
        head = finite([joints[name] for name in self.head_names], (2,), "head joints")
        positions = common.arm_positions(self.config, arm)
        positions.update(zip(self.head_names, head))
        return positions

    def camera_poses(self, joints):
        positions = self.positions(joints)
        base_wrist = self.model.matrix("base_Link", "wrist_roll_R_Link", positions) @ self.wrist_mount
        base_head = self.model.matrix("base_Link", "head_pitch_Link", positions) @ self.head_mount
        return base_wrist, base_head

    def evaluate(self, image, joints):
        if (image.shape[1], image.shape[0]) != self.size:
            raise ValueError(f"head image size differs from calibration {self.size}; do not resize silently")
        base_wrist, base_head = self.camera_poses(joints)
        result = project_center(base_wrist, base_head, self.k, self.distortion, self.size)
        axes = project_axes(base_wrist, base_head, self.k, self.distortion, self.size, self.axis_length_m)
        return {**result, **axes, "T_base_wrist_camera": base_wrist.tolist(),
                "T_base_head_camera": base_head.tolist(), "calibration_files": self.provenance,
                "calibration_summary": str(self.summary_path), "calibration_id": self.calibration_id,
                "image_size": list(self.size)}


@dataclass
class Sample:
    stamp_ms: int
    received: float
    value: object


def joint_sample(raw, now):
    value = json.loads(raw)
    if value.get("type") == "error":
        raise RuntimeError("Bridge subscription error: " + str(value.get("message") or value.get("error")))
    if value.get("type") != "joint_state":
        return None
    stamp = value.get("timestamp")
    data = value.get("data", {})
    names, positions = data.get("names"), data.get("positions")
    if (isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or
            not math.isfinite(stamp) or stamp <= 0 or not isinstance(names, list) or
            not names or not all(isinstance(n, str) and n for n in names) or
            len(names) != len(set(names)) or not isinstance(positions, list) or
            len(names) != len(positions)):
        raise ValueError("Bridge feedback requires an explicit timestamp and matching unique joint names")
    values = finite(positions, (len(names),), "joint positions")
    return Sample(int(stamp), now, dict(zip(names, values.tolist())))


def select_pair(images, states, now, max_age_ms, max_skew_ms):
    """Match the newest usable RGB to measured state at its bridge timestamp."""
    for image in reversed(images):
        if not 0 <= (now - image.received) * 1000 <= max_age_ms:
            continue
        eligible = [s for s in states if 0 <= (now - s.received) * 1000 <= max_age_ms
                    and abs(image.stamp_ms - s.stamp_ms) <= max_skew_ms]
        if eligible:
            return image, min(eligible, key=lambda s: abs(image.stamp_ms - s.stamp_ms))
    return None


def make_record(calibration, image, joints, image_stamp, joint_stamp, source):
    return {"schema_version": 1, "source": source, "read_only": True,
            "accuracy_verified": False, "validation_scope": "wrist_camera_position_and_orientation_overlay",
            "timestamp_source": "bridge_forwarding_ms" if source == "live" else source,
            "image_stamp_ms": image_stamp, "joint_stamp_ms": joint_stamp,
            "state_skew_ms": abs(image_stamp - joint_stamp), "joints": joints,
            **calibration.evaluate(image, joints)}


def add_reference(record, pixel_uv):
    """Measure a user-selected visible feature; it is not an optical ground truth."""
    point = finite(pixel_uv, (2,), "reference pixel")
    width, height = record["image_size"]
    if not (0 <= point[0] < width and 0 <= point[1] < height):
        raise ValueError("reference pixel must be in the original RGB area")
    result = deepcopy(record)
    predicted = record["pixel_uv"]
    delta = point - np.asarray(predicted) if predicted is not None else None
    result["manual_reference"] = {
        "kind": "user_selected_visible_feature",
        "optical_center_equivalence_verified": False,
        "pixel_uv": point.tolist(), "image_stamp_ms": record["image_stamp_ms"],
        "observed_minus_predicted_px": delta.tolist() if delta is not None else None,
        "distance_px": float(np.linalg.norm(delta)) if delta is not None else None}
    return result


class FrameReview:
    """Freeze exactly the displayed RGB and its matched feedback before clicking."""

    def __init__(self):
        self.image = self.record = None

    def toggle(self, image, record):
        if self.record is not None:
            self.image = self.record = None
        elif record is not None:
            self.image, self.record = image.copy(), deepcopy(record)
            self.record["display_frozen"] = True

    def displayed(self, image, record):
        return (self.image, self.record) if self.record is not None else (image, record)

    def mouse(self, event, x, y, flags, parameter):
        if event != cv2.EVENT_LBUTTONDOWN or self.record is None:
            return
        if 0 <= x < self.image.shape[1] and 0 <= y < self.image.shape[0]:
            self.record = add_reference(self.record, [x, y])


def render(image, record=None, status="Waiting for fresh RGB and named joint feedback"):
    # Keep RGB pixels unobstructed and unchanged in scale; diagnostics go below.
    drawing = np.vstack((image, np.full((137, image.shape[1], 3), 20, np.uint8)))
    rgb = drawing[:image.shape[0]]
    lines = [status, "READ ONLY: position + orientation; physical accuracy unverified"]
    if record is not None:
        point = record["wrist_position_base_m"]
        uv = record["pixel_uv"]
        pixel_text = "n/a" if uv is None else f"{uv[0]:.1f}, {uv[1]:.1f}"
        frozen = "FROZEN " if record.get("display_frozen") else ""
        reference = record.get("manual_reference")
        reference_text = "Space: freeze/resume; frozen: click same lens feature; s: save"
        if reference is not None:
            delta = reference["observed_minus_predicted_px"]
            reference_text = (f"Selected feature: du={delta[0]:+.1f}, dv={delta[1]:+.1f}; distance={reference['distance_px']:.1f} px"
                              if delta is not None else "Selected feature; predicted center unavailable")
        lines = [f"{frozen}{record['source'].upper()}: {record['status']}  skew={record['state_skew_ms']:.1f} ms",
                 f"base XYZ (m): {point[0]:+.4f} {point[1]:+.4f} {point[2]:+.4f}",
                 f"pixel uv: {pixel_text}; head-camera Z: {record['wrist_position_head_m'][2]:.4f} m",
                 f"Optical axes: X red, Y green, Z blue; length={record.get('axis_length_m', .08):g} m",
                 reference_text,
                 "READ ONLY: position + orientation; physical accuracy unverified"]
        if record["visible"]:
            for name, axis in record.get("axes", {}).items():
                if not axis["drawable"]:
                    continue
                start, end = (tuple(p) for p in axis["segment_uv"])
                color = AXIS_COLORS[name]
                if np.linalg.norm(np.subtract(end, start)) >= 2:
                    cv2.arrowedLine(rgb, start, end, color, 2, cv2.LINE_AA, tipLength=.15)
                else:
                    cv2.circle(rgb, end, 3, color, -1, cv2.LINE_AA)
                label = (min(max(2, end[0] + 5), image.shape[1] - 16),
                         min(max(14, end[1] - 5), image.shape[0] - 3))
                cv2.putText(rgb, name, label, cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2, cv2.LINE_AA)
            pixel = tuple(np.rint(record["pixel_uv"]).astype(int))
            cv2.drawMarker(rgb, pixel, (0, 0, 255), cv2.MARKER_CROSS, 23, 2)
            cv2.circle(rgb, pixel, 10, (0, 0, 255), 2)
            cv2.putText(rgb, "wrist optical center", (max(0, pixel[0] - 75), max(14, pixel[1] - 16)),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 1, cv2.LINE_AA)
        if reference is not None:
            selected = tuple(map(int, np.rint(reference["pixel_uv"])))
            selected = (min(selected[0], image.shape[1] - 1), min(selected[1], image.shape[0] - 1))
            if uv is not None and np.max(np.abs(uv)) <= 1e8:
                start = tuple(map(int, np.rint(uv)))
                ok, p0, p1 = cv2.clipLine((0, 0, image.shape[1], image.shape[0]), start, selected)
                if ok:
                    cv2.line(rgb, p0, p1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.drawMarker(rgb, selected, (0, 255, 0), cv2.MARKER_TILTED_CROSS, 17, 2)
            label = (min(selected[0] + 7, image.shape[1] - 86), max(14, selected[1] - 8))
            cv2.putText(rgb, "reference", label, cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1, cv2.LINE_AA)
    for i, line in enumerate(lines):
        cv2.putText(drawing, line, (8, image.shape[0] + 19 + 21 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    .45, (240, 240, 240), 1, cv2.LINE_AA)
    return drawing


def save_capture(output, number, image, record):
    directory = Path(output) / f"view-{number:03d}"
    directory.mkdir()
    for filename, pixels in (("color.png", image), ("overlay.png", render(image, record))):
        if not cv2.imwrite(str(directory / filename), pixels):
            raise OSError(f"cannot save {directory / filename}")
    common.write_json(directory / "projection.json", {**record, "color_sha256": sha256(directory / "color.png")})
    print(json.dumps({"capture": str(directory), "source": record["source"],
                      "status": record["status"], "pixel_uv": record["pixel_uv"],
                      "state_skew_ms": record["state_skew_ms"]}), flush=True)


def mock_capture(calibration):
    width, height = calibration.size
    image = np.full((height, width, 3), 205, np.uint8)
    for x in range(0, width, 40):
        cv2.line(image, (x, 0), (x, height - 1), (180, 180, 180), 1)
    for y in range(0, height, 40):
        cv2.line(image, (0, y), (width - 1, y), (180, 180, 180), 1)
    cv2.putText(image, "MOCK: synthetic background and joints", (12, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (30, 30, 30), 2)
    values = [0.] * 7 + [.8, -2.9, -.5, -1.1, 0., -.7, 0.]
    joints = dict(zip(calibration.arm_names, values))
    joints.update(zip(calibration.head_names, [1., 0.]))
    stamp = int(time.time() * 1000)
    return image, make_record(calibration, image, joints, stamp, stamp, "mock")


def bridge_settings(profile_path, domain_id):
    camera = common.read_json(profile_path)["camera"]
    if camera.get("backend") != "bridge" or camera.get("color_topic") != HEAD_TOPIC:
        raise ValueError(f"live mode requires bridge profile for {HEAD_TOPIC}")
    host = camera["bridge_host"].rstrip("/")
    if "://" not in host:
        host = "wss://" + host
    parsed = urlsplit(host)
    if parsed.scheme not in ("ws", "wss") or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in ("", "/"):
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
    token = os.environ.get(camera.get("token_env", "TRON2_BRIDGE_TOKEN"))
    joint_topic = camera.get("joint_state_topic", "/joint_states")
    def url(topic, kind):
        query = {"topic": topic, "kind": kind, "domain_id": domain_id}
        if kind == "image":
            query["max_fps"] = 30
        return host + path + "?" + urlencode(query)
    return url(HEAD_TOPIC, "image"), url(joint_topic, "joint"), context, token


def bridge_options(context, token, timeout):
    import websockets
    keywords = {"ssl": context, "open_timeout": timeout, "close_timeout": .25,
                "max_size": 8 * 1024 * 1024, "max_queue": 4}
    signature = inspect.signature(websockets.connect).parameters
    if "proxy" in signature:
        keywords["proxy"] = None
    if token:
        keywords["additional_headers" if "additional_headers" in signature else "extra_headers"] = {"X-Internal-Token": token}
    return keywords


async def read_bridge(url, sink, calibration, keywords, size=None):
    """Read images or named measured joints; this sends no robot commands."""
    import websockets
    try:
        async with websockets.connect(url, **keywords) as socket:
            try:
                async for raw in socket:
                    now = time.monotonic()
                    sample = None
                    if isinstance(raw, str):
                        parsed = joint_sample(raw, now)
                        if size is None:
                            sample = parsed
                            if sample is not None:
                                calibration.positions(sample.value)
                    elif size is not None:
                        frame = _decode_brdg(raw)
                        if frame is not None:
                            if frame.timestamp_ms <= 0 or frame.mime not in ("image/jpeg", "image/png"):
                                raise ValueError("RGB lacks a valid timestamp or compressed color payload")
                            image = cv2.imdecode(np.frombuffer(frame.payload, np.uint8), cv2.IMREAD_COLOR)
                            if image is None or (image.shape[1], image.shape[0]) != size:
                                raise ValueError("RGB dimensions/encoding differ from calibration")
                            sample = Sample(frame.timestamp_ms, now, image)
                    if sample is not None and (not sink or sample.stamp_ms > sink[-1].stamp_ms):
                        sink.append(sample)
                raise RuntimeError("Bridge subscription closed")
            finally:
                socket.transport.abort()
    except (websockets.exceptions.WebSocketException, asyncio.TimeoutError) as error:
        raise RuntimeError(f"Bridge connection failed: {error}") from error


async def live(calibration, args):
    image_url, joint_url, context, token = bridge_settings(args.profile, args.domain_id)
    images, states = deque(maxlen=30), deque(maxlen=200)
    keywords = bridge_options(context, token, args.timeout)
    tasks = [asyncio.create_task(read_bridge(image_url, images, calibration, keywords, calibration.size)),
             asyncio.create_task(read_bridge(joint_url, states, calibration, keywords))]
    started = last_good = time.monotonic()
    previous = None
    processed = saved = 0
    reference_captures = 0
    review = FrameReview()
    mouse_ready = False
    failure = None
    try:
        while True:
            for task in tasks:
                if task.done():
                    task.result()
            now = time.monotonic()
            pair = select_pair(images, states, now, args.max_age_ms, args.max_skew_ms)
            record = None
            image = images[-1].value if images else np.zeros((calibration.size[1], calibration.size[0], 3), np.uint8)
            if pair:
                rgb, state = pair
                image = rgb.value
                record = make_record(calibration, image, state.value, rgb.stamp_ms, state.stamp_ms, "live")
                record["image_receipt_age_ms"] = (now - rgb.received) * 1000
                record["joint_receipt_age_ms"] = (now - state.received) * 1000
                record["newest_image_gap_ms"] = images[-1].stamp_ms - rgb.stamp_ms
                last_good = now
                if rgb.stamp_ms != previous:
                    previous = rgb.stamp_ms
                    processed += 1
                    if args.headless or processed == 1 or args.frames:
                        saved += 1
                        save_capture(args.output, saved, image, record)
                    if args.frames and processed >= args.frames:
                        return
            if not args.headless:
                shown_image, shown_record = review.displayed(image, record)
                cv2.imshow(WINDOW, render(shown_image, shown_record, "STALE / UNSYNCHRONIZED: projection hidden"))
                if not mouse_ready:
                    cv2.setMouseCallback(WINDOW, review.mouse)
                    mouse_ready = True
                key = cv2.waitKey(1) & 0xff
                if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    return
                if key == ord(" "):
                    review.toggle(image, record)
                shown_image, shown_record = review.displayed(image, record)
                if key == ord("s") and shown_record is not None:
                    saved += 1
                    save_capture(args.output, saved, shown_image, shown_record)
                    reference_captures += int("manual_reference" in shown_record)
            if now - last_good > args.timeout:
                raise TimeoutError(f"no fresh synchronized head RGB/joints for {args.timeout:g}s (RGB={len(images)}, joints={len(states)})")
            await asyncio.sleep(.02)
    except Exception as error:
        failure = f"{type(error).__name__}: {error}"
        raise
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        if not args.headless:
            cv2.destroyAllWindows()
        common.write_json(args.output / "run.json", {"mode": "live", "read_only": True,
                          "accuracy_verified": False, "processed_frames": processed,
                          "saved_captures": saved, "elapsed_s": time.monotonic() - started,
                          "reference_captures": reference_captures,
                          "max_state_skew_ms": args.max_skew_ms, "max_receipt_age_ms": args.max_age_ms,
                          "axis_length_m": calibration.axis_length_m,
                          "error": failure})


def select_board_samples(head_images, wrist_images, states, now, max_age_ms, max_skew_ms):
    """Pair both RGB streams, each with feedback matched to its own image time."""
    for head in reversed(head_images):
        head_pair = select_pair([head], states, now, max_age_ms, max_skew_ms)
        if head_pair is None:
            continue
        for wrist_image in sorted(wrist_images, key=lambda w: abs(w.stamp_ms - head.stamp_ms)):
            if abs(wrist_image.stamp_ms - head.stamp_ms) > max_skew_ms:
                continue
            wrist_pair = select_pair([wrist_image], states, now, max_age_ms, max_skew_ms)
            if wrist_pair is not None:
                return head, head_pair[1], wrist_image, wrist_pair[1]
    return None


async def capture_board_samples(calibration, args):
    head_url, joint_url, context, token = bridge_settings(args.profile, args.domain_id)
    parsed = urlsplit(head_url)
    query = dict(parse_qsl(parsed.query))
    query["topic"] = calibration.config["capture"]["color_topic"]
    wrist_url = parsed._replace(query=urlencode(query)).geturl()
    heads, wrists, states = deque(maxlen=30), deque(maxlen=30), deque(maxlen=200)
    keywords = bridge_options(context, token, args.timeout)
    tasks = [asyncio.create_task(read_bridge(head_url, heads, calibration, keywords, calibration.size)),
             asyncio.create_task(read_bridge(wrist_url, wrists, calibration, keywords, calibration.wrist_size)),
             asyncio.create_task(read_bridge(joint_url, states, calibration, keywords))]
    started = time.monotonic()
    try:
        while True:
            for task in tasks:
                if task.done():
                    task.result()
            now = time.monotonic()
            pair = select_board_samples(heads, wrists, states, now, args.max_age_ms, args.max_skew_ms)
            if pair is not None:
                head, head_state, wrist_image, wrist_state = pair
                images, metadata = {}, {}
                for name, image, state in (("head", head, head_state), ("wrist", wrist_image, wrist_state)):
                    images[name] = image.value.copy()
                    metadata[name] = {"image_stamp_ms": image.stamp_ms, "joint_stamp_ms": state.stamp_ms,
                                      "state_skew_ms": abs(image.stamp_ms - state.stamp_ms),
                                      "image_receipt_age_ms": (now - image.received) * 1000,
                                      "joint_receipt_age_ms": (now - state.received) * 1000,
                                      "joints": dict(state.value)}
                metadata["image_skew_ms"] = abs(head.stamp_ms - wrist_image.stamp_ms)
                return images, metadata
            if now - started > args.timeout:
                raise TimeoutError(f"no synchronized dual RGB/joints for {args.timeout:g}s "
                                   f"(head={len(heads)}, wrist={len(wrists)}, joints={len(states)})")
            await asyncio.sleep(.02)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


def compare_board_geometry(calibration, head_corners, wrist_corners, base_head, base_wrist, objects):
    """Use independently detected, physically corresponding board corner IDs."""
    head_board, head_rms = common.solve_pnp(head_corners, calibration.k, calibration.distortion, objects)
    wrist_board, wrist_rms = common.solve_pnp(wrist_corners, calibration.wrist_k, calibration.wrist_distortion, objects)
    base_board_head = base_head @ head_board
    base_board_wrist = base_wrist @ wrist_board
    difference = common.invert(base_board_head) @ base_board_wrist
    predictions = {"head": common.invert(base_head) @ base_board_wrist,
                   "wrist": common.invert(base_wrist) @ base_board_head}
    result = {"head_pnp_rms_px": head_rms, "wrist_pnp_rms_px": wrist_rms,
              "T_base_board_from_head": base_board_head.tolist(),
              "T_base_board_from_wrist": base_board_wrist.tolist(),
              "translation_difference_mm": float(np.linalg.norm(difference[:3, 3]) * 1000),
              "rotation_difference_deg": math.degrees(common.rotation_angle(difference[:3, :3])),
              "cross_projection": {}}
    for name, corners, k, dist in (("head", head_corners, calibration.k, calibration.distortion),
                                    ("wrist", wrist_corners, calibration.wrist_k, calibration.wrist_distortion)):
        transform = predictions[name]
        points = objects @ transform[:3, :3].T + transform[:3, 3]
        if not np.isfinite(points).all() or np.any(points[:, 2] <= 1e-9):
            raise ValueError(f"{name} cross-projected board has invalid/negative depth")
        uv = cv2.projectPoints(points, np.zeros(3), np.zeros(3), k, dist)[0].reshape(-1, 2)
        if not np.isfinite(uv).all():
            raise ValueError(f"{name} board cross-projection is nonfinite")
        errors = np.linalg.norm(uv - corners, axis=1)
        result["cross_projection"][name] = {"observed_uv": corners.tolist(), "predicted_uv": uv.tolist(),
                                             "rms_px": float(np.sqrt(np.mean(errors**2))),
                                             "max_px": float(errors.max())}
    return result


async def board_check(calibration, args):
    """Save one dual-camera board check or replay one, without motor commands."""
    source = "replay" if args.capture is not None else "live"
    if args.capture is not None:
        old = common.read_json(args.capture / "board-check.json")
        capture = deepcopy(old["capture"])
        images = {}
        for name in ("head", "wrist"):
            path = args.capture / f"{name}-color.png"
            if sha256(path) != capture[name]["color_sha256"]:
                raise ValueError(f"{name} replay RGB differs from its recorded hash")
            if abs(capture[name]["image_stamp_ms"] - capture[name]["joint_stamp_ms"]) > args.max_skew_ms:
                raise ValueError(f"{name} replay image/state skew exceeds limit")
            images[name] = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if abs(capture["head"]["image_stamp_ms"] - capture["wrist"]["image_stamp_ms"]) > args.max_skew_ms:
            raise ValueError("replayed dual RGB timestamps exceed skew limit")
    else:
        images, capture = await capture_board_samples(calibration, args)
    config = deepcopy(calibration.config)
    common.apply_board_arguments(config, args)
    objects = common.board_points(config)
    report = {"schema_version": 1, "source": source, "read_only": True, "accuracy_verified": False,
              "calibration_id": calibration.calibration_id, "calibration_summary": str(calibration.summary_path),
              "calibration_files": calibration.provenance, "board_pattern": config["board"],
              "assumed_board_fixed_in_base": True, "corner_order_confirmed": args.confirm_corner_order,
              "flip_head": args.flip_head, "flip_wrist": args.flip_wrist,
              "capture": capture, "measurement_available": False, "status": "board_not_detected"}
    if args.capture is not None:
        report["replay_capture"] = str(args.capture.resolve())
        report["original_source"] = old.get("original_source", old["source"])
    detected, metrics = {}, {}
    for name, size, flip in (("head", calibration.size, args.flip_head),
                              ("wrist", calibration.wrist_size, args.flip_wrist)):
        image = images[name]
        if image is None or (image.shape[1], image.shape[0]) != size:
            raise ValueError(f"{name} board RGB dimensions differ from its calibration")
        calibration.positions(capture[name]["joints"])
        raw = args.output / f"{name}-color.png"
        if not cv2.imwrite(str(raw), image):
            raise OSError(f"cannot save {raw}")
        capture[name]["color_sha256"] = sha256(raw)
        detected[name], metrics[name] = common.detect_board(image, config, flip)
        marked = common.draw_detection(image, detected[name], config, metrics[name])
        if not cv2.imwrite(str(args.output / f"{name}-corners.png"), marked):
            raise OSError(f"cannot save {name} corner preview")
    report["detection"] = metrics
    if all(detected[name] is not None for name in ("head", "wrist")):
        if not all(metrics[name]["accepted"] for name in ("head", "wrist")):
            report["status"] = "board_detection_quality_rejected"
        else:
            try:
                base_head = calibration.camera_poses(capture["head"]["joints"])[1]
                base_wrist = calibration.camera_poses(capture["wrist"]["joints"])[0]
                geometry = compare_board_geometry(calibration, detected["head"], detected["wrist"],
                                                   base_head, base_wrist, objects)
                report.update(geometry)
                pnp_limit = float(config["quality"].get("max_pnp_rms_px", 1.5))
                report["measurement_available"] = max(geometry["head_pnp_rms_px"], geometry["wrist_pnp_rms_px"]) <= pnp_limit
                report["status"] = "needs_corner_identity_review" if report["measurement_available"] else "pnp_quality_rejected"
                limits = {"translation_mm": float(config["quality"]["max_board_residual_m"]) * 1000,
                          "rotation_deg": float(config["quality"]["max_board_residual_deg"])}
                report["comparison_limits"] = limits
                report["relative_pose_within_limits"] = None
                if args.confirm_corner_order and report["measurement_available"]:
                    within = (geometry["translation_difference_mm"] <= limits["translation_mm"] and
                              geometry["rotation_difference_deg"] <= limits["rotation_deg"])
                    report["relative_pose_within_limits"] = within
                    report["status"] = "relative_pose_within_limits" if within else "relative_pose_exceeds_limits"
                for name in ("head", "wrist"):
                    drawing = common.draw_detection(images[name], detected[name], config, metrics[name])
                    for actual, predicted in zip(detected[name], geometry["cross_projection"][name]["predicted_uv"]):
                        if np.max(np.abs(predicted)) > 1e8:
                            continue
                        start = tuple(map(int, np.rint(actual))); end = tuple(map(int, np.rint(predicted)))
                        ok, p0, p1 = cv2.clipLine((0, 0, drawing.shape[1], drawing.shape[0]), start, end)
                        if ok:
                            cv2.line(drawing, p0, p1, (0, 255, 255), 1, cv2.LINE_AA)
                        if 0 <= end[0] < drawing.shape[1] and 0 <= end[1] < drawing.shape[0]:
                            cv2.circle(drawing, end, 3, (0, 0, 255), 1, cv2.LINE_AA)
                    panel = np.full((52, drawing.shape[1], 3), 20, np.uint8)
                    text = f"Board difference: {geometry['translation_difference_mm']:.2f} mm, {geometry['rotation_difference_deg']:.2f} deg"
                    cv2.putText(panel, text, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .45, (240, 240, 240), 1)
                    note = "Corner IDs confirmed; relative consistency only" if args.confirm_corner_order else "Check that (0,0) is the SAME physical corner in both images"
                    cv2.putText(panel, note, (8, 41), cv2.FONT_HERSHEY_SIMPLEX, .43, (240, 240, 240), 1)
                    if not cv2.imwrite(str(args.output / f"{name}-cross-projection.png"), np.vstack((drawing, panel))):
                        raise OSError("cannot save board cross-projection")
            except (ValueError, cv2.error) as error:
                report.update(status="board_pose_or_projection_failed", error=str(error))
    common.write_json(args.output / "board-check.json", report)
    print(json.dumps({"board_check": str(args.output / "board-check.json"), "status": report["status"],
                      "measurement_available": report["measurement_available"],
                      "translation_difference_mm": report.get("translation_difference_mm"),
                      "rotation_difference_deg": report.get("rotation_difference_deg")}), flush=True)
    return report["measurement_available"] and report.get("relative_pose_within_limits") is not False


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--calibration-summary", type=Path, default=DEFAULT_CALIBRATION_SUMMARY,
                        help="pinned calibration manifest; defaults to configs/local-camera-calibration-latest.json")
    result.add_argument("--wrist-config", type=Path, default=ROOT / "sp_vision/configs/wrist_config.example.json")
    result.add_argument("--output", required=True, type=Path, help="new or empty output directory")
    result.add_argument("--axis-length-m", type=float, default=.08,
                        help="positive X/Y/Z axis length in meters (default: 0.08)")
    result.add_argument("--max-skew-ms", type=float, default=30.)
    result.add_argument("--max-age-ms", type=float, default=750.)
    result.add_argument("--timeout", type=float, default=8.)
    modes = result.add_subparsers(dest="mode", required=True)
    modes.add_parser("mock", help="save a synthetic example without opening hardware or a window")
    replay = modes.add_parser("replay", help="recompute one saved color.png + projection.json")
    replay.add_argument("--capture", required=True, type=Path)
    online = modes.add_parser("live", help="explicitly subscribe to head RGB and measured joints")
    online.add_argument("--profile", required=True, type=Path)
    online.add_argument("--domain-id", type=int, default=0)
    online.add_argument("--headless", action="store_true")
    online.add_argument("--frames", type=int, default=0, help="save and exit after N matched frames; 0 is unlimited GUI mode")
    board = modes.add_parser("board-check", help="compare one fixed checkerboard seen by both RGB cameras; no GUI")
    source = board.add_mutually_exclusive_group(required=True)
    source.add_argument("--profile", type=Path, help="explicitly read both camera streams and measured joints")
    source.add_argument("--capture", type=Path, help="replay a saved board-check directory without a connection")
    board.add_argument("--domain-id", type=int, default=0)
    board.add_argument("--flip-head", action="store_true", help="reverse the head checkerboard corner IDs by 180 degrees")
    board.add_argument("--flip-wrist", action="store_true", help="reverse the wrist checkerboard corner IDs by 180 degrees")
    board.add_argument("--confirm-corner-order", action="store_true", help="use only after both (0,0) labels identify the same physical corner")
    common.add_board_arguments(board)
    return result


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    try:
        if any(not math.isfinite(v) or v <= 0 for v in (args.max_skew_ms, args.max_age_ms, args.timeout)):
            raise ValueError("time limits must be positive and finite")
        if args.mode == "live":
            if args.frames < 0 or (args.headless and not args.frames):
                raise ValueError("headless live mode requires --frames N > 0")
            if not args.headless and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                raise ValueError("no graphical display; use live --headless --frames 1")
        calibration = Calibration(args.calibration_summary, args.wrist_config, args.axis_length_m)
        if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
            raise ValueError("output must be a new or empty directory")
        args.output.mkdir(parents=True, exist_ok=True)
        print(f"Calibration: {calibration.calibration_id}\nManifest: {calibration.summary_path}", flush=True)
        if args.mode == "board-check":
            return 0 if asyncio.run(board_check(calibration, args)) else 1
        if args.mode == "live":
            print("READ ONLY: head RGB + /joint_states; Space=freeze/resume, click frozen RGB=reference, s=save, q=quit.", flush=True)
            asyncio.run(live(calibration, args))
        else:
            if args.mode == "mock":
                image, record = mock_capture(calibration)
            else:
                old = common.read_json(args.capture / "projection.json")
                path = args.capture / "color.png"
                if sha256(path) != old["color_sha256"]:
                    raise ValueError("replay image does not match the recorded image hash")
                if abs(old["image_stamp_ms"] - old["joint_stamp_ms"]) > args.max_skew_ms:
                    raise ValueError("replay state/image timestamp skew exceeds limit")
                image = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("cannot decode replay color.png")
                record = make_record(calibration, image, old["joints"], old["image_stamp_ms"], old["joint_stamp_ms"], "replay")
                record["original_source"] = old["source"]
                record["replay_capture"] = str(args.capture.resolve())
                if old.get("manual_reference") is not None:
                    record = add_reference(record, old["manual_reference"]["pixel_uv"])
            save_capture(args.output, 1, image, record)
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, KeyError, RuntimeError, cv2.error) as error:
        print(f"Projection unavailable: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
