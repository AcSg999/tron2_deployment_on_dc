"""Intrinsic/hand-eye calibration and stationary per-arm sample collection."""
from pathlib import Path
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .camera import capture, decode_image
from .config import finite, rigid, write_json
from .handeye import calibrate_handeye


def board_points(pattern=(9, 6), square_m=0.025):
    if len(pattern) != 2 or min(pattern) < 2 or not np.isfinite(square_m) or square_m <= 0:
        raise ValueError("board dimensions and measured square size must be positive")
    points = np.zeros((pattern[0]*pattern[1], 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)*square_m
    return points


def corners(image, pattern=(9, 6)):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    found, points = cv2.findChessboardCorners(gray, pattern)
    if not found:
        raise ValueError("chessboard inner corners were not detected")
    return cv2.cornerSubPix(gray, points, (11, 11), (-1, -1),
                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))


def intrinsic_fit(image_paths, pattern=(9, 6), square_m=0.025):
    object_points, image_points, accepted = [], [], []
    shape = None
    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"cannot read calibration image {path}")
        if shape is not None and image.shape[:2] != shape:
            raise ValueError("calibration images must use one camera resolution")
        shape = image.shape[:2]
        try:
            found = corners(image, pattern)
        except ValueError:
            continue
        object_points.append(board_points(pattern, square_m))
        image_points.append(found)
        accepted.append(str(path))
    if len(accepted) < 5:
        raise ValueError("at least five detected board views are required")
    rms, k, dist, _, _ = cv2.calibrateCamera(object_points, image_points, shape[::-1], None, None)
    if not np.isfinite(rms) or not np.isfinite(k).all() or not np.isfinite(dist).all():
        raise ValueError("intrinsic solve returned non-finite calibration")
    return {"K": k.tolist(), "dist": dist.ravel().tolist(), "height": shape[0], "width": shape[1],
            "samples": len(accepted), "rms_px": float(rms), "images": accepted,
            "pattern": list(pattern), "square_m": square_m, "verified": False}


def record_sample(profile, side, output, *, mock=False, pattern=(9, 6), square_m=0.025, on_frame=None):
    """Observe a stationary arm; reposition through the robot's separate operator interface."""
    from .robot import MockRobot, WebsocketRobot
    from .kinematics import RobotModel
    from .fp_client import pose7_to_matrix

    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    adapter = MockRobot(profile) if mock else WebsocketRobot(profile)
    try:
        before = adapter.read_state()
        before_received = time.time()
        frame = capture(profile, mock=mock, undistort=False)
        after = adapter.read_state()
    finally:
        adapter.close()
    now = time.time()
    max_age = float(profile.get("execution", {}).get("start_max_age_s", 0.25))
    calibration = profile["calibration"]
    max_span = float(calibration.get("max_sample_span_s", 1.0))
    max_skew = float(calibration.get("max_sample_skew_s", 0.25))
    if not np.isfinite([max_age, max_span, max_skew]).all() or min(max_age, max_span, max_skew) <= 0:
        raise ValueError("calibration sample timing tolerances must be positive and finite")
    before_stamp, after_stamp = float(before["timestamp_s"]), float(after["timestamp_s"])
    if (not np.isfinite([before_stamp, after_stamp]).all() or min(before_stamp, after_stamp) <= 0
            or before_received-before_stamp > max_age or now-after_stamp > max_age
            or before_stamp-before_received > 0.05 or after_stamp-now > 0.05):
        raise ValueError("stale robot feedback during calibration capture")
    captured = float(frame["capture_timestamp_s"])
    if (not np.isfinite(captured) or after_stamp < before_stamp or now-before_received > max_span
            or captured < before_stamp-0.05 or captured > after_stamp+0.05
            or max(abs(captured-before_stamp), abs(after_stamp-captured)) > max_skew):
        raise ValueError("calibration image is not closely bracketed by fresh arm feedback")
    sync = frame.get("sensor_sync", {})
    if (sync.get("state_header_skew_exceeded") or sync.get("state_alignment") == "stable_head_receipt_fallback"
            or float(sync.get("state_skew_ms", 0)) > float(profile["camera"].get("max_state_skew_ms", 100))):
        raise ValueError("hand-eye calibration requires synchronized arm/image timestamps; head-only fallback is insufficient")
    if max(np.max(np.abs(np.array(after[key])-np.array(before[key])))
           for key in ("arm_q14", "head_q2")) > 0.005:
        raise ValueError("robot moved during calibration sample; collect after settling")
    if np.max(np.abs(np.array(frame["head_q2"])-np.array(after["head_q2"]))) > 0.005:
        raise ValueError("image and measured head pose do not match")
    if on_frame is not None:
        from copy import deepcopy
        on_frame(deepcopy(frame))
    image = decode_image(frame["image"], cv2.IMREAD_COLOR)
    found = corners(image, pattern)
    k = np.array(profile["camera"]["intrinsics"])
    dist = np.array(profile["camera"]["distortion"])
    ok, rv, tv = cv2.solvePnP(board_points(pattern, square_m), found, k, dist)
    if not ok:
        raise ValueError("board pose solve failed")
    target = np.eye(4)
    target[:3, :3] = cv2.Rodrigues(rv)[0]
    target[:3, 3] = tv.ravel()
    model = RobotModel(profile)
    model.set_state(after["arm_q14"], after["head_q2"])
    wrist = model.wrist_poses()[side]
    path = Path(output)
    path.mkdir(parents=True, exist_ok=True)
    stamp = str(time.time_ns())
    cv2.imwrite(str(path/f"{stamp}.png"), image)
    sample = {"side": side, "source": frame["source"], "timestamp_s": now,
              "robot_gripper_to_base": pose7_to_matrix(wrist).tolist(),
              "target_to_camera": target.tolist(), "arm_q14": after["arm_q14"],
              "head_q2": after["head_q2"], "frame": {k: v for k, v in frame.items() if k not in ("image", "depth")},
              "image": f"{stamp}.png", "pattern": list(pattern), "square_m": square_m}
    return write_json(path/f"{stamp}.json", sample)


def solve_samples(samples, mode="eye_to_hand"):
    if len(samples) < 5:
        raise ValueError("at least five stationary samples are required")
    for key in ("side", "source", "pattern", "square_m"):
        if any(item[key] != samples[0][key] for item in samples):
            raise ValueError(f"hand-eye sample {key} must be consistent")
    camera_ids = {item["frame"]["camera_id"] for item in samples}
    if len(camera_ids) != 1:
        raise ValueError("hand-eye samples must use the same physical camera")
    head = np.array([item["head_q2"] for item in samples])
    if np.max(np.abs(head-head[0])) > 0.005:
        raise ValueError("fixed high-camera hand-eye calibration requires a stationary head")
    robot = np.array([rigid(item["robot_gripper_to_base"], "wrist transform") for item in samples])
    target = np.array([rigid(item["target_to_camera"], "board transform") for item in samples])
    rotations = Rotation.from_matrix(robot[:, :3, :3])
    if max((rotations[0].inv()*rotations).magnitude()) < np.deg2rad(15):
        raise ValueError("hand-eye samples need at least 15 degrees of rotation spread")
    result = calibrate_handeye(robot, target, mode=mode)
    result.update(source=samples[0]["source"], head_q2=head[0].tolist(),
                  side=samples[0]["side"], camera_id=next(iter(camera_ids)), verified=False)
    return result


def validate_extrinsics(camera_to_base, points_camera, points_base, max_error_m):
    a, b = np.asarray(points_camera, dtype=float), np.asarray(points_base, dtype=float)
    if a.ndim != 2 or a.shape[1:] != (3,) or a.shape != b.shape or len(a) < 3:
        raise ValueError("held-out points must be matching Nx3 arrays, N>=3")
    if not np.isfinite(a).all() or not np.isfinite(b).all() or np.linalg.matrix_rank(a-a.mean(0)) < 2:
        raise ValueError("held-out points must be finite and non-collinear")
    if not np.isfinite(max_error_m) or max_error_m <= 0:
        raise ValueError("validation tolerance must be positive")
    transform = rigid(camera_to_base, "camera_to_base")
    errors = np.linalg.norm(a @ transform[:3, :3].T + transform[:3, 3] - b, axis=1)
    return {"passed": bool(np.max(errors) <= max_error_m), "rms_m": float(np.sqrt(np.mean(errors**2))),
            "max_error_m": float(np.max(errors)), "tolerance_m": max_error_m, "sample_count": len(a)}
