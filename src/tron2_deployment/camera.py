"""Profile-bound, read-only RGB-D capture. Nothing connects at import time."""
import base64
import os
import time
import uuid

import cv2
import numpy as np

from .config import finite, rigid


def encode_image(image, suffix=".png"):
    ok, buffer = cv2.imencode(suffix, image)
    if not ok:
        raise ValueError("cannot encode image")
    mime = "jpeg" if suffix == ".jpg" else "png"
    return f"data:image/{mime};base64," + base64.b64encode(buffer).decode()


def decode_image(value, flags=cv2.IMREAD_UNCHANGED):
    if not isinstance(value, str) or not value.startswith("data:image/") or ";base64," not in value:
        raise ValueError("image must be an embedded base64 image")
    raw = base64.b64decode(value.split(",", 1)[1], validate=True)
    result = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), flags)
    if result is None:
        raise ValueError("cannot decode image")
    return result


def capture(profile, mock=False, undistort=True):
    config = profile["camera"]
    calibration = profile["calibration"]
    source = "mock" if mock else "real"
    if not mock and config["backend"] == "mock":
        raise ValueError("mock camera requires --mock; it cannot provide real observations")
    if mock:
        height, width = config["height"], config["width"]
        rgb = np.full((height, width, 3), (218, 224, 229), dtype=np.uint8)
        cv2.ellipse(rgb, (width//2, height//2), (width//6, height//5), 0, 0, 360, (61, 153, 173), -1)
        depth = np.full((height, width), 800, dtype=np.uint16)
        head = np.array(calibration["head_q2"])
        sync = {"source": "synthetic", "color_timestamp_ms": int(time.time()*1000)}
    else:
        from .rgbd import Tron2HighRgbdConfig, Tron2HighRgbdCapture, Tron2RosHighRgbdCapture
        depth_to_color = rigid(config["depth_to_color"], "depth_to_color")
        options = {
            "color_k": tuple(np.array(config["intrinsics"]).ravel()),
            "color_dist": tuple(config["distortion"]),
            "depth_k": tuple(np.array(config["depth_intrinsics"]).ravel()),
            "depth_to_color_r": tuple(depth_to_color[:3, :3].ravel()),
            "depth_to_color_t_m": tuple(depth_to_color[:3, 3]),
            "ros_color_topic": config["color_topic"],
            "ros_depth_topic": config["depth_topic"],
            "joint_state_topic": config.get("joint_state_topic", "/joint_states"),
            "host": config.get("bridge_host", "127.0.0.1:18443"),
            "ws_path": config.get("bridge_path", "/bridge/ws"),
            "internal_token": os.environ.get(config.get("token_env", "TRON2_BRIDGE_TOKEN")),
            "verify_tls": config.get("verify_tls", True),
            "max_skew_ms": config.get("max_skew_ms", 100),
            "max_state_skew_ms": config.get("max_state_skew_ms", 100),
        }
        if config["backend"] == "ros":
            os.environ["ROS_MASTER_URI"] = config["ros_master_uri"]
            if config.get("ros_ip"):
                os.environ["ROS_IP"] = config["ros_ip"]
            driver_type = Tron2RosHighRgbdCapture
        else:
            driver_type = Tron2HighRgbdCapture
        driver = driver_type(Tron2HighRgbdConfig(**options))
        try:
            rgb, depth, sync = driver.capture(include_joint_state=True)
        finally:
            driver.close()
        if rgb.shape[:2] != (config["height"], config["width"]):
            raise ValueError("camera image dimensions differ from calibration")
        # A fixed-head calibration cannot silently follow a moving head.
        if "head_pitch_yaw" not in sync:
            raise ValueError("capture lacks synchronized head feedback")
        head = finite(sync["head_pitch_yaw"], (2,), "capture head state")
        tolerance = float(calibration.get("head_tolerance_rad", 0.01))
        if not np.isfinite(tolerance) or tolerance <= 0:
            raise ValueError("head_tolerance_rad must be positive")
        if np.max(np.abs(head - np.array(calibration["head_q2"]))) > tolerance:
            raise ValueError("head moved away from the calibrated pose; recalibrate or restore it")
    # Undistort RGB and aligned depth together; FoundationPose consumes pinhole K.
    distortion = np.asarray(config.get("distortion", [0]*5), dtype=float)
    k = np.asarray(config["intrinsics"], dtype=float)
    if not mock and undistort and np.any(distortion):
        maps = cv2.initUndistortRectifyMap(k, distortion, None, k,
                                          (config["width"], config["height"]), cv2.CV_32FC1)
        rgb = cv2.remap(rgb, *maps, interpolation=cv2.INTER_LINEAR)
        depth = cv2.remap(depth, *maps, interpolation=cv2.INTER_NEAREST)
    now = time.time()
    # The deployed WebSocket gateway stamps frames with the robot's wall clock
    # when forwarding them, rather than preserving the ROS sensor header. Use
    # the local receipt time for freshness while retaining gateway timestamps
    # in sensor_sync for RGB/depth synchronization and diagnostics.
    freshness_timestamp_ms = sync.get(
        "color_received_timestamp_ms", sync["color_timestamp_ms"])
    captured = float(freshness_timestamp_ms) / 1000.0
    max_age = float(config.get("max_frame_age_s", 0.75))
    if not np.isfinite(max_age) or max_age <= 0:
        raise ValueError("camera.max_frame_age_s must be positive and finite")
    # Native ROS timestamps must share the deployment machine's wall clock.
    # The bridge path instead supplies a local receipt timestamp above.
    if (not np.isfinite(captured) or captured <= 0 or now - captured > max_age
            or captured - now > 0.25):
        raise ValueError("captured RGB-D frame is stale or camera/host clocks are not synchronized")
    return {
        "frame_ref": uuid.uuid4().hex, "source": source, "timestamp_s": now,
        "capture_timestamp_s": captured, "sensor_sync": sync,
        "image": encode_image(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)),
        "depth": encode_image(depth), "depth_scale": 0.001,
        "width": int(rgb.shape[1]), "height": int(rgb.shape[0]),
        "intrinsics": config["intrinsics"], "distortion": ([0]*5 if undistort else distortion.tolist()),
        "camera_to_base": calibration["camera_to_base"],
        "head_q2": head.tolist(), "reference_frame": "base_Link",
        "camera_id": config.get("identity", "mock-camera"),
        "calibration_id": calibration["id"],
    }
