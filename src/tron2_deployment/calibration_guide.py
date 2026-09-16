"""Read-only camera observations for a CLI-launched calibration walkthrough."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from . import aruco, calibration, camera
from .config import profile_fingerprint, write_json


def _words(en, zh):
    return {"en": en, "zh": zh}


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class CalibrationGuide:
    """Collect fresh observations; never command joints or apply calibration."""

    def __init__(self, profile, output, *, stage="intrinsics", side=None,
                 pattern=(9, 6), square_m=.025, target=None, mock=False, profile_path=None):
        if stage not in ("intrinsics", "handeye"):
            raise ValueError("stage must be intrinsics or handeye")
        if stage == "handeye" and side not in ("left", "right"):
            raise ValueError("handeye requires side left or right")
        if target is not None and stage != "handeye":
            raise ValueError("ArUco targets are supported for hand-eye collection only")
        if len(pattern) != 2 or any(isinstance(n, bool) or not isinstance(n, int) for n in pattern):
            raise ValueError("pattern must contain two integer inner-corner counts")
        calibration.board_points(pattern, square_m)
        self.profile = deepcopy(profile)
        self.profile_path = Path(profile_path).expanduser().resolve() if profile_path else None
        self.profile_hash = _digest(self.profile_path) if self.profile_path else None
        self.profile_fingerprint = profile_fingerprint(self.profile)
        path = Path(output).expanduser()
        if path.is_symlink() or (path.exists() and (not path.is_dir() or any(path.iterdir()))):
            raise ValueError("use a new or empty output directory for each calibration session")
        path.mkdir(parents=True, exist_ok=True)
        self.output = path.resolve()
        self.stage, self.side, self.pattern = stage, side, tuple(pattern)
        self.square_m, self.mock, self.target = float(square_m), bool(mock), target
        self.target_hash = _digest(target["spec_file"]) if target else None
        self._views, self._samples, self._artifacts, self._regions = [], [], {}, set()
        self._preview_image, self._board_detected, self._result = None, None, None
        self._marker_ids = []
        self._result_hash = None
        self._notice = _words("Ready. Place the whole board in view, then preview.",
                              "准备就绪。让完整棋盘进入画面，然后预览。")
        self._notice["level"] = "info"
        self._next = self._collection_hint()
        write_json(self.output / "session.json", {
            "stage": stage, "side": side, "source": "mock" if mock else "real",
            "pattern": list(pattern), "square_m": self.square_m,
            "target": aruco.target_record(target) if target is not None else None,
            "profile_hash": self.profile_fingerprint,
            "profile_file_sha256": self.profile_hash,
        })

    def _check_profile(self):
        if self.profile_path and _digest(self.profile_path) != self.profile_hash:
            raise ValueError("profile changed during collection; start a new calibration session")
        if profile_fingerprint(self.profile) != self.profile_fingerprint:
            raise ValueError("profile or robot model changed during collection; start a new calibration session")
        if self.target is not None and _digest(self.target["spec_file"]) != self.target_hash:
            raise ValueError("target spec changed during collection; start a new calibration session")

    def _check_result(self):
        if self._result:
            path = Path(self._result["path"])
            if path.is_symlink() or not path.is_file() or _digest(path) != self._result_hash:
                raise ValueError("saved solution changed outside this session; preserve it and start a new session")

    def _rotation_spread(self):
        if not self._samples:
            return None
        rotations = Rotation.from_matrix(np.array([
            item["robot_gripper_to_base"] for item in self._samples])[:, :3, :3])
        # Match solve_samples' readiness check relative to the first sample.
        return float(np.rad2deg(np.max((rotations[0].inv() * rotations).magnitude())))

    def _collection_hint(self):
        if self.stage == "intrinsics":
            return _words("Move the board to another area and tilt it. Save at least 5 different views; 10–15 is preferable.",
                          "把棋盘移到另一区域并改变倾斜角度。至少保存 5 个不同视角，建议 10–15 个。")
        if self.target is not None:
            return _words("Keep the head fixed and the target rigidly attached to the selected wrist. Reposition through the robot's separate controls, let it settle, then save. Keep enough markers visible and rotate about different axes.",
                          "保持头部不动，目标刚性固定在所选手腕上。通过机器人独立控制界面调整姿态，等待静止后保存。保持足够的标记可见，并绕不同轴改变转角。")
        return _words("Keep the head fixed and the board rigidly attached to the selected wrist. Reposition through the robot's separate controls, let it settle, then save. Collect at least 5 poses with at least 15° rotation spread and rotations about different axes.",
                      "保持头部不动，棋盘刚性固定在所选手腕上。通过机器人独立控制界面调整姿态，等待静止后保存。至少采集 5 个姿态，转角变化至少达到 15°，并绕不同轴改变转角。")

    def status(self):
        count = len(self._views) if self.stage == "intrinsics" else len(self._samples)
        spread = self._rotation_spread()
        return deepcopy({
            "stage": self.stage, "side": self.side, "mock": self.mock,
            "saved_count": count,
            "can_solve": count >= 5 and (self.stage == "intrinsics" or spread >= 15),
            "preview_image": self._preview_image, "board_detected": self._board_detected,
            "notice": self._notice, "next": self._next, "result": self._result,
            "coverage_regions": sorted(self._regions), "rotation_spread_deg": spread,
            "pattern": list(self.pattern) if self.target is None else None,
            "square_m": self.square_m if self.target is None else None,
            "target": None if self.target is None else {
                "kind": self.target["kind"], "dictionary": self.target["dictionary"],
                "marker_length_m": self.target["marker_length_m"],
                "marker_separation_m": self.target["marker_separation_m"],
                "markers_x": self.target["markers_x"], "markers_y": self.target["markers_y"],
                "marker_ids": list(self.target["marker_ids"]),
                "min_visible_markers": self.target["min_visible_markers"]},
            "detected_marker_ids": list(self._marker_ids),
            "output": str(self.output), "hardware_commanded": False,
            "calibration_applied": False,
        })

    def _observe(self, image):
        if self.target is None:
            self._preview_image = camera.encode_image(image)
            self._board_detected = False
            found = calibration.corners(image, self.pattern)
            overlay = image.copy()
            cv2.drawChessboardCorners(overlay, self.pattern, found, True)
            self._preview_image = camera.encode_image(overlay)
            self._board_detected = True
            self._marker_ids = []
            return found.reshape(-1, 2)
        self._preview_image = camera.encode_image(image)
        self._board_detected = False
        found, order = aruco.detect_markers(image, self.target)
        self._marker_ids = list(order)
        overlay = image.copy()
        if order:
            cv2.aruco.drawDetectedMarkers(
                overlay, [found[value].reshape(1, 4, 2) for value in order],
                np.asarray(order, dtype=np.int32).reshape(-1, 1))
        self._preview_image = camera.encode_image(overlay)
        self._board_detected = bool(order)
        if not order:
            raise ValueError("no ArUco markers were detected")
        return np.vstack([found[value].reshape(4, 2) for value in order])

    def _failure(self, exc):
        message = str(exc)
        self._notice = _words(f"Not completed: {message}", f"未完成：{message}")
        self._notice["level"] = "error"
        if "corners" in message:
            self._next = _words("Show the entire board, reduce glare or blur, and check the inner-corner count. Preview again.",
                                "显示完整棋盘，减少反光或模糊，并核对内角点数量，然后重新预览。")
        elif "similar" in message:
            self._next = self._collection_hint()
        elif "collect at least" in message:
            self._next = self._collection_hint()
        elif any(word in message for word in ("moved", "settling", "head", "pose")):
            self._next = _words("Keep the head at its configured pose. Wait for the arm and board to stop, then save again.",
                                "保持头部在配置的姿态。等待机械臂和棋盘静止后，再次保存。")
        elif any(word in message for word in ("timestamp", "stale", "feedback", "synchronized", "bracketed")):
            self._next = _words("Check camera/robot feedback and clock synchronization, then preview and save again.",
                                "检查相机、机器人反馈和时钟同步，然后重新预览并保存。")
        else:
            self._next = _words("Check the message and camera/profile settings, then retry. Existing saved samples are retained.",
                                "根据提示检查相机和配置后重试，已保存的样本会保留。")
        return self.status()

    def _fresh_frame(self):
        if self.mock:
            frame, target = self._mock_observation(len(self._views) if self.stage == "intrinsics" else len(self._samples))
            return frame, target
        return camera.capture(self.profile, mock=False, undistort=False), None

    def preview(self):
        self._check_profile()
        self._preview_image, self._board_detected = None, None
        try:
            frame, _ = self._fresh_frame()
            self._observe(camera.decode_image(frame["image"], cv2.IMREAD_COLOR))
            self._notice = _words("Board detected. Save captures a fresh observation.",
                                  "已检测到棋盘。点击保存会重新采集当前画面。")
            self._notice["level"] = "success"
            self._next = self._collection_hint()
            return self.status()
        except (ValueError, OSError) as exc:
            return self._failure(exc)

    def _record_artifact(self, path):
        self._artifacts[str(path)] = _digest(path)

    def _remember_view(self, image, found):
        height, width = image.shape[:2]
        center = found.mean(axis=0) / [width, height]
        column, row = np.clip((center * 3).astype(int), 0, 2)
        self._regions.add(f"{('top', 'middle', 'bottom')[row]}-{('left', 'center', 'right')[column]}")

    def _check_duplicate(self, found, image, sample=None):
        if sample is not None:
            wrist = np.array(sample["robot_gripper_to_base"])
            for previous in self._samples:
                before = np.array(previous["robot_gripper_to_base"])
                angle = Rotation.from_matrix(before[:3, :3].T @ wrist[:3, :3]).magnitude()
                if np.linalg.norm(before[:3, 3] - wrist[:3, 3]) < .01 and angle < np.deg2rad(3):
                    raise ValueError("wrist pose is too similar to an already saved sample")
        else:
            diagonal = np.linalg.norm(image.shape[:2])
            for previous in self._views:
                points = np.asarray(previous["corners"])
                if min(np.sqrt(np.mean(np.sum((found - points)**2, axis=1))),
                       np.sqrt(np.mean(np.sum((found[::-1] - points)**2, axis=1)))) / diagonal < .01:
                    raise ValueError("board view is too similar to an already saved sample")

    def save(self):
        self._check_profile()
        self._preview_image, self._board_detected = None, None
        temporary = Path(tempfile.mkdtemp(prefix=".capture-", dir=self.output))
        try:
            self._check_result()
            if self.stage == "intrinsics":
                frame, _ = self._fresh_frame()
                image = camera.decode_image(frame["image"], cv2.IMREAD_COLOR)
                found = self._observe(image)
                self._check_duplicate(found, image)
                if self._views and image.shape[:2] != tuple(self._views[0]["shape"]):
                    raise ValueError("calibration images must use one camera resolution")
                if not cv2.imwrite(str(temporary / "color.png"), image):
                    raise OSError("cannot save color image")
                if not cv2.imwrite(str(temporary / "depth.png"), camera.decode_image(frame["depth"])):
                    raise OSError("cannot save depth image")
                write_json(temporary / "frame.json", frame)
                destination = self.output / f"view-{len(self._views) + 1:03d}"
                if destination.exists():
                    raise ValueError("sample output already exists; start a new session")
                temporary.rename(destination)
                for path in destination.iterdir():
                    self._record_artifact(path)
                self._views.append({"path": str(destination / "color.png"),
                                    "corners": found.tolist(), "shape": list(image.shape[:2])})
            else:
                if self.mock:
                    frame, target = self._fresh_frame()
                    image = camera.decode_image(frame["image"], cv2.IMREAD_COLOR)
                    found = self._observe(image)
                    sample = self._mock_sample(frame, target)
                    image_path = temporary / f"{time.time_ns()}.png"
                    if not cv2.imwrite(str(image_path), image):
                        raise OSError("cannot save sample image")
                    sample["image"] = image_path.name
                    sample_path = write_json(image_path.with_suffix(".json"), sample)
                else:
                    # Preserve all freshness, synchronization and stationarity gates.
                    sample_path = calibration.record_sample(
                        self.profile, self.side, temporary, mock=False,
                        pattern=self.pattern, square_m=self.square_m, target=self.target,
                        on_frame=lambda frame: self._observe(camera.decode_image(frame["image"], cv2.IMREAD_COLOR)))
                    sample = json.loads(Path(sample_path).read_text())
                    image = cv2.imread(str(Path(sample_path).parent / sample["image"]))
                    if image is None:
                        raise ValueError("cannot read saved calibration image")
                    found = self._observe(image)
                self._check_duplicate(found, image, sample)
                if self._samples and np.max(np.abs(np.array(sample["head_q2"]) - self._samples[0]["head_q2"])) > .005:
                    raise ValueError("head pose differs from earlier samples; restore it or start a new session")
                destination = self.output / "samples"
                destination.mkdir(exist_ok=True)
                for path in temporary.iterdir():
                    final = destination / path.name
                    if final.exists():
                        raise ValueError("sample output already exists; start a new session")
                for path in temporary.iterdir():
                    final = destination / path.name
                    path.rename(final)
                    self._record_artifact(final)
                self._samples.append(sample)
            self._remember_view(image, found)
            if self._result:
                # The old solution no longer represents this session's observations.
                Path(self._result["path"]).unlink(missing_ok=True)
            self._result = None
            self._result_hash = None
            count = len(self._views) if self.stage == "intrinsics" else len(self._samples)
            self._notice = _words(f"Saved sample {count}. The image shows the saved observation.",
                                  f"已保存第 {count} 个样本。画面显示本次保存的观测。")
            self._notice["level"] = "success"
            self._next = self._collection_hint()
            return self.status()
        except (ValueError, OSError) as exc:
            return self._failure(exc)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)

    def solve(self):
        self._check_profile()
        try:
            self._check_result()
            if not self.status()["can_solve"]:
                raise ValueError("collect at least 5 different views; hand-eye also needs at least 15 degrees of wrist rotation spread")
            for path, digest in self._artifacts.items():
                if _digest(path) != digest:
                    raise ValueError("a saved sample changed outside this session; start a new session")
            if self.stage == "intrinsics":
                solution = calibration.intrinsic_fit([v["path"] for v in self._views], self.pattern, self.square_m)
                solution.update(source="mock" if self.mock else "real", profile_hash=self.profile_fingerprint,
                                camera_id=self.profile["camera"].get("identity", "unspecified"))
                path = self.output / "intrinsics.json"
                label = _words("Reprojection RMS (fit only)", "重投影 RMS（仅表示拟合误差）")
                metric = f"{solution['rms_px']:.3f} px"
                next_step = _words("Intrinsic parameters saved. Review this fit, apply intrinsics to a new profile with the CLI, then start hand-eye collection. This number is not an independent accuracy check.",
                                   "已保存内参。检查拟合结果，通过 CLI 将内参写入新配置，再开始手眼采集。该数值不代表独立精度验证。")
            else:
                solution = calibration.solve_samples(deepcopy(self._samples))
                path = self.output / f"handeye-{self.side}.json"
                label, metric = _words("Samples used", "使用样本数"), str(len(self._samples))
                next_step = _words("Hand-eye transform saved. Check it with independent measured points before applying calibration through the CLI.",
                                   "已保存手眼变换。先使用独立实测点验证，再通过 CLI 应用标定。")
            if self.mock:
                next_step = _words("Synthetic walkthrough complete. Start a new session without --mock to collect real calibration.",
                                   "合成数据演示完成。请新建不带 --mock 的会话，采集真实标定。")
            if path.exists() and not self._result:
                raise ValueError("solution output already exists; preserve it and start a new session")
            write_json(path, solution)
            self._result_hash = _digest(path)
            self._result = {"path": str(path), "status": "unverified",
                            "metric_label": label, "metric_value": metric, "next": next_step}
            self._notice = _words("Calibration saved; not applied.", "标定已保存，尚未应用。")
            self._notice["level"] = "success"
            self._next = next_step
            return self.status()
        except (ValueError, OSError) as exc:
            return self._failure(exc)

    def _mock_observation(self, index):
        """A deterministic projected board; no robot/camera adapter is created."""
        if self.target is not None:
            return self._mock_target_observation(index)
        config = self.profile["camera"]
        width, height = config["width"], config["height"]
        k = np.array([[width * .9, 0, width / 2], [0, width * .9, height / 2], [0, 0, 1.]])
        angles = [(-22, -16, 0), (18, -15, 7), (-12, 20, -8), (20, 15, 2),
                  (-18, 8, 10), (10, -22, -12), (-8, 24, 3), (25, -7, 12), (3, 14, -16)]
        locations = [(.28, .28), (.72, .28), (.5, .5), (.28, .72), (.72, .72),
                     (.5, .28), (.28, .5), (.72, .5), (.5, .72)]
        tilt = np.array(angles[index % len(angles)], dtype=float)
        cycle = (index // len(angles)) % 3
        tilt[2] += cycle * 14
        rotation = Rotation.from_euler("xyz", tilt, degrees=True).as_matrix()
        cols, rows = self.pattern
        z = max(.5, (cols + 1) * self.square_m * k[0, 0] / (width * .38),
                (rows + 1) * self.square_m * k[1, 1] / (height * .35)) * (1 + .04 * (index % 3)) * (1 + .14 * cycle)
        u, v = locations[index % len(locations)]
        center = np.array([(u * width - k[0, 2]) * z / k[0, 0],
                           (v * height - k[1, 2]) * z / k[1, 1], z])
        target = np.eye(4)
        target[:3, :3] = rotation
        target[:3, 3] = center - rotation @ np.array([(cols - 1) * self.square_m / 2,
                                                       (rows - 1) * self.square_m / 2, 0])
        image = np.full((height * 2, width * 2, 3), 210, np.uint8)
        for row in range(rows + 1):
            for col in range(cols + 1):
                square = np.array([[col - 1, row - 1, 0], [col, row - 1, 0],
                                   [col, row, 0], [col - 1, row, 0]], float) * self.square_m
                pixels, _ = cv2.projectPoints(square, Rotation.from_matrix(rotation).as_rotvec(),
                                              target[:3, 3], k, np.zeros(5))
                color = 250 if (row + col) % 2 else 15
                cv2.fillConvexPoly(image, np.rint(pixels.reshape(-1, 2) * 2).astype(np.int32), (color,) * 3)
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
        now = time.time()
        frame = {"image": camera.encode_image(image),
                 "depth": camera.encode_image(np.full((height, width), int(z * 1000), np.uint16)),
                 "source": "mock", "camera_id": "synthetic-calibration-guide",
                 "timestamp_s": now, "capture_timestamp_s": now,
                 "width": width, "height": height, "intrinsics": k.tolist(), "distortion": [0] * 5,
                 "head_q2": list(self.profile["calibration"]["head_q2"]),
                 "sensor_sync": {"source": "synthetic"}, "depth_scale": .001}
        return frame, target

    def _mock_target_observation(self, index):
        """Deterministic perspective view of the configured ArUco target."""
        config = self.profile["camera"]
        width, height = config["width"], config["height"]
        k = np.array([[width * .9, 0, width / 2], [0, width * .9, height / 2], [0, 0, 1.]])
        angles = [(-22, -16, 0), (18, -15, 7), (-12, 20, -8), (20, 15, 2),
                  (-18, 8, 10), (10, -22, -12), (-8, 24, 3), (25, -7, 12), (3, 14, -16)]
        locations = [(.28, .28), (.72, .28), (.5, .5), (.28, .72), (.72, .72),
                     (.5, .28), (.28, .5), (.72, .5), (.5, .72)]
        tilt = np.array(angles[index % len(angles)], dtype=float)
        cycle = (index // len(angles)) % 3
        tilt[2] += cycle * 14
        rotation = Rotation.from_euler("xyz", tilt, degrees=True).as_matrix()
        extent = aruco.layout_extent(self.target)
        z = max(.35, extent[0] * k[0, 0] / (width * .38),
                extent[1] * k[1, 1] / (height * .35)) * (1 + .04 * (index % 3)) * (1 + .14 * cycle)
        u, v = locations[index % len(locations)]
        center = np.array([(u * width - k[0, 2]) * z / k[0, 0],
                           (v * height - k[1, 2]) * z / k[1, 1], z])
        target = np.eye(4)
        target[:3, :3] = rotation
        target[:3, 3] = center - rotation @ aruco.layout_center(self.target)
        image = aruco.render_target(self.target, (height, width), rotation, target[:3, 3], k)
        now = time.time()
        frame = {"image": camera.encode_image(image),
                 "depth": camera.encode_image(np.full((height, width), int(z * 1000), np.uint16)),
                 "source": "mock", "camera_id": "synthetic-calibration-guide",
                 "timestamp_s": now, "capture_timestamp_s": now,
                 "width": width, "height": height, "intrinsics": k.tolist(), "distortion": [0] * 5,
                 "head_q2": list(self.profile["calibration"]["head_q2"]),
                 "sensor_sync": {"source": "synthetic"}, "depth_scale": .001}
        return frame, target

    def _mock_sample(self, frame, target):
        camera_to_base = np.eye(4)
        camera_to_base[:3, :3] = Rotation.from_euler("xyz", [.2, -.3, .1]).as_matrix()
        camera_to_base[:3, 3] = [.2, -.1, .5]
        mount = np.eye(4)
        mount[:3, 3] = [.02, .03, .08]
        wrist = camera_to_base @ target @ np.linalg.inv(mount)
        sample = {"side": self.side, "source": "mock", "timestamp_s": frame["timestamp_s"],
                  "robot_gripper_to_base": wrist.tolist(), "target_to_camera": target.tolist(),
                  "head_q2": frame["head_q2"],
                  "frame": {key: value for key, value in frame.items() if key not in ("image", "depth")}}
        if self.target is None:
            sample.update(pattern=list(self.pattern), square_m=self.square_m)
        else:
            sample["target"] = aruco.target_record(self.target)
        return sample
