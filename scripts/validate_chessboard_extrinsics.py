#!/usr/bin/env python3
"""Build depth-free held-out points from a fixed chessboard and wrist-tip touches."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from tron2_deployment.calibration import board_points, corners, validate_extrinsics
from tron2_deployment.config import load_profile, write_json
from tron2_deployment.geometry import pose_matrix
from tron2_deployment.kinematics import RobotModel


def build_points(profile, intrinsic, touches, side, annotation_dir=None):
    """Return corresponding camera/base points and per-capture PnP diagnostics."""
    pattern = tuple(intrinsic["pattern"])
    square_m = float(intrinsic["square_m"])
    board = board_points(pattern, square_m)
    k = np.asarray(intrinsic["K"], dtype=float)
    dist = np.asarray(intrinsic["dist"], dtype=float)
    if not np.allclose(k, profile["camera"]["intrinsics"], atol=1e-9, rtol=0) or not np.allclose(
            dist, profile["camera"]["distortion"], atol=1e-9, rtol=0):
        raise ValueError("profile camera intrinsics differ from --intrinsics; apply the final intrinsics first")
    mount = profile["pregrasp"][side]["wrist_to_tcp_pose7"]
    if mount is None:
        raise ValueError(f"pregrasp.{side}.wrist_to_tcp_pose7 is not calibrated")
    mount = pose_matrix(mount)
    model = RobotModel(profile)
    camera_points, base_points, diagnostics = [], [], []
    seen_states, seen_corners = set(), set()
    captures = {}
    sources = set()
    for directory, row_text, col_text, state_name in touches:
        directory = Path(directory).resolve()
        state_path = Path(state_name).resolve()
        row, col = int(row_text), int(col_text)
        if not 0 <= row < pattern[1] or not 0 <= col < pattern[0]:
            raise ValueError(f"corner ({row}, {col}) outside {pattern[1]} rows x {pattern[0]} columns")
        if state_path in seen_states:
            raise ValueError(f"state file reused: {state_path}; save a separate reading for every touch")
        if (directory, row, col) in seen_corners:
            raise ValueError(f"corner ({row}, {col}) repeated in {directory}")
        seen_states.add(state_path)
        seen_corners.add((directory, row, col))
        if directory not in captures:
            frame = json.loads((directory / "frame.json").read_text())
            image = cv2.imread(str(directory / "color.png"), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"cannot read {directory / 'color.png'}")
            if image.shape[:2] != (intrinsic["height"], intrinsic["width"]):
                raise ValueError(f"{directory}: image size differs from the intrinsic calibration")
            if frame["camera_id"] != profile["camera"]["identity"]:
                raise ValueError(f"{directory}: camera identity differs from profile")
            if not np.allclose(frame["intrinsics"], k, atol=1e-9, rtol=0) or not np.allclose(
                    frame["distortion"], dist, atol=1e-9, rtol=0):
                raise ValueError(f"{directory}: frame intrinsics/distortion differ; capture a new raw image")
            if frame["source"] not in ("real", "mock"):
                raise ValueError(f"{directory}: unknown frame source")
            image_corners = np.asarray(corners(image, pattern), dtype=float).reshape(-1, 2)
            ok, rvec, tvec = cv2.solvePnP(board, image_corners, k, dist,
                                           flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                raise ValueError(f"{directory}: chessboard PnP failed")
            rotation = cv2.Rodrigues(rvec)[0]
            all_camera = board @ rotation.T + tvec.ravel()
            if not np.isfinite(all_camera).all() or np.any(all_camera[:, 2] <= 0):
                raise ValueError(f"{directory}: chessboard pose is invalid or behind camera")
            projected = cv2.projectPoints(board, rvec, tvec, k, dist)[0].reshape(-1, 2)
            reprojection = np.linalg.norm(projected - image_corners, axis=1)
            captures[directory] = (frame, all_camera, image, image_corners, {
                "capture": str(directory),
                "reprojection_rms_px": float(np.sqrt(np.mean(reprojection**2))),
                "reprojection_max_px": float(np.max(reprojection)),
            })
            diagnostics.append(captures[directory][4])
        frame, all_camera, _, _, _ = captures[directory]
        state = json.loads(state_path.read_text())
        if state["source"] != frame["source"]:
            raise ValueError(f"{state_path}: state and capture sources differ")
        sources.add(state["source"])
        if state["source"] == "real" and float(state["timestamp_s"]) < float(frame["capture_timestamp_s"]):
            raise ValueError(f"{state_path}: state was recorded before the chessboard image")
        if np.max(np.abs(np.asarray(state["head_q2"]) - frame["head_q2"])) > 0.005:
            raise ValueError(f"{state_path}: head moved since the chessboard capture")
        model.set_state(state["arm_q14"], state["head_q2"])
        tip = pose_matrix(model.wrist_poses()[side]) @ mount
        index = row * pattern[0] + col
        camera_points.append(all_camera[index])
        base_points.append(tip[:3, 3])
    if len(sources) != 1:
        raise ValueError("all captures and states must have one real or mock source")
    if annotation_dir is not None:
        annotation_dir = Path(annotation_dir)
        annotation_dir.mkdir(parents=True, exist_ok=True)
        for number, (directory, (_, _, image, image_corners, diagnostic)) in enumerate(captures.items(), 1):
            marked = image.copy()
            for capture_name, row_text, col_text, _ in touches:
                if Path(capture_name).resolve() != directory:
                    continue
                row, col = int(row_text), int(col_text)
                x, y = np.rint(image_corners[row * pattern[0] + col]).astype(int)
                cv2.circle(marked, (x, y), 7, (0, 0, 255), 2)
                cv2.putText(marked, f"({row},{col})", (x + 9, y - 9),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            path = annotation_dir / f"view-{number:02d}-corners.png"
            if not cv2.imwrite(str(path), marked):
                raise ValueError(f"could not write {path}")
            diagnostic["marked_image"] = str(path)
    return np.asarray(camera_points), np.asarray(base_points), diagnostics, next(iter(sources))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--intrinsics", required=True, type=Path)
    parser.add_argument("--handeye", required=True, type=Path)
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--touch", required=True, nargs=4, action="append",
                        metavar=("CAPTURE_DIR", "ROW", "COL", "STATE_JSON"),
                        help="repeat for each touch; ROW and COL are zero-based inner-corner indices")
    parser.add_argument("--max-error-m", required=True, type=float)
    parser.add_argument("--max-reprojection-px", type=float,
                        help="optional maximum PnP corner reprojection RMS in pixels")
    parser.add_argument("--output", required=True, type=Path, help="output held-out NPZ path")
    args = parser.parse_args(argv)
    try:
        if len(args.touch) < 3:
            raise ValueError("at least three different corners are required")
        profile = load_profile(args.profile)
        intrinsic = json.loads(args.intrinsics.read_text())
        handeye = json.loads(args.handeye.read_text())
        if handeye["mode"] != "eye_to_hand" or handeye["source"] not in ("real", "mock"):
            raise ValueError("--handeye must be an eye-to-hand solve")
        if handeye["camera_id"] != profile["camera"]["identity"]:
            raise ValueError("hand-eye camera identity differs from profile")
        if intrinsic.get("camera_id", profile["camera"]["identity"]) != profile["camera"]["identity"]:
            raise ValueError("intrinsic camera identity differs from profile")
        if intrinsic.get("source", handeye["source"]) != handeye["source"]:
            raise ValueError("intrinsic and hand-eye sources differ")
        if args.max_reprojection_px is not None and (
                not np.isfinite(args.max_reprojection_px) or args.max_reprojection_px <= 0):
            raise ValueError("--max-reprojection-px must be positive and finite")
        points_camera, points_base, pnp, source = build_points(
            profile, intrinsic, args.touch, args.side,
            annotation_dir=args.output.parent / f"{args.output.stem}-marked")
        if handeye["source"] != source:
            raise ValueError("hand-eye and observations have different sources")
        if any(np.max(np.abs(np.asarray(json.loads((Path(item[0]) / "frame.json").read_text())["head_q2"])
                             - handeye["head_q2"])) > 0.005 for item in args.touch):
            raise ValueError("validation camera head pose differs from hand-eye calibration")
        if args.max_reprojection_px is not None and any(
                item["reprojection_rms_px"] > args.max_reprojection_px for item in pnp):
            raise ValueError("chessboard PnP reprojection RMS exceeds --max-reprojection-px; "
                             "inspect the marked images and intrinsic calibration")
        report = validate_extrinsics(handeye["camera_to_base"], points_camera,
                                     points_base, args.max_error_m)
        transform = np.asarray(handeye["camera_to_base"])
        predicted = points_camera @ transform[:3, :3].T + transform[:3, 3]
        report.update(source=source, side=args.side, pnp=pnp,
                      points_camera=points_camera.tolist(), points_base=points_base.tolist(),
                      per_point_error_m=np.linalg.norm(predicted - points_base, axis=1).tolist(),
                      residual_base_m=(predicted - points_base).tolist(),
                      touches=[{"capture": str(Path(c).resolve()), "row": int(r), "col": int(k),
                                "state": str(Path(s).resolve())} for c, r, k, s in args.touch])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez(args.output, points_camera=points_camera, points_base=points_base)
        write_json(args.output.with_suffix(".report.json"), report)
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0 if report["passed"] else 1
    except (OSError, KeyError, TypeError, ValueError, cv2.error) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
