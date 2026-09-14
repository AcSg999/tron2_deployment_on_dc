"""Hand-eye calibration with AX=XB solvers, in both mounting geometries.

TRON2 publishes three cameras and they need different formulations:

``eye_in_hand``
    Camera rides the gripper (``cam_left_wrist`` / ``cam_right_wrist``) while
    the target is fixed in the world.  Solves camera -> gripper.

``eye_to_hand``
    Camera is fixed relative to the base (``cam_high``) and the target rides
    the gripper.  Solves camera -> base.  Feeding base->gripper poses where
    the eye-in-hand form takes gripper->base turns the same AX=XB solver into
    this case, so both share one implementation.

For ``cam_high`` the head joints move the camera, so a solve is only valid
for the head pose it was captured at; record it alongside (see
``tron2_deployment/calibration.py``).
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

DEFAULT_OUTPUT = Path.cwd() / "output" / "handeye_calib.json"
METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}
# mode -> the frame the solved camera transform is expressed against
MODES = {"eye_in_hand": "gripper", "eye_to_hand": "base"}
# Convergence bound for the AX=XB residual RMS. Entries mix metres and
# radians, so 0.05 is roughly 5 cm / 2.9 deg: loose enough for a real capture
# (rendering, corner detection and FK all contribute), tight enough that a
# wrong solution still fails.
RESIDUAL_TOL = 0.05


def _validate_transforms(values: np.ndarray, name: str) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    if values.ndim != 3 or values.shape[1:] != (4, 4) or len(values) < 3:
        raise ValueError(f"{name} must have shape (N>=3, 4, 4)")
    if not np.isfinite(values).all() or not np.allclose(
            values[:, 3, :], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f"{name} contains invalid homogeneous transforms")
    for transform in values:
        rotation = transform[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise ValueError(f"{name} contains a non-rigid rotation")
    return values


def _params_to_transform(params: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(params[:3]).as_matrix()
    transform[:3, 3] = params[3:]
    return transform


def _scipy_ax_xb(gripper: np.ndarray, target: np.ndarray, *,
                 residual_tol: float = RESIDUAL_TOL) -> tuple[np.ndarray, float]:
    """Solve pairwise A X = X B when OpenCV omits calibrateHandEye.

    OpenCV 5's Python bindings no longer expose ``calibrateHandEye``, so this
    is the live path, not a rare fallback.  Convergence is judged on the RMS
    per residual entry: the total norm grows with the number of pose pairs
    (N(N-1)/2 of them) and with sensor noise, so an absolute bound on it
    rejects every real capture while accepting only noiseless fixtures.
    """
    pairs = []
    for left in range(len(gripper) - 1):
        for right in range(left + 1, len(gripper)):
            a = np.linalg.inv(gripper[right]) @ gripper[left]
            b = target[right] @ np.linalg.inv(target[left])
            pairs.append((a, b))

    def residual(params: np.ndarray) -> np.ndarray:
        x = _params_to_transform(params)
        values = []
        for a, b in pairs:
            error = np.linalg.inv(a @ x) @ (x @ b)
            values.extend(error[:3, 3])
            values.extend(Rotation.from_matrix(error[:3, :3]).as_rotvec())
        return np.asarray(values)

    solution = least_squares(
        residual, np.zeros(6), method="trf", max_nfev=2000,
        ftol=1e-12, xtol=1e-12, gtol=1e-12)
    rms = float(np.linalg.norm(solution.fun)
                / max(1.0, np.sqrt(solution.fun.size)))
    if not solution.success or rms > residual_tol:
        raise RuntimeError(
            f"SciPy hand-eye solver did not converge: {solution.message} "
            f"(residual RMS {rms:.4g} > {residual_tol:.4g}; residuals mix "
            f"metres and radians)")
    return _params_to_transform(solution.x), rms


def calibrate_handeye(robot_gripper_to_base: np.ndarray,
                      target_to_camera: np.ndarray, *,
                      method: str = "park",
                      mode: str = "eye_in_hand",
                      residual_tol: float = RESIDUAL_TOL) -> dict:
    """Estimate the fixed camera transform from synchronized poses.

    ``mode="eye_in_hand"`` returns camera -> gripper; ``mode="eye_to_hand"``
    returns camera -> base.  Both take the same inputs: the gripper pose in
    the base frame from FK, and the target pose in the camera frame from
    solvePnP.
    """
    if mode not in MODES:
        raise ValueError(f"unknown hand-eye mode: {mode}")
    gripper = _validate_transforms(
        robot_gripper_to_base, "robot_gripper_to_base")
    target = _validate_transforms(target_to_camera, "target_to_camera")
    if gripper.shape != target.shape:
        raise ValueError("robot and camera transform counts must match")
    if method not in METHODS:
        raise ValueError(f"unknown hand-eye method: {method}")
    reference = MODES[mode]
    if mode == "eye_to_hand":
        # A fixed camera watching a gripper-mounted target is the eye-in-hand
        # problem with the robot motion inverted; the invariant recovered
        # below then becomes target-to-gripper.
        gripper = np.asarray([np.linalg.inv(pose) for pose in gripper])
    if hasattr(cv2, "calibrateHandEye"):
        rotation, translation = cv2.calibrateHandEye(
            [pose[:3, :3] for pose in gripper],
            [pose[:3, 3] for pose in gripper],
            [pose[:3, :3] for pose in target],
            [pose[:3, 3] for pose in target],
            method=METHODS[method],
        )
        camera_to_reference = np.eye(4)
        camera_to_reference[:3, :3] = rotation
        camera_to_reference[:3, 3] = np.asarray(translation).reshape(3)
        solver = f"opencv_{method}"
        solver_rms = None
    else:
        camera_to_reference, solver_rms = _scipy_ax_xb(
            gripper, target, residual_tol=residual_tol)
        solver = "scipy_pairwise_ax_xb"

    # The recovered invariant: target-to-base for eye-in-hand, target-to-
    # gripper for eye-to-hand.  Its spread across samples is the residual.
    invariant = np.asarray([
        gripper_pose @ camera_to_reference @ target_pose
        for gripper_pose, target_pose in zip(gripper, target)
    ])
    translations = invariant[:, :3, 3]
    translation_rms = float(np.sqrt(np.mean(
        (translations - translations.mean(axis=0)) ** 2)))
    rotations = invariant[:, :3, :3]
    mean_rotation_raw = rotations.mean(axis=0)
    u, _, vt = np.linalg.svd(mean_rotation_raw)
    mean_rotation = u @ vt
    angles = []
    for value in rotations:
        relative = mean_rotation.T @ value
        angles.append(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
    reference_to_camera = np.linalg.inv(camera_to_reference)
    result = {
        "schema_version": 2,
        "mode": mode,
        "reference_frame": reference,
        "method": method,
        "solver": solver,
        "samples": len(gripper),
        "camera_to_reference": camera_to_reference.tolist(),
        "reference_to_camera": reference_to_camera.tolist(),
        "target_translation_rms_m": translation_rms,
        "target_rotation_rms_deg": float(np.degrees(
            np.sqrt(np.mean(np.square(angles))))),
        "solver_residual_rms": solver_rms,
        "created_unix_s": time.time(),
    }
    # Frame-named aliases so consumers never have to remember which mode
    # produced a file.
    result[f"camera_to_{reference}"] = result["camera_to_reference"]
    result[f"{reference}_to_camera"] = result["reference_to_camera"]
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("samples", type=Path,
                        help="NPZ with robot_gripper_to_base/target_to_camera")
    parser.add_argument("--method", choices=tuple(METHODS), default="park")
    parser.add_argument("--mode", choices=tuple(MODES), default="eye_in_hand",
                        help="eye_in_hand for wrist cameras, eye_to_hand for "
                             "the head/scene camera (cam_high)")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    with np.load(args.samples) as data:
        result = calibrate_handeye(
            data["robot_gripper_to_base"], data["target_to_camera"],
            mode=args.mode,
            method=args.method)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
