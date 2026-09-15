"""Offline calibration diagnostics; never fit new intrinsics or accept a profile."""
from __future__ import annotations

import hashlib
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import board_points, corners, validate_extrinsics
from .config import finite, rigid


def _pattern(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or any(isinstance(n, bool) or not isinstance(n, int) or n < 2 for n in value)):
        raise ValueError("pattern must contain two integer inner-corner dimensions >= 2")
    return tuple(value)


def _square(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
        raise ValueError("square_m must be positive and finite")
    return float(value)


def _optional_finite(value, name):
    if value is None:
        return None
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def intrinsic_diagnostics(intrinsic: dict, image_paths: list[Path]) -> dict:
    """Reproject original board images using the supplied, fixed K/distortion.

    A separate solvePnP estimates each view's board pose. This diagnoses fit
    consistency; it cannot establish independent metric accuracy. Invalid image
    inputs are retained as rejected entries instead of silently disappearing.
    """
    k = finite(intrinsic["K"], (3, 3), "intrinsic K")
    if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
        raise ValueError("intrinsic K has invalid focal lengths or bottom row")
    dist = np.asarray(intrinsic["dist"], dtype=float)
    if dist.shape not in ((4,), (5,), (8,), (12,), (14,)) or not np.isfinite(dist).all():
        raise ValueError("intrinsic distortion needs 4, 5, 8, 12, or 14 finite coefficients")
    width, height = intrinsic["width"], intrinsic["height"]
    if any(isinstance(n, bool) or not isinstance(n, int) or n <= 0 for n in (width, height)):
        raise ValueError("intrinsic image dimensions must be positive integers")
    pattern, square = _pattern(intrinsic["pattern"]), _square(intrinsic["square_m"])
    points = board_points(pattern, square)
    fit_images = {str(Path(p).expanduser().resolve()) for p in intrinsic.get("images", [])}
    views, observed_all, errors_all = [], [], []
    source = str(intrinsic.get("source", "unspecified"))
    for image_path in image_paths:
        path = Path(image_path).expanduser().resolve()
        view = {"image_path": str(path), "source": source,
                "included_in_fit": str(path) in fit_images, "status": "rejected"}
        views.append(view)
        try:
            data = path.read_bytes()
            view["image_sha256"] = hashlib.sha256(data).hexdigest()
            image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("cannot decode calibration image")
            if image.shape[:2] != (height, width):
                raise ValueError(f"image resolution {image.shape[1]}x{image.shape[0]} differs from calibration {width}x{height}")
            observed = np.asarray(corners(image, pattern), dtype=float).reshape(-1, 2)
            if (observed.shape != (len(points), 2) or not np.isfinite(observed).all()
                    or np.any(observed < 0) or np.any(observed >= [width, height])):
                raise ValueError("detected corners must be finite and inside the image")
            ok, rv, tv = cv2.solvePnP(points, observed, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or not np.isfinite(rv).all() or not np.isfinite(tv).all():
                raise ValueError("board pose solve failed or returned non-finite values")
            rotation = cv2.Rodrigues(rv)[0]
            if np.any((points @ rotation.T + tv.ravel())[:, 2] <= 0):
                raise ValueError("board pose places corners behind the camera")
            predicted = cv2.projectPoints(points, rv, tv, k, dist)[0].reshape(-1, 2)
            if not np.isfinite(predicted).all():
                raise ValueError("corner projection returned non-finite values")
            residual = predicted - observed
            errors = np.linalg.norm(residual, axis=1)
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = rotation, tv.ravel()
            view.update(status="accepted", observed_xy_px=observed.tolist(),
                        predicted_xy_px=predicted.tolist(), residual_xy_px=residual.tolist(),
                        error_px=errors.tolist(), rms_px=float(np.sqrt(np.mean(errors**2))),
                        max_error_px=float(np.max(errors)), rotation_vector=rv.ravel().tolist(),
                        translation_m=tv.ravel().tolist(), target_to_camera=transform.tolist())
            observed_all.append(observed)
            errors_all.append(errors)
        except (OSError, ValueError, cv2.error) as exc:
            view["reason"] = str(exc)

    columns, rows = 8, 6
    occupied, hull_fraction = 0, 0.0
    if observed_all:
        combined = np.vstack(observed_all)
        cells = np.floor(combined / [width, height] * [columns, rows]).astype(int)
        occupied = len(np.unique(cells, axis=0))
        hull = cv2.convexHull(combined.astype(np.float32))
        hull_fraction = float(cv2.contourArea(hull) / (width * height))
    errors = np.concatenate(errors_all) if errors_all else np.array([])
    notes = [
        "Each board pose is recomputed with solvePnP using the supplied fixed K and distortion; intrinsics are not refitted.",
        "Use original, distorted camera images. Previously undistorted images are incompatible with this comparison.",
        "Reprojection residuals and coverage diagnose consistency; they do not prove independent metric calibration accuracy.",
        "Residual vectors are predicted minus observed pixels. No acceptance or execution settings are changed.",
    ]
    if source == "unspecified":
        notes.append("The intrinsic file does not identify a real or mock source; image provenance cannot be inferred.")
    if not observed_all:
        notes.append("No usable board images: reprojection metrics are unavailable.")
    return {
        "kind": "intrinsic", "source": source,
        "calibration": {"K": k.tolist(), "dist": dist.tolist(), "width": width, "height": height,
                        "pattern": list(pattern), "square_m": square,
                        "fit_rms_px": _optional_finite(intrinsic.get("rms_px"), "intrinsic fit RMS")},
        "accepted_count": len(observed_all), "rejected_count": len(views) - len(observed_all),
        "rms_px": float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
        "max_error_px": float(np.max(errors)) if len(errors) else None,
        "coverage": {"convex_hull_fraction": hull_fraction, "grid_columns": columns,
                     "grid_rows": rows, "occupied_cells": occupied, "total_cells": columns * rows,
                     "occupied_fraction": occupied / (columns * rows)},
        "views": views, "notes": notes,
    }


def _camera_transform(solution):
    transform = rigid(solution.get("camera_to_base", solution.get("camera_to_reference")), "camera_to_base")
    if "camera_to_reference" in solution and not np.allclose(
            rigid(solution["camera_to_reference"], "camera_to_reference"), transform, atol=1e-7):
        raise ValueError("hand-eye camera transform aliases disagree")
    for key in ("base_to_camera", "reference_to_camera"):
        if key in solution and not np.allclose(rigid(solution[key], key), np.linalg.inv(transform), atol=1e-7):
            raise ValueError("hand-eye camera transform inverse aliases disagree")
    return transform


def handeye_diagnostics(solution: dict, samples: list[dict], sample_labels: list[str] | None = None) -> dict:
    """Measure the fixed board-to-wrist transform's spread without refitting X."""
    if solution.get("mode") != "eye_to_hand" or solution.get("reference_frame", "base") != "base":
        raise ValueError("hand-eye diagnostics require an eye_to_hand camera-to-base solution")
    if 0 < len(samples) < 3:
        raise ValueError("hand-eye diagnostics require at least three samples")
    if sample_labels is not None and len(sample_labels) != len(samples):
        raise ValueError("sample label count must match samples")
    transform = _camera_transform(solution)
    source, side, camera_id = solution.get("source"), solution.get("side"), solution.get("camera_id")
    if source not in ("mock", "real"):
        raise ValueError("hand-eye solution source must be mock or real")
    if side not in ("left", "right"):
        raise ValueError("hand-eye solution side must be left or right")
    if not isinstance(camera_id, str) or not camera_id:
        raise ValueError("hand-eye solution camera_id is required")
    head = finite(solution.get("head_q2"), (2,), "solution head_q2")
    if not samples:
        return {
            "kind": "handeye", "source": source, "mode": "eye_to_hand", "camera_id": camera_id,
            "side": side, "head_q2": head.tolist(), "sample_count": 0,
            "solution_sample_count": solution.get("samples"), "camera_to_base": transform.tolist(),
            "mean_target_to_wrist": None, "translation_rms_mm": None, "translation_max_mm": None,
            "rotation_rms_deg": None, "rotation_max_deg": None, "wrist_spread": None, "samples": [],
            "notes": ["No hand-eye sample files were supplied: only the saved camera transform can be displayed; consistency and wrist spread are unavailable.",
                      "Displaying a transform does not validate its accuracy. No acceptance or execution settings are changed."],
        }
    pattern, square = _pattern(samples[0]["pattern"]), _square(samples[0]["square_m"])
    if (("pattern" in solution and _pattern(solution["pattern"]) != pattern)
            or ("square_m" in solution and not np.isclose(_square(solution["square_m"]), square, rtol=0, atol=1e-12))):
        raise ValueError("sample board pattern or square_m differs from the hand-eye solution")
    # Older samples contain only camera_id. Where imaging metadata is present,
    # reject mixed intrinsics/resolutions rather than blending incompatible PnP poses.
    first_frame = samples[0].get("frame", {})
    for key in ("intrinsics", "distortion", "width", "height"):
        if any((key in sample.get("frame", {})) != (key in first_frame) for sample in samples):
            raise ValueError(f"hand-eye sample frame {key} provenance is incomplete")
        if key in first_frame:
            baseline = np.asarray(first_frame[key], dtype=float)
            if not np.isfinite(baseline).all():
                raise ValueError(f"hand-eye sample frame {key} must be finite")
            for sample in samples[1:]:
                observed = np.asarray(sample["frame"][key], dtype=float)
                if observed.shape != baseline.shape or not np.isfinite(observed).all() or not np.allclose(observed, baseline, rtol=0, atol=1e-10):
                    raise ValueError(f"hand-eye sample frame {key} must be consistent")
    wrists, boards, invariants = [], [], []
    for i, sample in enumerate(samples):
        for key, expected in (("source", source), ("side", side)):
            if sample.get(key) != expected:
                raise ValueError(f"sample {i + 1} {key} differs from the hand-eye solution")
        frame = sample.get("frame", {})
        if frame.get("camera_id") != camera_id:
            raise ValueError(f"sample {i + 1} camera_id differs from the hand-eye solution")
        if "source" in frame and frame["source"] != source:
            raise ValueError(f"sample {i + 1} frame source differs from the hand-eye solution")
        measured_head = finite(sample.get("head_q2"), (2,), f"sample {i + 1} head_q2")
        if np.max(np.abs(measured_head - head)) > 0.005:
            raise ValueError("hand-eye samples must match the solution's stationary head pose")
        if "head_q2" in frame and np.max(np.abs(finite(frame["head_q2"], (2,), "frame head_q2") - head)) > 0.005:
            raise ValueError("hand-eye frame head pose differs from the solution")
        if _pattern(sample["pattern"]) != pattern or not np.isclose(
                _square(sample["square_m"]), square, rtol=0, atol=1e-12):
            raise ValueError("hand-eye sample board pattern and square_m must be consistent")
        wrist = rigid(sample["robot_gripper_to_base"], f"sample {i + 1} wrist transform")
        board = rigid(sample["target_to_camera"], f"sample {i + 1} board transform")
        wrists.append(wrist)
        boards.append(board)
        invariants.append(np.linalg.inv(wrist) @ transform @ board)
    wrists, invariants = np.asarray(wrists), np.asarray(invariants)
    mean = np.eye(4)
    mean[:3, 3] = invariants[:, :3, 3].mean(axis=0)
    u, _, vt = np.linalg.svd(invariants[:, :3, :3].mean(axis=0))
    # Project the average onto SO(3), including the determinant correction.
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    mean[:3, :3] = u @ correction @ vt
    translation_residuals = (invariants[:, :3, 3] - mean[:3, 3]) * 1000
    translation_errors = np.linalg.norm(translation_residuals, axis=1)
    rotation_errors = np.degrees(Rotation.from_matrix(mean[:3, :3].T @ invariants[:, :3, :3]).magnitude())
    wrist_rotations = Rotation.from_matrix(wrists[:, :3, :3])
    max_translation, max_rotation = 0.0, 0.0
    for i in range(len(wrists) - 1):
        max_translation = max(max_translation, float(np.max(np.linalg.norm(wrists[i + 1:, :3, 3] - wrists[i, :3, 3], axis=1))) * 1000)
        max_rotation = max(max_rotation, float(np.degrees(np.max((wrist_rotations[i].inv() * wrist_rotations[i + 1:]).magnitude()))))
    entries = []
    for i in range(len(samples)):
        entries.append({
            "label": str(sample_labels[i]) if sample_labels is not None else f"sample_{i + 1:03d}",
            "robot_gripper_to_base": wrists[i].tolist(), "target_to_camera": boards[i].tolist(),
            "target_to_wrist": invariants[i].tolist(), "translation_residual_mm": translation_residuals[i].tolist(),
            "translation_error_mm": float(translation_errors[i]), "rotation_error_deg": float(rotation_errors[i]),
        })
    notes = [
        "The board-to-wrist invariant is inverse(T_base_wrist) @ T_base_camera @ T_camera_board; the camera transform is not refitted.",
        "Translation errors are Euclidean distances from the mean translation; rotation errors are geodesic angles from the proper mean rotation.",
        "These sample consistency residuals are not an independent accuracy test; use independently measured held-out points.",
        "Rotation spread is a diagnostic only: 15 degrees does not by itself establish sufficient multi-axis excitation.",
        "No acceptance or execution settings are changed.",
    ]
    if max_rotation < 15:
        notes.append("Wrist rotation spread is below 15 degrees; collect more varied wrist orientations.")
    if solution.get("samples") is not None and solution["samples"] != len(samples):
        notes.append("The supplied diagnostic sample count differs from the solution's original sample count.")
    return {
        "kind": "handeye", "source": source, "mode": "eye_to_hand", "camera_id": camera_id,
        "side": side, "head_q2": head.tolist(), "sample_count": len(samples),
        "solution_sample_count": solution.get("samples"), "camera_to_base": transform.tolist(),
        "mean_target_to_wrist": mean.tolist(),
        "translation_rms_mm": float(np.sqrt(np.mean(translation_errors**2))),
        "translation_max_mm": float(np.max(translation_errors)),
        "rotation_rms_deg": float(np.sqrt(np.mean(rotation_errors**2))),
        "rotation_max_deg": float(np.max(rotation_errors)),
        "wrist_spread": {"translation_range_mm": (np.ptp(wrists[:, :3, 3], axis=0) * 1000).tolist(),
                         "max_pairwise_translation_mm": max_translation, "max_pairwise_rotation_deg": max_rotation,
                         "rotation_spread_at_least_15_deg": bool(max_rotation >= 15)},
        "samples": entries, "notes": notes,
    }


def heldout_diagnostics(camera_to_base, points_camera, points_base, tolerance_m) -> dict:
    """Expand the existing held-out metric check into plottable point residuals."""
    if isinstance(tolerance_m, bool):
        raise ValueError("validation tolerance must be positive")
    result = validate_extrinsics(camera_to_base, points_camera, points_base, tolerance_m)
    transform = rigid(camera_to_base, "camera_to_base")
    camera, reference = np.asarray(points_camera, float), np.asarray(points_base, float)
    if np.linalg.matrix_rank(reference - reference.mean(axis=0)) < 2:
        raise ValueError("held-out base-frame reference points must be non-collinear")
    predicted = camera @ transform[:3, :3].T + transform[:3, 3]
    residual_mm = (predicted - reference) * 1000
    return {
        "kind": "heldout", "source": "caller_supplied_points", **result,
        "tolerance_m": float(tolerance_m), "camera_to_base": transform.tolist(),
        "points_camera_m": camera.tolist(), "reference_base_m": reference.tolist(),
        "predicted_base_m": predicted.tolist(), "residual_base_mm": residual_mm.tolist(),
        "error_mm": np.linalg.norm(residual_mm, axis=1).tolist(),
        "rms_mm": result["rms_m"] * 1000, "max_error_mm": result["max_error_m"] * 1000,
        "notes": [
            "Reference base-frame points must be measured independently and excluded from fitting; independence cannot be inferred from these arrays.",
            "Residual vectors are transformed camera points minus the base-frame reference points.",
            "The pass result checks only the supplied point set and tolerance; no calibration acceptance or execution settings are changed.",
        ],
    }
