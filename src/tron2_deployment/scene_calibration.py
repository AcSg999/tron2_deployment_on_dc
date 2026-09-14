"""Rigid table-to-robot-base calibration from three or more correspondences."""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

DEFAULT_OUTPUT = Path.cwd() / "output" / "scene_calib.json"


@dataclass(frozen=True)
class SceneCalibration:
    table_to_base: np.ndarray
    rms_m: float
    max_error_m: float
    inliers: np.ndarray
    source: str = "correspondences"

    def validate(self) -> None:
        transform = np.asarray(self.table_to_base, dtype=np.float64)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError("table_to_base must be a finite 4x4 matrix")
        if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9):
            raise ValueError("table_to_base has an invalid homogeneous row")
        rotation = transform[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise ValueError("table_to_base rotation is not orthonormal")
        if np.linalg.det(rotation) < 0.999:
            raise ValueError("table_to_base rotation contains a reflection")

    @property
    def base_to_table(self) -> np.ndarray:
        return np.linalg.inv(self.table_to_base)

    def transform_points(self, points_table: np.ndarray) -> np.ndarray:
        points = np.asarray(points_table, dtype=np.float64)
        return points @ self.table_to_base[:3, :3].T + self.table_to_base[:3, 3]

    def to_dict(self) -> dict:
        self.validate()
        return {
            "schema_version": 1,
            "source": self.source,
            "created_unix_s": time.time(),
            "table_to_base": self.table_to_base.tolist(),
            "base_to_table": self.base_to_table.tolist(),
            "rms_m": self.rms_m,
            "max_error_m": self.max_error_m,
            "inliers": np.asarray(self.inliers, dtype=int).tolist(),
        }


def _validate_points(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("correspondences must be matching shape-(N, 3) arrays")
    if len(source) < 3 or not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("at least three finite correspondences are required")
    if np.linalg.matrix_rank(source - source.mean(axis=0)) < 2:
        raise ValueError("source correspondences are collinear")
    return source, target


def fit_rigid_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return a 4x4 transform mapping source-frame points into target frame."""
    source, target = _validate_points(source, target)
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_center).T @
                             (target - target_center))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def calibrate_scene(
    table_points: np.ndarray,
    base_points: np.ndarray,
    *,
    inlier_threshold_m: float = 0.01,
    ransac_iterations: int = 256,
    seed: int = 0,
    source: str = "correspondences",
) -> SceneCalibration:
    """Robustly estimate table-to-base using three-point RANSAC and Kabsch."""
    table_points, base_points = _validate_points(table_points, base_points)
    if inlier_threshold_m <= 0 or ransac_iterations < 1:
        raise ValueError("RANSAC threshold/iterations must be positive")
    count = len(table_points)
    rng = np.random.default_rng(seed)
    candidates = [np.arange(count)] if count == 3 else [
        rng.choice(count, 3, replace=False) for _ in range(ransac_iterations)
    ]
    best_inliers: np.ndarray | None = None
    best_median = np.inf
    for indices in candidates:
        try:
            transform = fit_rigid_transform(
                table_points[indices], base_points[indices])
        except ValueError:
            continue
        predicted = (table_points @ transform[:3, :3].T
                     + transform[:3, 3])
        error = np.linalg.norm(predicted - base_points, axis=1)
        inliers = np.flatnonzero(error <= inlier_threshold_m)
        median = float(np.median(error[inliers])) if len(inliers) else np.inf
        if (best_inliers is None or len(inliers) > len(best_inliers)
                or (len(inliers) == len(best_inliers) and median < best_median)):
            best_inliers = inliers
            best_median = median
    if best_inliers is None or len(best_inliers) < 3:
        raise ValueError("scene calibration found fewer than three inliers")
    transform = fit_rigid_transform(
        table_points[best_inliers], base_points[best_inliers])
    predicted = table_points @ transform[:3, :3].T + transform[:3, 3]
    error = np.linalg.norm(predicted - base_points, axis=1)
    calibration = SceneCalibration(
        table_to_base=transform,
        rms_m=float(np.sqrt(np.mean(error[best_inliers] ** 2))),
        max_error_m=float(np.max(error[best_inliers])),
        inliers=best_inliers,
        source=source,
    )
    calibration.validate()
    return calibration


def save_scene_calibration(calibration: SceneCalibration, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(calibration.to_dict(), indent=2))


def load_scene_calibration(path: Path) -> SceneCalibration:
    data = json.loads(path.read_text())
    calibration = SceneCalibration(
        table_to_base=np.asarray(data["table_to_base"], dtype=np.float64),
        rms_m=float(data["rms_m"]),
        max_error_m=float(data["max_error_m"]),
        inliers=np.asarray(data["inliers"], dtype=np.int64),
        source=str(data.get("source", path)),
    )
    calibration.validate()
    return calibration


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("correspondences", type=Path,
                        help="NPZ containing table_points and base_points")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--threshold-mm", type=float, default=10.0)
    args = parser.parse_args(argv)
    with np.load(args.correspondences) as data:
        calibration = calibrate_scene(
            data["table_points"], data["base_points"],
            inlier_threshold_m=args.threshold_mm / 1000.0,
            source=str(args.correspondences),
        )
    save_scene_calibration(calibration, args.output)
    print(json.dumps(calibration.to_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
