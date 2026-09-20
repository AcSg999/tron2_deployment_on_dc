#!/usr/bin/env python3
"""Fit a wrist-frame tip position from repeated touches of one fixed point."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from tron2_deployment.config import load_profile
from tron2_deployment.kinematics import RobotModel


def fit_pivot(wrists):
    """Return tip in wrist, common point in base, and per-touch errors in metres."""
    if len(wrists) < 4:
        raise ValueError("at least four distinct wrist poses are required")
    poses = np.asarray(wrists, dtype=float)
    if poses.shape != (len(wrists), 4, 4) or not np.isfinite(poses).all():
        raise ValueError("wrist poses must be finite 4x4 matrices")
    rotations = poses[:, :3, :3]
    positions = poses[:, :3, 3]
    design = np.concatenate((rotations, -np.broadcast_to(np.eye(3), rotations.shape)), axis=2).reshape(-1, 6)
    singular = np.linalg.svd(design, compute_uv=False)
    if singular[-1] < 1e-3 or singular[0] / singular[-1] > 1000:
        raise ValueError("wrist orientations are too similar; collect more varied tilts")
    solution = np.linalg.lstsq(design, -positions.reshape(-1), rcond=None)[0]
    tip, point = solution[:3], solution[3:]
    errors = np.linalg.norm(rotations @ tip + positions - point, axis=1)
    return tip, point, errors, float(singular[0] / singular[-1])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--states", required=True, nargs="+", type=Path,
                        help="four or more state JSON files, all touching the same fixed point")
    parser.add_argument("--tcp-quat-wxyz", nargs=4, type=float, metavar=("QW", "QX", "QY", "QZ"),
                        help="independently measured TCP axes in the wrist frame")
    parser.add_argument("--max-residual-mm", type=float, default=5.0)
    args = parser.parse_args(argv)
    try:
        if len(args.states) < 4 or len(set(args.states)) != len(args.states):
            raise ValueError("provide at least four different state files")
        if not np.isfinite(args.max_residual_mm) or args.max_residual_mm <= 0:
            raise ValueError("--max-residual-mm must be positive and finite")
        profile = load_profile(args.profile)
        model = RobotModel(profile)
        selected = slice(0, 7) if args.side == "left" else slice(7, 14)
        other = slice(7, 14) if args.side == "left" else slice(0, 7)
        ancestors = set()
        body = model.wrist_ids[args.side]
        while body:
            ancestors.add(body)
            body = int(model.model.body_parentid[body])
        if any(int(model.model.jnt_bodyid[joint]) in ancestors for joint in model.joint_ids[other]):
            raise ValueError("the other arm affects the selected wrist in this model")
        wrists = []
        sources = set()
        ignored_other_arm_limits = []
        for path in args.states:
            state = json.loads(path.read_text())
            sources.add(state.get("source"))
            arms = np.asarray(state["arm_q14"], dtype=float)
            if arms.shape != (14,) or not np.isfinite(arms).all():
                raise ValueError(f"{path}: arm_q14 must contain 14 finite numbers")
            for index in range(*selected.indices(14)):
                if arms[index] < model.lower[index] - 1e-8 or arms[index] > model.upper[index] + 1e-8:
                    name = profile["robot"]["arm_joint_names"][args.side][index % 7]
                    raise ValueError(f"{path}: selected arm joint {name} (index {index}) is "
                                     f"{arms[index]:.9f} rad, outside configured limits "
                                     f"[{model.lower[index]:.9f}, {model.upper[index]:.9f}]")
            for index in range(*other.indices(14)):
                if arms[index] < model.lower[index] or arms[index] > model.upper[index]:
                    ignored_other_arm_limits.append({
                        "state": str(path),
                        "joint": profile["robot"]["arm_joint_names"]["left" if index < 7 else "right"][index % 7],
                        "index": index, "measured_rad": float(arms[index]),
                        "configured_lower_rad": float(model.lower[index]),
                        "configured_upper_rad": float(model.upper[index]),
                    })
            arms[other] = np.clip(arms[other], model.lower[other], model.upper[other])
            # Only the unused arm is adjusted for FK; the selected arm remains measured.
            model.set_state(arms, state["head_q2"])
            wrists.append(model.wrist_matrices()[args.side])
        if len(sources) != 1 or next(iter(sources)) not in ("real", "mock"):
            raise ValueError("state files must all have the same real or mock source")
        tip, point, errors, condition = fit_pivot(wrists)
        if np.max(errors) * 1000 > args.max_residual_mm:
            raise ValueError(f"touch residual {np.max(errors)*1000:.2f} mm exceeds "
                             f"{args.max_residual_mm:g} mm; check the fixed point and samples")
        result = {
            "side": args.side, "source": next(iter(sources)),
            "tip_in_wrist_m": tip.tolist(), "common_point_in_base_m": point.tolist(),
            "touch_residuals_mm": (errors * 1000).tolist(),
            "max_residual_mm": float(np.max(errors) * 1000),
            "rms_residual_mm": float(np.sqrt(np.mean(errors**2)) * 1000),
            "design_condition": condition,
            "ignored_other_arm_limit_violations": ignored_other_arm_limits,
            "wrist_to_tcp_pose7": None,
        }
        if args.tcp_quat_wxyz is not None:
            quat = np.asarray(args.tcp_quat_wxyz)
            if not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1) > 1e-4:
                raise ValueError("--tcp-quat-wxyz must be a unit quaternion")
            # SciPy validates the supplied rotation; contact observations do not estimate it.
            Rotation.from_quat(quat[[1, 2, 3, 0]])
            result["wrist_to_tcp_pose7"] = tip.tolist() + quat.tolist()
        print(json.dumps(result, indent=2, allow_nan=False))
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    main()
