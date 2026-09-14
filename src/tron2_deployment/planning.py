"""Frozen-observation pregrasp planning with independently recheckable artifacts."""
from __future__ import annotations

import copy
import hashlib
import json
import math
import time

import numpy as np

from .config import profile_fingerprint
from .geometry import interpolate_transform, pose_matrix, rotation_error, vector
from .kinematics import RobotModel
from .targets import build_targets


def plan_digest(plan):
    payload = {key: value for key, value in plan.items() if key != "plan_id"}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


plan_fingerprint = plan_digest


def validate_plan_integrity(plan):
    if not isinstance(plan, dict) or plan.get("plan_id") != plan_digest(plan):
        raise ValueError("plan content does not match its plan_id")
    if plan.get("mode") != "pregrasp" or plan.get("schema_version") != 1:
        raise ValueError("unsupported plan schema/mode")
    if plan.get("interpolation") != "quintic_stop":
        raise ValueError("unsupported trajectory interpolation")


def _limits(profile):
    robot = profile["robot"]
    velocity = vector(robot["velocity_limits"], 14, "velocity_limits")
    acceleration = vector(robot["acceleration_limits"], 14, "acceleration_limits")
    if np.any(velocity <= 0) or np.any(acceleration <= 0):
        raise ValueError("velocity and acceleration limits must be positive")
    return velocity, acceleration


def _duration(delta, velocity, acceleration):
    return max(float(np.max(1.875 * np.abs(delta) / velocity)),
        float(np.max(np.sqrt((10 / np.sqrt(3)) * np.abs(delta) / acceleration))), 0.05)


def _validate_observation(profile, observation, state, *, check_state_age=False):
    # Pure geometry callers may omit acquisition metadata; when present, it is
    # binding. Real execution additionally requires every provenance field.
    if "profile_hash" in observation and observation["profile_hash"] != profile_fingerprint(profile):
        raise ValueError("object observation was acquired under a different calibration/profile")
    if "source" in observation and observation["source"] != state.get("source"):
        raise ValueError("object observation and measured state sources differ")
    vision = profile.get("vision", {})
    if "mesh_id" in observation and "mesh_id" in vision and observation["mesh_id"] != vision["mesh_id"]:
        raise ValueError("object observation mesh differs from configured collision geometry")
    if "confidence" in observation:
        confidence = float(observation["confidence"])
        threshold = float(vision.get("min_confidence", 0.5))
        if not np.isfinite([confidence, threshold]).all() or not 0 <= threshold <= 1 or not threshold <= confidence <= 1:
            raise ValueError("object pose confidence is invalid or below the accepted threshold")
    if "head_q2" in observation:
        observed_head = vector(observation["head_q2"], 2, "observation head_q2")
        measured_head = vector(state["head_q2"], 2, "measured head_q2")
        tolerance = float(profile.get("execution", {}).get("head_tolerance_rad", 0.01))
        if not np.isfinite(tolerance) or tolerance <= 0 or np.max(np.abs(observed_head - measured_head)) > tolerance:
            raise ValueError("head moved after object observation; acquire a new observation")
    if state.get("source") == "real" or observation.get("source") == "real":
        required = ("source", "profile_hash", "calibration_id", "head_q2", "mesh_id", "confidence", "capture_timestamp_s")
        if any(key not in observation for key in required) or observation.get("source") != "real" or state.get("source") != "real":
            raise ValueError("real planning requires complete real observation provenance")
        calibration = profile.get("calibration", {})
        if calibration.get("verified") is not True:
            raise ValueError("real planning requires independently verified calibration")
        if calibration.get("source") != "real" or observation["calibration_id"] != calibration.get("id"):
            raise ValueError("real observation calibration does not match the accepted profile")
        if observation["mesh_id"] != vision.get("mesh_id"):
            raise ValueError("real observation requires the configured object mesh")
        calibrated_head = vector(calibration["head_q2"], 2, "calibrated head_q2")
        if np.max(np.abs(calibrated_head - observed_head)) > tolerance:
            raise ValueError("object observation head differs from calibrated head pose")
        now = time.time()
        settings = profile.get("execution", {})
        age_limit = float(settings.get("observation_max_age_s", 120.0))
        captured = float(observation["capture_timestamp_s"])
        if not np.isfinite([age_limit, captured]).all() or age_limit <= 0 or captured <= 0 or not -0.05 <= now - captured <= age_limit:
            raise ValueError("object observation is stale or has an invalid timestamp")
        if check_state_age:
            state_age_limit = float(settings.get("start_max_age_s", 0.25))
            measured_at = float(state["timestamp_s"])
            if not np.isfinite([state_age_limit, measured_at]).all() or state_age_limit <= 0 or not -0.05 <= now - measured_at <= state_age_limit:
                raise ValueError("measured planning state is stale or has an invalid timestamp")


def _collision_segment(model, start, end, head):
    # Quintic scalar progress traverses the same joint-space line segment.
    # Evaluate both arms together throughout that segment, including endpoints.
    delta = np.abs(end - start)
    # Between samples each geometry moves by at most this weighted L1 bound;
    # double it for two moving bodies. Keep that relative motion below half the
    # checked clearance so a narrow obstacle cannot be skipped between samples.
    relative_motion_bound = 2 * float(np.dot(delta, model.motion_weights))
    count = max(1, int(np.ceil(np.max(delta) / 0.01)),
                int(np.ceil(relative_motion_bound / (model.clearance / 2))))
    for fraction in np.linspace(0, 1, count + 1):
        model.set_state(start + (end - start) * fraction, head)
        model.check_collision()


def plan_pregrasp(profile, observation, state, sides=("left", "right")):
    arms = vector(state["arm_q14"], 14, "arm_q14")
    head = vector(state["head_q2"], 2, "head_q2")
    if state.get("source") not in ("real", "mock"):
        raise ValueError("state source must identify real or mock feedback")
    stamp = float(state["timestamp_s"])
    if not np.isfinite(stamp) or stamp <= 0:
        raise ValueError("state timestamp must be positive and finite")
    _validate_observation(profile, observation, state, check_state_age=True)
    model = RobotModel(profile, observation)
    model.set_state(arms, head)
    model.check_collision()
    targets = build_targets(profile, observation, model.wrist_poses(), sides)
    current = model.wrist_matrices()
    endpoints = {side: pose_matrix(target["wrist_pregrasp_pose7_base"]) for side, target in targets.items()}
    # Lift, move above the target while changing orientation, then descend to
    # standoff. The model tests actual swept arm configurations on every segment.
    waypoints = []
    mounts = {side: pose_matrix(profile["pregrasp"][side]["wrist_to_tcp_pose7"]) for side in targets}
    for stage in range(3):
        waypoint = {}
        for side, end in endpoints.items():
            start_tcp = current[side] @ mounts[side]
            target_tcp = end @ mounts[side]
            transit_z = max(start_tcp[2, 3], target_tcp[2, 3]) + targets[side]["lift_clearance_m"]
            tcp = start_tcp.copy() if stage == 0 else target_tcp.copy()
            if stage != 2:
                tcp[2, 3] = transit_z
            waypoint[side] = tcp @ np.linalg.inv(mounts[side])
        waypoints.append(waypoint)
    path = [arms.copy()]
    previous = {side: current[side] for side in targets}
    for waypoint in waypoints:
        translation = max(np.linalg.norm(waypoint[s][:3, 3] - previous[s][:3, 3]) for s in targets)
        angle = max(np.linalg.norm(rotation_error(waypoint[s][:3, :3], previous[s][:3, :3])) for s in targets)
        count = max(1, math.ceil(translation / 0.02), math.ceil(angle / 0.1))
        for fraction in np.linspace(0, 1, count + 1)[1:]:
            goals = {side: interpolate_transform(previous[side], waypoint[side], float(fraction)) for side in targets}
            solved = model.solve(goals, path[-1], head)
            _collision_segment(model, path[-1], solved, head)
            if np.max(np.abs(solved - path[-1])) > 1e-8:
                path.append(solved)
        previous = waypoint
    if len(path) == 1:
        path.append(path[0].copy())
    velocity, acceleration = _limits(profile)
    times = [0.0]
    for start, end in zip(path, path[1:]):
        times.append(times[-1] + _duration(end - start, velocity, acceleration) * 1.01)
    array = np.asarray(path)
    plan = {
        "schema_version": 1, "mode": "pregrasp", "interpolation": "quintic_stop",
        "selected_sides": list(targets), "times_s": times,
        "left_arm_qpos": array[:, :7].tolist(), "right_arm_qpos": array[:, 7:].tolist(),
        "head_qpos": np.tile(head, (len(array), 1)).tolist(),
        "targets": targets, "profile_hash": profile_fingerprint(profile),
        "source": {"observation": copy.deepcopy(observation), "state": copy.deepcopy(state)},
        "checks": {"model_hash": model.model_hash},
    }
    plan["checks"] = _validate_path(profile, plan, model)
    plan["plan_id"] = plan_digest(plan)
    return plan


def _validate_path(profile, plan, model):
    if plan.get("interpolation") != "quintic_stop":
        raise ValueError("unsupported trajectory interpolation")
    if plan.get("profile_hash") != profile_fingerprint(profile):
        raise ValueError("plan calibration/profile no longer matches")
    if plan.get("checks", {}).get("model_hash") != model.model_hash:
        raise ValueError("robot model no longer matches planned model")
    times = np.asarray(plan["times_s"], dtype=float)
    left = np.asarray(plan["left_arm_qpos"], dtype=float)
    right = np.asarray(plan["right_arm_qpos"], dtype=float)
    heads = np.asarray(plan["head_qpos"], dtype=float)
    if times.ndim != 1 or len(times) < 2 or left.shape != (len(times), 7) or right.shape != left.shape or heads.shape != (len(times), 2):
        raise ValueError("malformed synchronized trajectory arrays")
    if not all(np.isfinite(value).all() for value in (times, left, right, heads)):
        raise ValueError("trajectory contains nonfinite values")
    if times[0] != 0 or np.any(np.diff(times) <= 0):
        raise ValueError("trajectory times must start at zero and strictly increase")
    source = plan["source"]["state"]
    _validate_observation(profile, plan["source"]["observation"], source)
    start = vector(source["arm_q14"], 14, "source arm_q14")
    head = vector(source["head_q2"], 2, "source head_q2")
    arms = np.column_stack((left, right))
    if not np.allclose(arms[0], start, atol=1e-9, rtol=0) or not np.allclose(heads, head, atol=1e-9, rtol=0):
        raise ValueError("trajectory start/head must match frozen measured state")
    selected = tuple(plan["selected_sides"])
    if not selected or len(set(selected)) != len(selected) or any(side not in ("left", "right") for side in selected):
        raise ValueError("invalid selected sides")
    for side, columns in (("left", slice(0, 7)), ("right", slice(7, 14))):
        if side not in selected and not np.allclose(arms[:, columns], start[columns], atol=1e-9, rtol=0):
            raise ValueError("unselected arm moves")
    velocity, acceleration = _limits(profile)
    for before, after, duration in zip(arms, arms[1:], np.diff(times)):
        delta = np.abs(after - before)
        if np.any(1.875 * delta / duration > velocity + 1e-8):
            raise ValueError("quintic trajectory violates velocity limits")
        if np.any((10 / np.sqrt(3)) * delta / duration**2 > acceleration + 1e-8):
            raise ValueError("quintic trajectory violates acceleration limits")
        _collision_segment(model, before, after, head)
    model.set_state(start, head)
    expected = build_targets(profile, plan["source"]["observation"], model.wrist_poses(), selected)
    if expected != plan["targets"]:
        raise ValueError("targets do not match frozen object pose and task configuration")
    model.set_state(arms[-1], head)
    actual = model.wrist_matrices()
    position_error = 0.0
    orientation_error = 0.0
    for side, target in expected.items():
        goal = pose_matrix(target["wrist_pregrasp_pose7_base"])
        position_error = max(position_error, float(np.linalg.norm(goal[:3, 3] - actual[side][:3, 3])))
        orientation_error = max(orientation_error, float(np.linalg.norm(rotation_error(goal[:3, :3], actual[side][:3, :3]))))
    if position_error > 0.001 or orientation_error > 0.01:
        raise ValueError("final wrist FK does not reach the pregrasp targets")
    return {
        "ik": True, "joint_limits": True, "collision": True, "velocity": True, "acceleration": True,
        "model_hash": model.model_hash, "max_position_error_m": position_error,
        "max_orientation_error_rad": orientation_error,
        "collision_joint_step_max": 0.01, "collision_clearance_m": model.clearance,
        "collision_method": "combined_joint_path_clearance_bounded_sampling",
    }


def validate_plan_path(profile, plan):
    validate_plan_integrity(plan)
    return _validate_path(profile, plan, RobotModel(profile, plan["source"]["observation"]))
