"""Validated arm-only pregrasp playback with measured feedback and bounded hold."""

from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from pathlib import Path
from threading import Event

import numpy as np

from .robot import _vector


DEFAULT_EXECUTION = {
    "start_tolerance_rad": 0.03,
    "head_tolerance_rad": 0.01,
    "arrival_tolerance_rad": 0.02,
    "tracking_tolerance_rad": 0.15,
    "start_max_age_s": 0.25,
    "feedback_max_age_s": 0.25,
    "observation_max_age_s": 120.0,
    "publish_hz": 500.0,
    "arrival_timeout_s": 3.0,
    "arrival_samples": 3,
    "hold_duration_s": 0.1,
    "max_publish_lag_s": 0.02,
    "max_duration_s": 120.0,
}


def _settings(profile):
    settings = {**DEFAULT_EXECUTION, **profile.get("execution", {})}
    for key in DEFAULT_EXECUTION:
        value = float(settings[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"execution.{key} must be positive and finite")
        settings[key] = value
    if settings["publish_hz"] < 500:
        raise ValueError("TRON2 ServoJ requires execution.publish_hz >= 500")
    if not settings["arrival_samples"].is_integer():
        raise ValueError("execution.arrival_samples must be an integer")
    return settings


def _arrays(plan, profile, settings):
    if plan.get("schema_version") != 1 or plan.get("mode") != "pregrasp":
        raise ValueError("only schema_version=1 pregrasp plans can execute")
    if plan.get("interpolation") != "quintic_stop":
        raise ValueError("plan interpolation must be quintic_stop")
    if any(key in plan for key in ("gripper_events", "gripper_commands", "grasp_events")):
        raise ValueError("pregrasp execution does not accept gripper or contact events")
    times = np.asarray(plan["times_s"], dtype=float)
    if (times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all()
            or abs(times[0]) > 1e-12 or np.any(np.diff(times) <= 0)):
        raise ValueError("times_s must start at zero and strictly increase")
    if times[-1] > settings["max_duration_s"]:
        raise ValueError("trajectory exceeds execution.max_duration_s")
    left = np.asarray(plan["left_arm_qpos"], dtype=float)
    right = np.asarray(plan["right_arm_qpos"], dtype=float)
    head = np.asarray(plan["head_qpos"], dtype=float)
    if left.shape != (len(times), 7) or right.shape != left.shape or head.shape != (len(times), 2):
        raise ValueError("arm/head trajectories must share one Nx7/Nx2 timeline")
    q = np.concatenate((left, right), axis=1)
    if not np.isfinite(q).all() or not np.isfinite(head).all():
        raise ValueError("joint trajectories must be finite")
    if not np.allclose(head, head[0], atol=1e-12, rtol=0):
        raise ValueError("pregrasp execution must hold the calibrated head pose")
    robot = profile["robot"]
    lower = _vector(robot["joint_lower"], 14, "robot.joint_lower")
    upper = _vector(robot["joint_upper"], 14, "robot.joint_upper")
    velocity = _vector(robot["velocity_limits"], 14, "robot.velocity_limits")
    acceleration = _vector(robot["acceleration_limits"], 14, "robot.acceleration_limits")
    if np.any(lower >= upper) or np.any(velocity <= 0) or np.any(acceleration <= 0):
        raise ValueError("joint/dynamic limits must describe a valid bounded robot")
    if np.any(q < lower - 1e-10) or np.any(q > upper + 1e-10):
        raise ValueError("trajectory exceeds joint limits")
    delta = np.abs(np.diff(q, axis=0))
    dt = np.diff(times)[:, None]
    if np.any(1.875 * delta / dt > velocity * (1 + 1e-8)):
        raise ValueError("interpolated trajectory exceeds velocity limits")
    if np.any((10 / math.sqrt(3)) * delta / dt**2 > acceleration * (1 + 1e-8)):
        raise ValueError("interpolated trajectory exceeds acceleration limits")
    return times, q, head, lower, upper


def _check_state(state, *, now, max_age, lower, upper, expected_source):
    if state.get("source") != expected_source:
        raise ValueError(f"expected {expected_source} robot feedback")
    q = _vector(state["arm_q14"], 14, "feedback.arm_q14")
    head = _vector(state["head_q2"], 2, "feedback.head_q2")
    stamp = float(state["timestamp_s"])
    if not math.isfinite(stamp) or stamp <= 0 or now - stamp > max_age or stamp - now > 0.05:
        raise ValueError("robot feedback is stale or has an invalid timestamp")
    if np.any(q < lower) or np.any(q > upper):
        raise ValueError("measured robot state exceeds configured joint limits")
    return q, head


def _check_observation(plan, profile, settings, now, real):
    observation = plan["source"]["observation"]
    if observation.get("reference_frame") != "base_Link":
        raise ValueError("observation must use calibrated base_Link coordinates")
    stamp = float(observation.get("capture_timestamp_s", observation.get("timestamp_s", 0)))
    if (not math.isfinite(stamp) or stamp <= 0 or now - stamp > settings["observation_max_age_s"]
            or stamp - now > 0.05):
        raise ValueError("object observation is stale or has an invalid timestamp")
    if real and observation.get("source") != "real":
        raise ValueError("real execution requires a real object observation")
    if real:
        from .config import profile_fingerprint
        if observation.get("profile_hash") != profile_fingerprint(profile):
            raise ValueError("observation belongs to a different calibration/deployment profile")
        if observation.get("mesh_id") != profile["vision"]["mesh_id"]:
            raise ValueError("observation mesh differs from configured object geometry")
        if observation.get("calibration_id") != profile["calibration"]["id"]:
            raise ValueError("observation calibration ID does not match")
        confidence = float(observation["confidence"])
        minimum = float(profile["vision"].get("min_confidence", 0.5))
        if not math.isfinite(minimum) or not 0 <= minimum <= 1:
            raise ValueError("vision.min_confidence must be in [0, 1]")
        if not math.isfinite(confidence) or not minimum <= confidence <= 1:
            raise ValueError("object pose confidence is below the accepted threshold")
        observed_head = _vector(observation["head_q2"], 2, "observation.head_q2")
        planned_head = _vector(plan["source"]["state"]["head_q2"], 2, "source.head_q2")
        if np.max(np.abs(observed_head - planned_head)) > settings["head_tolerance_rad"]:
            raise ValueError("observed camera head pose differs from the planned head pose")


def preflight(plan, profile, state, *, real=False, reviewed_plan_id=None,
              supervisor_confirmed=False) -> dict:
    """Return a report; failed preflight never authorizes any motor commands."""
    report = {"ok": False, "real": bool(real),
              "plan_id": plan.get("plan_id") if isinstance(plan, dict) else None, "errors": []}
    try:
        from .config import profile_fingerprint
        from .planning import validate_plan_integrity

        validate_plan_integrity(plan)
        digest = profile_fingerprint(profile)
        if plan.get("profile_hash") != digest:
            raise ValueError("plan calibration/deployment profile does not match")
        settings = _settings(profile)
        times, q, head, lower, upper = _arrays(plan, profile, settings)
        planned_state = plan["source"]["state"]
        if (not np.allclose(_vector(planned_state["arm_q14"], 14, "source.arm_q14"), q[0], atol=1e-9, rtol=0)
                or not np.allclose(_vector(planned_state["head_q2"], 2, "source.head_q2"), head[0], atol=1e-9, rtol=0)):
            raise ValueError("trajectory start/head differs from the frozen measured state")
        if real and planned_state.get("source") != "real":
            raise ValueError("real execution requires a plan from real measured robot state")
        checks = plan.get("checks", {})
        for name in ("ik", "joint_limits", "collision", "velocity", "acceleration"):
            if checks.get(name) is not True:
                raise ValueError(f"plan is missing successful {name} checks")
        now = time.time()
        measured_q, measured_head = _check_state(
            state, now=now, max_age=settings["start_max_age_s"], lower=lower, upper=upper,
            expected_source="real" if real else "mock")
        if np.max(np.abs(measured_q - q[0])) > settings["start_tolerance_rad"]:
            raise ValueError("measured arms do not match the reviewed trajectory start")
        if np.max(np.abs(measured_head - head[0])) > settings["head_tolerance_rad"]:
            raise ValueError("measured head does not match the frozen planning head pose")
        _check_observation(plan, profile, settings, now, real)
        if real:
            if reviewed_plan_id != plan["plan_id"]:
                raise ValueError("real execution requires review of this exact plan_id")
            if supervisor_confirmed is not True:
                raise ValueError("real execution requires explicit supervisor confirmation")
            if settings.get("allow_real") is not True or settings.get("hold_behavior_verified") is not True:
                raise ValueError("real execution and bounded hold behavior must be accepted in the profile")
            calibration = profile.get("calibration", {})
            if calibration.get("verified") is not True or calibration.get("source") != "real":
                raise ValueError("real execution requires independently verified real calibration")
            calibrated_head = _vector(calibration["head_q2"], 2, "calibration.head_q2")
            if np.max(np.abs(calibrated_head - head[0])) > settings["head_tolerance_rad"]:
                raise ValueError("planned head differs from the camera calibration head pose")
            if not profile["robot"].get("model_hash") or checks.get("model_hash") != profile["robot"]["model_hash"]:
                raise ValueError("plan must match the accepted installed robot model hash")
            from .planning import validate_plan_path
            verified = validate_plan_path(profile, plan)
            if verified.get("model_hash") != profile["robot"]["model_hash"]:
                raise ValueError("current robot model/assets differ from the accepted model hash")
            for name in ("ik", "joint_limits", "collision", "velocity", "acceleration"):
                if verified.get(name) is not True:
                    raise ValueError(f"independent path validation failed: {name}")
        report.update(ok=True, profile_hash=digest, duration_s=float(times[-1]),
                      start_error_rad=float(np.max(np.abs(measured_q - q[0]))),
                      head_error_rad=float(np.max(np.abs(measured_head - head[0]))),
                      checked_at_s=time.time())
    except (ValueError, TypeError, KeyError, RuntimeError, OSError, ImportError, OverflowError, AttributeError) as exc:
        report["errors"].append(str(exc))
    return report


def _interpolate(times, q, elapsed):
    if elapsed >= times[-1]:
        return q[-1].copy()
    index = max(0, int(np.searchsorted(times, elapsed, side="right")) - 1)
    u = float(np.clip((elapsed - times[index]) / (times[index + 1] - times[index]), 0, 1))
    s = u**3 * (10 + u * (-15 + 6 * u))
    return q[index] + s * (q[index + 1] - q[index])


def execute_plan(plan, profile, adapter, *, real=False, reviewed_plan_id=None,
                 supervisor_confirmed=False, stop_event=None, log_path=None) -> dict:
    """Play the reviewed pregrasp trajectory and stop at its final wrist poses.

    Real streaming requires explicit opt-in, a matched review, a validated
    hardware profile, and a writable log. This function sends no gripper,
    initialization, MoveJ, or grasp/contact command.
    """
    plan, profile = deepcopy(plan), deepcopy(profile)
    stop_event = stop_event or Event()
    result = {"status": "rejected", "plan_id": plan.get("plan_id"), "real": bool(real),
              "hardware_commanded": False, "samples_count": 0, "errors": []}
    stream = None
    authorized = False
    last_q = last_head = last_state = None
    settings = None

    def record(kind, **fields):
        if stream is not None:
            stream.write(json.dumps({"event": kind, "timestamp_s": time.time(), **fields},
                                    allow_nan=False) + "\n")
            stream.flush()

    try:
        if getattr(adapter, "source", None) != ("real" if real else "mock"):
            raise ValueError("adapter source and requested execution mode do not match")
        if real and log_path is None:
            raise ValueError("real execution requires a log_path")
        settings = _settings(profile)
        times, q, head, lower, upper = _arrays(plan, profile, settings)
        # The first explicit real state read is lazy and read-only.
        last_state = adapter.read_state()
        report = preflight(plan, profile, last_state, real=real,
                           reviewed_plan_id=reviewed_plan_id,
                           supervisor_confirmed=supervisor_confirmed)
        result["preflight"] = report
        if not report["ok"]:
            result["errors"] = report["errors"]
            return result
        if log_path is not None:
            path = Path(log_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive creation prevents silently overwriting prior execution evidence.
            stream = path.open("x", encoding="utf-8")
            result["log_path"] = str(path)
            record("preflight", report=report, plan=plan)
        # Expensive collision validation may outlast feedback or observation
        # freshness. Check them again immediately before command authorization.
        last_state = adapter.read_state()
        last_q, last_head = _check_state(
            last_state, now=time.time(), max_age=settings["start_max_age_s"],
            lower=lower, upper=upper, expected_source="real" if real else "mock")
        if np.max(np.abs(last_q - q[0])) > settings["start_tolerance_rad"]:
            raise ValueError("arms moved during preflight; review and replan")
        if np.max(np.abs(last_head - head[0])) > settings["head_tolerance_rad"]:
            raise ValueError("head moved during preflight; acquire a new observation")
        _check_observation(plan, profile, settings, time.time(), real)
        if stop_event.is_set():
            raise InterruptedError("execution stopped before authorization")
        if real:
            from .config import profile_fingerprint
            if profile_fingerprint(adapter.profile) != profile_fingerprint(profile):
                raise ValueError("robot adapter uses a different deployment profile")
            adapter._authorize_execution(report)
        authorized = True
        result["status"] = "running"
        started = time.monotonic()
        interval = 1 / settings["publish_hz"]
        tick = 0
        arrival_count = 0
        last_arrival_stamp = None
        previous_command = q[0].copy()
        while True:
            deadline = started + tick * interval
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            now_mono = time.monotonic()
            elapsed = now_mono - started
            if stop_event.is_set():
                raise InterruptedError("execution stopped by operator")
            if now_mono - deadline > settings["max_publish_lag_s"]:
                raise RuntimeError("ServoJ publisher missed its configured timing limit")
            measured = adapter.read_state()
            measured_q, measured_head = _check_state(
                measured, now=time.time(), max_age=settings["feedback_max_age_s"],
                lower=lower, upper=upper, expected_source="real" if real else "mock")
            now_mono = time.monotonic()
            elapsed = now_mono - started
            if now_mono - deadline > settings["max_publish_lag_s"]:
                raise RuntimeError("feedback read exceeded the ServoJ timing limit")
            last_state = measured
            if np.max(np.abs(measured_head - head[0])) > settings["head_tolerance_rad"]:
                raise RuntimeError("head changed during pregrasp execution")
            tracking_error = float(np.max(np.abs(measured_q - previous_command)))
            if tracking_error > settings["tracking_tolerance_rad"]:
                raise RuntimeError("arm tracking error exceeds configured tolerance")
            command_q = _interpolate(times, q, elapsed)
            send_result = adapter.command(command_q.tolist(), head[0].tolist())
            if send_result is False:
                raise RuntimeError("adapter rejected the arm command")
            result["hardware_commanded"] = bool(real)
            last_q, last_head = command_q, head[0]
            previous_command = command_q
            result["samples_count"] += 1
            record("sample", elapsed_s=elapsed, deadline_lag_s=now_mono - deadline,
                   planned_arm_q14=command_q.tolist(), measured=measured,
                   tracking_error_rad=tracking_error)
            if elapsed >= times[-1]:
                final_error = float(np.max(np.abs(measured_q - q[-1])))
                if final_error <= settings["arrival_tolerance_rad"]:
                    stamp = measured["timestamp_s"]
                    if last_arrival_stamp is None or stamp > last_arrival_stamp:
                        arrival_count += 1
                        last_arrival_stamp = stamp
                else:
                    arrival_count = 0
                if arrival_count >= settings["arrival_samples"]:
                    result.update(status="completed", final_state=measured,
                                  arrival_error_rad=final_error, elapsed_s=elapsed)
                    record("completed", result=result)
                    break
                if elapsed > times[-1] + settings["arrival_timeout_s"]:
                    raise TimeoutError("pregrasp arrival was not verified before timeout")
            tick += 1
    except (Exception, KeyboardInterrupt) as exc:
        result["errors"].append(str(exc) or type(exc).__name__)
        result["status"] = ("stopped" if isinstance(exc, (InterruptedError, KeyboardInterrupt))
                            else "failed" if authorized else "rejected")
        if authorized and last_q is not None:
            # Freeze a recent measured state where possible; otherwise retain
            # the last validated setpoint. Never advance to another waypoint.
            try:
                measured = adapter.read_state()
                hold_q, hold_head = _check_state(
                    measured, now=time.time(), max_age=settings["feedback_max_age_s"],
                    lower=lower, upper=upper, expected_source="real" if real else "mock")
                last_q, last_head = hold_q, hold_head
            except Exception as feedback_exc:
                result["hold_feedback_error"] = str(feedback_exc)
            try:
                hold_start = time.monotonic()
                for index in range(max(1, math.ceil(settings["hold_duration_s"] * settings["publish_hz"]))):
                    remaining = hold_start + index / settings["publish_hz"] - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
                    if adapter.command(last_q.tolist(), last_head.tolist()) is False:
                        raise RuntimeError("adapter rejected hold command")
                    result["hardware_commanded"] = bool(real)
                result["hold"] = {"sent": True, "arm_q14": last_q.tolist(), "head_q2": last_head.tolist(),
                                  "completion_verified": False}
            except Exception as hold_exc:
                result["hold"] = {"sent": False, "error": str(hold_exc), "completion_verified": False}
                result["errors"].append(f"hold failed: {hold_exc}")
        try:
            record("aborted", result=result)
        except Exception as log_exc:
            result["errors"].append(f"execution log failed: {log_exc}")
    finally:
        if real and authorized:
            adapter._revoke_execution()
        if stream is not None:
            stream.close()
    return result
