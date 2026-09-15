"""Explicit command-line stages from calibration to reviewed pregrasp execution."""
import argparse
import glob
import hashlib
import json
from pathlib import Path
import signal
import threading
import time

from .config import load_profile, write_json


def read_json(path):
    return json.loads(Path(path).read_text())


def calibration_report_command(args):
    """Inspect saved evidence without changing calibration or connecting hardware."""
    import numpy as np
    from .calibration_diagnostics import (
        intrinsic_diagnostics, handeye_diagnostics, heldout_diagnostics,
    )
    from .calibration_report import write_calibration_report

    if not args.intrinsics and not args.handeye:
        raise ValueError("provide --intrinsics, --handeye, or both")
    if args.images and not args.intrinsics:
        raise ValueError("--images requires --intrinsics")
    if args.samples and not args.handeye:
        raise ValueError("--samples requires --handeye")
    if args.validation_points and not args.handeye:
        raise ValueError("--validation-points requires --handeye")
    if bool(args.validation_points) != (args.max_error_m is not None):
        raise ValueError("provide --validation-points and --max-error-m together")
    output = Path(args.output)
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError("report output must be a new or empty directory")

    def matched(pattern, name):
        paths = [Path(p).resolve() for p in sorted(glob.glob(pattern))]
        if not paths:
            raise ValueError(f"{name} matched no files: {pattern}")
        return paths

    def provenance(path):
        path = Path(path).resolve()
        return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    intrinsic = handeye = heldout = None
    if args.intrinsics:
        source_path = Path(args.intrinsics).resolve()
        calibration = read_json(source_path)
        if args.images:
            images = matched(args.images, "--images")
            fit_images = []
            for name in calibration.get("images", []):
                image = Path(name).expanduser()
                candidates = {image.resolve()} if image.is_absolute() else {
                    image.resolve(), (source_path.parent / image).resolve()}
                available = [p for p in candidates if p.exists()]
                if len(available) == 1:
                    fit_images.append(str(available[0]))
                else:
                    selected = [p for p in available if p in images]
                    if len(selected) == 1:
                        fit_images.append(str(selected[0]))
            calibration["images"] = fit_images
        else:
            images = []
            for name in calibration.get("images", []):
                image = Path(name).expanduser()
                if not image.is_absolute():
                    local = image.resolve()
                    adjacent = (source_path.parent / image).resolve()
                    if local.exists() and adjacent.exists() and local != adjacent:
                        raise ValueError(f"ambiguous calibration image {name}; provide --images explicitly")
                    image = local if local.exists() else adjacent
                images.append(image.resolve())
            if not images:
                raise ValueError("intrinsic JSON has no images; provide --images with original calibration views")
            calibration["images"] = [str(p) for p in images]
        intrinsic = intrinsic_diagnostics(calibration, images)
        intrinsic["input_file"] = provenance(source_path)
    if args.handeye:
        source_path = Path(args.handeye).resolve()
        calibration = read_json(source_path)
        sample_paths = matched(args.samples, "--samples") if args.samples else []
        handeye = handeye_diagnostics(calibration, [read_json(p) for p in sample_paths],
                                      sample_labels=[p.name for p in sample_paths])
        handeye["input_file"] = provenance(source_path)
        handeye["sample_files"] = [provenance(p) for p in sample_paths]
        if args.validation_points:
            with np.load(args.validation_points, allow_pickle=False) as data:
                heldout = heldout_diagnostics(handeye["camera_to_base"],
                    data["points_camera"], data["points_base"], args.max_error_m)
            heldout["input_file"] = provenance(args.validation_points)
            heldout["calibration_file"] = provenance(source_path)
    result = write_calibration_report(output, intrinsic=intrinsic, handeye=handeye,
                                      heldout=heldout, title=args.title)
    result.update(calibration_applied=False, hardware_commanded=False)
    if intrinsic is not None and intrinsic["accepted_count"] == 0:
        result["status"] = "failed"
        result["reason"] = "no usable chessboard views; inspect the saved report"
    elif heldout is not None and not heldout["passed"]:
        result["status"] = "failed"
        result["reason"] = "held-out point errors exceed the requested tolerance; inspect the saved report"
    else:
        result["status"] = "report_written"
    return result


def demo(output):
    from .camera import capture
    from .vision import segment, estimate
    from .robot import MockRobot
    from .planning import plan_pregrasp
    from .execution import execute_plan
    from .calibration import validate_extrinsics

    profile = load_profile(Path(__file__).parent / "assets/demo.json")
    evidence = validate_extrinsics(profile["calibration"]["camera_to_base"],
        [[0,0,1], [.1,0,1], [0,.1,1]], [[0,0,1], [.1,0,1], [0,.1,1]], .001)
    write_json(Path(output)/"calibration-validation.json", {**evidence, "source": "mock"})
    frame = capture(profile, mock=True)
    mask = segment(profile, frame, {"type": "box", "xyxy": [.25,.25,.75,.75]}, mock=True)
    observation = estimate(profile, frame, mask, profile["vision"]["mesh_id"], mock=True)
    adapter = MockRobot(profile)
    plan = plan_pregrasp(profile, observation, adapter.read_state())
    path = write_json(Path(output)/"pregrasp-plan.json", plan)
    result = execute_plan(plan, profile, adapter, reviewed_plan_id=plan["plan_id"],
                          log_path=Path(output)/f"execution-{time.time_ns()}.jsonl")
    write_json(Path(output)/"result.json", result)
    return {"mode": "mock", "plan": str(path), "result": result}


def main(argv=None):
    parser = argparse.ArgumentParser(description="TRON2 object-aware pregrasp deployment")
    commands = parser.add_subparsers(dest="command", required=True)
    p = commands.add_parser("demo", help="run synthetic calibration-to-pregrasp pipeline")
    p.add_argument("--output", default="output/demo")
    p = commands.add_parser("calibration-report", help="write offline HTML/PNG calibration diagnostics from saved evidence")
    p.add_argument("--intrinsics", help="intrinsic calibration JSON")
    p.add_argument("--images", help="quoted glob for original images; defaults to the intrinsic JSON image list")
    p.add_argument("--handeye", help="fixed-camera eye-to-hand solve JSON")
    p.add_argument("--samples", help="quoted glob for stationary hand-eye sample JSON files")
    p.add_argument("--validation-points", help="NPZ with independent points_camera and points_base")
    p.add_argument("--max-error-m", type=float, help="maximum held-out point error in metres")
    p.add_argument("--title", default="Calibration review", help="report title; label synthetic examples explicitly")
    p.add_argument("--output", required=True, help="new or empty report directory")
    for name in ("operator", "capture", "state", "plan", "execute", "model-hash", "record-sample", "apply-intrinsics", "apply-calibration"):
        p = commands.add_parser(name)
        p.add_argument("--profile", required=True)
        if name in ("operator", "capture", "state", "plan", "execute", "record-sample"):
            p.add_argument("--mock", action="store_true")
        if name in ("capture", "state", "plan", "record-sample", "apply-intrinsics", "apply-calibration"):
            p.add_argument("--output", required=True)
        if name == "operator":
            p.add_argument("--host", default="127.0.0.1")
            p.add_argument("--port", type=int, default=8787)
        if name == "capture":
            p.add_argument("--raw", action="store_true", help="save unrectified color for intrinsic calibration")
        if name == "plan":
            p.add_argument("--observation", required=True)
            p.add_argument("--state", default="live", help="state JSON or 'live' for immediate feedback")
            p.add_argument("--sides", nargs="+", choices=("left", "right"), default=["left", "right"])
        if name == "execute":
            p.add_argument("--plan", required=True)
            p.add_argument("--reviewed-plan-id", required=True)
            p.add_argument("--confirm-supervised", action="store_true")
            p.add_argument("--real", action="store_true")
            p.add_argument("--log", required=True)
        if name == "record-sample":
            p.add_argument("--side", choices=("left", "right"), required=True)
            p.add_argument("--pattern", default="9x6")
            p.add_argument("--square-m", type=float, default=.025)
        if name in ("apply-intrinsics", "apply-calibration"):
            p.add_argument("--intrinsics", required=True)
        if name == "apply-calibration":
            p.add_argument("--handeye", required=True)
            p.add_argument("--validation-points", required=True)
            p.add_argument("--max-error-m", type=float, required=True)
            p.add_argument("--calibration-id", required=True)
    p = commands.add_parser("intrinsics")
    p.add_argument("--images", required=True, help="quoted glob for original camera images")
    p.add_argument("--pattern", default="9x6")
    p.add_argument("--square-m", type=float, default=.025)
    p.add_argument("--output", required=True)
    p = commands.add_parser("handeye")
    p.add_argument("--samples", required=True, help="quoted glob for stationary sample JSON files")
    p.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "calibration-report":
            result = calibration_report_command(args)
        elif args.command == "demo":
            result = demo(args.output)
        elif args.command == "intrinsics":
            from .calibration import intrinsic_fit
            result = intrinsic_fit(sorted(glob.glob(args.images)), tuple(map(int, args.pattern.split("x"))), args.square_m)
            write_json(args.output, result)
        elif args.command == "handeye":
            from .calibration import solve_samples
            result = solve_samples([read_json(path) for path in sorted(glob.glob(args.samples))])
            write_json(args.output, result)
        else:
            profile = load_profile(args.profile)
            if args.command == "operator":
                from .operator import serve
                serve(args.profile, host=args.host, port=args.port, mock=args.mock)
                return 0
            if args.command == "capture":
                from .camera import capture, decode_image
                import cv2
                result = capture(profile, mock=args.mock, undistort=not args.raw)
                output = Path(args.output)
                output.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(output/"color.png"), decode_image(result["image"]))
                cv2.imwrite(str(output/"depth.png"), decode_image(result["depth"]))
                write_json(output/"frame.json", result)
            elif args.command == "state":
                from .robot import MockRobot, WebsocketRobot
                adapter = MockRobot(profile) if args.mock else WebsocketRobot(profile)
                try:
                    result = adapter.read_state()
                finally:
                    adapter.close()
                write_json(args.output, result)
            elif args.command == "plan":
                from .planning import plan_pregrasp
                if args.state == "live":
                    from .robot import MockRobot, WebsocketRobot
                    adapter = MockRobot(profile) if args.mock else WebsocketRobot(profile)
                    try:
                        state = adapter.read_state()
                    finally:
                        adapter.close()
                else:
                    state = read_json(args.state)
                result = plan_pregrasp(profile, read_json(args.observation), state, args.sides)
                write_json(args.output, result)
            elif args.command == "execute":
                from .robot import MockRobot, WebsocketRobot
                from .execution import execute_plan
                if args.mock == args.real:
                    raise ValueError("choose exactly one of --mock or --real")
                adapter = MockRobot(profile) if args.mock else WebsocketRobot(profile)
                stop = threading.Event()
                previous = signal.signal(signal.SIGINT, lambda *_: stop.set())
                try:
                    result = execute_plan(read_json(args.plan), profile, adapter, real=args.real,
                        reviewed_plan_id=args.reviewed_plan_id, supervisor_confirmed=args.confirm_supervised,
                        stop_event=stop, log_path=args.log)
                finally:
                    signal.signal(signal.SIGINT, previous)
                    adapter.close()
            elif args.command == "model-hash":
                from .kinematics import model_hash
                result = {"model_hash": model_hash(profile)}
            elif args.command == "record-sample":
                from .calibration import record_sample
                result = {"sample": str(record_sample(profile, args.side, args.output, mock=args.mock,
                    pattern=tuple(map(int, args.pattern.split("x"))), square_m=args.square_m))}
            elif args.command == "apply-intrinsics":
                intrinsic = read_json(args.intrinsics)
                profile["camera"].update(intrinsics=intrinsic["K"], distortion=intrinsic["dist"],
                                         width=intrinsic["width"], height=intrinsic["height"])
                profile["calibration"]["verified"] = False
                profile["execution"]["allow_real"] = False
                profile.pop("_profile_path", None)
                write_json(args.output, profile)
                result = {"profile": args.output, "calibration_verified": False}
            elif args.command == "apply-calibration":
                import numpy as np
                from .calibration import validate_extrinsics
                intrinsic, handeye = read_json(args.intrinsics), read_json(args.handeye)
                if handeye["mode"] != "eye_to_hand" or handeye["source"] != "real":
                    raise ValueError("live top-camera deployment needs a real eye-to-hand solve")
                if handeye["camera_id"] != profile["camera"]["identity"]:
                    raise ValueError("solved camera identity differs from deployment camera")
                points = np.load(args.validation_points, allow_pickle=False)
                report = validate_extrinsics(handeye["camera_to_base"], points["points_camera"],
                                             points["points_base"], args.max_error_m)
                if not report["passed"]:
                    raise ValueError("held-out extrinsic validation failed")
                profile["camera"].update(intrinsics=intrinsic["K"], distortion=intrinsic["dist"],
                                         width=intrinsic["width"], height=intrinsic["height"])
                profile["calibration"].update(id=args.calibration_id, source="real", verified=True,
                    camera_to_base=handeye["camera_to_base"], head_q2=handeye["head_q2"], validation=report)
                profile["execution"]["allow_real"] = False
                profile.pop("_profile_path", None)
                write_json(args.output, profile)
                result = report
        print(json.dumps(result, indent=2, allow_nan=False))
        if isinstance(result, dict) and result.get("status") in ("failed", "stopped", "rejected"):
            return 1
        return 0
    except (ValueError, RuntimeError, TimeoutError, KeyError, FileNotFoundError) as error:
        parser.exit(1, f"error: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
