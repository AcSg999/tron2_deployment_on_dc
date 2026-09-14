"""A small HTTP workbench for observed-object pregrasp planning.

Startup reads configuration only. Camera, vision and robot access happen after
an explicit operation. Browser execution is limited to the mock robot.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shlex
import threading
import time
from typing import Any
from urllib.parse import urlsplit
import uuid

WEB_ROOT = Path(__file__).with_name("web")
STATIC_FILES = {"/": ("index.html", "text/html; charset=utf-8"),
                "/index.html": ("index.html", "text/html; charset=utf-8"),
                "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                "/workflow.css": ("workflow.css", "text/css; charset=utf-8")}
MAX_REQUEST_BYTES = 1024 * 1024


def _token() -> str:
    return uuid.uuid4().hex


def _require_ref(payload: dict, key: str, expected: str | None) -> None:
    if not expected or not isinstance(payload.get(key), str) or payload[key] != expected:
        raise ValueError(f"{key} is missing or stale; repeat the preceding step")


def _json_copy(value: Any) -> Any:
    return json.loads(json.dumps(value, allow_nan=False))


class OperatorService:
    def __init__(self, profile: dict, mock: bool = False, *,
                 output_dir: str | Path | None = None, profile_path=None):
        from .config import profile_fingerprint
        self.profile = deepcopy(profile)
        self.mock = bool(mock)
        self.profile_path = str(profile_path or profile.get("_profile_path") or "")
        self.fingerprint = profile_fingerprint(self.profile)
        self.output_dir = Path(output_dir or Path.cwd() / "output").resolve()
        self._lock = threading.RLock()
        self._frame = self._mask = self._observation = self._plan = None
        self._plan_path = None
        self._robot_instance = None
        self._job = None
        self._worker = None
        self._stop_event = threading.Event()

    def _check_profile(self):
        from .config import load_profile, profile_fingerprint
        current = load_profile(self.profile_path) if self.profile_path else self.profile
        if profile_fingerprint(current) != self.fingerprint:
            self._clear("frame")
            raise ValueError("profile or calibrated model changed; restart the service with the current profile")

    def _idle(self):
        if self._worker is not None and self._worker.is_alive():
            raise ValueError("mock execution is running; stop it before changing the observation or plan")

    def _clear(self, level):
        self._plan = self._plan_path = None
        if level != "plan":
            self._observation = None
        if level in ("frame", "mask"):
            self._mask = None
        if level == "frame":
            self._frame = None

    def _robot(self):
        if self._robot_instance is None:
            from .robot import MockRobot, WebsocketRobot
            self._robot_instance = MockRobot(self.profile) if self.mock else WebsocketRobot(self.profile)
        return self._robot_instance

    def status(self):
        with self._lock:
            obj = self.profile.get("object", {}).copy()
            obj["mesh_id"] = self.profile.get("vision", {}).get("mesh_id", obj.get("mesh_id"))
            obj["radius_m"] = self.profile.get("scene", {}).get("object_radius_m", obj.get("radius_m"))
            profile_error = None
            try:
                self._check_profile()
            except (ValueError, OSError, KeyError) as error:
                self._clear("frame")
                profile_error = str(error)
            return _json_copy({
                "workflow": "object_pregrasp", "mock": self.mock,
                "profile_fingerprint": self.fingerprint,
                "profile_name": self.profile.get("profile_id", "TRON2"),
                "profile_error": profile_error,
                "object": {key: obj[key] for key in ("mesh_id", "name", "radius_m") if key in obj},
                "pregrasp": self.profile.get("pregrasp", {}),
                "latest_frame_ref": self._frame.get("frame_ref") if self._frame else None,
                "latest_mask_ref": self._mask.get("mask_ref") if self._mask else None,
                "latest_observation_id": self._observation.get("observation_id") if self._observation else None,
                "latest_plan_id": self._plan.get("plan_id") if self._plan else None,
                "job": self._job,
                "capabilities": {"capture": True, "mask": True, "estimate": True,
                                 "plan": True, "mock_execution": self.mock,
                                 "browser_real_execution": False},
            })

    def capture_frame(self, payload=None):
        from .camera import capture
        with self._lock:
            self._idle(); self._check_profile(); self._clear("frame")
            result = _json_copy(capture(self.profile, mock=self.mock))
            result["frame_ref"] = _token()
            result["profile_fingerprint"] = self.fingerprint
            self._frame = result
            return deepcopy(result)

    def segment_mask(self, payload):
        from .vision import segment
        with self._lock:
            self._idle(); self._check_profile()
            _require_ref(payload, "frame_ref", self._frame.get("frame_ref") if self._frame else None)
            if not isinstance(payload.get("prompt"), dict):
                raise ValueError("prompt must be a SAM box or point prompt")
            self._clear("mask")
            result = _json_copy(segment(self.profile, deepcopy(self._frame), payload["prompt"], mock=self.mock))
            result.update(mask_ref=_token(), frame_ref=self._frame["frame_ref"])
            self._mask = result
            return deepcopy(result)

    def estimate_pose(self, payload):
        from .vision import estimate
        with self._lock:
            self._idle(); self._check_profile()
            _require_ref(payload, "frame_ref", self._frame.get("frame_ref") if self._frame else None)
            _require_ref(payload, "mask_ref", self._mask.get("mask_ref") if self._mask else None)
            configured_mesh = self.profile.get("vision", {}).get("mesh_id", self.profile.get("object", {}).get("mesh_id"))
            mesh_id = payload.get("mesh_id", configured_mesh)
            if not isinstance(mesh_id, str) or not mesh_id.strip():
                raise ValueError("mesh_id must be configured")
            if configured_mesh and mesh_id != configured_mesh:
                raise ValueError("mesh_id must match the calibrated object profile")
            self._clear("observation")
            result = _json_copy(estimate(self.profile, deepcopy(self._frame),
                                        deepcopy(self._mask), mesh_id, mock=self.mock))
            result.update(observation_id=_token(), frame_ref=self._frame["frame_ref"],
                          mask_ref=self._mask["mask_ref"], mesh_id=mesh_id,
                          profile_fingerprint=self.fingerprint)
            if result.get("reference_frame") != "base_Link":
                raise ValueError("FoundationPose observation must use base_Link")
            self._observation = result
            return deepcopy(result)

    def plan(self, payload):
        from .planning import plan_pregrasp
        with self._lock:
            self._idle(); self._check_profile()
            _require_ref(payload, "observation_id", self._observation.get("observation_id") if self._observation else None)
            sides = payload.get("sides", ["left", "right"])
            if not isinstance(sides, list) or not sides or len(set(sides)) != len(sides) or any(side not in ("left", "right") for side in sides):
                raise ValueError("sides must select left, right, or both once")
            self._clear("plan")
            measured = self._robot().read_state()
            result = _json_copy(plan_pregrasp(self.profile, deepcopy(self._observation), measured, sides=tuple(sides)))
            if not isinstance(result.get("plan_id"), str) or not result["plan_id"]:
                raise ValueError("planner returned no plan_id")
            # The planner owns the plan digest. Persist exactly those bytes/data.
            self.output_dir.mkdir(parents=True, exist_ok=True)
            path = self.output_dir / f"pregrasp_{_token()}.json"
            temporary = path.with_suffix(".tmp")
            temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
            temporary.replace(path)
            self._plan = result
            self._plan_path = path
            return self._plan_response()

    def _plan_response(self):
        args = ["python", "-m", "tron2_deployment.rviz", "--profile", self.profile_path or "configs/robot.json",
                "--plan", str(self._plan_path), "--ros-master-uri", "http://127.0.0.1:11331"]
        return {"plan": deepcopy(self._plan), "plan_path": str(self._plan_path),
                "rviz_command": shlex.join(args)}

    def mock_execute(self, payload):
        if not self.mock:
            raise PermissionError("browser execution is mock-only; use the reviewed CLI flow for real hardware")
        with self._lock:
            self._idle(); self._check_profile()
            _require_ref(payload, "plan_id", self._plan.get("plan_id") if self._plan else None)
            _require_ref(payload, "reviewed_plan_id", self._plan["plan_id"])
            plan = deepcopy(self._plan)
            self._stop_event = threading.Event()
            job_id = _token()
            self._job = {"job_id": job_id, "state": "running", "plan_id": plan["plan_id"], "mock": True}
            self._worker = threading.Thread(target=self._run_mock, args=(plan, payload["reviewed_plan_id"], job_id), daemon=True)
            self._worker.start()
            return deepcopy(self._job)

    def _run_mock(self, plan, reviewed_plan_id, job_id):
        try:
            from .execution import execute_plan
            report = execute_plan(plan, self.profile, self._robot(), real=False,
                                  reviewed_plan_id=reviewed_plan_id, stop_event=self._stop_event,
                                  log_path=self.output_dir / f"mock_{job_id}.jsonl")
            with self._lock:
                self._job = {"job_id": job_id, "state": report.get("status", "failed"),
                             "plan_id": plan["plan_id"], "mock": True, "report": _json_copy(report)}
        except Exception as error:
            with self._lock:
                self._job = {"job_id": job_id, "state": "failed", "plan_id": plan["plan_id"], "mock": True, "error": str(error)}

    def stop(self, payload=None):
        if not self.mock:
            raise PermissionError("browser stop applies only to browser mock execution")
        self._stop_event.set()
        with self._lock:
            return {"stop_requested": True, "job": deepcopy(self._job)}

    def close(self):
        self._stop_event.set()
        if self._worker is not None:
            self._worker.join(timeout=3)
        if self._robot_instance is not None:
            close = getattr(self._robot_instance, "close", None)
            if close:
                close()


POST_ROUTES = {"/api/capture": "capture_frame", "/api/mask": "segment_mask",
               "/api/estimate": "estimate_pose", "/api/plan": "plan",
               "/api/mock-execute": "mock_execute", "/api/stop": "stop"}


def make_handler(service):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def _reply(self, data, status=200):
            body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path == "/api/status":
                return self._reply(service.status())
            entry = STATIC_FILES.get(self.path)
            if entry is None:
                return self._reply({"error": "route not found"}, 404)
            content = (WEB_ROOT / entry[0]).read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", entry[1])
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; connect-src 'self'; style-src 'self'; script-src 'self'; object-src 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(content)

        def do_POST(self):
            method = POST_ROUTES.get(self.path)
            if method is None:
                return self._reply({"error": "route not found"}, 404)
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                return self._reply({"error": "cross-origin requests are not allowed"}, 403)
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size <= 0 or size > MAX_REQUEST_BYTES:
                    return self._reply({"error": "request body must be JSON below 1 MiB"}, 413)
                if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    return self._reply({"error": "Content-Type must be application/json"}, 415)
                payload = json.loads(self.rfile.read(size), parse_constant=lambda _: (_ for _ in ()).throw(ValueError("non-finite JSON is not allowed")))
                if not isinstance(payload, dict):
                    raise ValueError("request JSON must be an object")
                return self._reply(getattr(service, method)(payload))
            except PermissionError as error:
                return self._reply({"error": str(error)}, 403)
            except (ValueError, KeyError, TypeError) as error:
                return self._reply({"error": str(error)}, 400)
            except Exception as error:
                return self._reply({"error": str(error)}, 503)
    return Handler


def make_server(service, host="127.0.0.1", port=8787):
    if host not in ("127.0.0.1", "localhost", "::1"):
        raise ValueError("operator binds to loopback; use an SSH tunnel for remote access")
    return ThreadingHTTPServer((host, port), make_handler(service))


def serve(profile_path, host="127.0.0.1", port=8787, mock=False, output_dir=None):
    from .config import load_profile
    service = OperatorService(load_profile(profile_path), mock=mock,
                              profile_path=profile_path, output_dir=output_dir)
    server = make_server(service, host, port)
    print(f"TRON2 pregrasp workbench: http://{host}:{server.server_port}/ ({'mock' if mock else 'observe / plan; real execution via CLI'})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        service.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--output-dir")
    args = parser.parse_args(argv)
    serve(args.profile, args.host, args.port, args.mock, args.output_dir)


if __name__ == "__main__":
    main()
