"""Standalone operator tests use mock perception/robots and local HTTP only."""
from copy import deepcopy
import http.client
import json
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from tron2_deployment import camera, execution, planning, vision
from tron2_deployment.config import load_profile, profile_fingerprint
from tron2_deployment.operator import OperatorService, make_server
from tron2_deployment.rviz import sample_plan, validate_master

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def service(tmp_path, monkeypatch):
    profile = load_profile(ROOT / "configs/demo.json")
    profile.pop("_profile_path", None)
    item = OperatorService(profile, mock=True, output_dir=tmp_path)
    monkeypatch.setattr(camera, "capture", lambda profile, mock=False: {
        "frame_ref": "camera-id", "image": "frame", "depth": "depth",
        "reference_frame": "base_Link", "source": "mock", "capture_timestamp_s": time.time()})
    monkeypatch.setattr(vision, "segment", lambda profile, frame, prompt, mock=False: {
        "frame_ref": frame["frame_ref"], "mask_ref": "segmentation-id", "mask": "mask", "area_px": 10})
    monkeypatch.setattr(vision, "estimate", lambda profile, frame, mask, mesh_id, mock=False: {
        "reference_frame": "base_Link", "pose7": [0, 0, .5, 1, 0, 0, 0],
        "source": "mock", "confidence": .9, "capture_timestamp_s": frame["capture_timestamp_s"]})
    yield item
    item.close()


def observe(service):
    frame = service.capture_frame({})
    mask = service.segment_mask({"frame_ref": frame["frame_ref"], "prompt": {"type": "box", "xyxy": [.2, .2, .8, .8]}})
    pose = service.estimate_pose({"frame_ref": frame["frame_ref"], "mask_ref": mask["mask_ref"], "mesh_id": service.profile["vision"]["mesh_id"]})
    return frame, mask, pose


def fake_plan(profile, observation, state, sides=("left", "right")):
    plan = {"schema_version": 1, "mode": "pregrasp", "interpolation": "quintic_stop",
            "profile_hash": profile_fingerprint(profile), "source": {"observation": observation, "state": state},
            "selected_sides": list(sides), "times_s": [0., 1.],
            "left_arm_qpos": [[0.] * 7, [.1] * 7], "right_arm_qpos": [[0.] * 7, [-.1] * 7],
            "head_qpos": [[0., 0.], [0., 0.]], "targets": {}, "checks": {}}
    plan["plan_id"] = planning.plan_digest(plan)
    return plan


def test_idle_start_has_no_camera_vision_or_robot_access(service, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("idle startup accessed hardware or vision")
    monkeypatch.setattr(camera, "capture", forbidden)
    monkeypatch.setattr(vision, "segment", forbidden)
    monkeypatch.setattr(vision, "estimate", forbidden)
    monkeypatch.setattr(service, "_robot", forbidden)
    status = service.status()
    assert status["workflow"] == "object_pregrasp"
    assert status["latest_frame_ref"] is None
    assert status["object"]["mesh_id"] == service.profile["vision"]["mesh_id"]
    assert not service.output_dir.exists() or not list(service.output_dir.iterdir())


def test_capture_tokens_are_unique_and_clear_all_downstream(service):
    frame, mask, pose = observe(service)
    service._plan = {"plan_id": "old"}
    newer = service.capture_frame({})
    assert newer["frame_ref"] != frame["frame_ref"]
    assert service._mask is service._observation is service._plan is None
    with pytest.raises(ValueError, match="stale"):
        service.segment_mask({"frame_ref": frame["frame_ref"], "prompt": {}})
    with pytest.raises(ValueError, match="stale"):
        service.plan({"observation_id": pose["observation_id"]})


def test_mask_token_prevents_old_or_other_tab_registration(service):
    frame, mask, _ = observe(service)
    new = service.segment_mask({"frame_ref": frame["frame_ref"], "prompt": {"type": "box"}})
    assert new["mask_ref"] != mask["mask_ref"]
    with pytest.raises(ValueError, match="mask_ref"):
        service.estimate_pose({"frame_ref": frame["frame_ref"], "mask_ref": mask["mask_ref"]})


def test_client_response_mutation_cannot_alter_server_observation(service):
    frame, mask, pose = observe(service)
    frame["source"] = "real"; mask["mask"] = "changed"; pose["pose7"][0] = 99
    assert service._frame["source"] == "mock"
    assert service._mask["mask"] == "mask"
    assert service._observation["pose7"][0] == 0


def test_mismatched_profile_mesh_rejected(service):
    frame, mask, _ = observe(service)
    with pytest.raises(ValueError, match="mesh_id"):
        service.estimate_pose({"frame_ref": frame["frame_ref"], "mask_ref": mask["mask_ref"], "mesh_id": "another-object"})


def test_failed_new_mask_discards_previous_pose_and_plan(service, monkeypatch):
    frame, _, _ = observe(service)
    service._plan = {"plan_id": "old"}
    def fail(*args, **kwargs):
        raise RuntimeError("SAM unavailable")
    monkeypatch.setattr(vision, "segment", fail)
    with pytest.raises(RuntimeError, match="SAM unavailable"):
        service.segment_mask({"frame_ref": frame["frame_ref"], "prompt": {}})
    assert service._observation is service._plan is service._mask is None


def test_plan_reads_state_and_exports_exact_signed_content(service, monkeypatch):
    _, _, pose = observe(service)
    monkeypatch.setattr(planning, "plan_pregrasp", fake_plan)
    response = service.plan({"observation_id": pose["observation_id"], "sides": ["left", "right"]})
    saved = json.loads(Path(response["plan_path"]).read_text())
    assert saved == response["plan"]
    assert saved["plan_id"] == planning.plan_digest(saved)
    assert saved["source"]["state"]["source"] == "mock"
    assert saved["source"]["observation"]["observation_id"] == pose["observation_id"]
    assert "tron2_deployment.rviz" in response["rviz_command"]
    assert "11331" in response["rviz_command"]
    assert service._robot_instance.commands == []


@pytest.mark.parametrize("outcome", ["completed", "rejected", "failed", "stopped"])
def test_mock_job_preserves_executor_status_and_argument_order(service, monkeypatch, outcome):
    _, _, pose = observe(service)
    monkeypatch.setattr(planning, "plan_pregrasp", fake_plan)
    plan = service.plan({"observation_id": pose["observation_id"]})["plan"]
    seen = []
    def execute(plan_arg, profile_arg, adapter, **kwargs):
        assert plan_arg == plan
        assert profile_arg == service.profile
        assert kwargs["real"] is False
        assert kwargs["reviewed_plan_id"] == plan["plan_id"]
        seen.append(True)
        return {"status": outcome, "hardware_commanded": False}
    monkeypatch.setattr(execution, "execute_plan", execute)
    service.mock_execute({"plan_id": plan["plan_id"], "reviewed_plan_id": plan["plan_id"]})
    service._worker.join(2)
    assert seen and service.status()["job"]["state"] == outcome


def test_mock_requires_current_plan_review_and_real_browser_is_rejected(service):
    service._plan = {"plan_id": "current"}
    with pytest.raises(ValueError, match="reviewed_plan_id"):
        service.mock_execute({"plan_id": "current", "reviewed_plan_id": "old"})
    service.mock = False
    with pytest.raises(PermissionError, match="mock-only"):
        service.mock_execute({"plan_id": "current", "reviewed_plan_id": "current"})
    assert service._robot_instance is None


def test_profile_change_invalidates_observation_on_status(service):
    observe(service)
    service.profile["calibration"]["head_q2"] = [.1, 0]
    status = service.status()
    assert status["profile_error"]
    assert status["latest_frame_ref"] is None
    with pytest.raises(ValueError, match="profile"):
        service.capture_frame({})


def test_http_allowlist_idle_start_port_conflict_and_no_real_route(service, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("HTTP idle route accessed hardware")
    monkeypatch.setattr(service, "_robot", forbidden)
    server = make_server(service, port=0)
    worker = threading.Thread(target=server.serve_forever, daemon=True); worker.start()
    connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=3)
    try:
        for path in ("/", "/app.js", "/workflow.css", "/api/status"):
            connection.request("GET", path); response=connection.getresponse(); assert response.status == 200; response.read()
        for path in ("/hand.html", "/api/execute", "/../config.py", "/api/status?unused=1"):
            connection.request("GET", path); response=connection.getresponse(); assert response.status == 404; response.read()
        connection.request("POST", "/api/execute", "{}", {"Content-Type":"application/json"})
        response=connection.getresponse(); assert response.status == 404; response.read()
        connection.request("POST", "/api/capture", "{}", {"Content-Type":"application/json", "Origin":"http://different.invalid"})
        response=connection.getresponse(); assert response.status == 403; response.read()
        with pytest.raises(OSError): make_server(service, port=server.server_port)
    finally:
        connection.close(); server.shutdown(); server.server_close(); worker.join(2)


def test_static_page_has_only_pregrasp_flow():
    root=ROOT/"src/tron2_deployment/web"
    html=(root/"index.html").read_text(); script=(root/"app.js").read_text()
    assert 'lang="zh-CN"' in html
    for legacy in ("Gaia20", "gripper", "getUserMedia", "/api/execute", "wave4", "iframe"):
        assert legacy not in html + script
    assert 'coordinates:"pixels"' in script
    assert "reviewed_plan_id" in script
    assert "new AbortController" in script


def test_rviz_interpolates_exact_plan_timing_and_requires_separate_master():
    plan={"interpolation":"quintic_stop", "times_s":[0,2],
          "left_arm_qpos":[[0]*7,[1]*7],"right_arm_qpos":[[0]*7,[-1]*7],"head_qpos":[[.1,.2],[.1,.2]]}
    np.testing.assert_allclose(sample_plan(plan,1),[.5]*7+[-.5]*7+[.1,.2])
    np.testing.assert_allclose(sample_plan(plan,9),[1]*7+[-1]*7+[.1,.2])
    np.testing.assert_allclose(sample_plan(plan,0),[0]*14+[.1,.2])
    assert validate_master("http://127.0.0.1:11331",{}) == 11331
    with pytest.raises(ValueError): validate_master("http://robot:11311",{})
    with pytest.raises(ValueError): validate_master("http://127.0.0.1:11311",{})
    with pytest.raises(ValueError): validate_master("http://127.0.0.1:11331",{"camera":{"ros_master_uri":"http://127.0.0.1:11331"}})
