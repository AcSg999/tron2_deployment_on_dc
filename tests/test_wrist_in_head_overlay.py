import asyncio
import json
from pathlib import Path
import struct
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tools.hardware import wrist_in_head_overlay as overlay


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "sp_vision/configs/wrist_config.example.json"


@pytest.fixture
def calibration_files(tmp_path):
    config = overlay.wrist.load_config(CONFIG)
    model = overlay.common.robot_kinematics(config)
    intrinsic = {"passed": True, "image_size": [640, 480],
                 "camera_matrix": [[430, 0, 320], [0, 430, 240], [0, 0, 1]],
                 "distortion": [0.] * 5}
    head_ext = {"passed": True, "kind": "sp_vision_pitch_camera_extrinsics",
                "pitch_link": "head_pitch_Link",
                "head_joint_names": ["head_pitch_Joint", "head_yaw_Joint"],
                "T_pitch_camera": model.matrix("head_pitch_Link", "head_camera_color_optical_frame", {}).tolist()}
    wrist_ext = {"passed": True, "kind": "sp_vision_right_wrist_extrinsics",
                 "mount_link": "wrist_roll_R_Link", "camera_frame": "right_wrist_camera_color_optical_frame",
                 "T_wrist_roll_camera": overlay.wrist.nominal_matrix(config, model).tolist()}
    summary = {"schema_version": 1, "cameras": {}}
    for camera, ext in (("head", head_ext), ("right_wrist", wrist_ext)):
        record = {"source_files": {}, "source_sha256": {}}
        for kind, value in (("intrinsics", intrinsic), ("extrinsics", ext)):
            p = tmp_path / f"{camera}_{kind}.json"
            p.write_text(json.dumps(value))
            record["source_files"][kind] = str(p)
            record["source_sha256"][kind] = overlay.sha256(p)
        summary["cameras"][camera] = record
    path = tmp_path / "summary.json"
    path.write_text(json.dumps(summary))
    return path


def test_projection_uses_inverse_head_transform_and_raw_distortion():
    head = np.eye(4)
    head[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    head[:3, 3] = [1, 2, 3]
    wrist = np.eye(4)
    wrist[:3, 3] = [1, 3, 5]
    k = np.array([[100., 0, 320], [0, 100, 240], [0, 0, 1]])
    result = overlay.project_center(wrist, head, k, np.zeros(5), (640, 480))
    assert result["wrist_position_head_m"] == pytest.approx([1, 0, 2])
    assert result["pixel_uv"] == pytest.approx([370, 240])
    assert result["visible"]
    distorted = overlay.project_center(wrist, head, k, np.array([.2, 0, 0, 0, 0]), (640, 480))
    assert distorted["pixel_uv"] == pytest.approx([372.5, 240])
    # Optical-origin projection cannot observe the wrist camera's rotation.
    wrist[:3, :3] = Rotation.from_euler("xyz", [.6, -.7, 1.2]).as_matrix()
    assert overlay.project_center(wrist, head, k, np.zeros(5), (640, 480))["pixel_uv"] == pytest.approx([370, 240])


def test_axes_respond_to_rotation_while_origin_stays_fixed():
    wrist = np.eye(4)
    wrist[:3, 3] = [0, 0, 2]
    k = np.array([[100., 0, 320], [0, 100, 240], [0, 0, 1]])
    initial = overlay.project_axes(wrist, np.eye(4), k, np.zeros(5), (640, 480), .1)
    assert initial["axes"]["X"]["pixel_uv"] == pytest.approx([325, 240])
    assert initial["axes"]["Y"]["pixel_uv"] == pytest.approx([320, 245])
    wrist[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    rotated = overlay.project_axes(wrist, np.eye(4), k, np.zeros(5), (640, 480), .1)
    assert rotated["axes"]["X"]["pixel_uv"] == pytest.approx([320, 245])
    assert rotated["axes"]["Y"]["pixel_uv"] == pytest.approx([315, 240])
    assert rotated["axes"]["Z"]["pixel_uv"] == pytest.approx([320, 240])
    assert np.array(rotated["R_head_wrist_camera"]) == pytest.approx(wrist[:3, :3])
    assert overlay.project_center(wrist, np.eye(4), k, np.zeros(5), (640, 480))["pixel_uv"] == pytest.approx([320, 240])


def test_axes_use_relative_head_rotation_and_projected_distortion():
    head = np.eye(4)
    head[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    head[:3, 3] = [1, 2, 3]
    wrist = np.eye(4)
    wrist[:3, 3] = [1, 3, 5]
    k = np.array([[100., 0, 320], [0, 100, 240], [0, 0, 1]])
    result = overlay.project_axes(wrist, head, k, np.array([.2, 0, 0, 0, 0]), (640, 480), .1)
    assert result["axes"]["X"]["endpoint_head_m"] == pytest.approx([1, -.1, 2])
    # x=0.5, y=-0.05; radial factor = 1 + 0.2 * (0.5**2 + 0.05**2).
    assert result["axes"]["X"]["pixel_uv"] == pytest.approx([372.525, 234.7475])


def test_axis_clipping_does_not_draw_in_the_diagnostics_panel():
    wrist = np.eye(4)
    wrist[2, 3] = .01
    k = np.array([[100., 0, 320], [0, 100, 240], [0, 0, 1]])
    center = overlay.project_center(wrist, np.eye(4), k, np.zeros(5), (640, 480))
    axes = overlay.project_axes(wrist, np.eye(4), k, np.zeros(5), (640, 480), .5)
    assert axes["axes"]["X"]["clipped"]
    assert axes["axes"]["X"]["segment_uv"] == [[320, 240], [639, 240]]
    assert axes["axes"]["Y"]["segment_uv"] == [[320, 240], [320, 479]]
    raw = np.zeros((480, 640, 3), np.uint8)
    record = {**center, **axes, "source": "mock", "state_skew_ms": 0}
    drawn = overlay.render(raw, record)
    assert drawn[240, 600, 2] > 200  # X is red.
    assert drawn[450, 320, 1] > 200  # Y is green.
    # Removing axes leaves the diagnostics area byte-identical.
    without = overlay.render(raw, {**record, "axes": {}})
    assert np.array_equal(drawn[480:], without[480:])


def test_behind_camera_axis_endpoint_is_not_drawable():
    wrist = np.eye(4)
    wrist[:3, :3] = Rotation.from_euler("y", 180, degrees=True).as_matrix()
    wrist[2, 3] = .05
    k = np.array([[100., 0, 320], [0, 100, 240], [0, 0, 1]])
    result = overlay.project_axes(wrist, np.eye(4), k, np.zeros(5), (640, 480), .1)
    assert result["axes"]["Z"]["status"] == "behind_camera"
    assert not result["axes"]["Z"]["drawable"]
    assert result["axes"]["Z"]["pixel_uv"] is None
    assert result["axes"]["X"]["drawable"]


@pytest.mark.parametrize("length", [0., -.1, float("nan"), float("inf")])
def test_invalid_axis_lengths_are_rejected(length):
    with pytest.raises(ValueError, match="axis length"):
        overlay.project_axes(np.eye(4), np.eye(4), np.eye(3), np.zeros(5), (640, 480), length)


@pytest.mark.parametrize("point,status", [([0, 0, -1], "behind_camera"),
                                         ([0, 0, 0], "behind_camera"),
                                         ([100, 0, 1], "outside_image")])
def test_invalid_visibility_never_draws_a_predicted_point(point, status):
    wrist = np.eye(4)
    wrist[:3, 3] = point
    k = np.array([[100., 0, 320], [0, 100, 240], [0, 0, 1]])
    result = overlay.project_center(wrist, np.eye(4), k, np.zeros(5), (640, 480))
    assert result["status"] == status and not result["visible"]
    axes = overlay.project_axes(wrist, np.eye(4), k, np.zeros(5), (640, 480), .08)
    assert all(not axis["drawable"] for axis in axes["axes"].values())
    raw = np.zeros((480, 640, 3), np.uint8)
    record = {**result, **axes, "source": "mock", "state_skew_ms": 0}
    assert not overlay.render(raw, record)[:480].any()


def test_matching_uses_image_time_and_hides_stale_or_unsynchronized_data():
    image = overlay.Sample(1000, 5., "RGB")
    states = [overlay.Sample(1002, 5., "near"), overlay.Sample(1050, 5., "newest")]
    assert overlay.select_pair([image], states, 5.1, 750, 30)[1].value == "near"
    assert overlay.select_pair([image], states[1:], 5.1, 750, 30) is None
    assert overlay.select_pair([image], states, 6., 750, 30) is None
    assert overlay.select_pair([image], [overlay.Sample(1002, 4., "old")], 5.1, 750, 30) is None


def test_named_mapping_and_moving_head_are_used(calibration_files):
    calibration = overlay.Calibration(calibration_files, CONFIG)
    image, record = overlay.mock_capture(calibration)
    joints = dict(reversed(list(record["joints"].items())))
    assert calibration.positions(joints)["proximal_pitch_R_Joint"] == .8
    initial = calibration.evaluate(image, joints)
    joints["head_yaw_Joint"] += .25
    moved = calibration.evaluate(image, joints)
    assert moved["wrist_position_base_m"] == pytest.approx(initial["wrist_position_base_m"])
    assert not np.allclose(moved["T_base_head_camera"], initial["T_base_head_camera"])
    assert moved["pixel_uv"] != pytest.approx(initial["pixel_uv"])
    with pytest.raises(ValueError, match="image size"):
        calibration.evaluate(image[:, :600], joints)
    del joints["hip_R_Joint"]
    with pytest.raises(ValueError, match="lacks"):
        calibration.evaluate(image, joints)


def test_modified_calibration_is_rejected(calibration_files):
    summary = json.loads(calibration_files.read_text())
    path = Path(summary["cameras"]["head"]["source_files"]["extrinsics"])
    value = json.loads(path.read_text())
    value["T_pitch_camera"][0][3] += .01
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="hash changed"):
        overlay.Calibration(calibration_files, CONFIG)


def test_default_local_binding_is_used_and_saved_without_connecting(calibration_files, tmp_path, monkeypatch):
    import websockets
    summary = json.loads(calibration_files.read_text())
    summary["calibration_id"] = "new-head-and-pinned-wrist"
    calibration_files.write_text(json.dumps(summary))
    monkeypatch.setattr(overlay, "DEFAULT_CALIBRATION_SUMMARY", calibration_files)
    def forbidden(*args, **kwargs):
        raise AssertionError("mock must not connect")
    monkeypatch.setattr(websockets, "connect", forbidden)
    output = tmp_path / "default-binding"
    assert overlay.main(["--output", str(output), "mock"]) == 0
    record = json.loads((output / "view-001/projection.json").read_text())
    assert record["calibration_id"] == "new-head-and-pinned-wrist"
    assert record["calibration_summary"] == str(calibration_files.resolve())


def test_missing_default_binding_does_not_fall_back_to_older_data(tmp_path, monkeypatch, capsys):
    missing = tmp_path / "missing-latest.json"
    monkeypatch.setattr(overlay, "DEFAULT_CALIBRATION_SUMMARY", missing)
    output = tmp_path / "not-created"
    assert overlay.main(["--output", str(output), "mock"]) == 1
    assert not output.exists()
    assert "provide --calibration-summary PATH" in capsys.readouterr().err


def test_head_intrinsics_and_extrinsics_must_belong_together(calibration_files):
    summary = json.loads(calibration_files.read_text())
    path = Path(summary["cameras"]["head"]["source_files"]["extrinsics"])
    value = json.loads(path.read_text())
    value["intrinsics_sha256"] = "incorrect-source-intrinsics"
    path.write_text(json.dumps(value))
    summary["cameras"]["head"]["source_sha256"]["extrinsics"] = overlay.sha256(path)
    calibration_files.write_text(json.dumps(summary))
    with pytest.raises(ValueError, match="head extrinsics were solved with different intrinsics"):
        overlay.Calibration(calibration_files, CONFIG)


def test_head_solve_model_must_match_the_tracking_model(calibration_files, tmp_path):
    manifest = tmp_path / "head-input-manifest.json"
    manifest.write_text(json.dumps({"solver_files": {"urdf": {"sha256": "different-urdf"}}}))
    summary = json.loads(calibration_files.read_text())
    path = Path(summary["cameras"]["head"]["source_files"]["extrinsics"])
    value = json.loads(path.read_text())
    value["input_manifest"] = str(manifest)
    path.write_text(json.dumps(value))
    summary["cameras"]["head"]["source_sha256"]["extrinsics"] = overlay.sha256(path)
    calibration_files.write_text(json.dumps(summary))
    with pytest.raises(ValueError, match="URDF differs from the head solve"):
        overlay.Calibration(calibration_files, CONFIG)


def _declare_head_chain(calibration_files, source_urdf, mount="head_pitch_Link"):
    value = json.loads(calibration_files.read_text())
    model = overlay.common.UrdfKinematics(source_urdf)
    value["cameras"]["head"]["kinematic_chain"] = {
        "base_link": "base_Link", "mount_link": mount,
        "signature_sha256": overlay.fk_chain_signature(model, "base_Link", mount),
        "source_urdf_sha256": overlay.sha256(source_urdf)}
    calibration_files.write_text(json.dumps(value))


def test_source_nominal_camera_mount_change_preserves_compatible_fk(calibration_files, tmp_path):
    cfg = overlay.wrist.load_config(CONFIG)
    tracking = overlay.common.config_path(cfg, cfg["robot"]["urdf"])
    model = overlay.common.UrdfKinematics(tracking)
    current = "head_camera_color_optical_frame"
    while model.by_child[current]["parent"] != "head_pitch_Link":
        current = model.by_child[current]["parent"]
    name = model.by_child[current]["name"]
    tree = ET.parse(tracking)
    origin = tree.getroot().find(f"./joint[@name='{name}']/origin")
    assert origin is not None
    xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    xyz[0] += .1
    origin.set("xyz", " ".join(map(str, xyz)))
    source = tmp_path / "different-nominal-camera.urdf"; tree.write(source)
    assert overlay.sha256(source) != overlay.sha256(tracking)
    _declare_head_chain(calibration_files, source)
    cal = overlay.Calibration(calibration_files, CONFIG)
    assert overlay.fk_chain_signature(cal.model, "base_Link", "head_pitch_Link") == overlay.fk_chain_signature(
        overlay.common.UrdfKinematics(source), "base_Link", "head_pitch_Link")


def test_changed_relevant_head_fk_is_rejected_even_with_chain_metadata(calibration_files, tmp_path):
    cfg = overlay.wrist.load_config(CONFIG)
    tracking = overlay.common.config_path(cfg, cfg["robot"]["urdf"])
    tree = ET.parse(tracking)
    origin = tree.getroot().find("./joint[@name='head_pitch_Joint']/origin")
    assert origin is not None
    xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    xyz[0] += .01
    origin.set("xyz", " ".join(map(str, xyz)))
    changed = tmp_path / "different-head-fk.urdf"; tree.write(changed)
    _declare_head_chain(calibration_files, changed)
    with pytest.raises(ValueError, match="FK chain differs from the head calibration"):
        overlay.Calibration(calibration_files, CONFIG)


def test_head_chain_metadata_cannot_substitute_another_mount(calibration_files):
    cfg = overlay.wrist.load_config(CONFIG)
    tracking = overlay.common.config_path(cfg, cfg["robot"]["urdf"])
    _declare_head_chain(calibration_files, tracking, "wrist_roll_R_Link")
    with pytest.raises(ValueError, match="unexpected head FK chain frames"):
        overlay.Calibration(calibration_files, CONFIG)


def test_manifest_pair_hash_is_checked_for_unmodified_external_results(calibration_files):
    value = json.loads(calibration_files.read_text())
    value["cameras"]["head"]["intrinsics_sha256_for_extrinsics"] = "different-source-intrinsics"
    calibration_files.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="head extrinsics were solved with different intrinsics"):
        overlay.Calibration(calibration_files, CONFIG)


def test_selected_feature_residual_is_observed_minus_predicted(calibration_files):
    calibration = overlay.Calibration(calibration_files, CONFIG)
    image, record = overlay.mock_capture(calibration)
    target = np.asarray(record["pixel_uv"]) + [12, -5]
    result = overlay.add_reference(record, target)
    assert result["manual_reference"]["observed_minus_predicted_px"] == pytest.approx([12, -5])
    assert result["manual_reference"]["distance_px"] == pytest.approx(13)
    assert result["manual_reference"]["image_stamp_ms"] == record["image_stamp_ms"]
    assert not result["accuracy_verified"]
    assert not result["manual_reference"]["optical_center_equivalence_verified"]
    assert "manual_reference" not in record
    result = overlay.add_reference({**record, "pixel_uv": None}, target)
    assert result["manual_reference"]["distance_px"] is None


def test_freeze_preserves_exact_image_and_feedback(calibration_files):
    calibration = overlay.Calibration(calibration_files, CONFIG)
    image, record = overlay.mock_capture(calibration)
    review = overlay.FrameReview()
    expected = image.copy()
    original_joint = record["joints"][calibration.arm_names[0]]
    review.toggle(image, record)
    image[:] = 0
    record["joints"][calibration.arm_names[0]] += 1
    assert np.array_equal(review.image, expected)
    assert review.record["joints"][calibration.arm_names[0]] == original_joint
    review.mouse(cv2.EVENT_LBUTTONDOWN, 100, 200, 0, None)
    assert review.record["manual_reference"]["pixel_uv"] == [100, 200]
    assert review.record["display_frozen"]
    review.toggle(image, record)
    assert review.record is None and review.image is None
    assert review.displayed(image, record)[1] is record


@pytest.mark.parametrize("pixel", [[-1, 0], [640, 0], [0, 480], [float("nan"), 20]])
def test_reference_must_be_inside_rgb(calibration_files, pixel):
    calibration = overlay.Calibration(calibration_files, CONFIG)
    image, record = overlay.mock_capture(calibration)
    with pytest.raises(ValueError, match="reference pixel"):
        overlay.add_reference(record, pixel)


def test_reference_click_requires_frozen_rgb_not_diagnostics(calibration_files):
    calibration = overlay.Calibration(calibration_files, CONFIG)
    image, record = overlay.mock_capture(calibration)
    review = overlay.FrameReview()
    review.mouse(cv2.EVENT_LBUTTONDOWN, 100, 200, 0, None)
    assert review.record is None
    review.toggle(image, record)
    review.mouse(cv2.EVENT_LBUTTONDOWN, 100, 500, 0, None)
    review.mouse(cv2.EVENT_MOUSEMOVE, 100, 200, 0, None)
    assert "manual_reference" not in review.record


def test_replay_preserves_reference_but_recomputes_its_residual(calibration_files, tmp_path):
    output = tmp_path / "with-reference"
    args = ["--calibration-summary", str(calibration_files), "--output", str(output)]
    assert overlay.main(args + ["mock"]) == 0
    path = output / "view-001/projection.json"
    old = json.loads(path.read_text())
    old = overlay.add_reference(old, np.asarray(old["pixel_uv"]) + [12, -5])
    old["manual_reference"]["distance_px"] = 999
    path.write_text(json.dumps(old))
    args[args.index(str(output))] = str(tmp_path / "reference-replay")
    assert overlay.main(args + ["replay", "--capture", str(output / "view-001")]) == 0
    new = json.loads((tmp_path / "reference-replay/view-001/projection.json").read_text())
    assert new["manual_reference"]["pixel_uv"] == old["manual_reference"]["pixel_uv"]
    assert new["manual_reference"]["distance_px"] == pytest.approx(13)


@pytest.mark.parametrize("invalid", [None, 0, True, float("nan")])
def test_joint_feedback_requires_valid_explicit_timestamp(invalid):
    raw = {"type": "joint_state", "timestamp": invalid,
           "data": {"names": ["a"], "positions": [0]}}
    with pytest.raises(ValueError, match="timestamp"):
        overlay.joint_sample(json.dumps(raw), 0.)


def test_mock_and_replay_never_connect_and_verify_image_binding(calibration_files, tmp_path, monkeypatch):
    import websockets
    def forbidden(*args, **kwargs):
        raise AssertionError("mock/replay must not open a connection")
    monkeypatch.setattr(websockets, "connect", forbidden)
    output = tmp_path / "mock"
    args = ["--calibration-summary", str(calibration_files), "--output", str(output)]
    assert overlay.main(args + ["mock"]) == 0
    record = json.loads((output / "view-001/projection.json").read_text())
    assert record["source"] == "mock" and not record["accuracy_verified"]
    assert record["validation_scope"] == "wrist_camera_position_and_orientation_overlay"
    assert set(record["axes"]) == {"X", "Y", "Z"}
    assert record["axis_length_m"] == .08
    assert record["visible"]
    image = cv2.imread(str(output / "view-001/overlay.png"))
    u, v = np.rint(record["pixel_uv"]).astype(int)
    assert image[v, u, 2] > 200  # predicted red marker remains visible in the RGB area
    replay = tmp_path / "replay"
    args[args.index(str(output))] = str(replay)
    assert overlay.main(args + ["replay", "--capture", str(output / "view-001")]) == 0
    result = json.loads((replay / "view-001/projection.json").read_text())
    assert result["source"] == "replay" and result["original_source"] == "mock"
    assert result["pixel_uv"] == pytest.approx(record["pixel_uv"])
    assert result["axes"] == record["axes"]
    (output / "view-001/color.png").write_bytes(b"changed image")
    args[args.index(str(replay))] = str(tmp_path / "bad")
    assert overlay.main(args + ["replay", "--capture", str(output / "view-001")]) == 1


def test_live_subscription_is_read_only_and_saves_named_feedback(calibration_files, tmp_path, monkeypatch):
    import websockets
    calibration = overlay.Calibration(calibration_files, CONFIG)
    image, synthetic = overlay.mock_capture(calibration)
    encoded = cv2.imencode(".png", image)[1].tobytes()
    mime = b"image/png"
    binary = b"BRDG" + bytes([1, len(mime)]) + mime + struct.pack("<Q", 1000) + encoded
    raw = json.dumps({"type": "joint_state", "timestamp": 1002, "data": {
        "names": list(reversed(synthetic["joints"])),
        "positions": list(reversed(list(synthetic["joints"].values())))}})
    urls, closed = [], []
    class Socket:
        def __init__(self, url):
            self.url = url
            self.transport = SimpleNamespace(abort=lambda: closed.append(url))
        async def __aenter__(self):
            return self
        async def __aexit__(self, *args):
            pass
        async def __aiter__(self):
            yield raw if "kind=joint" in self.url else binary
            await asyncio.Future()
    def connect(url, **kwargs):
        urls.append(url)
        return Socket(url)
    monkeypatch.setattr(websockets, "connect", connect)
    profile = tmp_path / "profile.json"
    profile.write_text(json.dumps({"camera": {"backend": "bridge", "color_topic": overlay.HEAD_TOPIC,
                                              "bridge_host": "ws://example.invalid"}}))
    output = tmp_path / "live"
    output.mkdir()
    args = SimpleNamespace(profile=profile, domain_id=0, timeout=1., max_age_ms=750.,
                           max_skew_ms=30., output=output, headless=True, frames=1)
    asyncio.run(overlay.live(calibration, args))
    assert len(urls) == len(closed) == 2
    assert any("kind=image" in url for url in urls) and any("kind=joint" in url for url in urls)
    assert "max_fps=30" in next(url for url in urls if "kind=image" in url)
    assert "max_fps" not in next(url for url in urls if "kind=joint" in url)
    record = json.loads((output / "view-001/projection.json").read_text())
    assert record["state_skew_ms"] == 2
    assert record["pixel_uv"] == pytest.approx(synthetic["pixel_uv"])
    assert record["read_only"] and not record["accuracy_verified"]
    assert record["image_receipt_age_ms"] >= 0 and record["newest_image_gap_ms"] == 0


def test_gui_freeze_click_and_save_bind_reference_to_the_shown_frame(calibration_files, tmp_path, monkeypatch):
    import websockets
    calibration = overlay.Calibration(calibration_files, CONFIG)
    image, initial = overlay.mock_capture(calibration)
    encoded = cv2.imencode(".png", image)[1].tobytes()
    mime = b"image/png"
    binary = b"BRDG" + bytes([1, len(mime)]) + mime + struct.pack("<Q", 1000) + encoded
    raw = json.dumps({"type": "joint_state", "timestamp": 1002, "data": {
        "names": list(initial["joints"]), "positions": list(initial["joints"].values())}})
    class Socket:
        transport = SimpleNamespace(abort=lambda: None)
        def __init__(self, url):
            self.url = url
        async def __aenter__(self):
            return self
        async def __aexit__(self, *args):
            pass
        async def __aiter__(self):
            yield raw if "kind=joint" in self.url else binary
            await asyncio.Future()
    monkeypatch.setattr(websockets, "connect", lambda url, **kw: Socket(url))
    callbacks = []
    monkeypatch.setattr(cv2, "setMouseCallback", lambda name, callback: callbacks.append(callback))
    monkeypatch.setattr(cv2, "imshow", lambda *a: None)
    monkeypatch.setattr(cv2, "getWindowProperty", lambda *a: 1)
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda: None)
    keys = iter([-1, ord(" "), ord("s"), ord("q")])
    def key(delay):
        value = next(keys)
        if value == ord("s"):
            callbacks[0](cv2.EVENT_LBUTTONDOWN, 100, 200, 0, None)
        return value
    monkeypatch.setattr(cv2, "waitKey", key)
    profile = tmp_path / "gui-profile.json"
    profile.write_text(json.dumps({"camera": {"backend": "bridge", "color_topic": overlay.HEAD_TOPIC,
                                              "bridge_host": "ws://example.invalid"}}))
    output = tmp_path / "gui"
    output.mkdir()
    args = SimpleNamespace(profile=profile, domain_id=0, timeout=1., max_age_ms=750.,
                           max_skew_ms=30., output=output, headless=False, frames=0)
    asyncio.run(overlay.live(calibration, args))
    record = json.loads((output / "view-002/projection.json").read_text())
    assert record["display_frozen"] and record["manual_reference"]["pixel_uv"] == [100, 200]
    assert record["image_stamp_ms"] == record["manual_reference"]["image_stamp_ms"] == 1000
    assert record["joints"] == initial["joints"]
    assert json.loads((output / "run.json").read_text())["reference_captures"] == 1


def test_live_handshake_timeout_is_reported_and_recorded(calibration_files, tmp_path, monkeypatch):
    import websockets
    class Unreachable:
        async def __aenter__(self):
            raise asyncio.TimeoutError("timed out during opening handshake")
        async def __aexit__(self, *args):
            pass
    monkeypatch.setattr(websockets, "connect", lambda *a, **kw: Unreachable())
    profile = tmp_path / "profile.json"
    profile.write_text(json.dumps({"camera": {"backend": "bridge", "color_topic": overlay.HEAD_TOPIC,
                                              "bridge_host": "ws://example.invalid"}}))
    output = tmp_path / "timeout"
    result = overlay.main(["--calibration-summary", str(calibration_files), "--output", str(output),
                           "live", "--profile", str(profile), "--headless", "--frames", "1"])
    assert result == 1
    record = json.loads((output / "run.json").read_text())
    assert "opening handshake" in record["error"]
    assert record["saved_captures"] == 0


@pytest.fixture
def synthetic_dual_board():
    geometry = SimpleNamespace(k=np.array([[400., 0, 320], [0, 400, 240], [0, 0, 1]]),
                               distortion=np.zeros(5),
                               wrist_k=np.array([[430., 0, 320], [0, 430, 240], [0, 0, 1]]),
                               wrist_distortion=np.zeros(5))
    objects = overlay.common.board_points({"board": {"columns": 7, "rows": 10, "square_m": .021}})
    head = np.eye(4)
    wrist = np.eye(4); wrist[0, 3] = .12
    board = np.eye(4); board[:3, 3] = [-.063, -.0945, .8]
    board[:3, :3] = Rotation.from_euler("z", .1).as_matrix()
    corners = {}
    for name, camera, k in (("head", head, geometry.k), ("wrist", wrist, geometry.wrist_k)):
        transform = overlay.common.invert(camera) @ board
        points = objects @ transform[:3, :3].T + transform[:3, 3]
        corners[name] = cv2.projectPoints(points, np.zeros(3), np.zeros(3), k, np.zeros(5))[0].reshape(-1, 2)
    return geometry, objects, head, wrist, corners


def test_dual_board_recovers_agreement_from_independent_images(synthetic_dual_board):
    cal, objects, head, wrist, corners = synthetic_dual_board
    result = overlay.compare_board_geometry(cal, corners["head"], corners["wrist"], head, wrist, objects)
    assert result["translation_difference_mm"] < .01
    assert result["rotation_difference_deg"] < .001
    assert result["cross_projection"]["head"]["rms_px"] < .001
    assert result["cross_projection"]["wrist"]["rms_px"] < .001


def test_dual_board_exposes_known_mount_translation_error(synthetic_dual_board):
    cal, objects, head, wrist, corners = synthetic_dual_board
    wrong = wrist.copy(); wrong[0, 3] += .02
    result = overlay.compare_board_geometry(cal, corners["head"], corners["wrist"], head, wrong, objects)
    assert result["translation_difference_mm"] == pytest.approx(20, abs=.01)
    assert result["rotation_difference_deg"] < .001
    assert result["cross_projection"]["head"]["rms_px"] == pytest.approx(10, abs=.001)
    assert result["cross_projection"]["wrist"]["rms_px"] == pytest.approx(10.75, abs=.001)


def test_dual_board_exposes_known_mount_rotation_error(synthetic_dual_board):
    cal, objects, head, wrist, corners = synthetic_dual_board
    wrong = wrist.copy(); wrong[:3, :3] = Rotation.from_euler("z", 5, degrees=True).as_matrix()
    result = overlay.compare_board_geometry(cal, corners["head"], corners["wrist"], head, wrong, objects)
    assert result["rotation_difference_deg"] == pytest.approx(5, abs=.001)


def test_dual_board_pairs_each_image_with_its_own_feedback():
    head = overlay.Sample(1000, 5., "head")
    wrist = overlay.Sample(1020, 5., "wrist")
    states = [overlay.Sample(1001, 5., "head-time"), overlay.Sample(1021, 5., "wrist-time")]
    pair = overlay.select_board_samples([head], [wrist], states, 5.1, 750, 30)
    assert pair[1].value == "head-time" and pair[3].value == "wrist-time"
    assert overlay.select_board_samples([head], [wrist], states, 6., 750, 30) is None
    assert overlay.select_board_samples([head], [overlay.Sample(1100, 5., "wrong-time")], states, 5.1, 750, 30) is None


@pytest.fixture
def fake_dual_board_bridge(calibration_files, synthetic_dual_board, tmp_path, monkeypatch):
    import websockets
    cal = overlay.Calibration(calibration_files, CONFIG)
    geometry, objects, head, wrist, corners = synthetic_dual_board
    cal.k, cal.distortion = geometry.k, geometry.distortion
    cal.wrist_k, cal.wrist_distortion = geometry.wrist_k, geometry.wrist_distortion
    cal.camera_poses = lambda joints: (wrist.copy(), head.copy())
    _, initial = overlay.mock_capture(cal)
    images = {name: np.full((480, 640, 3), code, np.uint8) for name, code in (("head", 10), ("wrist", 20))}
    binary = {}
    for name, stamp in (("head", 1000), ("wrist", 1004)):
        mime = b"image/png"
        data = cv2.imencode(".png", images[name])[1].tobytes()
        binary[name] = b"BRDG" + bytes([1, len(mime)]) + mime + struct.pack("<Q", stamp) + data
    raw = json.dumps({"type": "joint_state", "timestamp": 1002, "data": {
        "names": list(initial["joints"]), "positions": list(initial["joints"].values())}})
    opened, closed = [], []
    class Socket:
        def __init__(self, url):
            self.url = url
            self.transport = SimpleNamespace(abort=lambda: closed.append(url))
        async def __aenter__(self):
            return self
        async def __aexit__(self, *args):
            pass
        async def __aiter__(self):
            yield raw if "kind=joint" in self.url else binary["wrist" if "right" in self.url else "head"]
            await asyncio.Future()
    def connect(url, **kwargs):
        opened.append(url)
        return Socket(url)
    monkeypatch.setattr(websockets, "connect", connect)
    def detect(image, config, flip=False):
        values = corners["head" if image[0, 0, 0] == 10 else "wrist"].copy()
        if flip:
            values = values[::-1].copy()
        return values, {"accepted": True, "detected": True, "corner_count": 70}
    monkeypatch.setattr(overlay.common, "detect_board", detect)
    profile = tmp_path / "board-profile.json"
    profile.write_text(json.dumps({"camera": {"backend": "bridge", "color_topic": overlay.HEAD_TOPIC,
                                              "bridge_host": "ws://example.invalid"}}))
    output = tmp_path / "board-live"; output.mkdir()
    args = SimpleNamespace(profile=profile, capture=None, domain_id=0, timeout=1.,
                           max_age_ms=750., max_skew_ms=30., output=output,
                           pattern=None, square_m=None, flip_head=False, flip_wrist=False,
                           confirm_corner_order=False)
    return cal, args, opened, closed


def test_dual_board_live_capture_closes_three_read_only_streams(fake_dual_board_bridge):
    cal, args, opened, closed = fake_dual_board_bridge
    assert asyncio.run(overlay.board_check(cal, args))
    assert len(opened) == len(closed) == 3
    report = json.loads((args.output / "board-check.json").read_text())
    assert report["status"] == "needs_corner_identity_review"
    assert report["relative_pose_within_limits"] is None
    assert not report["accuracy_verified"]
    assert report["translation_difference_mm"] < .01
    assert report["capture"]["image_skew_ms"] == 4
    assert report["capture"]["head"]["state_skew_ms"] == 2
    assert report["capture"]["wrist"]["state_skew_ms"] == 2


def test_dual_board_replay_is_offline_and_confirms_only_explicit_corner_ids(fake_dual_board_bridge, tmp_path, monkeypatch):
    import websockets
    cal, args, opened, closed = fake_dual_board_bridge
    assert asyncio.run(overlay.board_check(cal, args))
    original = args.output
    def forbidden(*a, **kw):
        raise AssertionError("board replay must not connect")
    monkeypatch.setattr(websockets, "connect", forbidden)
    args.capture, args.profile = original, None
    args.output = tmp_path / "board-replay"; args.output.mkdir()
    args.confirm_corner_order = True
    assert asyncio.run(overlay.board_check(cal, args))
    report = json.loads((args.output / "board-check.json").read_text())
    assert report["source"] == "replay" and report["original_source"] == "live"
    assert report["relative_pose_within_limits"] is True
    assert not report["accuracy_verified"]
    (original / "head-color.png").write_bytes(b"wrong image")
    args.output = tmp_path / "tampered-replay"; args.output.mkdir()
    with pytest.raises(ValueError, match="recorded hash"):
        asyncio.run(overlay.board_check(cal, args))


def test_dual_board_does_not_silently_choose_corner_order_to_reduce_error(fake_dual_board_bridge):
    cal, args, opened, closed = fake_dual_board_bridge
    args.flip_wrist = True
    assert asyncio.run(overlay.board_check(cal, args))
    report = json.loads((args.output / "board-check.json").read_text())
    assert report["rotation_difference_deg"] == pytest.approx(180, abs=.001)
    assert report["status"] == "needs_corner_identity_review"
    assert report["relative_pose_within_limits"] is None


def test_dual_board_missing_detection_keeps_images_without_claiming_measurement(fake_dual_board_bridge, monkeypatch):
    cal, args, opened, closed = fake_dual_board_bridge
    detect = overlay.common.detect_board
    def missing_wrist(image, config, flip=False):
        if image[0, 0, 0] == 20:
            return None, {"accepted": False, "detected": False, "reason": "expected 70 inner corners"}
        return detect(image, config, flip)
    monkeypatch.setattr(overlay.common, "detect_board", missing_wrist)
    assert not asyncio.run(overlay.board_check(cal, args))
    report = json.loads((args.output / "board-check.json").read_text())
    assert report["status"] == "board_not_detected" and not report["measurement_available"]
    assert "translation_difference_mm" not in report
    assert (args.output / "head-color.png").is_file() and (args.output / "wrist-color.png").is_file()


def test_board_check_cli_dispatch_is_explicit_and_offline(calibration_files, tmp_path, monkeypatch):
    called = []
    async def check(cal, args):
        called.append((cal.summary_path, args.capture, args.profile))
        return True
    monkeypatch.setattr(overlay, "board_check", check)
    capture = tmp_path / "saved-board-pair"
    assert overlay.main(["--calibration-summary", str(calibration_files), "--output", str(tmp_path / "board-cli"),
                         "board-check", "--capture", str(capture)]) == 0
    assert called == [(calibration_files.resolve(), capture, None)]
