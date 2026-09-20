import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from scripts import validate_chessboard_extrinsics as tool
from tron2_deployment.calibration import board_points
from tron2_deployment.config import load_profile


def test_depth_free_chessboard_points_and_report(tmp_path, monkeypatch):
    profile_path = Path("src/tron2_deployment/assets/demo.json")
    profile = load_profile(profile_path)
    pattern = (7, 9)
    board = board_points(pattern, 0.02)
    k = np.asarray(profile["camera"]["intrinsics"], dtype=float)
    rvec = np.array([0.2, -0.3, 0.1])
    tvec = np.array([0.02, -0.04, 0.8])
    observed = cv2.projectPoints(board, rvec, tvec, k, np.zeros(5))[0]
    monkeypatch.setattr(tool, "corners", lambda image, board_pattern: observed)
    intrinsic = {"K": k.tolist(), "dist": [0] * 5, "width": 640, "height": 480,
                 "pattern": list(pattern), "square_m": 0.02}
    intrinsic_path = tmp_path / "intrinsics.json"
    intrinsic_path.write_text(json.dumps(intrinsic))
    capture = tmp_path / "capture"
    capture.mkdir()
    cv2.imwrite(str(capture / "color.png"), np.zeros((480, 640, 3), np.uint8))
    (capture / "frame.json").write_text(json.dumps({
        "camera_id": profile["camera"]["identity"], "source": "mock",
        "intrinsics": k.tolist(), "distortion": [0] * 5, "head_q2": [0, 0],
    }))
    states = []
    for number, shoulder in enumerate((0.0, 0.1, -0.1), 1):
        state = tmp_path / f"state-{number:02d}.json"
        arm = [0.0] * 14
        arm[2] = arm[9] = 0.2
        arm[0] = shoulder
        state.write_text(json.dumps({"arm_q14": arm, "head_q2": [0, 0], "source": "mock"}))
        states.append(state)
    touches = [(str(capture), str(row), str(col), str(state))
               for (row, col), state in zip(((0, 0), (4, 3), (8, 5)), states)]
    camera, base, diagnostics, source = tool.build_points(profile, intrinsic, touches, "left")
    expected = board[[0, 4 * 7 + 3, 8 * 7 + 5]] @ cv2.Rodrigues(rvec)[0].T + tvec
    np.testing.assert_allclose(camera, expected, atol=1e-6)
    assert base.shape == (3, 3) and np.isfinite(base).all()
    assert diagnostics[0]["reprojection_rms_px"] < 1e-4
    assert source == "mock"

    handeye = tmp_path / "handeye.json"
    handeye.write_text(json.dumps({"mode": "eye_to_hand", "source": "mock",
                                   "camera_id": profile["camera"]["identity"],
                                   "head_q2": [0, 0], "camera_to_base": np.eye(4).tolist()}))
    output = tmp_path / "points.npz"
    args = ["--profile", str(profile_path), "--intrinsics", str(intrinsic_path),
            "--handeye", str(handeye), "--side", "left", "--max-error-m", "5",
            "--output", str(output)]
    for touch in touches:
        args += ["--touch", *touch]
    assert tool.main(args) == 0
    with np.load(output) as saved:
        np.testing.assert_allclose(saved["points_camera"], camera)
        np.testing.assert_allclose(saved["points_base"], base)
    report = json.loads(output.with_suffix(".report.json").read_text())
    assert len(report["per_point_error_m"]) == 3
    assert report["passed"]
    assert Path(report["pnp"][0]["marked_image"]).is_file()

    strict_args = args.copy()
    strict_args[strict_args.index("--max-error-m") + 1] = "0.001"
    assert tool.main(strict_args) == 1
    assert not json.loads(output.with_suffix(".report.json").read_text())["passed"]

    with pytest.raises(ValueError, match="state file reused"):
        tool.build_points(profile, intrinsic, touches[:2] + [touches[2][:3] + (str(states[0]),)], "left")
