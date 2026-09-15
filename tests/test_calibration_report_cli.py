"""CLI reports preserve evidence and expose failed independent validation."""
import json
from pathlib import Path

import numpy as np
import pytest

from tron2_deployment.cli import main


def saved_solution(tmp_path):
    path = tmp_path / "handeye.json"
    path.write_text(json.dumps({
        "mode": "eye_to_hand", "reference_frame": "base", "source": "mock",
        "side": "left", "camera_id": "synthetic", "head_q2": [0, 0],
        "camera_to_reference": np.eye(4).tolist(), "samples": 5,
        "verified": False,
    }))
    return path


@pytest.mark.parametrize("offset_m,expected_status,expected_exit", [
    (0.0, "report_written", 0), (0.012, "failed", 1),
])
def test_saved_report_and_exit_code_follow_independent_error(tmp_path, capsys,
                                                          offset_m, expected_status, expected_exit):
    solution = saved_solution(tmp_path)
    original = solution.read_bytes()
    camera = np.array([[0, 0, .5], [.1, 0, .5], [0, .1, .5], [.1, .1, .6]])
    reference = camera.copy()
    reference[-1, 0] += offset_m
    points = tmp_path / "points.npz"
    np.savez(points, points_camera=camera, points_base=reference)
    output = tmp_path / "report"
    code = main(["calibration-report", "--handeye", str(solution),
                 "--validation-points", str(points), "--max-error-m", ".005",
                 "--output", str(output)])
    result = json.loads(capsys.readouterr().out)
    assert code == expected_exit and result["status"] == expected_status
    assert Path(result["report_path"]).is_file()
    metrics = json.loads(Path(result["metrics_path"]).read_text())
    assert metrics["heldout"]["passed"] == (offset_m <= .005)
    assert metrics["handeye"]["sample_count"] == 0
    assert metrics["calibration_modified"] is False
    assert result["calibration_applied"] is False and result["hardware_commanded"] is False
    assert solution.read_bytes() == original
    with pytest.raises(SystemExit) as error:
        main(["calibration-report", "--handeye", str(solution), "--output", str(output)])
    assert error.value.code == 1
    assert solution.read_bytes() == original


def test_missing_measurements_cannot_be_reported_as_a_validation_pass(tmp_path, capsys):
    solution = saved_solution(tmp_path)
    output = tmp_path / "report"
    assert main(["calibration-report", "--handeye", str(solution), "--output", str(output)]) == 0
    result = json.loads(capsys.readouterr().out)
    metrics = json.loads(Path(result["metrics_path"]).read_text())
    assert metrics["heldout"] is None and metrics["review_status"] == "unverified"
    with pytest.raises(SystemExit) as error:
        main(["calibration-report", "--handeye", str(solution),
              "--max-error-m", ".005", "--output", str(tmp_path / "invalid")])
    assert error.value.code == 1
    assert not (tmp_path / "invalid").exists()
