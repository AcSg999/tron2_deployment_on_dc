"""Reports must remain offline, explicit about evidence, and non-destructive."""
import copy
import json
from pathlib import Path
import socket

import cv2
import numpy as np
import pytest

from tron2_deployment.calibration_report import write_calibration_report


def _intrinsic(tmp_path):
    path = tmp_path / 'board-<example>.png'
    image = np.full((120, 160, 3), 220, dtype=np.uint8)
    image[30:90:10] = 70
    assert cv2.imwrite(str(path), image)
    observed = np.array([[30, 30], [70, 30], [110, 30], [30, 80], [70, 80], [110, 80]], dtype=float)
    projected = observed + [0.1, 0.2]
    return {'kind': 'intrinsic', 'source': 'synthetic',
            'calibration': {'K': [[130, 0, 80], [0, 130, 60], [0, 0, 1]],
                            'dist': [0.05, 0, 0, 0, 0], 'width': 160, 'height': 120,
                            'pattern': [3, 2], 'square_m': 0.025, 'fit_rms_px': 0.224},
            'accepted_count': 1, 'rejected_count': 0, 'rms_px': 0.224, 'max_error_px': 0.224,
            'coverage': {'convex_hull_fraction': 0.21, 'occupied_fraction': 0.125,
                         'grid_columns': 8, 'grid_rows': 6, 'occupied_cells': 6, 'total_cells': 48},
            'views': [{'image_path': str(path), 'status': 'accepted',
                       'observed_xy_px': observed.tolist(), 'predicted_xy_px': projected.tolist(),
                       'residual_xy_px': (projected-observed).tolist(), 'error_px': [0.224]*6,
                       'rms_px': 0.224, 'max_error_px': 0.224}],
            'notes': ['Synthetic fixture, not a real calibration.']}


def _handeye(empty=False):
    camera = np.eye(4)
    camera[:3, 3] = [0.1, 0.2, 0.6]
    samples = []
    for index in range(3):
        wrist = np.eye(4)
        wrist[:3, 3] = [index * 0.1, 0.1, 0.2]
        samples.append({'label': str(index), 'robot_gripper_to_base': wrist.tolist(),
                        'target_to_wrist': np.eye(4).tolist(), 'translation_error_mm': index*0.1,
                        'rotation_error_deg': index*0.2})
    return {'kind': 'handeye', 'source': 'synthetic', 'mode': 'eye_to_hand', 'camera_id': '<camera>',
            'side': 'left', 'head_q2': [0, 0], 'camera_to_base': camera.tolist(),
            'mean_target_to_wrist': None if empty else np.eye(4).tolist(),
            'translation_rms_mm': None if empty else 0.129, 'translation_max_mm': None if empty else 0.2,
            'rotation_rms_deg': None if empty else 0.258, 'rotation_max_deg': None if empty else 0.4,
            'wrist_spread': None if empty else {'translation_range_mm': [200, 0, 0],
                                               'max_pairwise_translation_mm': 200,
                                               'max_pairwise_rotation_deg': 0},
            'samples': [] if empty else samples, 'notes': []}


def _heldout(passed=True):
    reference = np.array([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0.05]])
    predicted = reference + [0.002, 0, 0]
    return {'kind': 'heldout', 'source': 'caller_supplied_points', 'camera_to_base': np.eye(4).tolist(),
            'points_camera_m': predicted.tolist(), 'reference_base_m': reference.tolist(),
            'predicted_base_m': predicted.tolist(), 'residual_base_mm': ((predicted-reference)*1000).tolist(),
            'error_mm': [2, 2, 2], 'rms_mm': 2, 'max_error_mm': 2,
            'rms_m': 0.002, 'max_error_m': 0.002, 'tolerance_m': 0.005 if passed else 0.001,
            'sample_count': 3, 'passed': passed, 'notes': ['Independence must be established by the operator.']}


def test_complete_report_embeds_figures_without_network_or_profile_writes(tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('offline report attempted a network connection')
    monkeypatch.setattr(socket, 'socket', forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    intrinsic, handeye, heldout = _intrinsic(tmp_path), _handeye(), _heldout()
    before = copy.deepcopy((intrinsic, handeye, heldout))
    result = write_calibration_report(tmp_path/'report', intrinsic=intrinsic, handeye=handeye,
                                      heldout=heldout, title='<script>alert("x")</script>')
    assert (intrinsic, handeye, heldout) == before
    html = Path(result['report_path']).read_text()
    metrics = json.loads(Path(result['metrics_path']).read_text())
    assert metrics['review_status'] == 'unverified'
    assert metrics['calibration_modified'] is False
    assert metrics['heldout'] == heldout
    assert '<script>' not in html and '&lt;script&gt;' in html
    assert '<camera>' not in html and '&lt;camera&gt;' in html
    assert 'https://' not in html and 'http://' not in html
    assert 'PASS / 通过' in html
    assert 'not independent accuracy measurements' in html
    assert 'does not authorize robot motion' in html
    assert len(result['figure_paths']) == 8
    assert html.count('src="data:image/png;base64,') == 8
    for filename in result['figure_paths']:
        assert Path(filename).read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
        assert Path(filename).stat().st_size > 1000
    assert not list(tmp_path.glob('.calibration-report-*'))


def test_intrinsic_only_is_explicit_about_missing_independent_validation(tmp_path):
    result = write_calibration_report(tmp_path/'report', intrinsic=_intrinsic(tmp_path))
    html = Path(result['report_path']).read_text()
    assert 'No held-out validation / 未提供独立验证点' in html
    assert 'Report review: UNVERIFIED' in html
    assert 'PASS / 通过' not in html
    assert 'same corners' in html
    assert len(result['figure_paths']) == 3


def test_transform_only_handeye_displays_no_sample_warning(tmp_path):
    result = write_calibration_report(tmp_path/'report', handeye=_handeye(empty=True))
    html = Path(result['report_path']).read_text()
    assert 'No hand-eye sample diagnostics' in html
    assert 'No held-out validation' in html
    assert [Path(path).name for path in result['figure_paths']] == ['handeye-transform.png']


def test_failed_heldout_is_visible(tmp_path):
    result = write_calibration_report(tmp_path/'report', heldout=_heldout(passed=False))
    html = Path(result['report_path']).read_text()
    assert 'FAIL / 未通过' in html
    assert 'PASS / 通过' not in html
    assert len(result['figure_paths']) == 3


def test_all_rejected_views_make_unavailable_intrinsics_visible(tmp_path):
    data = _intrinsic(tmp_path)
    data.update(accepted_count=0, rejected_count=1, rms_px=None, max_error_px=None,
                views=[{'image_path': 'not-found.png', 'status': 'rejected', 'reason': '<board not found>'}])
    data['coverage'].update(convex_hull_fraction=0, occupied_fraction=0, occupied_cells=0)
    result = write_calibration_report(tmp_path/'report', intrinsic=data)
    html = Path(result['report_path']).read_text()
    assert 'No usable board views' in html
    assert '&lt;board not found&gt;' in html
    assert not result['figure_paths']


@pytest.mark.parametrize('existing', ['directory', 'file', 'symlink'])
def test_existing_output_is_not_overwritten(tmp_path, existing):
    output = tmp_path/'report'
    if existing == 'directory':
        output.mkdir()
        (output/'index.html').write_text('keep original')
    elif existing == 'file':
        output.write_text('keep original')
    else:
        target = tmp_path/'target'
        target.mkdir()
        output.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match='never overwritten'):
        write_calibration_report(output, heldout=_heldout())
    if existing == 'directory':
        assert (output/'index.html').read_text() == 'keep original'
    elif existing == 'file':
        assert output.read_text() == 'keep original'
    else:
        assert output.is_symlink() and not list(target.iterdir())


def test_empty_output_directory_is_allowed(tmp_path):
    output = tmp_path/'report'
    output.mkdir()
    result = write_calibration_report(output, handeye=_handeye(empty=True))
    assert Path(result['report_path']).is_file()


@pytest.mark.parametrize('change', ['nan', 'shape', 'inconsistent_status', 'missing', 'nonrigid'])
def test_malformed_diagnostics_do_not_create_report(tmp_path, change):
    data = _heldout()
    if change == 'nan':
        data['reference_base_m'][0][0] = float('nan')
    elif change == 'shape':
        data['error_mm'] = [1]
    elif change == 'inconsistent_status':
        data['passed'] = False
    elif change == 'missing':
        del data['reference_base_m']
    else:
        data['camera_to_base'][0][0] = 2
    with pytest.raises(ValueError):
        write_calibration_report(tmp_path/'report', heldout=data)
    assert not (tmp_path/'report').exists()
    assert not list(tmp_path.glob('.calibration-report-*'))


def test_missing_source_image_does_not_leave_partial_report(tmp_path):
    data = _intrinsic(tmp_path)
    Path(data['views'][0]['image_path']).unlink()
    with pytest.raises(ValueError, match='cannot read intrinsic source image'):
        write_calibration_report(tmp_path/'report', intrinsic=data)
    assert not (tmp_path/'report').exists()
    assert not list(tmp_path.glob('.calibration-report-*'))


def test_no_diagnostics_is_an_error(tmp_path):
    with pytest.raises(ValueError, match='at least one'):
        write_calibration_report(tmp_path/'report')


def test_source_image_change_is_detected_before_report_publication(tmp_path):
    import hashlib
    data = _intrinsic(tmp_path)
    view = data['views'][0]
    path = Path(view['image_path'])
    view['image_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    assert cv2.imwrite(str(path), np.full((120, 160, 3), 100, dtype=np.uint8))
    with pytest.raises(ValueError, match='changed after diagnostics'):
        write_calibration_report(tmp_path/'report', intrinsic=data)
    assert not (tmp_path/'report').exists()


def test_heldout_threshold_roundoff_preserves_diagnostic_verdict(tmp_path):
    data = _heldout()
    data.update(error_mm=[5.000000000000001]*3, tolerance_m=0.005)
    result = write_calibration_report(tmp_path/'report', heldout=data)
    assert json.loads(Path(result['metrics_path']).read_text())['heldout']['passed'] is True
