"""Offline calibration review figures and a portable, self-contained HTML report.

The renderer reads diagnostic dictionaries and source images only. It does not
connect to cameras/robots, alter profiles, or declare a calibration accepted.
"""
from __future__ import annotations

import base64
import hashlib
from html import escape
import json
from pathlib import Path
import shutil
import tempfile

import cv2
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure


_TEAL = "#0f766e"
_BLUE = "#2563eb"
_AMBER = "#c2410c"
_AXES = ("#dc2626", "#16a34a", "#2563eb")


def _array(value, name, shape=None):
    try:
        array = np.asarray(value, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} must contain numeric values") from exc
    if not np.isfinite(array).all() or (shape is not None and array.shape != shape):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return array


def _number(value, name, *, positive=False):
    array = _array(value, name, ())
    number = float(array)
    if number < 0 or (positive and number <= 0):
        raise ValueError(f"{name} must be {'positive' if positive else 'nonnegative'}")
    return number


def _transform(value, name):
    matrix = _array(value, name, (4, 4))
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
        raise ValueError(f"{name} must be a rigid transform")
    return matrix


def _validate(intrinsic, handeye, heldout):
    if all(item is None for item in (intrinsic, handeye, heldout)):
        raise ValueError("at least one calibration diagnostic is required")
    for kind, item in (("intrinsic", intrinsic), ("handeye", handeye), ("heldout", heldout)):
        if item is not None and (not isinstance(item, dict) or item.get("kind") != kind):
            raise ValueError(f"{kind} must be a {kind} diagnostic dictionary")
    if intrinsic is not None:
        calibration = intrinsic["calibration"]
        _array(calibration["K"], "intrinsic K", (3, 3))
        distortion = _array(calibration["dist"], "intrinsic distortion")
        if distortion.ndim != 1 or distortion.size not in (4, 5, 8, 12, 14):
            raise ValueError("intrinsic distortion must contain 4, 5, 8, 12, or 14 coefficients")
        for name in ("width", "height"):
            value = _number(calibration[name], name, positive=True)
            if value != int(value):
                raise ValueError("image dimensions must be integers")
        accepted = 0
        for view in intrinsic["views"]:
            if not isinstance(view.get("image_path"), str):
                raise ValueError("intrinsic image_path must be a string")
            if view["status"] == "accepted":
                observed = _array(view["observed_xy_px"], "observed corners")
                if observed.ndim != 2 or observed.shape[1:] != (2,) or not len(observed):
                    raise ValueError("observed corners must be nonempty Nx2 coordinates")
                _array(view["predicted_xy_px"], "predicted corners", observed.shape)
                if np.any(_array(view["error_px"], "corner error", (len(observed),)) < 0):
                    raise ValueError("corner errors must be nonnegative")
                _number(view["rms_px"], "view RMS")
                _number(view["max_error_px"], "view maximum error")
                accepted += 1
            elif view["status"] != "rejected":
                raise ValueError("intrinsic view status must be accepted or rejected")
        if (accepted != intrinsic["accepted_count"]
                or len(intrinsic["views"]) - accepted != intrinsic["rejected_count"]):
            raise ValueError("intrinsic view counts do not match views")
        for name in ("rms_px", "max_error_px"):
            if accepted:
                _number(intrinsic[name], name)
        coverage = intrinsic["coverage"]
        for name in ("convex_hull_fraction", "occupied_fraction"):
            if _number(coverage[name], name) > 1:
                raise ValueError("coverage fractions must be in [0, 1]")
        for name in ("grid_columns", "grid_rows"):
            value = _number(coverage[name], name, positive=True)
            if value != int(value) or value > 100:
                raise ValueError("coverage grid dimensions must be integers <= 100")
    if handeye is not None:
        _transform(handeye["camera_to_base"], "camera_to_base")
        if handeye["samples"]:
            _transform(handeye["mean_target_to_wrist"], "mean_target_to_wrist")
        for sample in handeye["samples"]:
            _transform(sample["robot_gripper_to_base"], "robot_gripper_to_base")
            _transform(sample["target_to_wrist"], "target_to_wrist")
            _number(sample["translation_error_mm"], "translation error")
            _number(sample["rotation_error_deg"], "rotation error")
        if handeye["samples"]:
            for name in ("translation_rms_mm", "translation_max_mm", "rotation_rms_deg", "rotation_max_deg"):
                _number(handeye[name], name)
            _array(handeye["wrist_spread"]["translation_range_mm"], "wrist translation range", (3,))
            for name in ("max_pairwise_translation_mm", "max_pairwise_rotation_deg"):
                _number(handeye["wrist_spread"][name], name)
    if heldout is not None:
        points = _array(heldout["reference_base_m"], "held-out reference points")
        if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 3:
            raise ValueError("held-out reference points must be Nx3, N >= 3")
        for name in ("points_camera_m", "predicted_base_m", "residual_base_mm"):
            _array(heldout[name], name, points.shape)
        errors = _array(heldout["error_mm"], "held-out error", (len(points),))
        if np.any(errors < 0):
            raise ValueError("held-out errors must be nonnegative")
        _transform(heldout["camera_to_base"], "camera_to_base")
        tolerance_mm = _number(heldout["tolerance_m"], "held-out tolerance", positive=True) * 1000
        for name in ("rms_mm", "max_error_mm"):
            _number(heldout[name], name)
        if not isinstance(heldout["passed"], bool):
            raise ValueError("held-out passed must be a boolean")
        maximum = float(np.max(errors))
        # Diagnostics compare in meters; conversion to millimeters may change
        # the last floating-point bit at the threshold. Preserve that verdict.
        epsilon = max(1e-9, abs(tolerance_mm) * 1e-12)
        if ((heldout["passed"] and maximum > tolerance_mm + epsilon)
                or (not heldout["passed"] and maximum < tolerance_mm - epsilon)):
            raise ValueError("held-out pass/fail is inconsistent with errors and tolerance")


def _figure(rows=1, columns=1, *, width=10, height=4):
    figure = Figure(figsize=(width, height), facecolor="white", layout="constrained")
    FigureCanvasAgg(figure)
    axes = figure.subplots(rows, columns, squeeze=False)
    for axis in axes.flat:
        axis.spines[["top", "right"]].set_visible(False)
        axis.tick_params(labelsize=8, colors="#334155")
        axis.grid(alpha=0.15)
        axis.set_axisbelow(True)
    return figure, axes


def _save(figure, directory, filename, caption, figures):
    path = directory / filename
    figure.savefig(path, dpi=140, facecolor="white")
    figure.clear()
    figures.append({"filename": filename, "caption": caption})


def _labels(axis, count, labels=None):
    positions = np.arange(count)
    step = max(1, int(np.ceil(count / 18)))
    ticks = positions[::step]
    axis.set_xticks(ticks, [(str(labels[index])[-22:] if labels else str(index + 1)) for index in ticks],
                    rotation=40 if labels else 0, ha="right" if labels else "center")
    return positions


def _render_intrinsic(data, directory, figures):
    views = [view for view in data["views"] if view["status"] == "accepted"]
    if not views:
        return
    calibration = data["calibration"]
    width, height = int(calibration["width"]), int(calibration["height"])
    k, dist = np.asarray(calibration["K"]), np.asarray(calibration["dist"])
    figure, axes = _figure()
    axis = axes[0, 0]
    positions = _labels(axis, len(views))
    axis.bar(positions, [view["rms_px"] for view in views], color=_TEAL, label="View RMS")
    axis.scatter(positions, [view["max_error_px"] for view in views], color=_AMBER, s=18, label="Largest corner error")
    axis.set(xlabel="Accepted view number (see image panels)", ylabel="Reprojection error (px)",
             title="Intrinsics: per-view reprojection fit (not independent validation)")
    axis.set_ylim(bottom=0)
    axis.legend(fontsize=8)
    _save(figure, directory, "intrinsic-errors.png", "Per-view fit errors / 每幅图像的拟合误差", figures)

    figure, axes = _figure(width=9, height=6)
    axis = axes[0, 0]
    all_points = np.vstack([view["observed_xy_px"] for view in views])
    for index, view in enumerate(views):
        points = np.asarray(view["observed_xy_px"])
        axis.scatter(points[:, 0], points[:, 1], s=7, alpha=0.65, label=str(index + 1))
        centroid = points.mean(axis=0)
        axis.text(*centroid, str(index + 1), fontsize=9, weight="bold")
    if len(all_points) >= 3:
        hull = cv2.convexHull(all_points.astype(np.float32)).reshape(-1, 2)
        axis.fill(hull[:, 0], hull[:, 1], color=_TEAL, alpha=0.07)
    axis.set(xlim=(0, width), ylim=(height, 0), xlabel="Image x (px)", ylabel="Image y (px)",
             title=(f"Detected-corner coverage: hull {data['coverage']['convex_hull_fraction']:.1%}; "
                    f"occupied grid {data['coverage']['occupied_fraction']:.1%}"))
    axis.set_xticks(np.linspace(0, width, int(data["coverage"]["grid_columns"]) + 1))
    axis.set_yticks(np.linspace(0, height, int(data["coverage"]["grid_rows"]) + 1))
    axis.set_aspect("equal")
    _save(figure, directory, "intrinsic-coverage.png", "Image coverage; inspect edges and corners / 图像覆盖范围；检查边缘与四角", figures)

    for index, view in enumerate(views):
        try:
            raw = Path(view["image_path"]).read_bytes()
        except OSError as exc:
            raise ValueError(f"cannot read intrinsic source image {view['image_path']}") from exc
        if view.get("image_sha256") and hashlib.sha256(raw).hexdigest() != view["image_sha256"]:
            raise ValueError(f"intrinsic source image changed after diagnostics: {view['image_path']}")
        image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"cannot read intrinsic source image {view['image_path']}")
        if image.shape[:2] != (height, width):
            raise ValueError(f"intrinsic source image resolution changed: {view['image_path']}")
        observed, predicted = np.asarray(view["observed_xy_px"]), np.asarray(view["predicted_xy_px"])
        corrected = cv2.undistort(image, k, dist, None, k)
        figure, axes = _figure(1, 2, width=12, height=5)
        for axis in axes.flat:
            axis.grid(False)
            axis.set_aspect("equal")
            axis.set(xlabel="x (px)", ylabel="y (px)")
        axes[0, 0].imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        axes[0, 0].scatter(observed[:, 0], observed[:, 1], facecolors="none", edgecolors="#22c55e", s=28, linewidths=0.9, label="Detected")
        axes[0, 0].scatter(predicted[:, 0], predicted[:, 1], color="#fb923c", marker="x", s=17, linewidths=0.8, label="Reprojected")
        axes[0, 0].legend(fontsize=8, loc="upper right", framealpha=0.95)
        axes[0, 0].set_title(f"View {index + 1}: original + corners\nRMS {view['rms_px']:.3f} px; max {view['max_error_px']:.3f} px", fontsize=10)
        axes[0, 1].imshow(cv2.cvtColor(corrected, cv2.COLOR_BGR2RGB))
        axes[0, 1].set_title("Undistorted (same K, full image)\nBlack borders may appear after correction", fontsize=10)
        _save(figure, directory, f"intrinsic-view-{index + 1:03d}.png",
              f"View {index + 1} / 图像 {index + 1}: {Path(view['image_path']).name}", figures)


def _frame(axis, transform, label, length):
    origin = transform[:3, 3]
    for index, color in enumerate(_AXES):
        vector = transform[:3, index] * length
        axis.quiver(*origin, *vector, color=color, arrow_length_ratio=0.15, linewidth=1.1)
    axis.text(*origin, label, fontsize=8, color="#334155")


def _space_axis(figure, positions):
    axis = figure.add_subplot(111, projection="3d")
    span = np.ptp(positions, axis=0)
    length = max(float(span.max()) * 0.12, 0.025)
    center = (positions.max(axis=0) + positions.min(axis=0)) / 2
    half = max(float(span.max()) * 0.65, length * 2)
    axis.set(xlim=(center[0] - half, center[0] + half), ylim=(center[1] - half, center[1] + half),
             zlim=(center[2] - half, center[2] + half), xlabel="Base x (m)", ylabel="Base y (m)", zlabel="Base z (m)")
    axis.set_box_aspect((1, 1, 1))
    axis.tick_params(labelsize=8)
    axis.view_init(elev=23, azim=-55)
    return axis, length


def _render_handeye(data, directory, figures):
    samples = data["samples"]
    if not samples:
        camera = np.asarray(data["camera_to_base"])
        figure = Figure(figsize=(9, 7), facecolor="white", layout="constrained")
        FigureCanvasAgg(figure)
        axis, length = _space_axis(figure, np.vstack((camera[:3, 3], np.zeros(3))))
        _frame(axis, np.eye(4), "Base", length)
        _frame(axis, camera, "Camera", length)
        axis.set_title("Camera-to-base transform only: no sample consistency data\nAxes: x red, y green, z blue", fontsize=11)
        _save(figure, directory, "handeye-transform.png", "Camera/base frames; no sample consistency assessment / 相机与基座坐标系；未评估样本一致性", figures)
        return
    figure, axes = _figure(2, 1, height=6)
    for axis, field, unit, color in ((axes[0, 0], "translation_error_mm", "Translation residual (mm)", _TEAL),
                                     (axes[1, 0], "rotation_error_deg", "Rotation residual (deg)", _BLUE)):
        positions = _labels(axis, len(samples))
        axis.bar(positions, [sample[field] for sample in samples], color=color)
        axis.set(xlabel="Sample number", ylabel=unit, ylim=(0, None))
    axes[0, 0].set_title("Hand-eye: board-to-wrist constancy (sample consistency only)")
    _save(figure, directory, "handeye-residuals.png", "Board-to-wrist residuals / 标定板相对腕部的一致性残差", figures)

    transforms = np.asarray([sample["robot_gripper_to_base"] for sample in samples])
    camera = np.asarray(data["camera_to_base"])
    positions = np.vstack((transforms[:, :3, 3], camera[:3, 3], np.zeros(3)))
    figure = Figure(figsize=(9, 7), facecolor="white", layout="constrained")
    FigureCanvasAgg(figure)
    axis, length = _space_axis(figure, positions)
    axis.scatter(*transforms[:, :3, 3].T, color=_TEAL, s=24, label="Wrist samples")
    for index, transform in enumerate(transforms):
        _frame(axis, transform, str(index + 1), length * 0.55)
    _frame(axis, np.eye(4), "Base", length)
    _frame(axis, camera, "Camera", length)
    axis.set_title("Wrist sample positions and orientation spread\nAxes: x red, y green, z blue", fontsize=11)
    axis.legend(fontsize=8)
    _save(figure, directory, "handeye-sample-spread.png", "Wrist poses in the base frame / 基座坐标系下的腕部采样位姿", figures)


def _render_heldout(data, directory, figures):
    reference = np.asarray(data["reference_base_m"])
    predicted = np.asarray(data["predicted_base_m"])
    camera = np.asarray(data["camera_to_base"])
    positions = np.vstack((reference, predicted, camera[:3, 3], np.zeros(3)))
    figure = Figure(figsize=(9, 7), facecolor="white", layout="constrained")
    FigureCanvasAgg(figure)
    axis, length = _space_axis(figure, positions)
    axis.scatter(*reference.T, color=_TEAL, s=40, marker="o", label="Measured base reference")
    axis.scatter(*predicted.T, color=_AMBER, s=35, marker="x", label="Camera points transformed to base")
    for index, (start, end) in enumerate(zip(reference, predicted)):
        axis.plot(*np.vstack((start, end)).T, color=_AMBER, linewidth=1.5)
        axis.text(*start, str(index + 1), fontsize=8)
    _frame(axis, np.eye(4), "Base", length)
    _frame(axis, camera, "Camera", length)
    axis.set_title("Held-out point correspondence (residual vectors at true scale)\nAxes: x red, y green, z blue", fontsize=11)
    axis.legend(fontsize=8, loc="upper left")
    _save(figure, directory, "heldout-points-3d.png", "Held-out references and transformed camera points / 验证参考点与变换后的相机点", figures)

    figure, axes = _figure(1, 3, width=12, height=4)
    for axis, (first, second) in zip(axes.flat, ((0, 1), (0, 2), (1, 2))):
        axis.scatter(reference[:, first], reference[:, second], color=_TEAL, s=22, label="Reference")
        axis.scatter(predicted[:, first], predicted[:, second], color=_AMBER, marker="x", s=30, label="Transformed")
        for index, (start, end) in enumerate(zip(reference, predicted)):
            axis.plot([start[first], end[first]], [start[second], end[second]], color=_AMBER, linewidth=1)
            axis.annotate(str(index + 1), (start[first], start[second]), fontsize=8, xytext=(3, 4), textcoords="offset points")
        axis.set(xlabel=f"Base {'xyz'[first]} (m)", ylabel=f"Base {'xyz'[second]} (m)", title=f"{'XYZ'[first]}{'XYZ'[second]} projection")
        axis.set_aspect("equal", adjustable="datalim")
        axis.legend(fontsize=7)
    _save(figure, directory, "heldout-points-projections.png", "Orthographic point comparison, actual residual scale / 正交投影对比，残差按实际比例显示", figures)

    figure, axes = _figure()
    axis = axes[0, 0]
    errors = np.asarray(data["error_mm"])
    tolerance_mm = data["tolerance_m"] * 1000
    axis.bar(_labels(axis, len(reference)), errors, color=[_TEAL if value <= tolerance_mm else _AMBER for value in errors])
    axis.axhline(tolerance_mm, color=_AMBER, linestyle="--", linewidth=1.3, label=f"Tolerance {tolerance_mm:g} mm")
    axis.set(xlabel="Corresponding point number", ylabel="Euclidean residual (mm)",
             title="Held-out point tolerance: " + ("PASS" if data["passed"] else "FAIL"))
    axis.set_ylim(bottom=0, top=max(tolerance_mm, float(errors.max())) * 1.18)
    axis.legend(fontsize=8)
    _save(figure, directory, "heldout-errors.png", "Held-out errors against the supplied tolerance / 验证误差与指定容差", figures)


def _metric(value, suffix=""):
    return "—" if value is None else f"{float(value):.3f}{suffix}"


def _notes(data):
    if not data or not data.get("notes"):
        return ""
    return '<ul class="notes">' + "".join(f"<li>{escape(str(note))}</li>" for note in data["notes"]) + "</ul>"


def _html(title, intrinsic, handeye, heldout, directory, figures):
    cards, sections = [], []
    if intrinsic:
        cards.append(("Intrinsics / 相机内参", _metric(intrinsic["rms_px"], " px"), "Corner reprojection RMS / 角点重投影 RMS"))
        sections.append(f'<section id="intrinsic"><h2>1. Intrinsics / 相机内参</h2><p>Image reprojection fit and distortion review. Per-view board poses are fitted from the same corners; low error alone does not validate the camera model.<br>检查图像重投影拟合与畸变。每幅图像的标定板位姿来自同一组角点；误差小不能单独证明相机模型正确。</p><p>Accepted / 采用：{intrinsic["accepted_count"]} · Rejected / 未采用：{intrinsic["rejected_count"]} · Source / 来源：{escape(str(intrinsic.get("source", "unspecified")))}</p>')
        if not intrinsic["accepted_count"]:
            sections.append('<p class="warning">No usable board views. Intrinsic quality cannot be assessed. / 没有可用的标定板图像，无法评估内参质量。</p>')
        rejected = [view for view in intrinsic["views"] if view["status"] == "rejected"]
        if rejected:
            sections.append('<details><summary>Rejected images / 未采用的图像</summary><ul>' + "".join(f'<li>{escape(Path(view["image_path"]).name)}: {escape(str(view.get("reason", "unspecified")))}</li>' for view in rejected) + '</ul></details>')
        sections.append(_notes(intrinsic) + _images(directory, figures, "intrinsic-") + '</section>')
    if handeye:
        cards.append(("Hand-eye / 手眼外参", _metric(handeye["translation_rms_mm"], " mm"), "Board-to-wrist consistency RMS / 板相对腕部一致性 RMS"))
        spread = handeye.get("wrist_spread") or {"max_pairwise_translation_mm": None, "max_pairwise_rotation_deg": None}
        unavailable = '' if handeye["samples"] else '<p class="warning">No hand-eye sample diagnostics / 未提供手眼样本诊断。Only the supplied transform is shown. / 仅展示所提供的坐标变换。</p>'
        sections.append(f'<section id="handeye"><h2>2. Hand-eye / 手眼外参</h2>{unavailable}<p>The board should remain fixed relative to the wrist across samples. These residuals show sample consistency; they are not independent accuracy measurements.<br>不同样本中的标定板应保持相对于腕部固定。这些残差反映样本一致性，不是独立的精度测量。</p><p>Translation RMS / 平移 RMS：{_metric(handeye["translation_rms_mm"], " mm")} · Rotation RMS / 旋转 RMS：{_metric(handeye["rotation_rms_deg"], "°")}<br>Maximum pairwise wrist spread / 腕部样本最大两两跨度：{_metric(spread["max_pairwise_translation_mm"], " mm")} · {_metric(spread["max_pairwise_rotation_deg"], "°")}<br>Camera / 相机：{escape(str(handeye.get("camera_id", "unspecified")))} · Side / 侧别：{escape(str(handeye.get("side", "unspecified")))} · Source / 来源：{escape(str(handeye.get("source", "unspecified")))}</p>' + _notes(handeye) + _images(directory, figures, "handeye-") + '</section>')
    if heldout:
        passed = heldout["passed"]
        status = "PASS / 通过" if passed else "FAIL / 未通过"
        cards.append(("Held-out points / 独立验证点", _metric(heldout["max_error_mm"], " mm"), "Maximum error / 最大误差 · " + status))
        sections.append(f'<section id="heldout"><h2>3. Held-out points / 独立验证点</h2><p class="{"pass" if passed else "warning"}">Supplied point tolerance: <strong>{status}</strong> · {_metric(heldout["tolerance_m"] * 1000, " mm")}<br>所提供点集的容差检查：<strong>{status}</strong>。此结果不会自动接受标定。</p><p>Reference points must be independently measured and excluded from fitting. The report cannot establish their independence. A tolerance pass only describes this point set and does not authorize robot motion.<br>参考点必须独立测量，且不参与拟合；报告无法判断其是否独立。容差通过仅说明该点集的结果，不授权机器人运动。</p>' + _notes(heldout) + _images(directory, figures, "heldout-") + '</section>')
    else:
        cards.append(("Held-out points / 独立验证点", "NOT PROVIDED / 未提供", "Independent accuracy remains unverified / 独立精度尚未验证"))
    card_html = "".join(f'<article class="card"><h2>{escape(label)}</h2><strong>{escape(value)}</strong><p>{escape(caption)}</p></article>' for label, value, caption in cards)
    missing = '' if heldout else '<p class="warning">No held-out validation / 未提供独立验证点。Fit and consistency plots alone cannot establish calibration accuracy. / 仅凭拟合和一致性图无法确认标定精度。</p>'
    if intrinsic and not intrinsic["accepted_count"]:
        next_step = "Capture clear chessboard images, then fit again. / 重新采集清晰的标定板图像，再计算内参。"
    elif heldout and not heldout["passed"]:
        next_step = "Recheck the point measurements and camera alignment before applying calibration. / 应用标定前，重新检查验证点测量与相机外参。"
    elif heldout:
        next_step = "The supplied points meet the tolerance. Apply the result with the same measurements in the CLI. / 所提供的验证点满足容差；在 CLI 中使用同一组测量应用结果。"
    elif handeye:
        next_step = "Measure independent check points, then validate and apply the result in the CLI. / 测量独立验证点，再通过 CLI 验证并应用结果。"
    else:
        next_step = "Check the corner overlay and distortion comparison below, then apply intrinsics and collect hand-eye samples. / 查看下方角点与畸变对比后，应用内参并采集手眼样本。"
    return f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; img-src data:; style-src 'unsafe-inline'; base-uri 'none'; form-action 'none'">
<title>{escape(title)}</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#f1f5f9;color:#1e293b;font:15px/1.6 system-ui,-apple-system,"Segoe UI",sans-serif}}main{{max-width:1160px;margin:auto;padding:28px 24px 60px}}header{{margin-bottom:24px}}h1{{font-size:29px;line-height:1.3;letter-spacing:-.5px;margin:0 0 12px}}h2{{font-size:20px;margin:0 0 10px}}p{{margin:10px 0}}a{{color:#0f766e}}.eyebrow{{text-transform:uppercase;letter-spacing:.1em;font-size:12px;color:#0f766e;font-weight:700}}.badge{{display:inline-block;border:1px solid #cbd5e1;border-radius:5px;padding:3px 9px;font-size:12px;background:white}}.warning,.pass{{background:#fffbeb;border-left:4px solid #d97706;padding:12px 16px;border-radius:4px}}.pass{{background:#f0fdfa;border-color:#0f766e}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(235px,1fr));gap:14px;margin:20px 0}}.card,section{{background:white;border:1px solid #e2e8f0;border-radius:10px;padding:20px}}.card h2{{font-size:15px;color:#475569}}.card strong{{font-size:23px;color:#0f766e}}.card p{{font-size:12px;color:#64748b}}section{{margin:18px 0}}section>p,.notes{{font-size:14px}}figure{{margin:22px 0}}figure img{{width:100%;height:auto;display:block;border:1px solid #e2e8f0;border-radius:6px}}figcaption{{font-size:13px;color:#475569;margin-top:7px;overflow-wrap:anywhere}}details{{padding:10px;background:#f8fafc;border-radius:6px}}summary{{cursor:pointer}}footer{{font-size:13px;color:#64748b}}.notes{{color:#64748b}}@media(max-width:600px){{main{{padding:18px 12px}}h1{{font-size:25px}}section,.card{{padding:15px}}}}
</style></head><body><main><header><p class="eyebrow">TRON2 · Calibration result / 标定结果</p><h1>{escape(title)}</h1><span class="badge">Report review: UNVERIFIED / 报告未更改标定验证状态</span></header><div class="cards">{card_html}</div><section><h2>Next step / 下一步</h2><p>{next_step}</p></section><details><summary>Optional details and plots / 按需查看详细图表</summary><p>This report visualizes existing results. It does not change calibration verification, profiles, or execution settings.<br>此报告仅可视化已有结果，不更改标定验证状态、配置或执行设置。</p>{missing}{''.join(sections)}<p><a href="metrics.json" download>Download metrics JSON / 下载指标</a> · Images are embedded; this HTML opens offline. / 图片已嵌入，可离线打开此 HTML。</p></details></main></body></html>'''


def _images(directory, figures, prefix):
    blocks = []
    for figure in figures:
        if not figure["filename"].startswith(prefix):
            continue
        encoded = base64.b64encode((directory / figure["filename"]).read_bytes()).decode("ascii")
        caption = escape(figure["caption"])
        filename = escape(figure["filename"], quote=True)
        blocks.append(f'<figure><img src="data:image/png;base64,{encoded}" alt="{caption}" loading="lazy"><figcaption>{caption} · <a href="{filename}" download>PNG</a></figcaption></figure>')
    return "".join(blocks)


def _empty_destination(path):
    if path.is_symlink() or (path.exists() and (not path.is_dir() or any(path.iterdir()))):
        raise ValueError("report output directory must be new or empty; existing reports are never overwritten")


def write_calibration_report(output_dir, *, intrinsic=None, handeye=None, heldout=None,
                             title="Calibration review"):
    """Write PNG exports, JSON metrics, and an HTML file with embedded images.

    Inputs are JSON-safe dictionaries from ``calibration_diagnostics``. Source
    intrinsic images must still exist at their recorded paths. No files in an
    existing nonempty output directory are overwritten. Failed rendering leaves
    no partial report behind. The return paths are absolute strings.
    """
    destination = Path(output_dir).expanduser().absolute()
    _empty_destination(destination)
    try:
        _validate(intrinsic, handeye, heldout)
        payload = {"schema_version": 1, "title": str(title), "review_status": "unverified",
                   "calibration_modified": False, "intrinsic": intrinsic, "handeye": handeye, "heldout": heldout}
        metrics = json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError(f"malformed calibration diagnostics: {exc}") from exc
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".calibration-report-", dir=destination.parent))
    figures = []
    try:
        if intrinsic:
            _render_intrinsic(intrinsic, temporary, figures)
        if handeye:
            _render_handeye(handeye, temporary, figures)
        if heldout:
            _render_heldout(heldout, temporary, figures)
        (temporary / "metrics.json").write_text(metrics, encoding="utf-8")
        (temporary / "index.html").write_text(_html(str(title), intrinsic, handeye, heldout, temporary, figures), encoding="utf-8")
        _empty_destination(destination)
        if destination.exists():
            destination.rmdir()
        temporary.rename(destination)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return {"report_path": str(destination / "index.html"), "metrics_path": str(destination / "metrics.json"),
            "figure_paths": [str(destination / figure["filename"]) for figure in figures]}
