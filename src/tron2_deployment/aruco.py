"""ArUco calibration targets: one marker or one rigid multi-marker board.

Target geometry is never hard-coded.  A JSON spec under ``configs/`` names the
dictionary and the measured marker size, and ``kind`` selects a single marker
or a board.  Both kinds are normalised into one layout -- a single marker is a
one-cell board -- so collection, solving and reporting share a single code path.

The target frame is the standard ArUco frame of ``frame_marker_id``: its origin
is that marker's printed top-left corner, +x runs along its printed right edge,
+y along its printed bottom edge, and +z completes the right-handed frame by
pointing into the printed plane, away from a viewer facing the mark.
``tests/test_aruco.py`` pins that convention by projecting a rendered target
from a known pose instead of trusting any implied detector convention.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

DEFAULT_MIN_SOLUTION_RATIO = 2.0
DEFAULT_MIN_VISIBLE_MARKERS = 2
# Below this reprojection RMS the two planar solutions are both "exact", so the
# ratio test is meaningless and the floor keeps it from degenerating.
RMS_FLOOR_PX = 0.05
MAX_MARKERS = 64
_COMMON_KEYS = {"schema_version", "kind", "dictionary", "marker_length_m",
                "min_solution_ratio", "spec_file", "spec_sha256"}
_MARKER_KEYS = {"marker_id"}
_BOARD_KEYS = {"marker_separation_m", "markers_x", "markers_y", "first_marker_id",
               "marker_ids", "frame_marker_id", "min_visible_markers"}


def _aruco():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "OpenCV was built without the aruco module; install opencv-contrib-python")
    return cv2.aruco


def _dictionary(name):
    aruco = _aruco()
    if not isinstance(name, str) or not name.startswith("DICT_"):
        raise ValueError("dictionary must be a cv2.aruco DICT_* name")
    flag = getattr(aruco, name, None)
    if flag is None:
        raise ValueError(f"unknown ArUco dictionary {name!r}")
    predefined = aruco.getPredefinedDictionary(flag)
    return predefined, int(predefined.bytesList.shape[0])


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return float(value)


def _non_negative(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and not negative")
    return float(value)


def _count(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


def _marker_id(value, capacity, name):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < capacity:
        raise ValueError(f"{name} must be an integer between 0 and {capacity - 1}")
    return int(value)


def validate_target_spec(value):
    """Normalise a spec so one marker and a board share the same layout fields."""
    if not isinstance(value, dict):
        raise ValueError("target spec must be a JSON object")
    if value.get("schema_version") != 1:
        raise ValueError("target spec schema_version must be 1")
    kind = value.get("kind")
    if kind not in ("marker", "board"):
        raise ValueError("target spec kind must be marker or board")
    allowed = _COMMON_KEYS | (_MARKER_KEYS if kind == "marker" else _BOARD_KEYS)
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError(f"target spec has unknown fields: {unknown}")
    dictionary, capacity = _dictionary(value.get("dictionary"))
    length = _positive(value.get("marker_length_m"), "marker_length_m")
    ratio = _positive(value.get("min_solution_ratio", DEFAULT_MIN_SOLUTION_RATIO),
                      "min_solution_ratio")
    if ratio <= 1:
        raise ValueError("min_solution_ratio must be greater than 1")
    common = {"schema_version": 1, "kind": kind, "dictionary": value["dictionary"],
              "marker_length_m": length, "min_solution_ratio": ratio}
    if kind == "marker":
        marker_id = _marker_id(value.get("marker_id"), capacity, "marker_id")
        return {**common, "marker_ids": [marker_id], "markers_x": 1, "markers_y": 1,
                "marker_separation_m": 0.0, "frame_marker_id": marker_id,
                "min_visible_markers": 1}
    columns = _count(value.get("markers_x"), "markers_x")
    rows = _count(value.get("markers_y"), "markers_y")
    if columns * rows > MAX_MARKERS:
        raise ValueError(f"target spec supports at most {MAX_MARKERS} markers")
    separation = _non_negative(value.get("marker_separation_m"), "marker_separation_m")
    listed = value.get("marker_ids")
    if listed is not None and "first_marker_id" in value:
        raise ValueError("provide either marker_ids or first_marker_id, not both")
    if listed is None:
        first = _marker_id(value.get("first_marker_id", 0), capacity, "first_marker_id")
        listed = list(range(first, first + columns * rows))
    if not isinstance(listed, list) or len(listed) != columns * rows:
        raise ValueError(f"marker_ids must list {columns * rows} ids in row-major order")
    marker_ids = [_marker_id(item, capacity, "marker_ids") for item in listed]
    if len(set(marker_ids)) != len(marker_ids):
        raise ValueError("marker_ids must be unique")
    frame = value.get("frame_marker_id", marker_ids[0])
    if frame not in marker_ids:
        raise ValueError("frame_marker_id must appear in marker_ids")
    visible = value.get("min_visible_markers", min(DEFAULT_MIN_VISIBLE_MARKERS, len(marker_ids)))
    if isinstance(visible, bool) or not isinstance(visible, int) or not 1 <= visible <= len(marker_ids):
        raise ValueError(f"min_visible_markers must be between 1 and {len(marker_ids)}")
    return {**common, "marker_separation_m": separation, "markers_x": columns,
            "markers_y": rows, "marker_ids": marker_ids, "frame_marker_id": frame,
            "min_visible_markers": visible}


def fingerprint(spec):
    payload = {key: value for key, value in spec.items() if key not in ("spec_file", "spec_sha256")}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def load_target_spec(path):
    """Read, validate and fingerprint one user-supplied target spec."""
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"target spec file does not exist: {path}")
    try:
        value = json.loads(path.read_text())
    except json.JSONDecodeError as error:
        raise ValueError(f"target spec is not valid JSON: {error}") from error
    spec = validate_target_spec(value)
    spec["spec_file"] = str(path)
    spec["spec_sha256"] = fingerprint(spec)
    return spec


def target_record(spec):
    """Provenance block stored in session, sample and solution files."""
    return dict(spec)


def layout(spec):
    """marker id -> (column, row), row-major from marker_ids."""
    columns = spec["markers_x"]
    return {marker_id: (index % columns, index // columns)
            for index, marker_id in enumerate(spec["marker_ids"])}


def layout_extent(spec):
    """(width_m, height_m) of the whole rigid layout in the target frame."""
    length = spec["marker_length_m"]
    pitch = length + spec["marker_separation_m"]
    return ((spec["markers_x"] - 1) * pitch + length,
            (spec["markers_y"] - 1) * pitch + length)


def layout_center(spec):
    """Centre of the layout expressed in the target frame."""
    width, height = layout_extent(spec)
    column, row = layout(spec)[spec["frame_marker_id"]]
    pitch = spec["marker_length_m"] + spec["marker_separation_m"]
    return np.array([width / 2 - column * pitch, height / 2 - row * pitch, 0.0])


def marker_object_points(spec, marker_id):
    """The four printed corners of one marker, in the target frame.

    Corner order matches ``detectMarkers``: top-left, top-right, bottom-right,
    bottom-left as printed, which is what pairs a square with its image points.
    """
    layout_map = layout(spec)
    if marker_id not in layout_map:
        raise ValueError(f"marker {marker_id} is not part of this target spec")
    length = spec["marker_length_m"]
    pitch = length + spec["marker_separation_m"]
    column, row = layout_map[marker_id]
    frame_column, frame_row = layout_map[spec["frame_marker_id"]]
    offset = np.array([(column - frame_column) * pitch, (row - frame_row) * pitch, 0.0],
                      dtype=np.float32)
    square = np.array([[0, 0, 0], [length, 0, 0], [length, length, 0], [0, length, 0]],
                      dtype=np.float32)
    return square + offset


def detect_markers(image, spec):
    """Every known marker in the frame: {id: (1, 4, 2) corners} plus id order.

    Lenient on purpose so the console can draw whatever it sees; the collection
    and solving paths apply ``min_visible_markers`` on top of this.
    """
    aruco = _aruco()
    dictionary, _ = _dictionary(spec["dictionary"])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    parameters = aruco.DetectorParameters()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = aruco.ArucoDetector(dictionary, parameters).detectMarkers(gray)
    if ids is None:
        return {}, []
    known = set(spec["marker_ids"])
    found = {}
    for value, corner in zip(ids.reshape(-1), corners):
        marker_id = int(value)
        if marker_id not in known:
            continue
        if marker_id in found:
            raise ValueError(f"marker {marker_id} was detected more than once in one frame")
        found[marker_id] = np.asarray(corner, dtype=np.float32).reshape(1, 4, 2)
    return found, [value for value in spec["marker_ids"] if value in found]


def _reprojection_rms(object_points, image_points, rotation, translation, k, dist):
    predicted = cv2.projectPoints(object_points, Rotation.from_matrix(rotation).as_rotvec(),
                                  translation, k, dist)[0].reshape(-1, 2)
    observed = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((predicted - observed) ** 2, axis=1))))


def estimate_pose(image, spec, K, dist):
    """(4, 4) target-frame pose in the camera frame, plus a detail block.

    Every visible marker is solved in one planar fit, so a board keeps its
    rigidity instead of averaging per-marker poses.  A planar fit always has a
    second solution; the frame is rejected unless that alternative fits clearly
    worse, which is the criterion that actually separates good from bad views.
    """
    found, order = detect_markers(image, spec)
    minimum = spec["min_visible_markers"]
    if len(order) < minimum:
        raise ValueError(f"target needs {minimum} visible marker(s); detected {len(order)}")
    object_points = np.vstack([marker_object_points(spec, value) for value in order])
    image_points = np.vstack([found[value].reshape(-1, 2) for value in order])
    k = np.asarray(K, dtype=np.float64)
    d = np.asarray(dist, dtype=np.float64).reshape(-1, 1)
    # SOLVEPNP_IPPE solves an arbitrary planar correspondence, so one marker and
    # a board share it.  SOLVEPNP_IPPE_SQUARE is rejected here on purpose: it
    # requires centre-origin object points in its own ordering, which would
    # silently return a pose in a different frame than the documented one.
    count, rvecs, tvecs, _ = cv2.solvePnPGeneric(object_points, image_points, k, d,
                                                 flags=cv2.SOLVEPNP_IPPE)
    if int(count) < 2:
        raise ValueError("planar target solve did not return the two expected solutions")
    rotations = [cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))[0] for rvec in rvecs[:2]]
    errors = [_reprojection_rms(object_points, image_points, rotation,
                                np.asarray(tvec, dtype=np.float64).reshape(3), k, d)
              for rotation, tvec in zip(rotations, tvecs[:2])]
    best = int(np.argmin(errors))
    alternative = errors[1 - best]
    if alternative < max(errors[best], RMS_FLOOR_PX) * spec["min_solution_ratio"]:
        raise ValueError(
            f"target pose is ambiguous: the alternative planar solution also fits "
            f"({alternative:.2f} px versus {errors[best]:.2f} px); bring the target closer, "
            f"tilt it further, or use more markers")
    span = float(np.degrees(
        Rotation.from_matrix(rotations[0].T @ rotations[1]).magnitude()))
    pose = np.eye(4)
    pose[:3, :3] = rotations[best]
    pose[:3, 3] = np.asarray(tvecs[best], dtype=np.float64).reshape(3)
    return pose, {"marker_ids": list(order), "visible_markers": len(order),
                  "solution_span_deg": span, "alternative_rms_px": float(alternative),
                  "reprojection_rms_px": float(errors[best])}


def render_target(spec, size, rotation, translation, K, dist=None, *,
                  background=210, pixels_per_m=2000):
    """Perspective view of the whole layout for a known target-frame pose.

    Used by the mock walkthrough and the synthetic tests; it never touches a
    camera.  The printed quiet zone comes from the white texture background.
    """
    aruco = _aruco()
    dictionary, _ = _dictionary(spec["dictionary"])
    length = spec["marker_length_m"]
    pitch = length + spec["marker_separation_m"]
    width_m, height_m = layout_extent(spec)
    scale = float(pixels_per_m)
    columns, rows = int(round(width_m * scale)), int(round(height_m * scale))
    side = int(round(length * scale))
    texture = np.full((rows, columns, 3), 255, np.uint8)
    for marker_id in spec["marker_ids"]:
        column, row = layout(spec)[marker_id]
        marker = aruco.generateImageMarker(dictionary, marker_id, side)
        if marker.ndim == 2:
            marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        x0, y0 = int(round(column * pitch * scale)), int(round(row * pitch * scale))
        texture[y0:y0 + side, x0:x0 + side] = marker[:side, :side]
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    outer = np.array([[0, 0, 0], [width_m, 0, 0], [width_m, height_m, 0],
                      [0, height_m, 0]], np.float32)
    projected = cv2.projectPoints(outer, Rotation.from_matrix(rotation).as_rotvec(),
                                  translation, np.asarray(K, dtype=np.float64),
                                  np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)
                                  )[0].reshape(4, 2)
    homography = cv2.getPerspectiveTransform(
        np.array([[0, 0], [columns, 0], [columns, rows], [0, rows]], np.float32), projected)
    height, width = int(size[0]), int(size[1])
    return cv2.warpPerspective(texture, homography, (width, height),
                               borderValue=(int(background),) * 3)
