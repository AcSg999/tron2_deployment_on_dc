# Head and right wrist camera calibration results

[简体中文](camera-calibration-results-20261110.zh-CN.md) · [Tracking check](wrist-in-head-projection.md)

This page preserves the historical 2026-11-10 results for `head-20261110-220522__wrist-20261109-223333`. The active validation binding was updated on 2026-11-11 to `head-20261110-220522__wrist-20261111-180956`; see the [current validation configuration](wrist-in-head-projection.md).

In the historical version, the supplied alternative head pair comes from `/home/dc/calibration/tron2_camera_calibration/sp_vision/data/head_camera_session`; the right wrist uses `wrist-20261109-223333`. All four parameter files were copied byte for byte into a versioned bundle. No calibration parameters were refitted or adjusted using validation observations.

Both cameras' intrinsic and extrinsic files record `passed: true` for their fitting gates. Each uses 34 training views and 6 held-out views, with no rejected views or outliers. Both use 640×480 images, 7×10 inner corners and 0.021 m squares. Independent physical accuracy remains unverified.

## Intrinsics

Focal lengths and principal points are in pixels. Values below are rounded; the JSON files retain full precision. These parameters apply to the calibrated image stream geometry.

```text
K = [[fx,  0, cx],
     [ 0, fy, cy],
     [ 0,  0,  1]]
```

| Camera | fx | fy | cx | cy | Reprojection RMS |
|---|---:|---:|---:|---:|---:|
| Head | 404.160815 | 408.624959 | 320.203505 | 230.261438 | 0.081087 px |
| Right wrist | 433.016129 | 433.630983 | 322.649964 | 231.992261 | 0.152248 px |

Distortion order: `[k1, k2, p1, p2, k3]`.

```text
head_D = [-0.019421023, 0.069445925, -0.019398520, -0.001448037, -0.036887054]
wrist_D = [-0.060437013, -0.018042387, -0.002824731, 0.000885365, 0.320308046]
```

## Extrinsics

Convention: `p_A = T_A_B @ p_B`. The last column is translation in meters. At measured joint state `q`, `T_base_camera(q) = T_base_parent(q) @ T_parent_camera`.

Head: `T_pitch_camera`, from camera optical coordinates to `head_pitch_Link`.

```text
[
  [ 0.011444, -0.010666,  0.999878,  0.034868],
  [-0.999919,  0.005495,  0.011504, -0.048125],
  [-0.005617, -0.999928, -0.010602,  0.061620],
  [ 0.000000,  0.000000,  0.000000,  1.000000]
]
```

Right wrist: `T_wrist_roll_camera`, from camera optical coordinates to `wrist_roll_R_Link`.

```text
[
  [-0.025944, -0.986669, -0.160661,  0.034908],
  [-0.999620,  0.027111, -0.005078,  0.009521],
  [ 0.009366,  0.160469, -0.986997, -0.092557],
  [ 0.000000,  0.000000,  0.000000,  1.000000]
]
```

## Fitting residuals

These are the supplied results' maximum per-view checkerboard pose residuals relative to the common fitted board pose in the base frame. The configured limits are 15 mm and 2°. Held-out images are excluded from fitting. Rechecking the supplied head pair against its original images reproduced the recorded residuals within 0.040 mm and 0.010°, without changing K, distortion or either fitted transform.

| Camera | Dataset | Views | Maximum translation residual | Maximum rotation residual |
|---|---|---:|---:|---:|
| Head | Training | 34 | 5.708 mm | 1.200° |
| Head | Holdout | 6 | 5.641 mm | 1.015° |
| Right wrist | Training | 34 | 9.224 mm | 0.960° |
| Right wrist | Holdout | 6 | 7.947 mm | 0.885° |

## Independent paired-board validation

The saved pair `board-20261110-201025`, whose physical corner IDs were previously reviewed, was replayed using the supplied alternative parameters. Its RGB hashes do not occur in the supplied head calibration dataset. No flip flag or geometric correction was applied.

| Metric | Alternative head pair | Limit | Result |
|---|---:|---:|---|
| Relative translation difference | 25.708 mm | 15 mm | Exceeds limit |
| Relative rotation difference | 1.746° | 2° | Within limit |

Own-camera PnP RMS was 0.108 px for head and 0.162 px for wrist; cross-projection RMS was 12.344 px / 21.065 px. Status is `relative_pose_exceeds_limits`. This is one relative-chain observation; it does not isolate an individual camera/FK error or establish absolute accuracy. Collect additional stationary board pairs at different arm poses, keeping the same board geometry and independently checking corner identities. Keep validation observations separate from fitting data.

Result: [alternative-parameter board check](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/validation/board-201025/board-check.json).

## Historical manual-reference replay: 19:02 head version

The following historical comparison used `head-20261110-190209`, before selecting the supplied alternative head pair. These are not measurements of the current 22:05 version. No parameters or compensating offsets were fitted to the lens clicks. Duplicate annotations of the same image count as one frame.

| Capture round | Unique frames | Previous RMS | 19:02 RMS | 19:02 maximum |
|---|---:|---:|---:|---:|
| Arm varies, head fixed | 5 | 43.38 px | 17.99 px | 32.64 px |
| Head varies, arm fixed | 5 | 34.98 px | 6.38 px | 10.05 px |

Projection improved, but residuals remain. These are pixel distances to a user-selected visible RGB lens feature, whose equivalence to the internal optical center is unverified. They do not establish millimeter or angular accuracy. See the [comparison plot](../output/head-recalibration-20261110-190209/replay-comparison.png).

With that historical 19:02 head version, the paired-board observation measured 17.308 mm and 2.069° relative disagreement. Three further nearly same-pose samples reproduced approximately 17.3 mm and 2.06°. These exceeded the 15 mm / 2° relative-consistency limits. The responsible camera/FK component remains unresolved. See the [historical reviewed observation](../output/wrist-in-head/board-20261110-201025-reviewed/board-check.json) and [repeatability report](../output/wrist-in-head/board-20261110-201025-analysis/summary.json).

## Difference from the nominal URDF mount

`calibrated_from_nominal` compares the fitted mount with the URDF nominal mount in the same parent frame. It describes disagreement between two geometric estimates, not an independently measured URDF error.

| Camera | Translation difference | Rotation difference |
|---|---:|---:|
| Head | 60.742 mm | 0.951° |
| Right wrist | 2.503 mm | 1.753° |

The head value above uses the external solve's nominal camera mount, which differs from the tracking URDF's nominal mount. It is not a common baseline for comparing calibration versions. The actual `base_Link` → `head_pitch_Link` FK chain is identical in both models and is checked by its signature; projection uses the supplied calibrated mount. The previous head session `head-20261109-201636` recorded 92.864 mm and 2.187°; its [previous combined summary](../output/calibration/camera-calibration-head-wrist-20261110.json) remains available.

## Result files

The bundle directory is `/home/dc/tron2_deployment_on_dc/output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/`. The verification tool defaults to the pinned local manifest and prints the loaded calibration ID at startup. An explicit `--calibration-summary PATH` selects another version. File hashes, intrinsic/extrinsic pairing and the relevant FK chain are checked. Changes to the original working files do not modify these snapshots.

| Record | File path |
|---|---|
| Head intrinsics | [head_intrinsics.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/head_intrinsics.json) |
| Head extrinsics | [head_extrinsics.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/head_extrinsics.json) |
| Right wrist intrinsics | [wrist_intrinsics.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/wrist_intrinsics.json) |
| Right wrist extrinsics | [wrist_extrinsics.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/wrist_extrinsics.json) |
| Versioned calibration manifest | [camera-calibration-summary.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/camera-calibration-summary.json) |
| Default local tracking manifest | [/home/dc/tron2_deployment_on_dc/configs/local-camera-calibration-latest.json](/home/dc/tron2_deployment_on_dc/configs/local-camera-calibration-latest.json) |
| Source and compatibility evidence | [provenance.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/provenance.json) |
| Current board validation | [board-check.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/validation/board-201025/board-check.json) |

`rms_px` is intrinsic reprojection RMS. Extrinsic `training` and `holdout` contain `translation_m` in meters and `rotation_rad` in radians. In `calibrated_from_nominal`, translation is in meters; the head uses `rotation_rad`, while the wrist uses `rotation_deg`. The bundle records source file hashes, original head-frame hashes, pairing checks and FK compatibility. The original 19:02 head working files had been overwritten by the supplied alternative files; its historical manifest remains a record, not a loadable snapshot of the original parameters. Local outputs and the local binding file are ignored by Git.
