# Minimal Head-Camera Calibration Experiment

[简体中文](calibration.zh-CN.md)

This directory implements one minimal workflow:

1. capture about 30 fixed-checkerboard images while moving head yaw and pitch;
2. solve color-camera intrinsics;
3. solve `T_pitch_camera`, from the color optical frame to `head_pitch_Link`;
4. validate three checkerboard corners by touching them with a calibrated arm tip.

The program does not command the head, arms, hands, or grippers. Perform motion manually through the robot's existing reviewed interface. Inputs and outputs use JSON, not YAML.

## Final model and frame convention

The final dexterous-hand models are authoritative:

- `configs/assembly.urdf` supplies kinematics;
- `configs/scene.xml` cross-checks the same head-camera chain;
- `head_camera_color_optical_frame` is the calibrated OpenCV color frame (+x right, +y down, +z forward);
- `head_pitch_Link` is the camera's nearest moving pitch frame.

The old `d435_Link` attached directly to `base_Link` is ignored. The nominal transform in the final URDF is reported only for comparison and does not constrain the calibration.

`T_A_B` maps coordinates from B into A. The head chain is:

```text
T_base_pitch(q_yaw, q_pitch)
  = T_base_headbase
  · T_headbase_yaw(q_yaw)
  · T_yaw_pitch_origin
  · R_pitch(q_pitch)
```

The yaw-to-pitch offset is `[0.051, 0.03, 0.097] m`. Yaw therefore moves the pitch origin; pitch rotates about its own origin and does not change yaw or move that origin. Captures store `[pitch, yaw]`, but the implementation maps values by joint name and follows the URDF yaw→pitch hierarchy.

Every extrinsic solve checks the relevant transforms from `assembly.urdf` against `scene.xml` at three head poses and stops if they disagree.

## Checkerboard and dependencies

The current board has 7×10 **inner corners** and `0.021 m` squares, meaning 8×11 printed squares. Use a flat physical board with a white outer border.

As in the earlier workflow, board geometry can be supplied directly as `--pattern COLSxROWS --square-m METRES`. Command-line values override JSON, and the program rejects disagreement between intrinsic, extrinsic, and validation stages.

```bash
cp sp_vision/config.example.json sp_vision/local-calibration.json
.venv/bin/python -m pip install -r sp_vision/requirements.txt
```

The example config already references `configs/assembly.urdf`, `configs/scene.xml`, and the existing local camera profile. Relative paths are resolved from the config JSON.

## 1. Capture 30 views

Rigidly fix the board where the camera and arm can both reach it. Do not move it during this dataset.

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/head-camera-session --count 30
```

Keys are:

- `s`: save an accepted frame;
- `r`, Space, or another ordinary key: discard and reacquire without saving;
- `f`: reverse corner order by 180° so `(0,0)` remains the same physical corner;
- `q` or Esc: quit.

Detection uses `findChessboardCornersSB` with `NORMALIZE_IMAGE`, `EXHAUSTIVE`, and `ACCURACY`. It must find all 70 subpixel corners and pass topology, coverage, border, and synchronization checks. Colored row overlays are saved as `corners.png` for review.

Move through combinations spanning positive and negative yaw and pitch, and wait for the head to stop before each save. The final six frames are held out; the preceding roughly 24 frames are fitted.

## 2. Solve intrinsics

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json intrinsics \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/head-camera-session \
  --output sp_vision/data/head-camera-session/intrinsics.json
```

Require `passed: true`. Review per-view RMS, the saved diagnostic overlays, and the radial-monotonicity result. Do not continue if focal length or principal point is implausibly different from factory values. Use a physical board; a checkerboard shown on a laptop introduces moiré and is unsuitable for final calibration.

## 3. Solve camera-to-pitch extrinsics

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json extrinsics \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/head-camera-session \
  --intrinsics sp_vision/data/head-camera-session/intrinsics.json \
  --output sp_vision/data/head-camera-session/extrinsics.json
```

For every frame, IPPE PnP provides `T_camera_board`, and final-URDF FK provides `T_base_pitch`. The fixed board imposes:

```text
T_base_pitch(i) · T_pitch_camera · T_camera_board(i)
  = T_base_board
```

Require `passed: true`. Use `T_pitch_camera` as the measured result. `nominal_T_pitch_camera` is only the final-URDF reference; `training`, `holdout`, and `model_consistency` provide the essential checks.

At runtime:

```text
T_base_camera(q_yaw, q_pitch)
  = T_base_pitch(q_yaw, q_pitch) · T_pitch_camera
```

Task yaw and pitch therefore need not match a calibration pose; use the live joint angles.

## 4. Calibrate the touch tip

Use a sharp point rigidly fixed relative to the wrist. With a dexterous hand, prefer a rigid probe. A fingertip is valid only if every finger joint remains fixed throughout pivot fitting and validation, because state JSON does not contain finger joints.

Touch one fixed point at four clearly different wrist orientations and read state after each settles:

```bash
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/tcp/pose-01.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/tcp/pose-02.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/tcp/pose-03.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/tcp/pose-04.json

.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json pivot --side left \
  --states sp_vision/data/tcp/pose-{01,02,03,04}.json \
  --output sp_vision/data/tcp/result.json
```

Require `passed: true`. This fits touch-point position, not tool orientation.

## 5. Touch three validation corners

Without moving the board, capture one new image after calibration:

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/touch-validation --count 1
```

Select three spread-out, non-collinear corners. Omit `--corner` for click-and-snap selection.

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json select-validation \
  --pattern 7x10 --square-m 0.021 \
  --frame sp_vision/data/touch-validation/view-001 \
  --intrinsics sp_vision/data/head-camera-session/intrinsics.json \
  --extrinsics sp_vision/data/head-camera-session/extrinsics.json \
  --corner 0,0 --corner 0,6 --corner 9,3 \
  --output sp_vision/data/touch-validation/selection.json
```

Inspect `selection.png`. Keep the board fixed, touch labeled points 1, 2, and 3 in order, and read state in the same order. The head may move after imaging: prediction uses the image-synchronized `head_q2`, while later head motion is recorded only as a diagnostic and does not affect the error gate.

```bash
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/touch-validation/state-01.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/touch-validation/state-02.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/touch-validation/state-03.json

.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json validate \
  --selection sp_vision/data/touch-validation/selection.json \
  --side left --tcp sp_vision/data/tcp/result.json \
  --states sp_vision/data/touch-validation/state-{01,02,03}.json \
  --output sp_vision/data/touch-validation/report.json
```

The comparison is:

```text
p_base_camera
  = T_base_pitch(head_q2) · T_pitch_camera · T_camera_board · p_board_corner

p_base_touch
  = T_base_wrist(arm_q14) · p_wrist_tip
```

The metric is 3D Cartesian distance in `base_Link`, not joint-angle difference. The default maximum is 10 mm; replace it with a limit justified by probe repeatability, board mounting, and task clearance. The program sends no motion commands—use supervision and low speed, and do not push the board.

## Offline check

```bash
.venv/bin/python -m pytest -q sp_vision/test_calibration.py
```

This checks 7×10 SB detection, hand-eye transform direction, final-model FK, yaw/pitch origin behavior, and URDF/XML agreement without connecting to hardware.

## Common failures

- `expected 70 inner corners`: check the inner-corner count and avoid blur, glare, occlusion, and display moiré.
- `(0,0)` jumps to the opposite corner: press `f` during capture before saving.
- `joint/image header skew exceeded`: wait for the head to settle and reacquire.
- `insufficient head excitation`: add combined poses spanning both yaw and pitch directions.
- URDF/XML consistency failure: fix the final model rather than compensating with an old transform.
- Large touch error with low reprojection error: inspect TCP calibration, finger posture, board motion, touch order, and head motion after imaging.
