# Right wrist camera calibration

[简体中文](wrist_calib.zh-CN.md)

This is a separate, read-only experiment for the right wrist color camera. Run commands from this directory. Images and results go under `../output/calibration/sp_vision/wrist_camera_session/`, which is ignored by Git. The bundled model assets keep offline calibration independent from the parent repository. The code never moves the robot.

Controller payload identification is not an input to this geometric calibration. If you use drag teaching to position the arm, configure and read back the current payload in the controller separately; keep those values out of the camera configuration.

## Calibration parameter publication

The latest local head results are `../output/calibration/sp_vision/head-camera-session/intrinsics.json` and `extrinsics.json` (2026-09-23); the latest right wrist results are `../output/calibration/sp_vision/wrist_camera_session/wrist_intrinsics.json` and `wrist_extrinsics.json` (2026-10-05). Their byte-preserving copies and source/hash manifest were published on 2026-10-08 to the dedicated [camera calibration repository](https://github.com/zengxiong111/tron2_camera_calibration/tree/main/calibration_results). Raw captures and local profiles stay Git-ignored here. All four fit results record `passed: true`, but independent touch validation remains `passed: false` (head maximum 14.42 mm, right wrist 12.26 mm, limit 10 mm). The published guide documents frame conventions, image size and the historical head-model limitation; publishing does not establish hardware acceptance.

## Frames and geometry

In `configs/assembly.urdf`, the camera bracket is fixed to `wrist_roll_R_Link`. `wrist_roll_R_Joint` is movable between that link and `wrist_pitch_R_Link`: its local X axis is orthogonal to the pitch joint's local Y axis. The constant to solve is therefore **`T_wrist_roll_camera`**, from `right_wrist_camera_color_optical_frame` into `wrist_roll_R_Link`. The URDF value is a nominal starting reference, not a measured calibration. At any measured arm state:

```text
T_base_camera(q) = T_base_wrist_roll(q) · T_wrist_roll_camera
T_base_board = T_base_camera(q) · T_camera_board
T_wrist_pitch_camera(q_roll)
  = T_wrist_pitch_wrist_roll(q_roll) · T_wrist_roll_camera
```

Seven measured right arm angles enter `T_base_wrist_roll`. Head angles and left arm angles are not part of that FK chain. The capture still saves all 14 arm angles, by name, for audit and for later touch validation.

Camera intrinsics do not depend on how many joints move. They must be calibrated separately for this **640×480 resized wrist image**: the current ROS 2 `CameraInfo` reports 848×480 for the source stream and cannot be used unchanged for the resized topic.

## Prepare and verify state mapping

```bash
cp -n configs/wrist_config.example.json configs/wrist_config.json
```

Keep an existing local configuration. The example defaults to `capture.backend: "ssh"`, which reads ROS 2 topics over SSH. To use the robot's LimX Studio camera WebSocket bridge without SSH, replace the `capture` object in `configs/wrist_config.json` with:

```json
{
  "backend": "bridge",
  "bridge_profile": "../../configs/local-robot-seed.json",
  "state_profile": "../../configs/local-robot-seed.json",
  "ros_domain_id": 0,
  "color_topic": "/camera/right/color/image_resized/compressed",
  "joint_topic": "/joint_states",
  "max_state_skew_ms": 100,
  "timeout_s": 8
}
```

Profile paths are relative to the configuration file. This machine already has the local profile above; on another machine use its actual profile. The bridge integration requires the parent `tron2_deployment` package with its `bridge` extra. It uses `camera.bridge_host`, `bridge_path`, TLS and token settings, but explicitly subscribes to the wrist topic above. The browser need not remain open. A bridge failure is reported without falling back to SSH. The controller state comparison still uses its existing read-only WebSocket connection.

The bridge supplies forwarding timestamps rather than original ROS sensor headers. Saved captures and probe reports identify `timestamp_source: "bridge_forwarding_ms"` and `sensor_header_timestamps_available: false`. The 100 ms limit therefore checks forwarding-time alignment; it does not prove exposure-time synchronization. Both image and state must also have been received within 750 ms. Keep the arm still while sampling and complete the independent touch validation. The optical frame is assigned from the configured camera topic and recorded as `frame_id_source: "configured_camera_topic"`; it is not a received ROS header.

Offline wrist solving only requires saved images and synchronized JSON containing `arm_q14`; neither live transport is needed. The configured name list maps external joint names into the 14-value order expected by the bundled URDF. Physical FK accuracy still needs the held-out board and independent touch checks.

If the terminal uses an HTTP proxy, bypass it for the robot LAN addresses before reading controller feedback:

```bash
export NO_PROXY=10.192.1.2,10.192.1.4,127.0.0.1,localhost
export no_proxy="$NO_PROXY"
```

```bash
../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json probe
```

`probe` saves a synchronized image and named joint JSON under `../output/calibration/sp_vision/wrist_camera_session/` without requiring a checkerboard. It also reads the same controller `arm_q14` used by head validation before and after the image, then reports `mapping_check.passed` only if the arm stayed still and each mapped ROS 2 value agrees within 0.005 rad. A failed or unavailable controller comparison does not mean image capture failed. The image/joint `state_skew_ms` is a separate check with a 100 ms limit. With the bridge backend, the summary must show `source: "limx_studio_bridge"`; interpret its timing as described above. The same configuration selects the transport for `capture`; all fitting commands remain the same.

Rigidly fix a flat 7×10 inner-corner checkerboard with measured 0.021 m squares. Keep it fixed through the image set and touch check. Move the arm only through the robot's existing reviewed interface, then let it settle. Vary position and orientation so the board covers the image and the wrist rotates about at least two distinct axes. Keep the full board visible; do not change the camera's resize setting during calibration.

## Capture, fit and check

```bash
../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  capture --session ../output/calibration/sp_vision/wrist_camera_session --count 40

../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  intrinsics --session ../output/calibration/sp_vision/wrist_camera_session

../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  extrinsics --session ../output/calibration/sp_vision/wrist_camera_session
```

In the capture window, `s` saves an accepted image, `f` reverses checkerboard corner order by 180°, `r` or Space reacquires, and `q` quits. Without `--append`, a complete capture replaces the session's previous `view-*` images; quitting early preserves them. Use `--append` to add views. The intrinsic fit reuses the head workflow's checkerboard detection, distortion fit, quality limits and holdout split. The extrinsic fit reuses its PnP and hand-eye solver, but uses synchronized `T_base_wrist_roll(q)` for every view. It checks the nominal URDF/MJCF camera chain and evaluates held-out views against the board pose fitted from training views. Require `passed: true`; also inspect `training`, `holdout`, rejected views, and deviation from the nominal mount.

### Meaning of the extrinsic summary parameters

The terminal summary for the current extrinsic calibration contains:

```json
"max_training_mm": 7.555524738640365,
"max_holdout_mm": 10.41927581138533,
"mount_rotation_span_deg": 176.67627544117101
```

- `max_training_mm`: the largest checkerboard translation residual across all training views, in millimeters. The training views participate in solving `T_wrist_roll_camera` and `T_base_board`. For every view, the program compares the board position composed from FK, the hand-eye extrinsic, and the PnP result against the fixed `T_base_board`, then takes the largest translation-difference norm. Lower is better and indicates stronger internal consistency in the training set; here it is about **7.56 mm**.
- `max_holdout_mm`: the largest checkerboard translation residual across all holdout views, computed in the same way and expressed in millimeters. Holdout views do not participate in the hand-eye extrinsic fit, so this value checks consistency on images that were not used for fitting; here it is about **10.42 mm**.
- `mount_rotation_span_deg`: the orientation coverage of the wrist mount chain across the training views, in degrees. Specifically, it is the largest relative rotation angle between the `T_base_wrist_roll` rotation of the first training view and that of every other training view. It measures pose excitation, not calibration error or the travel of any single wrist joint; here it is about **176.68°**.

The first two values are both below the current **15 mm** threshold corresponding to `max_board_residual_m: 0.015`, and the rotation span is above the minimum `min_mount_rotation_span_deg: 20`. These three numbers alone do not establish a passing calibration. Overall `passed` also requires every view's rotation residual to be no more than **2°**, at least one valid holdout view, successful optimization, and sampling that is not nearly limited to rotation about a single axis.

## Calibrate the right-arm touch TCP

To recalibrate the TCP, keep one rigid tip against one fixed point and read measured arm state at four clearly different wrist orientations. A dexterous fingertip is usable only if every finger joint remains at the same pose; the state files do not record finger joints. Use the robot integration's read-only state command after each pose settles. The command is intentionally not part of the standalone package; only its resulting JSON files are consumed below. The examples use this machine's real profile, `../configs/local-robot-seed.json`; replace it with your actual profile on another machine, rather than using a template with a placeholder host.

Before collecting the four TCP poses, use the following read-only commands to obtain one sample of the current live head `q2`:

```bash
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output ../output/calibration/sp_vision/wrist_camera_session/tcp/current-state.json
../.venv/bin/python -c 'import json, pathlib; p = pathlib.Path("../output/calibration/sp_vision/wrist_camera_session/tcp/current-state.json"); print(json.loads(p.read_text())["head_q2"])'
```

The `head_q2` order is `[head_pitch_Joint, head_yaw_Joint]`, in radians. Rerun the first command to read a fresh state. It saves the complete `arm_q14` and `head_q2` state and does not send any motion command.

```bash
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output ../output/calibration/sp_vision/wrist_camera_session/tcp/pose-01.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output ../output/calibration/sp_vision/wrist_camera_session/tcp/pose-02.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output ../output/calibration/sp_vision/wrist_camera_session/tcp/pose-03.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output ../output/calibration/sp_vision/wrist_camera_session/tcp/pose-04.json
```

The wrist command reuses the head workflow's fixed-point solver and returns the tip position in `wrist_roll_R_Link`:

```bash
../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  pivot --states ../output/calibration/sp_vision/wrist_camera_session/tcp/pose-{01,02,03,04}.json \
  --output ../output/calibration/sp_vision/wrist_camera_session/tcp/wrist_tcp_pivot.json
```

Require `passed: true` and inspect `max_residual_m`, wrist orientation spread, and `condition_number`. Four-pose fitting determines tip **position**, not tool orientation. If the physical tip and mounting have remained unchanged since head validation, you may skip new pivot samples and use `../output/calibration/sp_vision/wrist_camera_session/tcp-pivot-from-head.json` instead. Its internal fit passed, but the earlier head_camera independent three-point check still had 13–14 mm errors, so the full measurement chain remains unverified.

If you already refitted the pivot through the head workflow and saved `../output/calibration/sp_vision/head_camera_session/tcp/head_tcp_pivot.json`, there is no need to recapture the four poses. Copy that new result to the wrist `wrist_tcp_pivot.json` path above, or use it directly as `--tcp` in the validation command below. Do not accidentally use the older `tcp-pivot-from-head.json`.

## Independent touch validation

After fitting extrinsics, keep the checkerboard fixed in the base frame and capture a **new** wrist image that was not used for fitting. The right arm may move after this image; predictions use the joint state synchronized to that image. Select three spread-out, non-collinear physical inner corners that the tip can touch:

```bash
../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  capture --session ../output/calibration/sp_vision/wrist_camera_validation --count 1

../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  select-validation --frame ../output/calibration/sp_vision/wrist_camera_validation/view-001 \
  --output ../output/calibration/sp_vision/wrist_camera_validation/wrist_selection.json
```

Each completed `capture --count 1` replaces the previous validation image with a new `view-001`; quitting with `q` preserves the old image. After recapturing, rerun `select-validation` so `wrist_selection.json` and `wrist_selection.png` match the new image. Previous touch states are usable only if the board and physical corners stayed fixed.

Inspect the numbers in `../output/calibration/sp_vision/wrist_camera_validation/wrist_selection.png` against the physical corners. Keep the board fixed and use the same tip and finger pose as in TCP fitting. Touch corners 1, 2, and 3 in order and read state after each pose settles:

```bash
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output ../output/calibration/sp_vision/wrist_camera_validation/state-01.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output ../output/calibration/sp_vision/wrist_camera_validation/state-02.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output ../output/calibration/sp_vision/wrist_camera_validation/state-03.json

../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  validate --selection ../output/calibration/sp_vision/wrist_camera_validation/wrist_selection.json \
  --side right --tcp ../output/calibration/sp_vision/wrist_camera_session/tcp/wrist_tcp_pivot.json \
  --states ../output/calibration/sp_vision/wrist_camera_validation/state-{01,02,03}.json \
  --output ../output/calibration/sp_vision/wrist_camera_validation/wrist_validation.json
```

If reusing the earlier TCP, change only `--tcp` to `../output/calibration/sp_vision/wrist_camera_session/tcp-pivot-from-head.json`. Validation compares camera-predicted corners with touched tip positions in `base_Link`; this report's `passed` is the independent check. Neither TCP fit residuals nor held-out extrinsic consistency replace it.

The `translation_m` values in the extrinsic JSON's `training` and `holdout` lists are in meters. The terminal summary converts their maxima to `max_training_mm` and `max_holdout_mm`, as the head command does. If touch errors are close to checkerboard corner spacing, check that numbers 1, 2, and 3 in `wrist_selection.png` match the physical touch order. Only after confirming the physical corners, rerun `select-validation --corner row,column` in that order and validate again. Inferring corners solely from the touch states does not make an independent validation pass.
