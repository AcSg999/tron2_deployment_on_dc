# Calibration before deployment

[简体中文](calibration.zh-CN.md) · [README](../README.md) · Next: [deployment](deployment.md)

The first route uses the head-mounted **Intel RealSense D435** RGB-D camera at a fixed, measured head pose. Calibrate its color camera, solve its transform into `base_Link`, and independently validate that transform before preparing pregrasp targets. Record both wrist frames and their TCP mounting transforms. This workflow sends no gripper commands.

Launch `calibration-guide` from the CLI, then use its browser page to preview the board, capture samples and solve, following the steps below. The page shows progress and a compact result with the next action. Profile updates and independent validation stay in the existing CLI workflow.

## Prepare the profile and measurements

Copy `configs/robot.example.json` to `configs/local-robot-seed.json` as described in [deployment](deployment.md), then replace every null and `REPLACE` value. Set the physical camera identity, image dimensions, initial color intrinsics/distortion, depth intrinsics, depth-to-color transform, camera transport, measured head position, actual robot model/joint mappings, and table height before acquisition. The current live RGB-D route uses 640 × 480 images. Factory camera parameters can seed acquisition; they are not evidence that calibration has passed. Keep `calibration.verified`, `execution.allow_real`, and `execution.hold_behavior_verified` false. The template's identity camera-to-base transform is an unverified acquisition placeholder; never use it to plan motion.

The model must describe the installed wrists and passive attachments. The current adapter maps exactly 14 arm joints and two head joints; other joints must be fixed while retaining their collision geometry. Synchronize robot, camera, and host clocks. The acquisition code rejects stale images and missing or inconsistent head feedback.

Measure the chessboard square size and count **inner corners**. Examples use a `9x6` inner-corner board with `0.025` m squares; change both arguments to match the actual board. During hand-eye sampling, rigidly attach the board to the selected wrist. Its mounting must not change within one sample set. Use the robot's separately reviewed positioning interface to change arm poses, then wait for settling before each capture. Collection itself only reads camera and joint state.

## Fit color intrinsics with visual guidance

Start the helper from the repository root in the configured camera environment. Use a new or empty session directory:

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-seed.json --stage intrinsics \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/intrinsics-guided
```

Open the printed URL, normally `http://127.0.0.1:8790`. Click **Preview** to check board visibility and corner detection, then **Capture and save a new sample** for a fresh accepted view. Move the board across the image and vary its distance and tilt; keep the camera resolution and head pose fixed. The helper saves original, unrectified images under `view-*/color.png`. Capturing reads the camera again; it does not save a possibly old preview.

Collect at least five accepted views, then click **Solve**. Check the displayed RMS, result path and next instruction. The minimum sample count and a small fit error alone do not establish accuracy; avoid repeated views clustered at the image center. If the board is not detected, show the whole board, reduce blur or glare, and check the inner-corner count. If a view is too similar, change the board position or tilt before capturing again.

After solving, stop the helper with Ctrl-C and apply the saved intrinsic result:

```bash
.venv/bin/tron2-deploy apply-intrinsics \
  --profile configs/local-robot-seed.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --output configs/local-robot-intrinsics.json
```

Applying the fit updates `K`, distortion, and image dimensions. It invalidates prior extrinsic acceptance and leaves real execution disabled. Depth intrinsics and depth-to-color alignment remain separate measured inputs; this chessboard fit does not calibrate them.

## Controller payloads for drag teaching

Payload identification configures the robot controller's gravity compensation for the currently installed end effectors. It is **not an input** to camera intrinsics, hand-eye transforms, FK, or touch-point calculations in this repository. Do not copy `[m, mc_x, mc_y, mc_z]` into `sp_vision` or replace URDF/MJCF inertial entries with it: the three `mc` values are first mass moments in kg·m, and these four numbers do not specify a full rigid-body inertia.

The payload values previously printed here and embedded in a controller-write script described an earlier installation. They are no longer a valid current command, so this guide does not retain them. An identification-page screenshot shows candidate values but does not establish robot identity or confirm controller readback.

If the installed end effector or its mounting changed and you will use drag teaching, identify the current payload on the target robot, confirm its device ID, apply the result through the reviewed robot-management interface, and read it back **before** entering drag mode. Keep the identification and readback in a local, Git-ignored session record. If the physical camera, tool tip, or their mount changed, repeat the relevant geometric calibration and independent touch validation. Changing controller payload compensation alone does not alter the nominal coordinate transforms, but it can change settling or physical deflection; capture measured joint state after the arm settles and repeat independent validation when the setup changes. This repository's calibration commands do not write controller payload parameters.

## Collect stationary hand-eye samples with visual guidance

For the fixed top camera, the supported solve is **eye-to-hand**: the camera and head stay fixed while a board rigidly attached to the selected wrist moves with that wrist. The result maps camera coordinates into `base_Link`. Restart the helper with the updated intrinsic profile and an explicit side:

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-intrinsics.json --stage handeye --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left-guided
```

Open the printed URL, normally `http://127.0.0.1:8790`. Use this collection loop:

1. Click **Preview** and check that the whole board and its detected corners are visible.
2. Reposition the wrist through the robot's separately reviewed controls. Vary rotation axes and position while keeping the board visible and the head fixed.
3. Wait for the arm and board to settle, then click **Capture and save a new sample**. Check the saved count and displayed wrist rotation change before repeating.

The helper guides sample collection; wrist positioning remains manual, and it does not generate the next wrist pose. **Solve** becomes available after at least five accepted samples and at least one wrist orientation differing from the first sample by 15 degrees or more. This is the minimum solver requirement, not an accuracy check or a requirement to rotate 15 degrees at every step. Use rotations about different axes and keep the board's mounting unchanged.

The collector brackets the image with fresh joint feedback and rejects arm/head motion, excessive timestamp skew, and head-only synchronization fallback. All samples must use the same side, camera, board, source and stationary head pose. Failed captures do not increase the accepted count; correct the displayed issue and capture again.

Click **Solve** when ready. Samples are saved under `calibration_data/handeye-left-guided/samples/*.json`; the separate result is `calibration_data/handeye-left-guided/handeye-left.json`. The result is still unverified until the independent check below. Stop the helper with Ctrl-C.

For a right-wrist sample set, use `--side right` and a separate `handeye-right-guided` directory. Solve the two sets independently; do not mix them. Their camera-to-base results should agree within the measured error budget. The sample field `robot_gripper_to_base` stores the selected **wrist** transform, despite its inherited name.

## Validate held-out points and apply the result

Measure at least three non-collinear points that were not used in the solve. For each point, record its camera-frame coordinates and independently measured `base_Link` coordinates in metres. Do not derive the expected base coordinates from the transform being tested.

Save these real measurements in `calibration_data/heldout_points.json` under `points_camera` and `points_base`, each an equally sized `N x 3` array. Convert the arrays to the required NPZ format:

```bash
.venv/bin/python - <<'PY'
import json
import numpy as np
from pathlib import Path
points = json.loads(Path('calibration_data/heldout_points.json').read_text())
np.savez('calibration_data/heldout_points.npz',
         points_camera=np.asarray(points['points_camera'], dtype=float),
         points_base=np.asarray(points['points_base'], dtype=float))
PY
```

The following example accepts a maximum point error of 5 mm. Choose the actual threshold from the deployment's measurement and clearance budget before running it. `apply-calibration` checks the independent points and writes the accepted profile only when they pass:

```bash
.venv/bin/tron2-deploy apply-calibration \
  --profile configs/local-robot-intrinsics.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --handeye calibration_data/handeye-left-guided/handeye-left.json \
  --validation-points calibration_data/heldout_points.npz \
  --max-error-m 0.005 --calibration-id top-camera-fixed-head-v1 \
  --output configs/local-robot-calibrated.json
```

This command requires a real eye-to-hand solve with the matching camera identity and passing held-out validation. It records the camera-to-base transform, fixed head pose, calibration ID, and error report, then sets `calibration.verified=true`. It explicitly leaves `execution.allow_real=false`.

<details>
<summary>Alternative: capture and solve entirely from the CLI</summary>

The original commands remain available for scripted collection. Use separate directories from the guided sessions. For intrinsics, repeat capture with a new `view-XX` directory for each view, then solve:

```bash
.venv/bin/tron2-deploy capture \
  --profile configs/local-robot-seed.json --raw \
  --output calibration_data/intrinsics-cli/view-01

.venv/bin/tron2-deploy intrinsics \
  --images 'calibration_data/intrinsics-cli/view-*/color.png' \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/intrinsics-cli/intrinsics.json
```

Apply that intrinsic file with `apply-intrinsics` before hand-eye acquisition. Then repeat `record-sample` after each arm repositioning and solve:

```bash
.venv/bin/tron2-deploy record-sample \
  --profile configs/local-robot-intrinsics.json --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left-cli/samples

.venv/bin/tron2-deploy handeye \
  --samples 'calibration_data/handeye-left-cli/samples/*.json' \
  --output calibration_data/handeye-left-cli/handeye-left.json
```

Use these CLI result paths in the validation and application step if you chose this alternative.

</details>

## Record wrist geometry and preserve evidence

For each side, record `pregrasp.<side>.wrist_to_tcp_pose7` as `[x,y,z,qw,qx,qy,qz]`: the TCP frame expressed in the configured wrist body frame, with translation in metres and a unit quaternion. Obtain it from the installed mount geometry and independent measurements. TCP `+Z` is the inward approach axis; TCP `+Y` is the roll/up reference used by target selection. An identity transform is valid only when these physical frames coincide.

Measure `scene.table_z_m` in `base_Link`; choose `scene.object_radius_m` to enclose the entire registered object mesh about its pose origin. Retain the session directories, raw images, sample files, both solves when available, held-out measurements, mount measurements, and accepted model revision. For an unexpected fit or failed point check, use `.venv/bin/tron2-deploy calibration-report --help` to see the optional diagnostic report inputs; retain generated reports with their inputs. The helper and reports do not apply calibration or enable execution. A passed extrinsic fit does not verify collision geometry, wrist mounting, transport timing, or stop behavior.

Moving the head away from the calibrated pose invalidates this fixed-head route. Restore that measured pose or recalibrate; the service does not extrapolate a camera/head kinematic chain. Continue with [deployment](deployment.md), then [pregrasp planning](pregrasp.md).
