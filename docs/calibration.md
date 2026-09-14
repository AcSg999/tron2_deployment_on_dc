# Calibration before deployment

[简体中文](calibration.zh-CN.md) · [README](../README.md) · Next: [deployment](deployment.md)

The first route uses the top RGB-D camera at a fixed, measured head pose. Calibrate the color camera, solve its transform into `base_Link`, and independently validate that transform before preparing pregrasp targets. Record both wrist frames and their TCP mounting transforms. This workflow sends no gripper commands.

## Prepare the profile and measurements

Copy `configs/robot.example.json` to `configs/local-robot-seed.json` as described in [deployment](deployment.md), then replace every null and `REPLACE` value. Set the physical camera identity, image dimensions, initial color intrinsics/distortion, depth intrinsics, depth-to-color transform, camera transport, measured head position, actual robot model/joint mappings, and table height before acquisition. The current live RGB-D route uses 640 × 480 images. Factory camera parameters can seed acquisition; they are not evidence that calibration has passed. Keep `calibration.verified`, `execution.allow_real`, and `execution.hold_behavior_verified` false. The template's identity camera-to-base transform is an unverified acquisition placeholder; never use it to plan motion.

The model must describe the installed wrists and passive attachments. The current adapter maps exactly 14 arm joints and two head joints; other joints must be fixed while retaining their collision geometry. Synchronize robot, camera, and host clocks. The acquisition code rejects stale images and missing or inconsistent head feedback.

Measure the chessboard square size and count **inner corners**. Examples use a `9x6` inner-corner board with `0.025` m squares; change both arguments to match the actual board. During hand-eye sampling, rigidly attach the board to the selected wrist. Its mounting must not change within one sample set. Use the robot's separately reviewed positioning interface to change arm poses, then wait for settling before each capture. Collection itself only reads camera and joint state.

## Fit color intrinsics from original images

Collect views across the image with varied board distances and tilts. Keep the camera resolution and head pose fixed. Run the capture command separately for each view, changing `view-01` to a new directory:

```bash
python -m tron2_deployment.cli capture \
  --profile configs/local-robot-seed.json --raw \
  --output calibration_data/intrinsics/view-01
```

`--raw` saves unrectified color for the fit. Each directory contains `color.png`, aligned `depth.png`, and `frame.json`. The solver requires at least five detected board views at one resolution; inspect coverage and the returned `rms_px` rather than treating the minimum count as acceptance.

```bash
python -m tron2_deployment.cli intrinsics \
  --images 'calibration_data/intrinsics/view-*/color.png' \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/intrinsics.json

python -m tron2_deployment.cli apply-intrinsics \
  --profile configs/local-robot-seed.json \
  --intrinsics calibration_data/intrinsics.json \
  --output configs/local-robot-intrinsics.json
```

Applying the fit updates `K`, distortion, and image dimensions. It invalidates prior extrinsic acceptance and leaves real execution disabled. Depth intrinsics and depth-to-color alignment remain separate measured inputs; this chessboard fit does not calibrate them.

## Collect stationary hand-eye samples

For the fixed top camera, the supported solve is **eye-to-hand**. Select the side carrying the board explicitly. Repeat this command after each separately reviewed arm repositioning; unique timestamped images and sample JSON files are created automatically:

```bash
python -m tron2_deployment.cli record-sample \
  --profile configs/local-robot-intrinsics.json --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left
```

At least five accepted samples are required, including wrist rotations spanning at least 15 degrees. Use varied rotation directions and translations. The sample collector brackets the image with fresh joint feedback and rejects arm/head motion, excessive timestamp skew, and head-only synchronization fallback. All samples in a solve must use one side, one camera, one board, one source, and the same stationary head pose.

```bash
python -m tron2_deployment.cli handeye \
  --samples 'calibration_data/handeye-left/*.json' \
  --output calibration_data/handeye-left.json
```

For a right-wrist sample set, use `--side right` and separate `handeye-right` paths. Solve the two sets independently; do not combine left and right samples. Their camera-to-base results should agree within the measured error budget. The sample field `robot_gripper_to_base` stores the selected **wrist** transform, despite its inherited name.

## Validate held-out points and apply the result

Measure at least three non-collinear points that were not used in the solve. For each point, record its camera-frame coordinates and independently measured `base_Link` coordinates in metres. Do not derive the expected base coordinates from the transform being tested.

Save these real measurements in `calibration_data/heldout_points.json` under `points_camera` and `points_base`, each an equally sized `N x 3` array. Convert the arrays to the required NPZ format:

```bash
python - <<'PY'
import json
import numpy as np
from pathlib import Path
points = json.loads(Path('calibration_data/heldout_points.json').read_text())
np.savez('calibration_data/heldout_points.npz',
         points_camera=np.asarray(points['points_camera'], dtype=float),
         points_base=np.asarray(points['points_base'], dtype=float))
PY
```

The following example accepts a maximum point error of 5 mm. Choose the actual threshold from the deployment's measurement and clearance budget before running it:

```bash
python -m tron2_deployment.cli apply-calibration \
  --profile configs/local-robot-intrinsics.json \
  --intrinsics calibration_data/intrinsics.json \
  --handeye calibration_data/handeye-left.json \
  --validation-points calibration_data/heldout_points.npz \
  --max-error-m 0.005 --calibration-id top-camera-fixed-head-v1 \
  --output configs/local-robot-calibrated.json
```

This command requires a real eye-to-hand solve with the matching camera identity and passing held-out validation. It records the camera-to-base transform, fixed head pose, calibration ID, and error report, then sets `calibration.verified=true`. It explicitly leaves `execution.allow_real=false`.

## Record wrist geometry and preserve evidence

For each side, record `pregrasp.<side>.wrist_to_tcp_pose7` as `[x,y,z,qw,qx,qy,qz]`: the TCP frame expressed in the configured wrist body frame, with translation in metres and a unit quaternion. Obtain it from the installed mount geometry and independent measurements. TCP `+Z` is the inward approach axis; TCP `+Y` is the roll/up reference used by target selection. An identity transform is valid only when these physical frames coincide.

Measure `scene.table_z_m` in `base_Link`; choose `scene.object_radius_m` to enclose the entire registered object mesh about its pose origin. Retain the raw images, sample files, both solves when available, held-out measurements, error report, mount measurements, and accepted model revision. A passed extrinsic fit does not verify collision geometry, wrist mounting, transport timing, or stop behavior.

Moving the head away from the calibrated pose invalidates this fixed-head route. Restore that measured pose or recalibrate; the service does not extrapolate a camera/head kinematic chain. Continue with [deployment](deployment.md), then [pregrasp planning](pregrasp.md).
