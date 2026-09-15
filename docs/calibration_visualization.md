# Guided calibration capture

[简体中文](calibration_visualization.zh-CN.md) · [Calibration workflow](calibration.md) · [README](../README.md)

Use the visual helper **during calibration**: see the board, capture usable samples, and follow the next action. Launch it from the CLI; the browser offers preview, capture and solve. The result stays compact: saved file, next action, and intrinsic RMS or hand-eye sample count. Independent measured points remain the acceptance check.

## 1. Start the intrinsic helper

Prepare the camera profile and measured board dimensions as described in [calibration](calibration.md). From the repository root, in the configured camera environment:

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-seed.json --stage intrinsics \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/intrinsics-guided
```

Open the printed address, normally `http://127.0.0.1:8790`. Each session directory must be new or empty. `--pattern` counts inner corners; `--square-m` is the measured square edge in metres. Use `--port` to choose another port.

The helper does not open a browser or access hardware at startup. **Preview** explicitly reads a camera frame and overlays detected board corners. **Capture and save a new sample** acquires a fresh frame, checks it and saves it only when accepted; it does not reuse the preview. The page shows the accepted count and the next action.

## 2. Capture varied views and solve

1. Preview the board. Keep the whole board visible and check that the marked corners follow its intersections.
2. Capture one usable view. Move the board to another image region; vary distance and tilt while keeping the head still.
3. Repeat for at least five accepted views, then select **Solve**. Five is a minimum, not proof of good coverage.

The result is saved as `calibration_data/intrinsics-guided/intrinsics.json`, with the original images under `view-*/color.png`. Review the compact fit error and next-step message. Stop the helper with Ctrl-C, then apply this file:

```bash
.venv/bin/tron2-deploy apply-intrinsics \
  --profile configs/local-robot-seed.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --output configs/local-robot-intrinsics.json
```

A small reprojection RMS means the fit explains these detected image points; it does not prove camera-to-base accuracy or correct depth alignment. Applying intrinsics invalidates earlier extrinsic acceptance and keeps real execution disabled.

## 3. Restart for hand-eye capture

Rigidly mount the board on the selected wrist. Restart with the updated intrinsic profile:

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-intrinsics.json --stage handeye --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left-guided
```

Open the printed URL. Use the robot's separately reviewed interface to position the arm, wait until it stops, then preview and capture. Repeat with varied wrist rotations and positions while keeping the board mounting and head pose fixed. The helper reads images and feedback; it never moves the robot.

Collect at least five accepted samples and at least 15 degrees of wrist rotation spread before solving. A rejected capture leaves the accepted count unchanged. Samples are saved to `samples/*.json` and the solution to `handeye-left.json` within the session directory. Use a separate directory and `--side right` for a right-wrist set; do not combine sides.

The solution remains **unverified**. Stop the helper with Ctrl-C, measure independent held-out points, then follow [validation and application](calibration.md#validate-held-out-points-and-apply-the-result). That step rechecks the point errors before writing the calibrated profile. The planning operator uses this profile later; it does not need to run during calibration.

## When the helper cannot continue

| Message or observation | Next action |
| --- | --- |
| Board not found or incomplete corners | Check the inner-corner count, lighting and focus. Keep the entire board in view, then preview again. |
| Old frame or inconsistent timestamps | Check the camera stream and clock synchronization; capture a fresh frame. |
| Arm or head moving | Wait for settling. Restore the configured fixed head pose before recapturing. |
| Too few accepted samples | Capture more views; rejected captures do not count. |
| A view or wrist pose is too similar to a saved sample | Move to a clearly different view or wrist pose before capturing again. |
| Insufficient wrist rotation spread | Reposition the wrist with varied rotations, keep the board visible, and capture again. Repeating the same pose does not help. |
| A fit looks wrong despite a small error | Check board dimensions, image coverage and rigid mounting; use the optional diagnostics below if needed. |

## Try the page without hardware

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/demo.json --stage intrinsics --mock \
  --pattern 9x6 --square-m 0.025 \
  --output output/calibration-guide-demo
```

Mock mode labels its synthetic data. It demonstrates preview, capture and solve without a camera or robot and cannot establish real calibration acceptance. Use a new directory for another session.

## Optional diagnostics

Use `calibration-report` when a fit or independent point check needs closer inspection. The offline report opens with a short summary; detailed plots are collapsed and can be expanded as needed. Generating a report is not required to complete the guided capture flow.

```bash
.venv/bin/tron2-deploy calibration-report \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --output output/intrinsics-review

xdg-open output/intrinsics-review/index.html
```

For hand-eye troubleshooting, include the guided session's sample directory and the independently measured points prepared in [calibration](calibration.md):

```bash
.venv/bin/tron2-deploy calibration-report \
  --handeye calibration_data/handeye-left-guided/handeye-left.json \
  --samples 'calibration_data/handeye-left-guided/samples/*.json' \
  --validation-points calibration_data/heldout_points.npz \
  --max-error-m 0.005 \
  --output output/handeye-review

xdg-open output/handeye-review/index.html
```

The example uses a maximum point error of 5 mm; choose the actual threshold from the deployment's measurement and clearance budget. Failure still writes the report and exits with status `1`, so open it even after a failed check. Without independent points, the result remains unverified. Fit consistency alone does not establish acceptance.

Reports use saved files and need no camera, ROS, GPU service or running frontend. Keep `index.html`, `metrics.json` and exported plots with their inputs; every report directory must be new or empty. Neither the helper nor a report changes a profile or enables execution.
