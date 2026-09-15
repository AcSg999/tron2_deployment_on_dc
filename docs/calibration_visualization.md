# Visually review calibration results

[简体中文](calibration_visualization.zh-CN.md) · [Calibration workflow](calibration.md) · [README](../README.md)

Keep acquisition, solving, and application in the CLI. The `calibration-report` command turns saved results into an offline visual report for inspection before applying calibration. It writes `index.html`, `metrics.json`, and PNG plots to a new or empty output directory. Open the HTML directly in a browser; rendering uses Matplotlib's non-interactive Agg backend and needs no display server, ROS, or operator service.

The report reads existing files. It does not connect to the robot, move joints, change a profile, mark calibration as verified, or enable execution. The frontend continues to consume the calibrated profile after the existing CLI application step.

## Review camera intrinsics

After fitting the original, unrectified chessboard images in [the calibration workflow](calibration.md), run from the repository root:

```bash
.venv/bin/tron2-deploy calibration-report \
  --intrinsics calibration_data/intrinsics.json \
  --images 'calibration_data/intrinsics/view-*/color.png' \
  --output output/intrinsics-review

xdg-open output/intrinsics-review/index.html
```

`--images` can be omitted when the image paths recorded in the intrinsic JSON still resolve. Use an explicit quoted glob after moving or restoring image data. Use the same camera resolution, chessboard pattern, and square size as the fit; the report reads the board geometry from the intrinsic result. Provide the original images, not images already undistorted with that calibration.

| Visual | What to inspect | What it establishes |
| --- | --- | --- |
| Detected corners and reprojected points over each image | Corners follow the real intersections; residuals do not show a strong position-dependent pattern | Whether the model explains detected corners in that view |
| Original and undistorted image comparison | Straight board edges and plausible geometry across the image, including its edges | A visible check of the distortion correction |
| Image coverage | Views reach different image regions rather than clustering around the center | Which image areas contributed measurements |
| Per-view reprojection errors in pixels | Outliers and differences between views alongside the solver's aggregate RMS | Which views need closer inspection |

The report estimates each displayed board pose using the supplied intrinsics. Reprojection errors on images used for fitting are **in-sample diagnostics**, not independent accuracy measurements. A small RMS does not demonstrate correct camera-to-base alignment or accurate RGB-D depth. Poor corner detection, repeated similar views, or an incorrect board size can undermine the result even when a plot looks tidy. Reacquire or correct the inputs, refit, and generate a report in a fresh directory when needed.

Review the images before running `apply-intrinsics`; that command remains the explicit profile update described in [calibration](calibration.md).

## Review hand-eye geometry and independent points

Use one side's eye-to-hand result and its sample set. Independent validation uses an NPZ containing matching `points_camera` and `points_base` arrays in metres, with at least three non-collinear points. Their base-frame coordinates must come from independent measurements, not from the transform under test. The [calibration workflow](calibration.md) explains how to prepare this file.

```bash
.venv/bin/tron2-deploy calibration-report \
  --handeye calibration_data/handeye-left.json \
  --samples 'calibration_data/handeye-left/*.json' \
  --validation-points calibration_data/heldout_points.npz \
  --max-error-m 0.005 \
  --output output/handeye-review

xdg-open output/handeye-review/index.html
```

`0.005` means a maximum point error of 5 mm. Choose a threshold from the measured deployment error budget before evaluating the result. This example is not a universal acceptance threshold. The comparison uses the largest point error, so a small average cannot hide a failed point.

| Visual | What to inspect | Interpretation |
| --- | --- | --- |
| Sampled wrist poses and pose spread | Translation coverage and varied rotation directions | Repeated near-identical poses provide little geometric diversity |
| Target-in-wrist consistency | Translation residuals in mm and rotation residuals in degrees across samples | A rigidly mounted board should remain in a consistent wrist-relative pose |
| Predicted and reference points in the base frame, with coordinate frames and residual vectors | Whether transformed camera points align with the independently measured references | The direction and location of alignment errors |
| Per-point error bars and the threshold | Every point within the chosen bound; investigate outliers | Independent validation passes only when the maximum error is within the threshold |

For each sample, the report reconstructs the board pose in the wrist frame:

```text
T_wrist_board = inverse(T_base_wrist) @ T_base_camera @ T_camera_board
```

The target-in-wrist residuals assess **consistency of the supplied sample set**. Those samples were used in the hand-eye solve, so their small residuals are not independent confirmation of the camera-to-base transform. Held-out point errors compare `T_base_camera @ point_camera` against independently measured `point_base` and supply the separate accuracy check.

The report displays pass/fail for the supplied held-out measurements. If they exceed the threshold, the files are still written so the failure can be inspected, and the CLI exits with status `1`. Open the HTML even after this nonzero exit. Do not chain the report and viewer with `&&` when inspecting failed checks.

`--samples` is optional; omitting it removes the sample-consistency and spread diagnostics. `--validation-points` is also optional, but `--max-error-m` is required when validation points are provided. Without held-out points, independent validation is **unverified**. You can inspect fit diagnostics first and rerun the report with independent measurements later. Neither an unverified report nor a report containing only fit diagnostics establishes calibration acceptance.

Use separate result files and report directories for left- and right-wrist sample sets. Do not mix the samples, since the board-to-wrist mounting differs between sides.

## Keep the report with its inputs

To combine intrinsic and hand-eye review, pass both `--intrinsics` and `--handeye` to one command, together with their `--images`, `--samples`, and validation arguments as needed. The output still uses a single report directory. Keep that directory, including `metrics.json` and PNG plots, alongside the input calibration files and measurements. Use a different output directory for each revision; the command rejects a nonempty directory.

After reviewing the held-out validation, use the existing `apply-calibration` command with those same independent measurements and threshold. It rechecks acceptance before writing the profile and leaves real execution disabled. Continue with [deployment](deployment.md) and RViz trajectory review in [pregrasp planning](pregrasp.md). Calibration plots do not replace trajectory review or verify collision geometry, wrist-to-TCP mounting, or stop behavior.

Any synthetic images or points used to demonstrate report rendering must be identified as synthetic. A successful synthetic report demonstrates the software path only; actual-device acceptance requires the real measurements described above.
