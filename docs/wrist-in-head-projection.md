# Project the wrist camera position and axes into head RGB

[简体中文](wrist-in-head-projection.zh-CN.md) · [Calibration results](camera-calibration-results-20261110.md)

## Current local calibration

The default pinned manifest is [configs/local-camera-calibration-latest.json](../configs/local-camera-calibration-latest.json), currently bound to `head-20261110-220522__wrist-20261111-180956`. The head retains the existing 22:05 pair; the wrist uses the pair from `sp_vision/data/wrist_camera_session_20261111-180956/`. All four files are byte-identical snapshots under `output/calibration/bundles/head-20261110-220522__wrist-20261111-180956/`. File hashes, intrinsic/extrinsic pairing and the relevant FK chains pass their checks. This switch neither refits nor changes numerical parameters; physical accuracy remains unverified.

The wrist intrinsic RMS is 0.2819 px. Extrinsic fitting automatically excluded training views `view-001`, `view-007` and `view-014`, retaining 31 training and 6 holdout views. Maximum training residuals are 14.51 mm / 1.579°, and holdout residuals are 13.00 mm / 1.382°, within the 15 mm / 2° limits. The previous binding is backed up as `previous-default-manifest.json` in this bundle; `provenance.json` records the sources.

`--calibration-summary` is optional and defaults to that local file. Startup prints the actual calibration ID and manifest path; each capture records `calibration_id`. Missing bindings fail explicitly rather than falling back to older data. An explicit `--calibration-summary PATH` can select a historical version. Close an already running overlay with `q` and restart to load new parameters.

The earlier `source hash changed` error occurred because the head working files had been replaced while the old manifest still recorded their previous hashes. The current manifest points to separate snapshots and retains checksum checks. The external and tracking URDFs have identical base-to-head-parent FK; their different nominal fixed camera mounts are replaced by the supplied calibrated transform. The relevant FK chain is pinned by signature; a change to that chain is rejected.

The historical binding `head-20261110-220522__wrist-20261109-223333` measured **25.708 mm / 1.746°** when replaying the reviewed pair `board-20261110-201025`; translation exceeded 15 mm. A separate read-only live check of that historical binding had 21 ms image/state skew and an off-screen predicted center. These records are not validation results for the current 18:09 wrist version, which still needs a new independent physical comparison. See the [historical result and file paths](camera-calibration-results-20261110.md).

## Your proposed experiment

Use the measured right arm joints and calibrated wrist camera mount to compute the wrist camera pose in `base_Link`. Transform its optical center and three positive axis endpoints into the head camera frame, then project them onto head RGB. As joint feedback changes, update the predicted center and orientation axes. This script runs locally and subscribes only to head RGB and measured joints; following means updating the overlay. It does not command the arm or head.

The red center is a geometric prediction, not a lens feature detected and followed in the RGB image. A stable offset at each stopped pose that changes between poses is a pose-dependent static residual. Perspective can magnify the same geometric bias differently with distance, and the visible lens surface may differ from the internal optical origin. The historical 19:02 calibration improved this comparison but did not eliminate the residual. Use the paired-board check below for a reference with known geometry.

The implementation is [tools/hardware/wrist_in_head_overlay.py](../tools/hardware/wrist_in_head_overlay.py). It reuses `sp_vision`'s URDF FK and named ROS-to-URDF joint mapping.

```text
B  = base_Link
W  = wrist_roll_R_Link
P  = head_pitch_Link
Cw = right_wrist_camera_color_optical_frame
Ch = head_camera_color_optical_frame

T_B_Cw = FK_B_W(q_arm)  @ T_wrist_roll_camera
T_B_Ch = FK_B_P(q_head) @ T_pitch_camera

p_B  = T_B_Cw @ [0, 0, 0, 1]
p_Ch = inverse(T_B_Ch) @ p_B
(u, v) = project(p_Ch.xyz, K_head, D_head)

T_Ch_Cw = inverse(T_B_Ch) @ T_B_Cw
L = 0.08
P_Cw = [[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]]
P_Ch = transform(T_Ch_Cw, P_Cw)
(u_i, v_i) = project(P_Ch[i], K_head, D_head)
```

Head FK uses the measured head joints associated with the image, so the overlay can also update when the head pose changes. Projection uses distortion on raw RGB. The script rejects image dimensions that differ from calibration and never silently resizes images or substitutes nominal camera mounts.

## What this checks

This is a visual consistency check of arm/head FK, wrist mount translation and rotation, head extrinsics and head intrinsics together. Compare the predicted center and axis directions with the visible wrist camera across different positions and orientations.

The wrist camera origin is `[0, 0, 0]`, so its base position is `R_B_W @ t_W_Cw + t_B_W`; mount rotation does not enter the center-only calculation. The nonzero X/Y/Z endpoints use the full wrist mount rotation, so they provide an orientation overlay. Wrist intrinsics remain unused. The visible lens or housing center can differ from the optical center. Predicted orientation is recorded in the head camera frame, not as a measured angular error. Quantitative rotation validation requires an independently observed reference with known orientation, such as a rigidly mounted, geometrically known fiducial target.

## Axis convention and orientation inspection

The axes belong to `right_wrist_camera_color_optical_frame`. Their directions are defined relative to the wrist camera's own image/view, following the [ROS optical-frame convention](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst#suffix-frames):

| Axis | Overlay color | Positive direction |
|---|---|---|
| X | Red | Right in the wrist camera image |
| Y | Green | Down in the wrist camera image |
| Z | Blue | Forward along the RGB camera's viewing direction |

Default length is 0.08 m (8 cm). Set `--axis-length-m 0.05` before the mode for 5 cm axes. These directions are expressed through the complete relative camera transform, so their lines in head RGB need not be horizontal, vertical or mutually perpendicular. An axis pointing along the head viewing ray may project to a very short line or a dot.

During teaching, first check the optical-center position. Then slowly vary the wrist orientation through the existing teaching controls and save stationary observations. Check whether the blue Z direction follows the RGB lens viewing direction and whether all three axes rotate consistently with the wrist camera. The overlay alone does not supply an independent angular ground truth.

## Local commands

Run these commands on DC, from `/home/dc/tron2_deployment_on_dc`. The calibration summary references both cameras' result JSON files and SHA-256 hashes. The script verifies those files and the intrinsic/extrinsic pair hash recorded in the result or binding manifest. For explicitly recorded FK chain signatures it checks the relevant base-to-parent joints; otherwise it retains the URDF hash check in each available camera input manifest. Local result/profile files are not stored in Git.

First save a mock example without a connection or GUI window:

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --output output/wrist-in-head/mock-preview \
  mock
```

Start the live RGB overlay in a desktop terminal:

```bash
cd /home/dc/tron2_deployment_on_dc
WRIST_PROJECTION_OUTPUT="output/wrist-in-head/teach-$(date +%Y%m%d-%H%M%S)"
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --axis-length-m 0.08 \
  --output "$WRIST_PROJECTION_OUTPUT" \
  live --profile configs/local-robot-seed.json
```

The profile supplies Bridge host/TLS/token settings for `/camera/top/color/image_raw/compressed` and `/joint_states`. RGB requests explicitly include `max_fps=30`; actual delivery may be slower. The default ROS domain is 0; use `live --domain-id N` for another domain. This mode requires a reachable camera Bridge and a graphical desktop. Press `q` or Esc to exit and disconnect; press `s` to save a matched frame. Press Space to freeze/resume; click a visible reference feature only while frozen. The first matched frame is saved automatically. Diagnostics appear below the original RGB area without changing its pixel coordinates.

For one live capture without a GUI:

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --output output/wrist-in-head/live-snapshot \
  live --profile configs/local-robot-seed.json --headless --frames 1
```

Replay that saved capture without a connection:

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --output output/wrist-in-head/replay-preview \
  replay --capture output/wrist-in-head/live-snapshot/view-001
```

Choose a new output directory for each run; a nonempty directory is rejected. Use `--wrist-config PATH` before the mode to select another calibration mapping/model configuration; the default is `sp_vision/configs/wrist_config.example.json`.

## Teaching-mode verification steps

1. Enter the existing controller's teaching/drag mode using the established onsite procedure. Confirm that payload compensation matches the installed attachments and retain the support required by that mode; see the [wrist calibration preparation](../sp_vision/wrist_calib.md). Keep the head fixed for the first round. This script supplies no mode changes or motor commands.
2. Close any older version's overlay with `q`, then run the live command above in a DC desktop terminal; an already running process does not reload script changes. Its timestamped `teach-...` output directory avoids overwriting earlier runs. Leave the terminal open; a separate RGB window appears with a red center cross and X/Y/Z axes.
3. Confirm that the RGB image updates and shows the head camera's view. Read the status below the image: `projected` means the predicted point is inside the image; `outside_image` means it is outside; `behind_camera` means its predicted head-camera Z is nonpositive. `STALE / UNSYNCHRONIZED` hides the prediction while data are unusable. Do not treat an unexpectedly stalled live image as fresh. Explicit `FROZEN` mode preserves one matched frame for annotation.
4. While following the onsite teaching procedure, slowly bring the right wrist camera into a clearly visible, unobstructed part of the head image. Keep the head fixed. Locate the wrist camera's RGB lens as an approximate visual reference; the camera housing or hand center is a different point.
5. Stop at the first usable pose and maintain the support required in teaching mode. Compare the red cross with the visible lens region and inspect the optical-axis directions, particularly blue Z along the lens viewing direction. If the real camera is clearly visible but the status remains `outside_image` or `behind_camera`, save that case too; it is useful evidence of disagreement. Do not alter calibration values merely to force visual agreement.
6. Click the RGB window to give it keyboard focus. With the arm stationary, press Space, click the same identifiable RGB lens feature in the frozen image, and press lowercase `s`. The terminal prints the saved directory. Press Space again to resume before the next pose. The first matched frame was saved automatically, so the first manual save normally becomes `view-002`. Record approximately 6 stationary poses covering left/center/right image regions at two practical distances within the permitted workspace. Save each pose separately and keep the head fixed throughout this first round. Pressing `s` without selecting a reference still saves a frame but does not measure a selected-feature residual.
7. Between stationary captures, use slow teaching movements to observe whether the center and colored axes follow the visible wrist camera. Add stationary captures with different wrist orientations while keeping the camera visible. Judge alignment again after stopping; record motion-only lag separately from a persistent stationary offset. After the first round, an optional second round can vary the head pose through the existing reviewed controls, again saving stationary observations.
8. Support the arm and complete teaching according to the controller's established procedure. Press `q` or Esc in the RGB window to close the overlay and disconnect subscriptions. Closing this window only stops observation; robot mode and holding behavior remain controlled by the robot controller.

Each manual save records `color.png`, `overlay.png` and `projection.json` in `/home/dc/tron2_deployment_on_dc/output/wrist-in-head/teach-<timestamp>/view-NNN/`. Retain all three files together. The optical center is internal to the lens and the axes are predictions, so this gives a visual position/orientation consistency check rather than a measured absolute error in millimeters or degrees.

## Recording an offset that remains after stopping

A stationary offset that varies with pose cannot be explained solely by display lag. Candidate causes include the chosen visible reference differing from the optical center, camera intrinsics/distortion, either camera mount calibration, and the arm/head kinematic model. The overlay alone does not identify which component is responsible; keep calibration values unchanged while gathering comparisons.

Keep the head fixed. First record 3–5 stationary positions with approximately similar camera orientation; then add different wrist orientations while keeping the camera visible. At each pose:

1. Stop moving according to the existing teaching procedure, then press Space in the RGB window. `FROZEN` means the image and its matched joint record have both been copied together. It freezes observation only; it does not stop or change robot control.
2. Click the same clearly identified visible lens feature each time. A green reference cross and yellow line appear. Clicks in the live view or diagnostics panel are ignored.
3. Read `du`, `dv` and the pixel distance, press `s` to save, then press Space to resume before the next pose.

The recorded vector is `observed_pixel - predicted_pixel`, with magnitude `sqrt(du**2 + dv**2)`. It measures separation from your chosen visible feature. A lens surface or housing center may have a fixed 3D offset from the optical origin; its projected separation can vary with pose even if the transforms are correct. This is why reference identity must be consistent and optical-center equivalence remains unverified. No constant translation or pixel correction is applied.

## Paired-board consistency check

`board-check` uses the same fixed physical checkerboard in both RGB images. It estimates board pose independently using each camera's own intrinsics, then uses each image's matched joints and camera mount to compare the two board poses in `base_Link`. It reports translation difference in millimeters, relative rotation in degrees and bidirectional corner projection errors in pixels. Wrist intrinsics and wrist mount rotation participate in this check.

Keep the board fixed relative to the base, with all 70 inner corners visible in both cameras. Through the existing supervised controls, position the wrist camera to see the board, then let both cameras and the arm settle. This mode only reads head RGB, wrist RGB and `/joint_states`, captures one synchronized pair, saves results and exits without opening a GUI.

```bash
cd /home/dc/tron2_deployment_on_dc
BOARD_CHECK_RUN="output/wrist-in-head/board-$(date +%Y%m%d-%H%M%S)"
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --output "$BOARD_CHECK_RUN" \
  board-check --profile configs/local-robot-seed.json
```

The default board is 7×10 inner corners with 0.021 m squares. Use `board-check --pattern COLSxROWS --square-m VALUE` for a different measured validation board. Each RGB is paired with its own image-time joint sample; both image stamps must also be within `--max-skew-ms`. Timing remains based on Bridge forwarding stamps, so use stationary observations.

Review `head-corners.png` and `wrist-corners.png`: their `(0,0)` labels must identify the same physical corner. A plain checkerboard can be detected with reversed 180° ordering. The tool does not silently choose the order that minimizes error. Until confirmed, status is `needs_corner_identity_review` and `relative_pose_within_limits` remains unset. After reviewing the labels, replay the saved pair offline:

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --output "${BOARD_CHECK_RUN}-review" \
  board-check --capture "$BOARD_CHECK_RUN" --confirm-corner-order
```

If a camera's order must be reversed to match the physical corner, add `--flip-head` or `--flip-wrist` to this offline replay, based on the images. The tool checks raw image hashes and timestamp skew before replay. The comparison uses the current calibration manifest, allowing historical pairs to be checked without reconnecting.

| Saved file | Meaning |
|---|---|
| `head-color.png`, `wrist-color.png` | Original paired RGB images |
| `head-corners.png`, `wrist-corners.png` | Detected corner IDs for correspondence review |
| `head-cross-projection.png`, `wrist-cross-projection.png` | The other camera's predicted corners drawn in red, with yellow residual segments |
| `board-check.json` | Own-camera PnP RMS, both base-frame board poses, translation/rotation difference, cross-projection RMS/max, timestamps, feedback and hashes |

If a board is missing or rejected, raw images and detection reasons are saved without reporting a board-pose measurement. Once IDs are explicitly confirmed, relative pose is compared with the existing 15 mm / 2° limits. This tests relative chain consistency; it does not establish absolute URDF accuracy, enable motion or update any calibration. `accuracy_verified` stays `false`.

The initial 2026-11-10 availability probe read both streams successfully: head detected 70 corners, but wrist RGB showed the floor and detected no board. RGB stamp skew was 12 ms; head/wrist state skew was 4 ms / 6 ms, so that probe did not produce a paired-board pose difference. See [head view](../output/wrist-in-head/board-availability-20261110/head-corners.png), [wrist view](../output/wrist-in-head/board-availability-20261110/wrist-corners.png) and [probe record](../output/wrist-in-head/board-availability-20261110/board-check.json). Synthetic checks verify detection of known 20 mm translation and 5° rotation discrepancies, offline replay and stream cleanup.

With the historical 19:02 head parameters, the later capture `board-20261110-201025` detected 70 corners in each camera. Asymmetric red tape matched after independent 2D rectification, confirming the same physical corner IDs without either flip flag. Own-camera PnP RMS was about 0.103 px / 0.162 px, while cross-projection RMS was 9.351 px / 5.926 px. The reviewed relative board difference was 17.308 mm and 2.069°, slightly above the configured 15 mm / 2° limits. Offline review therefore reported `relative_pose_exceeds_limits`; this is a consistency result, not a program crash.

Three additional read-only samples with nearly unchanged recorded joints reproduced the discrepancy using that 19:02 version. Across the original and repeats, reviewed translation ranged from 17.292 to 17.348 mm and rotation from 2.044 to 2.069°. This supports a repeatable relative geometric disagreement, beyond the visible-lens/optical-origin ambiguity. It does not identify a particular faulty camera or FK component, and it is not independent absolute accuracy. No parameters were changed during those checks. Further refinement needs diverse paired-board poses and separate held-out validation, rather than a correction derived from one pose.

Records: [corner identity evidence](../output/wrist-in-head/board-20261110-201025/corner-identity-review.png), [reviewed result](../output/wrist-in-head/board-20261110-201025-reviewed/board-check.json), [repeatability summary](../output/wrist-in-head/board-20261110-201025-analysis/summary.json), and [repeatability plot](../output/wrist-in-head/board-20261110-201025-analysis/repeatability.png).

## Second round: keep the right arm fixed and vary the head

The first round varied the arm while holding the head fixed. This complementary round helps distinguish a head-dependent error from an error shared by the right arm/base chain. Stabilize the right arm using the controller's established supervised holding procedure, keeping the same wrist position, orientation and camera mounting throughout this round.

Close the old overlay with `q` and start a separate output directory:

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --axis-length-m 0.08 \
  --output "output/wrist-in-head/head-check-$(date +%Y%m%d-%H%M%S)" \
  live --profile configs/local-robot-seed.json
```

1. Save a baseline observation with the right wrist RGB lens clearly visible.
2. Through the existing reviewed head controls, make small changes within the permitted range. Aim for approximately 5 views: the starting view, slightly left, slightly right, slightly up and slightly down, while keeping the same lens visible.
3. At each settled head pose, press Space, click the same physical RGB lens center, press `s`, then press Space to resume.
4. Keep the right arm fixed while the head changes; the lens's image position is expected to move. If the arm moves, stabilize it and repeat that observation. The saved joint records allow arm-pose consistency to be checked later.
5. Finish using the controller's established procedure and close the overlay with `q`.

The script already updates head FK from the matched measured head joints. This round supplies diagnostic comparisons; calibration files remain unchanged. Separating the two chains still requires checking the new observations, rather than assuming which calibration is wrong.

## Synchronization, visibility and saved records

The script matches named joint feedback to the image's Bridge timestamp. Defaults are at most 30 ms image/state skew and 750 ms local receipt age for both streams. These are Bridge forwarding timestamps, so they do not establish hardware exposure synchronization. Duplicate/out-of-order timestamps do not refresh cached samples. Missing/stale/mismatched feedback hides the projection; after 8 s without a usable pair, the run exits with an error. Set `--max-skew-ms`, `--max-age-ms` or `--timeout` before the mode when needed, and retain their values with the results.

When the center is behind the head camera or outside the image, the center and all axes are hidden. With a visible center, an axis endpoint behind the camera is hidden independently; endpoints outside the image are clipped at the RGB border for drawing. Full endpoint coordinates remain in JSON. Extremely large projections are retained as coordinates without drawing an unsafe line. Axes never extend into the diagnostics panel. Mock and replay outputs are labeled explicitly. `accuracy_verified` remains `false` in every mode.

Each saved `view-NNN/` contains:

| File | Contents |
|---|---|
| `color.png` | Original head RGB, or the mock background |
| `overlay.png` | Predicted center and colored X/Y/Z axes, base XYZ, pixel coordinates, head-camera Z, time skew and axis length |
| `projection.json` | Named joints, timestamps, predicted center and axis endpoints, axis clipping/visibility, camera-to-base and relative camera transforms, rotation/quaternion, calibration/model paths and hashes, and the image hash |

When selected, `manual_reference` stores the observed `pixel_uv`, original `image_stamp_ms`, `observed_minus_predicted_px` and `distance_px`. `display_frozen` marks a user-frozen frame. Live captures also retain `image_receipt_age_ms`, `joint_receipt_age_ms` and `newest_image_gap_ms`; these describe local receipt/cache timing, not hardware exposure age. Replay preserves the selected reference and recomputes its residual against the replayed prediction.

Orientation fields are `T_head_wrist_camera`, `R_head_wrist_camera` and `orientation_head_quaternion_xyzw`; they describe the wrist optical frame relative to the head optical frame. `axis_length_m` records the chosen scale. `axes.X`, `axes.Y` and `axes.Z` contain their 3D endpoints, projected `pixel_uv`, `drawable`, `clipped` and drawn `segment_uv`. Replay recomputes axes using the current `--axis-length-m`, including for older center-only captures.

Live runs also write `run.json` with processed/saved counts, `reference_captures`, timing limits, elapsed time and any runtime error. Replay verifies that `color.png` matches its recorded hash. Output belongs under the Git-ignored `output/` directory. These records support review of this position overlay; they do not establish physical accuracy or authorize robot motion.

## Local verification on 2026-11-10

The current full test suite passed 273 tests, including simulated GUI freeze/click/save and replayed-reference checks; the packaged mock demo completed with `hardware_commanded: false`. An earlier read-only check of the axes version saved 3 live frames, with Bridge image/state skew of 0–13 ms, and then disconnected. In the last frame, the center was `(353.203, 292.827)` with head-camera Z of approximately 0.5327 m. The center and all three 0.08 m axis endpoints were in the 640×480 image and were drawn. Two subsequent manual-reference rounds are analyzed below. Independent physical position/rotation accuracy remains unverified.

Records: [live axes overlay](../output/wrist-in-head/axes-live-20261110/view-003/overlay.png), [live projection JSON](../output/wrist-in-head/axes-live-20261110/view-003/projection.json), [live run record](../output/wrist-in-head/axes-live-20261110/run.json), and [mock axes overlay](../output/wrist-in-head/axes-mock-20261110/view-001/overlay.png).

## Two-round offset diagnosis on 2026-11-10

The arm-variation session `offset-20261110-161533` contains 6 annotations of 5 unique images. `view-006` and `view-007` annotate the same frame; the last annotation is used. The head stayed fixed, while wrist orientation changed by up to 21.2° relative to the first pose. The original RMS distance to the selected RGB lens feature was 43.38 px.

The head-variation session `head-check-20261110-163647` contains 5 annotated images, including a return to the baseline head pose. The maximum recorded right-arm joint change was about 0.00020 rad. According to the current FK model, the wrist-camera pose changed by at most 0.098 mm and 0.016°; these describe consistency of feedback/model calculations, not independently measured physical precision. Head rotation changed by up to 7.14° relative to the first annotated pose.

All diagnostic parameters were estimated from the first round and kept fixed when predicting the second round. No model was refitted to the second-round clicks.

| Model evaluated on the second round | RMS distance to the selected visible feature |
|---|---:|
| Original overlay | 34.98 px |
| Head-camera-coordinate translation hypothesis | 3.66 px |
| Common arm/base translation hypothesis | 9.11 px |
| Fixed visible-feature offset in the wrist camera frame | 19.53 px |

This comparison supports prioritizing the head camera mounting transform and rigid head-chain geometry for review. It does not uniquely identify a defective component: intrinsic errors, model errors and the difference between the visible lens feature and optical origin remain possible contributors. Calibration files remain unchanged. Verify with an independent known-geometry target before treating any fitted diagnostic offset as a calibration correction; angular accuracy is also still unverified.

Records: [first-round summary](../output/wrist-in-head/offset-20261110-161533-analysis/summary.json), [second-round summary](../output/wrist-in-head/head-check-20261110-163647-analysis/summary.json), [second-round per-view CSV](../output/wrist-in-head/head-check-20261110-163647-analysis/per_view.csv), and [independent-round model comparison](../output/wrist-in-head/head-check-20261110-163647-analysis/second-round-comparison.png).

## Head intrinsic capture procedure

The earlier `head-recalibration-20261110-190209` session is complete. Its operator confirmed a fixed board while the head moved, so it was used for both intrinsic and extrinsic fitting. The procedure below remains available for a future intrinsic-only dataset; moving-board images must be kept separate from a fixed-board hand-eye dataset.

The original head session's detected inner-corner area was only about 1.7%–2.0% of the image across all 40 views. This narrow size distribution is a reason to improve the capture coverage before deciding on an extrinsic correction, rather than an area-only proof of calibration failure. Keep the head fixed and move the flat checkerboard through different positions, distances and tilts. The board may move for intrinsic calibration; a hand-eye extrinsic dataset requires a board fixed relative to the base and must be treated separately.

A local [intrinsic-capture config](../output/head-recalibration-prep-20261110/head-intrinsics-capture.json) is prepared with the same 7×10 inner corners and 0.021 m square size. Its `min_board_area_ratio` is 0.015, so a sharp, complete 0.017-area view can be included as a distant sample. The initially imposed 0.10 per-image gate was too restrictive and has been removed; 0.10–0.30 is a suggested range for some larger views, not a universal calibration acceptance threshold. Model/profile paths are explicit, and original configs, solver limits and calibration files are unchanged. An old 0.017-area image was checked offline and passes the revised basic detection gate.

For this intrinsic round, the board need not remain on the table. A rigid backing can support it while it is raised or brought closer, provided the corners remain sharp and fully visible. If the board must remain at table height, a larger physical board is another option; measure its actual square size and update the board configuration before capture. Aim for varied image position, size and tilt as described in the [ROS calibration tutorial](https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/doc/tutorial_mono.rst#moving-the-checkerboard). The whole dataset should provide this variation even though individual distant views are allowed.

In a DC desktop terminal:

```bash
cd /home/dc/tron2_deployment_on_dc
HEAD_CALIB_RUN="output/head-recalibration-$(date +%Y%m%d-%H%M%S)"
.venv/bin/python sp_vision/calibration.py \
  --config output/head-recalibration-prep-20261110/head-intrinsics-capture.json \
  capture --pattern 7x10 --square-m 0.021 \
  --session "$HEAD_CALIB_RUN/intrinsics-session" --append --count 40
```

Stabilize the arm and head through the existing controller procedure. Move only the board during this capture. Include larger views where practical, with all 70 corners visible and sharp; aim for `area` around 0.10–0.30 in some views while retaining useful distant views such as 0.017. Include the center and peripheral image regions, several distances and different board tilts. After positioning the board, press `r` to acquire a fresh preview, then `s` to save an accepted view. The capture finishes automatically after 40 saved images; the keys here belong to the calibration capture window, where `r` refreshes the frame.

The config is read at program startup. If the saved count is still zero, quit and restart to apply the revised gate. This command uses `--append` so each accepted sample is saved immediately. `--count 40` means 40 new images in this invocation; when resuming an existing session, use the remaining count. If images have already been staged in an unfinished non-append capture, retain or complete that capture before restarting: quitting early discards the newly staged images.

After collection, fit and inspect the new head intrinsics. The existing head extrinsic images and synchronized head states can then be reused for an independently split extrinsic solve, followed by offline replay of the two diagnostic rounds. Additional extrinsic capture should depend on those results. This preparation does not apply any diagnostic offset or replace the active calibration.

## Historical 19:02 head solve and actual-parameter replay

All 40 saved images were complete, with 40 unique RGB hashes, matching 640×480 dimensions and checkerboard metadata. Head joint spans were about 0.3575 rad and 1.3346 rad. The actual new intrinsic and extrinsic results passed their existing fitting limits without rejected views or outliers, using 34 training and 6 held-out views.

| 19:02 head metric | Result |
|---|---:|
| Intrinsic reprojection RMS | 0.096475 px |
| Maximum training translation / rotation | 9.117 mm / 1.987° |
| Maximum held-out translation / rotation | 4.044 mm / 1.551° |

The prior RGB images, feedback and lens clicks were then replayed using the actual new checkerboard-derived parameters. No diagnostic translation or other correction was fitted to the manual references.

| Manual-reference round | Previous RMS | 19:02 RMS | 19:02 maximum |
|---|---:|---:|---:|
| Arm varies, head fixed | 43.38 px | 17.99 px | 32.64 px |
| Head varies, arm fixed | 34.98 px | 6.38 px | 10.05 px |

These residuals improved but remain nonzero; they measure distance to a visible lens feature, not independently established optical-center or angular accuracy. The historical diagnostic hypotheses above are separate from this actual-parameter comparison. Replay records remain under `output/head-recalibration-20261110-190209/`. Its two head working parameter files were later overwritten by the supplied alternative pair; the historical manifest records the original 19:02 hashes and cannot load those overwritten working files.

At the time of the 19:02 replay, the tool passed 269 tests and the packaged mock demo. A separate read-only live check loaded that calibration and saved 3 frames with Bridge image/state skew of 0–4 ms. At that sampled arm pose the predicted center was below the image, so the live check establishes data loading and transport, while the replay records show the comparison with visible references.

Records: [historical manifest](../output/head-recalibration-20261110-190209/camera-calibration-summary.json), [replay comparison JSON](../output/head-recalibration-20261110-190209/replay-comparison.json), [comparison plot](../output/head-recalibration-20261110-190209/replay-comparison.png), and [historical live loading record](../output/wrist-in-head/latest-head-live-20261110-190209/run.json). Current parameter paths and validation measurements are in the [calibration results](camera-calibration-results-20261110.md).
