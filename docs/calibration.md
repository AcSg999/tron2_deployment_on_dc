# Calibration before deployment

[简体中文](calibration.zh-CN.md) · [README](../README.md) · Next: [deployment](deployment.md)

The first route uses the head-mounted **Intel RealSense D435** RGB-D camera at a fixed, measured head pose. Calibrate its color camera, solve its transform into `base_Link`, and independently validate that transform before preparing pregrasp targets. Record both wrist frames and their TCP mounting transforms. This workflow sends no gripper commands.

Launch `calibration-guide` from the CLI, then use its browser page to preview the board, capture samples and solve, following the steps below. The page shows progress and a compact result with the next action. Profile updates and independent validation stay in the existing CLI workflow.

## Prepare the profile and measurements

Copy `configs/robot.example.json` to `configs/local-robot-seed.json` as described in [deployment](deployment.md), then replace every null and `REPLACE` value. Set the physical camera identity, image dimensions, initial color intrinsics/distortion, depth intrinsics, depth-to-color transform, camera transport, measured head position, actual robot model/joint mappings, and table height before acquisition. The current live RGB-D route uses 640 × 480 images. Factory camera parameters can seed acquisition; they are not evidence that calibration has passed. Keep `calibration.verified`, `execution.allow_real`, and `execution.hold_behavior_verified` false. The template's identity camera-to-base transform is an unverified acquisition placeholder; never use it to plan motion.

The model must describe the installed wrists and passive attachments. The current adapter maps exactly 14 arm joints and two head joints; other joints must be fixed while retaining their collision geometry. Synchronize robot, camera, and host clocks. The acquisition code rejects stale images and missing or inconsistent head feedback.

### Read the fixed head pose

On the current TRON2 deployment, `/joint_states` is a ROS 2 Foxy topic on the development host, not a topic on its ROS 1 Noetic master. A ROS 1 `rostopic list` against `10.192.1.4:11311` therefore does not show it. Log in to the development host, load Foxy, and map joint names to positions instead of relying on fixed array indices:

```bash
ssh -o BatchMode=yes guest@10.192.1.4 'bash -s' <<'REMOTE'
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0
python3 - <<'PY'
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('read_head_joint_state_once')
head_q2 = None

def receive(message):
    global head_q2
    positions = dict(zip(message.name, message.position))
    required = ('head_pitch_Joint', 'head_yaw_Joint')
    if all(name in positions for name in required):
        head_q2 = [positions[name] for name in required]

node.create_subscription(JointState, '/joint_states', receive,
                         qos_profile_sensor_data)
deadline = time.monotonic() + 5.0
while head_q2 is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
if head_q2 is None:
    raise SystemExit('no head joint state received within 5 seconds')
print(json.dumps({'head_q2': head_q2}, indent=2))
PY
REMOTE
```

`BatchMode=yes` forces SSH to use an existing public key and prevents a password prompt; the complete block above runs on the development host. Passwordless login has been verified from the `dc` user's environment on the DC. If the command immediately reports `Permission denied (publickey)`, run `whoami` and confirm that it is being run on the DC as the user for whom the key is configured, rather than entering or storing an unknown password.

Copy the printed `[head_pitch_Joint, head_yaw_Joint]` values, in radians, into `calibration.head_q2`. Read them only after placing the head in the pose that will remain fixed throughout intrinsic and hand-eye collection. Re-read them if the head moves; do not assume that a visually level head is `[0, 0]`.

### Identify the object radius

`scene.object_radius_m` belongs to the task object tracked under `vision.mesh_id`—for example, the bowl when the registered FoundationPose mesh is the bowl. It is not the robot, gripper, camera, calibration board, or table radius. In metres, it is the radius of a conservative sphere centred at the registered mesh origin that contains every mesh vertex after applying the deployed mesh scale:

```text
scene.object_radius_m >= max(norm(vertex_m - mesh_origin_m))
```

If the mesh origin is off-centre, the required sphere may be much larger than half the object's width. This collision-envelope value is distinct from `pregrasp.<side>.radius_m`, which selects an axial object's approach-surface anchor. Record the mesh ID, scale, origin convention, and computed enclosing radius together; changing any of them invalidates the value.

Measure the chessboard square size and count **inner corners**. The intrinsic command below uses the current `7x9` inner-corner board with `0.020` m squares; the hand-eye examples still use a `9x6` inner-corner board with `0.025` m squares. Every argument must match the physical target. During hand-eye sampling, rigidly attach the board to the selected wrist. Its mounting must not change within one sample set. Use the robot's separately reviewed positioning interface to change arm poses, then wait for settling before each capture. Collection itself only reads camera and joint state. Hand-eye sampling can use an ArUco target instead of a wrist-mounted chessboard; see the ArUco subsection under hand-eye collection.

## Fit color intrinsics with visual guidance

Start the helper from the repository root in the configured camera environment. Use a new or empty session directory:

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-seed.json --stage intrinsics \
  --pattern 7x9 --square-m 0.020 \
  --output calibration_data/intrinsics-guided
```

Open the printed URL, normally `http://127.0.0.1:8790`. Click **Preview (do not save)** to check board visibility and corner detection, then **Capture and save a new sample** for a fresh accepted view. Move the board across the image and vary its distance and tilt; keep the camera resolution and head pose fixed. The helper saves original, unrectified images under `view-*/color.png`. Capturing reads the camera again; it does not save a possibly old preview.

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

1. Click **Preview (do not save)** and check that the whole board and its detected corners are visible.
2. Reposition the wrist through the robot's separately reviewed controls. Vary rotation axes and position while keeping the board visible and the head fixed.
3. Wait for the arm and board to settle, then click **Capture and save a new sample**. Check the saved count and displayed wrist rotation change before repeating.

The helper guides sample collection; wrist positioning remains manual, and it does not generate the next wrist pose. **Solve** becomes available after at least five accepted samples and at least one wrist orientation differing from the first sample by 15 degrees or more. This is the minimum solver requirement, not an accuracy check or a requirement to rotate 15 degrees at every step. Use rotations about different axes and keep the board's mounting unchanged.

The collector brackets the image with fresh joint feedback and rejects arm/head motion, excessive timestamp skew, and head-only synchronization fallback. All samples must use the same side, camera, board, source and stationary head pose. Failed captures do not increase the accepted count; correct the displayed issue and capture again.

Click **Solve** when ready. Samples are saved under `calibration_data/handeye-left-guided/samples/*.json`; the separate result is `calibration_data/handeye-left-guided/handeye-left.json`. The result is still unverified until the independent check below. Stop the helper with Ctrl-C.

For a right-wrist sample set, use `--side right` and a separate `handeye-right-guided` directory. Solve the two sets independently; do not mix them. Their camera-to-base results should agree within the measured error budget. The sample field `robot_gripper_to_base` stores the selected **wrist** transform, despite its inherited name.

### ArUco targets instead of a chessboard

A wrist-mounted chessboard is bulky. The fixed-camera eye-to-hand solve also accepts an ArUco target, and its geometry is never hard-coded: you supply a JSON spec through `--target aruco --target-spec`. Intrinsic calibration still uses a chessboard.

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-intrinsics.json --stage handeye --side left \
  --target aruco --target-spec configs/local-aruco-target.json \
  --output calibration_data/handeye-left-guided
```

`configs/aruco-marker.example.json` covers a single marker and `configs/aruco-board.example.json` covers a rigid multi-marker board. Copy one into a local file (`configs/local*.json` is not tracked) and replace every measured value.

| Field | Applies to | Meaning |
| --- | --- | --- |
| `kind` | both | `marker` for one marker, `board` for a rigid multi-marker layout |
| `dictionary` | both | a `cv2.aruco` dictionary name such as `DICT_6X6_250` |
| `marker_length_m` | both | printed black-square edge length in metres, excluding any white border |
| `marker_id` | `marker` | printed id of the single marker |
| `markers_x`, `markers_y` | `board` | markers per row and per column of the printed layout |
| `marker_separation_m` | `board` | gap between adjacent black squares, not centre distance |
| `first_marker_id` | `board` | starting id when the ids run consecutively in row-major order |
| `marker_ids` | `board` | explicit row-major id list; use it instead of `first_marker_id` for a non-consecutive layout |
| `frame_marker_id` | `board` | marker whose printed top-left corner is the target origin |
| `min_visible_markers` | `board` | markers a frame must show to count; default 2 |
| `min_solution_ratio` | both | how much worse the alternative planar solution must fit; default 2.0 |

The target frame is the standard ArUco frame of `frame_marker_id`: origin at that marker's printed top-left corner, +x along its printed right edge, +y along its printed bottom edge, +z into the printed plane. You measure only the printed geometry. The marker-to-wrist mounting offset does not need to be measured, because it cancels in the eye-to-hand solve, but it must not change within one sample set.

A planar target always has a second pose solution. A frame is rejected unless that alternative fits at least `min_solution_ratio` times worse, so a small or nearly front-facing target is refused instead of being accepted at a silently wrong tilt. In practice: prefer a board over a single small marker, keep the target within roughly half a metre of the top camera, and keep several markers in view. A single 5 cm marker at one metre is not accurate even when it is detected.

CLI collection takes the same arguments, for example `record-sample --side left --target aruco --target-spec configs/local-aruco-target.json`. Every other collection and acceptance rule is unchanged: five samples, 15-degree rotation spread, stationarity, synchronization and the independent held-out point check all still apply, and each sample records the target spec so that a later change invalidates the comparison.

## Validate held-out points and apply the result

A held-out point is just a point that was not used to compute the transform: you measure a few of them afterwards to check the result. The hand-eye residuals shown earlier describe sample consistency, not accuracy, so these points are the only independent evidence. They are measured by hand, and the program only consumes the two arrays.

Pick at least three fixed points on the table or on the fixture that the top camera can see and the arm can reach, for example tape crosses, a fixture tip or a marked corner. Measure each point twice — once in camera coordinates, once in `base_Link` — keep both arrays in the same order, and never derive `points_base` from the transform being tested.

### Camera coordinates

Take one rectified RGB-D capture and keep it with the measurements:

```bash
.venv/bin/tron2-deploy capture \
  --profile configs/local-robot-intrinsics.json \
  --output calibration_data/heldout/frame
```

That writes `color.png` (rectified), `depth.png` (16-bit millimetres, aligned to colour) and `frame.json`. In a terminal with a graphical desktop, run the following command. Left-click the held-out points in `color.png` in order, then press Enter (or middle-click) to finish. The terminal prints a `pixels` list ready to copy. You can zoom with the figure toolbar first; leave zoom mode before selecting points.

```bash
.venv/bin/python - <<'PY'
import matplotlib.pyplot as plt

image = plt.imread('calibration_data/heldout/frame/color.png')
fig, ax = plt.subplots()
ax.imshow(image, interpolation='nearest')
ax.set_title('Left-click points in order; Enter or middle-click to finish')
chosen = plt.ginput(n=-1, timeout=0)
plt.close(fig)
pixels = [(round(x), round(y)) for x, y in chosen]
print(f'pixels = {pixels}')
PY
```

Copy the printed `pixels` list into the script below and back-project it. Coordinates are `(u, v)`, with `(0, 0)` at the image's top-left corner:

```bash
.venv/bin/python - <<'PY'
import json
import cv2
import numpy as np
pixels = [(320, 240), (180, 300), (470, 210)]   # replace with your measured pixels
frame = json.load(open('calibration_data/heldout/frame/frame.json'))
k = np.array(frame['intrinsics'], dtype=float)
depth = cv2.imread('calibration_data/heldout/frame/depth.png', cv2.IMREAD_UNCHANGED)
points = []
for u, v in pixels:
    z = float(depth[v, u]) * frame['depth_scale']
    if not z > 0:
        raise SystemExit(f'no depth at pixel ({u}, {v}); pick another pixel')
    points.append([(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z])
print(json.dumps(points))
PY
```

Use this rectified capture, not an intrinsic `--raw` view: `depth.png` is already aligned to the rectified colour, so the back-projection needs only `K`.

### Base-frame coordinates

First calibrate the position of a temporary or permanent tip. Choose one fixed point that can be touched repeatably. Through the robot's own reviewed controls, touch that point with the **same tip** on one arm in at least four clearly different wrist tilt orientations; read the joints after each stable contact. The commands below only read state. The fitting script runs offline forward kinematics and does not command motion. Use a repeatable point or hole rather than different places on a broad mark.

```bash
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/tcp-pivot/pose-01.json
# Change wrist tilt through the robot controls; touch the same point with the same tip
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/tcp-pivot/pose-02.json
# Change wrist tilt again and repeat contact
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/tcp-pivot/pose-03.json
# Fourth distinct tilt
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/tcp-pivot/pose-04.json

.venv/bin/python scripts/solve_tcp_pivot.py \
  --profile configs/local-robot-intrinsics.json --side left \
  --states calibration_data/tcp-pivot/pose-{01,02,03,04}.json \
  > calibration_data/tcp-pivot/result.json
cat calibration_data/tcp-pivot/result.json
```

`tip_in_wrist_m` gives the first three numbers of `wrist_to_tcp_pose7`. `touch_residuals_mm` and `max_residual_mm` show contact consistency; the default maximum residual is 5 mm, adjustable with `--max-residual-mm` to match measurement accuracy. Insufficient orientation spread or excessive residual causes an error; collect new samples. Fit residuals check only these contacts, not independent acceptance. The script strictly checks limits on the selected arm. If only the other arm's measured joints exceed configured limits, it clips them for offline FK and lists them under `ignored_other_arm_limit_violations`; the selected wrist calculation is unaffected. Still investigate any discrepancy between robot readings and configured limits separately. Touching one fixed point **cannot determine TCP orientation**: rotating the TCP axes leaves the tip at that point. Independently determine the TCP `+Z` approach and `+Y` up axes from the installed tool geometry, then pass their unit quaternion in wrist coordinates as `--tcp-quat-wxyz QW QX QY QZ` to the same command; only then does the result include a complete `wrist_to_tcp_pose7`. Use `1 0 0 0` only if measurements confirm the TCP and wrist axes align. The script does not edit the profile; inspect the result and enter it in `pregrasp.left.wrist_to_tcp_pose7` (use `--side right` and separate samples for the right arm).

Then touch **several different points** for independent camera-extrinsic validation with the calibrated TCP-origin tip and read the joints:

```bash
.venv/bin/tron2-deploy state \
  --profile configs/local-robot-intrinsics.json \
  --output calibration_data/heldout/state-01.json
```

Convert that reading into the touched point in `base_Link`:

```bash
.venv/bin/python - <<'PY'
import json
from tron2_deployment.config import load_profile
from tron2_deployment.geometry import pose_matrix
from tron2_deployment.kinematics import RobotModel
profile = load_profile('configs/local-robot-intrinsics.json')
state = json.load(open('calibration_data/heldout/state-01.json'))
side = 'left'   # the wrist that touched the point
model = RobotModel(profile)
model.set_state(state['arm_q14'], state['head_q2'])
touched = pose_matrix(model.wrist_poses()[side]) @ pose_matrix(profile['pregrasp'][side]['wrist_to_tcp_pose7'])
print(touched[:3, 3].tolist())
PY
```

Repeat the `state` reading and the conversion for every point, always with the same `side`. Confirm that the same calibrated tip touches each point and that the tool mount has not moved; otherwise recalibrate the tip position.

Spread the points out: at least three, not collinear, and varied in depth, height and horizontal position, because the check is a worst-case gate rather than an average. Choose the tolerance from the depth sensor's real accuracy and the deployment clearance budget; the `0.005` below means 5 mm and may be tighter than an RGB-D can support across the whole workspace.

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

### Optional: generate depth-free validation points from a chessboard

You may reuse the **physical chessboard** used for intrinsics, but take new validation images after solving extrinsics. Do not reuse hand-eye fitting images or derive base points through the extrinsic transform being checked. The script reads the inner-corner count and measured square size from the intrinsic JSON, detects corners in the color image, estimates board pose with PnP, and uses saved joint states and the calibrated TCP to locate the touched corners in `base_Link`. It does not read depth, connect to the robot, or command motion.

Fix the board where the arm can reach it. For each placement, use `capture --raw` to save original color. Keep the board, camera head, and tip mount stationary while the tip touches the selected **inner corners**. Save a distinct state file after each stable touch. Move the arm only through its existing reviewed controls. If contact moves the board, capture again. After moving the board to a new placement, fix it before taking another image. The example uses a `7x9` inner-corner board; row and column indices are zero-based. Visually confirm the detected corner ordering, especially the first corner. `view-01` covers the first two touches; fix the board at a new placement before capturing `view-02` and recording the third touch.

```bash
.venv/bin/tron2-deploy capture --raw --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/view-01
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/state-01.json
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/state-02.json
# Fix the board at a second placement, then capture it again
.venv/bin/tron2-deploy capture --raw --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/view-02
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/state-03.json

.venv/bin/python scripts/validate_chessboard_extrinsics.py \
  --profile configs/local-robot-intrinsics.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --handeye calibration_data/handeye-right-guided/handeye-right.json \
  --side left --max-error-m 0.005 --max-reprojection-px 2.0 \
  --touch calibration_data/heldout-board/view-01 0 0 calibration_data/heldout-board/state-01.json \
  --touch calibration_data/heldout-board/view-01 8 6 calibration_data/heldout-board/state-02.json \
  --touch calibration_data/heldout-board/view-02 2 4 calibration_data/heldout-board/state-03.json \
  --output calibration_data/heldout-board/points.npz
```

Each `--touch` supplies a capture directory, inner-corner row, column, and the corresponding state file; add more than three touches when possible. First open `points-marked/view-*.png` and confirm each red circle marks the corner actually touched. The script prints per-point 3D errors, the maximum error, and PnP reprojection errors for each image. It writes `points.npz` and `points.report.json`. The example `--max-reprojection-px 2.0` is a quality gate; choose it from observed corner quality and inspect the images and intrinsics if it fails. On 3D validation failure the script exits with code 1 but retains diagnostics. After the check passes, supply `points.npz` to `apply-calibration` below as `--validation-points`. Corners in one image share a PnP pose, so repeat at different workspace positions and board tilts. PnP still depends on intrinsics and board dimensions, while contact depends on TCP accuracy and board rigidity. A small reprojection error alone cannot establish millimetre-scale 3D accuracy. Set the maximum error from the measurement and deployment clearance budget.

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

For each side, record `pregrasp.<side>.wrist_to_tcp_pose7` as `[x,y,z,qw,qx,qy,qz]`: the TCP frame expressed in the configured wrist body frame, with translation in metres and a unit quaternion. The fixed-point fit above can supply translation; orientation still requires an independent determination from installed mount geometry. TCP `+Z` is the inward approach axis; TCP `+Y` is the roll/up reference used by target selection. An identity transform is valid only when these physical frames coincide.

Measure `scene.table_z_m` in `base_Link`; choose `scene.object_radius_m` to enclose the entire registered object mesh about its pose origin. Retain the session directories, raw images, sample files, both solves when available, held-out measurements, mount measurements, and accepted model revision. For an unexpected fit or failed point check, use `.venv/bin/tron2-deploy calibration-report --help` to see the optional diagnostic report inputs; retain generated reports with their inputs. The helper and reports do not apply calibration or enable execution. A passed extrinsic fit does not verify collision geometry, wrist mounting, transport timing, or stop behavior.

Moving the head away from the calibrated pose invalidates this fixed-head route. Restore that measured pose or recalibrate; the service does not extrapolate a camera/head kinematic chain. Continue with [deployment](deployment.md), then [pregrasp planning](pregrasp.md).
