# Calibration before deployment

[简体中文](calibration.zh-CN.md) · [README](../README.md) · Next: [deployment](deployment.md)

The first route uses the top RGB-D camera at a fixed, measured head pose. Calibrate the color camera, solve its transform into `base_Link`, and independently validate that transform before preparing pregrasp targets. Record both wrist frames and their TCP mounting transforms. This workflow sends no gripper commands.

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

That writes `color.png` (rectified), `depth.png` (16-bit millimetres, aligned to colour) and `frame.json`. Read the pixel of each chosen point in `color.png`, list them below, and back-project them:

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

For the same points, jog the arm through its own reviewed controls until the tip that coincides with the configured TCP frame touches the point, then read the joints:

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

Repeat the `state` reading and the conversion for every point, always with the same `side`. If the installed end effector has no tip at the configured TCP origin, mount a temporary one or correct `pregrasp.<side>.wrist_to_tcp_pose7` first; otherwise the point you touched is not the point you measured.

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
