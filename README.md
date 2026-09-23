# TRON2 deployment on DC

[简体中文](README.zh-CN.md)

Independent deployment tools for **object-aware dual-wrist pregrasp**: locate an object, choose a reachable approach and wrist orientation, preview the trajectory in RViz, move to the standoff pose, and stop. There are no gripper commands or contact/grasp actions in this application.

The installed head/top RGB-D camera is an **Intel RealSense D435**. The runtime name `cam_high` and `/camera/top/...` topics refer to this D435.

The object pose informs target geometry; its quaternion is not copied to the wrists. Axially symmetric objects use the current wrist's radial approach, making target selection invariant to uninformative object yaw. Asymmetric objects use a configured object-frame anchor and outward direction. TCP +Z points toward the object and +Y follows the projected object up axis; the calibrated wrist-to-TCP transform converts that frame to a wrist goal.

| Stage | Guide | Output |
| --- | --- | --- |
| 1. Calibration | [Calibration workflow](docs/calibration.md) | Guided image/sample collection, intrinsics and hand-eye solve, independent validation and accepted deployment profile |
| 2. Deployment | [DC device: Ubuntu 20.04](docs/deployment.md) | Installed runtime, configured camera/vision/robot connections and compact operator |
| 3. Planning and execution | [Pregrasp workflow](docs/pregrasp.md) | Checked dual-arm plan, RViz review, preflight and measured execution log |

The verified DC host runs Ubuntu 20.04.6 on x86_64, with glibc 2.31, a separate Python 3.10.13 installation and ROS Noetic. Use that existing device environment; keep Ubuntu's system Python 3.8 unchanged. Install the project environment before acquiring live calibration; see the [device setup](docs/deployment.md) for interpreter selection and ROS checks. Keep both guide languages synchronized.

## Try the complete mock workflow

```bash
git clone https://github.com/Shukashuki/tron2_deployment_on_dc.git
cd tron2_deployment_on_dc
bash scripts/install.sh
.venv/bin/tron2-deploy demo --output output/demo
.venv/bin/tron2-deploy operator --profile configs/demo.json --mock
```

Open `http://127.0.0.1:8787`. The demo uses synthetic RGB-D, a toy dual-arm model and simulated feedback. It exercises validation, estimation, orientation-aware targets, IK, collision/limit checks, timed motion and logs. It does not establish hardware readiness. Live execution is an explicit CLI operation; the browser executes mock plans only.

Start calibration from the CLI with `tron2-deploy calibration-guide`. Its browser page helps you see the board, capture usable views, track sample progress, and solve. The result shows the saved file and next step, plus the intrinsic RMS or hand-eye sample count. Apply the result through the existing CLI commands after independent validation. Detailed offline reports remain optional troubleshooting tools. See the [calibration workflow](docs/calibration.md).

## Implementation and limits

The planner follows a lift–transit–descend corridor and fails if IK or collision checks reject it. It does not search arbitrary obstacle routes. The object is conservatively enclosed by a configured sphere. Numeric MuJoCo supplies FK/IK and collision queries; **RViz is the trajectory viewer**. The deployment model must represent the actual fixed base and installed attachments, with only the mapped 14 arm and 2 head joints movable. Uncommanded attachments remain fixed and retain collision geometry.

Plans bind the object observation, calibration/profile fingerprint, compiled model hash, initial feedback, target poses and exact time interpolation. Real execution requires fresh real observations/feedback, reviewed plan identity, accepted calibration/model/hold behavior, and explicit supervision confirmation. Hardware timing, model accuracy, calibration and hold behavior remain deployment acceptance work; see [operation and stopping](docs/operation.md).

```bash
.venv/bin/python -m pytest -q
```

The package has no runtime imports from `dexpipe`, Gaia20, RL or retargeting pipelines. FoundationPose and SAM GPU services remain external; this repository contains their RPC clients. `third_party/tron2_env` preserves the patched transport snapshot and upstream notices without nested Git metadata. `SOURCE_MANIFEST.json` records its origin and the source ports. Source history remains in [dexpipe](https://github.com/Shukashuki/dexpipe).

## Recheck the head camera using earlier touch points

After accepting the right-side TCP pivot and saving the first checkerboard touch selection and three touch states, keep the checkerboard fixed in the same base-frame position and orientation, and keep the same rigid tip. The head camera may move. Capture one new color image with its synchronized head pose, reuse the earlier three physical corner indices in their original touch order, and compare the newly predicted base-frame points with the saved arm states. This is an offline check; it does not move the robot or require new touches.

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/touch-recheck --count 1

.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json select-validation \
  --pattern 7x10 --square-m 0.021 \
  --frame sp_vision/data/touch-recheck/view-001 \
  --intrinsics sp_vision/data/head-camera-session/intrinsics.json \
  --extrinsics sp_vision/data/head-camera-session/extrinsics.json \
  --reuse-selection sp_vision/data/touch-validation/selection.json \
  --output sp_vision/data/touch-recheck/selection.json

.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json validate \
  --selection sp_vision/data/touch-recheck/selection.json \
  --side right --tcp sp_vision/data/tcp/result.json \
  --states sp_vision/data/touch-validation/state-{01,02,03}.json \
  --output sp_vision/data/touch-recheck/report.json
```

Inspect `touch-recheck/selection.png` to confirm the labeled corners are the same physical corners. The new image supplies one board PnP pose; the saved states supply the three base-frame touch positions. If the board moved since the original touches, or the tip changed, the comparison is invalid. The report gives each 3D distance in metres; an error above the configured threshold exits with status 1. See the [moving-head calibration guide](sp_vision/calibration.md) for the original touch workflow.
