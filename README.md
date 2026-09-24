# TRON2 deployment on DC

[简体中文](README.zh-CN.md)

Independent deployment tools for **object-aware dual-wrist pregrasp**: locate an object, choose a reachable approach and wrist orientation, preview the trajectory in RViz, move to the standoff pose, and stop. There are no gripper commands or contact/grasp actions in this application.

The installed head/top RGB-D camera is an **Intel RealSense D455**. The runtime name `cam_high` and `/camera/top/...` topics refer to this D455.

The object pose informs target geometry; its quaternion is not copied to the wrists. Axially symmetric objects use the current wrist's radial approach, making target selection invariant to uninformative object yaw. Asymmetric objects use a configured object-frame anchor and outward direction. TCP +Z points toward the object and +Y follows the projected object up axis; the calibrated wrist-to-TCP transform converts that frame to a wrist goal.

## Repository architecture

```text
tron2_deployment_on_dc/
├── configs/                 # Full deployment profiles and robot models
│   ├── assembly.urdf        # Final URDF including the installed dexterous hands
│   ├── robot.urdf           # Base robot URDF without the dexterous hands
│   ├── robot.xml            # Base robot MuJoCo model
│   ├── scene.xml            # Final scene/model used by deployment checks
│   ├── robot.example.json   # Deployment profile template
│   └── demo.json            # Synthetic mock profile
├── sp_vision/               # Standalone camera calibration unit
│   ├── configs/             # Calibration configs and model snapshots
│   ├── data/                # Local captures and generated results, ignored by Git
│   ├── head_calib.md        # Head-camera calibration guide
│   └── wrist_calib.md       # Wrist-camera calibration guide
├── src/                     # Main deployment package and CLI
├── scripts/                 # Installation and offline helper scripts
├── tests/                   # Full-package software and mock tests
└── docs/                    # Deployment, pregrasp and operation guides
```

`configs/assembly.urdf` is the authoritative final kinematic model for the installed robot with dexterous hands. `configs/robot.urdf` is retained as the hand-free base model and must not be substituted for `assembly.urdf` during final calibration, model hashing or real deployment. The copies under `sp_vision/configs/` are standalone calibration snapshots; they keep offline calibration independent from the parent deployment package and are not operator profiles.

| Stage | Guide | Output |
| --- | --- | --- |
| 1. Software deployment and mock | [DC device: Ubuntu 20.04](docs/deployment.md) | Full runtime, pytest, synthetic demo and mock operator |
| 2. Geometry calibration and validation | [`sp_vision` calibration unit](sp_vision/head_calib.md) | Head/wrist camera intrinsics/extrinsics, TCP and independent touch validation |
| 3. Real deployment profile | [DC device: Ubuntu 20.04](docs/deployment.md) | Feed reviewed calibration results into `configs/` profiles |
| 4. Planning and execution | [Pregrasp workflow](docs/pregrasp.md) | Checked dual-arm plan, RViz review, preflight and measured execution log |

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

Use the standalone `sp_vision/` head and wrist calibration programs for current camera geometry calibration; they do not replace the full deployment package or enable real execution automatically. After independent touch validation, review and feed the results into the deployment profiles under `configs/`. The old calibration workflow and ArUco examples are removed; `sp_vision` is now the calibration entry point.

## Implementation and limits

The planner follows a lift–transit–descend corridor and fails if IK or collision checks reject it. It does not search arbitrary obstacle routes. The object is conservatively enclosed by a configured sphere. Numeric MuJoCo supplies FK/IK and collision queries; **RViz is the trajectory viewer**. The deployment model must represent the actual fixed base and installed attachments, with only the mapped 14 arm and 2 head joints movable. Uncommanded attachments remain fixed and retain collision geometry.

Plans bind the object observation, calibration/profile fingerprint, compiled model hash, initial feedback, target poses and exact time interpolation. Real execution requires fresh real observations/feedback, reviewed plan identity, accepted calibration/model/hold behavior, and explicit supervision confirmation. Hardware timing, model accuracy, calibration and hold behavior remain deployment acceptance work; see [operation and stopping](docs/operation.md).

```bash
.venv/bin/python -m pytest -q
```

The package has no runtime imports from `dexpipe`, Gaia20, RL or retargeting pipelines. FoundationPose and SAM GPU services remain external; this repository contains their RPC clients. `third_party/tron2_env` preserves the patched transport snapshot and upstream notices without nested Git metadata. `SOURCE_MANIFEST.json` records its origin and the source ports. Source history remains in [dexpipe](https://github.com/Shukashuki/dexpipe).

