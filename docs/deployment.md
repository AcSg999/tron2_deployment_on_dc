# Ubuntu 22.04 deployment

[简体中文](deployment.zh-CN.md) · [Workflow overview](../README.md) · [Previous: calibration](calibration.md) · [Next: pregrasp](pregrasp.md)

Deploy on native Ubuntu 22.04 with Python 3.10. Install this environment before collecting live calibration. The current task is **object-aware wrist pregrasp**: approach a reviewed standoff pose and stop. The application sends no gripper commands and performs no contact or grasp action.

## Install the standalone package

```bash
sudo apt update
sudo apt install -y python3.10-venv git libgl1 libglib2.0-0
git clone https://github.com/Shukashuki/tron2_deployment_on_dc.git
cd tron2_deployment_on_dc
bash scripts/install.sh
.venv/bin/python -m pip check
.venv/bin/tron2-deploy --help
```

`scripts/install.sh` creates `.venv`, installs this package with its `bridge` and `dev` extras, and installs the patched runtime in `third_party/tron2_env`. Both packages use **`opencv-contrib-python` as the single provider of `cv2`**. Keep this environment separate from installations of `opencv-python` or either headless OpenCV wheel. No `dexpipe` checkout, Gaia20 SDK, RL pipeline or retargeting package is required.

The installer uses `constraints-ubuntu22-py310.txt`, recording the dependency versions tested in an isolated Linux/Python 3.10 installation. Updating these versions requires rerunning the tests and mock workflow; a changed MuJoCo version can also change the compiled model hash and requires renewed model/plan review.

Verify the software with synthetic inputs:

```bash
.venv/bin/python -m pytest -q
.venv/bin/tron2-deploy demo --output output/demo
.venv/bin/tron2-deploy operator --profile configs/demo.json --mock
```

Open `http://127.0.0.1:8787`. The mock profile uses a toy model, synthetic RGB-D and simulated feedback; its result is software evidence, not a deployment acceptance record. Stop the service with Ctrl-C before starting another operator on the same port.

## Prepare the installation profile

```bash
cp configs/robot.example.json configs/local-robot-seed.json
```

The example intentionally contains `null` and `REPLACE-...` placeholders. It is not loadable until the required fields are filled. `configs/local*.json` is ignored by Git. Populate the seed with the actual installation and available factory/measured camera calibration, keeping `calibration.verified=false` and `execution.allow_real=false`. Complete [calibration](calibration.md) to produce `configs/local-robot-calibrated.json`; copying the template or entering factory values does not independently verify the camera-to-base transform.

| Profile fields | Required installation data |
| --- | --- |
| `robot.model_xml`, `base_body`, `wrist_bodies`, joint names | Current fixed-base MuJoCo model with exactly the mapped 14 arm and 2 head joints movable. Fix other attachments while preserving their collision geometry. Match `base_Link` and actual wrist links. |
| `robot.urdf`, joint/velocity/acceleration limits | URDF and resolvable meshes matching the XML and installed attachments; 14-element limit arrays in left-arm-then-right-arm order. Angles are radians. Use limits accepted for the real controller and model. |
| `robot.host`, `port` | Actual TRON2 control endpoint; default port is `5000`. The patched adapter reads arm/head feedback without requesting gripper feedback. |
| `camera` | Physical identity, ROS or bridge connection, `640×480` color/depth calibration, color K/distortion, depth K and rigid `depth_to_color` transform. |
| `calibration` | Camera-to-`base_Link` transform, fixed calibrated head pitch/yaw, tolerance and calibration identity. Keep the head at that pose during capture and pregrasp. |
| `scene`, `vision`, `pregrasp` | Measured table height, clearance, object sphere bound and registered mesh ID; per-side calibrated wrist-to-TCP pose7, object symmetry/approach geometry and positive standoff. Pose7 uses metres and `[x,y,z,qw,qx,qy,qz]`. |

Paths in the profile are relative to the profile file; absolute paths and environment variables are also supported. Set `TRON2_MODEL_XML` and `TRON2_URDF` if retaining those placeholders. After completing the seed, compute its compiled model hash and copy the reported value into `robot.model_hash`:

```bash
.venv/bin/tron2-deploy model-hash --profile configs/local-robot-seed.json
```

Recompute and review the model after geometry changes. The XML must include the real collision geometry; a hash confirms the selected model identity, not its physical accuracy. Numerical MuJoCo supplies FK/IK and collision checks without a viewer. RViz is the only graphical trajectory review step.

## Connect the top RGB-D camera

The real capture adapter currently supports **640×480 color and depth**. It aligns depth using the profile's factory/measured depth intrinsics and depth-to-color transform, then undistorts RGB and aligned depth together. Retain the correct metric depth scale and validate alignment for the selected physical camera.

For `camera.backend="ros"`, configure `ros_master_uri`, the workstation's reachable `ros_ip`, and the color, depth and joint-state topics. The template's topics are `/camera/top/color/image_raw/compressed`, `/camera/top/depth/image_rect_raw` and `/joint_states`. For `camera.backend="bridge"`, configure `bridge_host`, `bridge_path` and, if required, `token_env` naming the environment variable holding the token. Use the deployed bridge's actual route and TLS settings; the client defaults are `127.0.0.1:18443` and `/bridge/ws`.

Synchronize robot and workstation wall clocks. Capture uses the sensor timestamp, rejects frames older than `camera.max_frame_age_s`, and requires synchronized head feedback within the configured skew limits. A head position outside `calibration.head_tolerance_rad` is rejected; the fixed-head calibration does not track head motion automatically.

ROS capture and RViz require an existing, tested ROS 1 environment compatible with the selected Python runtime. This installer does not install ROS Noetic, and a native Ubuntu 22.04 installation alone does not supply that environment. Install the Python helper extras and source the tested workspace, replacing the path below:

```bash
.venv/bin/python -m pip install '.[ros]'
source /absolute/path/to/tested_ros1_workspace/devel/setup.bash
.venv/bin/python -c "import rospy; import sensor_msgs.msg; import geometry_msgs.msg; import visualization_msgs.msg"
```

The helper extras do not install `rospy`, ROS messages, `roscore`, `robot_state_publisher` or RViz. Confirm those come from the compatible ROS environment. Bridge capture does not require local ROS; RViz still does.

## Connect FoundationPose and SAM

Run the existing GPU services separately. Configure `vision.pose_endpoint` for FoundationPose, normally `tcp://127.0.0.1:5557`, and `vision.mask_endpoint` for SAM, normally `tcp://127.0.0.1:5560`. Forward those ports when the services are remote. There is no live-pose relay and no requirement for port `5558`.

GPU weights and object meshes are external assets. `vision.mesh_id` must identify the deployed mesh, with its metric scale and origin matching the object profile. `scene.object_radius_m` must conservatively enclose that mesh about the estimated object origin; the planner uses this sphere for object clearance. Verify the table, TCP and approach geometry against that same coordinate convention. The object quaternion informs target selection; it is not copied directly to the wrists.

## Start real observation and planning

After calibration, start the operator in the configured camera environment:

```bash
.venv/bin/tron2-deploy operator \
  --profile configs/local-robot-calibrated.json \
  --host 127.0.0.1 --port 8787
```

The page follows capture → box mask → FoundationPose → left/right pregrasp planning → saved plan and RViz review. Startup reads configuration only. Capture accesses the camera; planning reads current robot feedback and checks the path. Neither operation commands motion. The service binds to loopback; use an SSH tunnel for remote browser access.

Browser execution is available only with `--mock`. **Real execution is an explicit `tron2-deploy execute --real` CLI operation**, with the reviewed plan ID and supervision confirmation described in [pregrasp](pregrasp.md) and [operation and stopping](operation.md). Keep execution disabled until the installation, calibration, current model and hold behavior have been accepted.

## Prepare RViz and hand off the plan

In the compatible ROS environment, preview a saved plan with its exact profile:

```bash
.venv/bin/python -m tron2_deployment.rviz \
  --profile configs/local-robot-calibrated.json \
  --plan /absolute/path/to/pregrasp-plan.json \
  --ros-master-uri http://127.0.0.1:11331 --once
```

The command starts an independent local visualization master when needed, publishes the model/object/wrist paths and opens RViz. It never uses the robot/camera master or port `11311`. Real-plan review requires the matching `robot.urdf`; the mock demo can show markers without one. `--once` plays once and holds the final pregrasp pose; Ctrl-C closes the preview processes it started.

Continue with [pregrasp planning and execution](pregrasp.md), keeping the accepted profile, fixed observation, saved plan, RViz review and measured execution log together. Changes to calibration, model or profile require a new observation/plan and review.
