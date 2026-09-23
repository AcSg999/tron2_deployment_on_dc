# Deployment on the DC Ubuntu 20.04 device

[简体中文](deployment.zh-CN.md) · [Workflow overview](../README.md) · [Previous: calibration](calibration.md) · [Next: pregrasp](pregrasp.md)

Use the existing DC control host's environment: **Ubuntu 20.04.6 LTS, x86_64, glibc 2.31, application Python 3.10 and ROS Noetic**. This baseline was checked on the device; an Ubuntu 22.04 upgrade is not required. Install the project environment before collecting live calibration. The task remains **object-aware wrist pregrasp**: approach a reviewed standoff pose and stop, with no gripper commands or contact/grasp action.

## Use the device's existing environment

| Component | Verified on DC | Use in this workflow |
| --- | --- | --- |
| Operating system | Ubuntu 20.04.6 LTS, x86_64; glibc 2.31 | Keep the installed control-host OS. |
| System Python | `/usr/bin/python3`, version 3.8.10 | Keep it for Ubuntu and installed ROS executables. |
| Application Python | `/home/dc/mambaforge/bin/python3.10`, version 3.10.13 | Create the project's isolated `.venv`; both project packages require Python ≥3.10. |
| ROS | Noetic at `/opt/ros/noetic` | Source its environment for camera messages and RViz publishing. |
| RViz / robot_state_publisher | 1.14.20 / 1.15.2 | Use the installed visualization executables and matching robot URDF. |
| FoundationPose / SAM | External GPU services configured through RPC endpoints | No local CUDA installation is required for these clients. |

Check the device before selecting the interpreter:

```bash
cat /etc/os-release
uname -m
getconf GNU_LIBC_VERSION
/usr/bin/python3 --version
command -v python3.10
python3.10 --version
```

These findings describe the DC host, not the robot controller or remote GPU server. On another device, record its actual OS, architecture and interpreter paths before adapting the setup. The dependency file below is validated for DC's Ubuntu 20.04/x86_64/Python 3.10 combination. The existing separate interpreter supplies Python 3.10 without replacing `/usr/bin/python3`; [Python's venv documentation](https://docs.python.org/3.10/library/venv.html) describes this separation.

## Install the standalone package

```bash
sudo apt update
sudo apt install -y git libgl1 libglib2.0-0 build-essential
git clone https://github.com/Shukashuki/tron2_deployment_on_dc.git
cd tron2_deployment_on_dc
TRON2_PYTHON=/home/dc/mambaforge/bin/python3.10 bash scripts/install.sh
.venv/bin/python -I -m pip check
.venv/bin/tron2-deploy --help
```

Install only missing OS packages. `TRON2_PYTHON` selects an existing Python 3.10 executable with `venv`/`ensurepip`; it defaults to `python3.10` on `PATH`. The shown path is the verified DC installation, including Python 3.10 headers; `netifaces` builds locally with the installed compiler. On another machine, use that machine's separate Python 3.10 installation and its matching headers. Ubuntu 20.04's system Python 3.8 cannot run this package; do not redirect `/usr/bin/python3` or assume Ubuntu 22.04's Python apt packages are available.

`scripts/install.sh` creates `.venv`, installs this package with its `bridge`, `dev` and `ros` helper extras, and installs the patched runtime in `third_party/tron2_env`. Installation ignores inherited Python package paths so ROS/system packages cannot silently satisfy virtual-environment dependencies. Both packages use **`opencv-contrib-python` as the single provider of `cv2`**. Keep this environment separate from installations of `opencv-python` or either headless OpenCV wheel. No `dexpipe` checkout, Gaia20 SDK, RL pipeline or retargeting package is required.

The installer uses `constraints-ubuntu20-py310.txt`, recording dependency versions tested on the actual Ubuntu 20.04.6/glibc 2.31 host in an isolated Python 3.10 environment. Updating these versions requires rerunning the tests and mock workflow; a changed MuJoCo version can also change the compiled model hash and requires renewed model/plan review.

For calibration, `calibration-guide` starts a separate local browser helper on port `8790`: preview the board, capture samples, then solve. It reads the camera only after an explicit browser action; start it with the camera's ROS environment when using ROS capture. Applying results remains an explicit CLI step. Matplotlib supports optional offline diagnostics through `calibration-report`. See the [calibration workflow](calibration.md).

Package downloads and isolated build dependencies use `https://pypi.org/simple` by default. The installer sets this index only for its own process and children; it does not rewrite your global pip configuration. To select another working index explicitly, set `TRON2_PIP_INDEX_URL` when running the script. This uses [pip's documented configuration precedence](https://pip.pypa.io/en/stable/topics/configuration/#precedence-override-order).

If an earlier installation reported TLS errors from the Tsinghua mirror followed by `No matching distribution found for setuptools` or a missing `.venv/bin/tron2-deploy`, package installation did not finish. Rerun the updated `bash scripts/install.sh`; it reuses the existing Python 3.10 venv and retrieves the missing packages from PyPI. TLS certificate verification remains enabled. Start the frontend after the installer completes and verifies the CLI entry point.

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

## Connect the head-mounted D435 RGB-D camera

The installed head/top camera is an **Intel RealSense D435**, exposed as `cam_high` and through `/camera/top/...` topics. The real capture adapter currently supports **640×480 color and depth**. It aligns depth using the profile's factory/measured depth intrinsics and depth-to-color transform, then undistorts RGB and aligned depth together. Retain the correct metric depth scale and validate alignment for this physical D435.

For `camera.backend="ros"`, configure `ros_master_uri`, the workstation's reachable `ros_ip`, and the color, depth and joint-state topics. The template's topics are `/camera/top/color/image_raw/compressed`, `/camera/top/depth/image_rect_raw` and `/joint_states`. For `camera.backend="bridge"`, configure `bridge_host`, `bridge_path` and, if required, `token_env` naming the environment variable holding the token. Use the deployed bridge's actual route and TLS settings; the client defaults are `127.0.0.1:18443` and `/bridge/ws`.

Synchronize robot and workstation wall clocks. Capture uses the sensor timestamp, rejects frames older than `camera.max_frame_age_s`, and requires synchronized head feedback within the configured skew limits. A head position outside `calibration.head_tolerance_rad` is rejected; the fixed-head calibration does not track head motion automatically.

Reuse the device's installed ROS Noetic environment. Noetic targets Ubuntu 20.04 and system Python 3.8; see [ROS REP 3](https://github.com/ros-infrastructure/rep/blob/master/rep-0003.rst#noetic-ninjemys-may-2020---may-2025). This application uses Noetic's Python messages and `rospy` from its separate Python 3.10 environment, while installed executables such as `roscore` retain system Python. The installer supplies Python 3.10 helper dependencies, including YAML and `netifaces`. Verify this combination without connecting to the robot:

```bash
# Match the ROS setup script to the current interactive shell.
if [ -n "${ZSH_VERSION:-}" ]; then
  source /opt/ros/noetic/setup.zsh
else
  source /opt/ros/noetic/setup.bash
fi
command -v roscore rosrun rviz
.venv/bin/python -c "import yaml, netifaces, rospkg, defusedxml, rospy, rosgraph, genpy, message_filters; from sensor_msgs.msg import Image, CompressedImage, JointState; from geometry_msgs.msg import Point; from visualization_msgs.msg import Marker, MarkerArray; print('ROS imports OK')"
```

If the current robot URDF requires packages from the existing DC workspace, additionally source `/home/dc/test_ws/devel/setup.zsh` from zsh or `/home/dc/test_ws/devel/setup.bash` from Bash; use the actual workspace path on another device. Do not source a `setup.bash` file from zsh: Catkin's Bash wrapper can resolve its installation directory incorrectly and try to load `setup.sh` from the current working directory. The helper extras do not install `rospy`, ROS messages, `roscore`, `robot_state_publisher` or RViz; these come from the installed ROS environment. Bridge capture does not require local ROS, but RViz still does.

The camera adapter decodes images directly with NumPy/OpenCV and does not use `cv_bridge`. The device's installed `cv_bridge` binary is linked to Python 3.8, so it is not a Python 3.10 dependency. Keep Python 3.8 system packages out of the project environment rather than adding `/usr/lib/python3/dist-packages` to its `PYTHONPATH`. Passing imports verifies software compatibility only; camera acquisition and physical execution still require the later checks.

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
