# 在 DC 的 Ubuntu 20.04 设备上部署

[English](deployment.md) · [工作流总览](../README.zh-CN.md) · [相机标定：sp_vision](../sp_vision/head_calib.zh-CN.md) · [下一步：预抓取](pregrasp.zh-CN.md)

沿用 DC 控制主机的现有环境：**Ubuntu 20.04.6 LTS、x86_64、glibc 2.31、应用 Python 3.10 和 ROS Noetic**。这些基线信息已在设备上核实，无需升级到 Ubuntu 22.04。采集真实标定数据前，先安装项目环境。任务仍是**考虑物体几何的腕部预抓取**：接近经过审核的退让位姿并停止，不发送夹爪命令，也不执行接触或抓取动作。

## 沿用设备的现有环境

| 组件 | DC 上已核实的信息 | 本流程中的用途 |
| --- | --- | --- |
| 操作系统 | Ubuntu 20.04.6 LTS、x86_64；glibc 2.31 | 保留控制主机现有操作系统。 |
| 系统 Python | `/usr/bin/python3`，版本 3.8.10 | 保留给 Ubuntu 和已安装的 ROS 可执行程序使用。 |
| 应用 Python | `/home/dc/mambaforge/bin/python3.10`，版本 3.10.13 | 创建项目独立的 `.venv`；两个项目软件包均要求 Python ≥3.10。 |
| ROS | Noetic，位于 `/opt/ros/noetic` | 加载该环境，用于相机消息和 RViz 发布。 |
| RViz / robot_state_publisher | 1.14.20 / 1.15.2 | 使用已安装的可视化程序和匹配的机器人 URDF。 |
| FoundationPose / SAM | 通过 RPC 端点配置的外部 GPU 服务 | 这些客户端不要求本地安装 CUDA。 |

本仓库现在有两个互补的部分，不能互相替代：

1. 完整部署包：提供机器人运行时、RGB-D 采集、FoundationPose/SAM 客户端、pregrasp 规划、RViz 审核、mock workflow 和受控执行入口。
2. `sp_vision/` 独立标定单元：提供头部相机和右腕相机的棋盘格内参、PnP/手眼外参、TCP pivot 与独立触点验证。它可以离线运行，不依赖 `tron2_deployment`；实时采集时仍需要相机/机器人所在环境的适配器。

因此正确顺序仍是：**部署软件环境 → mock 测试 → 准备现场 profile → 使用 `sp_vision` 标定 → 独立验证并回填/生成部署配置 → 连接真实观测与视觉服务 → pregrasp 规划和 RViz 审核 → 在明确监督门禁下执行**。独立 `sp_vision` 不是完整部署流程的替代品。

选择解释器前，检查设备信息：

```bash
cat /etc/os-release
uname -m
getconf GNU_LIBC_VERSION
/usr/bin/python3 --version
command -v python3.10
python3.10 --version
```

这些检查描述的是 DC 主机，不是机器人控制器或远程 GPU 服务器。在另一台设备上部署时，先记录其实际操作系统、架构和解释器路径，再调整配置。下方依赖文件已针对 DC 的 Ubuntu 20.04/x86_64/Python 3.10 组合验证。现有独立解释器提供 Python 3.10，无需替换 `/usr/bin/python3`；[Python venv 文档](https://docs.python.org/3.10/library/venv.html)说明了这种隔离方式。

## 安装独立软件包

```bash
sudo apt update
sudo apt install -y git libgl1 libglib2.0-0 build-essential
git clone https://github.com/Shukashuki/tron2_deployment_on_dc.git
cd tron2_deployment_on_dc
TRON2_PYTHON=/home/dc/mambaforge/bin/python3.10 bash scripts/install.sh
.venv/bin/python -I -m pip check
.venv/bin/tron2-deploy --help
```

只安装缺失的系统软件包。`TRON2_PYTHON` 用于选择已安装且带有 `venv`/`ensurepip` 的 Python 3.10 可执行文件，默认使用 `PATH` 中的 `python3.10`。示例路径是 DC 上已核实的安装位置，包含 Python 3.10 头文件；`netifaces` 使用已安装的编译器在本地构建。其他设备应使用其独立 Python 3.10 安装及匹配的头文件。Ubuntu 20.04 的系统 Python 3.8 无法运行本软件包；不要改写 `/usr/bin/python3` 的指向，也不要假设 Ubuntu 22.04 的 Python apt 软件包在此设备上可用。

`scripts/install.sh` 创建 `.venv`，安装本软件包及其 `bridge`、`dev` 和 `ros` 辅助依赖，并安装 `third_party/tron2_env` 中经过修补的运行时。安装时忽略继承的 Python 软件包路径，避免将 ROS/系统包误认为已安装的虚拟环境依赖。两个软件包都使用 **`opencv-contrib-python` 作为唯一的 `cv2` 提供者**。此环境应与安装了 `opencv-python` 或任一种 headless OpenCV wheel 的环境分开。不需要 `dexpipe` 工作副本、Gaia20 SDK、RL 流水线或重定向软件包。

安装脚本使用 `constraints-ubuntu20-py310.txt`，记录在实际 Ubuntu 20.04.6/glibc 2.31 主机的独立 Python 3.10 环境中验证过的依赖版本。更新版本后需重新运行测试和模拟工作流；MuJoCo 版本变化也可能改变编译模型哈希，需要重新审核模型和规划。

当前相机几何标定入口是 `sp_vision/head_calib.zh-CN.md` 和 `sp_vision/wrist_calib.zh-CN.md`。`sp_vision` 的结果不会自动启用真实执行，仍需完成独立验证、模型/profile 核对和执行门禁。审核通过后，将结果写入 `configs/` 下的部署 profile。旧标定流程和 ArUco 示例不再属于当前流程。

软件包下载和隔离构建依赖默认使用 `https://pypi.org/simple`。安装脚本只为自身及子进程设置该源，不改写全局 pip 配置。如需显式指定另一个可用源，运行脚本时设置 `TRON2_PIP_INDEX_URL`。此行为遵循 [pip 文档中的配置优先级](https://pip.pypa.io/en/stable/topics/configuration/#precedence-override-order)。

如果此前安装出现清华镜像 TLS 错误，随后提示 `No matching distribution found for setuptools` 或找不到 `.venv/bin/tron2-deploy`，说明软件包安装尚未完成。重新运行更新后的 `bash scripts/install.sh`，脚本会复用现有 Python 3.10 虚拟环境，并从 PyPI 获取缺失的软件包。TLS 证书验证保持启用。等待安装完成且 CLI 入口检查通过后，再启动前端。

使用合成输入验证软件：

```bash
.venv/bin/python -m pytest -q
.venv/bin/tron2-deploy demo --output output/demo
.venv/bin/tron2-deploy operator --profile configs/demo.json --mock
```

打开 `http://127.0.0.1:8787`。模拟配置使用简化模型、合成 RGB-D 和模拟反馈；其结果是软件验证证据，不是真机部署验收记录。在同一端口启动另一个 operator 前，先用 Ctrl-C 停止服务。

## 先完成软件与 mock 验证

这一步仍然需要，且不能被 `sp_vision` 替代：

```bash
.venv/bin/python -m pytest -q
.venv/bin/tron2-deploy demo --output output/demo
.venv/bin/tron2-deploy operator --profile configs/demo.json --mock
```

`pytest` 检查软件模块；`demo` 生成合成标定证据、计划和日志；`operator --mock` 检查采集→视觉估计→pregrasp→IK/碰撞检查→模拟执行的完整软件路径。它们都不证明真实相机、真实机器人、真实模型或真实 TCP 已经验收。打开 `http://127.0.0.1:8787`，验证后用 Ctrl-C 停止 mock 服务。

## 准备现场配置

```bash
cp configs/robot.example.json configs/local-robot-seed.json
```

示例有意保留 `null` 和 `REPLACE-...` 占位值，填写必需字段后才能加载。`configs/local*.json` 已被 Git 忽略。使用实际安装信息以及可用的出厂/实测相机标定填写初始配置，保持 `calibration.verified=false` 和 `execution.allow_real=false`。完成 `sp_vision` 的头部/腕部标定和独立触点验证后，再把已审核的矩阵、TCP 和标定身份写入部署 profile，生成 `configs/local-robot-calibrated.json`；复制模板或填写出厂参数本身不能独立验证相机到基座变换。`sp_vision` 结果 JSON 不是 operator profile，不能直接替代 profile。

| 配置字段 | 必需的现场数据 |
| --- | --- |
| `robot.model_xml`、`base_body`、`wrist_bodies`、关节名称 | 当前固定基座 MuJoCo 模型，仅映射的 14 个机械臂关节和 2 个头部关节可动。其他附件保持固定，并保留碰撞几何。匹配 `base_Link` 和实际腕部 link。 |
| `robot.urdf`、关节/速度/加速度限值 | 与 XML 和实际附件一致的 URDF 及可解析 mesh；限值数组包含 14 个元素，顺序为左臂后右臂，角度单位为弧度。采用真实控制器和模型认可的限值。 |
| `robot.host`、`port` | 实际 TRON2 控制端点，默认端口为 `5000`。修补后的适配器读取机械臂/头部反馈，不请求夹爪反馈。 |
| `camera` | 实物身份、ROS 或 bridge 连接、`640×480` 彩色/深度标定、彩色 K/畸变、深度 K，以及刚体 `depth_to_color` 变换。 |
| `calibration` | 相机到 `base_Link` 的变换、标定时固定的头部 pitch/yaw、容差及标定标识。采集和预抓取期间保持该头姿。 |
| `scene`、`vision`、`pregrasp` | 实测桌面高度、间隙、物体球形包络和已注册 mesh ID；各侧已标定的腕部到 TCP pose7、物体对称性/接近几何及正退让距离。pose7 使用米和 `[x,y,z,qw,qx,qy,qz]`。 |

配置中的相对路径以配置文件所在目录为基准，也支持绝对路径和环境变量。若保留对应占位变量，请设置 `TRON2_MODEL_XML` 和 `TRON2_URDF`。填写完整初始配置后，计算编译后模型的哈希，并将输出值写入 `robot.model_hash`：

```bash
.venv/bin/tron2-deploy model-hash --profile configs/local-robot-seed.json
```

几何变化后重新计算并审核模型。XML 必须包含真实碰撞几何；哈希确认选定模型的身份，不能证明其与实物一致。MuJoCo 以数值方式提供 FK/IK 和碰撞检查，不打开查看器。RViz 是唯一的图形轨迹审核步骤。

## 连接头部 D455 RGB-D 相机

实机头部/高位相机为 **Intel RealSense D455**，运行时名称为 `cam_high`，并通过 `/camera/top/...` 话题提供数据。真实采集适配器当前支持 **640×480 彩色与深度图像**。它使用配置中的出厂/实测深度内参和深度到彩色变换对齐深度，再同步去除 RGB 与对齐深度的畸变。保留正确的米制深度尺度，并针对这台实物 D455 验证对齐。

使用 `camera.backend="ros"` 时，配置 `ros_master_uri`、可被机器人访问的工作站 `ros_ip`，以及彩色、深度和关节状态话题。模板中的话题为 `/camera/top/color/image_raw/compressed`、`/camera/top/depth/image_rect_raw` 和 `/joint_states`。使用 `camera.backend="bridge"` 时，配置 `bridge_host`、`bridge_path`，如需令牌则用 `token_env` 指定保存令牌的环境变量名。采用已部署 bridge 的实际路由及 TLS 设置；客户端默认值为 `127.0.0.1:18443` 和 `/bridge/ws`。

同步机器人和工作站的系统时钟。采集使用传感器时间戳，拒绝超过 `camera.max_frame_age_s` 的帧，并要求头部反馈满足配置的时间偏差限制。头姿偏离 `calibration.head_tolerance_rad` 时会拒绝采集；固定头姿标定不会自动跟随头部运动。

复用设备已安装的 ROS Noetic 环境。Noetic 面向 Ubuntu 20.04 和系统 Python 3.8，参见 [ROS REP 3](https://github.com/ros-infrastructure/rep/blob/master/rep-0003.rst#noetic-ninjemys-may-2020---may-2025)。本应用在独立 Python 3.10 环境中使用 Noetic 的 Python 消息和 `rospy`，而 `roscore` 等已安装的可执行程序继续使用系统 Python。安装脚本会提供 Python 3.10 的辅助依赖，包括 YAML 和 `netifaces`。无需连接机器人即可验证该组合：

```bash
source /opt/ros/noetic/setup.bash
command -v roscore rosrun rviz
.venv/bin/python -c "import yaml, netifaces, rospkg, defusedxml, rospy, rosgraph, genpy, message_filters; from sensor_msgs.msg import Image, CompressedImage, JointState; from geometry_msgs.msg import Point; from visualization_msgs.msg import Marker, MarkerArray; print('ROS imports OK')"
```

如果当前机器人 URDF 依赖 DC 现有工作空间中的软件包，再加载 `/home/dc/test_ws/devel/setup.bash`；其他设备使用其实际工作空间路径。辅助依赖不会安装 `rospy`、ROS 消息、`roscore`、`robot_state_publisher` 或 RViz，这些组件来自已安装的 ROS 环境。Bridge 采集不需要本地 ROS，但 RViz 仍然需要。

相机适配器直接使用 NumPy/OpenCV 解码图像，不使用 `cv_bridge`。设备上已安装的 `cv_bridge` 二进制链接到 Python 3.8，因此不应作为 Python 3.10 的依赖。不要通过给项目 `PYTHONPATH` 添加 `/usr/lib/python3/dist-packages` 来混入 Python 3.8 系统软件包。导入成功只验证软件兼容性，相机采集和真实执行仍需完成后续检查。

## 连接 FoundationPose 和 SAM

单独运行已有 GPU 服务。用 `vision.pose_endpoint` 配置 FoundationPose，通常为 `tcp://127.0.0.1:5557`；用 `vision.mask_endpoint` 配置 SAM，通常为 `tcp://127.0.0.1:5560`。服务位于远程主机时转发这两个端口。不使用实时位姿 relay，也不需要 `5558` 端口。

GPU 权重和物体 mesh 属于外部资源。`vision.mesh_id` 必须标识已部署的 mesh，其米制尺度和原点需与物体配置一致。`scene.object_radius_m` 必须以估计的物体原点为中心，保守地包围整个 mesh；规划器使用该球体检查物体间隙。按同一坐标约定核实桌面、TCP 和接近几何。物体四元数参与目标选择，不会直接复制给腕部。

## 启动真实观测和规划

完成部署环境安装、`sp_vision` 标定、独立验证并生成已验收 profile 后，在配置好的相机环境中启动 operator：

```bash
.venv/bin/tron2-deploy operator \
  --profile configs/local-robot-calibrated.json \
  --host 127.0.0.1 --port 8787
```

页面按采集 → 框选 mask → FoundationPose → 左/右预抓取规划 → 保存计划和 RViz 审核排列。启动时只读取配置。采集访问相机；规划读取当前机器人反馈并检查路径，两者都不发送运动命令。服务绑定回环地址，远程浏览器访问使用 SSH 隧道。

浏览器执行仅在 `--mock` 模式下可用。**真实执行必须显式调用 `tron2-deploy execute --real` CLI**，并按[预抓取](pregrasp.zh-CN.md)和[操作与停止](operation.zh-CN.md)中的说明提供已审核计划 ID 及监护确认。现场安装、标定、当前模型和保持行为通过验收前，保持执行禁用。

## 准备 RViz 并交接计划

在兼容的 ROS 环境中，使用保存计划对应的原始配置进行预览：

```bash
.venv/bin/python -m tron2_deployment.rviz \
  --profile configs/local-robot-calibrated.json \
  --plan /absolute/path/to/pregrasp-plan.json \
  --ros-master-uri http://127.0.0.1:11331 --once
```

该命令按需启动独立的本地可视化 Master，发布模型/物体/腕部路径并打开 RViz。它不使用机器人/相机 Master 或 `11311` 端口。真实计划审核要求匹配的 `robot.urdf`；模拟 demo 没有 URDF 时仍可显示标记。`--once` 播放一次后保持最终预抓取姿态，Ctrl-C 关闭本次预览启动的进程。

继续完成[预抓取规划与执行](pregrasp.zh-CN.md)，将已验收配置、固定观测、保存计划、RViz 审核和实测执行日志放在一起。标定、模型或配置变化后，重新观测、规划并审核。
