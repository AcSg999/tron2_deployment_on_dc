# Ubuntu 22.04 部署

[English](deployment.md) · [工作流总览](../README.zh-CN.md) · [上一步：标定](calibration.zh-CN.md) · [下一步：预抓取](pregrasp.zh-CN.md)

在原生 Ubuntu 22.04 上使用 Python 3.10 部署。采集真实标定数据前，先安装此环境。当前任务是**考虑物体几何的腕部预抓取**：接近经过审核的退让位姿并停止。应用不发送夹爪命令，不执行接触或抓取动作。

## 安装独立软件包

```bash
sudo apt update
sudo apt install -y python3.10-venv git libgl1 libglib2.0-0
git clone https://github.com/Shukashuki/tron2_deployment_on_dc.git
cd tron2_deployment_on_dc
bash scripts/install.sh
.venv/bin/python -m pip check
.venv/bin/tron2-deploy --help
```

`scripts/install.sh` 创建 `.venv`，安装本软件包及其 `bridge`、`dev` 可选依赖，并安装 `third_party/tron2_env` 中经过修补的运行时。两个软件包都使用 **`opencv-contrib-python` 作为唯一的 `cv2` 提供者**。此环境应与安装了 `opencv-python` 或任一种 headless OpenCV wheel 的环境分开。不需要 `dexpipe` 工作副本、Gaia20 SDK、RL 流水线或重定向软件包。

安装脚本使用 `constraints-ubuntu22-py310.txt`，记录在独立 Linux/Python 3.10 环境中验证过的依赖版本。更新版本后需重新运行测试和模拟工作流；MuJoCo 版本变化也可能改变编译模型哈希，需要重新审核模型和规划。

使用合成输入验证软件：

```bash
.venv/bin/python -m pytest -q
.venv/bin/tron2-deploy demo --output output/demo
.venv/bin/tron2-deploy operator --profile configs/demo.json --mock
```

打开 `http://127.0.0.1:8787`。模拟配置使用简化模型、合成 RGB-D 和模拟反馈；其结果是软件验证证据，不是真机部署验收记录。在同一端口启动另一个 operator 前，先用 Ctrl-C 停止服务。

## 准备现场配置

```bash
cp configs/robot.example.json configs/local-robot-seed.json
```

示例有意保留 `null` 和 `REPLACE-...` 占位值，填写必需字段后才能加载。`configs/local*.json` 已被 Git 忽略。使用实际安装信息以及可用的出厂/实测相机标定填写初始配置，保持 `calibration.verified=false` 和 `execution.allow_real=false`。完成[标定](calibration.zh-CN.md)，生成 `configs/local-robot-calibrated.json`；复制模板或填写出厂参数本身不能独立验证相机到基座变换。

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

## 连接高位 RGB-D 相机

真实采集适配器当前支持 **640×480 彩色与深度图像**。它使用配置中的出厂/实测深度内参和深度到彩色变换对齐深度，再同步去除 RGB 与对齐深度的畸变。保留正确的米制深度尺度，并针对选定实物相机验证对齐。

使用 `camera.backend="ros"` 时，配置 `ros_master_uri`、可被机器人访问的工作站 `ros_ip`，以及彩色、深度和关节状态话题。模板中的话题为 `/camera/top/color/image_raw/compressed`、`/camera/top/depth/image_rect_raw` 和 `/joint_states`。使用 `camera.backend="bridge"` 时，配置 `bridge_host`、`bridge_path`，如需令牌则用 `token_env` 指定保存令牌的环境变量名。采用已部署 bridge 的实际路由及 TLS 设置；客户端默认值为 `127.0.0.1:18443` 和 `/bridge/ws`。

同步机器人和工作站的系统时钟。采集使用传感器时间戳，拒绝超过 `camera.max_frame_age_s` 的帧，并要求头部反馈满足配置的时间偏差限制。头姿偏离 `calibration.head_tolerance_rad` 时会拒绝采集；固定头姿标定不会自动跟随头部运动。

ROS 采集和 RViz 需要已有、经过测试且与所选 Python 运行时兼容的 ROS 1 环境。此安装脚本不会安装 ROS Noetic，仅安装原生 Ubuntu 22.04 也不会提供该环境。安装 Python 辅助依赖并加载已测试工作空间，替换下方路径：

```bash
.venv/bin/python -m pip install '.[ros]'
source /absolute/path/to/tested_ros1_workspace/devel/setup.bash
.venv/bin/python -c "import rospy; import sensor_msgs.msg; import geometry_msgs.msg; import visualization_msgs.msg"
```

辅助依赖不会安装 `rospy`、ROS 消息、`roscore`、`robot_state_publisher` 或 RViz；需确认这些组件来自兼容的 ROS 环境。Bridge 采集不需要本地 ROS，但 RViz 仍然需要。

## 连接 FoundationPose 和 SAM

单独运行已有 GPU 服务。用 `vision.pose_endpoint` 配置 FoundationPose，通常为 `tcp://127.0.0.1:5557`；用 `vision.mask_endpoint` 配置 SAM，通常为 `tcp://127.0.0.1:5560`。服务位于远程主机时转发这两个端口。不使用实时位姿 relay，也不需要 `5558` 端口。

GPU 权重和物体 mesh 属于外部资源。`vision.mesh_id` 必须标识已部署的 mesh，其米制尺度和原点需与物体配置一致。`scene.object_radius_m` 必须以估计的物体原点为中心，保守地包围整个 mesh；规划器使用该球体检查物体间隙。按同一坐标约定核实桌面、TCP 和接近几何。物体四元数参与目标选择，不会直接复制给腕部。

## 启动真实观测和规划

完成标定后，在配置好的相机环境中启动 operator：

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
