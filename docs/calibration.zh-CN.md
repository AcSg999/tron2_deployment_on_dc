# 部署前标定

[English](calibration.md) · [README](../README.zh-CN.md) · 下一步：[部署](deployment.zh-CN.md)

首条流程使用顶部 RGB-D 相机，并将头部保持在经过测量的固定姿态。先标定彩色相机，求解其到 `base_Link` 的变换，再独立验证该变换，之后才能准备预抓取目标。记录左右腕部坐标系及各自的 TCP 安装变换。此流程不发送夹爪指令。

标定继续通过 CLI 完成。内参拟合与手眼求解后，生成[离线可视化报告](calibration_visualization.zh-CN.md)，检查后再应用结果。报告可直接在浏览器中打开，无需启动操作服务或 ROS。

## 准备配置与测量数据

按照[部署文档](deployment.zh-CN.md)将 `configs/robot.example.json` 复制为 `configs/local-robot-seed.json`，然后替换所有 null 和 `REPLACE` 值。采集前必须填写实际相机标识、图像尺寸、初始彩色内参与畸变、深度内参、深度到彩色的变换、相机传输方式、实测头部位置、实际机器人模型与关节映射，以及桌面高度。当前实机 RGB-D 流程使用 640 × 480 图像。相机出厂参数可用于启动采集，但不能作为标定通过的证据。保持 `calibration.verified`、`execution.allow_real` 和 `execution.hold_behavior_verified` 为 false。模板中的单位相机到基座变换只是未经验证的采集占位值，不能用于运动规划。

模型必须描述实际安装的腕部与被动附件。当前适配器只映射 14 个手臂关节和两个头部关节；其他关节必须固定，同时保留其碰撞几何。同步机器人、相机和主机时钟。采集代码会拒绝过期图像，以及缺失或不一致的头部反馈。

测量棋盘格方格边长，并统计**内角点**数量。示例使用 `9x6` 个内角点、方格边长 `0.025` m 的标定板；这两个参数都应改为实际数值。采集手眼样本时，将标定板刚性固定在所选腕部，同一组样本内不得改变安装关系。通过机器人另外经过审核的定位界面调整手臂姿态，待其稳定后逐次采集。采集程序本身只读取相机和关节状态。

## 使用原始图像拟合彩色内参

改变标定板距离和倾角，使其覆盖图像各区域。保持相机分辨率和头部姿态不变。每个视角单独运行采集命令，并将 `view-01` 改为新的目录：

```bash
python -m tron2_deployment.cli capture \
  --profile configs/local-robot-seed.json --raw \
  --output calibration_data/intrinsics/view-01
```

`--raw` 保存尚未去畸变的彩色图像，用于内参拟合。每个目录包含 `color.png`、已对齐的 `depth.png` 和 `frame.json`。求解器要求至少五个成功检测到棋盘格且分辨率一致的视角；应检查覆盖范围和返回的 `rms_px`，不能仅凭达到最少样本数判定通过。

```bash
python -m tron2_deployment.cli intrinsics \
  --images 'calibration_data/intrinsics/view-*/color.png' \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/intrinsics.json

python -m tron2_deployment.cli calibration-report \
  --intrinsics calibration_data/intrinsics.json \
  --images 'calibration_data/intrinsics/view-*/color.png' \
  --output output/intrinsics-review

xdg-open output/intrinsics-review/index.html
```

检查检测角点与重投影叠加图、原始与去畸变图像对比、覆盖范围，以及逐视角像素误差。继续前先排查异常视角和覆盖不足的问题。这些拟合诊断使用标定视角，不能证明独立精度。每次报告的输出目录必须不存在或为空。若内参 JSON 保存的图像路径仍可解析，可以省略 `--images`。

```bash
python -m tron2_deployment.cli apply-intrinsics \
  --profile configs/local-robot-seed.json \
  --intrinsics calibration_data/intrinsics.json \
  --output configs/local-robot-intrinsics.json
```

应用拟合结果会更新 `K`、畸变和图像尺寸，使已有外参验收失效，并保持实机执行关闭。深度内参和深度到彩色的对齐变换仍需单独提供测量结果；此棋盘格内参拟合不会标定它们。

## 采集静止状态下的手眼样本

对于固定的顶部相机，当前支持 **eye-to-hand（眼在手外）** 求解。必须明确选择安装标定板的一侧。每次通过另外经过审核的方式调整手臂姿态后，重复运行以下命令；程序会自动创建带唯一时间戳的图像和样本 JSON 文件：

```bash
python -m tron2_deployment.cli record-sample \
  --profile configs/local-robot-intrinsics.json --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left
```

至少需要五个合格样本，且腕部旋转跨度至少为 15 度。应改变旋转方向和平移位置。采集器在图像前后读取新鲜关节反馈，并拒绝手臂或头部移动、时间戳偏差过大，以及仅依赖头部稳定性的同步回退。一次求解的所有样本必须来自同一侧、同一相机、同一标定板、同一数据来源，并保持相同的静止头部姿态。

```bash
python -m tron2_deployment.cli handeye \
  --samples 'calibration_data/handeye-left/*.json' \
  --output calibration_data/handeye-left.json
```

采集右腕样本时，使用 `--side right` 和独立的 `handeye-right` 路径。左右样本集应分别求解，不要混合。两次求得的相机到基座变换应在实测误差预算内一致。样本字段 `robot_gripper_to_base` 虽然沿用了旧名称，实际保存的是所选**腕部**变换。

## 使用保留点验证并应用结果

测量至少三个未参与求解且不共线的点。每个点都需要记录以米为单位的相机坐标，以及独立测量的 `base_Link` 坐标。不要使用待验证的变换生成“期望”基座坐标。

将这些实测数据保存到 `calibration_data/heldout_points.json`，使用 `points_camera` 和 `points_base` 两个字段，均为大小相同的 `N x 3` 数组。转换为所需的 NPZ 格式：

```bash
python - <<'PY'
import json
import numpy as np
from pathlib import Path
points = json.loads(Path('calibration_data/heldout_points.json').read_text())
np.savez('calibration_data/heldout_points.npz',
         points_camera=np.asarray(points['points_camera'], dtype=float),
         points_base=np.asarray(points['points_base'], dtype=float))
PY
```

以下示例允许最大点误差为 5 mm。运行前，应依据部署的测量与间隙预算确定实际阈值。先生成报告：

```bash
python -m tron2_deployment.cli calibration-report \
  --handeye calibration_data/handeye-left.json \
  --samples 'calibration_data/handeye-left/*.json' \
  --validation-points calibration_data/heldout_points.npz \
  --max-error-m 0.005 \
  --output output/handeye-review

xdg-open output/handeye-review/index.html
```

检查腕部姿态覆盖与标定板在腕部坐标系中的一致性，再将变换后的保留点与独立测得的基座系参考坐标比较。报告显示残差向量，以及逐点误差与所选阈值的对比。保留点验证失败时仍会生成报告，同时 CLI 以状态码 1 退出。没有保留点测量时，报告维持未验证状态；仅凭很小的拟合残差不能证明精度。判读方法见[可视化指南](calibration_visualization.zh-CN.md)。

检查独立验证通过的结果后，使用相同测量数据和阈值应用标定：

```bash
python -m tron2_deployment.cli apply-calibration \
  --profile configs/local-robot-intrinsics.json \
  --intrinsics calibration_data/intrinsics.json \
  --handeye calibration_data/handeye-left.json \
  --validation-points calibration_data/heldout_points.npz \
  --max-error-m 0.005 --calibration-id top-camera-fixed-head-v1 \
  --output configs/local-robot-calibrated.json
```

此命令要求手眼结果来自实机 eye-to-hand 求解、相机标识一致，且通过保留点验证。它记录相机到基座的变换、固定头部姿态、标定 ID 和误差报告，然后设置 `calibration.verified=true`。它会明确保持 `execution.allow_real=false`。

## 记录腕部几何并保留证据

分别记录 `pregrasp.<side>.wrist_to_tcp_pose7`，格式为 `[x,y,z,qw,qx,qy,qz]`：表示 TCP 坐标系在配置的腕部刚体坐标系中的位姿，平移单位为米，四元数必须归一化。该变换应来自实际安装几何和独立测量。TCP `+Z` 是朝向物体的接近轴，TCP `+Y` 是目标选择时使用的滚转/向上参考。只有两个实际坐标系重合时，单位变换才有效。

在 `base_Link` 中测量 `scene.table_z_m`；设置 `scene.object_radius_m`，使其以物体位姿原点为中心包住整个已注册物体网格。保留原始图像、样本文件、可用时的左右两次求解、保留点测量、可视化报告目录及其中的 `metrics.json`、安装测量和已接受的模型版本。报告不会应用标定或启用执行。外参拟合通过不代表碰撞几何、腕部安装、传输时序或停止行为已经验证。

头部偏离标定姿态后，此固定头部流程将失效。应恢复到该实测姿态或重新标定；服务不会外推相机与头部的运动学链。接下来完成[部署](deployment.zh-CN.md)，再进行[预抓取规划](pregrasp.zh-CN.md)。
