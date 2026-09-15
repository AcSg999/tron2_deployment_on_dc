# 部署前标定

[English](calibration.md) · [README](../README.zh-CN.md) · 下一步：[部署](deployment.zh-CN.md)

首条流程使用顶部 RGB-D 相机，并将头部保持在经过测量的固定姿态。先标定彩色相机，求解其到 `base_Link` 的变换，再独立验证该变换，之后才能准备预抓取目标。记录左右腕部坐标系及各自的 TCP 安装变换。此流程不发送夹爪指令。

通过 CLI 启动 `calibration-guide`，再按下文步骤，在浏览器页面中预览标定板、采集样本并求解。页面显示采集进度、简洁结果和下一步操作。配置更新与独立验证仍沿用现有 CLI 流程。

## 准备配置与测量数据

按照[部署文档](deployment.zh-CN.md)将 `configs/robot.example.json` 复制为 `configs/local-robot-seed.json`，然后替换所有 null 和 `REPLACE` 值。采集前必须填写实际相机标识、图像尺寸、初始彩色内参与畸变、深度内参、深度到彩色的变换、相机传输方式、实测头部位置、实际机器人模型与关节映射，以及桌面高度。当前实机 RGB-D 流程使用 640 × 480 图像。相机出厂参数可用于启动采集，但不能作为标定通过的证据。保持 `calibration.verified`、`execution.allow_real` 和 `execution.hold_behavior_verified` 为 false。模板中的单位相机到基座变换只是未经验证的采集占位值，不能用于运动规划。

模型必须描述实际安装的腕部与被动附件。当前适配器只映射 14 个手臂关节和两个头部关节；其他关节必须固定，同时保留其碰撞几何。同步机器人、相机和主机时钟。采集代码会拒绝过期图像，以及缺失或不一致的头部反馈。

测量棋盘格方格边长，并统计**内角点**数量。示例使用 `9x6` 个内角点、方格边长 `0.025` m 的标定板；这两个参数都应改为实际数值。采集手眼样本时，将标定板刚性固定在所选腕部，同一组样本内不得改变安装关系。通过机器人另外经过审核的定位界面调整手臂姿态，待其稳定后逐次采集。采集程序本身只读取相机和关节状态。

## 通过可视化引导拟合彩色内参

在已配置相机环境的终端中，从仓库根目录启动辅助页。使用不存在或为空的会话目录：

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-seed.json --stage intrinsics \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/intrinsics-guided
```

打开终端显示的地址，通常为 `http://127.0.0.1:8790`。点击 **查看画面** 检查标定板是否可见、角点是否检测正确，再点击 **重新采集并保存样本** 取得新的合格视角。移动标定板，使其覆盖图像各区域，并改变距离和倾角；保持相机分辨率和头部姿态不变。辅助页将原始、未去畸变图像保存在 `view-*/color.png`。采集时会重新读取相机，不会保存可能已经过时的预览帧。

至少采集五个合格视角后，点击 **计算结果**。查看显示的 RMS、结果路径和下一步提示。达到最少样本数或拟合误差较小都不能单独证明精度；避免重复采集集中于画面中心的相似视角。未识别到棋盘时，应显示完整棋盘、减少模糊或反光，并核对内角点数量。视角过于相似时，先改变棋盘位置或倾斜角度，再重新采集。

求解后按 Ctrl-C 停止辅助页，再应用已保存的内参结果：

```bash
.venv/bin/tron2-deploy apply-intrinsics \
  --profile configs/local-robot-seed.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --output configs/local-robot-intrinsics.json
```

应用拟合结果会更新 `K`、畸变和图像尺寸，使已有外参验收失效，并保持实机执行关闭。深度内参和深度到彩色的对齐变换仍需单独提供测量结果；此棋盘格内参拟合不会标定它们。

## 通过可视化引导采集静止手眼样本

对于固定的顶部相机，当前支持 **eye-to-hand（眼在手外）** 求解：相机和头部保持固定，标定板刚性安装在所选腕部，并随腕部一起运动。求解结果将相机坐标映射到 `base_Link`。使用更新后的内参配置重新启动辅助页，并明确指定一侧：

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-intrinsics.json --stage handeye --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left-guided
```

打开终端显示的地址，通常为 `http://127.0.0.1:8790`。按以下顺序循环采集：

1. 点击 **查看画面**，确认完整棋盘及检测到的角点都清晰可见。
2. 通过机器人另外经过审核的控制界面调整腕部。改变旋转轴和位置，同时保持棋盘可见、头部固定。
3. 等待手臂和棋盘静止，再点击 **重新采集并保存样本**。查看已保存样本数和显示的腕部转角变化，然后重复采集。

辅助页提供采集引导，腕部定位仍由用户手动控制，不会生成下一个腕部目标位姿。至少保存五个合格样本，且至少一个腕部姿态相对首个样本的旋转差达到 15 度时，才能点击 **计算结果**。这只是求解器的最低要求，不代表精度验证通过，也不要求每一步都旋转 15 度。应绕不同轴改变姿态，并保持标定板与腕部的安装关系不变。

采集器在图像前后读取新鲜关节反馈，并拒绝手臂或头部移动、时间戳偏差过大，以及仅依赖头部稳定性的同步回退。所有样本必须来自同一侧、同一相机、同一标定板、同一数据来源，并保持相同的静止头部姿态。失败的采集不会增加合格样本数；根据页面提示修正问题后重新采集。

准备好后点击 **计算结果**。样本保存在 `calibration_data/handeye-left-guided/samples/*.json`，单独的结果文件为 `calibration_data/handeye-left-guided/handeye-left.json`。完成下面的独立验证前，结果仍处于未验证状态。按 Ctrl-C 停止辅助页。

采集右腕样本时，使用 `--side right` 和独立的 `handeye-right-guided` 目录。左右样本集应分别求解，不要混合。两次求得的相机到基座变换应在实测误差预算内一致。样本字段 `robot_gripper_to_base` 虽然沿用了旧名称，实际保存的是所选**腕部**变换。

## 使用保留点验证并应用结果

测量至少三个未参与求解且不共线的点。每个点都需要记录以米为单位的相机坐标，以及独立测量的 `base_Link` 坐标。不要使用待验证的变换生成“期望”基座坐标。

将这些实测数据保存到 `calibration_data/heldout_points.json`，使用 `points_camera` 和 `points_base` 两个字段，均为大小相同的 `N x 3` 数组。转换为所需的 NPZ 格式：

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

以下示例允许最大点误差为 5 mm。运行前，应依据部署的测量与间隙预算确定实际阈值。`apply-calibration` 会检查独立测量点，只有通过后才写入已验收配置：

```bash
.venv/bin/tron2-deploy apply-calibration \
  --profile configs/local-robot-intrinsics.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --handeye calibration_data/handeye-left-guided/handeye-left.json \
  --validation-points calibration_data/heldout_points.npz \
  --max-error-m 0.005 --calibration-id top-camera-fixed-head-v1 \
  --output configs/local-robot-calibrated.json
```

此命令要求手眼结果来自实机 eye-to-hand 求解、相机标识一致，且通过保留点验证。它记录相机到基座的变换、固定头部姿态、标定 ID 和误差报告，然后设置 `calibration.verified=true`。它会明确保持 `execution.allow_real=false`。

<details>
<summary>替代方式：完全通过 CLI 采集与求解</summary>

原有命令仍可用于脚本化采集。请使用与引导会话分开的目录。内参采集时，每个视角使用新的 `view-XX` 目录重复运行采集命令，然后求解：

```bash
.venv/bin/tron2-deploy capture \
  --profile configs/local-robot-seed.json --raw \
  --output calibration_data/intrinsics-cli/view-01

.venv/bin/tron2-deploy intrinsics \
  --images 'calibration_data/intrinsics-cli/view-*/color.png' \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/intrinsics-cli/intrinsics.json
```

手眼采集前，先用 `apply-intrinsics` 应用上述内参文件。随后每次调整手臂姿态后重复运行 `record-sample`，最后求解：

```bash
.venv/bin/tron2-deploy record-sample \
  --profile configs/local-robot-intrinsics.json --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left-cli/samples

.venv/bin/tron2-deploy handeye \
  --samples 'calibration_data/handeye-left-cli/samples/*.json' \
  --output calibration_data/handeye-left-cli/handeye-left.json
```

如果采用此方式，后续验证与应用步骤应改用这些 CLI 结果路径。

</details>

## 记录腕部几何并保留证据

分别记录 `pregrasp.<side>.wrist_to_tcp_pose7`，格式为 `[x,y,z,qw,qx,qy,qz]`：表示 TCP 坐标系在配置的腕部刚体坐标系中的位姿，平移单位为米，四元数必须归一化。该变换应来自实际安装几何和独立测量。TCP `+Z` 是朝向物体的接近轴，TCP `+Y` 是目标选择时使用的滚转/向上参考。只有两个实际坐标系重合时，单位变换才有效。

在 `base_Link` 中测量 `scene.table_z_m`；设置 `scene.object_radius_m`，使其以物体位姿原点为中心包住整个已注册物体网格。保留会话目录、原始图像、样本文件、可用时的左右两次求解、保留点测量、安装测量和已接受的模型版本。拟合结果异常或验证点检查失败时，可运行 `.venv/bin/tron2-deploy calibration-report --help` 查看可选诊断报告的输入参数，并将生成的报告与输入数据一起保存。辅助页和报告不会应用标定或启用执行。外参拟合通过不代表碰撞几何、腕部安装、传输时序或停止行为已经验证。

头部偏离标定姿态后，此固定头部流程将失效。应恢复到该实测姿态或重新标定；服务不会外推相机与头部的运动学链。接下来完成[部署](deployment.zh-CN.md)，再进行[预抓取规划](pregrasp.zh-CN.md)。
