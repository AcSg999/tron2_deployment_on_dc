# 部署前标定

[English](calibration.md) · [README](../README.zh-CN.md) · 下一步：[部署](deployment.zh-CN.md)

首条流程使用顶部 RGB-D 相机，并将头部保持在经过测量的固定姿态。先标定彩色相机，求解其到 `base_Link` 的变换，再独立验证该变换，之后才能准备预抓取目标。记录左右腕部坐标系及各自的 TCP 安装变换。此流程不发送夹爪指令。

通过 CLI 启动 `calibration-guide`，再按下文步骤，在浏览器页面中预览标定板、采集样本并求解。页面显示采集进度、简洁结果和下一步操作。配置更新与独立验证仍沿用现有 CLI 流程。

## 准备配置与测量数据

按照[部署文档](deployment.zh-CN.md)将 `configs/robot.example.json` 复制为 `configs/local-robot-seed.json`，然后替换所有 null 和 `REPLACE` 值。采集前必须填写实际相机标识、图像尺寸、初始彩色内参与畸变、深度内参、深度到彩色的变换、相机传输方式、实测头部位置、实际机器人模型与关节映射，以及桌面高度。当前实机 RGB-D 流程使用 640 × 480 图像。相机出厂参数可用于启动采集，但不能作为标定通过的证据。保持 `calibration.verified`、`execution.allow_real` 和 `execution.hold_behavior_verified` 为 false。模板中的单位相机到基座变换只是未经验证的采集占位值，不能用于运动规划。

模型必须描述实际安装的腕部与被动附件。当前适配器只映射 14 个手臂关节和两个头部关节；其他关节必须固定，同时保留其碰撞几何。同步机器人、相机和主机时钟。采集代码会拒绝过期图像，以及缺失或不一致的头部反馈。

### 读取固定头部姿态

在当前 TRON2 部署中，`/joint_states` 是开发主机上的 ROS 2 Foxy 话题，不是其 ROS 1 Noetic Master 上的话题。因此，对 `10.192.1.4:11311` 执行 ROS 1 `rostopic list` 看不到它。登录开发主机、加载 Foxy，并按关节名称匹配位置，不要依赖固定数组下标：

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

`BatchMode=yes` 强制 SSH 仅使用已有公钥，不会提示输入密码；以上整个代码块都在开发主机上执行。在 DC 的 `dc` 用户环境中已验证该免密登录可用。如果命令直接报告 `Permission denied (publickey)`，先运行 `whoami`，确认正在 DC 上以配置了该公钥的用户执行，而不是输入或保存未知密码。

将输出的 `[head_pitch_Joint, head_yaw_Joint]` 值按此顺序填入 `calibration.head_q2`，单位为弧度。先把头部放到整个内参与手眼采集期间都将保持不动的姿态，再读取这些值。头部移动后必须重新读取；不能因为外观看起来水平就假定它是 `[0, 0]`。

### 确认物体半径对应的对象

`scene.object_radius_m` 属于 `vision.mesh_id` 所跟踪的任务物体——例如注册给 FoundationPose 的 mesh 是碗时，它就是碗的半径。它不是机器人、夹爪、相机、标定板或桌子的半径。该值以米为单位，表示以已注册 mesh 原点为球心、在应用实际部署的 mesh 缩放后能包住所有 mesh 顶点的保守球半径：

```text
scene.object_radius_m >= max(norm(vertex_m - mesh_origin_m))
```

如果 mesh 原点偏离物体中心，所需球半径可能明显大于物体宽度的一半。这个值用于碰撞包络，不同于 `pregrasp.<side>.radius_m`；后者用于选择轴对称物体表面上的接近锚点。应将 mesh ID、缩放、原点约定和计算出的包围半径一起记录；其中任何一项变化都会使该值失效。

测量棋盘格方格边长，并统计**内角点**数量。下方内参命令使用当前 `7x9` 个内角点、方格边长 `0.020` m 的标定板；手眼示例仍使用 `9x6` 个内角点、方格边长 `0.025` m 的标定板。所有参数都必须与实际标定板一致。采集手眼样本时，将标定板刚性固定在所选腕部，同一组样本内不得改变安装关系。通过机器人另外经过审核的定位界面调整手臂姿态，待其稳定后逐次采集。采集程序本身只读取相机和关节状态。手眼采集也可改用 ArUco 目标，而不必把棋盘固定在腕上；见手眼采集一节中的 ArUco 小节。

## 通过可视化引导拟合彩色内参

在已配置相机环境的终端中，从仓库根目录启动辅助页。使用不存在或为空的会话目录：

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-seed.json --stage intrinsics \
  --pattern 7x9 --square-m 0.020 \
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

### 改用 ArUco 目标替代棋盘

腕上固定棋盘体积大。固定相机的眼在手外求解也可以使用 ArUco 目标，且其几何参数不写死在代码里：由一个 JSON 配置给出，通过 `--target aruco --target-spec` 传入。内参标定仍使用棋盘。

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-intrinsics.json --stage handeye --side left \
  --target aruco --target-spec configs/local-aruco-target.json \
  --output calibration_data/handeye-left-guided
```

`configs/aruco-marker.example.json` 是单码示例，`configs/aruco-board.example.json` 是刚性集成板示例。复制其中一个为本地文件（`configs/local*.json` 不纳入版本管理），并替换所有实测数值。

| 字段 | 适用 | 含义 |
| --- | --- | --- |
| `kind` | 两者 | `marker` 表示单个标记，`board` 表示刚性多标记布局 |
| `dictionary` | 两者 | `cv2.aruco` 字典名，如 `DICT_6X6_250` |
| `marker_length_m` | 两者 | 印刷黑方块边长（米），不含白边 |
| `marker_id` | `marker` | 单个标记的印刷 ID |
| `markers_x`、`markers_y` | `board` | 布局每行、每列的标记数 |
| `marker_separation_m` | `board` | 相邻黑方块之间的间隙，不是中心距 |
| `first_marker_id` | `board` | ID 按行主序连续时的起始 ID |
| `marker_ids` | `board` | 显式的行主序 ID 列表；ID 不连续时用它替代 `first_marker_id` |
| `frame_marker_id` | `board` | 以其印刷左上角作为目标坐标系原点的标记 |
| `min_visible_markers` | `board` | 一帧至少需要可见的标记数，默认 2 |
| `min_solution_ratio` | 两者 | 另一个平面解的重投影误差至少要差多少倍，默认 2.0 |

目标坐标系取 `frame_marker_id` 的标准 ArUco 坐标系：原点在该标记印刷左上角，+x 沿印刷右边缘，+y 沿印刷下边缘，+z 指向印刷平面内部。你只需测量印刷几何。标记到腕部的安装偏置在眼在手外求解中会抵消，无需测量，但同一组样本内不得改变。

平面目标总存在第二个位姿解。只有当另一个解的重投影误差至少差 `min_solution_ratio` 倍时该帧才会被接受；因此尺寸偏小或接近正对相机的目标会被拒绝，而不是以静默错误的倾角通过。实际影响：优先使用集成板而不是单个小标记，把目标保持在顶部相机约半米以内，并让多个标记同时可见。单个 5 cm 标记在 1 m 处即使被检测到，精度也不足。

CLI 采集使用相同参数，例如 `record-sample --side left --target aruco --target-spec configs/local-aruco-target.json`。其余采集与验收规则不变：同样要求五个样本、15° 转角跨度、静止、同步与独立保留点验证；样本文件会记录目标配置，之后任何修改都会使比较失效。

## 使用保留点验证并应用结果

保留点就是没有参与求解的点：标定算完之后，你另外量几个点来检验结果。前面显示的手眼残差只反映样本一致性，不代表精度，所以这些点是唯一的独立证据。它们由你手工测量，程序只消费这两组数组。

在桌面或工装上选至少三个固定的点，要求顶部相机看得到、机械臂也够得到，例如贴纸十字、工装尖点、画了标记的角点。每个点测两次——一次得到相机坐标，一次得到 `base_Link` 坐标——两组数组顺序必须一一对应，并且**绝不能用待验证的变换反算 `points_base`**。

### 相机坐标

先拍一帧已去畸变的 RGB-D，并和测量数据放在一起：

```bash
.venv/bin/tron2-deploy capture \
  --profile configs/local-robot-intrinsics.json \
  --output calibration_data/heldout/frame
```

该命令写出 `color.png`（已去畸变）、`depth.png`（16 位，单位 mm，已对齐到彩色）和 `frame.json`。在 `color.png` 上读出每个点的像素，填入下面的列表并反投影：

```bash
.venv/bin/python - <<'PY'
import json
import cv2
import numpy as np
pixels = [(320, 240), (180, 300), (470, 210)]   # 换成你读到的像素
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

要用这一帧已去畸变的采集，而不是内参用的 `--raw` 视角：`depth.png` 已经对齐到去畸变彩色，所以反投影只需要 `K`。

### 基座坐标

对同样的点，通过机器人自身经过审核的控制界面移动机械臂，让与所配置 TCP 坐标系重合的尖点顶到该点，然后读取关节：

```bash
.venv/bin/tron2-deploy state \
  --profile configs/local-robot-intrinsics.json \
  --output calibration_data/heldout/state-01.json
```

把这次读数换算成 `base_Link` 中被顶到的那个点：

```bash
.venv/bin/python - <<'PY'
import json
from tron2_deployment.config import load_profile
from tron2_deployment.geometry import pose_matrix
from tron2_deployment.kinematics import RobotModel
profile = load_profile('configs/local-robot-intrinsics.json')
state = json.load(open('calibration_data/heldout/state-01.json'))
side = 'left'   # 顶到该点的腕部
model = RobotModel(profile)
model.set_state(state['arm_q14'], state['head_q2'])
touched = pose_matrix(model.wrist_poses()[side]) @ pose_matrix(profile['pregrasp'][side]['wrist_to_tcp_pose7'])
print(touched[:3, 3].tolist())
PY
```

每个点重复 `state` 与换算，`side` 保持一致。如果实际末端没有与所配置 TCP 原点重合的尖点，先临时装一个，或先修正 `pregrasp.<side>.wrist_to_tcp_pose7`；否则你顶到的点并不是你测的那个点。

这些点要分散：至少三个、不共线，并在深度、高度和水平位置上都有变化，因为该检查取最坏值而不是平均值。阈值按深度传感器的实际精度和部署间隙预算确定；下文示例中的 `0.005` 是 5 mm，可能比 RGB-D 在整个工作空间内能支持的还要紧。

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
