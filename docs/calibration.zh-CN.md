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

打开终端显示的地址，通常为 `http://127.0.0.1:8790`。点击 **查看画面（不保存）** 检查标定板是否可见、角点是否检测正确，再点击 **重新采集并保存样本** 取得新的合格视角。移动标定板，使其覆盖图像各区域，并改变距离和倾角；保持相机分辨率和头部姿态不变。辅助页将原始、未去畸变图像保存在 `view-*/color.png`。采集时会重新读取相机，不会保存可能已经过时的预览帧。

至少采集五个合格视角后，点击 **计算结果**。查看显示的 RMS、结果路径和下一步提示。达到最少样本数或拟合误差较小都不能单独证明精度；避免重复采集集中于画面中心的相似视角。未识别到棋盘时，应显示完整棋盘、减少模糊或反光，并核对内角点数量。视角过于相似时，先改变棋盘位置或倾斜角度，再重新采集。

求解后按 Ctrl-C 停止辅助页，再应用已保存的内参结果：

```bash
.venv/bin/tron2-deploy apply-intrinsics \
  --profile configs/local-robot-seed.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --output configs/local-robot-intrinsics.json
```

应用拟合结果会更新 `K`、畸变和图像尺寸，使已有外参验收失效，并保持实机执行关闭。深度内参和深度到彩色的对齐变换仍需单独提供测量结果；此棋盘格内参拟合不会标定它们。

## 设置双臂负载并准备拖动示教

采集外参前需要用拖动示教改变腕部姿态。先在机器人管理页完成当前末端执行器的负载辨识，再将辨识结果写入控制器。TRON2 接口使用 `[m, mc_x, mc_y, mc_z]`：`m` 的单位为 kg，后三项是质量一阶矩，单位为 kg·m，**不能**除以质量后当作质心坐标写入。更换末端执行器或改变安装后，原参数失效，必须重新辨识。

当前机器人 `DACH_TRON2A_091` 的实测值为：

| 机械臂 | `m` (kg) | `mc_x` (kg·m) | `mc_y` (kg·m) | `mc_z` (kg·m) |
| --- | ---: | ---: | ---: | ---: |
| 左臂（`arm=0`） | 1.199574 | -0.011601 | 0.008230 | -0.158851 |
| 右臂（`arm=1`） | 1.103249 | -0.005505 | -0.003338 | -0.176561 |

退出拖动示教并安全支撑双臂，确认现场有人监护、急停可用、机器人地址和设备 ID 与下方脚本完全一致。停止其他占用控制端口的客户端，然后从仓库根目录执行。该脚本只设置并回读负载，不会进入拖动示教，也不会发送关节、底盘或夹爪运动命令：

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY \
  NO_PROXY=10.192.1.2 no_proxy=10.192.1.2 \
  .venv/bin/python - <<'PY'
import json
import time
import uuid

import websocket

url = "ws://10.192.1.2:5000"
expected_accid = "DACH_TRON2A_091"
target = [
    [1.199574, -0.011601, 0.008230, -0.158851],
    [1.103249, -0.005505, -0.003338, -0.176561],
]

ws = websocket.create_connection(url, timeout=10)


def receive(title, guid=None):
    while True:
        message = json.loads(ws.recv())
        if message.get("title") == title and (
            guid is None or message.get("guid") == guid
        ):
            return message


robot_info = receive("notify_robot_info")
accid = robot_info.get("accid")
if accid != expected_accid:
    raise SystemExit(f"wrong robot: expected {expected_accid}, received {accid}")


def request(title, data):
    guid = str(uuid.uuid4())
    ws.send(json.dumps({
        "accid": accid,
        "title": title,
        "timestamp": int(time.time() * 1000),
        "guid": guid,
        "data": data,
    }))
    return guid


for arm, values in enumerate(target):
    guid = request("request_set_payload_param", {"arm": arm, "data": values})
    result = receive("response_set_payload_param", guid).get("data", {})
    if result.get("result") != "success" or result.get("arm") != arm:
        raise SystemExit(f"arm {arm} write failed: {result}")

guid = request("request_get_payload_param", {})
result = receive("response_get_payload_param", guid).get("data", {})
actual = result.get("data")
expected = target[0] + target[1]
if result.get("result") != "success" or not isinstance(actual, list):
    raise SystemExit(f"readback failed: {result}")
if len(actual) != 8 or any(
    abs(float(got) - want) > 1e-6 for got, want in zip(actual, expected)
):
    raise SystemExit(f"readback mismatch: expected {expected}, received {actual}")
print(json.dumps({"accid": accid, "payload": actual, "verified": True}, indent=2))
ws.close()
PY
```

只有左右臂写入响应均为 `success`，且最终输出包含 `"verified": true`，才能进入拖动示教。若机器人能够 `ping` 通但 WebSocket 握手超时，先检查本机代理；上面的命令已经对 `10.192.1.2` 显式绕过代理。若回读返回 `fail_payload_get` 或数值不一致，不要启用拖动示教，应保持双臂支撑并排查控制器模式、末端安装和辨识结果。

## 通过可视化引导采集静止手眼样本

对于固定的顶部相机，当前支持 **eye-to-hand（眼在手外）** 求解：相机和头部保持固定，标定板刚性安装在所选腕部，并随腕部一起运动。求解结果将相机坐标映射到 `base_Link`。使用更新后的内参配置重新启动辅助页，并明确指定一侧：

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-intrinsics.json --stage handeye --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left-guided
```

打开终端显示的地址，通常为 `http://127.0.0.1:8790`。按以下顺序循环采集：

1. 点击 **查看画面（不保存）**，确认完整棋盘及检测到的角点都清晰可见。
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
  --profile configs/local-robot-intrinsics.json --stage handeye --side right \
  --target aruco --target-spec configs/local-aruco-target.json \
  --output calibration_data/handeye-right-guided
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

该命令会创建 `color.png`（已去畸变）、`depth.png`（16 位，单位 mm，已对齐到彩色）和 `frame.json`。
在有图形桌面的终端运行下面的命令，按顺序左键点击 `color.png` 中的保留点，按 Enter（或鼠标中键）结束；终端会打印可直接复制的 `pixels` 列表。需要放大时可先用图窗工具栏缩放，点选前退出缩放模式。

```bash
.venv/bin/python - <<'PY'
import matplotlib.pyplot as plt

image = plt.imread('calibration_data/heldout/frame/color.png')
fig, ax = plt.subplots()
ax.imshow(image, interpolation='nearest')
ax.set_title('Left-click points in order; Enter or middle-click to finish')
chosen = plt.ginput(n=-1, timeout=0)
plt.close(fig)
pixels = [(round(x), round(y)) for x, y in chosen]
print(f'pixels = {pixels}')
PY
```

将打印的 `pixels` 列表填入下面的脚本并反投影；坐标顺序为 `(u, v)`，图像左上角是 `(0, 0)`：

```bash
.venv/bin/python - <<'PY'
import json
import cv2
import numpy as np
pixels = [(167, 318), (345, 289), (420, 227)]   # 换成你读到的像素
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

先标定临时或正式安装的尖点位置。选一个不动且可重复接触的固定点，用机器人自身经过审核的控制界面，保持同一侧机械臂的**同一个尖端**每次顶到该点；换至少四种明显不同的腕部倾斜姿态，每次稳定后分别读取关节。下面命令只读取状态，拟合脚本只做离线正运动学和计算，不发送运动指令。不要让尖端在不同位置接触一个有面积的标记；应使用可重复定位的点或孔。

```bash
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/tcp-pivot/pose-01.json
# 通过机器人控制界面改变腕部倾斜，仍让同一尖端顶住同一固定点
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/tcp-pivot/pose-02.json
# 再改变腕部倾斜并重复接触
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/tcp-pivot/pose-03.json
# 第四种不同的倾斜姿态
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/tcp-pivot/pose-04.json

.venv/bin/python scripts/solve_tcp_pivot.py \
  --profile configs/local-robot-intrinsics.json --side left \
  --states calibration_data/tcp-pivot/pose-{01,02,03,04}.json \
  > calibration_data/tcp-pivot/result.json
cat calibration_data/tcp-pivot/result.json
```

结果中的 `tip_in_wrist_m` 是可填入 `wrist_to_tcp_pose7` 的前三个数，`touch_residuals_mm` 和 `max_residual_mm` 显示各次接触的一致性；默认最大允许残差为 5 mm，可按测量精度设置 `--max-residual-mm`。姿态变化不足或残差超限时脚本报错，需重新采样。拟合残差仅检查这几次接触自身，不是独立验收。脚本严格检查所选机械臂的关节限位；若仅另一侧关节读数越过配置限位，会在离线正运动学计算中将其截到限位并在 `ignored_other_arm_limit_violations` 列出，所选腕部的计算不受影响。该记录仍提示需要另行核对机器人读数与限位配置。这个固定点方法**不能求出 TCP 的朝向**：无论 TCP 坐标轴怎样旋转，尖点仍在同一个位置。按照实际工具安装几何独立确定 TCP `+Z` 接近轴和 `+Y` 向上参考，再将腕部坐标系中的单位四元数以 `--tcp-quat-wxyz QW QX QY QZ` 传入同一命令；此时结果才会给出完整的 `wrist_to_tcp_pose7`。只有实测确认 TCP 轴与腕部轴重合，才能传入 `1 0 0 0`。脚本不会修改配置文件；核对结果后手动填入当前配置的 `pregrasp.left.wrist_to_tcp_pose7`（右臂改用 `--side right` 并独立采样）。

随后对用于相机外参独立验证的**多个不同点**，让已标定的 TCP 原点尖端分别顶到各点并读取关节：

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

每个点重复 `state` 与换算，`side` 保持一致。确认顶点的仍是标定时使用的同一个尖端，且工具安装没有移动；否则应重新标定尖点位置。

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

### 可选：用棋盘格生成不依赖深度的验证点

可复用标定内参时的**实体棋盘格**，但必须在外参求解结束后拍新的验证图像；不要用参与手眼求解的图像或用待验证外参反算基座点。脚本读取内参 JSON 中的内角点数量和实测方格边长，检测彩色图像中的角点，用 PnP 求棋盘格相对相机的位姿，再用保存的关节状态与已标定的 TCP 求对应角点在 `base_Link` 中的位置。它不读取深度图，不连接机器人，也不发送运动命令。

将棋盘格固定在机械臂能触及的位置。每次使用 `capture --raw` 拍原始彩色图像；拍完后保持棋盘、相机头部和尖端安装不动，让尖端分别接触指定**内角点**，每次稳定后将状态保存到不同文件。只能通过机器人已有的受审核控制界面移动机械臂。若接触使棋盘移动，重新拍摄；若要换棋盘位置，先固定在新位置，再拍新图像。以下示例使用 `7x9` 个内角点，行、列都从零开始；图像中角点的检测顺序需人工核对，尤其要确认第一角点。`view-01` 对应前两次接触；移动棋盘并固定后拍 `view-02`，再记录第三次接触。

```bash
.venv/bin/tron2-deploy capture --raw --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/view-01
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/state-01.json
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/state-02.json
# 将棋盘固定在第二个位置，然后重新拍摄
.venv/bin/tron2-deploy capture --raw --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/view-02
.venv/bin/tron2-deploy state --profile configs/local-robot-intrinsics.json --output calibration_data/heldout-board/state-03.json

.venv/bin/python scripts/validate_chessboard_extrinsics.py \
  --profile configs/local-robot-intrinsics.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --handeye calibration_data/handeye-right-guided/handeye-right.json \
  --side left --max-error-m 0.005 --max-reprojection-px 2.0 \
  --touch calibration_data/heldout-board/view-01 0 0 calibration_data/heldout-board/state-01.json \
  --touch calibration_data/heldout-board/view-01 8 6 calibration_data/heldout-board/state-02.json \
  --touch calibration_data/heldout-board/view-02 2 4 calibration_data/heldout-board/state-03.json \
  --output calibration_data/heldout-board/points.npz
```

每个 `--touch` 依次指定采集目录、内角点行号、列号、该角点的关节状态文件；重复行数可超过三。先打开 `points-marked/view-*.png` 核对红圈与实际接触的角点一致。脚本打印每点三维误差、最大误差与每张图像的 PnP 重投影误差，并保存 `points.npz` 和 `points.report.json`。`--max-reprojection-px 2.0` 是示例质量门限，应按实际角点质量确定；超限时检查图像与内参。三维验证失败时退出码为 1，但仍保存诊断数据；只有检查通过后，才将 `points.npz` 传给下方 `apply-calibration` 的 `--validation-points`。相同图像上的多个角点共享一次 PnP 位姿；应在工作空间不同位置和倾角重复采集。PnP 仍依赖内参和棋盘尺寸，接触仍依赖 TCP 精度与棋盘固定程度；低重投影误差本身不能证明毫米级三维精度。最大误差阈值应按测量精度和部署间隙预算确定。

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

分别记录 `pregrasp.<side>.wrist_to_tcp_pose7`，格式为 `[x,y,z,qw,qx,qy,qz]`：表示 TCP 坐标系在配置的腕部刚体坐标系中的位姿，平移单位为米，四元数必须归一化。平移可由上述固定点拟合，朝向仍需从实际安装几何独立确定。TCP `+Z` 是朝向物体的接近轴，TCP `+Y` 是目标选择时使用的滚转/向上参考。只有两个实际坐标系重合时，单位变换才有效。

在 `base_Link` 中测量 `scene.table_z_m`；设置 `scene.object_radius_m`，使其以物体位姿原点为中心包住整个已注册物体网格。保留会话目录、原始图像、样本文件、可用时的左右两次求解、保留点测量、安装测量和已接受的模型版本。拟合结果异常或验证点检查失败时，可运行 `.venv/bin/tron2-deploy calibration-report --help` 查看可选诊断报告的输入参数，并将生成的报告与输入数据一起保存。辅助页和报告不会应用标定或启用执行。外参拟合通过不代表碰撞几何、腕部安装、传输时序或停止行为已经验证。

头部偏离标定姿态后，此固定头部流程将失效。应恢复到该实测姿态或重新标定；服务不会外推相机与头部的运动学链。接下来完成[部署](deployment.zh-CN.md)，再进行[预抓取规划](pregrasp.zh-CN.md)。
