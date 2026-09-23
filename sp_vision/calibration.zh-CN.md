# 头部相机最小标定实验

[English](calibration.md)

本目录是一个独立的最小实验，只标定：

1. 头部彩色相机内参；
2. 彩色光学相机坐标系到 `head_pitch_Link` 的刚体外参 `T_pitch_camera`；
3. 使用棋盘格内角点和机械臂触点进行独立验证。

采集时固定棋盘格并移动头部的 yaw、pitch。程序不控制头部、机械臂、灵巧手或夹爪；所有运动都通过机器人已有且经过审核的界面人工完成。代码和生成配置均使用 JSON，不使用 YAML。

## 最终模型与坐标约定

本实验以仓库中的最终带灵巧手模型为准：

- `configs/assembly.urdf`：运动学真值；
- `configs/scene.xml`：MuJoCo 场景真值及交叉检查；
- `head_camera_color_optical_frame`：被标定的彩色光学坐标系，OpenCV 约定为 +x 向右、+y 向下、+z 向前；
- `head_pitch_Link`：与相机最近的可动 pitch 轴坐标系。

模型中还保留了一个直接挂在 `base_Link` 下的旧 `d435_Link`。本实验明确忽略它，只使用 `head_pitch_Link` 下的 `head_camera_color_optical_frame`。求解外参时，URDF 中的相机固定变换仅作为名义值对照，不作为求解约束。

所有矩阵采用 `T_A_B` 表示“把 B 坐标转换到 A 坐标”。头部完整链为：

```text
T_base_pitch(q_yaw, q_pitch)
  = T_base_headbase
  · T_headbase_yaw(q_yaw)
  · T_yaw_pitch_origin
  · R_pitch(q_pitch)
```

yaw→pitch 的固定偏移是 `[0.051, 0.03, 0.097] m`。因此 yaw 旋转会带着 pitch 轴原点移动；pitch 只绕自己的原点旋转，不会改变 yaw，也不会移动自身原点。虽然采集 JSON 的顺序是 `[head_pitch_Joint, head_yaw_Joint]`，程序先按关节名称建立映射，再依照 URDF 的父子链执行 yaw→pitch，不依赖数组顺序猜测。

每次外参求解都会检查 `assembly.urdf` 和 `scene.xml` 在三个不同头姿下的 `base→pitch` 与 `pitch→color optical` 变换；两者不一致时直接停止。

## 棋盘格与依赖

当前实体板为横向 7、纵向 10 个**内角点**，方格边长 `0.021 m`。即印刷图案应有 8×11 个方格，并在四周留至少约一个方格宽的白边。修改标定板后必须同步修改 JSON 中的三个参数。

棋盘参数也可以像旧流程一样直接写在命令中：`--pattern` 始终表示“列数×行数”的内角点，`--square-m` 表示实测方格边长（米）。命令行值优先于 JSON；内参、外参和验证参数不一致时程序会拒绝继续。

从仓库根目录准备实验配置：

```bash
cp sp_vision/config.example.json sp_vision/local-calibration.json
.venv/bin/python -m pip install -r sp_vision/requirements.txt
```

检查 `sp_vision/local-calibration.json` 中的路径。默认已经指向：

```json
{
  "capture": {"profile": "../configs/local-robot-seed.json"},
  "robot": {
    "urdf": "../configs/assembly.urdf",
    "model_xml": "../configs/scene.xml",
    "pitch_link": "head_pitch_Link",
    "camera_frame": "head_camera_color_optical_frame"
  }
}
```

相对路径均相对于该配置 JSON 所在目录。现有项目 `.venv` 已经包含 NumPy、SciPy、OpenCV contrib，以及 bridge/ROS 采集适配器，通常不需要额外安装。

## 一、采集约 30 个头姿

把棋盘刚性固定在相机和机械臂都能看到、触及的位置。整个内参和外参数据集内不得移动棋盘。运行：

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/head-camera-session --count 30
```

程序直接复用现有 profile 的 bridge 或 ROS 相机配置，同时读取与图像对应的 `[pitch, yaw]`。它不会沿用旧固定头姿检查。同步状态回退、关节/图像时间差超限或未完整检测 70 个角点的帧不能保存。

窗口按键：

- `s`：仅保存当前通过检查的图像和 JSON；
- `r`、空格或其他普通按键：丢弃当前帧并重新采集，不写照片；
- `f`：将角点顺序旋转 180°；当 `(0,0)` 没有落在与其他视角相同的实体角点时使用；
- `q` 或 Esc：退出。

预览使用与附图相同的彩色逐行连线，并标出四角的 `(row,column)`。每帧保存为：

```text
sp_vision/data/head-camera-session/
  view-001/
    color.png
    corners.png
    frame.json
  ...
```

保持棋盘不动，通过已有控制界面改变头部。采样应同时覆盖 yaw 和 pitch 的正负方向、不同组合，并在每次保存前等待头部完全停止。避免只沿单轴运动、连续保存几乎相同的姿态，或让棋盘始终只占画面中央。默认把最后 6 帧留作外参保留集，其余约 24 帧参与求解。

检测器使用：

```text
findChessboardCornersSB(
  CALIB_CB_NORMALIZE_IMAGE |
  CALIB_CB_EXHAUSTIVE |
  CALIB_CB_ACCURACY
)
```

该检测器直接返回亚像素内角点，不再额外调用 `cornerSubPix`。程序还检查角点拓扑、棋盘画面覆盖率、边界距离和可选清晰度阈值。`corners.png` 是必须保留的人工复核证据。

## 二、求解内参

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json intrinsics \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/head-camera-session \
  --output sp_vision/data/head-camera-session/intrinsics.json
```

程序只用训练帧拟合 `K` 和五参数畸变，并用逐帧重投影误差剔除明显异常视角。剔除记录和原因写入 JSON，不会静默消失。结果还检查整个图像范围内的径向畸变映射是否保持单调；非单调结果视为标定失败。

查看重投影误差：
```bash
.venv/bin/python - <<'PY'
import json

path = "sp_vision/data/head-camera-session/intrinsics.json"
result = json.load(open(path))

print("总体重投影 RMS:", result["rms_px"], "px")
for view, error in result["training_view_rms_px"].items():
    print(f"{view}: {error:.4f} px")
PY
```
重点检查：

- `passed` 必须为 `true`；
- `training_view_rms_px` 不应出现单帧明显突增；
- `radial_monotonicity.minimum_radial_derivative` 必须大于零；
- `diagnostics/intrinsics/*.png` 中所有角点顺序正确。

RMS 小并不能弥补姿态覆盖不足。焦距或主点与相机出厂值相差异常大时，应先检查棋盘尺寸、屏幕摩尔纹、角点方向和视角分布，不要继续求外参。优先使用平整的实体印刷板，不要把电脑屏幕上的棋盘作为最终标定板。

## 三、求解相机到 pitch 轴外参

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json extrinsics \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/head-camera-session \
  --intrinsics sp_vision/data/head-camera-session/intrinsics.json \
  --output sp_vision/data/head-camera-session/extrinsics.json
```

每帧先用 IPPE PnP 求 `T_camera_board`，再从最终 URDF 按该帧同步的 yaw、pitch 求 `T_base_pitch`。固定棋盘满足：

```text
T_base_pitch(i) · T_pitch_camera · T_camera_board(i)
  = T_base_board
```

程序比较 OpenCV 的多种 hand-eye 初值，再对 `T_pitch_camera` 和固定的 `T_base_board` 做联合鲁棒优化。它要求 yaw、pitch 都有足够角度跨度，并用最后 6 帧检查未参与求解的棋盘位姿是否仍保持固定。

输出中的关键字段为：

- `T_pitch_camera`：实际标定结果，运行时应使用它；
- `nominal_T_pitch_camera`：最终 URDF 中的名义安装值，仅供比较；
- `calibrated_from_nominal`：实测与名义安装的差异；
- `training`、`holdout`：每帧 PnP、平移和旋转残差；
- `model_consistency`：`assembly.urdf` 与 `scene.xml` 的链一致性检查。

运行时相机在基座中的动态位姿为：

```text
T_base_camera(q_yaw, q_pitch)
  = T_base_pitch(q_yaw, q_pitch) · T_pitch_camera
```

所以完成本标定后，执行任务时的 yaw、pitch 不必与标定中的某一个姿态相同；必须使用任务当时的实时关节角做上述动态组合。

## 四、标定机械臂触点 TCP

最终验证需要一个相对腕部刚性不动、能重复接触内角点的尖端。带灵巧手时建议安装刚性探针；也可以使用固定姿态的指尖，但整个 TCP 标定和验证期间所有手指关节必须保持完全相同。当前状态文件只记录 `arm_q14` 和头部关节，不记录手指关节，因此手指一旦移动，验证立即失效。

用同一个尖端抵住同一个固定点，改变至少四种明显不同的腕部朝向，每次稳定后只读取状态：

```bash
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/tcp/pose-01.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/tcp/pose-02.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/tcp/pose-03.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/tcp/pose-04.json
```

离线拟合腕部坐标中的尖端位置：

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json pivot --side right\
  --states sp_vision/data/tcp/pose-{01,02,03,04}.json \
  --output sp_vision/data/tcp/result.json
```

`passed` 必须为 `true`。该步骤只标定触点位置，不标定工具朝向。

## 五、选择三个角点并触碰验证

外参求解结束后，下面是拍一张不参与求解的新图像：

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session sp_vision/data/touch-validation --count 1
```

选择三个分散且不共线的内角点。省略 `--corner` 时可在窗口中点击，程序会吸附到最近的已检测角点；下面示例直接按行列指定：

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json select-validation \
  --pattern 7x10 --square-m 0.021 \
  --frame sp_vision/data/touch-validation/view-001 \
  --intrinsics sp_vision/data/head-camera-session/intrinsics.json \
  --extrinsics sp_vision/data/head-camera-session/extrinsics.json \
  --output sp_vision/data/touch-validation/selection.json
```
--corner 0,0 --corner 0,6 --corner 9,3 \
先打开 `selection.png`，确认编号 1、2、3 与将要触碰的实体角点完全一致。保持棋盘和头部不动，通过机器人已有的受审核控制界面，让同一个已标定尖端依次接触 1、2、3，并按同一顺序读取状态：

```bash
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/touch-validation/state-01.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/touch-validation/state-02.json
.venv/bin/tron2-deploy state --profile configs/local-robot-seed.json \
  --output sp_vision/data/touch-validation/state-03.json
```

程序不会发送任何运动命令。现场必须有人监护，使用低速和可重复定位的尖端，避免碰撞或推动棋盘。

执行比较：

```bash
.venv/bin/python sp_vision/calibration.py \
  --config sp_vision/local-calibration.json validate \
  --selection sp_vision/data/touch-validation/selection.json \
  --side right sp_vision/data/tcp/result.json \
  --states sp_vision/data/touch-validation/state-{01,02,03}.json \
  --output sp_vision/data/touch-validation/report.json
```

相机链预测值为：

```text
p_base_camera
  = T_base_pitch(head_q2)
  · T_pitch_camera
  · T_camera_board
  · p_board_corner
```

机械臂接触实测值为：

```text
p_base_touch
  = T_base_wrist(arm_q14) · p_wrist_tip
```

最终比较的是两者在 `base_Link` 下的三维欧氏距离，不比较关节角。同一个空间点可能对应多组关节角，因此用关节角作为误差指标不成立。`report.json` 给出三点各自误差、平均误差和最大误差；默认最大允许误差为 10 mm，必须根据 TCP 重复性、棋盘固定误差和实际任务间隙预算重新确定。

## 离线测试

```bash
.venv/bin/python -m pytest -q sp_vision/test_calibration.py
```

测试覆盖 7×10 SB 角点检测、hand-eye 数学方向、最终 URDF 的头部 FK、yaw 引起 pitch 原点移动而 pitch 不移动自身原点，以及 `assembly.urdf`/`scene.xml` 的头部相机链一致性。测试不连接相机或机器人。

## 常见失败

- `expected 70 inner corners`：核对内角点数量，避免模糊、反光、遮挡和拍摄显示器产生的摩尔纹。
- `(0,0)` 在不同视角跳到棋盘对角：采集时按 `f` 修正 180° 顺序后再保存。
- `joint/image header skew exceeded`：该帧不是严格同步样本，重新等待头部稳定后采集。
- `insufficient head excitation`：yaw 或 pitch 覆盖不足，补采两个方向的组合姿态。
- URDF/XML 一致性失败：最终模型的关节轴、偏移或相机链已变化，先修复模型，不能用旧外参补偿。
- 触点误差大但重投影误差小：检查 TCP、手指姿态、棋盘是否被碰动、触点顺序及头部是否在拍照后移动。
