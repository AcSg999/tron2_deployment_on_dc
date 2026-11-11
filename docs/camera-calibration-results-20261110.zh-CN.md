# 头部与右腕相机标定结果

[English](camera-calibration-results-20261110.md) · [跟踪检查](wrist-in-head-projection.zh-CN.md)

本页保留 2026-11-10 的历史结果，以下数值对应 `head-20261110-220522__wrist-20261109-223333`。当前验证绑定已于 2026-11-11 更新为 `head-20261110-220522__wrist-20261111-180956`，见[当前验证配置](wrist-in-head-projection.zh-CN.md)。

历史版本中，你提供的另一组头部内外参来自 `/home/dc/calibration/tron2_camera_calibration/sp_vision/data/head_camera_session`；右腕使用 `wrist-20261109-223333`。四份参数文件已逐字节复制到独立版本目录，没有利用验证观察重新拟合或调整标定参数。

两台相机的内参、外参文件均记录拟合门槛 `passed: true`。每组使用 34 张训练、6 张留出，无拒绝视角或离群样本。两组图像均为 640×480，棋盘为 7×10 内角点，格长为 0.021 m。独立的实物精度仍未验证。

## 内参

焦距和主点单位为像素。下列数值已四舍五入，完整精度保存在 JSON 中；参数适用于标定时的图像流几何关系。

```text
K = [[fx,  0, cx],
     [ 0, fy, cy],
     [ 0,  0,  1]]
```

| 相机 | fx | fy | cx | cy | 重投影 RMS |
|---|---:|---:|---:|---:|---:|
| 头部 | 404.160815 | 408.624959 | 320.203505 | 230.261438 | 0.081087 px |
| 右腕 | 433.016129 | 433.630983 | 322.649964 | 231.992261 | 0.152248 px |

畸变顺序：`[k1, k2, p1, p2, k3]`。

```text
head_D = [-0.019421023, 0.069445925, -0.019398520, -0.001448037, -0.036887054]
wrist_D = [-0.060437013, -0.018042387, -0.002824731, 0.000885365, 0.320308046]
```

## 外参

约定为 `p_A = T_A_B @ p_B`，最后一列为平移，单位为米。在实测关节状态 `q` 下，`T_base_camera(q) = T_base_parent(q) @ T_parent_camera`。

头部：`T_pitch_camera`，将相机光学坐标转换到 `head_pitch_Link`。

```text
[
  [ 0.011444, -0.010666,  0.999878,  0.034868],
  [-0.999919,  0.005495,  0.011504, -0.048125],
  [-0.005617, -0.999928, -0.010602,  0.061620],
  [ 0.000000,  0.000000,  0.000000,  1.000000]
]
```

右腕：`T_wrist_roll_camera`，将相机光学坐标转换到 `wrist_roll_R_Link`。

```text
[
  [-0.025944, -0.986669, -0.160661,  0.034908],
  [-0.999620,  0.027111, -0.005078,  0.009521],
  [ 0.009366,  0.160469, -0.986997, -0.092557],
  [ 0.000000,  0.000000,  0.000000,  1.000000]
]
```

## 拟合残差

以下为所提供结果中，各视角棋盘位姿相对基座系中共同拟合棋盘位姿的最大残差，配置阈值为 15 mm、2°，留出图像不参与拟合。用原始图像复核所提供的头部内外参后，与记录残差的差异不超过 0.040 mm、0.010°；K、畸变和两个已拟合变换均保持原值。

| 相机 | 数据集 | 张数 | 最大平移残差 | 最大旋转残差 |
|---|---|---:|---:|---:|
| 头部 | 训练集 | 34 | 5.708 mm | 1.200° |
| 头部 | 留出集 | 6 | 5.641 mm | 1.015° |
| 右腕 | 训练集 | 34 | 9.224 mm | 0.960° |
| 右腕 | 留出集 | 6 | 7.947 mm | 0.885° |

## 独立双相机棋盘验证

使用你提供的另一组参数，回放了此前保存且已核实实体角点身份的 `board-20261110-201025`。该验证图像的 RGB 哈希未出现在所提供的头部标定数据集中，没有使用翻转标志或几何补偿。

| 指标 | 另一组头部参数 | 门槛 | 结果 |
|---|---:|---:|---|
| 相对平移差异 | 25.708 mm | 15 mm | 超限 |
| 相对旋转差异 | 1.746° | 2° | 门槛内 |

各自相机的 PnP RMS 为头部 0.108 px、腕部 0.162 px；交叉投影 RMS 为 12.344 px / 21.065 px。状态为 `relative_pose_exceeds_limits`。这是一组姿态下的相对链路观察，不能单独定位某台相机/FK 的误差或确定绝对精度。下一步在不同手臂姿态下补充停稳后的棋盘图像对，保持同一棋盘几何，并独立核对角点身份。验证观察与拟合数据继续分开。

结果：[另一组参数的棋盘检查](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/validation/board-201025/board-check.json)。

## 历史手动参照回放：19:02 头部版本

以下历史对照使用 `head-20261110-190209`，发生在切换你提供的另一组头部参数之前；这些数值不是当前 22:05 版本的测量结果。没有利用镜头点选结果拟合参数或补偿量，同一图像的重复标注按一帧计算。

| 采集轮次 | 独立图像数 | 原版 RMS | 19:02 RMS | 19:02 最大值 |
|---|---:|---:|---:|---:|
| 右臂变化、头部固定 | 5 | 43.38 px | 17.99 px | 32.64 px |
| 头部变化、右臂固定 | 5 | 34.98 px | 6.38 px | 10.05 px |

投影有所改善，但仍有残差。这些数值是预测点与用户点选的可见 RGB 镜头特征之间的像素距离，尚未确认该特征与内部光学中心等价，因此不能据此确定毫米或角度精度。参见[回放对比图](../output/head-recalibration-20261110-190209/replay-comparison.png)。

使用历史 19:02 头部版本时，双相机棋盘观察测得相对差异 17.308 mm、2.069°；另外三组近似同姿态数据复现约 17.3 mm、2.06° 的差异，超过 15 mm / 2° 相对一致性门槛。具体是哪段相机/FK 几何有误仍待定位。参见[历史复核观察记录](../output/wrist-in-head/board-20261110-201025-reviewed/board-check.json)、[重复性报告](../output/wrist-in-head/board-20261110-201025-analysis/summary.json)。

## 相对 URDF 名义安装值的差异

`calibrated_from_nominal` 在同一父坐标系下比较拟合安装变换与 URDF 名义安装变换，描述两份几何估计的不一致，并非独立测得的 URDF 实物误差。

| 相机 | 平移差异 | 旋转差异 |
|---|---:|---:|
| 头部 | 60.742 mm | 0.951° |
| 右腕 | 2.503 mm | 1.753° |

上述头部数值相对于外部解算所用的名义相机安装值，该名义值与跟踪 URDF 的名义安装值不同，因此不能用这一列作为各标定版本的统一对比基准。两个模型实际使用的 `base_Link` → `head_pitch_Link` 正运动学链相同，并通过链签名校验；投影使用你提供的标定安装变换。此前头部会话 `head-20261109-201636` 记录的是 92.864 mm、2.187°，[旧版两相机汇总](../output/calibration/camera-calibration-head-wrist-20261110.json)仍保留。

## 结果文件

独立版本目录为 `/home/dc/tron2_deployment_on_dc/output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/`。验证工具默认读取固定绑定的本机清单，启动时打印实际加载的标定 ID；显式传入 `--calibration-summary PATH` 可以选择其他版本。程序校验文件哈希、内外参配对和相关正运动学链。修改原工作文件不会改变这些快照。

| 记录内容 | 文件路径 |
|---|---|
| 头部内参 | [head_intrinsics.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/head_intrinsics.json) |
| 头部外参 | [head_extrinsics.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/head_extrinsics.json) |
| 右腕内参 | [wrist_intrinsics.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/wrist_intrinsics.json) |
| 右腕外参 | [wrist_extrinsics.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/wrist_extrinsics.json) |
| 本版标定清单 | [camera-calibration-summary.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/camera-calibration-summary.json) |
| 跟踪工具默认读取的本机清单 | [/home/dc/tron2_deployment_on_dc/configs/local-camera-calibration-latest.json](/home/dc/tron2_deployment_on_dc/configs/local-camera-calibration-latest.json) |
| 来源和兼容性证据 | [provenance.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/provenance.json) |
| 当前棋盘验证 | [board-check.json](../output/calibration/bundles/head-20261110-220522__wrist-20261109-223333/validation/board-201025/board-check.json) |

`rms_px` 为内参重投影 RMS。外参 `training`、`holdout` 中的 `translation_m` 单位为米，`rotation_rad` 单位为弧度。`calibrated_from_nominal` 中平移为米；头部使用 `rotation_rad`，右腕使用 `rotation_deg`。独立版本记录了来源文件哈希、原始头部图像哈希、配对检查与正运动学兼容性。原 19:02 头部工作文件已被你提供的另一组文件覆盖；其历史清单保留为记录，不能作为原参数的完整可加载快照。本机输出与本机绑定文件均由 Git 忽略。
