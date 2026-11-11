# 右腕相机标定

[English](wrist_calib.md)

这是右手腕彩色相机的独立只读实验。从本目录运行命令，图像和结果保存于 `data/wrist_camera_session/`；该目录不会进入 Git。模型文件随本目录提供，离线标定不再依赖外部仓库。程序不驱动机器人。

控制器负载辨识值不参与这项几何标定。若用拖动示教调整手臂，应单独在控制器中设置并回读当前负载，不要将这些值写入相机配置。

## 标定参数发布

本地最新头部结果为 `data/head-camera-session/intrinsics.json` 和 `extrinsics.json`（2026-09-23）；最新右腕结果为 `data/wrist_camera_session/wrist_intrinsics.json` 和 `wrist_extrinsics.json`（2026-10-05）。其逐字节副本与来源/校验值清单已于 2026-10-08 发布到专门的[相机标定仓库](https://github.com/zengxiong111/tron2_camera_calibration/tree/main/calibration_results)。本仓库的原始采集和本机配置仍由 Git 忽略。四份拟合结果均记录 `passed: true`，但独立触点验证仍为 `passed: false`（头部最大 14.42 mm，右腕 12.26 mm，阈值 10 mm）。发布说明记录坐标系约定、图像尺寸和历史头部模型限制；发布不代表硬件验收通过。

## 坐标系与几何关系

`configs/assembly.urdf` 中，相机支架固定在 `wrist_roll_R_Link`。它与 `wrist_pitch_R_Link` 之间还有可动的 `wrist_roll_R_Joint`：roll 的局部 X 轴与 pitch 的局部 Y 轴正交。因此应求解的常量是 **`T_wrist_roll_camera`**，即从 `right_wrist_camera_color_optical_frame` 到 `wrist_roll_R_Link` 的变换。URDF 中的数值只是名义安装参考，不是实测标定结果。对任一实测手臂状态：

```text
T_base_camera(q) = T_base_wrist_roll(q) · T_wrist_roll_camera
T_base_board = T_base_camera(q) · T_camera_board
T_wrist_pitch_camera(q_roll)
  = T_wrist_pitch_wrist_roll(q_roll) · T_wrist_roll_camera
```

`T_base_wrist_roll` 需要右臂七个实测关节角；头部与左臂关节角不在这条 FK 链上。采集仍按名称保存双臂全部 14 个角，供审计及后续触点验证使用。

相机内参不取决于可动关节数量，但必须针对当前 **640×480 的腕部缩放图像** 单独标定。当前 ROS 2 `CameraInfo` 报告的是原始流的 848×480，不能直接原样用于缩放话题。

## 准备配置并核对关节映射

```bash
cp -n configs/wrist_config.example.json configs/wrist_config.json
```

保留已有本机配置。示例默认使用 `capture.backend: "ssh"`，通过 SSH 读取 ROS 2 话题。若要使用机器人自带 LimX Studio 的相机 WebSocket 桥接、绕过 SSH，将 `configs/wrist_config.json` 中的 `capture` 对象替换为：

```json
{
  "backend": "bridge",
  "bridge_profile": "../../configs/local-robot-seed.json",
  "state_profile": "../../configs/local-robot-seed.json",
  "ros_domain_id": 0,
  "color_topic": "/camera/right/color/image_resized/compressed",
  "joint_topic": "/joint_states",
  "max_state_skew_ms": 100,
  "timeout_s": 8
}
```

profile 路径相对于配置文件解析。本机已有上述本地 profile；其他机器应使用自己的实际 profile。桥接集成需要父仓库的 `tron2_deployment` 软件包及其 `bridge` 可选依赖。它读取 `camera.bridge_host`、`bridge_path`、TLS 和令牌设置，但明确订阅上面的腕部话题。浏览器无需一直打开。桥接失败会直接报错，不会退回 SSH。控制器状态比较仍使用已有的只读 WebSocket 连接。

桥接提供转发时间戳，不提供原始 ROS 传感器消息头。保存的采集数据与 probe 报告会标记 `timestamp_source: "bridge_forwarding_ms"` 和 `sensor_header_timestamps_available: false`。因此 100 ms 阈值检查的是转发时间对齐，不能证明曝光时刻同步。图像与状态还必须均在最近 750 ms 内收到。采样时保持手臂静止，并完成独立触碰验证。光学坐标系按配置中的相机话题指定，并记录 `frame_id_source: "configured_camera_topic"`，不是接收到的 ROS 消息头。

腕部离线求解只要求保存的图像和包含同步 `arm_q14` 的 JSON，不依赖这两种实时传输方式。配置的名称列表将外部关节名称映射为随附 URDF 要求的 14 值顺序。FK 的实物精度仍需用留出集棋盘图像和独立触碰检查。

如果终端设置了 HTTP 代理，读取控制器反馈前先让机器人局域网地址绕过代理：

```bash
export NO_PROXY=10.192.1.2,10.192.1.4,127.0.0.1,localhost
export no_proxy="$NO_PROXY"
```

```bash
../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json probe
```

`probe` 不要求画面中有棋盘，会把同步图像和带名称的关节 JSON 保存到 `data/wrist_camera_session/`。它还会在拍照前后读取头部验证所用的同一控制器 `arm_q14`；只有手臂保持静止且 ROS 2 映射值逐项相差不超过 0.005 rad，`mapping_check.passed` 才为 `true`。控制器比较失败或不可用，不代表图像获取失败。图像与 ROS 2 关节状态的 `state_skew_ms` 是另一项检查，阈值为 100 ms。使用桥接后端时，终端汇总应显示 `source: "limx_studio_bridge"`，其时间含义见上文。`capture` 使用同一配置选择传输方式，所有拟合命令保持一致。

把平整的 7×10 内角点棋盘刚性固定，实测方格边长为 0.021 m。在图像采集及触点验证期间保持其位置不变。只通过机器人已有的审核接口调整手臂，并在稳定后采样。采集不同的位置和方向，让棋盘覆盖画面不同区域，并使腕部至少绕两个不同方向转动。每张图应能看到完整棋盘；标定期间不要改变图像缩放设置。

## 采样、拟合与检查

```bash
../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  capture --session data/wrist_camera_session --count 40

../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  intrinsics --session data/wrist_camera_session

../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  extrinsics --session data/wrist_camera_session
```

采集窗口中，`s` 保存通过检查的图像，`f` 将棋盘角点顺序旋转 180°，`r` 或空格重新取图，`q` 退出。不加 `--append` 时，完整采集成功后覆盖该 session 的旧 `view-*`；中途退出会保留旧图。需要追加视角时加 `--append`。内参拟合复用头部流程的棋盘检测、畸变拟合、质量阈值及留出集划分。外参拟合复用 PnP 与手眼求解，但每张图都使用同步的 `T_base_wrist_roll(q)`。程序检查 URDF/MJCF 中的名义相机链，并用未参与拟合的图像检查训练所得棋盘位姿。要求结果 `passed: true`；同时检查 `training`、`holdout`、被拒绝视角及相对名义安装值的偏差。

### 外参汇总参数的含义

当前这组外参标定的终端汇总包含：

```json
"max_training_mm": 7.555524738640365,
"max_holdout_mm": 10.41927581138533,
"mount_rotation_span_deg": 176.67627544117101
```

- `max_training_mm`：所有训练视角中最大的棋盘平移残差，单位为毫米。训练视角参与求解 `T_wrist_roll_camera` 和 `T_base_board`；对每个视角，程序将 FK、手眼外参与 PnP 结果合成的棋盘位置与固定的 `T_base_board` 比较，取平移差模长的最大值。数值越小，训练集内部一致性越好；此处约为 **7.56 mm**。
- `max_holdout_mm`：所有留出视角中按相同方法计算的最大棋盘平移残差，单位为毫米。留出视角不参与手眼外参拟合，因此该值用于检查标定对未参与拟合图像的一致性；此处约为 **10.42 mm**。
- `mount_rotation_span_deg`：训练视角中腕部安装链姿态的旋转覆盖范围，单位为度。具体是第一个训练视角的 `T_base_wrist_roll` 旋转与其他每个训练视角之间的相对旋转角最大值。它表示采样的姿态激励范围，不是标定误差，也不是某一个腕部关节的行程；此处约为 **176.68°**。

前两个值均低于当前 `max_board_residual_m: 0.015` 对应的 **15 mm** 阈值，旋转覆盖范围也高于 `min_mount_rotation_span_deg: 20` 的最低要求。但不能只根据这三个数宣布标定通过；总体 `passed` 还要求每个视角的旋转残差不超过 **2°**、至少有一个有效留出视角、优化器成功，并且采样不得几乎只绕单轴旋转。

## 标定右臂触点 TCP

若要重新标定 TCP，用同一支相对右腕刚性不动的尖端抵住同一个固定点，在至少四个明显不同的腕部朝向下读取实测状态。灵巧手指尖只有在所有手指关节始终保持同一姿态时才能作为尖端；状态文件不记录手指关节。每次由已有的受审核界面调整姿态，稳定后使用机器人集成提供的只读状态命令。该命令不属于独立发布的 `sp_vision`，下面只消费它生成的 JSON 文件。示例使用本机真实 profile `../configs/local-robot-seed.json`；其他机器应替换为实际 profile，不要使用主机地址仍为占位符的模板。

采集四个 TCP 姿态前，可以使用以下只读命令读取一次当前头部 `q2`：

```bash
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output data/wrist_camera_session/tcp/current-state.json
../.venv/bin/python -c 'import json, pathlib; p = pathlib.Path("data/wrist_camera_session/tcp/current-state.json"); print(json.loads(p.read_text())["head_q2"])'
```

`head_q2` 顺序为 `[head_pitch_Joint, head_yaw_Joint]`，单位为弧度。需要新状态时重新运行第一条命令。它保存完整的 `arm_q14` 和 `head_q2`，不会发送运动命令。

```bash
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output data/wrist_camera_session/tcp/pose-01.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output data/wrist_camera_session/tcp/pose-02.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output data/wrist_camera_session/tcp/pose-03.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output data/wrist_camera_session/tcp/pose-04.json
```

腕部程序直接复用头部流程的固定点拟合器，输出 `wrist_roll_R_Link` 中的尖端位置：

```bash
../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  pivot --states data/wrist_camera_session/tcp/pose-{01,02,03,04}.json \
  --output data/wrist_camera_session/tcp/wrist_tcp_pivot.json
```

要求结果 `passed: true`，并检查 `max_residual_m`、姿态跨度和 `condition_number`。四姿态拟合只确定尖端**位置**，不确定工具朝向。若实体尖端和安装从头部验证时起完全未变，也可跳过重采样，改用已复制的 `data/wrist_camera_session/tcp-pivot-from-head.json`；它的内部拟合通过，但先前头部相机的独立三点验证仍有 13–14 mm 误差，不能视为整条测量链通过。

如果已通过头部流程重新拟合并保存为 `data/head_camera_session/tcp/head_tcp_pivot.json`，无需再采集四姿态；将这份新结果复制到上述腕部 `wrist_tcp_pivot.json` 路径，或在下方验证命令中直接把它作为 `--tcp` 参数。不要误用更早的 `tcp-pivot-from-head.json`。

## 独立触碰验证

外参拟合后，保持棋盘在基座中固定，用腕部相机拍摄一张**不参与求解**的新图像。拍照后右臂可以移动；预测使用图像保存时同步的关节状态。选择三个分散、不共线且能用尖端接触的实体内角点：

```bash
../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  capture --session data/wrist_camera_validation --count 1

../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  select-validation --frame data/wrist_camera_validation/view-001 \
  --output data/wrist_camera_validation/wrist_selection.json
```

这条 `capture --count 1` 每次完整保存后都会用新的 `view-001` 覆盖旧验证图像；按 `q` 退出则保留旧图。重拍后必须重新运行 `select-validation`，使 `wrist_selection.json` 和 `wrist_selection.png` 对应新图；旧触点状态是否还能使用，取决于棋盘和实体角点是否保持不动。

先检查 `data/wrist_camera_validation/wrist_selection.png` 的编号是否对应将要触碰的实体角点。保持棋盘固定，用与 TCP 拟合时相同的尖端和手指姿态，按编号 1、2、3 触碰；每次稳定后读取一次状态：

```bash
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output data/wrist_camera_validation/state-01.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output data/wrist_camera_validation/state-02.json
../.venv/bin/tron2-deploy state --profile ../configs/local-robot-seed.json \
  --output data/wrist_camera_validation/state-03.json

../.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  validate --selection data/wrist_camera_validation/wrist_selection.json \
  --side right --tcp data/wrist_camera_session/tcp/wrist_tcp_pivot.json \
  --states data/wrist_camera_validation/state-{01,02,03}.json \
  --output data/wrist_camera_validation/wrist_validation.json
```

如果复用先前 TCP，仅把上述 `--tcp` 路径改为 `data/wrist_camera_session/tcp-pivot-from-head.json`。验证在 `base_Link` 中比较相机预测的三个角点和尖端实测位置；报告的 `passed` 才是这次独立检查的结果。TCP 内部残差和外参留出集通过都不能替代它。

外参 JSON 的 `training`、`holdout` 中，`translation_m` 以米为单位；终端汇总中的 `max_training_mm`、`max_holdout_mm` 将其换算为毫米，与头部命令一致。若三点触碰误差接近棋盘角点间距，先核对 `wrist_selection.png` 上的 1、2、3 与实际触碰顺序。只有确认物理触点后，才用 `select-validation --corner 行,列` 按真实触碰顺序重新选点并验证；不要仅根据触碰状态反推角点后宣称独立验证通过。
