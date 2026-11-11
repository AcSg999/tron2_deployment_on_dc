# 将腕部相机位置与坐标轴投影到头部 RGB

[English](wrist-in-head-projection.md) · [标定结果](camera-calibration-results-20261110.zh-CN.md)

## 当前本机标定版本

默认绑定清单为 [configs/local-camera-calibration-latest.json](../configs/local-camera-calibration-latest.json)，目前绑定 `head-20261110-220522__wrist-20261111-180956`。头部继续使用原有 22:05 版本；右腕使用 `sp_vision/data/wrist_camera_session_20261111-180956/` 的内外参。四份文件已逐字节复制到 `output/calibration/bundles/head-20261110-220522__wrist-20261111-180956/`，文件哈希、内外参配对和相关正运动学链检查通过。本次切换没有重新拟合或修改参数数值，实物精度仍未验收。

本次腕部内参 RMS 为 0.2819 px；外参自动排除训练视角 `view-001`、`view-007`、`view-014`，保留 31 张训练图和 6 张留出图。训练集最大残差为 14.51 mm / 1.579°，留出集为 13.00 mm / 1.382°，均在 15 mm / 2° 门槛内。旧绑定备份保存在本版目录的 `previous-default-manifest.json`，完整来源记录见 `provenance.json`。

`--calibration-summary` 现为可选参数，默认读取上述本机文件。启动时打印实际标定 ID 和清单路径，每帧记录保存 `calibration_id`。默认绑定缺失时会明确报错，不会自动退回旧数据；显式传入 `--calibration-summary PATH` 可以选择历史版本。已运行的旧窗口需按 `q` 退出后重新启动，才能加载新参数。

此前 `source hash changed` 的原因是头部工作文件已被替换，但旧清单仍记录原哈希。当前清单指向独立快照，继续保留校验。外部 URDF 与跟踪 URDF 的基座到头部父坐标系正运动学相同；两者不同的名义固定相机安装变换均由你提供的标定变换替代。相关正运动学链通过签名固定，链路发生变化时会拒绝加载。

历史绑定 `head-20261110-220522__wrist-20261109-223333` 回放已复核的图像对 `board-20261110-201025` 时，测得 **25.708 mm / 1.746°**，平移超过 15 mm。该历史绑定另一次只读实时检查的图像/状态时间差为 21 ms，预测中心位于画面外。这些记录不代表本次 18:09 腕部版本的验证结果；本次仍需新的独立实物对照。参见[历史结果与文件路径](camera-calibration-results-20261110.zh-CN.md)。

## 你提出的实验思路

利用右臂实测关节角和已标定的腕部相机安装外参，计算腕部相机在 `base_Link` 中的位姿。将光学中心和三个正向坐标轴端点转换到头部相机坐标系，再投影到头部 RGB。关节反馈变化时，更新预测中心和姿态坐标轴。脚本在本机运行，只订阅头部 RGB 和实测关节；这里的跟随指叠加结果随反馈更新。它不控制手臂或头部运动。

红色中心是几何预测，并非从 RGB 图像中检测镜头后跟踪得到的位置。每个姿态停稳后偏差稳定、换姿态后偏差改变，属于姿态相关的静态残差。透视投影会随距离不同放大相同的几何偏差，可见镜头表面与内部光学原点也可能不同。历史 19:02 标定改善了这项对照，但没有消除残差；下文的双相机棋盘检查使用具有已知几何的参照。

实现位于 [tools/hardware/wrist_in_head_overlay.py](../tools/hardware/wrist_in_head_overlay.py)，复用 `sp_vision` 的 URDF 正运动学和按名称配置的 ROS 到 URDF 关节映射。

```text
B  = base_Link
W  = wrist_roll_R_Link
P  = head_pitch_Link
Cw = right_wrist_camera_color_optical_frame
Ch = head_camera_color_optical_frame

T_B_Cw = FK_B_W(q_arm)  @ T_wrist_roll_camera
T_B_Ch = FK_B_P(q_head) @ T_pitch_camera

p_B  = T_B_Cw @ [0, 0, 0, 1]
p_Ch = inverse(T_B_Ch) @ p_B
(u, v) = project(p_Ch.xyz, K_head, D_head)

T_Ch_Cw = inverse(T_B_Ch) @ T_B_Cw
L = 0.08
P_Cw = [[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]]
P_Ch = transform(T_Ch_Cw, P_Cw)
(u_i, v_i) = project(P_Ch[i], K_head, D_head)
```

头部 FK 使用与图像匹配的实测头部关节，因此头部姿态变化时也能更新投影。投影使用原始 RGB 和畸变参数。图像尺寸与标定不一致时会报错；程序不会静默缩放图像或改用名义相机安装值。

## 这个实验能检查什么

它可以直观检查右臂/头部 FK、腕部安装平移与旋转、头部外参与头部内参组合后的一致性。在不同位置和朝向下，将预测中心、坐标轴方向与可见的腕部相机作比较。

腕部相机原点为 `[0, 0, 0]`，所以它在基座中的位置是 `R_B_W @ t_W_Cw + t_B_W`；仅计算中心时不涉及安装旋转。X/Y/Z 的非零端点会使用完整的腕部安装旋转，因此能够叠加姿态方向。腕部内参仍未参与计算。可见镜头或外壳中心可能与光学中心不同；记录的旋转描述其相对头部相机的预测朝向，并非实测角度误差。定量验证旋转需要独立观测、朝向已知的参照，例如刚性固定且几何关系已知的标记板。

## 坐标轴约定与旋转观察

三轴属于 `right_wrist_camera_color_optical_frame`，方向相对于腕部相机自己的图像/视角定义，遵循 [ROS 光学坐标系约定](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst#suffix-frames)：

| 坐标轴 | 叠加颜色 | 正方向 |
|---|---|---|
| X | 红色 | 腕部相机图像的右方 |
| Y | 绿色 | 腕部相机图像的下方 |
| Z | 蓝色 | 沿 RGB 镜头观察方向向前 |

默认轴长为 0.08 m（8 cm）。在模式参数前设置 `--axis-length-m 0.05`，可改用 5 cm 坐标轴。方向通过完整的相机间相对变换计算，因此在头部 RGB 中不一定水平、竖直或互相垂直。沿头部相机视线方向的轴可能缩成很短的线或一个点。

示教时先检查光学中心位置，再通过现有示教操作缓慢改变腕部朝向，停稳后保存。观察蓝色 Z 是否随 RGB 镜头的观察方向变化，三个坐标轴是否与腕部相机一起旋转。叠加结果本身不提供独立的角度真值。

## 本地运行命令

在 DC 主机的 `/home/dc/tron2_deployment_on_dc` 目录运行。标定汇总指向两台相机的结果 JSON 及其 SHA-256 校验值。程序校验这些文件，以及结果或绑定清单中记录的内外参配对哈希。显式记录了正运动学链签名时，校验相关基座到父坐标系的关节；否则继续校验各相机可用输入清单中的 URDF 哈希。本机结果和 profile 文件不进入 Git。

先生成 mock 示例，不建立连接，也不打开图形窗口：

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --output output/wrist-in-head/mock-preview \
  mock
```

在桌面终端启动实时 RGB 叠加：

```bash
cd /home/dc/tron2_deployment_on_dc
WRIST_PROJECTION_OUTPUT="output/wrist-in-head/teach-$(date +%Y%m%d-%H%M%S)"
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --axis-length-m 0.08 \
  --output "$WRIST_PROJECTION_OUTPUT" \
  live --profile configs/local-robot-seed.json
```

profile 提供 `/camera/top/color/image_raw/compressed` 和 `/joint_states` 所用的 Bridge 地址、TLS 和令牌设置。RGB 请求显式包含 `max_fps=30`，实际传输可能更慢。ROS domain 默认为 0；其他 domain 使用 `live --domain-id N`。此模式需要能访问相机 Bridge，并在图形桌面中运行。按 `q` 或 Esc 退出并断开订阅；按 `s` 保存匹配帧；按空格冻结/恢复，冻结后才能点击可见参照特征。第一张匹配帧会自动保存。诊断信息显示在原始 RGB 区域下方，不改变图像像素坐标。

无图形窗口时，只保存一张实时数据：

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --output output/wrist-in-head/live-snapshot \
  live --profile configs/local-robot-seed.json --headless --frames 1
```

对该记录进行离线回放，不建立连接：

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --output output/wrist-in-head/replay-preview \
  replay --capture output/wrist-in-head/live-snapshot/view-001
```

每次运行选择新的输出目录；非空目录会被拒绝。如需选择其他标定映射/模型配置，在模式参数之前传入 `--wrist-config PATH`；默认使用 `sp_vision/configs/wrist_config.example.json`。

## 示教模式下的验证步骤

1. 按现场既有流程，通过控制器进入示教/拖动模式。确认负载补偿与当前安装的附件相符，并保持该模式要求的手臂支撑；参见[腕部标定准备说明](../sp_vision/wrist_calib.zh-CN.md)。第一轮先固定头部。本脚本不切换模式，也不发送电机命令。
2. 先在旧版窗口按 `q` 退出，再到 DC 桌面终端运行上面的实时命令；已经运行的进程不会自动加载脚本修改。带时间戳的 `teach-...` 输出目录可以避免覆盖旧记录。保持终端开启，程序会另外显示一个带红色中心十字和 X/Y/Z 三轴的 RGB 窗口。
3. 确认 RGB 图像持续更新，并且确实来自头部相机。查看图像下方状态：`projected` 表示预测点在画面内；`outside_image` 表示在画面外；`behind_camera` 表示预测的头部相机 Z 非正；`STALE / UNSYNCHRONIZED` 表示数据不可用，投影暂时隐藏。不要把非预期卡住的实时画面当作新图像；主动进入的 `FROZEN` 模式则会保留一张配对帧用于标注。
4. 按现场示教流程缓慢将右腕相机带到头部画面中清晰、无遮挡的位置，头部保持不动。以腕部相机的 RGB 镜头附近作为粗略视觉参照；相机外壳中心或手掌中心是不同的点。
5. 到达第一个可观察姿态后停稳，并保持示教模式所要求的支撑，比较红色十字与可见镜头区域，同时检查光学坐标轴的方向，尤其是沿镜头观察方向的蓝色 Z。如果真实相机已清楚出现在图像里，状态却仍是 `outside_image` 或 `behind_camera`，也要保存这一例；它是判断不一致的有效证据。不要为了强行让画面对齐而修改标定值。
6. 点击 RGB 窗口使其获得键盘焦点。手臂停稳后按空格，在冻结图像中点击同一个可辨认的 RGB 镜头特征，再按小写 `s`。终端会打印保存目录；再次按空格恢复后再换姿态。第一张匹配帧已自动保存，因此第一次手动保存通常是 `view-002`。在允许的工作空间内，记录约 6 个静止姿态，覆盖画面左/中/右区域和两个便于操作的距离；每个姿态分别保存，第一轮始终固定头部。没有点选参照时直接按 `s` 仍能保存帧，但不会测量与所选特征的像素残差。
7. 两次静止采样之间，通过缓慢示教动作观察中心和三色坐标轴是否随可见腕部相机一起变化。在相机保持可见的前提下，增加不同腕部朝向的静止记录。停稳后再次判断对齐，将仅在运动时出现的拖尾与静止后仍存在的偏移分别记录。第一轮完成后，可以通过现有已审核的控制方式改变头部姿态做第二轮，同样保存静止观察结果。
8. 托稳手臂，按控制器的既有流程结束示教。在 RGB 窗口按 `q` 或 Esc 关闭叠加并断开订阅。关闭此窗口只停止观测；机器人的模式与保持行为仍由机器人控制器负责。

每次手动保存都会在 `/home/dc/tron2_deployment_on_dc/output/wrist-in-head/teach-<timestamp>/view-NNN/` 下记录 `color.png`、`overlay.png` 和 `projection.json`，请将这三个文件一起保留。光学中心位于镜头内部，坐标轴也是预测结果，因此这里得到的是直观的位置/朝向一致性检查，还不能测出毫米或角度单位的绝对误差。

## 记录停稳后仍存在的偏移

停稳后仍然存在、且随姿态变化的偏移，不能仅由显示拖尾解释。可能涉及所选可见参照点与光学中心不同、内参/畸变、任一相机安装外参，以及右臂/头部运动学模型。仅凭叠加图还不能确定是哪一项，采集对照时先保持标定值不变。

保持头部固定，先记录 3–5 个朝向尽量相近的静止位置，再在相机保持可见时补充不同腕部朝向。每个姿态执行：

1. 按既有示教流程停稳，在 RGB 窗口按空格。`FROZEN` 表示图像和匹配的关节记录已一起复制；它只冻结观测，不停止或改变机器人控制。
2. 每次点击同一个、明确辨认的可见镜头特征。画面会出现绿色参照十字和黄色连线；实时画面或诊断面板中的点击会被忽略。
3. 查看 `du`、`dv` 和像素距离，按 `s` 保存，再按空格恢复后换下一个姿态。

记录的向量为 `observed_pixel - predicted_pixel`，长度为 `sqrt(du**2 + dv**2)`，描述预测点与所选可见特征的距离。镜头表面或外壳中心与光学原点可能存在固定三维偏移；即使变换正确，其投影距离也可能随姿态变化。因此必须保持参照点身份一致，且光学中心等价性仍未确认。程序不应用固定平移或像素补偿。

## 双相机棋盘一致性检查

`board-check` 使用两幅 RGB 中的同一张固定实体棋盘。程序分别使用各相机自己的内参估计棋盘位姿，再结合各自图像匹配的关节与相机安装外参，比较两份 `base_Link` 中的棋盘位姿，报告毫米单位的平移差、角度单位的旋转差，以及双向角点投影的像素误差。此检查会使用腕部内参和腕部安装旋转。

棋盘须相对基座固定，两台相机都能完整看到 70 个内角点。通过现有受监督控制方式让腕部相机看向棋盘，并等待相机与手臂停稳。该模式只读取头部 RGB、腕部 RGB 和 `/joint_states`，采集一组同步数据，保存后退出，不打开图形窗口。

```bash
cd /home/dc/tron2_deployment_on_dc
BOARD_CHECK_RUN="output/wrist-in-head/board-$(date +%Y%m%d-%H%M%S)"
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --output "$BOARD_CHECK_RUN" \
  board-check --profile configs/local-robot-seed.json
```

默认棋盘为 7×10 内角点、格长 0.021 m；使用其他实测验证板时，可传入 `board-check --pattern COLSxROWS --square-m VALUE`。每路 RGB 分别匹配自己图像时刻的关节，且两幅图像时间差也须在 `--max-skew-ms` 内。时间戳仍来自 Bridge 转发，因此应在停稳后观察。

检查 `head-corners.png`、`wrist-corners.png`，两图的 `(0,0)` 必须对应同一个实体角点。普通棋盘检测可能出现 180° 反序，程序不会静默选择误差最小的顺序。确认之前，状态为 `needs_corner_identity_review`，`relative_pose_within_limits` 保持未确定。检查标号后，离线回放同一组数据：

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --output "${BOARD_CHECK_RUN}-review" \
  board-check --capture "$BOARD_CHECK_RUN" --confirm-corner-order
```

若某路需要反序才能匹配实体角点，应根据图像在该离线回放命令中添加 `--flip-head` 或 `--flip-wrist`。回放前会校验原图哈希和时间差，计算使用当前标定清单，因此可以离线检查历史图像对，无需重新连接。

| 保存文件 | 含义 |
|---|---|
| `head-color.png`, `wrist-color.png` | 配对的原始 RGB 图像 |
| `head-corners.png`, `wrist-corners.png` | 用于核对对应关系的角点编号 |
| `head-cross-projection.png`, `wrist-cross-projection.png` | 另一相机预测的角点为红色，黄色线段表示投影残差 |
| `board-check.json` | 各自相机的 PnP RMS、两份基座系棋盘位姿、平移/旋转差、交叉投影 RMS/最大值、时间戳、反馈和哈希 |

若棋盘缺失或检测被拒绝，仍保存原图和检测原因，但不报告棋盘位姿测量值。显式确认角点身份后，才按现有 15 mm / 2° 门槛比较相对位姿。这项检查描述两条链的相对一致性，不能据此确定绝对 URDF 精度，不启用运动，也不更新标定；`accuracy_verified` 始终为 `false`。

2026-11-10 最初的可用性探测已成功读取两路图像：头部检测到 70 个角点，腕部 RGB 则朝向地面，没有检测到棋盘。图像时间差为 12 ms，头部/腕部状态时间差分别为 4 ms / 6 ms，因此该次探测未产生双相机棋盘位姿差。参见[头部视图](../output/wrist-in-head/board-availability-20261110/head-corners.png)、[腕部视图](../output/wrist-in-head/board-availability-20261110/wrist-corners.png)、[探测记录](../output/wrist-in-head/board-availability-20261110/board-check.json)。合成检查已验证能检出已知的 20 mm 平移与 5° 旋转差异，并覆盖离线回放和订阅清理。

使用历史 19:02 头部参数时，后续 `board-20261110-201025` 采集在两台相机中都检测到 70 个角点。独立二维校正后的不对称红胶带排列一致，确认了实体角点对应关系，两路都不需要反序。各自相机的 PnP RMS 约为 0.103 px / 0.162 px，交叉投影 RMS 则为 9.351 px / 5.926 px。确认后的相对棋盘位姿差为 17.308 mm、2.069°，略高于配置的 15 mm / 2° 门槛，因此离线复核返回 `relative_pose_exceeds_limits`；这是相对一致性结果，并非程序崩溃。

使用该 19:02 版本，在关节记录基本不变时又只读采集了三组，差异可重复出现。原始观察和重复观察经复核后，平移范围为 17.292–17.348 mm，旋转范围为 2.044–2.069°。这支持存在可重复的相对几何不一致，超出了可见镜头/内部光学原点参照不明这一因素；但仍不能确定具体哪台相机或哪一段 FK 有误，也不能据此确定绝对实物精度。这些检查中没有修改参数。后续修正需要不同姿态的双机棋盘数据及独立留出验证，不能根据单个姿态直接加补偿。

记录：[角点身份核对图](../output/wrist-in-head/board-20261110-201025/corner-identity-review.png)、[复核结果](../output/wrist-in-head/board-20261110-201025-reviewed/board-check.json)、[重复性汇总](../output/wrist-in-head/board-20261110-201025-analysis/summary.json)、[重复性图](../output/wrist-in-head/board-20261110-201025-analysis/repeatability.png)。

## 第二轮：固定右臂，改变头部

第一轮是头部固定、右臂变化。这一轮反过来，有助于区分随头部变化的误差与右臂/基座链中的共同误差。按控制器既有、受监督的保持方式稳定右臂；整轮保持相同的腕部位置、朝向和相机安装状态。

在旧窗口按 `q` 退出，再用独立输出目录启动：

```bash
cd /home/dc/tron2_deployment_on_dc
.venv/bin/python tools/hardware/wrist_in_head_overlay.py \
  --calibration-summary configs/local-camera-calibration-latest.json \
  --axis-length-m 0.08 \
  --output "output/wrist-in-head/head-check-$(date +%Y%m%d-%H%M%S)" \
  live --profile configs/local-robot-seed.json
```

1. 让右腕 RGB 镜头清楚出现在头部画面中，先保存一组原位对照。
2. 通过现有已审核的头部控制，在允许范围内小幅改变头部姿态。目标约为 5 组：原位、稍左、稍右、稍上、稍下，并始终让同一个镜头保持可见。
3. 每个头部姿态停稳后，按空格、点击同一个实体 RGB 镜头中心、按 `s`，再按空格恢复。
4. 改变头部时，右臂始终固定；镜头在图像中的位置随头部变化是正常现象。如果右臂动了，重新稳住后补这一组；后续可以用保存的关节记录核对右臂姿态的一致性。
5. 按控制器的既有流程结束操作，再按 `q` 关闭叠加。

脚本已能根据匹配的实测头部关节更新头部 FK。这轮提供诊断对照，标定文件保持原值；仍需检查新记录，才能进一步区分两条链，不能预先认定哪份标定有误。

## 同步、可见性与保存记录

程序按图像的 Bridge 时间戳匹配带名称的实测关节。默认要求图像/状态时间差不超过 30 ms，两个数据流的本机接收年龄均不超过 750 ms。这些是 Bridge 转发时间戳，不能据此确立硬件曝光同步。重复或乱序的时间戳不会刷新缓存。反馈缺失、过期或不同步时隐藏投影；连续 8 s 没有可用配对时，程序报错退出。如需调整，在模式参数之前设置 `--max-skew-ms`、`--max-age-ms` 或 `--timeout`，并随结果保留这些设置值。

中心位于头部相机后方或画面外时，中心和三轴均隐藏。中心可见时，位于相机后方的轴端点单独隐藏；画面外的端点在绘制时裁剪到 RGB 边界。完整端点坐标仍保存在 JSON 中。极端大的投影会保留坐标，但不绘制可能溢出的线段。坐标轴不会延伸到诊断面板中。mock 和回放结果都有明确来源标签；所有模式下 `accuracy_verified` 均保持 `false`。

每个保存的 `view-NNN/` 包含：

| 文件 | 内容 |
|---|---|
| `color.png` | 原始头部 RGB，或 mock 背景 |
| `overlay.png` | 预测中心与三色 X/Y/Z 坐标轴、基座 XYZ、像素坐标、头部相机 Z、时间差和轴长 |
| `projection.json` | 带名称的关节、时间戳、预测中心和轴端点、轴裁剪/可见性、相机到基座及相机间相对变换、旋转/四元数、标定/模型路径与哈希、图像哈希 |

点选后，`manual_reference` 会保存观察到的 `pixel_uv`、原始 `image_stamp_ms`、`observed_minus_predicted_px` 和 `distance_px`；`display_frozen` 标记主动冻结的帧。实时采集还保留 `image_receipt_age_ms`、`joint_receipt_age_ms` 和 `newest_image_gap_ms`，用于描述本机接收/缓存时序，不能当作硬件曝光年龄。回放保留所选参照点，并相对重新计算的预测点重算残差。

姿态字段为 `T_head_wrist_camera`、`R_head_wrist_camera` 和 `orientation_head_quaternion_xyzw`，描述腕部光学坐标系相对头部光学坐标系的姿态。`axis_length_m` 记录选用的轴长；`axes.X`、`axes.Y`、`axes.Z` 保存各自的三维端点、投影 `pixel_uv`、`drawable`、`clipped` 和绘制用的 `segment_uv`。回放会根据当前 `--axis-length-m` 重新计算三轴，也支持旧版只记录中心的采集文件。

实时运行还会生成 `run.json`，记录处理/保存帧数、`reference_captures`、时间限制、运行时长及运行错误。回放会校验 `color.png` 与记录的哈希一致。输出应放在 Git 忽略的 `output/` 目录中。这些记录用于审核位置叠加效果，不能据此确立实物精度或授权机器人运动。

## 2026-11-10 本机验证记录

当前完整测试共 273 项通过，包含模拟 GUI 冻结/点选/保存与参照点回放检查；打包 mock demo 完成，记录 `hardware_commanded: false`。此前三轴版本进行了一次只读检查，保存 3 帧真实数据，Bridge 图像/状态时间差为 0–13 ms，随后断开订阅。最后一帧的中心为 `(353.203, 292.827)`，头部相机坐标系中的 Z 约为 0.5327 m。中心和三个 0.08 m 轴端点均在 640×480 画面内，已实际绘出。后续两轮手动参照点记录的分析见下文；独立的实物位置/旋转精度仍未验证。

记录：[实时三轴叠加图](../output/wrist-in-head/axes-live-20261110/view-003/overlay.png)、[实时投影 JSON](../output/wrist-in-head/axes-live-20261110/view-003/projection.json)、[实时运行记录](../output/wrist-in-head/axes-live-20261110/run.json)、[mock 三轴叠加图](../output/wrist-in-head/axes-mock-20261110/view-001/overlay.png)。

## 2026-11-10 两轮偏移诊断

右臂变化会话 `offset-20261110-161533` 包含对 5 张独立图像的 6 次标注；`view-006` 与 `view-007` 标注的是同一帧，分析采用最后一次标注。头部保持固定，腕部朝向相对第一组最多变化 21.2°；预测点与所选 RGB 镜头特征的原始 RMS 距离为 43.38 px。

头部变化会话 `head-check-20261110-163647` 包含 5 张点选图像，其中有一次回到原始头部姿态的重复观察。记录中右臂关节最大变化约为 0.00020 rad；按当前 FK 模型计算，腕部相机位姿最多变化 0.098 mm、0.016°。这些数值描述反馈/模型计算的一致性，并非独立测得的实物精度。头部朝向相对第一张点选图像最多变化 7.14°。

以下诊断参数全部由第一轮估计，在预测第二轮时保持固定，没有用第二轮点选数据重新拟合。

| 在第二轮评估的模型 | 与所选可见特征的 RMS 距离 |
|---|---:|
| 原始叠加 | 34.98 px |
| 头部相机坐标系中的平移假设 | 3.66 px |
| 右臂/基座共同平移假设 | 9.11 px |
| 腕部相机坐标系中固定的可见特征偏移 | 19.53 px |

这一比较支持优先复查头部相机安装变换与头部刚性运动学链。但尚不能唯一确定哪一项有误，内参误差、模型误差、可见镜头特征与光学原点的差异仍可能贡献偏移。标定文件保持原值；应使用独立的已知几何目标验证后，才能考虑将诊断拟合量作为标定修正。旋转精度也仍未独立验证。

记录：[第一轮汇总](../output/wrist-in-head/offset-20261110-161533-analysis/summary.json)、[第二轮汇总](../output/wrist-in-head/head-check-20261110-163647-analysis/summary.json)、[第二轮逐帧 CSV](../output/wrist-in-head/head-check-20261110-163647-analysis/per_view.csv)、[独立第二轮模型对比图](../output/wrist-in-head/head-check-20261110-163647-analysis/second-round-comparison.png)。

## 头部内参采集方法

此前的 `head-recalibration-20261110-190209` 会话已经完成。操作人员确认采集时棋盘固定、头部变化，因此该组数据同时用于内参与外参拟合。下述方法保留供以后采集独立内参数据使用；移动棋盘的数据必须与固定棋盘的手眼外参数据分开。

原头部会话的 40 张图中，检测到的内角点区域仅占画面约 1.7%–2.0%。成像大小变化有限，因此应先改善采集覆盖，再决定外参如何修正；不能仅凭面积就断定标定失败。这次固定头部，移动平整棋盘，使其具有不同画面位置、距离和倾角。内参标定允许移动棋盘；手眼外参数据要求棋盘相对基座固定，须分开处理。

已准备本机[内参采集配置](../output/head-recalibration-prep-20261110/head-intrinsics-capture.json)，沿用 7×10 内角点与 0.021 m 方格边长。`min_board_area_ratio` 现为 0.015，因此清晰、完整且面积为 0.017 的图像可以作为远景样本保留。最初设置的单张 0.10 硬门槛过严，现已取消；0.10–0.30 是部分较大视角的采集建议，不是统一的标定验收门槛。模型/profile 路径已显式解析，原配置、求解阈值和标定文件保持原值。已离线检查一张面积约为 0.017 的旧图，它能通过调整后的基本检测门槛。

本轮标内参，棋盘不必留在桌面上，可以用硬板支撑后垫高或拿近，前提是角点完整、清晰。如果只能放在桌面高度，也可以使用更大的实体棋盘，但须先实测方格边长并更新棋盘配置。采集目标是不同的图像位置、大小和倾角，可参考 [ROS 标定教程](https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/doc/tutorial_mono.rst#moving-the-checkerboard)。允许个别远景图，但整组仍应提供这些变化。

在 DC 桌面终端运行：

```bash
cd /home/dc/tron2_deployment_on_dc
HEAD_CALIB_RUN="output/head-recalibration-$(date +%Y%m%d-%H%M%S)"
.venv/bin/python sp_vision/calibration.py \
  --config output/head-recalibration-prep-20261110/head-intrinsics-capture.json \
  capture --pattern 7x10 --square-m 0.021 \
  --session "$HEAD_CALIB_RUN/intrinsics-session" --append --count 40
```

按现有控制器流程稳定手臂和头部，本轮只移动棋盘。在可操作的条件下补充较大的视角，完整、清晰地看到 70 个角点；部分图像可争取 `area` 约为 0.10–0.30，同时保留 0.017 等有用的远景图。覆盖图像中心和周边、几个不同距离以及不同倾角。每次摆好棋盘后按 `r` 获取新预览，再按 `s` 保存通过检查的视角。保存满 40 张后程序自动结束；这里使用的是标定采集窗口按键，`r` 用于刷新图像。

配置只在程序启动时读取。若已保存计数仍为零，可以退出并重启，使调整后的门槛生效。上面命令使用 `--append`，每张通过检查的样本会立即保存；`--count 40` 表示本次新增 40 张，续采已有会话时应填写剩余张数。若非追加采集已经暂存了图像，应先保留或完成该轮采集再重启，因为中途退出会丢弃本轮新暂存的图像。

采集后先拟合并检查新头部内参，再复用已有头部外参图像和同步头姿，独立划分外参训练/留出数据进行重解，随后离线回放两轮诊断记录。是否补采外参数据由这些结果决定。本次准备不应用诊断偏移，也不替换当前使用的标定。

## 历史 19:02 头部解算与实际参数回放

已检查 40 张采集图完整，40 个 RGB 哈希均不相同，图像尺寸均为 640×480，棋盘元数据一致。头部关节跨度约为 0.3575 rad、1.3346 rad。新内参和外参均通过现有拟合门槛，无拒绝视角或离群样本，使用 34 张训练、6 张留出图像。

| 19:02 头部指标 | 结果 |
|---|---:|
| 内参重投影 RMS | 0.096475 px |
| 训练集最大平移 / 旋转残差 | 9.117 mm / 1.987° |
| 留出集最大平移 / 旋转残差 | 4.044 mm / 1.551° |

随后使用棋盘数据实际解出的新参数，对之前的 RGB、反馈和镜头点选记录进行了回放；没有利用手动参照点拟合诊断平移或其他修正量。

| 手动参照轮次 | 原版 RMS | 19:02 RMS | 19:02 最大值 |
|---|---:|---:|---:|
| 右臂变化、头部固定 | 43.38 px | 17.99 px | 32.64 px |
| 头部变化、右臂固定 | 34.98 px | 6.38 px | 10.05 px |

残差已有改善，但尚未归零；这些数值描述与可见镜头特征的距离，不能直接确定光学中心或旋转的实物精度。上文历史诊断假设与这里的实际参数比较相互独立。回放记录仍位于 `output/head-recalibration-20261110-190209/`；其中两份头部工作参数文件后来被你提供的另一组参数覆盖，历史清单记录的是原 19:02 哈希，不能加载这些已覆盖的工作文件。

19:02 回放时的工具通过 269 项测试和打包 mock demo；另一次只读实时检查加载了该版标定，保存 3 帧，Bridge 图像/状态时间差为 0–4 ms。该次采样时腕部预测中心位于画面下方，因此实时检查验证了数据加载和传输，实际与可见参照的对比则由回放记录提供。

记录：[历史清单](../output/head-recalibration-20261110-190209/camera-calibration-summary.json)、[回放比较 JSON](../output/head-recalibration-20261110-190209/replay-comparison.json)、[比较图](../output/head-recalibration-20261110-190209/replay-comparison.png)、[历史实时加载记录](../output/wrist-in-head/latest-head-live-20261110-190209/run.json)。当前参数路径与验证测量见[标定结果](camera-calibration-results-20261110.zh-CN.md)。
