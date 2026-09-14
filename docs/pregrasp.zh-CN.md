# 感知物体位姿的预抓取规划

[English](pregrasp.md) · [README](../README.zh-CN.md) · 上一步：[部署](deployment.zh-CN.md) · 下一步：[操作执行](operation.zh-CN.md)

任务在所选腕部到达经过碰撞检查的**预留距离位姿**后结束。FoundationPose 提供物体坐标系，物体几何和接近约束决定腕部的位置与方向。此流程不包含接触、抓取或夹爪开合。

## 定义物体和腕部接近方式

所有平移量均以米为单位。`pose7` 数组格式为 `[x,y,z,qw,qx,qy,qz]`，四元数必须归一化。观测必须表达在 `base_Link` 中。配置已注册物体的 `vision.mesh_id`、保守包围物体的 `scene.object_radius_m`、实测 `scene.table_z_m`，以及正值 `scene.clearance_m`。

每个所选腕部分别使用 `pregrasp.left` 或 `pregrasp.right` 配置：

| 字段 | 含义 |
| --- | --- |
| `wrist_to_tcp_pose7` | 实测 TCP 坐标系在配置的腕部刚体坐标系中的位姿。 |
| `symmetry` | `axial` 表示绕指定物体向上轴对称；`none` 表示采用明确的物体相对接近方式。 |
| `up_axis_object` | 物体局部的向上/滚转参考轴，默认为 `[0,0,1]`。 |
| `standoff_m` | 从锚点沿向外接近方向留出的正距离。 |
| `lift_clearance_m` | 在初始与目标 TCP 高度较大值之上增加的非负抬升高度。 |
| `radius_m`、`axial_offset_m` | 用于 `axial`：表面锚点半径，以及沿物体向上轴的高度。 |
| `azimuth_hint_base` | 可选的 `axial` 基座坐标系向外方向；否则由当前实测 TCP 位置推导。 |
| `anchor_object`、`outward_axis_object` | 用于 `none`：物体局部锚点和向外方向，均为三维向量。 |

对于轴对称碗，规划器先将物体向上轴旋转到基座坐标系，再将当前 TCP 相对物体的偏移投影到径向平面。该径向方向决定接近侧。因此，物体绕对称轴的偏航变化不会强制腕部随之旋转。物体倾斜仍会改变向上轴和目标几何。也可以通过明确的方位提示选择其他接近侧。

TCP `+Z` 朝向物体，与向外方向相反。TCP `+Y` 遵循物体向上轴在垂直接近方向平面中的投影。当该投影退化时，使用当前实测 TCP 坐标轴提供滚转参考。对于 `symmetry=none`，锚点和向外方向随物体旋转，但腕部方向仍由这些接近轴约束确定。

```text
T_base_object = T_base_camera @ T_camera_object
p_base_pregrasp = p_base_anchor + standoff_m * outward_base
T_base_wrist = T_base_tcp @ inverse(T_wrist_tcp)
```

锚点是几何参考，不是会下发给机器人的接触位姿。仅设置 `standoff_m` 不能保证间隙：整个实际安装附件和两条手臂都必须通过模型检查。

## 采集、估计与规划

使用完成标定的部署配置启动精简工作台：

```bash
python -m tron2_deployment.cli operator \
  --profile configs/local-robot-calibrated.json \
  --host 127.0.0.1 --port 8787
```

打开 `http://127.0.0.1:8787/`。启动时只读取配置；明确执行操作后才会访问相机、视觉服务或状态接口。

1. 采集顶部 RGB-D 图像，并框选配置的物体。
2. 生成并检查 SAM 遮罩，然后运行 FoundationPose。
3. 选择左腕、右腕或双腕。点击**读取状态并规划**；服务会在规划前获取新鲜的实测手臂和头部状态。
4. 检查返回的检查结果和目标。服务将不可变计划保存为 `output/pregrasp_<id>.json`，显示实际路径与 RViz 命令，并提供 JSON 下载。

重新采集、修改遮罩或重新估计位姿，会使依赖它们的结果失效。改变所选腕部会使当前显示的计划失效。修改部署配置或模型后，需要重启服务并重新观测、规划。实机规划会拒绝配置/标定/网格来源不匹配、置信度过低或无效、采集或状态时间戳过期，以及与采集或标定时不一致的头部姿态。

如果已经有一个序列化且仍有效的 `vision.estimate()` 观测，CLI 可以直接读取新鲜状态。以下命令只进行规划：

```bash
python -m tron2_deployment.cli plan \
  --profile configs/local-robot-calibrated.json \
  --observation output/observation.json --state live \
  --sides left right --output output/pregrasp-plan.json
```

完整合成演示可以使用 `operator --profile configs/demo.json --mock`，或[部署文档](deployment.zh-CN.md)中的 `demo` 命令。合成观测和简化机器人不能授权实机执行。

## 理解规划路径与检查项

规划器只使用一条确定的通道：先抬升 TCP，再在目标上方平移并调整方向，最后下降到预留距离位姿。连续 IK 以上一解和实测关节作为初值。两个所选腕部共用时间轴；未选择的手臂和头部保持不动。如果该通道受阻或不可达，规划会失败，不会输出可执行的部分计划。规划器不会全局搜索绕过障碍物的替代路线。

每一段使用 `quintic_stop` 插值：

```text
s(u) = 10*u^3 - 15*u^4 + 6*u^5,  0 <= u <= 1
q(u) = q_start + s(u) * (q_end - q_start)
```

每一段边界处的关节速度和加速度均为零。各段时长满足配置的逐关节速度与加速度限制。终点数值 IK 容差为位置 1 mm、方向 0.01 rad；实机测量精度需要单独验收。

MuJoCo 数值查询检查 FK、关节范围、模型自碰撞和双臂碰撞、桌面半空间，以及包围物体的球体。采样同时评估双臂，并根据连杆偏移、关节范围、附件包围半径和配置间隙，约束采样点之间的几何运动。部署模型中的碰撞掩码和排除项仍是检查依据，必须正确描述实际安装硬件。MuJoCo 不作为额外可视化器或操作审核步骤。

计划文件包含左右两组 `N x 7` 手臂数组、`N x 2` 固定头部位置、`times_s`、所选腕部、TCP/腕部目标、冻结的观测与初始状态、配置哈希、编译后模型哈希、数值检查结果，以及由内容生成的 `plan_id`。重新验证会重新计算目标并检查路径；不能通过编辑 JSON 来调整已经审核的轨迹。

## 在 RViz 中审核同一份计划

使用工作台显示的命令及其实际服务端计划路径。对于上面 CLI 生成的文件名：

```bash
python -m tron2_deployment.rviz \
  --profile configs/local-robot-calibrated.json \
  --plan output/pregrasp-plan.json \
  --ros-master-uri http://127.0.0.1:11331
```

RViz 使用独立的本地可视化 ROS master。它显示冻结的物体坐标系和包围球、预抓取目标坐标轴、FK 腕部路径，以及规划关节。实机配置必须提供匹配的 `robot.urdf`，包含实际安装附件。预览与执行器使用相同的五次插值和时间轴。`--once` 播放一次后保持最终位姿；`--no-gui` 只运行发布器而不打开 RViz；`--no-start-master` 使用已经运行的可视化 master。

审核整条路径、双臂间隙、工具接近轴、目标预留距离，以及最终停止位姿。RViz 审核不会发送机器人指令。随后携带准确的已审核计划 ID，继续[操作执行](operation.zh-CN.md)。
