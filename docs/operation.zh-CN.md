# 审核后的预抓取执行

[English](operation.md) · [README](../README.zh-CN.md) · 上一步：[预抓取规划](pregrasp.zh-CN.md)

执行器将所选腕部移动到已审核的预抓取预留距离位姿，并验证到达状态。它只发送手臂和头部目标。标定时的头部姿态和未选择的手臂保持不动；到达后不会继续执行接触或夹爪指令。

## 先用模拟机器人验证

运行完整的合成流程：

```bash
python -m tron2_deployment.cli demo --output output/demo
```

它会在 `output/demo` 中写入合成标定证据、预抓取计划、执行 JSONL 日志和结果。简化模型用于测试软件行为；其测量、几何和标定不能代表实际 TRON2 安装状态。

交互测试可启动模拟工作台：

```bash
python -m tron2_deployment.cli operator \
  --profile configs/demo.json --mock \
  --host 127.0.0.1 --port 8787
```

依次完成采集、分割、估计和规划。在 RViz 中审核显示的计划，然后勾选审核框并启动模拟执行。浏览器显示执行状态，并提供模拟停止按钮。浏览器中的执行和停止仅适用于模拟机器人；实机执行使用下方 CLI。

## 验收实际安装配置

启用实机执行前，完成以下针对当前安装的检查：

1. 确认只读 D435 与关节反馈新鲜、相机标识匹配、时钟同步，并保持记录的固定头部姿态。
2. 完成[标定](calibration.zh-CN.md)，独立检查左右腕部/TCP 安装关系，并确认已注册物体几何和实测桌面坐标。
3. 检查数值模型与 RViz URDF 是否匹配实际机器人及被动附件。核对所有关节映射、限制、碰撞掩码/排除项，以及附件间隙。
4. 在监督下验证控制器的短时保持行为和物理急停流程，记录实际行为、时序与故障处理结果。
5. 实机运动验收先使用单腕并留出充分的物体间隙，再测试另一腕，最后测试双腕。每次都审核新生成的计划，并记录实测到达与中断结果。

本仓库提供自动化和合成验证。仓库迁移本身不会执行或认证上述实机检查。

计算实际数值模型及其编译资产的标识：

```bash
python -m tron2_deployment.cli model-hash \
  --profile configs/local-robot-calibrated.json
```

完成相应物理检查后，将返回值记录到 `robot.model_hash`，保留验收证据，并准备 `configs/local-robot-accepted.json`，其中设置 `calibration.source="real"`、`calibration.verified=true`、`execution.allow_real=true` 和 `execution.hold_behavior_verified=true`。这些值用于记录已经完成的验收；修改它们本身不能证明物理检查通过。

**在观测和规划之前冻结已验收配置。** 使用 `configs/local-robot-accepted.json` 重启工作台，重新获取观测、生成计划，并使用同一份已验收配置进行审核。规划后修改任何配置值都会改变配置哈希，使该计划失效。`apply-calibration` 会有意将实机执行重置为关闭。

## 执行准确对应的已审核计划

使用已验收配置和工作台显示的实际计划路径。以下示例假定审核后的计划已保存为 `output/pregrasp-plan.json`。先检查其标识和最终目标：

```bash
python - <<'PY'
import json
from pathlib import Path
plan = json.loads(Path('output/pregrasp-plan.json').read_text())
print(json.dumps({key: plan[key] for key in
                 ('plan_id', 'profile_hash', 'selected_sides', 'targets')}, indent=2))
PY
```

在 RViz 中审核这份准确的计划后，输入完整的 `plan_id` 并运行受监督的执行命令。每次尝试都使用新的日志文件名：

```bash
read -r -p 'Reviewed plan_id: ' TRON2_REVIEWED_PLAN_ID
python -m tron2_deployment.cli execute \
  --profile configs/local-robot-accepted.json \
  --plan output/pregrasp-plan.json --real \
  --reviewed-plan-id "$TRON2_REVIEWED_PLAN_ID" \
  --confirm-supervised --log output/execution-001.jsonl
```

命令会先检查计划完整性、配置/模型标识、实机观测来源、标定与头部一致性、观测时效，以及新鲜的起始反馈。它还会独立重新检查目标合成、双臂联合碰撞、关节限制，以及与规划和 RViz 播放一致的五次插值速度/加速度边界。耗时验证完成后会再次读取反馈，在授权下发目标前拒绝状态变化。

传输层按照已接受的速率发布 `[left7, right7, head2]`，频率至少为 500 Hz。反馈采用明确映射；机器人原始状态中的被动夹爪条目不是手臂关节。执行器监控反馈时效、头部移动、跟踪误差、发布时间和到达状态。默认配置阈值可在 `execution.DEFAULT_EXECUTION` 中查看；应在规划前确定当前安装的限制，因为修改它们会使配置哈希失效。

成功完成要求连续多个新鲜反馈样本处于最终关节容差内。检查结果和 JSONL 证据，包括规划与实测状态、时序、跟踪误差和到达结果。传输发送成功不等于已验证到达。实际 TCP 位置与方向精度需要在实机验收中测量。

## 停止与恢复

在**执行命令所在终端按 Ctrl+C** 请求停止。停止请求或执行故障会阻止路径继续推进。条件允许时，执行器读取新鲜实测关节，并在该位置短时发送保持指令；反馈不可用时，使用最后一个已经验证的目标。默认保持时长为 0.1 s，且不会验证保持动作完成。

这种软件保持不是电机失能指令，也不保证实现急停。其实际行为取决于已验收的控制器配置。需要立即干预时，使用当前安装已经验证的物理急停装置。浏览器中的模拟停止按钮不能停止另一个独立的实机 CLI 进程。

日志记录拒绝、失败、停止和保持错误，不会覆盖已有日志文件。发生中断后，检查机器人及记录的反馈，必要时恢复标定头部姿态，重新获取观测，并从当前实测状态生成和审核新计划。不要从假定的路径点继续，也不要复用起始状态、物体位姿或标定已经不匹配的计划。
