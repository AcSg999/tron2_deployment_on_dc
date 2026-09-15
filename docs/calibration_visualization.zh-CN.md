# 标定可视化采集引导

[English](calibration_visualization.md) · [标定流程](calibration.zh-CN.md) · [README](../README.zh-CN.md)

在**标定过程中**使用可视化辅助页：看清标定板、采集合格样本，并根据提示完成下一步。通过 CLI 启动，在浏览器中预览、采集和求解。结果保持简洁：保存文件、下一步操作，以及内参 RMS 或手眼样本数。独立测量点仍用于最终验收。

## 1. 启动内参辅助页

按照[标定流程](calibration.zh-CN.md)准备相机配置和实测标定板尺寸。在已配置相机环境的终端中，从仓库根目录运行：

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-seed.json --stage intrinsics \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/intrinsics-guided
```

打开终端显示的地址，通常为 `http://127.0.0.1:8790`。每次会话目录必须不存在或为空。`--pattern` 是内角点数量；`--square-m` 是实测方格边长，单位为米。可用 `--port` 指定其他端口。

辅助页启动时不会打开浏览器或访问硬件。**查看画面** 显式读取一帧相机图像，并叠加检测到的标定板角点。**重新采集并保存样本** 会重新获取并检查新帧，仅保存合格数据，不会复用预览。页面显示合格样本数及下一步操作。

## 2. 采集不同视角并求解

1. 预览标定板。保持整块板在画面内，确认标记角点落在棋盘交点上。
2. 采集一个合格视角。将标定板移到其他画面区域，改变距离和倾角，同时保持头部不动。
3. 重复采集，至少取得五个合格视角后选择 **计算结果**。五个只是下限，不代表覆盖范围充分。

结果保存在 `calibration_data/intrinsics-guided/intrinsics.json`，原始图像位于 `view-*/color.png`。查看简洁的拟合误差及下一步提示。按 Ctrl-C 停止辅助页，然后应用该文件：

```bash
.venv/bin/tron2-deploy apply-intrinsics \
  --profile configs/local-robot-seed.json \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --output configs/local-robot-intrinsics.json
```

较小的重投影 RMS 表明拟合能解释这些检测到的图像点，不代表相机到基座变换准确，也不代表深度对齐正确。应用内参会使已有外参验收失效，并保持实机执行关闭。

## 3. 重新启动并采集手眼样本

将标定板刚性固定在所选腕部。使用更新后的内参配置重新启动：

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/local-robot-intrinsics.json --stage handeye --side left \
  --pattern 9x6 --square-m 0.025 \
  --output calibration_data/handeye-left-guided
```

打开终端显示的地址。通过机器人另外经过审核的定位界面调整手臂，待其停止后预览并采集。保持标定板安装关系和头部姿态不变，改变腕部旋转与位置，重复采集。辅助页只读取图像和反馈，不会控制机器人运动。

求解前至少采集五个合格样本，且腕部旋转跨度至少为 15 度。采集被拒绝时，合格样本数不会增加。样本保存在会话目录中的 `samples/*.json`，求解结果为 `handeye-left.json`。采集右腕样本时使用独立目录和 `--side right`，不要混合左右样本。

求解结果仍处于**未验证**状态。按 Ctrl-C 停止辅助页，测量独立保留点，然后完成[验证与应用](calibration.zh-CN.md#使用保留点验证并应用结果)。该步骤会重新检查点误差，通过后才写入已标定配置。规划操作页随后读取这份配置，标定时无需运行规划服务。

## 辅助页无法继续时

| 提示或现象 | 下一步 |
| --- | --- |
| 未找到标定板或角点不完整 | 检查内角点数量、光照与对焦。让整块标定板进入画面，再次预览。 |
| 图像过期或时间戳不一致 | 检查相机数据流和时钟同步，重新采集新帧。 |
| 手臂或头部正在移动 | 等待稳定；重新采集前恢复配置中的固定头部姿态。 |
| 合格样本不足 | 继续采集不同视角；失败的采集不计入数量。 |
| 视角或腕部姿态与已保存样本过于相似 | 换到明显不同的视角或腕部姿态后再采集。 |
| 腕部旋转跨度不足 | 调整腕部，增加不同方向的旋转，保持标定板可见并重新采集。重复同一姿态没有帮助。 |
| 误差较小但拟合看起来不对 | 检查标定板尺寸、图像覆盖范围及刚性安装关系；必要时使用下方可选诊断。 |

## 无硬件试用页面

```bash
.venv/bin/tron2-deploy calibration-guide \
  --profile configs/demo.json --stage intrinsics --mock \
  --pattern 9x6 --square-m 0.025 \
  --output output/calibration-guide-demo
```

模拟模式明确标注合成数据，无需相机或机器人即可演示预览、采集和求解，不能作为真实标定验收依据。另一次会话应使用新的目录。

## 可选诊断

拟合或独立点检查需要进一步排查时，使用 `calibration-report`。离线报告默认显示简短摘要，详细图表折叠显示，按需展开。完成可视化引导采集不要求生成报告。

```bash
.venv/bin/tron2-deploy calibration-report \
  --intrinsics calibration_data/intrinsics-guided/intrinsics.json \
  --output output/intrinsics-review

xdg-open output/intrinsics-review/index.html
```

排查手眼问题时，传入引导会话的样本目录，以及按照[标定流程](calibration.zh-CN.md)准备的独立测量点：

```bash
.venv/bin/tron2-deploy calibration-report \
  --handeye calibration_data/handeye-left-guided/handeye-left.json \
  --samples 'calibration_data/handeye-left-guided/samples/*.json' \
  --validation-points calibration_data/heldout_points.npz \
  --max-error-m 0.005 \
  --output output/handeye-review

xdg-open output/handeye-review/index.html
```

示例允许最大点误差为 5 mm；实际阈值应根据部署的测量与间隙预算确定。验证失败时仍会写入报告，并以状态码 `1` 退出，因此即使检查失败也应打开报告。没有独立测量点时，结果仍为未验证状态。拟合一致性不能单独证明通过验收。

报告使用已保存文件，无需相机、ROS、GPU 服务或运行中的前端。将 `index.html`、`metrics.json` 和导出的图表与输入数据一起保存；每个报告目录必须不存在或为空。辅助页和报告都不会修改配置或启用执行。
