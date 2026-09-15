# DC 上的 TRON2 部署

[English](README.md)

本仓库提供独立的**基于物体位姿的双腕预抓取**部署工具：定位物体，选择可达的接近方向与腕部姿态，在 RViz 中审核轨迹，运动到预留间距处并停止。应用不发送夹爪指令，也不执行接触或抓取动作。

物体位姿用于确定目标几何关系，而不是直接复制物体四元数。轴对称物体根据当前腕部位置选择径向接近方向，因此无实际意义的物体偏航不会改变目标。非对称物体使用配置的物体系锚点和向外方向。TCP 的 +Z 指向物体，+Y 沿投影后的物体向上轴；再用已标定的腕部到 TCP 变换换算腕部目标。

| 阶段 | 文档 | 输出 |
| --- | --- | --- |
| 1. 标定 | [标定](docs/calibration.zh-CN.md) · [可视化采集引导](docs/calibration_visualization.zh-CN.md) | 引导采集图像与样本、内参与手眼求解、独立验证和已验收部署配置 |
| 2. 部署 | [DC 设备：Ubuntu 20.04](docs/deployment.zh-CN.md) | 已安装运行时、相机/视觉/机器人连接配置和精简操作页 |
| 3. 规划与执行 | [预抓取流程](docs/pregrasp.zh-CN.md) | 已检查双臂计划、RViz 审核、执行前检查与实测日志 |

已核实 DC 主机运行 x86_64 Ubuntu 20.04.6，使用 glibc 2.31，另行安装了 Python 3.10.13 和 ROS Noetic。沿用该设备的现有环境，保留 Ubuntu 系统 Python 3.8。采集真实标定数据之前，先安装项目环境；解释器选择与 ROS 检查见[设备环境配置](docs/deployment.zh-CN.md)。两种语言的文档应保持同步。

## 运行完整模拟流程

```bash
git clone https://github.com/Shukashuki/tron2_deployment_on_dc.git
cd tron2_deployment_on_dc
bash scripts/install.sh
.venv/bin/tron2-deploy demo --output output/demo
.venv/bin/tron2-deploy operator --profile configs/demo.json --mock
```

打开 `http://127.0.0.1:8787`。演示使用合成 RGB-D、简化双臂模型和模拟反馈，覆盖验证、估计、基于接近方向的目标生成、IK、碰撞/限位检查、定时运动与日志。它不代表真机已就绪。真机执行必须通过显式 CLI 操作；浏览器仅执行模拟计划。

通过 CLI 的 `tron2-deploy calibration-guide` 启动标定。浏览器页面帮助用户看清标定板、采集合格视角、查看样本进度并求解。结果仅显示保存文件、下一步，以及内参 RMS 或手眼样本数；独立验证后，仍通过现有 CLI 命令应用结果。详细离线报告作为可选的排查工具。参见[可视化采集引导](docs/calibration_visualization.zh-CN.md)。

## 实现与限制

规划器沿抬升—转移—下降通道生成路径，IK 或碰撞检查失败时拒绝计划，不进行任意障碍绕行搜索。物体由配置的保守包围球表示。MuJoCo 仅进行 FK/IK 和碰撞数值计算；**轨迹通过 RViz 查看**。部署模型必须对应实际固定基座及已安装附件，仅保留映射的 14 个臂关节和 2 个头部关节可动。未控制的附件保持固定，并保留碰撞几何。

计划绑定物体观测、标定/配置指纹、编译后模型哈希、初始反馈、目标位姿及确切时间插值。真机执行要求新鲜的真实观测和反馈、已审核计划标识、已验收的标定/模型/保持行为，以及显式监督确认。硬件时序、模型精度、标定和保持行为仍需部署验收；参见[操作与停止](docs/operation.zh-CN.md)。

```bash
.venv/bin/python -m pytest -q
```

软件包运行时不导入 `dexpipe`、Gaia20、RL 或重定向流水线。FoundationPose 和 SAM 的 GPU 服务仍在外部运行，本仓库包含其 RPC 客户端。`third_party/tron2_env` 保留带本地补丁的传输层快照及上游声明，不包含嵌套 Git 元数据。`SOURCE_MANIFEST.json` 记录来源与移植文件。源历史保留在 [dexpipe](https://github.com/Shukashuki/dexpipe)。
