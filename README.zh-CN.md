# DC 上的 TRON2 部署

[English](README.md)

本仓库提供独立的**基于物体位姿的双腕预抓取**部署工具：定位物体，选择可达的接近方向与腕部姿态，在 RViz 中审核轨迹，运动到预留间距处并停止。应用不发送夹爪指令，也不执行接触或抓取动作。

实机头部/高位 RGB-D 相机为 **Intel RealSense D455**。运行时名称 `cam_high` 和 `/camera/top/...` 话题均指这台 D455。

物体位姿用于确定目标几何关系，而不是直接复制物体四元数。轴对称物体根据当前腕部位置选择径向接近方向，因此无实际意义的物体偏航不会改变目标。非对称物体使用配置的物体系锚点和向外方向。TCP 的 +Z 指向物体，+Y 沿投影后的物体向上轴；再用已标定的腕部到 TCP 变换换算腕部目标。

## 仓库架构

```text
tron2_deployment_on_dc/
├── configs/                 # 完整部署配置和机器人模型
│   ├── assembly.urdf        # 包含已安装灵巧手的最终 URDF
│   ├── robot.urdf           # 不包含灵巧手的基础机器人 URDF
│   ├── robot.xml            # 基础机器人 MuJoCo 模型
│   ├── scene.xml            # 部署检查使用的最终场景/模型
│   ├── robot.example.json   # 部署 profile 模板
│   └── demo.json            # 合成 mock profile
├── sp_vision/               # 独立相机标定单元
│   ├── configs/             # 标定配置和模型快照
│   ├── data/                # 本地采集数据和生成结果
│   ├── head_calib.zh-CN.md  # 头部相机标定指南
│   └── wrist_calib.zh-CN.md # 腕部相机标定指南
├── src/                     # 主部署包和 CLI
├── scripts/                 # 安装及离线辅助脚本
├── tests/                   # 完整软件和 mock 测试
└── docs/                    # 部署、预抓取和操作指南
```

`configs/assembly.urdf` 是包含灵巧手的最终机器人运动学真值，用于最终标定、模型哈希和真实部署。`configs/robot.urdf` 是不含灵巧手的基础模型，不能在最终标定、模型哈希或真实部署中替代 `assembly.urdf`。`sp_vision/configs/` 下的文件是独立标定快照，用于让离线标定脱离父部署包；它们不是 operator profile。

| 阶段 | 文档 | 输出 |
| --- | --- | --- |
| 1. 软件部署与 mock | [DC 设备：Ubuntu 20.04](docs/deployment.zh-CN.md) | 完整运行时、pytest、合成 demo、mock operator |
| 2. 几何标定与验证 | [`sp_vision` 标定单元](sp_vision/head_calib.zh-CN.md) | 头/腕相机内外参、TCP、独立触点验证 |
| 3. 真实部署配置 | [DC 设备：Ubuntu 20.04](docs/deployment.zh-CN.md) | 将审核后的标定结果写入 `configs/` profile |
| 4. 规划与执行 | [预抓取流程](docs/pregrasp.zh-CN.md) | 已检查双臂计划、RViz 审核、执行前检查与实测日志 |

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

当前使用 `sp_vision/` 的独立头部和腕部标定程序完成相机几何标定；它不替代完整部署包，也不会自动启用真实执行。完成独立触点验证后，将审核后的结果写入 `configs/` 下的部署 profile。旧的 `docs/calibration*` 标定流程和 ArUco 示例已移除，`sp_vision` 是当前标定入口。

## 实现与限制

规划器沿抬升—转移—下降通道生成路径，IK 或碰撞检查失败时拒绝计划，不进行任意障碍绕行搜索。物体由配置的保守包围球表示。MuJoCo 仅进行 FK/IK 和碰撞数值计算；**轨迹通过 RViz 查看**。部署模型必须对应实际固定基座及已安装附件，仅保留映射的 14 个臂关节和 2 个头部关节可动。未控制的附件保持固定，并保留碰撞几何。

计划绑定物体观测、标定/配置指纹、编译后模型哈希、初始反馈、目标位姿及确切时间插值。真机执行要求新鲜的真实观测和反馈、已审核计划标识、已验收的标定/模型/保持行为，以及显式监督确认。硬件时序、模型精度、标定和保持行为仍需部署验收；参见[操作与停止](docs/operation.zh-CN.md)。

```bash
.venv/bin/python -m pytest -q
```

软件包运行时不导入 `dexpipe`、Gaia20、RL 或重定向流水线。FoundationPose 和 SAM 的 GPU 服务仍在外部运行，本仓库包含其 RPC 客户端。`third_party/tron2_env` 保留带本地补丁的传输层快照及上游声明，不包含嵌套 Git 元数据。`SOURCE_MANIFEST.json` 记录来源与移植文件。源历史保留在 [dexpipe](https://github.com/Shukashuki/dexpipe)。


