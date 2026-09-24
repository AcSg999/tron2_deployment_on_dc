# tron2_env

[English](README.md)

`tron2_env` 是 TRON2 OpenPI 部署示例使用的公开 TRON2 运行时包，负责机器人通信、
运动执行、观测采集和 RTC 辅助逻辑。

当前公开版本只提供 WebSocket 机器人 transport。尚未开发完成的 low-level
transport 路径已主动排除，不在本次开源范围内。

## 本包包含什么

- `Tron2Config`：机器人连接和初始化配置。
- `WebsocketTransport`：用于 TRON2 机器人状态和命令的 WebSocket JSON transport。
- `MotionController`：插值和高频指令发布。
- `Tron2Env`：面向 policy 执行的 gym-style 运行时封装。
- `BridgeObservationProvider`：通过 TRON2 Bridge 获取图像和状态。
- `MultiCameraManager`：legacy 本机 RealSense 相机观测。
- `ActionQueue` 和 `LatencyTracker`：RTC 辅助工具。
- `examples/replay_data.py`：通过 runtime controller 回放录制动作。
- 关节索引工具和包级异常类型。

## 本包不包含什么

- low-level 机器人 transport。
- 私有机器人安全控制器。
- 机器人标定、客户配置或本地网络设置。
- 模型权重、数据集或 policy 训练代码。

## 推荐目录结构

请将本包和 `tron2_openpi` 放在同级：

```text
同级目录/
├── tron2_openpi/
└── tron2_env/
    ├── examples/
    ├── pyproject.toml
    ├── src/
    │   └── tron2_env/
    └── tests/
```

`tron2_openpi/examples/tron2/pi_client.py` 会把 `../tron2_env/src` 加入
`sys.path`，因此本地开发时可以直接导入 `tron2_env`。

## 依赖安装

作为独立 editable 包安装：

```bash
git clone https://github.com/limx-tron2/tron2_env.git
cd tron2_env
python -m pip install -e ".[bridge,openpi]"
```

可选 extras：

| Extra | 用途 |
| --- | --- |
| `bridge` | Bridge WebSocket 观测支持。 |
| `camera` | 通过 `pyrealsense2` 使用本机 RealSense 相机。 |
| `openpi` | 可选 policy wrapper 使用的图像格式化 helper 依赖。 |
| `replay` | `examples/replay_data.py` 使用的 parquet 和 YAML 依赖。 |
| `dev` | 测试运行依赖。 |
| `all` | Bridge、camera、OpenPI helper 和 replay 依赖。 |

如果需要配合同级的 TRON2 OpenPI 部署仓库使用，请单独 clone 并安装/sync
`tron2_openpi`：

```bash
cd ..
git clone https://github.com/limx-tron2/tron2_openpi.git
cd tron2_openpi
uv sync
```

## 导入检查

从 `tron2_openpi/` 中运行：

```bash
PYTHONPATH="$(cd ../tron2_env/src && pwd)" uv run python -c "import tron2_env; print(tron2_env.__file__)"
```

输出路径应该指向 `../tron2_env/src/tron2_env/__init__.py`。

## 无硬件 Quick Start

安装开发依赖并运行 mock transport 示例：

```bash
python -m pip install -e ".[dev]"
python examples/mock_quickstart.py
python -m pytest -q
```

该示例只使用内存 transport，不会建立网络连接、发现硬件或向真实机器人发送指令。

## 真机用法

以下示例会连接配置的机器人，并可能发送指令。只能在已授权环境中运行，并应遵循
正常的机器人初始化和急停流程。

```python
from tron2_env import Tron2Config, create_motion_controller

config = Tron2Config(
    robot_ip="ROBOT_IP",
    port=5000,
    init_joints=None,
    init_head=None,
)

controller = create_motion_controller(
    config,
    backend="websocket",
    publish_rate=500.0,
)

try:
    state = controller.get_joint_states(timeout=1.0)
    print(state)
finally:
    controller.disconnect()
```

实际 policy 部署建议直接使用 `tron2_openpi/examples/tron2/pi_client.py`，它已经把
policy client、`Tron2Env`、观测和 action 播放串联好。

## 回放录制数据

`examples/replay_data.py` 会通过 `tron2_env` 直接回放 parquet 轨迹。它使用同级
`tron2_openpi` 仓库中的部署 YAML 读取机器人 IP、初始化姿态、backend 和播放频率。

必须先 dry-run：

```bash
python -m pip install -e ".[replay]"
python examples/replay_data.py \
  --file /path/to/trajectory.parquet \
  --deploy-config ../tron2_openpi/configs/deploy/tron2_deploy.local.yaml \
  --data-key action \
  --dry-run
```

只有在确认文件、回放范围、机器人地址、初始化姿态和工作空间安全后，才移除
`--dry-run`。

## 配置概念

`Tron2Config` 包含机器人级设置：

| 字段 | 说明 |
| --- | --- |
| `robot_ip` | TRON2 机器人 WebSocket 控制器地址。 |
| `port` | 机器人控制器 WebSocket 端口。 |
| `init_joints` | 可选 14 维双臂初始化姿态。 |
| `init_head` | 可选 2 维头部初始化姿态。 |
| `init_ee_z_min` | restart 前的末端 Z 下限；低于该值时先经过官方 intermediate 姿态，设为 `None` 可禁用。 |
| `state_queue_maxlen` | 机器人状态反馈队列长度。 |
| `polling_rate` | 机器人状态轮询频率。 |
| `connection_timeout` | WebSocket 连接超时时间。 |

`EnvConfig` 由 `Tron2Env` 和 TRON2 客户端使用，用于选择观测模式、action 播放频率、
调试记录、Bridge 设置和相机设置。推荐值可以参考
`tron2_openpi/configs/deploy/` 中的公开部署模板。

## 观测模式

Bridge 模式：

- 从 TRON2 Bridge 读取图像。
- 可以读取 Bridge 对齐后的关节状态。
- 不需要将相机 USB 线连接到主机电脑。

Legacy RealSense 模式：

- 从本机直连 RealSense 相机读取图像。
- 从 WebSocket 机器人 transport 读取状态。
- 需要把相机序列号映射到 policy 期望的相机名。

policy 图像名为：

- `cam_high`（头部安装的 Intel RealSense D455）
- `cam_left_wrist`
- `cam_right_wrist`

## 安全注意事项

- 指令真实机器人前先确认初始化姿态安全可达。
- 首次运动测试使用较低速度和较短 action chunk。
- 真机运行时保持急停可用。
- 本地配置文件属于私有部署资产，不应提交。
- 本包提供运行时管线，不替代机器人级安全流程或私有 low-level 安全系统。
- 公共 CI 和 `examples/mock_quickstart.py` 只覆盖纯软件路径，不验证机器人安全、
  标定、Bridge、RealSense 或真机行为。

## 网络安全边界

- 当前全部运行时网络接口——机器人控制 WebSocket、TRON2 Bridge 观测
  WebSocket 和 OpenPI 策略 WebSocket——仅支持在只允许授权系统接入的受控机器人
  局域网内使用。
- 不要将这些接口直接暴露到互联网或不受信任、共享网络。公网端口映射、跨站点连接
  和云端部署需要单独进行安全评审，并配置相应的传输保护。
- 当前机器人控制接口使用 `ws://`，自身不提供传输加密或客户端认证；Bridge 部署
  也可能配置为跳过服务器证书验证。因此，网络分段、防火墙和访问控制属于部署所需
  的信任边界。
- 相机图像、机器人状态及 metadata、策略动作均应作为受保护的运行数据；未经相应
  数据和部署审批，不要采集、转发或公开。

## 开发检查

运行与 CI 相同的纯软件检查：

```bash
python -m pip install -e ".[dev]"
python -m compileall -q src tests examples
python -m pytest -q
python examples/mock_quickstart.py
python -m build
```

贡献边界见 `CONTRIBUTING.md`，安全漏洞的私密报告方式见 `SECURITY.md`。

## 常见问题

- 如果导入失败，确认 `tron2_env` 和 `tron2_openpi` 是同级目录，并且 `PYTHONPATH`
  包含父目录。
- 如果 WebSocket 连接失败，检查 `robot_ip`、`port`、网络可达性和机器人控制器状态。
- 如果 Bridge 观测超时，检查 Bridge 地址、WebSocket path、TLS 设置和 topic 状态。
- 如果 legacy 相机缺失，检查 RealSense 权限和相机序列号映射。
- 如果动作不连续，检查 policy FPS、publish rate、RTC 设置和 checkpoint 中的 action
  归一化统计。

## 第三方来源

本包大部分为 LimX 自研 TRON2 运行时代码。RTC `ActionQueue` 是基于 LeRobot
commit `ca87ccd9413c59c30f524967222d2e3f1b7bb549` 中实现修改的第三方内嵌源码，
已由 PyTorch 移植到 NumPy 并适配 TRON2。详细信息见 `NOTICE`。

## 许可证

除非另有说明，本包使用 `LICENSE` 中的 Apache License 2.0。内嵌第三方源码的
声明见 `NOTICE`，外部直接依赖的许可证清单见 `THIRD_PARTY_DEPENDENCIES.md`。
