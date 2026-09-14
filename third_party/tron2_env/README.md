# tron2_env

[中文文档](README_CN.md)

`tron2_env` is the public TRON2 runtime package used by the TRON2 OpenPI
deployment example. It provides robot communication, motion execution,
observation collection, and RTC helper utilities for real-robot deployment.

The public version currently exposes the WebSocket robot transport only. The
undeveloped low-level transport path is intentionally not included.

## What This Package Provides

- `Tron2Config`: robot connection and bring-up configuration.
- `WebsocketTransport`: WebSocket JSON transport for TRON2 robot state and commands.
- `MotionController`: interpolation plus high-rate command publishing.
- `Tron2Env`: a gym-style runtime wrapper for policy execution.
- `BridgeObservationProvider`: image and state observation support through TRON2 Bridge.
- `MultiCameraManager`: legacy local RealSense camera observation support.
- `ActionQueue` and `LatencyTracker`: RTC helper utilities.
- `examples/replay_data.py`: recorded-action replay through the runtime controller.
- Joint index helpers and package-level exceptions.

## What Is Not Included

- Low-level robot transport.
- Private robot safety controllers.
- Robot-specific calibration, customer profiles, or local network settings.
- Model weights, datasets, or policy training code.

## Expected Layout

Keep this package next to `tron2_openpi`:

```text
parent-directory/
├── tron2_openpi/
└── tron2_env/
    ├── examples/
    ├── pyproject.toml
    ├── src/
    │   └── tron2_env/
    └── tests/
```

`tron2_openpi/examples/tron2/pi_client.py` adds `../tron2_env/src` to
`sys.path`, so it can import `tron2_env` directly during local development.

## Dependencies

Install the runtime as an independent editable package:

```bash
git clone https://github.com/limx-tron2/tron2_env.git
cd tron2_env
python -m pip install -e ".[bridge,openpi]"
```

Optional extras:

| Extra | Purpose |
| --- | --- |
| `bridge` | WebSocket bridge observation support. |
| `camera` | Local RealSense camera support through `pyrealsense2`. |
| `openpi` | Image formatting helper dependency used by the optional policy wrapper. |
| `replay` | Parquet and YAML dependencies for `examples/replay_data.py`. |
| `dev` | Test runner dependencies. |
| `all` | Bridge, camera, OpenPI helper, and replay dependencies. |

When using this package with the TRON2 OpenPI deployment repository, clone
`tron2_openpi` as a sibling and install/sync it separately:

```bash
cd ..
git clone https://github.com/limx-tron2/tron2_openpi.git
cd tron2_openpi
uv sync
```

## Import Check

From `tron2_openpi/`:

```bash
PYTHONPATH="$(cd ../tron2_env/src && pwd)" uv run python -c "import tron2_env; print(tron2_env.__file__)"
```

The printed path should point to `../tron2_env/src/tron2_env/__init__.py`.

## No-Hardware Quick Start

Install the development dependencies and run the mock transport example:

```bash
python -m pip install -e ".[dev]"
python examples/mock_quickstart.py
python -m pytest -q
```

The example uses an in-memory transport. It does not open a network connection,
discover hardware, or send commands to a real robot.

## Real-Robot Usage

The following example connects to the configured robot and may send commands.
Only run it in an authorised environment with the normal robot bring-up and
emergency-stop procedures in place.

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

For policy deployment, use `tron2_openpi/examples/tron2/pi_client.py`; it wires
the policy client, `Tron2Env`, observations, and action playback together.

## Replay Recorded Data

`examples/replay_data.py` replays a recorded parquet trajectory directly through
`tron2_env`. It uses the deployment YAML from the sibling `tron2_openpi`
repository for robot IP, initialization pose, backend, and playback rate.

Always dry-run first:

```bash
python -m pip install -e ".[replay]"
python examples/replay_data.py \
  --file /path/to/trajectory.parquet \
  --deploy-config ../tron2_openpi/configs/deploy/tron2_deploy.local.yaml \
  --data-key action \
  --dry-run
```

Remove `--dry-run` only after confirming the file, replay range, robot address,
initial pose, and workspace safety.

## Configuration Concepts

`Tron2Config` contains robot-level settings:

| Field | Description |
| --- | --- |
| `robot_ip` | TRON2 robot WebSocket controller address. |
| `port` | Robot controller WebSocket port. |
| `init_joints` | Optional 14-value arm initialization pose. |
| `init_head` | Optional 2-value head initialization pose. |
| `init_ee_z_min` | Minimum end-effector Z before restart routes through the official intermediate pose; set to `None` to disable. |
| `state_queue_maxlen` | Robot state feedback queue length. |
| `polling_rate` | Robot state polling rate. |
| `connection_timeout` | WebSocket connection timeout. |

`EnvConfig` is used by `Tron2Env` and the TRON2 client to select observation
mode, action playback rate, debug recording, bridge settings, and camera
settings. The public deployment templates in `tron2_openpi/configs/deploy/`
show the recommended values.

## Observation Modes

Bridge mode:

- Reads images from TRON2 Bridge.
- Can read bridge-aligned joint state.
- Does not require camera USB cables to be attached to the host computer.

Legacy RealSense mode:

- Reads images from locally attached RealSense cameras.
- Reads robot state from the WebSocket robot transport.
- Requires camera serial numbers to be mapped to policy camera names.

The policy image names are:

- `cam_high`
- `cam_left_wrist`
- `cam_right_wrist`

## Safety Notes

- Confirm bring-up poses before commanding a real robot.
- Start motion tests at low speed and with short action chunks.
- Keep emergency stop access available while using real-robot clients.
- Treat local configuration files as private deployment assets.
- This package exposes runtime plumbing; it does not replace robot-level safety
  procedures or private low-level safety systems.
- The public CI and `examples/mock_quickstart.py` cover software-only paths. They
  do not validate robot safety, calibration, Bridge, RealSense, or real-hardware
  behaviour.

## Network Security Boundary

- All runtime network interfaces—the robot-control WebSocket, TRON2 Bridge
  observation WebSockets, and OpenPI policy WebSocket—are supported only on a
  controlled robot LAN restricted to authorised systems.
- Do not expose these interfaces directly to the Internet or to an untrusted or
  shared network. Public port forwarding, cross-site links, and cloud deployment
  require a separate security review and appropriate transport protection.
- The current robot-control interface uses `ws://` and does not establish
  transport encryption or client authentication. Bridge deployments may also be
  configured to skip server-certificate verification. Network segmentation,
  firewalling, and access control are therefore part of the required deployment
  trust boundary.
- Treat camera images, robot state and metadata, and policy actions as protected
  runtime data. Do not capture, forward, or publish them without the applicable
  data and deployment approval.

## Development Checks

Run the same software-only checks used by CI:

```bash
python -m pip install -e ".[dev]"
python -m compileall -q src tests examples
python -m pytest -q
python examples/mock_quickstart.py
python -m build
```

See `CONTRIBUTING.md` for contribution boundaries and `SECURITY.md` for private
vulnerability reporting instructions.

## Troubleshooting

- If imports fail, make sure `tron2_env` and `tron2_openpi` are siblings and
  that `PYTHONPATH` includes the parent directory.
- If WebSocket connection fails, verify `robot_ip`, `port`, network reachability,
  and robot controller state.
- If bridge observations time out, verify the Bridge host, WebSocket path, TLS
  setting, and topic availability.
- If legacy cameras are missing, verify RealSense permissions and serial number
  mappings.
- If actions look discontinuous, check policy FPS, publish rate, RTC settings,
  and action normalization stats in the policy checkpoint.

## Third-Party Origins

Most of this package is LimX-developed TRON2 runtime code. The RTC
`ActionQueue` is modified vendored source derived from the LeRobot
implementation at commit `ca87ccd9413c59c30f524967222d2e3f1b7bb549`. It was
ported from PyTorch to NumPy and adapted for TRON2. See `NOTICE` for details.

## License

Unless otherwise noted, this package is distributed under the Apache License
2.0 in `LICENSE`. Notices for vendored third-party source are listed in
`NOTICE`; licenses for external direct dependencies are inventoried in
`THIRD_PARTY_DEPENDENCIES.md`.
