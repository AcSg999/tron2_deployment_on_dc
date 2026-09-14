"""Tron2 dual-arm robot control + environment library.

Top-level layout:

  * ``transport/``     — single-shot send/recv primitives (WebSocket JSON)
  * ``interpolation/`` — pure-math joint trajectory interpolation
  * ``motion/``        — MotionController = transport + interpolator + publish loop
  * ``env.py``         — Tron2Env (gym-style env on top of MotionController)
  * ``bridge.py``      — Bridge ws observation provider
  * ``camera.py``      — RealSense multi-camera manager
  * ``rtc/``           — Real-Time Chunking helpers (ActionQueue, LatencyTracker)
"""

from tron2_env.bridge import BridgeConfig
from tron2_env.config import Tron2Config
from tron2_env.env import (
    CameraConfig,
    EnvConfig,
    PolicyWrapper,
    Tron2Env,
    WebsocketPolicyWrapper,
)
from tron2_env.errors import (
    CommandError,
    ConnectionError,
    StateError,
    Tron2Error,
)
from tron2_env.interpolation import JointInterpolator, LinearInterpolator
from tron2_env.joints import JointIndex
from tron2_env.motion import MotionController, create_motion_controller
from tron2_env.transport import RobotTransport, WebsocketTransport

__all__ = [
    # config / joints / errors
    "Tron2Config",
    "JointIndex",
    "Tron2Error",
    "ConnectionError",
    "CommandError",
    "StateError",
    # transports
    "RobotTransport",
    "WebsocketTransport",
    # interpolation
    "JointInterpolator",
    "LinearInterpolator",
    # motion
    "MotionController",
    "create_motion_controller",
    # env
    "Tron2Env",
    "EnvConfig",
    "CameraConfig",
    "PolicyWrapper",
    "WebsocketPolicyWrapper",
    # bridge + camera (lazy)
    "BridgeConfig",
    "BridgeObservationProvider",
    "MultiCameraManager",
]


def __getattr__(name):
    """Lazy import for optional deps (RealSense / websockets)."""
    if name == "MultiCameraManager":
        from tron2_env.camera import MultiCameraManager

        return MultiCameraManager
    if name == "BridgeObservationProvider":
        from tron2_env.bridge import BridgeObservationProvider

        return BridgeObservationProvider
    raise AttributeError(f"module 'tron2_env' has no attribute {name}")
