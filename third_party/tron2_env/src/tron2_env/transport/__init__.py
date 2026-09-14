"""Robot transport layer — single-shot communication primitives.

The public runtime currently exposes the WebSocket JSON protocol used by the
TRON2 controller.
"""

from tron2_env.transport.base import RobotTransport
from tron2_env.transport.websocket import WebsocketTransport

__all__ = [
    "RobotTransport",
    "WebsocketTransport",
]
