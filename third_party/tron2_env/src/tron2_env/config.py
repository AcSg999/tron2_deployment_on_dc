"""Robot-level configuration.

Holds connection params (`robot_ip` / `port`), bring-up pose and safety
settings (`init_joints` / `init_head` / `init_ee_z_min`), and WebSocket
transport tuning shared by the TRON2 runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from tron2_env.joints import JointIndex


@dataclass
class Tron2Config:
    """Connection and bring-up parameters for the Tron2 robot."""

    # ws connection
    robot_ip: str = "ROBOT_IP"
    port: int = 5000

    # bring-up pose
    init_joints: Optional[List[float]] = None      # 14-dim (arm-only)
    init_head: Optional[List[float]] = None        # 2-dim
    init_ee_z_min: Optional[float] = -0.6          # route via second joint if either EE is lower

    # ws transport internals
    state_queue_maxlen: int = 7
    polling_rate: float = 200.0
    connection_timeout: float = 5.0

    def __post_init__(self) -> None:
        if self.init_joints is not None and len(self.init_joints) != JointIndex.MOVEJ_DIM:
            raise ValueError(
                f"init_joints must have {JointIndex.MOVEJ_DIM} elements, got {len(self.init_joints)}"
            )
        if self.init_head is not None and len(self.init_head) != JointIndex.HEAD_DIM:
            raise ValueError(
                f"init_head must have {JointIndex.HEAD_DIM} elements, got {len(self.init_head)}"
            )
