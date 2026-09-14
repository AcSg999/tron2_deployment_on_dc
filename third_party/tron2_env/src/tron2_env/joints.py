"""Joint index layout for the Tron2 robot.

State layout (18-dim): [L_arm(7), L_gripper(1), R_arm(7), R_gripper(1), head(2)]
ServoJ / publishRobotCmd q layout (16-dim): [L_arm(7), R_arm(7), head(2)]
"""

from __future__ import annotations


class JointIndex:
    """Centralised joint index constants — shared across transports + env."""

    # primitives
    ARM_DIM = 7
    GRIPPER_DIM = 1
    HEAD_DIM = 2

    # composite dims
    STATE_DIM = ARM_DIM + GRIPPER_DIM + ARM_DIM + GRIPPER_DIM + HEAD_DIM  # 18
    SERVOJ_DIM = ARM_DIM * 2 + HEAD_DIM                                   # 16
    MOVEJ_DIM = ARM_DIM * 2                                               # 14

    # left arm
    LEFT_ARM_START = 0
    LEFT_ARM_END = ARM_DIM
    LEFT_ARM = slice(LEFT_ARM_START, LEFT_ARM_END)

    # left gripper
    LEFT_GRIPPER = LEFT_ARM_END

    # right arm
    RIGHT_ARM_START = LEFT_GRIPPER + GRIPPER_DIM
    RIGHT_ARM_END = RIGHT_ARM_START + ARM_DIM
    RIGHT_ARM = slice(RIGHT_ARM_START, RIGHT_ARM_END)

    # right gripper
    RIGHT_GRIPPER = RIGHT_ARM_END

    # head
    HEAD_START = RIGHT_GRIPPER + GRIPPER_DIM
    HEAD_END = HEAD_START + HEAD_DIM
    HEAD = slice(HEAD_START, HEAD_END)
    HEAD_PITCH = HEAD_START
    HEAD_YAW = HEAD_START + 1

    # legacy aliases (kept short list — used by env.py / pi_client_rtc.py)
    STATE_DIM_WITH_HEAD = STATE_DIM
    ARM_JOINT_DIM = ARM_DIM
    TOTAL_ARM_DIM = ARM_DIM * 2
