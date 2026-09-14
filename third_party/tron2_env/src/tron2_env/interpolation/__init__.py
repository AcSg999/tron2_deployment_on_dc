"""Joint interpolation primitives — used by ``MotionController``."""

from tron2_env.interpolation.base import JointInterpolator
from tron2_env.interpolation.linear import LinearInterpolator

__all__ = ["JointInterpolator", "LinearInterpolator"]
