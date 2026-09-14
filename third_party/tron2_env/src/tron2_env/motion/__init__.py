"""Motion control orchestration — owns transport + interpolator + publish loop."""

from tron2_env.motion.controller import MotionController, create_motion_controller

__all__ = ["MotionController", "create_motion_controller"]
