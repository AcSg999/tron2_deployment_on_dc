"""Shared exceptions for the tron2_env package.

Used by transports, motion controller, and the environment alike. Kept in a
single module so callers can `from tron2_env.errors import ConnectionError`
without needing to know which transport raised it.
"""

from __future__ import annotations


class Tron2Error(Exception):
    """Base class for all tron2_env runtime errors."""


class ConnectionError(Tron2Error):
    """Failed to reach or maintain the robot connection."""


class CommandError(Tron2Error):
    """A command (servoj, gripper, ...) failed validation or transport."""


class StateError(Tron2Error):
    """State retrieval failed (timeout, bad shape, ...)."""
