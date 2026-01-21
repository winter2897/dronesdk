"""
Attitude Data Model.

This module provides the Attitude dataclass for representing vehicle
orientation in 3D space.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Attitude:
    """
    Attitude information (pitch, yaw, roll).

    An object of this type is returned by Vehicle.attitude.

    The attitude angles follow standard aerospace conventions:
    - Pitch: Rotation about lateral axis (nose up/down)
    - Yaw: Rotation about vertical axis (heading)
    - Roll: Rotation about longitudinal axis (wing up/down)

    All angles are in radians.

    Attributes:
        pitch: Pitch angle in radians (positive = nose up).
        yaw: Yaw angle in radians (positive = clockwise from north).
        roll: Roll angle in radians (positive = right wing down).
    """

    pitch: float
    yaw: float
    roll: float

    def __str__(self) -> str:
        return (
            f"{self.__class__.__name__}:"
            f"pitch={self.pitch},yaw={self.yaw},roll={self.roll}"
        )

    @classmethod
    def from_mavlink(
        cls,
        pitch: float,
        yaw: float,
        roll: float,
    ) -> "Attitude":
        """
        Create from MAVLink ATTITUDE message values.

        Args:
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            roll: Roll angle in radians

        Returns:
            Attitude instance
        """
        return cls(pitch=pitch, yaw=yaw, roll=roll)

    @property
    def pitch_deg(self) -> float:
        """Pitch angle in degrees."""
        import math

        return math.degrees(self.pitch)

    @property
    def yaw_deg(self) -> float:
        """Yaw angle in degrees."""
        import math

        return math.degrees(self.yaw)

    @property
    def roll_deg(self) -> float:
        """Roll angle in degrees."""
        import math

        return math.degrees(self.roll)
