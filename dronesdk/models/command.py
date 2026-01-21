"""
Command Data Model.

This module provides the Command class for mission waypoints
and MAVLink commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pymavlink.dialects.v10 import ardupilotmega


@dataclass
class Command:
    """
    A MAVLink command/waypoint.

    This class represents a single command in a mission sequence.
    It wraps the MAVLink MISSION_ITEM message format.

    The command parameters (param1-param4, x, y, z) have different
    meanings depending on the command type. See:
    https://mavlink.io/en/messages/common.html#MAV_CMD

    Attributes:
        seq: Sequence number (0-indexed position in mission).
        frame: Coordinate frame (MAV_FRAME).
        command: Command ID (MAV_CMD).
        current: Whether this is the current command (0 or 1).
        autocontinue: Autocontinue to next waypoint (0 or 1).
        param1: Command-specific parameter 1.
        param2: Command-specific parameter 2.
        param3: Command-specific parameter 3.
        param4: Command-specific parameter 4.
        x: X position or latitude (depending on frame).
        y: Y position or longitude (depending on frame).
        z: Z position or altitude (depending on frame).
    """

    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return (
            f"Command(seq={self.seq}, cmd={self.command}, "
            f"frame={self.frame}, x={self.x}, y={self.y}, z={self.z})"
        )

    @classmethod
    def from_mavlink(
        cls,
        msg: "ardupilotmega.MAVLink_mission_item_message",
    ) -> "Command":
        """
        Create from MAVLink MISSION_ITEM message.

        Args:
            msg: MAVLink mission item message

        Returns:
            Command instance
        """
        return cls(
            seq=msg.seq,
            frame=msg.frame,
            command=msg.command,
            current=msg.current,
            autocontinue=msg.autocontinue,
            param1=msg.param1,
            param2=msg.param2,
            param3=msg.param3,
            param4=msg.param4,
            x=msg.x,
            y=msg.y,
            z=msg.z,
        )

    def to_mavlink(
        self,
        target_system: int,
        target_component: int,
    ) -> "ardupilotmega.MAVLink_mission_item_message":
        """
        Convert to MAVLink MISSION_ITEM message.

        Args:
            target_system: Target system ID
            target_component: Target component ID

        Returns:
            MAVLink mission item message
        """
        from pymavlink.dialects.v10 import ardupilotmega

        return ardupilotmega.MAVLink_mission_item_message(
            target_system,
            target_component,
            self.seq,
            self.frame,
            self.command,
            self.current,
            self.autocontinue,
            self.param1,
            self.param2,
            self.param3,
            self.param4,
            self.x,
            self.y,
            self.z,
        )

    @property
    def is_waypoint(self) -> bool:
        """Check if this is a navigation waypoint command."""
        from pymavlink import mavutil

        return self.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

    @property
    def is_takeoff(self) -> bool:
        """Check if this is a takeoff command."""
        from pymavlink import mavutil

        return self.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

    @property
    def is_land(self) -> bool:
        """Check if this is a land command."""
        from pymavlink import mavutil

        return self.command == mavutil.mavlink.MAV_CMD_NAV_LAND

    @property
    def is_rtl(self) -> bool:
        """Check if this is a return-to-launch command."""
        from pymavlink import mavutil

        return self.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
