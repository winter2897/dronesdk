"""
Mission Manager.

This module provides high-level mission management functionality.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Callable

from dronesdk.mission.sequence import CommandSequence
from dronesdk.models.command import Command
from dronesdk.models.location import LocationGlobal, LocationGlobalRelative

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus
    from dronesdk.datalink.connection import MAVConnection

logger = logging.getLogger(__name__)


class MissionManager:
    """
    High-level mission management.

    Provides convenient methods for common mission operations
    like creating waypoint missions.

    Example:
        >>> mission = MissionManager(connection)
        >>> mission.add_waypoint(LocationGlobalRelative(-35.36, 149.17, 20))
        >>> mission.add_waypoint(LocationGlobalRelative(-35.37, 149.18, 20))
        >>> mission.upload()
    """

    __slots__ = (
        "_commands",
        "_connection",
        "_home_location",
    )

    def __init__(self) -> None:
        self._commands = CommandSequence()
        self._connection: MAVConnection | None = None
        self._home_location: LocationGlobal | None = None

    @property
    def name(self) -> str:
        """Module name."""
        return "mission"

    def set_connection(self, connection: "MAVConnection") -> None:
        """Set the MAVLink connection."""
        self._connection = connection
        self._commands.set_connection(connection)

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus.

        Args:
            event_bus: The event bus to subscribe to
        """
        self._commands.attach(event_bus)
        logger.debug("MissionManager attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        self._commands.detach()
        logger.debug("MissionManager detached from event bus")

    def initialize(self) -> None:
        """Initialize the module."""
        self._commands.initialize()

    def set_home_location(self, location: LocationGlobal) -> None:
        """Set the home location for relative altitude calculations."""
        self._home_location = location

    @property
    def commands(self) -> CommandSequence:
        """Get the underlying CommandSequence."""
        return self._commands

    def download(self) -> None:
        """Download mission from vehicle."""
        self._commands.download()

    def upload(self) -> None:
        """Upload mission to vehicle."""
        self._commands.upload()

    def wait_ready(self, timeout: float = 30.0) -> bool:
        """Wait for mission download to complete."""
        return self._commands.wait_ready(timeout)

    def clear(self) -> None:
        """Clear all mission commands."""
        self._commands.clear()

    def add_waypoint(
        self,
        location: LocationGlobalRelative | LocationGlobal,
        hold_time: float = 0,
        accept_radius: float = 2.0,
        pass_radius: float = 0,
        yaw: float = 0,
    ) -> None:
        """
        Add a waypoint to the mission.

        Args:
            location: Target location
            hold_time: Time to hold at waypoint (seconds)
            accept_radius: Acceptance radius (meters)
            pass_radius: Pass radius (0 = through waypoint)
            yaw: Desired yaw angle (degrees)
        """
        from pymavlink import mavutil

        if isinstance(location, LocationGlobalRelative):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        else:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL

        cmd = Command(
            seq=0,  # Will be set by add()
            frame=frame,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0,
            autocontinue=1,
            param1=hold_time,
            param2=accept_radius,
            param3=pass_radius,
            param4=yaw,
            x=location.lat or 0,
            y=location.lon or 0,
            z=location.alt or 0,
        )
        self._commands.add(cmd)

    def add_takeoff(self, altitude: float) -> None:
        """
        Add a takeoff command.

        Args:
            altitude: Target altitude in meters (relative to home)
        """
        from pymavlink import mavutil

        cmd = Command(
            seq=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            current=0,
            autocontinue=1,
            param1=0,  # Minimum pitch
            param2=0,
            param3=0,
            param4=0,  # Yaw angle
            x=0,
            y=0,
            z=altitude,
        )
        self._commands.add(cmd)

    def add_land(
        self,
        location: LocationGlobalRelative | LocationGlobal | None = None,
    ) -> None:
        """
        Add a land command.

        Args:
            location: Optional landing location (current position if None)
        """
        from pymavlink import mavutil

        lat = location.lat if location else 0
        lon = location.lon if location else 0

        cmd = Command(
            seq=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_LAND,
            current=0,
            autocontinue=1,
            param1=0,  # Abort altitude
            param2=0,  # Precision land mode
            param3=0,
            param4=0,  # Yaw angle
            x=lat or 0,
            y=lon or 0,
            z=0,
        )
        self._commands.add(cmd)

    def add_rtl(self) -> None:
        """Add a return-to-launch command."""
        from pymavlink import mavutil

        cmd = Command(
            seq=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            current=0,
            autocontinue=1,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            x=0,
            y=0,
            z=0,
        )
        self._commands.add(cmd)

    def add_loiter(
        self,
        location: LocationGlobalRelative | LocationGlobal,
        duration: float = 0,
        radius: float = 0,
    ) -> None:
        """
        Add a loiter command.

        Args:
            location: Loiter location
            duration: Loiter time in seconds (0 = unlimited)
            radius: Loiter radius in meters (0 = default)
        """
        from pymavlink import mavutil

        if isinstance(location, LocationGlobalRelative):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        else:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL

        if duration > 0:
            command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
            param1 = duration
        else:
            command = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
            param1 = 0

        cmd = Command(
            seq=0,
            frame=frame,
            command=command,
            current=0,
            autocontinue=1,
            param1=param1,
            param2=0,
            param3=radius,
            param4=0,  # Yaw
            x=location.lat or 0,
            y=location.lon or 0,
            z=location.alt or 0,
        )
        self._commands.add(cmd)

    @property
    def next_waypoint(self) -> int:
        """Get the index of the next waypoint."""
        return self._commands.next

    @next_waypoint.setter
    def next_waypoint(self, value: int) -> None:
        """Set the next waypoint to execute."""
        self._commands.next = value

    def __len__(self) -> int:
        return len(self._commands)

    def __str__(self) -> str:
        return f"MissionManager({len(self._commands)} commands)"
