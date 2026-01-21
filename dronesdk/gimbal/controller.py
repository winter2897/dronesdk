"""
Gimbal Controller.

This module provides gimbal status monitoring and control.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable

from pymavlink import mavutil

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent
    from dronesdk.datalink.connection import MAVConnection
    from dronesdk.models.location import LocationGlobal, LocationGlobalRelative

logger = logging.getLogger(__name__)


@dataclass
class GimbalState:
    """
    Current gimbal orientation.

    Attributes:
        pitch: Pitch angle in degrees (-90 = straight down, 0 = forward)
        roll: Roll angle in degrees
        yaw: Yaw angle in degrees (0 = North, 90 = East)
    """

    pitch: float | None
    roll: float | None
    yaw: float | None

    def __str__(self) -> str:
        return f"Gimbal: pitch={self.pitch}, roll={self.roll}, yaw={self.yaw}"


class GimbalController:
    """
    Controls and monitors gimbal orientation.

    The gimbal can be controlled in several ways:
    - rotate(): Set explicit pitch/roll/yaw angles
    - target_location(): Point at a GPS location
    - release(): Return control to RC

    Args:
        on_update: Optional callback when gimbal state changes
    """

    __slots__ = (
        "_pitch",
        "_roll",
        "_yaw",
        "_connection",
        "_on_update",
        "_unsubscribe_fns",
    )

    def __init__(
        self,
        on_update: Callable[[GimbalState], None] | None = None,
    ) -> None:
        self._pitch: float | None = None
        self._roll: float | None = None
        self._yaw: float | None = None
        self._connection: MAVConnection | None = None
        self._on_update = on_update
        self._unsubscribe_fns: list[Callable[[], None]] = []

    @property
    def name(self) -> str:
        """Module name."""
        return "gimbal"

    def set_connection(self, connection: "MAVConnection") -> None:
        """Set the MAVLink connection for sending commands."""
        self._connection = connection

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive gimbal messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        unsub1 = event_bus.subscribe_message(
            "MOUNT_STATUS",
            self._handle_mount_status,
        )
        unsub2 = event_bus.subscribe_message(
            "MOUNT_ORIENTATION",
            self._handle_mount_orientation,
        )
        self._unsubscribe_fns = [unsub1, unsub2]
        logger.debug("GimbalController attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        for unsub in self._unsubscribe_fns:
            unsub()
        self._unsubscribe_fns.clear()
        logger.debug("GimbalController detached from event bus")

    def initialize(self) -> None:
        """Initialize the module."""
        pass

    def _handle_mount_status(self, event: "MAVLinkMessageEvent") -> None:
        """Handle MOUNT_STATUS message."""
        msg = event.message
        self._pitch = msg.pointing_a / 100.0
        self._roll = msg.pointing_b / 100.0
        self._yaw = msg.pointing_c / 100.0
        self._notify_update()

    def _handle_mount_orientation(self, event: "MAVLinkMessageEvent") -> None:
        """Handle MOUNT_ORIENTATION message."""
        msg = event.message
        self._pitch = msg.pitch
        self._roll = msg.roll
        self._yaw = msg.yaw
        self._notify_update()

    def _notify_update(self) -> None:
        """Notify listeners of gimbal state change."""
        if self._on_update:
            try:
                self._on_update(self.state)
            except Exception:
                logger.exception("Error in gimbal update callback")

    @property
    def state(self) -> GimbalState:
        """Get current gimbal state."""
        return GimbalState(
            pitch=self._pitch,
            roll=self._roll,
            yaw=self._yaw,
        )

    @property
    def pitch(self) -> float | None:
        """
        Gimbal pitch in degrees relative to vehicle.

        0 = straight ahead, -90 = straight down.
        """
        return self._pitch

    @property
    def roll(self) -> float | None:
        """Gimbal roll in degrees relative to vehicle."""
        return self._roll

    @property
    def yaw(self) -> float | None:
        """Gimbal yaw in degrees (0 = North, 90 = East)."""
        return self._yaw

    def rotate(self, pitch: float, roll: float, yaw: float) -> None:
        """
        Rotate the gimbal to a specific orientation.

        Args:
            pitch: Pitch in degrees (0 = forward, -90 = down)
            roll: Roll in degrees
            yaw: Yaw in degrees (0 = North, 90 = East)

        Example:
            >>> vehicle.gimbal.rotate(-45, 0, 0)  # Point 45° down
        """
        if not self._connection:
            logger.error("Cannot rotate gimbal: no connection")
            return

        # Configure mount for MAVLink targeting
        msg = self._connection.master.mav.mount_configure_encode(
            0,
            1,  # target system, target component
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            1,  # stabilize roll
            1,  # stabilize pitch
            1,  # stabilize yaw
        )
        self._connection.master.mav.send(msg)

        # Send mount control
        msg = self._connection.master.mav.mount_control_encode(
            0,
            1,  # target system, target component
            int(pitch * 100),  # centidegrees
            int(roll * 100),
            int(yaw * 100),
            0,  # save position
        )
        self._connection.master.mav.send(msg)

    def target_location(
        self,
        location: "LocationGlobal | LocationGlobalRelative",
        home_location: "LocationGlobal | None" = None,
    ) -> None:
        """
        Point the gimbal at a specific GPS location.

        Args:
            location: Target location (Global or GlobalRelative)
            home_location: Home location for altitude calculation
                          (required if location is LocationGlobal)

        Example:
            >>> vehicle.gimbal.target_location(vehicle.home_location)
        """
        if not self._connection:
            logger.error("Cannot target location: no connection")
            return

        from dronesdk.models.location import LocationGlobal, LocationGlobalRelative

        # Configure mount for GPS targeting
        msg = self._connection.master.mav.mount_configure_encode(
            0,
            1,
            mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT,
            1,
            1,
            1,  # stabilize roll, pitch, yaw
        )
        self._connection.master.mav.send(msg)

        # Calculate altitude relative to home
        if isinstance(location, LocationGlobalRelative):
            alt = location.alt or 0
        elif isinstance(location, LocationGlobal):
            if home_location is None:
                raise ValueError("home_location required for LocationGlobal targeting")
            home_alt = home_location.alt or 0
            loc_alt = location.alt or 0
            alt = loc_alt - home_alt
        else:
            raise ValueError(
                "location must be LocationGlobal or LocationGlobalRelative"
            )

        # Send ROI command
        msg = self._connection.master.mav.command_long_encode(
            0,
            1,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            0,  # confirmation
            0,
            0,
            0,
            0,  # params 1-4
            location.lat or 0,
            location.lon or 0,
            alt,
        )
        self._connection.master.mav.send(msg)

    def release(self) -> None:
        """
        Release gimbal control back to RC.

        Call this when finished with programmatic control.
        Control is also released automatically on mode change.
        """
        if not self._connection:
            logger.error("Cannot release gimbal: no connection")
            return

        msg = self._connection.master.mav.mount_configure_encode(
            0,
            1,
            mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,
            1,
            1,
            1,  # stabilize roll, pitch, yaw
        )
        self._connection.master.mav.send(msg)

    def __str__(self) -> str:
        return str(self.state)
