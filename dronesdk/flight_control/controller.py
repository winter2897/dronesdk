"""
Flight Controller.

This module provides flight control functionality including
mode control, arming, takeoff, and goto operations.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any, Callable

import monotonic
from pymavlink import mavutil

from dronesdk.core.exceptions import APIException
from dronesdk.models.location import LocationGlobal, LocationGlobalRelative
from dronesdk.models.status import VehicleMode

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent
    from dronesdk.datalink.connection import MAVConnection

logger = logging.getLogger(__name__)


class FlightController:
    """
    Controls vehicle flight operations.

    This module handles:
    - Mode changes
    - Arming/disarming
    - Takeoff
    - Goto (position navigation)
    - Velocity commands

    Args:
        on_mode_change: Optional callback when mode changes
        on_armed_change: Optional callback when armed state changes
    """

    __slots__ = (
        "_connection",
        "_mode",
        "_armed",
        "_autopilot_type",
        "_vehicle_type",
        "_on_mode_change",
        "_on_armed_change",
        "_unsubscribe_fns",
    )

    def __init__(
        self,
        on_mode_change: Callable[[VehicleMode], None] | None = None,
        on_armed_change: Callable[[bool], None] | None = None,
    ) -> None:
        self._connection: MAVConnection | None = None
        self._mode: VehicleMode | None = None
        self._armed: bool | None = None
        self._autopilot_type: int | None = None
        self._vehicle_type: int | None = None
        self._on_mode_change = on_mode_change
        self._on_armed_change = on_armed_change
        self._unsubscribe_fns: list[Callable[[], None]] = []

    @property
    def name(self) -> str:
        """Module name."""
        return "flight_control"

    def set_connection(self, connection: "MAVConnection") -> None:
        """Set the MAVLink connection."""
        self._connection = connection

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive flight-related messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        unsub = event_bus.subscribe_message(
            "HEARTBEAT",
            self._handle_heartbeat,
        )
        self._unsubscribe_fns.append(unsub)
        logger.debug("FlightController attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        for unsub in self._unsubscribe_fns:
            unsub()
        self._unsubscribe_fns.clear()
        logger.debug("FlightController detached from event bus")

    def initialize(self) -> None:
        """Initialize the module."""
        pass

    def _handle_heartbeat(self, event: "MAVLinkMessageEvent") -> None:
        """Handle HEARTBEAT message for mode and armed updates."""
        msg = event.message

        # Ignore non-vehicle heartbeats
        if msg.type in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_GIMBAL,
            mavutil.mavlink.MAV_TYPE_ADSB,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        ):
            return

        self._autopilot_type = msg.autopilot
        self._vehicle_type = msg.type

        # Update armed state
        new_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if self._armed != new_armed:
            old_armed = self._armed
            self._armed = new_armed
            if old_armed is not None and self._on_armed_change:
                try:
                    self._on_armed_change(new_armed)
                except Exception:
                    logger.exception("Error in armed change callback")

        # Update mode
        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            mode_name = mavutil.interpret_px4_mode(
                msg.base_mode,
                msg.custom_mode,
            )
        else:
            mode_mapping = mavutil.mode_mapping_bynumber(self._vehicle_type)
            if mode_mapping:
                mode_name = mode_mapping.get(msg.custom_mode, "UNKNOWN")
            else:
                mode_name = "UNKNOWN"

        if self._mode is None or self._mode.name != mode_name:
            old_mode = self._mode
            self._mode = VehicleMode.from_mavlink(mode_name)
            if old_mode is not None and self._on_mode_change:
                try:
                    self._on_mode_change(self._mode)
                except Exception:
                    logger.exception("Error in mode change callback")

    @property
    def mode(self) -> VehicleMode | None:
        """Get current vehicle mode."""
        return self._mode

    @mode.setter
    def mode(self, value: VehicleMode | str) -> None:
        """
        Set the vehicle mode.

        Args:
            value: VehicleMode object or mode name string

        Example:
            >>> vehicle.mode = VehicleMode("GUIDED")
            >>> vehicle.mode = "GUIDED"  # Also works
        """
        if not self._connection:
            raise APIException("No connection available")

        if isinstance(value, str):
            mode_name = value
        else:
            mode_name = value.name

        # Get mode mapping
        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            # PX4 mode setting
            mode_id = mavutil.interpret_px4_mode(mode_name)
            if mode_id is None:
                raise ValueError(f"Unknown mode: {mode_name}")
            self._connection.master.mav.command_long_send(
                self._connection.target_system,
                0,  # component
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,  # confirmation
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
                0,
                0,
                0,
                0,
                0,
            )
        else:
            # ArduPilot mode setting
            mode_mapping = mavutil.mode_mapping_byname(self._vehicle_type)
            if mode_mapping is None or mode_name.upper() not in mode_mapping:
                raise ValueError(f"Unknown mode: {mode_name}")

            mode_id = mode_mapping[mode_name.upper()]
            self._connection.master.set_mode(mode_id)

    @property
    def armed(self) -> bool | None:
        """Get armed state."""
        return self._armed

    @armed.setter
    def armed(self, value: bool) -> None:
        """
        Arm or disarm the vehicle.

        Args:
            value: True to arm, False to disarm

        Example:
            >>> vehicle.armed = True  # Arm
            >>> vehicle.armed = False  # Disarm
        """
        if not self._connection:
            raise APIException("No connection available")

        if value:
            self._connection.master.arducopter_arm()
        else:
            self._connection.master.arducopter_disarm()

    def simple_takeoff(self, altitude: float) -> None:
        """
        Take off to a specified altitude.

        The vehicle must be in GUIDED mode and armed before calling.

        Args:
            altitude: Target altitude in meters (relative to home)

        Example:
            >>> vehicle.mode = VehicleMode("GUIDED")
            >>> vehicle.armed = True
            >>> vehicle.simple_takeoff(10)
        """
        if not self._connection:
            raise APIException("No connection available")

        self._connection.master.mav.command_long_send(
            self._connection.target_system,
            0,  # component
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,
            0,
            0,
            0,  # params 1-4
            0,
            0,  # lat, lon (ignored)
            altitude,
        )

    def simple_goto(
        self,
        location: LocationGlobal | LocationGlobalRelative,
        airspeed: float | None = None,
        groundspeed: float | None = None,
    ) -> None:
        """
        Navigate to a specified location.

        The vehicle must be in GUIDED mode.

        Args:
            location: Target location
            airspeed: Optional airspeed in m/s
            groundspeed: Optional groundspeed in m/s

        Example:
            >>> target = LocationGlobalRelative(-35.36, 149.17, 20)
            >>> vehicle.simple_goto(target)
        """
        if not self._connection:
            raise APIException("No connection available")

        # Determine frame
        if isinstance(location, LocationGlobalRelative):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        else:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL

        # Send position target
        self._connection.master.mav.mission_item_send(
            self._connection.target_system,
            0,  # component
            0,  # seq
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,  # current (2 = guided mode)
            0,  # autocontinue
            0,
            0,
            0,
            0,  # params
            location.lat or 0,
            location.lon or 0,
            location.alt or 0,
        )

        # Set speed if specified
        if airspeed is not None:
            self._set_airspeed(airspeed)
        if groundspeed is not None:
            self._set_groundspeed(groundspeed)

    def _set_airspeed(self, speed: float) -> None:
        """Set target airspeed."""
        if not self._connection:
            return
        self._connection.master.mav.command_long_send(
            self._connection.target_system,
            0,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            0,  # airspeed
            speed,
            -1,  # throttle (no change)
            0,
            0,
            0,
            0,
        )

    def _set_groundspeed(self, speed: float) -> None:
        """Set target groundspeed."""
        if not self._connection:
            return
        self._connection.master.mav.command_long_send(
            self._connection.target_system,
            0,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,  # groundspeed
            speed,
            -1,  # throttle (no change)
            0,
            0,
            0,
            0,
        )

    def send_ned_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        duration: float = 1.0,
    ) -> None:
        """
        Send velocity command in NED frame.

        Args:
            vx: Velocity north (m/s)
            vy: Velocity east (m/s)
            vz: Velocity down (m/s, positive = descend)
            duration: Duration to apply velocity (seconds)

        Example:
            >>> vehicle.send_ned_velocity(1, 0, 0, 1)  # Move north 1m/s for 1s
        """
        if not self._connection:
            raise APIException("No connection available")

        self._connection.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self._connection.target_system,
            0,  # component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # type_mask (velocity only)
            0,
            0,
            0,  # position (ignored)
            vx,
            vy,
            vz,  # velocity
            0,
            0,
            0,  # acceleration (ignored)
            0,
            0,  # yaw, yaw_rate (ignored)
        )

    def send_global_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        """
        Send velocity command in global frame.

        Args:
            vx: Velocity north (m/s)
            vy: Velocity east (m/s)
            vz: Velocity down (m/s, positive = descend)
        """
        if not self._connection:
            raise APIException("No connection available")

        self._connection.master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self._connection.target_system,
            0,  # component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000111,  # type_mask (velocity only)
            0,
            0,
            0,  # position (ignored)
            vx,
            vy,
            vz,  # velocity
            0,
            0,
            0,  # acceleration (ignored)
            0,
            0,  # yaw, yaw_rate (ignored)
        )

    def condition_yaw(
        self,
        heading: float,
        relative: bool = False,
        clockwise: bool = True,
        speed: float = 0,
    ) -> None:
        """
        Command the vehicle to face a specific heading.

        Args:
            heading: Target heading in degrees (0-360)
            relative: If True, heading is relative to current
            clockwise: If True, rotate clockwise
            speed: Rotation speed in deg/s (0 = default)

        Example:
            >>> vehicle.condition_yaw(90)  # Face east
        """
        if not self._connection:
            raise APIException("No connection available")

        if relative:
            is_relative = 1
        else:
            is_relative = 0

        if clockwise:
            direction = 1
        else:
            direction = -1

        self._connection.master.mav.command_long_send(
            self._connection.target_system,
            0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            speed,
            direction,
            is_relative,
            0,
            0,
            0,
        )

    def reboot(self) -> None:
        """Reboot the autopilot."""
        if not self._connection:
            raise APIException("No connection available")

        self._connection.master.reboot_autopilot()

    @property
    def autopilot_type(self) -> int | None:
        """Get autopilot type (MAV_AUTOPILOT enum)."""
        return self._autopilot_type

    @property
    def vehicle_type(self) -> int | None:
        """Get vehicle type (MAV_TYPE enum)."""
        return self._vehicle_type
