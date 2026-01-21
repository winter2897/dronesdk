"""
Vehicle Facade.

This module provides the Vehicle class that composes all modules
and maintains backwards compatibility with the original API.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any, Callable

import monotonic
from pymavlink import mavutil

from dronesdk.core.events import EventBus
from dronesdk.core.exceptions import APIException, TimeoutError
from dronesdk.core.observer import HasObservers
from dronesdk.channels.reader import Channels
from dronesdk.datalink.connection import MAVConnection
from dronesdk.datalink.heartbeat import HeartbeatManager
from dronesdk.datalink.message_router import LegacyMessageAdapter, MessageRouter
from dronesdk.flight_control.controller import FlightController
from dronesdk.gimbal.controller import GimbalController
from dronesdk.health.monitor import HealthMonitor
from dronesdk.mission.sequence import CommandSequence
from dronesdk.models.attitude import Attitude
from dronesdk.models.battery import Battery
from dronesdk.models.location import (
    LocationGlobal,
    LocationGlobalRelative,
    LocationLocal,
)
from dronesdk.models.sensors import GPSInfo, Rangefinder, Wind
from dronesdk.models.status import SystemStatus, VehicleMode
from dronesdk.models.version import Capabilities, Version
from dronesdk.parameters.manager import ParameterManager
from dronesdk.power.monitor import BatteryMonitor
from dronesdk.sensors.location import Locations
from dronesdk.sensors.manager import SensorManager

if TYPE_CHECKING:
    from dronesdk.gimbal.controller import GimbalState

logger = logging.getLogger(__name__)


class Vehicle(HasObservers):
    """
    The main vehicle class providing access to vehicle state and control.

    This class composes all the modular components and provides
    a backwards-compatible API matching the original dronesdk.

    Example:
        >>> from dronesdk import connect
        >>> vehicle = connect('127.0.0.1:14550', wait_ready=True)
        >>> print(vehicle.mode)
        >>> print(vehicle.location.global_frame)
        >>> vehicle.mode = VehicleMode("GUIDED")
        >>> vehicle.armed = True
        >>> vehicle.simple_takeoff(10)
    """

    def __init__(self, handler: MAVConnection) -> None:
        super().__init__()
        self._handler = handler
        self._master = handler.master

        # Create event bus
        self._event_bus = EventBus()

        # Create message router
        self._message_router = MessageRouter(handler, self._event_bus)
        self._message_router.attach()

        # Legacy message adapter for on_message()
        self._legacy_adapter = LegacyMessageAdapter(self._event_bus)

        # Create modules
        self._heartbeat = HeartbeatManager()
        self._flight_control = FlightController(
            on_mode_change=self._on_mode_change,
            on_armed_change=self._on_armed_change,
        )
        self._sensors = SensorManager(
            on_attitude_update=self._on_attitude_update,
        )
        self._location = Locations()
        self._battery_monitor = BatteryMonitor(
            on_update=self._on_battery_update,
        )
        self._health = HealthMonitor()
        self._parameters = ParameterManager()
        self._channels = Channels()
        self._gimbal = GimbalController(
            on_update=self._on_gimbal_update,
        )
        self._commands = CommandSequence()

        # Set connections
        self._flight_control.set_connection(handler)
        self._channels.set_connection(handler)
        self._gimbal.set_connection(handler)
        self._commands.set_connection(handler)

        # Attach modules to event bus
        self._heartbeat.attach(self._event_bus)
        self._flight_control.attach(self._event_bus)
        self._sensors.attach(self._event_bus)
        self._location.attach(self._event_bus)
        self._battery_monitor.attach(self._event_bus)
        self._health.attach(self._event_bus)
        self._parameters.attach(self._event_bus, handler)
        self._channels.attach(self._event_bus)
        self._gimbal.attach(self._event_bus)
        self._commands.attach(self._event_bus)

        # Initialize parameters
        self._parameters.initialize()

        # Home location cache
        self._home_location: LocationGlobal | None = None

        # For backwards compatibility
        self._params_map: dict[str, float] = self._parameters._params

    # ===== Callbacks for attribute notifications =====

    def _on_mode_change(self, mode: VehicleMode) -> None:
        """Handle mode change from flight controller."""
        self.notify_attribute_listeners("mode", mode, cache=True)

    def _on_armed_change(self, armed: bool) -> None:
        """Handle armed change from flight controller."""
        self.notify_attribute_listeners("armed", armed, cache=True)

    def _on_attitude_update(self, attitude: Attitude) -> None:
        """Handle attitude update from sensors."""
        self.notify_attribute_listeners("attitude", attitude)

    def _on_battery_update(self, battery: Battery) -> None:
        """Handle battery update."""
        self.notify_attribute_listeners("battery", battery)

    def _on_gimbal_update(self, state: "GimbalState") -> None:
        """Handle gimbal update."""
        self.notify_attribute_listeners("gimbal", self._gimbal)

    # ===== Mode and Armed properties =====

    @property
    def mode(self) -> VehicleMode | None:
        """Current vehicle mode."""
        return self._flight_control.mode

    @mode.setter
    def mode(self, value: VehicleMode | str) -> None:
        """Set vehicle mode."""
        self._flight_control.mode = value

    @property
    def armed(self) -> bool | None:
        """Whether the vehicle is armed."""
        return self._flight_control.armed

    @armed.setter
    def armed(self, value: bool) -> None:
        """Arm or disarm the vehicle."""
        self._flight_control.armed = value

    # ===== Location properties =====

    @property
    def location(self) -> Locations:
        """Vehicle location in various frames."""
        return self._location

    @property
    def home_location(self) -> LocationGlobal | None:
        """Home location."""
        return self._home_location

    @home_location.setter
    def home_location(self, value: LocationGlobal) -> None:
        """Set home location."""
        self._home_location = value

    # ===== Sensor properties =====

    @property
    def attitude(self) -> Attitude | None:
        """Current attitude (pitch, yaw, roll)."""
        return self._sensors.attitude

    @property
    def velocity(self) -> list[float] | None:
        """Current velocity [vx, vy, vz] in m/s."""
        return self._sensors.velocity

    @property
    def heading(self) -> int | None:
        """Current heading in degrees."""
        return self._sensors.heading

    @property
    def airspeed(self) -> float | None:
        """Current airspeed in m/s."""
        return self._sensors.airspeed

    @property
    def groundspeed(self) -> float | None:
        """Current groundspeed in m/s."""
        return self._sensors.groundspeed

    @property
    def gps_0(self) -> GPSInfo | None:
        """GPS information."""
        return self._sensors.gps

    @property
    def rangefinder(self) -> Rangefinder:
        """Rangefinder reading."""
        return self._sensors.rangefinder

    @property
    def wind(self) -> Wind | None:
        """Wind information."""
        return self._sensors.wind

    # ===== Battery and power =====

    @property
    def battery(self) -> Battery | None:
        """Battery status."""
        return self._battery_monitor.battery

    # ===== Health and status =====

    @property
    def system_status(self) -> SystemStatus | None:
        """System status."""
        return self._health.system_status

    @property
    def ekf_ok(self) -> bool:
        """Whether EKF is healthy."""
        return self._health.ekf_ok

    @property
    def is_armable(self) -> bool:
        """Whether the vehicle can be armed."""
        return self._health.is_armable

    @property
    def last_heartbeat(self) -> float | None:
        """Time since last heartbeat."""
        return self._heartbeat.time_since_last

    # ===== Version and capabilities =====

    @property
    def version(self) -> Version | None:
        """Autopilot version."""
        v = self._health.version
        if v is None:
            return None
        # Update version with correct autopilot/vehicle type if needed
        if v.autopilot_type == 0 and self._flight_control.autopilot_type:
            return Version(
                raw_version=v.raw_version,
                autopilot_type=self._flight_control.autopilot_type,
                vehicle_type=self._flight_control.vehicle_type or 0,
            )
        return v

    @property
    def capabilities(self) -> Capabilities | None:
        """Autopilot capabilities."""
        return self._health.capabilities

    # ===== Parameters =====

    @property
    def parameters(self) -> ParameterManager:
        """Vehicle parameters."""
        return self._parameters

    # ===== Channels =====

    @property
    def channels(self) -> Channels:
        """RC channels."""
        return self._channels

    # ===== Gimbal =====

    @property
    def gimbal(self) -> GimbalController:
        """Gimbal controller."""
        return self._gimbal

    # ===== Mission commands =====

    @property
    def commands(self) -> CommandSequence:
        """Mission commands."""
        return self._commands

    # ===== Message factory =====

    @property
    def message_factory(self) -> Any:
        """MAVLink message factory."""
        return self._master.mav

    # ===== Flight control methods =====

    def simple_takeoff(self, alt: float) -> None:
        """
        Take off to a specified altitude.

        Args:
            alt: Target altitude in meters (relative to home)
        """
        self._flight_control.simple_takeoff(alt)

    def simple_goto(
        self,
        location: LocationGlobal | LocationGlobalRelative,
        airspeed: float | None = None,
        groundspeed: float | None = None,
    ) -> None:
        """
        Navigate to a location.

        Args:
            location: Target location
            airspeed: Optional target airspeed
            groundspeed: Optional target groundspeed
        """
        self._flight_control.simple_goto(location, airspeed, groundspeed)

    # ===== MAVLink methods =====

    def send_mavlink(self, message: Any) -> None:
        """
        Send a MAVLink message.

        Args:
            message: MAVLink message to send
        """
        self._master.mav.send(message)

    def flush(self) -> None:
        """Flush pending messages."""
        pass  # No-op in current implementation

    # ===== Message listeners =====

    def on_message(self, name: str | list[str]) -> Callable[[Any], Any]:
        """
        Decorator for message listeners.

        Args:
            name: Message type(s) to listen for

        Example:
            >>> @vehicle.on_message('HEARTBEAT')
            ... def heartbeat_callback(self, name, message):
            ...     print("Got heartbeat")
        """

        def decorator(
            fn: Callable[[Any, str, Any], None],
        ) -> Callable[[Any, str, Any], None]:
            if isinstance(name, list):
                for n in name:
                    self._legacy_adapter.add_message_listener(n, fn, self)
            elif name == "*":
                self._legacy_adapter.add_wildcard_listener(fn, self)
            else:
                self._legacy_adapter.add_message_listener(name, fn, self)
            return fn

        return decorator

    def add_message_listener(
        self,
        name: str,
        fn: Callable[[Any, str, Any], None],
    ) -> None:
        """
        Add a message listener.

        Args:
            name: Message type to listen for
            fn: Callback function
        """
        if name == "*":
            self._legacy_adapter.add_wildcard_listener(fn, self)
        else:
            self._legacy_adapter.add_message_listener(name, fn, self)

    def remove_message_listener(
        self,
        name: str,
        fn: Callable[[Any, str, Any], None],
    ) -> None:
        """
        Remove a message listener.

        Args:
            name: Message type
            fn: Callback function to remove
        """
        if name == "*":
            self._legacy_adapter.remove_wildcard_listener(fn)
        else:
            self._legacy_adapter.remove_message_listener(name, fn)

    # ===== Wait methods =====

    def wait_ready(
        self,
        *args: str,
        timeout: float | None = None,
        raise_exception: bool = True,
    ) -> bool:
        """
        Wait for specified attributes to be ready.

        Args:
            *args: Attribute names to wait for
            timeout: Timeout in seconds
            raise_exception: Whether to raise on timeout

        Returns:
            True if all attributes ready, False if timeout

        Example:
            >>> vehicle.wait_ready('mode', 'armed', timeout=30)
        """
        if not args:
            args = ("parameters", "gps_0", "armed", "mode", "attitude")

        start = monotonic.monotonic()
        for attr in args:
            while True:
                if timeout and (monotonic.monotonic() - start) > timeout:
                    if raise_exception:
                        raise TimeoutError(f"Timeout waiting for attribute: {attr}")
                    return False

                value = self._get_attribute_value(attr)
                if value is not None:
                    break
                time.sleep(0.1)

        return True

    def _get_attribute_value(self, attr: str) -> Any:
        """Get the value of an attribute by name."""
        if attr == "parameters":
            return self._parameters.is_loaded or None
        elif attr == "gps_0":
            return self._sensors.gps
        elif attr == "armed":
            return self._flight_control.armed
        elif attr == "mode":
            return self._flight_control.mode
        elif attr == "attitude":
            return self._sensors.attitude
        else:
            return getattr(self, attr, None)

    # ===== Reboot =====

    def reboot(self) -> None:
        """Reboot the autopilot."""
        self._flight_control.reboot()

    # ===== Connection management =====

    def close(self) -> None:
        """Close the connection."""
        self._handler.close()

    @property
    def _vehicle(self) -> "Vehicle":
        """Self-reference for backwards compatibility."""
        return self

    def __repr__(self) -> str:
        mode = self.mode.name if self.mode else "Unknown"
        armed = "Armed" if self.armed else "Disarmed"
        return f"<Vehicle mode={mode} {armed}>"
