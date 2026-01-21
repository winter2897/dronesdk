"""
Sensor Manager.

This module manages various sensor data including attitude, GPS,
velocity, wind, and rangefinder readings.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Callable

from dronesdk.models.attitude import Attitude
from dronesdk.models.sensors import GPSInfo, Rangefinder, Wind

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent

logger = logging.getLogger(__name__)


class SensorManager:
    """
    Manages sensor data from MAVLink messages.

    This module subscribes to various sensor-related MAVLink messages
    and provides access to the current sensor readings.

    Args:
        on_attitude_update: Optional callback when attitude changes
        on_gps_update: Optional callback when GPS data changes
    """

    __slots__ = (
        "_attitude",
        "_gps",
        "_rangefinder",
        "_wind",
        "_velocity",
        "_heading",
        "_airspeed",
        "_groundspeed",
        "_on_attitude_update",
        "_on_gps_update",
        "_unsubscribe_fns",
    )

    def __init__(
        self,
        on_attitude_update: Callable[[Attitude], None] | None = None,
        on_gps_update: Callable[[GPSInfo], None] | None = None,
    ) -> None:
        self._attitude: Attitude | None = None
        self._gps: GPSInfo | None = None
        self._rangefinder: Rangefinder = Rangefinder(distance=None, voltage=None)
        self._wind: Wind | None = None
        self._velocity: list[float] | None = None
        self._heading: int | None = None
        self._airspeed: float | None = None
        self._groundspeed: float | None = None
        self._on_attitude_update = on_attitude_update
        self._on_gps_update = on_gps_update
        self._unsubscribe_fns: list[Callable[[], None]] = []

    @property
    def name(self) -> str:
        """Module name."""
        return "sensors"

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive sensor messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        subscriptions = [
            ("ATTITUDE", self._handle_attitude),
            ("GPS_RAW_INT", self._handle_gps_raw),
            ("RANGEFINDER", self._handle_rangefinder),
            ("WIND", self._handle_wind),
            ("GLOBAL_POSITION_INT", self._handle_global_position),
            ("VFR_HUD", self._handle_vfr_hud),
        ]

        for msg_type, handler in subscriptions:
            unsub = event_bus.subscribe_message(msg_type, handler)
            self._unsubscribe_fns.append(unsub)

        logger.debug("SensorManager attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        for unsub in self._unsubscribe_fns:
            unsub()
        self._unsubscribe_fns.clear()
        logger.debug("SensorManager detached from event bus")

    def initialize(self) -> None:
        """Initialize the module."""
        pass

    def _handle_attitude(self, event: "MAVLinkMessageEvent") -> None:
        """Handle ATTITUDE message."""
        msg = event.message
        self._attitude = Attitude.from_mavlink(
            pitch=msg.pitch,
            yaw=msg.yaw,
            roll=msg.roll,
        )

        if self._on_attitude_update:
            try:
                self._on_attitude_update(self._attitude)
            except Exception:
                logger.exception("Error in attitude update callback")

    def _handle_gps_raw(self, event: "MAVLinkMessageEvent") -> None:
        """Handle GPS_RAW_INT message."""
        msg = event.message
        self._gps = GPSInfo.from_mavlink(
            eph=msg.eph,
            epv=msg.epv,
            fix_type=msg.fix_type,
            satellites_visible=msg.satellites_visible,
        )

        if self._on_gps_update:
            try:
                self._on_gps_update(self._gps)
            except Exception:
                logger.exception("Error in GPS update callback")

    def _handle_rangefinder(self, event: "MAVLinkMessageEvent") -> None:
        """Handle RANGEFINDER message."""
        msg = event.message
        self._rangefinder = Rangefinder.from_mavlink(
            distance=msg.distance,
            voltage=msg.voltage,
        )

    def _handle_wind(self, event: "MAVLinkMessageEvent") -> None:
        """Handle WIND message."""
        msg = event.message
        self._wind = Wind.from_mavlink(
            direction=msg.direction,
            speed=msg.speed,
            speed_z=msg.speed_z,
        )

    def _handle_global_position(self, event: "MAVLinkMessageEvent") -> None:
        """Handle GLOBAL_POSITION_INT message."""
        msg = event.message
        # Velocity in cm/s -> m/s
        self._velocity = [msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0]
        # Heading in centidegrees -> degrees
        self._heading = int(msg.hdg / 100) if msg.hdg != 65535 else None

    def _handle_vfr_hud(self, event: "MAVLinkMessageEvent") -> None:
        """Handle VFR_HUD message."""
        msg = event.message
        self._groundspeed = msg.groundspeed
        self._airspeed = msg.airspeed
        # Also update heading from VFR_HUD if available
        if hasattr(msg, "heading"):
            self._heading = msg.heading

    @property
    def attitude(self) -> Attitude | None:
        """Get current attitude."""
        return self._attitude

    @property
    def gps(self) -> GPSInfo | None:
        """Get current GPS info."""
        return self._gps

    @property
    def rangefinder(self) -> Rangefinder:
        """Get current rangefinder reading."""
        return self._rangefinder

    @property
    def wind(self) -> Wind | None:
        """Get current wind information."""
        return self._wind

    @property
    def velocity(self) -> list[float] | None:
        """Get current velocity [vx, vy, vz] in m/s."""
        return self._velocity

    @property
    def heading(self) -> int | None:
        """Get current heading in degrees (0-360)."""
        return self._heading

    @property
    def airspeed(self) -> float | None:
        """Get current airspeed in m/s."""
        return self._airspeed

    @property
    def groundspeed(self) -> float | None:
        """Get current groundspeed in m/s."""
        return self._groundspeed
