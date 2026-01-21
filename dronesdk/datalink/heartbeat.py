"""
Heartbeat Manager.

This module provides heartbeat monitoring for MAVLink connections.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable

import monotonic

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus
    from dronesdk.datalink.connection import MAVConnection

logger = logging.getLogger(__name__)


@dataclass
class HeartbeatStatus:
    """
    Status of heartbeat monitoring.

    Attributes:
        last_heartbeat: Timestamp of last received heartbeat
        last_heartbeat_raw: Raw MAVLink heartbeat message
        is_connected: Whether connection is considered active
        time_since_last: Seconds since last heartbeat
    """

    last_heartbeat: float | None
    last_heartbeat_raw: Any | None
    is_connected: bool
    time_since_last: float | None


class HeartbeatManager:
    """
    Monitors heartbeat messages to track connection status.

    The manager tracks incoming HEARTBEAT messages and maintains
    connection status based on configurable timeout.

    Args:
        timeout: Seconds without heartbeat before connection is dead
        on_connect: Callback when connection is established
        on_disconnect: Callback when connection is lost
    """

    __slots__ = (
        "_timeout",
        "_last_heartbeat",
        "_last_heartbeat_raw",
        "_on_connect",
        "_on_disconnect",
        "_was_connected",
        "_target_system",
        "_target_component",
        "_unsubscribe",
    )

    def __init__(
        self,
        timeout: float = 5.0,
        on_connect: Callable[[], None] | None = None,
        on_disconnect: Callable[[], None] | None = None,
    ) -> None:
        self._timeout = timeout
        self._last_heartbeat: float | None = None
        self._last_heartbeat_raw: Any | None = None
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._was_connected = False
        self._target_system: int | None = None
        self._target_component: int | None = None
        self._unsubscribe: Callable[[], None] | None = None

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive heartbeat messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        from dronesdk.core.events import MAVLinkMessageEvent

        def handle_heartbeat(event: MAVLinkMessageEvent) -> None:
            self._handle_heartbeat(event.message)

        self._unsubscribe = event_bus.subscribe_message(
            "HEARTBEAT",
            handle_heartbeat,
        )

    def attach_to_connection(self, handler: "MAVConnection") -> None:
        """
        Attach directly to a MAVConnection.

        Args:
            handler: The MAVConnection to monitor
        """

        @handler.forward_message
        def on_message(_: Any, msg: Any) -> None:
            if msg.get_type() == "HEARTBEAT":
                self._handle_heartbeat(msg)

    def detach(self) -> None:
        """Detach from the event bus."""
        if self._unsubscribe:
            self._unsubscribe()
            self._unsubscribe = None

    def _handle_heartbeat(self, msg: Any) -> None:
        """Handle an incoming heartbeat message."""
        from pymavlink import mavutil

        # Ignore non-vehicle heartbeats (GCS, etc.)
        if msg.type in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_GIMBAL,
            mavutil.mavlink.MAV_TYPE_ADSB,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        ):
            return

        was_connected = self.is_connected
        self._last_heartbeat = monotonic.monotonic()
        self._last_heartbeat_raw = msg

        # Track the first heartbeat for target system
        if self._target_system is None:
            self._target_system = msg.get_srcSystem()
            self._target_component = msg.get_srcComponent()

        # Notify on connection state change
        if not was_connected and self.is_connected:
            self._was_connected = True
            if self._on_connect:
                try:
                    self._on_connect()
                except Exception:
                    logger.exception("Error in on_connect callback")
        elif was_connected and not self.is_connected:
            self._was_connected = False
            if self._on_disconnect:
                try:
                    self._on_disconnect()
                except Exception:
                    logger.exception("Error in on_disconnect callback")

    @property
    def is_connected(self) -> bool:
        """Check if connection is active (received heartbeat within timeout)."""
        if self._last_heartbeat is None:
            return False
        return (monotonic.monotonic() - self._last_heartbeat) < self._timeout

    @property
    def last_heartbeat(self) -> float | None:
        """Get timestamp of last heartbeat."""
        return self._last_heartbeat

    @property
    def last_heartbeat_raw(self) -> Any | None:
        """Get the raw MAVLink heartbeat message."""
        return self._last_heartbeat_raw

    @property
    def time_since_last(self) -> float | None:
        """Get seconds since last heartbeat."""
        if self._last_heartbeat is None:
            return None
        return monotonic.monotonic() - self._last_heartbeat

    @property
    def target_system(self) -> int | None:
        """Get the detected target system ID."""
        return self._target_system

    @property
    def target_component(self) -> int | None:
        """Get the detected target component ID."""
        return self._target_component

    def get_status(self) -> HeartbeatStatus:
        """Get current heartbeat status."""
        return HeartbeatStatus(
            last_heartbeat=self._last_heartbeat,
            last_heartbeat_raw=self._last_heartbeat_raw,
            is_connected=self.is_connected,
            time_since_last=self.time_since_last,
        )

    def wait_for_connection(self, timeout: float = 30.0) -> bool:
        """
        Wait for a heartbeat to be received.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if connection established, False if timed out
        """
        start = monotonic.monotonic()
        while not self.is_connected:
            if (monotonic.monotonic() - start) > timeout:
                return False
            time.sleep(0.1)
        return True
