"""
Locations Container.

This module provides the Locations class for managing vehicle location
in different coordinate frames.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Callable

from dronesdk.core.observer import HasObservers
from dronesdk.models.location import (
    LocationGlobal,
    LocationGlobalRelative,
    LocationLocal,
)

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent

logger = logging.getLogger(__name__)


class Locations(HasObservers):
    """
    Container for vehicle location in different coordinate frames.

    This class manages location data in three frames:
    - Global (WGS84 with MSL altitude)
    - Global Relative (WGS84 with altitude relative to home)
    - Local NED (North-East-Down relative to EKF origin)

    The location data is updated from GLOBAL_POSITION_INT and
    LOCAL_POSITION_NED MAVLink messages.
    """

    __slots__ = (
        "_lat",
        "_lon",
        "_alt",
        "_relative_alt",
        "_north",
        "_east",
        "_down",
        "_on_global_update",
        "_on_local_update",
        "_unsubscribe_fns",
    )

    def __init__(
        self,
        on_global_update: Callable[[LocationGlobal], None] | None = None,
        on_local_update: Callable[[LocationLocal], None] | None = None,
    ) -> None:
        super().__init__()

        # Global position
        self._lat: float | None = None
        self._lon: float | None = None
        self._alt: float | None = None
        self._relative_alt: float | None = None

        # Local position
        self._north: float | None = None
        self._east: float | None = None
        self._down: float | None = None

        # Callbacks
        self._on_global_update = on_global_update
        self._on_local_update = on_local_update
        self._unsubscribe_fns: list[Callable[[], None]] = []

    @property
    def name(self) -> str:
        """Module name."""
        return "location"

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive position messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        unsub1 = event_bus.subscribe_message(
            "GLOBAL_POSITION_INT",
            self._handle_global_position,
        )
        unsub2 = event_bus.subscribe_message(
            "LOCAL_POSITION_NED",
            self._handle_local_position,
        )
        self._unsubscribe_fns = [unsub1, unsub2]
        logger.debug("Locations attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        for unsub in self._unsubscribe_fns:
            unsub()
        self._unsubscribe_fns.clear()
        logger.debug("Locations detached from event bus")

    def initialize(self) -> None:
        """Initialize the module."""
        pass

    def _handle_global_position(self, event: "MAVLinkMessageEvent") -> None:
        """Handle GLOBAL_POSITION_INT message."""
        msg = event.message
        self._lat = msg.lat / 1.0e7
        self._lon = msg.lon / 1.0e7
        self._relative_alt = msg.relative_alt / 1000.0

        # Notify listeners for global_relative_frame
        self.notify_attribute_listeners(
            "global_relative_frame",
            self.global_relative_frame,
        )

        # Only set alt if non-zero (wait for valid barometer reading)
        if self._alt is not None or msg.alt != 0:
            self._alt = msg.alt / 1000.0
            self.notify_attribute_listeners(
                "global_frame",
                self.global_frame,
            )

        if self._on_global_update:
            try:
                self._on_global_update(self.global_frame)
            except Exception:
                logger.exception("Error in global location callback")

    def _handle_local_position(self, event: "MAVLinkMessageEvent") -> None:
        """Handle LOCAL_POSITION_NED message."""
        msg = event.message
        self._north = msg.x
        self._east = msg.y
        self._down = msg.z

        self.notify_attribute_listeners("local_frame", self.local_frame)

        if self._on_local_update:
            try:
                self._on_local_update(self.local_frame)
            except Exception:
                logger.exception("Error in local location callback")

    @property
    def local_frame(self) -> LocationLocal:
        """
        Location in local NED frame.

        Returns:
            LocationLocal with north, east, down coordinates.
            Note: This will not update until the vehicle is armed.
        """
        return LocationLocal(self._north, self._east, self._down)

    @property
    def global_frame(self) -> LocationGlobal:
        """
        Location in global frame (WGS84 with MSL altitude).

        Returns:
            LocationGlobal with lat, lon, alt.
            Note: Alt may take several seconds to populate from barometer.
        """
        return LocationGlobal(self._lat, self._lon, self._alt)

    @property
    def global_relative_frame(self) -> LocationGlobalRelative:
        """
        Location in global relative frame (WGS84 with altitude relative to home).

        Returns:
            LocationGlobalRelative with lat, lon, alt relative to home.
        """
        return LocationGlobalRelative(self._lat, self._lon, self._relative_alt)

    def __str__(self) -> str:
        return (
            f"Locations(global={self.global_frame}, "
            f"global_relative={self.global_relative_frame}, "
            f"local={self.local_frame})"
        )
