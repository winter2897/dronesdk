"""
Battery Monitor.

This module provides battery monitoring functionality using MAVLink
SYS_STATUS messages.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Callable

from dronesdk.models.battery import Battery

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent

logger = logging.getLogger(__name__)


class BatteryMonitor:
    """
    Monitors battery status from MAVLink messages.

    This module subscribes to SYS_STATUS messages and extracts
    battery information.

    Args:
        on_update: Optional callback when battery data is updated.
                  Signature: callback(battery: Battery)
    """

    __slots__ = (
        "_battery",
        "_on_update",
        "_unsubscribe",
    )

    def __init__(
        self,
        on_update: Callable[[Battery], None] | None = None,
    ) -> None:
        self._battery: Battery | None = None
        self._on_update = on_update
        self._unsubscribe: Callable[[], None] | None = None

    @property
    def name(self) -> str:
        """Module name."""
        return "power"

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive SYS_STATUS messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        self._unsubscribe = event_bus.subscribe_message(
            "SYS_STATUS",
            self._handle_sys_status,
        )
        logger.debug("BatteryMonitor attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        if self._unsubscribe:
            self._unsubscribe()
            self._unsubscribe = None
        logger.debug("BatteryMonitor detached from event bus")

    def initialize(self) -> None:
        """Initialize the module (no-op for battery monitor)."""
        pass

    def _handle_sys_status(self, event: "MAVLinkMessageEvent") -> None:
        """Handle SYS_STATUS message."""
        msg = event.message
        self._battery = Battery.from_mavlink(
            voltage_mv=msg.voltage_battery,
            current_ca=msg.current_battery,
            battery_remaining=msg.battery_remaining,
        )

        if self._on_update:
            try:
                self._on_update(self._battery)
            except Exception:
                logger.exception("Error in battery update callback")

    @property
    def battery(self) -> Battery | None:
        """Get current battery status."""
        return self._battery

    @property
    def voltage(self) -> float | None:
        """Get battery voltage in volts."""
        return self._battery.voltage if self._battery else None

    @property
    def current(self) -> float | None:
        """Get battery current in amperes."""
        return self._battery.current if self._battery else None

    @property
    def level(self) -> int | None:
        """Get battery level as percentage."""
        return self._battery.level if self._battery else None

    @property
    def is_low(self) -> bool | None:
        """Check if battery is low (below 20%)."""
        return self._battery.is_low if self._battery else None

    @property
    def is_critical(self) -> bool | None:
        """Check if battery is critical (below 10%)."""
        return self._battery.is_critical if self._battery else None
