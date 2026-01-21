"""
Channels Reader.

This module provides RC channel reading functionality.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Callable, Iterator

from dronesdk.channels.override import ChannelsOverride

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent
    from dronesdk.datalink.connection import MAVConnection

logger = logging.getLogger(__name__)


class Channels(dict):
    """
    A dictionary class for managing RC channel information.

    Provides read-only access to current RC channel values and
    read-write access to channel overrides via the `overrides` property.

    Example:
        >>> print(vehicle.channels['1'])  # Read channel 1
        1500
        >>> vehicle.channels.overrides['3'] = 1600  # Override channel 3
    """

    __slots__ = ("_connection", "_count", "_overrides", "_readonly", "_unsubscribe")

    def __init__(
        self,
        connection: "MAVConnection | None" = None,
        count: int = 8,
    ) -> None:
        super().__init__()
        self._connection = connection
        self._count = count
        self._overrides = ChannelsOverride(connection)
        self._unsubscribe: Callable[[], None] | None = None

        # Initialize with None values
        self._readonly = False
        for k in range(count):
            self[k + 1] = None
        self._readonly = True

    def set_connection(self, connection: "MAVConnection") -> None:
        """Set the MAVLink connection."""
        self._connection = connection
        self._overrides.set_connection(connection)

    @property
    def name(self) -> str:
        """Module name."""
        return "channels"

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive RC channel messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        self._unsubscribe = event_bus.subscribe_message(
            "RC_CHANNELS_RAW",
            self._handle_rc_channels,
        )
        # Also subscribe to RC_CHANNELS for newer protocol
        event_bus.subscribe_message(
            "RC_CHANNELS",
            self._handle_rc_channels_new,
        )
        logger.debug("Channels attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        if self._unsubscribe:
            self._unsubscribe()
            self._unsubscribe = None
        logger.debug("Channels detached from event bus")

    def initialize(self) -> None:
        """Initialize the module."""
        pass

    def _handle_rc_channels(self, event: "MAVLinkMessageEvent") -> None:
        """Handle RC_CHANNELS_RAW message."""
        msg = event.message
        # Update channels 1-8
        for i in range(1, 9):
            value = getattr(msg, f"chan{i}_raw", None)
            if value is not None:
                self._update_channel(i, value)

    def _handle_rc_channels_new(self, event: "MAVLinkMessageEvent") -> None:
        """Handle RC_CHANNELS message (newer protocol)."""
        msg = event.message
        # Update all available channels
        for i in range(1, 19):
            attr = f"chan{i}_raw"
            if hasattr(msg, attr):
                value = getattr(msg, attr)
                if value != 65535:  # 65535 = not available
                    self._update_channel(i, value)

    @property
    def count(self) -> int:
        """The number of channels available."""
        return self._count

    def __getitem__(self, key: str | int) -> int | None:
        """Get a channel value by number."""
        return dict.__getitem__(self, str(key))

    def __setitem__(self, key: str | int, value: int | None) -> None:
        """Set a channel value (only allowed internally)."""
        if self._readonly:
            raise TypeError("Channel values are read-only")
        return dict.__setitem__(self, str(key), value)

    def __len__(self) -> int:
        """Return the number of channels."""
        return self._count

    def _update_channel(self, channel: int, value: int) -> None:
        """
        Update a channel value (internal use).

        Expands the channel count if necessary.
        """
        channel = int(channel)
        self._readonly = False
        self[channel] = value
        self._readonly = True
        self._count = max(self._count, channel)

    @property
    def overrides(self) -> ChannelsOverride:
        """
        Get or set channel overrides.

        Returns:
            ChannelsOverride dictionary for reading/writing overrides

        Example:
            >>> vehicle.channels.overrides = {'5': None, '3': 500}
            >>> vehicle.channels.overrides['2'] = 200
            >>> del vehicle.channels.overrides['3']
            >>> vehicle.channels.overrides = {}  # Clear all
        """
        return self._overrides

    @overrides.setter
    def overrides(self, newch: dict[str | int, int | None]) -> None:
        """Set multiple overrides at once."""
        self._overrides._active = False
        self._overrides.clear()
        for k, v in newch.items():
            if v:
                self._overrides[str(k)] = v
            else:
                try:
                    del self._overrides[str(k)]
                except KeyError:
                    pass
        self._overrides._active = True
        self._overrides._send()

    def __str__(self) -> str:
        """Return string representation of channel values."""
        values = [f"{k}:{v}" for k, v in sorted(self.items(), key=lambda x: int(x[0]))]
        return f"Channels({', '.join(values)})"
