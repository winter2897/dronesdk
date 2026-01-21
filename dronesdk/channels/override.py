"""
Channels Override.

This module provides RC channel override functionality.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Iterator

if TYPE_CHECKING:
    from dronesdk.datalink.connection import MAVConnection

logger = logging.getLogger(__name__)


class ChannelsOverride(dict):
    """
    A dictionary class for managing vehicle channel overrides.

    Channels can be read, written, or cleared by index or using
    dictionary syntax. To clear a value, set it to None or use del.

    The override values are sent to the vehicle when any change is made.

    Example:
        >>> vehicle.channels.overrides['3'] = 1500  # Set throttle
        >>> vehicle.channels.overrides = {'3': 1500, '4': 1500}  # Set multiple
        >>> del vehicle.channels.overrides['3']  # Clear override
        >>> vehicle.channels.overrides = {}  # Clear all
    """

    __slots__ = ("_connection", "_count", "_active")

    def __init__(self, connection: "MAVConnection | None" = None) -> None:
        super().__init__()
        self._connection = connection
        self._count = 8  # Fixed by MAVLink
        self._active = True

    def set_connection(self, connection: "MAVConnection") -> None:
        """Set the MAVLink connection for sending overrides."""
        self._connection = connection

    def __getitem__(self, key: str | int) -> int:
        """Get an override value by channel number."""
        return dict.__getitem__(self, str(key))

    def __setitem__(self, key: str | int, value: int | None) -> None:
        """Set or clear an override value."""
        channel = int(key)
        if not (0 < channel <= self._count):
            raise KeyError(f"Invalid channel index {key}")

        if not value:
            # Clear override
            try:
                dict.__delitem__(self, str(key))
            except KeyError:
                pass
        else:
            dict.__setitem__(self, str(key), value)

        self._send()

    def __delitem__(self, key: str | int) -> None:
        """Clear an override value."""
        dict.__delitem__(self, str(key))
        self._send()

    def __len__(self) -> int:
        """Return the number of channels (always 8)."""
        return self._count

    def _send(self) -> None:
        """Send the current overrides to the vehicle."""
        if not self._active or not self._connection:
            return

        overrides = [0] * 8
        for k, v in self.items():
            overrides[int(k) - 1] = v

        self._connection.master.mav.rc_channels_override_send(0, 0, *overrides)

    def clear_all(self) -> None:
        """Clear all overrides."""
        self._active = False
        dict.clear(self)
        self._active = True
        self._send()


class ChannelsOverrideManager:
    """
    Manages channel overrides with a cleaner API.

    This provides a more Pythonic interface for channel overrides
    while maintaining backwards compatibility.
    """

    __slots__ = ("_overrides",)

    def __init__(self, connection: "MAVConnection | None" = None) -> None:
        self._overrides = ChannelsOverride(connection)

    def set_connection(self, connection: "MAVConnection") -> None:
        """Set the MAVLink connection."""
        self._overrides.set_connection(connection)

    def set(self, channel: int, value: int) -> None:
        """
        Set an override for a specific channel.

        Args:
            channel: Channel number (1-8)
            value: PWM value (typically 1000-2000)
        """
        self._overrides[channel] = value

    def clear(self, channel: int) -> None:
        """
        Clear an override for a specific channel.

        Args:
            channel: Channel number (1-8)
        """
        if str(channel) in self._overrides:
            del self._overrides[channel]

    def clear_all(self) -> None:
        """Clear all overrides."""
        self._overrides.clear_all()

    def set_multiple(self, overrides: dict[int, int | None]) -> None:
        """
        Set multiple overrides at once.

        Args:
            overrides: Dictionary mapping channel numbers to values.
                      Use None to clear an override.
        """
        self._overrides._active = False
        for channel, value in overrides.items():
            if value is None:
                if str(channel) in self._overrides:
                    dict.__delitem__(self._overrides, str(channel))
            else:
                dict.__setitem__(self._overrides, str(channel), value)
        self._overrides._active = True
        self._overrides._send()

    def get(self, channel: int) -> int | None:
        """
        Get the current override for a channel.

        Args:
            channel: Channel number (1-8)

        Returns:
            Override value or None if not set
        """
        return self._overrides.get(str(channel))

    @property
    def overrides(self) -> ChannelsOverride:
        """Get the underlying ChannelsOverride object."""
        return self._overrides
