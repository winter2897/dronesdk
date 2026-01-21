"""
Parameter Manager.

This module provides parameter management for vehicle parameters
using a MutableMapping interface.
"""

from __future__ import annotations

import logging
import struct
import sys
import time
from collections.abc import MutableMapping
from typing import TYPE_CHECKING, Any, Callable, Iterator

import monotonic

from dronesdk.core.exceptions import APIException
from dronesdk.core.observer import HasObservers

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent
    from dronesdk.datalink.connection import MAVConnection

logger = logging.getLogger(__name__)


class ParameterManager(MutableMapping, HasObservers):
    """
    Manages vehicle parameters with MutableMapping interface.

    This class provides dictionary-like access to vehicle parameters
    with automatic download and caching.

    Parameters are case-insensitive and are automatically converted
    to uppercase.

    Example:
        >>> params = vehicle.parameters
        >>> print(params['THR_MIN'])
        130.0
        >>> params['THR_MIN'] = 100
    """

    __slots__ = (
        "_params",
        "_params_count",
        "_params_set",
        "_connection",
        "_loaded",
        "_wait_ready_event",
        "_unsubscribe",
    )

    def __init__(self) -> None:
        super().__init__()
        self._params: dict[str, float] = {}
        self._params_count: int = -1
        self._params_set: set[str] = set()
        self._connection: MAVConnection | None = None
        self._loaded = False
        self._unsubscribe: Callable[[], None] | None = None

    @property
    def name(self) -> str:
        """Module name."""
        return "parameters"

    def attach(
        self,
        event_bus: "EventBus",
        connection: "MAVConnection",
    ) -> None:
        """
        Attach to an event bus and connection.

        Args:
            event_bus: The event bus to subscribe to
            connection: The MAVConnection for sending parameter requests
        """
        self._connection = connection
        self._unsubscribe = event_bus.subscribe_message(
            "PARAM_VALUE",
            self._handle_param_value,
        )
        logger.debug("ParameterManager attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        if self._unsubscribe:
            self._unsubscribe()
            self._unsubscribe = None
        logger.debug("ParameterManager detached from event bus")

    def initialize(self) -> None:
        """Initialize by requesting parameter list."""
        self._request_all_parameters()

    def _request_all_parameters(self) -> None:
        """Request all parameters from the vehicle."""
        if self._connection:
            self._connection.master.param_fetch_all()

    def _handle_param_value(self, event: "MAVLinkMessageEvent") -> None:
        """Handle PARAM_VALUE message."""
        msg = event.message
        param_id = msg.param_id

        # Handle null-terminated strings
        if isinstance(param_id, bytes):
            param_id = param_id.decode("utf-8")
        param_id = param_id.rstrip("\x00").upper()

        # Store the value
        old_value = self._params.get(param_id)
        self._params[param_id] = msg.param_value
        self._params_set.add(param_id)

        # Track total count
        if self._params_count == -1:
            self._params_count = msg.param_count

        # Notify listeners if value changed
        if old_value != msg.param_value:
            self.notify_attribute_listeners(param_id, msg.param_value)
            self.notify_attribute_listeners("*", msg.param_value)

        # Check if all parameters loaded
        if not self._loaded and len(self._params_set) >= self._params_count > 0:
            self._loaded = True
            logger.info("All %d parameters loaded", self._params_count)

    @property
    def is_loaded(self) -> bool:
        """Check if all parameters have been loaded."""
        return self._loaded

    def wait_ready(self, timeout: float | None = None) -> bool:
        """
        Wait for parameters to be loaded.

        Args:
            timeout: Maximum time to wait in seconds (None = forever)

        Returns:
            True if parameters loaded, False if timeout

        Raises:
            APIException: If connection is lost
        """
        start = monotonic.monotonic()
        while not self._loaded:
            if timeout and (monotonic.monotonic() - start) > timeout:
                return False
            if self._connection and not self._connection.is_alive:
                raise APIException("Connection lost while waiting for parameters")
            time.sleep(0.1)
        return True

    def __getitem__(self, name: str) -> float:
        """Get a parameter value."""
        name = name.upper()
        self.wait_ready()
        if name not in self._params:
            raise KeyError(f"Parameter {name} not found")
        return self._params[name]

    def __setitem__(self, name: str, value: float) -> None:
        """Set a parameter value."""
        name = name.upper()
        self.wait_ready()
        self.set(name, value)

    def __delitem__(self, name: str) -> None:
        """Delete is not supported for parameters."""
        raise APIException("Cannot delete parameter from vehicle")

    def __len__(self) -> int:
        """Return number of parameters."""
        return len(self._params)

    def __iter__(self) -> Iterator[str]:
        """Iterate over parameter names."""
        return iter(self._params)

    def __contains__(self, name: object) -> bool:
        """Check if parameter exists."""
        if isinstance(name, str):
            return name.upper() in self._params
        return False

    def get(  # type: ignore[override]
        self,
        name: str,
        default: float | None = None,
        wait_ready: bool = True,
    ) -> float | None:
        """
        Get a parameter value.

        Args:
            name: Parameter name (case-insensitive)
            default: Value to return if parameter not found
            wait_ready: Whether to wait for parameters to load

        Returns:
            Parameter value or default
        """
        name = name.upper()
        if wait_ready:
            self.wait_ready()
        return self._params.get(name, default)

    def set(
        self,
        name: str,
        value: float,
        retries: int = 3,
        wait_ready: bool = False,
    ) -> bool:
        """
        Set a parameter value.

        Args:
            name: Parameter name (case-insensitive)
            value: Value to set
            retries: Number of retries if set fails
            wait_ready: Whether to wait for parameters first

        Returns:
            True if parameter was set successfully
        """
        if wait_ready:
            self.wait_ready()

        if not self._connection:
            logger.error("Cannot set parameter: no connection")
            return False

        name = name.upper()
        # Convert to single precision float (MAVLink uses float32)
        value = float(struct.unpack("f", struct.pack("f", value))[0])

        remaining = retries
        while True:
            self._connection.master.param_set_send(name, value)
            tstart = monotonic.monotonic()

            if remaining == 0:
                break
            remaining -= 1

            # Wait for acknowledgment
            while monotonic.monotonic() - tstart < 1:
                if name in self._params and self._params[name] == value:
                    return True
                time.sleep(0.1)

        if retries > 0:
            logger.error("Timeout setting parameter %s to %f", name, value)
        return False

    def add_attribute_listener(
        self,
        attr_name: str,
        observer: Callable[[Any, str, Any], None],
    ) -> None:
        """
        Add a listener for parameter changes.

        Args:
            attr_name: Parameter name (or '*' for all)
            observer: Callback function
        """
        attr_name = attr_name.upper()
        super().add_attribute_listener(attr_name, observer)

    def remove_attribute_listener(
        self,
        attr_name: str,
        observer: Callable[[Any, str, Any], None],
    ) -> None:
        """Remove a parameter listener."""
        attr_name = attr_name.upper()
        super().remove_attribute_listener(attr_name, observer)

    def on_attribute(
        self,
        name: str | list[str],
    ) -> Callable[[Callable[[Any, str, Any], None]], None]:
        """
        Decorator for parameter listeners.

        Args:
            name: Parameter name(s) to watch

        Returns:
            Decorator function
        """
        if isinstance(name, str):
            name = name.upper()
        elif isinstance(name, list):
            name = [n.upper() for n in name]
        return super().on_attribute(name)
