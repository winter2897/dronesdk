"""
Message Router.

This module provides the bridge between MAVConnection and EventBus,
routing incoming MAVLink messages to the event system.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import monotonic

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus
    from dronesdk.datalink.connection import MAVConnection

logger = logging.getLogger(__name__)


class MessageRouter:
    """
    Routes MAVLink messages from a connection to an event bus.

    This class bridges the low-level MAVConnection to the high-level
    EventBus, converting raw MAVLink messages into events.

    Args:
        connection: The MAVConnection to receive messages from
        event_bus: The EventBus to publish events to
    """

    __slots__ = ("_connection", "_event_bus", "_attached")

    def __init__(
        self,
        connection: "MAVConnection",
        event_bus: "EventBus",
    ) -> None:
        self._connection = connection
        self._event_bus = event_bus
        self._attached = False

    def attach(self) -> None:
        """
        Attach to the connection and start routing messages.

        After calling this method, all incoming MAVLink messages
        will be published to the event bus.
        """
        if self._attached:
            return

        @self._connection.forward_message
        def on_message(_: "MAVConnection", msg: Any) -> None:
            self._route_message(msg)

        self._attached = True
        logger.debug("MessageRouter attached to connection")

    def _route_message(self, msg: Any) -> None:
        """Route a MAVLink message to the event bus."""
        from dronesdk.core.events import MAVLinkMessageEvent

        event = MAVLinkMessageEvent(
            timestamp=monotonic.monotonic(),
            message_type=msg.get_type(),
            message=msg,
        )
        self._event_bus.publish_message(event)

    def detach(self) -> None:
        """
        Detach from the connection.

        Note: Currently this doesn't actually remove the listener
        from the connection as MAVConnection doesn't support removal.
        """
        self._attached = False
        logger.debug("MessageRouter detached from connection")

    @property
    def is_attached(self) -> bool:
        """Check if router is attached to connection."""
        return self._attached


class LegacyMessageAdapter:
    """
    Adapter for legacy message listener patterns.

    This class provides backwards compatibility for the old-style
    message listeners used by Vehicle.on_message().

    It subscribes to the event bus and calls legacy-style callbacks
    with the signature: callback(vehicle, message_name, message)
    """

    __slots__ = ("_event_bus", "_listeners", "_unsubscribe_all")

    def __init__(self, event_bus: "EventBus") -> None:
        self._event_bus = event_bus
        self._listeners: dict[str, list[Any]] = {}
        self._unsubscribe_all: list[Any] = []

    def add_message_listener(
        self,
        message_type: str,
        callback: Any,
        vehicle: Any,
    ) -> None:
        """
        Add a legacy-style message listener.

        Args:
            message_type: MAVLink message type to listen for
            callback: Callback function with signature (vehicle, name, msg)
            vehicle: Vehicle instance to pass to callback
        """
        from dronesdk.core.events import MAVLinkMessageEvent

        def adapter(event: MAVLinkMessageEvent) -> None:
            try:
                callback(vehicle, event.message_type, event.message)
            except Exception:
                logger.exception(
                    "Exception in message listener for %s",
                    message_type,
                )

        unsubscribe = self._event_bus.subscribe_message(message_type, adapter)

        if message_type not in self._listeners:
            self._listeners[message_type] = []
        self._listeners[message_type].append((callback, unsubscribe))

    def remove_message_listener(
        self,
        message_type: str,
        callback: Any,
    ) -> None:
        """
        Remove a legacy-style message listener.

        Args:
            message_type: MAVLink message type
            callback: The callback function to remove
        """
        if message_type not in self._listeners:
            return

        for cb, unsub in self._listeners[message_type]:
            if cb == callback:
                unsub()
                self._listeners[message_type].remove((cb, unsub))
                break

        if not self._listeners[message_type]:
            del self._listeners[message_type]

    def add_wildcard_listener(self, callback: Any, vehicle: Any) -> None:
        """
        Add a listener for all messages.

        Args:
            callback: Callback function with signature (vehicle, name, msg)
            vehicle: Vehicle instance to pass to callback
        """
        from dronesdk.core.events import MAVLinkMessageEvent

        def adapter(event: MAVLinkMessageEvent) -> None:
            try:
                callback(vehicle, event.message_type, event.message)
            except Exception:
                logger.exception(
                    "Exception in wildcard message listener for %s",
                    event.message_type,
                )

        unsubscribe = self._event_bus.subscribe_all_messages(adapter)
        self._unsubscribe_all.append((callback, unsubscribe))

    def remove_wildcard_listener(self, callback: Any) -> None:
        """Remove a wildcard listener."""
        for cb, unsub in self._unsubscribe_all:
            if cb == callback:
                unsub()
                self._unsubscribe_all.remove((cb, unsub))
                break

    def clear(self) -> None:
        """Remove all listeners."""
        for listeners in self._listeners.values():
            for _, unsub in listeners:
                unsub()
        self._listeners.clear()

        for _, unsub in self._unsubscribe_all:
            unsub()
        self._unsubscribe_all.clear()
