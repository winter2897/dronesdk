"""
Event Bus Implementation.

This module provides an event bus system for decoupled message routing
in dronesdk. It allows modules to subscribe to MAVLink messages and
other events without tight coupling.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Callable

logger = logging.getLogger(__name__)


class EventPriority(IntEnum):
    """Priority levels for event handlers."""

    HIGH = 0  # Processed first (e.g., critical state updates)
    NORMAL = 50  # Default priority
    LOW = 100  # Processed last (e.g., logging, telemetry)


@dataclass(frozen=True)
class MAVLinkMessageEvent:
    """
    Event representing a received MAVLink message.

    Attributes:
        timestamp: Monotonic timestamp when the message was received
        message_type: The MAVLink message type name (e.g., 'HEARTBEAT')
        message: The actual MAVLink message object
    """

    timestamp: float
    message_type: str
    message: Any


@dataclass(frozen=True)
class AttributeChangedEvent:
    """
    Event representing an attribute change.

    Attributes:
        timestamp: Monotonic timestamp when the change was detected
        attribute_name: Name of the changed attribute
        old_value: Previous value (may be None)
        new_value: New value
    """

    timestamp: float
    attribute_name: str
    old_value: Any
    new_value: Any


@dataclass
class _Subscription:
    """Internal class representing a subscription."""

    handler: Callable[[Any], None]
    priority: EventPriority = EventPriority.NORMAL


@dataclass
class _MessageSubscription(_Subscription):
    """Subscription for a specific message type."""

    message_type: str = ""


class EventBus:
    """
    Central event bus for routing MAVLink messages and internal events.

    The EventBus provides a decoupled way for modules to subscribe to
    messages and events without direct dependencies on each other.

    Features:
    - Priority-based handler execution
    - Thread-safe subscription management
    - Support for message type filtering
    - Wildcard subscriptions for all messages

    Example:
        >>> bus = EventBus()
        >>> def handle_heartbeat(event: MAVLinkMessageEvent):
        ...     print(f"Got heartbeat at {event.timestamp}")
        >>> unsubscribe = bus.subscribe_message('HEARTBEAT', handle_heartbeat)
        >>> # Later...
        >>> unsubscribe()  # Remove subscription
    """

    __slots__ = (
        "_message_handlers",
        "_wildcard_handlers",
        "_attribute_handlers",
        "_lock",
    )

    def __init__(self) -> None:
        self._lock = threading.RLock()
        # Map from message_type to list of subscriptions
        self._message_handlers: dict[str, list[_MessageSubscription]] = {}
        # Handlers for all messages (wildcard)
        self._wildcard_handlers: list[_Subscription] = []
        # Map from attribute_name to list of subscriptions
        self._attribute_handlers: dict[str, list[_Subscription]] = {}

    def subscribe_message(
        self,
        message_type: str,
        handler: Callable[[MAVLinkMessageEvent], None],
        priority: EventPriority = EventPriority.NORMAL,
    ) -> Callable[[], None]:
        """
        Subscribe to a specific MAVLink message type.

        Args:
            message_type: The MAVLink message type to subscribe to
                         (e.g., 'HEARTBEAT', 'ATTITUDE')
            handler: Callback function that receives MAVLinkMessageEvent
            priority: Handler priority (lower values = higher priority)

        Returns:
            Unsubscribe function - call it to remove the subscription
        """
        subscription = _MessageSubscription(
            handler=handler,
            priority=priority,
            message_type=message_type,
        )

        with self._lock:
            if message_type not in self._message_handlers:
                self._message_handlers[message_type] = []
            handlers = self._message_handlers[message_type]
            handlers.append(subscription)
            # Sort by priority
            handlers.sort(key=lambda s: s.priority)

        def unsubscribe() -> None:
            with self._lock:
                if message_type in self._message_handlers:
                    try:
                        self._message_handlers[message_type].remove(subscription)
                        if not self._message_handlers[message_type]:
                            del self._message_handlers[message_type]
                    except ValueError:
                        pass  # Already removed

        return unsubscribe

    def subscribe_all_messages(
        self,
        handler: Callable[[MAVLinkMessageEvent], None],
        priority: EventPriority = EventPriority.NORMAL,
    ) -> Callable[[], None]:
        """
        Subscribe to all MAVLink messages.

        Args:
            handler: Callback function that receives MAVLinkMessageEvent
            priority: Handler priority (lower values = higher priority)

        Returns:
            Unsubscribe function - call it to remove the subscription
        """
        subscription = _Subscription(handler=handler, priority=priority)

        with self._lock:
            self._wildcard_handlers.append(subscription)
            self._wildcard_handlers.sort(key=lambda s: s.priority)

        def unsubscribe() -> None:
            with self._lock:
                try:
                    self._wildcard_handlers.remove(subscription)
                except ValueError:
                    pass

        return unsubscribe

    def subscribe_attribute(
        self,
        attribute_name: str,
        handler: Callable[[AttributeChangedEvent], None],
        priority: EventPriority = EventPriority.NORMAL,
    ) -> Callable[[], None]:
        """
        Subscribe to attribute change events.

        Args:
            attribute_name: The attribute to watch (or '*' for all)
            handler: Callback function that receives AttributeChangedEvent
            priority: Handler priority

        Returns:
            Unsubscribe function
        """
        subscription = _Subscription(handler=handler, priority=priority)

        with self._lock:
            if attribute_name not in self._attribute_handlers:
                self._attribute_handlers[attribute_name] = []
            handlers = self._attribute_handlers[attribute_name]
            handlers.append(subscription)
            handlers.sort(key=lambda s: s.priority)

        def unsubscribe() -> None:
            with self._lock:
                if attribute_name in self._attribute_handlers:
                    try:
                        self._attribute_handlers[attribute_name].remove(subscription)
                        if not self._attribute_handlers[attribute_name]:
                            del self._attribute_handlers[attribute_name]
                    except ValueError:
                        pass

        return unsubscribe

    def publish_message(self, event: MAVLinkMessageEvent) -> None:
        """
        Publish a MAVLink message event to all subscribers.

        Args:
            event: The MAVLinkMessageEvent to publish
        """
        with self._lock:
            # Get handlers for this specific message type
            specific_handlers = list(self._message_handlers.get(event.message_type, []))
            # Get wildcard handlers
            wildcard_handlers = list(self._wildcard_handlers)

        # Combine and sort by priority
        all_handlers = specific_handlers + wildcard_handlers
        all_handlers.sort(key=lambda s: s.priority)

        # Invoke handlers outside the lock
        for subscription in all_handlers:
            try:
                subscription.handler(event)
            except Exception:
                logger.exception(
                    "Exception in message handler for %s",
                    event.message_type,
                )

    def publish_attribute_change(self, event: AttributeChangedEvent) -> None:
        """
        Publish an attribute change event to all subscribers.

        Args:
            event: The AttributeChangedEvent to publish
        """
        with self._lock:
            specific_handlers = list(
                self._attribute_handlers.get(event.attribute_name, [])
            )
            wildcard_handlers = list(self._attribute_handlers.get("*", []))

        all_handlers = specific_handlers + wildcard_handlers
        all_handlers.sort(key=lambda s: s.priority)

        for subscription in all_handlers:
            try:
                subscription.handler(event)
            except Exception:
                logger.exception(
                    "Exception in attribute handler for %s",
                    event.attribute_name,
                )

    def clear(self) -> None:
        """Remove all subscriptions."""
        with self._lock:
            self._message_handlers.clear()
            self._wildcard_handlers.clear()
            self._attribute_handlers.clear()
