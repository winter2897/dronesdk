"""
Observer Pattern Implementation.

This module provides the HasObservers base class that enables attribute
change notifications throughout dronesdk.
"""

from __future__ import annotations

import logging
from typing import Any, Callable

# Type alias for observer callbacks
ObserverCallback = Callable[["HasObservers", str, Any], None]


class HasObservers:
    """
    Base class providing observer pattern for attribute change notifications.

    This class is inherited by Vehicle and other classes that need to
    notify listeners when attributes change.

    Attributes can be observed using two patterns:

    1. Callback-based (allows removal):
        >>> def callback(self, attr_name, value):
        ...     print(f"{attr_name} changed to {value}")
        >>> vehicle.add_attribute_listener('mode', callback)
        >>> vehicle.remove_attribute_listener('mode', callback)

    2. Decorator-based:
        >>> @vehicle.on_attribute('mode')
        ... def mode_callback(self, attr_name, value):
        ...     print(f"Mode changed to {value}")
    """

    __slots__ = ("_logger", "_attribute_listeners", "_attribute_cache")

    def __init__(self) -> None:
        logging.basicConfig()
        self._logger = logging.getLogger(__name__)
        # A mapping from attr_name to a list of observers
        self._attribute_listeners: dict[str, list[ObserverCallback]] = {}
        self._attribute_cache: dict[str, Any] = {}

    def add_attribute_listener(
        self,
        attr_name: str,
        observer: ObserverCallback,
    ) -> None:
        """
        Add an attribute listener callback.

        The callback function is invoked differently depending on the type of attribute.
        Attributes that represent sensor values or which are used to monitor connection
        status are updated whenever a message is received from the vehicle. Attributes
        which reflect vehicle "state" are only updated when their values change
        (e.g., system_status, armed, mode).

        The callback can be removed using :py:func:`remove_attribute_listener`.

        Args:
            attr_name: The name of the attribute to watch (or '*' to watch all).
            observer: The callback to invoke when a change is detected.
                     Signature: observer(self, attr_name, value)

        Note:
            The :py:func:`on_attribute` decorator performs the same operation
            but with more elegant syntax. Use ``add_attribute_listener`` if
            you need to remove the observer later.
        """
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is None:
            listeners_for_attr = []
            self._attribute_listeners[attr_name] = listeners_for_attr
        if observer not in listeners_for_attr:
            listeners_for_attr.append(observer)

    def remove_attribute_listener(
        self,
        attr_name: str,
        observer: ObserverCallback,
    ) -> None:
        """
        Remove an attribute listener that was previously added.

        Args:
            attr_name: The attribute name to remove the observer from
                      (or '*' to remove an 'all attribute' observer).
            observer: The callback function to remove.
        """
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is not None:
            listeners_for_attr.remove(observer)
            if len(listeners_for_attr) == 0:
                del self._attribute_listeners[attr_name]

    def notify_attribute_listeners(
        self,
        attr_name: str,
        value: Any,
        cache: bool = False,
    ) -> None:
        """
        Notify observers when the named attribute is updated.

        Call this method in message listeners after updating an attribute
        with information from a vehicle message.

        Args:
            attr_name: The name of the attribute that has been updated.
            value: The current value of the attribute.
            cache: If True, only notify observers when the value actually
                  changes. Default is False (notify on every update).
                  Use True for state attributes like 'mode'.
                  Use False for sensor/heartbeat monitoring.
        """
        # Cached values are not re-sent if they are unchanged.
        if cache:
            if self._attribute_cache.get(attr_name) == value:
                return
            self._attribute_cache[attr_name] = value

        # Notify specific attribute observers
        for fn in self._attribute_listeners.get(attr_name, []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception(
                    "Exception in attribute handler for %s",
                    attr_name,
                    exc_info=True,
                )

        # Notify wildcard observers
        for fn in self._attribute_listeners.get("*", []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception(
                    "Exception in attribute handler for %s",
                    attr_name,
                    exc_info=True,
                )

    def on_attribute(self, name: str | list[str]) -> Callable[[ObserverCallback], None]:
        """
        Decorator for attribute listeners.

        The decorated function is invoked when the specified attribute(s) change.

        Args:
            name: The name of the attribute to watch, or a list of names,
                 or '*' to watch all attributes.

        Returns:
            Decorator function that registers the observer.

        Example:
            >>> @vehicle.on_attribute('attitude')
            ... def attitude_listener(self, name, msg):
            ...     print(f'{name} attribute is: {msg}')

        Note:
            There is no way to remove an attribute listener added with this
            decorator. Use :py:func:`add_attribute_listener` if you need
            to remove the listener later.
        """

        def decorator(fn: ObserverCallback) -> None:
            if isinstance(name, list):
                for n in name:
                    self.add_attribute_listener(n, fn)
            else:
                self.add_attribute_listener(name, fn)

        return decorator
