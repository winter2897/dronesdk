"""
Protocol and Type Definitions.

This module defines the protocols (interfaces) and type aliases used
throughout dronesdk for type checking and documentation.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable, Protocol, runtime_checkable

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus


@runtime_checkable
class Observable(Protocol):
    """
    Protocol for objects that support attribute observation.

    Any object implementing this protocol can have listeners added
    for attribute changes.
    """

    def add_attribute_listener(
        self,
        attr_name: str,
        observer: Callable[[Any, str, Any], None],
    ) -> None:
        """Add a listener for attribute changes."""
        ...

    def remove_attribute_listener(
        self,
        attr_name: str,
        observer: Callable[[Any, str, Any], None],
    ) -> None:
        """Remove a previously added listener."""
        ...

    def notify_attribute_listeners(
        self,
        attr_name: str,
        value: Any,
        cache: bool = False,
    ) -> None:
        """Notify all listeners of an attribute change."""
        ...


@runtime_checkable
class VehicleModule(Protocol):
    """
    Protocol for vehicle subsystem modules.

    All vehicle modules (sensors, flight control, mission, etc.) should
    implement this protocol to ensure consistent lifecycle management.
    """

    @property
    def name(self) -> str:
        """Return the module name (e.g., 'sensors', 'flight_control')."""
        ...

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach the module to an event bus.

        Called when the module is being connected to a vehicle.
        The module should subscribe to relevant MAVLink messages here.
        """
        ...

    def detach(self) -> None:
        """
        Detach the module from the event bus.

        Called when disconnecting. The module should unsubscribe
        from all events and clean up resources.
        """
        ...

    def initialize(self) -> None:
        """
        Initialize the module after attachment.

        Called after attach() to perform any initialization that
        requires the event bus to be set up.
        """
        ...


@runtime_checkable
class ConnectionProtocol(Protocol):
    """
    Protocol for MAVLink connection handlers.

    Defines the interface that connection implementations must provide.
    """

    @property
    def target_system(self) -> int:
        """Return the target system ID."""
        ...

    @property
    def target_component(self) -> int:
        """Return the target component ID."""
        ...

    def send_mavlink(self, message: Any) -> None:
        """Send a MAVLink message."""
        ...

    def close(self) -> None:
        """Close the connection."""
        ...


@runtime_checkable
class LocationProtocol(Protocol):
    """
    Protocol for location objects.

    All location types (Global, GlobalRelative, Local) should
    implement this protocol.
    """

    @property
    def lat(self) -> float | None:
        """Latitude in degrees (or None for local frames)."""
        ...

    @property
    def lon(self) -> float | None:
        """Longitude in degrees (or None for local frames)."""
        ...

    @property
    def alt(self) -> float | None:
        """Altitude in meters."""
        ...


# Type aliases for common callback signatures
MessageCallback = Callable[[Any, str, Any], None]
AttributeCallback = Callable[[Any, str, Any], None]
