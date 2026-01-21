"""
Core Infrastructure.

This module provides the foundational components used throughout dronesdk:
- Exception classes
- Observer pattern implementation
- Event bus for message routing
- Protocol/type definitions
"""

from dronesdk.core.exceptions import APIException, TimeoutError
from dronesdk.core.observer import HasObservers
from dronesdk.core.events import EventBus, EventPriority, MAVLinkMessageEvent
from dronesdk.core.types import (
    Observable,
    VehicleModule,
    ConnectionProtocol,
    LocationProtocol,
)

__all__ = [
    # Exceptions
    "APIException",
    "TimeoutError",
    # Observer
    "HasObservers",
    # Events
    "EventBus",
    "EventPriority",
    "MAVLinkMessageEvent",
    # Types
    "Observable",
    "VehicleModule",
    "ConnectionProtocol",
    "LocationProtocol",
]
