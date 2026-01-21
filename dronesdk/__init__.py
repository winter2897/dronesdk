"""
Dronesdk-Python API.

This is the API Reference for the dronesdk-Python API.

The main API is the :py:class:`Vehicle` class.
The code snippet below shows how to use :py:func:`connect` to obtain an instance of a connected vehicle:

.. code:: python

    from dronesdk import connect

    # Connect to the Vehicle using "connection string" (in this case an address on network)
    vehicle = connect('127.0.0.1:14550', wait_ready=True)

:py:class:`Vehicle` provides access to vehicle *state* through python attributes
(e.g. :py:attr:`Vehicle.mode`)
and to settings/parameters though the :py:attr:`Vehicle.parameters` attribute.
Asynchronous notification on vehicle attribute changes is available by registering listeners/observers.

Vehicle movement is primarily controlled using the :py:attr:`Vehicle.armed` attribute and
:py:func:`Vehicle.simple_takeoff` and :py:attr:`Vehicle.simple_goto` in GUIDED mode.

Velocity-based movement and control over other vehicle features can be achieved using custom MAVLink messages
(:py:func:`Vehicle.send_mavlink`, :py:func:`Vehicle.message_factory`).

It is also possible to work with vehicle "missions" using the :py:attr:`Vehicle.commands` attribute, and run them in AUTO mode.

All the logging is handled through the builtin Python `logging` module.

A number of other useful classes and methods are listed below.

----

This module provides the public API with full backwards compatibility.
The implementation has been refactored into a modular architecture.
"""

from __future__ import annotations

# Core infrastructure
from dronesdk.core.exceptions import APIException, TimeoutError
from dronesdk.core.observer import HasObservers
from dronesdk.core.events import EventBus, EventPriority, MAVLinkMessageEvent
from dronesdk.core.types import Observable, VehicleModule, ConnectionProtocol

# Data models
from dronesdk.models.location import (
    LocationGlobal,
    LocationGlobalRelative,
    LocationLocal,
)
from dronesdk.models.attitude import Attitude
from dronesdk.models.battery import Battery
from dronesdk.models.sensors import GPSInfo, Rangefinder, Wind
from dronesdk.models.version import Version, Capabilities
from dronesdk.models.status import VehicleMode, SystemStatus
from dronesdk.models.command import Command

# Data link
from dronesdk.datalink.connection import MAVConnection, MAVWriter
from dronesdk.datalink.heartbeat import HeartbeatManager
from dronesdk.datalink.message_router import MessageRouter

# Modules
from dronesdk.sensors.manager import SensorManager
from dronesdk.sensors.location import Locations
from dronesdk.power.monitor import BatteryMonitor
from dronesdk.health.monitor import HealthMonitor, EKFStatus
from dronesdk.parameters.manager import ParameterManager
from dronesdk.channels.reader import Channels
from dronesdk.channels.override import ChannelsOverride
from dronesdk.gimbal.controller import GimbalController, GimbalState
from dronesdk.mission.sequence import CommandSequence
from dronesdk.mission.manager import MissionManager
from dronesdk.flight_control.controller import FlightController
from dronesdk.flight_control.navigation import (
    get_distance_metres,
    get_location_metres,
    get_bearing,
)

# Logging utilities (from original util.py for backwards compatibility)
from dronesdk.util import ErrprinterHandler

# Compatibility layer (main public API)
from dronesdk.compat.facade import Vehicle
from dronesdk.compat.connect import connect

# Re-export pymavlink for convenience
from pymavlink import mavutil

# Legacy alias for Parameters class
Parameters = ParameterManager

# Legacy alias for Gimbal class
Gimbal = GimbalController

__all__ = [
    # Main API
    "connect",
    "Vehicle",
    # Exceptions
    "APIException",
    "TimeoutError",
    # Locations
    "LocationGlobal",
    "LocationGlobalRelative",
    "LocationLocal",
    "Locations",
    # Data models
    "Attitude",
    "Battery",
    "GPSInfo",
    "Rangefinder",
    "Wind",
    "Version",
    "Capabilities",
    "VehicleMode",
    "SystemStatus",
    "Command",
    # Mission
    "CommandSequence",
    "MissionManager",
    # Channels
    "Channels",
    "ChannelsOverride",
    # Gimbal
    "Gimbal",
    "GimbalController",
    "GimbalState",
    # Parameters
    "Parameters",
    "ParameterManager",
    # Observer
    "HasObservers",
    # Data link
    "MAVConnection",
    "MAVWriter",
    "HeartbeatManager",
    "MessageRouter",
    # Event system
    "EventBus",
    "EventPriority",
    "MAVLinkMessageEvent",
    # Types
    "Observable",
    "VehicleModule",
    "ConnectionProtocol",
    # Modules
    "SensorManager",
    "BatteryMonitor",
    "HealthMonitor",
    "EKFStatus",
    "FlightController",
    # Navigation utilities
    "get_distance_metres",
    "get_location_metres",
    "get_bearing",
    # Logging
    "ErrprinterHandler",
    # pymavlink
    "mavutil",
]

# Version info
__version__ = "2.10.0"
