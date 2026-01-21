"""
Data Models.

This module provides immutable dataclasses for representing vehicle state,
sensor data, and other information.
"""

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

__all__ = [
    # Location
    "LocationGlobal",
    "LocationGlobalRelative",
    "LocationLocal",
    # Attitude
    "Attitude",
    # Battery
    "Battery",
    # Sensors
    "GPSInfo",
    "Rangefinder",
    "Wind",
    # Version
    "Version",
    "Capabilities",
    # Status
    "VehicleMode",
    "SystemStatus",
    # Command
    "Command",
]
