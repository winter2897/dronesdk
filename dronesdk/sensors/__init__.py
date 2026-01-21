"""
Sensors Module.

This module provides sensor data management including attitude,
GPS, velocity, wind, and rangefinder.
"""

from dronesdk.sensors.manager import SensorManager
from dronesdk.sensors.location import Locations

__all__ = ["SensorManager", "Locations"]
