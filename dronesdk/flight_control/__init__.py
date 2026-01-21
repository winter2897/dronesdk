"""
Flight Control Module.

This module provides flight control functionality including
arming, takeoff, and navigation.
"""

from dronesdk.flight_control.controller import FlightController
from dronesdk.flight_control.navigation import (
    get_distance_metres,
    get_location_metres,
    get_bearing,
)

__all__ = [
    "FlightController",
    "get_distance_metres",
    "get_location_metres",
    "get_bearing",
]
